//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// my True Random Number Generator standard library
//*******************************************************
#ifndef STDSTM32_LL_TRNG_H
#define STDSTM32_LL_TRNG_H
#ifdef __cplusplus
extern "C" {
#endif


#if defined STM32G4 || defined STM32WL

uint32_t trng_get32(void)
{
// CExS: clock error flag
// SExS: seed error flag
// HAL is not doing any handling of the error flags, only DRDY
// datasheet says that clock error has no impact on generated random numbers, application can still read RNG_DR

    uint32_t retry_cnt = 10000; // 200 roughly corresponds to 1 us

    while (retry_cnt--) {
        if (LL_RNG_IsActiveFlag_SEIS(RNG)) { // seed error, do recover sequence per datasheet
            LL_RNG_ClearFlag_SEIS(RNG);
            for (uint8_t i = 0; i < 12; i++) LL_RNG_ReadRandData32(RNG);
        } else {
            if (LL_RNG_IsActiveFlag_DRDY(RNG)) return LL_RNG_ReadRandData32(RNG);
        }
    }

    return UINT32_MAX;
}


void trng_init(void)
{
#ifdef STM32G4
    // Note:
    // On the STM32G4, USB and RNG use the same 48 MHz clock source (HSI48 or PLLQ, normally HSI48).
    // Hence, if USB is used in addition, configuring the clock can be tricky.
    // We thus do the following:
    // - if USB is used, ensure to initialize it before RNG
    // - RNG then checks if HSI48 or PLLQ is already enabled, and if not enables HSI48
    uint8_t rng_clk_enabled = 0;

    switch (LL_RCC_GetRNGClockSource(LL_RCC_RNG_CLKSOURCE)) {
    case LL_RCC_RNG_CLKSOURCE_HSI48:
        rng_clk_enabled = LL_RCC_HSI48_IsReady();
        break;
    case LL_RCC_RNG_CLKSOURCE_PLL:
        rng_clk_enabled = LL_RCC_PLL_IsReady();
        uint32_t clk_freq = LL_RCC_GetRNGClockFreq(LL_RCC_RNG_CLKSOURCE);
        if (clk_freq == 0 || clk_freq > 48000000) rng_clk_enabled = 0;
        break;
    }

    if (!rng_clk_enabled) {
        LL_RCC_HSI48_Enable();
        while (!LL_RCC_HSI48_IsReady()) {}
        LL_RCC_SetRNGClockSource(LL_RCC_RNG_CLKSOURCE_HSI48);
    }

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_RNG);
#endif
#ifdef STM32WL
    // RNG is essentially the only user of MSI, so MSI can be used without concern
    LL_RCC_MSI_Enable();
    while (!LL_RCC_MSI_IsReady()) {}
    LL_RCC_SetRNGClockSource(LL_RCC_RNG_CLKSOURCE_MSI);
    LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RNG);
#endif

    LL_RNG_Enable(RNG);

    trng_get32(); // call it once
}

#elif defined STM32F1

// The F1 has no RNG, so at boot ADC noise of the temperature sensor (ch16) and Vrefint (ch17) is hashed with BLAKE2b.
// On R9M, SP 800-90B gives >= 0.40 bit min-entropy per conversion; trng_init() takes 31 ms (ADC 10 ms, BLAKE2b 21 ms).
#include "monocypher/src/monocypher.h"

#define TRNG_F1_CONVERSIONS   4096 // credits ~1640 bits
#define TRNG_F1_DISCARD       32 // covers tSTAB and the 10 us temperature sensor start-up
#define TRNG_F1_RCT_CUTOFF    51 // repetition count test, 1 + ceil(20 / 0.40)

uint32_t trng_f1_pool[8]; // 6 words are needed
uint8_t trng_f1_pos = 8; // empty until a harvest succeeded


bool trng_f1_harvest(crypto_blake2b_ctx* ctx)
{
    uint8_t last[2] = {2, 2}, rep[2] = {};
    for (uint16_t i = 0; i < TRNG_F1_DISCARD + TRNG_F1_CONVERSIONS; i++) {
        uint8_t ch = i & 1;
        ADC1->SQR3 = 16 + ch;
        ADC1->CR2 |= ADC_CR2_ADON; // writing ADON = 1 while ADON = 1 starts a conversion
        uint32_t tmo = 10000;
        while (!(ADC1->SR & ADC_SR_EOC)) {
            if (!--tmo) return false;
        }
        uint16_t v = ADC1->DR;
        if (i < TRNG_F1_DISCARD) continue;
        crypto_blake2b_update(ctx, (uint8_t*)&v, 2);

        // health test on the credited low bit, a stuck ADC fails the harvest
        rep[ch] = ((v & 1) == last[ch]) ? rep[ch] + 1 : 0;
        if (rep[ch] >= TRNG_F1_RCT_CUTOFF - 1) return false;
        last[ch] = v & 1;
    }
    return true;
}


void trng_init(void)
{
    // ADC clock = PCLK2/6 = 12 MHz, 1.5 cycles sample time (reset value), as used for the entropy measurement
    uint32_t cfgr_adcpre = RCC->CFGR & RCC_CFGR_ADCPRE;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;

    ADC1->CR2 = ADC_CR2_TSVREFE | ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while (ADC1->CR2 & ADC_CR2_RSTCAL) {}
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL) {}

    crypto_blake2b_ctx ctx;
    crypto_blake2b_init(&ctx, sizeof(trng_f1_pool));
    bool ok = trng_f1_harvest(&ctx);
    crypto_blake2b_final(&ctx, (uint8_t*)trng_f1_pool);
    if (ok) trng_f1_pos = 0;

    // return ADC1 to its reset state
    ADC1->CR2 = 0;
    RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | cfgr_adcpre;
}


uint32_t trng_get32(void)
{
    if (trng_f1_pos >= 8) return UINT32_MAX;
    return trng_f1_pool[trng_f1_pos++];
}

#else

void trng_init(void) {}
uint32_t trng_get32(void) { return UINT32_MAX; }

#endif


//-------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif // STDSTM32_LL_TRNG_H
