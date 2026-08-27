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

#else

void trng_init(void) {}
uint32_t trng_get32(void) { return 0xFFFFFFFF; }

#endif // #ifdef STM32G4


//-------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif // STDSTM32_LL_TRNG_H
