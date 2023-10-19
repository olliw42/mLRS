//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// test
//********************************************************
#ifndef BOARD_TEST_H
#define BOARD_TEST_H
#pragma once


#define PORTA_N  (sizeof(porta)/sizeof(porta[0]))
#define PORTB_N  (sizeof(portb)/sizeof(portb[0]))
#define PORTC_N  (sizeof(portc)/sizeof(portc[0]))


void init_test(void)
{
LL_GPIO_InitTypeDef GPIO_InitStruct = {};
uint32_t porta_all, portb_all, portc_all, n;

    porta_all = portb_all = portc_all= 0;
    for (n = 0; n < PORTA_N; n++) porta_all |= porta[n];
    for (n = 0; n < PORTB_N; n++) portb_all |= portb[n];
    for (n = 0; n < PORTC_N; n++) portc_all |= portc[n];

    rcc_init_gpio(GPIOA);
    rcc_init_gpio(GPIOB);
    rcc_init_gpio(GPIOC);

    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;

#if defined STM32F7 || defined STM32G4 || defined STM32F3
    GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
#endif

    GPIO_InitStruct.Pin = porta_all;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = portb_all;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = portc_all;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    delay_init();
}


int main_test(void)
{
uint32_t porta_all, portb_all, portc_all;
uint32_t n, nr, bitpos;

    init_test();

    porta_all = portb_all = portc_all= 0;
    for (n = 0; n < PORTA_N; n++) porta_all |= porta[n];
    for (n = 0; n < PORTB_N; n++) portb_all |= portb[n];
    for (n = 0; n < PORTC_N; n++) portc_all |= portc[n];

    while(1) {
        delay_us(1000);

        // port marker
        LL_GPIO_SetOutputPin(GPIOA, porta_all);
        LL_GPIO_SetOutputPin(GPIOB, portb_all);
        LL_GPIO_SetOutputPin(GPIOC, portc_all);
        delay_us(8);
        LL_GPIO_ResetOutputPin(GPIOC, portc_all);
        delay_us(8);
        LL_GPIO_SetOutputPin(GPIOC, portc_all);
        delay_us(8);
        LL_GPIO_ResetOutputPin(GPIOB, portb_all);
        LL_GPIO_ResetOutputPin(GPIOC, portc_all);
        delay_us(10);
        LL_GPIO_SetOutputPin(GPIOB, portb_all);
        LL_GPIO_SetOutputPin(GPIOC, portc_all);
        delay_us(8);
        LL_GPIO_ResetOutputPin(GPIOA, porta_all);
        LL_GPIO_ResetOutputPin(GPIOB, portb_all);
        LL_GPIO_ResetOutputPin(GPIOC, portc_all);

        delay_us(40);

#if defined STM32F1
#define PP(pn,gpiox)  uint32_t port_nr = pn & 0x000000FF; \
                      if (pn & 0x04000000) port_nr <<= 8; \
                      if (port_nr > bitpos) LL_GPIO_SetOutputPin(gpiox, pn);
#elif defined STM32G4 || defined STM32L4 || defined STM32WL || defined STM32F0 || defined STM32F3
#define PP(pn,gpiox)  if (pn > bitpos) LL_GPIO_SetOutputPin(gpiox, pn);
#endif

        // pin marker
        for (nr = 0; nr < 16; nr++) {
            bitpos = 1 << nr;
            for (n = 0; n < PORTA_N; n++) {
//              uint32_t port_nr = porta[n] & 0x000000FF;
//              if (porta[n] & 0x04000000) port_nr <<= 8;
//              if (port_nr > bitpos) LL_GPIO_SetOutputPin(GPIOA, porta[n]);
                PP(porta[n],GPIOA);
            }
            for (n = 0; n < PORTB_N; n++) {
                PP(portb[n],GPIOB);
            }
            for (n = 0; n < PORTC_N; n++) {
                PP(portc[n],GPIOC);
            }
            delay_us(5);
            LL_GPIO_ResetOutputPin(GPIOA, porta_all);
            //delay_us(3);
            LL_GPIO_ResetOutputPin(GPIOB, portb_all);
            //delay_us(3);
            LL_GPIO_ResetOutputPin(GPIOC, portc_all);
            delay_us(10);
            // group into groups of 4
            if (nr == 3) delay_us(15);
            if (nr == 7) delay_us(15);
            if (nr == 11) delay_us(15);
        }
    }
}


#endif // BOARD_TEST_H
