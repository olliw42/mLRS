//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP Peripherals
//********************************************************
#ifndef ESPLIB_PERIPHERALS_H
#define ESPLIB_PERIPHERALS_H


#define GPIO_INLINE_FORCED  IRAM_ATTR static inline __attribute__ ((always_inline))


#define IO_P0       0
#define IO_P1       1
#define IO_P2       2
#define IO_P3       3
#define IO_P4       4
#define IO_P5       5
#define IO_P6       6
#define IO_P7       7
#define IO_P8       8
#define IO_P9       9
#define IO_P10      10
#define IO_P11      11
#define IO_P12      12
#define IO_P13      13
#define IO_P14      14
#define IO_P15      15
#define IO_P16      16


typedef enum {
    IO_MODE_Z = 0,
    IO_MODE_INPUT_ANALOG,
    IO_MODE_INPUT_PU,
    IO_MODE_OUTPUT_PP,
    IO_MODE_OUTPUT_PP_LOW,
    IO_MODE_OUTPUT_PP_HIGH,
} IOMODEENUM;


void gpio_init(uint8_t GPIO_Pin, IOMODEENUM mode)
{
    switch (mode) {
    case IO_MODE_Z:
        pinMode(GPIO_Pin, INPUT);
        break;
    case IO_MODE_INPUT_ANALOG:
        pinMode(GPIO_Pin, INPUT);
        break;
    case IO_MODE_INPUT_PU:
        pinMode(GPIO_Pin, INPUT_PULLUP);
        break;
    case IO_MODE_OUTPUT_PP:
        pinMode(GPIO_Pin, OUTPUT);
        break;
    case IO_MODE_OUTPUT_PP_LOW:
        pinMode(GPIO_Pin, OUTPUT);
        digitalWrite(GPIO_Pin, LOW);
        break;
    case IO_MODE_OUTPUT_PP_HIGH:
        pinMode(GPIO_Pin, OUTPUT);
        digitalWrite(GPIO_Pin, HIGH);
        break;
    }
}


GPIO_INLINE_FORCED void gpio_low(uint8_t GPIO_Pin)
{
    digitalWrite(GPIO_Pin, LOW);
}


GPIO_INLINE_FORCED void gpio_high(uint8_t GPIO_Pin)
{
    digitalWrite(GPIO_Pin, HIGH);
}


GPIO_INLINE_FORCED void gpio_toggle(uint8_t GPIO_Pin)
{
    digitalWrite(GPIO_Pin, !digitalRead(GPIO_Pin));
}


GPIO_INLINE_FORCED uint16_t gpio_read_activehigh(uint8_t GPIO_Pin)
{
    return (digitalRead(GPIO_Pin) == HIGH) ? 1 : 0;
}


GPIO_INLINE_FORCED uint16_t gpio_read_activelow(uint8_t GPIO_Pin)
{
    return (digitalRead(GPIO_Pin) != HIGH) ? 1 : 0;
}


GPIO_INLINE_FORCED uint16_t gpio_readoutput(uint8_t GPIO_Pin)
{
    return (digitalRead(GPIO_Pin) == HIGH) ? 1 : 0;
}


#endif // ESPLIB_PERIPHERALS_H
