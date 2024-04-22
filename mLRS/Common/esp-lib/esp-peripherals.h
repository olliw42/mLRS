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

// note: ESP32 does not break out every pin, hence the gaps

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
#define IO_P17      17
#define IO_P18      18
#define IO_P19      19
#define IO_P21      21
#define IO_P22      22
#define IO_P23      23
#define IO_P25      25
#define IO_P26      26
#define IO_P27      27
#define IO_P32      32
#define IO_P33      33
#define IO_P34      34
#define IO_P35      35
#define IO_P36      36
#define IO_P37      37
#define IO_P38      38
#define IO_P39      39


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
#ifdef ESP32
    GPIO.out_w1tc = ((uint32_t)1 << GPIO_Pin);
#elif defined ESP8266
    if (GPIO_Pin < 16) {
        GPOC = (1 << GPIO_Pin);
    } else if (GPIO_Pin == 16) { // special handling needed for pin 16
        GP16O &=~ 1;
    }
#endif
}


GPIO_INLINE_FORCED void gpio_high(uint8_t GPIO_Pin)
{
#ifdef ESP32
    GPIO.out_w1ts = ((uint32_t)1 << GPIO_Pin);
#elif defined ESP8266
    if (GPIO_Pin < 16) {
        GPOS = (1 << GPIO_Pin);
    } else if (GPIO_Pin == 16) { // special handling needed for pin 16
        GP16O |= 1;
    }
#endif
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
