//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP Peripherals
//********************************************************
#ifndef ESPLIB_PERIPHERALS_H
#define ESPLIB_PERIPHERALS_H


#define GPIO_INLINE_FORCED  IRAM_ATTR

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
#define IO_P20      20  // ESP32C3
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
#define IO_P40      40  // ESP32S3
#define IO_P41      41  // ESP32S3
#define IO_P42      42  // ESP32S3
#define IO_P43      43  // ESP32S3
#define IO_P44      44  // ESP32S3
#define IO_P45      45  // ESP32S3
#define IO_P46      46  // ESP32S3
#define IO_P47      47  // ESP32S3
#define IO_P48      48  // ESP32S3


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
#if defined CONFIG_IDF_TARGET_ESP32 || defined CONFIG_IDF_TARGET_ESP32S3
    // special handling for pins >= 32
    // pinMode(), digitalWrite() do not work
    if (GPIO_Pin >= 32) {
        gpio_config_t io_conf;
        io_conf.pin_bit_mask = 1ULL << GPIO_Pin;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        switch (mode) {
        case IO_MODE_Z:
        case IO_MODE_INPUT_ANALOG:
            io_conf.mode = GPIO_MODE_INPUT;
            break;
        case IO_MODE_INPUT_PU:
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
            break;
        case IO_MODE_OUTPUT_PP:
        case IO_MODE_OUTPUT_PP_LOW:
        case IO_MODE_OUTPUT_PP_HIGH:
            io_conf.mode = GPIO_MODE_OUTPUT;
            break;
        }
        gpio_config(&io_conf);
        switch (mode) {
        case IO_MODE_OUTPUT_PP_LOW:
            GPIO.out1_w1tc.data = ((uint32_t)1 << (GPIO_Pin - 32));
            break;
        case IO_MODE_OUTPUT_PP_HIGH:
            GPIO.out1_w1ts.data = ((uint32_t)1 << (GPIO_Pin - 32));
            break;
        }
        return;
    }
#endif
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
#ifdef CONFIG_IDF_TARGET_ESP32C3
    GPIO.out_w1tc.out_w1tc = (1 << GPIO_Pin);
#elif defined CONFIG_IDF_TARGET_ESP32 || defined CONFIG_IDF_TARGET_ESP32S3
    if (GPIO_Pin < 32) {
        GPIO.out_w1tc = ((uint32_t)1 << GPIO_Pin);
    } else {
        GPIO.out1_w1tc.data = ((uint32_t)1 << (GPIO_Pin - 32));
    }
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
#ifdef CONFIG_IDF_TARGET_ESP32C3
    GPIO.out_w1ts.out_w1ts = (1 << GPIO_Pin);
#elif defined CONFIG_IDF_TARGET_ESP32 || defined CONFIG_IDF_TARGET_ESP32S3
    if (GPIO_Pin < 32) {
        GPIO.out_w1ts = ((uint32_t)1 << GPIO_Pin);
    } else {
        GPIO.out1_w1ts.data = ((uint32_t)1 << (GPIO_Pin - 32));
    }
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
