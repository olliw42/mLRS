//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// ESP32, ELRS GENERIC 900 True Diversity PA RX
//-------------------------------------------------------

#define DEVICE_HAS_SINGLE_LED_RGB
#define DEVICE_HAS_DIVERSITY_SINGLE_SPI // must be set, doesn't work without it
#define DEVICE_HAS_NO_DEBUG
//#define DEVICE_HAS_SERIAL_OR_DEBUG


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_SERIAL
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE

#define UARTF_USE_SERIAL
#define UARTF_BAUD                115200


//-- SX1: SX12xx & SPI
// antenna1 = left ufl

#define SPI_CS_IO                 IO_P13
#define SPI_MISO                  IO_P33
#define SPI_MOSI                  IO_P32
#define SPI_SCK                   IO_P25
#define SPI_FREQUENCY             10000000L
#define SX_DIO0                   IO_P39
#define SX_DIO1                   IO_P34
#define SX_RESET                  IO_P21
#define SX_RX_EN                  IO_P9
#define SX_TX_EN                  IO_P15

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_DIO0, IO_MODE_INPUT_PU);
    gpio_init(SX_DIO1, IO_MODE_INPUT_PU);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_LOW);
}

IRAM_ATTR void sx_amp_transmit(void)
{
    gpio_low(SX_RX_EN);
    gpio_high(SX_TX_EN);
}

IRAM_ATTR void sx_amp_receive(void)
{
    gpio_low(SX_TX_EN);
    gpio_high(SX_RX_EN);
}

void sx_dio_init_exti_isroff(void)
{
    detachInterrupt(SX_DIO0);
}

void sx_dio_enable_exti_isr(void)
{
    attachInterrupt(SX_DIO0, SX_DIO_EXTI_IRQHandler, RISING);
}

IRAM_ATTR void sx_dio_exti_isr_clearflag(void) {}


//-- SX2: SX12xx & SPI
// antenna2 = right ufl

#define SX2_CS_IO                 IO_P27
#define SX2_DIO0                  IO_P36
#define SX2_DIO1                  IO_P37
#define SX2_RESET                 IO_P26
#define SX2_RX_EN                 IO_P10
#define SX2_TX_EN                 IO_P14

IRQHANDLER(void SX2_DIO_EXTI_IRQHandler(void);)

void sx2_init_gpio(void)
{
    gpio_init(SX2_CS_IO, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX2_DIO0, IO_MODE_INPUT_PU);
    gpio_init(SX2_DIO1, IO_MODE_INPUT_PU);
    gpio_init(SX2_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX2_RX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX2_RESET, IO_MODE_OUTPUT_PP_LOW);
}

IRAM_ATTR void spib_select(void)
{
    gpio_low(SX2_CS_IO);
}

IRAM_ATTR void spib_deselect(void)
{
    gpio_high(SX2_CS_IO);
}

IRAM_ATTR void sx2_amp_transmit(void)
{
    gpio_low(SX2_RX_EN);
    gpio_high(SX2_TX_EN);
}

IRAM_ATTR void sx2_amp_receive(void)
{
    gpio_low(SX2_TX_EN);
    gpio_high(SX2_RX_EN);
}

void sx2_dio_init_exti_isroff(void)
{
    detachInterrupt(SX2_DIO0);
}

void sx2_dio_enable_exti_isr(void)
{
    attachInterrupt(SX2_DIO0, SX2_DIO_EXTI_IRQHandler, RISING);
}

void sx2_dio_exti_isr_clearflag(void) {}


//-- Button

#define BUTTON                    IO_P0

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU);
}

IRAM_ATTR bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON) ? true : false;
}


//-- LEDs

#define LED_RGB                   IO_P22
#define LED_RGB_PIXEL_NUM         1
#include "../esp-hal-led-rgb.h"


//-- POWER

#define POWER_GAIN_DBM            16 // gain of a PA stage if present
#define POWER_SX1276_MAX_DBM      SX1276_OUTPUT_POWER_MAX // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_20_DBM, .mW = 100 }, // PA=14 gives ~22.5 dBm, 176 mW, PA=16 ~20.5 dBm, 111 mW,
    { .dbm = POWER_24_DBM, .mW = 250 }, // PA=14 gives ~25 dBm, 330 mW, PA=16 ~24.2 dBm, 256 mW
    { .dbm = POWER_27_DBM, .mW = 500 }, // PA=14 gives ~26.3 dBm, 420 mW, PA=16 ~25.7 dBm, 370 mW
};
