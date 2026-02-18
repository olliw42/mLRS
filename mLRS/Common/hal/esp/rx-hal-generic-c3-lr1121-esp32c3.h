//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// ESP32, ELRS GENERIC C3 LR1121 Receiver
//-------------------------------------------------------

#define DEVICE_HAS_SINGLE_LED_RGB
#define DEVICE_HAS_NO_DEBUG
//#define DEVICE_HAS_SERIAL_OR_DEBUG


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_SERIAL
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_USE_TX_IO           IO_P21
#define UARTB_USE_RX_IO           IO_P20
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE

#define UARTF_USE_SERIAL
#define UARTF_BAUD                115200


//-- SX1: LR11xx & SPI

#define SPI_CS_IO                 IO_P7
#define SPI_MISO                  IO_P5
#define SPI_MOSI                  IO_P4
#define SPI_SCK                   IO_P6
#define SPI_FREQUENCY             16000000L  // 16 MHz max per datasheet
#define SX_BUSY                   IO_P3
#define SX_DIO1                   IO_P1
#define SX_RESET                  IO_P2

#define SX_USE_REGULATOR_MODE_DCDC

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_DIO1, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_BUSY, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_LOW);
}

IRAM_ATTR bool sx_busy_read(void)
{
    return (gpio_read_activehigh(SX_BUSY)) ? true : false;
}

IRAM_ATTR void sx_amp_transmit(void) {}

IRAM_ATTR void sx_amp_receive(void) {}

void sx_dio_init_exti_isroff(void)
{
    detachInterrupt(SX_DIO1);
}

void sx_dio_enable_exti_isr(void)
{
    attachInterrupt(SX_DIO1, SX_DIO_EXTI_IRQHandler, RISING);
}

IRAM_ATTR void sx_dio_exti_isr_clearflag(void) {}


//-- Button

#define BUTTON                    IO_P9

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU);
}

IRAM_ATTR bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON) ? true : false;
}


//-- LEDs

#define LED_RGB                   IO_P8
#define LED_RGB_PIXEL_NUM         1
#include "../esp-hal-led-rgb.h"


//-- POWER

#include "../../setup_types.h" // needed for frequency band condition in rfpower calc

void lr11xx_rfpower_calc(const int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm, const uint8_t frequency_band)
{
    if (frequency_band == SX_FHSS_FREQUENCY_BAND_2P4_GHZ) {
        if (power_dbm >= POWER_20_DBM) { // -> 20
            *sx_power = 2;
            *actual_power_dbm = 20;  // xr1 measures about 19 dBm here, further power shows little increase, PA max input is +5 dBm
        } else if (power_dbm >= POWER_14_DBM) { // -> 14
            *sx_power = -6;
            *actual_power_dbm = 14;
        } else if (power_dbm >= POWER_10_DBM) { // -> 10
            *sx_power = -11;
            *actual_power_dbm = 10;
        } else {
            *sx_power = -18;
            *actual_power_dbm = 3;
        }
    } else {
        if (power_dbm >= POWER_20_DBM) { // -> 20
            *sx_power = 22;
            *actual_power_dbm = 20;
        } else if (power_dbm >= POWER_14_DBM) { // -> 14
            *sx_power = 16;
            *actual_power_dbm = 14;
        } else if (power_dbm >= POWER_10_DBM) { // -> 10
            *sx_power = 12;
            *actual_power_dbm = 10;
        } else {
            *sx_power = 5;
            *actual_power_dbm = 3;
        }

    }
}

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_3_DBM, .mW = 2 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_14_DBM, .mW = 25 },
    { .dbm = POWER_20_DBM, .mW = 100 },
};
