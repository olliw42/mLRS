//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

/*
  BAYCKRC UR500 receiver flashed as Tx module (ESP32-C3 + LR1121)

  Wiring for use as JR bay Tx module:
  - radio module bay pin 5 (CRSF signal) -> UR500 GPIO18 (serial1 rx pad)
  - radio module bay pin 1 (GND)         -> UR500 GND
  - power: feed 5V to UR500 VCC pad (check board markings!)

  Pin source: ExpressLRS Targets ur500 overlay on "Generic C3 LR1121.json"
*/

//-------------------------------------------------------
// ESP32-C3, BAYCKRC UR500 as Tx Module, LR1121 2400 & 900
//-------------------------------------------------------

#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_SINGLE_LED_RGB
#define DEVICE_HAS_NO_DEBUG
#define DEVICE_HAS_NO_COM


//-- UARTS
// UARTB = serial port (USB-serial bridge on GPIO21/20)
// UART  = JR bay pin5, CRSF from radio (Serial1, half-duplex on GPIO18)
// UARTF = debug port (not used)

#define UARTB_USE_SERIAL
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX_IO           IO_P21
#define UARTB_USE_RX_IO           IO_P20
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UART_USE_SERIAL1 // JR bay pin5, half-duplex
#define UART_BAUD                 400000
#define UART_USE_TX_IO            IO_P18
#define UART_USE_RX_IO            IO_P18
#define UART_TXBUFSIZE            0  // TX FIFO = 128
#define UART_RXBUFSIZE            0  // RX FIFO = 128 + 1


//-- SX1: LR11xx & SPI
// same pinout as rx-hal-generic-c3-lr1121-esp32c3.h (identical hardware)

#define SPI_CS_IO                 IO_P7
#define SPI_MISO                  IO_P5
#define SPI_MOSI                  IO_P4
#define SPI_SCK                   IO_P6
#define SPI_FREQUENCY             16000000L  // 16 MHz max per datasheet
#define SX_RESET                  IO_P2
#define SX_DIO                    IO_P1
#define SX_BUSY                   IO_P3

#define SX_USE_REGULATOR_MODE_DCDC

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_DIO, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_BUSY, IO_MODE_INPUT_ANALOG);
}

IRAM_ATTR bool sx_busy_read(void) { return (gpio_read_activehigh(SX_BUSY)) ? true : false; }

IRAM_ATTR void sx_amp_transmit(void) {}
IRAM_ATTR void sx_amp_receive(void) {}

void sx_dio_init_exti_isroff(void) { detachInterrupt(SX_DIO); }
void sx_dio_enable_exti_isr(void) { attachInterrupt(SX_DIO, SX_DIO_EXTI_IRQHandler, RISING); }
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
#include "esp-hal-led-rgb.h"


//-- POWER
// power values from ExpressLRS ur500 overlay:
//   power_values      (2.4 GHz): [-7, -3, 0]  for ELRS levels 3/4/5 (100/250/500 mW)
//   power_values_dual (900 MHz): [-7, -2, 3]  for ELRS levels 3/4/5
// board uses external PA with roughly 24-27 dB gain, chip drives it via RFO (radio_rfo_hf)

#include "../../setup_types.h" // needed for frequency band condition in rfpower calc

#define SX_USE_LP_PA               // radio_rfo_hf option, use low power PA to drive external PA
#define SX_USE_RFSW_CTRL  {15, 0, 4, 12, 0, 2, 0, 1} // radio_rfsw_ctrl array

void lr11xx_rfpower_calc(const int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm, const uint8_t frequency_band)
{
    if (frequency_band == SX_FHSS_FREQUENCY_BAND_2P4_GHZ) {
        if (power_dbm >= POWER_27_DBM) { // -> 27 (500 mW, ELRS level 5)
            *sx_power = 0;
            *actual_power_dbm = 27;
        } else if (power_dbm >= POWER_24_DBM) { // -> 24 (250 mW, ELRS level 4)
            *sx_power = -3;
            *actual_power_dbm = 24;
        } else if (power_dbm >= POWER_20_DBM) { // -> 20 (100 mW, ELRS level 3)
            *sx_power = -7;
            *actual_power_dbm = 20;
        } else if (power_dbm >= POWER_14_DBM) { // -> 14
            *sx_power = -13;
            *actual_power_dbm = 14;
        } else if (power_dbm >= POWER_10_DBM) { // -> 10
            *sx_power = -17;
            *actual_power_dbm = 10;
        } else {
            *sx_power = -18;
            *actual_power_dbm = 3;
        }
    } else { // 868/915 MHz
        if (power_dbm >= POWER_27_DBM) { // -> 27 (500 mW, ELRS level 5)
            *sx_power = 3;
            *actual_power_dbm = 27;
        } else if (power_dbm >= POWER_24_DBM) { // -> 24 (250 mW, ELRS level 4)
            *sx_power = -2;
            *actual_power_dbm = 24;
        } else if (power_dbm >= POWER_20_DBM) { // -> 20 (100 mW, ELRS level 3)
            *sx_power = -7;
            *actual_power_dbm = 20;
        } else if (power_dbm >= POWER_14_DBM) { // -> 14
            *sx_power = -11;
            *actual_power_dbm = 14;
        } else if (power_dbm >= POWER_10_DBM) { // -> 10
            *sx_power = -15;
            *actual_power_dbm = 10;
        } else {
            *sx_power = -18;
            *actual_power_dbm = 3;
        }
    }
}

#define RFPOWER_DEFAULT           2 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_3_DBM, .mW = 2 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_14_DBM, .mW = 25 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
};
