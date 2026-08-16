//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************

//-------------------------------------------------------
// ESP32 Pico D4, Jumper Tx, Internal LR1121, good for T15 Duo, T22 Duo
// Backpack: Generic ESP32-C3 Module, define LED_IO ???
//-------------------------------------------------------
// https://github.com/ExpressLRS/Targets/pull/244

#define DEVICE_HAS_JRPIN5
//?? #define DEVICE_HAS_NO_LED
#define DEVICE_HAS_SINGLE_LED_RGB
#define DEVICE_HAS_FAN_ONOFF
#define DEVICE_HAS_NO_SERIAL
#define DEVICE_HAS_NO_COM
#define DEVICE_HAS_ESP_WIFI_BRIDGE
#define DEVICE_HAS_ESP_WIFI_BRIDGE_CONFIGURE
#define DEVICE_HAS_ESP_WIFI_BRIDGE_ESP32C3
#define DEVICE_HAS_ESP_WIFI_BRIDGE_W_PASSTHRU_VIA_JRPIN5
#define DEVICE_HAS_NO_DEBUG


//-- UARTS
// UARTB = serial port
// UARTC (or USB) = com (CLI) port
// UARTD = serial2 port or wireless bridge port
// UART  = JR bay pin5, full duplex CRSF serial connection to radio - code still calls it JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF or SWUART = debug port

#define UARTD_USE_SERIAL // serial2 or wireless bridge
#define UARTD_BAUD                TX_SERIAL_BAUDRATE
#define UARTD_USE_TX_IO           IO_P5
#define UARTD_USE_RX_IO           IO_P18
#define UARTD_TXBUFSIZE           TX_SERIAL_TXBUFSIZE // 512
#define UARTD_RXBUFSIZE           TX_SERIAL_RXBUFSIZE // 2048

#define UART_USE_SERIAL1 // full duplex CRSF/MBridge (JR pin5)
#define UART_BAUD                 400000
#define UART_USE_TX_IO            IO_P1
#define UART_USE_RX_IO            IO_P3
#define UART_TXBUFSIZE            0 // TX_SERIAL_TXBUFSIZE  // 512
#define UART_RXBUFSIZE            0 // TX_SERIAL_RXBUFSIZE  // 2048

#define JR_PIN5_FULL_DUPLEX       // internal module


//-- SX1: LR11xx & SPI

#define SPI_CS_IO                 IO_P27
#define SPI_MISO                  IO_P33
#define SPI_MOSI                  IO_P32
#define SPI_SCK                   IO_P25
#define SPI_FREQUENCY             16000000L
#define SX_RESET                  IO_P15
#define SX_DIO                    IO_P37
#define SX_BUSY                   IO_P36

#define SX_USE_REGULATOR_MODE_DCDC

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX_DIO, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);
}

IRAM_ATTR bool sx_busy_read(void) { return (gpio_read_activehigh(SX_BUSY)) ? true : false; }

IRAM_ATTR void sx_amp_transmit(void) {}
IRAM_ATTR void sx_amp_receive(void) {}

void sx_dio_enable_exti_isr(void) { attachInterrupt(SX_DIO, SX_DIO_EXTI_IRQHandler, RISING); }
void sx_dio_init_exti_isroff(void) { detachInterrupt(SX_DIO); }
IRAM_ATTR void sx_dio_exti_isr_clearflag(void) {}


//-- Button

void button_init(void) {}
IRAM_ATTR bool button_pressed(void) { return false; }


//-- LEDs

#define LED_RGB                   IO_P22
#define LED_RGB_PIXEL_NUM         1
#include "../esp-hal-led-rgb.h"


//-- Cooling Fan

#define FAN_IO                    IO_P2

void fan_init(void) { gpio_init(FAN_IO, IO_MODE_OUTPUT_PP_LOW); }

IRAM_ATTR void fan_set_power(int8_t power_dbm)
{
    if (power_dbm >= POWER_23_DBM) { gpio_high(FAN_IO); } else { gpio_low(FAN_IO); }
}


//-- ESP32 Wifi Bridge

#define ESP_RESET                 IO_P19 // backpack_en
#define ESP_GPIO0                 IO_P23 // backpack_boot inverted?
#define ESP_BOOT0                 IO_P0  // will always be IO_P0

uint8_t esp_boot0()
{
    return gpio_read_activelow(ESP_BOOT0);
}

void esp_init(void)
{
    // no need to configure ESP_BOOT0 which will always be IO_P0 and is pull-up by default
    gpio_init(ESP_GPIO0, IO_MODE_OUTPUT_PP_LOW); // high -> esp will start in bootloader mode
    gpio_init(ESP_RESET, IO_MODE_OUTPUT_PP_LOW); // low -> esp is in reset
}

IRAM_ATTR void esp_reset_high(void) { gpio_high(ESP_RESET); }
IRAM_ATTR void esp_reset_low(void) { gpio_low(ESP_RESET); }

IRAM_ATTR void esp_gpio0_high(void) { gpio_low(ESP_GPIO0); }
IRAM_ATTR void esp_gpio0_low(void) { gpio_high(ESP_GPIO0); }


//-- POWER

#include "../../setup_types.h" // needed for frequency band condition in rfpower calc
#define SX_USE_LP_PA  // uses the low power amplifier for the 900 side
#define SX_PA_DAC_IO               IO_P26

void lr11xx_rfpower_calc(const int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm, const uint8_t frequency_band)
{
    if (frequency_band == SX_FHSS_FREQUENCY_BAND_2P4_GHZ) {
        if (power_dbm >= POWER_30_DBM) { // -> 30
            *sx_power = 8;
            *actual_power_dbm = 30;
        } else if (power_dbm >= POWER_27_DBM) { // -> 27
            *sx_power = 0;
            *actual_power_dbm = 27;
        } else if (power_dbm >= POWER_24_DBM) { // -> 24
            *sx_power = -3;
            *actual_power_dbm = 24;
        } else if (power_dbm >= POWER_20_DBM) { // -> 20
            *sx_power = -6;
            *actual_power_dbm = 20;
        } else if (power_dbm >= POWER_17_DBM) { // -> 17
            *sx_power = -7;
            *actual_power_dbm = 17;
        } else if (power_dbm >= POWER_14_DBM) { // -> 14
            *sx_power = -12;
            *actual_power_dbm = 14;
        } else {
            *sx_power = -16;
            *actual_power_dbm = 10; // measures about 11 dBm
        }
    } else {
        uint8_t dac = 88;
        if (power_dbm >= POWER_30_DBM) { // -> 30
            dac = 70;
            *sx_power = 3;
            *actual_power_dbm = 30;
        } else if (power_dbm >= POWER_27_DBM) { // -> 27
            dac = 85;
            *sx_power = 0;
            *actual_power_dbm = 27;
        } else if (power_dbm >= POWER_24_DBM) { // -> 24
            *sx_power = -6;
            *actual_power_dbm = 24;
        } else if (power_dbm >= POWER_20_DBM) { // -> 20
            *sx_power = -9;
            *actual_power_dbm = 20;
        } else if (power_dbm >= POWER_17_DBM) { // -> 17
            *sx_power = -12;
            *actual_power_dbm = 17;
        } else if (power_dbm >= POWER_14_DBM) { // -> 14
            *sx_power = -15;
            *actual_power_dbm = 14;
        } else {
            dac = 150;
            *sx_power = -18;
            *actual_power_dbm = 10; // measures about 11 dBm
        }
        dacWrite(SX_PA_DAC_IO, dac);
    }
}

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_14_DBM, .mW = 25 },
    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};
