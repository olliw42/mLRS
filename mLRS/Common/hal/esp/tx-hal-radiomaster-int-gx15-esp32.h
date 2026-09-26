//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************


//-------------------------------------------------------
// ESP32, RadioMaster Tx GX15, Internal Gemini SX128x 2400, diversity
//-------------------------------------------------------
// https://github.com/ExpressLRS/targets/blob/master/targets.json  "gx15"
// https://github.com/ExpressLRS/targets/blob/master/TX/Generic%202400%20Gemini.json
// GX15 = Generic 2400 Gemini layout, with rx_en pins and power values overlayed

#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_DIVERSITY_SINGLE_SPI
#define DEVICE_HAS_SINGLE_LED_RGB
#define DEVICE_HAS_FAN_ONOFF
#define DEVICE_HAS_NO_SERIAL
#define DEVICE_HAS_NO_COM
#define DEVICE_HAS_ESP_WIFI_BRIDGE_ESP32C3
#define DEVICE_HAS_ESP_WIFI_BRIDGE_CONFIGURE
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
#define UARTD_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTD_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UART_USE_SERIAL1 // full duplex CRSF/MBridge (JR pin5)
#define UART_BAUD                 400000
#define UART_USE_TX_IO            IO_P1
#define UART_USE_RX_IO            IO_P3
#define UART_TXBUFSIZE            0  // TX FIFO = 128
#define UART_RXBUFSIZE            0  // RX FIFO = 128 + 1

#define JR_PIN5_FULL_DUPLEX


//-- SX1: SX128x & SPI

#define SPI_CS_IO                 IO_P27
#define SPI_MISO                  IO_P33
#define SPI_MOSI                  IO_P32
#define SPI_SCK                   IO_P25
#define SPI_FREQUENCY             18000000L
#define SX_RESET                  IO_P26
#define SX_DIO                    IO_P37
#define SX_BUSY                   IO_P36
#define SX_RX_EN                  IO_P12
#define SX_TX_EN                  IO_P14

#define SX_USE_REGULATOR_MODE_DCDC

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_DIO, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW);
}

IRAM_ATTR bool sx_busy_read(void) { return (gpio_read_activehigh(SX_BUSY)) ? true : false; }

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

void sx_dio_enable_exti_isr(void) { attachInterrupt(SX_DIO, SX_DIO_EXTI_IRQHandler, RISING); }
void sx_dio_init_exti_isroff(void) { detachInterrupt(SX_DIO); }
IRAM_ATTR void sx_dio_exti_isr_clearflag(void) {}


//-- SX2: SX128x & SPI

#define SX2_CS_IO                 IO_P13
#define SX2_RESET                 IO_P21
#define SX2_DIO                   IO_P34
#define SX2_BUSY                  IO_P39
#define SX2_RX_EN                 IO_P2
#define SX2_TX_EN                 IO_P15

#define SX2_USE_REGULATOR_MODE_DCDC

IRQHANDLER(void SX2_DIO_EXTI_IRQHandler(void);)

void sx2_init_gpio(void)
{
    gpio_init(SX2_CS_IO, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX2_RESET, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX2_DIO, IO_MODE_INPUT_ANALOG);
    gpio_init(SX2_BUSY, IO_MODE_INPUT_PU);
    gpio_init(SX2_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX2_RX_EN, IO_MODE_OUTPUT_PP_LOW);
}

IRAM_ATTR void spib_select(void) { gpio_low(SX2_CS_IO); }
IRAM_ATTR void spib_deselect(void) { gpio_high(SX2_CS_IO); }

IRAM_ATTR bool sx2_busy_read(void) { return (gpio_read_activehigh(SX2_BUSY)) ? true : false; }

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

void sx2_dio_init_exti_isroff(void) { detachInterrupt(SX2_DIO); }
void sx2_dio_enable_exti_isr(void) { attachInterrupt(SX2_DIO, SX2_DIO_EXTI_IRQHandler, RISING); }
IRAM_ATTR void sx2_dio_exti_isr_clearflag(void) {}


//-- Button

void button_init(void) {}
IRAM_ATTR bool button_pressed(void) { return false; }


//-- LEDs

#define LED_RGB                   IO_P22
#define LED_RGB_PIXEL_NUM         1
#include "esp-hal-led-rgb.h"


//-- Cooling Fan

#define FAN_IO                    IO_P4

void fan_init(void) { analogWriteFrequency(25000); }

IRAM_ATTR void fan_set_power(int8_t power_dbm)
{
    if (power_dbm >= POWER_24_DBM) {
        analogWrite(FAN_IO, 255);
    } else if (power_dbm >= POWER_20_DBM) {
        analogWrite(FAN_IO, 127);
    } else {
        analogWrite(FAN_IO, 0);
    }
}


//-- ESP32 Wifi Bridge

#define ESP_RESET                 IO_P19 // backpack_en
#define ESP_GPIO0                 IO_P23 // backpack_boot inverted?
#define ESP_BOOTPIN               IO_P0  // will always be IO_P0

uint8_t esp_bootpin()
{
    return gpio_read_activelow(ESP_BOOTPIN);
}

void esp_init(void)
{
    // no need to configure ESP_BOOTPIN which will always be IO_P0 and is pull-up by default
    gpio_init(ESP_GPIO0, IO_MODE_OUTPUT_PP_LOW); // high -> esp will start in bootloader mode
    gpio_init(ESP_RESET, IO_MODE_OUTPUT_PP_LOW); // low -> esp is in reset
}

IRAM_ATTR void esp_reset_high(void) { gpio_high(ESP_RESET); }
IRAM_ATTR void esp_reset_low(void) { gpio_low(ESP_RESET); }

IRAM_ATTR void esp_gpio0_high(void) { gpio_low(ESP_GPIO0); }
IRAM_ATTR void esp_gpio0_low(void) { gpio_high(ESP_GPIO0); }


//-- POWER

// ELRS power_values [-18,-16,-13,-10,-5] for 10,25,50,100,250 mW, i.e. 28...30 dB PA gain
void sx128x_rfpower_calc(const int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm)
{
    if (power_dbm >= POWER_24_DBM) {
        *sx_power = -5;
        *actual_power_dbm = 24;
    } else if (power_dbm >= POWER_20_DBM) {
        *sx_power = -10;
        *actual_power_dbm = 20;
    } else if (power_dbm >= POWER_17_DBM) {
        *sx_power = -13;
        *actual_power_dbm = 17;
    } else if (power_dbm >= POWER_14_DBM) {
        *sx_power = -16;
        *actual_power_dbm = 14;
    } else {
        *sx_power = -18;
        *actual_power_dbm = 10;
    }
}

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_14_DBM, .mW = 25 },
    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
};