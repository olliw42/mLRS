//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************

/*
  Flashing ESP32C3 backpack WiFi bridge:
  - Board: Generic ESP32C3 Dev Module, use MODULE_ESP32C3_ELRS_TX target
  - First flash can be via web browser/ELRS WiFi
  - Need ELRS version of esptool from https://github.com/ExpressLRS/Backpack
  - Power up radio, plug in USB, select VCP
  - Run something like "Backpack/python/external/esptool/esptool.py --passthrough --port /dev/ttyACM0 --baud 115200 --before etx --after hard_reset
  - PUT ADDITIONAL LINES HERE

  Flashing ESP32 module:
  - Need ELRS python folder from https://github.com/ExpressLRS/ExpressLRS
  - Power up radio, plug in USB, select VCP
  - Run something like "python ExpressLRS/src/python/ETXinitPassthrough.py"
  - Use Visual Studio Code or esptool to flash via ACM serial port.
*/

//-------------------------------------------------------
// ESP32, RadioMaster Tx Internal GX12 LR1121
//-------------------------------------------------------
// https://github.com/ExpressLRS/targets/blob/master/TX/Radiomaster%20GX12.json

#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_NO_COM
#define DEVICE_HAS_NO_DEBUG
#define DEVICE_HAS_DIVERSITY_SINGLE_SPI
#define DEVICE_HAS_SINGLE_LED_RGB
#define DEVICE_HAS_FAN_ONOFF
#define DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL
#define DEVICE_HAS_ESP_WIFI_BRIDGE_CONFIGURE
#define DEVICE_HAS_ESP_WIFI_BRIDGE_ESP32C3
#define DEVICE_HAS_ESP_WIFI_BRIDGE_W_PASSTHRU_VIA_JRPIN5


//-- UARTS
// UARTB = serial port BT/ESP port
// UARTC = COM (CLI)
// UARTD = serial2
// UART  = JR bay pin5, full duplex CRSF serial connection to radio - code still calls it JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_SERIAL // serial
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX_IO           IO_P5
#define UARTB_USE_RX_IO           IO_P18
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define JR_PIN5_FULL_DUPLEX
#define UART_USE_SERIAL1 // full duplex CRSF/MBridge (JR pin5)
#define UART_BAUD                 400000
#define UART_USE_TX_IO            IO_P1
#define UART_USE_RX_IO            IO_P3
#define UART_TXBUFSIZE            0  // TX FIFO = 128
#define UART_RXBUFSIZE            0  // RX FIFO = 128 + 1


//-- SX1: LR11xx & SPI

#define SPI_CS_IO                 IO_P27
#define SPI_MISO                  IO_P33
#define SPI_MOSI                  IO_P32
#define SPI_SCK                   IO_P25
#define SPI_FREQUENCY             16000000L
#define SX_RESET                  IO_P15
#define SX_BUSY                   IO_P36
#define SX_DIO1                   IO_P37

#define SX_USE_REGULATOR_MODE_DCDC

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_DIO1, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_BUSY, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_LOW);
}

IRAM_ATTR bool sx_busy_read(void) { return (gpio_read_activehigh(SX_BUSY)) ? true : false; }

IRAM_ATTR void sx_amp_transmit(void) {}

IRAM_ATTR void sx_amp_receive(void) {}

void sx_dio_enable_exti_isr(void) { attachInterrupt(SX_DIO1, SX_DIO_EXTI_IRQHandler, RISING); }

void sx_dio_init_exti_isroff(void) { detachInterrupt(SX_DIO1); }

void sx_dio_exti_isr_clearflag(void) {}


//-- SX2: LR11xx & SPI

#define SX2_CS_IO                 IO_P13
#define SX2_BUSY                  IO_P39
#define SX2_DIO1                  IO_P34
#define SX2_RESET                 IO_P21

#define SX2_USE_REGULATOR_MODE_DCDC

IRQHANDLER(void SX2_DIO_EXTI_IRQHandler(void);)

void sx2_init_gpio(void)
{
    gpio_init(SX2_CS_IO, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX2_DIO1, IO_MODE_INPUT_ANALOG);
    gpio_init(SX2_BUSY, IO_MODE_INPUT_ANALOG);
    gpio_init(SX2_RESET, IO_MODE_OUTPUT_PP_LOW);
}

#define SX2_USE_REGULATOR_MODE_DCDC

IRAM_ATTR void spib_select(void) { gpio_low(SX2_CS_IO); }

IRAM_ATTR void spib_deselect(void) { gpio_high(SX2_CS_IO); }

IRAM_ATTR bool sx2_busy_read(void) { return (gpio_read_activehigh(SX2_BUSY)) ? true : false; }

IRAM_ATTR void sx2_amp_transmit(void) {}

IRAM_ATTR void sx2_amp_receive(void) {}

void sx2_dio_init_exti_isroff(void) { detachInterrupt(SX2_DIO1); }

void sx2_dio_enable_exti_isr(void) { attachInterrupt(SX2_DIO1, SX2_DIO_EXTI_IRQHandler, RISING); }

void sx2_dio_exti_isr_clearflag(void) {}


//-- Button

void button_init(void) {}
IRAM_ATTR bool button_pressed(void) { return false; }


//-- LEDs

#define LED_RGB                   IO_P22
#define LED_RGB_PIXEL_NUM         1
#include "../esp-hal-led-rgb.h"


//-- Cooling Fan

#define FAN_IO                    IO_P2

void fan_init(void) { analogWriteFrequency(25000); }

IRAM_ATTR void fan_set_power(int8_t power_dbm)
{
    if (power_dbm >= POWER_27_DBM) {
        analogWrite(FAN_IO, 255);
    } else if (power_dbm >= POWER_23_DBM) {
        analogWrite(FAN_IO, 127);
    } else {
        analogWrite(FAN_IO, 0);
    }
}


//-- ESP32 Wifi Bridge

#define ESP_RESET                 IO_P19 // backpack_en
#define ESP_GPIO0                 IO_P23 // backpack_boot inverted?
#define ESP_BOOT0                 IO_P0 // Will always be IO_P0

uint8_t esp_boot0()
{
    return gpio_read_activelow(ESP_BOOT0);
}

void esp_init(void)
{
    // No need to configure ESP_BOOT0 which will always be IO_P0 and is pull-up by default
    gpio_init(ESP_GPIO0, IO_MODE_OUTPUT_PP_LOW); // high -> esp will start in bootloader mode
    gpio_init(ESP_RESET, IO_MODE_OUTPUT_PP_LOW); // low -> esp is in reset
}

IRAM_ATTR void esp_reset_high(void) { gpio_high(ESP_RESET); }
IRAM_ATTR void esp_reset_low(void) { gpio_low(ESP_RESET); }

IRAM_ATTR void esp_gpio0_high(void) { gpio_low(ESP_GPIO0); }
IRAM_ATTR void esp_gpio0_low(void) { gpio_high(ESP_GPIO0); }


//-- POWER

#include "../../setup_types.h" // needed for frequency band condition in rfpower calc
#define SX_USE_LP_PA  // GX12 uses the low power amplifier for the 900 side, radio_rfo_hf option
#define SX_USE_RFSW_CTRL {31, 0, 20, 24, 24, 2, 0, 1} // radio_rfsw_ctrl array


void lr11xx_rfpower_calc(const int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm, const uint8_t frequency_band)
{
    if (frequency_band == SX_FHSS_CONFIG_FREQUENCY_BAND_2P4_GHZ) {  
        if (power_dbm >= POWER_30_DBM) {
            *sx_power = 2;
            *actual_power_dbm = 30;
        } else if (power_dbm >= POWER_27_DBM) {
            *sx_power = -2;
            *actual_power_dbm = 27;
        } else if (power_dbm >= POWER_24_DBM) {
            *sx_power = -6;
            *actual_power_dbm = 24;
        } else if (power_dbm >= POWER_20_DBM) {
            *sx_power = -10;
            *actual_power_dbm = 20;
        } else if (power_dbm >= POWER_17_DBM) {
            *sx_power = -15;
            *actual_power_dbm = 17;
        } else if (power_dbm >= POWER_14_DBM) {
            *sx_power = -18;
            *actual_power_dbm = 14;
        } else {
            *sx_power = -18;
            *actual_power_dbm = 10;
        }
    } else {
        uint8_t dac = 120;

        if (power_dbm >= POWER_30_DBM) {
            dac = 100;
            *sx_power = 7;
            *actual_power_dbm = 30;
        } else if (power_dbm >= POWER_27_DBM) {
            dac = 120;
            *sx_power = 0;
            *actual_power_dbm = 27;
        } else if (power_dbm >= POWER_24_DBM) {
            dac = 120;
            *sx_power = -5;
            *actual_power_dbm = 24;
        } else if (power_dbm >= POWER_20_DBM) {
            dac = 120;
            *sx_power = -9;
            *actual_power_dbm = 20;
        } else if (power_dbm >= POWER_17_DBM) {
            dac = 120;
            *sx_power = -12;
            *actual_power_dbm = 17;
        } else if (power_dbm >= POWER_14_DBM) {
            dac = 120;
            *sx_power = -15;
            *actual_power_dbm = 14;
        } else {
            dac = 120;
            *sx_power = -17;
            *actual_power_dbm = 10;
        }

        dacWrite(IO_P26, dac);  // power_apc_2
    }
}


#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    //{ .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_14_DBM, .mW = 25 },
    { .dbm = POWER_17_DBM, .mW = 50 }, // 6 power levels allowed
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};
