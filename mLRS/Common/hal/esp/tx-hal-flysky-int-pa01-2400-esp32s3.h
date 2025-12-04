//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************

//-------------------------------------------------------
// ESP32S3, Flysky PA01 Tx Internal ELRS module
// Backpack: Generic ESP32C3 Module
//-------------------------------------------------------
// https://github.com/ExpressLRS/targets/blob/master/TX/FLYSKY%20PA01.json
/*
  Flashing ESP32S3 module:
  - Need ELRS python folder from https://github.com/ExpressLRS/ExpressLRS
  - Power up radio, plug in USB, select VCP
  - Run "python ExpressLRS/src/python/ETXinitPassthrough.py"
  - Use Visual Studio Code to flash via edgetx VCP.
  - Set the Internal RF to CRSF 400k baud in SYS->HARDWARE on EdgeTx radio

  Flashing ESP32C3 backpack WiFi bridge:
  - Board: Generic ESP32C3 Module
  - Download backpack passthrough tool : https://github.com/heuyck/ExpressLRS/blob/edgetx-passthrough-backpack/src/python/edgetx-passthrough-backpack.py
  - Power up radio, plug in USB, select VCP
  - Run "python edgetx-passthrough-backpack.py" to open passthrough from edgetx VCP to backpack.
  - Use esptool to upload firmware or use Arduino to compile and upload via edgetx VCP.
*/

#define DEVICE_HAS_JRPIN5
#define JR_PIN5_FULL_DUPLEX       // internal module
#define DEVICE_HAS_SINGLE_LED
#define DEVICE_HAS_NO_COM
#define DEVICE_HAS_NO_DEBUG
#define DEVICE_HAS_FAN_ONOFF
#define DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL
#define DEVICE_HAS_ESP_WIFI_BRIDGE_W_PASSTHRU_VIA_JRPIN5
#define DEVICE_HAS_ESP_WIFI_BRIDGE_CONFIGURE
#define DEVICE_HAS_ESP_WIFI_BRIDGE_ESP32C3

//-- UARTS
// UARTB = serial port BT/ESP port
// UARTC = COM (CLI)
// UARTD = serial2
// UART  = JR bay pin5, full duplex CRSF serial connection to radio - code still calls it JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_SERIAL // serial
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX_IO           IO_P17
#define UARTB_USE_RX_IO           IO_P18
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE // 512
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE // 2048

#define UART_USE_SERIAL1 // full duplex CRSF/MBridge (JR pin5)
#define UART_BAUD                 400000
#define UART_USE_TX_IO            IO_P43
#define UART_USE_RX_IO            IO_P44
#define UART_TXBUFSIZE            0 // TX_SERIAL_TXBUFSIZE  // 512
#define UART_RXBUFSIZE            0 // TX_SERIAL_RXBUFSIZE  // 2048


//-- SX1: SX12xx & SPI

#define SPI_CS_IO                 IO_P10
#define SPI_MISO                  IO_P13
#define SPI_MOSI                  IO_P11
#define SPI_SCK                   IO_P12
#define SPI_FREQUENCY             18000000L
#define SX_RESET                  IO_P14
#define SX_BUSY                   IO_P9
#define SX_DIO1                   IO_P8
#define SX_TX_EN                  IO_P6
#define SX_RX_EN                  IO_P7
#define SX_USE_REGULATOR_MODE_DCDC

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_DIO1, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH);
#ifdef PWR_EN
    gpio_init(PWR_EN, IO_MODE_OUTPUT_PP_HIGH);  // front-end enable
#endif
}

IRAM_ATTR bool sx_busy_read(void)
{
    return (gpio_read_activehigh(SX_BUSY)) ? true : false;
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

void sx_dio_enable_exti_isr(void)
{
    attachInterrupt(SX_DIO1, SX_DIO_EXTI_IRQHandler, RISING);
}

void sx_dio_init_exti_isroff(void)
{
    detachInterrupt(SX_DIO1);
}

void sx_dio_exti_isr_clearflag(void) {}


//-- Button

void button_init(void) {}
IRAM_ATTR bool button_pressed(void) { return false; }


//-- LEDs
#define LED_RED                   IO_P3

void leds_init(void)
{
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_HIGH);
}

IRAM_ATTR void led_red_off(void) { gpio_low(LED_RED); }
IRAM_ATTR void led_red_on(void) { gpio_high(LED_RED); }
IRAM_ATTR void led_red_toggle(void) { gpio_toggle(LED_RED); }

//-- Cooling Fan

#define FAN_IO                    IO_P1

void fan_init(void)
{
    analogWriteFrequency(25000);
}

IRAM_ATTR void fan_set_power(int8_t power_dbm)
{
  //"misc_fan_speeds": [90,95,110,125,140,160]
    switch (power_dbm)
    {
        case POWER_10_DBM: analogWrite(FAN_IO, 0); break;  // no fan needed at 10 mW
        case POWER_14_DBM: analogWrite(FAN_IO, 95); break; 
        case POWER_17_DBM: analogWrite(FAN_IO, 110); break;
        case POWER_20_DBM: analogWrite(FAN_IO, 125); break;
        case POWER_24_DBM: analogWrite(FAN_IO, 140); break;
        case POWER_27_DBM: analogWrite(FAN_IO, 160); break;
        default:
        case POWER_30_DBM: analogWrite(FAN_IO, 255); break;
    }
}


//-- ESP32C3 Wifi Bridge
#define ESP_RESET                 IO_P34 // backpack_en
#define ESP_GPIO0                 IO_P15  // backpack_boot inverted?
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
#define POWER_GAIN_DBM            24 // gain of a PA stage if present
#define POWER_SX1280_MAX_DBM      SX1280_POWER_6_DBM  // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_14_DBM, .mW = 25 },
    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 }
};

