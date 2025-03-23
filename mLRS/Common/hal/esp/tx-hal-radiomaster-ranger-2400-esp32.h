//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************

/*
  Flashing ESP8285:
  - change ser dest to serial2
  - change ser baudrate to 115200
  - put Tx module into FLASH_ESP mode from tools
  - Board: Generic ESP8266 Module
  - Upload Speed: 115200
  - Reset Method: no dtr (aka ck)
*/

//-------------------------------------------------------
// ESP32, ELRS RADIOMASTER RANGER TX
//-------------------------------------------------------
// Ranger, "big" Ranger: https://github.com/ExpressLRS/targets/blob/master/TX/Radiomaster%20Ranger.json
// This could probably be expanded to also support Ranger Micro: https://github.com/ExpressLRS/targets/blob/master/TX/Radiomaster%20Ranger%20Micro.json

#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_SERIAL_OR_COM // hold 5-way in down direction at boot to enable CLI
#define DEVICE_HAS_IN
#define DEVICE_HAS_NO_DEBUG
//#define DEVICE_HAS_NO_SERIAL
//#define DEVICE_HAS_NO_COM

#define DEVICE_HAS_I2C_DISPLAY

#define DEVICE_HAS_FAN_ONOFF
#define DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2 // board has an ESP8285 wireless bridge with GPIO,RST, but no CONFIGURE for now

// Note on SERIAL_OR_COM:
// The com uart is not initialized, the serial uart is, So, buffers are set as by the RX/TXBUFSIZE defines for serial.
// The TXBUFSIZE setting for the com affects however the CLI's chunkenizer behavior.


//-- UARTS
// UARTB = serial port
// UARTC or USB = COM (CLI)
// UARTD = serial2 BT/ESP port
// UART  = JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_SERIAL // serial, is connected to USB-C via USB<>UART
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX_IO           IO_P1
#define UARTB_USE_RX_IO           IO_P3
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UARTC_USE_SERIAL // COM (CLI), is connected to USB-C via USB<>UART
#define UARTC_BAUD                115200
#define UARTC_USE_TX_IO           IO_P1
#define UARTC_USE_RX_IO           IO_P3
#define UARTC_TXBUFSIZE           0 // TX FIFO = 128
#define UARTC_RXBUFSIZE           TX_COM_RXBUFSIZE

#define UARTD_USE_SERIAL2 // serial2 BT/ESP
#define UARTD_BAUD                115200
#define UARTD_USE_TX_IO           IO_P17
#define UARTD_USE_RX_IO           IO_P16
#define UARTD_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTD_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UART_USE_SERIAL1 // JR bay pin5
#define UART_BAUD                 400000
#define UART_USE_TX_IO            IO_P13
#define UART_USE_RX_IO            IO_P13
#define UART_TXBUFSIZE            0 // TX FIFO = 128
#define UART_RXBUFSIZE            0 // RX FIFO = 128 + 1

#define UARTE_USE_SERIAL1 // in port, uses JRPin5
#define UARTE_BAUD                 100000
#define UARTE_USE_TX_IO            -1
#define UARTE_USE_RX_IO            IO_P13
#define UARTE_RXBUFSIZE            0 // RX FIFO = 128 + 1

#define UARTF_USE_SERIAL // debug, if needed, debug on USB; need to disable serial and com
#define UARTF_BAUD                115200
#define UARTF_USE_TX_IO           IO_P1
#define UARTF_USE_RX_IO           -1
#define UARTF_TXBUFSIZE           512


//-- SX1: SX12xx & SPI

#define SPI_CS_IO                 IO_P4
#define SPI_MISO                  IO_P19
#define SPI_MOSI                  IO_P23
#define SPI_SCK                   IO_P18
#define SPI_FREQUENCY             18000000L
#define SX_RESET                  IO_P5
#define SX_BUSY                   IO_P22
#define SX_DIO1                   IO_P21
#define SX_TX_EN                  IO_P33
#define SX_RX_EN                  IO_P32

#define SX_USE_REGULATOR_MODE_DCDC

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_DIO1, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH);
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


//-- In port

void in_init_gpio(void) {}

void in_set_normal(void) { gpio_matrix_in((gpio_num_t)UARTE_USE_RX_IO, U1RXD_IN_IDX, false); }

void in_set_inverted(void) { gpio_matrix_in((gpio_num_t)UARTE_USE_RX_IO, U1RXD_IN_IDX, true); }


//-- Button

void button_init(void) {}
IRAM_ATTR bool button_pressed(void) { return false; }


//-- LEDs

#define LED_RED                   IO_P15 // pin for both Ranger and Ranger Micro, even though they have different functionality

// Ranger, "big" Ranger have RGB LEDs, so we use our normal red/green

#include <NeoPixelBus.h>
bool ledRedState;
bool ledGreenState;
bool ledBlueState;

uint8_t pixelNum = 6;

NeoPixelBus<NeoGrbFeature, NeoEsp32I2s0Ws2812xMethod> ledRGB(pixelNum, LED_RED);

void leds_init(void)
{
    ledRGB.Begin();
    ledRGB.Show();
}

IRAM_ATTR void led_red_off(void)
{
    if (!ledRedState) return;
    ledRGB.SetPixelColor(0, RgbColor(0, 0, 0));
    ledRGB.Show();
    ledRedState = 0;
}

IRAM_ATTR void led_red_on(void)
{
    if (ledRedState) return;
    ledRGB.SetPixelColor(0, RgbColor(255, 0, 0));
    ledRGB.Show();
    ledRedState = 1;
}

IRAM_ATTR void led_red_toggle(void)
{
    if (ledRedState) { led_red_off(); } else { led_red_on(); }
}

IRAM_ATTR void led_green_off(void)
{
    if (!ledGreenState) return;
    ledRGB.SetPixelColor(1, RgbColor(0, 0, 0));
    ledRGB.Show();
    ledGreenState = 0;
}

IRAM_ATTR void led_green_on(void)
{
    if (ledGreenState) return;
    ledRGB.SetPixelColor(1, RgbColor(0, 255, 0));
    ledRGB.Show();
    ledGreenState = 1;
}

IRAM_ATTR void led_green_toggle(void)
{
    if (ledGreenState) { led_green_off(); } else { led_green_on(); }
}

IRAM_ATTR void led_blue_off(void)
{
    if (!ledBlueState) return;
    ledRGB.SetPixelColor(0, RgbColor(0, 0, 0));
    ledRGB.Show();
    ledBlueState = 0;
}

IRAM_ATTR void led_blue_on(void)
{
    if (ledBlueState) return;
    ledRGB.SetPixelColor(0, RgbColor(0, 0, 255));
    ledRGB.Show();
    ledBlueState = 1;
}

IRAM_ATTR void led_blue_toggle(void)
{
    if (ledBlueState) { led_blue_off(); } else { led_blue_on(); }
}


//-- Display I2C

#define I2C_SDA_IO                IO_P14
#define I2C_SCL_IO                IO_P12
#define I2C_CLOCKSPEED            1000000L  // fix - rather too much, but helps with LQ, ESP32 max speed
#define I2C_BUFFER_SIZE           1024


//-- 5 Way Switch

#define FIVEWAY_ADC_IO            IO_P39
#define KEY_UP_THRESH             3230
#define KEY_DOWN_THRESH           0
#define KEY_LEFT_THRESH           1890
#define KEY_RIGHT_THRESH          2623
#define KEY_CENTER_THRESH         1205

#if defined DEVICE_HAS_I2C_DISPLAY || defined DEVICE_HAS_I2C_DISPLAY_ROT180 || defined DEVICE_HAS_FIVEWAY

void fiveway_init(void) {} // no init needed to read an analog pin in Arduino

IRAM_ATTR uint16_t fiveway_adc_read(void)
{
    return analogRead(FIVEWAY_ADC_IO);
}

IRAM_ATTR uint8_t fiveway_read(void)
{
    int16_t adc = analogRead(FIVEWAY_ADC_IO);
    if (adc > (KEY_CENTER_THRESH-250) && adc < (KEY_CENTER_THRESH+250)) return (1 << KEY_CENTER);
    if (adc > (KEY_LEFT_THRESH-250) && adc < (KEY_LEFT_THRESH+250)) return (1 << KEY_LEFT); 
    if (adc > (KEY_DOWN_THRESH-250) && adc < (KEY_DOWN_THRESH+250)) return (1 << KEY_DOWN);
    if (adc > (KEY_UP_THRESH-250) && adc < (KEY_UP_THRESH+250)) return (1 << KEY_UP);
    if (adc > (KEY_RIGHT_THRESH-250) && adc < (KEY_RIGHT_THRESH+250)) return (1 << KEY_RIGHT);
    return 0;
}
#endif


//-- Serial or Com Switch
// use com if FIVEWAY is DOWN during power up, else use serial
// FIVEWAY-DONW becomes bind button later on

#ifdef DEVICE_HAS_SERIAL_OR_COM
bool tx_ser_or_com_serial = true; // we use serial as default

void ser_or_com_init(void)
{
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 16; i++) {
        int16_t adc = analogRead(FIVEWAY_ADC_IO);
        if (adc > (KEY_DOWN_THRESH-200) && adc < (KEY_DOWN_THRESH+200)) cnt++;
    }
    tx_ser_or_com_serial = !(cnt > 8);
}

IRAM_ATTR bool ser_or_com_serial(void)
{
    return tx_ser_or_com_serial;
}

IRAM_ATTR void ser_or_com_set_to_com(void)
{
    tx_ser_or_com_serial = false;
}
#endif


//-- Cooling Fan

#define FAN_IO                    IO_P2

void fan_init(void)
{
    analogWriteFrequency(25000);
}

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


//-- ESP8285 Wifi Bridge

#define ESP_RESET                 IO_P25 // backpack_en
#define ESP_GPIO0                 IO_P26 // backpack_boot, seems to be inverted
//#define ESP_DTR                   IO_PC14 // DTR from USB-TTL adapter -> GPIO
//#define ESP_RTS                   IO_PC3  // RTS from USB-TTL adapter -> RESET

#ifdef DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2
void esp_init(void)
{
    gpio_init(ESP_GPIO0, IO_MODE_OUTPUT_PP_LOW); // high -> esp will start in bootloader mode
    gpio_init(ESP_RESET, IO_MODE_OUTPUT_PP_LOW); // low -> esp is in reset
}

IRAM_ATTR void esp_reset_high(void) { gpio_high(ESP_RESET); }
IRAM_ATTR void esp_reset_low(void) { gpio_low(ESP_RESET); }

IRAM_ATTR void esp_gpio0_high(void) { gpio_low(ESP_GPIO0); }
IRAM_ATTR void esp_gpio0_low(void) { gpio_high(ESP_GPIO0); }

//IRAM_ATTR uint8_t esp_dtr_rts(void) { return 0; }
#endif


//-- POWER

#define POWER_GAIN_DBM            33 // gain of a PA stage if present
#define POWER_SX1280_MAX_DBM      SX1280_POWER_0_DBM  // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};

