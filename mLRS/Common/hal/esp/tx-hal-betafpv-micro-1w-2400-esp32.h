//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//*******************************************************

#define MLRS_FEATURE_OLED

//-------------------------------------------------------
// TX BETAFPV 2400 MICRO 1W
//-------------------------------------------------------
/*
    Pin Defs
    "serial_rx": 13
    "serial_tx": 13
    "radio_busy": 21
    "radio_dio1": 4
    "radio_miso": 19
    "radio_mosi": 23
    "radio_nss": 5
    "radio_rst": 14
    "radio_sck": 18
    "power_rxen": 27
    "power_txen": 26
    "power_lna_gain": 12
    "power_min": 1
    "power_high": 6
    "power_max": 6
    "power_default": 2
    "power_control": 0
    "power_values": [-18,-15,-12,-7,-4,2]
    "joystick": 25
    "joystick_values": [2839,2191,1616,3511,0,4095]
    "led_rgb": 16
    "led_rgb_isgrb": true
    "screen_sck": 32
    "screen_sda": 22
    "screen_type": 1
    "screen_reversed": 1
    "use_backpack": true
    "debug_backpack_baud": 460800
    "debug_backpack_rx": 3
    "debug_backpack_tx": 1
    "misc_fan_en": 17
*/


#define DEVICE_HAS_SINGLE_LED
//#define DEVICE_HAS_JRPIN5
//#define DEVICE_HAS_IN

#define DEVICE_HAS_SERIAL_OR_COM    // board has UART which is shared between Serial or Com, selected by e.g. a switch
//#define DEVICE_HAS_NO_SERIAL  
//#define DEVICE_HAS_NO_COM
#define DEVICE_HAS_NO_DEBUG
#define DEVICE_HAS_I2C_DISPLAY_ROT180
#define DEVICE_HAS_FAN_ONOFF
#define USE_FEATURE_MAVLINK_PARAMS // has no CLI, no Lua, hence needs this

//-- Timers, Timing, EEPROM, and such stuff

#define EE_START_PAGE             0

//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTC = debug port

#define UARTB_USE_SERIAL
#define UARTB_BAUD                57600
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UARTC_USE_SERIAL          // com USB/CLI
#define UARTC_BAUD                TX_COM_BAUDRATE
#define UARTC_TXBUFSIZE           TX_COM_TXBUFSIZE
#define UARTC_RXBUFSIZE           TX_COM_RXBUFSIZE

#define UARTF_USE_SERIAL
#define UARTF_BAUD                115200


//-- SX1: SX12xx & SPI
#define SPI_CS_IO                 IO_P5
#define SPI_MISO                  IO_P19
#define SPI_MOSI                  IO_P23
#define SPI_SCK                   IO_P18
#define SPI_FREQUENCY             18000000L
#define SX_RESET                  IO_P14
#define SX_BUSY                   IO_P21
#define SX_DIO1                   IO_P4
#define SX_TX_EN                  IO_P26
#define SX_RX_EN                  IO_P27

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

IRAM_ATTR void sx_dio_enable_exti_isr(void)
{
    attachInterrupt(SX_DIO1, SX_DIO_EXTI_IRQHandler, RISING);
}

void sx_dio_init_exti_isroff(void) {}
void sx_dio_exti_isr_clearflag(void) {}


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
#include <NeoPixelBus.h>
#define LED_RED                    16
bool ledRedState;

NeoPixelBus<NeoGrbFeature, NeoEsp32I2s0Ws2812xMethod> ledRGB(1, LED_RED);

IRAM_ATTR void leds_init(void)
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

//-- Display I2C
#define OLED_SDA                IO_P22
#define OLED_SCL                IO_P32
#define OLED_I2C_BUFFER_SIZE    1024
#define OLED_CLOCK_FREQ         400000L

//-- 5 Way Switch

#define FIVEWAY_ADC_IO          IO_P25
#define KEY_UP_THRESH           2839     
#define KEY_DOWN_THRESH         2191
#define KEY_LEFT_THRESH         1616
#define KEY_RIGHT_THRESH        3511
#define KEY_CENTER_THRESH       0

#if defined DEVICE_HAS_I2C_DISPLAY || defined DEVICE_HAS_I2C_DISPLAY_ROT180

// No init needed to read an analog pin in Arduino
void fiveway_init(void) { }

uint16_t fiveway_adc_read(void)
{
    return analogRead(FIVEWAY_ADC_IO);
}

uint8_t fiveway_read(void)
{
    uint16_t adc = analogRead(FIVEWAY_ADC_IO);
    if (adc < (KEY_CENTER_THRESH+200)) return (1 << KEY_CENTER); // 0
    if (adc > (KEY_LEFT_THRESH-200) && adc < (KEY_LEFT_THRESH+200)) return (1 << KEY_LEFT); 
    if (adc > (KEY_DOWN_THRESH-200) && adc < (KEY_DOWN_THRESH+200)) return (1 << KEY_DOWN);
    if (adc > (KEY_UP_THRESH-200) && adc < (KEY_UP_THRESH+200)) return (1 << KEY_UP);
    if (adc > (KEY_RIGHT_THRESH-200) && adc < (KEY_RIGHT_THRESH+200)) return (1 << KEY_RIGHT);
    return 0;
}
#endif

//-- Cooling Fan

#define FAN_IO      IO_P17

void fan_init(void)
{
    gpio_init(FAN_IO, IO_MODE_OUTPUT_PP_LOW);
    gpio_low(FAN_IO);
}

void fan_set_power(int8_t power_dbm)
{
    if (power_dbm >= POWER_23_DBM) {
        gpio_high(FAN_IO);
    } else {
        gpio_low(FAN_IO);
    }
}


//-- POWER
#define POWER_GAIN_DBM            28 // gain of a PA stage if present
#define POWER_SX1280_MAX_DBM      SX1280_POWER_3_DBM  // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
};

// #ifdef MLRS_FEATURE_OLED
//   #undef DEVICE_HAS_COM_ON_USB
//   #define DEVICE_HAS_NO_COM
//   #define DEVICE_HAS_I2C_DISPLAY_ROT180
// #endif

//-- Serial or Com Switch
// use com if FIVEWAY is DOWN during power up, else use serial
// FIVEWAY-DONW becomes bind button later on

#ifdef DEVICE_HAS_SERIAL_OR_COM
bool tx_ser_or_com_serial = true; // we use serial as default

void ser_or_com_init(void)
{
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < 16; i++) {

    uint16_t adc = analogRead(FIVEWAY_ADC_IO);
    if (adc > (KEY_DOWN_THRESH-200) && adc < (KEY_DOWN_THRESH+200)) cnt++;
  }
  tx_ser_or_com_serial = !(cnt > 8);
}

bool ser_or_com_serial(void)
{
  return tx_ser_or_com_serial;
}
#endif


// //-- POWER

// #define POWER_GAIN_DBM            24 // 35 // gain of a PA stage if present // datasheet of SKY66312-11 says 35dB !
// #define POWER_SX1280_MAX_DBM      SX1280_POWER_6_DBM //SX1280_POWER_m3_DBM // maximum allowed sx power
// #define POWER_USE_DEFAULT_RFPOWER_CALC

// #define RFPOWER_DEFAULT           1 // index into rfpower_list array

// const rfpower_t rfpower_list[] = {
//     { .dbm = POWER_MIN, .mW = INT8_MIN },
//     { .dbm = POWER_20_DBM, .mW = 100 },
//     { .dbm = POWER_24_DBM, .mW = 250 },
//     { .dbm = POWER_27_DBM, .mW = 500 },
//     { .dbm = POWER_30_DBM, .mW = 1000 },
// };




