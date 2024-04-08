//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// ESP32, ELRS GENERIC 2400 PA RX
//-------------------------------------------------------

#define DEVICE_HAS_SINGLE_LED
//#define DEVICE_HAS_NO_DEBUG
#define DEVICE_HAS_SERIAL_OR_DEBUG


//-- Timers, Timing, EEPROM, and such stuff

#define SYSTICK_TIMESTEP          1000
#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)

#define EE_START_PAGE             0


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTC = debug port

#define UARTB_USE_UART1_PA9PA10 // serial
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE

#define UARTC_USE_SERIAL
#define UARTC_BAUD                115200

//#define SWUART_USE_TIM15 // debug
#define SWUART_TX_IO              10
#define SWUART_BAUD               57600
#define SWUART_USE_TX
#define SWUART_TXBUFSIZE          512

//-- SX1: SX12xx & SPI

//#define SPI_USE_SPI2              // PB13, PB14, PB15
// #define SPI_MISO                  33
// #define SPI_MOSI                  32
// #define SPI_SCK                   25
#define SPI_CS_IO                 27
#define HSPI_MISO 33
#define HSPI_MOSI 32
#define HSPI_SCLK 25
#define HSPI_SS   27
#define SPI_FREQUENCY             10000000L

#define SX_RESET                  26
#define SX_BUSY                   36
#define SX_DIO1                   37
#define SX_TX_EN                  14
//#define SX_RX_EN                  

#define PA_ANTENNA                9

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_DIO1, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH);
    
    gpio_init(PA_ANTENNA, IO_MODE_OUTPUT_PP_HIGH); // force to antenna 2
}

IRAM_ATTR bool sx_busy_read(void)
{
    return (gpio_read_activehigh(SX_BUSY)) ? true : false;
}

IRAM_ATTR void sx_amp_transmit(void)
{
    gpio_high(SX_TX_EN);
}

IRAM_ATTR void sx_amp_receive(void)
{
    gpio_low(SX_TX_EN);
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
#include <FastLED.h>
#define DATA_PIN                    22
#define NUM_LEDS                    1
#define BRIGHTNESS                  127
CRGB fastleds[NUM_LEDS];
uint8_t fastleds_state;

void leds_init(void)
{
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(fastleds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
    fastleds[0] = CRGB::Black; 
    FastLED.show();
    fastleds_state = 0;
}

void led_red_off(void) 
{
    fastleds[0] = CRGB::Black; 
    FastLED.show();
    fastleds_state = 0; 
}

void led_red_on(void) 
{ 
    fastleds[0] = CRGB::Red; 
    FastLED.show();
    fastleds_state = 1;  
}

void led_red_toggle(void) 
{ 
    if (fastleds_state) { led_red_off(); } else { led_red_on(); }
}

void led_green_off(void) {}
void led_green_on(void) {}
void led_green_toggle(void) {}


//-- POWER
#define POWER_GAIN_DBM            23 // gain of a PA stage if present
#define POWER_SX1280_MAX_DBM      SX1280_POWER_0_DBM  // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_23_DBM, .mW = 200 },
};
