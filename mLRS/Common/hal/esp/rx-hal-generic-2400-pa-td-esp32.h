//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// GENERIC 2400 True Diversity PA RX
//-------------------------------------------------------

#define DEVICE_HAS_SINGLE_LED
//#define DEVICE_HAS_SERIAL_OR_DEBUG
#define DEVICE_HAS_NO_DEBUG

//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define SYSTICK_TIMESTEP          1000
#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)

#define EE_START_PAGE             0 // 128 kB flash, 2 kB page

#define MICROS_TIMx               TIM15

//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTC = debug port

#define UARTB_USE_SERIAL
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE // 1024 // 512
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE // 1024 // 512

#define UARTC_USE_SERIAL
#define UARTC_BAUD                  115200

//#define SWUART_USE_TIM15 // debug
#define SWUART_TX_IO              10
#define SWUART_BAUD               57600
#define SWUART_USE_TX
#define SWUART_TXBUFSIZE          512

//-- SX1: SX12xx & SPI

#define MISO                      33
#define MOSI                      32
#define SCK                       25
#define SPI_FREQUENCY             16000000L

#define SX_BUSY                   36
#define SX_DIO1                   37
#define SX_NSS                    27
#define SX_RESET                  26

#define SX_RX_EN                  10
#define SX_TX_EN                  14

#define SX_BUSY_2                 39
#define SX_DIO1_2                 34
#define SX_NSS_2                  13
#define SX_RESET_2                21

#define SX_RX_EN_2                 9
#define SX_TX_EN_2                15


IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    pinMode(SX_DIO1, INPUT);
    pinMode(SX_BUSY, INPUT_PULLUP);
    pinMode(SX_RESET, OUTPUT);
    pinMode(SX_NSS, OUTPUT);
    pinMode(SX_RX_EN, OUTPUT);
    pinMode(SX_TX_EN, OUTPUT);
    
    digitalWrite(SX_NSS, HIGH);
    digitalWrite(SX_RESET, LOW);

    pinMode(SX_DIO1_2, INPUT);
    pinMode(SX_BUSY_2, INPUT_PULLUP);
    pinMode(SX_RESET_2, OUTPUT);
    pinMode(SX_NSS_2, OUTPUT);
    pinMode(SX_RX_EN_2, OUTPUT);
    pinMode(SX_TX_EN_2, OUTPUT);
    
    digitalWrite(SX_NSS_2, HIGH);
    digitalWrite(SX_RESET_2, LOW);

    pinMode(SX_RX_EN_2, LOW);
    pinMode(SX_TX_EN_2, LOW );
}

bool sx_busy_read(void)
{
    return (digitalRead(SX_BUSY) == HIGH) ? true : false;
}

void sx_amp_transmit(void)
{
    digitalWrite(SX_RX_EN, LOW);
    digitalWrite(SX_TX_EN, HIGH);
}

void sx_amp_receive(void)
{
    digitalWrite(SX_TX_EN, LOW);
    digitalWrite(SX_RX_EN, HIGH);
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

#define BUTTON                    0

void button_init(void)
{
    pinMode(BUTTON, INPUT_PULLUP);
}

bool button_pressed(void)
{
    return (digitalRead(BUTTON) == HIGH) ? false : true;
}

//-- LEDs
#include <NeoPixelBus.h>
#define LED_RED                    22
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

//-- POWER

#define POWER_GAIN_DBM            18 // gain of a PA stage if present
#define POWER_SX1280_MAX_DBM      SX1280_POWER_3_DBM  // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_0_DBM, .mW =  1 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
};
