//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// BAYCKRC 900 PA RX
//-------------------------------------------------------

#define DEVICE_HAS_SINGLE_LED
#define DEVICE_HAS_NO_DEBUG
//#define DEVICE_HAS_SERIAL_OR_DEBUG

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
#define UARTB_BAUD                  RX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE             RX_SERIAL_TXBUFSIZE // 1024 // 512
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE             RX_SERIAL_RXBUFSIZE // 1024 // 512

#define UARTC_USE_SERIAL
#define UARTC_BAUD                  115200


//-- SX1: SX12xx & SPI
#define SPI_CS_IO                 15
#define SPI_FREQUENCY             10000000L
#define SX_RESET                  2
#define SX_DIO0                   4
#define SX_DIO1                   5

IRQHANDLER(void IRAM_ATTR SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    pinMode(SX_RESET, OUTPUT);
    pinMode(SX_DIO0, INPUT);

    digitalWrite(SX_RESET, HIGH);
} 

void sx_dio_enable_exti_isr(void)
{
    attachInterrupt(SX_DIO0, SX_DIO_EXTI_IRQHandler, RISING);
}

void sx_amp_transmit(void) {}
void sx_amp_receive(void) {}
void sx_dio_init_exti_isroff(void) {}
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
#define LED_RED                   16

void leds_init(void)
{
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, LOW);// LED_RED_OFF
}

void led_red_off(void) { gpio_low(LED_RED); }
void led_red_on(void) { gpio_high(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }

void led_green_off(void) {}
void led_green_on(void) {}
void led_green_toggle(void) {}


//-- POWER
#define POWER_GAIN_DBM            13 // gain of a PA stage if present
#define POWER_SX1276_MAX_DBM      SX1276_OUTPUT_POWER_MAX // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
};