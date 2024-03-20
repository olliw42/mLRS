//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// GENERIC 2400 PA RX
//-------------------------------------------------------

#define DEVICE_HAS_SERIAL_OR_DEBUG
#define DEVICE_HAS_SYSTEMBOOT
//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define SYSTICK_TIMESTEP          1000
#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)

#define EE_START_PAGE             0 // 128 kB flash, 2 kB page

#define MICROS_TIMx               TIM15

//-------------------------------------------------------

// https://forum.arduino.cc/t/very-short-delays/43445
// You can "waste" one cycle (62.5ns on a 16MHz Arduino) with this inline assembly instruction
#define __NOP() __asm__("nop\n\t")

// #define CLOCK_TIMx                TIM2
// #define CLOCK_IRQn                TIM2_IRQn
// #define CLOCK_IRQHandler          BLAH1


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTC = debug port

#define UARTB_USE_UART1_PA9PA10 // serial
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

//#define SPI_USE_SPI2              // PB13, PB14, PB15
#define SPI_CS_IO                 15
#define SPI_FREQUENCY             10000000L

#define SX_RESET                  2
#define SX_BUSY                   5
#define SX_DIO1                   4
#define SX_TX_EN                  10
//#define SX_RX_EN

#define PA_ANTENNA                9


IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

typedef enum
{
  HAL_TICK_FREQ_10HZ         = 100U,
  HAL_TICK_FREQ_100HZ        = 10U,
  HAL_TICK_FREQ_1KHZ         = 1U,
  HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;

typedef enum
{
    HAL_OK = 0x00,
    HAL_ERROR = 0x01,
    HAL_BUSY = 0x02,
    HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

#define     __IO    volatile             /*!< Defines 'read / write' permissions */

inline uint32_t uwTick;
extern uint32_t uwTickPrio;
extern HAL_TickFreqTypeDef uwTickFreq = HAL_TICK_FREQ_1KHZ;  // For esp we will call tick increment every 1ms

void sx_init_gpio(void)
{
    pinMode(SX_DIO1, INPUT);
    pinMode(SX_BUSY, INPUT_PULLUP);
    pinMode(SX_TX_EN, OUTPUT);
    pinMode(SX_RESET, OUTPUT);
    pinMode(PA_ANTENNA, OUTPUT);

    digitalWrite(SX_RESET, HIGH);
    digitalWrite(PA_ANTENNA, HIGH); // Force to Antenna 2
}

bool sx_busy_read(void)
{
    return digitalRead(SX_BUSY) ? true : false;
}

void sx_amp_transmit(void)
{
    digitalWrite(SX_TX_EN, HIGH);
}

void sx_amp_receive(void)
{
    digitalWrite(SX_TX_EN, LOW);
}

void sx_dio_init_exti_isroff(void){ }

void sx_dio_enable_exti_isr(void)
{
    attachInterrupt(SX_DIO1, SX_DIO_EXTI_IRQHandler, RISING);
}

void sx_dio_exti_isr_clearflag(void) { }

//-- Button

#define BUTTON                    0

void button_init(void)
{
    pinMode(BUTTON, INPUT_PULLUP);
}

bool button_pressed(void)
{
    return digitalRead(BUTTON) ? false : true;
}

//-- LEDs
#define LED_RED                   16

void leds_init(void)
{
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, HIGH);// LED_RED_OFF
}

void led_green_off(void) { }
void led_green_on(void) { }
void led_green_toggle(void) { }

void led_red_off(void) { gpio_high(LED_RED); }
void led_red_on(void) { gpio_low(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }

//-- SystemBootLoader

void systembootloader_init(void)
{
    // Not needed on the ESP chips, this built in.
}

//-- POWER

#define POWER_GAIN_DBM            23 // gain of a PA stage if present
#define POWER_SX1280_MAX_DBM      SX1280_POWER_3_DBM  // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_23_DBM, .mW = 200 },
};
