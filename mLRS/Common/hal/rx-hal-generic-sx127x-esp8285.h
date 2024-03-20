//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// R9MX RX Module STM32L433CB
//-------------------------------------------------------
// https://github.com/ExpressLRS/ExpressLRS/blob/master/src/include/target/Frsky_RX_R9M.h
// many THX to the ExpressLRS project !
// Connection pads:
//   Pin1 GND
//   Pin2 VCC
//   Pin3 SPort/FPort PA5  (looks strange)
//   Pin4 Inv SPort   PB11
//   Pin5 SBusOut     PA2 / U2_Tx inverted
//   Pin6 SBusIn      ???
//   Ch1    PA8           -> Buzzer (TIM1)
//   Ch2    PA9 / U1Tx    -> Serial Tx
//   Ch3    PA10 / U1Rx   -> Serial Rx
//   Ch4    PA11          -> Debug Tx (TIM15)

#define DEVICE_HAS_SYSTEMBOOT

#define DEVICE_HAS_SERIAL_OR_DEBUG

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

#define UART_USE_UART2_PA2PA3 // out pin
#define UART_BAUD                 100000 // SBus normal baud rate, is being set later anyhow
#define UART_USE_TX
#define UART_TXBUFSIZE            256 // 512
#define UART_USE_TX_ISR
//#define UART_USE_RX
//#define UART_RXBUFSIZE            512
#define OUT_UARTx                 USART2 // UART_UARTx is not known yet, so define by hand

//#define SWUART_USE_TIM15 // debug
#define SWUART_TX_IO              10
#define SWUART_BAUD               57600
#define SWUART_USE_TX
#define SWUART_TXBUFSIZE          512

//-- SX1: SX12xx & SPI
#define SPI_CS_IO                 15
#define SPI_FREQUENCY             10000000L

#define SX_RESET                  2
#define SX_DIO0                   4
#define SX_DIO1                   5

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
extern HAL_TickFreqTypeDef uwTickFreq = HAL_TICK_FREQ_1KHZ; // For esp we will call tick increment every 1ms

void sx_init_gpio(void)
{
    pinMode(SX_RESET, OUTPUT);
    pinMode(SX_DIO0, INPUT);

    digitalWrite(SX_RESET, HIGH);
} 

void sx_amp_transmit(void) { }

void sx_amp_receive(void) { }

void sx_dio_init_exti_isroff(void) { }

void sx_dio_enable_exti_isr(void)
{
    attachInterrupt(SX_DIO0, SX_DIO_EXTI_IRQHandler, RISING);
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

void led_green_off(void) {  }
void led_green_on(void) {  }
void led_green_toggle(void) {  }

void led_red_off(void) { gpio_high(LED_RED); }
void led_red_on(void) { gpio_low(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }

//-- SystemBootLoader

void systembootloader_init(void)
{
    // Not needed on the ESP chips, this built in.
}


//-- POWER

#define POWER_GAIN_DBM            0 // gain of a PA stage if present
#define POWER_SX1276_MAX_DBM      SX1276_OUTPUT_POWER_MAX // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_0_DBM, .mW = 1 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_17_DBM, .mW = 50 },
};
