//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//*******************************************************
//
//-------------------------------------------------------
// R9MX TX Module STM32L433CB
//-------------------------------------------------------
// https://github.com/ExpressLRS/ExpressLRS/blob/master/src/include/target/Frsky_RX_R9M.h
// many THX to the ExpressLRS project !
// Connection pads:
//   Pin1 GND
//   Pin2 VCC
//   Pin3 SPort/FPort PA5  (looks strange)
//   Pin4 Inv SPort   PB11
//   Pin5 SBusOut     PA2 inverted
//   Pin6 SBusIn      ???
//   Ch1    PA11          -> Debug TX
//   Ch2    PA10 / U1Rx   -> Serial Rx
//   Ch3    PA9 / U1Tx    -> Serial Tx
//   Ch4    PA8           -> Buzzer

#define DEVICE_HAS_IN
#define DEVICE_HAS_SERIAL_OR_COM // serial or com is selected by pressing BUTTON during power on
#define DEVICE_HAS_DEBUG_SWUART
// #define DEVICE_HAS_BUZZER


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define SYSTICK_TIMESTEP          1000
#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)

#define EE_START_PAGE             60 // 128 kB flash, 2 kB page


//-- UARTS
// UARTB = serial port or COM (CLI)
// UARTC = --
// UART = --
// UARTE = in port, SBus or whatever
// UARTF = --
// SWUART= debug port

#define UARTB_USE_UART1 // serial or COM (CLI)
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           TX_COM_TXBUFSIZE // TX_SERIAL_TXBUFSIZE // choose the bigger one
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UARTE_USE_UART3 // in port // PB11, pin4 Inv SPort
#define UARTE_BAUD                100000 // SBus normal baud rate, is being set later anyhow
//#define UARTE_USE_TX
//#define UARTE_TXBUFSIZE           512
//#define UARTE_USE_TX_ISR
#define UARTE_USE_RX
#define UARTE_RXBUFSIZE           512

#define SWUART_USE_TIM15 // debug
#define SWUART_TX_IO              IO_PA11
#define SWUART_BAUD               115200
#define SWUART_USE_TX
#define SWUART_TXBUFSIZE          512
//#define SWUART_TIM_IRQ_PRIORITY   11


//-- SX1: SX12xx & SPI

#define SPI_USE_SPI2              // PB13, PB14, PB15
#define SPI_CS_IO                 IO_PB12
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PC14
#define SX_DIO0                   IO_PA15
#define SX_DIO1                   // IO_PA1 ???
#define SX_RX_EN                  //
#define SX_TX_EN                  //

#define SX_DIO0_SYSCFG_EXTI_PORTx     LL_SYSCFG_EXTI_PORTA
#define SX_DIO0_SYSCFG_EXTI_LINEx     LL_SYSCFG_EXTI_LINE15
#define SX_DIO_EXTI_LINE_x            LL_EXTI_LINE_15
#define SX_DIO_EXTI_IRQn              EXTI15_10_IRQn
#define SX_DIO_EXTI_IRQHandler        EXTI15_10_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY    11

void sx_init_gpio(void)
{
  gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
  gpio_init(SX_DIO0, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
}

void sx_amp_transmit(void)
{
}

void sx_amp_receive(void)
{
}

void sx_dio_init_exti_isroff(void)
{
  LL_SYSCFG_SetEXTISource(SX_DIO0_SYSCFG_EXTI_PORTx, SX_DIO0_SYSCFG_EXTI_LINEx);

  // let's not use LL_EXTI_Init(), but let's do it by hand, is easier to allow enabling isr later
  LL_EXTI_DisableEvent_0_31(SX_DIO_EXTI_LINE_x);
  LL_EXTI_DisableIT_0_31(SX_DIO_EXTI_LINE_x);
  LL_EXTI_DisableFallingTrig_0_31(SX_DIO_EXTI_LINE_x);
  LL_EXTI_EnableRisingTrig_0_31(SX_DIO_EXTI_LINE_x);

  NVIC_SetPriority(SX_DIO_EXTI_IRQn, SX_DIO_EXTI_IRQ_PRIORITY);
  NVIC_EnableIRQ(SX_DIO_EXTI_IRQn);
}

void sx_dio_enable_exti_isr(void)
{
  LL_EXTI_ClearFlag_0_31(SX_DIO_EXTI_LINE_x);
  LL_EXTI_EnableIT_0_31(SX_DIO_EXTI_LINE_x);
}

void sx_dio_exti_isr_clearflag(void)
{
  LL_EXTI_ClearFlag_0_31(SX_DIO_EXTI_LINE_x);
}


//-- In port
// is on IO_PB11, USART3 RX, inverted
// UARTE_UARTx = USART3

void in_init_gpio(void)
{
}

void in_set_normal(void)
{
  LL_USART_Disable(USART3);
  LL_USART_SetRXPinLevel(USART3, LL_USART_RXPIN_LEVEL_STANDARD);
  LL_USART_Enable(USART3);
}

void in_set_inverted(void)
{
  LL_USART_Disable(USART3);
  LL_USART_SetRXPinLevel(USART3, LL_USART_RXPIN_LEVEL_INVERTED);
  LL_USART_Enable(USART3);
}


//-- Button

#define BUTTON                    IO_PB0

void button_init(void)
{
  gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}

bool button_pressed(void)
{
  return gpio_read_activelow(BUTTON);
}


//-- LEDs

#define LED_GREEN                 IO_PB3
#define LED_RED                   IO_PB2

void leds_init(void)
{
  gpio_init(LED_GREEN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
  gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
  gpio_low(LED_GREEN); // LED_GREEN_OFF
  gpio_low(LED_RED); // LED_RED_OFF
}

void led_green_off(void) { gpio_low(LED_GREEN); }
void led_green_on(void) { gpio_high(LED_GREEN); }
void led_green_toggle(void) { gpio_toggle(LED_GREEN); }

void led_red_off(void) { gpio_low(LED_RED); }
void led_red_on(void) { gpio_high(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- Serial or Com Switch
// use com if BUTTON is pressed during power up, else use serial
// BUTTON becomes bind button later on

bool r9mx_ser_or_com_serial = true; // we use serial as default

void ser_or_com_init(void)
{
  gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < 16; i++) {
    if (gpio_read_activelow(BUTTON)) cnt++;
  }
  r9mx_ser_or_com_serial = !(cnt > 8);
}

bool ser_or_com_serial(void)
{
  return r9mx_ser_or_com_serial;
}


//-- Position Switch

void pos_switch_init(void)
{
}


//-- Buzzer

#define BUZZER                    IO_PA8
#define BUZZER_IO_AF              IO_AF_1
#define BUZZER_TIMx               TIM1
#define BUZZER_IRQn               TIM1_UP_TIM16_IRQn
#define BUZZER_IRQHandler         TIM1_UP_TIM16_IRQHandler
#define BUZZER_TIM_CHANNEL        LL_TIM_CHANNEL_CH1
//#define BUZZER_TIM_IRQ_PRIORITY   14


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


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3,
    LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11,
    LL_GPIO_PIN_12, LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_3,
    LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12, LL_GPIO_PIN_13,
    LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};

uint32_t portc[] = {
    LL_GPIO_PIN_1,
    LL_GPIO_PIN_13, LL_GPIO_PIN_14,
};
/*
uint32_t porta[] = {
    LL_GPIO_PIN_2, LL_GPIO_PIN_5,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11,
};

uint32_t portb[] = {
    LL_GPIO_PIN_3,
    LL_GPIO_PIN_11,
};

uint32_t portc[] = {
    LL_GPIO_PIN_1,
};
*/

