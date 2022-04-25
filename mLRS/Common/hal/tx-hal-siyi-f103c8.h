//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//*******************************************************

//-------------------------------------------------------
// TX Siyi FM30 STM32F103C8
//-------------------------------------------------------
// the info on the pin assignments is taken from
// https://github.com/ExpressLRS/ExpressLRS/blob/master/src/include/target/FM30_TX.h
// https://github.com/ExpressLRS/ExpressLRS/issues/381
// https://github.com/ExpressLRS/ExpressLRS/pull/388
// https://github.com/ExpressLRS/ExpressLRS/blob/master/src/include/target/FM30_RX_MINI.h
// https://github.com/ExpressLRS/ExpressLRS/issues/308
// MANY thx to CapnBry !

#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_SERIAL_OR_COM // is selected in device specific ways, here: encoder switch


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define SYSTICK_TIMESTEP          1000
#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)

#define EE_START_PAGE             60 // 64 kB flash, 1 kB page


//-- UARTS
// UARTB = serial port
// UARTC = debug port
// UART = SPORT (pin5) on JR bay
// UARTE = in port, SBus or whatever

#define UARTB_USE_UART2 // serial
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE // 512
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE // 512

#define UARTC_USE_UART3 // debug
#define UARTC_BAUD                115200
#define UARTC_USE_TX
#define UARTC_TXBUFSIZE           512
#define UARTC_USE_TX_ISR
//#define UARTC_USE_RX
//#define UARTC_RXBUFSIZE           512

#define UART_USE_UART1 // JR pin5, MBridge
#define UART_BAUD                 400000
#define UART_USE_TX
#define UART_TXBUFSIZE            512
#define UART_USE_TX_ISR
#define UART_USE_RX
#define UART_RXBUFSIZE            512

#define JRPIN5_TX_XOR             IO_PB7
#define JRPIN5_TX_SET_NORMAL      gpio_low(JRPIN5_TX_XOR)
#define JRPIN5_TX_SET_INVERTED    gpio_high(JRPIN5_TX_XOR)
#define JRPIN5_RX_XOR             IO_PB6
#define JRPIN5_RX_SET_NORMAL      gpio_low(JRPIN5_RX_XOR)
#define JRPIN5_RX_SET_INVERTED    gpio_high(JRPIN5_RX_XOR)


//-- SX1: SX12xx & SPI

#define SPI_USE_SPI2
#define SPI_CS_IO                 IO_PB12
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PB3
#define SX_DIO1                   IO_PB8
//#define SX_BUSY
#define SX_AMP_CTX                IO_PB9 // high for transmit, low for receive
#define SX_ANT_SELECT             IO_PB4 // low for left (stock), high for right (empty)

#define SX_USE_DCDC

#define SX_DIO1_GPIO_AF_EXTI_PORTx    LL_GPIO_AF_EXTI_PORTB
#define SX_DIO1_GPIO_AF_EXTI_LINEx    LL_GPIO_AF_EXTI_LINE8
#define SX_DIO_EXTI_LINE_x            LL_EXTI_LINE_8
#define SX_DIO_EXTI_IRQn              EXTI9_5_IRQn
#define SX_DIO_EXTI_IRQHandler        EXTI9_5_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY    11

void sx_init_gpio(void)
{
  gpio_init(SX_ANT_SELECT, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
  gpio_init(SX_AMP_CTX, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);

  gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
  gpio_init(SX_DIO1, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
}

bool sx_dio_read(void)
{
  return (gpio_read_activehigh(SX_DIO1)) ? true : false;
}

void sx_amp_transmit(void)
{
  gpio_high(SX_AMP_CTX);
}

void sx_amp_receive(void)
{
  gpio_low(SX_AMP_CTX);
}

void sx_dio_init_exti_isroff(void)
{
  LL_GPIO_AF_SetEXTISource(SX_DIO1_GPIO_AF_EXTI_PORTx, SX_DIO1_GPIO_AF_EXTI_LINEx);

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

#define LED_LEFT_GREEN            IO_PA7
#define LED_LEFT_RED              IO_PA15
#define LED_RIGHT_GREEN           IO_PB1
#define LED_RIGHT_RED             IO_PB2

#define LED_RIGHT_RED_ON          gpio_low(LED_RIGHT_RED)
#define LED_RIGHT_GREEN_ON        gpio_low(LED_RIGHT_GREEN)

#define LED_RIGHT_RED_OFF         gpio_high(LED_RIGHT_RED)
#define LED_RIGHT_GREEN_OFF       gpio_high(LED_RIGHT_GREEN)

#define LED_RIGHT_RED_TOGGLE      gpio_toggle(LED_RIGHT_RED)
#define LED_RIGHT_GREEN_TOGGLE    gpio_toggle(LED_RIGHT_GREEN)

void leds_init(void)
{
  gpio_init(LED_LEFT_GREEN, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);
  gpio_init(LED_LEFT_RED, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);
  gpio_init(LED_RIGHT_GREEN, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);
  gpio_init(LED_RIGHT_RED, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);
  gpio_high(LED_LEFT_GREEN); // LED_GREEN_OFF
  gpio_high(LED_LEFT_RED); // LED_RED_OFF
  LED_RIGHT_RED_OFF;
  LED_RIGHT_GREEN_OFF;
}

void led_green_off(void) { gpio_high(LED_LEFT_GREEN); }
void led_green_on(void) { gpio_low(LED_LEFT_GREEN); }
void led_green_toggle(void) { gpio_toggle(LED_LEFT_GREEN); }

void led_red_off(void) { gpio_high(LED_LEFT_RED); }
void led_red_on(void) { gpio_low(LED_LEFT_RED); }
void led_red_toggle(void) { gpio_toggle(LED_LEFT_RED); }


//-- Serial or Com Switch

void ser_or_com_init(void)
{
}

bool ser_or_com_serial(void)
{
  return true; // we use serial as default
}


//-- Position Switch

#define POS_SWITCH_BIT1           IO_PA0
#define POS_SWITCH_BIT2           IO_PA1
#define POS_SWITCH_BIT3           IO_PA4
#define POS_SWITCH_BIT4           IO_PA5

void pos_switch_init(void)
{
  gpio_init(POS_SWITCH_BIT1, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
  gpio_init(POS_SWITCH_BIT2, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
  gpio_init(POS_SWITCH_BIT3, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
  gpio_init(POS_SWITCH_BIT4, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}

uint8_t pos_switch_read(void)
{
  return (uint8_t)gpio_read_activelow(POS_SWITCH_BIT1) +
         ((uint8_t)gpio_read_activelow(POS_SWITCH_BIT2) << 1) +
         ((uint8_t)gpio_read_activelow(POS_SWITCH_BIT3) << 2) +
         ((uint8_t)gpio_read_activelow(POS_SWITCH_BIT4) << 3);
}


//-- POWER

#define POWER_GAIN_DBM            22 // gain of a PA stage if present
#define POWER_SX1280_MAX_DBM      SX1280_POWER_3_DBM // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_0_DBM, .mW = 1 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_23_DBM, .mW = 200 },
};






