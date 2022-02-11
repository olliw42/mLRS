//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// RX Siyi FM30 STM32F373CC
//-------------------------------------------------------
// the info on the pin assignments is taken from
// https://github.com/ExpressLRS/ExpressLRS/blob/master/src/include/target/FM30_TX.h
// https://github.com/ExpressLRS/ExpressLRS/issues/381
// https://github.com/ExpressLRS/ExpressLRS/pull/388
// https://github.com/ExpressLRS/ExpressLRS/blob/master/src/include/target/FM30_RX_MINI.h
// https://github.com/ExpressLRS/ExpressLRS/issues/308
// MANY thx to CapnBry !


#define DEVICE_IS_RECEIVER


//-- Timers, Timing and such stuff

#define DELAY_USE_DWT

#define SYSTICK_TIMESTEP          1000
#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)


//-- UARTS
// UARTB = serial port
// UARTC = debug port
// UART = output port, SBus or whatever

#define UARTB_USE_UART2 // serial
#define UARTB_BAUD                SETUP_RX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           512
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           512

#define UARTC_USE_UART3 // debug
#define UARTC_BAUD                115200
#define UARTC_USE_TX
#define UARTC_TXBUFSIZE           4096 //512
#define UARTC_USE_TX_ISR
//#define UARTC_USE_RX
//#define UARTC_RXBUFSIZE           512

#define UART_USE_UART1 // SBus
#define UART_BAUD                 100000 // SBus normal baud rate, is being set later anyhow
#define UART_USE_TX
#define UART_TXBUFSIZE            512
#define UART_USE_TX_ISR
//#define UART_USE_RX
//#define UART_RXBUFSIZE            512


//-- SX1: SX12xx & SPI

#define SPI_USE_SPI1              // PB3, PB4, PB5
#define SPI_CS_IO                 IO_PA15
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PB2
#define SX_DIO1                   IO_PE8
#define SX_BUSY                   IO_PE9
#define SX_AMP_CTX                IO_PD8 // high for transmit, low for receive
#define SX_ANT_SELECT             IO_PA8 // low for left (stock), high for right (empty)

#define SX_USE_DCDC

#define SX_DIO1_SYSCFG_EXTI_PORTx     LL_SYSCFG_EXTI_PORTE
#define SX_DIO1_SYSCFG_EXTI_LINEx     LL_SYSCFG_EXTI_LINE8
#define SX_DIO1_EXTI_LINE_x           LL_EXTI_LINE_8
#define SX_DIO1_EXTI_IRQn             EXTI9_5_IRQn
#define SX_DIO1_EXTI_IRQHandler       EXTI9_5_IRQHandler
//#define SX_DIO1_EXTI_IRQ_PRIORITY   11

void sx_init_gpio(void)
{
  gpio_init(SX_AMP_CTX, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
#ifdef SETUP_RX_ANTENNA
  #if SETUP_RX_ANTENNA == 1
    gpio_init(SX_ANT_SELECT, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
  #else
    gpio_init(SX_ANT_SELECT, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
  #endif
#else
  gpio_init(SX_ANT_SELECT, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
#endif

  gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
  gpio_init(SX_DIO1, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
#ifdef SX_BUSY
  gpio_init(SX_BUSY, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
#endif
}

bool sx_dio1_read(void)
{
  return (gpio_read_activehigh(SX_DIO1)) ? true : false;
}

#ifdef SX_BUSY
bool sx_busy_read(void)
{
  return (gpio_read_activehigh(SX_BUSY)) ? true : false;
}
#endif

void sx_amp_transmit(void)
{
  gpio_high(SX_AMP_CTX);
}

void sx_amp_receive(void)
{
  gpio_low(SX_AMP_CTX);
}

void sx_dio1_init_exti_isroff(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_SYSCFG_SetEXTISource(SX_DIO1_SYSCFG_EXTI_PORTx, SX_DIO1_SYSCFG_EXTI_LINEx);

  // let's not use LL_EXTI_Init(), but let's do it by hand, is easier to allow enabling isr later
  LL_EXTI_DisableEvent_0_31(SX_DIO1_EXTI_LINE_x);
  LL_EXTI_DisableIT_0_31(SX_DIO1_EXTI_LINE_x);
  LL_EXTI_DisableFallingTrig_0_31(SX_DIO1_EXTI_LINE_x);
  LL_EXTI_EnableRisingTrig_0_31(SX_DIO1_EXTI_LINE_x);

  NVIC_SetPriority(SX_DIO1_EXTI_IRQn, SX_DIO1_EXTI_IRQ_PRIORITY);
  NVIC_EnableIRQ(SX_DIO1_EXTI_IRQn);
}

void sx_dio1_enable_exti_isr(void)
{
  LL_EXTI_ClearFlag_0_31(SX_DIO1_EXTI_LINE_x);
  LL_EXTI_EnableIT_0_31(SX_DIO1_EXTI_LINE_x);
}


//-- SBus output pin

#define OUT                       IO_PA9 // UART1 TX
#define OUT_XOR                   IO_PF6

void out_init_gpio(void)
{
  gpio_init(OUT_XOR, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
  gpio_low(OUT_XOR);
}

void out_set_normal(void)
{
  gpio_low(OUT_XOR);
}

void out_set_inverted(void)
{
  gpio_high(OUT_XOR);
}


//-- Button

//#define BUTTON

void button_init(void)
{
}

bool button_pressed(void)
{
  return false;
}


//-- LEDs

#define LED_GREEN 		            IO_PB7
#define LED_RED		                IO_PB6

#define LED_GREEN_ON              gpio_low(LED_GREEN)
#define LED_RED_ON                gpio_low(LED_RED)

#define LED_GREEN_OFF             gpio_high(LED_GREEN)
#define LED_RED_OFF               gpio_high(LED_RED)

#define LED_GREEN_TOGGLE          gpio_toggle(LED_GREEN)
#define LED_RED_TOGGLE            gpio_toggle(LED_RED)

void leds_init(void)
{
  gpio_init(LED_GREEN, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);
  gpio_init(LED_RED, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);
  LED_GREEN_OFF;
  LED_RED_OFF;
}


//-- POWER

#define POWER_GAIN_DBM            22
#define POWER_SX1280_MAX_DBM      SX1280_POWER_3_DBM



//-------------------------------------------------------
// there are more pins/pads which are accessible
//-------------------------------------------------------
/*
inspect also pic https://user-images.githubusercontent.com/42233240/105626347-4f30bc80-5e2f-11eb-9e3d-2d209b07b91d.jpg

UART1             ->  PA10, PA9
UART1 TX Inverter ->  PF6

                  ->  PA1? from pic it goes to somewhere

UAV               ->  PA0 (seems obvious from pic, needs to be confirmed)

RSSI              ->  PA4 (seems obvious from pic, needs to be confirmed)














*/
