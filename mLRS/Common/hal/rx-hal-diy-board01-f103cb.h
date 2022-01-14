//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// RX DIY BOARD01 v014 STM32F103CB
//-------------------------------------------------------
#define DEVICE_IS_RECEIVER


//-- Timers, Timing and such stuff

#define DELAY_USE_DWT

#define SYSTICK_TIMESTEP          1000
#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)


//-- UARTS
// UARTB = serial port, UARTC = debug port
// UART = SBus output or whatever

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

#define UART_USE_UART1_REMAPPED // SBus
#define UART_BAUD                 100000 // SBus normal baud rate, is being set later anyhow
#define UART_USE_TX
#define UART_TXBUFSIZE            512
#define UART_USE_TX_ISR
//#define UART_USE_RX
//#define UART_RXBUFSIZE            512


//-- SX1: SX12xx & SPI

#define SPI_USE_SPI1              // PA65, PA6, PA7
#define SPI_CS_IO                 IO_PA4
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_4500KHZ //SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PB11
#define SX_DIO1                   IO_PB0
#define SX_BUSY                   IO_PB1
//#define SX_AMP_CTX
//#define SX_ANT_SELECT

//#define SX_USE_DCDC

//#define SX_POWER_MAX

#define SX_DIO1_GPIO_AF_EXTI_PORTx    LL_GPIO_AF_EXTI_PORTB
#define SX_DIO1_GPIO_AF_EXTI_LINEx    LL_GPIO_AF_EXTI_LINE0
#define SX_DIO1_EXTI_LINE_x           LL_EXTI_LINE_0
#define SX_DIO1_EXTI_IRQn             EXTI0_IRQn
#define SX_DIO1_EXTI_IRQHandler       EXTI0_IRQHandler
//#define SX_DIO1_EXTI_IRQ_PRIORITY   11

/*
#define SPI_USE_SPI2              // PB13, PB14, PB15
#define SPI_CS_IO                 IO_PB12
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_4500KHZ

#define SX_RESET                  IO_PA10
#define SX_DIO1                   IO_PA8
#define SX_BUSY                   IO_PA9

#define SX_DIO1_GPIO_AF_EXTI_PORTx    LL_GPIO_AF_EXTI_PORTA
#define SX_DIO1_GPIO_AF_EXTI_LINEx    LL_GPIO_AF_EXTI_LINE8
#define SX_DIO1_EXTI_LINE_x           LL_EXTI_LINE_8
#define SX_DIO1_EXTI_IRQn             EXTI9_5_IRQn
#define SX_DIO1_EXTI_IRQHandler       EXTI9_5_IRQHandler
*/

void sx_init_gpio(void)
{
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
}

void sx_amp_receive(void)
{
}

void sx_dio1_init_exti_isroff(void)
{
  LL_GPIO_AF_SetEXTISource(SX_DIO1_GPIO_AF_EXTI_PORTx, SX_DIO1_GPIO_AF_EXTI_LINEx);

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


//-- SX2: SX12xxB & SPIB

#define SPIB_USE_SPI2             // PB13, PB14, PB15
#define SPIB_CS_IO                IO_PB12
#define SPIB_USE_CLK_LOW_1EDGE    // datasheet says CPHA = 0  CPOL = 0
#define SPIB_USE_CLOCKSPEED_4500KHZ

#define SX2_RESET                 IO_PA10
#define SX2_DIO1                  IO_PA8
#define SX2_BUSY                  IO_PA9
//#define SX2_AMP_CTX
//#define SX2_ANT_SELECT

//#define SX2_USE_DCDC

//#define SX2_POWER_MAX

#define SX2_DIO1_GPIO_AF_EXTI_PORTx   LL_GPIO_AF_EXTI_PORTA
#define SX2_DIO1_GPIO_AF_EXTI_LINEx   LL_GPIO_AF_EXTI_LINE8
#define SX2_DIO1_EXTI_LINE_x          LL_EXTI_LINE_8
#define SX2_DIO1_EXTI_IRQn            EXTI9_5_IRQn
#define SX2_DIO1_EXTI_IRQHandler      EXTI9_5_IRQHandler
//#define SX2_DIO1_EXTI_IRQ_PRIORITY   11


//-- SBus output pin

#define OUT                       IO_PA9 // UART1 TX // PB6 !!!!!!!!!!
#define OUT_XOR                   IO_PA15

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

#define BUTTON                    IO_PC13

void button_init(void)
{
  gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}

bool button_pressed(void)
{
  return gpio_read_activelow(BUTTON);
}


//-- LEDs

#define LED_GREEN 		            IO_PB4
#define LED_RED		                IO_PB3

#define LED_GREEN_ON              gpio_high(LED_GREEN)
#define LED_RED_ON                gpio_high(LED_RED)

#define LED_GREEN_OFF             gpio_low(LED_GREEN)
#define LED_RED_OFF               gpio_low(LED_RED)

#define LED_GREEN_TOGGLE          gpio_toggle(LED_GREEN)
#define LED_RED_TOGGLE            gpio_toggle(LED_RED)

void leds_init(void)
{
  gpio_init(LED_GREEN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
  gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
  LED_GREEN_OFF;
  LED_RED_OFF;
}


//-- POWER

#define POWER_GAIN_DBM            0
#define POWER_SX1280_MAX_DBM      SX1280_POWER_12p5_DBM

#define POWER_NUM                 3

const uint16_t power_list[POWER_NUM][2] = {
    { POWER_0_DBM, 1 },
    { POWER_10_DBM, 10 },
    { POWER_12p5_DBM, 18 },
};


//-- TEST

#define PORTA_N  13

uint32_t porta[PORTA_N] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3,
    LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11,
    LL_GPIO_PIN_15,
};

#define PORTB_N  12

uint32_t portb[PORTB_N] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1,
    LL_GPIO_PIN_3, LL_GPIO_PIN_4,
    LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_10, LL_GPIO_PIN_11,
    LL_GPIO_PIN_12, LL_GPIO_PIN_13, LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};

#define PORTC_N  1

uint32_t portc[PORTC_N] = {
    LL_GPIO_PIN_13,
};


