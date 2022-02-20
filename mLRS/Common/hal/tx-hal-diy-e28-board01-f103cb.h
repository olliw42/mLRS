//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// TX DIY E28 BOARD01 v004 STM32F103CB
//-------------------------------------------------------

//#define DEVICE_HAS_IN
#define DEVICE_HAS_JRPIN5 // requires external diode-R network connected to OUT,IN,GND


//-- Timers, Timing and such stuff

#define DELAY_USE_DWT

#define SYSTICK_TIMESTEP          1000
#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)


//-- UARTS
// UARTB = serial port
// UARTC = debug port
// UART = SPORT (pin5) on JR bay
// UARTE = in port, SBus or whatever

#define UARTB_USE_UART2 // serial
#define UARTB_BAUD                57600
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

#ifdef DEVICE_HAS_IN
#define UARTE_USE_UART1_REMAPPED // SBus
#define UARTE_BAUD                100000 // SBus normal baud rate, is being set later anyhow
//#define UARTE_USE_TX
//#define UARTE_TXBUFSIZE           512
//#define UARTE_USE_TX_ISR
#define UARTE_USE_RX
#define UARTE_RXBUFSIZE           512
#endif
#ifdef DEVICE_HAS_JRPIN5
#define UART_USE_UART1_REMAPPED // JR pin5, MBridge
#define UART_BAUD                 400000 // 115200
#define UART_USE_TX
#define UART_TXBUFSIZE            512
#define UART_USE_TX_ISR
#define UART_USE_RX
#define UART_RXBUFSIZE            512

#define JRPIN5_TX_XOR             IO_PB3
#define JRPIN5_TX_SET_NORMAL      gpio_low(JRPIN5_TX_XOR)
#define JRPIN5_TX_SET_INVERTED    gpio_high(JRPIN5_TX_XOR)
#define JRPIN5_RX_XOR             IO_PA15
#define JRPIN5_RX_SET_NORMAL      gpio_low(JRPIN5_RX_XOR)
#define JRPIN5_RX_SET_INVERTED    gpio_high(JRPIN5_RX_XOR)
#endif


//-- SX1: SX12xx & SPI

#define SPI_USE_SPI1              // PA65, PA6, PA7
#define SPI_CS_IO                 IO_PA4
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PB12
#define SX_DIO1                   IO_PB11
#define SX_BUSY                   IO_PB1
#define SX_RX_EN                  IO_PB0
#define SX_TX_EN                  IO_PB13
//#define SX_ANT_SELECT
//#define SX_USE_DCDC

#define SX_DIO1_GPIO_AF_EXTI_PORTx    LL_GPIO_AF_EXTI_PORTB
#define SX_DIO1_GPIO_AF_EXTI_LINEx    LL_GPIO_AF_EXTI_LINE11
#define SX_DIO_EXTI_LINE_x            LL_EXTI_LINE_11
#define SX_DIO_EXTI_IRQn              EXTI15_10_IRQn
#define SX_DIO_EXTI_IRQHandler        EXTI15_10_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY    11

void sx_init_gpio(void)
{
  gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
  gpio_init(SX_DIO1, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
#ifdef SX_BUSY
  gpio_init(SX_BUSY, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
#endif
  gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
  gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

bool sx_dio_read(void)
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
  gpio_low(SX_RX_EN);
  gpio_high(SX_TX_EN);
}

void sx_amp_receive(void)
{
  gpio_low(SX_TX_EN);
  gpio_high(SX_RX_EN);
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


//-- SBus input pin

#define IN                        IO_PB7 // UART1 RX
#define IN_XOR                    IO_PA15

void in_init_gpio(void)
{
  gpio_init(IN_XOR, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
  gpio_low(IN_XOR);
}

void in_set_normal(void)
{
  gpio_low(IN_XOR);
}

void in_set_inverted(void)
{
  gpio_high(IN_XOR);
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

#define LED_GREEN 		          IO_PB5
#define LED_RED		              IO_PB4

#define LED_GREEN_ON              gpio_high(LED_GREEN)
#define LED_RED_ON                gpio_high(LED_RED)

#define LED_GREEN_OFF             gpio_low(LED_GREEN)
#define LED_RED_OFF               gpio_low(LED_RED)

#define LED_GREEN_TOGGLE          gpio_toggle(LED_GREEN)
#define LED_RED_TOGGLE            gpio_toggle(LED_RED)

#define LED_RIGHT_GREEN_ON        // not available
#define LED_RIGHT_GREEN_OFF       // not available

void leds_init(void)
{
  gpio_init(LED_GREEN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
  gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
  LED_GREEN_OFF;
  LED_RED_OFF;
}


//-- Position Switch

void pos_switch_init(void)
{
}


//-- POWER

#define POWER_GAIN_DBM            27
#define POWER_SX1280_MAX_DBM      SX1280_POWER_0_DBM

#define POWER_NUM                 5

const uint16_t power_list[POWER_NUM][2] = {
    { POWER_0_DBM, 1 },
    { POWER_10_DBM, 10 },
    { POWER_20_DBM, 100 },
    { POWER_23_DBM, 200 },
    { POWER_27_DBM, 500 },
 };


//-- TEST

#define PORTA_N  9

uint32_t porta[PORTA_N] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3,
    LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_15,
};

#define PORTB_N  12

uint32_t portb[PORTB_N] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_3,
    LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12, LL_GPIO_PIN_13,
    LL_GPIO_PIN_15,
};

#define PORTC_N  1

uint32_t portc[PORTC_N] = {
    LL_GPIO_PIN_13,
};


