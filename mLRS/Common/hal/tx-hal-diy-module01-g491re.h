//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//*******************************************************

//-------------------------------------------------------
// TX DIY MODULE01v014 STM32G491RE
//-------------------------------------------------------

#define DEVICE_HAS_DIVERSITY
#define DEVICE_HAS_JRPIN5


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define SYSTICK_TIMESTEP          1000
#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)

#define EE_START_PAGE             250 // 512 kB flash, 2 kB page


//-- UARTS
// UARTB = serial port
// UARTC = USB (debug port)
// UARTD = BT/ESP port
// UART = SPORT (pin5) on JR bay
// UARTE = in port, SBus or whatever

#define UARTB_USE_UART5 // serial
#define UARTB_BAUD                SETUP_TX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE // 512
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE // 512

#define UARTB_RTS                 IO_PB4
#define UARTB_CTS                 IO_PB5

//#define UARTC_USE_UART1 // USB, debug
#define UARTC_USE_UART1_REMAPPED // PB6,PB7, debug only
#define UARTC_BAUD                115200
#define UARTC_USE_TX
#define UARTC_TXBUFSIZE           512
#define UARTC_USE_TX_ISR
#define UARTC_USE_RX
#define UARTC_RXBUFSIZE           512

#define UART_USE_UART2 // JR pin5, MBridge
#define UART_BAUD                 400000 // 115200
#define UART_USE_TX
#define UART_TXBUFSIZE            512
#define UART_USE_TX_ISR
#define UART_USE_RX
#define UART_RXBUFSIZE            512

#define JRPIN5_UARTx              UART_UARTx
#define JRPIN5_RX_TX_INVERT_INTERNAL

#define UARTD_USE_UART4 // BT/ESP
#define UARTD_BAUD                115200
#define UARTD_USE_TX
#define UARTD_TXBUFSIZE           512
#define UARTD_USE_TX_ISR
#define UARTD_USE_RX
#define UARTD_RXBUFSIZE           512

#define ESP_RESET                 IO_PA11
#define ESP_GPIO0                 IO_PA12


//-- SX12xx & SPI

#define SPI_USE_SPI1              // PA5, PA6, PA7
#define SPI_CS_IO                 IO_PA4
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PB0
#define SX_DIO1                   IO_PC4
#define SX_BUSY                   IO_PC5
//#define SX_AMP_CTX
//#define SX_ANT_SELECT

//#define SX_USE_DCDC

#define SX_DIO1_SYSCFG_EXTI_PORTx     LL_SYSCFG_EXTI_PORTC
#define SX_DIO1_SYSCFG_EXTI_LINEx     LL_SYSCFG_EXTI_LINE4
#define SX_DIO_EXTI_LINE_x            LL_EXTI_LINE_4
#define SX_DIO_EXTI_IRQn              EXTI4_IRQn
#define SX_DIO_EXTI_IRQHandler        EXTI4_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY   11

void sx_init_gpio(void)
{
  gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
  gpio_init(SX_DIO1, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
#ifdef SX_BUSY
  gpio_init(SX_BUSY, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
#endif
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
}

void sx_amp_receive(void)
{
}

void sx_dio_init_exti_isroff(void)
{
  LL_SYSCFG_SetEXTISource(SX_DIO1_SYSCFG_EXTI_PORTx, SX_DIO1_SYSCFG_EXTI_LINEx);

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


//-- SX12xx II & SPIB

#define SPIB_USE_SPI2             // PB13, PB14, PB15
#define SPIB_CS_IO                IO_PB12
#define SPIB_USE_CLK_LOW_1EDGE    // datasheet says CPHA = 0  CPOL = 0
#define SPIB_USE_CLOCKSPEED_9MHZ

#define SX2_RESET                 IO_PC8
#define SX2_DIO1                  IO_PC6
#define SX2_BUSY                  IO_PC7
//#define SX2_AMP_CTX
//#define SX2_ANT_SELECT

//#define SX2_USE_DCDC

#define SX2_DIO1_SYSCFG_EXTI_PORTx    LL_SYSCFG_EXTI_PORTC
#define SX2_DIO1_SYSCFG_EXTI_LINEx    LL_SYSCFG_EXTI_LINE6
#define SX2_DIO_EXTI_LINE_x           LL_EXTI_LINE_6
#define SX2_DIO_EXTI_IRQn             EXTI9_5_IRQn
#define SX2_DIO_EXTI_IRQHandler       EXTI9_5_IRQHandler
//#define SX2_DIO_EXTI_IRQ_PRIORITY   11

void sx2_init_gpio(void)
{
  gpio_init(SX2_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
  gpio_init(SX2_DIO1, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
  gpio_init(SX2_BUSY, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
}

bool sx2_dio_read(void)
{
  return (gpio_read_activehigh(SX2_DIO1)) ? true : false;
}

bool sx2_busy_read(void)
{
  return (gpio_read_activehigh(SX2_BUSY)) ? true : false;
}

void sx2_amp_transmit(void)
{
}

void sx2_amp_receive(void)
{
}

void sx2_dio_init_exti_isroff(void)
{
  LL_SYSCFG_SetEXTISource(SX2_DIO1_SYSCFG_EXTI_PORTx, SX2_DIO1_SYSCFG_EXTI_LINEx);

  // let's not use LL_EXTI_Init(), but let's do it by hand, is easier to allow enabling isr later
  LL_EXTI_DisableEvent_0_31(SX2_DIO_EXTI_LINE_x);
  LL_EXTI_DisableIT_0_31(SX2_DIO_EXTI_LINE_x);
  LL_EXTI_DisableFallingTrig_0_31(SX2_DIO_EXTI_LINE_x);
  LL_EXTI_EnableRisingTrig_0_31(SX2_DIO_EXTI_LINE_x);

  NVIC_SetPriority(SX2_DIO_EXTI_IRQn, SX2_DIO_EXTI_IRQ_PRIORITY);
  NVIC_EnableIRQ(SX2_DIO_EXTI_IRQn);
}

void sx2_dio_enable_exti_isr(void)
{
  LL_EXTI_ClearFlag_0_31(SX2_DIO_EXTI_LINE_x);
  LL_EXTI_EnableIT_0_31(SX2_DIO_EXTI_LINE_x);
}


//-- Button

#define BUTTON                    IO_PB9

void button_init(void)
{
  gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}

bool button_pressed(void)
{
  return gpio_read_activelow(BUTTON);
}


//-- LEDs

#define LED_LEFT_GREEN            IO_PC2
#define LED_LEFT_RED              IO_PC3
#define LED_RIGHT_GREEN           IO_PC0
//#define LED_RIGHT_RED             IO_PC13 // ???????

#define LED_GREEN_ON              gpio_high(LED_LEFT_GREEN)
#define LED_RED_ON                gpio_high(LED_LEFT_RED)
#define LED_RIGHT_RED_ON          //gpio_high(LED_RIGHT_RED)
#define LED_RIGHT_GREEN_ON        gpio_high(LED_RIGHT_GREEN)

#define LED_GREEN_OFF             gpio_low(LED_LEFT_GREEN)
#define LED_RED_OFF               gpio_low(LED_LEFT_RED)
#define LED_RIGHT_RED_OFF         //gpio_low(LED_RIGHT_RED)
#define LED_RIGHT_GREEN_OFF       gpio_low(LED_RIGHT_GREEN)

#define LED_GREEN_TOGGLE          gpio_toggle(LED_LEFT_GREEN)
#define LED_RED_TOGGLE            gpio_toggle(LED_LEFT_RED)
#define LED_RIGHT_RED_TOGGLE      //gpio_toggle(LED_RIGHT_RED)
#define LED_RIGHT_GREEN_TOGGLE    gpio_toggle(LED_RIGHT_GREEN)

void leds_init(void)
{
  gpio_init(LED_LEFT_GREEN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
  gpio_init(LED_LEFT_RED, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
  //gpio_init(LED_RIGHT_RED, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
  gpio_init(LED_RIGHT_GREEN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
  LED_GREEN_OFF;
  LED_RED_OFF;
  LED_RIGHT_RED_OFF;
  LED_RIGHT_GREEN_OFF;
}


//-- Position Switch

#define POS_SWITCH_BIT1
#define POS_SWITCH_BIT2
#define POS_SWITCH_BIT3
#define POS_SWITCH_BIT4

void pos_switch_init(void)
{
}

uint8_t pos_switch_read(void)
{
  return 0;
}


//-- 5 Way Switch

#define FIVEWAY_SWITCH_CENTER     IO_PC15
#define FIVEWAY_SWITCH_UP         IO_PC14
#define FIVEWAY_SWITCH_DOWN       IO_PA1
#define FIVEWAY_SWITCH_LEFT       IO_PA0
#define FIVEWAY_SWITCH_RIGHT      IO_PC1

void fiveway_init(void)
{
  gpio_init(FIVEWAY_SWITCH_CENTER, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
  gpio_init(FIVEWAY_SWITCH_UP, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
  gpio_init(FIVEWAY_SWITCH_DOWN, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
  gpio_init(FIVEWAY_SWITCH_LEFT, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
  gpio_init(FIVEWAY_SWITCH_RIGHT, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}

bool fiveway_pressed(void)
{
  return gpio_read_activelow(FIVEWAY_SWITCH_CENTER);
}

uint8_t fiveway_read(void)
{
  return (uint8_t)gpio_read_activelow(FIVEWAY_SWITCH_UP) +
         ((uint8_t)gpio_read_activelow(FIVEWAY_SWITCH_DOWN) << 1) +
         ((uint8_t)gpio_read_activelow(FIVEWAY_SWITCH_LEFT) << 2) +
         ((uint8_t)gpio_read_activelow(FIVEWAY_SWITCH_RIGHT) << 3);
}


//-- OLED I2C

#define I2C_USE_I2C3              // PA8, PC9
#define I2C_SCL                   IO_PA8
#define I2C_SDA                   IO_PC9

void oled_init_gpio(void)
{
}


//-- POWER

#define POWER_GAIN_DBM            0 // gain of a PA stage if present
#define POWER_SX1280_MAX_DBM      SX1280_POWER_12p5_DBM // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_0_DBM, .mW = 1 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_12p5_DBM, .mW = 18 },
};


//-- TEST

#define PORTA_N  13

uint32_t porta[PORTA_N] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3,
    LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11,
    LL_GPIO_PIN_12,
};

#define PORTB_N  11

uint32_t portb[PORTB_N] = {
    LL_GPIO_PIN_0,
    LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_9, LL_GPIO_PIN_11,
    LL_GPIO_PIN_12, LL_GPIO_PIN_13, LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};

#define PORTC_N  15

uint32_t portc[PORTC_N] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3,
    LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11,
    LL_GPIO_PIN_12, LL_GPIO_PIN_14, LL_GPIO_PIN_15
};









