//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//*******************************************************

#define MLRS_FEATURE_OLED

//-------------------------------------------------------
// TX FRM303 STM32F070CB
//-------------------------------------------------------
// this MCU has no DWT
// 4 UARTS, but U3 & U4 are on same IRQ so cannot be used both
// T1, T3, T14, T15, T16, T17, internal T6, T7

#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_COM_ON_USB
//#define DEVICE_HAS_NO_COM
//#define DEVICE_HAS_I2C_DISPLAY_ROT180
//#define DEVICE_HAS_FIVEWAY
//#define DEVICE_HAS_BUZZER
//#define DEVICE_HAS_DEBUG_SWUART

#ifdef MLRS_FEATURE_OLED
  #undef DEVICE_HAS_COM_ON_USB
  #define DEVICE_HAS_NO_COM
  #define DEVICE_HAS_I2C_DISPLAY_ROT180
#endif

#ifdef DEBUG_ENABLED
#undef DEBUG_ENABLED
#endif


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_TIM7_W_INIT //DELAY_USE_DWT,
static inline void delay_ns(uint32_t ns) { __NOP();__NOP(); __NOP(); __NOP(); } // 48 MHz => 4x nop = 100 ns

#define SYSTICK_TIMESTEP          1000
#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)

#define EE_START_PAGE             60 // 128 kB flash, 2 kB page


//-- UARTS
// UARTB = serial port
// UARTC or USB = COM (CLI)
// UARTD = -
// UART  = JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_UART2 // serial
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE
/*
#define UARTC_USE_UART2 // com USB/CLI
#define UARTC_BAUD                TX_COM_BAUDRATE
#define UARTC_USE_TX
#define UARTC_TXBUFSIZE           TX_COM_TXBUFSIZE
#define UARTC_USE_TX_ISR
#define UARTC_USE_RX
#define UARTC_RXBUFSIZE           TX_COM_RXBUFSIZE
*/
#define UART_USE_UART1 // JR pin5, MBridge
#define UART_BAUD                 400000
#define UART_USE_TX
#define UART_TXBUFSIZE            512
#define UART_USE_TX_ISR
#define UART_USE_RX
#define UART_RXBUFSIZE            512

#define JRPIN5_RX_TX_INVERT_INTERNAL
#define JRPIN5_DISABLE_TX_WHILE_RX

#define SWUART_USE_TIM17 // debug
#define SWUART_TX_IO              IO_PB14 // that's the I2C2 SDA pin
#define SWUART_BAUD               115200
#define SWUART_USE_TX
#define SWUART_TXBUFSIZE          512
//#define SWUART_TIM_IRQ_PRIORITY   11


//-- SX12xx & SPI

#define SPI_USE_SPI1_PB3PB4PB5    // PB3, PB4, PB5
#define SPI_CS_IO                 IO_PA15
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PA0
#define SX_DIO1                   IO_PB10
#define SX_BUSY                   IO_PB6
#define SX_RX_EN                  IO_PB1
#define SX_TX_EN                  IO_PA4
#define SX_PA_EN                  IO_PA5

#define SX_DIO1_SYSCFG_EXTI_PORTx     LL_SYSCFG_EXTI_PORTB
#define SX_DIO1_SYSCFG_EXTI_LINEx     LL_SYSCFG_EXTI_LINE10
#define SX_DIO_EXTI_LINE_x            LL_EXTI_LINE_10
#define SX_DIO_EXTI_IRQn              EXTI4_15_IRQn
#define SX_DIO_EXTI_IRQHandler        EXTI4_15_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY   11

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(SX_DIO1, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
    gpio_init(SX_PA_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

bool sx_busy_read(void)
{
    return (gpio_read_activehigh(SX_BUSY)) ? true : false;
}

void sx_amp_transmit(void)
{
    gpio_low(SX_RX_EN);
    gpio_high(SX_PA_EN);
    gpio_high(SX_TX_EN);
}

void sx_amp_receive(void)
{
    gpio_low(SX_TX_EN);
    gpio_low(SX_PA_EN);
    gpio_high(SX_RX_EN);
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

void sx_dio_exti_isr_clearflag(void)
{
    LL_EXTI_ClearFlag_0_31(SX_DIO_EXTI_LINE_x);
}


//-- SX12xx II & SPIB
// has none


//-- Button
// has none, use 5 way on PA7, down

#define BUTTON                    IO_PA7

#ifndef DEVICE_HAS_I2C_DISPLAY_ROT180
void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}

bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON);
}
#else
void button_init(void) {}
bool button_pressed(void) { return 0; }
#endif


//-- LEDs

#define LED_GREEN                 IO_PB8
#define LED_RED                   IO_PB7
#define LED_RIGHT_GREEN           IO_PB9 // blue

void leds_init(void)
{
    gpio_init(LED_GREEN, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);
    gpio_init(LED_RIGHT_GREEN, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);
}

void led_green_off(void) { gpio_high(LED_GREEN); }
void led_green_on(void) { gpio_low(LED_GREEN); }
void led_green_toggle(void) { gpio_toggle(LED_GREEN); }

void led_red_off(void) { gpio_high(LED_RED); }
void led_red_on(void) { gpio_low(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- Serial or Com Switch
// use com if FIVEWAY is DOWN during power up, else use serial
// FIVEWAY-DONW becomes bind button later on

#ifdef DEVICE_HAS_SERIAL_OR_COM
bool frm303_ser_or_com_serial = true; // we use serial as default

void ser_or_com_init(void)
{
  gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < 16; i++) {
    if (gpio_read_activelow(BUTTON)) cnt++;
  }
  frm303_ser_or_com_serial = !(cnt > 8);
}

bool ser_or_com_serial(void)
{
  return frm303_ser_or_com_serial;
}
#endif


//-- 5 Way Switch
// resistor chain Vcc - 10k - down - 3.3k - right - 6.2k - up - 18k - left - 34.8k - center

#define FIVEWAY_ADCx              ADC1
#define FIVEWAY_ADC_IO            IO_PA7 // ADC1_IN7
#define FIVEWAY_ADC_CHANNELx      LL_ADC_CHANNEL_7

#ifdef DEVICE_HAS_I2C_DISPLAY_ROT180
void fiveway_init(void)
{
    rcc_init_afio();
    rcc_init_adc(FIVEWAY_ADCx);
    adc_init_one_channel(FIVEWAY_ADCx);
    adc_config_channel(FIVEWAY_ADCx, 0, FIVEWAY_ADC_CHANNELx, FIVEWAY_ADC_IO);
    adc_enable(FIVEWAY_ADCx);
    adc_start_conversion(FIVEWAY_ADCx);
}

uint16_t fiveway_adc_read(void)
{
    return LL_ADC_REG_ReadConversionData12(FIVEWAY_ADCx);
}

uint8_t fiveway_read(void)
{
    uint16_t adc = LL_ADC_REG_ReadConversionData12(FIVEWAY_ADCx);
    if (adc < (0+200)) return (1 << KEY_DOWN); // 0
    if (adc > (990-200) && adc < (990+200)) return (1 << KEY_RIGHT); // 990 (0.8 V)
    if (adc > (1990-200) && adc < (1990+200)) return (1 << KEY_UP); // 1990 (1.6 V)
    if (adc > (2940-200) && adc < (2940+200)) return (1 << KEY_LEFT); // 2940  (2.4 V)
    if (adc > (3465-200) && adc < (3465+200)) return (1 << KEY_CENTER); // 3465 (2.8 V)
    return 0;
}
#endif


//-- Display I2C

#define I2C_USE_I2C2_PB13PB14     // PB13, PB14
#define I2C_CLOCKSPEED_400KHZ     // not all displays seem to work well with I2C_CLOCKSPEED_1000KHZ
#define I2C_USE_DMAMODE


//-- Buzzer
// Buzzer is active high, buzzer is not on a timer pin
/*
#define BUZZER                    IO_PB11
#define BUZZER_IO_AF              IO_AF_12
#define BUZZER_TIMx               TIM1
#define BUZZER_IRQn               TIM1_UP_TIM16_IRQn
#define BUZZER_IRQHandler         TIM1_UP_TIM16_IRQHandler
#define BUZZER_TIM_CHANNEL        LL_TIM_CHANNEL_CH3N
//#define BUZZER_TIM_IRQ_PRIORITY   14
*/

//-- POWER

#define POWER_GAIN_DBM            24 // 35 // gain of a PA stage if present // datasheet of SKY66312-11 says 35dB !
#define POWER_SX1280_MAX_DBM      SX1280_POWER_6_DBM //SX1280_POWER_m3_DBM // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12,
    LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, /*LL_GPIO_PIN_7,*/
    /*LL_GPIO_PIN_8,*/ /*LL_GPIO_PIN_9,*/ LL_GPIO_PIN_10, /*LL_GPIO_PIN_11,*/ LL_GPIO_PIN_12, LL_GPIO_PIN_13,
    LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};

uint32_t portc[] = {
    LL_GPIO_PIN_13, LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};








