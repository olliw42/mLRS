//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//*******************************************************

//-------------------------------------------------------
// RX Seeedstudio Grove Wio-E5 board STM32WLE5JC, https://wiki.seeedstudio.com/Grove_LoRa_E5_New_Version/
//-------------------------------------------------------

#define DEVICE_HAS_SERIAL_OR_DEBUG

#ifdef DEBUG_ENABLED
#undef DEBUG_ENABLED
#endif


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define SYSTICK_TIMESTEP          1000
#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)

#define EE_START_PAGE             120 // 256 kB flash, 2 kB page

#define MICROS_TIMx               TIM16

#define CLOCK_TIMx                TIM2
#define CLOCK_IRQn                TIM2_IRQn
#define CLOCK_IRQHandler          TIM2_IRQHandler
//#define CLOCK_IRQ_PRIORITY        10


//-- UARTS
// UARTB = serial port
// UARTC = debug port
// UART = output port, SBus or whatever

#define UARTB_USE_UART1_REMAPPED // serial // PB6,PB7 Grove plug
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE

#define UARTC_USE_UART2 // debug // also UART2
#define UARTC_BAUD                115200
#define UARTC_USE_TX
#define UARTC_TXBUFSIZE           512
#define UARTC_USE_TX_ISR
//#define UARTC_USE_RX
//#define UARTC_RXBUFSIZE           512

#define UART_USE_LPUART1_REMAPPED // out pin // PC1
#define UART_BAUD                 100000 // SBus normal baud rate, is being set later anyhow
#define UART_USE_TX
#define UART_TXBUFSIZE            256
#define UART_USE_TX_ISR
//#define UART_USE_RX
//#define UART_RXBUFSIZE            512


//-- SX12xx & SPI

#define SPI_USE_SUBGHZSPI

#define SX_BUSY                   0 // busy is provided by subghz, we need to define a dummy to fool sx126x_driver lib

#define SX_RX_EN                  IO_PA4
#define SX_TX_EN                  IO_PA5

//#define SX_DIO1_SYSCFG_EXTI_PORTx     LL_SYSCFG_EXTI_PORTA
//#define SX_DIO1_SYSCFG_EXTI_LINEx     LL_SYSCFG_EXTI_LINE1
//#define SX_DIO_EXTI_LINE_x            LL_EXTI_LINE_1 // not defining SX_DIO_EXTI_LINE_x has impact on ISR routine
#define SX_DIO_EXTI_IRQn              SUBGHZ_Radio_IRQn
#define SX_DIO_EXTI_IRQHandler        SUBGHZ_Radio_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY    11

void sx_init_gpio(void)
{
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

bool sx_busy_read(void)
{
    return subghz_is_busy();
}

// we need to provide it as we don't have SX_RESET defined, but is empty since reset is done by spi_init()
void sx_reset(void)
{
}

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
    // there is no EXTI_LINE_44 interrupt flag
    //LL_EXTI_DisableEvent_32_63(SX_DIO_EXTI_LINE_x);
    //LL_EXTI_DisableIT_32_63(SX_DIO_EXTI_LINE_x);

    NVIC_SetPriority(SX_DIO_EXTI_IRQn, SX_DIO_EXTI_IRQ_PRIORITY);
    //NVIC_EnableIRQ(SX_DIO_EXTI_IRQn);
}

void sx_dio_enable_exti_isr(void)
{ /*
    LL_EXTI_ClearFlag_0_31(SX_DIO_EXTI_LINE_x);
    LL_EXTI_EnableIT_0_31(SX_DIO_EXTI_LINE_x); */

    // there is no EXTI_LINE_44 interrupt flag
    //LL_EXTI_ClearFlag_32_63(SX_DIO_EXTI_LINE_x);
    //LL_EXTI_EnableIT_32_63(SX_DIO_EXTI_LINE_x);
    NVIC_EnableIRQ(SX_DIO_EXTI_IRQn);
}

void sx_dio_exti_isr_clearflag(void)
{
    // there is no EXTI_LINE_44 interrupt flag
}


//-- Out port
// UART_UARTx = LPUART1

void out_init_gpio(void)
{
}

void out_set_normal(void)
{ /*
    LL_USART_Disable(LPUART1);
    LL_USART_SetTXPinLevel(LPUART1, LL_USART_TXPIN_LEVEL_INVERTED);
    LL_USART_Enable(LPUART1); */
}

void out_set_inverted(void)
{ /*
    LL_USART_Disable(LPUART1);
    LL_USART_SetTXPinLevel(LPUART1, LL_USART_TXPIN_LEVEL_STANDARD);
    LL_USART_Enable(LPUART1); */
}


//-- Button

#define BUTTON                    IO_PB13

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}

bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON);
}


//-- LEDs

#define LED_GREEN                 IO_PB4
#define LED_RED                   IO_PA9

void leds_init(void)
{
    gpio_init(LED_GREEN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);
}

void led_green_off(void) { gpio_low(LED_GREEN); }
void led_green_on(void) { gpio_high(LED_GREEN); }
void led_green_toggle(void) { gpio_toggle(LED_GREEN); }

void led_red_off(void) { gpio_high(LED_RED); }
void led_red_on(void) { gpio_low(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- POWER

#define POWER_GAIN_DBM            0 // gain of a PA stage if present
#define POWER_SX126X_MAX_DBM      SX126X_POWER_MAX // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           2 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_0_DBM, .mW = 1 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_22_DBM, .mW = 158 },
};


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_9,
};

uint32_t portb[] = {
    LL_GPIO_PIN_6, LL_GPIO_PIN_7,
};

uint32_t portc[] = {
};









