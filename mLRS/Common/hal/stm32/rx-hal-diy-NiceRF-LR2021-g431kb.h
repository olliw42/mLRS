//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// RX_DIY_NICERF_LR2021_G431KB v002 STM32G431KB
//-------------------------------------------------------

#define DEVICE_HAS_OUT


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define EE_START_PAGE             60 // 128 kB flash, 2 kB page

#define MICROS_TIMx               TIM3

#define CLOCK_TIMx                TIM2
#define CLOCK_IRQn                TIM2_IRQn
#define CLOCK_IRQHandler          TIM2_IRQHandler
//#define CLOCK_IRQ_PRIORITY        10


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_UART1_PA9PA10 // serial
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE // 1024 // 512
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE // 1024 // 512

#define UART_USE_UART2_PB3PA15 // out pin
#define UART_BAUD                 100000 // SBus normal baud rate, is being set later anyhow
#define UART_USE_TX
#define UART_TXBUFSIZE            256 // 512
#define UART_USE_TX_ISR
//#define UART_USE_RX
//#define UART_RXBUFSIZE            512
#define OUT_UARTx                 USART2 // UART_UARTx is not known yet, so define by hand

#define UARTF_USE_LPUART1_PA2PA3 // debug
#define UARTF_BAUD                115200
#define UARTF_USE_TX
#define UARTF_TXBUFSIZE           512
#define UARTF_USE_TX_ISR
//#define UARTF_USE_RX
//#define UARTF_RXBUFSIZE           512


//-- SX1: SX12xx & SPI

#define SPI_USE_SPI1              // PA5, PA6, PA7
#define SPI_CS_IO                 IO_PA4
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PB6
#define SX_DIO1                   IO_PB4 // DIO7
#define SX_BUSY                   IO_PB0

#define SX_DIO7                   IO_PB4
#define SX_DIO8                   IO_PB9
#define SX_DIO9                   IO_PB7

#define SX_DIO1_SYSCFG_EXTI_PORTx     LL_SYSCFG_EXTI_PORTB
#define SX_DIO1_SYSCFG_EXTI_LINEx     LL_SYSCFG_EXTI_LINE4
#define SX_DIO_EXTI_LINE_x            LL_EXTI_LINE_4
#define SX_DIO_EXTI_IRQn              EXTI4_IRQn
#define SX_DIO_EXTI_IRQHandler        EXTI4_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY    11

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(SX_DIO1, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
}

bool sx_busy_read(void)
{
    return (gpio_read_activehigh(SX_BUSY)) ? true : false;
}

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

void sx_dio_exti_isr_clearflag(void)
{
    LL_EXTI_ClearFlag_0_31(SX_DIO_EXTI_LINE_x);
}


//-- Out port

void out_init_gpio(void)
{
}

void out_set_normal(void)
{
    LL_USART_Disable(OUT_UARTx);
    LL_USART_SetTXPinLevel(OUT_UARTx, LL_USART_TXPIN_LEVEL_STANDARD);
    LL_USART_Enable(OUT_UARTx);
}

void out_set_inverted(void)
{
    LL_USART_Disable(OUT_UARTx);
    LL_USART_SetTXPinLevel(OUT_UARTx, LL_USART_TXPIN_LEVEL_INVERTED);
    LL_USART_Enable(OUT_UARTx);
}


//-- Button

#define BUTTON                    IO_PA11

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}

bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON);
}


//-- LEDs

#define LED_GREEN 		            IO_PA1
#define LED_RED		                IO_PA0

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


//-- POWER

//#define POWER_PA_NONE_LR20XX
//#include "../hal-power-pa.h"

#define POWER_GAIN_DBM            0 // gain of a PA stage if present
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           2 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_0_DBM, .mW = 1 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_14_DBM, .mW = 25 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_22_DBM, .mW = 158 },
};


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
};

uint32_t portc[] = {
};


