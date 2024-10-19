//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//*******************************************************
// 24.Feb.2023: LED pin changed! pin for artificial GND (PA0)! consistent with tx-wioe5

//-------------------------------------------------------
// RX Seeedstudio Wio-E5 Mini Dev board STM32WLE5JC, https://wiki.seeedstudio.com/LoRa_E5_mini
//-------------------------------------------------------

#define DEVICE_HAS_OUT
#define DEVICE_HAS_SINGLE_LED
#define DEVICE_HAS_SYSTEMBOOT


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define EE_START_PAGE             120 // 256 kB flash, 2 kB page

#define MICROS_TIMx               TIM16

#define CLOCK_TIMx                TIM2
#define CLOCK_IRQn                TIM2_IRQn
#define CLOCK_IRQHandler          TIM2_IRQHandler
//#define CLOCK_IRQ_PRIORITY        10


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_UART2_PA2PA3 // serial // PA2,PA3
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE

#define UART_USE_LPUART1_PC1PC0 // out pin // PC1
#define UART_BAUD                 100000 // SBus normal baud rate, is being set later anyhow
#define UART_USE_TX
#define UART_TXBUFSIZE            256
#define UART_USE_TX_ISR
//#define UART_USE_RX
//#define UART_RXBUFSIZE            512
#define OUT_UARTx                 LPUART1 // UART_UARTx is not known yet, so define by hand

#define UARTF_USE_UART1_PB6PB7 // debug // PB6,PB7 usb plug
#define UARTF_BAUD                115200
#define UARTF_USE_TX
#define UARTF_TXBUFSIZE           512
#define UARTF_USE_TX_ISR
//#define UARTF_USE_RX
//#define UARTF_RXBUFSIZE           512


//-- SX12xx & SPI

#define SPI_USE_SUBGHZSPI
#define SPI_USE_CLOCKSPEED_12MHZ

#define SX_BUSY                   0 // busy is provided by subghz, we need to define a dummy to fool sx126x_driver lib
#define SX_HAS_NO_RESET           // SubGHz has no reset, reset is done by spi_init()

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
// we keep the green LED stuff in case a user wants it

#define LED_GREEN                 IO_PA15
#define LED_RED                   IO_PB5

void leds_init(void)
{
    gpio_init(LED_GREEN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);

    // pin IO_PB15 must be floating, is used as artificial pad for green LED!
    gpio_init(IO_PB15, IO_MODE_Z, IO_SPEED_DEFAULT);
}

void led_green_off(void) { gpio_low(LED_GREEN); }
void led_green_on(void) { gpio_high(LED_GREEN); }
void led_green_toggle(void) { gpio_toggle(LED_GREEN); }

void led_red_off(void) { gpio_high(LED_RED); }
void led_red_on(void) { gpio_low(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- SystemBootLoader

#define BOOT_BUTTON               IO_PB13

extern "C" { void delay_ms(uint16_t ms); }

void systembootloader_init(void)
{
    gpio_init(BOOT_BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (gpio_read_activelow(BOOT_BUTTON)) cnt++;
    }
    if (cnt > 12) {
        BootLoaderInit();
    }
}


//-- POWER

#define POWER_PA_NONE_SX126X
#include "../hal-power-pa.h"


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_2, LL_GPIO_PIN_3,
};

uint32_t portb[] = {
    LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_9, LL_GPIO_PIN_13,
};

uint32_t portc[] = {
    LL_GPIO_PIN_1,
};









