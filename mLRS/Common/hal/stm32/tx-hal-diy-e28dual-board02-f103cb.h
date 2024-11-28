//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//********************************************************

//#define MLRS_FEATURE_JRPIN5
//#define MLRS_FEATURE_IN
//#define MLRS_FEATURE_DIVERSITY
//#define MLRS_FEATURE_NO_DIVERSITY

//-------------------------------------------------------
// TX DIY DUAL-E28 BOARD02 v010 STM32F103CB
//-------------------------------------------------------

#define DEVICE_HAS_DIVERSITY
#define DEVICE_HAS_JRPIN5 // requires diode from Tx to Rx soldered on the board
//#define DEVICE_HAS_IN
#define DEVICE_HAS_DEBUG_SWUART


#ifdef MLRS_FEATURE_JRPIN5
  #undef DEVICE_HAS_IN
  #define DEVICE_HAS_JRPIN5
#endif
#ifdef MLRS_FEATURE_IN
  #undef DEVICE_HAS_JRPIN5
  #define DEVICE_HAS_IN
#endif
#ifdef MLRS_FEATURE_DIVERSITY
  #define DEVICE_HAS_DIVERSITY
#endif
#ifdef MLRS_FEATURE_NO_DIVERSITY
  #undef DEVICE_HAS_DIVERSITY
#endif


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define EE_START_PAGE             124 // 128 kB flash, 1 kB page

#define MICROS_TIMx               TIM3


//-- UARTS
// UARTB = serial port
// UARTC = USB (CLI) or debug port
// UART = JR bay pin5 (SPORT)
// UARTE = in port, SBus or whatever

#define UARTB_USE_UART3_PB10PB11 // serial
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UARTC_USE_UART1_PA9PA10 // COM (CLI)
#define UARTC_BAUD                115200
#define UARTC_USE_TX
#define UARTC_TXBUFSIZE           TX_COM_TXBUFSIZE_SMALL
#define UARTC_USE_TX_ISR
#define UARTC_USE_RX
#define UARTC_RXBUFSIZE           TX_COM_RXBUFSIZE

#ifdef DEVICE_HAS_IN
#define UARTE_USE_UART2_PA2PA3 // in pin
#define UARTE_BAUD                100000 // SBus normal baud rate, is being set later anyhow
//#define UARTE_USE_TX
//#define UARTE_TXBUFSIZE           512
//#define UARTE_USE_TX_ISR
#define UARTE_USE_RX
#define UARTE_RXBUFSIZE           512
#endif
#ifdef DEVICE_HAS_JRPIN5
#define UART_USE_UART2_PA2PA3 // JR pin5, MBridge
#define UART_BAUD                 400000
#define UART_USE_TX
#define UART_TXBUFSIZE            512
#define UART_USE_TX_ISR
#define UART_USE_RX
#define UART_RXBUFSIZE            512

#define JRPIN5_TX_XOR             IO_PB9
#define JRPIN5_TX_SET_NORMAL      gpio_low(JRPIN5_TX_XOR)
#define JRPIN5_TX_SET_INVERTED    gpio_high(JRPIN5_TX_XOR)
#define JRPIN5_RX_XOR             IO_PC15
#define JRPIN5_RX_SET_NORMAL      gpio_low(JRPIN5_RX_XOR)
#define JRPIN5_RX_SET_INVERTED    gpio_high(JRPIN5_RX_XOR)
#endif

#define SWUART_USE_TIM2 // debug
#define SWUART_TX_IO              IO_PB7 // is the BUZZER/EXT pad
#define SWUART_BAUD               115200
#define SWUART_USE_TX
#define SWUART_TXBUFSIZE          512
//#define SWUART_TIM_IRQ_PRIORITY   9


//-- SX1: SX12xx & SPI

#define SPI_USE_SPI1 // PA5, PA6, PA7
#define SPI_CS_IO                 IO_PB0
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PB5
#define SX_DIO1                   IO_PB3
#define SX_BUSY                   IO_PB4
#define SX_RX_EN                  IO_PB8
#define SX_TX_EN                  IO_PB12

#define SX_DIO1_GPIO_AF_EXTI_PORTx    LL_GPIO_AF_EXTI_PORTB
#define SX_DIO1_GPIO_AF_EXTI_LINEx    LL_GPIO_AF_EXTI_LINE3
#define SX_DIO_EXTI_LINE_x            LL_EXTI_LINE_3
#define SX_DIO_EXTI_IRQn              EXTI3_IRQn
#define SX_DIO_EXTI_IRQHandler        EXTI3_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY    11

#define SX_USE_REGULATOR_MODE_DCDC

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(SX_DIO1, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

bool sx_busy_read(void)
{
    return (gpio_read_activehigh(SX_BUSY)) ? true : false;
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

void sx_dio_exti_isr_clearflag(void)
{
    LL_EXTI_ClearFlag_0_31(SX_DIO_EXTI_LINE_x);
}


//-- SX12xx II & SPIB

#define SPIB_USE_SPI2             // PB13, PB14, PB15
#define SPIB_CS_IO                IO_PA8
#define SPIB_USE_CLK_LOW_1EDGE    // datasheet says CPHA = 0  CPOL = 0
#define SPIB_USE_CLOCKSPEED_9MHZ

#define SX2_RESET                 IO_PA4
#define SX2_DIO1                  IO_PA1
#define SX2_BUSY                  IO_PA0
#define SX2_RX_EN                 IO_PB6
#define SX2_TX_EN                 IO_PB1

#define SX2_DIO1_GPIO_AF_EXTI_PORTx   LL_GPIO_AF_EXTI_PORTA
#define SX2_DIO1_GPIO_AF_EXTI_LINEx   LL_GPIO_AF_EXTI_LINE1
#define SX2_DIO_EXTI_LINE_x           LL_EXTI_LINE_1
#define SX2_DIO_EXTI_IRQn             EXTI1_IRQn
#define SX2_DIO_EXTI_IRQHandler       EXTI1_IRQHandler
//#define SX2_DIO_EXTI_IRQ_PRIORITY   11

#define SX2_USE_REGULATOR_MODE_DCDC

void sx2_init_gpio(void)
{
    gpio_init(SX2_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(SX2_DIO1, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
    gpio_init(SX2_BUSY, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
    gpio_init(SX2_TX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
    gpio_init(SX2_RX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

bool sx2_busy_read(void)
{
    return (gpio_read_activehigh(SX2_BUSY)) ? true : false;
}

void sx2_amp_transmit(void)
{
    gpio_low(SX2_RX_EN);
    gpio_high(SX2_TX_EN);
}

void sx2_amp_receive(void)
{
    gpio_low(SX2_TX_EN);
    gpio_high(SX2_RX_EN);
}

void sx2_dio_init_exti_isroff(void)
{
    LL_GPIO_AF_SetEXTISource(SX2_DIO1_GPIO_AF_EXTI_PORTx, SX2_DIO1_GPIO_AF_EXTI_LINEx);

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

void sx2_dio_exti_isr_clearflag(void)
{
    LL_EXTI_ClearFlag_0_31(SX2_DIO_EXTI_LINE_x);
}


//-- In port

#define IN_XOR                    IO_PC15

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

#define BUTTON                    IO_PA15

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}

bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON);
}


//-- LEDs

#define LED_GREEN 		          IO_PC14
#define LED_RED		              IO_PC13

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

#define POWER_PA_E28_2G4M27SX
#include "../hal-power-pa.h"


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11,
    LL_GPIO_PIN_12, LL_GPIO_PIN_13, LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};

uint32_t portc[] = {
    LL_GPIO_PIN_13, LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};


