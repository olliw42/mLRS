//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************

//-------------------------------------------------------
// XRC Technologies NOVALINK 433MHz Transmitter, STM32G473CE
//-------------------------------------------------------

/*
Pinout

label    pin      functions                      alternative functions
TX1      PA9      U1_TX, ESP32-C3 RXD0 pin       I2C3_SMBA, I2C2_SCL, I2S3_MCK, TIM1_CH2, USART1_TX, OMP5_OUT, TIM15_BKIN, TIM2_CH3, SAI1_FS_A, EVENTOUT
RX1      PA10     U1_RX, ESP32-C3 TXD0 pin       TIM17_BKIN, USB_CRS_SYNC, I2C2_SMBA, SPI2_MISO, TIM1_CH3, USART1_RX, COMP6_OUT, TIM2_CH4, TIM8_BKIN, SAI1_D1, SAI1_SD_A, EVENTOUT
TX2      PB3      U2_TX                          JTDO-TRACESWO, TIM2_CH2, TIM4_ETR, UCPD1_CRS_SYNC, TIM8_CH1N, SPI1_SCK, SPI3_SCK/I2S3_CK, USART2_TX, TIM3_ETR, FDCAN3_RX, SAI1_SCK_B, EVENTOUT
RX2      PB4      U2_RX                          JTRST, TIM16_CH1, TIM3_CH1, TIM8_CH2N, SPI1_MISO, SPI3_MISO, USART2_RX, UART5_RTS_DE, TIM17_BKIN, FDCAN3_TX, SAI1_MCLK_B, EVENTOUT
LTx      PA2      LPU1_TX                        TIM2_CH3, TIM5_CH3, USART2_TX, COMP2_OUT, TIM15_CH1, QUADSPI1_BK1_NCS, LPUART1_TX, UCPD1_FRSTX, EVENTOUT
LRx      PA3      LPU1_RX                        TIM2_CH4, TIM5_CH4, SAI1_CK1, USART2_RX, TIM15_CH2, QUADSPI1_CLK, LPUART1_RX, SAI1_MCLK_A, EVENTOUT
CAN_TX   PB13     CAN_TX                         SPI2_SCK/I2S2_CK, TIM1_CH1N, USART3_CTS, LPUART1_CTS, FDCAN2_TX, EVENTOUT
CAN_RX   PB12     CAN_RX                         TIM5_ETR, I2C2_SMBA, SPI2_NSS/I2S2_WS, TIM1_BKIN, USART3_CK, LPUART1_RTS_DE, FDCAN2_RX, EVENTOUT
D-       PA11     USB DM                         SPI2_MOSI/I2S2_SD, TIM1_CH1N, USART1_CTS, COMP1_OUT, FDCAN1_RX, TIM4_CH1, TIM1_CH4, TIM1_BKIN2, EVENTOUT
D+       PA12     USB DP                         TIM16_CH1, I2SCKIN, TIM1_CH2N, USART1_RTS_DE, COMP2_OUT, FDCAN1_TX, TIM4_CH2, TIM1_ETR, EVENTOUT
SWD      PA13     SWDIO                          SWDIO-JTMS, TIM16_CH1N, I2C4_SCL, I2C1_SCL, IR_OUT, USART3_CTS, TIM4_CH3, SAI1_SD_B, EVENTOUT
SWC      PA14     SWDCLK                         SWCLK-JTCK, LPTIM1_OUT, I2C4_SMBA, I2C1_SDA, TIM8_CH2, TIM1_BKIN, USART2_TX, SAI1_FS_B, EVENTOUT

*/

//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define EE_START_PAGE             252 // 512 kB flash, 2 kB page

#define MICROS_TIMx               TIM3
#define MICROS_TIM_NAMEPREFIX     TIM3_

#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_IN_ON_JRPIN5_TX
#define DEVICE_HAS_ESP_WIFI_BRIDGE_ESP32C3    // target has ESP32-C3 with RESET,GPIO support
#define DEVICE_HAS_ESP_WIFI_BRIDGE_CONFIGURE  // target has ESPxx which allows configuration
#define DEVICE_HAS_COM_ON_USB
#define DEVICE_HAS_NO_DEBUG

//-- UARTS
// UARTB = serial port
// UARTD = Mavlink port/wireless bridge port
// UART  = JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF or SWUART = debug port

#define UARTB_USE_LPUART1_PA2PA3 // serial
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UARTD_USE_UART1_PA9PA10 // Mavlink/wireless bridge
#define UARTD_BAUD                115200
#define UARTD_USE_TX
#define UARTD_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTD_USE_TX_ISR
#define UARTD_USE_RX
#define UARTD_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UART_USE_UART2_PB3PB4 // JR pin5, MBridge
#define UART_BAUD                 400000
#define UART_USE_TX
#define UART_TXBUFSIZE            512
#define UART_USE_TX_ISR
#define UART_USE_RX
#define UART_RXBUFSIZE            512

#define JRPIN5_FULL_INTERNAL_ON_TX // does not require an external diode

#define UARTE_USE_UART2_PB3PB4 // in pin
#define UARTE_BAUD                100000 // SBus normal baud rate, is being set later anyhow

#define UARTE_USE_RX
#define UARTE_RXBUFSIZE           512

#define UARTF_USE_LPUART1_PA2PA3 // debug
#define UARTF_BAUD                115200
#define UARTF_USE_TX
#define UARTF_TXBUFSIZE           512
#define UARTF_USE_TX_ISR

//-- SX1: SX127x & SPI

#define SPI_USE_SPI1              // PA5, PA6, PA7
#define SPI_CS_IO                 IO_PA4
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PB6
#define SX_DIO0                   IO_PA15
#define SX_DIO1                   IO_PB7


#define SX_DIO_SYSCFG_EXTI_PORTx      LL_SYSCFG_EXTI_PORTA
#define SX_DIO_SYSCFG_EXTI_LINEx      LL_SYSCFG_EXTI_LINE15
#define SX_DIO_EXTI_LINE_x            LL_EXTI_LINE_15
#define SX_DIO_EXTI_IRQn              EXTI15_10_IRQn
#define SX_DIO_EXTI_IRQHandler        EXTI15_10_IRQHandler

// TODO for FSK
#define SX_DIO1_SYSCFG_EXTI_PORTx     LL_SYSCFG_EXTI_PORTB
#define SX_DIO1_SYSCFG_EXTI_LINEx     LL_SYSCFG_EXTI_LINE7
#define SX_DIO1_EXTI_LINE_x           LL_EXTI_LINE_7
#define SX_DIO1_EXTI_IRQn             EXTI9_5_IRQn
#define SX_DIO1_EXTI_IRQHandler       EXTI9_5_IRQHandler

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(SX_DIO0, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
    gpio_init(SX_DIO1, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
}

void sx_amp_transmit(void)
{

}

void sx_amp_receive(void)
{

}

void sx_dio_init_exti_isroff(void)
{
    LL_SYSCFG_SetEXTISource(SX_DIO_SYSCFG_EXTI_PORTx, SX_DIO_SYSCFG_EXTI_LINEx);

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

//-- Button, pull-down externally, shared with boot0, active high

#define BUTTON                    IO_PB0

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PD, IO_SPEED_DEFAULT);
}

bool button_pressed(void)
{
    return gpio_read_activehigh(BUTTON);
}

//-- LEDs

#define LED_GREEN                 IO_PA1
#define LED_RED                   IO_PA0

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

//-- In port
// this is nasty, UARTE defines not yet known, but cumbersome to add, so we include the lib
#ifdef DEVICE_HAS_IN
#include "../../../modules/stm32ll-lib/src/stdstm32-uarte.h"

void in_init_gpio(void)
{
}

void in_set_normal(void)
{
    LL_USART_Disable(UARTE_UARTx);
    LL_USART_SetTXRXSwap(UARTE_UARTx, LL_USART_TXRX_SWAPPED);
    LL_USART_SetRXPinLevel(UARTE_UARTx, LL_USART_RXPIN_LEVEL_STANDARD);
    LL_USART_Enable(UARTE_UARTx);
    gpio_init_af(UARTE_TX_IO, IO_MODE_INPUT_PU, UARTE_IO_AF, IO_SPEED_VERYFAST);
}

void in_set_inverted(void)
{
    LL_USART_Disable(UARTE_UARTx);
    LL_USART_SetTXRXSwap(UARTE_UARTx, LL_USART_TXRX_SWAPPED);
    LL_USART_SetRXPinLevel(UARTE_UARTx, LL_USART_RXPIN_LEVEL_INVERTED);
    LL_USART_Enable(UARTE_UARTx);
    gpio_init_af(UARTE_TX_IO, IO_MODE_INPUT_PD, UARTE_IO_AF, IO_SPEED_VERYFAST);
}
#endif

#define ESP_GPIO0               IO_PA8 // boot0/IO9 on C3
#define ESP_RESET               IO_PB5

#ifdef DEVICE_HAS_ESP_WIFI_BRIDGE_ESP32C3

void esp_init(void)
{
    gpio_init(ESP_GPIO0, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT); // low -> esp will start in bootloader mode
    gpio_init(ESP_RESET, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);  // low -> esp is in reset
}

void esp_reset_high(void) { gpio_high(ESP_RESET); }
void esp_reset_low(void)  { gpio_low(ESP_RESET); }

void esp_gpio0_high(void) { gpio_high(ESP_GPIO0); }
void esp_gpio0_low(void)  { gpio_low(ESP_GPIO0); }

#endif

//-- POWER

#define POWER_PA_XRC_NOVALINK433
#include "../hal-power-pa.h"

//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3,
    LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11,
    LL_GPIO_PIN_12, LL_GPIO_PIN_13, LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_3, LL_GPIO_PIN_4,
    LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_12, LL_GPIO_PIN_13,
};

uint32_t portc[] = {
};
