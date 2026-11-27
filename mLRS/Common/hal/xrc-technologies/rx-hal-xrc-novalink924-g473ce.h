//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************

//-------------------------------------------------------
// XRC Technologies NOVALINK 915MHz/2.4GHz Receiver, STM32G473CE
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


#define DEVICE_HAS_OUT
#define DEVICE_HAS_DRONECAN_FD


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define EE_START_PAGE             120 // 512 kB flash, 2 kB page

#define MICROS_TIMx               TIM3
#define MICROS_TIM_NAMEPREFIX     TIM3_

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

#define UART_USE_UART2_PB3PB4 // out pin
#define UART_BAUD                 100000 // SBus normal baud rate, is being set later anyhow
#define UART_USE_TX
#define UART_TXBUFSIZE            256 // 512
#define UART_USE_TX_ISR

#define UARTF_USE_LPUART1_PA2PA3 // debug
#define UARTF_BAUD                115200
#define UARTF_USE_TX
#define UARTF_TXBUFSIZE           1024 // for CAN debug
#define UARTF_USE_TX_ISR


//-- CAN

#define CAN_USE_FDCAN2_PB12PB13


//-- SX1: SX127x & SPI

#define SPI_USE_SPI1              // PA5, PA6, PA7
#define SPI_CS_IO                 IO_PA4
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PB6
#define SX_DIO                    IO_PB7
#define SX_BUSY                   IO_PC4

#define SX_DIO_EXTI               EXTI_IO_PB7
#define SX_DIO_EXTI_IRQn          EXTI9_5_IRQn
#define SX_DIO_EXTI_IRQHandler    EXTI9_5_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY    13

#define SX_USE_IRQ_DIO_NO         LR20XX_DIO_9
#define SX_USE_TCXO_VOLTAGE       LR20XX_TCXO_SUPPLY_VOLTAGE_3_3
#define SX_USE_RFSW_DIO_CONFIG  {{LR20XX_DIO_5, LR20XX_DIO_RF_SWITCH_CONFIG_RX_HF}, \
                                 {LR20XX_DIO_6, LR20XX_DIO_RF_SWITCH_CONFIG_TX_LF}, \
                                 {LR20XX_DIO_7, LR20XX_DIO_RF_SWITCH_CONFIG_RX_HF | LR20XX_DIO_RF_SWITCH_CONFIG_TX_HF}, \
                                 {LR20XX_DIO_8, LR20XX_DIO_RF_SWITCH_CONFIG_TX_HF }}

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(SX_DIO, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
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

void sx_band(bool high_band)
{
}

void sx_dio_init_exti_isroff(void)
{
    exti_init_isroff(SX_DIO_EXTI, EXTI_TRIG_RISING);

    NVIC_SetPriority(SX_DIO_EXTI_IRQn, SX_DIO_EXTI_IRQ_PRIORITY);
    NVIC_EnableIRQ(SX_DIO_EXTI_IRQn);
}

void sx_dio_enable_exti_isr(void)
{
    exti_enableisr(SX_DIO_EXTI);
}

void sx_dio_exti_isr_clearflag(void)
{
    exti_clearisrflag(SX_DIO_EXTI);
}


//-- Out port
// this is nasty, UART defines not yet known, but cumbersome to add, so we include the lib
#include "../../../modules/stm32ll-lib/src/stdstm32-uart.h"

void out_init_gpio(void)
{
}

void out_set_normal(void)
{
    LL_USART_Disable(UART_UARTx);
    LL_USART_SetTXPinLevel(UART_UARTx, LL_USART_TXPIN_LEVEL_STANDARD);
    LL_USART_Enable(UART_UARTx);
}

void out_set_inverted(void)
{
    LL_USART_Disable(UART_UARTx);
    LL_USART_SetTXPinLevel(UART_UARTx, LL_USART_TXPIN_LEVEL_INVERTED);
    LL_USART_Enable(UART_UARTx);
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


//-- POWER

void lr20xx_rfpower_calc(const int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm, const uint8_t frequency_band)
{
    if (frequency_band == SX_FHSS_FREQUENCY_BAND_2P4_GHZ) {
        if (power_dbm >= 30)        {   *sx_power =  -3;   *actual_power_dbm = 30;
        } else if (power_dbm >= 27) {   *sx_power = -10;   *actual_power_dbm = 27;
        } else if (power_dbm >= 24) {   *sx_power = -17;   *actual_power_dbm = 24;
        } else if (power_dbm >= 20) {   *sx_power = -27;   *actual_power_dbm = 20;
        } else {
            *sx_power = -39; // equals LR20XX_POWER_HF_MIN
            *actual_power_dbm = 15;
        }
    } else {
        if (power_dbm >= 30)        {   *sx_power =  35;   *actual_power_dbm = 30;
        } else if (power_dbm >= 27) {   *sx_power =  26;   *actual_power_dbm = 27;
        } else if (power_dbm >= 24) {   *sx_power =  21;   *actual_power_dbm = 24;
        } else if (power_dbm >= 20) {   *sx_power =  12;   *actual_power_dbm = 20;
        } else if (power_dbm >= 14) {   *sx_power =  -1;   *actual_power_dbm = 14;
        } else if (power_dbm >= 10) {   *sx_power = -12;   *actual_power_dbm = 10;
        } else {
            *sx_power = -18; // equals LR20XX_POWER_LF_MIN
            *actual_power_dbm = 6;
        }
    }
}

#define RFPOWER_DEFAULT           2 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_14_DBM, .mW = 25 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3,
    LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11,
    LL_GPIO_PIN_12, LL_GPIO_PIN_13, LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_3, LL_GPIO_PIN_4,
    LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_12, LL_GPIO_PIN_13,
};

uint32_t portc[] = {
    LL_GPIO_PIN_4
};
