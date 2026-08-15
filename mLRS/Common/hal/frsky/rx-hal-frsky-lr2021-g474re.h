//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//*******************************************************

//-------------------------------------------------------
// RX FRSKY LR2021 STM32G474RE V1.2, as receiver
//-------------------------------------------------------

#define DEVICE_HAS_OUT
#define DEVICE_HAS_DRONECAN_FD
#define DEVICE_HAS_SINGLE_LED_RGB


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define EE_START_PAGE             120 // 512 kB flash, 2 kB page, G474 has dual bank flash, hence only 256 kB available

#define MICROS_TIMx               TIM3

#define CLOCK_TIMx                TIM2
#define CLOCK_IRQn                TIM2_IRQn
#define CLOCK_IRQHandler          TIM2_IRQHandler
//#define CLOCK_IRQ_PRIORITY        10


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF or SWUART = debug port

#define UARTB_USE_UART2_PA2PA3 // serial
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE // 1024 // 512
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE // 1024 // 512

#define UART_USE_UART4_PC10PC11 // out pin
#define UART_BAUD                 100000 // SBus normal baud rate, is being set later anyhow
#define UART_USE_TX
#define UART_TXBUFSIZE            256 // 512
#define UART_USE_TX_ISR
//#define UART_USE_RX
//#define UART_RXBUFSIZE            512

#define UARTF_USE_UART1_PA9PA10 // debug
#define UARTF_BAUD                115200
#define UARTF_USE_TX
#define UARTF_TXBUFSIZE           1024 // for CAN debug
#define UARTF_USE_TX_ISR
//#define UARTF_USE_RX
//#define UARTF_RXBUFSIZE           512


//-- CAN

#define CAN_USE_FDCAN2_PB5PB6
#define CAN_STB_NORMAL_LOW        IO_PC14 // TCAN1044: low: normal mode, high: standby mode, integrated pull up


//-- LR2021 & SPI - left RF chain, as seen from top

#define SPI_USE_SPI1              // PA5, PA6, PA7
#define SPI_CS_IO                 IO_PA4
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ

#define SX_RESET                  IO_PC7
#define SX_DIO                    IO_PC4
#define SX_BUSY                   IO_PA1
#define SX_VC                     IO_PB9

#define SX_DIO_SYSCFG_EXTI_PORTx     LL_SYSCFG_EXTI_PORTC
#define SX_DIO_SYSCFG_EXTI_LINEx     LL_SYSCFG_EXTI_LINE4
#define SX_DIO_EXTI_LINE_x           LL_EXTI_LINE_4
#define SX_DIO_EXTI_IRQn             EXTI4_IRQn
#define SX_DIO_EXTI_IRQHandler       EXTI4_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY    11

#define SX_USE_IRQ_DIO_NO         LR20XX_DIO_9
#define SX_USE_TCXO_VOLTAGE       LR20XX_TCXO_SUPPLY_VOLTAGE_3_3

#define SX_USE_RFSW_DIO_CONFIG  {{LR20XX_DIO_5, LR20XX_DIO_RF_SWITCH_CONFIG_RX_HF}, \
                                 {LR20XX_DIO_6, LR20XX_DIO_RF_SWITCH_CONFIG_TX_HF}, \
                                 {LR20XX_DIO_7, LR20XX_DIO_RF_SWITCH_CONFIG_TX_LF}, \
                                 {LR20XX_DIO_8, LR20XX_DIO_RF_SWITCH_CONFIG_RX_LF}, \
                                 {LR20XX_DIO_10, LR20XX_DIO_RF_SWITCH_CONFIG_TX_LF | LR20XX_DIO_RF_SWITCH_CONFIG_RX_LF}, \
                                 {LR20XX_DIO_11, LR20XX_DIO_RF_SWITCH_CONFIG_TX_HF }}

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(SX_DIO, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
    gpio_init(SX_VC, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

bool sx_busy_read(void)
{
    return (gpio_read_activehigh(SX_BUSY)) ? true : false;
}

void sx_amp_transmit(void)
{
    // done via DIO
}

void sx_amp_receive(void)
{
    // done via DIO
}

void sx_band(bool high_band)
{
    if (high_band) gpio_low(SX_VC); else gpio_high(SX_VC);
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


//-- LR2021 & SPIB - right RF chain, as seen from top

#define SPIB_USE_SPI2             // PB13, PB14, PB15
#define SPIB_CS_IO                IO_PB12
#define SPIB_USE_CLK_LOW_1EDGE    // datasheet says CPHA = 0  CPOL = 0
#define SPIB_USE_CLOCKSPEED_9MHZ

#define SX2_RESET                 IO_PC6
#define SX2_DIO                   IO_PC5
#define SX2_BUSY                  IO_PC12
#define SX2_VC                    IO_PB4

#define SX2_DIO_SYSCFG_EXTI_PORTx    LL_SYSCFG_EXTI_PORTC
#define SX2_DIO_SYSCFG_EXTI_LINEx    LL_SYSCFG_EXTI_LINE5
#define SX2_DIO_EXTI_LINE_x          LL_EXTI_LINE_5
#define SX2_DIO_EXTI_IRQn            EXTI9_5_IRQn
#define SX2_DIO_EXTI_IRQHandler      EXTI9_5_IRQHandler
//#define SX2_DIO_EXTI_IRQ_PRIORITY    11

#define SX2_USE_IRQ_DIO_NO        LR20XX_DIO_9
#define SX2_USE_TCXO_VOLTAGE      LR20XX_TCXO_SUPPLY_VOLTAGE_3_3
#define SX2_USE_RFSW_DIO_CONFIG   SX_USE_RFSW_DIO_CONFIG // use same as for SX

void sx2_init_gpio(void)
{
    gpio_init(SX2_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(SX2_DIO, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
    gpio_init(SX2_BUSY, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
    gpio_init(SX2_VC, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

bool sx2_busy_read(void)
{
    return (gpio_read_activehigh(SX2_BUSY)) ? true : false;
}

void sx2_amp_transmit(void)
{
    // done via DIO
}

void sx2_amp_receive(void)
{
    // done via DIO
}

void sx2_band(bool high_band)
{
    if (high_band) gpio_low(SX2_VC); else gpio_high(SX2_VC);
}

void sx2_dio_init_exti_isroff(void)
{
    LL_SYSCFG_SetEXTISource(SX2_DIO_SYSCFG_EXTI_PORTx, SX2_DIO_SYSCFG_EXTI_LINEx);

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


//-- Button
// let both buttons do the same
// we misuse the button init to keep the wireless ESP in reset

#define BUTTON_LEFT               IO_PB3 // left: PB3, right: PD2
#define BUTTON_RIGHT              IO_PD2
#define ESP_WIFI_BRIDGE_RESET     IO_PB1

void button_init(void)
{
    gpio_init(BUTTON_LEFT, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
    gpio_init(BUTTON_RIGHT, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);

    gpio_init(ESP_WIFI_BRIDGE_RESET, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT); // low -> esp is in reset
}

bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON_LEFT) || gpio_read_activelow(BUTTON_RIGHT);
}


//-- LEDs

#define WS2812_NUMBER_OF_LEDS     4
#define WS2812_IO                 IO_PC8
#define WS2812_IO_AF              IO_AF_4
#define WS2812_TIMx               TIM8
#define WS2812_TIMno              8
#define WS2812_CHno               3
#define WS2812_DMAx               DMA2
#define WS2812_DMA_CHANNEL_x      LL_DMA_CHANNEL_3
#define WS2812_FREQUENCY          160
#include "../../thirdparty/stdstm32-ws2812.h"

tWs2812Color leds_current_color;

void leds_set_color(tWs2812Color color)
{
    if (color == leds_current_color) return;
    leds_current_color = color;
    ws2812_fill(0, color);
    ws2812_fill(1, color);
    ws2812_send();
}

void leds_init(void)
{
    ws2812_init();
    leds_current_color = 0;
}

void led_red_off(void) { leds_set_color(0); }
void led_red_on(void) { leds_set_color(WS2812_RED); }
void led_red_toggle(void) { (leds_current_color == WS2812_RED) ? led_red_off() : led_red_on(); }

void led_green_off(void) { leds_set_color(0); }
void led_green_on(void) { leds_set_color(WS2812_GREEN); }
void led_green_toggle(void) { (leds_current_color == WS2812_GREEN) ? led_green_off() : led_green_on(); }

void led_blue_off(void) { leds_set_color(0); }
void led_blue_on(void) { leds_set_color(WS2812_BLUE); }
void led_blue_toggle(void) { (leds_current_color == WS2812_BLUE) ? led_blue_off() : led_blue_on(); }

void led_purple_off(void) { leds_set_color(0); }
void led_purple_on(void) { leds_set_color(WS2812_PURPLE); }
void led_purple_toggle(void) { (leds_current_color == WS2812_PURPLE) ? led_purple_off() : led_purple_on(); }


//-- Power Supply Detect

#define POWER_SUPPLY_DETECT_IO    IO_PC1
#define POWER_SUPPLY_LOW_POWER_MAX_IDX  2 // see below in rfpower_list[]

void power_supply_detect_init(void)
{
    gpio_init(POWER_SUPPLY_DETECT_IO, IO_MODE_INPUT_PD, IO_SPEED_DEFAULT);
}

bool power_supply_ok(void)
{
    return gpio_read_activehigh(POWER_SUPPLY_DETECT_IO);
}

void power_supply_set_leds(tWs2812Color color)
{
    ws2812_fill(2, color);
    ws2812_fill(3, color);
//xx    ws2812_send();
}


//-- POWER

#define POWER_GAIN_DBM_HF         26 // gain of a PA stage if present
#define POWER_GAIN_DBM_LF         20 // gain of a PA stage if present
//#define POWER_USE_DEFAULT_RFPOWER_CALC

#ifndef POWER_USE_DEFAULT_RFPOWER_CALC
void lr20xx_rfpower_calc(const int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm, const uint8_t frequency_band)
{
    if (frequency_band == SX_FHSS_FREQUENCY_BAND_2P4_GHZ) {
      if (power_dbm >= POWER_30_DBM) { // -> 30
          *sx_power = 24; // LR20XX_POWER_HF_MAX
          *actual_power_dbm = 30;
      } else if (power_dbm >= POWER_27_DBM) { // -> 27
          *sx_power = 10;
          *actual_power_dbm = 27;
      } else {
          *sx_power = ((int16_t)power_dbm - POWER_GAIN_DBM_HF) * 2; // LR20xx power is in units of 0.5 dBm
          *actual_power_dbm = *sx_power / 2 + POWER_GAIN_DBM_HF;
      }

    } else {
        if (power_dbm >= POWER_30_DBM) { // -> 30
            *sx_power = LR20XX_POWER_LF_22_DBM;
            *actual_power_dbm = 44; // LR20XX_POWER_LF_MAX
        } else if (power_dbm >= POWER_27_DBM) { // -> 27
            *sx_power = LR20XX_POWER_LF_22_DBM;
            *actual_power_dbm = 27;
        } else if (power_dbm >= POWER_24_DBM) { // -> 24
            *sx_power = LR20XX_POWER_LF_22_DBM;
            *actual_power_dbm = 24;
        } else {
            *sx_power = ((int16_t)power_dbm - POWER_GAIN_DBM_LF) * 2;
            if (*sx_power < LR20XX_POWER_LF_MIN) *sx_power = LR20XX_POWER_LF_MIN;
            if (*sx_power > LR20XX_POWER_LF_MAX) *sx_power = LR20XX_POWER_LF_MAX;
            *actual_power_dbm = *sx_power / 2 + POWER_GAIN_DBM_LF;
        }
    }
}
#endif

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_14_DBM, .mW = 25 },
//    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12, LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12, LL_GPIO_PIN_13, LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};

uint32_t portc[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12, LL_GPIO_PIN_14,
};

