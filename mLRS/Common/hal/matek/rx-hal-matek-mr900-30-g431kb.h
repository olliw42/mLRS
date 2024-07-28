//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************

//-------------------------------------------------------
// MATEKSYS mR900-30 STM32G431KB, as receiver
//-------------------------------------------------------

#define DEVICE_HAS_OUT
#define DEVICE_HAS_FAN_ONOFF


#include "hal-matek-mr-g431kb-common.h"


//-- Timers, Timing, EEPROM, and such stuff

#define CLOCK_TIMx                TIM2
#define CLOCK_IRQn                TIM2_IRQn
#define CLOCK_IRQHandler          TIM2_IRQHandler
//#define CLOCK_IRQ_PRIORITY        10


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTC = debug port

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
//#define UART_USE_RX
//#define UART_RXBUFSIZE            512

#define UARTC_USE_LPUART1_PA2PA3 // debug
#define UARTC_BAUD                115200
#define UARTC_USE_TX
#define UARTC_TXBUFSIZE           512
#define UARTC_USE_TX_ISR
//#define UARTC_USE_RX
//#define UARTC_RXBUFSIZE           512


//-- SX1: SX12xx & SPI


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


//-- LEDs


//-- Cooling Fan


//-- POWER
// SX126X power setting can vary from -9 .. 22 for -9 dBm ... 22 dBm
#include "../../setup_types.h"

void sx126x_rfpower_calc(const int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm, const uint8_t frequency_band)
{
    if (power_dbm >= POWER_30_DBM) {
        *sx_power = (frequency_band == SETUP_FREQUENCY_BAND_868_MHZ) ? 10 : 10;
        *actual_power_dbm = 30;
    } else
    if (power_dbm >= POWER_27_DBM) {
        *sx_power = (frequency_band == SETUP_FREQUENCY_BAND_868_MHZ) ? -7 : -4;
        *actual_power_dbm = 27;
    } else
    if (power_dbm >= POWER_24_DBM) {
        *sx_power = (frequency_band == SETUP_FREQUENCY_BAND_868_MHZ) ? -9 : -7;
        *actual_power_dbm = 24;
    } else
    if (power_dbm >= POWER_20_DBM) {
        *sx_power = -9;
        *actual_power_dbm = (frequency_band == SETUP_FREQUENCY_BAND_868_MHZ) ? 24 : 22;
    } else {
        *sx_power = -9;
        *actual_power_dbm = (frequency_band == SETUP_FREQUENCY_BAND_868_MHZ) ? 24 : 22;
    }
}

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN }, // doesn't make sense IMHO
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};



