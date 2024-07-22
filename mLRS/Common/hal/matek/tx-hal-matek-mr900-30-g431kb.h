//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************

//-------------------------------------------------------
// MATEKSYS mR900-30 STM32G431KB, as Tx module
//-------------------------------------------------------
#include "hal-matek-mr-g431kb-common.h"


#define DEVICE_HAS_JRPIN5
//#define DEVICE_HAS_IN
#define DEVICE_HAS_IN_ON_JRPIN5_TX
#define DEVICE_HAS_NO_DEBUG
#define DEVICE_HAS_FAN_ONOFF


//-- Timers, Timing, EEPROM, and such stuff


//-- UARTS
// UARTB = serial port
// UARTC = COM (CLI)
// UARTD = serial2 BT/ESP port
// UART  = JR bay pin5
// UARTE = in port, SBus or whatever

#define UARTB_USE_UART1_PA9PA10 // serial
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UARTC_USE_LPUART1_PA2PA3 // com USB/CLI
#define UARTC_BAUD                TX_COM_BAUDRATE
#define UARTC_USE_TX
#define UARTC_TXBUFSIZE           TX_COM_TXBUFSIZE
#define UARTC_USE_TX_ISR
#define UARTC_USE_RX
#define UARTC_RXBUFSIZE           TX_COM_RXBUFSIZE

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
//#define UARTE_USE_TX
//#define UARTE_TXBUFSIZE           512
//#define UARTE_USE_TX_ISR
#define UARTE_USE_RX
#define UARTE_RXBUFSIZE           512


//-- SX1: SX12xx & SPI


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


//-- Button


//-- LEDs


//-- Cooling Fan


//-- POWER

#define POWER_GAIN_DBM            28
#define POWER_SX126X_MAX_DBM      SX126X_POWER_MAX // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           2 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};



