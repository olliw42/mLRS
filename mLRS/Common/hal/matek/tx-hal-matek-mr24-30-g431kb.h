//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************

//#define MLRS_FEATURE_COM_ON_USB // this MLRS_FEATURE define can be used locally here


//-------------------------------------------------------
// MATEKSYS mR24-30 STM32G431KB, as Tx module
//-------------------------------------------------------
#include "hal-matek-mr-g431kb-common.h"


#define DEVICE_HAS_JRPIN5
//#define DEVICE_HAS_IN
#define DEVICE_HAS_IN_ON_JRPIN5_TX
#define DEVICE_HAS_NO_DEBUG
#define DEVICE_HAS_FAN_ONOFF
#define DEVICE_HAS_HC04_MODULE_ON_SERIAL

#ifdef MLRS_FEATURE_COM_ON_USB
#define DEVICE_HAS_COM_ON_USB
#endif


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

#ifndef DEVICE_HAS_COM_ON_USB
#define UARTC_USE_LPUART1_PA2PA3 // com USB/CLI
#define UARTC_BAUD                TX_COM_BAUDRATE
#define UARTC_USE_TX
#define UARTC_TXBUFSIZE           TX_COM_TXBUFSIZE
#define UARTC_USE_TX_ISR
#define UARTC_USE_RX
#define UARTC_RXBUFSIZE           TX_COM_RXBUFSIZE
#endif

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

#define SX_USE_REGULATOR_MODE_DCDC


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

#define POWER_GAIN_DBM            31 // gain of a PA stage if present
#define POWER_SX1280_MAX_DBM      SX1280_POWER_0_DBM // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_10_DBM, .mW = 20 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};




//-- Display I2C, OLED

/*#define I2C_USE_I2C1     // G431 PA13, PA14
#define I2C_CLOCKSPEED_400KHZ     // not all displays seem to work well with I2C_CLOCKSPEED_1000KHZ
#define I2C_USE_DMAMODE
#define MLRS_FEATURE_OLED
#define DEVICE_HAS_I2C_DISPLAY

  #define I2C_SCL_IO             IO_PA13
  #define I2C_SDA_IO             IO_PA14
  #define I2C_SCL_IO_AF          IO_AF_4
  #define I2C_SDA_IO_AF          IO_AF_4

  #define I2C_EV_IRQn            I2C1_EV_IRQn
  #define I2C_ER_IRQn            I2C1_ER_IRQn
  #define I2C_EV_IRQHandler      I2C1_EV_IRQHandler
  #define I2C_ER_IRQHandler      I2C1_ER_IRQHandler

  #define I2C_TX_DMAx_Channely_IRQn        DMA1_Channel2_IRQn
  #define I2C_RX_DMAx_Channely_IRQn        DMA1_Channel1_IRQn
  #define I2C_TX_DMAx_Channely_IRQHandler  DMA1_Channel2_IRQHandler
  #define I2C_RX_DMAx_Channely_IRQHandler  DMA1_Channel1_IRQHandler
*/
