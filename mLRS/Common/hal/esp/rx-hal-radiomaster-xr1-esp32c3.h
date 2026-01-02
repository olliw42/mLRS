//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// ESP32, ELRS RadioMaster XR1 RX
//-------------------------------------------------------

#include "rx-hal-generic-c3-lr1121-esp32c3.h"


#define DEVICE_HAS_OUT


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#define UART_USE_SERIAL1 
#define UART_BAUD                 416666   // CRSF baud rate
#define UART_USE_TX_IO            IO_P19   // tx2 pad on the receiver
#define UART_USE_RX_IO            -1       // no Rx pin needed
#define UART_TXBUFSIZE            256


//-- LR11xx DIO switch control

#define SX_USE_RFSW_CTRL {15, 0, 12, 8, 8, 6, 0, 5}
