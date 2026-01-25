//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

/*
  Info on pin usage

  TX,RX (main pads on top&bottom):    serial
  TX2 (corner pad on bottom):         rc signal/CRSF/SBUS

  top:      side with button, LED, antenna, and PA chips
  bottom:   side with ESP chip
*/


//-------------------------------------------------------
// ESP32, ELRS RadioMaster XR4 RX
//-------------------------------------------------------

#include "rx-hal-generic-lr1121-td-esp32.h"


#define DEVICE_HAS_OUT


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#define UART_USE_SERIAL1 
#define UART_BAUD                 416666   // CRSF baud rate
#define UART_USE_TX_IO            IO_P18   // tx2 pad on the receiver
#define UART_USE_RX_IO            -1       // no Rx pin needed
#define UART_TXBUFSIZE            256


//-- Out port

void out_init_gpio(void) {}

void out_set_normal(void)
{
    // https://github.com/espressif/esp-idf/blob/release/v4.4/components/esp_rom/include/esp32/rom/gpio.h#L228-L242
    gpio_matrix_out((gpio_num_t)UART_USE_TX_IO, U1TXD_OUT_IDX, false, false);
}

void out_set_inverted(void) 
{
    gpio_matrix_out((gpio_num_t)UART_USE_TX_IO, U1TXD_OUT_IDX, true, false);
}


//-- LR11xx DIO switch control

#define SX_USE_RFSW_CTRL {15, 0, 12, 8, 8, 6, 0, 5}
