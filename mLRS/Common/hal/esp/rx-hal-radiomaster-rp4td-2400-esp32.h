//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// ESP32, ELRS RadioMaster RP4TD 2400 RX
//-------------------------------------------------------

#include "rx-hal-generic-2400-td-pa-esp32.h"


#define DEVICE_HAS_OUT


//-- UARTS
// UART = output port, SBus or whatever

#define UART_USE_SERIAL1 
#define UART_BAUD                 416666   // CRSF baud rate
#define UART_USE_TX_IO            18       // t pad on the receiver
#define UART_USE_RX_IO            -1       // no Rx pin needed
#define UART_TXBUFSIZE            128


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
