//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// IN Interface
//*******************************************************
#ifndef IN_INTERFACE_H
#define IN_INTERFACE_H
#pragma once


#include "../Common/hal/hal.h" // not needed but helps editor to get defines correct LOL
#include "in.h"


#ifdef USE_IN

//-------------------------------------------------------
// Interface Implementation

#if !(defined DEVICE_HAS_IN_ON_JRPIN5_RX || defined DEVICE_HAS_IN_ON_JRPIN5_TX)
// DEVICE_HAS_IN or DEVICE_HAS_IN_NORMAL or DEVICE_HAS_IN_INVERTED

#include "../modules/stm32ll-lib/src/stdstm32-uarte.h"


class tIn : public tInBase
{
  public:
    void Init(bool enable_flag)
    {
        tInBase::Init(enable_flag);
        if (!enable_flag) return;

        in_init_gpio();

        uarte_init_isroff();
    }

#if defined DEVICE_HAS_IN || defined DEVICE_HAS_IN_INVERTED
    bool config_sbus(bool enable_flag) override
    {
        uarte_rx_enableisr(DISABLE);
        if (enable_flag) {
            uarte_setprotocol(100000, XUART_PARITY_EVEN, UART_STOPBIT_2);
            in_set_inverted();
            uarte_rx_enableisr(ENABLE);
        }
        return true;
    }
#endif

#if defined DEVICE_HAS_IN || defined DEVICE_HAS_IN_NORMAL
    bool config_sbus_inverted(bool enable_flag) override
    {
        uarte_rx_enableisr(DISABLE);
        if (enable_flag) {
            uarte_setprotocol(100000, XUART_PARITY_EVEN, UART_STOPBIT_2);
            in_set_normal();
            uarte_rx_enableisr(ENABLE);
        }
        return true;
    }
#endif

    bool available(void) override { return uarte_rx_available(); }
    char getc(void) override { return uarte_getc(); }
};


#else
// DEVICE_HAS_IN_ON_JRPIN5_RX or DEVICE_HAS_IN_ON_JRPIN5_TX

#include "jr_pin5_interface.h" // in case DEVICE_HAS_JRPIN5 was not defined


class tIn : public tInBase
{
  public:
    void Init(bool enable_flag)
    {
        tInBase::Init(enable_flag);
        if (!enable_flag) return;

        uart_rx_callback_ptr = &uart_rx_putc_torxbuf;
        uart_tc_callback_ptr = &uart_tc_callback_dummy;

        uart_init_isroff();

#ifdef JRPIN5_FULL_INTERNAL_ON_RX_TX
  #ifdef DEVICE_HAS_IN_ON_JRPIN5_TX
        gpio_init(UART_RX_IO, IO_MODE_INPUT_ANALOG, IO_SPEED_VERYFAST);
  #else
        gpio_init(UART_TX_IO, IO_MODE_INPUT_ANALOG, IO_SPEED_VERYFAST);
  #endif
#endif
    }

    bool config_sbus(bool enable_flag) override
    {
        uart_rx_enableisr(DISABLE);
        if (enable_flag) {
            uart_setprotocol(100000, XUART_PARITY_EVEN, UART_STOPBIT_2);
            set_inverted();
            uart_rx_enableisr(ENABLE);
        }
        return true;
    }

    bool config_sbus_inverted(bool enable_flag) override
    {
        uart_rx_enableisr(DISABLE);
        if (enable_flag) {
            uart_setprotocol(100000, XUART_PARITY_EVEN, UART_STOPBIT_2);
            set_normal();
            uart_rx_enableisr(ENABLE);
        }
        return true;
    }

    bool available(void) override { return uart_rx_available(); }
    char getc(void) override { return uart_getc(); }

    void set_normal(void)
    {
        LL_USART_Disable(UART_UARTx);
#ifdef DEVICE_HAS_IN_ON_JRPIN5_TX
        LL_USART_SetTXRXSwap(UART_UARTx, LL_USART_TXRX_SWAPPED);
#endif
        LL_USART_SetRXPinLevel(UART_UARTx, LL_USART_RXPIN_LEVEL_STANDARD);
        LL_USART_Enable(UART_UARTx);
#ifdef DEVICE_HAS_IN_ON_JRPIN5_TX
        gpio_init_af(UART_TX_IO, IO_MODE_INPUT_PU, UART_IO_AF, IO_SPEED_VERYFAST);
#else
        gpio_init_af(UART_RX_IO, IO_MODE_INPUT_PU, UART_IO_AF, IO_SPEED_VERYFAST);
#endif
    }

    void set_inverted(void)
    {
        LL_USART_Disable(UART_UARTx);
#ifdef DEVICE_HAS_IN_ON_JRPIN5_TX
        LL_USART_SetTXRXSwap(UART_UARTx, LL_USART_TXRX_SWAPPED);
#endif
        LL_USART_SetRXPinLevel(UART_UARTx, LL_USART_RXPIN_LEVEL_INVERTED);
        LL_USART_Enable(UART_UARTx);
#ifdef DEVICE_HAS_IN_ON_JRPIN5_TX
        gpio_init_af(UART_TX_IO, IO_MODE_INPUT_PD, UART_IO_AF, IO_SPEED_VERYFAST);
#else
        gpio_init_af(UART_RX_IO, IO_MODE_INPUT_PD, UART_IO_AF, IO_SPEED_VERYFAST);
#endif
    }
};


#endif

#else
// !defined USE_IN

class tIn : public tInBase
{
};

#endif // if (defined USE_IN)


tIn in;


#endif // IN_INTERFACE_H
