//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// OUT Interface
//********************************************************
#ifndef OUT_INTERFACE_H
#define OUT_INTERFACE_H
#pragma once


#include "../Common/hal/hal.h" // not needed but helps editor to get defines correct LOL
#include "out.h"


extern tRxStats rxstats;


#ifdef USE_OUT

//-------------------------------------------------------
// Interface Implementation

#include "../modules/stm32ll-lib/src/stdstm32-uart.h"


class tOut : public OutBase
{
  public:
    void Init(void)
    {
        OutBase::Init(&Setup.Rx);
        out_init_gpio();
        uart_init_isroff();
    }

#if defined DEVICE_HAS_OUT || defined DEVICE_HAS_OUT_NORMAL || defined DEVICE_HAS_OUT_INTERNAL
    bool config_crsf(bool enable_flag) override
    {
        if (enable_flag) {
            uart_setprotocol(416666, XUART_PARITY_NO, UART_STOPBIT_1);
            out_set_normal();
        }
        return true;
    }

    bool config_sbus_inverted(bool enable_flag) override
    {
        if (enable_flag) {
            uart_setprotocol(100000, XUART_PARITY_EVEN, UART_STOPBIT_2);
            out_set_normal();
        }
        return true;
    }
#endif

#if defined DEVICE_HAS_OUT || defined DEVICE_HAS_OUT_INVERTED || defined DEVICE_HAS_OUT_INTERNAL
    bool config_sbus(bool enable_flag) override
    {
        if (enable_flag) {
            uart_setprotocol(100000, XUART_PARITY_EVEN, UART_STOPBIT_2);
            out_set_inverted();
        }
        return true;
    }
#endif

#if defined DEVICE_HAS_OUT_INTERNAL
    bool config_crsf_tx_jrpin5(bool enable_flag) override
    {
        if (enable_flag) {
            uart_setprotocol(400000, XUART_PARITY_NO, UART_STOPBIT_1);
            out_set_inverted();
        }
        return true;
    }
#endif

    void putc(char c) override { uart_putc(c); }

    void SendLinkStatistics(void)
    {
        tOutLinkStats lstats = {
          .receiver_rssi1 = stats.last_rssi1,
          .receiver_rssi2 = stats.last_rssi2,
          .receiver_LQ = rxstats.GetLQ_rc(),
          .receiver_snr = stats.GetLastSnr(),
          .receiver_antenna = stats.last_antenna,
          .receiver_transmit_antenna = stats.last_transmit_antenna,
          .receiver_power_dbm = sx.RfPower_dbm(),
          .transmitter_rssi = stats.received_rssi,
          .transmitter_LQ = stats.received_LQ_serial,
          .transmitter_snr = 0,
          .transmitter_antenna = stats.received_antenna,
          .transmitter_transmit_antenna = stats.received_transmit_antenna,
          .mode = (uint8_t)Config.Mode,
        };
        if (USE_ANTENNA1 && USE_ANTENNA2) {
            lstats.antenna_config = 3;
        } else if (USE_ANTENNA2) {
            lstats.antenna_config = 2;
        } else {
            lstats.antenna_config = 1;
        }
        OutBase::SendLinkStatistics(&lstats);
    }
};

tOut out;


#else

class tOut : public OutBase
{
  public:
    void Init(void) 
    {
        OutBase::Init(&Setup.Rx);
    }

    void SendLinkStatistics(void) {}
    void SendLinkStatisticsDisconnected(void) {}
};

tOut out;

#endif // #ifdef USE_OUT

#endif // OUT_INTERFACE_H
