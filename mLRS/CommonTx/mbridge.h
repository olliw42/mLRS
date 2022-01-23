//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// M Bridge
//********************************************************
#ifndef MBRIDGE_H
#define MMBRIDGE_H
#pragma once

#include <inttypes.h>

#include "mbridge_protocol.h"

class tMBridgeBase
{
  public:
    void Init(void);

    // interface to the uart hardware peripheral used for the bridge
    virtual void transmit_enable(bool flag);
    virtual bool mb_rx_available(void);
    virtual char mb_getc(void);
    virtual void mb_putc(char c);

    // for in-isr processing
    void parse_nextchar(uint8_t c, uint16_t tnow_us);
    bool transmit_start(void); // returns true if transmission should be started

    uint8_t send_serial(void);
    void send_command(void);

    typedef enum {
      STATE_IDLE = 0,

      STATE_MBRIDGE_RECEIVE_STX2,
      STATE_MBRIDGE_RECEIVE_LEN,
      STATE_MBRIDGE_RECEIVE_SERIALPACKET,
      STATE_MBRIDGE_RECEIVE_CHANNELPACKET,
      STATE_MBRIDGE_RECEIVE_COMMANDPACKET,

      STATE_TRANSMIT_START,
      STATE_TRANSMITING,
    } STATE_ENUM;

    uint8_t state;
    uint8_t len;
    uint8_t cnt;
    uint16_t tlast_us;

    uint8_t type;

    bool channels_received;
    tMBridgeChannelBuffer channels;

    bool cmd_received;
    uint8_t cmd_rx_frame[MBRIDGE_RX_COMMAND_FRAME_LEN];

    volatile uint8_t cmd_tx_available;
    uint8_t cmd_tx_frame[MBRIDGE_TX_COMMAND_FRAME_LEN];

    // mimics a serial interface to the main code, usually two fifo
    virtual bool serial_rx_available(void);
    virtual char serial_getc(void);
    virtual void serial_putc(char c);

    // send/receive commands interface to the main code, usually two fifo, can be empty
    void GetCommand(uint8_t* cmd, uint8_t* payload);
    void SendCommand(uint8_t cmd, uint8_t* payload, uint8_t payload_len);
};


#endif // MBRIDGE_H

