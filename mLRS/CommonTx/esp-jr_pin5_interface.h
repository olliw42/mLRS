//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// ESP JR Pin5 Interface Header
// Currently works only for internal modules (Full duplex)
//********************************************************
#ifndef ESP_JRPIN5_INTERFACE_H
#define ESP_JRPIN5_INTERFACE_H

#include "../Common/esp-lib/esp-uart.h"

class tPin5BridgeBase
{
  public:
    void Init(void);

    // telemetry handling
    bool telemetry_start_next_tick;
    uint16_t telemetry_state;

    void TelemetryStart(void);

    // interface to the uart hardware peripheral used for the bridge, may be called in isr context
    void pin5_tx_start(void) {}
    void pin5_putbuf(uint8_t* const buf, uint16_t len) {uart_putbuf(buf, len);}

    // for in-isr processing
    void pin5_tx_enable(bool enable_flag);
    virtual void parse_nextchar(uint8_t c) = 0;
    virtual bool transmit_start(void) = 0; // returns true if transmission should be started

    // actual isr functions
    void uart_rx_callback(uint8_t c);
    void uart_tc_callback(void);

    // parser
    typedef enum {
        STATE_IDLE = 0,

        // mBridge receive states
        STATE_RECEIVE_MBRIDGE_STX2,
        STATE_RECEIVE_MBRIDGE_LEN,
        STATE_RECEIVE_MBRIDGE_SERIALPACKET,
        STATE_RECEIVE_MBRIDGE_CHANNELPACKET,
        STATE_RECEIVE_MBRIDGE_COMMANDPACKET,

        // CRSF receive states
        STATE_RECEIVE_CRSF_LEN,
        STATE_RECEIVE_CRSF_PAYLOAD,
        STATE_RECEIVE_CRSF_CRC,

        // transmit states, used by all
        STATE_TRANSMIT_START,
        STATE_TRANSMITING,
    } STATE_ENUM;

    // not used in this class, but required by the children, so just add them here
    // no need for volatile since used only in isr context
    uint8_t state;
    uint8_t len;
    uint8_t cnt;
    uint16_t tlast_us;

    // check and rescue
    // the FRM303 can get stuck, whatever we tried, so brutal rescue
    // can't hurt generally as safety net
    uint32_t nottransmiting_tlast_ms;
    void CheckAndRescue(void);
    void uart_poll(void);
};


void tPin5BridgeBase::Init(void)
{
    state = STATE_IDLE;
    len = 0;
    cnt = 0;
    tlast_us = 0;

    telemetry_start_next_tick = false;
    telemetry_state = 0;

    nottransmiting_tlast_ms = 0;
    uart_init();
}


void tPin5BridgeBase::TelemetryStart(void)
{
    telemetry_start_next_tick = true;
}


//-------------------------------------------------------
// Interface to the uart hardware peripheral used for the bridge
// called in isr context

void tPin5BridgeBase::pin5_tx_enable(bool enable_flag)
{
    // nothing to do for full duplex
}


// we do not add a delay here before we transmit
// the logic analyzer shows this gives a 30-35 us gap nevertheless, which is perfect

void tPin5BridgeBase::uart_rx_callback(uint8_t c)
{
    // not yet used
    // parse_nextchar(c);
}


void tPin5BridgeBase::uart_tc_callback(void)
{
    // not needed for full duplex
}


//-------------------------------------------------------
// Check and rescue
// a good place to call it could be ChannelsUpdated()
// Note: For the FRM303 it was observed that the TC callback may be missed in the uart isr, basically when
// the jrpin5's uart isr priority is too low. This caused the jrpin5 loop to get stuck in STATE_TRANSMITING,
// and not even channel data would be received anymore (= very catastrophic). This code avoids this.
// With proper isr priorities, the issue is mainly gone, but the code remains, as safety net.
//
// For full duplex CRSF (internal module), this is used to poll for received bytes and pace telemetry

void tPin5BridgeBase::CheckAndRescue(void)
{
    // not needed for full duplex
}

void tPin5BridgeBase::uart_poll(void)
{
    // polling for now
    while (uart_rx_available() && state != STATE_TRANSMIT_START) { // read at most 1 message
        parse_nextchar(uart_getc());
    }

    // Send telemetry after every received message
    if (state == STATE_TRANSMIT_START) { // time to send telemetry
        transmit_start();
        state = STATE_IDLE;
    }
}


#endif // ESP_JRPIN5_INTERFACE_H
