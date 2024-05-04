//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// IN
//********************************************************

#include <string.h>
#include "in.h"
#include "../Common/setup_types.h"
#include "../Common/protocols/sbus_protocol.h"


extern uint16_t micros16(void);


typedef enum {
    IN_STATE_IDLE = 0,
    IN_STATE_RECEIVING,
} IN_STATE_ENUM;


void InBase::Init(bool enable_flag)
{
    enabled = enable_flag;

    config = UINT8_MAX;
    initialized = false;

    tlast_us = 0;
    state = IN_STATE_IDLE;
}


void InBase::Configure(uint8_t new_config)
{
    if (!enabled) return;
    if (new_config == config) return;

    // first disable the previous setting
    switch (config) {
    case IN_CONFIG_SBUS:
        config_sbus(false);
        break;
    case IN_CONFIG_SBUS_INVERTED:
        config_sbus(false);
        break;
    }

    initialized = false;

    config = new_config;

    switch (config) {
    case IN_CONFIG_SBUS:
        initialized = config_sbus(true);
        break;
    case IN_CONFIG_SBUS_INVERTED:
        initialized = config_sbus(true);
        break;
    }
}


bool InBase::Update(tRcData* rc)
{
    if (!initialized) return false;

    switch (config) {
    case IN_CONFIG_SBUS:
    case IN_CONFIG_SBUS_INVERTED:
        return parse_sbus(rc);
    }

    return false;
}


//-------------------------------------------------------
// SBus
//-------------------------------------------------------

bool InBase::parse_sbus(tRcData* rc)
{
    uint16_t tnow_us = micros16();
    bool updated = false;

    while (available()) {
        char c = getc();

        if (state == IN_STATE_IDLE) { // scan for new frame
            if (c == SBUS_STX) { // new frame
                buf_pos = 0;
                state = IN_STATE_RECEIVING;
            }
        }

        if (state == IN_STATE_RECEIVING) {
            _buf[buf_pos] = c;
            buf_pos++;
            if (buf_pos >= SBUS_FRAME_SIZE) {
                get_sbus_data(rc);
                updated = true;
                state = IN_STATE_IDLE;
                break; // is this what we want, or shouldn't we catch all?
            }
        }

        tlast_us = tnow_us;
    }

    if (state == IN_STATE_RECEIVING) {
        if ((tnow_us - tlast_us) > 2500) state = IN_STATE_IDLE;
    }

    return updated;
}


void InBase::get_sbus_data(tRcData* rc)
{
tSBusChannelBuffer sbus_buf;

    memcpy(sbus_buf.c, &(_buf[1]), SBUS_CHANNELPACKET_SIZE);
    rc->ch[0] = rc_from_sbus(sbus_buf.ch0);
    rc->ch[1] = rc_from_sbus(sbus_buf.ch1);
    rc->ch[2] = rc_from_sbus(sbus_buf.ch2);
    rc->ch[3] = rc_from_sbus(sbus_buf.ch3);
    rc->ch[4] = rc_from_sbus(sbus_buf.ch4);
    rc->ch[5] = rc_from_sbus(sbus_buf.ch5);
    rc->ch[6] = rc_from_sbus(sbus_buf.ch6);
    rc->ch[7] = rc_from_sbus(sbus_buf.ch7);
    rc->ch[8] = rc_from_sbus(sbus_buf.ch8);
    rc->ch[9] = rc_from_sbus(sbus_buf.ch9);
    rc->ch[10] = rc_from_sbus(sbus_buf.ch10);
    rc->ch[11] = rc_from_sbus(sbus_buf.ch11);
    rc->ch[12] = rc_from_sbus(sbus_buf.ch12);
    rc->ch[13] = rc_from_sbus(sbus_buf.ch13);
    rc->ch[14] = rc_from_sbus(sbus_buf.ch14);
    rc->ch[15] = rc_from_sbus(sbus_buf.ch15);

    rc->ch[16] = 1024;
    rc->ch[17] = 1024;
}



//-------------------------------------------------------
// FPort
//-------------------------------------------------------

// FPort, https://github.com/betaflight/betaflight/files/1491056/F.Port.protocol.betaFlight.V2.1.2017.11.21.pdf
// FPort frame: 0x7E , len = 0x19, type = 0x0,  22 bytes channel data , flags byte, rssi byte, crc byte, 0x7E
// 115200 bps, 8N1
// byte stuffing: 0x7E -> 0x7D,0x5E, 0x7D -> 0x7D,0x5D


//-------------------------------------------------------
// IBus
//-------------------------------------------------------

//IBUS frame: 0x20, 0x40, 28 bytes for 14 channels, 2 byte checksum
// 115200 bps, 8N1
// center = 0x05DC. My transmitter sends values between 0x3E8 and 0x7D0


//-------------------------------------------------------
// SUMD
//-------------------------------------------------------

//SUMD, https://www.deviationtx.com/media/kunena/attachments/98/HoTT-SUMD-Spec-REV01-12062012-pdf.pdf
// 115200 bps, 8N1
// 0xA8, 0x01 or 0x00 or 0x81, len byte, u16 x channels bytes, 2 crc byte, telem, crc8



