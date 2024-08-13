//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// MSP Interface RX Side
//*******************************************************
#ifndef MSP_INTERFACE_RX_H
#define MSP_INTERFACE_RX_H
#pragma once


// TODO: share fixed buffers with mavlink interface


#ifdef USE_FEATURE_MAVLINKX
#include "../Common/libs/fifo.h"
#include "../Common/protocols/msp_protocol.h"
#include "../Common/thirdparty/mspx.h"


extern volatile uint32_t millis32(void);
extern bool connected(void);


#define MSP_BUF_SIZE  (MSP_FRAME_LEN_MAX + 16) // needs to be larger than max supported msp frame size


class tRxMsp
{
  public:
    void Init(void);
    void Do(void);
    void SendRcData(tRcData* rc_out, bool frame_missed, bool failsafe);
    void FrameLost(void);

    void putc(char c);
    bool available(void);
    uint8_t getc(void);
    void flush(void);

  private:

    // fields for link in -> parser -> serial out
    msp_status_t status_link_in;
    msp_message_t msp_msg_link_in;

    // fields for serial in -> parser -> link out
    msp_status_t status_ser_in;
    msp_message_t msp_msg_ser_in;
    tFifo<char,2*512> fifo_link_out; // needs to be at least ??

    // to inject MSP_SET_RAW_RC
    bool inject_rc_channels;
    uint16_t rc_chan[16]; // holds the rc data in MSP format

    // to inject MSP requests if there are no requests from a gcs
    uint32_t tick_tlast_ms;

    #define MSP_TELM_COUNT  6
    #define MSP_TELM_BOXNAMES_ID  5

    const uint16_t telm_function[MSP_TELM_COUNT] = {
        MSP2_INAV_STATUS,
        MSP_ATTITUDE,
        MSP2_INAV_ANALOG,
        MSP_RAW_GPS,
        MSP_ALTITUDE,
        MSP_BOXNAMES, // MSP_BOXIDS, seems to hold incorrect flags ??
    };

    const uint8_t telm_freq[MSP_TELM_COUNT] = {
        2,  // 2 Hz = 5*100 ms, MSP_INAV_STATUS
        5,  // 5 Hz = 2*100 ms, MSP_ATTITUDE
        1,  // 1 Hz = 10*100 ms, MSP_INAV_ANALOG
        2,  // 2 Hz = 5*100 ms, MSP_RAW_GPS
        2,  // 2 Hz = 5*100 ms, MSP_ALTITUDE
        1,  // this is set to zero once it is gotten once, disables request
    };

    typedef struct {
        uint8_t rate;
        uint8_t cnt;
        uint32_t tlast_ms; // time of last request received from a gcs
    } tMspTelm;
    tMspTelm telm[MSP_TELM_COUNT];

    void telm_set_default_rate(uint8_t n) { telm[n].rate = (telm_freq[n] > 0) ? 10 / telm_freq[n] : 0; } // 0 = off, do not send

    uint8_t inav_flight_modes_box_mode_flags[INAV_FLIGHT_MODES_COUNT]; // store info from MSP_BOXNAMES

    // miscellaneous
    uint8_t _buf[MSP_BUF_SIZE]; // temporary working buffer, to not burden stack
};


void tRxMsp::Init(void)
{
    msp_init();

    status_link_in = {};
    status_ser_in = {};
    fifo_link_out.Init();

    inject_rc_channels = false;

    tick_tlast_ms = 0;
    for (uint8_t n = 0; n < MSP_TELM_COUNT; n ++) {
        telm_set_default_rate(n); // telm[n].rate = (telm_freq[n] > 0) ? 10 / telm_freq[n] : 0; // 0 = off, do not send
        telm[n].cnt = 0;
        telm[n].tlast_ms = 0;
    }

    memset(inav_flight_modes_box_mode_flags, INAV_FLIGHT_MODES_COUNT, 255); // 255 = is empty
}


void tRxMsp::SendRcData(tRcData* rc_out, bool frame_missed, bool failsafe)
{
    if (Setup.Rx.SendRcChannels == SEND_RC_CHANNELS_OFF) return;

    uint8_t failsafe_mode = Setup.Rx.FailsafeMode;

    if (failsafe) {
        switch (failsafe_mode) {
        case FAILSAFE_MODE_NO_SIGNAL:
            // do not output anything, so jump out
            return;
        case FAILSAFE_MODE_CH1CH4_CENTER:
            // in this mode do not report bad signal
            break;
        }
    }

    for (uint8_t i = 0; i < 16; i++) {
        rc_chan[i] = rc_to_mavlink(rc_out->ch[i]);
    }

    inject_rc_channels = true;
}


void tRxMsp::Do(void)
{
    if (!connected()) {
        fifo_link_out.Flush();
        telm_set_default_rate(MSP_TELM_BOXNAMES_ID);
    }

    if (!SERIAL_LINK_MODE_IS_MSP(Setup.Rx.SerialLinkMode)) return;

    // parse serial in -> link out
    if (fifo_link_out.HasSpace(MSP_FRAME_LEN_MAX + 16)) { // we have space for a full MSP message, so can safely parse
        while (serial.available()) {
            char c = serial.getc();
            if (msp_parse_to_msg(&msp_msg_ser_in, &status_ser_in, c)) {
                bool send = true;

                if (msp_msg_ser_in.type == MSP_TYPE_RESPONSE) { // this is a response from the FC
                    if (msp_msg_ser_in.function == MSP2_INAV_STATUS && telm[MSP_TELM_BOXNAMES_ID].rate == 0) {
                        // send out our home-brewed MSPX_STATUS message in addition
                        // is being send before original message
                        uint32_t flight_mode = 0;
                        uint8_t* boxflags = ((tMspInavStatus*)(msp_msg_ser_in.payload))->msp_box_mode_flags;
                        for (uint8_t n = 0; n < INAV_FLIGHT_MODES_COUNT; n++) {
                            if (inav_flight_modes_box_mode_flags[n] == 255) continue; // is empty
                            if (boxflags[inav_flight_modes_box_mode_flags[n] / 8] & (1 << (inav_flight_modes_box_mode_flags[n] % 8))) {
                                flight_mode |= ((uint32_t)1 << n);
                            }
                        }
                        uint16_t len = msp_generate_frame_bufX(_buf, MSP_TYPE_RESPONSE, MSPX_STATUS, (uint8_t*)(&flight_mode), 4);
                        fifo_link_out.PutBuf(_buf, len);
                    }
                    if (msp_msg_ser_in.function == MSP_BOXNAMES) {
                        // compress, and also get inav_flight_modes_box_mode_flags[]
                        uint8_t new_payload[512]; // MSP_BOXNAMES is 340 bytes in INAV 7.1
                        uint16_t new_len = 0;
                        mspX_boxnames_payload_compress(
                            new_payload, &new_len, msp_msg_ser_in.payload, msp_msg_ser_in.len,
                            inav_flight_modes_box_mode_flags);

                        telm[MSP_TELM_BOXNAMES_ID].rate = 0; // disable MSP_BOXNAMES requesting

                        uint16_t len = msp_generate_frame_bufX(_buf, MSP_TYPE_RESPONSE, MSP_BOXNAMES, new_payload, new_len);
                        fifo_link_out.PutBuf(_buf, len);

                        send = false; // mark as handled
                    }
                }

                if (send) {
                    uint16_t len = msp_msg_to_frame_bufX(_buf, &msp_msg_ser_in); // converting to mspX
                    fifo_link_out.PutBuf(_buf, len);
                }

/*
dbg.puts("\n");
dbg.putc(msp_msg_ser_in.type);
char s[32]; msp_function_str_from_msg(s, &msp_msg_ser_in); dbg.puts(s);
dbg.puts(" ");
dbg.puts(u16toBCD_s(msp_msg_ser_in.len));
*/
            }

        }
    }

    // inject radio rc channels

    if (inject_rc_channels) { // give it priority // && serial.tx_is_empty()) // check available size!?
        inject_rc_channels = false;
        uint16_t len = msp_generate_frame_buf(_buf, MSP_TYPE_REQUEST, MSP_SET_RAW_RC, (uint8_t*)rc_chan, 32);
        serial.putbuf(_buf, len);
        return;
    }

    // inject msp requests for telemetry data as needed

    uint32_t tnow_ms = millis32();

    // generate 10 ms ticks
    bool ticked = false;
    if ((tnow_ms - tick_tlast_ms) >= 100) {
        tick_tlast_ms = tnow_ms;
        ticked = true;
    }

    // send scheduler
    if (ticked) {
//dbg.puts("\nSend ");
        for (uint8_t n = 0; n < MSP_TELM_COUNT; n++) {
            if (telm[n].rate == 0) continue; // disabled
            INCc(telm[n].cnt, telm[n].rate);
            if (!telm[n].cnt && (tnow_ms - telm[n].tlast_ms) >= 1500) { // we want to send and did not got a request recently
                uint16_t len = msp_generate_request_to_frame_buf(_buf, MSP_TYPE_REQUEST, telm_function[n]);
                serial.putbuf(_buf, len);
//dbg.puts(u16toHEX_s(telm_function[n]));dbg.puts(" ");
            }
        }
    }
}


void tRxMsp::FrameLost(void)
{
    msp_parse_reset(&status_link_in);
}


void tRxMsp::putc(char c)
{
    // parse link in -> serial out
    if (msp_parseX_to_msg(&msp_msg_link_in, &status_link_in, c)) { // converting from mspX
        uint16_t len = msp_msg_to_frame_buf(_buf, &msp_msg_link_in);
        serial.putbuf(_buf, len);

        if (msp_msg_link_in.type == MSP_TYPE_REQUEST) { // this is a request from a gcs
            for (uint8_t n = 0; n < MSP_TELM_COUNT; n++) {
                if (msp_msg_link_in.function == telm_function[n]) { // to indicate we got this request from a gcs
                    telm[n].tlast_ms = millis32();
                }
            }
        }
/*
dbg.puts("\n");
dbg.putc(msp_msg_link_in.type);
char s[32]; msp_function_str_from_msg(s, &msp_msg_link_in); dbg.puts(s);
dbg.puts(" ");
dbg.puts(u16toBCD_s(msp_msg_link_in.len));
dbg.puts(" ");
dbg.puts(u8toHEX_s(msp_msg_link_in.checksum));
uint8_t crc8 = crsf_crc8_update(0, &(_buf[3]), len - 4);
dbg.puts(" ");
dbg.puts(u8toHEX_s(crc8));
*/
    }
}


bool tRxMsp::available(void)
{
    return fifo_link_out.Available();
    //return serial.available();
}


uint8_t tRxMsp::getc(void)
{
    return fifo_link_out.Get();
    //return serial.getc();
}


void tRxMsp::flush(void)
{
    fifo_link_out.Flush();
    // serial is flushed by caller
}


#else //!USE_FEATURE_MAVLINKX

class tRxMsp
{
  public:
    void Init(void) {}
    void Do(void) {}
    void SendRcData(tRcData* rc_out, bool frame_missed, bool failsafe) {}
    void FrameLost(void) {}

    void putc(char c) {}
    bool available(void) { return false; }
    uint8_t getc(void) { return 0; }
    void flush(void) {}
};

#endif //USE_FEATURE_MAVLINKX


#endif // MSP_INTERFACE_RX_H
