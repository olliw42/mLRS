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


#ifdef USE_FEATURE_MAVLINKX
#include "../Common/libs/fifo.h"
#include "../Common/thirdparty/mspx.h"


extern volatile uint32_t millis32(void);
static inline bool connected(void);


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
    FifoBase<char,2*512> fifo_link_out; // needs to be at least ??

    // to inject MSP_SET_RAW_RC
    bool inject_rc_channels;
    uint16_t rc_chan[16]; // holds the rc data in MSP format

    // to inject MSP requests if there are no requests from a gcs
    uint32_t tick_tlast_ms;

    #define MSP_TELM_COUNT  6

    const uint16_t telm_function[MSP_TELM_COUNT] = {
        MSP_INAV_STATUS,
        MSP_ATTITUDE,
        MSP_INAV_ANALOG,
        MSP_RAW_GPS,
        MSP_ALTITUDE,
        MSP_BOXNAMES, // MSP_BOXIDS, seems to hold incorrect flags ??
    };

    typedef struct {
        uint8_t rate;
        uint8_t cnt;
        uint32_t tlast_ms; // time of last request received form a gcs
    } tMspTelm;
    tMspTelm telm[MSP_TELM_COUNT];

    const uint8_t telm_freq[MSP_TELM_COUNT] = {
        2,  // 2 Hz = 5*100 ms, MSP_INAV_STATUS
        5,  // 5 Hz = 2*100 ms, MSP_ATTITUDE
        1,  // 1 Hz = 10*100 ms, MSP_INAV_ANALOG
        2,  // 2 Hz = 5*100 ms, MSP_RAW_GPS
        2,  // 2 Hz = 5*100 ms, MSP_ALTITUDE
        1,
    };

    bool boxarray_available;
    uint16_t boxarray_crc;
    uint8_t boxnames_force_send;

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
        telm[n].rate = (telm_freq[n] > 0) ? 10 / telm_freq[n] : 0; // 0 = off, do not send
        telm[n].cnt = 0;
        telm[n].tlast_ms = 0;
    }
    boxarray_available = false;
    boxarray_crc = 0;
    boxnames_force_send = 0;
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
    }

    // parse serial in -> link out
    if (fifo_link_out.HasSpace(MSP_FRAME_LEN_MAX + 16)) { // we have space for a full MSP message, so can safely parse
        while (serial.available()) {
            char c = serial.getc();
            if (msp_parse_to_msg(&msp_msg_ser_in, &status_ser_in, c)) {
#ifdef USE_MSPX
                uint16_t len = msp_msg_to_frame_bufX(_buf, &msp_msg_ser_in); // converting to mspX !!
#else
                uint16_t len = msp_msg_to_frame_buf(_buf, &msp_msg_ser_in);
#endif

                //fifo_link_out.PutBuf(_buf, len);

                if (msp_msg_ser_in.type == MSP_TYPE_RESPONSE) { // this is a response from the FC
                    if (msp_msg_ser_in.function == MSP_INAV_STATUS && boxarray_available) {
                        // send the original message
                        fifo_link_out.PutBuf(_buf, len);

                        // send out our home-brewed SMP message in addition
                        uint32_t flight_mode = 0;
                        tMspInavStatus* payload = (tMspInavStatus*)(msp_msg_ser_in.payload);
                        uint8_t* boxflags = payload->msp_box_mode_flags;
                        for (uint8_t n = 0; n < MSP_BOXARRAY_COUNT; n++) {
                            if (boxarray[n].boxModeFlag == 255) continue; // is empty
                            if (boxflags[boxarray[n].boxModeFlag / 8] & (1 << (boxarray[n].boxModeFlag % 8))) {
                                flight_mode |= ((uint32_t)1 << n);
                            }
                        }
                        len = msp_generate_frame_bufX(_buf, MSP_TYPE_RESPONSE, MSPX_STATUS, (uint8_t*)(&flight_mode), 4);
                        fifo_link_out.PutBuf(_buf, len);
                        len = 0; // mark as handled
                    }
                    if (msp_msg_ser_in.function == MSP_BOXNAMES) {
                        if (boxnames_force_send) {
                            boxnames_force_send--;
                            fifo_link_out.PutBuf(_buf, len);
                        }

                        uint16_t crc16 = fmav_crc_calculate(msp_msg_ser_in.payload, msp_msg_ser_in.len);
                        if (!boxarray_available || boxarray_crc != crc16) {
                            char s[48];
                            uint8_t pos = 0;
                            uint8_t box = 0;
                            for (uint16_t i = 0; i < msp_msg_ser_in.len; i++) {
                                char c = msp_msg_ser_in.payload[i];
                                if (c != ';') {
                                    s[pos++] = c;
                                    if (pos >= 32) pos = 0;
                                } else {
                                    s[pos++] = '\0';
                                    for (uint8_t n = 0; n < MSP_BOXARRAY_COUNT; n++) {
                                        if (boxarray[n].boxModeFlag != 255) continue; // has been set already
                                        if (!strcmp(s, boxarray[n].boxName)) {
                                            boxarray[n].boxModeFlag = box;
                                            break; // found, no need to look further
                                        }
                                    }
                                    pos = 0;
                                    box++;
                                }
                            }
                            boxarray_available = true;
                            boxarray_crc = crc16;
                        }
                        len = 0; // mark as handled
                    }
                }

                if (len) fifo_link_out.PutBuf(_buf, len);
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
            if (telm[n].rate == 0) continue;
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
#ifdef USE_MSPX
    if (msp_parseX_to_msg(&msp_msg_link_in, &status_link_in, c)) { // converting from mspX
#else
    if (msp_parse_to_msg(&msp_msg_link_in, &status_link_in, c)) {
#endif
        uint16_t len = msp_msg_to_frame_buf(_buf, &msp_msg_link_in);
        serial.putbuf(_buf, len);

        if (msp_msg_link_in.type == MSP_TYPE_REQUEST) { // this is a request from a gcs
            for (uint8_t n = 0; n < MSP_TELM_COUNT; n++) {
                if (msp_msg_link_in.function == telm_function[n]) { // to indicate we got this request from a gcs
                    telm[n].tlast_ms = millis32();
                }
            }
            if (msp_msg_link_in.function == MSP_BOXNAMES) {
                boxnames_force_send = 3;
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
