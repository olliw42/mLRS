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

#define MSP_REBOOT_MAGIC  1234321


class tRxMsp
{
  public:
    void Init(void);
    void Do(void);
    void SendRcData(tRcData* const rc_out, bool frame_missed, bool failsafe);
    void FrameLost(void);

    void putc(char c);
    bool available(void);
    uint8_t getc(void);
    void flush(void);

  private:

    // fields for link in -> parser -> serial out
    msp_status_t status_link_in;
    msp_message_t msp_msg_link_in;
    void parse_link_in_serial_out(char c);

    // fields for serial in -> parser -> link out (or serial out in case of response to flight controller)
    msp_status_t status_ser_in;
    msp_message_t msp_msg_ser_in;
    tFifo<char,1024> fifo_link_out; // needs to be at least ??
    void parse_serial_in_link_out(void);

    // to inject MSP_SET_RAW_RC, MSP2_COMMON_SET_MSP_RC_LINK_STATS, MSP2_COMMON_SET_MSP_RC_INFO
    tMspSetRawRc rc_channels; // holds the rc data in MSP format
    bool inject_rc_channels;
    bool inject_rc_link_stats;
    bool inject_rc_info;
    bool rc_link_stats_disabled;
    bool rc_info_disabled;
    uint32_t rc_channels_tlast_ms;
    uint32_t rc_link_stats_tlast_ms;
    uint32_t rc_info_tlast_ms = 0;
    int8_t rc_info_power_dbm_last = 125;

    void send_rc_channels(void);
    void send_rc_link_stats(void);
    void send_rc_info(void);
    void send_request(uint16_t function);

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
        1,  // this is set to zero once MSP_BOXNAMES has been gotten once, disables request
    };

    typedef struct {
        uint8_t rate; // rate determined from telm_freq, or 0 = off, do not send
        uint8_t cnt;
        uint32_t tlast_ms; // time of last request received from a gcs
    } tMspTelm;
    tMspTelm telm[MSP_TELM_COUNT];

    void telm_set_default_rate(uint8_t n) { telm[n].rate = (telm_freq[n] > 0) ? 10 / telm_freq[n] : 0; }

    uint8_t inav_flight_modes_box_mode_flags[INAV_FLIGHT_MODES_COUNT]; // store info from MSP_BOXNAMES

    // to handle command MSP_REBOOT
    uint32_t reboot_activate_ms;

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
    inject_rc_link_stats = false;
    inject_rc_info = false;
    rc_link_stats_disabled = false;
    rc_info_disabled = false;
    rc_channels_tlast_ms = 0;
    rc_link_stats_tlast_ms = 0;
    rc_info_tlast_ms = 0;
    rc_info_power_dbm_last = 125;

    tick_tlast_ms = 0;
    for (uint8_t n = 0; n < MSP_TELM_COUNT; n ++) {
        telm_set_default_rate(n); // telm[n].rate = (telm_freq[n] > 0) ? 10 / telm_freq[n] : 0; // 0 = off, do not send
        telm[n].cnt = 0;
        telm[n].tlast_ms = 0;
    }

    memset(inav_flight_modes_box_mode_flags, 255, INAV_FLIGHT_MODES_COUNT); // 255 = is empty

    reboot_activate_ms = 0;
}


// rc_out is the rc data stored in out class
// so after handling of channel order and failsafes by out class.
void tRxMsp::SendRcData(tRcData* const rc_out, bool frame_missed, bool failsafe)
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
        rc_channels.rc[i] = rc_to_mavlink(rc_out->ch[i]);
    }

    inject_rc_channels = true;
    inject_rc_link_stats = !rc_link_stats_disabled; // true if not disabled
    inject_rc_info = !rc_info_disabled; // true if not disabled
}


void tRxMsp::Do(void)
{
    if (!connected()) {
        fifo_link_out.Flush();
        telm_set_default_rate(MSP_TELM_BOXNAMES_ID);
    }

    if (!SERIAL_LINK_MODE_IS_MSP(Setup.Rx.SerialLinkMode)) return;

    // parse serial in -> link out
    parse_serial_in_link_out();

    // inject radio rc channels

    if (inject_rc_channels) { // give it priority // && serial.tx_is_empty()) // check available size!?
        inject_rc_channels = false;
        send_rc_channels();
        return;
    }

    if (inject_rc_link_stats) {
        inject_rc_link_stats = false;
        send_rc_link_stats();
        return;
    }

    if (inject_rc_info) {
        inject_rc_info = false;
        send_rc_info();
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
            if (!telm[n].cnt && (tnow_ms - telm[n].tlast_ms) >= 3500) { // we want to send and did not got a request recently
                send_request(telm_function[n]);
//dbg.puts(u16toHEX_s(telm_function[n]));dbg.puts(" ");
            }
        }
    }

    // trigger bootloader after delay
    if (reboot_activate_ms && (tnow_ms - reboot_activate_ms) > 1000) {
        BootLoaderInit();
    }
}


void tRxMsp::FrameLost(void)
{
    msp_parse_reset(&status_link_in);
}


void tRxMsp::parse_serial_in_link_out(void)
{
    // parse serial in -> link out
    if (fifo_link_out.HasSpace(MSP_FRAME_LEN_MAX + 16)) { // we have space for a full MSP message, so can safely parse
        while (serial.available()) {
            char c = serial.getc();
            if (msp_parse_to_msg(&msp_msg_ser_in, &status_ser_in, c)) {
                bool send = true;

                if (msp_msg_ser_in.type == MSP_TYPE_RESPONSE) { // this is a response from the FC
                    if (msp_msg_ser_in.function == MSP2_INAV_STATUS && telm[MSP_TELM_BOXNAMES_ID].rate == 0) {
                        // when we get a MSP2_INAV_STATUS
                        // send out our home-brewed MSPX_STATUS message in addition
                        // do only after we have seen MSP_BOXNAMES
                        // is being send before original message
                        uint32_t flight_mode = 0;
                        uint8_t* boxflags = ((tMspInavStatus*)(msp_msg_ser_in.payload))->msp_box_mode_flags;
                        for (uint8_t n = 0; n < INAV_FLIGHT_MODES_COUNT; n++) {
                            if (inav_flight_modes_box_mode_flags[n] == 255) continue; // is empty
                            if (boxflags[inav_flight_modes_box_mode_flags[n] / 8] & (1 << (inav_flight_modes_box_mode_flags[n] % 8))) {
                                flight_mode |= ((uint32_t)1 << n);
                            }
                        }
                        uint16_t len = msp_generate_v2_frame_bufX(
                            _buf,
                            MSP_TYPE_RESPONSE,
                            MSP_FLAG_SOURCE_ID_RC_LINK,
                            MSPX_STATUS,
                            (uint8_t*)(&flight_mode),
                            sizeof(flight_mode));
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

                        uint16_t len = msp_generate_v2_frame_bufX(
                            _buf,
                            MSP_TYPE_RESPONSE,
                            MSP_FLAG_SOURCE_ID_RC_LINK,
                            MSP_BOXNAMES,
                            new_payload,
                            new_len);
                        fifo_link_out.PutBuf(_buf, len);

                        send = false; // mark as handled
                    }
                    if (msp_msg_ser_in.function == MSP2_RX_BIND && msp_msg_ser_in.magic2 == MSP_MAGIC_2_V2) {
                        // handle MSP2_RX_BIND, only if MSP V2
                        // this is a really a request from the FC but sent as a response
                        // payload is ignored
                        bind.StartBind();
                        send = false; // don't forward to ground
                    }
                }

                if (msp_msg_ser_in.type == MSP_TYPE_ERROR) { // this is an error response from the FC
                    switch (msp_msg_ser_in.function) {
                    case MSP2_COMMON_SET_MSP_RC_LINK_STATS: // FC doesn't support this message and complains
                        rc_link_stats_disabled = true;
                        send = false; // don't forward to ground
                        break;
                    case MSP2_COMMON_SET_MSP_RC_INFO: // FC doesn't support this message and complains
                        rc_info_disabled = true;
                        send = false; // don't forward to ground
                        break;
                    }
                }

                if (msp_msg_ser_in.type == MSP_TYPE_REQUEST) { // this is a request from the FC
                    if (msp_msg_ser_in.function == MSP_REBOOT && msp_msg_ser_in.magic2 == MSP_MAGIC_2_V2) {
                        // handle MSP_REBOOT, only if MSP V2
                        uint32_t magic = ((tMspReboot*)msp_msg_ser_in.payload)->magic;
                        if (serial.has_systemboot() && magic == MSP_REBOOT_MAGIC) {
                            reboot_activate_ms = millis32(); // set to non zero to enter system bootloader
                            // send response back to the FC
                            uint16_t len = msp_generate_v2_frame_buf(
                                _buf,
                                MSP_TYPE_RESPONSE,
                                MSP_FLAG_SOURCE_ID_RC_LINK,
                                MSP_REBOOT,
                                0, // dummy pointer
                                0);
                            serial.putbuf(_buf, len);
                        }
                        send = false; // don't forward to ground
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
}


void tRxMsp::parse_link_in_serial_out(char c)
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


void tRxMsp::putc(char c)
{
    // parse link in -> serial out
    parse_link_in_serial_out(c);
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


//-------------------------------------------------------
// Generate Messages
//-------------------------------------------------------

void tRxMsp::send_request(uint16_t function)
{
    uint16_t len = msp_generate_v2_request_to_frame_buf(
        _buf,
        MSP_TYPE_REQUEST,
        MSP_FLAG_SOURCE_ID_RC_LINK,
        function);

    serial.putbuf(_buf, len);
}


void tRxMsp::send_rc_channels(void)
{
    uint32_t tnow_ms = millis32();
    if ((tnow_ms - rc_channels_tlast_ms) < 19) return; // don't send too fast, MSP-RC is not for racing
    rc_channels_tlast_ms = tnow_ms;

    uint16_t len = msp_generate_v2_frame_buf(
        _buf,
        MSP_TYPE_REQUEST,
        MSP_FLAG_SOURCE_ID_RC_LINK | MSP_FLAG_NO_RESPONSE, // avoid response message from flight controller
        MSP_SET_RAW_RC,
        (uint8_t*)&rc_channels,
        MSP_SET_RAW_RC_LEN);

    serial.putbuf(_buf, len);
}


void tRxMsp::send_rc_link_stats(void)
{
tMspCommonSetMspRcLinkStats payload;

    uint32_t tnow_ms = millis32();
    if ((tnow_ms - rc_link_stats_tlast_ms) < 200) return; // really slow down, INAV FCs can be over-burdened
    rc_link_stats_tlast_ms = tnow_ms;

    payload.sublink_id = 0;
    payload.valid_link = 1;
    payload.uplink_rssi_perc = crsf_cvt_rssi_percent(stats.GetLastRssi(), sx.ReceiverSensitivity_dbm());
    payload.uplink_rssi = crsf_cvt_rssi_rx(stats.GetLastRssi());
    payload.downlink_link_quality = stats.received_LQ_serial;
    payload.uplink_link_quality = stats.GetLQ_rc();
    payload.uplink_snr = stats.GetLastSnr();

    uint16_t len = msp_generate_v2_frame_buf(
        _buf,
        MSP_TYPE_REQUEST,
        MSP_FLAG_SOURCE_ID_RC_LINK | MSP_FLAG_NO_RESPONSE, // avoid response message from flight controller
        MSP2_COMMON_SET_MSP_RC_LINK_STATS,
        (uint8_t*)&payload,
        MSP_COMMON_SET_MSP_RC_LINK_STATS_LEN);

    serial.putbuf(_buf, len);
}


void tRxMsp::send_rc_info(void)
{
tMspCommonSetMspRcInfo payload;

    uint32_t tnow_ms = millis32();
    int8_t power_dbm = sx.RfPower_dbm();
    if ((tnow_ms - rc_info_tlast_ms < 2500) && (power_dbm == rc_info_power_dbm_last)) return; // not yet time nor no need to send

    rc_info_tlast_ms = tnow_ms;
    rc_info_power_dbm_last = power_dbm;

    payload.sublink_id = 0;
    payload.uplink_tx_power = cvt_power(power_dbm); // WRONG, should be tx power, but to have something we send rx power
    payload.downlink_tx_power = payload.uplink_tx_power;

    frequency_band_str_to_strbuf(payload.band, Config.FrequencyBand, sizeof(payload.band));
    mode_str_to_strbuf(payload.mode, Config.Mode, sizeof(payload.mode));

    uint16_t len = msp_generate_v2_frame_buf(
        _buf,
        MSP_TYPE_REQUEST,
        MSP_FLAG_SOURCE_ID_RC_LINK | MSP_FLAG_NO_RESPONSE, // avoid response message from flight controller
        MSP2_COMMON_SET_MSP_RC_INFO,
        (uint8_t*)&payload,
        MSP_COMMON_SET_MSP_RC_INFO_LEN);

    serial.putbuf(_buf, len);
}


#else //!USE_FEATURE_MAVLINKX

class tRxMsp
{
  public:
    void Init(void) {}
    void Do(void) {}
    void SendRcData(tRcData* const rc_out, bool frame_missed, bool failsafe) {}
    void FrameLost(void) {}

    void putc(char c) {}
    bool available(void) { return false; }
    uint8_t getc(void) { return 0; }
    void flush(void) {}
};

#endif //USE_FEATURE_MAVLINKX


#endif // MSP_INTERFACE_RX_H
