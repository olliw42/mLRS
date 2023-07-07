//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Mavlink Interface RX Side
//*******************************************************
#ifndef MAVLINK_INTERFACE_RX_H
#define MAVLINK_INTERFACE_RX_H
#pragma once


#include "../Common/mavlink/fmav_extension.h"
#include "../Common/libs/filters.h"

static inline bool connected(void);

#define RADIO_LINK_SYSTEM_ID        51 // SiK uses 51, 68
#define GCS_SYSTEM_ID               255 // default of MissionPlanner, QGC

#define MAVLINK_BUF_SIZE            300 // needs to be larger than max mavlink frame size = 280 bytes

#define MAVLINK_OPT_FAKE_PARAMFTP   2 // 0: off, 1: always, 2: determined from mode & baudrate


class MavlinkBase
{
  public:
    void Init(void);
    void Do(void);
    void SendRcData(tRcData* rc_out, bool failsafe);

    void putc(char c);
    bool available(void);
    uint8_t getc(void);
    void flush(void);

  private:
    void send_msg_serial_out(void);
    void generate_radio_status(void);
    void generate_rc_channels_override(void);
    void generate_radio_rc_channels(void);
    void generate_radio_link_stats(void);
    void generate_radio_link_flow_control(void);

    bool handle_txbuf_ardupilot(uint32_t tnow_ms);
    bool handle_txbuf_method_b(uint32_t tnow_ms); // for PX4, aka "brad"

    // fields for link in -> serial out parser
    fmav_status_t status_link_in;
    fmav_result_t result_link_in;
    uint8_t buf_link_in[MAVLINK_BUF_SIZE]; // buffer for link in parser
    fmav_status_t status_serial_out;
    fmav_message_t msg_serial_out;

    // to inject RADIO_STATUS or RADIO_LINK_FLOW_CONTROL
    uint32_t radio_status_tlast_ms;
    uint32_t bytes_serial_in;
    uint8_t radio_status_txbuf;

    uint32_t bytes_serial_in_cnt;
    LPFilterRate bytes_serial_in_rate_filt;

    typedef enum {
        TXBUF_STATE_NORMAL = 0,
        TXBUF_STATE_BURST,
        TXBUF_STATE_BURST_HIGH,
        TXBUF_STATE_PX4_RECOVER, // for PX4, buffer draining, resume bulk download
    } TXBUF_STATE_ENUM;
    uint8_t txbuf_state;

    // to inject RC_CHANNELS_OVERRIDE or RADIO_RC_CHANNELS & RADIO_LINK_STATS
    bool inject_rc_channels;
    uint16_t rc_chan[16]; // holds the rc data in MAVLink format
    int16_t rc_chan_13b[16]; // holds the rc data in MAVLink RADIO_RC_CHANNELS format
    bool rc_failsafe;

    uint8_t _buf[MAVLINK_BUF_SIZE]; // temporary working buffer, to not burden stack
};


void MavlinkBase::Init(void)
{
    fmav_init();

    result_link_in = {};
    status_link_in = {};
    status_serial_out = {};

    radio_status_tlast_ms = millis32() + 1000;
    radio_status_txbuf = 0;
    txbuf_state = TXBUF_STATE_NORMAL;

    bytes_serial_in = 0;
    bytes_serial_in_cnt = 0;
    bytes_serial_in_rate_filt.Reset();

    inject_rc_channels = false;
    for (uint8_t i = 0; i < 16; i++) { rc_chan[i] = 0; rc_chan_13b[i] = 0; }
    rc_failsafe = false;
}


void MavlinkBase::SendRcData(tRcData* rc_out, bool failsafe)
{
    if (Setup.Rx.SendRcChannels == SEND_RC_CHANNELS_OFF) return;

    uint8_t failsafe_mode = Setup.Rx.FailsafeMode;

    if (failsafe) {
        switch (failsafe_mode) {
        case FAILSAFE_MODE_NO_SIGNAL:
            // we do not output anything, so jump out
            return;
        }
    }

    for (uint8_t i = 0; i < 16; i++) {
        rc_chan[i] = rc_to_mavlink(rc_out->ch[i]);
        rc_chan_13b[i] = rc_to_mavlink_13bcentered(rc_out->ch[i]);
    }
    rc_failsafe = failsafe;

    inject_rc_channels = true;
}


void MavlinkBase::Do(void)
{
    uint32_t tnow_ms = millis32();
    bool inject_radio_link_stats = false;
    bool inject_radio_status = false;

    if (!connected()) {
        //Init();
        radio_status_tlast_ms = tnow_ms + 1000;
    }

    if (Setup.Rx.SerialLinkMode != SERIAL_LINK_MODE_MAVLINK) return;

    if (Setup.Rx.SendRadioStatus && connected()) {
        // we currently know that if we determine inject_radio_status here it will be executed immediately
        switch (Setup.Rx.SendRadioStatus) {
        case RX_SEND_RADIO_STATUS_METHOD_ARDUPILOT_1:
            inject_radio_status = handle_txbuf_ardupilot(tnow_ms);
            break;
        case RX_SEND_RADIO_STATUS_METHOD_PX4:
            inject_radio_status = handle_txbuf_method_b(tnow_ms);
            break;
        }
    } else {
        radio_status_tlast_ms = tnow_ms;
        bytes_serial_in_rate_filt.Reset();
    }

    // TODO: either the buffer must be guaranteed to be large, or we need to check filling

    if (inject_rc_channels) { // give it priority // && serial.tx_is_empty()) // check available size!?
        inject_rc_channels = false;
        switch (Setup.Rx.SendRcChannels) {
        case SEND_RC_CHANNELS_RCCHANNELSOVERRIDE:
            generate_rc_channels_override();
            send_msg_serial_out();
            break;
        case SEND_RC_CHANNELS_RADIORCCHANNELS:
            generate_radio_rc_channels();
            send_msg_serial_out();
            inject_radio_link_stats = true;
            break;
        }
    }

    if (inject_radio_link_stats) { // check available size!?
        inject_radio_link_stats = false;
        generate_radio_link_stats();
        send_msg_serial_out();
    }

    if (inject_radio_status) { // check available size!?
        inject_radio_status = false;
        if (Setup.Rx.SendRcChannels == SEND_RC_CHANNELS_RADIORCCHANNELS) {
            generate_radio_link_flow_control();
        } else {
            generate_radio_status();
        }
        send_msg_serial_out();
    }
}


typedef enum {
    MAVFTP_OPCODE_OpenFileRO = 4,
} MAVFTPOPCODEENUM;


void MavlinkBase::putc(char c)
{
    // parse link in -> serial out
    if (fmav_parse_and_check_to_frame_buf(&result_link_in, buf_link_in, &status_link_in, c)) {
        fmav_frame_buf_to_msg(&msg_serial_out, &result_link_in, buf_link_in);

#if MAVLINK_OPT_FAKE_PARAMFTP > 0
#if MAVLINK_OPT_FAKE_PARAMFTP > 1
        bool force_param_list = true;
        switch (Config.Mode) {
        case MODE_50HZ: force_param_list = (Config.SerialBaudrate > 57600); break; // 57600 bps and lower is ok for mftp
        case MODE_31HZ: force_param_list = (Config.SerialBaudrate > 57600); break; // 57600 bps and lower is ok for mftp
        case MODE_19HZ: force_param_list = (Config.SerialBaudrate > 38400); break; // 38400 bps and lower is ok for mftp
        }
        if (force_param_list)
#endif
        // if it's a mavftp call to @PARAM/param.pck we fake the url
        // this will make ArduPilot to response with a NACK:FileNotFound
        // which will make MissionPlanner (any GCS?) to fallback to normal parameter upload
        if (msg_serial_out.msgid == FASTMAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL) {
            uint8_t target_component = msg_serial_out.payload[2];
            uint8_t opcode = msg_serial_out.payload[6];
            char* url = (char*)(msg_serial_out.payload + 15);
            if (((target_component == MAV_COMP_ID_AUTOPILOT1) || (target_component == MAV_COMP_ID_ALL)) &&
                (opcode == MAVFTP_OPCODE_OpenFileRO)) {
                if (!strncmp(url, "@PARAM/param.pck", 16)) {
                    url[1] = url[7] = url[13] = 'x'; // now fake it to "@xARAM/xaram.xck"
                    fmav_msg_recalculate_crc(&msg_serial_out); // we need to recalculate CRC
                }
            }
        }
#endif

        send_msg_serial_out();
    }
}


bool MavlinkBase::available(void)
{
    return serial.available();
}


uint8_t MavlinkBase::getc(void)
{
    bytes_serial_in++;
    bytes_serial_in_cnt++;

    return serial.getc();
}


void MavlinkBase::flush(void)
{
    serial.flush();
}


void MavlinkBase::send_msg_serial_out(void)
{
    uint16_t len = fmav_msg_to_frame_buf(_buf, &msg_serial_out);

    serial.putbuf(_buf, len);
}


//-------------------------------------------------------
// Handle txbuf
//-------------------------------------------------------
// for the txbuf rate-based mechanism see design_decissions.h for details

bool MavlinkBase::handle_txbuf_ardupilot(uint32_t tnow_ms)
{
    // work out state
    bool inject_radio_status = false;
    uint8_t txbuf_state_last = txbuf_state; // to track changes in txbuf_state

    if ((tnow_ms - radio_status_tlast_ms) >= 1000) {
        //txbuf_state = TXBUF_STATE_NORMAL; // ??? or should we enter this case only if in normal?? does it matter ??
        radio_status_tlast_ms = tnow_ms;
        inject_radio_status = true;
    } else
    if ((tnow_ms - radio_status_tlast_ms) >= 100) { // limit to 10 Hz
        switch (txbuf_state) {
        case TXBUF_STATE_NORMAL:
            if (serial.bytes_available() > 1024) { // ups, suddenly lots of traffic
                txbuf_state = TXBUF_STATE_BURST;
                radio_status_tlast_ms = tnow_ms;
                inject_radio_status = true;
            }
            break;
        case TXBUF_STATE_BURST:
            if (serial.bytes_available() > 1024) { // it hasn't depleted, so raise alarm high
                txbuf_state = TXBUF_STATE_BURST_HIGH;
                radio_status_tlast_ms = tnow_ms;
                inject_radio_status = true;
            } else
            if (serial.bytes_available() < 384) { // quite empty, so we can go back and try normal
                txbuf_state = TXBUF_STATE_NORMAL;
                radio_status_tlast_ms = tnow_ms;
                inject_radio_status = true;
            }
            break;
        case TXBUF_STATE_BURST_HIGH:
            if (serial.bytes_available() < 1024) { // it has depleted, we can go back and try burst
                txbuf_state = TXBUF_STATE_BURST;
            }
            radio_status_tlast_ms = tnow_ms;
            inject_radio_status = true;
            break;
        }
    }

    if (!inject_radio_status) return false;

    // now calculate txbuf
    uint8_t txbuf = 100;

    // method C, with improvements
    // assumes 1 sec delta time
    uint32_t rate_max = ((uint32_t)1000 * FRAME_RX_PAYLOAD_LEN) / Config.frame_rate_ms; // theoretical rate, bytes per sec
    uint32_t rate_percentage = (bytes_serial_in * 100) / rate_max;

    // https://github.com/ArduPilot/ardupilot/blob/fa6441544639bd5dc84c3e6e3d2f7bfd2aecf96d/libraries/GCS_MAVLink/GCS_Common.cpp#L782-L801
    // aim at 75%..85% rate usage in steady state
    if (rate_percentage > 95) {
        txbuf = 0;                        // ArduPilot:  0-19  -> +60 ms,    PX4:  0-24  -> *0.8
    } else if (rate_percentage > 85) {
        txbuf = 30;                       // ArduPilot: 20-49  -> +20 ms,    PX4: 25-34  -> *0.975
    } else if (rate_percentage < 60) {
        txbuf = 100;                      // ArduPilot: 96-100 -> -40 ms,    PX4: 51-100 -> *1.025
    } else if (rate_percentage < 75) {
        txbuf = 91;                       // ArduPilot: 91-95  -> -20 ms,    PX4: 51-100 -> *1.025
    } else {
        txbuf = 50;                       // ArduPilot: 50-90  -> no change, PX4: 35-50  -> no change
    }

    if (txbuf_state == TXBUF_STATE_BURST_HIGH) {
        txbuf = 0; // try to slow down as much as possible
    } else
    if (txbuf_state == TXBUF_STATE_BURST) {
        txbuf = 50; // cut out PARAMS but don't change stream rate
    } else
    if ((txbuf_state == TXBUF_STATE_NORMAL) && (txbuf_state_last > TXBUF_STATE_NORMAL)) { // has changed back to NORMAL
        txbuf = 51; // allow PARAMS but don't change stream rate
    }
    txbuf_state_last = txbuf_state;

    // only for "educational" purposes currently
    bytes_serial_in_rate_filt.Update(tnow_ms, bytes_serial_in_cnt, 1000);
/*
static uint32_t t_last = 0;
uint32_t t = millis32(), dt = t - t_last; t_last = t;
dbg.puts("\nMa: ");
dbg.puts(u16toBCD_s(t));dbg.puts(" (");dbg.puts(u16toBCD_s(dt));dbg.puts("), ");
//dbg.puts(u16toBCD_s(stats.GetTransmitBandwidthUsage()*41));dbg.puts(", ");
dbg.puts(u16toBCD_s(bytes_serial_in));dbg.puts(" (");
dbg.puts(u16toBCD_s(bytes_serial_in_rate_filt.Get()));dbg.puts("), ");
dbg.puts(u16toBCD_s(serial.bytes_available()));dbg.puts(", ");
dbg.puts(u8toBCD_s((rate_percentage<256)?rate_percentage:255));dbg.puts(", ");
if(txbuf_state==2) dbg.puts("high, "); else
if(txbuf_state==1) dbg.puts("brst, "); else dbg.puts("norm, ");
dbg.puts(u8toBCD_s(txbuf));dbg.puts(", ");
if(txbuf<20) dbg.puts("+60 "); else
if(txbuf<50) dbg.puts("+20 "); else
if(txbuf>95) dbg.puts("-40 "); else
if(txbuf>90) dbg.puts("-20 "); else dbg.puts("0   ");
*/
    if ((txbuf_state == TXBUF_STATE_NORMAL) && (txbuf == 100)) {
        radio_status_tlast_ms -= 666; // do again in 1/3 sec
        bytes_serial_in = (bytes_serial_in * 2)/3; // approximate by 2/3 of what was received in the last 1 sec
    } else {
        bytes_serial_in = 0; // reset, to restart rate measurement
    }

    radio_status_txbuf = txbuf;
    return true;
}


// this method should be selected for PX4 and currently may be a useful alternative for Ardupilot
bool MavlinkBase::handle_txbuf_method_b(uint32_t tnow_ms)
{
    // work out state
    bool inject_radio_status = false;

    if ((tnow_ms - radio_status_tlast_ms) >= 1000) {
        radio_status_tlast_ms = tnow_ms;
        inject_radio_status = true;
    } else
    switch (txbuf_state) {
        case TXBUF_STATE_NORMAL:
            if (serial.bytes_available() > 800) { // oops, buffer filling
                txbuf_state = TXBUF_STATE_BURST;
                radio_status_tlast_ms = tnow_ms;
                inject_radio_status = true;
            }
            break;
        case TXBUF_STATE_BURST:
            if (serial.bytes_available() > 1400) { // Still growing, so raise alarm high
                txbuf_state = TXBUF_STATE_BURST_HIGH;
                radio_status_tlast_ms = tnow_ms;
                inject_radio_status = true;
            } else
            if (serial.bytes_available() < FRAME_RX_PAYLOAD_LEN*2) { // less than 2 radio messages remain, back to normal
                txbuf_state = TXBUF_STATE_PX4_RECOVER;
                radio_status_tlast_ms = tnow_ms;
                inject_radio_status = true;
            }
            break;
        case TXBUF_STATE_BURST_HIGH:
            if ((tnow_ms - radio_status_tlast_ms) >= 100) { // limit to 10 Hz
                if (serial.bytes_available() < 1400) { // it has stopped growing, we can go back and try burst
                    txbuf_state = TXBUF_STATE_BURST;
                }
		            radio_status_tlast_ms = tnow_ms;
		            inject_radio_status = true;
            }
            break;
        case TXBUF_STATE_PX4_RECOVER: // transient state so we don't need txbuf_state_last
            txbuf_state = TXBUF_STATE_NORMAL;
            break;
    }

    if (!inject_radio_status) return false;

    // now calculate txbuf
    uint8_t txbuf = 100;

    // method C, with improvements
    // assumes 1 sec delta time
    uint32_t rate_max = ((uint32_t)1000 * FRAME_RX_PAYLOAD_LEN) / Config.frame_rate_ms; // theoretical rate, bytes per sec
    uint32_t rate_percentage = (bytes_serial_in * 100) / rate_max;

    // https://github.com/ArduPilot/ardupilot/blob/fa6441544639bd5dc84c3e6e3d2f7bfd2aecf96d/libraries/GCS_MAVLink/GCS_Common.cpp#L782-L801
    // https://github.com/PX4/PX4-Autopilot/blob/fe80e7aa468a50bec6b035d0e8e4e37e516c84ff/src/modules/mavlink/mavlink_main.cpp#L1436-L1463
    // https://github.com/PX4/PX4-Autopilot/blob/fe80e7aa468a50bec6b035d0e8e4e37e516c84ff/src/modules/mavlink/mavlink_main.h#L690
    // PX4 is less bursty for normal streams, we might be able to sustain higher rates of 80%..90%
    switch (txbuf_state) {
        case TXBUF_STATE_NORMAL:
            if (rate_percentage > 95) {
                txbuf = 0;                        // ArduPilot:  0-19  -> +60 ms,    PX4:  0-24  -> *0.8
            } else if (rate_percentage > 85) {
                txbuf = 30;                       // ArduPilot: 20-49  -> +20 ms,    PX4: 25-34  -> *0.975
            } else if (rate_percentage < 60) {
                txbuf = 100;                      // ArduPilot: 96-100 -> -40 ms,    PX4: 51-100 -> *1.025
            } else if (rate_percentage < 75) {
                txbuf = 91;                       // ArduPilot: 91-95  -> -20 ms,    PX4: 51-100 -> *1.025
            } else {
                txbuf = 50;                       // ArduPilot: 50-90  -> no change, PX4: 35-50  -> no change
            }
            break;

        case TXBUF_STATE_BURST:
            txbuf = 33; // just enough to stop parameter flow
            break;

        case TXBUF_STATE_BURST_HIGH:
            txbuf = 0; // slow down as much as possible
            break;

        case TXBUF_STATE_PX4_RECOVER:
            txbuf = 93; // restart data flow
            break;
    }

    // only for "educational" purposes currently
    bytes_serial_in_rate_filt.Update(tnow_ms, bytes_serial_in_cnt, 1000);
#if 0 // Debug
static uint32_t t_last = 0;
uint32_t t = millis32(), dt = t - t_last; t_last = t;
dbg.puts("\nMp: ");
dbg.puts(u16toBCD_s(t));dbg.puts(" (");dbg.puts(u16toBCD_s(dt));dbg.puts("), ");
//dbg.puts(u16toBCD_s(stats.GetTransmitBandwidthUsage()*41));dbg.puts(", ");
dbg.puts(u16toBCD_s(bytes_serial_in));dbg.puts(", ");
dbg.puts(u16toBCD_s(serial.bytes_available()));dbg.puts(", ");
dbg.puts(u8toBCD_s((rate_percentage<256)?rate_percentage:255));dbg.puts(", ");
if(txbuf_state==1) dbg.puts("brst, "); else
if(txbuf_state==2) dbg.puts("high, "); else
if(txbuf_state==3) dbg.puts("recv, "); else dbg.puts("norm, ");
dbg.puts(u8toBCD_s(txbuf));dbg.puts(", ");
if(txbuf<25) dbg.puts("*0.8 "); else
if(txbuf<35) dbg.puts("*0.975 "); else
if(txbuf>50) dbg.puts("*1.025 "); else dbg.puts("*1 ");
#endif
    // increase rate faster after transient traffic since PX4 currently has no fast recovery. Could also try 100ms
    if ((txbuf_state == TXBUF_STATE_NORMAL) && (txbuf == 100)) {
        radio_status_tlast_ms -= 800; // do again in 200ms
        bytes_serial_in = (bytes_serial_in * 4)/5; // rolling average
    } else {
        bytes_serial_in = 0; // reset, to restart rate measurement
    }

    radio_status_txbuf = txbuf;
    return true;
}


//-------------------------------------------------------
// Handle Messages
//-------------------------------------------------------

// nothing yet


//-------------------------------------------------------
// Generate Messages
//-------------------------------------------------------

// see design_decissions.h for details
void MavlinkBase::generate_radio_status(void)
{
uint8_t rssi, remrssi, txbuf, noise;

    rssi = rssi_i8_to_ap(stats.GetLastRssi());
    remrssi = rssi_i8_to_ap(stats.received_rssi);

    // we don't have a reasonable noise measurement, but can use this field to report on the snr
    // the snr can be positive and negative however, so we artificially set snr = 10 to zero
    int16_t snr = -stats.GetLastSnr() + 10;
    noise = (snr < 0) ? 0 : (snr > 127) ? 127 : snr;

    txbuf = radio_status_txbuf;

    fmav_msg_radio_status_pack(
        &msg_serial_out,
        RADIO_LINK_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO, // SiK uses 51, 68
        rssi, remrssi, txbuf, noise, UINT8_MAX, 0, 0,
        //uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed,
        &status_serial_out);
}


void MavlinkBase::generate_rc_channels_override(void)
{
    fmav_msg_rc_channels_override_pack(
        &msg_serial_out,
        GCS_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO, // ArduPilot accepts it only if it comes from its GCS sysid
        0, 0, // we do not know the sysid, compid of the flight controller
        rc_chan[0], rc_chan[1], rc_chan[2], rc_chan[3], rc_chan[4], rc_chan[5], rc_chan[6], rc_chan[7],
        rc_chan[8], rc_chan[9], rc_chan[10], rc_chan[11], rc_chan[12], rc_chan[13], rc_chan[14], rc_chan[15],
        0, 0,
        // uint8_t target_system, uint8_t target_component,
        // uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw,
        // uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw,
        // uint16_t chan17_raw, uint16_t chan18_raw,
        &status_serial_out);
}


void MavlinkBase::generate_radio_rc_channels(void)
{
int16_t channels[24]; // FASTMAVLINK_MSG_RADIO_RC_CHANNELS_FIELD_CHANNELS_NUM = 24

    memcpy(channels, rc_chan_13b, 16*2);
    channels[16] = 0;
    channels[17] = 0;
    channels[18] = 0;
    channels[19] = 0;
    channels[20] = 0;
    channels[21] = 0;
    channels[22] = 0;
    channels[23] = 0;

    uint8_t flags = 0;
    if (rc_failsafe) flags |= RADIO_RC_CHANNELS_FLAGS_FAILSAFE;

    fmav_msg_radio_rc_channels_pack(
        &msg_serial_out,
        RADIO_LINK_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO,
        16, flags, channels,
        //uint8_t count, uint8_t flags, const int16_t* channels,
        &status_serial_out);
}


void MavlinkBase::generate_radio_link_stats(void)
{
uint8_t flags, rx_rssi1, rx_rssi2, tx_rssi;

    flags = 0; // rssi are in MAVLink units

    if (USE_ANTENNA1 && USE_ANTENNA2) {
        rx_rssi1 = rssi_i8_to_ap(stats.last_rssi1);
        rx_rssi2 = rssi_i8_to_ap(stats.last_rssi2);
    } else if (USE_ANTENNA2) {
        rx_rssi1 = UINT8_MAX;
        rx_rssi2 = rssi_i8_to_ap(stats.last_rssi2);
    } else {
        rx_rssi1 = rssi_i8_to_ap(stats.last_rssi1);
        rx_rssi2 = UINT8_MAX;
    }

    tx_rssi = rssi_i8_to_ap(stats.received_rssi);

    fmav_msg_radio_link_stats_pack(
        &msg_serial_out,
        RADIO_LINK_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO,

        flags,

        // rx stats
        rxstats.GetLQ(), // uint8_t rx_LQ
        rx_rssi1, // uint8_t rx_rssi1
        stats.last_snr1, // int8_t rx_snr1
        rx_rssi2, // uint8_t rx_rssi2
        stats.last_snr2, // int8_t rx_snr2
        stats.last_antenna, // uint8_t rx_receive_antenna
        stats.last_transmit_antenna, // uint8_t rx_transmit_antenna

        // tx stats
        stats.received_LQ, // uint8_t tx_LQ
        tx_rssi, //uint8_t tx_rssi1
        INT8_MAX, // int8_t tx_snr1
        UINT8_MAX, // uint8_t tx_rssi2
        INT8_MAX, // int8_t tx_snr2
        UINT8_MAX, //stats.received_antenna, we know that antenna but invalidate so that rssi1 is used // uint8_t tx_receive_antenna
        UINT8_MAX, //stats.received_transmit_antenna, we know that antenna but invalidate so that rssi1 is used // uint8_t tx_transmit_antenna

        //uint8_t flags,
        //uint8_t rx_LQ, uint8_t rx_rssi1, int8_t rx_snr1, uint8_t rx_rssi2, int8_t rx_snr2,
        //uint8_t rx_receive_antenna, uint8_t rx_transmit_antenna,
        //uint8_t tx_LQ, uint8_t tx_rssi1, int8_t tx_snr1, uint8_t tx_rssi2, int8_t tx_snr2,
        //uint8_t tx_receive_antenna, uint8_t tx_transmit_antenna,
        &status_serial_out);
}


void MavlinkBase::generate_radio_link_flow_control(void)
{
    uint8_t txbuf = radio_status_txbuf;

    fmav_msg_radio_link_flow_control_pack(
        &msg_serial_out,
        RADIO_LINK_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO,
        UINT16_MAX, UINT16_MAX,
        UINT8_MAX, UINT8_MAX,
        txbuf,
        //uint16_t tx_rate, uint16_t rx_rate, uint8_t tx_used_bandwidth, uint8_t rx_used_bandwidth, uint8_t txbuf,
        &status_serial_out);
}


#endif // MAVLINK_INTERFACE_RX_H
