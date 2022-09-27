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

static inline bool connected(void);

#define RADIO_LINK_SYSTEM_ID        51 // SiK uses 51, 68
#define GCS_SYSTEM_ID               255 // default of MissionPlanner, QGC

#define MAVLINK_BUF_SIZE            300 // needs to be larger than max mavlink frame size = 286 bytes


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
    void generate_radio_status(void);
    void generate_rc_channels_override(void);
    //void generate_rc_channels(void);
    void generate_radio_rc_channels(void);
    void generate_radio_link_stats(void);
    void generate_radio_link_flow_control(void);
    void send_msg_serial_out(void);

    uint8_t _calc_txbuf(void);

    fmav_status_t status_link_in;
    fmav_result_t result_link_in;
    uint8_t buf_link_in[MAVLINK_BUF_SIZE]; // buffer for link in parser
    fmav_status_t status_serial_out;
    fmav_message_t msg_serial_out;

    uint8_t _buf[MAVLINK_BUF_SIZE]; // working buffer

    // to inject RADIO_STATUS or RADIO_LINK_FLOW_CONTROL
    bool inject_radio_status;
    uint32_t radio_status_tlast_ms;

    uint32_t bytes_serial_in;

    // to inject RC_CHANNELS_OVERRIDE or RADIO_RC_CHANNELS & RADIO_LINK_STATS
    bool inject_rc_channels;
    uint16_t rc_chan[16]; // holds the rc data in MAVLink format
    int16_t rc_chan_13b[16]; // holds the rc data in MAVLink RADIO_RC_CHANNELS format
    bool rc_failsafe;
    bool inject_radio_link_stats;
};


void MavlinkBase::Init(void)
{
    fmav_init();

    result_link_in = {0};
    status_link_in = {0};
    status_serial_out = {0};

    inject_radio_status = false;
    radio_status_tlast_ms = millis32() + 1000;

    bytes_serial_in = 0;

    inject_rc_channels = false;
    for (uint8_t i = 0; i < 16; i++) { rc_chan[i] = 0; rc_chan_13b[i] = 0; }
    rc_failsafe = false;
    inject_radio_link_stats = false;
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

    if (!connected()) {
        //Init();
        inject_radio_status = false;
        radio_status_tlast_ms = tnow_ms + 1000;
    }

    if (Setup.Rx.SerialLinkMode != SERIAL_LINK_MODE_MAVLINK) return;

    if (Setup.Rx.SendRadioStatus) {
        if ((tnow_ms - radio_status_tlast_ms) >= (1000 / Setup.Rx.SendRadioStatus)) {
            radio_status_tlast_ms = tnow_ms;
            if (connected()) inject_radio_status = true;
        }
    } else {
        radio_status_tlast_ms = tnow_ms;
    }

    // TODO: either the buffer must be guaranteed to be large, or we need to check filling

    if (inject_rc_channels) { // give it priority // && serial.tx_is_empty()) // check available size!?
        inject_rc_channels = false;
        switch (Setup.Rx.SendRcChannels) {
        case SEND_RC_CHANNELS_OVERRIDE:
            generate_rc_channels_override();
            send_msg_serial_out();
            break;
        case SEND_RC_CHANNELS_RCCHANNELS:
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
        switch (Setup.Rx.SendRcChannels) {
        case SEND_RC_CHANNELS_OVERRIDE:
            generate_radio_status();
            send_msg_serial_out();
            break;
        case SEND_RC_CHANNELS_RCCHANNELS:
            generate_radio_link_flow_control();
            send_msg_serial_out();
            break;
        }
    }
}


void MavlinkBase::putc(char c)
{
    if (fmav_parse_and_check_to_frame_buf(&result_link_in, buf_link_in, &status_link_in, c)) {
        fmav_frame_buf_to_msg(&msg_serial_out, &result_link_in, buf_link_in);

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


// see design_decissions.h for details
uint8_t MavlinkBase::_calc_txbuf(void)
{

    uint8_t txbuf = 100;

    if (Setup.Rx.RadioStatusMethod == RADIO_STATUS_METHOD_W_TXBUF) {
        // method C
        uint32_t rate_max = ((uint32_t)1000 * FRAME_RX_PAYLOAD_LEN) / Config.frame_rate_ms; // theoretical rate, bytes per sec
        // we need to account for RADIO_STATUS, RADIO_LINK_FLOW_CONTROL interval
        uint32_t rate_percentage = (bytes_serial_in * 100 * Setup.Rx.SendRadioStatus) / rate_max;
        if (rate_percentage > 80) {
            txbuf = 0; // +60 ms
        } else if (rate_percentage > 70) {
            txbuf = 30; // +20 ms
        } else if (rate_percentage < 45) {
            txbuf = 100; // -40 ms
        } else if (rate_percentage < 55) {
            txbuf = 91; // -20 ms
        } else {
            txbuf = 60; // no change
        }

        if (serial.bytes_available() > 512) txbuf = 0; // keep the buffer low !!

/*dbg.puts("\nM: ");
dbg.puts(u16toBCD_s(stats.GetTransmitBandwidthUsage()*41));dbg.puts(", ");
dbg.puts(u16toBCD_s(bytes_serial_in));dbg.puts(", ");
dbg.puts(u16toBCD_s(serial.bytes_available()));dbg.puts(", ");
dbg.puts(u8toBCD_s(rate_percentage));dbg.puts(", ");
dbg.puts(u8toBCD_s(txbuf));dbg.puts(", ");
if(txbuf<20) dbg.puts("+60 "); else
if(txbuf<40) dbg.puts("+20 "); else
if(txbuf>95) dbg.puts("-40 "); else
if(txbuf>90) dbg.puts("-20 "); else dbg.puts("+-0 ");*/
    }
    bytes_serial_in = 0; // reset, to restart rate measurement

    return txbuf;
}


// see design_decissions.h for details
void MavlinkBase::generate_radio_status(void)
{
uint8_t rssi, remrssi, txbuf, noise;

    rssi = rssi_i8_to_ap(stats.GetLastRxRssi());
    remrssi = rssi_i8_to_ap(stats.received_rssi);

    // we don't have a reasonable noise measurement, but can use this field to report on the snr
    // the snr can be positive and negative however, so we artificially set snr = 10 to zero
    int16_t snr = -stats.GetLastRxSnr() + 10;
    noise = (snr < 0) ? 0 : (snr > 127) ? 127 : snr;

    txbuf = _calc_txbuf();

#if 0
    txbuf = 100;
    if (Setup.Rx.RadioStatusMethod == RADIO_STATUS_METHOD_W_TXBUF) {
        // method C
        uint32_t rate_max = ((uint32_t)1000 * FRAME_RX_PAYLOAD_LEN) / Config.frame_rate_ms; // theoretical rate, bytes per sec
        // we need to account for RADIO_STATUS interval
        uint32_t rate_percentage = (bytes_serial_in * 100 * Setup.Rx.SendRadioStatus) / rate_max;
        if (rate_percentage > 80) {
            txbuf = 0; // +60 ms
        } else if (rate_percentage > 70) {
            txbuf = 30; // +20 ms
        } else if (rate_percentage < 45) {
            txbuf = 100; // -40 ms
        } else if (rate_percentage < 55) {
            txbuf = 91; // -20 ms
        } else {
            txbuf = 60; // no change
        }

        if (serial.bytes_available() > 512) txbuf = 0; // keep the buffer low !!

/*dbg.puts("\nM: ");
dbg.puts(u16toBCD_s(stats.GetTransmitBandwidthUsage()*41));dbg.puts(", ");
dbg.puts(u16toBCD_s(bytes_serial_in));dbg.puts(", ");
dbg.puts(u16toBCD_s(serial.bytes_available()));dbg.puts(", ");
dbg.puts(u8toBCD_s(rate_percentage));dbg.puts(", ");
dbg.puts(u8toBCD_s(txbuf));dbg.puts(", ");
if(txbuf<20) dbg.puts("+60 "); else
if(txbuf<40) dbg.puts("+20 "); else
if(txbuf>95) dbg.puts("-40 "); else
if(txbuf>90) dbg.puts("-20 "); else dbg.puts("+-0 ");*/
    }
    bytes_serial_in = 0; // reset, to restart rate measurement
#endif

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
        rx_rssi1 = rssi_i8_to_ap(stats.last_rx_rssi1);
        rx_rssi2 = rssi_i8_to_ap(stats.last_rx_rssi2);
    } else if (USE_ANTENNA2) {
        rx_rssi1 = UINT8_MAX;
        rx_rssi2 = rssi_i8_to_ap(stats.last_rx_rssi2);
    } else {
        rx_rssi1 = rssi_i8_to_ap(stats.last_rx_rssi1);
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
        stats.last_rx_snr1, // int8_t rx_snr1
        rx_rssi2, // uint8_t rx_rssi2
        stats.last_rx_snr2, // int8_t rx_snr2
        stats.last_rx_antenna, // uint8_t rx_receive_antenna
        stats.last_tx_antenna, // uint8_t rx_transmit_antenna

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
    uint8_t txbuf = _calc_txbuf();

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
