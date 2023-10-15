//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Mavlink Interface TX Side
//*******************************************************
#ifndef MAVLINK_INTERFACE_TX_H
#define MAVLINK_INTERFACE_TX_H
#pragma once


#include "../Common/mavlink/fmav_extension.h"
#include "../Common/protocols/ardupilot_protocol.h"
#ifdef USE_FEATURE_MAVLINKX
#include "../Common/thirdparty/fmav_mavlinkx.h"
#include "../Common/libs/fifo.h"
#endif


extern volatile uint32_t millis32(void);
static inline bool connected_and_rx_setup_available(void);
extern tSerialBase* serialport;


#define RADIO_STATUS_SYSTEM_ID      51 // SiK uses 51, 68

#define MAVLINK_BUF_SIZE            300 // needs to be larger than max mavlink frame size = 286 bytes


class MavlinkBase
{
  public:
    void Init(void);
    void Do(void);
    uint8_t VehicleState(void);
    void FrameLost(void);

    void putc(char c);
    bool available(void);
    uint8_t getc(void);
    void flush(void);

  private:
    void send_msg_serial_out(void);
    void handle_msg_serial_out(void);
    void generate_radio_status(void);

    // fields for link in -> parser -> serial out
    fmav_status_t status_link_in;
    fmav_result_t result_link_in;
    uint8_t buf_link_in[MAVLINK_BUF_SIZE]; // buffer for link in parser
    fmav_status_t status_serial_out; // not needed, status_link_in could be used, but clearer so
    fmav_message_t msg_serial_out; // could be avoided by more efficient coding

    // fields for serial in -> parser -> link out
#ifdef USE_FEATURE_MAVLINKX
    fmav_status_t status_serial_in;
    fmav_result_t result_serial_in;
    uint8_t buf_serial_in[MAVLINK_BUF_SIZE]; // buffer for serial in parser
    fmav_message_t msg_link_out; // could be avoided by more efficient coding
    FifoBase<char,512> fifo_link_out; // needs to be at least 82 + 280
#endif

    // to inject RADIO_STATUS messages
    uint32_t radio_status_tlast_ms;

    uint8_t vehicle_sysid; // 0 indicates data is invalid
    uint8_t vehicle_is_armed;
    uint8_t vehicle_is_flying;
    uint8_t vehicle_type;
    uint8_t vehicle_flight_mode;

    uint8_t _buf[MAVLINK_BUF_SIZE]; // temporary working buffer, to not burden stack

    // relay
    uint32_t main_radio_stats_tlast_ms;
    void generate_main_radio_stats(void);
};


void MavlinkBase::Init(void)
{
    fmav_init();

    result_link_in = {};
    status_link_in = {};
    status_serial_out = {};

#ifdef USE_FEATURE_MAVLINKX
    fmavX_init();
    fmavX_config_compression((Config.Mode == MODE_19HZ) ? 1 : 0); // use compression only in 19 Hz mode

    result_serial_in = {};
    status_serial_in = {};
    fifo_link_out.Init();
#endif

    radio_status_tlast_ms = millis32() + 1000;

    vehicle_sysid = 0;
    vehicle_is_armed = UINT8_MAX;
    vehicle_is_flying = UINT8_MAX;
    vehicle_type = UINT8_MAX;
    vehicle_flight_mode = UINT8_MAX;

    main_radio_stats_tlast_ms = millis32() + 1050;
}


void MavlinkBase::Do(void)
{
    uint32_t tnow_ms = millis32();

    if (!connected_and_rx_setup_available()) {
        //Init();
        radio_status_tlast_ms = tnow_ms;
#ifdef USE_FEATURE_MAVLINKX
        fifo_link_out.Flush();
#endif
    }

    if (!SERIAL_LINK_MODE_IS_MAVLINK(Setup.Rx.SerialLinkMode)) return;

    // parse serial in -> link out
#ifdef USE_FEATURE_MAVLINKX
    if (fifo_link_out.HasSpace(290)) { // we have space for a full MAVLink message, so can safely parse
        while (serialport->available()) {
            char c = serialport->getc();
            if (fmav_parse_and_check_to_frame_buf(&result_serial_in, buf_serial_in, &status_serial_in, c)) {

                // TODO: this could be be done more efficiently by not going via msg_link_out
                // but by directly going buf_serial_in -> _buf

                fmav_frame_buf_to_msg(&msg_link_out, &result_serial_in, buf_serial_in);

                uint16_t len;
                if (Setup.Rx.SerialLinkMode == SERIAL_LINK_MODE_MAVLINK_X) {
                    len = fmavX_msg_to_frame_buf(_buf, &msg_link_out);
                } else {
                    len = fmav_msg_to_frame_buf(_buf, &msg_link_out);
                }

                fifo_link_out.PutBuf(_buf, len);
            }
        }
    }
#endif

    bool inject_radio_status = false;
    bool inject_main_radio_stats = false;

    if (Setup.Tx[Config.ConfigId].SendRadioStatus) {
        if ((tnow_ms - radio_status_tlast_ms) >= 1000) {
            radio_status_tlast_ms = tnow_ms;
            inject_radio_status = true;
        }
    } else {
        radio_status_tlast_ms = tnow_ms;
    }

    if (crsf.IsRelayMain()) {
        if ((tnow_ms - main_radio_stats_tlast_ms) >= 100) {
            main_radio_stats_tlast_ms = tnow_ms;
            inject_main_radio_stats = true;
        }
    } else {
        main_radio_stats_tlast_ms = tnow_ms;
    }

    if (crsf.IsRelayMain() || crsf.IsRelaySecondary()) { // this is a inner tx module
        inject_radio_status = false;
        inject_main_radio_stats = false;
    }

    if (inject_main_radio_stats) { // check available size!?
        inject_main_radio_stats = false;
        generate_main_radio_stats();
        send_msg_serial_out();
    }

    if (inject_radio_status) { // && serial.tx_is_empty()) { // check available size!?
        inject_radio_status = false;
        generate_radio_status();
        send_msg_serial_out();
    }
}


uint8_t MavlinkBase::VehicleState(void)
{
    if (vehicle_is_armed == UINT8_MAX) return UINT8_MAX;
    if (vehicle_is_armed == 1 && vehicle_is_flying == 1) return 2;
    return vehicle_is_armed;
}


void MavlinkBase::FrameLost(void)
{
#ifdef USE_FEATURE_MAVLINKX
    // reset parser link in -> serial out
    fmav_parse_reset(&status_link_in);
#endif
}


void MavlinkBase::putc(char c)
{
    // parse link in -> serial out
#ifdef USE_FEATURE_MAVLINKX
    uint8_t res;
    if (Setup.Rx.SerialLinkMode == SERIAL_LINK_MODE_MAVLINK_X) {
        res = fmavX_parse_and_check_to_frame_buf(&result_link_in, buf_link_in, &status_link_in, c);
    } else {
        res = fmav_parse_and_check_to_frame_buf(&result_link_in, buf_link_in, &status_link_in, c);
    }
    if (res) {
#else
    if (fmav_parse_and_check_to_frame_buf(&result_link_in, buf_link_in, &status_link_in, c)) {
#endif
        fmav_frame_buf_to_msg(&msg_serial_out, &result_link_in, buf_link_in);

        send_msg_serial_out();

        // allow crsf to capture it
        crsf.TelemetryHandleMavlinkMsg(&msg_serial_out);

        // we also want to capture it to extract some info
        handle_msg_serial_out();
    }
}


bool MavlinkBase::available(void)
{
    if (!serialport) return false; // should not happen

#ifdef USE_FEATURE_MAVLINKX
    return fifo_link_out.Available();
#else
    return serialport->available();
#endif
}


uint8_t MavlinkBase::getc(void)
{
    if (!serialport) return 0; // should not happen

#ifdef USE_FEATURE_MAVLINKX
    return fifo_link_out.Get();
#else
    return serialport->getc();
#endif
}


void MavlinkBase::flush(void)
{
    if (!serialport) return; // should not happen

#ifdef USE_FEATURE_MAVLINKX
    fifo_link_out.Flush();
#endif
    serialport->flush();
}


void MavlinkBase::send_msg_serial_out(void)
{
    if (!serialport) return; // should not happen

    uint16_t len = fmav_msg_to_frame_buf(_buf, &msg_serial_out);

    serialport->putbuf(_buf, len);
}


//-------------------------------------------------------
// Handle Messages
//-------------------------------------------------------

void MavlinkBase::handle_msg_serial_out(void)
{
    if ((msg_serial_out.sysid == RADIO_STATUS_SYSTEM_ID) &&
        (msg_serial_out.compid == MAV_COMP_ID_TELEMETRY_RADIO) &&
        (msg_serial_out.msgid == FASTMAVLINK_MSG_ID_MLRS_MAIN_RADIO_STATS)) {
        crsf.HandleMainRadioStatsMavlinkMsg(&msg_serial_out);
        return;
    }

    if ((msg_serial_out.msgid == FASTMAVLINK_MSG_ID_HEARTBEAT) && (msg_serial_out.compid == MAV_COMP_ID_AUTOPILOT1)) {
        fmav_heartbeat_t payload;
        fmav_msg_heartbeat_decode(&payload, &msg_serial_out);
        if (payload.autopilot != MAV_AUTOPILOT_INVALID) {
            // this is an autopilot
            vehicle_sysid = msg_serial_out.sysid;
            vehicle_is_armed = (payload.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? 1 : 0;

            // ArduPilot provides flight mode number in custom mode
            if (payload.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA) {
                vehicle_type = ap_vehicle_from_mavtype(payload.type);
                vehicle_flight_mode = payload.custom_mode;
            } else {
                vehicle_type = UINT8_MAX;
                vehicle_flight_mode = UINT8_MAX;
            }
        }
    }

    if (!vehicle_sysid) return;

    switch (msg_serial_out.msgid) {
    case FASTMAVLINK_MSG_ID_EXTENDED_SYS_STATE:{
        fmav_extended_sys_state_t payload;
        fmav_msg_extended_sys_state_decode(&payload, &msg_serial_out);
        vehicle_is_flying = (payload.landed_state == MAV_LANDED_STATE_IN_AIR) ? 1 : 0;
        }break;
    }
}


//-------------------------------------------------------
// Generate Messages
//-------------------------------------------------------

void MavlinkBase::generate_radio_status(void)
{
uint8_t rssi, remrssi, txbuf, noise;

    rssi = rssi_i8_to_ap(stats.GetLastRssi());
    remrssi = rssi_i8_to_ap(stats.received_rssi);

    // we don't have a reasonable noise measurement, but can use this field to report on the snr
    // the snr can be positive and negative however, so we artificially set snr = 10 to zero
    int16_t snr = -stats.GetLastSnr() + 10;
    noise = (snr < 0) ? 0 : (snr > 127) ? 127 : snr;

    // we do nothing, I'm not aware that any GCS respect this, and if so, it needs detailed investigation
    txbuf = 100;

    fmav_msg_radio_status_pack(
        &msg_serial_out,
        RADIO_STATUS_SYSTEM_ID, // sysid, SiK uses 51, 68
        MAV_COMP_ID_TELEMETRY_RADIO + (crsf.IsRelaySecondary() ? 1 : 0),
        rssi, remrssi, txbuf, noise, UINT8_MAX, 0, 0,
        //uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed,
        &status_serial_out);
}


void MavlinkBase::generate_main_radio_stats(void)
{
    fmav_msg_mlrs_main_radio_stats_pack(
        &msg_serial_out,
        RADIO_STATUS_SYSTEM_ID,
        MAV_COMP_ID_TELEMETRY_RADIO,

        crsf_cvt_rssi_tx(stats.received_rssi),
        stats.received_LQ,
        stats.received_antenna,
        crsf_cvt_mode(Config.Mode),
        crsf_cvt_power(sx.RfPower_dbm()),
        crsf_cvt_rssi_tx(stats.GetLastRssi()),
        txstats.GetLQ(),
        stats.GetLastSnr(),

        crsf_cvt_rssi_percent(stats.GetLastRssi(), sx.ReceiverSensitivity_dbm()),
        crsf_cvt_fps(Config.Mode),

        crsf_cvt_rssi_percent(stats.received_rssi, sx.ReceiverSensitivity_dbm()),
        sx.RfPower_dbm(),

        //uint8_t uplink_rssi1, uint8_t uplink_LQ, uint8_t active_antenna, uint8_t mode,
        //uint8_t uplink_transmit_power2, uint8_t downlink_rssi, uint8_t downlink_LQ, int8_t downlink_snr,
        //uint8_t uplink_rssi_percent, uint8_t uplink_fps,
        //uint8_t downlink_rssi_percent, uint8_t uplink_transmit_power,

        &status_serial_out);
}


#endif // MAVLINK_INTERFACE_TX_H
