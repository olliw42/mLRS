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

    // to inject RADIO_STATUS messages
    uint32_t radio_status_tlast_ms;

    uint8_t vehicle_sysid; // 0 indicates data is invalid
    uint8_t vehicle_is_armed;
    uint8_t vehicle_is_flying;
    uint8_t vehicle_type;
    uint8_t vehicle_flight_mode;

    uint8_t _buf[MAVLINK_BUF_SIZE]; // temporary working buffer, to not burden stack
};


void MavlinkBase::Init(void)
{
    fmav_init();

    result_link_in = {};
    status_link_in = {};
    status_serial_out = {};

    radio_status_tlast_ms = millis32() + 1000;

    vehicle_sysid = 0;
    vehicle_is_armed = UINT8_MAX;
    vehicle_is_flying = UINT8_MAX;
    vehicle_type = UINT8_MAX;
    vehicle_flight_mode = UINT8_MAX;
}


void MavlinkBase::Do(void)
{
    uint32_t tnow_ms = millis32();
    bool inject_radio_status = false;

    if (!connected_and_rx_setup_available()) {
        //Init();
        radio_status_tlast_ms = tnow_ms;
    }

    if (!SERIAL_LINK_MODE_IS_MAVLINK(Setup.Rx.SerialLinkMode)) return;

    if (Setup.Tx[Config.ConfigId].SendRadioStatus) {
        if ((tnow_ms - radio_status_tlast_ms) >= 1000) {
            radio_status_tlast_ms = tnow_ms;
            inject_radio_status = true;
        }
    } else {
        radio_status_tlast_ms = tnow_ms;
    }

    if (inject_radio_status) { // && serial.tx_is_empty()) {
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
}


void MavlinkBase::putc(char c)
{
    // parse link in -> serial out
    if (fmav_parse_and_check_to_frame_buf(&result_link_in, buf_link_in, &status_link_in, c)) {
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

    return serialport->available();
}


uint8_t MavlinkBase::getc(void)
{
    if (!serialport) return 0; // should not happen

    return serialport->getc();
}


void MavlinkBase::flush(void)
{
    if (!serialport) return; // should not happen

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
        MAV_COMP_ID_TELEMETRY_RADIO,
        rssi, remrssi, txbuf, noise, UINT8_MAX, 0, 0,
        //uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed,
        &status_serial_out);
}


#endif // MAVLINK_INTERFACE_TX_H
