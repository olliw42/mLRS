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
#define FASTMAVLINK_ROUTER_LINKS_MAX  4
#define FASTMAVLINK_ROUTER_COMPONENTS_MAX  12
#define FASTMAVLINK_ROUTER_LINK_PROPERTY_DEFAULT  FASTMAVLINK_ROUTER_LINK_PROPERTY_FLAG_ALWAYS_SEND_HEARTBEAT
#include "../Common/mavlink/out/lib/fastmavlink_router.h"
#endif


extern volatile uint32_t millis32(void);
static inline bool connected_and_rx_setup_available(void);


#define RADIO_STATUS_SYSTEM_ID      51 // SiK uses 51, 68

#define MAVLINK_BUF_SIZE            300 // needs to be larger than max mavlink frame size = 286 bytes


class MavlinkBase
{
  public:
    void Init(tSerialBase* _serial, tSerialBase* _mbridge, tSerialBase* _serial2);
    void Do(void);
    uint8_t VehicleState(void);
    void FrameLost(void);

    void putc(char c);
    bool available(void);
    uint8_t getc(void);
    void flush(void);

  private:
    void send_msg_fifo_link_out(fmav_result_t* result, uint8_t* buf_in);
    void handle_msg_serial_out(fmav_message_t* msg);
    void generate_radio_status(void);
    void send_msg_serial_out(void);

    tSerialBase* serialport;
    tSerialBase* serialport2;
    bool do_router(void) { return (serialport2 != nullptr); }

    // fields for link in -> parser -> serial out
    fmav_status_t status_link_in; // status for link in parser
    uint8_t buf_link_in[MAVLINK_BUF_SIZE]; // buffer for link in parser
    fmav_status_t status_serial_out; // status for serial out messages
    fmav_message_t msg_serial_out; // essentially a temporary working buffer, to not burden stack

    // fields for serial in -> parser -> link out
#ifdef USE_FEATURE_MAVLINKX
    fmav_status_t status_serialport_in; // status for serialport in parser
    uint8_t buf_serialport_in[MAVLINK_BUF_SIZE]; // buffer for serialport in parser
    fmav_status_t status_serialport2_in;
    uint8_t buf_serialport2_in[MAVLINK_BUF_SIZE];
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
};


void MavlinkBase::Init(tSerialBase* _serial, tSerialBase* _mbridge, tSerialBase* _serial2)
{
    // if ChannelsSource = MBRIDGE:
    //   SerialDestination = SERIAL or SERIAL2 => router with port = mbridge & port2 = serial/serial2
    //   SerialDestination = MBRDIGE           => no router, only port = mbridge (port2 = null)
    // => serialport2 != nullptr indicates that router is to be used
    switch (Setup.Tx[Config.ConfigId].SerialDestination) {
    case SERIAL_DESTINATION_SERIAL:
        serialport = _serial;
        serialport2 = (Setup.Tx[Config.ConfigId].ChannelsSource == CHANNEL_SOURCE_MBRIDGE) ? _mbridge : nullptr;
        break;
    case SERIAL_DESTINATION_SERIAL2:
        serialport = _serial2;
        serialport2 = (Setup.Tx[Config.ConfigId].ChannelsSource == CHANNEL_SOURCE_MBRIDGE) ? _mbridge : nullptr;
        break;
    case SERIAL_DESTINATION_MBRDIGE:
        serialport = _mbridge;
        serialport2 = nullptr;
        break;
    default:
        while (1) {} // must not happen
    }
    if (!serialport) while (1) {} // must not happen

    fmav_init();

    status_link_in = {};
    status_serial_out = {};

#ifdef USE_FEATURE_MAVLINKX
    fmavX_init();
    fmavX_config_compression((Config.Mode == MODE_19HZ) ? 1 : 0); // use compression only in 19 Hz mode

    status_serialport_in = {};
    status_serialport2_in = {};
    fifo_link_out.Init();

    fmav_router_init();
#endif

    radio_status_tlast_ms = millis32() + 1000;

    vehicle_sysid = 0;
    vehicle_is_armed = UINT8_MAX;
    vehicle_is_flying = UINT8_MAX;
    vehicle_type = UINT8_MAX;
    vehicle_flight_mode = UINT8_MAX;
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


void MavlinkBase::Do(void)
{
    uint32_t tnow_ms = millis32();
    bool inject_radio_status = false;

    if (!connected_and_rx_setup_available()) {
        //Init();
        radio_status_tlast_ms = tnow_ms;
#ifdef USE_FEATURE_MAVLINKX
        fifo_link_out.Flush();
#endif
    }

    if (!SERIAL_LINK_MODE_IS_MAVLINK(Setup.Rx.SerialLinkMode)) return;

    // parse serialport in -> link out
#ifdef USE_FEATURE_MAVLINKX
    fmav_result_t result;
if (!do_router()) {
    // without router
    if (fifo_link_out.HasSpace(290)) { // we have space for a full MAVLink message, so can safely parse
        while (serialport->available()) {
            char c = serialport->getc();
            fmav_parse_and_check_to_frame_buf(&result, buf_serialport_in, &status_serialport_in, c);
            if (result.res == FASTMAVLINK_PARSE_RESULT_OK) {
                send_msg_fifo_link_out(&result, buf_serialport_in); // requires RESULT_OK
            }
        }
    }
} else {
    // with router
    if (fifo_link_out.HasSpace(290)) { // we have space for a full MAVLink message, so can safely parse
        // link 0 = sx_serial
        // link 1 = serialport
        // link 2 = serialport2

        static uint8_t scheduled_serialport = 0;
        INCc(scheduled_serialport, 2);
        if ((scheduled_serialport == 0) && !serialport->available()) scheduled_serialport = 1;
        if ((scheduled_serialport == 1) && !serialport2->available()) scheduled_serialport = 0;

        while ((scheduled_serialport == 0) && serialport->available()) {
            char c = serialport->getc();
            if (fmav_parse_and_check_to_frame_buf(&result, buf_serialport_in, &status_serialport_in, c)) {
                fmav_router_handle_message(1, &result);
                if (fmav_router_send_to_link(1)) {} // WE DO NOT REFLECT, SO THIS MUST NEVER HAPPEN !!
                if (fmav_router_send_to_link(2)) {
                    serialport2->putbuf(buf_serialport_in, result.frame_len);
                }
                if (fmav_router_send_to_link(0) && (result.res == FASTMAVLINK_PARSE_RESULT_OK)) {
                    send_msg_fifo_link_out(&result, buf_serialport_in); // requires RESULT_OK
                }
                break; // do only one message per loop
            }
        }

        while ((scheduled_serialport == 1) && serialport2->available()) {
            char c = serialport2->getc();
            if (fmav_parse_and_check_to_frame_buf(&result, buf_serialport2_in, &status_serialport2_in, c)) {
                fmav_router_handle_message(2, &result);
                if (fmav_router_send_to_link(1)) {
                    serialport->putbuf(buf_serialport2_in, result.frame_len);
                }
                if (fmav_router_send_to_link(2)) {} // WE DO NOT REFLECT, SO THIS MUST NEVER HAPPEN !!
                if (fmav_router_send_to_link(0) && (result.res == FASTMAVLINK_PARSE_RESULT_OK)) {
                    send_msg_fifo_link_out(&result, buf_serialport2_in); // requires RESULT_OK
                }
                break; // do only one message per loop
            }
        }

    }
} // end if(do_router())
#endif

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


void MavlinkBase::putc(char c)
{
    // parse link in -> serial out
    fmav_result_t result;
#ifdef USE_FEATURE_MAVLINKX
    if (Setup.Rx.SerialLinkMode == SERIAL_LINK_MODE_MAVLINK_X) {
        fmavX_parse_and_check_to_frame_buf(&result, buf_link_in, &status_link_in, c);
    } else {
        fmav_parse_and_check_to_frame_buf(&result, buf_link_in, &status_link_in, c);
    }
    if (result.res == FASTMAVLINK_PARSE_RESULT_OK) {

if (!do_router()) {
        // without router
        serialport->putbuf(buf_link_in, result.frame_len);
} else {
        // with router
        fmav_router_handle_message(0, &result);
        if (fmav_router_send_to_link(1)) {
            serialport->putbuf(buf_link_in, result.frame_len);
        }
        if (fmav_router_send_to_link(2)) {
            serialport2->putbuf(buf_link_in, result.frame_len);
        }
        if (fmav_router_send_to_link(0)) {} // WE DO NOT REFLECT, SO THIS MUST NEVER HAPPEN !!
} // end if(do_router())

#else
    fmav_parse_and_check_to_frame_buf(&result, buf_link_in, &status_link_in, c);
    if (result.res == FASTMAVLINK_PARSE_RESULT_OK) {
        serialport->putbuf(buf_link_in, result.frame_len);
#endif

        // we misuse msg_serial_out as temporary message buffer

        fmav_frame_buf_to_msg(&msg_serial_out, &result, buf_link_in); // requires RESULT_OK

        // allow crsf to capture it
        crsf.TelemetryHandleMavlinkMsg(&msg_serial_out);

        // we also want to capture it to extract some info
        handle_msg_serial_out(&msg_serial_out);
    }
}


void MavlinkBase::send_msg_fifo_link_out(fmav_result_t* result, uint8_t* buf_in)
{
#ifdef USE_FEATURE_MAVLINKX
    // we misuse msg_serial_out as temporary message buffer

    fmav_frame_buf_to_msg(&msg_serial_out, result, buf_in); // requires RESULT_OK

    uint16_t len;
    if (Setup.Rx.SerialLinkMode == SERIAL_LINK_MODE_MAVLINK_X) {
        len = fmavX_msg_to_frame_buf(_buf, &msg_serial_out);
    } else {
        len = fmav_msg_to_frame_buf(_buf, &msg_serial_out);
    }

    fifo_link_out.PutBuf(_buf, len);
#endif
}


void MavlinkBase::send_msg_serial_out(void)
{
    // TODO: this could be be done more efficiently by not going via msg_serial_out
    // but by generating directly into _buf

    uint16_t len = fmav_msg_to_frame_buf(_buf, &msg_serial_out);

    serialport->putbuf(_buf, len);

#ifdef USE_FEATURE_MAVLINKX
    if (serialport2) serialport2->putbuf(_buf, len);
#endif
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
    if (serialport2) serialport2->flush();
#endif
    serialport->flush();
}


//-------------------------------------------------------
// Handle Messages
//-------------------------------------------------------

void MavlinkBase::handle_msg_serial_out(fmav_message_t* msg)
{
    if ((msg->msgid == FASTMAVLINK_MSG_ID_HEARTBEAT) && (msg->compid == MAV_COMP_ID_AUTOPILOT1)) {
        fmav_heartbeat_t payload;
        fmav_msg_heartbeat_decode(&payload, msg);
        if (payload.autopilot != MAV_AUTOPILOT_INVALID) {
            // this is an autopilot
            vehicle_sysid = msg->sysid;
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

    switch (msg->msgid) {
    case FASTMAVLINK_MSG_ID_EXTENDED_SYS_STATE:{
        fmav_extended_sys_state_t payload;
        fmav_msg_extended_sys_state_decode(&payload, msg);
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

/*
sx_serial:

  serial   o --\
  serial2  o -- o <====> o  sx_serial
  mbridge  o --/
  mavlink  o --/

    ---> sx_serial.available() ---> transmit ~~~>
         sx_serial.putc()

    <--- sx_serial.getc()      <--- receive <~~~


mavlink (without router):

  serial   o --\                     sx_serial
  serial2  o -- o <=============> o
  mbridge  o --/      mavlink        available()  ~~transmit~~>
                                     getc()

                                     putc()       <~~receive~~
  Do:
    serialport->available()  --->  parse&check  --->  msg_to_buf(X)
    serialport->getc()             buf_serial_in      fifo_link_out.PutBuf()
                                   status_serial_in

  read serialport, send fifo_link_in:
    available():  fifo_link_out.Available()
    getc()        fifo_link_out.Get()

  read link, send serialport:
    putc():       parse&check(X)  --->  but_to_msg(), msgtobuf(), serialport->putbuf()
                  buf_link_in
                  status_link_in
*/
