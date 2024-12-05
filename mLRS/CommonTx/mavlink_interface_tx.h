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
#include "../Common/thirdparty/mavlinkx.h"
#include "../Common/libs/fifo.h"
#define FASTMAVLINK_ROUTER_LINKS_MAX  4
#define FASTMAVLINK_ROUTER_COMPONENTS_MAX  12
#define FASTMAVLINK_ROUTER_LINK_PROPERTY_DEFAULT  FASTMAVLINK_ROUTER_LINK_PROPERTY_FLAG_ALWAYS_SEND_HEARTBEAT
#include "../Common/mavlink/out/lib/fastmavlink_router.h"
#endif


extern bool link_task_free(void);
extern volatile uint32_t millis32(void);
extern bool connected_and_rx_setup_available(void);
extern tStats stats;


#define RADIO_STATUS_SYSTEM_ID      51 // SiK uses 51, 68

#define MAVLINK_BUF_SIZE            300 // needs to be larger than max MAVLink frame size = 286 bytes


class tTxMavlink
{
  public:
    void Init(tSerialBase* const _serialport, tSerialBase* const _mbridge, tSerialBase* const _serial2port);
    void Do(void);
    uint8_t Task(void);
    uint8_t VehicleState(void);
    void FrameLost(void);

    void putc(char c);
    bool available(void);
    uint8_t getc(void);
    void flush(void);

  private:
    void send_msg_fifo_link_out(fmav_message_t* const msg);
    void handle_msg_serial_out(fmav_message_t* const msg);
    void send_msg_serial_out(void);

    void send_radio_status(void); // to serial_out

    uint16_t task_pending_mask;
    uint32_t task_pending_delay_ms;

    tSerialBase* ser;
    tSerialBase* ser2;
    bool do_router(void) { return (ser2 != nullptr); }

    // fields for link in -> parser -> serial out
    fmav_status_t status_link_in; // status for link in parser
    uint8_t buf_link_in[MAVLINK_BUF_SIZE]; // buffer for link in parser
    fmav_status_t status_serial_out; // status for serial out (ser and ser2) messages
    tFifo<char,128> fifo_link_in; // should need to only hold one OTA payload
    void parse_link_in_serial_out(void);

    // fields for ser/ser2 in -> parser -> link out
#ifdef USE_FEATURE_MAVLINKX
    fmav_status_t status_ser_in; // status for ser in parser
    uint8_t buf_ser_in[MAVLINK_BUF_SIZE]; // buffer for ser in parser
    fmav_status_t status_ser2_in;
    uint8_t buf_ser2_in[MAVLINK_BUF_SIZE];
    tFifo<char,512> fifo_link_out; // needs to be at least 82 + 280
#endif
    void parse_serial_in_link_out(void);

    fmav_message_t msg_buf; // temporary working buffer, to not burden stack
    uint8_t _buf[MAVLINK_BUF_SIZE]; // temporary working buffer, to not burden stack

    // to inject RADIO_STATUS messages
    uint32_t radio_status_tlast_ms;

    uint8_t vehicle_sysid; // 0 indicates data is invalid
    uint8_t vehicle_is_armed;
    uint8_t vehicle_is_flying;
    uint8_t vehicle_type;
    uint8_t vehicle_flight_mode;

    // MAVLink packet link quality
    bool msg_seq_initialized;
    uint8_t msg_seq_last;

    // MAVLink component handling
#ifdef USE_FEATURE_MAVLINK_COMPONENT
    void send_heartbeat(void);
    void send_autopilot_version(void);
    //void send_protocol_version(void); // not used
    void send_cmd_ack(uint16_t cmd, uint8_t res, uint8_t sysid, uint8_t compid);
    void send_param_value(uint16_t param_idx);
    void component_init(void);
    void component_do(void);
    bool component_task(uint8_t* const task);
    void component_handle_msg(fmav_message_t* const msg);

    uint16_t inject_task;
    uint32_t heartbeat_tlast_ms;
    bool param_request_list;
    uint16_t param_send_param_idx;
    uint32_t param_send_tlast_ms;
#endif
};


void tTxMavlink::Init(tSerialBase* const _serialport, tSerialBase* const _mbridge, tSerialBase* const _serial2port)
{
    // if ChannelsSource = MBRIDGE:
    //   SerialDestination = SERIAL or SERIAL2 => router with ser = mbridge & ser2 = serial/serial2
    //   SerialDestination = MBRDIGE           => no router, only ser = mbridge (ser2 = null)
    // => ser2 != nullptr indicates that router is to be used
    switch (Setup.Tx[Config.ConfigId].SerialDestination) {
    case SERIAL_DESTINATION_SERIAL:
        ser = _serialport;
        ser2 = (Setup.Tx[Config.ConfigId].ChannelsSource == CHANNEL_SOURCE_MBRIDGE) ? _mbridge : nullptr;
        break;
    case SERIAL_DESTINATION_SERIAL2:
        ser = _serial2port;
        ser2 = (Setup.Tx[Config.ConfigId].ChannelsSource == CHANNEL_SOURCE_MBRIDGE) ? _mbridge : nullptr;
        break;
    case SERIAL_DESTINATION_MBRDIGE:
        ser = _mbridge;
        ser2 = nullptr;
        break;
    default:
        while(1){} // must not happen
    }
    if (!ser) while(1){} // must not happen

    fmav_init();

    status_link_in = {};
    status_serial_out = {};
    fifo_link_in.Init();

#ifdef USE_FEATURE_MAVLINKX
    fmavX_init();
    fmavX_config_compression((Config.Mode == MODE_19HZ) ? 1 : 0); // use compression only in 19 Hz mode

    status_ser_in = {};
    status_ser2_in = {};
    fifo_link_out.Init();

    fmav_router_init();
#endif

    radio_status_tlast_ms = millis32() + 1000;

    vehicle_sysid = 0;
    vehicle_is_armed = UINT8_MAX;
    vehicle_is_flying = UINT8_MAX;
    vehicle_type = UINT8_MAX;
    vehicle_flight_mode = UINT8_MAX;

    msg_seq_initialized = false;
    msg_seq_last = 0;

    task_pending_mask = 0;
    task_pending_delay_ms = 0; // 0 means delay is disabled

#ifdef USE_FEATURE_MAVLINK_COMPONENT
    component_init();
#endif
}


uint8_t tTxMavlink::VehicleState(void)
{
    if (vehicle_is_armed == UINT8_MAX) return UINT8_MAX;
    if (vehicle_is_armed == 1 && vehicle_is_flying == 1) return 2;
    return vehicle_is_armed;
}


void tTxMavlink::FrameLost(void)
{
#ifdef USE_FEATURE_MAVLINKX
    // reset parser link in -> serial out
    fmav_parse_reset(&status_link_in);
#endif
}


void tTxMavlink::Do(void)
{
    uint32_t tnow_ms = millis32();
    bool inject_radio_status = false;

    if (!connected_and_rx_setup_available()) {
        //Init();
        radio_status_tlast_ms = tnow_ms;
        fifo_link_in.Flush();
#ifdef USE_FEATURE_MAVLINKX
        fifo_link_out.Flush();
#endif
        msg_seq_initialized = false;
    }

    if (!SERIAL_LINK_MODE_IS_MAVLINK(Setup.Rx.SerialLinkMode)) return;

    // parse link in -> serial out, do it before parse_serial_in_link_out()
    parse_link_in_serial_out();

    // parse ser in -> link out
    parse_serial_in_link_out();

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
        send_radio_status();
        return; // only one per loop
    }

#ifdef USE_FEATURE_MAVLINK_COMPONENT
    component_do();
#endif
}


uint8_t tTxMavlink::Task(void)
{
#ifdef USE_FEATURE_MAVLINK_COMPONENT
    uint8_t task;
    if (component_task(&task)) {
        return task;
    }
#endif

    task_pending_mask = 0;
    return TX_TASK_NONE;
}


void tTxMavlink::parse_serial_in_link_out(void)
{
fmav_result_t result;

    // parse ser in -> link out
#ifdef USE_FEATURE_MAVLINKX
if (!do_router()) {
    // without router, parse ser in -> link out
    if (fifo_link_out.HasSpace(290)) { // we have space for a full MAVLink message, so can safely parse
        while (ser->available()) {
            char c = ser->getc();
            fmav_parse_and_check_to_frame_buf(&result, buf_ser_in, &status_ser_in, c);
            if (result.res == FASTMAVLINK_PARSE_RESULT_OK) {
                fmav_frame_buf_to_msg(&msg_buf, &result, buf_ser_in); // requires RESULT_OK
                send_msg_fifo_link_out(&msg_buf);
#ifdef USE_FEATURE_MAVLINK_COMPONENT
                component_handle_msg(&msg_buf);
#endif
            }
        }
    }
} else {
    // with router, parse ser in, ser2 in -> link out
    if (fifo_link_out.HasSpace(290)) { // we have space for a full MAVLink message, so can safely parse
        // link 0 = sx_serial
        // link 1 = ser
        // link 2 = ser2

        static uint8_t scheduled_ser = 0;
        INCc(scheduled_ser, 2);
        if ((scheduled_ser == 0) && !ser->available()) scheduled_ser = 1; // take next if nothing to do
        if ((scheduled_ser == 1) && !ser2->available()) scheduled_ser = 0; // take next if nothing to do

        while ((scheduled_ser == 0) && ser->available()) {
            char c = ser->getc();
            if (fmav_parse_and_check_to_frame_buf(&result, buf_ser_in, &status_ser_in, c)) {
                fmav_router_handle_message(1, &result);
                if (fmav_router_send_to_link(1)) {} // WE DO NOT REFLECT, SO THIS MUST NEVER HAPPEN !!
                if (fmav_router_send_to_link(2)) {
                    ser2->putbuf(buf_ser_in, result.frame_len);
                }
                if (result.res == FASTMAVLINK_PARSE_RESULT_OK) {
                    fmav_frame_buf_to_msg(&msg_buf, &result, buf_ser_in); // requires RESULT_OK
                    if (fmav_router_send_to_link(0)) {
                        send_msg_fifo_link_out(&msg_buf);
                    }
#ifdef USE_FEATURE_MAVLINK_COMPONENT
                    component_handle_msg(&msg_buf);
#endif
                }
                break; // do only one message per loop
            }
        }

        while ((scheduled_ser == 1) && ser2->available()) {
            char c = ser2->getc();
            if (fmav_parse_and_check_to_frame_buf(&result, buf_ser2_in, &status_ser2_in, c)) {
                fmav_router_handle_message(2, &result);
                if (fmav_router_send_to_link(1)) {
                    ser->putbuf(buf_ser2_in, result.frame_len);
                }
                if (fmav_router_send_to_link(2)) {} // WE DO NOT REFLECT, SO THIS MUST NEVER HAPPEN !!
                if (result.res == FASTMAVLINK_PARSE_RESULT_OK) {
                    fmav_frame_buf_to_msg(&msg_buf, &result, buf_ser2_in); // requires RESULT_OK
                    if (fmav_router_send_to_link(0)) {
                        send_msg_fifo_link_out(&msg_buf);
                    }
#ifdef USE_FEATURE_MAVLINK_COMPONENT
                    component_handle_msg(&msg_buf);
#endif
                }
                break; // do only one message per loop
            }
        }

    }
} // end if(do_router())
#endif // USE_FEATURE_MAVLINKX
}


void tTxMavlink::parse_link_in_serial_out(void)
{
fmav_result_t result;

while (fifo_link_in.Available()) {
    char c = fifo_link_in.Get();

    // parse link in -> serial out
#ifdef USE_FEATURE_MAVLINKX
    if (Setup.Rx.SerialLinkMode == SERIAL_LINK_MODE_MAVLINK_X) {
        fmavX_parse_and_checkX_to_frame_buf(&result, buf_link_in, &status_link_in, c);
    } else {
        fmav_parse_and_check_to_frame_buf(&result, buf_link_in, &status_link_in, c);
    }
    if (result.res == FASTMAVLINK_PARSE_RESULT_OK) {

if (!do_router()) {
        // without router
        ser->putbuf(buf_link_in, result.frame_len);
} else {
        // with router
        fmav_router_handle_message(0, &result);
        if (fmav_router_send_to_link(1)) {
            ser->putbuf(buf_link_in, result.frame_len);
        }
        if (fmav_router_send_to_link(2)) {
            ser2->putbuf(buf_link_in, result.frame_len);
        }
        if (fmav_router_send_to_link(0)) {} // WE DO NOT REFLECT, SO THIS MUST NEVER HAPPEN !!
} // end if(do_router())

#else
    fmav_parse_and_check_to_frame_buf(&result, buf_link_in, &status_link_in, c);
    if (result.res == FASTMAVLINK_PARSE_RESULT_OK) {
        ser->putbuf(buf_link_in, result.frame_len);
#endif

        fmav_frame_buf_to_msg(&msg_buf, &result, buf_link_in); // requires RESULT_OK

        // allow CRSF to capture it
        crsf.TelemetryHandleMavlinkMsg(&msg_buf);

        // we also want to capture it to extract some info
        handle_msg_serial_out(&msg_buf);

#ifdef DEBUG_ENABLEDx
// test if _buf = buf_link_in
uint16_t len = fmav_msg_to_frame_buf(_buf, &msg_buf); // _buf should be equal buf_link_in !?!
if (len != result.frame_len) while(1){}
for (uint16_t i = 0; i < len; i++) if (_buf[i] != buf_link_in[i]) while(1){}
#endif

        // don't jump out early here, seems to be important to do all
        //return; // do only one message per loop
    }

} // while (fifo_link_in.Available())
}


void tTxMavlink::send_msg_fifo_link_out(fmav_message_t* const msg)
{
#ifdef USE_FEATURE_MAVLINKX
    uint16_t len;
    if (Setup.Rx.SerialLinkMode == SERIAL_LINK_MODE_MAVLINK_X) {
        len = fmavX_msg_to_frame_bufX(_buf, msg);
    } else {
        len = fmav_msg_to_frame_buf(_buf, msg);
    }

    fifo_link_out.PutBuf(_buf, len);
#endif
}


void tTxMavlink::send_msg_serial_out(void)
{
    // TODO: this could be be done more efficiently by not going via msg_serial_out
    // but by generating directly into _buf

    uint16_t len = fmav_msg_to_frame_buf(_buf, &msg_buf);

    ser->putbuf(_buf, len);

#ifdef USE_FEATURE_MAVLINKX
    if (ser2) ser2->putbuf(_buf, len);
#endif
}


void tTxMavlink::putc(char c)
{
    fifo_link_in.Put(c);
}


bool tTxMavlink::available(void)
{
    if (!ser) return false; // should not happen

#ifdef USE_FEATURE_MAVLINKX
    return fifo_link_out.Available();
#else
    return ser->available();
#endif
}


uint8_t tTxMavlink::getc(void)
{
    if (!ser) return 0; // should not happen

#ifdef USE_FEATURE_MAVLINKX
    return fifo_link_out.Get();
#else
    return ser->getc();
#endif
}


void tTxMavlink::flush(void)
{
    if (!ser) return; // should not happen

#ifdef USE_FEATURE_MAVLINKX
    fifo_link_out.Flush();
    if (ser2) ser2->flush();
#endif
    ser->flush();
}


//-------------------------------------------------------
// Handle Messages
//-------------------------------------------------------

void tTxMavlink::handle_msg_serial_out(fmav_message_t* const msg)
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

    // continue only for autopilot
    if (msg->sysid != vehicle_sysid || msg->compid != MAV_COMP_ID_AUTOPILOT1) return;

    switch (msg->msgid) {
    case FASTMAVLINK_MSG_ID_EXTENDED_SYS_STATE:{
        fmav_extended_sys_state_t payload;
        fmav_msg_extended_sys_state_decode(&payload, msg);
        vehicle_is_flying = (payload.landed_state == MAV_LANDED_STATE_IN_AIR) ? 1 : 0;
        }break;
    }

    // MAVLink packet link quality, for stream from autopilot only
    if (msg_seq_initialized) {
        uint8_t expected_seq = msg_seq_last + 1;
        stats.doMavlinkCnt(msg->seq == expected_seq);
    }
    msg_seq_initialized = true;
    msg_seq_last = msg->seq;
}


//-------------------------------------------------------
// Generate Messages
//-------------------------------------------------------

void tTxMavlink::send_radio_status(void)
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
        &msg_buf,
        RADIO_STATUS_SYSTEM_ID, // sysid, SiK uses 51, 68
        MAV_COMP_ID_TELEMETRY_RADIO,
        rssi, remrssi, txbuf, noise, UINT8_MAX, 0, 0,
        //uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed,
        &status_serial_out);

    send_msg_serial_out();
}


//-------------------------------------------------------
// Parameter Handling
//-------------------------------------------------------
// MissionPlanner inconveniences
// - connection possible only with autopilot present
// - one needs to call RefreshParams explicitly
// - it doesn't disconnect/connect component upon loss of heartbeat and re-appearance of heartbeat
// - doesn't update list if PARAM_VALUE are send voluntarily
// MissionPlanner goodies
// - sends MAV_CMD_PREFLIGHT_STORAGE ca 10 seconds after WriteParams
#ifdef USE_FEATURE_MAVLINK_COMPONENT

// my mcselec usb vid/pids
#define MAVLINK_VID               0x16D0
#define MAVLINK_PID               0x0FCB


typedef enum {
    INJECT_TASK_NONE                = 0,
    INJECT_TASK_HEARTBEAT           = 0x0001,
    INJECT_TASK_AUTOPILOT_VERSION   = 0x0002,
    INJECT_TASK_PROTOCOL_VERSION    = 0x0004,
    INJECT_TASK_PARAM_VALUE         = 0x0010,
} INJECT_TASK;


// the parameter list needs to be mangled
uint8_t pstore_u8 = 0;
uint32_t bindphrase_u32;

#define PARAM_LIST_TYPE_LIST  MAV_PARAM_TYPE_UINT8
#define PARAM_LIST_TYPE_INT8  MAV_PARAM_TYPE_INT8

// we don't put it in flash but ram, since some pointers need to be modified on init
fmav_param_entry_t fmav_param_list[] = {
    { (uint8_t*)&(pstore_u8), MAV_PARAM_TYPE_UINT8, "PSTORE" },
    { (uint8_t*)&(Setup._ConfigId), MAV_PARAM_TYPE_UINT8, "CONFIG ID" }, // we don't use config_id class here
    { (uint32_t*)&(bindphrase_u32), MAV_PARAM_TYPE_UINT32, "BIND_PHRASE_U32" },
    #define X(p,t, n,mn, d,mi,ma,u, s, amp) { \
            .ptr = (t*)&(p), \
            .type = PARAM_LIST_TYPE_##t, \
            .name = mn },
    SETUP_PARAMETER_LIST_COMMON_FURTHER \
    SETUP_PARAMETER_LIST_TX \
    SETUP_PARAMETER_LIST_RX
    #undef X
};

#define MAV_PARAM_PSTORE_IDX      0
#define MAV_PARAM_BINDPHRASE_IDX  2

#define FASTMAVLINK_PARAM_NUM     sizeof(fmav_param_list)/sizeof(fmav_param_entry_t)
STATIC_ASSERT(FASTMAVLINK_PARAM_NUM == SETUP_PARAMETER_NUM + MAV_PARAM_BINDPHRASE_IDX, "FASTMAVLINK_PARAM_NUM missmatch")
#include "../Common/mavlink/out/lib/fastmavlink_parameters.h"


void tTxMavlink::send_heartbeat(void)
{
    fmav_msg_heartbeat_pack(
        &msg_buf,
        RADIO_STATUS_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO,  // sysid, compid, SiK uses 51, 68
        MAV_TYPE_GENERIC, // type ???
        MAV_AUTOPILOT_INVALID,
        MAV_MODE_FLAG_SAFETY_ARMED,
        0,
        MAV_STATE_ACTIVE,
        //uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status,
        &status_serial_out);

    send_msg_serial_out();
}


void tTxMavlink::send_autopilot_version(void)
{
uint8_t dummy[18+2] = {};

    fmav_msg_autopilot_version_pack(
        &msg_buf,
        RADIO_STATUS_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO, // sysid, compid, SiK uses 51, 68
        MAV_PROTOCOL_CAPABILITY_MAVLINK2 | MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE,
        VERSION, 0, 0, 0,
        dummy, dummy, dummy,
        MAVLINK_VID, MAVLINK_PID, 0, dummy,
        //uint64_t capabilities,
        //uint32_t flight_sw_version, uint32_t middleware_sw_version, uint32_t os_sw_version, uint32_t board_version,
        //const uint8_t* flight_custom_version, const uint8_t* middleware_custom_version, const uint8_t* os_custom_version,
        //uint16_t vendor_id, uint16_t product_id, uint64_t uid, const uint8_t* uid2,
        &status_serial_out);

    send_msg_serial_out();
}


/* not used.
void tTxMavlink::send_protocol_version(void)
{
uint8_t dummy[8+2] = {};

    fmav_msg_protocol_version_pack(
        &msg_buf,
        RADIO_STATUS_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO, // sysid, compid, SiK uses 51, 68
        200, 100, 200,
        dummy, dummy,
        //uint16_t version, uint16_t min_version, uint16_t max_version,
        //const uint8_t* spec_version_hash, const uint8_t* library_version_hash,
        &status_serial_out);

    send_msg_serial_out();
} */


void tTxMavlink::send_cmd_ack(uint16_t cmd, uint8_t res, uint8_t sysid, uint8_t compid)
{
    fmav_msg_command_ack_pack(
        &msg_buf,
        RADIO_STATUS_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO,  // sysid, compid, SiK uses 51, 68
        cmd, res, 0, 0,
        sysid, compid,
        //uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2,
        //uint8_t target_system, uint8_t target_component,
        &status_serial_out);

    send_msg_serial_out();
}


void tTxMavlink::send_param_value(uint16_t param_idx)
{
    // if BIND_PHRASE_U32, then do special handling, get bind_phrase_u32 from bind_phrase
    if (param_idx == MAV_PARAM_BINDPHRASE_IDX) {
        bindphrase_u32 = u32_from_bindphrase(Setup.Common[Config.ConfigId].BindPhrase);
    }

    fmav_param_value_t payload;
    if (!fmav_param_get_param_value(&payload, param_idx)) return; // fill fmav_param_value_t structure from list

    fmav_msg_param_value_encode(
        &msg_buf,
        RADIO_STATUS_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO,  // sysid, compid, SiK uses 51, 68
        &payload,
        &status_serial_out);

    send_msg_serial_out();
}


void tTxMavlink::component_handle_msg(fmav_message_t* const msg)
{
    if (Setup.Tx[Config.ConfigId].MavlinkComponent != TX_MAVLINK_COMPONENT_ENABLED) return;

    if (!fmav_msg_is_for_me(RADIO_STATUS_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO, msg)) return; // not for us

    switch (msg->msgid) {
        case FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ: {
            fmav_param_request_read_t payload;
            fmav_msg_param_request_read_decode(&payload, msg);
            uint16_t param_idx;
            if (fmav_param_do_param_request_read(&param_idx, &payload)) { // search list for param idx
                param_send_param_idx = param_idx;
                inject_task |= INJECT_TASK_PARAM_VALUE;
            }
            break;}

        case FASTMAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
            param_request_list = true;
            param_send_param_idx = 0;
            param_send_tlast_ms = millis32() - 100;
            break;}

        case FASTMAVLINK_MSG_ID_PARAM_SET: {
            if (msg->target_sysid != RADIO_STATUS_SYSTEM_ID ||
                msg->target_compid != MAV_COMP_ID_TELEMETRY_RADIO) break; // only accept if targeted at us

            fmav_param_set_t payload;
            fmav_msg_param_set_decode(&payload, msg);
            uint16_t param_idx;
            if (fmav_param_do_param_set(&param_idx, &payload)) { // search list for param idx
                fmav_param_set_value(param_idx, payload.param_value); // set value
                // sanitize settings
                setup_sanitize_config(Config.ConfigId);
                // don't allow setting TX_MAV_PARAMS
                Setup.Tx[Config.ConfigId].MavlinkComponent = TX_MAVLINK_COMPONENT_ENABLED;
                // don't allow disabling setting RX_SER_LNK_MODE
                if (Setup.Rx.SerialLinkMode == SERIAL_LINK_MODE_TRANSPARENT) {
                    Setup.Rx.SerialLinkMode = SERIAL_LINK_MODE_MAVLINK;
                }
                // if BIND_PHRASE_U32 do special handling
                if (param_idx == MAV_PARAM_BINDPHRASE_IDX) {
                    bindphrase_from_u32(Setup.Common[Config.ConfigId].BindPhrase, bindphrase_u32);
                }
                // if PSTORE invoke paramstore
                if (param_idx == MAV_PARAM_PSTORE_IDX && pstore_u8) {
                    pstore_u8 = 0;
                    task_pending_mask = (1 << TX_TASK_RX_PARAM_SET) | (1 << TX_TASK_PARAM_STORE);
                    task_pending_delay_ms = millis32();
                    if (task_pending_delay_ms == 0) task_pending_delay_ms = -1; // 0 indicates disabled
                }
                // send a PARAM_VALUE response
                param_send_param_idx = param_idx;
                inject_task |= INJECT_TASK_PARAM_VALUE;
            }
            break;}

        case FASTMAVLINK_MSG_ID_COMMAND_INT:
        case FASTMAVLINK_MSG_ID_COMMAND_LONG: {
            if (msg->target_sysid != RADIO_STATUS_SYSTEM_ID ||
                msg->target_compid != MAV_COMP_ID_TELEMETRY_RADIO) break; // only accept if targeted at us

            uint16_t command;
            uint32_t param1;
            if (msg->msgid == FASTMAVLINK_MSG_ID_COMMAND_LONG) {
                command = fmav_msg_command_long_get_field_command(msg);
                param1 = (uint32_t)fmav_msg_command_long_get_field_param1(msg);
            } else {
                command = fmav_msg_command_int_get_field_command(msg);
                param1 = (uint32_t)fmav_msg_command_int_get_field_param1(msg);
            }

            uint8_t res = MAV_RESULT_DENIED;

            switch (command) {
                case MAV_CMD_REQUEST_MESSAGE: // #512
                    switch (param1) {
                        case FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION: // #148
                            inject_task |= INJECT_TASK_AUTOPILOT_VERSION;
                            res = MAV_RESULT_ACCEPTED;
                            break;
                        case FASTMAVLINK_MSG_ID_PROTOCOL_VERSION: // #300
                            inject_task |= INJECT_TASK_PROTOCOL_VERSION;
                            res = MAV_RESULT_ACCEPTED;
                            break;
                    }
                    break;
                case MAV_CMD_REQUEST_PROTOCOL_VERSION: // #519, replaced by MAV_CMD_REQUEST_MESSAGE
                    inject_task |= INJECT_TASK_PROTOCOL_VERSION;
                    res = MAV_RESULT_ACCEPTED;
                    break;
                case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: // #520, replaced by MAV_CMD_REQUEST_MESSAGE
                    inject_task |= INJECT_TASK_AUTOPILOT_VERSION;
                    res = MAV_RESULT_ACCEPTED;
                    break;
            } //end of switch (command)

            // that's a small message, so let's hope buf has enough space
            send_cmd_ack(command, res, msg->sysid, msg->compid);
            break;}

      } // end of switch (msg->msgid)
}


void tTxMavlink::component_do(void)
{
    if (Setup.Tx[Config.ConfigId].MavlinkComponent != TX_MAVLINK_COMPONENT_ENABLED) return;

    uint32_t tnow_ms = millis32();

    // heartbeat

    if ((tnow_ms - heartbeat_tlast_ms) >= 1000) {
        heartbeat_tlast_ms = tnow_ms;
        inject_task |= INJECT_TASK_HEARTBEAT;
    }

    // param handling

    if (param_request_list) {
        if ((tnow_ms - param_send_tlast_ms) >= 50) { // send next param in 50 ms
            param_send_tlast_ms = tnow_ms;
            inject_task |= INJECT_TASK_PARAM_VALUE;
        }
    }

    // send pending messages

    if (inject_task & INJECT_TASK_PARAM_VALUE) {
        inject_task &=~ INJECT_TASK_PARAM_VALUE;
        send_param_value(param_send_param_idx);
        param_send_param_idx++;
        if (param_send_param_idx >= FASTMAVLINK_PARAM_NUM) param_request_list = false;
        return; // only one per loop
    }

    if (inject_task & INJECT_TASK_AUTOPILOT_VERSION) {
        inject_task &=~ INJECT_TASK_AUTOPILOT_VERSION;
        send_autopilot_version();
        return; // only one per loop
    }

    if (inject_task & INJECT_TASK_PROTOCOL_VERSION) {
        inject_task &=~ INJECT_TASK_PROTOCOL_VERSION;
        send_autopilot_version();
        return; // only one per loop
    }

    if (inject_task & INJECT_TASK_HEARTBEAT) { // check available size!?
        inject_task &=~ INJECT_TASK_HEARTBEAT;
        send_heartbeat();
        return; // only one per loop
    }
}


bool tTxMavlink::component_task(uint8_t* const task)
{
    if (Setup.Tx[Config.ConfigId].MavlinkComponent != TX_MAVLINK_COMPONENT_ENABLED) {
        *task = TX_TASK_NONE;
        return false;
    }

    if (task_pending_mask & (1 << TX_TASK_RX_PARAM_SET)) {
        task_pending_mask &=~ (1 << TX_TASK_RX_PARAM_SET);
        *task = TX_TASK_RX_PARAM_SET;
        return true;
    }

    if (task_pending_mask & (1 << TX_TASK_PARAM_STORE)) {
        // we need to wait for the previous link task to finish
        // link task is not free in connection stage, so skip if not in connection stage
        if (connected_and_rx_setup_available() && !link_task_free()) {
            *task = TX_TASK_NONE;
            return true;
        }
        // delay by 50 ms, so a response message is send
        if (task_pending_delay_ms) {
            if ((millis32() - task_pending_delay_ms) < 50) {
                *task = TX_TASK_NONE;
                return true;
            }
            task_pending_delay_ms = 0;
        }
        task_pending_mask &=~ (1 << TX_TASK_PARAM_STORE);
        *task = TX_TASK_PARAM_STORE;
        return true;
    }

    *task = TX_TASK_NONE;
    return false;
}


void tTxMavlink::component_init(void)
{
    heartbeat_tlast_ms = millis32();

    param_request_list = false;
    param_send_param_idx = 0;
    param_send_tlast_ms = 0;

    inject_task = INJECT_TASK_NONE;

    pstore_u8 = 0;
    bindphrase_u32 = u32_from_bindphrase(Setup.Common[Config.ConfigId].BindPhrase);
    // set pointers, taking into account ConfigId
    // idx points into SetupParameter[], not fmav_param_list
    // all extra parameters need to come before bind phrase
    for (uint8_t idx = MAV_PARAM_BINDPHRASE_IDX; idx < SETUP_PARAMETER_NUM; idx++) {
        fmav_param_list[MAV_PARAM_BINDPHRASE_IDX + idx].ptr = SetupParameterPtr(idx);
    }
}


#endif // USE_FEATURE_MAVLINK_COMPONENT

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
    ser->available()  --->  parse&check  --->  msg_to_buf(X)
    ser->getc()             buf_serial_in      fifo_link_out.PutBuf()
                            status_serial_in

  read serialport, send fifo_link_in:
    available():  fifo_link_out.Available()
    getc()        fifo_link_out.Get()

  read link, send serialport:
    putc():       parse&check(X)  --->  but_to_msg(), msgtobuf(), ser->putbuf()
                  buf_link_in
                  status_link_in
*/
