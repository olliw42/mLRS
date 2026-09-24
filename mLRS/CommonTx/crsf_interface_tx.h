//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// CRSF Interface TX Side
//*******************************************************
#ifndef CRSF_INTERFACE_TX_H
#define CRSF_INTERFACE_TX_H
#pragma once


#include "../Common/protocols/msp_protocol.h"


#ifdef DEVICE_HAS_JRPIN5

#include "math.h"
#include "time.h"
#include "../Common/thirdparty/thirdparty.h"
#include "../Common/protocols/crsf_protocol.h"
#include "../Common/protocols/passthrough_protocol.h"
#include "../Common/protocols/ardupilot_protocol.h"
#include "jr_pin5_interface.h"


extern uint16_t micros16(void);
extern volatile uint32_t millis32(void);
extern tSetupMetaData SetupMetaData;
extern tSetup Setup;
extern tGlobalConfig Config;
extern tStats stats;

#define USE_CRSF_MB
//-------------------------------------------------------
// Interface Implementation

#define CRSF_FRAME_BUNDLING // bundle several CRSF frames into one slot, only momentarily

#define CRSF_BUF_SIZE  (CRSF_FRAME_LEN_MAX + 16)

typedef enum {
    TXCRSF_SEND_LINK_STATISTICS = 0,
    TXCRSF_SEND_LINK_STATISTICS_TX,
    TXCRSF_SEND_LINK_STATISTICS_RX,
    TXCRSF_SEND_LINK_STATISTICS_ALL,
    TXCRSF_SEND_TELEMETRY_FRAME, // native or passthrough telemetry frame
    TXCRSF_SEND_DEVICE_INFO,
} TXCRSF_SEND_ENUM;


typedef enum {
    TXCRSF_CMD_MODELID_SET = 0,
    TXCRSF_CMD_BIND_START,
    TXCRSF_CMD_BIND_STOP,
    TXCRSF_CMD_MBRIDGE_IN,
} TXCRSF_CMD_ENUM;


class tTxCrsf : public tPin5BridgeBase, public tSerialBase
{
  public:
    using tSerialBase::Init; // tTxCrsf redefines Init(), incompatible with tSerialBase's Init()
    void Init(bool enable_flag, bool crsfbridge_enable_flag);
    void Do(void);
    bool ChannelsUpdated(tRcData* const rc);
    bool TelemetryUpdate(uint8_t* const task, uint16_t frame_rate_ms);

    bool CommandReceived(uint8_t* const cmd);
    uint8_t* GetPayloadPtr(void);
    uint8_t GetPayloadLen(void);
    uint8_t GetCmdModelId(void);

    void TelemetryHandleMavlinkMsg(fmav_message_t* const msg);
    void TelemetryHandleMspMsg(msp_message_t* const msg);
    void SendTelemetryFrame(void);

    void SendLinkStatistics(void); // in OpenTx this triggers telemetryStreaming
    void SendLinkStatisticsTx(void);
    void SendLinkStatisticsRx(void);
    void SendDeviceInfo(void);
    void SendLinkStatisticsAll(void);
    void SendMbStatistics(void);

    void SendMBridgeFrame(void* const payload, uint8_t payload_len);

    void PassthroughSetBattery0Capacity(uint32_t capacity); // wrapper since not available to all targets

    // CRSF envelope handling
    // provides serial interface to the main code
    void putbuf(uint8_t* const buf, uint16_t len) override { if (crsfbridge_enabled) put_fifo.PutBuf(buf, len); }
    bool available(void) override { return get_fifo.Available(); }
    char getc(void) override { return get_fifo.Get(); }
    void flush(void) override { get_fifo.Flush(); }

  private:
    // helper
    void send_frame(const uint8_t frame_id, void* const payload, uint8_t payload_len);

    uint8_t crc8(const uint8_t* const buf);
    bool fill_rcdata(tRcData* const rc);
    bool fill_rcdata_0x17(tRcData* const rc);

    // for in-isr processing, used in half-duplex mode
    void parse_nextchar(uint8_t c) override;
    bool transmit_start(void) override; // returns true if transmission should be started

    bool enabled;
    bool crsfbridge_enabled;

    // parser, state is defined in tPin5BridgeBase
    // no need for volatile since used only in isr context
    uint8_t rx_frame[CRSF_BUF_SIZE]; // received frame
    uint8_t rx_len;
    uint8_t rx_cnt;
    uint16_t rx_tlast_us;
    volatile bool rx_frame_received;
    tCrsfFrame frame; // double buffered received frame

    bool channels_received;
    bool mbridge_cmd_received;
    bool ping_device_received;
    bool cmd_modelid_received; // we handle it extra just to really catch it, could do also cmd fifo
    uint8_t cmd_modelid_value;
    bool cmd_bind_set_received;
    bool cmd_bind_cancel_received;

    uint8_t tx_frame[CRSF_BUF_SIZE];
    volatile bool tx_free; // to signal that the tx buffer can be filled
    volatile uint8_t tx_available; // this signals if something needs to be send to radio

    bool startup_passed; // send CRSF frames only after at least a RC channels frame has been received, helps with catching MODEILID

    // CRSF telemetry

    // CRSF_FRAME_ID_GPS (0x02), collected from several MAVLink messages:
    //   GPS_RAW_INT, GPS2_RAW (SRy_EXTENDED_STATUS), VFR_HUD (SRy_EXTRA2), GLOBAL_POSITION_INT (SRy_POSITION)
    uint8_t gps_raw_int_sat;
    uint8_t gps2_raw_sat;
    float vfr_hud_groundspd_mps;
    tCrsfGps gps;

    // Note: ArduPilot sends in GPS_RAW_INT.time_usec the last fix time in ms since boot, so cannot be used.
    // We thus grab SYSTEM_TIME.time_unix_usec, which ArduPilot takes from AP::rtc().
    // This can be set to various sources in parameter RTC_TYPES: 0:GPS,1:MAVLINK_SYSTEM_TIME,2:HW.
    tCrsfGpsTime gps_time;

    tCrsfFlightMode flight_mode;
    tCrsfBattery battery;
    tCrsfAttitude attitude;
    tCrsfVariometer variometer;
    tCrsfBaroAltitude baro_altitude;
    tCrsfAirspeed airspeed;
    tCrsfTemp temp_ambient;
    tCrsfBarometer barometer;

    typedef struct {
        bool updated;
        uint32_t send_tlast_ms;
    } tCrsfStatus;

    typedef struct {
        uint8_t frame_id;
        void* payload_ptr;
        uint8_t payload_len;
    } tCrsfItem;

    // the sequence gives the priority
    // Note: sequence in CRSF_ITEM_ENUM and crsf_items[] must match!
    typedef enum {
        CRSF_ITEM_FLIGHT_MODE = 0,  // CRSF_FRAME_ID_FLIGHT_MODE (0x21), collected from HEARTBEAT
        CRSF_ITEM_GPS,              // CRSF_FRAME_ID_GPS (0x02),        collected from several MAVLink messages (SRy_EXTENDED_STATUS,SRy_EXTRA2,SRy_POSITION)
        CRSF_ITEM_ATTITUDE,         // CRSF_FRAME_ID_ATTITUDE (0x1E),   collected from ATTITUDE (SRy_EXTRA1)
        CRSF_ITEM_VARIOMETER,       // CRSF_FRAME_ID_VARIOMETER (0x07), collected from VFR_HUD (SRy_EXTRA2)
        CRSF_ITEM_BARO_ALTITUDE,    // not yet populated from a MAVLink message, AP does not appear to provide baro alt at all
        CRSF_ITEM_AIRSPEED,         // CRSF_FRAME_ID_AIRSPEED (0x0A),   collected from VFR_HUD (SRy_EXTRA2)
        CRSF_ITEM_TEMP,             // CRSF_FRAME_ID_TEMP (0x0D),       collected from SCALED_PRESSURE (SRy_RAW_SENSORS)
        CRSF_ITEM_BAROMETER,        // CRSF_FRAME_ID_BAROMETER (0x11),  collected from SCALED_PRESSURE (SRy_RAW_SENSORS)
        CRSF_ITEM_BATTERY,          // CRSF_FRAME_ID_BATTERY (0x08),    collected from BATTERY_STATUS (SRy_EXTRA2)
        CRSF_ITEM_GPS_TIME,         // CRSF_FRAME_ID_GPS_TIME (0x03),   collected from SYSTEM_TIME (SRy_EXTRA2)
        CRSF_ITEMS_LEN,
    } CRSF_ITEM_ENUM;

    const tCrsfItem crsf_items[CRSF_ITEMS_LEN] = {
        { CRSF_FRAME_ID_FLIGHT_MODE, &flight_mode, CRSF_FLIGHT_MODE_LEN },        // flight_mode
        { CRSF_FRAME_ID_GPS, &gps, CRSF_GPS_LEN },                                // gps
        { CRSF_FRAME_ID_ATTITUDE, &attitude, CRSF_ATTITUDE_LEN },                 // attitude
        { CRSF_FRAME_ID_VARIOMETER, &variometer, CRSF_VARIOMETER_LEN },           // variometer
        { CRSF_FRAME_ID_BARO_ALTITUDE, &baro_altitude, CRSF_BARO_ALTITUDE_LEN },  // baro_alt
        { CRSF_FRAME_ID_AIRSPEED, &airspeed, CRSF_AIRSPEED_LEN },                 // airspeed
        { CRSF_FRAME_ID_TEMP, &temp_ambient, CRSF_TEMP_LEN },                     // temp_ambient
        { CRSF_FRAME_ID_BAROMETER, &barometer, CRSF_BAROMETER_LEN },              // barometer
        { CRSF_FRAME_ID_BATTERY, &battery, CRSF_BATTERY_LEN },                    // battery
        { CRSF_FRAME_ID_GPS_TIME, &gps_time, CRSF_GPS_TIME_LEN },                 // gps_time
    };

    tCrsfStatus crsf_status[CRSF_ITEMS_LEN];

    // CS(CF_RPM_ID,      0, STR_DEF(STR_SENSOR_RPM),           UNIT_RPMS,              0),
    // CS(CELLS_ID,       0, STR_DEF(STR_SENSOR_CELLS),         UNIT_CELLS,             2),
    // CS(VOLT_ARRAY_ID,  0, STR_DEF(STR_SENSOR_VOLT),          UNIT_VOLTS,             2),

    // MAVLink handlers

    void handle_mavlink_msg_heartbeat(fmav_heartbeat_t* const payload);
    void handle_mavlink_msg_system_time(fmav_system_time_t* const payload);
    void handle_mavlink_msg_battery_status(fmav_battery_status_t* const payload);
    void handle_mavlink_msg_attitude(fmav_attitude_t* const payload);
    void handle_mavlink_msg_gps_raw_int(fmav_gps_raw_int_t* const payload);
    void handle_mavlink_msg_gps2_raw(fmav_gps2_raw_t* const payload);
    void handle_mavlink_msg_global_position_int(fmav_global_position_int_t* const payload);
    void handle_mavlink_msg_vfr_hud(fmav_vfr_hud_t* const payload);
    void handle_mavlink_msg_scaled_pressure(fmav_scaled_pressure_t* const payload);

    // CRSF passthrough telemetry

    tPassThrough passthrough;

    // MSP handlers

    int32_t inav_baro_altitude; // needed to make INAV happy
    uint16_t msp_inav_status_sensor_status;
    uint32_t msp_inav_status_arming_flags;

    // CRSF envelope

    tFifo<char,TX_CRSFBRIDGE_TXBUFSIZE> put_fifo; // TODO: how large do they really need to be?
    tFifo<char,TX_CRSFBRIDGE_RXBUFSIZE> get_fifo;

    union {
        tCrsfMbEnvelope mb;           // 0xEA, len, 0x82, 0x66
        tCrsfMavlinkEnvelope mavlink; // 0xEA, len, 0xAA
    } crsf_envelope_out;
    uint8_t crsf_envelop_out_sequence;
    bool crsf_envelop_use_mb;
    uint32_t crsf_envelop_out_tlast_ms;

    // momentarily for debug, detect discarded bytes
#ifdef USE_DEBUG
    uint16_t discarded = 0;
#endif
};

tTxCrsf crsf;


//-------------------------------------------------------
// CRSF half-duplex interface, used for radio <-> mLRS tx module

// to avoid error: ISO C++ forbids taking the address of a bound member function to form a pointer to member function
void crsf_pin5_rx_callback(uint8_t c) { crsf.pin5_rx_callback(c); }
void crsf_pin5_tc_callback(void) { crsf.pin5_tc_callback(); }
void crsf_pin5_cc1_callback(void) { crsf.pin5_cc1_callback(); }


// is called in isr context
bool tTxCrsf::transmit_start(void)
{
    tx_free = true; // tell external code that tx_frame can be filled with new data

    if (!tx_available) { // nothing to send
        return false;
    }

    pin5_putbuf(tx_frame, tx_available);

    tx_available = 0;

    return true;
}


// a frame is sent every 4 ms, frame length is max 64 bytes
// a byte is 25 us
// gaps between received frames are at least 2.4 ms
#define CRSF_PARSE_NEXTCHAR_TMO_US  500


// CRSF frame format:
// address len type payload crc
// len is the length including type, payload, crc

// is called in isr context
void tTxCrsf::parse_nextchar(uint8_t c)
{
    uint16_t tnow_us = micros16();

    if (state != STATE_IDLE) {
        uint16_t dt = tnow_us - rx_tlast_us;
        if (dt > CRSF_PARSE_NEXTCHAR_TMO_US) state = STATE_IDLE;

        if (rx_cnt >= sizeof(rx_frame)) state = STATE_IDLE; // prevent buffer overflow
    }

    rx_tlast_us = tnow_us;

    switch (state) {
    case STATE_IDLE:
        if ((c == CRSF_ADDRESS_TRANSMITTER_MODULE) || (c == CRSF_OPENTX_SYNC)) {
            rx_cnt = 0;
            rx_frame[rx_cnt++] = c;
            state = STATE_RECEIVE_CRSF_LEN;
#ifdef USE_DEBUG
            if (discarded) {
                if (discarded > 1) {
                    dbg.puts(u16toBCD_s(discarded));
                    dbg.puts(" bytes lost!\n");
                }
                discarded = 0;
            }
        } else {
            discarded++;
#endif
        }
        break;

    case STATE_RECEIVE_CRSF_LEN:
        if (c >= (CRSF_FRAME_LEN_MAX - 2)) { state = STATE_IDLE; break; } // cannot be a valid CRSF frame
        rx_frame[rx_cnt++] = c;
        rx_len = c;
        state = STATE_RECEIVE_CRSF_PAYLOAD;
        break;
    case STATE_RECEIVE_CRSF_PAYLOAD:
        rx_frame[rx_cnt++] = c;
        if (rx_cnt >= rx_len + 1) {
            state = STATE_RECEIVE_CRSF_CRC;
        }
        break;
    case STATE_RECEIVE_CRSF_CRC:
        rx_frame[rx_cnt++] = c;
        memcpy(&frame, rx_frame, rx_cnt);
        rx_frame_received = true;
        state = STATE_TRANSMIT_START;
        break;
    }
}


//-------------------------------------------------------
// miscellaneous

// CRSF:
// 11 bit, 173 ... 992 .. 1811 for +-100%
// so: 9 ... 173 ... 992 .. 1811 ... 1965  for -120%  -100%    0%    +100%    +120%
// 100% = 819 span
// 120% = 983 span
// rcData: 11 bits,  1 .. 1024 .. 2047 for +-120%
// see design_decissions.h
// Comment: technically, according to the CRSF spec, frame's len is allowed to be smaller
// than CRSF_RCCHANNELPACKET_LEN, in which case we would have to not set high rc data.
// We assume that's not happening. Note, that len can also be larger, which is in fact
// done by EdgeTx to provide an additional status byte carrying arming info for ELRS.

bool tTxCrsf::fill_rcdata(tRcData* const rc)
{
tCrsfRcChannelV2* buf = (tCrsfRcChannelV2*)frame.payload;
bool is_32channels;

    // TODO: variable size frames ??

    if (frame.len >= 1 + 22 + 1 && frame.len <= 1 + 23 + 1) { // V1 frame, we only accept frames with 16 channels
        is_32channels = false;
    } else if (frame.len == 1 + 22 + 1 + 22 + 1) { // V2 frame, we only accept frames with 32 channels
        rc->do_32channels = true;
        is_32channels = true;
    } else {
        return false;
    }

    rc->ch[0] = rc_from_crsf(buf->ch0);
    rc->ch[1] = rc_from_crsf(buf->ch1);
    rc->ch[2] = rc_from_crsf(buf->ch2);
    rc->ch[3] = rc_from_crsf(buf->ch3);
    rc->ch[4] = rc_from_crsf(buf->ch4);
    rc->ch[5] = rc_from_crsf(buf->ch5);
    rc->ch[6] = rc_from_crsf(buf->ch6);
    rc->ch[7] = rc_from_crsf(buf->ch7);
    rc->ch[8] = rc_from_crsf(buf->ch8);
    rc->ch[9] = rc_from_crsf(buf->ch9);
    rc->ch[10] = rc_from_crsf(buf->ch10);
    rc->ch[11] = rc_from_crsf(buf->ch11);
    rc->ch[12] = rc_from_crsf(buf->ch12);
    rc->ch[13] = rc_from_crsf(buf->ch13);
    rc->ch[14] = rc_from_crsf(buf->ch14);
    rc->ch[15] = rc_from_crsf(buf->ch15);

    if (is_32channels) {
        rc->ch[16] = rc_from_crsf(buf->ch16);
        rc->ch[17] = rc_from_crsf(buf->ch17);
        rc->ch[18] = rc_from_crsf(buf->ch18);
        rc->ch[19] = rc_from_crsf(buf->ch19);
        rc->ch[20] = rc_from_crsf(buf->ch20);
        rc->ch[21] = rc_from_crsf(buf->ch21);
        rc->ch[22] = rc_from_crsf(buf->ch22);
        rc->ch[23] = rc_from_crsf(buf->ch23);
        rc->ch[24] = rc_from_crsf(buf->ch24);
        rc->ch[25] = rc_from_crsf(buf->ch25);
        rc->ch[26] = rc_from_crsf(buf->ch26);
        rc->ch[27] = rc_from_crsf(buf->ch27);
        rc->ch[28] = rc_from_crsf(buf->ch28);
        rc->ch[29] = rc_from_crsf(buf->ch29);
        rc->ch[30] = rc_from_crsf(buf->ch30);
        rc->ch[31] = rc_from_crsf(buf->ch31);
    }

    return true;
}


bool tTxCrsf::fill_rcdata_0x17(tRcData* const rc)
{
tCrsfSubsetRcChannelsPacked_16x11bit* buf = (tCrsfSubsetRcChannelsPacked_16x11bit*)frame.payload;

    if (frame.len != 25 || frame.payload[0] != 0x30) { // we only accept 0x17 with 11 bit, 16 channels, ch 16 start
DBG_CRSF_32CH(dbg.puts(" err");)
        return false;
    }
DBG_CRSF_32CH(dbg.puts(" ok");)

    rc->do_32channels = true;

    rc->ch[16] = rc_from_crsf_0x17_11bit(buf->ch_16x11bit.ch0);
    rc->ch[17] = rc_from_crsf_0x17_11bit(buf->ch_16x11bit.ch1);
    rc->ch[18] = rc_from_crsf_0x17_11bit(buf->ch_16x11bit.ch2);
    rc->ch[19] = rc_from_crsf_0x17_11bit(buf->ch_16x11bit.ch3);
    rc->ch[20] = rc_from_crsf_0x17_11bit(buf->ch_16x11bit.ch4);
    rc->ch[21] = rc_from_crsf_0x17_11bit(buf->ch_16x11bit.ch5);
    rc->ch[22] = rc_from_crsf_0x17_11bit(buf->ch_16x11bit.ch6);
    rc->ch[23] = rc_from_crsf_0x17_11bit(buf->ch_16x11bit.ch7);
    rc->ch[24] = rc_from_crsf_0x17_11bit(buf->ch_16x11bit.ch8);
    rc->ch[25] = rc_from_crsf_0x17_11bit(buf->ch_16x11bit.ch9);
    rc->ch[26] = rc_from_crsf_0x17_11bit(buf->ch_16x11bit.ch10);
    rc->ch[27] = rc_from_crsf_0x17_11bit(buf->ch_16x11bit.ch11);
    rc->ch[28] = rc_from_crsf_0x17_11bit(buf->ch_16x11bit.ch12);
    rc->ch[29] = rc_from_crsf_0x17_11bit(buf->ch_16x11bit.ch13);
    rc->ch[30] = rc_from_crsf_0x17_11bit(buf->ch_16x11bit.ch14);
    rc->ch[31] = rc_from_crsf_0x17_11bit(buf->ch_16x11bit.ch15);

    return true;
}


uint8_t tTxCrsf::crc8(const uint8_t* const buf)
{
    return crsf_crc8_update(CRSF_CRC8_INIT, &(buf[2]), buf[1] - 1);
}


//-------------------------------------------------------
// CRSF user interface

void tTxCrsf::Init(bool enable_flag, bool crsfbridge_enable_flag)
{
    enabled = enable_flag;
    crsfbridge_enabled = (enabled) ? crsfbridge_enable_flag : false;

    if (!enabled) return;

    rx_len = 0;
    rx_cnt = 0;
    rx_tlast_us = 0;
    rx_frame_received = false;

    tx_available = 0;
    tx_free = false;

    startup_passed = false;

    channels_received = false;
    mbridge_cmd_received = false;
    ping_device_received = false;
    cmd_modelid_received = false;
    cmd_bind_set_received = false;
    cmd_bind_cancel_received = false;

    for (uint8_t i = 0; i < CRSF_ITEMS_LEN; i++) {
        crsf_status[i].updated = false;
        crsf_status[i].send_tlast_ms = 0;
    }
    gps_raw_int_sat = UINT8_MAX; // unknown
    gps2_raw_sat = UINT8_MAX; // unknown
    vfr_hud_groundspd_mps = NAN; // unknown

    passthrough.Init();

    inav_baro_altitude = 0;
    msp_inav_status_sensor_status = 0;
    msp_inav_status_arming_flags = 0;

    put_fifo.Init();
    get_fifo.Init();
    crsf_envelop_out_sequence = 0;
    crsf_envelop_use_mb = true;
    crsf_envelop_out_tlast_ms = 0;

    uart_rx_callback_ptr = &crsf_pin5_rx_callback;
    uart_tc_callback_ptr = &crsf_pin5_tc_callback;

    tPin5BridgeBase::Init();
    tSerialBase::Init();

    // needs to come after tPin5BridgeBase::Init() since it calls txclock.Init()
//    txclock.SetCC1Callback(crsf_pin5_cc1_callback);
}

// polled in main loop
void tTxCrsf::Do(void)
{
    if (!enabled) return;

    CheckAndRescue();

    if (!rx_frame_received) return;
    rx_frame_received = false;

    if (frame.frame_id == CRSF_FRAME_ID_RC_CHANNELS) { // len = 24 or 25
        // EdgeTx sets frame[0] = MODULE_ADDRESS
        channels_received = true;
    } else
    if (frame.frame_id == CRSF_FRAME_ID_SUBSET_RC_CHANNELS_PACKED) { // len = 25
        channels_received = true;

DBG_CRSF_32CH(dbg.puts(" c0x17 ");/*dbg.puts(u8toHEX_s(frame.c[4]));*/)

    } else
    if (crsfbridge_enabled &&
        frame.address == CRSF_ADDRESS_TRANSMITTER_MODULE && frame.frame_id == CRSF_FRAME_ID_MBRIDGE_TO_MODULE &&
        frame.payload[0] == CRSF_MB_ENVELOPE_CMD) { // 0xEE, len, 0x81, 0x66
        get_fifo.PutBuf(&frame.c[6], frame.c[5]);
        crsf_envelop_use_mb = true;

DBG_CRSF_ENVELOPE(dbg.puts("\nc rx ");dbg.puts(u8toHEX_s(frame.address));
dbg.puts(" ");dbg.puts(u8toBCD_s(frame.len));
dbg.puts(" ");dbg.puts(u8toHEX_s(frame.frame_id));
dbg.puts(" ");dbg.puts(u8toHEX_s(frame.c[3]));
dbg.puts(" ");dbg.puts(u8toBCD_s(frame.c[4] & 0x0F));
dbg.puts(" ");dbg.puts(u8toBCD_s(frame.c[5]));
dbg.puts(" ");dbg.puts(u8toHEX_s(frame.c[6]));)

    } else
    if (crsfbridge_enabled && frame.frame_id == CRSF_FRAME_ID_MAVLINK_ENVELOPE) {
        get_fifo.PutBuf(&frame.c[5], frame.c[4]);
        crsf_envelop_use_mb = false;

DBG_CRSF_ENVELOPE(dbg.puts("\nc rx ");dbg.puts(u8toHEX_s(frame.address));
dbg.puts(" ");dbg.puts(u8toBCD_s(frame.len));
dbg.puts(" ");dbg.puts(u8toHEX_s(frame.frame_id));
dbg.puts(" ");dbg.puts(u8toBCD_s((frame.c[3] >> 4) & 0x0F));
dbg.puts(" ");dbg.puts(u8toBCD_s(frame.c[4]));)

    } else
    if (frame.address == CRSF_OPENTX_SYNC && frame.frame_id == CRSF_FRAME_ID_PING_DEVICES) { // len = 4
        // EdgeTx sets frame[3] = BROADCAST_ADDRESS, frame[4] = RADIO_ADDRESS
        ping_device_received = true;
    } else
    if (frame.address == CRSF_OPENTX_SYNC && frame.frame_id == CRSF_FRAME_ID_COMMAND &&
        frame.cmd_id == CRSF_COMMAND_ID) {
        switch (frame.cmd_data[0]) {
        case CRSF_COMMAND_SET_BIND_MODE: // len = 7
            // EdgeTx sets frame[3] = MODULE_ADDRESS or RECEIVER_ADDRESS, frame[4] = RADIO_ADDRESS, frame[5] = SUBCOMMAND_CRSF
            if (frame.cmd_dest_address == CRSF_ADDRESS_TRANSMITTER_MODULE) cmd_bind_set_received = true;
        case CRSF_COMMAND_CANCEL_BIND_MODE: // len = 7
            // not used by EdgeTx
            if (frame.cmd_dest_address == CRSF_ADDRESS_TRANSMITTER_MODULE) cmd_bind_cancel_received = true;
        case CRSF_COMMAND_SET_MODEL_SELECTION: // len = 8
            // OpenTx/EdgeTx sets frame[3] = MODULE_ADDRESS, frame[4] = RADIO_ADDRESS, frame[5] = SUBCOMMAND_CRSF
            cmd_modelid_received = true;
            cmd_modelid_value = frame.cmd_data[1];
            break;
        }
    } else
    if (frame.address == CRSF_ADDRESS_TRANSMITTER_MODULE && frame.frame_id == CRSF_FRAME_ID_MBRIDGE_TO_MODULE) {
        mbridge_cmd_received = true;
    }
}


// polled in main loop
bool tTxCrsf::ChannelsUpdated(tRcData* const rc)
{
    if (!enabled) return false;

    if (!channels_received) return false;
    channels_received = false;

    // check crc before we accept it
    uint8_t crc = crc8(frame.c);
    if (crc != frame.c[frame.len + 1]) {
DBG_CRSF_32CH(dbg.puts(" crc err");)
      return false;
    }

    if (frame.frame_id == CRSF_FRAME_ID_SUBSET_RC_CHANNELS_PACKED) {
DBG_CRSF_32CH(dbg.puts(" 0x17");)
        return fill_rcdata_0x17(rc);
    }

    startup_passed = true;

    return fill_rcdata(rc);
}


// polled in main loop
bool tTxCrsf::TelemetryUpdate(uint8_t* const task, uint16_t frame_rate_ms)
{
    if (!enabled) return false;

    // check if we can transmit
    if (!tx_free) return false;
    tx_free = false;

    if (!startup_passed) return false; // not yet ready to send CRSF frames to the radio

    // check if we should restart telemetry sequence
    if (telemetry_start_next_tick) {
        telemetry_start_next_tick = false;

        // slow it down if frame time is too short
        if (frame_rate_ms <= 19) {
            static uint8_t cnt = 0;
            DECc(cnt, 2);
            if (!cnt) telemetry_state = 0;
        } else {
            telemetry_state = 0;
        }
    }

    // next slot
    uint8_t curr_telemetry_state = telemetry_state;
    telemetry_state++;

    // now determine what to transmit
    // frame rate is
    //   50 Hz:  20 ms -> ca 5  = 3 + 2
    //   31 Hz:  32 ms -> ca 8  = 3 + 5
    //   19 Hz:  53 ms -> ca 13 = 3 + 10
    //   111 Hz:  9 ms -> 3x = 27 ms -> ca 6 = 3 + 3
#ifndef CRSF_FRAME_BUNDLING
    // this is what it was before, keep it for now just in case
    switch (curr_telemetry_state) {
        case 0: *task = TXCRSF_SEND_LINK_STATISTICS; return true;
        case 1: *task = TXCRSF_SEND_LINK_STATISTICS_TX; return true;
        case 2: *task = TXCRSF_SEND_LINK_STATISTICS_RX; return true;
    }
#else
    if (curr_telemetry_state == 0) {
        *task = TXCRSF_SEND_LINK_STATISTICS_ALL;
        return true;
    }
#endif

    // if we got a PING_DEVICE, send a DEVICE_INFO instead of a telemetry frame
    if (ping_device_received) {
        ping_device_received = false;
        *task = TXCRSF_SEND_DEVICE_INFO;
        return true;
    }

    *task = TXCRSF_SEND_TELEMETRY_FRAME;
    return true;
}


// polled in main loop
bool tTxCrsf::CommandReceived(uint8_t* const cmd)
{
    if (!enabled) return false;

    if (cmd_modelid_received) {
        cmd_modelid_received = false;
        *cmd = TXCRSF_CMD_MODELID_SET;
        return true;
    }

    if (cmd_bind_set_received) {
        cmd_bind_set_received = false;
        *cmd = TXCRSF_CMD_BIND_START;
        return true;
    }

    if (cmd_bind_cancel_received) {
        cmd_bind_cancel_received = false;
        *cmd = TXCRSF_CMD_BIND_STOP;
        return true;
    }

    if (mbridge_cmd_received) {
        mbridge_cmd_received = false;
        // TODO: we could check crc if we wanted to
        *cmd = TXCRSF_CMD_MBRIDGE_IN;
        return true;
    }

    return false;
}


uint8_t* tTxCrsf::GetPayloadPtr(void)
{
    return frame.payload;
}


uint8_t tTxCrsf::GetPayloadLen(void)
{
    return frame.len - 2;
}


uint8_t tTxCrsf::GetCmdModelId(void)
{
    return cmd_modelid_value;
}


void tTxCrsf::SendMBridgeFrame(void* const payload, uint8_t payload_len)
{
    send_frame(CRSF_FRAME_ID_MBRIDGE_TO_RADIO, payload, payload_len);
}


void tTxCrsf::PassthroughSetBattery0Capacity(uint32_t capacity)
{
    crsf.passthrough.SetBattery0Capacity(capacity);
}


//-------------------------------------------------------
// helper

void tTxCrsf::send_frame(const uint8_t frame_id, void* const payload, uint8_t payload_len)
{
    tx_frame[0] = CRSF_ADDRESS_RADIO; // correct? OpenTx accepts CRSF_ADDRESS_RADIO or CRSF_OPENTX_SYNC, so correct
    tx_frame[1] = (4-2) + payload_len;
    tx_frame[2] = frame_id;
    memcpy(&(tx_frame[3]), payload, payload_len);
    tx_frame[3 + payload_len] = crc8(tx_frame);

    tx_available = 4 + payload_len;
}


//-------------------------------------------------------
// CRSF Telemetry Handler

// called in main loop, when crsf.TelemetryUpdate() true
void tTxCrsf::SendTelemetryFrame(void)
{
uint8_t data[CRSF_BUF_SIZE];
uint8_t len;

    // native CRSF

    uint32_t tnow_ms = millis32();

    #define CRSF_REFRESH_TIME_MS  2500 // what is actually a proper value ??

    // auto update to prevent EdgeTx/OpenTx telemetry/sensor lost message, do only if at least once seen
    for (uint8_t i = 0; i < CRSF_ITEMS_LEN; i++) {
        if (!crsf_status[i].send_tlast_ms) continue;
        if ((tnow_ms - crsf_status[i].send_tlast_ms) > CRSF_REFRESH_TIME_MS) { crsf_status[i].updated = true; }
    }

    // MAVLink envelope
    uint16_t available = put_fifo.Available();
    if (crsfbridge_enabled && (available > 20 || (tnow_ms - crsf_envelop_out_tlast_ms) > 9)) {
        if (crsf_envelop_use_mb) {
            crsf_envelope_out.mb.cmd = CRSF_MB_ENVELOPE_CMD;
            crsf_envelope_out.mb.seq = crsf_envelop_out_sequence;
            crsf_envelope_out.mb.data_size = 0;
            for (uint8_t i = 0; i < CRSF_MB_ENVELOPE_DATA_LEN_MAX; i++) { // 57
                if (!put_fifo.Available()) break;
                crsf_envelope_out.mb.data[i] = put_fifo.Get();
                crsf_envelope_out.mb.data_size++;
            }
            send_frame(
                CRSF_FRAME_ID_MBRIDGE_TO_RADIO, // 0xEA, len, 0x82, 0x66
                &(crsf_envelope_out),
                crsf_envelope_out.mb.data_size + 3);
        } else {
            crsf_envelope_out.mavlink.total_chunks = 0;
            crsf_envelope_out.mavlink.current_chunk = crsf_envelop_out_sequence;
            crsf_envelope_out.mavlink.data_size = 0;
            for (uint8_t i = 0; i < CRSF_MAVLINK_ENVELOPE_DATA_LEN_MAX; i++) { // 58
                if (!put_fifo.Available()) break;
                crsf_envelope_out.mavlink.data[i] = put_fifo.Get();
                crsf_envelope_out.mavlink.data_size++;
            }
            send_frame(
                CRSF_FRAME_ID_MAVLINK_ENVELOPE, // 0xEA, len, 0xAA
                &(crsf_envelope_out),
                crsf_envelope_out.mavlink.data_size + 2);
        }
        crsf_envelop_out_sequence++;
        crsf_envelop_out_tlast_ms = tnow_ms;

//dbg.puts("\nc tx ");dbg.puts(u8toHEX_s(tx_frame[0]));
//dbg.puts(" ");dbg.puts(u8toBCD_s(tx_frame[1]));
//dbg.puts(" ");dbg.puts(u8toHEX_s(tx_frame[2]));
        return; // send only one per slot
    }

    // one by one, order by desired priority
#ifndef CRSF_FRAME_BUNDLING
    for (uint8_t i = 0; i < CRSF_ITEMS_LEN; i++) {
        if (crsf_status[i].updated) {
            crsf_status[i].updated = false;
            crsf_status[i].send_tlast_ms = tnow_ms;
            send_frame(crsf_items[i].frame_id, crsf_items[i].payload_ptr, crsf_items[i].payload_len);
            return; // only send one per slot
        }
    }
#else
    len = 0;

    for (uint8_t i = 0; i < CRSF_ITEMS_LEN; i++) {
        if (!crsf_status[i].updated) continue;

        uint8_t payload_len = crsf_items[i].payload_len;
        if (len + payload_len >= CRSF_FRAME_LEN_MAX - 4 - 16) break; // no space left
        crsf_status[i].updated = false;
        crsf_status[i].send_tlast_ms = tnow_ms;

        send_frame(crsf_items[i].frame_id, crsf_items[i].payload_ptr, payload_len);
        memcpy(data + len, tx_frame, tx_available);
        len += tx_available;
    }

    if (len) {
        memcpy(tx_frame, data, len);
        tx_available = len;
        return; // only send one per slot
    }
#endif

    // passthrough

    if (passthrough.GetTelemetryFrameMulti(data, &len)) {
        send_frame(CRSF_FRAME_ID_AP_CUSTOM_TELEM, data, len);
        return;
    }
}


//-------------------------------------------------------
// CRSF Telemetry Mavlink Handling
// we have two kinds to consider:
// - native CRSF telemetry frames:
//   these are filled from MAVLink messages by the tTxCrsf class
// - passthrough packets which are packed into CRSF passthrough telemetry frames:
//   these are filled from MAVLink messages through the tPassThrough class

#define CRSF_REV_U16(x)  __REV16(x)
#define CRSF_REV_I16(x)  __REVSH(x)
#define CRSF_REV_U32(x)  __REV(x)


void tTxCrsf::handle_mavlink_msg_heartbeat(fmav_heartbeat_t* const payload)
{
    memset(flight_mode.flight_mode, 0, sizeof(flight_mode.flight_mode));

    if (payload->autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA) {
        ap_flight_mode_name4(flight_mode.flight_mode, ap_vehicle_from_mavtype(payload->type), payload->custom_mode);

        if ((payload->base_mode & MAV_MODE_FLAG_SAFETY_ARMED) == 0) {
            // if (flight_mode.flight_mode[3] == ' ') flight_mode.flight_mode[3] = '\0';
            strcat(flight_mode.flight_mode, "*");
        }
    }

    crsf_status[CRSF_ITEM_FLIGHT_MODE].updated = true;
}


void tTxCrsf::handle_mavlink_msg_system_time(fmav_system_time_t* const payload)
{
    // SYSTEM_TIME.time_unix_usec is != 0 if AP::rtc() gives a value, so not GPS time strictly
    if (payload->time_unix_usec == 0) return; // not available

    time_t time_unix = payload->time_unix_usec / 1000000; // standard unix time is in seconds since 1970
    struct tm* time_info = gmtime(&time_unix); // UTC

    gps_time.year = CRSF_REV_U16(time_info->tm_year + 1900); // EdgeTx since v2.12.?=? -> Date
    gps_time.month = time_info->tm_mon + 1;
    gps_time.day = time_info->tm_mday;
    gps_time.hour = time_info->tm_hour;
    gps_time.minute = time_info->tm_min;
    gps_time.second = time_info->tm_sec;
    gps_time.millisecond = CRSF_REV_U16((payload->time_unix_usec % 1000000) / 1000);

    crsf_status[CRSF_ITEM_GPS_TIME].updated = true;
}


int32_t mav_battery_voltage(fmav_battery_status_t* payload)
{
    int32_t voltage = 0;
    for (uint8_t i = 0; i < 10; i++) {
        if (payload->voltages[i] != UINT16_MAX) {
            voltage += payload->voltages[i]; // uint16_t mV, UINT16_MAX if not known
        }
    }
    for (uint8_t i = 0; i < 4; i++) { // we assume this never is relevant if validcellcount = false
        if (payload->voltages_ext[i] != 0) {
            voltage += payload->voltages_ext[i]; // uint16_t mV, 0 if not known
        }
    }
    return voltage;
}


void tTxCrsf::handle_mavlink_msg_battery_status(fmav_battery_status_t* const payload)
{
    if (payload->id != 0) return;

    battery.voltage = CRSF_REV_U16(mav_battery_voltage(payload) / 100);
    battery.current = CRSF_REV_U16((payload->current_battery == -1) ? 0 : payload->current_battery / 10); // CRSF is in 0.1 A, MAVLink is in 0.01 A
    uint32_t capacity = (payload->current_consumed < 0) ? 0 : payload->current_consumed; // -1 = unknown, but can become negative
    if (capacity > 8388607) capacity = 8388607; // int 24 bit
    battery.capacity[0] = (capacity >> 16);
    battery.capacity[1] = (capacity >> 8);
    battery.capacity[2] = capacity;
    battery.remaining = (payload->battery_remaining == -1) ? 0 : payload->battery_remaining;
    crsf_status[CRSF_ITEM_BATTERY].updated = true;
}


void tTxCrsf::handle_mavlink_msg_attitude(fmav_attitude_t* const payload)
{
    attitude.pitch = CRSF_REV_I16(10000.0f * payload->pitch);
    attitude.roll = CRSF_REV_I16(10000.0f * payload->roll);
    attitude.yaw = CRSF_REV_I16(10000.0f * payload->yaw);
    crsf_status[CRSF_ITEM_ATTITUDE].updated = true;
}


void tTxCrsf::handle_mavlink_msg_gps_raw_int(fmav_gps_raw_int_t* const payload)
{
    gps_raw_int_sat = payload->satellites_visible;
}


void tTxCrsf::handle_mavlink_msg_gps2_raw(fmav_gps2_raw_t* const payload)
{
    gps2_raw_sat = payload->satellites_visible;
}


void tTxCrsf::handle_mavlink_msg_global_position_int(fmav_global_position_int_t* const payload)
{
    gps.latitude = CRSF_REV_U32(payload->lat);
    gps.longitude = CRSF_REV_U32(payload->lon);
    int32_t alt = payload->alt / 1000 + 1000;
    if (alt < 0) alt = 0;
    if (alt > UINT16_MAX) alt = UINT16_MAX;
    gps.altitude = CRSF_REV_U16(alt);
    gps.gps_heading = CRSF_REV_U16(payload->hdg);

    // take the ground speed from VFR_HUD
    if (vfr_hud_groundspd_mps != NAN) {
        gps.groundspeed = CRSF_REV_U16(10.0f * vfr_hud_groundspd_mps * 3.6f); // TBS docs say 'km/h / 100' but seems to be 'km/h / 10'
    } else {
        gps.groundspeed = 0;
    }

    // take the satellites of the previous reports
    if (gps_raw_int_sat != UINT8_MAX && gps2_raw_sat != UINT8_MAX) { // we have two gps
        gps.satellites = (gps_raw_int_sat > gps2_raw_sat) ? gps_raw_int_sat : gps2_raw_sat; // take the larger
    } else
    if (gps_raw_int_sat != UINT8_MAX) {
        gps.satellites = gps_raw_int_sat;
    } else {
        gps.satellites = 0;
    }

    // mark as updated
    crsf_status[CRSF_ITEM_GPS].updated = true;
}


void tTxCrsf::handle_mavlink_msg_vfr_hud(fmav_vfr_hud_t* const payload)
{
    // MAVLink: m/s -> CRSF: 0.1 * km/h (hectometers/h)
    airspeed.speed = CRSF_REV_U16(36.0f * payload->airspeed);
    crsf_status[CRSF_ITEM_AIRSPEED].updated = true;

    // MAVLink: m/s
    vfr_hud_groundspd_mps = payload->groundspeed;

    // MAVLink: m/s -> CRSF: Vertical speed cm/s
    variometer.v_speed = CRSF_REV_I16(100.0f * payload->climb);
    crsf_status[CRSF_ITEM_VARIOMETER].updated = true;
}


void tTxCrsf::handle_mavlink_msg_scaled_pressure(fmav_scaled_pressure_t* const payload)
{
    // MAVLink: hPa -> CRSF: Pascals
    barometer.pressure_pa = CRSF_REV_U32(100.0f * payload->press_abs);
    // MAVLink: cdegC -> CRSF: centidegrees
    barometer.baro_temp = CRSF_REV_U32(payload->temperature);
    crsf_status[CRSF_ITEM_BAROMETER].updated = true;

    // MAVLink: cdegC -> CRSF: in deci-degree (tenths of a degree) Celsius (e.g., 250 = 25.0 Celsius, -50 = -5.0 Celsius)
    temp_ambient.temp_source_id = 1;
    temp_ambient.temperature = CRSF_REV_I16(0.1f * payload->temperature);
    crsf_status[CRSF_ITEM_TEMP].updated = true;

}


// called by MAVLink interface, when a MAVLink frame has been received
void tTxCrsf::TelemetryHandleMavlinkMsg(fmav_message_t* const msg)
{
    if (msg->sysid == 0) return; // this can't be anything meaningful

    if (msg->compid != MAV_COMP_ID_AUTOPILOT1) return;

    // from here on we only see the MAVLink messages from our vehicle

    switch (msg->msgid) {

    // these are for CRSF telemetry, some are also for passthrough

    case FASTMAVLINK_MSG_ID_HEARTBEAT: {
        fmav_heartbeat_t payload;
        fmav_msg_heartbeat_decode(&payload, msg);
        handle_mavlink_msg_heartbeat(&payload);
        passthrough.handle_mavlink_msg_heartbeat(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_SYSTEM_TIME: { // not used by passthrough
        fmav_system_time_t payload;
        fmav_msg_system_time_decode(&payload, msg);
        handle_mavlink_msg_system_time(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_BATTERY_STATUS: {
        fmav_battery_status_t payload;
        fmav_msg_battery_status_decode(&payload, msg);
        handle_mavlink_msg_battery_status(&payload);
        passthrough.handle_mavlink_msg_battery_status(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_ATTITUDE: {
        fmav_attitude_t payload;
        fmav_msg_attitude_decode(&payload, msg);
        handle_mavlink_msg_attitude(&payload);
        passthrough.handle_mavlink_msg_attitude(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_GPS_RAW_INT: {
        fmav_gps_raw_int_t payload;
        fmav_msg_gps_raw_int_decode(&payload, msg);
        handle_mavlink_msg_gps_raw_int(&payload);
        passthrough.handle_mavlink_msg_gps_raw_int(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_GPS2_RAW: { // not used by passthrough
        fmav_gps2_raw_t payload;
        fmav_msg_gps2_raw_decode(&payload, msg);
        handle_mavlink_msg_gps2_raw(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_VFR_HUD: {
        fmav_vfr_hud_t payload;
        fmav_msg_vfr_hud_decode(&payload, msg);
        handle_mavlink_msg_vfr_hud(&payload);
        passthrough.handle_mavlink_msg_vfr_hud(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        fmav_global_position_int_t payload;
        fmav_msg_global_position_int_decode(&payload, msg);
        handle_mavlink_msg_global_position_int(&payload);
        passthrough.handle_mavlink_msg_global_position_int(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_SCALED_PRESSURE: { // not used by passthrough
        fmav_scaled_pressure_t payload;
        fmav_msg_scaled_pressure_decode(&payload, msg);
        handle_mavlink_msg_scaled_pressure(&payload);
        }break;


    // these are for passthrough only

    // case FASTMAVLINK_MSG_ID_PARAM_VALUE, is handled by mavlink/vehicle class as needed

    case FASTMAVLINK_MSG_ID_SYS_STATUS: {
        fmav_sys_status_t payload;
        fmav_msg_sys_status_decode(&payload, msg);
        passthrough.handle_mavlink_msg_sys_status(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_RAW_IMU: {
        fmav_raw_imu_t payload;
        fmav_msg_raw_imu_decode(&payload, msg);
        passthrough.handle_mavlink_msg_raw_imu(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_MISSION_CURRENT: {
        fmav_mission_current_t payload;
        fmav_msg_mission_current_decode(&payload, msg);
        passthrough.handle_mavlink_msg_mission_current(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: {
        fmav_nav_controller_output_t payload;
        fmav_msg_nav_controller_output_decode(&payload, msg);
        passthrough.handle_mavlink_msg_nav_controller_output(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_TERRAIN_REPORT: {
        fmav_terrain_report_t payload;
        fmav_msg_terrain_report_decode(&payload, msg);
        passthrough.handle_mavlink_msg_terrain_report(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_FENCE_STATUS: {
        fmav_fence_status_t payload;
        fmav_msg_fence_status_decode(&payload, msg);
        passthrough.handle_mavlink_msg_fence_status(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_DISTANCE_SENSOR: {
        fmav_distance_sensor_t payload;
        fmav_msg_distance_sensor_decode(&payload, msg);
        passthrough.handle_mavlink_msg_distance_sensor(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_WIND: {
        fmav_wind_t payload;
        fmav_msg_wind_decode(&payload, msg);
        passthrough.handle_mavlink_msg_wind(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_RANGEFINDER: {
        fmav_rangefinder_t payload;
        fmav_msg_rangefinder_decode(&payload, msg);
        passthrough.handle_mavlink_msg_rangefinder(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_RPM: {
        fmav_rpm_t payload;
        fmav_msg_rpm_decode(&payload, msg);
        passthrough.handle_mavlink_msg_rpm(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_HOME_POSITION: {
        fmav_home_position_t payload;
        fmav_msg_home_position_decode(&payload, msg);
        passthrough.handle_mavlink_msg_home_position(&payload);
        }break;

    case FASTMAVLINK_MSG_ID_STATUSTEXT: {
        fmav_statustext_t payload;
        fmav_msg_statustext_decode(&payload, msg);
        passthrough.handle_mavlink_msg_statustext(&payload);
        }break;

    }
}


//-------------------------------------------------------
// CRSF Telemetry MSP Handling

#define DEG2RADF  1.745329252E-02f

int16_t wrap180_cdeg(int16_t angle_cdeg)
{
    while (angle_cdeg > 1800) { angle_cdeg -= 3600; }
    while (angle_cdeg < -1800) { angle_cdeg += 3600; }
    return angle_cdeg;
}


void tTxCrsf::TelemetryHandleMspMsg(msp_message_t* const msg)
{
    // conversions deduced from comparing
    //  src/main/fc/fc_msp.c for MSP units
    //  src/main/telemetry/crsf.c for CRSF telemetry units

    // to suppress sensor auto updating
    // assumes that this function is being called within 1500 ms
    if (!(msp_inav_status_sensor_status & (1 << INAV_SENSOR_STATUS_GPS))) crsf_status[CRSF_ITEM_GPS].send_tlast_ms = 0;
    if (!(msp_inav_status_sensor_status & (1 << INAV_SENSOR_STATUS_BARO))) crsf_status[CRSF_ITEM_BARO_ALTITUDE].send_tlast_ms = 0;

    switch (msg->function) {
#if 0 // was a test to see if one can provide more sensors for Yaapu
    case MSP_SONAR_ALTITUDE: { //
        tMspSonarAltitude* payload = (tMspSonarAltitude*)(msg->payload);
        if (msp_inav_status_sensor_status & (1 << INAV_SENSOR_STATUS_RANGEFINDER)) {
            // for the moment we do it by creating a fake mavlink message
            fmav_rangefinder_t p;
            p.distance = (float)((payload->rangefinder_altitude + 5) / 10); // float m <- uint32_t cm
            p.voltage = 0.0f;
            passthrough.handle_mavlink_msg_rangefinder(&p);
        }
        } break;
#endif

    case MSP_ATTITUDE: { // tCrsfAttitude, CRSF_FRAME_ID_ATTITUDE = 0x1E
        tMspAttitude* payload = (tMspAttitude*)(msg->payload);
        attitude.pitch = CRSF_REV_I16((DEG2RADF * 1000.0f) * wrap180_cdeg(payload->pitch)); // int16_t rad * 1e4  // cdeg -> rad * 1e4
        attitude.roll = CRSF_REV_I16((DEG2RADF * 1000.0f) * wrap180_cdeg(payload->roll));   // int16_t rad * 1e4  // cdeg -> rad * 1e4
        attitude.yaw = CRSF_REV_I16((DEG2RADF * 10000.0f) * wrap180_cdeg(payload->yaw));    // int16_t rad * 1e4  // deg -> rad * 1e4
        crsf_status[CRSF_ITEM_ATTITUDE].updated = true;
        }break;

    case MSP2_INAV_ANALOG: { // tCrsfBattery, CRSF_FRAME_ID_BATTERY = 0x08
        tMspInavAnalog* payload = (tMspInavAnalog*)(msg->payload);
        battery.voltage = CRSF_REV_U16(payload->battery_voltage / 10);  // uint16_t mV * 100      // uint16_t  seems to be 0.01 V
        battery.current = CRSF_REV_U16(payload->amperage / 10);         // uint16_t mA * 100      // uint16_t  send amperage in 0.01 A steps
        uint32_t capacity = payload->mAh_drawn;                         // uint8_t[3] mAh         // uint32_t  milliamp hours drawn from battery
        battery.capacity[0] = (capacity >> 16);
        battery.capacity[1] = (capacity >> 8);
        battery.capacity[2] = capacity;
        battery.remaining = payload->battery_percentage;                // uint8_t percent        // uint8_t
        crsf_status[CRSF_ITEM_BATTERY].updated = true;
        }break;

    case MSP_RAW_GPS: { // tCrsfGps, CRSF_FRAME_ID_GPS = 0x02
        if (!(msp_inav_status_sensor_status & (1 << INAV_SENSOR_STATUS_GPS))) break;
        tMspRawGps* payload = (tMspRawGps*)(msg->payload);
        gps.latitude = CRSF_REV_U32(payload->lat);                    // int32_t degree / 1e7           // uint32_t  1 / 10 000 000 deg
        gps.longitude = CRSF_REV_U32(payload->lon);                   // int32_t degree / 1e7           // uint32_t 1 / 10 000 000 deg
        gps.groundspeed = CRSF_REV_U16((payload->ground_speed * 36 + 50) / 100);  // uint16_t km/h / 100  // uint16_t  cm/s
        gps.gps_heading = CRSF_REV_U16(payload->ground_course * 10);  // uint16_t degree / 100          // uint16_t  degree*10
        // INAV wants the baro alt in the gps alt field
        //gps.altitude = CRSF_REV_U16(payload->alt + 1000);           // uint16_t meter - 1000m offset  // uint16_t  meters
        gps.altitude = CRSF_REV_U16(inav_baro_altitude / 100 + 1000);
        gps.satellites = payload->numSat;                             // uint8_t                        // uint8_t
        crsf_status[CRSF_ITEM_GPS].updated = true;
        }break;

    case MSP_ALTITUDE: {
        tMspAltitude* payload = (tMspAltitude*)(msg->payload);
        // tCrsfVariometer, CRSF_FRAME_ID_VARIOMETER = 0x07
        variometer.v_speed = CRSF_REV_I16(payload->estimated_velocity_z);   // int16_t cm/s    // int16_t  cm/s
        crsf_status[CRSF_ITEM_VARIOMETER].updated = true;
        // tCrsfBaroAltitude, CRSF_FRAME_ID_BARO_ALTITUDE = 0x09
        if (msp_inav_status_sensor_status & (1 << INAV_SENSOR_STATUS_BARO)) {
/*
            int32_t alt = payload->baro_altitude / 10 + 10000; // uint32_t seems to be cm, convert to dm - 1000m
            if (alt < 0) alt = 0;
            if (alt > 0x7FFF) alt = 0x7FFF; // 0x7FFF = 32767
            baro_altitude.altitude = CRSF_REV_U16(alt); // uint16_t dm -1000m if 0x8000 not set
            crsf_status[CRSF_ITEM_BARO_ALTITUDE].updated = true;
*/
            // INAV wants the baro alt in the gps alt field
            // so we store the baro altitude, and tell that gps is updated
            inav_baro_altitude = payload->baro_altitude;
            crsf_status[CRSF_ITEM_GPS].updated = true;
        }
        }break;

    case MSP2_INAV_STATUS: {
        tMspInavStatus* payload = (tMspInavStatus*)(msg->payload);
        // report it
        msp_inav_status_sensor_status = payload->sensor_status;
        msp_inav_status_arming_flags = payload->arming_flags;
        }break;

    case MSPX_STATUS: { // this is send by the rx shortly after MSP2_INAV_STATUS
        uint32_t flight_mode_flags = *(uint32_t*)(msg->payload);
        inav_flight_mode_str5(flight_mode.flight_mode, flight_mode_flags, msp_inav_status_arming_flags);
        crsf_status[CRSF_ITEM_FLIGHT_MODE].updated = true;
        }break;
    }
}


//-------------------------------------------------------
// CRSF Link Statistics

// on CRSF rssi
// rssi = 255 -> red in otx
//      = 130 -> -126 dB
//      = 129 -> -127 dB
//      = 128 -> -128 dB
//      = 127 ->  127dB
//      = 126 ->  126dB
// hmhm ...

// uplink:   Tx (tx -> rx)
// downlink: Rx (rx -> tx)
// somehow the OpenTx naming/usage doesn't make fully sense
// so we "correct" things here such that the names make sense, irrespective of uplink/downlink notation

void tTxCrsf::SendLinkStatistics(void)
{
tCrsfLinkStatistics clstats;

    clstats.uplink_rssi1 = crsf_cvt_rssi_tx(stats.received_rssi);           // OpenTX -> "1RSS"
    clstats.uplink_rssi2 = 0; // we don't know it                           // OpenTX -> "2RSS"
    clstats.uplink_LQ = stats.GetReceivedLQ_rc(); // this sets main rssi in OpenTx, 0 = resets main rssi   // OpenTx -> "RQly"
    clstats.uplink_snr = 0; // we don't know it                             // OpenTx -> "RSNR"
    clstats.active_antenna = stats.received_antenna;                        // OpenTx -> "ANT"
    clstats.mode = crsf_cvt_mode(Config.Mode);                              // OpenTx -> "RFMD"
    clstats.uplink_transmit_power = crsf_cvt_power(                         // OpenTx -> "TPw2"
        SX_OR_SX2(sx.RfPower_dbm(),sx2.RfPower_dbm())
        );

    clstats.downlink_rssi = crsf_cvt_rssi_tx(stats.GetLastRssi());          // OpenTx -> "TRSS"
    clstats.downlink_LQ = stats.GetLQ_serial();                             // OpenTx -> "TQly"
    clstats.downlink_snr = stats.GetLastSnr();                              // OpenTx -> "TSNR"

    // misuse 2RSS for reporting the MAVLink packet link quality
    clstats.uplink_rssi2 = stats.GetMavlinkLQ();

    send_frame(CRSF_FRAME_ID_LINK_STATISTICS, &clstats, CRSF_LINK_STATISTICS_LEN);
}


void tTxCrsf::SendLinkStatisticsTx(void)
{
tCrsfLinkStatisticsTx clstats;

    clstats.uplink_rssi = crsf_cvt_rssi_tx(stats.GetLastRssi());                  // ignored by OpenTx
    clstats.uplink_rssi_percent = crsf_cvt_rssi_percent(                          // OpenTx -> "TRSP" // ??? uplink but "T" ??
        stats.GetLastRssi(),
        SX_OR_SX2(sx.ReceiverSensitivity_dbm(),sx2.ReceiverSensitivity_dbm())
        );
    clstats.uplink_LQ = stats.GetLQ_serial();                                     // ignored by OpenTx
    clstats.uplink_snr = stats.GetLastSnr();                                      // ignored by OpenTx
    clstats.downlink_transmit_power = UINT8_MAX; // we don't know it              // OpenTx -> "RPWR"
    clstats.uplink_fps = crsf_cvt_fps(Config.Mode); // *10 in OpenTx              // OpenTx -> "TFPS"

    send_frame(CRSF_FRAME_ID_LINK_STATISTICS_TX, &clstats, CRSF_LINK_STATISTICS_TX_LEN);
}


void tTxCrsf::SendLinkStatisticsRx(void)
{
tCrsfLinkStatisticsRx clstats;

    clstats.downlink_rssi = crsf_cvt_rssi_tx(stats.received_rssi);                // ignored by OpenTx
    clstats.downlink_rssi_percent = crsf_cvt_rssi_percent(                        // OpenTx -> "RRSP" // ??? downlink but "R" ??
        stats.received_rssi,
        SX_OR_SX2(sx.ReceiverSensitivity_dbm(),sx2.ReceiverSensitivity_dbm())
        );
    clstats.downlink_LQ = stats.GetReceivedLQ_rc();                               // ignored by OpenTx
    clstats.downlink_snr = 0; // we don't know it                                 // ignored by OpenTx
    clstats.uplink_transmit_power = SX_OR_SX2(sx.RfPower_dbm(),sx2.RfPower_dbm());// OpenTx -> "TPWR"

    send_frame(CRSF_FRAME_ID_LINK_STATISTICS_RX, &clstats, CRSF_LINK_STATISTICS_RX_LEN);
}


uint32_t version_to_u32(uint32_t version)
{
    uint32_t major = version / 10000;
    version -= major * 10000;
    uint32_t minor = version / 100;
    version -= minor * 100;
    uint32_t patch = version;

    return (major << 16) + (minor << 8) + patch;
}


void tTxCrsf::SendDeviceInfo(void)
{
char buf[CRSF_BUF_SIZE]; // DEVICE_NAME is limited to 20 chars max, so this is plenty of space

    // extended frame, so need to send destination and origin addresses
    buf[0] = CRSF_ADDRESS_RADIO; // destination address, ignored by EdgeTx (ELRS uses BROADCAST)
    buf[1] = CRSF_ADDRESS_TRANSMITTER_MODULE; // origin address, EdgeTx looks for this

    strstrbufcpy(buf + 2, DEVICE_NAME, 20); // this fills max 21 bytes, EdgeTx is limiting name to 15 chars (16-1)
    uint8_t len = 2 + strlen(buf + 2) + 1;

    tCrsfDeviceInfoFragment* dvif_ptr = (tCrsfDeviceInfoFragment*)(buf + len);
    dvif_ptr->serial_number = 0x53524C6D; // EdgeTx digests it as 4 chars to identify ELRS, so let's set it to mLRS
    dvif_ptr->hardware_id = 54321; // TODO, we could use stm32 uid, as for hc04, but this we haven't currently for esp
    dvif_ptr->firmware_id = CRSF_REV_U32(version_to_u32(VERSION)); // EdgeTx is showing it as Vmaj.min.patch
    dvif_ptr->parameters_total = 0;
    dvif_ptr->parameter_version_number = 0;

    send_frame(CRSF_FRAME_ID_DEVICE_INFO, buf, len + CRSF_DEVICE_INFO_FRAGMENT_LEN);
}


void tTxCrsf::SendLinkStatisticsAll(void)
{
uint8_t data[CRSF_BUF_SIZE];
uint8_t len;

    SendLinkStatistics(); // 3 + 10 + 1 = 14
    memcpy(data, tx_frame, tx_available);
    len = tx_available;

    SendLinkStatisticsTx(); // 3 + 6 + 1 = 10
    memcpy(data + len, tx_frame, tx_available);
    len += tx_available;

    SendLinkStatisticsRx(); // 3 + 5 + 1 = 9
    memcpy(data + len, tx_frame, tx_available);
    len += tx_available;

    SendMbStatistics(); // 3 + 18 + 1 = 22
    memcpy(data + len, tx_frame, tx_available);
    len += tx_available;

    memcpy(tx_frame, data, len); // 14 + 10 + 9 + 22 = 55
    tx_available = len;
}


//-------------------------------------------------------
// CRSF Mb Statistics

CRSF_PACKED(
typedef struct
{
    uint8_t cmd; // always 0x65

    uint8_t connected : 1;
    uint8_t binding : 1;
    uint8_t dualband : 1;
    uint8_t rx_available : 1;
    uint8_t privacy : 2;
    uint8_t spare : 2;

    uint8_t rx_actual_diversity : 4;
    uint8_t tx_actual_diversity : 4;

    uint8_t receive_antenna : 1;
    uint8_t transmit_antenna : 1;
    uint8_t receiver_receive_antenna : 1;
    uint8_t receiver_transmit_antenna : 1;
    uint8_t spare2 : 4;

    int8_t rssi1_instantaneous;
    int8_t rssi2_instantaneous;
    int8_t receiver_rssi_instantaneous;

    uint8_t LQ_serial;
    uint8_t receiver_LQ_rc;
    uint8_t receiver_LQ_serial;

    uint32_t bytes_transmitted : 14;
    uint32_t bytes_received : 14;
    uint32_t spare3 : 4;

    uint32_t fhss1_curr_i : 5;
    uint32_t fhss1_cnt : 5;
    uint32_t fhss2_curr_i : 5;
    uint32_t fhss2_cnt : 5;
    uint32_t spare4 : 12;
}) tCrsfMbStatistics; // 18 bytes


void tTxCrsf::SendMbStatistics(void)
{
tCrsfMbStatistics lstats = {};

    lstats.cmd = 0x65;

    lstats.connected = connected();
    lstats.binding = bind.IsInBind();
    lstats.dualband = Config.IsDualBand;
    lstats.rx_available = SetupMetaData.rx_available;
    lstats.privacy = Setup.Common[Config.ConfigId].Privacy;

    lstats.rx_actual_diversity = SetupMetaData.rx_actual_diversity;
    lstats.tx_actual_diversity = Config.Diversity;

    lstats.receive_antenna = stats.last_antenna;
    lstats.transmit_antenna = stats.last_transmit_antenna;
    lstats.receiver_receive_antenna = stats.received_antenna;
    lstats.receiver_transmit_antenna = stats.received_transmit_antenna;

    lstats.rssi1_instantaneous = stats.last_rssi1;
    lstats.rssi2_instantaneous = stats.last_rssi2;
    lstats.receiver_rssi_instantaneous = stats.received_rssi;

    lstats.LQ_serial = stats.GetLQ_serial();
    lstats.receiver_LQ_rc = stats.GetReceivedLQ_rc();
    lstats.receiver_LQ_serial = stats.received_LQ_serial;

    lstats.bytes_transmitted = stats.bytes_transmitted.GetBytesPerSec();
    lstats.bytes_received = stats.bytes_received.GetBytesPerSec();

    lstats.fhss1_curr_i = stats.fhss_curr_i;
    lstats.fhss1_cnt = fhss.Cnt();
    lstats.fhss2_curr_i = 0;
    lstats.fhss2_cnt = 0;

    send_frame(CRSF_FRAME_ID_MBRIDGE_TO_RADIO, &lstats, sizeof(tCrsfMbStatistics));
}


#else

class tTxCrsf : public tSerialBase
{
  public:
    void Init(bool enable_flag, bool crsfbridge_enable_flag) {}
    bool Update(tRcData* const rc) { return false; }
    void TelemetryStart(void) {}
    bool TelemetryUpdate(uint8_t* const task, uint16_t frame_rate_ms) { return false; }
    void TelemetryHandleMavlinkMsg(fmav_message_t* const msg) {}
    void TelemetryHandleMspMsg(msp_message_t* const msg) {}

    void PassthroughSetBattery0Capacity(uint32_t capacity) {}
};

tTxCrsf crsf;

#endif // ifdef DEVICE_HAS_JRPIN5

#endif // CRSF_INTERFACE_TX_H

