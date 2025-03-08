//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// DroneCAN Interface RX Side
//*******************************************************
#ifndef DRONECAN_INTERFACE_RX_H
#define DRONECAN_INTERFACE_RX_H
#pragma once


#define DBG_DC(x) x // only temporary


#include "dronecan_interface_rx_types.h"

#ifdef DEVICE_HAS_DRONECAN

#include "../Common/hal/hal.h"
#include "../Common/thirdparty/stdstm32-can.h"

#ifndef DRONECAN_USE_RX_ISR
#error DRONECAN_USE_RX_ISR not defined !
#endif

#if FDCAN_IRQ_PRIORITY != DRONECAN_IRQ_PRIORITY
#error FDCAN_IRQ_PRIORITY not eq DRONECAN_IRQ_PRIORITY !
#endif

extern tRxMavlink mavlink;
extern tRxDroneCan dronecan;

extern uint16_t micros16(void);
extern volatile uint32_t millis32(void);
extern bool connected(void);
extern tStats stats;
extern tSetup Setup;
extern tGlobalConfig Config;

#define DRONECAN_PREFERRED_NODE_ID  68

#define CANARD_POOL_SIZE  4096

#define DRONECAN_BUF_SIZE  512 // needs to be larger than the largest DroneCAN frame size

CanardInstance canard;
uint8_t canard_memory_pool[CANARD_POOL_SIZE]; // doing this static leads to crash in full mLRS code !?


void dronecan_uid(uint8_t uid[DC_UNIQUE_ID_LEN])
{
    mcu_uid(uid); // fill first 12 bytes with UID
    uint32_t cpu_id = mcu_cpu_id(); // fill last bytes with cpu id, this idea is taken from ArduPilot. THX.
    memcpy(&uid[12], &cpu_id, 4);
}


// that's the same as used in fhss lib, is Microsoft Visual/Quick C/C++'s
uint16_t dronecan_prng(void)
{
static uint32_t _seed = 1234;
const uint32_t a = 214013;
const uint32_t c = 2531011;
const uint32_t m = 2147483648;

    _seed = (a * _seed + c) % m;
    return _seed >> 16;
}


// receive one frame, only called from isr context
void dronecan_receive_frames(void)
{
CanardCANFrame frame;

    while (1) {
        int16_t res = dc_hal_receive(&frame); // 0: no receive, 1: receive, <0: error
        if (res < 0) {
            DBG_DC(dbg.puts("\nERR: rec ");dbg.puts(s16toBCD_s(res));)
        }
        if (res <= 0) break; // no receive or error
        res = canardHandleRxFrame(&canard, &frame, micros64()); // 0: ok, <0: error
        return; // only do one
    }
}


// transmits all frames from the TX queue, for calling from non-isr context
void dronecan_process_tx_queue(void)
{
const CanardCANFrame* frame;

    while (1) {
        frame = canardPeekTxQueue(&canard);
        if (!frame) break; // no frame in tx queue
        int16_t res = dc_hal_transmit(frame, millis32());
        if (res != 0) { // successfully submitted or error, so drop the frame
            canardPopTxQueue(&canard);
        }
        return; // only do one
    }
}


// DroneCAN/Libcanard call back, forward declaration
bool dronecan_should_accept_transfer(
    const CanardInstance* const ins,
    uint64_t* const out_data_type_signature,
    uint16_t data_type_id,
    CanardTransferType transfer_type,
    uint8_t source_node_id);


// DroneCAN/Libcanard call back, forward declaration
void dronecan_on_transfer_received(CanardInstance* const ins, CanardRxTransfer* const transfer);


// CAN peripheral init, forward declaration
void can_init(void);


//-------------------------------------------------------
// RxDroneCan class implementation
//-------------------------------------------------------

void tRxDroneCan::Init(bool ser_over_can_enable_flag)
{
    tick_1Hz = 0;
    node_status_transfer_id = 0;
    rc_input.transfer_id = 0;
    rc_input.tlast_ms = 0;
    node_id_allocation.transfer_id = 0;
    node_id_allocation.send_next_request_at_ms = 0;
    node_id_allocation.unique_id_offset = 0;
    node_id_allocation.is_running = false;
    tunnel_targetted.transfer_id = 0;
    tunnel_targetted.to_fc_tlast_ms = 0;
    tunnel_targetted.server_node_id = 0;
    fifo_fc_to_ser.Flush();
    fifo_ser_to_fc.Flush();
    flex_debug.transfer_id = 0;

    tunnel_targetted_fc_to_ser_rate = 0;
    tunnel_targetted_ser_to_fc_rate = 0;
    tunnel_targetted_handle_rate = 0;
    tunnel_targetted_send_rate = 0;
    fifo_fc_to_ser_tx_full_error_cnt = 0;
    tunnel_targetted_error_cnt = 0;

    ser_over_can_enabled = ser_over_can_enable_flag;

    DBG_DC(dbg.puts("\n\n\nCAN init");)

    can_init();

    canardInit(
        &canard,                          // uninitialized library instance
        canard_memory_pool,               // raw memory chunk used for dynamic allocation
        sizeof(canard_memory_pool),       // size of the above, in bytes
        dronecan_on_transfer_received,    // callback, see CanardOnTransferReception
        dronecan_should_accept_transfer,  // callback, see CanardShouldAcceptTransfer
        nullptr);                         // user_reference, unused

    if (!ser_over_can_enabled) {
        // canardSetLocalNodeID(&canard, DRONECAN_PREFERRED_NODE_ID);
    } else {
        // ArduPilot's MAVLink via CAN seems to need a fixed node id, so don't do dynamic id allocation
        canardSetLocalNodeID(&canard, DRONECAN_PREFERRED_NODE_ID);
    }
    node_id_allocation.is_running = (canardGetLocalNodeID(&canard) == 0);

    int16_t res = set_can_filters();
    if (res < 0) {
        DBG_DC(dbg.puts("\nERROR: filter config failed");)
    }

    // it appears to not matter if first isr is enabled and then start, or vice versa
    res = dc_hal_enable_isr();
    if (res < 0) {
        DBG_DC(dbg.puts("\nERROR: can isr config failed");)
    }

    res = dc_hal_start();
    if (res < 0) {
        DBG_DC(dbg.puts("\nERROR: can start failed");)
    }

    DBG_DC(dbg.puts("\nCAN inited");)
}


void tRxDroneCan::Start(void)
{
    // Hum?? it somehow does not work to call dc_hal_enable_isr() here ??
    dc_hal_rx_flush();
    DBG_DC(dbg.puts("\nCAN started");)
}


bool tRxDroneCan::id_is_allcoated(void)
{
    return (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID); // a node id was set
}


int16_t tRxDroneCan::set_can_filters(void)
{
tDcHalAcceptanceFilterConfiguration filter_configs[2];
uint8_t filter_num = 0;

    if (!id_is_allcoated()) {
        // initialize filters as needed for node id allocation at startup, only accept
        // - DYNAMIC_NODE_ID_ALLOCATION broadcasts
        filter_configs[0].rx_fifo = DC_HAL_RX_FIFO0;
        filter_configs[0].id =
            DC_MESSAGE_TYPE_TO_CAN_ID(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID) |
            DC_SERVICE_NOT_MESSAGE_TO_CAN_ID(0x00);
        filter_configs[0].mask =
            DC_MESSAGE_TYPE_MASK | DC_SERVICE_NOT_MESSAGE_MASK;
        filter_num = 1;

    } else {
        // set reduced filters as needed for normal operation, only accept
        // - GETNODEINFO requests
        // - TUNNEL_TARGETTED broadcasts
        filter_configs[0].rx_fifo = DC_HAL_RX_FIFO0;
        filter_configs[0].id =
            DC_SERVICE_TYPE_TO_CAN_ID(UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_ID) |
            DC_REQUEST_NOT_RESPONSE_TO_CAN_ID(0x01) |
            DC_DESTINATION_ID_TO_CAN_ID(canardGetLocalNodeID(&canard)) |
            DC_SERVICE_NOT_MESSAGE_TO_CAN_ID(0x01);
        filter_configs[0].mask =
            DC_SERVICE_TYPE_MASK | DC_REQUEST_NOT_RESPONSE_MASK | DC_DESTINATION_ID_MASK | DC_SERVICE_NOT_MESSAGE_MASK;
        filter_num = 1;
        if (ser_over_can_enabled) { // we do accept tunnel targetted transfers
            filter_configs[0].rx_fifo = DC_HAL_RX_FIFO1;
            filter_configs[1].id =
                DC_MESSAGE_TYPE_TO_CAN_ID(UAVCAN_TUNNEL_TARGETTED_ID) |
                DC_SERVICE_NOT_MESSAGE_TO_CAN_ID(0x00);
            filter_configs[1].mask =
                DC_MESSAGE_TYPE_MASK | DC_SERVICE_NOT_MESSAGE_MASK;
            filter_num = 2;
        }
    }

    DBG_DC(dbg.puts("\nFilter");
    for (uint8_t n = 0; n < filter_num; n++) {
        dbg.puts("\n  id:   ");dbg.puts(u32toHEX_s(filter_configs[n].id));
        dbg.puts("\n  mask: ");dbg.puts(u32toHEX_s(filter_configs[n].mask));
    })

    return dc_hal_config_acceptance_filters(filter_configs, filter_num);
}


// This function is called at 1 ms rate from the main loop
void tRxDroneCan::Tick_ms(void)
{
    uint64_t tnow_us = micros64(); // call it every ms to ensure it is updated

    // do dynamic node allocation if still needed
    if (!dronecan.id_is_allcoated()) {
        node_id_allocation.is_running = true;
        if (millis32() > node_id_allocation.send_next_request_at_ms) {
            send_dynamic_node_id_allocation_request();
        }
        return;
    }
    if (node_id_allocation.is_running) {
        node_id_allocation.is_running = false;
        set_can_filters();
        return;
    }

    uint32_t tnow_ms = millis32();
    if (tunnel_targetted.server_node_id) { // don't send before we haven't gotten a tunnel.Targetted from the fc
        if (fifo_ser_to_fc.Available() > 0 || (tnow_ms - tunnel_targetted.to_fc_tlast_ms) > 500) {
            tunnel_targetted.to_fc_tlast_ms = tnow_ms;
            send_tunnel_targetted();
        }
    } else {
        tunnel_targetted.to_fc_tlast_ms = tnow_ms;
        // this is important
        // otherwise the fifo is pretty full and many CAN messages would be send, and the rx crashes
        // behaving badly, ok, but why does it crash ??
        fifo_ser_to_fc.Flush();
    }

    DECc(tick_1Hz, SYSTICK_DELAY_MS(1000));
    if (!tick_1Hz) {
        // purge transfers that are no longer transmitted. This can free up some memory.
        canardCleanupStaleTransfers(&canard, tnow_us);

        // emit node status message
        send_node_status();

DBG_DC(dbg.puts("\n fc->ser:   ");dbg.puts(u16toBCD_s(tunnel_targetted_fc_to_ser_rate));
DBG_DC(dbg.puts(" h: "));dbg.puts(u16toBCD_s(tunnel_targetted_handle_rate));
dbg.puts("\n ser->fc:   ");dbg.puts(u16toBCD_s(tunnel_targetted_ser_to_fc_rate));
DBG_DC(dbg.puts(" s: "));dbg.puts(u16toBCD_s(tunnel_targetted_send_rate));
tunnel_targetted_fc_to_ser_rate = 0;
tunnel_targetted_ser_to_fc_rate = 0;
tunnel_targetted_handle_rate = 0;
tunnel_targetted_send_rate = 0;
dbg.puts("\n   err tx_fifo, tt: ");dbg.puts(u16toBCD_s(fifo_fc_to_ser_tx_full_error_cnt));
dbg.puts(" ,  ");dbg.puts(u16toBCD_s(tunnel_targetted_error_cnt));
dbg.puts("\n        err dc sum: ");dbg.puts(u16toBCD_s(dc_hal_get_stats().error_sum_count));)
    }
}


void tRxDroneCan::Do(void)
{
    dronecan_receive_frames();
    dronecan_process_tx_queue();
}


//-------------------------------------------------------
// SendRcData
//-------------------------------------------------------

// rc_out is the rc data stored in out class
// so after handling of channel order and failsafes by out class.
void tRxDroneCan::SendRcData(tRcData* const rc_out, bool failsafe)
{
    if (!dronecan.id_is_allcoated()) return;

    uint32_t tnow_ms = millis32();
    if ((tnow_ms - rc_input.tlast_ms) < 19) return; // don't send too fast, DroneCAN is not for racing ...
    rc_input.tlast_ms = tnow_ms;

    uint8_t failsafe_mode = Setup.Rx.FailsafeMode;
    if (failsafe) {
        switch (failsafe_mode) {
        case FAILSAFE_MODE_NO_SIGNAL:
            // do not output anything, so jump out
            return;
        case FAILSAFE_MODE_CH1CH4_CENTER:
            // in this mode do not report bad signal
            failsafe = false;
            break;
        }
    }

    _p.rc_input.id = 0;
    _p.rc_input.status = 0;
    if (failsafe) _p.rc_input.status |= DRONECAN_SENSORS_RC_RCINPUT_STATUS_FAILSAFE;

    // this message's quality is used by ArduPilot for setting rssi (not LQ)
    // it goes from 0 ... 255
    // so we use the same conversion as in e.g. RADIO_STATUS, so that ArduPilot shows us (nearly) the same value

#define DRONECAN_SENSORS_RC_RCINPUT_STATUS_QUALITY_TYPE  28 // 4+8+16

#define DRONECAN_SENSORS_RC_RCINPUT_QUALITY_TYPE_RSSI  0
#define DRONECAN_SENSORS_RC_RCINPUT_QUALITY_TYPE_LQ_ACTIVE_ANTENNA  4
#define DRONECAN_SENSORS_RC_RCINPUT_QUALITY_TYPE_RSSI_DBM  8
#define DRONECAN_SENSORS_RC_RCINPUT_QUALITY_TYPE_SNR  12
#define DRONECAN_SENSORS_RC_RCINPUT_QUALITY_TYPE_TX_POWER  16

    _p.rc_input.quality = 0;
    if (connected()) {
        if (mavlink.autopilot.HasDroneCanExtendedRcStats()) {
            // send LQ, RSSI_DBM, SNR round robin
            static uint8_t slot = UINT8_MAX;
            INCc(slot, 3);
            if (slot == 1) { // LQ
                _p.rc_input.quality = stats.GetLQ_rc();
                if (stats.last_antenna == ANTENNA_2) _p.rc_input.quality |= 0x80;
                _p.rc_input.status |= DRONECAN_SENSORS_RC_RCINPUT_QUALITY_TYPE_LQ_ACTIVE_ANTENNA;
            } else
            if (slot == 2) { // SNR or TX_POWER
                static int8_t power_dbm_last = 125;
                static uint32_t tlast_ms = 0;
                uint32_t tnow_ms = millis32();
                int8_t power_dbm = sx.RfPower_dbm();
                if ((tnow_ms - tlast_ms > 2500) || (power_dbm != power_dbm_last)) {
                    tlast_ms = tnow_ms;
                    power_dbm_last = power_dbm;
                    _p.rc_input.quality = dronecan_cvt_power(power_dbm);
                    _p.rc_input.status |= DRONECAN_SENSORS_RC_RCINPUT_QUALITY_TYPE_TX_POWER;
                } else {
                    _p.rc_input.quality = 128 + stats.GetLastSnr();
                    _p.rc_input.status |= DRONECAN_SENSORS_RC_RCINPUT_QUALITY_TYPE_SNR;
                }
            } else { // slot 0: RSSI_DBM
                _p.rc_input.quality = crsf_cvt_rssi_rx(stats.GetLastRssi());
                _p.rc_input.status |= DRONECAN_SENSORS_RC_RCINPUT_QUALITY_TYPE_RSSI_DBM;
            }
        } else {
            // just send RSSI
            _p.rc_input.quality = rssi_i8_to_ap(stats.GetLastRssi());
        }
        _p.rc_input.status |= DRONECAN_SENSORS_RC_RCINPUT_STATUS_QUALITY_VALID;
    }

    _p.rc_input.rcin.len = 16;
    for (uint8_t i = 0; i < 16; i++) {
        // to get the same as mavlink rc we have
        // pwm = [ (rc-1024)*15/4 ] * 5/32 + 1500 = (rc - 1024) * 75 / 128 + 1500
        // in order to get the full range we x8 sso we can add +1 to the multiplier
        _p.rc_input.rcin.data[i] = (((int32_t)(rc_out->ch[i]) - 1024) * 601) / 1024 + 1500;
    }

    uint16_t len = dronecan_sensors_rc_RCInput_encode(&_p.rc_input, _buf);

    canardBroadcast(
        &canard,
        DRONECAN_SENSORS_RC_RCINPUT_SIGNATURE,
        DRONECAN_SENSORS_RC_RCINPUT_ID,
        &rc_input.transfer_id,
        CANARD_TRANSFER_PRIORITY_HIGH,
        _buf,
        len);

#if 0
    _p.flex_debug.id = 110; // we just grab it, PR is pending
    memset(_p.flex_debug.u8.data, 0, 255);

    static uint8_t cnt = 0;
    _p.flex_debug.u8.data[0] = cnt++;
    _p.flex_debug.u8.len = 1;

    len = dronecan_protocol_FlexDebug_encode(&_p.flex_debug, _buf);

    canardBroadcast(
        &canard,
        DRONECAN_PROTOCOL_FLEXDEBUG_SIGNATURE,
        DRONECAN_PROTOCOL_FLEXDEBUG_ID,
        &flex_debug.transfer_id,
        CANARD_TRANSFER_PRIORITY_MEDIUM,
        _buf,
        len);
#endif
}


//-------------------------------------------------------
// serial interface
//-------------------------------------------------------

void tRxDroneCan::putbuf(uint8_t* const buf, uint16_t len)
{
    fifo_ser_to_fc.PutBuf(buf, len);
    tunnel_targetted_ser_to_fc_rate += len;
}


bool tRxDroneCan::available(void)
{
    return (fifo_fc_to_ser.Available() > 0);
}


uint8_t tRxDroneCan::getc(void)
{
    tunnel_targetted_fc_to_ser_rate++;
    return fifo_fc_to_ser.Get();
}


void tRxDroneCan::flush(void)
{
    fifo_fc_to_ser.Flush();
    fifo_ser_to_fc.Flush();
}


uint16_t tRxDroneCan::bytes_available(void)
{
    return fifo_fc_to_ser.Available();
}


//-------------------------------------------------------
// DroneCAN default services
//-------------------------------------------------------

// Send NodeStatus message
// This allows node to show up in the DroneCAN GUI tool and in the flight controller logs
void tRxDroneCan::send_node_status(void)
{
    _p.node_status.uptime_sec = millis32() / 1000;
    _p.node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    _p.node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    _p.node_status.sub_mode = 0;

    // put something in vendor specific status, simply count up
    static uint16_t cnt = 0;
    _p.node_status.vendor_specific_status_code = cnt;
    cnt++;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&_p.node_status, _buf);

    canardBroadcast(
        &canard,
        UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
        UAVCAN_PROTOCOL_NODESTATUS_ID,
        &node_status_transfer_id,
        CANARD_TRANSFER_PRIORITY_LOW,
        _buf,
        len);
}


// Handle a GetNodeInfo request, and send a response
void tRxDroneCan::handle_get_node_info_request(CanardInstance* const ins, CanardRxTransfer* const transfer)
{
    _p.node_info_resp.status.uptime_sec = millis32() / 1000;
    _p.node_info_resp.status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    _p.node_info_resp.status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    _p.node_info_resp.status.sub_mode = 0;
    _p.node_info_resp.status.vendor_specific_status_code = 0;

    uint32_t version = VERSION;
    uint32_t major = version / 10000;
    version -= major * 10000;
    uint32_t minor = version / 100;
    version -= minor * 100;
    uint32_t patch = version;

    _p.node_info_resp.software_version.major = major;
    _p.node_info_resp.software_version.minor = minor;
    _p.node_info_resp.software_version.optional_field_flags = patch;
    _p.node_info_resp.software_version.vcs_commit = 0; // should put git hash in here

    _p.node_info_resp.hardware_version.major = 0; // we don't have such a thing
    _p.node_info_resp.hardware_version.minor = 0;

    dronecan_uid(_p.node_info_resp.hardware_version.unique_id);

    // data can be 80 chars, which is always larger than our device name, so no need to worry about too long string
    strcpy((char*)_p.node_info_resp.name.data, "mlrs.");
    strcat((char*)_p.node_info_resp.name.data, DEVICE_NAME);
    for (uint8_t n = 0; n < strlen((char*)_p.node_info_resp.name.data); n ++) {
        if (_p.node_info_resp.name.data[n] == ' ') _p.node_info_resp.name.data[n] = '_';
        if (_p.node_info_resp.name.data[n] >= 'A' && _p.node_info_resp.name.data[n] <= 'Z') {
            _p.node_info_resp.name.data[n] = _p.node_info_resp.name.data[n] - 'A' + 'a';
        }
    }
    _p.node_info_resp.name.len = strlen((char*)_p.node_info_resp.name.data);

    uint16_t len = uavcan_protocol_GetNodeInfoResponse_encode(&_p.node_info_resp, _buf);

    canardRequestOrRespond(
        ins,
        transfer->source_node_id,
        UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
        UAVCAN_PROTOCOL_GETNODEINFO_ID,
        &transfer->transfer_id,
        transfer->priority,
        CanardResponse,
        _buf,
        len);
}


// The next two functions for dynamic node id allocation VERY closely follow an example source which
// was provided by the UAVACN and libcanard projects in around 2017. The original sources seem to not be
// available anymore. The license was almost surely permissive (MIT?) and the author Pavel Kirienko. We
// apologize for not giving more appropriate credit.

// Handle a dynamic node allocation message
void tRxDroneCan::handle_dynamic_node_id_allocation_broadcast(CanardInstance* const ins, CanardRxTransfer* const transfer)
{
    // Dynamic node ID allocation protocol.
    // Taking this branch only if we don't have a node ID, ignoring otherwise.

    // Rule C - updating the randomized time interval
    node_id_allocation.send_next_request_at_ms =
        millis32() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (dronecan_prng() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID) {
        node_id_allocation.unique_id_offset = 0;
        return;
    }

    // Copying the unique ID from the message
    struct uavcan_protocol_dynamic_node_id_Allocation payload;
    if (uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &payload)) {
        return; // something went wrong
    }

    // Obtaining the local unique ID
    uint8_t my_uid[DC_UNIQUE_ID_LEN];
    dronecan_uid(my_uid);

    // Matching the received UID against the local one
    if (memcmp(payload.unique_id.data, my_uid, payload.unique_id.len) != 0) {
        node_id_allocation.unique_id_offset = 0;
        return; // No match, return
    }

    if (payload.unique_id.len < DC_UNIQUE_ID_LEN) {
        // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
        node_id_allocation.unique_id_offset = payload.unique_id.len;
        node_id_allocation.send_next_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;
    } else {
        // Allocation complete - copying the allocated node ID from the message
        canardSetLocalNodeID(ins, payload.node_id);
    }
}


// Request a dynamic node allocation
void tRxDroneCan::send_dynamic_node_id_allocation_request(void)
{
    node_id_allocation.send_next_request_at_ms =
        millis32() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (dronecan_prng() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    allocation_request[0] = DRONECAN_PREFERRED_NODE_ID << 1;

    if (node_id_allocation.unique_id_offset == 0) {
        allocation_request[0] |= 1; // First part of unique ID
    }

    uint8_t my_uid[DC_UNIQUE_ID_LEN];
    dronecan_uid(my_uid);

    uint8_t uid_size = (uint8_t)(DC_UNIQUE_ID_LEN - node_id_allocation.unique_id_offset);
    if (uid_size > UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST) {
        uid_size = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST;
    }

    memmove(&allocation_request[1], &my_uid[node_id_allocation.unique_id_offset], uid_size);

    // Broadcasting the request
    canardBroadcast(
        &canard,
        UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
        UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
        &node_id_allocation.transfer_id,
        CANARD_TRANSFER_PRIORITY_LOW,
        allocation_request,
        uid_size + 1);

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    node_id_allocation.unique_id_offset = 0;
}


//-------------------------------------------------------
// DroneCAN tunnel handling
//-------------------------------------------------------

// Handle a tunnel.Targetted message, check if it is for us and proper
void tRxDroneCan::handle_tunnel_targetted_broadcast(CanardRxTransfer* const transfer)
{
    if (!dronecan.id_is_allcoated()) { // this should never happen, but play it safe
        tunnel_targetted_error_cnt++;
        return;
    }

    if (uavcan_tunnel_Targetted_decode(transfer, &_p.tunnel_targetted)) { // something is wrong here
        tunnel_targetted_error_cnt++;
        return;
    }

    if (transfer->source_node_id == 0) { // this should never happen, but play it safe
        tunnel_targetted_error_cnt++;
        return;
    }

    // must be targeted at us
    if (_p.tunnel_targetted.target_node != canardGetLocalNodeID(&canard)) {
        return;
    }

    // we expect serial_id to be 0
    // TODO: should we store it and reuse whatever we got when sending?
    if (_p.tunnel_targetted.serial_id != 0) {
        return;
    }

    if (SERIAL_LINK_MODE_IS_MAVLINK(Setup.Rx.SerialLinkMode)) {
        // ArduPilot <= v4.5.x doesn't set protocol correctly, so we cannot generally check
        // -> we check only when it is set
        // when v4.6 is out, we could require users having to use v4.6, by mandating also correct protocol
        if (_p.tunnel_targetted.protocol.protocol != UAVCAN_TUNNEL_PROTOCOL_UNDEFINED &&
            _p.tunnel_targetted.protocol.protocol != UAVCAN_TUNNEL_PROTOCOL_MAVLINK2) {
            tunnel_targetted_error_cnt++;
            return;
        }
    } else
    if (SERIAL_LINK_MODE_IS_MSP(Setup.Rx.SerialLinkMode)) {
        if (_p.tunnel_targetted.protocol.protocol != UAVCAN_TUNNEL_PROTOCOL_UNDEFINED) {
            tunnel_targetted_error_cnt++;
            return;
        }
    }

    // memorize the node_id of the sender, this is most likely our fc (hopefully true)
    // just always respond to whoever sends to us
    if (tunnel_targetted.server_node_id && tunnel_targetted.server_node_id != transfer->source_node_id) {
        tunnel_targetted_error_cnt++; // not actually an error, we count it here for testing
    }
    tunnel_targetted.server_node_id = transfer->source_node_id;

    if (_p.tunnel_targetted.buffer.len == 0) return; // a short cut

    if (fifo_fc_to_ser.IsFull() || !fifo_fc_to_ser.HasSpace(_p.tunnel_targetted.buffer.len)) {
        fifo_fc_to_ser_tx_full_error_cnt++;
        return; // don't try to put into fifo
    }

    fifo_fc_to_ser.PutBuf(_p.tunnel_targetted.buffer.data, _p.tunnel_targetted.buffer.len);

    tunnel_targetted_handle_rate += _p.tunnel_targetted.buffer.len;
}


// Send a tunnel.Targetted message to the node which hopefully is the flight controller
void tRxDroneCan::send_tunnel_targetted(void)
{
    _p.tunnel_targetted.target_node = tunnel_targetted.server_node_id;
    if (SERIAL_LINK_MODE_IS_MAVLINK(Setup.Rx.SerialLinkMode)) {
        _p.tunnel_targetted.protocol.protocol = UAVCAN_TUNNEL_PROTOCOL_MAVLINK2;
    } else {
        _p.tunnel_targetted.protocol.protocol = UAVCAN_TUNNEL_PROTOCOL_UNDEFINED;
    }
    _p.tunnel_targetted.serial_id = 0;
    _p.tunnel_targetted.options = UAVCAN_TUNNEL_TARGETTED_OPTION_LOCK_PORT;
    _p.tunnel_targetted.baudrate = Config.SerialBaudrate; // this is ignored by ArduPilot (as it should)

    uint16_t data_len = fifo_ser_to_fc.Available();
    _p.tunnel_targetted.buffer.len = (data_len < 120) ? data_len : 120;
    for (uint8_t n = 0; n < data_len; n++) _p.tunnel_targetted.buffer.data[n] = fifo_ser_to_fc.Get();

    uint16_t len = uavcan_tunnel_Targetted_encode(&_p.tunnel_targetted, _buf);

    canardBroadcast(
        &canard,
        UAVCAN_TUNNEL_TARGETTED_SIGNATURE,
        UAVCAN_TUNNEL_TARGETTED_ID,
        &tunnel_targetted.transfer_id,
        CANARD_TRANSFER_PRIORITY_MEDIUM,
        _buf,
        len);

    tunnel_targetted_send_rate += _p.tunnel_targetted.buffer.len;
}


//-------------------------------------------------------
// DroneCAN/Libcanard call backs
//-------------------------------------------------------

// This callback is invoked when it detects beginning of a new transfer on the bus that can be
// received by our node.
// Return value:
//   true: library will accept the transfer
//   false: library will ignore the transfer.
// This function must fill in the out_data_type_signature to be the signature of the message.
bool dronecan_should_accept_transfer(
    const CanardInstance* const ins,
    uint64_t* const out_data_type_signature,
    uint16_t data_type_id,
    CanardTransferType transfer_type,
    uint8_t source_node_id)
{
    // handle service requests
    if (transfer_type == CanardTransferTypeRequest) {
        switch (data_type_id) {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID:
            if (!dronecan.id_is_allcoated()) return false;
            *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
            return true;
        }
    }
    // handle broadcast
    if (transfer_type == CanardTransferTypeBroadcast) {
        switch (data_type_id) {
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
            if (dronecan.id_is_allcoated()) return false; // we are already done with node id allocation
            *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
            return true;
        case UAVCAN_TUNNEL_TARGETTED_ID:
            if (!dronecan.ser_over_can_enabled) return false; // should not happen, play it safe
            if (!dronecan.id_is_allcoated()) return false;
            *out_data_type_signature = UAVCAN_TUNNEL_TARGETTED_SIGNATURE;
            return true;
        }
    }
    return false;
}


// This callback is invoked when a new message or request or response is received
void dronecan_on_transfer_received(CanardInstance* const ins, CanardRxTransfer* const transfer)
{
    // handle service requests
    if (transfer->transfer_type == CanardTransferTypeRequest) {
        switch (transfer->data_type_id) {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID:
            dronecan.handle_get_node_info_request(ins, transfer);
            return;
        }
    }
    // handle broadcasts
    if (transfer->transfer_type == CanardTransferTypeBroadcast) {
        switch (transfer->data_type_id) {
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
            dronecan.handle_dynamic_node_id_allocation_broadcast(ins, transfer);
            return;
        case UAVCAN_TUNNEL_TARGETTED_ID:
            dronecan.handle_tunnel_targetted_broadcast(transfer);
            return;
        }
    }
}


//-------------------------------------------------------
//-- check some sizes
//-------------------------------------------------------

STATIC_ASSERT(DRONECAN_BUF_SIZE >= UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE, "DRONECAN_BUF_SIZE too small")
STATIC_ASSERT(DRONECAN_BUF_SIZE >= UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE, "DRONECAN_BUF_SIZE too small")
STATIC_ASSERT(DRONECAN_BUF_SIZE >= DRONECAN_SENSORS_RC_RCINPUT_MAX_SIZE, "DRONECAN_BUF_SIZE too small")
STATIC_ASSERT(DRONECAN_BUF_SIZE >= UAVCAN_TUNNEL_TARGETTED_MAX_SIZE, "DRONECAN_BUF_SIZE too small")


#endif // DEVICE_HAS_DRONECAN

#endif // DRONECAN_INTERFACE_RX_H
