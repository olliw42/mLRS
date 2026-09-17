//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// MBridge Interface Header
//********************************************************
#ifndef MBRIDGE_INTERFACE_H
#define MBRIDGE_INTERFACE_H
#pragma once


#ifdef DEVICE_HAS_JRPIN5

#include "../Common/libs/fifo.h"
#include "setup_tx.h"
#include "../Common/protocols/mbridge_protocol.h"


extern uint16_t micros16(void);
extern volatile uint32_t millis32(void);
extern bool connected(void);
extern uint8_t mavlink_vehicle_state(void);
extern tStats stats;


//-------------------------------------------------------
// Interface Implementation

class tMBridge
{
  public:
    void Init(bool crsf_emulation_flag);

    bool CommandReceived(uint8_t* const cmd);
    uint8_t* GetPayloadPtr(void);
    uint8_t GetModelId(void);
    void SendCommand(uint8_t cmd, uint8_t* const payload);
    bool CommandInFifo(uint8_t* const cmd);
    void Lock(uint8_t cmd);
    void Unlock(void);
    void HandleRequestCmd(uint8_t* const payload);
    void HandleCmd(uint8_t cmd);

    void ParseCrsfFrame(uint8_t* const crsf, uint8_t len);
    bool CrsfFrameAvailable(uint8_t** const buf, uint8_t* const len);
    void parse_nextchar(uint8_t c);

    bool enabled;
    bool crsf_emulation;

    typedef enum {
        STATE_IDLE = 0,
        STATE_RECEIVE_MBRIDGE_STX2,
        STATE_RECEIVE_MBRIDGE_LEN,
        STATE_RECEIVE_MBRIDGE_SERIALPACKET,
        STATE_RECEIVE_MBRIDGE_CHANNELPACKET,
        STATE_RECEIVE_MBRIDGE_COMMANDPACKET,
        STATE_TRANSMIT_START,
    } STATE_ENUM;

    uint8_t state;
    uint8_t len;
    uint8_t cnt;
    uint16_t tlast_us;

    uint8_t cmd_r2m_frame[MBRIDGE_R2M_COMMAND_FRAME_LEN_MAX];
    volatile bool cmd_received;

    uint8_t cmd_m2r_frame[MBRIDGE_M2R_COMMAND_FRAME_LEN_MAX];
    volatile uint8_t cmd_m2r_available;

    // for communication
    tFifo<uint8_t,128> cmd_fifo; // TODO: how large does it really need to be?
    uint8_t cmd_in_process;
    uint32_t cmd_processed_tlast_ms;
    uint8_t ack_cmd;
    bool ack_ok;

    // momentarily for debug, detect discarded bytes
#ifdef USE_DEBUG
    uint16_t discarded = 0;
#endif
};

tMBridge mbridge;


//-------------------------------------------------------
// MBridge parser

#define MBRIDGE_TMO_US  250


// is called in ParseCrsfFrame() for CRSF emulation
void tMBridge::parse_nextchar(uint8_t c)
{
    uint16_t tnow_us = micros16();

    if (state != STATE_IDLE) {
        uint16_t dt = tnow_us - tlast_us;
        if (dt > MBRIDGE_TMO_US) state = STATE_IDLE; // timeout error
    }

    tlast_us = tnow_us;

    switch (state) {
    case STATE_IDLE:
        if (c == MBRIDGE_STX1) {
            state = STATE_RECEIVE_MBRIDGE_STX2;
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

    case STATE_RECEIVE_MBRIDGE_STX2:
        if (c == MBRIDGE_STX2) state = STATE_RECEIVE_MBRIDGE_LEN; else state = STATE_IDLE; // error
        break;
    case STATE_RECEIVE_MBRIDGE_LEN:
        cnt = 0;
        if (c == MBRIDGE_CHANNELPACKET_STX) {
            len = MBRIDGE_CHANNELPACKET_SIZE;
            state = STATE_RECEIVE_MBRIDGE_CHANNELPACKET;
        } else
        if (c >= MBRIDGE_COMMANDPACKET_STX) {
            uint8_t cmd = c & (~MBRIDGE_COMMANDPACKET_MASK);
            cmd_r2m_frame[cnt++] = cmd;
            len = mbridge_cmd_payload_len(cmd);
            if (len == 0) {
                cmd_received = true;
                state = STATE_TRANSMIT_START;
            } else {
                state = STATE_RECEIVE_MBRIDGE_COMMANDPACKET;
            }
        } else
        if (c > MBRIDGE_R2M_SERIAL_PAYLOAD_LEN_MAX) {
            state = STATE_IDLE; // error
        } else
        if (c > 0) {
            len = c;
            state = STATE_RECEIVE_MBRIDGE_SERIALPACKET;
        } else {
            state = STATE_TRANSMIT_START; // tx_len = 0, no payload
        }
        break;
    case STATE_RECEIVE_MBRIDGE_SERIALPACKET:
        cnt++;
        if (cnt >= len) state = STATE_TRANSMIT_START;
        break;
    case STATE_RECEIVE_MBRIDGE_CHANNELPACKET:
        if (cnt >= len) {
            state = STATE_TRANSMIT_START;
        }
        break;
    case STATE_RECEIVE_MBRIDGE_COMMANDPACKET:
        cmd_r2m_frame[cnt++] = c;
        if (cnt >= len + 1) {
            cmd_received = true;
            state = STATE_TRANSMIT_START;
        }
        break;
    }
}


//-------------------------------------------------------
// CRSF MBridge emulation

void tMBridge::ParseCrsfFrame(uint8_t* const crsf, uint8_t len)
{
    if (!crsf_emulation) return;

    state = STATE_IDLE; // to start the parser, also resets time gap check

    for (uint8_t i = 0; i < len; i++) parse_nextchar(crsf[i]);

    state = STATE_IDLE; // this is to suppress that mBridge sends

    // we should have now a good cmd in cmd_r2m_frame[]
    // mbridge.ChannelsUpdated() should not trigger
    // mbridge.TelemetryUpdate() should not trigger, since mbridge.TelemetryStart() not called
    // mbridge.CommandReceived() should however trigger and should be called"
}


bool tMBridge::CrsfFrameAvailable(uint8_t** const buf, uint8_t* const len)
{
    if (!crsf_emulation) return false;

    if (cmd_m2r_available) {
        *buf = cmd_m2r_frame;
        *len = cmd_m2r_available;
        cmd_m2r_available = 0;
        return true;
    }

    return false;
}


//-------------------------------------------------------
// MBridge user interface

void tMBridge::Init(bool crsf_emulation_flag)
{
    enabled = crsf_emulation = crsf_emulation_flag;

    if (!enabled) return;

    cmd_received = false;
    cmd_m2r_available = 0;

    cmd_fifo.Init();
    cmd_in_process = 0;
    cmd_processed_tlast_ms = 0;
}


// polled in main loop
bool tMBridge::CommandReceived(uint8_t* const cmd)
{
    if (!enabled) return false;
    if (!cmd_received) return false;

    cmd_received = false;

    *cmd = cmd_r2m_frame[0] & (~MBRIDGE_COMMANDPACKET_MASK);

    return true;
}


uint8_t* tMBridge::GetPayloadPtr(void)
{
    return &(cmd_r2m_frame[1]);
}


uint8_t tMBridge::GetModelId(void)
{
    return cmd_r2m_frame[1];
}


/* void tMBridge::GetCommand(uint8_t* cmd, uint8_t* payload)
{
    *cmd = cmd_r2m_frame[0] & (~MBRIDGE_COMMANDPACKET_MASK);

    uint8_t payload_len = mbridge_cmd_payload_len(*cmd);
    memcpy(payload, &(cmd_r2m_frame[1]), payload_len);
} */


void tMBridge::SendCommand(uint8_t cmd, uint8_t* const payload)
{
    memset(cmd_m2r_frame, 0, MBRIDGE_M2R_COMMAND_FRAME_LEN_MAX);

    uint8_t payload_len = mbridge_cmd_payload_len(cmd);

    cmd_m2r_frame[0] = MBRIDGE_COMMANDPACKET_STX + (cmd & (~MBRIDGE_COMMANDPACKET_MASK));
    memcpy(&(cmd_m2r_frame[1]), payload, payload_len);

    cmd_m2r_available = payload_len + 1;
}


bool tMBridge::CommandInFifo(uint8_t* const cmd)
{
    if (cmd_in_process) return false;

    if (!cmd_fifo.Available()) return false;

    // before this was attempted by do_cnt in the main loop
    // cleaner and more precise so now
    // on F4 radios seems not to be needed anymore since lua was changed to request-response
    // on H7 EdgeTx radios, without lua has startup problems (errors out with CRSF not 400k)
    uint32_t tnow_ms = millis32();
    if (crsf_emulation && (cmd_processed_tlast_ms - tnow_ms < 10)) return false; // don't do too fast
    cmd_processed_tlast_ms = tnow_ms;

    cmd_in_process = 0;

    *cmd = cmd_fifo.Get();

    return true;
}


void tMBridge::Lock(uint8_t cmd = 0xFF)
{
    cmd_in_process = cmd;
}


void tMBridge::Unlock(void)
{
    cmd_in_process = 0;
}


//-------------------------------------------------------
// handler

void mbridge_start_ParamRequestByIndex(uint8_t idx);


void tMBridge::HandleRequestCmd(uint8_t* const payload)
{
tMBridgeRequestCmd* request = (tMBridgeRequestCmd*)payload;

    switch (request->cmd_requested) {
    case MBRIDGE_CMD_DEVICE_ITEM_TX:
        cmd_fifo.Put(MBRIDGE_CMD_DEVICE_ITEM_TX);
        break;

    case MBRIDGE_CMD_DEVICE_ITEM_RX:
        cmd_fifo.Put(MBRIDGE_CMD_DEVICE_ITEM_RX);
        break;

    case MBRIDGE_CMD_INFO:
        cmd_fifo.Put(MBRIDGE_CMD_INFO);
        break;

    case MBRIDGE_CMD_REQUEST_INFO:
        cmd_fifo.Put(MBRIDGE_CMD_DEVICE_ITEM_TX);
        cmd_fifo.Put(MBRIDGE_CMD_DEVICE_ITEM_RX);
        cmd_fifo.Put(MBRIDGE_CMD_INFO);
        break;

    case MBRIDGE_CMD_PARAM_ITEM: {
        uint8_t idx = request->param_item.index;
        //if (request->name[0] != 0) { // name is specified, so search for index of parameter
        //}
        mbridge_start_ParamRequestByIndex(idx);
        break; }
    }
}


void tMBridge::HandleCmd(uint8_t cmd)
{
    // this is somewhat dirty, since just the first byte of tMBridgeRequestCmd, but does the job :)
    HandleRequestCmd(&cmd);
}


//-------------------------------------------------------
// convenience helper

void mbridge_send_Info(void)
{
tMBridgeInfo info = {};

    info.tx_config_id = Config.ConfigId;

    if (!TRANSMIT_USE_ANTENNA1) {
        // Config.Diversity = DIVERSITY_ANTENNA2, DIVERSITY_R_ENABLED_T_ANTENNA2
        info.receiver_sensitivity = sx2.ReceiverSensitivity_dbm();
        info.tx_actual_power_dbm = sx2.RfPower_dbm();
    } else {
        // Config.Diversity = DIVERSITY_DEFAULT, DIVERSITY_ANTENNA1, DIVERSITY_R_ENABLED_T_ANTENNA1
        info.receiver_sensitivity = sx.ReceiverSensitivity_dbm(); // is equal for Tx and Rx
        info.tx_actual_power_dbm = sx.RfPower_dbm();
    }
    info.tx_actual_diversity = Config.Diversity;

    if (SetupMetaData.rx_available) {
        info.rx_available = 1;
        info.rx_actual_power_dbm = SetupMetaData.rx_actual_power_dbm;
        info.rx_actual_diversity = SetupMetaData.rx_actual_diversity;
    } else {
        info.rx_available = 0;
        info.rx_actual_power_dbm = INT8_MAX; // INT8_MAX = invalid
        info.rx_actual_diversity = DIVERSITY_NUM; // 5 = invalid
    }

    info.has_status = 1; // to indicate it has these flags
    info.binding = (bind.IsInBind()) ? 1 : 0;

    info.param_num = SETUP_PARAMETER_NUM; // non-zero if known

    mbridge.SendCommand(MBRIDGE_CMD_INFO, (uint8_t*)&info);
}


void mbridge_send_DeviceItemTx(void)
{
tMBridgeDeviceItem item = {};

    item.firmware_version_u16 = version_to_u16(VERSION);
    item.setup_layout_u16 = version_to_u16(SETUPLAYOUT);
    strbufstrcpy(item.device_name_20, DEVICE_NAME, 20);
    mbridge.SendCommand(MBRIDGE_CMD_DEVICE_ITEM_TX, (uint8_t*)&item);
}


void mbridge_send_DeviceItemRx(void)
{
tMBridgeDeviceItem item = {};

    if (SetupMetaData.rx_available) {
        item.firmware_version_u16 = version_to_u16(SetupMetaData.rx_firmware_version);
        item.setup_layout_u16 = version_to_u16(SetupMetaData.rx_setup_layout);
        strbufstrcpy(item.device_name_20, SetupMetaData.rx_device_name, 20);
    } else {
        item.firmware_version_u16 = 0;
        item.setup_layout_u16 = 0;
        strbufstrcpy(item.device_name_20, "", 20);
    }
    mbridge.SendCommand(MBRIDGE_CMD_DEVICE_ITEM_RX, (uint8_t*)&item);
}


uint8_t param_idx; // next param index to send
uint8_t param_itemtype_to_send; // count through sending PARAM_ITEM, PARAM_ITEM2, PARAM_ITEM3_4
char param_optstr[96]; // is currently limited to 67 max

typedef enum {
    MB_PARAM_ITEM = 0,
    MB_PARAM_ITEM1,
    MB_PARAM_ITEM2,
    MB_PARAM_ITEM3,
    MB_PARAM_ITEM4,
} MB_PARAM_ITEM_ENUM;


// we have to send (much) more than SETUP_PARAMETER_NUM PARAM_ITEM messages
// since all parameters need 2 and some even 3 or 4 of them
// currently it are about 80 for the 36 parameters => 80 x 20ms = 1600 ms

// shorten parameter's option string, as follows:
// - each not allowed option is replaced by a '-'
// - keep however option for the current setting (this handles allowed mask = 0)
void param_get_opt_shortened_str(char* const out, uint8_t param_idx)
{
    const char* optstr = SetupParameter[param_idx].optstr;
    uint16_t allowed_mask = param_get_allowed_mask(param_idx);

    if (SetupParameter[param_idx].type != SETUP_PARAM_TYPE_LIST || allowed_mask == UINT16_MAX) {
        strcpy(out, optstr);
        return;
    }

    uint8_t val = *(int8_t*)SetupParameterPtr(param_idx);

    // we have something like "50 Hz,31 Hz,19 Hz,FLRC,FSK"
    uint8_t out_pos = 0;
    char s[24];
    uint8_t pos = 0;
    uint8_t opt_i = 0;
    for (uint8_t n = 0; n < strlen(optstr) + 1; n++) {
        s[pos++] = optstr[n];
        if (optstr[n] == ',' || optstr[n] == '\0') {
            if (opt_i == val || allowed_mask & (1 << opt_i)) { // is current selection or is allowed option, keep it
                for (uint8_t i = 0; i < pos; i++) out[out_pos++] = s[i];
            } else {
                out[out_pos++] = '-';
                out[out_pos++] = optstr[n]; // finish with ',' or '\0'
            }
            opt_i++;
            pos = 0;
            if (out_pos > 80) while(1){} // must not happen
        }
    }
/*
dbg.puts("\nparam   ");dbg.puts(SetupParameter[param_idx].name);
dbg.puts("\n  idx   ");dbg.puts(u8toBCD_s(param_idx));
dbg.puts("\n  opt   ");dbg.puts(optstr);
dbg.puts("\n  mask x");dbg.puts(u16toHEX_s(allowed_mask));
dbg.puts("\n  val   ");dbg.puts(u8toBCD_s(val));
dbg.puts("\n->      ");dbg.puts(out);*/
}


void mbridge_start_ParamRequestByIndex(uint8_t idx)
{
    param_idx = idx;
    param_itemtype_to_send = MB_PARAM_ITEM;

    mbridge.cmd_fifo.Put(MBRIDGE_CMD_PARAM_ITEM); // trigger sending out
}


void mbridge_send_ParamItem(void)
{
    if (param_idx >= SETUP_PARAMETER_NUM) {
        // we send a mBridge message, but don't put a MBRIDGE_CMD_PARAM_ITEM into the fifo, this stops it
        tMBridgeParamItem item = {};
        item.index = UINT8_MAX; // indicates end of list
        mbridge.SendCommand(MBRIDGE_CMD_PARAM_ITEM, (uint8_t*)&item);
        return;
    }

    if (param_itemtype_to_send == MB_PARAM_ITEM) {
        // we always have a 2nd ParamItem
        param_itemtype_to_send = MB_PARAM_ITEM2; // send the 2nd ParamItem in the next call

        tMBridgeParamItem item = {};
        item.index = param_idx;
        switch (SetupParameter[param_idx].type) {
        case SETUP_PARAM_TYPE_INT8:
            item.type = MBRIDGE_PARAM_TYPE_INT8;
            item.value.i8 = *(int8_t*)SetupParameterPtr(param_idx);
            break;
        case SETUP_PARAM_TYPE_LIST:
            item.type = MBRIDGE_PARAM_TYPE_LIST;
            item.value.u8 = *(uint8_t*)SetupParameterPtr(param_idx);
            break;
        case SETUP_PARAM_TYPE_STR6:
            item.type = MBRIDGE_PARAM_TYPE_STR6;
            strbufstrcpy(item.str6_6, (char*)SetupParameterPtr(param_idx), 6);
            break;
        }
        strbufstrcpy(item.name_16, SetupParameter[param_idx].name, 16);

        mbridge.SendCommand(MBRIDGE_CMD_PARAM_ITEM, (uint8_t*)&item);

        param_get_opt_shortened_str(param_optstr, param_idx); // set it for the next items

    } else
    if (param_itemtype_to_send == MB_PARAM_ITEM2) {
        // if a LIST parameter has a long option string, we send a 3rd or 4th ParamItem
        // but start with assuming this is the last ParamItem
        param_itemtype_to_send = MB_PARAM_ITEM;

        tMBridgeParamItem2 item2 = {};
        item2.index = param_idx;
        switch (SetupParameter[param_idx].type) {
        case SETUP_PARAM_TYPE_INT8:
            item2.dflt.i8 = SetupParameter[param_idx].dflt.INT8_value;
            item2.min.i8 = SetupParameter[param_idx].min.INT8_value;
            item2.max.i8 = SetupParameter[param_idx].max.INT8_value;
            strbufstrcpy(item2.unit_6, SetupParameter[param_idx].unit, 6);
            break;
        case SETUP_PARAM_TYPE_LIST:
            if (SetupParameter[param_idx].allowed_mask_ptr != nullptr) {
                item2.allowed_mask = *SetupParameter[param_idx].allowed_mask_ptr;
            } else {
                item2.allowed_mask = UINT16_MAX;
            }
            strbufstrcpy(item2.options_21, param_optstr, 21);
            if (strlen(param_optstr) >= 21) param_itemtype_to_send = MB_PARAM_ITEM3; // we need to send a 3rd ParamItem
            break;
        }

        mbridge.SendCommand(MBRIDGE_CMD_PARAM_ITEM2, (uint8_t*)&item2);

        if (param_itemtype_to_send == MB_PARAM_ITEM) { // done with this parameter
            // next param item
            param_idx++;

            return; // last param item, so stop
        }
    } else
    if (param_itemtype_to_send == MB_PARAM_ITEM3) {
        // if a LIST parameter has a long option string, we send a 3rd or 4th ParamItem
        // but start with assuming this is the last ParamItem
        param_itemtype_to_send = MB_PARAM_ITEM;

        tMBridgeParamItem3_4 item3 = {};
        item3.index = param_idx;
        strbufstrcpy(item3.options2_23, param_optstr + 21, 23);
        if (strlen(param_optstr) >= 21+23) param_itemtype_to_send = MB_PARAM_ITEM4; // we need to send a 4th ParamItem

        mbridge.SendCommand(MBRIDGE_CMD_PARAM_ITEM3_4, (uint8_t*)&item3);

        if (param_itemtype_to_send == MB_PARAM_ITEM) { // done with this parameter
            // next param item
            param_idx++;

            return; // last param item, so stop
        }

    } else
    if (param_itemtype_to_send >= MB_PARAM_ITEM4) {
        // this is the last ParamItem for sure
        param_itemtype_to_send = MB_PARAM_ITEM; // done with this parameter

        tMBridgeParamItem3_4 item4 = {};
        item4.index = param_idx;
        strbufstrcpy(item4.options2_23, param_optstr + 21 + 23, 23);

        // we would have to match MAVLink4OpenTx code
        // to avoid this let's play foul: set highest bit of index
        item4.index += 128;
        mbridge.SendCommand(MBRIDGE_CMD_PARAM_ITEM3_4, (uint8_t*)&item4);

        // next param item
        param_idx++;

        return; // last param item, so stop
    }

    mbridge.cmd_fifo.Put(MBRIDGE_CMD_PARAM_ITEM); // trigger sending out next
}


bool mbridge_do_ParamSet(uint8_t* payload, bool* rx_param_changed)
{
tMBridgeParamSet* param = (tMBridgeParamSet*)payload;

    *rx_param_changed = false;

    if (param->index >= SETUP_PARAMETER_NUM) return false;

    if (SetupParameter[param->index].type <= SETUP_PARAM_TYPE_LIST) {
        *rx_param_changed = setup_set_param(param->index, param->value);
        return true;
    }

    if (SetupParameter[param->index].type == SETUP_PARAM_TYPE_STR6) {
        *rx_param_changed = setup_set_param_str6(param->index, param->str6_6);
        return true;
    }

    return false;
}


void mbridge_send_cmd(uint8_t cmd)
{
    switch (cmd) {
    case MBRIDGE_CMD_DEVICE_ITEM_TX:
        mbridge_send_DeviceItemTx();
        break;
    case MBRIDGE_CMD_DEVICE_ITEM_RX:
        mbridge_send_DeviceItemRx();
        break;
    case MBRIDGE_CMD_PARAM_ITEM:
        mbridge_send_ParamItem();
        break;
    case MBRIDGE_CMD_INFO:
        mbridge_send_Info();
        break;
    }
}


#else

class tMBridge
{
  public:
    void Init(bool crsf_emulation_flag) {}
    void TelemetryStart(void) {}
    void Lock(void) {}
    void Unlock(void) {}
};

tMBridge mbridge;

#endif // ifdef DEVICE_HAS_JRPIN5

#endif // MBRIDGE_INTERFACE_H
