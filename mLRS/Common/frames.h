//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// frames
//*******************************************************
#ifndef FRAMES_H
#define FRAMES_H
#pragma once


#include "frame_types.h"


extern SX_DRIVER sx;
extern SX2_DRIVER sx2;


//-------------------------------------------------------
// Tx, Rx Frames
//-------------------------------------------------------

typedef enum {
    CHECK_OK = 0,
    CHECK_ERROR_SYNCWORD, // 1
    CHECK_ERROR_HEADER,   // 2
    CHECK_ERROR_CRC1,     // 3
    CHECK_ERROR_CRC,      // 4
} CHECK_ENUM;


void _pack_txframe_w_type(tTxFrame* frame, uint8_t type, tFrameStats* frame_stats, tRcData* rc, uint8_t* payload, uint8_t payload_len)
{
uint16_t crc;

    if (payload_len > FRAME_TX_PAYLOAD_LEN) payload_len = FRAME_TX_PAYLOAD_LEN; // should never occur, but play it safe

    memset(frame, 0, sizeof(tTxFrame));

    // generate header
    frame->sync_word = Config.FrameSyncWord;
    frame->status.seq_no = frame_stats->seq_no;
    frame->status.ack = frame_stats->ack;
    frame->status.frame_type = type; //FRAME_TYPE_TX;
    frame->status.antenna = frame_stats->antenna;
    frame->status.transmit_antenna = frame_stats->transmit_antenna;
    frame->status.rssi_u7 = rssi_u7_from_i8(frame_stats->rssi);
    frame->status.LQ = frame_stats->LQ;
    frame->status.LQ_serial_data = frame_stats->LQ_serial_data;
    frame->status.payload_len = payload_len;

    // pack rc data
    // rcData: 0 .. 1024 .. 2047, 11 bits
    frame->rc1.ch0  = rc->ch[0]; // 0 .. 1024 .. 2047, 11 bits
    frame->rc1.ch1  = rc->ch[1];
    frame->rc1.ch2  = rc->ch[2];
    frame->rc1.ch3  = rc->ch[3];

    frame->rc2.ch4  = rc->ch[4]; // 0 .. 1024 .. 2047, 11 bits
    frame->rc2.ch5  = rc->ch[5];
    frame->rc2.ch6  = rc->ch[6];
    frame->rc2.ch7  = rc->ch[7];

    frame->rc2.ch8  = rc->ch[8] / 8; // 0 .. 128 .. 255, 8 bits
    frame->rc2.ch9  = rc->ch[9] / 8;
    frame->rc2.ch10 = rc->ch[10] / 8;
    frame->rc2.ch11 = rc->ch[11] / 8;

    frame->rc1.ch12 = (rc->ch[12] >= 1536) ? 2 : ((rc->ch[12] <= 512) ? 0 : 1); // 0 .. 1 .. 2, bits, 3-way
    frame->rc1.ch13 = (rc->ch[13] >= 1536) ? 2 : ((rc->ch[13] <= 512) ? 0 : 1);
    frame->rc2.ch14 = (rc->ch[14] >= 1536) ? 2 : ((rc->ch[14] <= 512) ? 0 : 1);
    frame->rc2.ch15 = (rc->ch[15] >= 1536) ? 2 : ((rc->ch[15] <= 512) ? 0 : 1);

    // pack the payload
    for (uint8_t i = 0; i < payload_len; i++) {
      frame->payload[i] = payload[i];
    }

    // finalize, crc
    fmav_crc_init(&crc);
    fmav_crc_accumulate_buf(&crc, (uint8_t*)frame, FRAME_TX_RX_HEADER_LEN + FRAME_TX_RCDATA1_LEN);
    frame->crc1 = crc;

    fmav_crc_accumulate_buf(&crc, (uint8_t*)frame + FRAME_TX_RX_HEADER_LEN + FRAME_TX_RCDATA1_LEN, FRAME_TX_RX_LEN - FRAME_TX_RX_HEADER_LEN - FRAME_TX_RCDATA1_LEN - 2);
    frame->crc = crc;
}


void pack_txframe(tTxFrame* frame, tFrameStats* frame_stats, tRcData* rc, uint8_t* payload, uint8_t payload_len)
{
    _pack_txframe_w_type(frame, FRAME_TYPE_TX, frame_stats, rc, payload, payload_len);
}


// returns 0 if OK !!
uint8_t check_txframe(tTxFrame* frame)
{
uint16_t crc;

    if (frame->sync_word != Config.FrameSyncWord) return CHECK_ERROR_SYNCWORD;

    if ((frame->status.frame_type != FRAME_TYPE_TX) && (frame->status.frame_type != FRAME_TYPE_TX_RX_CMD)) {
        return CHECK_ERROR_HEADER;
    }

    if (frame->status.payload_len > FRAME_TX_PAYLOAD_LEN) return CHECK_ERROR_HEADER;

    fmav_crc_init(&crc);
    fmav_crc_accumulate_buf(&crc, (uint8_t*)frame, FRAME_TX_RX_HEADER_LEN + FRAME_TX_RCDATA1_LEN);
    if (crc != frame->crc1) return CHECK_ERROR_CRC1;

    fmav_crc_accumulate_buf(&crc, (uint8_t*)frame + FRAME_TX_RX_HEADER_LEN + FRAME_TX_RCDATA1_LEN, FRAME_TX_RX_LEN - FRAME_TX_RX_HEADER_LEN - FRAME_TX_RCDATA1_LEN - 2);
    if (crc != frame->crc) return CHECK_ERROR_CRC;

    return CHECK_OK;
}


void rcdata_rc1_from_txframe(tRcData* rc, tTxFrame* frame)
{
    rc->ch[0] = frame->rc1.ch0;
    rc->ch[1] = frame->rc1.ch1;
    rc->ch[2] = frame->rc1.ch2;
    rc->ch[3] = frame->rc1.ch3;

    rc->ch[12] = (frame->rc1.ch12 > 1) ? 2047 : ((frame->rc1.ch12 < 1) ? 0 : 1024);
    rc->ch[13] = (frame->rc1.ch13 > 1) ? 2047 : ((frame->rc1.ch13 < 1) ? 0 : 1024);
}


void rcdata_from_txframe(tRcData* rc, tTxFrame* frame)
{
    rc->ch[0] = frame->rc1.ch0;
    rc->ch[1] = frame->rc1.ch1;
    rc->ch[2] = frame->rc1.ch2;
    rc->ch[3] = frame->rc1.ch3;

    rc->ch[4] = frame->rc2.ch4;
    rc->ch[5] = frame->rc2.ch5;
    rc->ch[6] = frame->rc2.ch6;
    rc->ch[7] = frame->rc2.ch7;

    rc->ch[8] = frame->rc2.ch8 * 8;
    rc->ch[9] = frame->rc2.ch9 * 8;
    rc->ch[10] = frame->rc2.ch10 * 8;
    rc->ch[11] = frame->rc2.ch11 * 8;

    rc->ch[12] = (frame->rc1.ch12 > 1) ? 2047 : ((frame->rc1.ch12 < 1) ? 0 : 1024);
    rc->ch[13] = (frame->rc1.ch13 > 1) ? 2047 : ((frame->rc1.ch13 < 1) ? 0 : 1024);
    rc->ch[14] = (frame->rc2.ch14 > 1) ? 2047 : ((frame->rc2.ch14 < 1) ? 0 : 1024);
    rc->ch[15] = (frame->rc2.ch15 > 1) ? 2047 : ((frame->rc2.ch15 < 1) ? 0 : 1024);

    rc->ch[16] = 1024;
    rc->ch[17] = 1024;
}


void _pack_rxframe_w_type(tRxFrame* frame, uint8_t type, tFrameStats* frame_stats, uint8_t* payload, uint8_t payload_len)
{
uint16_t crc;

    if (payload_len > FRAME_RX_PAYLOAD_LEN) payload_len = FRAME_RX_PAYLOAD_LEN; // should never occur, but play it safe

    memset(frame, 0, sizeof(tRxFrame));

    frame->sync_word = Config.FrameSyncWord;
    frame->status.seq_no = frame_stats->seq_no;
    frame->status.ack = frame_stats->ack;
    frame->status.frame_type = type; //FRAME_TYPE_RX;
    frame->status.antenna = frame_stats->antenna;
    frame->status.transmit_antenna = frame_stats->transmit_antenna;
    frame->status.rssi_u7 = rssi_u7_from_i8(frame_stats->rssi);
    frame->status.LQ = frame_stats->LQ;
    frame->status.LQ_serial_data = frame_stats->LQ_serial_data;
    frame->status.payload_len = payload_len;

    for (uint8_t i = 0; i < payload_len; i++) {
      frame->payload[i] = payload[i];
    }

    fmav_crc_init(&crc);
    fmav_crc_accumulate_buf(&crc, (uint8_t*)frame, FRAME_TX_RX_LEN - 2);
    frame->crc = crc;
}


void pack_rxframe(tRxFrame* frame, tFrameStats* frame_stats, uint8_t* payload, uint8_t payload_len)
{
    _pack_rxframe_w_type(frame, FRAME_TYPE_RX, frame_stats, payload, payload_len);
}

// returns 0 if OK !!
uint8_t check_rxframe(tRxFrame* frame)
{
uint16_t crc;

    if (frame->sync_word != Config.FrameSyncWord) return CHECK_ERROR_SYNCWORD;

    if ((frame->status.frame_type != FRAME_TYPE_RX) && (frame->status.frame_type != FRAME_TYPE_TX_RX_CMD)) {
        return CHECK_ERROR_HEADER;
    }

    if (frame->status.payload_len > FRAME_RX_PAYLOAD_LEN) return CHECK_ERROR_HEADER;

    fmav_crc_init(&crc);
    fmav_crc_accumulate_buf(&crc, (uint8_t*)frame, FRAME_TX_RX_LEN - 2);
    if (crc != frame->crc) return CHECK_ERROR_CRC;

    return CHECK_OK;
}



//-------------------------------------------------------
// Tx/Rx Cmd Frames
//-------------------------------------------------------

void cmdframerxparameters_rxparams_from_rxsetup(tCmdFrameRxParameters* rx_params)
{
    rx_params->Power = Setup.Rx.Power;
    rx_params->Diversity = Setup.Rx.Diversity;
    rx_params->ChannelOrder = Setup.Rx.ChannelOrder;
    rx_params->OutMode = Setup.Rx.OutMode;
    rx_params->OutRssiChannelMode = Setup.Rx.OutRssiChannelMode;
    rx_params->FailsafeMode = Setup.Rx.FailsafeMode;
    rx_params->SerialBaudrate = Setup.Rx.SerialBaudrate;
    rx_params->SerialLinkMode = Setup.Rx.SerialLinkMode;
    rx_params->SendRadioStatus = Setup.Rx.SendRadioStatus;
    rx_params->Buzzer = Setup.Rx.Buzzer;
    rx_params->SendRcChannels = Setup.Rx.SendRcChannels;
    // deprecated rx_params->RadioStatusMethod = Setup.Rx.RadioStatusMethod;

    for (uint8_t i = 0; i < 12; i++) {
        rx_params->FailsafeOutChannelValues_Ch1_Ch12[i] = Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[i];
    }
    rx_params->FailsafeOutChannelValue_Ch13 = Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[0];
    rx_params->FailsafeOutChannelValue_Ch14 = Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[1];
    rx_params->FailsafeOutChannelValue_Ch15 = Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[2];
    rx_params->FailsafeOutChannelValue_Ch16 = Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[3];
}


void cmdframerxparameters_rxparams_to_rxsetup(tCmdFrameRxParameters* rx_params)
{
    Setup.Rx.Power = rx_params->Power;
    Setup.Rx.Diversity = rx_params->Diversity;
    Setup.Rx.ChannelOrder = rx_params->ChannelOrder;
    Setup.Rx.OutMode = rx_params->OutMode;
    Setup.Rx.OutRssiChannelMode = rx_params->OutRssiChannelMode;
    Setup.Rx.FailsafeMode = rx_params->FailsafeMode;
    Setup.Rx.SerialBaudrate = rx_params->SerialBaudrate;
    Setup.Rx.SerialLinkMode = rx_params->SerialLinkMode;
    Setup.Rx.SendRadioStatus = rx_params->SendRadioStatus;
    Setup.Rx.Buzzer = rx_params->Buzzer;
    Setup.Rx.SendRcChannels = rx_params->SendRcChannels;
    // deprecated Setup.Rx.RadioStatusMethod = rx_params->RadioStatusMethod;

    for (uint8_t i = 0; i < 12; i++) {
        Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[i] = rx_params->FailsafeOutChannelValues_Ch1_Ch12[i];
    }
    Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[0] = rx_params->FailsafeOutChannelValue_Ch13;
    Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[1] = rx_params->FailsafeOutChannelValue_Ch14;
    Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[2] = rx_params->FailsafeOutChannelValue_Ch15;
    Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[3] = rx_params->FailsafeOutChannelValue_Ch16;
}


#ifdef DEVICE_IS_TRANSMITTER

// Tx: send cmd to Rx
void pack_txcmdframe_cmd(tTxFrame* frame, tFrameStats* frame_stats, tRcData* rc, uint8_t cmd)
{
uint8_t payload[1];

    payload[0] = cmd;

    _pack_txframe_w_type(frame, FRAME_TYPE_TX_RX_CMD, frame_stats, rc, payload, 1);
}


// Tx: handle FRAME_CMD_RX_SETUPDATA from Rx
void unpack_rxcmdframe_rxsetupdata(tRxFrame* frame)
{
tRxCmdFrameRxSetupData* rx_setupdata = (tRxCmdFrameRxSetupData*)frame->payload;

    SetupMetaData.rx_available = true;

    SetupMetaData.rx_firmware_version = version_from_u16(rx_setupdata->firmware_version_u16);
    SetupMetaData.rx_setup_layout = rx_setupdata->setup_layout;
    strstrbufcpy(SetupMetaData.rx_device_name, rx_setupdata->device_name_20, 20);
    SetupMetaData.rx_actual_power_dbm = rx_setupdata->actual_power_dbm;
    SetupMetaData.rx_actual_diversity = rx_setupdata->actual_diversity;

    cmdframerxparameters_rxparams_to_rxsetup(&(rx_setupdata->RxParams));

    // TODO
    //SetupMetaData.FrequencyBand_allowed_mask = rx_setupdata->FrequencyBand_allowed_mask;
    //SetupMetaData.Mode_allowed_mask = rx_setupdata->Mode_allowed_mask;

    int16_t power_list[8];
    for (uint8_t i = 0; i < 8; i++) power_list[i] = rx_setupdata->Power_list[i]; // to avoid unaligned warning
    power_optstr_from_power_list(SetupMetaData.Rx_Power_optstr, power_list, 8, 44);
    SetupMetaData.Rx_Diversity_allowed_mask = rx_setupdata->Diversity_allowed_mask;
    SetupMetaData.Rx_OutMode_allowed_mask = rx_setupdata->OutMode_allowed_mask;
    SetupMetaData.Rx_Buzzer_allowed_mask = rx_setupdata->Buzzer_allowed_mask;
}


// Tx: send new receiver parameters with FRAME_CMD_SET_RX_PARAMS to Rx
// we take the values from Tx' Setup.Rx structure
void pack_txcmdframe_setrxparams(tTxFrame* frame, tFrameStats* frame_stats, tRcData* rc)
{
tTxCmdFrameRxParams rx_params = {0};

    rx_params.cmd = FRAME_CMD_SET_RX_PARAMS;

    strbufstrcpy(rx_params.BindPhrase_6, Setup.BindPhrase, 6);
    rx_params.FrequencyBand = Setup.FrequencyBand;
    rx_params.Mode = Setup.Mode;

    cmdframerxparameters_rxparams_from_rxsetup(&(rx_params.RxParams));

    _pack_txframe_w_type(frame, FRAME_TYPE_TX_RX_CMD, frame_stats, rc, (uint8_t*)&rx_params, sizeof(rx_params));
}

#endif
#ifdef DEVICE_IS_RECEIVER

// Rx: send cmd to Tx
void pack_rxcmdframe_cmd(tRxFrame* frame, tFrameStats* frame_stats, uint8_t cmd)
{
uint8_t payload[1];

    payload[0] = cmd;

    _pack_rxframe_w_type(frame, FRAME_TYPE_TX_RX_CMD, frame_stats, payload, 1);
}


// Rx: send FRAME_CMD_RX_SETUPDATA to Tx
void pack_rxcmdframe_rxsetupdata(tRxFrame* frame, tFrameStats* frame_stats)
{
tRxCmdFrameRxSetupData rx_setupdata = {0};

    rx_setupdata.cmd = FRAME_CMD_RX_SETUPDATA;

    rx_setupdata.firmware_version_u16 = version_to_u16(VERSION);
    rx_setupdata.setup_layout = SETUPLAYOUT;
    strbufstrcpy(rx_setupdata.device_name_20, DEVICE_NAME, 20);
    rx_setupdata.actual_power_dbm = sx.RfPower_dbm();
    if (USE_ANTENNA1 && USE_ANTENNA2) {
        rx_setupdata.actual_diversity = 0;
    } else
    if (USE_ANTENNA1) {
        rx_setupdata.actual_diversity = 1;
    } else
    if (USE_ANTENNA2) {
        rx_setupdata.actual_diversity = 2;
    } else {
        rx_setupdata.actual_diversity = 3; // 3 = invalid
    }

    cmdframerxparameters_rxparams_from_rxsetup(&(rx_setupdata.RxParams));

    // TODO
    //rx_setupdata.FrequencyBand_allowed_mask = SetupMetaData.FrequencyBand_allowed_mask;
    //rx_setupdata.Mode_allowed_mask = SetupMetaData.Mode_allowed_mask;

    for (uint8_t i = 0; i < 8; i++) {
        rx_setupdata.Power_list[i] = (i < RFPOWER_LIST_NUM) ? rfpower_list[i].mW : INT16_MAX;
    }
    rx_setupdata.Diversity_allowed_mask = SetupMetaData.Rx_Diversity_allowed_mask;
    rx_setupdata.OutMode_allowed_mask = SetupMetaData.Rx_OutMode_allowed_mask;
    rx_setupdata.Buzzer_allowed_mask = SetupMetaData.Rx_Buzzer_allowed_mask;

    _pack_rxframe_w_type(frame, FRAME_TYPE_TX_RX_CMD, frame_stats, (uint8_t*)&rx_setupdata, sizeof(rx_setupdata));
}


// Rx: handle FRAME_CMD_SET_RX_PARAMS
// new parameter values are stored in Rx' Setup.Rx fields
void unpack_txcmdframe_setrxparams(tTxFrame* frame)
{
tTxCmdFrameRxParams* rx_params = (tTxCmdFrameRxParams*)frame->payload;

    strstrbufcpy(Setup.BindPhrase, rx_params->BindPhrase_6, 6);
    Setup.FrequencyBand = rx_params->FrequencyBand;
    Setup.Mode = rx_params->Mode;

    cmdframerxparameters_rxparams_to_rxsetup(&(rx_params->RxParams));
}
#endif


#endif // FRAMES_H
