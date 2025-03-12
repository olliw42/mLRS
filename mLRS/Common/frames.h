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


extern tGlobalConfig Config;
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


void _pack_txframe_w_type(
    tTxFrame* const frame,
    uint8_t type,
    tFrameStats* const frame_stats,
    tRcData* const rc,
    uint8_t* const payload,
    uint8_t payload_len)
{
uint16_t crc;

    if (payload_len > FRAME_TX_PAYLOAD_LEN) payload_len = FRAME_TX_PAYLOAD_LEN; // should never occur, but play it safe

    memset((uint8_t*)frame, 0, sizeof(tTxFrame));

    // generate header
    frame->sync_word = Config.FrameSyncWord;
    frame->status.seq_no = frame_stats->seq_no;
    frame->status.ack = frame_stats->ack;
    frame->status.frame_type = type; // FRAME_TYPE_TX, FRAME_TYPE_TX_RX_CMD
    frame->status.antenna = frame_stats->antenna;
    frame->status.transmit_antenna = frame_stats->transmit_antenna;
    frame->status.rssi_u7 = rssi_u7_from_i8(frame_stats->rssi);
    frame->status.fhss_index_band = frame_stats->tx_fhss_index_band;
    frame->status.fhss_index = frame_stats->tx_fhss_index;
    frame->status.LQ_serial = frame_stats->LQ_serial;
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


void pack_txframe(
    tTxFrame* const frame,
    tFrameStats* const frame_stats,
    tRcData* const rc,
    uint8_t* const payload,
    uint8_t payload_len)
{
    _pack_txframe_w_type(frame, FRAME_TYPE_TX, frame_stats, rc, payload, payload_len);
}


// returns 0 if OK !!
uint8_t check_txframe(tTxFrame* const frame)
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


void rcdata_rc1_from_txframe(tRcData* const rc, tTxFrame* const frame)
{
    rc->ch[0] = frame->rc1.ch0;
    rc->ch[1] = frame->rc1.ch1;
    rc->ch[2] = frame->rc1.ch2;
    rc->ch[3] = frame->rc1.ch3;

    rc->ch[12] = (frame->rc1.ch12 > 1) ? 2047 : ((frame->rc1.ch12 < 1) ? 0 : 1024);
    rc->ch[13] = (frame->rc1.ch13 > 1) ? 2047 : ((frame->rc1.ch13 < 1) ? 0 : 1024);
}


void rcdata_from_txframe(tRcData* const rc, tTxFrame* const frame)
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


// update header info with new data, keep payload
void update_rxframe_stats(tRxFrame* const frame, tFrameStats* const frame_stats)
{
uint16_t crc;

    frame->sync_word = Config.FrameSyncWord;
    // keep !! frame->status.seq_no = frame_stats->seq_no;
    frame->status.ack = frame_stats->ack;
    // keep !! frame->status.frame_type = type; // FRAME_TYPE_RX, FRAME_TYPE_TX_RX_CMD
    frame->status.antenna = frame_stats->antenna;
    frame->status.transmit_antenna = frame_stats->transmit_antenna;
    frame->status.rssi_u7 = rssi_u7_from_i8(frame_stats->rssi);
    frame->status.LQ_rc = frame_stats->LQ_rc;
    frame->status.LQ_serial = frame_stats->LQ_serial;
    // keep !! frame->status.payload_len = payload_len;

    fmav_crc_init(&crc);
    fmav_crc_accumulate_buf(&crc, (uint8_t*)frame, FRAME_TX_RX_LEN - 2);
    frame->crc = crc;
}


void _pack_rxframe_w_type(
    tRxFrame* const frame,
    uint8_t type,
    tFrameStats* const frame_stats,
    uint8_t* const payload,
    uint8_t payload_len)
{
uint16_t crc;

    if (payload_len > FRAME_RX_PAYLOAD_LEN) payload_len = FRAME_RX_PAYLOAD_LEN; // should never occur, but play it safe

    memset((uint8_t*)frame, 0, sizeof(tRxFrame));

    frame->sync_word = Config.FrameSyncWord;
    frame->status.seq_no = frame_stats->seq_no;
    frame->status.ack = frame_stats->ack;
    frame->status.frame_type = type; // FRAME_TYPE_RX, FRAME_TYPE_TX_RX_CMD
    frame->status.antenna = frame_stats->antenna;
    frame->status.transmit_antenna = frame_stats->transmit_antenna;
    frame->status.rssi_u7 = rssi_u7_from_i8(frame_stats->rssi);
    frame->status.LQ_rc = frame_stats->LQ_rc;
    frame->status.LQ_serial = frame_stats->LQ_serial;
    frame->status.payload_len = payload_len;

    for (uint8_t i = 0; i < payload_len; i++) {
        frame->payload[i] = payload[i];
    }

    fmav_crc_init(&crc);
    fmav_crc_accumulate_buf(&crc, (uint8_t*)frame, FRAME_TX_RX_LEN - 2);
    frame->crc = crc;
}


void pack_rxframe(
    tRxFrame* const frame,
    tFrameStats* const frame_stats,
    uint8_t* const payload,
    uint8_t payload_len)
{
    _pack_rxframe_w_type(frame, FRAME_TYPE_RX, frame_stats, payload, payload_len);
}


// returns 0 if OK !!
uint8_t check_rxframe(tRxFrame* const frame)
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

void cmdframerxparameters_rxparams_from_rxsetup(tCmdFrameRxParameters* const rx_params)
{
    rx_params->Power = Setup.Rx.Power;
    rx_params->Diversity = Setup.Rx.Diversity;
    rx_params->ChannelOrder = Setup.Rx.ChannelOrder;
    rx_params->OutMode = Setup.Rx.OutMode;
    rx_params->OutRssiChannelMode = Setup.Rx.OutRssiChannelMode;
    rx_params->OutLqChannelMode = Setup.Rx.OutLqChannelMode;
    rx_params->FailsafeMode = Setup.Rx.FailsafeMode;
    rx_params->SerialPort = Setup.Rx.SerialPort;
    rx_params->SerialBaudrate = Setup.Rx.SerialBaudrate;
    rx_params->SerialLinkMode = Setup.Rx.SerialLinkMode;
    rx_params->SendRadioStatus = Setup.Rx.SendRadioStatus;
    // deprecated rx_params->Buzzer = Setup.Rx.Buzzer;
    rx_params->SendRcChannels = Setup.Rx.SendRcChannels;
    // deprecated rx_params->RadioStatusMethod = Setup.Rx.RadioStatusMethod;
    rx_params->PowerSwitchChannel = Setup.Rx.PowerSwitchChannel;

    for (uint8_t i = 0; i < 12; i++) {
        rx_params->FailsafeOutChannelValues_Ch1_Ch12[i] = Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[i];
    }
    rx_params->FailsafeOutChannelValue_Ch13 = Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[0];
    rx_params->FailsafeOutChannelValue_Ch14 = Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[1];
    rx_params->FailsafeOutChannelValue_Ch15 = Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[2];
    rx_params->FailsafeOutChannelValue_Ch16 = Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[3];
}


void cmdframerxparameters_rxparams_to_rxsetup(tCmdFrameRxParameters* const rx_params)
{
    Setup.Rx.Power = rx_params->Power;
    Setup.Rx.Diversity = rx_params->Diversity;
    Setup.Rx.ChannelOrder = rx_params->ChannelOrder;
    Setup.Rx.OutMode = rx_params->OutMode;
    Setup.Rx.OutRssiChannelMode = rx_params->OutRssiChannelMode;
    Setup.Rx.OutLqChannelMode = rx_params->OutLqChannelMode;
    Setup.Rx.FailsafeMode = rx_params->FailsafeMode;
    Setup.Rx.SerialPort = rx_params->SerialPort;
    Setup.Rx.SerialBaudrate = rx_params->SerialBaudrate;
    Setup.Rx.SerialLinkMode = rx_params->SerialLinkMode;
    Setup.Rx.SendRadioStatus = rx_params->SendRadioStatus;
    // deprecated Setup.Rx.Buzzer = rx_params->Buzzer;
    Setup.Rx.SendRcChannels = rx_params->SendRcChannels;
    // deprecated Setup.Rx.RadioStatusMethod = rx_params->RadioStatusMethod;
    Setup.Rx.PowerSwitchChannel = rx_params->PowerSwitchChannel;

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
void pack_txcmdframe_cmd(tTxFrame* const frame, tFrameStats* const frame_stats, tRcData* const rc, uint8_t cmd)
{
uint8_t payload[1];

    payload[0] = cmd;

    _pack_txframe_w_type(frame, FRAME_TYPE_TX_RX_CMD, frame_stats, rc, payload, 1);
}


// Tx: handle FRAME_CMD_RX_SETUPDATA from Rx
void unpack_rxcmdframe_rxsetupdata(tRxFrame* const frame)
{
tRxCmdFrameRxSetupData* rx_setupdata = (tRxCmdFrameRxSetupData*)frame->payload;

    SetupMetaData.rx_available = true;

    SetupMetaData.rx_firmware_version = version_from_u16(rx_setupdata->firmware_version_u16);
    SetupMetaData.rx_setup_layout = version_from_u16(rx_setupdata->setup_layout_u16);
    strstrbufcpy(SetupMetaData.rx_device_name, rx_setupdata->device_name_20, 20);
    SetupMetaData.rx_actual_power_dbm = rx_setupdata->actual_power_dbm;
    SetupMetaData.rx_actual_diversity = rx_setupdata->actual_diversity;

    cmdframerxparameters_rxparams_to_rxsetup(&(rx_setupdata->RxParams));

    // TODO
    // These are for common parameters. It should work such, that the Tx only provides options also allowed by the Rx.
    //SetupMetaData.FrequencyBand_allowed_mask = rx_setupdata->FrequencyBand_allowed_mask;
    //SetupMetaData.Mode_allowed_mask = rx_setupdata->Mode_allowed_mask;
    //SetupMetaData.Ortho_allowed_mask = rx_setupdata->Ortho_allowed_mask;

    int16_t power_list[8];
    for (uint8_t i = 0; i < 8; i++) power_list[i] = rx_setupdata->Power_list[i]; // to avoid unaligned warning
    power_optstr_from_power_list(SetupMetaData.Rx_Power_optstr, power_list, 8, 44);
    SetupMetaData.Rx_Diversity_allowed_mask = rx_setupdata->Diversity_allowed_mask;
    SetupMetaData.Rx_OutMode_allowed_mask = rx_setupdata->OutMode_allowed_mask;
    SetupMetaData.Rx_SerialPort_allowed_mask = rx_setupdata->SerialPort_allowed_mask;
}


// Tx: send new receiver parameters with FRAME_CMD_SET_RX_PARAMS to Rx
// we take the values from Tx' Setup.Rx structure
void pack_txcmdframe_setrxparams(tTxFrame* const frame, tFrameStats* const frame_stats, tRcData* const rc)
{
tTxCmdFrameRxParams rx_params = {};

    rx_params.cmd = FRAME_CMD_SET_RX_PARAMS;

    strbufstrcpy(rx_params.BindPhrase_6, Setup.Common[Config.ConfigId].BindPhrase, 6);
    rx_params.FrequencyBand = Setup.Common[Config.ConfigId].FrequencyBand;
    rx_params.Mode = Setup.Common[Config.ConfigId].Mode;
    rx_params.Ortho = Setup.Common[Config.ConfigId].Ortho;

    cmdframerxparameters_rxparams_from_rxsetup(&(rx_params.RxParams));

    _pack_txframe_w_type(frame, FRAME_TYPE_TX_RX_CMD, frame_stats, rc, (uint8_t*)&rx_params, sizeof(rx_params));
}

#endif
#ifdef DEVICE_IS_RECEIVER

// Rx: send FRAME_CMD_RX_SETUPDATA to Tx
void pack_rxcmdframe_rxsetupdata(tRxFrame* const frame, tFrameStats* const frame_stats)
{
tRxCmdFrameRxSetupData rx_setupdata = {};

    rx_setupdata.cmd = FRAME_CMD_RX_SETUPDATA;

    rx_setupdata.firmware_version_u16 = version_to_u16(VERSION);
    rx_setupdata.setup_layout_u16 = version_to_u16(SETUPLAYOUT);
    strbufstrcpy(rx_setupdata.device_name_20, DEVICE_NAME, 20);
    rx_setupdata.actual_power_dbm = sx.RfPower_dbm();
    rx_setupdata.actual_diversity = Config.Diversity;

    cmdframerxparameters_rxparams_from_rxsetup(&(rx_setupdata.RxParams));

    // TODO
    // These are for common parameters. It should work such, that the Tx only provides options also allowed by the Rx.
    //rx_setupdata.FrequencyBand_allowed_mask = SetupMetaData.FrequencyBand_allowed_mask;
    //rx_setupdata.Mode_allowed_mask = SetupMetaData.Mode_allowed_mask;
    //rx_setupdata.Ortho_allowed_mask = SetupMetaData.Ortho_allowed_mask;

    for (uint8_t i = 0; i < 8; i++) {
        rx_setupdata.Power_list[i] = (i < RFPOWER_LIST_NUM) ? rfpower_list[i].mW : INT16_MAX;
    }
    rx_setupdata.Diversity_allowed_mask = SetupMetaData.Rx_Diversity_allowed_mask;
    rx_setupdata.OutMode_allowed_mask = SetupMetaData.Rx_OutMode_allowed_mask;
    rx_setupdata.SerialPort_allowed_mask = SetupMetaData.Rx_SerialPort_allowed_mask;

    _pack_rxframe_w_type(frame, FRAME_TYPE_TX_RX_CMD, frame_stats, (uint8_t*)&rx_setupdata, sizeof(rx_setupdata));
}


// Rx: handle FRAME_CMD_SET_RX_PARAMS
// new parameter values are stored in Rx' Setup.Rx fields
void unpack_txcmdframe_setrxparams(tTxFrame* const frame)
{
tTxCmdFrameRxParams* rx_params = (tTxCmdFrameRxParams*)frame->payload;

    strstrbufcpy(Setup.Common[0].BindPhrase, rx_params->BindPhrase_6, 6);
    Setup.Common[0].FrequencyBand = (SETUP_FREQUENCY_BAND_ENUM)rx_params->FrequencyBand;
    Setup.Common[0].Mode = rx_params->Mode;
    Setup.Common[0].Ortho = rx_params->Ortho;

    cmdframerxparameters_rxparams_to_rxsetup(&(rx_params->RxParams));
}

#endif


//-------------------------------------------------------
// Helper
//-------------------------------------------------------

// Numerical Recipe's quick generator randq1()
uint32_t nr_randq1(void)
{
    static uint32_t seed = 0;
    seed = 1664525UL * seed + 1013904223UL;
    return seed;
}


uint8_t fhss_band_next(void)
{
    static uint8_t fhss_band = 0;
    static uint8_t fhss_band_last = 0;

    if (fhss_band == fhss_band_last) { // we had it two times, so toggle
        fhss_band_last = fhss_band;
        fhss_band++;
    } else { // toggle with 50% probability
        fhss_band_last = fhss_band;
        if (nr_randq1() < UINT32_MAX/2) fhss_band++;
    }
    return fhss_band;
}


#endif // FRAMES_H
