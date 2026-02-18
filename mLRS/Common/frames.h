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
#include "hal/hal.h"


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
    frame->status.frame_type = type; // FRAME_TYPE_TX, FRAME_TYPE_TX_RX_CMD
    frame->status.broadcast = frame_stats->broadcast;
    frame->status.fhss_index_band = frame_stats->tx_fhss_index_band;
    frame->status.fhss_index = frame_stats->tx_fhss_index;
    if (frame->status.broadcast)
    {
    	frame->status.sys_id = 0x00;
		frame->status.show_group = 0xFF;
    }
    else
    {
    	frame->status.sys_id = frame_stats->sys_id;
		frame->status.show_group = frame_stats->show_group;
    }
    frame->status.payload_len = payload_len;

    // pack the payload
    for (uint8_t i = 0; i < payload_len; i++) {
        frame->payload[i] = payload[i];
    }

    // finalize, crc
    fmav_crc_init(&crc);
    fmav_crc_accumulate_buf(&crc, (uint8_t*)frame, FRAME_TX_RX_HEADER_LEN - 2);
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
    fmav_crc_accumulate_buf(&crc, (uint8_t*)frame, FRAME_TX_RX_HEADER_LEN - 2);
    if (crc != frame->crc) return CHECK_ERROR_CRC;

    return CHECK_OK;
}

// update header info with new data, keep payload
void update_rxframe_stats(tRxFrame* const frame, tFrameStats* const frame_stats)
{
uint16_t crc;

    frame->sync_word = Config.FrameSyncWord;

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
    frame->status.frame_type = type; // FRAME_TYPE_RX, FRAME_TYPE_TX_RX_CMD
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
    rx_params->MavlinkSystemID = Setup.Rx.MavlinkSystemID;

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
    Setup.Rx.MavlinkSystemID = rx_params->MavlinkSystemID;

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

    strstrbufcpy(SetupMetaData.rx_device_name, rx_setupdata->device_name_20, 20);
    SetupMetaData.rx_actual_power_dbm = rx_setupdata->actual_power_dbm;

    cmdframerxparameters_rxparams_to_rxsetup(&(rx_setupdata->RxParams));

    // TODO
    // These are for common parameters. It should work such, that the Tx only provides options also allowed by the Rx.
    //SetupMetaData.FrequencyBand_allowed_mask = rx_setupdata->FrequencyBand_allowed_mask;
    //SetupMetaData.Mode_allowed_mask = rx_setupdata->Mode_allowed_mask;
    //SetupMetaData.Ortho_allowed_mask = rx_setupdata->Ortho_allowed_mask;

    int16_t power_list[8];
    for (uint8_t i = 0; i < 8; i++) power_list[i] = rx_setupdata->Power_list[i]; // to avoid unaligned warning
    power_optstr_from_power_list(SetupMetaData.Rx_Power_optstr, power_list, 8, 67);
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

    strbufstrcpy(rx_setupdata.device_name_20, DEVICE_NAME, 20);
    rx_setupdata.actual_power_dbm = sx.RfPower_dbm();

    cmdframerxparameters_rxparams_from_rxsetup(&(rx_setupdata.RxParams));

    // TODO
    // These are for common parameters. It should work such, that the Tx only provides options also allowed by the Rx.
    //rx_setupdata.FrequencyBand_allowed_mask = SetupMetaData.FrequencyBand_allowed_mask;
    //rx_setupdata.Mode_allowed_mask = SetupMetaData.Mode_allowed_mask;
    //rx_setupdata.Ortho_allowed_mask = SetupMetaData.Ortho_allowed_mask;

    for (uint8_t i = 0; i < 8; i++) {
        rx_setupdata.Power_list[i] = (i < RFPOWER_LIST_NUM) ? rfpower_list[i].mW : INT16_MAX;
    }

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
        fhss_band++;
    } else { // toggle with 50% probability
        fhss_band_last = fhss_band;
        if (nr_randq1() < UINT32_MAX/2) fhss_band++;
    }

    return fhss_band & 0x01;
}


#endif // FRAMES_H
