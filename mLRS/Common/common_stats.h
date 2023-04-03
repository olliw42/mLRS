//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Common Statistics
//*******************************************************
#ifndef COMMON_STATS_H
#define COMMON_STATS_H
#pragma once


#include "common_conf.h"
#include "lq_counter.h"


//-------------------------------------------------------
// Stats
//-------------------------------------------------------

class Stats
{
  public:
    StatsLQ frames_received; // number of frames received, practically not very relevant
#ifdef DEVICE_IS_RECEIVER
    StatsLQ valid_crc1_received; // received frames which passed crc1 check, but not crc
#endif
    StatsLQ valid_frames_received; // received frames which also passed crc check

    StatsLQ serial_data_transmitted; // frames with serial data transmitted, retransmissions are not counted
    StatsLQ serial_data_received; // frames with serial data received, retransmissions are not counted

    StatsBytes bytes_transmitted; // retransmissions are not counted
    StatsBytes bytes_received; // retransmissions are not counted

    // statistics for our device
    int8_t last_rssi1;
    int8_t last_rssi2;
    int8_t last_snr1;
    int8_t last_snr2;
    uint8_t last_antenna;
    uint8_t last_transmit_antenna;

    // statistics received from the other end
    int8_t received_rssi;
    uint8_t received_LQ;
    uint8_t received_LQ_serial_data;
    uint8_t received_antenna;
    uint8_t received_transmit_antenna;

    // transmission/retransmission handling
    uint8_t received_seq_no;
    uint8_t received_ack;
    uint8_t transmit_seq_no; // seq no in the last transmitted frame
    uint8_t retransmit_cnt;

    void Init(void)
    {
        frames_received.Init();
#ifdef DEVICE_IS_RECEIVER
        valid_crc1_received.Init();
#endif
        valid_frames_received.Init();
        serial_data_transmitted.Init();
        serial_data_received.Init();
        bytes_transmitted.Init();
        bytes_received.Init();

        Clear();
    }

    void Clear(void) // called then not connected
    {
        last_rssi1 = RSSI_INVALID;
        last_rssi2 = RSSI_INVALID;
        last_snr1 = SNR_INVALID;
        last_snr2 = SNR_INVALID;
        last_antenna = UINT8_MAX;
        last_transmit_antenna = UINT8_MAX;

        received_rssi = RSSI_INVALID;
        received_LQ = 0; //UINT8_MAX;
        received_antenna = UINT8_MAX;
        received_transmit_antenna = UINT8_MAX;

        received_seq_no = UINT8_MAX;
        received_ack = 0;
        transmit_seq_no = 0;
        retransmit_cnt = 0;
    }

    void Update1Hz(void)
    {
        frames_received.Update1Hz();
#ifdef DEVICE_IS_RECEIVER
        valid_crc1_received.Update1Hz();
#endif
        valid_frames_received.Update1Hz();
        serial_data_transmitted.Update1Hz();
        serial_data_received.Update1Hz();
        bytes_transmitted.Update1Hz();
        bytes_received.Update1Hz();
    }

    uint8_t GetTransmitBandwidthUsage(void)
    {
        // just simply scale it always to the largest theoretical bandwidth
        // is 4100 bytes/s max
        uint32_t bps = bytes_transmitted.GetBytesPerSec();
        uint8_t bw =  (bps + 20) / 41;
        if ((bw == 0) && (bps > 0)) bw = 1; // ensure it is always at least 1% if some bytes are transmitted
        return bw;
    }

    uint8_t GetReceiveBandwidthUsage(void)
    {
        // just simply scale it always to the largest theoretical bandwidth
        // is 4100 bytes/s max
        uint32_t bps = bytes_received.GetBytesPerSec();
        uint8_t bw =  (bps + 20) / 41;
        if ((bw == 0) && (bps > 0)) bw = 1; // ensure it is always at least 1% if some bytes are received
        return bw;
    }

    int8_t GetLastRssi(void)
    {
        return (last_antenna == ANTENNA_1) ? last_rssi1 : last_rssi2;
    }

    int8_t GetLastSnr(void)
    {
        return (last_antenna == ANTENNA_1) ? last_snr1 : last_snr2;
    }
};


#endif // COMMON_STATS_H
