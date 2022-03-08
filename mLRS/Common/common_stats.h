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

class Stats {
  public:
    StatsLQ frames_received; // number of frames received, practically not very relevant
#ifdef DEVICE_IS_RECEIVER
    StatsLQ valid_crc1_received; // received frames which passed crc1 check, but not crc
#endif
    StatsLQ valid_frames_received; // received frames which also passed crc check

    StatsLQ fresh_serial_data_transmitted; // frames with fresh serial data transmitted
    StatsLQ fresh_serial_data_received; // frames with fresh serial data transmitted

    StatsBytes bytes_transmitted;
    StatsBytes bytes_received;

    // statistics for our device
    int8_t last_rx_rssi1;
    int8_t last_rx_rssi2;
    int8_t last_rx_snr1;
    int8_t last_rx_snr2;

    uint8_t last_rx_antenna;
    uint8_t last_tx_antenna;

    // statistics received from the other end
    int8_t received_rssi;
    uint8_t received_LQ;
    uint8_t received_LQ_serial_data;
    uint8_t received_antenna;
    uint8_t received_transmit_antenna;

    // transmission/retransmission handling
    uint8_t transmit_seq_no;
    bool serial_data_received;
    uint8_t received_seq_no_last;
    uint8_t received_ack_last;
    uint8_t retransmit_cnt;

    void Init(void)
    {
        frames_received.Init();
#ifdef DEVICE_IS_RECEIVER
        valid_crc1_received.Init();
#endif
        valid_frames_received.Init();
        fresh_serial_data_transmitted.Init();
        fresh_serial_data_received.Init();
        bytes_transmitted.Init();
        bytes_received.Init();

        Clear();
    }

    void Clear(void) // called then not connected
    {
        last_rx_rssi1 = RSSI_INVALID;
        last_rx_rssi2 = RSSI_INVALID;
        last_rx_snr1 = SNR_INVALID;
        last_rx_snr2 = SNR_INVALID;
        last_rx_antenna = UINT8_MAX;
        last_tx_antenna = UINT8_MAX;

        received_rssi = RSSI_INVALID;
        received_LQ = 0; //UINT8_MAX;
        received_antenna = UINT8_MAX;
        received_transmit_antenna = UINT8_MAX;

        transmit_seq_no = 0;
        serial_data_received = false;
        received_seq_no_last = UINT8_MAX;
        received_ack_last = 0;
        retransmit_cnt = 0;
    }

    void Update1Hz(void)
    {
        frames_received.Update1Hz();
#ifdef DEVICE_IS_RECEIVER
        valid_crc1_received.Update1Hz();
#endif
        valid_frames_received.Update1Hz();
        fresh_serial_data_transmitted.Update1Hz();
        fresh_serial_data_received.Update1Hz();
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

    int8_t GetLastRxRssi(void)
    {
        return (last_rx_antenna == ANTENNA_1) ? last_rx_rssi1 : last_rx_rssi2;
    }

    int8_t GetLastRxSnr(void)
    {
        return (last_rx_antenna == ANTENNA_1) ? last_rx_snr1 : last_rx_snr2;
    }
};


#endif // COMMON_STATS_H
