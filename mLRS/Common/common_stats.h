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

#include <stdint.h>
#include "common_conf.h"
#include "hal/device_conf.h"
#include "lq_counter.h"
#include "common_types.h"


//-------------------------------------------------------
// Common stats
//-------------------------------------------------------

class tStats
{
  public:
    void Init(uint16_t _frame_rate_hz);
    void Clear(void);                 // called then not connected
    void Update1Hz(void);             // called at 1 Hz

    uint8_t GetTransmitBandwidthUsage(void);
    uint8_t GetReceiveBandwidthUsage(void);
    int8_t GetLastRssi(void);
    int8_t GetLastSnr(void);

    // statistics for our device

    tStatsLQ frames_received;         // number of frames received, practically not very relevant
#ifdef DEVICE_IS_RECEIVER
    tStatsLQ valid_crc1_received;     // received frames which passed crc1 check, but not crc
#endif
    tStatsLQ valid_frames_received;   // received frames which also passed crc check

    tStatsLQ serial_data_transmitted; // frames with serial data transmitted, retransmissions are not counted
    tStatsLQ serial_data_received;    // frames with serial data received, retransmissions are not counted

    tStatsBytes bytes_transmitted;    // retransmissions are not counted
    tStatsBytes bytes_received;       // retransmissions are not counted

    // RF statistics for our device

    int8_t last_rssi1;
    int8_t last_rssi2;
    int8_t last_snr1;
    int8_t last_snr2;
    uint8_t last_antenna;
    uint8_t last_transmit_antenna;

    // statistics received from the other end

    int8_t received_rssi;
#ifdef DEVICE_IS_TRANSMITTER
    uint8_t received_LQ_rc;
#endif
    uint8_t received_LQ_serial;
    uint8_t received_antenna;
    uint8_t received_transmit_antenna;

    // seq no in the last transmitted frame

    uint8_t transmit_seq_no;

    // private fields

    uint16_t frame_rate_hz;
};


#endif // COMMON_STATS_H
