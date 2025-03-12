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
#include "libs/filters.h"
#include "lq_counter.h"
#include "common_types.h"


extern bool connected(void);


//-------------------------------------------------------
// Common stats
//-------------------------------------------------------

class tStats
{
  public:
    void Init(uint8_t _maverage_period, uint16_t _frame_rate_hz, uint16_t _frame_rate_ms);

    void Update1Hz(void);             // called at 1 Hz
    void Next(void);                  // called at each cycle, is called when transmit starts, or shortly after
    void Clear(void);                 // called then not connected

    void doFrameReceived(void);
#ifdef DEVICE_IS_RECEIVER
    void doValidCrc1FrameReceived(void);
#endif
    void doValidFrameReceived(void);

    uint8_t GetTransmitBandwidthUsage(void);
    uint8_t GetReceiveBandwidthUsage(void);
    int8_t GetLastRssi(void);
    int8_t GetLastSnr(void);

#ifdef DEVICE_IS_RECEIVER
    uint8_t GetLQ_rc(void);           // this is the "main" LQ, in case of Rx reflects the crc1-rcdata LQ
#endif
    uint8_t GetLQ_serial(void);

    void doMavlinkCnt(bool valid);
    uint8_t GetMavlinkLQ(void);

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

    tStatsMavlinkLQ mav_packets_received;   // number of MAVLink packets received

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
#ifdef DEVICE_IS_RECEIVER
    uint8_t received_fhss_index_band;
    uint8_t received_fhss_index;
#endif
    uint8_t received_LQ_serial;
    uint8_t received_antenna;
    uint8_t received_transmit_antenna;

    // statistics for ARQ, LPF filter for the number of frames with fresh payload

    tLpFilter frame_cnt;
    void cntFrameTransmitted(void);
    void cntFrameSkipped(void);
    int32_t GetFrameCnt(void);

    // seq no in the last transmitted frame, only tx

    uint8_t transmit_seq_no;

    // extra stats available with mBridge
#ifdef DEVICE_IS_TRANSMITTER
    bool rx1_valid;
    bool rx2_valid;
    uint8_t fhss_curr_i;
#endif

    // moving average fields

//    tLqCounterBase LQma_received;
//    tLqCounterBase LQma_valid_crc1;
//    tLqCounterBase LQma_valid;
};


#endif // COMMON_STATS_H
