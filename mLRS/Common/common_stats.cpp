//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Common Statistics
//*******************************************************

#include "common_stats.h"


//-------------------------------------------------------
// Common stats
//-------------------------------------------------------

void tStats::Init(uint8_t _maverage_period, uint16_t _frame_rate_hz)
{
    frames_received.Init(_frame_rate_hz);
#ifdef DEVICE_IS_RECEIVER
    valid_crc1_received.Init(_frame_rate_hz);
#endif
    valid_frames_received.Init(_frame_rate_hz);
    serial_data_transmitted.Init(_frame_rate_hz);
    serial_data_received.Init(_frame_rate_hz);
    bytes_transmitted.Init(_frame_rate_hz);
    bytes_received.Init(_frame_rate_hz);

    Clear();

#ifdef DEVICE_IS_TRANSMITTER
    rx1_valid = false;
    rx2_valid = false;
    fhss_curr_i = UINT8_MAX;
#endif

//    LQma_valid_crc1.Init(_maverage_period);
//    LQma_valid.Init(_maverage_period);
//    LQma_received.Init(_maverage_period);
}


void tStats::Clear(void) // called then not connected
{
    last_rssi1 = RSSI_INVALID;
    last_rssi2 = RSSI_INVALID;
    last_snr1 = SNR_INVALID;
    last_snr2 = SNR_INVALID;
    last_antenna = UINT8_MAX;
    last_transmit_antenna = UINT8_MAX;

    received_rssi = RSSI_INVALID;
#ifdef DEVICE_IS_TRANSMITTER
    received_LQ_rc = 0; //UINT8_MAX;
#endif
    received_LQ_serial = 0; //UINT8_MAX;
    received_antenna = UINT8_MAX;
    received_transmit_antenna = UINT8_MAX;

    transmit_seq_no = 0;
}


void tStats::Update1Hz(void)
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


void tStats::Next(void)
{
//    LQma_valid_crc1.Next();
//    LQma_valid.Next();
//    LQma_received.Next();
}


void tStats::doFrameReceived(void)
{
    frames_received.Inc();

//    LQma_received.Set();
}


#ifdef DEVICE_IS_RECEIVER
void tStats::doValidCrc1FrameReceived(void)
{
    valid_crc1_received.Inc();

//    LQma_valid_crc1.Set();
}
#endif


void tStats::doValidFrameReceived(void)
{
    valid_frames_received.Inc();

//    LQma_valid.Set();
}


uint8_t tStats::GetTransmitBandwidthUsage(void)
{
    // just simply scale it always to the largest theoretical bandwidth
    // is 4100 bytes/s max
    uint32_t bps = bytes_transmitted.GetBytesPerSec();
    uint8_t bw =  (bps + 20) / 41;
    if ((bw == 0) && (bps > 0)) bw = 1; // ensure it is always at least 1% if some bytes are transmitted
    return bw;
}


uint8_t tStats::GetReceiveBandwidthUsage(void)
{
    // just simply scale it always to the largest theoretical bandwidth
    // is 4100 bytes/s max
    uint32_t bps = bytes_received.GetBytesPerSec();
    uint8_t bw =  (bps + 20) / 41;
    if ((bw == 0) && (bps > 0)) bw = 1; // ensure it is always at least 1% if some bytes are received
    return bw;
}


int8_t tStats::GetLastRssi(void)
{
    return (last_antenna == ANTENNA_1) ? last_rssi1 : last_rssi2;
}


int8_t tStats::GetLastSnr(void)
{
    return (last_antenna == ANTENNA_1) ? last_snr1 : last_snr2;
}


#ifdef DEVICE_IS_RECEIVER
uint8_t tStats::GetLQ_rc(void)
{
    if (!connected()) return 0;

    uint8_t LQ = valid_crc1_received.GetLQ();
    if (LQ == 0) return 1;
    return LQ;
}
#endif


uint8_t tStats::GetLQ_serial(void)
{
    if (!connected()) return 0;

    uint8_t LQser = serial_data_received.GetLQ();
    if (LQser == 0) return 1;
    return LQser;
}

