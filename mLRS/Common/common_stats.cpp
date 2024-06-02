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

void tStats::Init(uint16_t _frame_rate_hz)
{
    frame_rate_hz = _frame_rate_hz;

    frames_received.Init(frame_rate_hz);
#ifdef DEVICE_IS_RECEIVER
    valid_crc1_received.Init(frame_rate_hz);
#endif
    valid_frames_received.Init(frame_rate_hz);
    serial_data_transmitted.Init(frame_rate_hz);
    serial_data_received.Init(frame_rate_hz);
    bytes_transmitted.Init(frame_rate_hz);
    bytes_received.Init(frame_rate_hz);

    Clear();
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


