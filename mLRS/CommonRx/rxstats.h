//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Statistics for Receiver
//********************************************************
#ifndef RXSTATS_H
#define RXSTATS_H
#pragma once


//-------------------------------------------------------
// we also handle the stats field with this class, this is somewhat dirty

class RxStats
{
  public:

    bool valid_crc1_frame_received;
    bool valid_frame_received;
    bool frame_received;

    LqCounterBase valid_crc1_lq;
    LqCounterBase valid_lq;
    LqCounterBase received_lq;

    void Init(uint8_t _period);
    void Update1Hz(void);
    void Reset(void);

    void Next(void);
    void Set(void);

    void doFrameReceived(void);
    void doValidCrc1FrameReceived(void);
    void doValidFrameReceived(void);
    void doFrameTransmitted(void);

    uint8_t GetRawLQ(void);
    uint8_t GetNormalizedLQ(void);
    // since we have full and crc1 LQ, this uses a more sophisticated algorithm to provide an LQ
    uint8_t GetLQ(bool is_connected);
};



void RxStats::Init(uint8_t _period)
{
    stats.Init();

    valid_crc1_lq.Init(_period);
    valid_lq.Init(_period);
    received_lq.Init(_period);

    Reset();
}


void RxStats::Update1Hz(void)
{
    stats.Update();
}


void RxStats::Reset(void)
{
    valid_crc1_frame_received = false;
    valid_frame_received = false;
    frame_received = false;
}


void RxStats::Next(void)
{
    valid_crc1_lq.Next();
    valid_lq.Next();
    received_lq.Next();
}


void RxStats::Set(void)
{
    if (frame_received) received_lq.Set();
    if (valid_crc1_frame_received) valid_crc1_lq.Set();
    if (valid_frame_received) valid_lq.Set();
}


void RxStats::doFrameReceived(void)
{
    frame_received = true;
    stats.frames_received++;
}


void RxStats::doValidCrc1FrameReceived(void)
{
    valid_crc1_frame_received = true;
    stats.valid_crc1_frames_received++;
}


void RxStats::doValidFrameReceived(void)
{
    valid_frame_received = true;
    stats.valid_frames_received++;
}


void RxStats::doFrameTransmitted(void)
{
    stats.frames_transmitted++;
}


uint8_t RxStats::GetRawLQ(void)
{
    return valid_lq.GetRaw();
}


uint8_t RxStats::GetNormalizedLQ(void)
{
    return valid_lq.GetNormalized();
}


// since we have full and crc1 LQ, this uses a more sophisticated algorithm to provide an LQ
uint8_t RxStats::GetLQ(bool is_connected)
{
    if (!is_connected) return 0;

    uint8_t valid_LQ = valid_lq.GetNormalized();
    if (valid_LQ >= 25) return valid_LQ;

    uint8_t valid_crc1_LQ = valid_crc1_lq.GetNormalized(); // we can expect that always valid_crc1_LQ >= valid_LQ

    if (valid_crc1_LQ >= 25) return 25;
    if (valid_crc1_LQ == 0) return 1; // when connected LQ should always be > 0, as it means that we receive "something"
    return valid_crc1_LQ;
}



#endif // RXSTATS_H
