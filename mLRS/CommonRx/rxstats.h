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

class RxStatsBase
{
  public:
    void Init(uint8_t _period);

    void Update1Hz(void);
    void Clear(void);
    void Next(void);
    void Set(void);

    void doFrameReceived(void);
    void doValidCrc1FrameReceived(void);
    void doValidFrameReceived(void);

    uint8_t GetRawLQ(void);
    uint8_t GetNormalizedLQ(void);
    uint8_t GetLQ(void);

    uint8_t GetLQ_rc_data(void) { return 0x7F; }

private:
    bool valid_crc1_frame_received;
    bool valid_frame_received;
    bool frame_received;

    LqCounterBase valid_crc1_lq;
    LqCounterBase valid_lq;
    LqCounterBase received_lq;

    virtual bool is_connected(void);
};


void RxStatsBase::Init(uint8_t _period)
{
    stats.Init();

    valid_crc1_lq.Init(_period);
    valid_lq.Init(_period);
    received_lq.Init(_period);

    Clear();
}


void RxStatsBase::Update1Hz(void)
{
    stats.Update1Hz();
}


void RxStatsBase::Clear(void)
{
    valid_crc1_frame_received = false;
    valid_frame_received = false;
    frame_received = false;
}


void RxStatsBase::Next(void)
{
    valid_crc1_lq.Next();
    valid_lq.Next();
    received_lq.Next();
}


void RxStatsBase::Set(void)
{
    if (frame_received) received_lq.Set();
    if (valid_crc1_frame_received) valid_crc1_lq.Set();
    if (valid_frame_received) valid_lq.Set();
}


void RxStatsBase::doFrameReceived(void)
{
    frame_received = true;
    stats.frames_received++;
}


void RxStatsBase::doValidCrc1FrameReceived(void)
{
    valid_crc1_frame_received = true;
    stats.valid_crc1_received++;
}


void RxStatsBase::doValidFrameReceived(void)
{
    valid_frame_received = true;
    stats.valid_frames_received++;
}


uint8_t RxStatsBase::GetRawLQ(void)
{
    return valid_lq.GetRaw();
}


uint8_t RxStatsBase::GetNormalizedLQ(void)
{
    return valid_lq.GetNormalized();
}


uint8_t RxStatsBase::GetLQ(void)
{
    if (!is_connected()) return 0;
    if (stats.LQ == 0) return 1;
    return stats.LQ;
}



#endif // RXSTATS_H
