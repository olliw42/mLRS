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
    void Next(void);

    void doFrameReceived(void);
    void doValidCrc1FrameReceived(void);
    void doValidFrameReceived(void);

    uint8_t GetRawLQ(void);
    uint8_t GetNormalizedLQ(void);
    uint8_t GetLQ(void); // this is the "main" LQ, in case of Rx reflects the crc1-rcdata LQ
    uint8_t GetLQ_rc_data(void) { return 0x7F; }

  private:
    LqCounterBase LQma_received;
    LqCounterBase LQma_valid_crc1;
    LqCounterBase LQma_valid;

    virtual bool is_connected(void);
};


void RxStatsBase::Init(uint8_t _period)
{
    stats.Init();

    LQma_valid_crc1.Init(_period);
    LQma_valid.Init(_period);
    LQma_received.Init(_period);
}


void RxStatsBase::Update1Hz(void)
{
    stats.Update1Hz();
}


void RxStatsBase::Next(void) // this is called when transmit starts, or shortly after
{
    LQma_valid_crc1.Next();
    LQma_valid.Next();
    LQma_received.Next();

    if (!is_connected()) { // start with 100% if not connected
      LQma_valid_crc1.Reset();
      LQma_valid.Reset();
      LQma_received.Reset();
    }
}


void RxStatsBase::doFrameReceived(void)
{
    LQma_received.Set();
    stats.frames_received++;
}


void RxStatsBase::doValidCrc1FrameReceived(void)
{
    LQma_valid_crc1.Set();
    stats.valid_crc1_received++;
}


void RxStatsBase::doValidFrameReceived(void)
{
    LQma_valid.Set();
    stats.valid_frames_received++;
}


uint8_t RxStatsBase::GetRawLQ(void)
{
    return LQma_valid.GetRaw();
}


uint8_t RxStatsBase::GetNormalizedLQ(void)
{
    return LQma_valid.GetNormalized();
}


uint8_t RxStatsBase::GetLQ(void)
{
    if (!is_connected()) return 0;
    uint8_t LQ = stats.GetLQ();
    if (LQ == 0) return 1;
    return LQ;
}



#endif // RXSTATS_H
