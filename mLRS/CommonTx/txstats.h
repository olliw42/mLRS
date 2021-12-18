//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Statistics for Transmitter
//********************************************************
#ifndef TXSTATS_H
#define TXSTATS_H
#pragma once


//-------------------------------------------------------

class TxStats
{
  public:
    void Init(uint8_t _period);

    void Update1Hz(void);
    void Next(void);

    void SetFrameReceived(void);
    void SetValidFrameReceived(void);
    void SetFrameTransmitted(void);

    uint8_t GetRawLQ(void);
    uint8_t GetNormalizedLQ(void);
    uint8_t GetLQ(bool is_connected = true);
    uint8_t GetFramesReceivedLQ(void);

  private:
    LqCounterBase valid_lq;
    LqCounterBase received_lq;
};


void TxStats::Init(uint8_t _period)
{
    stats.Init();

    valid_lq.Init(_period);
    received_lq.Init(_period);
}


void TxStats::Update1Hz(void)
{
    stats.Update();
}


void TxStats::Next(void)
{
    valid_lq.Next();
    received_lq.Next();
}


void TxStats::SetFrameReceived(void)
{
    received_lq.Set();
    stats.frames_received++;
}


void TxStats::SetValidFrameReceived(void)
{
    valid_lq.Set();
    stats.valid_frames_received++;
}


void TxStats::SetFrameTransmitted(void)
{
    stats.frames_transmitted++;
}


uint8_t TxStats::GetRawLQ(void)
{
    return valid_lq.GetRaw();
}


uint8_t TxStats::GetNormalizedLQ(void)
{
    return valid_lq.GetNormalized();
}


uint8_t TxStats::GetLQ(bool is_connected)
{
    if (!is_connected) return 0;

    uint8_t valid_LQ = valid_lq.GetNormalized();
    if (valid_LQ == 0) return 1; // when connected LQ should always be > 0, as it means that we receive "something"
    return valid_LQ;
}


uint8_t TxStats::GetFramesReceivedLQ(void)
{
    return received_lq.GetNormalized();
}



#endif // TXSTATS_H
