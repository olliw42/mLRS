//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Diversity
//********************************************************
#ifndef TDIVERSITY_H
#define TDIVERSITY_H
#pragma once


#include <inttypes.h>


//-------------------------------------------------------
// Receive Diversity Estimator Class
//-------------------------------------------------------

class tRDiversity
{
  public:
    void Init(void) {}
    uint8_t Antenna(uint8_t link_rx1_status, uint8_t link_rx2_status, int8_t rssi1, int8_t rssi2);
};


//-------------------------------------------------------
// Transmit Diversity Estimator Class
//-------------------------------------------------------

class tTDiversity
{
  public:
    void Init(uint16_t _frame_rate_ms);
    void DoEstimate(uint8_t link_rx1_status, uint8_t link_rx2_status, int8_t rssi1, int8_t rssi2);
    uint8_t Antenna(void) { return proposed_antenna; }
    void SetAntenna(uint8_t antenna) { proposed_antenna = antenna; }

//XX  private:
    int16_t frame_rate_ms;

    int16_t estimator_value;
    int16_t estimator_step_last;
    uint8_t invalid1_cnt;
    uint8_t invalid2_cnt;

    uint8_t proposed_antenna;

    uint32_t _seed;
    uint16_t prng(void);
};


#endif // TDIVERSITY_H
