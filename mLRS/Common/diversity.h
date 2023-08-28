//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Diversity
//********************************************************
#ifndef DIVERSITY_H
#define DIVERSITY_H
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


#endif // DIVERSITY_H
