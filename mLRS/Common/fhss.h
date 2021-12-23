//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// FHSS
//*******************************************************
#ifndef FHSS_H
#define FHSS_H
#pragma once



#include <stdint.h>
#include "..\modules\sx12xx-lib\src\sx128x.h"
#include "common_conf.h"


#define FHSS_MAX_NUM    32

#if FHSS_NUM > FHSS_MAX_NUM
#error FHSS_NUM too large !
#endif

//-------------------------------------------------------
// Frequency list
//-------------------------------------------------------
// 2406.0 ... 2473.0  in 1 MHz steps
// = 68 channels

const uint32_t freq_list[] = {
    SX1280_FREQ_GHZ_TO_REG(2.406), // channel 0
    SX1280_FREQ_GHZ_TO_REG(2.407),
    SX1280_FREQ_GHZ_TO_REG(2.408),
    SX1280_FREQ_GHZ_TO_REG(2.409),

    SX1280_FREQ_GHZ_TO_REG(2.410), // channel 4
    SX1280_FREQ_GHZ_TO_REG(2.411),
    SX1280_FREQ_GHZ_TO_REG(2.412),
    SX1280_FREQ_GHZ_TO_REG(2.413),
    SX1280_FREQ_GHZ_TO_REG(2.414),
    SX1280_FREQ_GHZ_TO_REG(2.415),
    SX1280_FREQ_GHZ_TO_REG(2.416),
    SX1280_FREQ_GHZ_TO_REG(2.417),
    SX1280_FREQ_GHZ_TO_REG(2.418),
    SX1280_FREQ_GHZ_TO_REG(2.419),

    SX1280_FREQ_GHZ_TO_REG(2.420), // channel 14
    SX1280_FREQ_GHZ_TO_REG(2.421),
    SX1280_FREQ_GHZ_TO_REG(2.422),
    SX1280_FREQ_GHZ_TO_REG(2.423),
    SX1280_FREQ_GHZ_TO_REG(2.424),
    SX1280_FREQ_GHZ_TO_REG(2.425),
    SX1280_FREQ_GHZ_TO_REG(2.426),
    SX1280_FREQ_GHZ_TO_REG(2.427),
    SX1280_FREQ_GHZ_TO_REG(2.428),
    SX1280_FREQ_GHZ_TO_REG(2.429),

    SX1280_FREQ_GHZ_TO_REG(2.430), // channel 24
    SX1280_FREQ_GHZ_TO_REG(2.431),
    SX1280_FREQ_GHZ_TO_REG(2.432),
    SX1280_FREQ_GHZ_TO_REG(2.433),
    SX1280_FREQ_GHZ_TO_REG(2.434),
    SX1280_FREQ_GHZ_TO_REG(2.435),
    SX1280_FREQ_GHZ_TO_REG(2.436),
    SX1280_FREQ_GHZ_TO_REG(2.437),
    SX1280_FREQ_GHZ_TO_REG(2.438),
    SX1280_FREQ_GHZ_TO_REG(2.439),

    SX1280_FREQ_GHZ_TO_REG(2.440), // channel 34
    SX1280_FREQ_GHZ_TO_REG(2.441),
    SX1280_FREQ_GHZ_TO_REG(2.442),
    SX1280_FREQ_GHZ_TO_REG(2.443),
    SX1280_FREQ_GHZ_TO_REG(2.444),
    SX1280_FREQ_GHZ_TO_REG(2.445),
    SX1280_FREQ_GHZ_TO_REG(2.446),
    SX1280_FREQ_GHZ_TO_REG(2.447),
    SX1280_FREQ_GHZ_TO_REG(2.448),
    SX1280_FREQ_GHZ_TO_REG(2.449),

    SX1280_FREQ_GHZ_TO_REG(2.450), // channel 44
    SX1280_FREQ_GHZ_TO_REG(2.451),
    SX1280_FREQ_GHZ_TO_REG(2.452),
    SX1280_FREQ_GHZ_TO_REG(2.453),
    SX1280_FREQ_GHZ_TO_REG(2.454),
    SX1280_FREQ_GHZ_TO_REG(2.455),
    SX1280_FREQ_GHZ_TO_REG(2.456),
    SX1280_FREQ_GHZ_TO_REG(2.457),
    SX1280_FREQ_GHZ_TO_REG(2.458),
    SX1280_FREQ_GHZ_TO_REG(2.459),

    SX1280_FREQ_GHZ_TO_REG(2.460), // channel 54
    SX1280_FREQ_GHZ_TO_REG(2.461),
    SX1280_FREQ_GHZ_TO_REG(2.462),
    SX1280_FREQ_GHZ_TO_REG(2.463),
    SX1280_FREQ_GHZ_TO_REG(2.464),
    SX1280_FREQ_GHZ_TO_REG(2.465),
    SX1280_FREQ_GHZ_TO_REG(2.466),
    SX1280_FREQ_GHZ_TO_REG(2.467),
    SX1280_FREQ_GHZ_TO_REG(2.468),
    SX1280_FREQ_GHZ_TO_REG(2.469),

    SX1280_FREQ_GHZ_TO_REG(2.470), // channel 64
    SX1280_FREQ_GHZ_TO_REG(2.471),
    SX1280_FREQ_GHZ_TO_REG(2.472),
    SX1280_FREQ_GHZ_TO_REG(2.473), // channel 67
};

const uint8_t FREQ_LIST_LEN = (uint16_t)(sizeof(freq_list)/sizeof(uint32_t)); // = 68


//-------------------------------------------------------
// FHSS Class
//-------------------------------------------------------

class FhssBase
{
  public:

#ifdef FHSS_DISABLED
  // dummy class
  void Init(uint32_t seed) { generate(seed); }
  void StartRx(void) {}
  void StartTx(void) {}
  uint32_t GetCurr(void) { return fhss_list[0]; }
  void HopToNext(void) {}
  void HopToConnect(void) {}
#else

    void Init(uint32_t seed)
    {
      cnt = FHSS_NUM;

      generate(seed);
    }

    void StartRx(void)
    {
      curr_i = 0;
    }

    void StartTx(void)
    {
      curr_i = 0;
    }

    uint8_t Cnt(void)
    {
      return cnt;
    }

    uint32_t GetCurr(void)
    {
      return fhss_list[curr_i];
    }

    void HopToNext(void)
    {
      curr_i++;
      if (curr_i >= cnt) curr_i = 0;
    }

    void HopToConnect(void) // we could (should?) implement a more sophisticated procedure
    {
      curr_i = 0;
    }

    uint32_t bestX(void)
    {
      uint8_t i_best = 0;
      for (uint8_t i = 0; i < cnt; i++) {
        if (fhss_last_rssi[i] > fhss_last_rssi[i_best]) i_best = i;
      }

      curr_i = i_best;
      return fhss_list[curr_i];
    }
#endif

//  private:
    uint32_t _seed;

    uint8_t curr_i;
    uint8_t cnt;
    uint32_t fhss_list[FHSS_MAX_NUM]; // that's our list of randomly selected frequencies
    int8_t fhss_last_rssi[FHSS_MAX_NUM];

    uint16_t prng(void);
    void generate(uint32_t seed);
};




#endif // FHSS_H
