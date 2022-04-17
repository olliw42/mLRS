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
#include "hal\device_conf.h"
#include "sx-drivers\sx12xx.h"
#include "common_conf.h"


#define FHSS_MAX_NUM    32


//-------------------------------------------------------
// Frequency list
//-------------------------------------------------------
#ifdef DEVICE_HAS_SX126x
#define SX12XX_FREQ_MHZ_TO_REG(f_mhz)  SX126X_FREQ_MHZ_TO_REG(f_mhz)
#elif defined DEVICE_HAS_SX127x
#define SX12XX_FREQ_MHZ_TO_REG(f_mhz)  SX127X_FREQ_MHZ_TO_REG(f_mhz)
#endif

#ifdef FREQUENCY_BAND_433_MHZ
// 433.050 ... 434.790 in 0.506 MH steps

const uint32_t fhss_freq_list[] = {
    SX12XX_FREQ_MHZ_TO_REG(433.360),
    SX12XX_FREQ_MHZ_TO_REG(433.920),
    SX12XX_FREQ_MHZ_TO_REG(433.480),
};

const uint8_t fhss_bind_channel_list[] = {
    0, // just pick some
};

#endif
#ifdef FREQUENCY_BAND_868_MHZ
// 863.275 ... 869.575  in 0.525 MHz steps

const uint32_t fhss_freq_list[] = {
    SX12XX_FREQ_MHZ_TO_REG(863.275),
    SX12XX_FREQ_MHZ_TO_REG(863.800),
    SX12XX_FREQ_MHZ_TO_REG(864.325),
    SX12XX_FREQ_MHZ_TO_REG(864.850),
    SX12XX_FREQ_MHZ_TO_REG(865.375),
    SX12XX_FREQ_MHZ_TO_REG(865.900),
    SX12XX_FREQ_MHZ_TO_REG(866.425),
    SX12XX_FREQ_MHZ_TO_REG(866.950),
    SX12XX_FREQ_MHZ_TO_REG(867.475),
    SX12XX_FREQ_MHZ_TO_REG(868.000),

    // SX12XX_FREQ_MHZ_TO_REG(868.525), // overlap with Alarmanlagen
    // SX12XX_FREQ_MHZ_TO_REG(869.050), // overlap with Alarmanlagen
    // SX12XX_FREQ_MHZ_TO_REG(869.575), // overlap with Alarmanlagen
};

const uint8_t fhss_bind_channel_list[] = {
    0, // just pick some
};

#endif
#ifdef FREQUENCY_BAND_915_MHZ_FCC
// based on ExpressLRS

const uint32_t fhss_freq_list[] = {
    SX12XX_FREQ_MHZ_TO_REG(903.5),
    SX12XX_FREQ_MHZ_TO_REG(904.1),
    SX12XX_FREQ_MHZ_TO_REG(904.7),
    SX12XX_FREQ_MHZ_TO_REG(905.3),
    SX12XX_FREQ_MHZ_TO_REG(905.9),
    SX12XX_FREQ_MHZ_TO_REG(906.5),
    SX12XX_FREQ_MHZ_TO_REG(907.1),
    SX12XX_FREQ_MHZ_TO_REG(907.7),
    SX12XX_FREQ_MHZ_TO_REG(908.3),
    SX12XX_FREQ_MHZ_TO_REG(908.9),

    SX12XX_FREQ_MHZ_TO_REG(909.5),
    SX12XX_FREQ_MHZ_TO_REG(910.1),
    SX12XX_FREQ_MHZ_TO_REG(910.7),
    SX12XX_FREQ_MHZ_TO_REG(911.3),
    SX12XX_FREQ_MHZ_TO_REG(911.9),
    SX12XX_FREQ_MHZ_TO_REG(912.5),
    SX12XX_FREQ_MHZ_TO_REG(913.1),
    SX12XX_FREQ_MHZ_TO_REG(913.7),
    SX12XX_FREQ_MHZ_TO_REG(914.3),
    SX12XX_FREQ_MHZ_TO_REG(914.9),

    SX12XX_FREQ_MHZ_TO_REG(916.1),
    SX12XX_FREQ_MHZ_TO_REG(916.7),
    SX12XX_FREQ_MHZ_TO_REG(917.3),
    SX12XX_FREQ_MHZ_TO_REG(917.9),
    SX12XX_FREQ_MHZ_TO_REG(918.5),
    SX12XX_FREQ_MHZ_TO_REG(919.1),
    SX12XX_FREQ_MHZ_TO_REG(919.7),
    SX12XX_FREQ_MHZ_TO_REG(920.3),
    SX12XX_FREQ_MHZ_TO_REG(920.9),
    SX12XX_FREQ_MHZ_TO_REG(921.5),
    SX12XX_FREQ_MHZ_TO_REG(922.1),

    SX12XX_FREQ_MHZ_TO_REG(922.7),
    SX12XX_FREQ_MHZ_TO_REG(923.3),
    SX12XX_FREQ_MHZ_TO_REG(923.9),
    SX12XX_FREQ_MHZ_TO_REG(924.5),
    SX12XX_FREQ_MHZ_TO_REG(925.1),
    SX12XX_FREQ_MHZ_TO_REG(925.7),
    SX12XX_FREQ_MHZ_TO_REG(926.3),
};

const uint8_t fhss_bind_channel_list[] = {
    19 // just pick some
};

#endif
#ifdef FREQUENCY_BAND_2P4_GHZ
// 2406.0 ... 2473.0  in 1 MHz steps
// = 68 channels

const uint32_t fhss_freq_list[] = {
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


const uint8_t fhss_bind_channel_list[] = {
    14, 33, 46, 61 // just pick some
};
#endif


const uint8_t FREQ_LIST_LEN = (uint8_t)(sizeof(fhss_freq_list)/sizeof(uint32_t)); // 2.4 GHz = 68

const uint8_t FHSS_BIND_CHANNEL_LIST_LEN = (uint8_t)(sizeof(fhss_bind_channel_list)/sizeof(uint8_t));


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
#else

    void Init(uint8_t fhss_num, uint32_t seed)
    {
        if (fhss_num > FHSS_MAX_NUM) while (1) {} // should not happen, but play it safe

        cnt = fhss_num;

        uint8_t cnt_max = (FREQ_LIST_LEN - FHSS_BIND_CHANNEL_LIST_LEN);
        if (cnt > cnt_max) cnt = cnt_max;

        generate(seed);

        is_in_binding = false;
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

    uint32_t GetCurrFreq(void)
    {
        if (is_in_binding) return fhss_freq_list[fhss_bind_channel_list[0]];

        return fhss_list[curr_i];
    }

    void HopToNext(void)
    {
        curr_i++;
        if (curr_i >= cnt) curr_i = 0;
    }

    void SetToBind(void)
    {
        is_in_binding = true;
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
    uint8_t ch_list[FHSS_MAX_NUM]; // that's our list of randomly selected channels
    uint32_t fhss_list[FHSS_MAX_NUM]; // that's our list of randomly selected frequencies
    int8_t fhss_last_rssi[FHSS_MAX_NUM];
    bool is_in_binding;

    uint16_t prng(void);
    void generate(uint32_t seed);
};


#endif // FHSS_H
