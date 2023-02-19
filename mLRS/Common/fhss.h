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
#include "hal/device_conf.h"
#include "sx-drivers/sx12xx.h"
#include "common_conf.h"
#include "setup_types.h"


#define FHSS_MAX_NUM            32
#define FHSS_FREQ_LIST_MAX_LEN  80 // 2.4 GHz is 68

//-------------------------------------------------------
// Frequency list
//-------------------------------------------------------

#ifdef FREQUENCY_BAND_433_MHZ
#define FHSS_HAS_CONFIG_433_MHZ
#endif
#if defined FREQUENCY_BAND_868_MHZ
#define FHSS_HAS_CONFIG_868_MHZ
#endif
#if defined FREQUENCY_BAND_915_MHZ_FCC
#define FHSS_HAS_CONFIG_915_MHZ_FCC
#endif
#ifdef FREQUENCY_BAND_2P4_GHZ
#define FHSS_HAS_CONFIG_2P4_GHZ
#endif


#ifdef DEVICE_HAS_SX126x
#define SX12XX_FREQ_MHZ_TO_REG(f_mhz)  SX126X_FREQ_MHZ_TO_REG(f_mhz)
#elif defined DEVICE_HAS_SX127x
#define SX12XX_FREQ_MHZ_TO_REG(f_mhz)  SX127X_FREQ_MHZ_TO_REG(f_mhz)
#endif


#ifdef FHSS_HAS_CONFIG_433_MHZ
// 433.050 ... 434.790 in 0.506 MHz steps

const uint32_t fhss_freq_list_433[] = {
    SX12XX_FREQ_MHZ_TO_REG(433.360),
    SX12XX_FREQ_MHZ_TO_REG(433.920),
    SX12XX_FREQ_MHZ_TO_REG(433.480),
};

const uint8_t fhss_bind_channel_list_433[] = {
    0, // just pick some
};

#endif
#ifdef FHSS_HAS_CONFIG_868_MHZ
// 863.275 ... 869.575  in 0.525 MHz steps

const uint32_t fhss_freq_list_868[] = {
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

const uint8_t fhss_bind_channel_list_868[] = {
    0, // just pick some
};

#endif
#ifdef FHSS_HAS_CONFIG_915_MHZ_FCC
// based on ExpressLRS

const uint32_t fhss_freq_list_915_fcc[] = {
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
    
    SX12XX_FREQ_MHZ_TO_REG(915.5), //added

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

    SX12XX_FREQ_MHZ_TO_REG(926.9), //added
};

const uint8_t fhss_bind_channel_list_915_fcc[] = {
    19 // just pick some
};

#endif
#ifdef FHSS_HAS_CONFIG_2P4_GHZ
// 2406.0 ... 2473.0  in 1 MHz steps
// = 68 channels

const uint32_t fhss_freq_list_2p4[] = {
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


const uint8_t fhss_bind_channel_list_2p4[] = {
    46, 14, 33, 61 // just pick some
};
#endif


//-------------------------------------------------------
// FHSS Class
//-------------------------------------------------------

typedef enum {
    FHSS_CONFIG_2P4_GHZ = 0,
    FHSS_CONFIG_915_MHZ_FCC,
    FHSS_CONFIG_868_MHZ,
    FHSS_CONFIG_433_MHZ,
    FHSS_CONFIG_NUM,
} FHSS_CONFIG_ENUM;


typedef struct {
    const uint32_t* freq_list;
    uint8_t freq_list_len;
    const uint8_t* bind_channel_list;
    uint8_t bind_channel_list_len;
} tFhssConfig;


const tFhssConfig fhss_config[] = {
#ifdef FHSS_HAS_CONFIG_2P4_GHZ
    {
        .freq_list = fhss_freq_list_2p4,
        .freq_list_len = (uint8_t)(sizeof(fhss_freq_list_2p4) / sizeof(uint32_t)),
        .bind_channel_list = fhss_bind_channel_list_2p4,
        .bind_channel_list_len = (uint8_t)(sizeof(fhss_bind_channel_list_2p4) / sizeof(uint8_t))
    },
#else
    { .freq_list = nullptr },
#endif
#ifdef FHSS_HAS_CONFIG_915_MHZ_FCC
    {
        .freq_list = fhss_freq_list_915_fcc,
        .freq_list_len = (uint8_t)(sizeof(fhss_freq_list_915_fcc) / sizeof(uint32_t)),
        .bind_channel_list = fhss_bind_channel_list_915_fcc,
        .bind_channel_list_len = (uint8_t)(sizeof(fhss_bind_channel_list_915_fcc) / sizeof(uint8_t))
    },
#else
    { .freq_list = nullptr },
#endif
#ifdef FHSS_HAS_CONFIG_868_MHZ
    {
        .freq_list = fhss_freq_list_868,
        .freq_list_len = (uint8_t)(sizeof(fhss_freq_list_868) / sizeof(uint32_t)),
        .bind_channel_list = fhss_bind_channel_list_868,
        .bind_channel_list_len = (uint8_t)(sizeof(fhss_bind_channel_list_868) / sizeof(uint8_t))
    },
#else
    { .freq_list = nullptr },
#endif
#ifdef FHSS_HAS_CONFIG_BAND_433_MHZ
    { Xhkahsdkhasd }, // to force an error
#else
    { .freq_list = nullptr },
#endif
};


class FhssBase
{
  public:
    void Init(uint8_t fhss_num, uint32_t seed, uint8_t frequency_band)
    {
        if (fhss_num > FHSS_MAX_NUM) while (1) {} // should not happen, but play it safe

        switch (frequency_band) {
        case SETUP_FREQUENCY_BAND_2P4_GHZ: config_i = FHSS_CONFIG_2P4_GHZ; break;
        case SETUP_FREQUENCY_BAND_915_MHZ_FCC: config_i = FHSS_CONFIG_915_MHZ_FCC; break;
        case SETUP_FREQUENCY_BAND_868_MHZ: config_i = FHSS_CONFIG_868_MHZ; break;
        // case SETUP_FREQUENCY_BAND_433_MHZ: config_i = FHSS_CONFIG_433_MHZ; break; // is not yet existing !!
        default:
          while (1) {} // should not happen, but play it safe
        }

        if (fhss_config[config_i].freq_list == nullptr) while (1) {} // should not happen, but play it safe

        fhss_freq_list = fhss_config[config_i].freq_list;
        FREQ_LIST_LEN = fhss_config[config_i].freq_list_len;
        fhss_bind_channel_list = fhss_config[config_i].bind_channel_list;
        BIND_CHANNEL_LIST_LEN = fhss_config[config_i].bind_channel_list_len;
        curr_bind_config_i = config_i; // we start with what setup suggests

        cnt = fhss_num;

        uint8_t cnt_max = (FREQ_LIST_LEN - BIND_CHANNEL_LIST_LEN);
        if (cnt > cnt_max) cnt = cnt_max;

        generate(seed);

        is_in_binding = false;

        curr_i = 0;
    }

    void Start(void)
    {
        curr_i = 0;
    }

    uint8_t Cnt(void)
    {
        return cnt;
    }

    uint8_t CurrI(void)
    {
        return curr_i;
    }

    uint32_t GetCurrFreq(void)
    {
        if (is_in_binding) {
            const uint32_t* curr_bind_freq_list = fhss_config[curr_bind_config_i].freq_list;
            const uint8_t* curr_bind_channel_list = fhss_config[curr_bind_config_i].bind_channel_list;
            return curr_bind_freq_list[curr_bind_channel_list[0]];
        }

        return fhss_list[curr_i];
    }

    void HopToNext(void)
    {
        curr_i++;
        if (curr_i >= cnt) curr_i = 0;
    }

    void SetToBind(uint16_t frame_rate_ms = 1) // preset so it is good for transmitter
    {
        is_in_binding = true;
        bind_listen_cnt = (5000 / frame_rate_ms); // should be 5 secs
        bind_listen_i = 0;
    }

    // only used by receiver, bool determines if it needs to switch back to LINK_STATE_RECEIVE
    bool HopToNextBind(void)
    {
        if (!is_in_binding) return false;

        bind_listen_i++;
        if (bind_listen_i >= bind_listen_cnt) {
            bind_listen_i = 0;
            // find next bind frequency
            uint8_t iii = curr_bind_config_i;
            for (uint8_t i = 0; i < FHSS_CONFIG_NUM; i++) { // we give it at most that much attempts
                iii++;
                if (iii >= FHSS_CONFIG_NUM) iii = 0;
                if (fhss_config[iii].freq_list != nullptr) { curr_bind_config_i = iii; return true; }
            }
        }

        return false;
    }

    // only used by receiver
    uint8_t GetCurrFrequencyBand(void)
    {
        switch (curr_bind_config_i) {
        case FHSS_CONFIG_2P4_GHZ: return SETUP_FREQUENCY_BAND_2P4_GHZ;
        case FHSS_CONFIG_915_MHZ_FCC: return SETUP_FREQUENCY_BAND_915_MHZ_FCC;
        case FHSS_CONFIG_868_MHZ: return SETUP_FREQUENCY_BAND_868_MHZ;
        // case FHSS_CONFIG_433_MHZ: return SETUP_FREQUENCY_BAND_433_MHZ; // is not yet existing !!
        }
        while (1) {} // should not happen, but play it safe
        return 0;
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

  private:
    uint32_t _seed;

    uint8_t config_i;
    uint8_t FREQ_LIST_LEN;
    const uint32_t* fhss_freq_list;
    uint8_t BIND_CHANNEL_LIST_LEN;
    const uint8_t* fhss_bind_channel_list;

    uint8_t curr_i;
    uint8_t cnt;
    uint8_t ch_list[FHSS_MAX_NUM]; // that's our list of randomly selected channels
    uint32_t fhss_list[FHSS_MAX_NUM]; // that's our list of randomly selected frequencies

    int8_t fhss_last_rssi[FHSS_MAX_NUM];

    bool is_in_binding;
    uint8_t curr_bind_config_i;
    uint16_t bind_listen_cnt;
    uint16_t bind_listen_i;

    uint16_t prng(void);
    void generate(uint32_t seed);
};


#endif // FHSS_H
