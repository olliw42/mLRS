//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Setup and configuration types
//*******************************************************

#include <stdint.h>
#include "setup_types.h"


SX_FHSS_FREQUENCY_BAND_ENUM cvt_to_sx_fhss_frequency_band(uint8_t setup_frequency_band)
{
    switch (setup_frequency_band) {
        case SETUP_FREQUENCY_BAND_2P4_GHZ: return SX_FHSS_FREQUENCY_BAND_2P4_GHZ;
        case SETUP_FREQUENCY_BAND_915_MHZ_FCC: return SX_FHSS_FREQUENCY_BAND_915_MHZ_FCC;
        case SETUP_FREQUENCY_BAND_868_MHZ: return SX_FHSS_FREQUENCY_BAND_868_MHZ;
        case SETUP_FREQUENCY_BAND_433_MHZ: return SX_FHSS_FREQUENCY_BAND_433_MHZ;
        case SETUP_FREQUENCY_BAND_70_CM_HAM: return SX_FHSS_FREQUENCY_BAND_70_CM_HAM;
        case SETUP_FREQUENCY_BAND_866_MHZ_IN: return SX_FHSS_FREQUENCY_BAND_866_MHZ_IN;
        case SETUP_FREQUENCY_DUAL_BAND_915_MHZ_2P4_GHZ: return SX_FHSS_FREQUENCY_BAND_915_MHZ_FCC;
        case SETUP_FREQUENCY_DUAL_BAND_866_MHZ_2P4_GHZ: return SX_FHSS_FREQUENCY_BAND_868_MHZ;
        default: while(1){}
    }
}


SETUP_FREQUENCY_BAND_ENUM cvt_to_setup_frequency_band(uint8_t sx_fhss_frequency_band)
{
    switch (sx_fhss_frequency_band) {
        case SX_FHSS_FREQUENCY_BAND_2P4_GHZ: return SETUP_FREQUENCY_BAND_2P4_GHZ;
        case SX_FHSS_FREQUENCY_BAND_915_MHZ_FCC: return SETUP_FREQUENCY_BAND_915_MHZ_FCC;
        case SX_FHSS_FREQUENCY_BAND_868_MHZ: return SETUP_FREQUENCY_BAND_868_MHZ;
        case SX_FHSS_FREQUENCY_BAND_433_MHZ: return SETUP_FREQUENCY_BAND_433_MHZ;
        case SX_FHSS_FREQUENCY_BAND_70_CM_HAM: return SETUP_FREQUENCY_BAND_70_CM_HAM;
        case SX_FHSS_FREQUENCY_BAND_866_MHZ_IN: return SETUP_FREQUENCY_BAND_866_MHZ_IN;
        default: while(1){}
    }
}


