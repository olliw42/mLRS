//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// rx hal splicer for ESP targets
//*******************************************************

//-------------------------------------------------------
// ESP Boards
//-------------------------------------------------------

//-- ELRS 868/915 MHz Generic Devices

#ifdef RX_ELRS_GENERIC_900_ESP8285
#include "rx-hal-generic-900-esp8285.h"
#endif

#ifdef RX_ELRS_GENERIC_900_PA_ESP8285
#include "rx-hal-generic-900-pa-esp8285.h"
#endif

#ifdef RX_ELRS_GENERIC_900_TD_PA_ESP32
#include "rx-hal-generic-900-td-pa-esp32.h"
#endif

#ifdef RX_ELRS_GENERIC_LR1121_TD_ESP32
#include "rx-hal-generic-lr1121-td-esp32.h"
#endif

#ifdef RX_ELRS_GENERIC_C3_LR1121_ESP32C3
#include "rx-hal-generic-c3-lr1121-esp32c3.h"
#endif

//-- Generic 868/915 MHz ELRS devices with overlays or other variations

#ifdef RX_ELRS_RADIOMASTER_BR3_900_ESP8285
#include "rx-hal-radiomaster-br3-900-esp8285.h"
#endif

#ifdef RX_ELRS_IFLIGHT_ELRS_TD_900_ESP32
#include "rx-hal-generic-900-td-pa-esp32.h"
#endif


//-- ELRS 2.4 GHz Generic Devices

#ifdef RX_ELRS_GENERIC_2400_ESP8285
#include "rx-hal-generic-2400-esp8285.h"
#endif

#ifdef RX_ELRS_GENERIC_2400_PA_ESP8285
#include "rx-hal-generic-2400-pa-esp8285.h"
#endif

#ifdef RX_ELRS_GENERIC_2400_D_PA_ESP8285
#include "rx-hal-generic-2400-d-pa-esp8285.h"
#endif

#ifdef RX_ELRS_GENERIC_2400_TD_PA_ESP32
#include "rx-hal-generic-2400-td-pa-esp32.h"
#endif

//-- Generic 2.4 GHz ELRS boards with overlays or other variations

#ifdef RX_ELRS_IFLIGHT_ELRS_TD_2400_ESP32
#include "rx-hal-iflight-elrs-td-2400-esp32.h"
#endif

#ifdef RX_ELRS_IFLIGHT_ELRS_2400_ESP8285
#include "rx-hal-iflight-elrs-2400-esp8285.h"
#endif


//-- ELRS Selected Devices

#ifdef RX_ELRS_BAYCK_NANO_PRO_900_ESP8285
#include "rx-hal-generic-900-pa-esp8285.h"
#endif

#ifdef RX_ELRS_SPEEDYBEE_NANO_2400_ESP8285
#include "rx-hal-generic-2400-pa-esp8285.h"
#endif

#ifdef RX_ELRS_RADIOMASTER_RP4TD_2400_ESP32
#include "rx-hal-radiomaster-rp4td-2400-esp32.h"
#endif

#ifdef RX_ELRS_BETAFPV_SUPERD_2400_ESP32
#include "rx-hal-generic-2400-td-pa-esp32.h"
#endif

#ifdef RX_ELRS_RADIOMASTER_XR4_ESP32
#include "rx-hal-radiomaster-xr4-esp32.h"
#endif

#ifdef RX_ELRS_RADIOMASTER_XR1_ESP32C3
#include "rx-hal-radiomaster-xr1-esp32c3.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_RP4TD_2400_ESP32
#include "tx-hal-radiomaster-rp4td-2400-esp32.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_2400_ESP32
#include "tx-hal-radiomaster-int-2400-esp32.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_BOXER_2400_ESP32
#include "tx-hal-radiomaster-int-boxer-2400-esp32.h"
#endif

#ifdef TX_ELRS_JUMPER_INTERNAL_2400_ESP32
#include "tx-hal-jumper-int-2400-esp32.h"
#endif

#ifdef TX_ELRS_JUMPER_INTERNAL_900_ESP32
#include "tx-hal-jumper-int-900-esp32.h"
#endif

#ifdef TX_ELRS_BETAFPV_MICRO_1W_2400_ESP32
#include "tx-hal-betafpv-micro-1w-2400-esp32.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_BANDIT_MICRO_900_ESP32
#include "tx-hal-radiomaster-bandit-series-900-esp32.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_BANDIT_900_ESP32
#include "tx-hal-radiomaster-bandit-series-900-esp32.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_RANGER_2400_ESP32
#include "tx-hal-radiomaster-ranger-2400-esp32.h"
#endif

#ifdef TX_ELRS_RADIOMASTER_NOMAD_ESP32
#include "tx-hal-radiomaster-nomad-esp32.h"
#endif

