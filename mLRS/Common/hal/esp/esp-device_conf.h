//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// device configuration splicer for ESP targets
//*******************************************************

//-------------------------------------------------------
// ESP Boards
//-------------------------------------------------------

//-- ELRS 868/915 MHz Generic Devices

#ifdef RX_ELRS_GENERIC_900_ESP8285
  #define DEVICE_NAME "GENERIC 900"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX127x
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef RX_ELRS_GENERIC_900_PA_ESP8285
  #define DEVICE_NAME "GENERIC 900 PA"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX127x
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef RX_ELRS_GENERIC_900_TD_PA_ESP32
  #define DEVICE_NAME "GENERIC 900 TD PA"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX127x
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef RX_ELRS_GENERIC_LR1121_TD_ESP32
  #define DEVICE_NAME "GENERIC LR1121 TD"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_LR11xx
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef RX_ELRS_GENERIC_C3_LR1121_ESP32C3
  #define DEVICE_NAME "GENERIC C3 LR1121"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_LR11xx
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

//-- Generic 868/915 MHz ELRS devices with overlays or other variations

#ifdef RX_ELRS_RADIOMASTER_BR3_900_ESP8285
  #define DEVICE_NAME "RM BR3 900"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX127x
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef RX_ELRS_IFLIGHT_ELRS_TD_900_ESP32
  #define DEVICE_NAME "IFLIGHT 900 TD"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX127x
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif


//-- ELRS 2.4 GHz Generic Devices

#ifdef RX_ELRS_GENERIC_2400_ESP8285
  #define DEVICE_NAME "GENERIC 2400"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef RX_ELRS_GENERIC_2400_PA_ESP8285
  #define DEVICE_NAME "GENERIC 2400 PA"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef RX_ELRS_GENERIC_2400_D_PA_ESP8285
  #define DEVICE_NAME "GENERIC 2400 D PA"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef RX_ELRS_GENERIC_2400_TD_PA_ESP32
  #define DEVICE_NAME "GENERIC 2400 TD PA"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

//-- Generic 2.4 GHz ELRS boards with overlays or other variations

#ifdef RX_ELRS_IFLIGHT_ELRS_TD_2400_ESP32
  #define DEVICE_NAME "IFLIGHT 2400 TD"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef RX_ELRS_IFLIGHT_ELRS_2400_ESP8285
  #define DEVICE_NAME "IFLIGHT 2400"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif


//-- ELRS Selected Receivers

#ifdef RX_ELRS_BAYCK_NANO_PRO_900_ESP8285
  #define DEVICE_NAME "BAYCK NANO PRO 900"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX127x
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef RX_ELRS_SPEEDYBEE_NANO_2400_ESP8285
  #define DEVICE_NAME "SPEEDYBEE NANO 2.4G"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef RX_ELRS_BETAFPV_SUPERD_2400_ESP32
  #define DEVICE_NAME "BetaFPV SuperD 2.4G"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef RX_ELRS_RADIOMASTER_RP4TD_2400_ESP32
  #define DEVICE_NAME "RM RP4TD 2.4G"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef RX_ELRS_RADIOMASTER_XR4_900_ESP32
  #define DEVICE_NAME "RM XR4 900"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_LR11xx
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef RX_ELRS_RADIOMASTER_XR1_900_ESP32C3
  #define DEVICE_NAME "RM XR1 900"
  #define DEVICE_IS_RECEIVER
  #define DEVICE_HAS_LR11xx
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif


//-- ELRS Tx Modules (external)

#ifdef TX_ELRS_RADIOMASTER_RP4TD_2400_ESP32 // receiver used as Tx module, SiK only
  #define DEVICE_NAME "RM RP4TD 2.4G"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_ELRS_BETAFPV_MICRO_1W_2400_ESP32
  #define DEVICE_NAME "BetaFPV Micro1W 2.4G"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_ELRS_RADIOMASTER_BANDIT_MICRO_900_ESP32
  #define DEVICE_NAME "RM Bandit Micro 900"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX127x
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef TX_ELRS_RADIOMASTER_BANDIT_900_ESP32
  #define DEVICE_NAME "RM Bandit 900"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX127x
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef TX_ELRS_RADIOMASTER_RANGER_2400_ESP32
  #define DEVICE_NAME "RM Ranger 2.4G"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_ELRS_RADIOMASTER_NOMAD_ESP32
  #define DEVICE_NAME "RM Nomad"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_LR11xx
  #define FREQUENCY_BAND_2P4_GHZ
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif


//-- ELRS Internal Tx Modules

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_2400_ESP32
  #define DEVICE_NAME "RM Int 2.4G"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_BOXER_2400_ESP32
  #define DEVICE_NAME "RM Int Boxer 2.4G"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_ELRS_JUMPER_INTERNAL_2400_ESP32
  #define DEVICE_NAME "Jumper Int 2.4G"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX128x
  #define FREQUENCY_BAND_2P4_GHZ
#endif

#ifdef TX_ELRS_JUMPER_INTERNAL_900_ESP32
  #define DEVICE_NAME "Jumper Int 900"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_SX127x
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

#ifdef TX_ELRS_RADIOMASTER_INTERNAL_TX15_ESP32
  #define DEVICE_NAME "RM TX15"
  #define DEVICE_IS_TRANSMITTER
  #define DEVICE_HAS_LR11xx
  #define FREQUENCY_BAND_2P4_GHZ
  #define FREQUENCY_BAND_868_MHZ
  #define FREQUENCY_BAND_915_MHZ_FCC
#endif

