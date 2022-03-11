//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// COMMON TYPES
//*******************************************************

#include "common_types.h"


uint16_t clip_rc(int32_t x)
{
  if (x <= 1) return 1;
  if (x >= 2047) return 2047;
  return x;
}


uint8_t rssi_u7_from_i8(int8_t rssi_i8)
{
  if (rssi_i8 == RSSI_INVALID) return RSSI_U7_INVALID;

  // constrain to -1...-127
  if (rssi_i8 > RSSI_MAX) return RSSI_U7_MAX;
  if (rssi_i8 < RSSI_MIN) return RSSI_U7_MIN;

  return -rssi_i8;
}


int8_t rssi_i8_from_u7(uint8_t rssi_u7)
{
  if (rssi_u7 == RSSI_U7_INVALID) return RSSI_INVALID;

  return -rssi_u7;
}


// this would be the naive thing
// scale is "inverted" to make it that it is higher the better
//   rssi = (127 + stats.last_rx_rssi);
//   remrssi = (127 + stats.received_rssi);
//
// https://ardupilot.org/copter/docs/common-3dr-radio-advanced-configuration-and-technical-information.html#monitoring-the-link-quality
// for Sik radios holds signal_dBm approx rssi_SiK/1.9 - 127  => 150 approx -48dBm, 70 approx -90dBm
// so we could here a SiK scale
//   rssi_SiK = ( signal_dBm + 127 ) * 1.9
//
//   int32_t rssi_SiK = ( ((int32_t)stats.last_rx_rssi + 127) * 19000 ) / 10000;
//   if (rssi_SiK < 0) rssi_SiK = 0;
//   if (rssi_SiK > 250) rssi_SiK = 250;
//   rssi = rssi_SiK;
//
// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp#L483-L510
//   -120 ... -50 -> 0 .. 254
uint8_t rssi_i8_to_ap(int8_t rssi_i8)
{
  if (rssi_i8 == RSSI_INVALID) return UINT8_MAX;
  if (rssi_i8 > -50) return 254; // max value
  if (rssi_i8 < -120) return 0;

  int32_t r = (int32_t)rssi_i8 - (-120);
  constexpr int32_t m = (int32_t)(-50) - (-120);

  return (r * 254 + m/2) / m;
}


//   -120 ... -50 -> 172 .. 1877
uint16_t rssi_i8_to_ap_sbus(int8_t rssi_i8)
{
  if (rssi_i8 == RSSI_INVALID) return 0;
  if (rssi_i8 > -50) return 1877; // max value
  if (rssi_i8 < -120) return 172; // min value

  int32_t r = (int32_t)rssi_i8 - (-120);
  constexpr int32_t m = (int32_t)(-50) - (-120);

  return (r * 1705 + m/2) / m + 172;
}

