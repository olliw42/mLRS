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
