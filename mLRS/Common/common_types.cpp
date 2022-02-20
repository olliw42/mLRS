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
  if (x <= 0) return 0;
  if (x >= 2047) return 2047;
  return x;
}


