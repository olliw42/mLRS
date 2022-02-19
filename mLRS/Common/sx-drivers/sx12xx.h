//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// SX12XX shim
//*******************************************************
#ifndef SX12XX_H
#define SX12XX_H
#pragma once


#include "..\hal\device_conf.h"

#ifdef DEVICE_HAS_SX126x
#include "..\..\modules\sx12xx-lib\src\sx126x.h"
#else
#include "..\..\modules\sx12xx-lib\src\sx128x.h"
#endif


#endif // SX12XX_H
