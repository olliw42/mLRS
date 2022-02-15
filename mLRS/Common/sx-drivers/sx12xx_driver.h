//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// SX12XX driver shim
//*******************************************************
#ifndef SX12XX_DRIVER_H
#define SX12XX_DRIVER_H
#pragma once



#include "..\hal\device_conf.h"

#ifdef DEVICE_HAS_SX126x
#include "sx126x_driver.h"
#elif defined DEVICE_HAS_SX127x
#include "sx127x_driver.h"
#else
#include "sx1280_driver.h"
#endif


#endif // SX12XX_DRIVER_H
