//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// rx hal splicer
//*******************************************************
#ifndef HAL_H
#define HAL_H
#pragma once

// enter define into "MCU G++ Compiler"->"Preprocessor" !!!


#include "device_conf.h"


#ifdef RX_SIYI_F373CC
#include "rx-hal-siyi-f373cc.h"
#endif

#ifdef TX_SIYI_F103C8
#include "tx-hal-siyi-f103c8.h"
#endif


#ifdef RX_DIY_BOARD01_F103CB
#include "rx-hal-diy-board01-f103cb.h"
#endif
#ifdef RX_DIY_E28_BOARD01_F103CB
#include "rx-hal-diy-e28-board01-f103cb.h"
#endif


#ifdef TX_DIY_E28_BOARD01_F103CB
#include "tx-hal-diy-e28-board01-f103cb.h"
#endif
#ifdef TX_DIY_MODULE01_G491RE
#include "tx-hal-diy-module01-g491re.h"
#endif


//-------------------------------------------------------
// Derived Defines
//-------------------------------------------------------
// should go somewhere else !?

#ifdef DEVICE_IS_TRANSMITTER
#ifdef DEVICE_HAS_MBRIDGE

  #if (SETUP_TX_SERIAL_DESTINATION == 1) || (SETUP_TX_CHANNELS_SOURCE == 1) // we use MBridge for serial
    #define USE_MBRIDGE
  #endif
  #if (SETUP_TX_SERIAL_DESTINATION != 1) && (SETUP_TX_CHANNELS_SOURCE == 3) // we use CRSF
    #define USE_CRSF
  #endif

  #if (SETUP_TX_SERIAL_DESTINATION == 1) && (SETUP_TX_CHANNELS_SOURCE == 3)
    #warning mBridge and CRSF cannot be used simukltaneoulsy, CRSF ignored!
  #endif

#endif
#endif



#endif // HAL_H
