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
#ifdef TX_DIY_BOARD01_G491RE
#include "tx-hal-diy-board01-g491re.h"
#endif


#ifdef RX_R9MX_868_L433CB
#include "rx-hal-R9MX-868-l433cb.h"
#endif

#ifdef RX_R9MM_868_F103RB
#include "rx-hal-R9MM-868-f103rb.h"
#endif

#ifdef TX_R9M_868_F103C8
#include "tx-hal-R9M-868-f103c8.h"
#endif


//-------------------------------------------------------
// Derived Defines
//-------------------------------------------------------
// should go somewhere else !?

#define RFPOWER_LIST_NUM  sizeof(rfpower_list)/sizeof(rfpower_t)


#ifdef DEVICE_IS_RECEIVER
#if defined DEVICE_HAS_SERIAL_OR_DEBUG
  #if !defined DEBUG_ENABLED
    #define USE_SERIAL
  #else
    #define USE_DEBUG
  #endif
#else
  #define USE_SERIAL
  #define USE_DEBUG
#endif
#endif

#ifdef DEVICE_IS_TRANSMITTER
  #define USE_SERIAL
  //#define USE_COM // we do not have such a case yet, so this we will have to work out
  #define USE_DEBUG
#endif


#endif // HAL_H
