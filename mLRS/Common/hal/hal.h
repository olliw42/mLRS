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
#ifdef RX_DIY_E28DUAL_BOARD02_F103CB
#include "rx-hal-diy-e28dual-board02-f103cb.h"
#endif


#ifdef TX_DIY_E28DUAL_BOARD02_F103CB
#include "tx-hal-diy-e28dual-board02-f103cb.h"
#endif
#ifdef TX_DIY_BOARD01_G491RE
#include "tx-hal-diy-board01-g491re.h"
#endif
#ifdef TX_DIY_SXDUAL_BOARD02_G491RE
#include "tx-hal-diy-sxdual-board02-g491re.h"
#endif
#ifdef TX_DIY_E28DUAL_MODULE02_G491RE
#include "tx-hal-diy-e28dual-module02-g491re.h"
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
  #ifdef DEBUG_ENABLED
    #define USE_DEBUG
  #endif
#endif
#endif

#ifdef DEVICE_IS_TRANSMITTER
#if defined DEVICE_HAS_COM_OR_DEBUG
  #define USE_SERIAL
  #if !defined DEBUG_ENABLED
    #define USE_COM
  #else
    #define USE_DEBUG
  #endif
#elif defined DEVICE_HAS_SERIAL_OR_COM // R9M, Siyi, has device dependent ways to select serial or com
  #define USE_SERIAL
  #define USE_COM_ON_SERIAL
  #ifdef DEBUG_ENABLED
    #define USE_DEBUG
  #endif
#else
  #define USE_SERIAL
  #define USE_COM
  #ifdef DEBUG_ENABLED
    #define USE_DEBUG
  #endif
#endif

#ifdef DEVICE_HAS_BT
  #define USE_SERIAL2
#endif
#endif


#if (defined DEVICE_HAS_IN) || (defined DEVICE_HAS_IN_NORMAL) || (defined DEVICE_HAS_IN_INVERTED)
  #define USE_IN
#endif


#if (defined DEVICE_HAS_OUT) || (defined DEVICE_HAS_OUT_NORMAL) || (defined DEVICE_HAS_OUT_INVERTED)
  #define USE_OUT
#endif


#if (defined DEVICE_HAS_I2C_DAC) || (defined DEVICE_HAS_I2C_DISPLAY)
  #define USE_I2C
#endif


#ifdef DEVICE_HAS_I2C_DISPLAY
  #define USE_DISPLAY
#endif


#ifdef DEVICE_HAS_SX126x
  #define SX_DRIVER Sx126xDriver
#elif defined DEVICE_HAS_SX127x
  #define SX_DRIVER Sx127xDriver
#else
  #define SX_DRIVER Sx128xDriver
#endif

#ifdef DEVICE_HAS_DIVERSITY
  #ifdef DEVICE_HAS_SX126x
    #define SX2_DRIVER Sx126xDriver2
  #elif defined DEVICE_HAS_SX127x
    #define SX2_DRIVER Sx127xDriver2
  #else
    #define SX2_DRIVER Sx128xDriver2
  #endif
#else
  #define SX2_DRIVER SxDriverDummy
#endif


#ifdef DEVICE_HAS_DIVERSITY
  #define IF_ANTENNA1(x)            if (Config.UseAntenna1) { x; }
  #define IF_ANTENNA2(x)            if (Config.UseAntenna2) { x; }
  #define USE_ANTENNA1              (Config.UseAntenna1)
  #define USE_ANTENNA2              (Config.UseAntenna2)
#else
  #define IF_ANTENNA1(x)            x;
  #define IF_ANTENNA2(x)
  #define USE_ANTENNA1              true
  #define USE_ANTENNA2              false
#endif

#ifdef DEVICE_HAS_JRPIN5
  #define IF_MBRIDGE(x)             if (Config.UseMbridge) { x; }
  #define IF_CRSF(x)                if (Config.UseCrsf) { x; }
#else
  #define IF_MBRIDGE(x)
  #define IF_CRSF(x)
#endif


//-- checks

#if !defined DEVICE_IS_TRANSMITTER && !defined DEVICE_IS_RECEIVER
  #error Must be either transmitter or receiver !
#endif

#if !defined DEVICE_HAS_SX128x && !defined DEVICE_HAS_SX127x && !defined DEVICE_HAS_SX126x
  #error Must be either SX128x or SX127x or SX126x !
#endif

#if !defined FREQUENCY_BAND_2P4_GHZ && \
    !defined FREQUENCY_BAND_915_MHZ_FCC && !defined FREQUENCY_BAND_868_MHZ && !defined FREQUENCY_BAND_868_915_MHZ && \
    !defined FREQUENCY_BAND_433_MHZ
  #error Correct frequency band must be defined !
#endif


#endif // HAL_H
