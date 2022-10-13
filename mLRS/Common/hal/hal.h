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

/* Documentation

The availability of the many features are handled via #define declarations. In order to keep somewhat track,
this naming convention is used (with few exceptions):
- DEVICE_HAS_XXXX: is set in a hal file to indicate the availability/non-availability of a feature
- USE_XXXX: these are determined through some processing, which can involve the DEVICE_HAS_XXXX flags
In follow up code therefore the USE_XXXX flags should be used (if available) to enable/disable code for a feature.
If a USE_XXXX flag is not available (example: DEVICE_HAS_DIVERSITY) then of course the respective DEVICE_HAS_XXXX
flag needs to be used. Also, DEVICE_HAS_XXXX flags may have to be used to distinguish the "flavor" of the feature
(example: IN feature with normal or inverted UART levels).

Many feature flags are available, which can be set in the hal files. They are listed in the following for the
tx-hal and rx-hal files.

In tx-hal files:

#define DEVICE_HAS_DIVERSITY        // board supports diversity
#define DEVICE_HAS_JRPIN5           // board has a pin for JR bay Pin5/SPort
#define DEVICE_HAS_IN               // board has an IN port, which supports both normal and inverted UART signals
#define DEVICE_HAS_IN_NORMAL        // board has an IN port, which supports only normal UART signals
#define DEVICE_HAS_IN_INVERTED      // board has an IN port, which supports only inverted UART signals
#define DEVICE_HAS_SERIAL_OR_DEBUG  // board has UART which is shared between SerialL or Debug, selected by DEBUG_ENABLED flag
#define DEVICE_HAS_COM_OR_DEBUG     // board has UART which is shared between Com or Debug, selected by DEBUG_ENABLED flag
#define DEVICE_HAS_SERIAL_OR_COM    // board has UART which is shared between Serial or Com, selected by e.g. a switch
#define DEVICE_HAS_NO_SERIAL        // board has no Serial port
#define DEVICE_HAS_NO_COM           // board has no Com port
#define DEVICE_HAS_DEBUG_SWUART     // implement Debug as software UART
#define DEVICE_HAS_I2C_DISPLAY      // board has DISPLAY on I2C
#define DEVICE_HAS_BT               // board has a Serial2 port
#define DEVICE_HAS_BUZZER           // board has a Buzzer
#define DEVICE_HAS_I2C_DAC          // board has a DAC for power control on I2C

In rx-hal files:

#define DEVICE_HAS_DIVERSITY        // board supports diversity
#define DEVICE_HAS_OUT              // board has an OUT port, which supports both normal and inverted UART signals
#define DEVICE_HAS_OUT_NORMAL       // board has an OUT port, which supports only normal UART signals
#define DEVICE_HAS_OUT_INVERTED     // board has an OUT port, which supports only inverted UART signals
#define DEVICE_HAS_SERIAL_OR_DEBUG  // is selected by DEBUG_ENABLED define
#define DEVICE_HAS_DEBUG_SWUART     // implement Debug as software UART
#define DEVICE_HAS_BUZZER           // board has a Buzzer

Note: Some "high-level" features are set for each device in the device_conf.h file, and not in the device's hal file.
*/


#include "device_conf.h"


//-- 2.4 GHz Devices

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
#ifdef RX_DIY_E28_G441KB
#include "rx-hal-diy-e28-g441kb.h"
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


//-- 868/915 MHz Devices

#ifdef RX_R9MX_868_L433CB
#include "rx-hal-R9MX-868-l433cb.h"
#endif
#ifdef RX_R9M_868_F103C8
#include "rx-hal-R9M-868-f103c8.h"
#endif
#ifdef RX_R9MM_868_F103RB
#include "rx-hal-R9MM-868-f103rb.h"
#endif

#ifdef TX_R9M_868_F103C8
#include "tx-hal-R9M-868-f103c8.h"
#endif


#ifdef RX_DIY_E22_G441KB
#include "rx-hal-diy-e22-g441kb.h"
#endif

#ifdef TX_DIY_E22DUAL_MODULE02_G491RE
#include "tx-hal-diy-e22dual-module02-g491re.h"
#endif


#ifdef RX_WIO_E5_GROVE_WLE5JC
#include "rx-hal-wio-e5-grove-wle5jc.h"
#endif

#ifdef TX_WIO_E5_MINI_WLE5JC
#include "tx-hal-wio-e5-mini-wle5jc.h"
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
  #if !defined DEVICE_HAS_NO_SERIAL
    #define USE_SERIAL
  #endif
  #if !defined DEVICE_HAS_NO_COM
    #define USE_COM
  #endif
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
  #ifndef HAL_I2C_MODULE_ENABLED
    #error HAL_I2C_MODULE_ENABLED is not defined, but I2C is used!
  #endif
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
  #define IF_MBRIDGE_OR_CRSF(x)     if (Config.UseMbridge | Config.UseCrsf) { x; }
#else
  #define IF_MBRIDGE(x)
  #define IF_CRSF(x)
  #define IF_MBRIDGE_OR_CRSF(x)
#endif


//-- checks

#if !defined DEVICE_IS_TRANSMITTER && !defined DEVICE_IS_RECEIVER
  #error Must be either transmitter or receiver !
#endif

#if !defined DEVICE_HAS_SX128x && !defined DEVICE_HAS_SX127x && !defined DEVICE_HAS_SX126x
  #error Must be either SX128x or SX127x or SX126x !
#endif

#if !defined FREQUENCY_BAND_2P4_GHZ && \
    !defined FREQUENCY_BAND_915_MHZ_FCC && !defined FREQUENCY_BAND_868_MHZ && \
    !defined FREQUENCY_BAND_433_MHZ
  #error Correct frequency band must be defined !
#endif


#endif // HAL_H
