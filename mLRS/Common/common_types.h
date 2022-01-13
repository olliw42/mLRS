//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// COMMON TYPES
//*******************************************************
#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H
#pragma once


#include <inttypes.h>


#ifndef PACKED
#  define PACKED(__Declaration__)  __Declaration__ __attribute__((packed)) //that's for __GNUC__
#endif


typedef enum {
  CHANNEL_ORDER_AETR = 0, // most common
  CHANNEL_ORDER_TAER, // spektrum/graupner/jr
  CHANNEL_ORDER_ETAR,
} CHANNEL_ORDER_ENUM;


// the sx power is calculated as
// sx_power = LIMIT(SX1280_POWER_m18_DBM, power - POWER_GAIN_DBM + 18, POWER_SX1280_MAX_DBM)
//
// example 1: no PA
//  POWER_GAIN_DBM = 0
//  POWER_SX1280_MAX_DBM = SX1280_POWER_12p5_DBM
//  => power = -18 ... 13
//  => sx_power = 0 ... 31 = SX1280_POWER_m18_DBM ... SX1280_POWER_12p5_DBM
//
// example 2: E28 PA 27 dBm gain
//  POWER_GAIN_DBM = 27
//  POWER_SX1280_MAX_DBM = SX1280_POWER_0_DBM
//  => power = 9 ... 27
//  => sx_power = 0 ... 18 = SX1280_POWER_m18_DBM ... SX1280_POWER_0_DBM
//
// example 2: siyi PA 22 dBm gain
//  POWER_GAIN_DBM = 22
//  POWER_SX1280_MAX_DBM = SX1280_POWER_3_DBM
//  => power = 4 ... 25
//  => sx_power = 0 ... 21 = SX1280_POWER_m18_DBM ... SX1280_POWER_3_DBM

typedef enum {
  POWER_m18_DBM   = -18, // 16 uW
  POWER_m10_DBM   = -10, // 100 uW
  POWER_0_DBM     = 0, // 1 mW
  POWER_10_DBM    = 10, // 10 mW
  POWER_12_DBM    = 12, // 16 mW
  POWER_12p5_DBM  = 13, // 18 mW
  POWER_20_DBM    = 20, // 100 mW
  POWER_23_DBM    = 24, // 200 mW
  POWER_24_DBM    = 24, // 251 mW
  POWER_27_DBM    = 27, // 501 mW
  POWER_30_DBM    = 30, // 1000 mW
} POWER_ENUM;


typedef enum {
  ANTENNA_1 = 0x00,
  ANTENNA_2 = 0x01,
} ANTENNA_ENUM;


#endif // COMMON_TYPES_H
