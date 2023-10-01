//*******************************************************
// This holds code taken from other open source projects.
// For the copyrights and licenses, please look up in the
// original project repositories. The links are given.
//********************************************************
#ifndef THIRDARTY_H
#define THIRDARTY_H
#pragma once


#include <stdint.h>


//-------------------------------------------------------
// ArduPilot PassThrough, Auxiliary Helper
//-------------------------------------------------------
// This code is taken from ArduPilot
// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Frsky_Telem/AP_Frsky_SPort.cpp#L378-L447
// it is different than what is used by MavToPT, for the first option, digits=2, power=0
// https://github.com/zs6buj/MavlinkToPassthru/blob/master/Source/Stable/MavToPass/Utilities.ino#L1321-L1396

uint32_t prep_number(int32_t number, uint8_t digits, uint8_t power);


#endif // THIRDARTY_H
