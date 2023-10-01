//*******************************************************
// This holds code taken from other open source projects.
// For the copyrights and licenses, please look up in the
// original project repositories. The links are given.
//********************************************************


#include "thirdparty.h"


//-------------------------------------------------------
// ArduPilot PassThrough, Auxiliary Helper
//-------------------------------------------------------
// This code is taken from ArduPilot
// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Frsky_Telem/AP_Frsky_SPort.cpp#L378-L447
// it is different than what is used by MavToPT, for the first option, digits=2, power=0
// https://github.com/zs6buj/MavlinkToPassthru/blob/master/Source/Stable/MavToPass/Utilities.ino#L1321-L1396

#include "math.h"


uint32_t constrain_int16(int32_t value, int32_t min, int32_t max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}


uint32_t prep_number(int32_t number, uint8_t digits, uint8_t power)
{
    uint32_t res = 0;
    uint32_t abs_number = (number >= 0) ? number : -number;

    if ((digits == 2) && (power == 0)) {
        // number encoded on 7 bits, client side needs to know if expected range is 0,127 or -63,63
        uint8_t max_value = (number < 0) ? (1 << 6) - 1 : (1 << 7) - 1;
        res = constrain_int16(abs_number, 0, max_value);
        if (number < 0) { // if number is negative, add sign bit in front
            res |= (1 << 6);
        }
    } else
    if ((digits == 2) && (power == 1)) {
        // number encoded on 8 bits: 7 bits for digits + 1 for 10^power
        if (abs_number < 100) {
            res = abs_number << 1;
        } else if (abs_number < 1270) {
            res = ((uint32_t)roundf(abs_number * 0.1f) << 1) | 0x1;
        } else { // transmit max possible value (0x7F x 10^1 = 1270)
            res = 0xFF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= (1 << 8);
        }
    } else
    if ((digits == 2) && (power == 2)) {
        // number encoded on 9 bits: 7 bits for digits + 2 for 10^power
        if (abs_number < 100) {
            res = abs_number << 2;
        } else if (abs_number < 1000) {
            res = ((uint32_t)roundf(abs_number * 0.1f) << 2) | 0x1;
        } else if (abs_number < 10000) {
            res = ((uint32_t)roundf(abs_number * 0.01f) << 2) | 0x2;
        } else if (abs_number < 127000) {
            res = ((uint32_t)roundf(abs_number * 0.001f) << 2) | 0x3;
        } else { // transmit max possible value (0x7F x 10^3 = 127000)
            res = 0x1FF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= (1 << 9);
        }
    } else
    if ((digits == 3) && (power == 1)) {
        // number encoded on 11 bits: 10 bits for digits + 1 for 10^power
        if (abs_number < 1000) {
            res = abs_number << 1;
        } else if (abs_number < 10240) {
            res = ((uint32_t)roundf(abs_number * 0.1f) << 1) | 0x1;
        } else { // transmit max possible value (0x3FF x 10^1 = 10230)
            res = 0x7FF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= (1 << 11);
        }
    } else
    if ((digits == 3) && (power == 2)) {
        // number encoded on 12 bits: 10 bits for digits + 2 for 10^power
        if (abs_number < 1000) {
            res = abs_number << 2;
        } else if (abs_number < 10000) {
            res = ((uint32_t)roundf(abs_number * 0.1f) << 2) | 0x1;
        } else if (abs_number < 100000) {
            res = ((uint32_t)roundf(abs_number * 0.01f) << 2) | 0x2;
        } else if (abs_number < 1024000) {
            res = ((uint32_t)roundf(abs_number * 0.001f) << 2) | 0x3;
        } else { // transmit max possible value (0x3FF x 10^3 = 1023000)
            res = 0xFFF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= (1 << 12);
        }
    }

    return res;
}

