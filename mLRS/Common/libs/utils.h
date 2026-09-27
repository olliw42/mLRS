//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Some Utilities
//*******************************************************
#ifndef UTILS_H
#define UTILS_H
#pragma once


#include <inttypes.h>


//-- string functions

void strbufstrcpy(char* const res, const char* const src, uint16_t len);
void strstrbufcpy(char* const res, const char* const src, uint16_t len);
bool strbufeq(char* const s1, const char* const s2, uint16_t len);

void remove_leading_zeros(char* const s);


//-- time functions

typedef struct
{
    uint16_t year;
    uint8_t month; // 1 .. 12
    uint8_t day; // 1 .. 31
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} tDateTime;

void datetime_from_unix_time(tDateTime* const dt, uint32_t time_unix);


#endif // UTILS_H
