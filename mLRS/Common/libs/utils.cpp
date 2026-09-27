//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Some Utilities
//*******************************************************

#include <string.h>
#include "utils.h"


//-- string functions

// copy a string into a buffer with max len chars
void strbufstrcpy(char* const res, const char* const src, uint16_t len)
{
    memset(res, '\0', len);
    for (uint16_t i = 0; i < len; i++) {
        if (src[i] == '\0') return;
        res[i] = src[i];
    }
}


// copy a buffer into a string with max len chars (i.e. len + 1 size)
void strstrbufcpy(char* const res, const char* const src, uint16_t len)
{
    memset(res, '\0', len + 1); // this ensures that res is terminated with a '\0'
    for (uint16_t i = 0; i < len; i++) {
        if (src[i] == '\0') return;
        res[i] = src[i];
    }
}


bool strbufeq(char* const s1, const char* const s2, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (s1[i] == '\0' && s2[i] == '\0') return true;
        if (s1[i] == '\0') return false;
        if (s2[i] == '\0') return false;
        if (s1[i] != s2[i]) return false;
    }
    return true;
}


void remove_leading_zeros(char* const s)
{
int16_t i, len; // int16 to avoid underflow in len -1

    len = strlen(s);
    for (i = 0; i < len - 1; i++) {
        if (s[i] != '0') break;
    }
    memmove(&s[0], &s[i], len - i + 1);
}


// replacement for atoi(), which via strtol() pulls in _impure_data & stdio buffers (~390 bytes RAM)
int32_t atoi32(const char* s)
{
    while (*s == ' ' || *s == '\t') s++;
    bool neg = (*s == '-');
    if (*s == '-' || *s == '+') s++;
    int32_t v = 0;
    while (*s >= '0' && *s <= '9') v = 10 * v + (*s++ - '0');
    return (neg) ? -v : v;
}


//-- time functions

// convert unix time (seconds since 1970, UTC) to date & time
// avoids gmtime() which pulls in malloc & stdio, valid 2000-03-01 .. 2100-02-28
// written with the help of Claude
void datetime_from_unix_time(tDateTime* const dt, uint32_t time_unix)
{
    uint32_t secs = time_unix % 86400;

    uint32_t days = time_unix / 86400 - 11017; // shift epoch to 2000-03-01
    uint32_t yoe = (4 * days + 3) / 1461;
    uint32_t doy = days - 1461 * yoe / 4;
    uint32_t mp = (5 * doy + 2) / 153;
    uint32_t month = (mp < 10) ? mp + 3 : mp - 9;

    dt->year = 2000 + yoe + (month <= 2);
    dt->month = month;
    dt->day = doy - (153 * mp + 2) / 5 + 1;
    dt->hour = secs / 3600;
    dt->minute = (secs / 60) % 60;
    dt->second = secs % 60;
}
