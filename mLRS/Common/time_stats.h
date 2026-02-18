//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Time statistics, primarily for dev-ing
//*******************************************************
// Time statistics for up to PM_NUM_BLOCK pairs of measurement points.
// The routines for one block can be used either in ISR or normal context, but not in both.
//*******************************************************
#ifndef TIME_STATS_H
#define TIME_STATS_H
#pragma once


extern uint16_t micros16(void);


#ifdef DEBUG_ENABLED // only if DEBUG is enabled


#define TS_NUM_BLOCK  10

typedef struct {
    uint32_t tstart_us; // 0 means reporting is disabled
    uint32_t tlast_report_us;
    uint32_t count; // counts the number of time measurements during the reporting time
    uint32_t total;
    uint32_t min;
    uint32_t max;
    uint32_t overflow_cnt;
    uint16_t last_cnt;
} tTsStore;

static tTsStore ts_data[TS_NUM_BLOCK] = {};


uint32_t TS_MICROS32(uint8_t block)
{
#if defined ESP8266 || defined ESP32
    return micros(); // allows longer interval between calls
#else
    // separate overflow and last_cnt for each block
    // needs to be called not later than every 65 ms
    // avoids some race conditions and allows either ISR/normal use; but not both with the same block
    uint16_t cnt = micros16();
    if (cnt < ts_data[block].last_cnt) {
        ts_data[block].overflow_cnt += 0x10000;
    }
    ts_data[block].last_cnt = cnt;
    return ts_data[block].overflow_cnt + cnt;
#endif
}


void TS_START(uint8_t block)
{
    ts_data[block].tstart_us = TS_MICROS32(block);
}


void TS_END(uint8_t block, uint32_t report_period_ms = 10000, bool _continue = false)
{
    if (!_continue && ts_data[block].tstart_us == 0) return; // no end without start

    uint32_t tnow_us = TS_MICROS32(block);
    uint32_t telapsed_us = tnow_us - ts_data[block].tstart_us;

    if (ts_data[block].count == 0) {
        ts_data[block].total = ts_data[block].max = ts_data[block].min = telapsed_us;
    } else {
        ts_data[block].total += telapsed_us;
        if (telapsed_us < ts_data[block].min) ts_data[block].min = telapsed_us;
        if (telapsed_us > ts_data[block].max) ts_data[block].max = telapsed_us;
    }

    ts_data[block].count ++;

    if ((tnow_us - ts_data[block].tlast_report_us) >= report_period_ms * 1000) {
        dbg.puts("[");
        dbg.puts(u8toBCD_s(block)); dbg.puts("] ");
        dbg.puts(u32toBCD_s(ts_data[block].total));
        dbg.puts("/");
        dbg.puts(u32toBCD_s(ts_data[block].count));
        dbg.puts(" ");
        dbg.puts(u16toBCD_s(ts_data[block].min));
        dbg.puts(" ");
        dbg.puts(u16toBCD_s(ts_data[block].max));
        dbg.puts("\r\n");

        ts_data[block].count = 0;
        ts_data[block].tlast_report_us = tnow_us;
    }

    if (_continue) {
        ts_data[block].tstart_us = TS_MICROS32(block); // continuous accumulation
    } else {
        ts_data[block].tstart_us = 0; // no end without start
    }
}


#endif // DEBUG_ENABLED

#endif // TIME_STATS_H
