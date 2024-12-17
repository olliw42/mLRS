//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// time_stats
//*******************************************************
#ifndef TIME_STATS_H
#define TIME_STATS_H
#pragma once

// Time Statistics for up to PM_NUM_BLOCK pairs of measurement points
//#define USE_TIME_STATS // Define this here or somewhere to use these functions
#if defined USE_DEBUG && defined USE_TIME_STATS
#define TS_NUM_BLOCK 10

struct TS_store {
    uint32_t start;
    uint32_t last_report;
    uint32_t total;
    uint32_t min;
    uint32_t max;
    uint32_t count;
    uint32_t overflow;
    uint16_t last_cnt;
};
static struct TS_store TS_data[TS_NUM_BLOCK];

static inline __attribute__((always_inline)) uint32_t TS_MICROS32(int block) {

    // Separate overflow and last_cnt for each block avoids some race conditions and
    // allows either ISR/normal use; but not both with the same block.
#if defined ESP8266 || defined ESP32
    uint16_t cnt = micros();
#else
    uint16_t cnt = MICROS_TIMx->CNT;
#endif
    if (cnt < TS_data[block].last_cnt) {
        TS_data[block].overflow += 0x10000;
    }
    TS_data[block].last_cnt = cnt;
    return TS_data[block].overflow + cnt;
}

inline __attribute__((always_inline)) void TS_START(int block) {
    TS_data[block].start = TS_MICROS32(block);
}

inline __attribute__((always_inline)) void TS_END(int block, uint32_t report_period = 10000000, bool cont = false) {
    if (!cont && TS_data[block].start == 0) return; // no end without start
    uint32_t time = TS_MICROS32(block);
    uint32_t elapsed = time - TS_data[block].start;
    if (TS_data[block].count == 0) {
        TS_data[block].total = TS_data[block].max = TS_data[block].min = elapsed;
    } else {
        TS_data[block].total += elapsed;
        if (elapsed < TS_data[block].min) TS_data[block].min = elapsed;
        if (elapsed > TS_data[block].max) TS_data[block].max = elapsed;
    }    
    TS_data[block].count ++;
    if (time - TS_data[block].last_report >= report_period) {
        dbg.puts("[");
        dbg.puts(u8toBCD_s(block)); dbg.puts("] ");
        dbg.puts(u32toBCD_s(TS_data[block].total));
        dbg.puts("/");
        dbg.puts(u32toBCD_s(TS_data[block].count));
        dbg.puts(" ");
        dbg.puts(u16toBCD_s(TS_data[block].min));
        dbg.puts(" ");
        dbg.puts(u16toBCD_s(TS_data[block].max));
        dbg.puts("\r\n");
        TS_data[block].count = 0;
        TS_data[block].last_report = time;
    }
    if (cont)
        TS_data[block].start = TS_MICROS32(block); // continuous accumulation
    else
        TS_data[block].start = 0; // no end without start
}
#endif

#endif // TIME_STATS_H
