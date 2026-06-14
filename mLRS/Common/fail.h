//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// fail
//*******************************************************
#ifndef FAIL_H
#define FAIL_H
#pragma once


#include <stdint.h>
#include "common_types.h"
#include "hal/hal.h"


typedef enum {
    FAIL_LED_PATTERN_GR_OFF_RD_BLINK = 1,
    FAIL_LED_PATTERN_RD_OFF_GR_BLINK = 2,
    FAIL_LED_PATTERN_GR_ON_RD_BLINK = 3,
    FAIL_LED_PATTERN_RD_ON_GR_BLINK = 4,
    FAIL_LED_PATTERN_BLINK_COMMON = 5,
    FAIL_LED_PATTERN_BLINK_ALTERNATE = 6,

    FAIL_LED_PATTERN_RD_BLINK_GR_BLINK1 = 10,
    FAIL_LED_PATTERN_RD_BLINK_GR_BLINK2,
    FAIL_LED_PATTERN_RD_BLINK_GR_BLINK3,
    FAIL_LED_PATTERN_RD_BLINK_GR_BLINK4,
    FAIL_LED_PATTERN_RD_BLINK_GR_BLINK5,
} FAIL_LED_PATTERN_ENUM;


uint16_t fail_dbg_cnt;


void fail_do_dbg(tSerialBase* const dbg, const char* const msg)
{
    fail_dbg_cnt++;
    if (fail_dbg_cnt > 80) {
        fail_dbg_cnt = 0;
        dbg->puts("\n");
        dbg->puts(msg);
    }
}


void fail(tSerialBase* const dbg, uint8_t led_pattern, const char* const msg)
{
    dbg->puts("\n");
    dbg->puts(msg);

    fail_dbg_cnt = 0;

#ifdef DEVICE_HAS_NO_LED
    while (1) { delay_ms(50); fail_do_dbg(dbg, msg); }
#elif defined DEVICE_HAS_SINGLE_LED || defined DEVICE_HAS_SINGLE_LED_RGB
    while (1) { led_red_on(); delay_ms(25); led_red_off(); delay_ms(25); fail_do_dbg(dbg, msg); }
#else
    if (led_pattern == FAIL_LED_PATTERN_GR_OFF_RD_BLINK /*1*/) {
        led_green_off();
        while (1) { led_red_on(); delay_ms(25); led_red_off(); delay_ms(25); fail_do_dbg(dbg, msg); }
    }
    if (led_pattern == FAIL_LED_PATTERN_RD_OFF_GR_BLINK /*2*/) {
        led_red_off();
        while (1) { led_green_on(); delay_ms(25); led_green_off(); delay_ms(25); fail_do_dbg(dbg, msg); }
    }

    if (led_pattern == FAIL_LED_PATTERN_GR_ON_RD_BLINK /*3*/) {
        led_green_on();
        while (1) { led_red_on(); delay_ms(25); led_red_off(); delay_ms(25); fail_do_dbg(dbg, msg); }
    }
    if (led_pattern == FAIL_LED_PATTERN_RD_ON_GR_BLINK /*4*/) {
        led_red_on();
        while (1) { led_green_on(); delay_ms(25); led_green_off(); delay_ms(25); fail_do_dbg(dbg, msg); }
    }

    if (led_pattern == FAIL_LED_PATTERN_BLINK_COMMON /*5*/) {
        while (1) {
            led_red_on(); led_green_on(); delay_ms(25); led_red_off(); led_green_off(); delay_ms(25);
            fail_do_dbg(dbg, msg);
        }
    }
    if (led_pattern == FAIL_LED_PATTERN_BLINK_ALTERNATE /*6*/) {
        while (1) {
            led_red_on(); led_green_off(); delay_ms(25); led_red_off(); led_green_on(); delay_ms(25);
            fail_do_dbg(dbg, msg);
        }
    }

    if (led_pattern >= FAIL_LED_PATTERN_RD_BLINK_GR_BLINK1 && led_pattern <= FAIL_LED_PATTERN_RD_BLINK_GR_BLINK5) {
        uint8_t cnt = 0;
        uint8_t cnt_max = 1 + (led_pattern - FAIL_LED_PATTERN_RD_BLINK_GR_BLINK1);
        while (1) {
            if (cnt < cnt_max) led_green_on();
            for (uint8_t i = 0; i< 4; i++) { led_red_on(); delay_ms(25); led_red_off(); delay_ms(25); fail_do_dbg(dbg, msg); }
            led_green_off();
            for (uint8_t i = 0; i< 4; i++) { led_red_on(); delay_ms(25); led_red_off(); delay_ms(25); fail_do_dbg(dbg, msg); }
            cnt++;
            if (cnt >= cnt_max + 2) cnt = 0;
        }
    }

    led_red_on();
    led_green_on();

    while (1) { delay_ms(50); fail_do_dbg(dbg, msg); }
#endif
}


#endif // FAIL_H
