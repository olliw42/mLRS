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


typedef enum {
    FAIL_LED_PATTERN_GR_OFF_RD_BLINK = 1,
    FAIL_LED_PATTERN_RD_OFF_GR_BLINK = 2,
    FAIL_LED_PATTERN_GR_ON_RD_BLINK = 3,
    FAIL_LED_PATTERN_RD_ON_GR_BLINK = 4,
    FAIL_LED_PATTERN_BLINK_COMMON = 5,
    FAIL_LED_PATTERN_BLINK_ALTERNATE = 6,
} FAIL_LED_PATTERN_ENUM;



uint16_t fail_dbg_cnt;


void fail_do_dbg(tSerialBase* dbg, const char* msg)
{
    fail_dbg_cnt++;
    if (fail_dbg_cnt > 80) {
        fail_dbg_cnt = 0;
        dbg->puts("\n");
        dbg->puts(msg);
    }
}


void fail(tSerialBase* dbg, uint8_t led_pattern, const char* msg)
{
    dbg->puts("\n");
    dbg->puts(msg);

    fail_dbg_cnt = 0;;

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

    led_red_on();
    led_green_on();
    while (1) { delay_ms(50); fail_do_dbg(dbg, msg); }
}


#endif // FAIL_H
