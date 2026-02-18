//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// LEDs
//*******************************************************
#ifndef LEDS_H
#define LEDS_H
#pragma once


#include "hal/hal.h"


#ifdef DEVICE_HAS_NO_LED

class tLEDs
{
  public:
    void Init(void) {}
    void Tick_ms(bool connected) {}
    void SetToBind(void) {}
    void SetToParamStore(void) {}
    void InitPassthrough(void) {}
    void TickPassthrough_ms(void) {}
};

#else


class tLEDs
{
  public:
    void Init(void)
    {
        blink = 0;
        is_in_bind = false;

        led_red_off();
        for (uint8_t i = 0; i < 7; i++) { led_red_toggle(); delay_ms(50); }
    }

    void Tick_ms(bool connected)
    {
#ifdef DEVICE_HAS_SINGLE_LED
        if (!is_in_bind) {
            DECc(blink, SYSTICK_DELAY_MS(500));
        } else {
            DECc(blink, SYSTICK_DELAY_MS(100));
        }

        if (connected && !is_in_bind) {
            led_red_on();
        } else if (!blink) {
            led_red_toggle();
        }
#elif defined DEVICE_HAS_SINGLE_LED_RGB 
        if (connected) {
            DECc(blink, SYSTICK_DELAY_MS(500));
        } else {
            DECc(blink, SYSTICK_DELAY_MS(200));
        }

        if (is_in_bind) {
            if (!blink) { led_blue_toggle(); }
        } else
        if (connected) {
            if (!blink) led_green_toggle();
        } else {
            if (!blink) led_red_toggle();
        }
#else
        if (connected) {
            DECc(blink, SYSTICK_DELAY_MS(500));
        } else {
            DECc(blink, SYSTICK_DELAY_MS(200));
        }

        if (is_in_bind) {
            if (!blink) { led_green_toggle(); led_red_toggle(); }
        } else
        if (connected) {
            if (!blink) led_green_toggle();
            led_red_off();
        } else {
            led_green_off();
            if (!blink) led_red_toggle();
        }
#endif
    }

    void SetToBind(void)
    {
#ifndef DEVICE_HAS_SINGLE_LED
        led_green_on();
#endif
        led_red_off();
        is_in_bind = true;
     }

    void SetToParamStore(void)
    {
        led_red_on();
#ifndef DEVICE_HAS_SINGLE_LED
        led_green_on();
#endif
     }

    void InitPassthrough(void)
    {
        led_red_off();
#ifndef DEVICE_HAS_SINGLE_LED
        led_green_on();
#endif
    }

    void TickPassthrough_ms(void)
    {
        DECc(blink, SYSTICK_DELAY_MS(100));

#ifdef DEVICE_HAS_SINGLE_LED
        if (!blink) { led_red_toggle(); }
#elif defined DEVICE_HAS_SINGLE_LED_RGB
        if (!blink) { led_purple_toggle(); }
#else
        if (!blink) { led_green_toggle(); led_red_toggle(); }
#endif
    }

  private:
    uint16_t blink;
    bool is_in_bind;
};


#endif // DEVICE_HAS_NO_LED

#endif // LEDS_H
