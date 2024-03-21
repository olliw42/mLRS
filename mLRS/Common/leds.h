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
    }

    void SetToBind(void)
    {
        led_green_on();
        led_red_off();
        is_in_bind = true;
     }

    void SetToParamStore(void)
    {
        led_red_on();
        led_green_on();
     }

    void InitPassthrough(void)
    {
        led_red_off();
        led_green_on();
    }

    void TickPassthrough_ms(void)
    {
        DECc(blink, SYSTICK_DELAY_MS(100));
        if (!blink) { led_green_toggle(); led_red_toggle(); }
    }

  private:
    uint16_t blink;
    bool is_in_bind;
};

#endif // LEDS_H
