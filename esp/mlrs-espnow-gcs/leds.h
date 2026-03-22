//*******************************************************
// mLRS ESP-NOW GCS Bridge
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// LEDs
//*******************************************************
// 2. Mar. 2026
//*******************************************************
#ifndef LEDS_H
#define LEDS_H


//-------------------------------------------------------
// blink state
//-------------------------------------------------------

bool _led_state;
unsigned long _led_tlast_ms;

// returns true when it's time to toggle
bool _led_blink(unsigned long interval_ms)
{
    unsigned long tnow = millis();
    if (tnow - _led_tlast_ms > interval_ms) {
        _led_tlast_ms = tnow;
        _led_state = !_led_state;
        return true;
    }
    return false;
}


//-------------------------------------------------------
// single LED
//-------------------------------------------------------

#if defined DEVICE_HAS_SINGLE_LED

#ifdef LED_ACTIVE_LOW
#define LED_STATE_ON  LOW
#define LED_STATE_OFF HIGH
#else
#define LED_STATE_ON  HIGH
#define LED_STATE_OFF LOW
#endif

void led_init(void)
{
    pinMode(LED_IO, OUTPUT);
    digitalWrite(LED_IO, LED_STATE_OFF);
    _led_state = false;
    _led_tlast_ms = 0;
}

void led_on(void) { digitalWrite(LED_IO, LED_STATE_ON); }
void led_off(void) { digitalWrite(LED_IO, LED_STATE_OFF); }

void _led_single_blink(unsigned long interval_ms)
{
    if (_led_blink(interval_ms)) {
        if (_led_state) led_on(); else led_off();
    }
}

void led_tick_connected(void) { led_on(); }
void led_tick_disconnected(void) { _led_single_blink(500); }
void led_tick_scanning(void) { _led_single_blink(100); }


//-------------------------------------------------------
// single RGB LED (NeoPixel/WS2812)
//-------------------------------------------------------

#elif defined DEVICE_HAS_SINGLE_LED_RGB

#include <Adafruit_NeoPixel.h>

#ifndef RGB_LED_COUNT
#define RGB_LED_COUNT 1
#endif

Adafruit_NeoPixel _rgb_strip(RGB_LED_COUNT, LED_IO, NEO_GRB + NEO_KHZ800);

void led_init(void)
{
    _rgb_strip.begin();
    _rgb_strip.clear();
    _rgb_strip.show();
    _led_state = false;
    _led_tlast_ms = 0;
}

void led_on(void)
{
    _rgb_strip.setPixelColor(0, _rgb_strip.Color(0, 255, 0)); // green
    _rgb_strip.show();
}

void led_off(void)
{
    _rgb_strip.clear();
    _rgb_strip.show();
}

void _led_rgb_blink(unsigned long interval_ms, uint8_t r, uint8_t g, uint8_t b)
{
    if (_led_blink(interval_ms)) {
        if (_led_state) {
            _rgb_strip.setPixelColor(0, _rgb_strip.Color(r, g, b));
        } else {
            _rgb_strip.clear();
        }
        _rgb_strip.show();
    }
}

// connected: green toggling at 500 ms
void led_tick_connected(void) { _led_rgb_blink(500, 0, 255, 0); }
// disconnected: red toggling at 200 ms
void led_tick_disconnected(void) { _led_rgb_blink(200, 255, 0, 0); }
// scanning: red toggling at 100 ms
void led_tick_scanning(void) { _led_rgb_blink(100, 255, 0, 0); }


//-------------------------------------------------------
// no LED
//-------------------------------------------------------

#else

void led_init(void) {}
void led_on(void) {}
void led_off(void) {}
void led_tick_connected(void) {}
void led_tick_disconnected(void) {}
void led_tick_scanning(void) {}

#endif


#endif // LEDS_H
