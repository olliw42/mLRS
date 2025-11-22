//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP RGB LED
//********************************************************
#ifndef ESPLIB_RGB_LED_H
#define ESPLIB_RGB_LED_H

#include <NeoPixelBus.h>

// Constants for the state manager
#define LED_OFF 0
#define LED_RED 1
#define LED_GREEN 2
#define LED_BLUE 3

uint8_t ledCurrentColorState = LED_OFF;

NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0Ws2812xMethod> ledRGB(PIXEL_NUM, LED_RGB);

void leds_init(void) { ledRGB.Begin(); }

// Helper, ClearTo() overwrites the previous color, this function is mutually exclusive by design.
IRAM_ATTR void set_led_color_and_state(uint8_t targetState, RgbColor color) {
    if (ledCurrentColorState == targetState) return;
    
    ledRGB.ClearTo(color);
    ledRGB.Show();
    ledCurrentColorState = targetState;
}

// LED Functions
IRAM_ATTR void led_red_off(void) { set_led_color_and_state(LED_OFF, RgbColor(0)); } 
IRAM_ATTR void led_red_on(void) { set_led_color_and_state(LED_RED, RgbColor(255, 0, 0)); }
IRAM_ATTR void led_red_toggle(void) { (ledCurrentColorState == LED_RED) ? led_red_off() : led_red_on(); }

IRAM_ATTR void led_green_off(void) { set_led_color_and_state(LED_OFF, RgbColor(0)); }
IRAM_ATTR void led_green_on(void) { set_led_color_and_state(LED_GREEN, RgbColor(0, 255, 0)); }
IRAM_ATTR void led_green_toggle(void) { (ledCurrentColorState == LED_GREEN) ? led_green_off() : led_green_on(); }

IRAM_ATTR void led_blue_off(void) { set_led_color_and_state(LED_OFF, RgbColor(0)); }
IRAM_ATTR void led_blue_on(void) { set_led_color_and_state(LED_BLUE, RgbColor(0, 0, 255)); }
IRAM_ATTR void led_blue_toggle(void) { (ledCurrentColorState == LED_BLUE) ? led_blue_off() : led_blue_on(); }

#endif // ESPLIB_RGB_LED_H
