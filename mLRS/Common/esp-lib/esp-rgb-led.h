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

bool ledRedState;
bool ledGreenState;
bool ledBlueState;

NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0Ws2812xMethod> ledRGB(PIXEL_NUM, LED_RGB);

void leds_init(void)
{
    ledRGB.Begin();
    ledRGB.Show();
}

IRAM_ATTR void set_led_color(RgbColor color)
{
    for (uint8_t i = 0; i < PIXEL_NUM; i++)
    {
        ledRGB.SetPixelColor(i, color);
    }
    ledRGB.Show();
}

IRAM_ATTR void led_red_off(void)
{
    if (!ledRedState) return;
    set_led_color(RgbColor(0, 0, 0));
    ledRedState = 0;
}

IRAM_ATTR void led_red_on(void)
{
    if (ledRedState) return;
    set_led_color(RgbColor(255, 0, 0));
    ledRedState = 1;
}

IRAM_ATTR void led_red_toggle(void)
{
    if (ledRedState) { led_red_off(); } else { led_red_on(); }
}

IRAM_ATTR void led_green_off(void)
{
    if (!ledGreenState) return;
    set_led_color(RgbColor(0, 0, 0));
    ledGreenState = 0;
}

IRAM_ATTR void led_green_on(void)
{
    if (ledGreenState) return;
    set_led_color(RgbColor(0, 255, 0));
    ledGreenState = 1;
}

IRAM_ATTR void led_green_toggle(void)
{
    if (ledGreenState) { led_green_off(); } else { led_green_on(); }
}

IRAM_ATTR void led_blue_off(void)
{
    if (!ledBlueState) return;
    set_led_color(RgbColor(0, 0, 0));
    ledBlueState = 0;
}

IRAM_ATTR void led_blue_on(void)
{
    if (ledBlueState) return;
    set_led_color(RgbColor(0, 0, 255));
    ledBlueState = 1;
}

IRAM_ATTR void led_blue_toggle(void)
{
    if (ledBlueState) { led_blue_off(); } else { led_blue_on(); }
}

#endif // ESPLIB_RGB_LED_H
