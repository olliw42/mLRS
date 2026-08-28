//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP TRNG
//********************************************************
#ifndef ESPLIB_TRNG_H
#define ESPLIB_MCU_H


#ifdef ESP8266
// ESP8266 transmitter targets currently do not exist; only transmitters draw random numbers, so no problem

void trng_init(void) {}
uint32_t trng_get32(void) { return UINT32_MAX; } // indicates TRNG not available

#else

#include "esp_random.h"
#include "bootloader_random.h"


#define ESP_TRNG_POOL_LEN  8 // 2 are needed for the session key, rest is head room

uint32_t esp_trng_pool[ESP_TRNG_POOL_LEN];

uint8_t esp_trng_pool_idx = ESP_TRNG_POOL_LEN; // is empty until trng_init() ran


// esp_random() only yields true randomness while either the RF subsystem or the SAR ADC
// entropy source is running. mLRS does not start WiFi/BT on an ESP32 main MCU, so harvest
// a pool with the ADC source switched on, then switch it off again.
// IMPORTANT: the ADC entropy source claims the SAR ADC (and I2S0 on the ESP32), so trng_init()
// must run before anything that uses them, i.e., before leds_init()/fiveway_init()/Serials.Init().

void trng_init(void)
{
    bootloader_random_enable();

    for (uint8_t i = 0; i < ESP_TRNG_POOL_LEN; i++) {
        uint32_t r;
        do {
            r = esp_random();
        } while (r == UINT32_MAX); // never store the "no TRNG" marker
        esp_trng_pool[i] = r;
    }

    bootloader_random_disable();

    esp_trng_pool_idx = 0;
}


uint32_t trng_get32(void)
{
    if (esp_trng_pool_idx >= ESP_TRNG_POOL_LEN) return UINT32_MAX; // pool exhausted

    return esp_trng_pool[esp_trng_pool_idx++];
}


#endif

#endif // ESPLIB_TRNG_H
