// Lightweight ESP32 powerup counter
// Requires Arduino Preferences (NVS) available in the ESP32 Arduino core.
// Counts power cycles and triggers after 2 power cycles.
// Designed to be small and easy to integrate: call Init() on boot,
// call Do() periodically (from main loop or a periodic task) so the
// stored counter can be cleared after the timeout.

#ifndef POWERUP_ESP32_SIMPLE_H
#define POWERUP_ESP32_SIMPLE_H
#pragma once

#if !defined(ARDUINO_ARCH_ESP32)
#error powerup_esp32_simple.h is only for ESP32 builds (ARDUINO_ARCH_ESP32)
#endif

#include <Arduino.h>
#include <Preferences.h>
#include <inttypes.h>

// Timeout after which an in-progress counter is cleared (ms).
#ifndef POWERUPCNT_SIMPLE_TMO_MS
#define POWERUPCNT_SIMPLE_TMO_MS 2000
#endif

// Number of power-cycles required to trigger the action.
#ifndef POWERUPCNT_SIMPLE_REQUIRED
#define POWERUPCNT_SIMPLE_REQUIRED 2
#endif

typedef enum {
    POWERUPCNT_TASK_NONE = 0,
    POWERUPCNT_TASK_BIND,
    POWERUPCNT_CLI_MODE
} POWERUPCNT_TASK_ENUM;


class tPowerupCounterESP32Simple {
  public:
    tPowerupCounterESP32Simple() : powerup_do(false), task(POWERUPCNT_TASK_NONE), start_ms(0) {}
    // Call once early during startup
    void Init(void);
    // Call periodically (e.g. in main loop) so the timeout handler clears the entry
    void Do(void);
    // Return any task detected by Init(); calling Task() clears the pending task
    uint8_t Task(void);

  private:
    bool powerup_do; // waiting for timeout to clear counter
    uint8_t task;
    uint32_t start_ms;
    static const char* PREF_NAMESPACE() { return "mLRS_pwr"; }
    static const char* PREF_KEY() { return "pwrcnt"; }
};


void tPowerupCounterESP32Simple::Init(void)
{
    Preferences prefs;
    uint16_t cnt = 0;

    // Open NVS namespace, read, increment, write back
    if (prefs.begin(PREF_NAMESPACE(), false)) {
        cnt = prefs.getUShort(PREF_KEY(), 0);
        cnt++;
        prefs.putUShort(PREF_KEY(), cnt);
        prefs.end();
    }

    // If we reached or exceeded required cycles, trigger and reset counter
    if (cnt >= POWERUPCNT_SIMPLE_REQUIRED) {
        task = POWERUPCNT_CLI_MODE; // conservative default
        // allow user to change to CLI if needed by checking their board macro
        // Clear immediately so subsequent boots start fresh
        if (prefs.begin(PREF_NAMESPACE(), false)) {
            prefs.putUShort(PREF_KEY(), 0);
            prefs.end();
        }
        powerup_do = false;
    } else {
        // Wait POWERUPCNT_SIMPLE_TMO_MS while the device stays on; if no
        // second boot happens within that window, Do() will clear the counter.
        powerup_do = true;
        start_ms = millis();
    }
}


void tPowerupCounterESP32Simple::Do(void)
{
    if (!powerup_do) return;
    uint32_t now = millis();
    // handle wrap-around safe comparison
    if ((uint32_t)(now - start_ms) < (uint32_t)POWERUPCNT_SIMPLE_TMO_MS) return;

    // timeout passed: clear the stored counter so next powerup starts fresh
    Preferences prefs;
    if (prefs.begin(PREF_NAMESPACE(), false)) {
        prefs.putUShort(PREF_KEY(), 0);
        prefs.end();
    }
    powerup_do = false;
}

uint8_t tPowerupCounterESP32Simple::Task(void)
{
    uint8_t ret = task;
    task = POWERUPCNT_TASK_NONE;
    return ret;
}

#endif // POWERUP_ESP32_SIMPLE_H
