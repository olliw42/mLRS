//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP Powerup Counter
//********************************************************
#ifndef ESP_POWERUP_CNT_H
#define ESP_POWERUP_CNT_H
#pragma once

// Needed for entering bind mode with rapid power cycles


#include <inttypes.h>


typedef enum {
    POWERUPCNT_TASK_NONE = 0,
    POWERUPCNT_TASK_BIND,
} POWERUPCNT_TASK_ENUM;


// the count is kept in its own nvs namespace, and not in the emulated eeprom, so that
// the setup data is not rewritten on each power up.
// ESP32: native nvs. ESP8266: vshymanskyy/Preferences, which is backed by LittleFS.

#include <Preferences.h>


extern volatile uint32_t millis32(void);


#define POWERUPCNT_NVS_NAMESPACE  "mlrs-powerup" // max 15 chars
#define POWERUPCNT_NVS_KEY        "cnt"

#define POWERUPCNT_BIND_CNT       4 // number of rapid power ups to enter bind mode

#define POWERUPCNT_TMO_MS         2000


static bool powerup_counter_initialized = false;


class tPowerupCounter
{
  public:
    void Init(void);
    void Do(void);
    uint8_t Task(void);

  private:
    bool powerup_do;
    uint8_t task;

    Preferences nvs;
};


void tPowerupCounter::Init(void)
{
    // a soft restart is not a power up, and must not disturb a count which is in progress
    if (powerup_counter_initialized) return;
    powerup_counter_initialized = true;

    powerup_do = false;
    task = POWERUPCNT_TASK_NONE;

    if (!nvs.begin(POWERUPCNT_NVS_NAMESPACE, false)) return;

    // the count is advanced on every boot, a reset counts like a power up.
    // the timeout in Do() is what keeps a reset from leaving a stale count behind.
    uint8_t cnt = nvs.getUChar(POWERUPCNT_NVS_KEY, 0);

    cnt++;
    if (cnt > POWERUPCNT_BIND_CNT) cnt = 1; // should not happen, but exit safely

    if (cnt >= POWERUPCNT_BIND_CNT) {
        task = POWERUPCNT_TASK_BIND;
        nvs.putUChar(POWERUPCNT_NVS_KEY, 0);
        return;
    }

    nvs.putUChar(POWERUPCNT_NVS_KEY, cnt);
    powerup_do = true; // count is cleared again if we stay powered for long enough
}


void tPowerupCounter::Do(void)
{
    if (!powerup_do) return;

    if (millis32() < POWERUPCNT_TMO_MS) return;

    powerup_do = false;

    nvs.putUChar(POWERUPCNT_NVS_KEY, 0);
}


uint8_t tPowerupCounter::Task(void)
{
    switch (task) {
    case POWERUPCNT_TASK_BIND:
        task = POWERUPCNT_TASK_NONE;
        return POWERUPCNT_TASK_BIND;
    }

    return POWERUPCNT_TASK_NONE;
}


#endif // ESP_POWERUP_CNT
