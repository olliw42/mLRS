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

// Needs more work to implement on ESP8266
// Needed for entering bind mode with rapid power cycles


#include <inttypes.h>


typedef enum {
    POWERUPCNT_TASK_NONE = 0,
    POWERUPCNT_TASK_BIND,
} POWERUPCNT_TASK_ENUM;



class tPowerupCounter
{
  public:
    void Init(void) {}
    void Do(void) {}
    uint8_t Task(void) { return POWERUPCNT_TASK_NONE; }
};


#endif // ESP_POWERUP_CNT
