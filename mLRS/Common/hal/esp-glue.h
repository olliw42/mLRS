//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP Glue
//*******************************************************
#ifndef ESP_GLUE_H
#define ESP_GLUE_H
#pragma once


#include <Arduino.h>
#ifdef CONFIG_IDF_TARGET_ESP32C3
#include "esp_task_wdt.h"
#endif

// undefine MIN/MAX from to prevent redefinition when stdstm32.h is included later
#undef MIN
#undef MAX

#define __NOP() _NOP()


#undef IRQHANDLER
#define IRQHANDLER(__Declaration__)  extern "C" {IRAM_ATTR __Declaration__}


void __disable_irq(void) {}
void __enable_irq(void) {}


typedef enum {
    DISABLE = 0,
    ENABLE = !DISABLE
} FunctionalState;


// that's to provide pieces from STM32 HAL used in the code
#define HAL_I2C_MODULE_ENABLED
typedef enum
{
    HAL_OK       = 0x00U,
    HAL_ERROR    = 0x01U,
    HAL_BUSY     = 0x02U,
    HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;


#define __REV16(x)  __builtin_bswap16(x)
#define __REVSH(x)  __builtin_bswap16(x)
#define __REV(x)    __builtin_bswap32(x)


// setup(), loop() streamlining between Arduino/STM code
static uint8_t restart_controller = 0;
void setup() {}
void main_loop(void);
void loop() {
#ifdef CONFIG_IDF_TARGET_ESP32C3 // ESP32C3 needs this to get around 5 ms delay every 2 s
    extern bool loopTaskWDTEnabled;
    for (;;) { if (loopTaskWDTEnabled) { esp_task_wdt_reset(); } main_loop(); }
#else
    main_loop();
#endif
}

#define INITCONTROLLER_ONCE \
    if(restart_controller <= 1){ \
    if(restart_controller == 0){
#define RESTARTCONTROLLER \
    }
#define INITCONTROLLER_END \
    restart_controller = UINT8_MAX; \
    }
#define GOTO_RESTARTCONTROLLER \
    restart_controller = 1; \
    return;


#endif // ESP_GLUE_H

