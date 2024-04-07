//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP Glue
//*******************************************************

#include <Arduino.h>

#define __NOP() _NOP()


void hal_init(void)
{
    // nothing to do
}


// setup(), loop() streamlining between Arduino/STM code
uint8_t restart_controller = 0;
void setup() {}
void main_loop(void);
void loop() { main_loop(); }

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



