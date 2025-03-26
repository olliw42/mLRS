//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Tx Tasks Handling
//********************************************************

#include <stdint.h>
#include "tasks.h"


//-------------------------------------------------------
// Tx Tasks Class Implementation
//-------------------------------------------------------


    void tTasks::Init(void)
    {
        mbridge_crsf_task_pending = MAIN_TASK_NONE;
        display_task_pending = MAIN_TASK_NONE;
        cli_task_pending = MAIN_TASK_NONE;
        esp_task_pending = MAIN_TASK_NONE;
    }

    uint8_t tTasks::Task(void)
    {
        uint8_t task;

        if (mbridge_crsf_task_pending != MAIN_TASK_NONE) {
            task = mbridge_crsf_task_pending;
            mbridge_crsf_task_pending = MAIN_TASK_NONE;
            return task;
        }

        if (display_task_pending != MAIN_TASK_NONE) {
            task = display_task_pending;
            display_task_pending = MAIN_TASK_NONE;
            return task;
        }

        if (cli_task_pending != MAIN_TASK_NONE) {
            task = cli_task_pending;
            cli_task_pending = MAIN_TASK_NONE;
            return task;
        }

        if (esp_task_pending != MAIN_TASK_NONE) {
            task = esp_task_pending;
            esp_task_pending = MAIN_TASK_NONE;
            return task;
        }

        return MAIN_TASK_NONE;
    }

    void tTasks::SetMBridgeTask(uint8_t task) { mbridge_crsf_task_pending = task; }
    void tTasks::SetCrsfTask(uint8_t task) { mbridge_crsf_task_pending = task; }
    void tTasks::SetDisplayTask(uint8_t task) { display_task_pending = task; }
    void tTasks::SetCliTask(uint8_t task) { cli_task_pending = task; }
    void tTasks::SetCliTaskAndValue(uint8_t task, int32_t value) { cli_task_pending = task; cli_task_value = value;}
    int32_t tTasks::GetCliTaskValue(void) { return cli_task_value; }
    void tTasks::SetEspTask(uint8_t task) { esp_task_pending = task; }

