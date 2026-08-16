//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Tx Tasks Handling
//********************************************************

#include <stdint.h>
#include <string.h>
#include "tasks.h"


//-------------------------------------------------------
// Tx Tasks Class Implementation
//-------------------------------------------------------

void tTxTasks::Init(void)
{
    mbridge_crsf_task_pending = TASK_NONE;
    display_task_pending = TASK_NONE;
    cli_task_pending = TASK_NONE;
    esp_task_pending = TASK_NONE;
}


uint8_t tTxTasks::Task(void)
{
uint8_t task;

    if (mbridge_crsf_task_pending != TASK_NONE) {
        task = mbridge_crsf_task_pending;
        mbridge_crsf_task_pending = TASK_NONE;
        return task;
    }

    if (display_task_pending != TASK_NONE) {
        task = display_task_pending;
        display_task_pending = TASK_NONE;
        return task;
    }

    if (cli_task_pending != TASK_NONE) {
        task = cli_task_pending;
        cli_task_pending = TASK_NONE;
        return task;
    }

    if (esp_task_pending != TASK_NONE) {
        task = esp_task_pending;
        esp_task_pending = TASK_NONE;
        return task;
    }

    if (mavlink_task_pending != TASK_NONE) {
        task = mavlink_task_pending;
        mavlink_task_pending = TASK_NONE;
        return task;
    }

    return TASK_NONE;
}


void tTxTasks::SetMBridgeTask(uint8_t task) { mbridge_crsf_task_pending = task; }
void tTxTasks::SetCrsfTask(uint8_t task) { mbridge_crsf_task_pending = task; }
void tTxTasks::SetDisplayTask(uint8_t task) { display_task_pending = task; }
void tTxTasks::SetCliTask(uint8_t task) { cli_task_pending = task; }
void tTxTasks::SetCliTaskConfigIdValue(uint8_t task, int32_t value) { cli_task_pending = task; config_id_value = value;}
void tTxTasks::SetCliTaskHc04Value(uint8_t task, int32_t value) { cli_task_pending = task; hc04_value = value;}
void tTxTasks::SetCliTaskEspStr(uint8_t task, char* const str) { cli_task_pending = task; strncpy(esp_str, str, sizeof(esp_str)-1); }
void tTxTasks::SetEspTask(uint8_t task) { esp_task_pending = task; }
void tTxTasks::SetMavlinkTask(uint8_t task) { mavlink_task_pending = task; }

uint32_t tTxTasks::GetConfigIdValue(void) { return config_id_value; }
uint32_t tTxTasks::GetHc04Value(void) { return hc04_value; }
char* tTxTasks::GetEspStr(void) { return esp_str; }


