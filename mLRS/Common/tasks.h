//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Tasks Handling
//********************************************************
#ifndef TASKS_H
#define TASKS_H
#pragma once


//-------------------------------------------------------
// Tasks Class
//-------------------------------------------------------

typedef enum {
    MAIN_TASK_NONE = 0,

    MAIN_TASK_BIND_START,
    MAIN_TASK_BIND_STOP,
    MAIN_TASK_SYSTEM_BOOT,
    MAIN_TASK_RESTART_CONTROLLER,

    TX_TASK_RX_PARAM_SET,
    TX_TASK_PARAM_STORE,
    TX_TASK_PARAM_RELOAD,

    TX_TASK_FLASH_ESP,
    TX_TASK_ESP_PASSTHROUGH,
    TX_TASK_CLI_CHANGE_CONFIG_ID,
    TX_TASK_HC04_PASSTHROUGH,
    TX_TASK_CLI_HC04_GETPIN,
    TX_TASK_CLI_HC04_SETPIN,
} MAIN_TASK_ENUM;


class tTasks
{
  public:
    void Init(void);
    uint8_t Task(void);

    void SetMBridgeTask(uint8_t task);
    void SetCrsfTask(uint8_t task);
    void SetDisplayTask(uint8_t task);
    void SetCliTask(uint8_t task);
    void SetCliTaskAndValue(uint8_t task, int32_t value);
    int32_t GetCliTaskValue(void);
    void SetEspTask(uint8_t task);

  private:
    uint8_t mbridge_crsf_task_pending;
    uint8_t display_task_pending;
    uint8_t cli_task_pending;
    int32_t cli_task_value;
    uint8_t esp_task_pending;
};


#endif // TASKS_H
