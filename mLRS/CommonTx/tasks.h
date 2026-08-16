//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Tx Tasks Handling
//********************************************************
#ifndef TX_TASKS_H
#define TX_TASKS_H
#pragma once


//-------------------------------------------------------
// Tasks Class
//-------------------------------------------------------

typedef enum {
    TASK_NONE = 0,

    TASK_BIND_START,
    TASK_BIND_STOP,
    TASK_SYSTEM_BOOT,
    TASK_RESTART_CONTROLLER,

    TASK_RX_PARAM_SET,
    TASK_PARAM_STORE,
    TASK_PARAM_RELOAD,
    TASK_CHANGE_CONFIG_ID,

    TASK_ESP_FLASH,
    TASK_ESP_PASSTHROUGH,
    TASK_ESP_GET_PASSWORD, // TCP, UDP, UDPSTA
    TASK_ESP_SET_PASSWORD,
    TASK_ESP_GET_NETWORK_SSID, // UDPSTA
    TASK_ESP_SET_NETWORK_SSID,

    TASK_HC04_PASSTHROUGH,
    TASK_HC04_GETPIN,
    TASK_HC04_SETPIN,
} TX_TASK_ENUM;


class tTxTasks
{
  public:
    void Init(void);
    uint8_t Task(void);

    void SetMBridgeTask(uint8_t task);
    void SetCrsfTask(uint8_t task);
    void SetDisplayTask(uint8_t task);
    void SetCliTask(uint8_t task);
    void SetCliTaskConfigIdValue(uint8_t task, int32_t value);
    void SetCliTaskHc04Value(uint8_t task, int32_t value);
    void SetCliTaskEspStr(uint8_t task, char* str);
    void SetEspTask(uint8_t task);
    void SetMavlinkTask(uint8_t task);

    uint32_t GetConfigIdValue(void);
    uint32_t GetHc04Value(void);
    char* GetEspStr(void);

  private:
    uint8_t mbridge_crsf_task_pending;
    uint8_t display_task_pending;
    uint8_t cli_task_pending;
    uint8_t esp_task_pending;
    uint8_t mavlink_task_pending;

    uint32_t config_id_value;
    uint32_t hc04_value;
    char esp_str[32+1];
};


#endif // TX_TASKS_H
