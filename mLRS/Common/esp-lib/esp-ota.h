//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP OTA
// thin wrapper around ESP-IDF OTA APIs for firmware
// update over CAN (DroneCAN)
//*******************************************************
#ifndef ESPLIB_OTA_H
#define ESPLIB_OTA_H

#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_system.h>


typedef enum {
    ESP_OTA_STATE_IDLE = 0,
    ESP_OTA_STATE_IN_PROGRESS,
    ESP_OTA_STATE_DONE,
    ESP_OTA_STATE_ERROR,
} esp_ota_state_e;


class tEspOta
{
  public:

    // prepare the next OTA partition for writing
    // returns true on success
    bool esp_ota_hal_begin(void)
    {
        if (_ota_state == ESP_OTA_STATE_IN_PROGRESS) return false; // already in progress

        _ota_partition = esp_ota_get_next_update_partition(NULL);
        if (_ota_partition == NULL) {
            _ota_state = ESP_OTA_STATE_ERROR;
            return false;
        }

        // OTA_SIZE_UNKNOWN allows streaming without knowing total size up front
        esp_err_t err = esp_ota_begin(_ota_partition, OTA_SIZE_UNKNOWN, &_ota_handle);
        if (err != ESP_OK) {
            _ota_state = ESP_OTA_STATE_ERROR;
            return false;
        }

        _ota_state = ESP_OTA_STATE_IN_PROGRESS;
        _ota_bytes_written = 0;
        return true;
    }


    // write a chunk of firmware data to the OTA partition
    // returns true on success
    bool esp_ota_hal_write_chunk(const uint8_t* data, uint16_t len)
    {
        if (_ota_state != ESP_OTA_STATE_IN_PROGRESS) return false;
        if (len == 0) return true; // nothing to write

        esp_err_t err = esp_ota_write(_ota_handle, data, len);
        if (err != ESP_OK) {
            _ota_state = ESP_OTA_STATE_ERROR;
            return false;
        }

        _ota_bytes_written += len;
        return true;
    }


    // finalize the OTA update, set boot partition, and restart
    // does not return on success
    bool esp_ota_hal_finish(void)
    {
        if (_ota_state != ESP_OTA_STATE_IN_PROGRESS) return false;

        esp_err_t err = esp_ota_end(_ota_handle);
        if (err != ESP_OK) {
            _ota_state = ESP_OTA_STATE_ERROR;
            return false;
        }

        err = esp_ota_set_boot_partition(_ota_partition);
        if (err != ESP_OK) {
            _ota_state = ESP_OTA_STATE_ERROR;
            return false;
        }

        _ota_state = ESP_OTA_STATE_DONE;

        // delay briefly so the CAN TX queue can drain any final status messages
        delay_ms(100);

        esp_restart(); // does not return
        return true; // unreachable
    }


    // abort an in-progress OTA update and clean up
    void esp_ota_hal_abort(void)
    {
        if (_ota_state == ESP_OTA_STATE_IN_PROGRESS) {
            esp_ota_abort(_ota_handle);
        }
        _ota_handle = 0;
        _ota_partition = NULL;
        _ota_bytes_written = 0;
        _ota_state = ESP_OTA_STATE_IDLE;
    }


    uint32_t esp_ota_hal_bytes_written(void) { return _ota_bytes_written; }
    esp_ota_state_e esp_ota_hal_state(void) { return _ota_state; }

  private:
    esp_ota_handle_t _ota_handle = 0;
    const esp_partition_t* _ota_partition = NULL;
    esp_ota_state_e _ota_state = ESP_OTA_STATE_IDLE;
    uint32_t _ota_bytes_written = 0;
};


#endif // ESPLIB_OTA_H
