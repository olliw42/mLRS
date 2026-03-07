//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP MCU
//********************************************************
#ifndef ESPLIB_MCU_H
#define ESPLIB_MCU_H


#include <esp_mac.h>
#include <esp_chip_info.h>


void BootLoaderInit(void)
{
    // not sure what this one needs for the Arduino.
    // maybe it can be a hard reset using reset pin.
}


//-------------------------------------------------------
// MCU UID for DroneCAN
//-------------------------------------------------------
// DroneCAN requires a 16-byte unique ID (DC_UNIQUE_ID_LEN).
// STM32 provides a 12-byte UID via flash registers.
// ESP32 provides a 6-byte base MAC via eFuse.
// we fill 12 bytes: 6-byte MAC + 6 bytes derived from chip info,
// then mcu_cpu_id() provides the remaining 4 bytes.

#define STM32_MCU_UID_LEN  12  // compatibility with existing code

void mcu_uid(uint8_t uid[STM32_MCU_UID_LEN])
{
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    memcpy(&uid[0], mac, 6);

    // fill bytes 6-11 with chip info for additional entropy
    esp_chip_info_t info;
    esp_chip_info(&info);
    uid[6] = (uint8_t)(info.features & 0xFF);
    uid[7] = (uint8_t)((info.features >> 8) & 0xFF);
    uid[8] = (uint8_t)((info.features >> 16) & 0xFF);
    uid[9] = (uint8_t)((info.features >> 24) & 0xFF);
    uid[10] = (uint8_t)(info.revision);
    uid[11] = (uint8_t)(info.cores);
}


uint32_t mcu_cpu_id(void)
{
    esp_chip_info_t info;
    esp_chip_info(&info);
    // combine model and revision into a 32-bit value
    return ((uint32_t)info.model << 16) | ((uint32_t)info.revision);
}


#endif // ESPLIB_MCU_H
