//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP MCU
//********************************************************
#ifndef ESPLIB_MCU_H
#define ESPLIB_MCU_H


#ifdef ESP8266
#include "user_interface.h"
#endif


//-------------------------------------------------------
// ESP Device Information
//-------------------------------------------------------

#define ESP_MCU_UID_LEN  12 // must be same as that for STM32


void mcu_uid(uint8_t uid[ESP_MCU_UID_LEN])
{
uint8_t mac[6];

#ifdef ESP8266
    wifi_get_macaddr(STATION_IF, mac);
#else
    esp_efuse_mac_get_default(mac);
#endif

    memcpy(uid, mac, 6);
    memcpy(uid + 6, mac, 6);
}


//-------------------------------------------------------
// BootLoaderInit
//-------------------------------------------------------

void BootLoaderInit(void)
{
    // Not sure what this one needs for the Arduino.
    // Maybe can be a hard reset using reset pin.
};


#endif // ESPLIB_MCU_H
