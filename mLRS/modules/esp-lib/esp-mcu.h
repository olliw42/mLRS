//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP MCU
//********************************************************
#ifndef ESPLIB_MCU_H
#define ESPLIB_MCU_H


//-------------------------------------------------------
// ESP Device Information
//-------------------------------------------------------

#define ESP_MCU_UID_LEN  12 // must be same as that for STM32

void mcu_uid(uint8_t uid[ESP_MCU_UID_LEN])
{
#ifndef ESP8266
uint8_t mac[6];

    esp_read_mac(mac, ESP_MAC_WIFI_STA);    

    memcpy(uid, mac, 6);
    memcpy(uid + 6, mac, 6);
#endif    
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
