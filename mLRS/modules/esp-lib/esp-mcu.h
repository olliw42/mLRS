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


//-------------------------------------------------------
// Flash QIO
//-------------------------------------------------------

#if defined CONFIG_IDF_TARGET_ESP32 && defined CONFIG_ESPTOOLPY_FLASHMODE_QIO
#include "bootloader_flash_config.h"
#include "esp32/rom/efuse.h"
#include "esp32/rom/spi_flash.h"
#include "esp_spi_flash.h"
#include "soc/spi_reg.h"

// a dio bootloader (e.g. kept when flashing via ELRS wifi) never sets up qio, which breaks nvs in our qio build
// so do what the qio bootloader does, except the quad enable bit, which the idf flash driver init already sets

// no caller in our code, constructor makes idf startup run it from do_global_ctors()
// that is after the flash driver init and before initArduino(), which does the first nvs_flash_init()
IRAM_ATTR __attribute__((constructor)) void esp_flash_qio_init(void)
{
    if (REG_READ(SPI_CTRL_REG(0)) & SPI_FREAD_QIO) return; // qio bootloader did it all, spi0 is what the cache reads through

    // rom: set the wp/hd pin functions for quad io, needed for nvs since the flash driver reads in qio
    // pins from the efuse spi config (0 = default), wp pin from bootloader_support for custom pin maps
    esp_rom_spiflash_select_qio_pins(bootloader_flash_get_wp_pin(), ets_efuse_get_spiconfig());

    // switch code execution from flash to qio, for performance, the guard turns the flash cache off and back on
    // scheduler not yet started and the app cpu spins in iram, so only iram and rom code until the cache is back on
    g_flash_guard_default_ops.start();
    esp_rom_spiflash_config_readmode(ESP_ROM_SPIFLASH_QIO_MODE); // idf iram patch, sets spi0 and spi1 and the cache read command and dummy
    g_flash_guard_default_ops.end();
}
#endif


#endif // ESPLIB_MCU_H