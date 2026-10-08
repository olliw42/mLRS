//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP Glue
//*******************************************************
#ifndef ESP_GLUE_H
#define ESP_GLUE_H
#pragma once


#include <Arduino.h>
#ifdef CONFIG_IDF_TARGET_ESP32C3
#include "esp_task_wdt.h"
// undefine MIN/MAX to prevent redefinition warning when stdstm32.h is included later
#undef MIN
#undef MAX
#endif


#define __NOP() _NOP()


// a dio bootloader (e.g. left in place by the ELRS flasher) doesn't route the flash WP/HD pins, and
// nvs then fails with our qio build. So do here what the qio bootloader does, before nvs is started.
// only in c++, this file is also included by c code
#if defined __cplusplus && defined CONFIG_IDF_TARGET_ESP32 && defined CONFIG_ESPTOOLPY_FLASHMODE_QIO
#include "bootloader_flash_config.h"
#include "esp32/rom/efuse.h"
#include "esp32/rom/spi_flash.h"
#include "soc/spi_reg.h"
#include "esp_flash.h"

extern "C" void spi_flash_disable_interrupts_caches_and_other_cpu(void);
extern "C" void spi_flash_enable_interrupts_caches_and_other_cpu(void);

// a dio bootloader also leaves code execution from flash in dio, so switch it to qio as the qio
// bootloader does. Must run from ram with the cache off, as it changes how code is read from flash.
IRAM_ATTR __attribute__((noinline)) void esp_flash_qio_exec_init(void)
{
    spi_flash_disable_interrupts_caches_and_other_cpu();
    esp_rom_spiflash_config_readmode(ESP_ROM_SPIFLASH_QIO_MODE);
    spi_flash_enable_interrupts_caches_and_other_cpu();
}

__attribute__((constructor)) void esp_flash_qio_pins_init(void)
{
    esp_rom_spiflash_select_qio_pins(bootloader_flash_get_wp_pin(), ets_efuse_get_spiconfig());

    // the flash driver has set the flash's quad enable bit at this point, skip if it has not
    if (!esp_flash_default_chip || !esp_flash_is_quad_mode(esp_flash_default_chip)) return;
    if (REG_READ(SPI_CTRL_REG(0)) & SPI_FREAD_QIO) return; // qio bootloader, nothing to do
    esp_flash_qio_exec_init();
}
#endif

#ifdef ESP32
#include <nvs_flash.h>
#endif


#undef IRQHANDLER
#define IRQHANDLER(__Declaration__)  extern "C" {IRAM_ATTR __Declaration__}


void __disable_irq(void) {}
void __enable_irq(void) {}


typedef enum {
    DISABLE = 0,
    ENABLE = !DISABLE
} FunctionalState;


// that's to provide pieces from STM32 HAL used in the code
#define HAL_I2C_MODULE_ENABLED
typedef enum
{
    HAL_OK       = 0x00U,
    HAL_ERROR    = 0x01U,
    HAL_BUSY     = 0x02U,
    HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;


#define __REV16(x)  __builtin_bswap16(x)
#define __REVSH(x)  __builtin_bswap16(x)
#define __REV(x)    __builtin_bswap32(x)


// setup(), loop() streamlining between Arduino/STM code
static uint8_t restart_controller = 0;
void setup() {
#ifdef ESP32
    // arduino recovers nvs only for some of the errors, so do it here for all others
    if (nvs_flash_init() != ESP_OK) { nvs_flash_erase(); nvs_flash_init(); }
#endif
}
void main_loop(void);
void loop() {
#ifdef CONFIG_IDF_TARGET_ESP32C3 // ESP32C3 needs this to get around 5 ms delay every 2 s
    extern bool loopTaskWDTEnabled;
    for (;;) { if (loopTaskWDTEnabled) { esp_task_wdt_reset(); } main_loop(); }
#else
    main_loop();
#endif
}

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


#endif // ESP_GLUE_H

