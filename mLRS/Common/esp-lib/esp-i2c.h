//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP I2C
//*******************************************************
#ifndef ESPLIB_I2C_H
#define ESPLIB_I2C_H
#ifdef __cplusplus
extern "C" {
#endif


//-------------------------------------------------------
// Defines
//-------------------------------------------------------

#include "esp-peripherals.h"
#include <Wire.h>
// https://github.com/espressif/arduino-esp32/blob/master/libraries/Wire/src/Wire.h


//-------------------------------------------------------
//  I2C Core 0 task
//-------------------------------------------------------
// offloads blocking Wire data transfers to Core 0 so that
// Core 1 (radio) is not blocked during display updates.
// the JRPin5 100us timer ISR also runs on Core 0 but is
// harmless — ~2us preemption, Wire resumes immediately.

uint8_t i2c_dev_adr;

struct {
    uint8_t reg_adr;
    const uint8_t* buf;
    uint16_t len;
    volatile bool busy;
    TaskHandle_t task_handle;
} i2c_async;


// call this before transaction
IRAM_ATTR void i2c_setdeviceadr(uint8_t dev_adr)
{
    i2c_dev_adr = dev_adr;
}


IRAM_ATTR uint8_t i2c_getdeviceadr(void)
{
    return i2c_dev_adr;
}


// blocking Wire call, used for init and small cmd writes (cmdhome)
// must only be called before i2c_task starts, or from Core 0
IRAM_ATTR HAL_StatusTypeDef i2c_put_blocked(uint8_t reg_adr, uint8_t* buf, uint16_t len)
{
    Wire.beginTransmission(i2c_dev_adr);
    Wire.write(reg_adr);
    Wire.write(buf, len);
    uint8_t error = Wire.endTransmission(true);
    return (error == 0) ? HAL_OK : HAL_ERROR;
}


// async transfer via Core 0 task, passes pointer (no copy)
// caller must ensure buf stays valid until transfer completes
IRAM_ATTR HAL_StatusTypeDef i2c_put(uint8_t reg_adr, uint8_t* buf, uint16_t len)
{
    if (i2c_async.busy) return HAL_ERROR;

    i2c_async.reg_adr = reg_adr;
    i2c_async.buf = buf;
    i2c_async.len = len;
    i2c_async.busy = true;
    xTaskNotifyGive(i2c_async.task_handle);
    return HAL_OK;
}


IRAM_ATTR HAL_StatusTypeDef i2c_device_ready(void)
{
    return (i2c_async.busy) ? HAL_BUSY : HAL_OK;
}


// page-by-page display transfer on Core 0
// uses page addressing mode (compatible with SSD1306 and CH1115/NFP1115)
// all Wire access stays on Core 0, no cross-core contention
void i2c_task(void* param)
{
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        const uint8_t* buf = i2c_async.buf;
        uint16_t remaining = i2c_async.len;

        for (uint8_t page = 0; remaining > 0; page++) {
            uint16_t chunk = (remaining < 128) ? remaining : 128;

            // set page address and column start
            Wire.beginTransmission(i2c_dev_adr);
            Wire.write((uint8_t)0x00); // command register
            Wire.write((uint8_t)0x00); // lower column address
            Wire.write((uint8_t)0x10); // upper column address
            Wire.write((uint8_t)(0xB0 + page)); // page address
            Wire.endTransmission(true);

            // write page data
            Wire.beginTransmission(i2c_dev_adr);
            Wire.write(i2c_async.reg_adr);
            Wire.write(buf, chunk);
            Wire.endTransmission(true);

            buf += chunk;
            remaining -= chunk;
        }

        i2c_async.busy = false;
    }
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void i2c_init(void)
{
    Wire.begin(I2C_SDA_IO, I2C_SCL_IO, I2C_CLOCKSPEED);
    Wire.setBufferSize(128 + 2); // 128 bytes per page + register address overhead
    Wire.setTimeout(2);

    i2c_async.busy = false;
    i2c_async.task_handle = NULL;

    xTaskCreatePinnedToCore(i2c_task, "I2C", 2048, NULL, 1, &i2c_async.task_handle, 0); // Core 0, Priority 1
}


#ifdef __cplusplus
}
#endif
#endif // ESPLIB_I2C_H
