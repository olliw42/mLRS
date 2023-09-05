//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Powerup Counter
//********************************************************
#ifndef POWERUP_CNT_H
#define POWERUP_CNT_H
#pragma once

// !!!!!!!!!!!
// ARGHHHH  we only can write twice to the same address after erased to FF, 1st to write arbitrary data, 2nd to write 00
// !!!!!!!!!!!
// So we use a full flash page, and count using the states FF, AA, 00

// ATTENTION: we must do that before RESTART!!! so that it are really only powerups!!


#include <inttypes.h>
#include "../Common/setup_types.h"


extern volatile uint32_t millis32(void);


//-------------------------------------------------------
// Generic PowerupCounter Class
//-------------------------------------------------------

#if !defined EE_USE_WORD && !defined EE_USE_DOUBLEWORD
#error Either EE_USE_WORD or EE_USE_DOUBLEWORD must be defined !
#endif
#ifndef EE_PAGE_SIZE
#error POWERUPCNT needs EE_PAGE_SIZE !
#endif
#if (EE_PAGE_SIZE < 0x0800) && (defined EE_USE_DOUBLEWORD)
#error EE_PAGE_SIZE is too small !
#endif

#define POWERUPCNT_EE_PAGE            (EE_START_PAGE + 2) // setup/eeprom uses two pages!
#define POWERUPCNT_EE_PAGE_ADDRESS    ((uint32_t)(0x08000000 + (POWERUPCNT_EE_PAGE * EE_PAGE_SIZE)))

#if defined EE_USE_WORD
  #define POWERUPCNT_FF               0xFFFFFFFF
  #define POWERUPCNT_AA               0xAAAAAAAA
  typedef uint32_t tPOWERUPCNT_UINT;
#else
  #define POWERUPCNT_FF               0xFFFFFFFFFFFFFFFF
  #define POWERUPCNT_AA               0xAAAAAAAAAAAAAAAA
  typedef uint64_t tPOWERUPCNT_UINT;
#endif


#define POWERUPCNT_TMO_MS             2000


#define POWERUPCNT_SIGNATURE_A        0x12345678FEDCBA90
#define POWERUPCNT_SIGNATURE_B        0x90BADCFE78563412


static uint64_t powerup_counter_signature = POWERUPCNT_SIGNATURE_A;


typedef enum {
    POWERUPCNT_TASK_NONE = 0,
    POWERUPCNT_TASK_BIND,
} POWERUPCNT_TASK_ENUM;


class PowerupCounterBase
{
  public:
    void Init(void);
    void Do(void);
    uint8_t Task(void);

  private:
    bool powerup_do;
    uint8_t task;

    uint32_t cur_adr;

    void ee_program_val(uint32_t adr, tPOWERUPCNT_UINT* data, uint8_t datalen);
};


void PowerupCounterBase::Init(void)
{
    powerup_do = true;
    task = POWERUPCNT_TASK_NONE;

    // check if this really was a power up, or just a reset
    if (powerup_counter_signature == POWERUPCNT_SIGNATURE_B) {
        powerup_do = false;
        return;
    }
    powerup_counter_signature = POWERUPCNT_SIGNATURE_B;

    // search for current adr to look at
    cur_adr = 0;

    for (uint16_t ofs = 0; ofs < EE_PAGE_SIZE; ofs += 2*sizeof(tPOWERUPCNT_UINT)) {
        uint32_t adr = POWERUPCNT_EE_PAGE_ADDRESS + ofs;
        tPOWERUPCNT_UINT val0 = ((tPOWERUPCNT_UINT*)(adr))[0];
        tPOWERUPCNT_UINT val1 = ((tPOWERUPCNT_UINT*)(adr))[1];
        if ((val0 == POWERUPCNT_FF || val0 == POWERUPCNT_AA || val0 == 0) &&
            (val1 == POWERUPCNT_FF || val1 == POWERUPCNT_AA)) {
            cur_adr = adr;
            break;
        }
    }

    // we couldn't find a useable adr, so erase page
    if (!cur_adr) {
        __disable_irq();
        ee_hal_erasepage(POWERUPCNT_EE_PAGE_ADDRESS, POWERUPCNT_EE_PAGE);
        __enable_irq();
        cur_adr = POWERUPCNT_EE_PAGE_ADDRESS; // let's assume erase was successful
    }

    // now we can count
    tPOWERUPCNT_UINT val[2];
    val[0] = ((tPOWERUPCNT_UINT*)(cur_adr))[0];
    val[1] = ((tPOWERUPCNT_UINT*)(cur_adr))[1];

    if (val[0] == POWERUPCNT_FF && val[1] == POWERUPCNT_FF) {
        val[0] = POWERUPCNT_AA;
        val[1] = POWERUPCNT_FF;
    } else
    if (val[0] == POWERUPCNT_AA && val[1] == POWERUPCNT_FF) {
        val[0] = POWERUPCNT_AA;
        val[1] = POWERUPCNT_AA;
    } else
    if (val[0] == POWERUPCNT_AA && val[1] == POWERUPCNT_AA) {
        val[0] = 0;
        val[1] = POWERUPCNT_AA;
    } else
    if (val[0] == 0 && val[1] == POWERUPCNT_AA) {
        task = POWERUPCNT_TASK_BIND;
        powerup_do = false;

        val[0] = 0;
        val[1] = 0;
    } else {
        // error, should not happen, but exit safely
        powerup_do = false;

        val[0] = 0;
        val[1] = 0;
    }

    ee_program_val(cur_adr, val, 2);
}


void PowerupCounterBase::Do(void)
{
    if (!powerup_do) return;

    uint32_t tnow_ms = millis32();
    if (tnow_ms < POWERUPCNT_TMO_MS) return;

    powerup_do = false;

    tPOWERUPCNT_UINT val[2] = { 0, 0 };

    ee_program_val(cur_adr, val, 2);
}


uint8_t PowerupCounterBase::Task(void)
{
    switch (task) {
    case POWERUPCNT_TASK_BIND:
        task = POWERUPCNT_TASK_NONE;
        return POWERUPCNT_TASK_BIND;
    }

    return POWERUPCNT_TASK_NONE;
}


//-------------------------------------------------------
// EE helper
//-------------------------------------------------------

#if defined EE_USE_WORD

EE_STATUS_ENUM ee_program_u32(uint32_t adr, uint32_t* data, uint8_t datalen)
{
EE_STATUS_ENUM status;
uint16_t n;

    ee_hal_unlock();

    // Write data to ToPage
#if !defined EE_USE_WORD && !defined EE_USE_DOUBLEWORD
uint16_t val;

    datalen *= 2; // adjust datalen to be 16 bit, i.e. half-word size

    for (n = 0; n < datalen; n++) {
        val = ((uint16_t*)data)[n]; // get data from specified data buffer
        if (val != *((uint16_t*)adr)) { // only write if different
            if (!ee_hal_programhalfword(adr, val)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }
        }
        adr += 2;
    }

#elif defined EE_USE_WORD
uint32_t val;

    for (n = 0; n < datalen; n++) {
        val = ((uint32_t*)data)[n]; // get data from specified data buffer
        if (val != *((uint32_t*)adr)) { // only write if different
            if (!ee_hal_programword(adr, val)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }
        }
        adr += 4;
    }

#else
#error Cannot be used with EE_USE_DOUBLEWORD !
#endif

    status = EE_STATUS_OK;
QUICK_EXIT:
    ee_hal_lock();
    return status;
}


void PowerupCounterBase::ee_program_val(uint32_t adr, uint32_t* data, uint8_t datalen)
{
    ee_program_u32(adr, data, datalen);
}

#else

EE_STATUS_ENUM ee_program_u64(uint32_t adr, uint64_t* data, uint8_t datalen)
{
EE_STATUS_ENUM status;
uint16_t n;

    ee_hal_unlock();

    // Write data to ToPage
#if !defined EE_USE_WORD && !defined EE_USE_DOUBLEWORD
uint16_t val;

    datalen *= 4; // adjust datalen to be 16 bit, i.e. half-word size

    for (n = 0; n < datalen; n++) {
        val = ((uint16_t*)data)[n]; // get data from specified data buffer
        if (val != *((uint16_t*)adr)) { // only write if different
            if (!ee_hal_programhalfword(adr, val)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }
        }
        adr += 2;
    }

#elif defined EE_USE_WORD
uint32_t val;

    datalen *= 2; // adjust datalen to be 32 bit, i.e. word size

    for (n = 0; n < datalen; n++) {
        val = ((uint32_t*)data)[n]; // get data from specified data buffer
        if (val != *((uint32_t*)adr)) { // only write if different
            if (!ee_hal_programword(adr, val)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }
        }
        adr += 4;
    }

#else
uint64_t val;

    for (n = 0; n < datalen; n++) {
        val = ((uint64_t*)data)[n];
        if (val != *((uint64_t*)adr)) { // only write if different
            if (!ee_hal_programdoubleword(adr, val)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }
        }
        adr += 8;
    }

#endif

    status = EE_STATUS_OK;
QUICK_EXIT:
    ee_hal_lock();
    return status;
}


void PowerupCounterBase::ee_program_val(uint32_t adr, uint64_t* data, uint8_t datalen)
{
    ee_program_u64(adr, data, datalen);
}

#endif


#endif // POWERUP_CNT
