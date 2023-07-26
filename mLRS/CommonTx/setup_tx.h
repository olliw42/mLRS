//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Setup and configuration types
//*******************************************************
#ifndef SETUP_TX_H
#define SETUP_TX_H
#pragma once


#include <stdint.h>
#include "../Common/setup.h"


//-------------------------------------------------------
// Setup parameter list
//-------------------------------------------------------
#include "../Common/setup_list.h"


#define SETUP_PARAMETER_LIST \
    SETUP_PARAMETER_LIST_COMMON \
    SETUP_PARAMETER_LIST_TX \
    SETUP_PARAMETER_LIST_RX


typedef enum {
    PARAM_INDEX_BIND_PHRASE = 0,
    PARAM_INDEX_MODE = 1,
    PARAM_INDEX_RF_BAND = 2,
} PARAM_INDEX_ENUM;


//-------------------------------------------------------
// defines
//-------------------------------------------------------

// this must EXACTLY match MAV_PARAM_TYPE !! Otherwise Mavlink will be broken !!
typedef enum {
    SETUP_PARAM_TYPE_UINT8  = 1, // 8-bit unsigned integer
    SETUP_PARAM_TYPE_INT8   = 2, // 8-bit signed integer
    SETUP_PARAM_TYPE_UINT16 = 3, // 16-bit unsigned integer
    SETUP_PARAM_TYPE_INT16  = 4, // 16-bit signed integer

    // our extensions, cannot be handled with mavlink!
    SETUP_PARAM_TYPE_LIST   = 254,
    SETUP_PARAM_TYPE_STR6   = 255,
} SETUP_PARAM_TYPE;


#define UINT8   uint8_t
#define INT8    int8_t
#define UINT16  uint16_t
#define INT16   int16_t
#define STR6    char
#define LIST    uint8_t


typedef union {
    uint8_t UINT8_value;
    int8_t INT8_value;
    uint16_t UINT16_value;
    int16_t INT16_value;
    char* STR6_value;
    uint8_t LIST_value;
} tSetupParameterValue;


typedef struct {
    void* ptr; // pointer to the field in the Setup structure
    uint8_t type;
    const char* name;
    const char* m_name;
    const char* unit;
    tSetupParameterValue dflt;
    tSetupParameterValue min;
    tSetupParameterValue max;
    const char* optstr;
    uint16_t* allowed_mask_ptr;
} tSetupParameterItem;


const tSetupParameterItem SetupParameter[] = {
    #define X(p,t, n,mn, d,mi,ma,u, s, amp) { \
                .ptr = (t*)&(p), \
                .type = SETUP_PARAM_TYPE_##t, \
                .name = n, \
                .m_name = mn, \
                .unit = u, \
                .dflt = {.t##_value=d}, \
                .min = {.t##_value=mi}, \
                .max = {.t##_value=ma}, \
                .optstr = s, \
                .allowed_mask_ptr = amp },
    SETUP_PARAMETER_LIST
    #undef X
};


#define SETUP_PARAMETER_NUM  sizeof(SetupParameter)/sizeof(tSetupParameterItem)


//-------------------------------------------------------
// helper
//-------------------------------------------------------

bool setup_param_is_tx(uint8_t param_idx)
{
    return (SetupParameter[param_idx].name[0] == 'T' && SetupParameter[param_idx].name[1] == 'x');
}


bool setup_param_is_rx(uint8_t param_idx)
{
    return (SetupParameter[param_idx].name[0] == 'R' && SetupParameter[param_idx].name[1] == 'x');
}


void* SetupParameterPtr(uint8_t param_idx)
{
    uint8_t* ptr = (uint8_t*)(SetupParameter[param_idx].ptr);
    if (setup_param_is_rx(param_idx)) return ptr;
    if (setup_param_is_tx(param_idx)) return ptr + sizeof(tTxSetup) * Config.ConfigId;
    return ptr + sizeof(tCommonSetup) * Config.ConfigId;
}


//-------------------------------------------------------
// handler
//-------------------------------------------------------

typedef union
{
    uint8_t u8;
    int8_t i8;
    uint16_t u16;
    int16_t i16;
} tParamValue;


bool _setup_param_is_for_rx(uint8_t param_idx)
{
    if (param_idx <= PARAM_INDEX_RF_BAND) return true; // BindPhrase, Mode, RF Band
    if (setup_param_is_rx(param_idx)) return true; // "Rx" name
    return false;
}


bool setup_set_param(uint8_t param_idx, tParamValue value)
{
    if (param_idx >= SETUP_PARAMETER_NUM) return false;

    bool param_changed = false;

    switch (SetupParameter[param_idx].type) {
    case SETUP_PARAM_TYPE_UINT8:
    case SETUP_PARAM_TYPE_LIST:
        if (*(uint8_t*)(SetupParameterPtr(param_idx)) != value.u8) {
            param_changed = true;
            *(uint8_t*)(SetupParameterPtr(param_idx)) = value.u8;
        }
        break;
    case SETUP_PARAM_TYPE_INT8:
        if (*(int8_t*)(SetupParameterPtr(param_idx)) != value.i8) {
            param_changed = true;
            *(int8_t*)(SetupParameterPtr(param_idx)) = value.i8;
        }
        break;
    case SETUP_PARAM_TYPE_UINT16:
        if (*(uint16_t*)(SetupParameterPtr(param_idx)) != value.u16) {
            param_changed = true;
            *(uint16_t*)(SetupParameterPtr(param_idx)) = value.u16;
        }
        break;
    case SETUP_PARAM_TYPE_INT16:
        if (*(int16_t*)(SetupParameterPtr(param_idx)) != value.i16) {
            param_changed = true;
            *(int16_t*)(SetupParameterPtr(param_idx)) = value.i16;
        }
        break;
    }

    // if a RX parameter has changed, tell it to main
    if (param_changed && _setup_param_is_for_rx(param_idx)) return true;

    return false;
}


bool setup_set_param_str6(uint8_t param_idx, char* str6_6)
{
    if (param_idx >= SETUP_PARAMETER_NUM) return false;

    bool param_changed = false;

    switch (SetupParameter[param_idx].type) {
    case SETUP_PARAM_TYPE_STR6:
        if (!strbufeq((char*)(SetupParameterPtr(param_idx)), str6_6, 6)) {
            param_changed = true;
            strstrbufcpy((char*)(SetupParameterPtr(param_idx)), str6_6, 6);
        }
        break;
    }

    // if a RX parameter has changed, tell it to main
    if (param_changed && _setup_param_is_for_rx(param_idx)) return true;

    return false;
}


#endif // SETUP_TX_H
