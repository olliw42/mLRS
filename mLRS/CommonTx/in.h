//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// IN
//********************************************************
#ifndef IN_H
#define IN_H
#pragma once


#include <inttypes.h>
#include "../Common/common_types.h"


extern uint16_t micros(void);


//-------------------------------------------------------
// Generic In Class
//-------------------------------------------------------

class InBase
{
  public:
    void Init(bool enable_flag);

    void Configure(uint8_t new_config);

    bool Update(tRcData* rc);

//XX  private:
    virtual bool available(void) { return false; }
    virtual char getc(void) { return 0; }

    virtual bool config_sbus(bool enable_flag) { return false; }
    virtual bool config_sbus_inverted(bool enable_flag) { return false; }

    bool parse_sbus(tRcData* rc);
    void get_sbus_data(tRcData* rc);

    bool enabled;
    uint8_t config;
    bool initialized;

    uint16_t tlast_us;
    uint8_t state;
    uint8_t buf_pos;
    uint8_t _buf[32];
};



#endif // IN_H
