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
#include "..\Common\common_types.h"
#include "..\Common\frame_types.h"
#include "..\Common\setup_types.h"


//-------------------------------------------------------
// Generic In Class
//-------------------------------------------------------

class InBase
{
  public:
    void Init(void);

    void Configure(uint8_t new_config);

    bool Update(tRcData* rc);

  private:
    virtual bool available(void) { return false; }
    virtual char getc(void) { return 0; }
    virtual uint16_t tim_1us(void) { return 0; }

    virtual void config_sbus(bool inverted) {}
    bool parse_sbus(tRcData* rc);
    void get_sbus_data(tRcData* rc);

    uint8_t _config;

    uint16_t _t_last_us;
    uint8_t _state;
    uint8_t _buf_pos;
    uint8_t _buf[32];
};



#endif // IN_H
