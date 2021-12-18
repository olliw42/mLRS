//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// OUT
//********************************************************
#ifndef OUT_H
#define OUT_H
#pragma once


#include <inttypes.h>
#include "..\Common\frame_types.h"


//-------------------------------------------------------
// Generic Out Class
//-------------------------------------------------------

typedef enum {
  OUT_CONFIG_SBUS = 0,
} OUT_CONFIG_ENUM;


class OutBase
{
  public:
    void Init(void);

    void Configure(uint8_t new_config);

    void send_rcdata(tRcData* rc);

  private:
    void send_sbus_rcdata(tRcData* rc);
    void putbuf(uint8_t* buf, uint16_t len);

    virtual void putc(char c);
    virtual void config_sbus(void) = 0;

    uint8_t _config;
};



#endif // OUT_H
