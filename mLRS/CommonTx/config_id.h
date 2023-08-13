//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// ConfiId
//********************************************************
#ifndef CONFIGID_H
#define CONFIGID_H
#pragma once


#include <inttypes.h>


//-------------------------------------------------------
// Generic ConfigId Class
//-------------------------------------------------------

class tConfigId
{
  public:
    void Init(void);
    void Change(uint8_t config_id);
    bool Do(void);

  private:
    uint32_t change_tlast_ms;
    uint8_t new_config_id;
};



#endif // CONFIGID_H
