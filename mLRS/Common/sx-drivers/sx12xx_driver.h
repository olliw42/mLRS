//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// SX12XX driver shim
//*******************************************************
#ifndef SX12XX_DRIVER_H
#define SX12XX_DRIVER_H
#pragma once


#include "..\hal\device_conf.h"

#ifdef DEVICE_HAS_SX126x
#include "sx126x_driver.h"
#elif defined DEVICE_HAS_SX127x
#include "sx127x_driver.h"
#else
#include "sx128x_driver.h"
#endif


class SxDriverDummy
{
  public:
    void Init(void) {}
    bool isOk(void) { return true; }
    void StartUp(void) {}
    void SetRfFrequency(uint32_t RfFrequency) {}
    void GetPacketStatus(int8_t* RssiSync, int8_t* Snr) {}
    void SendFrame(uint8_t* data, uint8_t len, uint16_t tmo_ms) {}
    void ReadFrame(uint8_t* data, uint8_t len) {}
    void SetToRx(uint16_t tmo_ms) {}
    void SetToIdle(void) {}
};


#endif // SX12XX_DRIVER_H
