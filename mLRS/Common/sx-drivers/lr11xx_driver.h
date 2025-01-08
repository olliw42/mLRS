//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// LR11XX Driver
//*******************************************************

#ifndef LR11XX_DRIVER_H
#define LR11XX_DRIVER_H
#pragma once


//-------------------------------------------------------
// SX Driver
//-------------------------------------------------------

const tSxLoraConfiguration Lr11xxLoraConfiguration[] = {
    { .SpreadingFactor = LR11XX_LORA_SF5,
      .Bandwidth = LR11XX_LORA_BW_500,
      .CodingRate = LR11XX_LORA_CR_4_5,
      .PreambleLength = 12,
      .HeaderType = LR11XX_LORA_HEADER_DISABLE,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CrcEnabled = LR11XX_LORA_CRC_DISABLE,
      .InvertIQ = LR11XX_LORA_IQ_NORMAL,
      .TimeOverAir = 13200,
      .ReceiverSensitivity = -111,
    },
    { .SpreadingFactor = LR11XX_LORA_SF6,
      .Bandwidth = LR11XX_LORA_BW_500,
      .CodingRate = LR11XX_LORA_CR_4_5,
      .PreambleLength = 12,
      .HeaderType = LR11XX_LORA_HEADER_DISABLE,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CrcEnabled = LR11XX_LORA_CRC_DISABLE,
      .InvertIQ = LR11XX_LORA_IQ_NORMAL,
      .TimeOverAir = 22560,
      .ReceiverSensitivity = -112,
    }
};


#ifdef POWER_USE_DEFAULT_RFPOWER_CALC
void lr11xx_rfpower_calc(const int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm, const uint8_t GAIN_DBM, const uint8_t LR11XX_MAX_DBM)
{
    int16_t power_sx = (int16_t)power_dbm - GAIN_DBM;

    if (power_sx < LR11XX_POWER_MIN) power_sx = LR11XX_POWER_MIN;
    if (power_sx > LR11XX_POWER_MAX) power_sx = LR11XX_POWER_MAX;
    if (power_sx > LR11XX_MAX_DBM) power_sx = LR11XX_MAX_DBM;

    *sx_power = power_sx;
    *actual_power_dbm = power_sx + GAIN_DBM;
}
#endif


class Lr11xxDriverCommon : public Lr11xxDriverBase
{
  public:

    void Init(void)
    {
        lora_configuration = nullptr;
    }

    bool isOk(void)
    {
        uint8_t hwVersion;
        uint8_t useCase;
        uint8_t fwMajor;
        uint8_t fwMinor;
        
        GetVersion(&hwVersion, &useCase, &fwMajor, &fwMinor);

        uint16_t firmwareRev = (static_cast<uint16_t>(fwMajor) << 8) | fwMinor;

        return ((firmwareRev != 0) && (firmwareRev != 65535));
    }

    void SetLoraConfiguration(const tSxLoraConfiguration* const config)
    {
        SetModulationParams(config->SpreadingFactor,
                            config->Bandwidth,
                            config->CodingRate,
                            LR11XX_LORA_LDR_OFF);

        SetPacketParams(config->PreambleLength,
                        config->HeaderType,
                        config->PayloadLength,
                        config->CrcEnabled,
                        config->InvertIQ);
    }

    void SetLoraConfigurationByIndex(uint8_t index)
    {
        if (index >= sizeof(Lr11xxLoraConfiguration)/sizeof(Lr11xxxLoraConfiguration[0])) while(1){} // must not happen

        lora_configuration = &(Lr11xxLoraConfiguration[index]);
        SetLoraConfiguration(lora_configuration);
    }

    void ResetToLoraConfiguration(void)
    {
        SetStandby(LR11XX_STDBY_CONFIG_STDBY_RC);
        delay_us(1000);
        SetPacketType(LR11XX_PACKET_TYPE_LORA);
        SetLoraConfigurationByIndex(gconfig->LoraConfigIndex);
    }

    void SetRfPower_dbm(int8_t power_dbm)
    {
        RfPowerCalc(power_dbm, &sx_power, &actual_power_dbm);
        SetTxParams(sx_power, LR11XX_RAMPTIME_48_US); // Closet to 40 uS used by SX126x / SX127x
    }

    void UpdateRfPower(tSxGlobalConfig* const global_config)
    {
        gconfig->Power_dbm = global_config->Power_dbm;
        SetRfPower_dbm(gconfig->Power_dbm);
    }

    void Configure(tSxGlobalConfig* const global_config)
    {
      gconfig = global_config;

        if (gconfig->modeIsLora()) {
            SetPacketType(LR11XX_PACKET_TYPE_LORA);
        } else {
            SetPacketType(LR11XX_PACKET_TYPE_GFSK);
        }

    }


  protected:
    tSxGlobalConfig* gconfig;

  private:
    const tSxLoraConfiguration* lora_configuration;
    uint8_t sx_power;
    int8_t actual_power_dbm;
};


//-------------------------------------------------------
// Driver for SX1
//-------------------------------------------------------


//-------------------------------------------------------
// Driver for SX2
//-------------------------------------------------------

// Note - ELRS hardware use single SPI, look at SX128x driver

#endif // LR11XX_DRIVER_H
