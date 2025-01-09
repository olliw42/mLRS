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
        SetTxParams(sx_power, LR11XX_RAMPTIME_48_US); // Closest to 40 uS used by SX126x / SX127x
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

      switch (gconfig->FrequencyBand) {
        case SX_FHSS_CONFIG_FREQUENCY_BAND_915_MHZ_FCC: CalibrateImage(LR11XX_CAL_IMG_902_MHZ_1, LR11XX_CAL_IMG_902_MHZ_2); break;
        case SX_FHSS_CONFIG_FREQUENCY_BAND_868_MHZ: CalibrateImage(LR11XX_CAL_IMG_863_MHZ_1, LR11XX_CAL_IMG_863_MHZ_2); break;
        case SX_FHSS_CONFIG_FREQUENCY_BAND_866_MHZ_IN: CalibrateImage(LR11XX_CAL_IMG_863_MHZ_1, LR11XX_CAL_IMG_863_MHZ_2); break;
        case SX_FHSS_CONFIG_FREQUENCY_BAND_433_MHZ: CalibrateImage(LR11XX_CAL_IMG_430_MHZ_1, LR11XX_CAL_IMG_430_MHZ_2); break;
        case SX_FHSS_CONFIG_FREQUENCY_BAND_70_CM_HAM: CalibrateImage(LR11XX_CAL_IMG_430_MHZ_1, LR11XX_CAL_IMG_430_MHZ_2); break;
        default:
            while(1){} // protection
      }

      SetRxTxFallbackMode(LR11XX_RX_TX_FALLBACK_MODE_FS);
      SetRxBoosted(LR11XX_RX_GAIN_BOOSTED_GAIN);

      SetPaConfig(LR11XX_PA_SELECT_HP_PA, LR11XX_REG_PA_SUPPLY_VBAT, LR11XX_PA_CONFIG_22_DBM_PA_DUTY_CYCLE, LR11XX_PA_CONFIG_22_DBM_HP_MAX);

      SetDioAsRfSwitch(0b0000111, 0, 0b0000010, 0b0000100,  0b00001000, 0b00000010)  // Clean up?

      SetDioIrqParams(LR11XX_IRQ_TX_DONE | LR11XX_IRQ_TX_DONE | LR11XX_IRQ_TIMEOUT, 0);  // DIO1 only
      ClearIrqStatus(LR11XX_IRQ_TX_DONE | LR11XX_IRQ_TX_DONE | LR11XX_IRQ_TIMEOUT, 0);  // DIO1 only

      if (gconfig->modeIsLora()) {
            SetLoraConfigurationByIndex(gconfig->LoraConfigIndex);
      } else {
          // FSK reserve
      }

      SetFs();
    }

    //-- this are the API functions used in the loop

    void ReadFrame(uint8_t* const data, uint8_t len)
    {
        uint8_t rxStartBufferPointer;
        uint8_t rxPayloadLength;

        GetRxBufferStatus(&rxPayloadLength, &rxStartBufferPointer);
        ReadBuffer(rxStartBufferPointer, data, len);
    }

    void SendFrame(uint8_t* const data, uint8_t len, uint16_t tmo_ms)
    {
        WriteBuffer(0, data, len);
        ClearIrqStatus(LR11XX_IRQ_TX_DONE | LR11XX_IRQ_TX_DONE | LR11XX_IRQ_TIMEOUT, 0);
        SetTx(tmo_ms * 33); // 0 = no timeout. TimeOut period in ms. LR11xx have static 30.517 uS (1 / 32768) period base, so for 1 ms needs 33 tmo value
    }

    void SetToRx(uint16_t tmo_ms)
    {
        ClearIrqStatus(LR11XX_IRQ_TX_DONE | LR11XX_IRQ_TX_DONE | LR11XX_IRQ_TIMEOUT, 0);
        SetRx(tmo_ms * 33); // 0 = no timeout
    }

    void SetToIdle(void)
    {
        SetFs();
        ClearIrqStatus(LR11XX_IRQ_TX_DONE | LR11XX_IRQ_TX_DONE | LR11XX_IRQ_TIMEOUT, 0);
    }

    void GetPacketStatus(int8_t* const RssiSync, int8_t* const Snr)
    {
        int16_t rssi;

        if (gconfig->modeIsLora()) {
            LR11xxDriverBase::GetPacketStatus(&rssi, Snr);
        } else {
            // FSK reserve
        }

        if (rssi > -1) rssi = -1; // we do not support values larger than this
        if (rssi < -127) rssi = -127; // we do not support values lower than this

        *RssiSync = rssi;
    }

    void HandleAFC(void) {} // ???

    //-- RF power interface

    virtual void RfPowerCalc(int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm) = 0;

    //-- helper

    void config_calc(void)
    {
        int8_t power_dbm = gconfig->Power_dbm;
        RfPowerCalc(power_dbm, &sx_power, &actual_power_dbm);

        if (gconfig->modeIsLora()) {
            uint8_t index = gconfig->LoraConfigIndex;
            if (index >= sizeof(Lr11xxxLoraConfiguration)/sizeof(Lr11xxxLoraConfiguration[0])) while(1){} // must not happen
            lora_configuration = &(Lr11xxxLoraConfiguration[index]);
        } else {
            // FSK reserve
        }
    }

    uint32_t TimeOverAir_us(void)
    {
        if (lora_configuration == nullptr) config_calc(); // ensure it is set

        return (gconfig->modeIsLora()) ? lora_configuration->TimeOverAir : while(1){};  // play it safe for the moment
    }

    int16_t ReceiverSensitivity_dbm(void)
    {
        if (lora_configuration == nullptr) config_calc(); // ensure it is set

        return (gconfig->modeIsLora()) ? lora_configuration->ReceiverSensitivity : : while(1){};  // play it safe for the moment
    }

    int8_t RfPower_dbm(void)
    {
        if (lora_configuration == nullptr) config_calc(); // ensure it is set

        return actual_power_dbm;
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
