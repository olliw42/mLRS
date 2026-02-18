//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// LR20XX Driver
//*******************************************************
// Configuration defines:
// #define POWER_USE_DEFAULT_RFPOWER_CALC
//*******************************************************

#ifndef LR20XX_DRIVER_H
#define LR20XX_DRIVER_H
#pragma once


//-------------------------------------------------------
// SX Driver
//-------------------------------------------------------

// the first two are for 900 MHz and the last three are for 2.4 GHz
const tSxLoraConfiguration Lr20xxLoraConfiguration[] = {
    { .SpreadingFactor = LR20XX_LORA_SF5, // 900 MHz, 31 Hz
      .Bandwidth = LR20XX_LORA_BW_500,
      .CodingRate = LR20XX_LORA_CR_4_5,
      .PreambleLength = 12,
      .HeaderType = LR20XX_LORA_HEADER_DISABLE,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CrcEnabled = LR20XX_LORA_CRC_DISABLE,
      .InvertIQ = LR20XX_LORA_IQ_NORMAL,
      .TimeOverAir = 13200,
      .ReceiverSensitivity = -111,
    },
    { .SpreadingFactor = LR20XX_LORA_SF6,// 900 MHz, 19 Hz
      .Bandwidth = LR20XX_LORA_BW_500,
      .CodingRate = LR20XX_LORA_CR_4_5,
      .PreambleLength = 12,
      .HeaderType = LR20XX_LORA_HEADER_DISABLE,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CrcEnabled = LR20XX_LORA_CRC_DISABLE,
      .InvertIQ = LR20XX_LORA_IQ_NORMAL,
      .TimeOverAir = 22560,
      .ReceiverSensitivity = -112,
    },
    { .SpreadingFactor = LR20XX_LORA_SF5, // 2.4 GHz, 50 Hz
      .Bandwidth = LR20XX_LORA_BW_812,
      .CodingRate = LR20XX_LORA_CR_LI_4_5,
      .PreambleLength = 12,
      .HeaderType = LR20XX_LORA_HEADER_DISABLE,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CrcEnabled = LR20XX_LORA_CRC_DISABLE,
      .InvertIQ = LR20XX_LORA_IQ_NORMAL,
      .TimeOverAir = 7892,
      .ReceiverSensitivity = -105,
    },
    { .SpreadingFactor = LR20XX_LORA_SF6, // 2.4 GHz, 31 Hz
      .Bandwidth = LR20XX_LORA_BW_812,
      .CodingRate = LR20XX_LORA_CR_LI_4_5,
      .PreambleLength = 12,
      .HeaderType = LR20XX_LORA_HEADER_DISABLE,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CrcEnabled = LR20XX_LORA_CRC_DISABLE,
      .InvertIQ = LR20XX_LORA_IQ_NORMAL,
      .TimeOverAir = 13418,
      .ReceiverSensitivity = -108,
    },
    { .SpreadingFactor = LR20XX_LORA_SF7, // 2.4 GHz, 19 Hz
      .Bandwidth = LR20XX_LORA_BW_812,
      .CodingRate = LR20XX_LORA_CR_LI_4_5,
      .PreambleLength = 12,
      .HeaderType = LR20XX_LORA_HEADER_DISABLE,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CrcEnabled = LR20XX_LORA_CRC_DISABLE,
      .InvertIQ = LR20XX_LORA_IQ_NORMAL,
      .TimeOverAir = 23527,
      .ReceiverSensitivity = -112,
    }
};

#if 0
const tSxGfskConfiguration Lr20xxGfskConfiguration[] = { // 900 MHz, FSK 50 Hz
    { .br_bps = 100000,
      .PulseShape = LR20XX_GFSK_PULSESHAPE_BT_1,
      .Bandwidth = LR20XX_GFSK_BW_312000,
      .Fdev_hz = 50000,
      .PreambleLength = 16,
      .PreambleDetectorLength = LR20XX_GFSK_PREAMBLE_DETECTOR_LENGTH_8BITS,
      .SyncWordLength = 16,
      .AddrComp = LR20XX_GFSK_ADDRESS_FILTERING_DISABLE,
      .PacketType = LR20XX_GFSK_PKT_FIX_LEN,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CRCType = LR20XX_GFSK_CRC_OFF,
      .Whitening = LR20XX_GFSK_WHITENING_ENABLE,
      .TimeOverAir = 7600,
      .ReceiverSensitivity = -106  // This is a guess, data sheet is vague here
    }
};
#endif


#ifdef POWER_USE_DEFAULT_RFPOWER_CALC
void lr20xx_rfpower_calc_default(const int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm, const int8_t GAIN_DBM, const uint8_t LR20XX_MAX_DBM)
{
    int16_t power_sx = (int16_t)power_dbm - GAIN_DBM;

    if (power_sx < LR20XX_POWER_LF_MIN) power_sx = LR20XX_POWER_LF_MIN;
    if (power_sx > LR20XX_POWER_LF_MAX) power_sx = LR20XX_POWER_LF_MAX;
    if (power_sx > LR20XX_MAX_DBM) power_sx = LR20XX_MAX_DBM;

    *sx_power = power_sx;
    *actual_power_dbm = power_sx + GAIN_DBM;
}
#endif


class Lr20xxDriverCommon : public Lr20xxDriverBase
{
  public:

    void Init(void)
    {
        gconfig = nullptr;
        lora_configuration = nullptr;
        gfsk_configuration = nullptr;
    }

    bool isOk(void) // TODO ???
    {
        uint8_t fwMajor;
        uint8_t fwMinor;
        
        GetVersion(&fwMajor, &fwMinor);

        return true;
    }

    void SetLoraConfiguration(const tSxLoraConfiguration* const config)
    {
        SetLoraModulationParams(config->SpreadingFactor,
                                config->Bandwidth,
                                config->CodingRate,
                                LR20XX_LORA_LDRO_OFF);

// TODO        if (Config.Mode == MODE_19HZ_7X) { EnableSx127xCompatibility(); }

        SetLoraPacketParams(config->PreambleLength,
                            config->HeaderType,
                            config->PayloadLength,
                            config->CrcEnabled,
                            config->InvertIQ);
    }

    void SetLoraConfigurationByIndex(uint8_t index)
    {
        if (index >= sizeof(Lr20xxLoraConfiguration)/sizeof(Lr20xxLoraConfiguration[0])) while(1){} // must not happen

        lora_configuration = &(Lr20xxLoraConfiguration[index]);
        SetLoraConfiguration(lora_configuration);
    }

    void ResetToLoraConfiguration(void)
    {
        if (!gconfig) while(1){} // must not happen

        SetStandby(LR20XX_STANDBY_MODE_RC); // TODO LR20XX_STANDBY_MODE_XOSC ???
        SetPacketType(LR20XX_PACKET_TYPE_LORA);
        SetLoraConfigurationByIndex(gconfig->LoraConfigIndex);
    }

    void SetGfskConfiguration(const tSxGfskConfiguration* const config, uint16_t sync_word)
    {
#if 0
        SetModulationParamsGFSK(config->br_bps,
                                config->PulseShape,
                                config->Bandwidth,
                                config->Fdev_hz);

        SetPacketParamsGFSK(config->PreambleLength,
                            config->PreambleDetectorLength,
                            config->SyncWordLength,
                            config->AddrComp,
                            config->PacketType,
                            config->PayloadLength,
                            config->CRCType,
                            config->Whitening);

        SetSyncWordGFSK(sync_word);
#endif
    }

    void SetGfskConfigurationByIndex(uint8_t index, uint16_t sync_word)
    {
#if 0
        if (index >= sizeof(Lr20xxGfskConfiguration)/sizeof(Lr20xxGfskConfiguration[0])) while(1){} // must not happen

        gfsk_configuration = &(Lr20xxGfskConfiguration[index]);
        SetGfskConfiguration(gfsk_configuration, sync_word);
#endif
    }

    void SetRfPower_dbm(int8_t power_dbm)
    {
        if (!gconfig) return;

        _rfpower_calc(power_dbm, &sx_power, &actual_power_dbm);
        SetTxParams(sx_power, LR20XX_RAMPTIME_48_US); // Closest to 40 uS used by SX126x / SX127x
    }

    void UpdateRfPower(tSxGlobalConfig* const global_config)
    {
        if (!gconfig) return;

        gconfig->Power_dbm = global_config->Power_dbm;
        SetRfPower_dbm(gconfig->Power_dbm);
    }

    void Configure(tSxGlobalConfig* const global_config)
    {
        // Order from User Manual:
        // SetPacketType, SetModulationParams, SetPacketParams, SetPAConfig, SetTxParams

        SetRxTxFallbackMode(LR20XX_RX_TX_FALLBACK_MODE_FS);
        SetRxPath(LR20XX_RX_PATH_LF, LR20XX_RX_BOOST_LF_0); // TODO HF??
#ifndef SX_USE_RFSW_CTRL
//        SetDioAsRfSwitch(15, 0, 4, 8, 8, 2, 0, 1);  // Default ELRS selection
// TODO SetDioRfSwitchConfig ???
#else
        uint8_t RfswCtrl[8] = SX_USE_RFSW_CTRL;
        SetDioAsRfSwitch(RfswCtrl[0], RfswCtrl[1], RfswCtrl[2], RfswCtrl[3], 
                         RfswCtrl[4], RfswCtrl[5], RfswCtrl[6], RfswCtrl[7]);
#endif
//        SetDioIrqParams(LR20XX_IRQ_TX_DONE | LR20XX_IRQ_RX_DONE | LR20XX_IRQ_TIMEOUT, 0);
// TODO  SetDioIrqConfig ????

        gconfig = global_config;

        switch (gconfig->FrequencyBand) {
/*
            case SX_FHSS_FREQUENCY_BAND_915_MHZ_FCC: CalibImage(LR20XX_CAL_IMG_902_MHZ_1, LR20XX_CAL_IMG_902_MHZ_2); break;
            case SX_FHSS_FREQUENCY_BAND_868_MHZ: CalibImage(LR20XX_CAL_IMG_863_MHZ_1, LR20XX_CAL_IMG_863_MHZ_2); break;
            case SX_FHSS_FREQUENCY_BAND_866_MHZ_IN: CalibImage(LR20XX_CAL_IMG_863_MHZ_1, LR20XX_CAL_IMG_863_MHZ_2); break;
            case SX_FHSS_FREQUENCY_BAND_433_MHZ: CalibImage(LR20XX_CAL_IMG_430_MHZ_1, LR20XX_CAL_IMG_430_MHZ_2); break;
            case SX_FHSS_FREQUENCY_BAND_70_CM_HAM: CalibImage(LR20XX_CAL_IMG_430_MHZ_1, LR20XX_CAL_IMG_430_MHZ_2); break;
*/
// TODO ????
#if 0
        case SX_FHSS_FREQUENCY_BAND_915_MHZ_FCC: CalibFE(LR20XX_CAL_FE_902_MHZ_1, LR20XX_CAL_FE_902_MHZ_2); break;
        case SX_FHSS_FREQUENCY_BAND_868_MHZ: CalibFE(LR20XX_CAL_FE_863_MHZ_1, LR20XX_CAL_FE_863_MHZ_2); break;
        case SX_FHSS_FREQUENCY_BAND_866_MHZ_IN: CalibFE(LR20XX_CAL_FE_863_MHZ_1, LR20XX_CAL_FE_863_MHZ_2); break;
        case SX_FHSS_FREQUENCY_BAND_433_MHZ: CalibFE(LR20XX_CAL_FE_430_MHZ_1, LR20XX_CAL_FE_430_MHZ_2); break;
        case SX_FHSS_FREQUENCY_BAND_70_CM_HAM: CalibFE(LR20XX_CAL_FE_430_MHZ_1, LR20XX_CAL_FE_430_MHZ_2); break;
#endif
            case SX_FHSS_FREQUENCY_BAND_2P4_GHZ: break;
            default:
                while(1){} // protection
        }

        SetStandby(LR20XX_STANDBY_MODE_RC);

        if (gconfig->modeIsLora()) {
            SetPacketType(LR20XX_PACKET_TYPE_LORA);
            SetLoraConfigurationByIndex(gconfig->LoraConfigIndex);
        } else {
            SetPacketType(LR20XX_PACKET_TYPE_FSK);
            SetGfskConfigurationByIndex(0, Config.FrameSyncWord);
        }

        if (gconfig->FrequencyBand == SX_FHSS_FREQUENCY_BAND_2P4_GHZ) {
// TODO            SetPaConfig(LR20XX_PA_SEL_HF, LR20XX_PA_MODE_FSM, REG_PA_SUPPLY_INTERNAL, 0, 0);
            SelPa(LR20XX_PA_SEL_HF);
        } else {
#ifndef SX_USE_LP_PA
// TODO            SetPaConfig(LR20XX_PA_SELECT_HP_PA, LR20XX_REG_PA_SUPPLY_VBAT, LR20XX_PA_DUTY_CYCLE_22_DBM, LR20XX_PA_HP_SEL_22_DBM);
#else
            SetPaConfig(LR20XX_PA_SELECT_LP_PA, LR20XX_REG_PA_SUPPLY_INTERNAL, LR20XX_PA_DUTY_CYCLE_14_DBM, 0);
#endif
            SelPa(LR20XX_PA_SEL_LF);
        }

        SetRfPower_dbm(gconfig->Power_dbm);
        ClearIrq(LR20XX_IRQ_ALL);
        SetFs();
    }

    //-- these are the API functions used in the loop

    uint32_t GetAndClearIrqStatus(uint32_t dummy)
    {
        return Lr20xxDriverBase::GetAndClearIrqStatus();
    }

    void ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len)
    {
// TODO
        ReadRadioRxFifo(data, len);
    }

    void ReadFrame(uint8_t* const data, uint8_t len)
    {
// TODO
        ReadRadioRxFifo(data, len);
    }

    void SendFrame(uint8_t* const data, uint8_t len, uint16_t tmo_ms)
    {
// TODO
        ClearTxFifo();
        WriteRadioTxFifo(data, len);
        ClearIrq(LR20XX_IRQ_ALL);
        SetTx(tmo_ms * 33); // 0 = no timeout. TimeOut period in ms. LR11xx have static 30.517 uS (1 / 32768) period base, so for 1 ms needs 33 tmo value
    }

    void SetToRx(void)
    {
        ClearRxFifo();
        ClearIrq(LR20XX_IRQ_ALL);
        SetRx(0); // 0 = no timeout
    }

    void SetToIdle(void)
    {
        SetFs();
        ClearIrq(LR20XX_IRQ_ALL);
    }

    void GetPacketStatus(int8_t* const RssiSync, int8_t* const Snr)
    {
        if (!gconfig) { *RssiSync = -127; *Snr = 0; return; } // should not happen in practice

        int16_t rssi;
        int16_t rssi_signal;

        if (gconfig->modeIsLora()) {
            Lr20xxDriverBase::GetPacketStatus(&rssi, &rssi_signal, Snr);
        } else {
#if 0
            Lr20xxDriverBase::GetPacketStatusFSK(&rssi);
#endif
            *Snr = 0;
        }

        if (rssi > -1) rssi = -1; // we do not support values larger than this
        if (rssi < -127) rssi = -127; // we do not support values lower than this

        *RssiSync = rssi;
    }

    void HandleAFC(void) {} // ???

    //-- RF power interface

    virtual void _rfpower_calc(int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm) = 0;

    //-- helper

    void _config_calc(void)
    {
        int8_t power_dbm = gconfig->Power_dbm;
        _rfpower_calc(power_dbm, &sx_power, &actual_power_dbm);

        if (gconfig->modeIsLora()) {
            uint8_t index = gconfig->LoraConfigIndex;
            if (index >= sizeof(Lr20xxLoraConfiguration)/sizeof(Lr20xxLoraConfiguration[0])) while(1){} // must not happen
            lora_configuration = &(Lr20xxLoraConfiguration[index]);
        } else {
#if 0
            gfsk_configuration = &(Lr20xxGfskConfiguration[0]);
#endif
        }
    }

    uint32_t TimeOverAir_us(void)
    {
        if (!gconfig) return 0; // should not happen in practice

        if (lora_configuration == nullptr && gfsk_configuration == nullptr) _config_calc(); // ensure it is set

        return (gconfig->modeIsLora()) ? lora_configuration->TimeOverAir : gfsk_configuration->TimeOverAir;
    }

    int16_t ReceiverSensitivity_dbm(void)
    {
        if (!gconfig) return 0; // should not happen in practice

        if (lora_configuration == nullptr && gfsk_configuration == nullptr) _config_calc(); // ensure it is set

        return (gconfig->modeIsLora()) ? lora_configuration->ReceiverSensitivity : gfsk_configuration->ReceiverSensitivity;
    }

    int8_t RfPower_dbm(void)
    {
        if (!gconfig) return 0; // should not happen in practice

        if (lora_configuration == nullptr && gfsk_configuration == nullptr) _config_calc(); // ensure it is set

        return actual_power_dbm;
    }

  protected:
    tSxGlobalConfig* gconfig;

  private:
    const tSxLoraConfiguration* lora_configuration;
    const tSxGfskConfiguration* gfsk_configuration;
    uint8_t sx_power;
    int8_t actual_power_dbm;
};


//-------------------------------------------------------
// Driver for SX1
//-------------------------------------------------------

#ifndef SX_BUSY
  #error SX must have a BUSY pin!
#endif
#ifndef SX_RESET
  #error SX must have a RESET pin!
#endif

// map the irq bits
typedef enum {
    SX_IRQ_TX_DONE = LR20XX_IRQ_TX_DONE,
    SX_IRQ_RX_DONE = LR20XX_IRQ_RX_DONE,
    SX_IRQ_TIMEOUT = LR20XX_IRQ_TIMEOUT,
    SX_IRQ_ALL     = LR20XX_IRQ_ALL,
} SX_IRQ_ENUM;


class Lr20xxDriver : public Lr20xxDriverCommon
{
  public:

    //-- interface to SPI peripheral

    void WaitOnBusy(void) override
    {
        while (sx_busy_read()) { __NOP(); };
    }

    void SpiSelect(void) override
    {
        spi_select();
        delay_ns(50); // datasheet says t1 = 25 ns, semtech driver doesn't do it, helps so do it
    }

    void SpiDeselect(void) override
    {
        delay_ns(50); // datasheet says t8 = 25 ns, semtech driver doesn't do it, helps so do it
        spi_deselect();
    }

    void SpiTransfer(uint8_t* dataout, uint8_t* datain, uint8_t len) override
    {
        spi_transfer(dataout, datain, len);
    }

    void SpiRead(uint8_t* datain, uint8_t len) override
    {
        spi_read(datain, len);
    }

    void SpiWrite(uint8_t* dataout, uint8_t len) override
    {
        spi_write(dataout, len);
    }

    //-- RF power interface

    void _rfpower_calc(int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm) override
    {
#ifdef POWER_USE_DEFAULT_RFPOWER_CALC
// TODO        lr20xx_rfpower_calc_default(power_dbm, sx_power, actual_power_dbm, POWER_GAIN_DBM, POWER_LR20XX_MAX_DBM);
#else
        lr20xx_rfpower_calc(power_dbm, sx_power, actual_power_dbm, gconfig->FrequencyBand);
#endif
    }

    //-- init API functions

    void _reset(void)
    {
        gpio_low(SX_RESET);
        delay_ms(1);
        gpio_high(SX_RESET);
        WaitOnBusy();
    }

    void Init(void)
    {
        Lr20xxDriverCommon::Init();

        spi_init();
        spi_setnop(0x00); // 0x00 = NOP
        sx_init_gpio();
        sx_dio_exti_isr_clearflag();
        sx_dio_init_exti_isroff();
       
        _reset(); // this is super crucial !
    }

    //-- high level API functions

    void StartUp(tSxGlobalConfig* const global_config)
    {
#ifdef SX_USE_REGULATOR_MODE_DCDC
        SetRegMode(LR20XX_REGULATOR_MODE_DCDC);
#endif
        Configure(global_config);
        
        sx_dio_enable_exti_isr();
    }

    //-- this are the API functions used in the loop

    void SendFrame(uint8_t* const data, uint8_t len, uint16_t tmo_ms)
    {
        sx_amp_transmit();
        Lr20xxDriverCommon::SendFrame(data, len, tmo_ms);
    }

    void SetToRx(void)
    {
        sx_amp_receive();
        Lr20xxDriverCommon::SetToRx();
    }
};


//-------------------------------------------------------
// Driver for SX2
//-------------------------------------------------------
#if defined DEVICE_HAS_DIVERSITY || defined DEVICE_HAS_DIVERSITY_SINGLE_SPI

#ifndef SX2_BUSY
  #error SX2 must have a BUSY pin!
#endif
#ifndef SX2_RESET
  #error SX2 must have a RESET pin!
#endif


#error SX2 is todo!

#endif

#endif // LR20XX_DRIVER_H
