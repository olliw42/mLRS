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
    // 900 MHz, Lora
    { .SpreadingFactor = LR20XX_LORA_SF5, // 900 MHz, 31 Hz
      .Bandwidth = LR20XX_LORA_BW_500,
      .CodingRate = LR20XX_LORA_CR_4_5,
      .PreambleLength = 12,
      .HeaderType = LR20XX_LORA_HEADER_DISABLE,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CrcEnabled = LR20XX_LORA_CRC_DISABLE,
      .InvertIQ = LR20XX_LORA_IQ_NORMAL,
      .TimeOverAir = 13200,
      .ReceiverSensitivity = -111, // ??
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
      .ReceiverSensitivity = -112, // ??
    },
    // 2.4 GHz, Lora
    { .SpreadingFactor = LR20XX_LORA_SF5, // 2.4 GHz, 50 Hz
      .Bandwidth = LR20XX_LORA_BW_812,
      .CodingRate = LR20XX_LORA_CR_LI_4_5,
      .PreambleLength = 12,
      .HeaderType = LR20XX_LORA_HEADER_DISABLE,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CrcEnabled = LR20XX_LORA_CRC_DISABLE,
      .InvertIQ = LR20XX_LORA_IQ_NORMAL,
      .TimeOverAir = 7892,
      .ReceiverSensitivity = -105, // ??
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
      .ReceiverSensitivity = -108, // ??
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
      .ReceiverSensitivity = -112, // ??
    }
};

const tSxFskConfiguration Lr20xxFskConfiguration[] = {
    // 900 MHz, FSK 50 Hz
    { .BitRate_bps = 100000,
      .PulseShape = LR20XX_FSK_PULSESHAPE_BT_1,
      .Bandwidth = LR20XX_FSK_BW_307700, // that's the closest to GFSK_BW_312000 used by sx126x
      .Fdev_hz = 50000,
      .PreambleLength = 16,
      .PreambleDetectorLength = LR20XX_FSK_PREAMBLE_DETECTOR_LENGTH_8_BITS,
      .SyncWordLength = 16,
      .AddrComp = LR20XX_FSK_ADDR_COMP_DISABLE,
      .PacketType = LR20XX_FSK_PKT_FORMAT_FIX_LEN,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CRCType = LR20XX_FSK_CRC_OFF,
      .Whitening = LR20XX_FSK_WHITENING_ENABLE,
      .TimeOverAir = 7600,
      .ReceiverSensitivity = -106, // ??
    }
};


#ifdef POWER_USE_DEFAULT_RFPOWER_CALC
void lr20xx_rfpower_calc_default(const int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm, const int8_t gain_dbm, const uint8_t frequency_band)
{
    int16_t power_sx = ((int16_t)power_dbm - gain_dbm) * 2;

    if (frequency_band == SX_FHSS_FREQUENCY_BAND_2P4_GHZ) {
        if (power_sx < LR20XX_POWER_HF_MIN) power_sx = LR20XX_POWER_HF_MIN;
        if (power_sx > LR20XX_POWER_HF_MAX) power_sx = LR20XX_POWER_HF_MAX;
    } else {
        if (power_sx < LR20XX_POWER_LF_MIN) power_sx = LR20XX_POWER_LF_MIN;
        if (power_sx > LR20XX_POWER_LF_MAX) power_sx = LR20XX_POWER_LF_MAX;
    }

    *sx_power = power_sx;
    *actual_power_dbm = power_sx / 2 + gain_dbm;
}
#endif


// map the irq bits
typedef enum {
    SX_IRQ_TX_DONE = LR20XX_IRQ_TX_DONE,
    SX_IRQ_RX_DONE = LR20XX_IRQ_RX_DONE,
    SX_IRQ_TIMEOUT = LR20XX_IRQ_TIMEOUT,
    SX_IRQ_ALL     = LR20XX_IRQ_ALL,
} SX_IRQ_ENUM;


class Lr20xxDriverCommon : public Lr20xxDriverBase
{
  public:

    void Init(void)
    {
        gconfig = nullptr;
        lora_configuration = nullptr;
        fsk_configuration = nullptr;
    }

    bool isOk(void)
    {
        uint8_t fwMajor;
        uint8_t fwMinor;
        
        GetVersion(&fwMajor, &fwMinor);

        if (GetLastStatusCmd() != LR20XX_STATUS_CMD_DAT) return false; // CMD_OK && CMD_DAT are ok

        return true;
    }

    void SetLoraConfiguration(const tSxLoraConfiguration* const config)
    {
        SetLoraModulationParams(
            config->SpreadingFactor,
            config->Bandwidth,
            config->CodingRate,
            LR20XX_LORA_LDRO_OFF); // recommended setting for BW500

        if (Config.Mode == MODE_19HZ_7X) { EnableSx127xCompatibility(); }

        SetLoraPacketParams(
            config->PreambleLength,
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

        SetStandby(LR20XX_STANDBY_MODE_RC); // LR20XX_STANDBY_MODE_XOSC ?
        SetPacketType(LR20XX_PACKET_TYPE_LORA);
        SetLoraConfigurationByIndex(gconfig->LoraConfigIndex);
    }

    void SetFskConfiguration(const tSxFskConfiguration* const config, uint16_t sync_word)
    {
        SetModulationParamsFSK(
            config->BitRate_bps | LR20XX_FSK_BITRATE_DIRECT,
            config->PulseShape,
            config->Bandwidth,
            config->Fdev_hz);

        SetPacketParamsFSK(
            config->PreambleLength,
            config->PreambleDetectorLength,
            LR20XX_FSK_LONG_PREAMBLE_MODE_OFF,
            LR20XX_FSK_PREAMBLE_LEN_UNIT_BYTES,
            config->AddrComp,
            config->PacketType,
            config->PayloadLength,
            config->CRCType,
            config->Whitening);

        SetWhiteningParamsFSK(LR20XX_FSK_WHITEN_TYPE_SX126x_SX127x, 0x0100); // LR1121 data sheet says seed default is 0x0100

        SetSyncWordFSK(sync_word, LR20XX_FSK_SYNC_BITORDER_MSB, config->SyncWordLength); // MSB for sx126x compatibility
    }

    void SetFskConfigurationByIndex(uint8_t index, uint16_t sync_word)
    {
        if (index >= sizeof(Lr20xxFskConfiguration)/sizeof(Lr20xxFskConfiguration[0])) while(1){} // must not happen

        fsk_configuration = &(Lr20xxFskConfiguration[index]);
        SetFskConfiguration(fsk_configuration, sync_word);
    }

    void SetRfPower_dbm(int8_t power_dbm)
    {
        if (!gconfig) return;

        _rfpower_calc(power_dbm, &sx_power, &actual_power_dbm);

        switch (gconfig->FrequencyBand) {
        case SX_FHSS_FREQUENCY_BAND_2P4_GHZ:
            SetPaConfig2p4Ghz(sx_power);
            break;
        case SX_FHSS_FREQUENCY_BAND_433_MHZ:
            SetPaConfig433Mhz(sx_power);
            break;
        default:
            SetPaConfig915Mhz(sx_power); // SelPa(LR20XX_PA_SEL_LF); // seems not to be needed
        }

        SetTxParams(sx_power, LR20XX_RAMPTIME_48_US); // closest to 40 us used by SX126x / SX127x
    }

    void UpdateRfPower(tSxGlobalConfig* const global_config)
    {
        if (!gconfig) return;

        gconfig->Power_dbm = global_config->Power_dbm;
        SetRfPower_dbm(gconfig->Power_dbm);
    }

    void Configure(tSxGlobalConfig* const global_config)
    {
        // Order from DATASHEET:
        // SetPacketType, SetModulationParams, SetPacketParams, SetPAConfig, SetTxParams

        Calibrate(LR20XX_CALIBRATE_ALL);

        SetStandby(LR20XX_STANDBY_MODE_RC);
        //SetStandby(LR20XX_STANDBY_MODE_XOSC);

//??        SetTcxoMode(LR20XX_TCXO_SUPPLY_VOLTAGE_2_7, 25000); // set output to 1.6V-3.3V, ask specification of TCXO to maker board
//??        SetRegMode(LR20XX_SIMO_USAGE_NORMAL); // Attention: requires DCDC workaround

        SetRxTxFallbackMode(LR20XX_RX_TX_FALLBACK_MODE_FS);
        SetDefaultRxTxTimeout(0, 0);

        gconfig = global_config;

        // TODO:
        // - calibrate front end
        // - calibrate rssi values

#if 0
        switch (gconfig->FrequencyBand) {
/*
            case SX_FHSS_FREQUENCY_BAND_915_MHZ_FCC: CalibImage(LR20XX_CAL_IMG_902_MHZ_1, LR20XX_CAL_IMG_902_MHZ_2); break;
            case SX_FHSS_FREQUENCY_BAND_868_MHZ: CalibImage(LR20XX_CAL_IMG_863_MHZ_1, LR20XX_CAL_IMG_863_MHZ_2); break;
            case SX_FHSS_FREQUENCY_BAND_866_MHZ_IN: CalibImage(LR20XX_CAL_IMG_863_MHZ_1, LR20XX_CAL_IMG_863_MHZ_2); break;
            case SX_FHSS_FREQUENCY_BAND_433_MHZ: CalibImage(LR20XX_CAL_IMG_430_MHZ_1, LR20XX_CAL_IMG_430_MHZ_2); break;
            case SX_FHSS_FREQUENCY_BAND_70_CM_HAM: CalibImage(LR20XX_CAL_IMG_430_MHZ_1, LR20XX_CAL_IMG_430_MHZ_2); break;
*/
// TODO ????
        case SX_FHSS_FREQUENCY_BAND_915_MHZ_FCC: CalibFE(LR20XX_CAL_FE_902_MHZ_1, LR20XX_CAL_FE_902_MHZ_2); break;
        case SX_FHSS_FREQUENCY_BAND_868_MHZ: CalibFE(LR20XX_CAL_FE_863_MHZ_1, LR20XX_CAL_FE_863_MHZ_2); break;
        case SX_FHSS_FREQUENCY_BAND_866_MHZ_IN: CalibFE(LR20XX_CAL_FE_863_MHZ_1, LR20XX_CAL_FE_863_MHZ_2); break;
        case SX_FHSS_FREQUENCY_BAND_433_MHZ: CalibFE(LR20XX_CAL_FE_430_MHZ_1, LR20XX_CAL_FE_430_MHZ_2); break;
        case SX_FHSS_FREQUENCY_BAND_70_CM_HAM: CalibFE(LR20XX_CAL_FE_430_MHZ_1, LR20XX_CAL_FE_430_MHZ_2); break;
            case SX_FHSS_FREQUENCY_BAND_2P4_GHZ: break;
            default:
                while(1){} // protection
        }
#endif

        if (gconfig->modeIsLora()) {
            SetPacketType(LR20XX_PACKET_TYPE_LORA);
            SetLoraConfigurationByIndex(gconfig->LoraConfigIndex);
        } else {
            SetPacketType(LR20XX_PACKET_TYPE_FSK);
            SetFskConfigurationByIndex(0, Config.FrameSyncWord);
        }

        if (gconfig->FrequencyBand == SX_FHSS_FREQUENCY_BAND_2P4_GHZ) {
            SetRxPath(LR20XX_RX_PATH_HF, LR20XX_RX_BOOST_4_HF);
        } else {
            SetRxPath(LR20XX_RX_PATH_LF, LR20XX_RX_BOOST_0_LF);
        }

        SetRfPower_dbm(gconfig->Power_dbm);

        SetDioFunction(7, LR20XX_DIO_FUNCTION_IRQ, LR20XX_DIO_SLEEP_PULL_DOWN);
        SetDioIrqConfig(7, LR20XX_IRQ_TX_DONE | LR20XX_IRQ_RX_DONE | LR20XX_IRQ_TIMEOUT);
        ClearIrq(LR20XX_IRQ_ALL);

        SetFs();
    }

    //-- these are the API functions used in the loop

    uint32_t GetAndClearIrqStatus(uint32_t dummy)
    {
        return Lr20xxDriverBase::GetAndClearIrqStatus();
    }

    // Attention. This is nasty.
    // LR2021 uses a rx fifo, so we can't peek into the received data without advancing the fifo.
    // We call ReadBuffer() in the ISR to read few bytes, and later call ReadFrame().
    // So, as a temporary workaround we double buffer. At some point in time we need to find
    // a proper solution.

    void ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len)
    {
        ReadRadioRxFifo(data, len);

        if (rx_buf_pos != 0) while(1){} // must not happen!
        if (len >= sizeof(rx_buf)) while(1){} // to play it safe
        memcpy(rx_buf, data, len);
        rx_buf_pos = len;
    }

    void ReadFrame(uint8_t* const data, uint8_t len)
    {
        if (rx_buf_pos == 0) while(1){} // must not happen!
        if (rx_buf_pos >= len) while(1){} // play it safe
        memcpy(data, rx_buf, rx_buf_pos);
        len -= rx_buf_pos;

        ReadRadioRxFifo(data + rx_buf_pos, len);
        ClearRxFifo();
    }

    void SendFrame(uint8_t* const data, uint8_t len, uint16_t tmo_ms)
    {
        ClearTxFifo();
        WriteRadioTxFifo(data, len);
        ClearIrq(LR20XX_IRQ_ALL);
        // TODO: What is LR20xx's timeout exactly doing?
        SetTx(tmo_ms * 33); // 0 = no timeout. TimeOut period in ms. LR20xx uses 30.52 = 1/32768 us period, so 1 ms needs 33 rc ticks
    }

    void SetToRx(void)
    {
        ClearRxFifo();
        ClearIrq(LR20XX_IRQ_ALL);
        SetRx(0); // 0 = no timeout
        rx_buf_pos = 0;
    }

    void SetToIdle(void)
    {
        SetFs();
        ClearIrq(LR20XX_IRQ_ALL);
    }

    void GetPacketStatus(int8_t* const Rssi, int8_t* const Snr)
    {
        if (!gconfig) { *Rssi = -127; *Snr = 0; return; } // should not happen in practice

        int16_t rssi;

        if (gconfig->modeIsLora()) {
            int16_t rssi_signal;
            GetLoraPacketStatus(&rssi, &rssi_signal, Snr);
            rssi = -(rssi / 2);
        } else {
            int16_t rssi_sync;
            GetPacketStatusFSK(&rssi, &rssi_sync, Snr);
            rssi = -(rssi / 2);
        }

        if (rssi > -1) rssi = -1; // we do not support values larger than this
        if (rssi < -127) rssi = -127; // we do not support values lower than this

        *Rssi = rssi;
    }

    void HandleAFC(void) {}

    //-- RF power interface

    virtual void _rfpower_calc(int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm) = 0;

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
            fsk_configuration = &(Lr20xxFskConfiguration[0]);
        }
    }

    uint32_t TimeOverAir_us(void)
    {
        if (!gconfig) return 0; // should not happen in practice

        if (lora_configuration == nullptr && fsk_configuration == nullptr) _config_calc(); // ensure it is set

        return (gconfig->modeIsLora()) ? lora_configuration->TimeOverAir : fsk_configuration->TimeOverAir;
    }

    int16_t ReceiverSensitivity_dbm(void)
    {
        if (!gconfig) return 0; // should not happen in practice

        if (lora_configuration == nullptr && fsk_configuration == nullptr) _config_calc(); // ensure it is set

        return (gconfig->modeIsLora()) ? lora_configuration->ReceiverSensitivity : fsk_configuration->ReceiverSensitivity;
    }

    int8_t RfPower_dbm(void)
    {
        if (!gconfig) return 0; // should not happen in practice

        if (lora_configuration == nullptr && fsk_configuration == nullptr) _config_calc(); // ensure it is set

        return actual_power_dbm;
    }

  protected:
    tSxGlobalConfig* gconfig;

  private:
    const tSxLoraConfiguration* lora_configuration;
    const tSxFskConfiguration* fsk_configuration;
    int8_t sx_power;
    int8_t actual_power_dbm;

    // for temporary workaround
    uint8_t rx_buf_pos;
    uint8_t rx_buf[16];
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

    void _rfpower_calc(int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm) override
    {
#ifdef POWER_USE_DEFAULT_RFPOWER_CALC
        lr20xx_rfpower_calc_default(power_dbm, sx_power, actual_power_dbm, POWER_GAIN_DBM, gconfig->FrequencyBand);
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
