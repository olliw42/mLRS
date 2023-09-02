//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// SX126X Driver
//*******************************************************
// contributed by jinchuuriki
//*******************************************************
#ifndef SX126X_DRIVER_H
#define SX126X_DRIVER_H
#pragma once


//-------------------------------------------------------
// SX Driver
//-------------------------------------------------------

const tSxLoraConfiguration Sx126xLoraConfiguration[] = {
    { .SpreadingFactor = SX126X_LORA_SF5,
      .Bandwidth = SX126X_LORA_BW_500,
      .CodingRate = SX126X_LORA_CR_4_5,
      .PreambleLength = 12,
      .HeaderType = SX126X_LORA_HEADER_DISABLE,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CrcEnabled = SX126X_LORA_CRC_DISABLE,
      .InvertIQ = SX126X_LORA_IQ_NORMAL,
      .TimeOverAir = 13200,
      .ReceiverSensitivity = -111,
    },
    { .SpreadingFactor = SX126X_LORA_SF6,
      .Bandwidth = SX126X_LORA_BW_500,
      .CodingRate = SX126X_LORA_CR_4_5,
      .PreambleLength = 12,
      .HeaderType = SX126X_LORA_HEADER_DISABLE,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CrcEnabled = SX126X_LORA_CRC_DISABLE,
      .InvertIQ = SX126X_LORA_IQ_NORMAL,
      .TimeOverAir = 22560,
      .ReceiverSensitivity = -112, // Q: SF5 with CR4/8 would be -111 dBm, 20.1 ms, better option??
    }
};


typedef enum {
    SX12xx_OSCILLATOR_CONFIG_TXCO_1P6_V = SX126X_DIO3_OUTPUT_1_6,
    SX12xx_OSCILLATOR_CONFIG_TXCO_1P7_V = SX126X_DIO3_OUTPUT_1_7,
    SX12xx_OSCILLATOR_CONFIG_TXCO_1P8_V = SX126X_DIO3_OUTPUT_1_8,
    SX12xx_OSCILLATOR_CONFIG_TXCO_2P2_V = SX126X_DIO3_OUTPUT_2_2,
    SX12xx_OSCILLATOR_CONFIG_TXCO_2P4_V = SX126X_DIO3_OUTPUT_2_4,
    SX12xx_OSCILLATOR_CONFIG_TXCO_2P7_V = SX126X_DIO3_OUTPUT_2_7,
    SX12xx_OSCILLATOR_CONFIG_TXCO_3P0_V = SX126X_DIO3_OUTPUT_3_0,
    SX12xx_OSCILLATOR_CONFIG_TXCO_3P3_V = SX126X_DIO3_OUTPUT_3_3,
    SX12xx_OSCILLATOR_CONFIG_CRYSTAL = UINT8_MAX,
} SX12xx_OSCILLATOR_CONFIG_ENUM;


#ifdef POWER_USE_DEFAULT_RFPOWER_CALC
void sx126x_rfpower_calc(const int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm, const uint8_t GAIN_DBM, const uint8_t SX126X_MAX_DBM)
{
    int16_t power_sx = (int16_t)power_dbm - GAIN_DBM;

    if (power_sx < SX126X_POWER_MIN) power_sx = SX126X_POWER_MIN;
    if (power_sx > SX126X_POWER_MAX) power_sx = SX126X_POWER_MAX;
    if (power_sx > SX126X_MAX_DBM) power_sx = SX126X_MAX_DBM;

    *sx_power = power_sx;
    *actual_power_dbm = power_sx + GAIN_DBM;
}
#endif


class Sx126xDriverCommon : public Sx126xDriverBase
{
  public:

    void Init(void)
    {
        osc_configuration = SX12xx_OSCILLATOR_CONFIG_TXCO_1P8_V;
        lora_configuration = nullptr;
    }

    //-- high level API functions

    bool isOk(void)
    {
        uint16_t firmwareRev = GetFirmwareRev();
        return ((firmwareRev != 0) && (firmwareRev != 65535));
    }

    void SetLoraConfiguration(const tSxLoraConfiguration* config)
    {
        // must come in this order, datasheet 14.5 Issuing Commands in the Right Order, p.103
        SetModulationParams(config->SpreadingFactor,
                            config->Bandwidth,
                            config->CodingRate);

        SetPacketParams(config->PreambleLength,
                        config->HeaderType,
                        config->PayloadLength,
                        config->CrcEnabled,
                        config->InvertIQ);

        // set LoRaSymbNumTimeout for false detection of preamble
        SetSymbNumTimeout((config->PreambleLength * 3) >> 2);
    }

    void SetLoraConfigurationByIndex(uint8_t index)
    {
        if (index >= sizeof(Sx126xLoraConfiguration)/sizeof(Sx126xLoraConfiguration[0])) while (1) {} // must not happen

        lora_configuration = &(Sx126xLoraConfiguration[index]);
        SetLoraConfiguration(lora_configuration);
    }

    void ResetToLoraConfiguration(void)
    {
        SetLoraConfigurationByIndex(gconfig->LoraConfigIndex);
    }

    void SetRfPower_dbm(int8_t power_dbm)
    {
        RfPowerCalc(power_dbm, &sx_power, &actual_power_dbm);
        SetTxParams(sx_power, SX126X_RAMPTIME_10_US);
    }

    void Configure(tSxGlobalConfig* global_config)
    {
        gconfig = global_config;

        SetPacketType(SX126X_PACKET_TYPE_LORA);

        // WORKAROUND: Better Resistance of the SX1262 Tx to Antenna Mismatch,
        // fixes overly eager PA clamping
        // see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.2 for details
        uint8_t data = ReadRegister(SX126X_REG_TX_CLAMP_CONFIG);
        data |= 0x1E;
        WriteRegister(SX126X_REG_TX_CLAMP_CONFIG, data);

        ClearDeviceError(); // XOSC_START_ERR is raised, datasheet 13.3.6 SetDIO3AsTCXOCtrl, p.84

        // set DIO3 as TCXO control
        if (osc_configuration != SX12xx_OSCILLATOR_CONFIG_CRYSTAL) {
            SetDio3AsTcxoControl(osc_configuration, 250); // set output to 1.6V-3.3V, ask specification of TCXO to maker board
        }

        // Image calibration per datasheet 9.2.1
        // default for SX1261/2 (E22/E77-900) is 902 to 928 MHz
        // default for SX1268 (E22/E77-400) is 470 to 510 MHz
        switch (Config.FrequencyBand) {
            case SETUP_FREQUENCY_BAND_915_MHZ_FCC: CalibrateImage(SX126X_CAL_IMG_902_MHZ_1, SX126X_CAL_IMG_902_MHZ_2); break;
            case SETUP_FREQUENCY_BAND_868_MHZ: CalibrateImage(SX126X_CAL_IMG_863_MHZ_1, SX126X_CAL_IMG_863_MHZ_2); break;
            case SETUP_FREQUENCY_BAND_866_MHZ_IN: CalibrateImage(SX126X_CAL_IMG_863_MHZ_1, SX126X_CAL_IMG_863_MHZ_2); break;
            case SETUP_FREQUENCY_BAND_433_MHZ: CalibrateImage(SX126X_CAL_IMG_430_MHZ_1, SX126X_CAL_IMG_430_MHZ_2); break;
            case SETUP_FREQUENCY_BAND_70_CM_HAM: CalibrateImage(SX126X_CAL_IMG_430_MHZ_1, SX126X_CAL_IMG_430_MHZ_2); break;
            default:
                while (1) {}  // protection
        }

        // set DIO2 as RF control switching
        // check the RF module if antenna switching is internally connected to DIO2
        // E22-900M rf switching controlled by IO pin of MCU
        // SetDio2AsRfSwitchControl(SX126X_DIO2_AS_RF_SWITCH);

        SetAutoFs(true); // SetRxTxFallbackMode to FS

        SetRxGain(SX126X_RX_GAIN_BOOSTED_GAIN);
        SetOverCurrentProtection(SX126X_OCP_CONFIGURATION_140_MA); // default for SX1262 according to data sheet, but can't hurt

        SetPaConfig_22dbm();

        //SetTxParams(calc_sx_power(Config.Power), SX126X_RAMPTIME_10_US);
        SetRfPower_dbm(gconfig->Power_dbm);

        SetLoraConfigurationByIndex(gconfig->LoraConfigIndex);

        // SetSyncWord(0x1424); // public network, no need to call as it defaults to 0x1424

        SetBufferBaseAddress(0, 0);

        SetDioIrqParams(SX126X_IRQ_ALL,
                        SX126X_IRQ_RX_DONE|SX126X_IRQ_TX_DONE|SX126X_IRQ_RX_TX_TIMEOUT,
                        SX126X_IRQ_NONE,
                        SX126X_IRQ_NONE);
        ClearIrqStatus(SX126X_IRQ_ALL);

        SetFs();
    }

    //-- this are the API functions used in the loop

    void ReadFrame(uint8_t* data, uint8_t len)
    {
        uint8_t rxStartBufferPointer;
        uint8_t rxPayloadLength;

        GetRxBufferStatus(&rxPayloadLength, &rxStartBufferPointer);
        ReadBuffer(rxStartBufferPointer, data, len);
    }

    void SendFrame(uint8_t* data, uint8_t len, uint16_t tmo_ms)
    {
        WriteBuffer(0, data, len);
        ClearIrqStatus(SX126X_IRQ_ALL);
        SetTx(tmo_ms * 64); // 0 = no timeout. TimeOut period inn ms. sx1262 have static 15p625 period base, so for 1 ms needs 64 tmo value
    }

    void SetToRx(uint16_t tmo_ms)
    {
        ClearIrqStatus(SX126X_IRQ_ALL);
        SetRx(tmo_ms * 64); // 0 = no timeout
    }

    void SetToIdle(void)
    {
        SetFs();
        ClearIrqStatus(SX126X_IRQ_ALL);
    }

    void GetPacketStatus(int8_t* RssiSync, int8_t* Snr)
    {
        int16_t rssi;
        Sx126xDriverBase::GetPacketStatus(&rssi, Snr);

        if (rssi > -1) rssi = -1; // we do not support values larger than this
        if (rssi < -127) rssi = -127; // we do not support values lower than this

        *RssiSync = rssi;
    }

    void HandleAFC(void) {}

    //-- RF power interface

    virtual void RfPowerCalc(int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm) = 0;

    //-- helper

    void config_calc(void)
    {
        int8_t power_dbm = gconfig->Power_dbm;
        RfPowerCalc(power_dbm, &sx_power, &actual_power_dbm);

        uint8_t index = gconfig->LoraConfigIndex;
        if (index >= sizeof(Sx126xLoraConfiguration)/sizeof(Sx126xLoraConfiguration[0])) while (1) {} // must not happen
        lora_configuration = &(Sx126xLoraConfiguration[index]);
    }

    // cumbersome to calculate in general, so use hardcoded for a specific settings
    uint32_t TimeOverAir_us(void)
    {
        if (lora_configuration == nullptr) config_calc(); // ensure it is set

        return lora_configuration->TimeOverAir;
    }

    int16_t ReceiverSensitivity_dbm(void)
    {
        if (lora_configuration == nullptr) config_calc(); // ensure it is set

        return lora_configuration->ReceiverSensitivity;
    }

    int8_t RfPower_dbm(void)
    {
        if (lora_configuration == nullptr) config_calc(); // ensure it is set

        return actual_power_dbm;
    }

  protected:
    uint8_t osc_configuration; // "hidden" variable, TXCO 1.8 V per default, allow child access

  private:
    const tSxLoraConfiguration* lora_configuration;
    tSxGlobalConfig* gconfig;
    uint8_t sx_power;
    int8_t actual_power_dbm;
};


//-------------------------------------------------------
// Driver for SX1
//-------------------------------------------------------

// map the irq bits
typedef enum {
    SX_IRQ_TX_DONE = SX126X_IRQ_TX_DONE,
    SX_IRQ_RX_DONE = SX126X_IRQ_RX_DONE,
    SX_IRQ_TIMEOUT = SX126X_IRQ_RX_TX_TIMEOUT,
    SX_IRQ_ALL     = SX126X_IRQ_ALL,
} SX_IRQ_ENUM;


class Sx126xDriver : public Sx126xDriverCommon
{
  public:

#ifdef SX_BUSY
    void WaitOnBusy(void) override
    {
        while (sx_busy_read()) { __NOP(); };
    }
#else
    uint32_t timer_us_tmo = 0;
    uint32_t timer_us_start_tick = 0;

    void WaitOnBusy(void) override
    {
        if (timer_us_tmo) {
            while ((DWT->CYCCNT - timer_us_start_tick) < timer_us_tmo) { __NOP(); };
            timer_us_tmo = 0;
        }
    }

    void SetDelay(uint16_t tmo_us) override // TODO: ?? has not yet been implemented, is it needed? should/can we require BUSY ?
    {
        timer_us_tmo = (uint32_t)tmo_us * (SystemCoreClock/1000000);
        timer_us_start_tick = DWT->CYCCNT;
    }
#endif

    void SpiSelect(void) override
    {
#ifndef SX_BUSY
        delay_ns(150); // datasheet says t9 >= 150 ns, NSS high time
#endif
        spi_select();
        delay_ns(50); // datasheet says t6 = 15 ns, NSS falling to MISO delay, t1 = 32 ns, NSS falling edge to SCK setup time
    }

    void SpiDeselect(void) override
    {
        delay_ns(50); // datasheet says t8 = 31.25 ns, SCK to NSS rising edge hold time
        spi_deselect();
#ifndef SX_BUSY
        delay_ns(100); // well...
#endif
    }

    void SpiTransferByte(uint8_t* byteout, uint8_t* bytein) override
    {
        *bytein = spi_transmitchar(*byteout);
    }

    //-- RF power interface

    void RfPowerCalc(int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm) override
    {
        sx126x_rfpower_calc(power_dbm, sx_power, actual_power_dbm, POWER_GAIN_DBM, POWER_SX126X_MAX_DBM);
    }

    //-- init API functions

    void _reset(void)
    {
#ifdef SX_RESET
        gpio_low(SX_RESET);
        delay_ms(5); // datasheet says > 100 us
        gpio_high(SX_RESET);
        delay_ms(50);
        WaitOnBusy();
#else
        sx_reset();
#endif
    }

    void Init(void)
    {
        Sx126xDriverCommon::Init();
#ifdef SX_USE_CRYSTALOSCILLATOR
        osc_configuration = SX12xx_OSCILLATOR_CONFIG_CRYSTAL;
#endif

        spi_init();
        sx_init_gpio();
        sx_dio_exti_isr_clearflag();
        sx_dio_init_exti_isroff();

        // no idea how long the SX126x takes to boot up, so give it some good time
        // we could probably speed up by using WaitOnBusy()
        delay_ms(300);
        _reset(); // this is super crucial ! was so for SX1280, is it also for the SX1262 ??

        SetStandby(SX126X_STDBY_CONFIG_STDBY_RC); // should be in STDBY_RC after reset
        delay_us(1000); // is this needed ????
    }

    //-- high level API functions

    void StartUp(tSxGlobalConfig* global_config)
    {
#ifdef SX_USE_DCDC // here ??? ELRS does it as last !!!
        SetRegulatorMode(SX126X_REGULATOR_MODE_DCDC);
#endif

        Configure(global_config);
        delay_us(125); // may not be needed if busy available

        sx_dio_enable_exti_isr();
    }

    //-- this are the API functions used in the loop

    void SendFrame(uint8_t* data, uint8_t len, uint16_t tmo_ms = 0)
    {
        sx_amp_transmit();
        Sx126xDriverCommon::SendFrame(data, len, tmo_ms);
        delay_us(125); // may not be needed if busy available
    }

    void SetToRx(uint16_t tmo_ms = 0)
    {
        sx_amp_receive();
        Sx126xDriverCommon::SetToRx(tmo_ms);
        delay_us(125); // may not be needed if busy available
    }
};


//-------------------------------------------------------
// Driver for SX2
//-------------------------------------------------------
#ifdef DEVICE_HAS_DIVERSITY

#ifndef SX2_BUSY
    #error SX2 must have a BUSY pin !!
#endif
#ifndef SX2_RESET
    #error SX2 must have a RESET pin !!
#endif

// map the irq bits
typedef enum {
    SX2_IRQ_TX_DONE = SX126X_IRQ_TX_DONE,
    SX2_IRQ_RX_DONE = SX126X_IRQ_RX_DONE,
    SX2_IRQ_TIMEOUT = SX126X_IRQ_RX_TX_TIMEOUT,
    SX2_IRQ_ALL     = SX126X_IRQ_ALL,
} SX2_IRQ_ENUM;


class Sx126xDriver2 : public Sx126xDriverCommon
{
  public:

    void WaitOnBusy(void) override
    {
        while (sx2_busy_read()) { __NOP(); };
    }

    void SpiSelect(void) override
    {
        spib_select();
        delay_ns(50); // datasheet says t6 = 15 ns, NSS falling to MISO delay, t1 = 32 ns, NSS falling edge to SCK setup time
    }

    void SpiDeselect(void) override
    {
        delay_ns(50); // datasheet says t8 = 31.25 ns, SCK to NSS rising edge hold time
        spib_deselect();
    }

    void SpiTransferByte(uint8_t* byteout, uint8_t* bytein) override
    {
        *bytein = spib_transmitchar(*byteout);
    }

    //-- RF power interface

    void RfPowerCalc(int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm) override
    {
        sx126x_rfpower_calc(power_dbm, sx_power, actual_power_dbm, POWER_GAIN_DBM, POWER_SX126X_MAX_DBM);
    }

    //-- init API functions

    void _reset(void)
    {
        gpio_low(SX2_RESET);
        delay_ms(5); // datasheet says > 100 us
        gpio_high(SX2_RESET);
        delay_ms(50);
        WaitOnBusy();
    }

    void Init(void)
    {
        Sx126xDriverCommon::Init();
#ifdef SX2_USE_CRYSTALOSCILLATOR
        osc_configuration = SX12xx_OSCILLATOR_CONFIG_CRYSTAL;
#endif

        spib_init();
        sx2_init_gpio();
        sx2_dio_init_exti_isroff();
        sx2_dio_exti_isr_clearflag();

        // no idea how long the SX126x takes to boot up, so give it some good time
        // we could probably speed up by using WaitOnBusy()
        delay_ms(300);
        _reset(); // this is super crucial ! was so for SX1280, is it also for the SX1262 ??

        SetStandby(SX126X_STDBY_CONFIG_STDBY_RC); // should be in STDBY_RC after reset
        delay_us(1000); // is this needed ????
    }

    //-- high level API functions

    void StartUp(tSxGlobalConfig* global_config)
    {
#ifdef SX2_USE_DCDC // here ??? ELRS does it as last !!!
        SetRegulatorMode(SX126X_REGULATOR_MODE_DCDC);
#endif

        Configure(global_config);
        delay_us(125); // may not be needed if busy available

        sx2_dio_enable_exti_isr();
    }

    //-- this are the API functions used in the loop

    void SendFrame(uint8_t* data, uint8_t len, uint16_t tmo_ms = 0)
    {
        sx2_amp_transmit();
        Sx126xDriverCommon::SendFrame(data, len, tmo_ms);
        delay_us(125); // may not be needed if busy available
    }

    void SetToRx(uint16_t tmo_ms = 0)
    {
        sx2_amp_receive();
        Sx126xDriverCommon::SetToRx(tmo_ms);
        delay_us(125); // may not be needed if busy available
    }
};

#endif


#endif // SX126X_DRIVER_H
