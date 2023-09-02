//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// SX128x Driver
//*******************************************************
#ifndef SX128X_DRIVER_H
#define SX128X_DRIVER_H
#pragma once


//-------------------------------------------------------
// SX Driver
//-------------------------------------------------------

typedef struct {
    uint8_t SpreadingFactor;
    uint8_t Bandwidth;
    uint8_t CodingRate;
    uint8_t PreambleLength;
    uint8_t HeaderType;
    uint8_t PayloadLength;
    uint8_t CrcEnabled;
    uint8_t InvertIQ;
    uint32_t TimeOverAir; // in us
    int16_t ReceiverSensitivity;
} tSxLoraConfiguration;


typedef struct {
    uint8_t Bandwidth;
    uint8_t CodingRate;
    uint8_t Bt;
    uint8_t AGCPreambleLength;
    uint8_t SyncWordLength;
    uint8_t SyncWordMatch;
    uint8_t PacketType;
    uint8_t PayloadLength;
    uint8_t CrcLength;
    uint16_t CrcSeed;
    uint32_t TimeOverAir; // in us
    int16_t ReceiverSensitivity;
} tSxFlrcConfiguration;


const tSxLoraConfiguration Sx128xLoraConfiguration[] = {
    { .SpreadingFactor = SX1280_LORA_SF5,
      .Bandwidth = SX1280_LORA_BW_800,
      .CodingRate = SX1280_LORA_CR_LI_4_5,
      .PreambleLength = 12,
      .HeaderType = SX1280_LORA_HEADER_DISABLE,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CrcEnabled = SX1280_LORA_CRC_DISABLE,
      .InvertIQ = SX1280_LORA_IQ_NORMAL,
      .TimeOverAir = 7892,
      .ReceiverSensitivity = -105,
    },
    { .SpreadingFactor = SX1280_LORA_SF6,
      .Bandwidth = SX1280_LORA_BW_800,
      .CodingRate = SX1280_LORA_CR_LI_4_5,
      .PreambleLength = 12,
      .HeaderType = SX1280_LORA_HEADER_DISABLE,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CrcEnabled = SX1280_LORA_CRC_DISABLE,
      .InvertIQ = SX1280_LORA_IQ_NORMAL,
      .TimeOverAir = 13418,
      .ReceiverSensitivity = -108,
    },
    { .SpreadingFactor = SX1280_LORA_SF7,
      .Bandwidth = SX1280_LORA_BW_800,
      .CodingRate = SX1280_LORA_CR_LI_4_5,
      .PreambleLength = 12,
      .HeaderType = SX1280_LORA_HEADER_DISABLE,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CrcEnabled = SX1280_LORA_CRC_DISABLE,
      .InvertIQ = SX1280_LORA_IQ_NORMAL,
      .TimeOverAir = 23527,
      .ReceiverSensitivity = -112,
    }
};


const tSxFlrcConfiguration Sx128xFlrcConfiguration[] = {
    { .Bandwidth = SX1280_FLRC_BR_0_650_BW_0_6,
      .CodingRate = SX1280_FLRC_CR_3_4,
      .Bt = SX1280_FLRC_BT_1,
      .AGCPreambleLength = SX1280_FLRC_PREAMBLE_LENGTH_32_BITS,
      .SyncWordLength = SX1280_FLRC_SYNCWORD_LEN_P32S,
      .SyncWordMatch = SX1280_FLRC_SYNCWORD_MATCH_1,
      .PacketType = SX1280_FLRC_PACKET_TYPE_FIXED_LENGTH,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CrcLength = SX1280_FLRC_CRC_DISABLE,
      .CrcSeed = 27368, // CrcSeed is 'j', 'p'. Not used.
      .TimeOverAir = 1631,
      .ReceiverSensitivity = -103,
    }
};


#ifdef POWER_USE_DEFAULT_RFPOWER_CALC
void sx1280_rfpower_calc(const int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm, const uint8_t GAIN_DBM, const uint8_t SX1280_MAX_DBM)
{
    int16_t power_sx = (int16_t)power_dbm - GAIN_DBM + 18;

    if (power_sx < SX1280_POWER_MIN) power_sx = SX1280_POWER_MIN;
    if (power_sx > SX1280_POWER_MAX) power_sx = SX1280_POWER_MAX;
    if (power_sx > SX1280_MAX_DBM) power_sx = SX1280_MAX_DBM;

    *sx_power = power_sx;
    *actual_power_dbm = power_sx + GAIN_DBM - 18;
}
#endif


class Sx128xDriverCommon : public Sx128xDriverBase
{
  public:
    void Init(void)
    {
        lora_configuration = nullptr;
        flrc_configuration = nullptr;
    }

    //-- high level API functions

    bool isOk(void)
    {
        uint16_t firmwareRev = GetFirmwareRev();
        return ((firmwareRev != 0) && (firmwareRev != 65535));
    }

    void SetLoraConfiguration(const tSxLoraConfiguration* config)
    {
        SetModulationParams(config->SpreadingFactor,
                            config->Bandwidth,
                            config->CodingRate);

        SetPacketParams(config->PreambleLength,
                        config->HeaderType,
                        config->PayloadLength,
                        config->CrcEnabled,
                        config->InvertIQ);
    }

    void SetLoraConfigurationByIndex(uint8_t index)
    {
        if (index >= sizeof(Sx128xLoraConfiguration)/sizeof(Sx128xLoraConfiguration[0])) while (1) {} // must not happen

        lora_configuration = &(Sx128xLoraConfiguration[index]);
        SetLoraConfiguration(lora_configuration);
    }

    void ResetToLoraConfiguration(void)
    {
        SetStandby(SX1280_STDBY_CONFIG_STDBY_RC);
        delay_us(1000); // seems ok without, but do it
        SetPacketType(SX1280_PACKET_TYPE_LORA);
        SetLoraConfigurationByIndex(Config.LoraConfigIndex);
    }

    void SetFlrcConfiguration(const tSxFlrcConfiguration* config, uint32_t sync_word)
    {
        SetModulationParamsFLRC(config->Bandwidth,
                                config->CodingRate,
                                config->Bt);

        SetPacketParamsFLRC(config->AGCPreambleLength,
                            config->SyncWordLength,
                            config->SyncWordMatch,
                            config->PacketType,
                            config->PayloadLength,
                            config->CrcLength,
                            config->CrcSeed,
                            sync_word,
                            config->CodingRate);
    }

    void SetFlrcConfigurationByIndex(uint8_t index, uint32_t sync_word)
    {
        if (index >= sizeof(Sx128xFlrcConfiguration)/sizeof(Sx128xFlrcConfiguration[0])) while (1) {} // must not happen

        flrc_configuration = &(Sx128xFlrcConfiguration[index]);
        SetFlrcConfiguration(flrc_configuration, sync_word);
    }

    void SetRfPower_dbm(int8_t power_dbm)
    {
        RfPowerCalc(power_dbm, &sx_power, &actual_power_dbm);
        SetTxParams(sx_power, SX1280_RAMPTIME_04_US);
    }

    void Configure(void)
    {
        if (Config.modeIsLora()) {
            SetPacketType(SX1280_PACKET_TYPE_LORA);

            SetAutoFs(true);

            SetLnaGainMode(SX1280_LNAGAIN_MODE_HIGH_SENSITIVITY);

            SetRfPower_dbm(Config.Power_dbm);

            SetLoraConfigurationByIndex(Config.LoraConfigIndex);

#ifdef LORA_SYNCWORD
            SetSyncWord(LORA_SYNCWORD);
#endif
        } else
        if (Config.modeIsFLRC()) {
            SetPacketType(SX1280_PACKET_TYPE_FLRC);
            SetAutoFs(true);
            SetLnaGainMode(SX1280_LNAGAIN_MODE_HIGH_SENSITIVITY);
            SetRfPower_dbm(Config.Power_dbm);
            SetFlrcConfigurationByIndex(0, Config.FlrcSyncWord);
        } else {
            while (1) {} // must not happen
        }

        SetBufferBaseAddress(0, 0);

        SetDioIrqParams(SX1280_IRQ_ALL,
                        SX1280_IRQ_RX_DONE | SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT,
                        SX1280_IRQ_NONE,
                        SX1280_IRQ_NONE);
        ClearIrqStatus(SX1280_IRQ_ALL);

        SetFs();
    }

    //-- this are the API functions used in the loop

    void ReadFrame(uint8_t* data, uint8_t len)
    {
        uint8_t rxStartBufferPointer;
        uint8_t rxPayloadLength;

        // rxPayloadLength is always 0 if no header
        GetRxBufferStatus(&rxPayloadLength, &rxStartBufferPointer);
        // if one wants it, it could be obtained from what had been set
        // rxPayloadLength = ReadRegister(SX1280_REG_PayloadLength);
        ReadBuffer(rxStartBufferPointer, data, len);
    }

    void SendFrame(uint8_t* data, uint8_t len, uint16_t tmo_ms)
    {
        WriteBuffer(0, data, len);
        ClearIrqStatus(SX1280_IRQ_ALL);
        SetTx(SX1280_PERIODBASE_62p5_US, tmo_ms*16); // 0 = no timeout, if a Tx timeout occurs we have a serious problem
    }

    void SetToRx(uint16_t tmo_ms)
    {
        ClearIrqStatus(SX1280_IRQ_ALL);
        SetRx(SX1280_PERIODBASE_62p5_US, tmo_ms*16); // 0 = no timeout
    }

    void SetToIdle(void)
    {
        SetFs();
        ClearIrqStatus(SX1280_IRQ_ALL);
    }

    void GetPacketStatus(int8_t* RssiSync, int8_t* Snr)
    {
        int16_t rssi;

        if (Config.modeIsLora()) {
            Sx128xDriverBase::GetPacketStatus(&rssi, Snr);
        } else {
            // FLRC has no SNR
            Sx128xDriverBase::GetPacketStatusFLRC(&rssi);
            *Snr = 0;
        }

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
        int8_t power_dbm = Config.Power_dbm;
        RfPowerCalc(power_dbm, &sx_power, &actual_power_dbm);

        if (Config.modeIsLora()) {
            uint8_t index = Config.LoraConfigIndex;
            if (index >= sizeof(Sx128xLoraConfiguration)/sizeof(Sx128xLoraConfiguration[0])) while (1) {} // must not happen
            lora_configuration = &(Sx128xLoraConfiguration[index]);
        } else {
            flrc_configuration = &(Sx128xFlrcConfiguration[0]);
        }
    }

    // cumbersome to calculate in general, so use hardcoded for a specific settings
    uint32_t TimeOverAir_us(void)
    {
        if (lora_configuration == nullptr && flrc_configuration == nullptr) config_calc(); // ensure it is set

        return (Config.modeIsLora()) ? lora_configuration->TimeOverAir : flrc_configuration->TimeOverAir;
    }

    int16_t ReceiverSensitivity_dbm(void)
    {
        if (lora_configuration == nullptr && flrc_configuration == nullptr) config_calc(); // ensure it is set

        return (Config.modeIsLora()) ? lora_configuration->ReceiverSensitivity : flrc_configuration->ReceiverSensitivity;
    }

    int8_t RfPower_dbm(void)
    {
        if (lora_configuration == nullptr && flrc_configuration == nullptr) config_calc(); // ensure it is set

        return actual_power_dbm;
    }

  private:
    const tSxLoraConfiguration* lora_configuration;
    const tSxFlrcConfiguration* flrc_configuration;
    uint8_t sx_power;
    int8_t actual_power_dbm;
};


//-------------------------------------------------------
// Driver for SX1
//-------------------------------------------------------

// map the irq bits
typedef enum {
    SX_IRQ_TX_DONE = SX1280_IRQ_TX_DONE,
    SX_IRQ_RX_DONE = SX1280_IRQ_RX_DONE,
    SX_IRQ_TIMEOUT = SX1280_IRQ_RX_TX_TIMEOUT,
    SX_IRQ_ALL     = SX1280_IRQ_ALL,
} SX_IRQ_ENUM;


class Sx128xDriver : public Sx128xDriverCommon
{
  public:

    //-- interface to SPI peripheral

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

    void SetDelay(uint16_t tmo_us) override
    {
        timer_us_tmo = (uint32_t)tmo_us * (SystemCoreClock/1000000);
        timer_us_start_tick = DWT->CYCCNT;
    }
#endif

    void SpiSelect(void) override
    {
#ifndef SX_BUSY
        delay_ns(150); // datasheet says t9 = 100 ns, semtech driver doesn't do it, helps so do it
#endif
        spi_select();
        delay_ns(50); // datasheet says t1 = 25 ns, semtech driver doesn't do it, helps so do it
    }

    void SpiDeselect(void) override
    {
        delay_ns(50); // datasheet says t8 = 25 ns, semtech driver doesn't do it, helps so do it
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
        sx1280_rfpower_calc(power_dbm, sx_power, actual_power_dbm, POWER_GAIN_DBM, POWER_SX1280_MAX_DBM);
    }

    //-- init API functions

    void _reset(void)
    {
#ifdef SX_RESET
        gpio_low(SX_RESET);
        delay_ms(5); // 10 us seems to be sufficient, play it safe, semtech driver uses 50 ms
        gpio_high(SX_RESET);
        delay_ms(50); // semtech driver says "typically 2ms observed"
        WaitOnBusy();
#else
        sx_reset();
#endif
    }

    void Init(void)
    {
        Sx128xDriverCommon::Init();

        spi_init();
        sx_init_gpio();
        sx_dio_exti_isr_clearflag();
        sx_dio_init_exti_isroff();

        // no idea how long the SX1280 takes to boot up, so give it some good time
        // we could probably speed up by using WaitOnBusy()
        delay_ms(300);
        _reset(); // this is super crucial !

        SetStandby(SX1280_STDBY_CONFIG_STDBY_RC); // should be in STDBY_RC after reset
        delay_us(1000); // this is important, 500 us ok
    }

    //-- high level API functions

    void StartUp(void)
    {
#ifdef SX_USE_DCDC // here ??? ELRS does it as last !!!
        SetRegulatorMode(SX1280_REGULATOR_MODE_DCDC);
#endif

        Configure();
        delay_us(125); // may not be needed if busy available

        sx_dio_enable_exti_isr();
    }

    //-- this are the API functions used in the loop

    void SendFrame(uint8_t* data, uint8_t len, uint16_t tmo_ms = 0)
    {
        sx_amp_transmit();
        Sx128xDriverCommon::SendFrame(data, len, tmo_ms);
        delay_us(125); // may not be needed if busy available
    }

    void SetToRx(uint16_t tmo_ms = 0)
    {
        sx_amp_receive();
        Sx128xDriverCommon::SetToRx(tmo_ms);
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
    SX2_IRQ_TX_DONE = SX1280_IRQ_TX_DONE,
    SX2_IRQ_RX_DONE = SX1280_IRQ_RX_DONE,
    SX2_IRQ_TIMEOUT = SX1280_IRQ_RX_TX_TIMEOUT,
    SX2_IRQ_ALL     = SX1280_IRQ_ALL,
} SX2_IRQ_ENUM;


class Sx128xDriver2 : public Sx128xDriverCommon
{
  public:

    void WaitOnBusy(void) override
    {
        while (sx2_busy_read()) { __NOP(); };
    }

    void SpiSelect(void) override
    {
        spib_select();
        delay_ns(50); // datasheet says t1 = 25 ns, semtech driver doesn't do it, helps so do it
    }

    void SpiDeselect(void) override
    {
        delay_ns(50); // datasheet says t8 = 25 ns, semtech driver doesn't do it, helps so do it
        spib_deselect();
    }

    void SpiTransferByte(uint8_t* byteout, uint8_t* bytein) override
    {
        *bytein = spib_transmitchar(*byteout);
    }

    //-- RF power interface

    void RfPowerCalc(int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm) override
    {
        sx1280_rfpower_calc(power_dbm, sx_power, actual_power_dbm, POWER_GAIN_DBM, POWER_SX1280_MAX_DBM);
    }

    //-- init API functions

    void _reset(void)
    {
        gpio_low(SX2_RESET);
        delay_ms(5); // 10 us seems to be sufficient, play it safe, semtech driver uses 50 ms
        gpio_high(SX2_RESET);
        delay_ms(50); // semtech driver says "typically 2ms observed"
        WaitOnBusy();
    }

    void Init(void)
    {
        Sx128xDriverCommon::Init();

        spib_init();
        sx2_init_gpio();
        sx2_dio_exti_isr_clearflag();
        sx2_dio_init_exti_isroff();

        // no idea how long the SX1280 takes to boot up, so give it some good time
        // we could probably speed up by using WaitOnBusy()
        delay_ms(300);
        _reset(); // this is super crucial !

        SetStandby(SX1280_STDBY_CONFIG_STDBY_RC); // should be in STDBY_RC after reset
        delay_us(1000); // this is important, 500 us ok
    }

    //-- high level API functions

    void StartUp(void)
    {
//XX        SetStandby(SX1280_STDBY_CONFIG_STDBY_RC); // should be in STDBY_RC after reset
//XX        delay_us(1000); // this is important, 500 us ok

#ifdef SX2_USE_DCDC // here ??? ELRS does it as last !!!
        SetRegulatorMode(SX1280_REGULATOR_MODE_DCDC);
#endif

        Configure();
        delay_us(125); // may not be needed if busy available

        sx2_dio_enable_exti_isr();
    }

    //-- this are the API functions used in the loop

    void SendFrame(uint8_t* data, uint8_t len, uint16_t tmo_ms = 0)
    {
        sx2_amp_transmit();
        Sx128xDriverCommon::SendFrame(data, len, tmo_ms);
        delay_us(125); // may not be needed if busy available
    }

    void SetToRx(uint16_t tmo_ms = 0)
    {
        sx2_amp_receive();
        Sx128xDriverCommon::SetToRx(tmo_ms);
        delay_us(125); // may not be needed if busy available
    }
};

#endif


#endif // SX128X_DRIVER_H
