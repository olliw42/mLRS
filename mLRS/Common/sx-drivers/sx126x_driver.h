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


// map the irq bits on some common
typedef enum {
    SX12xx_IRQ_TX_DONE = SX126X_IRQ_TX_DONE,
    SX12xx_IRQ_RX_DONE = SX126X_IRQ_RX_DONE,
    SX12xx_IRQ_TIMEOUT = SX126X_IRQ_RX_TX_TIMEOUT,
    SX12xx_IRQ_ALL     = SX126X_IRQ_ALL,
} SX12xx_IRQ_ENUM;


#ifdef POWER_USE_DEFAULT_RFPOWER_CALC
void rfpower_calc(int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm)
{
    int16_t power_sx = (int16_t)power_dbm - POWER_GAIN_DBM;

    if (power_sx < SX126X_POWER_m9_DBM) power_sx = SX126X_POWER_m9_DBM;
    if (power_sx > POWER_SX126X_MAX_DBM) power_sx = POWER_SX126X_MAX_DBM;

    *sx_power = power_sx;
    *actual_power_dbm = power_sx + POWER_GAIN_DBM;
}
#endif


class Sx126xDriverCommon : public Sx126xDriverBase
{
  public:

    void Init(void)
    {
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

    void SetRfPower_dbm(int8_t power_dbm)
    {
        rfpower_calc(power_dbm, &sx_power, &actual_power_dbm);
        SetTxParams(sx_power, SX126X_RAMPTIME_10_US);
    }

    void Configure(void)
    {
        SetPacketType(SX126X_PACKET_TYPE_LORA);

        // WORKAROUND: Better Resistance of the SX1262 Tx to Antenna Mismatch,
        // fixes overly eager PA clamping
        // see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.2 for details
        uint8_t data = ReadRegister(SX126X_REG_TX_CLAMP_CONFIG);
        data |= 0x1E;
        WriteRegister(SX126X_REG_TX_CLAMP_CONFIG, data);

        ClearDeviceError();

        // set DIO3 as TCXO control
        SetDio3AsTcxoControl(SX126X_DIO3_OUTPUT_1_8, 250); // set output to 1.8V, ask specification of TCXO to maker board
        // set DIO2 as RF control switching
        // check the RF module if antenna switching is internally connected to DIO2
        // E22-900M rf switching controlled by IO pin of MCU
        // SetDio2AsRfSwitchControl(SX126X_DIO2_AS_RF_SWITCH);

        SetAutoFs(true); // SetRxTxFallbackMode to FS

        SetLnaGainMode(SX126X_LNA_GAIN_MODE_HIGH_SENSITIVITY);

        SetPaConfig_22dbm();

        //SetTxParams(calc_sx_power(Config.Power), SX126X_RAMPTIME_10_US);
        SetRfPower_dbm(Config.Power_dbm);

        SetLoraConfigurationByIndex(Config.LoraConfigIndex);

        // SetSyncWord(0x1424); // public network, no need to call as it defaults to 0x1424

        SetBufferBaseAddress(0, 0);

        SetDioIrqParams(SX126X_IRQ_ALL,
                        SX126X_IRQ_RX_DONE|SX126X_IRQ_TX_DONE|SX126X_IRQ_RX_TX_TIMEOUT,
                        SX126X_IRQ_NONE,
                        SX126X_IRQ_NONE);
        ClearIrqStatus(SX126X_IRQ_ALL);

        SetFs();
    }

    void ReadFrame(uint8_t* data, uint8_t len)
    {
        uint8_t rxStartBufferPointer;
        uint8_t rxPayloadLength;

        GetRxBufferStatus(&rxPayloadLength, &rxStartBufferPointer);
        ReadBuffer(rxStartBufferPointer, data, len);
    }

    void SendFrame(uint8_t* data, uint8_t len, uint16_t tmo_ms = 100)
    {
        WriteBuffer(0, data, len);
        ClearIrqStatus(SX126X_IRQ_ALL);
        SetTx(tmo_ms * 64); // TimeOut period inn ms. sx1262 have static 15p625 period base, so for 1 ms needs 64 tmo value
    }

    void SetToRx(uint16_t tmo_ms = 10)
    {
        ClearIrqStatus(SX126X_IRQ_ALL);
        SetRx(tmo_ms * 64);
    }

    //-- helper

    void SetToIdle(void)
    {
        SetFs();
    }

    void config_calc(void)
    {
         int8_t power_dbm = Config.Power_dbm;
         rfpower_calc(power_dbm, &sx_power, &actual_power_dbm);

         uint8_t index = Config.LoraConfigIndex;
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

  private:
    const tSxLoraConfiguration* lora_configuration;
    uint8_t sx_power;
    int8_t actual_power_dbm;
};


//-------------------------------------------------------
// Driver for SX1
//-------------------------------------------------------

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

    void SpiTransfer(uint8_t* dataout, uint8_t* datain, uint8_t len) override
    {
        spi_transfer(dataout, datain, len);
    }

    //-- init API functions

    void _reset(void)
    {
        gpio_low(SX_RESET);
        delay_ms(5); // datasheet says > 100 us
        gpio_high(SX_RESET);
        delay_ms(50);
        WaitOnBusy();
    }

    void Init(void)
    {
        Sx126xDriverCommon::Init();

        spi_init();
        sx_init_gpio();
        sx_dio_init_exti_isroff();

        // no idea how long the SX1280 takes to boot up, so give it some good time
        // we could probably speed up by using WaitOnBusy()
        delay_ms(300);
        _reset(); // this is super crucial ! was so for SX1280, is it also for the SX1262 ??
    }

    //-- high level API functions

    void StartUp(void)
    {
        SetStandby(SX126X_STDBY_CONFIG_STDBY_RC); // should be in STDBY_RC after reset
        delay_us(1000); // is this needed ????

#ifdef SX_USE_DCDC // here ??? ELRS does it as last !!!
        SetRegulatorMode(SX126X_REGULATOR_MODE_DCDC);
#endif

        Configure();
        delay_us(125); // may not be needed if busy available

        sx_dio_enable_exti_isr();
    }

    //-- this are the API functions used in the loop

    void SendFrame(uint8_t* data, uint8_t len, uint16_t tmo_ms = 100)
    {
        sx_amp_transmit();
        Sx126xDriverCommon::SendFrame(data, len, tmo_ms);
        delay_us(125); // may not be needed if busy available
    }

    void SetToRx(uint16_t tmo_ms = 10)
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

#endif


#endif // SX126X_DRIVER_H
