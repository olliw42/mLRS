//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// SX1280 Driver
//*******************************************************
#ifndef SX1280_DRIVER_H
#define SX1280_DRIVER_H
#pragma once


//-------------------------------------------------------
// SX Driver
//-------------------------------------------------------
// some comments:
//
// I was polling for irq in the main loop, and initially it looked fine for both Rx & Tx
// I was then getting more and more issues with RX, and did a rescue method to somehow recover
// but the more code I added the more weird the behavior became. A single change in a line could make it
// given millions of !0404, disconnect issues, and so on. It could happen that the mode was not changed
// to TX, but stayed at FS. It seemed that there is then no rescue besides a hard reset. I
// empirically concluded also that it doesn't like if much is done between the irq and ReadFrame().
// For some reasons, all this wasn't required on TX. I noted however that on TX the LQ dropped quite
// quickly with moving away.
// For RX, I then tried using the busy flag in WaitOnBusy(), and all issues immediately went totally away.
// I when tried doing the dio1 in an exti isr, and this improved things a bit, but not totally. Only when I
// did also the ReadFrame() in the etxi isr, also all issues went away.
// Doing the later on TX also improved it's LQ loss behavior a lot. (but it is still not as good as for RX)
// => either do doi1 isr with ReadFrame() in there, or even better have a busy


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


const tSxLoraConfiguration SxLoraConfiguration[] = {
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


// map the irq bits on some common
typedef enum {
    SX12xx_IRQ_TX_DONE = SX1280_IRQ_TX_DONE,
    SX12xx_IRQ_RX_DONE = SX1280_IRQ_RX_DONE,
    SX12xx_IRQ_TIMEOUT = SX1280_IRQ_RX_TX_TIMEOUT,
    SX12xx_IRQ_ALL     = SX1280_IRQ_ALL,
} SX12xx_IRQ_ENUM;


class Sx128xDriverCommon : public Sx128xDriverBase
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
    }

    void SetLoraConfigurationByIndex(uint8_t index)
    {
        lora_configuration = &(SxLoraConfiguration[index]);
        SetLoraConfiguration(lora_configuration);
    }

    void Configure(void)
    {
        SetPacketType(SX1280_PACKET_TYPE_LORA);
        SetBufferBaseAddress(0, 0);

        SetAutoFs(true);

        SetLnaGainMode(SX1280_LNAGAIN_MODE_HIGH_SENSITIVITY);

        SetLoraConfigurationByIndex(Config.LoraConfigIndex);

#ifdef LORA_SYNCWORD
        SetSyncWord(LORA_SYNCWORD);
#endif

        SetTxParams(calc_sx_power(Config.Power), SX1280_RAMPTIME_04_US);

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

    void SendFrame(uint8_t* data, uint8_t len, uint16_t tmo_ms = 100)
    {
        WriteBuffer(0, data, len);
        ClearIrqStatus(SX1280_IRQ_ALL);
        SetTx(SX1280_PERIODBASE_62p5_US, tmo_ms*16); // if a Tx timeout occurs we have a serious problem
    }

    void SetToRx(uint16_t tmo_ms = 10)
    {
        ClearIrqStatus(SX1280_IRQ_ALL);
        SetRx(SX1280_PERIODBASE_62p5_US, tmo_ms*16);
    }

    void SetToIdle(void)
    {
        SetFs();
    }

    //-- helper

    void SetRfPower(uint8_t power)
    {
        SetTxParams(power, SX1280_RAMPTIME_04_US);
    }

    uint32_t TimeOverAir_us(void)
    {
        // cumbersome to calculate in general, so use hardcoded for a specific settings
        if (lora_configuration == nullptr) return 0;

        return lora_configuration->TimeOverAir;
    }

    int16_t ReceiverSensitivity_dbm(void)
    {
        if (lora_configuration == nullptr) return 0;

        return lora_configuration->ReceiverSensitivity;
    }

    int8_t GetActualPower(void)
    {
        return (int8_t)sx_power + POWER_GAIN_DBM - 18;
    }

    uint8_t GetActualSxPower(void)
    {
        return sx_power;
    }

  private:
    const tSxLoraConfiguration* lora_configuration;

    int8_t sx_power;

    uint8_t calc_sx_power(int8_t power)
    {
        sx_power = power - POWER_GAIN_DBM + 18;
        if (sx_power > POWER_SX1280_MAX_DBM) sx_power = POWER_SX1280_MAX_DBM;
        if (sx_power < SX1280_POWER_m18_DBM) sx_power = SX1280_POWER_m18_DBM;
        if (sx_power > SX1280_POWER_12p5_DBM) sx_power = SX1280_POWER_12p5_DBM;
        return sx_power;
    }
};


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
    void SetToRx(uint16_t tmo_us) {}
    void SetToIdle(void) {}
};


//-------------------------------------------------------
// Driver for SX1
//-------------------------------------------------------

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

    void SpiTransfer(uint8_t* dataout, uint8_t* datain, uint8_t len) override
    {
        spi_transfer(dataout, datain, len);
    }

    //-- init API functions

    void _reset(void)
    {
        gpio_low(SX_RESET);
        delay_ms(5); // 10 us seems to be sufficient, play it safe, semtech driver uses 50 ms
        gpio_high(SX_RESET);
        delay_ms(50); // semtech driver says "typically 2ms observed"
        WaitOnBusy();
    }

    void Init(void)
    {
        Sx128xDriverCommon::Init();

        spi_init();
        sx_init_gpio();
        sx_dio_init_exti_isroff();

        // no idea how long the SX1280 takes to boot up, so give it some good time
        // we could probably speed up by using WaitOnBusy()
        delay_ms(300);
        _reset(); // this is super crucial !
    }

    //-- high level API functions

    //bool isOk(void)
    //void SetLoraConfiguration(const tSxLoraConfiguration* config)
    //void SetLoraConfigurationByIndex(uint8_t index)
    //void Configure(void)

    void StartUp(void)
    {
        SetStandby(SX1280_STDBY_CONFIG_STDBY_RC); // should be in STDBY_RC after reset
        delay_us(1000); // this is important, 500 us ok

#ifdef SX_USE_DCDC // here ??? ELRS does it as last !!!
        SetRegulatorMode(SX1280_REGULATOR_MODE_DCDC);
#endif

        Configure();
        delay_us(125); // may not be needed if busy available

        sx_dio_enable_exti_isr();
    }

    //-- this are the API functions used in the loop

    void SendFrame(uint8_t* data, uint8_t len, uint16_t tmo_ms = 100)
    {
        sx_amp_transmit();
        Sx128xDriverCommon::SendFrame(data, len, tmo_ms);
        delay_us(125); // may not be needed if busy available
    }

    void SetToRx(uint16_t tmo_ms = 10)
    {
        sx_amp_receive();
        Sx128xDriverCommon::SetToRx(tmo_ms);
        delay_us(125); // may not be needed if busy available
    }

    //void ReadFrame(uint8_t* data, uint8_t len)
};


//-------------------------------------------------------
// Driver for SX2
//-------------------------------------------------------
#ifdef DEVICE_HAS_DIVERSITY

#ifndef SX2_BUSY
    #error SX2 must have a BUSY pin !!
#endif

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

    void SpiTransfer(uint8_t* dataout, uint8_t* datain, uint8_t len) override
    {
        spib_transfer(dataout, datain, len);
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
        sx2_dio_init_exti_isroff();

        // no idea how long the SX1280 takes to boot up, so give it some good time
        // we could probably speed up by using WaitOnBusy()
        delay_ms(300);
        _reset(); // this is super crucial !
    }

    //-- high level API functions

    void StartUp(void)
    {
        SetStandby(SX1280_STDBY_CONFIG_STDBY_RC); // should be in STDBY_RC after reset
        delay_us(1000); // this is important, 500 us ok

#ifdef SX2_USE_DCDC // here ??? ELRS does it as last !!!
        SetRegulatorMode(SX1280_REGULATOR_MODE_DCDC);
#endif

        Configure();
        delay_us(125); // may not be needed if busy available

        sx2_dio_enable_exti_isr();
    }

    //-- this are the API functions used in the loop

    void SendFrame(uint8_t* data, uint8_t len, uint16_t tmo_ms = 100)
    {
        sx2_amp_transmit();
        Sx128xDriverCommon::SendFrame(data, len, tmo_ms);
        delay_us(125); // may not be needed if busy available
    }

    void SetToRx(uint16_t tmo_ms = 10)
    {
        sx2_amp_receive();
        Sx128xDriverCommon::SetToRx(tmo_ms);
        delay_us(125); // may not be needed if busy available
    }
};

#endif


#endif // SX1280_DRIVER_H
