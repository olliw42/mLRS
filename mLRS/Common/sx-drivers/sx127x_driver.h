//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// SX1276 Driver
//*******************************************************
#ifndef SX1276_DRIVER_H
#define SX1276_DRIVER_H
#pragma once


/* on syncword
https://forum.arduino.cc/t/what-is-sync-word-lora/629624/5:
The default private syncwords are 0x12 for SX127x devices and 0x1424 for SX126x devices.
The syncwords used for public networks such as LoRaWAN\TTN are 0x34 for SX127x devices and 0x3444 for SX126x devices.
https://www.thethingsnetwork.org/forum/t/should-private-lorawan-networks-use-a-different-sync-word/34496/5
https://www.thethingsnetwork.org/forum/t/should-private-lorawan-networks-use-a-different-sync-word/34496/15
*/


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


const tSxLoraConfiguration Sx127xLoraConfiguration[] = {
    { .SpreadingFactor = SX1276_LORA_SF6,
      .Bandwidth = SX1276_LORA_BW_500,
      .CodingRate = SX1276_LORA_CR_4_5,
      .PreambleLength = 12,
      .HeaderType = SX1276_LORA_HEADER_DISABLE,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CrcEnabled = SX1276_LORA_CRC_DISABLE,
      .InvertIQ = SX1276_LORA_IQ_NORMAL,
      .TimeOverAir = 22300,
      .ReceiverSensitivity = -112,
    }
};


// map the irq bits on some common
typedef enum {
    SX12xx_IRQ_TX_DONE = SX1276_IRQ_TX_DONE,
    SX12xx_IRQ_RX_DONE = SX1276_IRQ_RX_DONE,
    SX12xx_IRQ_TIMEOUT = SX1276_IRQ_RX_TIMEOUT,
    SX12xx_IRQ_ALL     = SX1276_IRQ_ALL,
} SX12xx_IRQ_ENUM;


#ifdef POWER_USE_DEFAULT_RFPOWER_CALC
void rfpower_calc(int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm)
{
    // Pout = 17 - (15 - OutputPower) if PaSelect = 1 (PA_BOOST pin)
    int16_t power_sx = (int16_t)power_dbm - POWER_GAIN_DBM + 2;

    if (power_sx < SX1276_OUTPUT_POWER_MIN) power_sx = SX1276_OUTPUT_POWER_MIN;
    if (power_sx > SX1276_OUTPUT_POWER_MAX) power_sx = SX1276_OUTPUT_POWER_MAX;
    if (power_sx > POWER_SX1276_MAX_DBM) power_sx = POWER_SX1276_MAX_DBM;

    *sx_power = power_sx;
    *actual_power_dbm = power_sx + POWER_GAIN_DBM - 2;
}
#endif


class Sx127xDriverCommon : public Sx127xDriverBase
{
  public:
    void Init(void)
    {
        lora_configuration = nullptr;
    }

    //-- high level API functions

    bool isOk(void)
    {
        uint8_t firmwareRev = GetFirmwareRev();
        return (firmwareRev == 0x12);
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

        symbol_time_us = calc_symbol_time_us(config->SpreadingFactor, config->Bandwidth);
    }

    void SetLoraConfigurationByIndex(uint8_t index)
    {
        if (index >= sizeof(Sx127xLoraConfiguration)/sizeof(Sx127xLoraConfiguration[0])) while (1) {} // must not happen

        lora_configuration = &(Sx127xLoraConfiguration[index]);
        SetLoraConfiguration(lora_configuration);
    }

    void SetRfPower_dbm(int8_t power_dbm)
    {
        uint8_t sx_power;
#ifdef DEVICE_HAS_I2C_DAC
        rfpower_calc(power_dbm, &sx_power, &actual_power_dbm, &dac);
#else
        rfpower_calc(power_dbm, &sx_power, &actual_power_dbm);
#endif
        // MaxPower is irrelevant, so set it to SX1276_MAX_POWER_15_DBM
        // there would be special setting for +20dBm mode, don't do it
        // 5 OcpOn, 4-0 OcpTrim
        ReadWriteRegister(SX1276_REG_Ocp, 0x3F, SX1276_OCP_ON | SX1276_OCP_TRIM_150_MA);
        SetPowerParams(SX1276_PA_SELECT_PA_BOOST, SX1276_MAX_POWER_15_DBM, sx_power, SX1276_PA_RAMP_40_US);
    }

    void Configure(void)
    {
        SetSleep(); // must be in sleep to switch to LoRa mode
        WriteRegister(SX1276_REG_OpMode, SX1276_PACKET_TYPE_LORA |
                                         SX1276_ACCESS_SHARED_REG_LORA |
                                         SX1276_LOW_FREQUENCY_MODE_OFF |
                                         SX1276_MODE_SLEEP);
        SetStandby();
        //SetOperationMode(SX1276_PACKET_TYPE_LORA, SX1276_LOW_FREQUENCY_MODE_OFF);

        uint8_t band_width = Sx127xLoraConfiguration[Config.LoraConfigIndex].Bandwidth;
        OptimizeSensitivity(band_width);
        OptimizeReceiverResponse(band_width);

        SetLnaParams(SX1276_LNA_GAIN_DEFAULT, SX1276_LNA_BOOST_HF_ON);
        // 3 LowDataRateOptimize, 2 AgcAutoOn
        ReadWriteRegister(SX1276_REG_ModemConfig3, 0x0C, SX1276_LORA_LOW_DATA_RATE_OPTIMIZE_OFF | SX1276_LORA_AGC_AUTO_ON);

        // 5 OcpOn, 4-0 OcpTrim
        //ReadWriteRegister(SX1276_REG_Ocp, 0x3F, SX1276_OCP_ON | SX1276_OCP_TRIM_150_MA);
        //SetPowerParams(SX1276_PA_SELECT_PA_BOOST, SX1276_MAX_POWER_15_DBM, 0, SX1276_PA_RAMP_40_US);
        SetRfPower_dbm(Config.Power_dbm);

        SetLoraConfigurationByIndex(Config.LoraConfigIndex);

        // SetSyncWord(0x12);

        SetBufferBaseAddress(0, 0);

        SetDioIrqParams(SX1276_IRQ_TX_DONE | SX1276_IRQ_RX_DONE | SX1276_IRQ_RX_TIMEOUT, // this helps for RX!! //  SX1276_IRQ_ALL,
                        SX1276_DIO0_MAPPING_RX_TX_DONE,
                        SX1276_DIO1_MAPPING_RX_TIMEOUT);
        ClearIrqStatus(SX1276_IRQ_ALL);
    }

    //-- this are the API functions used in the loop

    void ReadFrame(uint8_t* data, uint8_t len)
    {
        uint8_t rxStartBufferPointer;
        uint8_t rxPayloadLength;

        GetRxBufferStatus(&rxPayloadLength, &rxStartBufferPointer);
        ReadBuffer(rxStartBufferPointer, data, len);
    }

    void SendFrame(uint8_t* data, uint8_t len, uint16_t tmo_ms = 100) // SX1276 doesn't have a Tx timeout
    {
        WriteBuffer(0, data, len);
        ClearIrqStatus(SX1276_IRQ_ALL);
        SetTx();
    }

    void SetToRx(uint16_t tmo_ms = 10)
    {
        WriteRegister(SX1276_REG_FifoAddrPtr, 0);
        ClearIrqStatus(SX1276_IRQ_ALL);
        if (tmo_ms == 0) {
            SetRxContinuous();
        } else {
            SetRxTimeout(((uint32_t)tmo_ms * 1000) / symbol_time_us);
            SetRxSingle();
        }
    }

    void SetToIdle(void)
    {
        SetStandby();
    }

    void GetPacketStatus(int8_t* RssiSync, int8_t* Snr)
    {
        int16_t rssi;
        Sx127xDriverBase::GetPacketStatus(&rssi, Snr);

        if (rssi > -1) rssi = -1; // we do not support values larger than this
        if (rssi < -127) rssi = -127; // we do not support values lower than this

        *RssiSync = rssi;
    }

    //-- helper

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

    int8_t RfPower_dbm(void)
    {
        return actual_power_dbm;
    }

  private:
    const tSxLoraConfiguration* lora_configuration;
    int8_t actual_power_dbm;
    uint32_t symbol_time_us;

    uint32_t calc_symbol_time_us(uint8_t SpreadingFactor, uint8_t Bandwidth)
    {
        uint32_t sf = (SpreadingFactor >> 4); // slightly dirty as it uses explicit knowledge

        uint32_t bw = 7800;
        switch (Bandwidth) {
          case SX1276_LORA_BW_7p8: bw = 7800; break;
          case SX1276_LORA_BW_10p4: bw = 10400; break;
          case SX1276_LORA_BW_15p6: bw = 15600; break;
          case SX1276_LORA_BW_20p8: bw = 20800; break;
          case SX1276_LORA_BW_31p25: bw = 31250; break;
          case SX1276_LORA_BW_41p7: bw = 41700; break;
          case SX1276_LORA_BW_62p5: bw = 62500; break;
          case SX1276_LORA_BW_125: bw = 125000; break;
          case SX1276_LORA_BW_250: bw = 250000; break;
          case SX1276_LORA_BW_500: bw = 500000; break;
        };

        return ((1 << sf) * 1000000) / bw;
    }
};


//-------------------------------------------------------
// Driver for SX1
//-------------------------------------------------------


class Sx127xDriver : public Sx127xDriverCommon
{
  public:

    //-- interface to SPI peripheral

    void SpiSelect(void) override
    {
        delay_ns(30); // datasheet says tnhigh = 20 ns, NSS high time between SPI accesses
        spi_select();
        delay_ns(40); // datasheet says tnsetup = 30 ns, NSS setup time, From NSS falling edge to SCK rising  edge
    }

    void SpiDeselect(void) override
    {
        delay_ns(100); // datasheet says tnhold = 100 ns, NSS hold time From SCK falling edge to NSS rising edge, normal mode
        spi_deselect();
        delay_ns(100); // well...
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
        delay_ms(50); // datasheet says 5 ms
    }

    void Init(void)
    {
        Sx127xDriverCommon::Init();

        spi_init();
        sx_init_gpio();
        sx_dio_init_exti_isroff();

        // no idea how long the SX1276 takes to boot up, so give it some good time
        delay_ms(300);
        _reset(); // this is super crucial ! was so for SX1280, is it also for the SX1276 ??
    }

    //-- high level API functions

    void StartUp(void)
    {
        // this is not nice, figure out where to place
#ifdef DEVICE_HAS_I2C_DAC
        dac.Init();
#endif

        SetStandby(); // should be in STDBY after reset
        delay_us(1000); // is this needed ????

        Configure();
        delay_us(125); // may not be needed

        sx_dio_enable_exti_isr();
    }

    //-- this are the API functions used in the loop

    void SendFrame(uint8_t* data, uint8_t len, uint16_t tmo_ms = 100)
    {
        sx_amp_transmit();
        Sx127xDriverCommon::SendFrame(data, len, tmo_ms);
        delay_us(125); // may not be needed if busy available
    }

    void SetToRx(uint16_t tmo_ms = 10)
    {
        sx_amp_receive();
        Sx127xDriverCommon::SetToRx(tmo_ms);
        delay_us(125); // may not be needed if busy available
    }
};


//-------------------------------------------------------
// Driver for SX2
//-------------------------------------------------------
#ifdef DEVICE_HAS_DIVERSITY
#error Diversity not yet supported for SX127x
#endif


#endif // SX1276_DRIVER_H
