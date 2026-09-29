//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// SX1276 Driver
//*******************************************************
// Configuration defines:
// #define POWER_USE_DEFAULT_RFPOWER_CALC
// #define SX_USE_RFO
// #define DEVICE_HAS_I2C_DAC
// #define DEVICE_HAS_INTERNAL_DAC_TWOCHANNELS
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

const tSxLoraConfiguration Sx127xLoraConfiguration[] = {
    { .SpreadingFactor = SX1276_LORA_SF6, // 900 MHz, 19 Hz 7x
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


// FSK 50 Hz, must match the Sx126x, Lr11xx, Lr20xx GfskConfiguration, frame > FIFO so it's streamed via DIO1
// whitening is done in software, as the SX127x seed can't be set to match the SX126x default seed
const tSxGfskConfiguration Sx127xGfskConfiguration[] = { // 900 MHz, 50 Hz FSK
    { .BitRate_bps = 100000,
      .PulseShape = SX1276_FSK_PULSESHAPE_BT_1,
      .Bandwidth = SX1276_FSK_BW_166700, // single side, sx126x uses 312 kHz double side
      .Fdev_hz = 50000,
      .PreambleLength = 16,
      .PreambleDetectorLength = SX1276_FSK_PREAMBLE_DETECTOR_LENGTH_8BITS,
      .SyncWordLength = 16,
      .AddrComp = 0, // not used
      .PacketType = SX1276_FSK_PKT_FIX_LEN,
      .PayloadLength = FRAME_TX_RX_LEN,
      .CRCType = SX1276_FSK_CRC_OFF,
      .Whitening = SX1276_FSK_WHITENING_OFF, // done in software
      .TimeOverAir = 7600,
      .ReceiverSensitivity = -104, // this is a guess
    }
};


#define SX127X_FSK_FIFO_THRESHOLD         32 // FifoLevel irq when more than 32 bytes in FIFO
#define SX127X_FSK_FIFO_SIZE              64 // streaming relies on 33 < FRAME_TX_RX_LEN <= 95

// ATTENTION: FSK FIFO burst writes at 10 MHz spi clock corrupt data, bit 0 of a byte is replaced by the MSB
// of the next byte, 8 MHz works, LoRa is not affected, so FSK capable boards need SPI_FREQUENCY <= 8 MHz

// whitening seed of SX126x, LR11xx, LR20xx chip default, SX126x register 0x06B8 resets to 0x0100
// verified on air against LR1121
#define SX127X_FSK_WHITENING_SEED         0x0100


// PN9 whitening, x^9 + x^5 + 1, as done by SX126x, LR11xx, LR20xx
// the key bits are taken MSB first, i.e. bit reversed compared to the SX127x/CC1101 PN9 byte
void sx127x_fsk_whiten(uint8_t* const data, uint8_t len, uint16_t seed)
{
    uint16_t lfsr = seed;
    for (uint8_t n = 0; n < len; n++) {
        uint8_t key = 0;
        for (uint8_t i = 0; i < 8; i++) {
            key = (key << 1) | (lfsr & 0x01);
            uint16_t bit = (lfsr ^ (lfsr >> 5)) & 0x01;
            lfsr = (lfsr >> 1) | (bit << 8);
        }
        data[n] ^= key;
    }
}


#ifdef POWER_USE_DEFAULT_RFPOWER_CALC
void sx1276_rfpower_calc_default(const int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm, const int8_t gain_dbm, const int8_t sx_power_max)
{
#ifdef SX_USE_RFO
    // Pout = OutputPower if PaSelect = 0 (RFO pin)
    int16_t power_sx = (int16_t)power_dbm - gain_dbm + 3;
#else
    // Pout = 17 - (15 - OutputPower) if PaSelect = 1 (PA_BOOST pin)
    int16_t power_sx = (int16_t)power_dbm - gain_dbm - 2;
#endif

    if (power_sx < SX1276_OUTPUT_POWER_MIN) power_sx = SX1276_OUTPUT_POWER_MIN;
    if (power_sx > SX1276_OUTPUT_POWER_MAX) power_sx = SX1276_OUTPUT_POWER_MAX;
    if (power_sx > sx_power_max) power_sx = sx_power_max;

    *sx_power = power_sx;

#ifdef SX_USE_RFO
    *actual_power_dbm = power_sx + gain_dbm - 3;
#else
    *actual_power_dbm = power_sx + gain_dbm + 2;
#endif
}
#endif


class Sx127xDriverCommon : public Sx127xDriverBase
{
  public:

    void Init(void)
    {
        gconfig = nullptr;
        lora_configuration = nullptr;
        gfsk_configuration = nullptr;
        is_in_lora_mode = true;
        low_frequency_mode = 0;
        fsk_rssi = -127;
        power_dbm_last = 0;
        fsk_state = FSK_STATE_IDLE;
        fsk_pos = 0;
    }

    //-- high level API functions

    bool isOk(void)
    {
        uint8_t firmwareRev = GetFirmwareRev();
        return (firmwareRev == 0x12);
    }

    void SetLoraConfiguration(const tSxLoraConfiguration* const config)
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
        if (index >= sizeof(Sx127xLoraConfiguration)/sizeof(Sx127xLoraConfiguration[0])) while(1){} // must not happen

        lora_configuration = &(Sx127xLoraConfiguration[index]);
        SetLoraConfiguration(lora_configuration);
    }

    void ResetToLoraConfiguration(tSxGlobalConfig* const _gconfig)
    {
        if (!gconfig) while(1){} // must not happen

        gconfig->LoraConfigIndex = _gconfig->LoraConfigIndex;

        if (!is_in_lora_mode) { // we need to switch from FSK to LoRa, keep the power, bind may have lowered it
            _configure_lora();
            SetRfPower_dbm(power_dbm_last);
            return;
        }

        SetLoraConfigurationByIndex(gconfig->LoraConfigIndex);
    }

    void SetGfskConfiguration(const tSxGfskConfiguration* const config, uint16_t sync_word)
    {
        SetModulationParamsFSK(config->BitRate_bps,
                               config->PulseShape,
                               config->Bandwidth,
                               config->Fdev_hz);

        SetPacketParamsFSK(config->PreambleLength,
                           config->PreambleDetectorLength,
                           config->SyncWordLength,
                           config->PacketType,
                           config->PayloadLength,
                           config->CRCType,
                           config->Whitening);

        SetSyncWordFSK(sync_word);
    }

    void SetGfskConfigurationByIndex(uint8_t index, uint16_t sync_word)
    {
        if (index >= sizeof(Sx127xGfskConfiguration)/sizeof(Sx127xGfskConfiguration[0])) while(1){} // must not happen

        gfsk_configuration = &(Sx127xGfskConfiguration[index]);
        SetGfskConfiguration(gfsk_configuration, sync_word);
    }

    void SetRfPower_dbm(int8_t power_dbm)
    {
        if (!gconfig) return;

        power_dbm_last = power_dbm;
        _rfpower_calc(power_dbm, &sx_power, &actual_power_dbm);
        // MaxPower is irrelevant, so set it to SX1276_MAX_POWER_15_DBM
        // there would be special setting for +20dBm mode, don't do it
        // 5 OcpOn, 4-0 OcpTrim
        ReadWriteRegister(SX1276_REG_Ocp, 0x3F, SX1276_OCP_ON | SX1276_OCP_TRIM_150_MA);
#ifdef SX_USE_RFO
        // was SX1276_MAX_POWER_15_DBM before
        SetPowerParams(SX1276_PA_SELECT_RFO, SX1276_MAX_POWER_11p4_DBM, sx_power, SX1276_PA_RAMP_40_US);
#else
        SetPowerParams(SX1276_PA_SELECT_PA_BOOST, SX1276_MAX_POWER_15_DBM, sx_power, SX1276_PA_RAMP_40_US);
#endif
    }

    void UpdateRfPower(tSxGlobalConfig* const global_config)
    {
        if (!gconfig) return;

        gconfig->Power_dbm = global_config->Power_dbm;
        SetRfPower_dbm(gconfig->Power_dbm);
    }

    void Configure(tSxGlobalConfig* const global_config)
    {
        gconfig = global_config;

        switch (gconfig->FrequencyBand) {
            case SX_FHSS_FREQUENCY_BAND_433_MHZ:
            case SX_FHSS_FREQUENCY_BAND_70_CM_HAM:
                low_frequency_mode = SX1276_LOW_FREQUENCY_MODE_ON;
                break;
            default:
                low_frequency_mode = SX1276_LOW_FREQUENCY_MODE_OFF;
        }

        if (gconfig->modeIsLora()) {
            _configure_lora();
        } else {
            _configure_fsk();
        }

        SetRfPower_dbm(gconfig->Power_dbm);
    }

    void _configure_fsk(void)
    {
        is_in_lora_mode = false;

        SetSleep(); // must be in sleep to switch to FSK mode
        WriteRegister(SX1276_REG_OpMode, SX1276_PACKET_TYPE_FSK_OOK |
                                         low_frequency_mode |
                                         SX1276_MODE_SLEEP);
        SetStandby();

        SetLnaParams(SX1276_LNA_GAIN_DEFAULT, SX1276_LNA_BOOST_HF_ON);

        SetGfskConfigurationByIndex(0, gconfig->FskSyncWord);

        // no AFC, loses ca 15% of frames with the 16 bit preamble
        SetRxConfigFSK(SX1276_FSK_RX_CONFIG_AGC_AUTO_ON | SX1276_FSK_RX_CONFIG_TRIGGER_PREAMBLE_DETECT,
                       SX1276_FSK_RSSI_SMOOTHING_32);

        SetFifoThresholdFSK(SX127X_FSK_FIFO_THRESHOLD);
        SetDioMappingFSK(SX1276_FSK_DIO0_MAPPING_PAYLOAD_READY_PACKET_SENT, SX1276_FSK_DIO1_MAPPING_FIFO_LEVEL);
        _fsk_standby();
    }

    void _configure_lora(void)
    {
        is_in_lora_mode = true;

        SetSleep(); // must be in sleep to switch to LoRa mode
        WriteRegister(SX1276_REG_OpMode, SX1276_PACKET_TYPE_LORA |
                                         SX1276_ACCESS_SHARED_REG_LORA |
                                         low_frequency_mode |
                                         SX1276_MODE_SLEEP);
        SetStandby();
        //SetOperationMode(SX1276_PACKET_TYPE_LORA, SX1276_LOW_FREQUENCY_MODE_OFF);

        uint8_t band_width = Sx127xLoraConfiguration[gconfig->LoraConfigIndex].Bandwidth;
        OptimizeSensitivity(band_width, low_frequency_mode);
        OptimizeReceiverResponse(band_width);

        SetLnaParams(SX1276_LNA_GAIN_DEFAULT, SX1276_LNA_BOOST_HF_ON);
        // 3 LowDataRateOptimize, 2 AgcAutoOn
        ReadWriteRegister(SX1276_REG_ModemConfig3, 0x0C, SX1276_LORA_LOW_DATA_RATE_OPTIMIZE_OFF | SX1276_LORA_AGC_AUTO_ON);

        // 5 OcpOn, 4-0 OcpTrim
        //ReadWriteRegister(SX1276_REG_Ocp, 0x3F, SX1276_OCP_ON | SX1276_OCP_TRIM_150_MA);
        //SetPowerParams(SX1276_PA_SELECT_PA_BOOST, SX1276_MAX_POWER_15_DBM, 0, SX1276_PA_RAMP_40_US);

        SetLoraConfigurationByIndex(gconfig->LoraConfigIndex);

        // SetSyncWord(0x12);

        SetBufferBaseAddress(0, 0);

        SetDioIrqParams(SX1276_IRQ_TX_DONE | SX1276_IRQ_RX_DONE | SX1276_IRQ_RX_TIMEOUT, // this helps for RX!! //  SX1276_IRQ_ALL,
                        SX1276_DIO0_MAPPING_RX_TX_DONE,
                        SX1276_DIO1_MAPPING_RX_TIMEOUT);
        ClearIrqStatus(SX1276_IRQ_ALL);
    }

    //-- these are the API functions used in the loop

    void ReadFrame(uint8_t* const data, uint8_t len)
    {
/*        uint8_t rxStartBufferPointer;
        uint8_t rxPayloadLength;

        GetRxBufferStatus(&rxPayloadLength, &rxStartBufferPointer);
        ReadBuffer(rxStartBufferPointer, data, len); */

        // it seems that rxStartBufferPointer is always 0, so we assume that
        ReadBuffer(0, data, len);
    }

    void SendFrame(uint8_t* const data, uint8_t len, uint16_t tmo_ms) // SX1276 doesn't have a Tx timeout
    {
        if (!is_in_lora_mode) {
            // fill FIFO, tx starts as soon as FIFO is not empty, the rest is written in HandleDio1Irq()
            // don't fill the FIFO completely, FifoLevel is invalid once FifoFull occurred
            _fsk_standby();
            memcpy(fsk_buf, data, FRAME_TX_RX_LEN);
            sx127x_fsk_whiten(fsk_buf, FRAME_TX_RX_LEN, SX127X_FSK_WHITENING_SEED);
            fsk_pos = SX127X_FSK_FIFO_SIZE - 1;
            WriteFifoFSK(fsk_buf, fsk_pos);
            fsk_state = FSK_STATE_TX;
            SetTx();
            return;
        }

        WriteBuffer(0, data, len);
        ClearIrqStatus(SX1276_IRQ_ALL);
        SetTx();
    }

    void SetToRx(void)
    {
        if (!is_in_lora_mode) {
            _fsk_standby();
            fsk_pos = 0;
            fsk_state = FSK_STATE_RX;
            SetRxContinuous();
            return;
        }

        uint16_t tmo_ms = 0;
        WriteRegister(SX1276_REG_FifoAddrPtr, 0);
        ClearIrqStatus(SX1276_IRQ_ALL);
        if (tmo_ms == 0) { // 0 = no timeout
            SetRxContinuous();
        } else {
            SetRxTimeout(((uint32_t)tmo_ms * 1000) / symbol_time_us);
            SetRxSingle();
        }
    }

    void SetToIdle(void)
    {
        if (!is_in_lora_mode) { // FSK flags can't be cleared by writing, register 0x12 is RxBw in FSK mode!
            _fsk_standby();
            return;
        }

        SetStandby();
        ClearIrqStatus(SX1276_IRQ_ALL);
    }

    void _fsk_standby(void)
    {
        fsk_state = FSK_STATE_IDLE; // first, so a DIO1 isr can't do spi in the middle of ours
        SetStandby();
        _clear_fifo_fsk();
    }

    // clearing the FIFO does not update FifoLevel (it is only updated by FIFO read/write operations),
    // so a stale high level would swallow the next rising edge, hence a dummy write and read, must be in Standby
    void _clear_fifo_fsk(void)
    {
        ClearFifoFSK();
        WriteRegister(SX1276_REG_Fifo, 0);
        ReadRegister(SX1276_REG_Fifo);
    }

    // the isrs use this to check the sync word or bind signature
    void ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len)
    {
        if (!is_in_lora_mode) { // frame was streamed into fsk_buf by the isrs
            memcpy(data, fsk_buf + offset, len);
            return;
        }

        Sx127xDriverBase::ReadBuffer(offset, data, len);
    }

    // DIO1 is FifoLevel, is called on both edges
    // Tx: tops up the FIFO once it has drained to the threshold
    // Rx: drains the FIFO whenever it has more than threshold bytes
    void HandleDio1Irq(void)
    {
        if (is_in_lora_mode) return;

        if (fsk_state == FSK_STATE_TX) {
            if (fsk_pos >= FRAME_TX_RX_LEN) return;
            if (GetIrqStatusFSK() & SX1276_FSK_IRQ2_FIFO_LEVEL) return; // not yet drained
            WriteFifoFSK(fsk_buf + fsk_pos, FRAME_TX_RX_LEN - fsk_pos);
            fsk_pos = FRAME_TX_RX_LEN;
            return;
        }

        if (fsk_state == FSK_STATE_RX) {
            // FifoLevel set means at least threshold + 1 bytes are available, so reading that many is safe
            // never drain the last bytes, PayloadReady is only a 31 ns pulse if the FIFO is empty at packet end, errata 3.1
            const uint8_t chunk = SX127X_FSK_FIFO_THRESHOLD + 1;
            while ((fsk_pos + chunk < FRAME_TX_RX_LEN) && (GetIrqStatusFSK() & SX1276_FSK_IRQ2_FIFO_LEVEL)) {
                if (fsk_pos == 0) GetRssiFSK(&fsk_rssi); // no packet rssi in FSK mode, so take it mid packet
                ReadFifoFSK(fsk_buf + fsk_pos, chunk);
                fsk_pos += chunk;
            }
        }
    }

    uint16_t GetAndClearIrqStatus(uint16_t IrqMask)
    {
        if (is_in_lora_mode) return Sx127xDriverBase::GetAndClearIrqStatus(IrqMask);

        // map FSK flags to LoRa irq bits, so the loops don't need to know
        // only look when a Tx or Rx is ongoing, in Idle the main loop may be using the spi
        if (fsk_state == FSK_STATE_TX) {
            if (!(GetIrqStatusFSK() & SX1276_FSK_IRQ2_PACKET_SENT)) return 0;
            fsk_state = FSK_STATE_IDLE;
            SetStandby(); // FSK stays in Tx after PacketSent
            return SX1276_IRQ_TX_DONE;
        } else if (fsk_state == FSK_STATE_RX) {
            if (!(GetIrqStatusFSK() & SX1276_FSK_IRQ2_PAYLOAD_READY)) return 0;
            // the rest of the frame is in the FIFO, fetch it
            ReadFifoFSK(fsk_buf + fsk_pos, FRAME_TX_RX_LEN - fsk_pos);
            sx127x_fsk_whiten(fsk_buf, FRAME_TX_RX_LEN, SX127X_FSK_WHITENING_SEED);
            fsk_state = FSK_STATE_IDLE;
            SetStandby();
            return SX1276_IRQ_RX_DONE;
        }

        return 0;
    }

    void GetPacketStatus(int8_t* const RssiSync, int8_t* const Snr)
    {
        if (!gconfig) { *RssiSync = -127; *Snr = 0; return; } // should not happen in practice

        int16_t rssi;
        if (is_in_lora_mode) {
            Sx127xDriverBase::GetPacketStatus(&rssi, Snr, low_frequency_mode);
        } else {
            rssi = fsk_rssi;
            *Snr = 0;
        }

        if (rssi > -1) rssi = -1; // we do not support values larger than this
        if (rssi < -127) rssi = -127; // we do not support values lower than this

        *RssiSync = rssi;
    }

    void SetRfFrequency(uint32_t RfFrequency)
    {
        Sx127xDriverBase::AfcSetRfFrequency(RfFrequency);
    }

    void HandleAFC(void)
    {
        if (!is_in_lora_mode) return; // no AFC in FSK, AfcDo() would mess with FSK registers

        AfcDo();
    }

    //-- RF power interface

    virtual void _rfpower_calc(int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm) = 0;

    //-- helper

    void _config_calc(void)
    {
        int8_t power_dbm = gconfig->Power_dbm;
        _rfpower_calc(power_dbm, &sx_power, &actual_power_dbm);

        if (gconfig->modeIsLora()) {
            uint8_t index = gconfig->LoraConfigIndex;
            if (index >= sizeof(Sx127xLoraConfiguration)/sizeof(Sx127xLoraConfiguration[0])) while(1){} // must not happen
            lora_configuration = &(Sx127xLoraConfiguration[index]);

            symbol_time_us = calc_symbol_time_us(lora_configuration->SpreadingFactor, lora_configuration->Bandwidth);
        } else {
            gfsk_configuration = &(Sx127xGfskConfiguration[0]);
        }
    }

    // cumbersome to calculate in general, so use hardcoded for a specific settings
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
    bool is_in_lora_mode; // tracks the actual chip mode, gconfig->is_lora is the requested mode
    int16_t fsk_rssi;
    int8_t power_dbm_last;

    // FSK frame streaming, frames are larger than the FIFO
    typedef enum {
        FSK_STATE_IDLE = 0,
        FSK_STATE_TX,
        FSK_STATE_RX,
    } FSK_STATE_ENUM;
    volatile uint8_t fsk_state;
    uint8_t fsk_buf[FRAME_TX_RX_LEN];
    volatile uint8_t fsk_pos;
    uint8_t low_frequency_mode;
    int8_t sx_power;
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

// SX1276 doesn't has BUSY
#ifndef SX_RESET
  #error SX must have a RESET pin!
#endif

// map the irq bits
typedef enum {
    SX_IRQ_TX_DONE = SX1276_IRQ_TX_DONE,
    SX_IRQ_RX_DONE = SX1276_IRQ_RX_DONE,
    SX_IRQ_TIMEOUT = SX1276_IRQ_RX_TIMEOUT,
    SX_IRQ_ALL     = SX1276_IRQ_ALL,
} SX_IRQ_ENUM;


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
#if defined DEVICE_HAS_I2C_DAC || defined DEVICE_HAS_INTERNAL_DAC_TWOCHANNELS
        rfpower_calc(power_dbm, sx_power, actual_power_dbm, &dac);
#elif defined POWER_USE_DEFAULT_RFPOWER_CALC
        sx1276_rfpower_calc_default(power_dbm, sx_power, actual_power_dbm, POWER_GAIN_DBM, POWER_SX1276_MAX);
#else
        sx1276_rfpower_calc(power_dbm, sx_power, actual_power_dbm);
#endif
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
        spi_setnop(0x00); // 0x00 = NOP
        sx_init_gpio();
        sx_dio_exti_isr_clearflag();
        sx_dio_init_exti_isroff();
#ifdef DEVICE_HAS_SX127x_FSK
        sx_dio1_exti_isr_clearflag();
        sx_dio1_init_exti_isroff();
#endif

        // no idea how long the SX1276 takes to boot up, so give it some good time
        delay_ms(300);
        _reset(); // this is super crucial ! was so for SX1280, is it also for the SX1276 ??

        // this is not nice, figure out where to place
#if defined DEVICE_HAS_I2C_DAC || defined DEVICE_HAS_INTERNAL_DAC_TWOCHANNELS
        dac.Init();
#endif

        SetStandby(); // should be in STDBY after reset
        delay_us(1000); // is this needed ????
    }

    //-- high level API functions

    void StartUp(tSxGlobalConfig* const global_config)
    {
        if (gconfig) return; // has been started up already

//XX        // this is not nice, figure out where to place
//XX#ifdef DEVICE_HAS_I2C_DAC
//XX        dac.Init();
//XX#endif

//XX        SetStandby(); // should be in STDBY after reset
//XX        delay_us(1000); // is this needed ????

        Configure(global_config);
        delay_us(125); // may not be needed
        sx_dio_enable_exti_isr();
#ifdef DEVICE_HAS_SX127x_FSK
        sx_dio1_enable_exti_isr();
#endif
    }

    //-- these are the API functions used in the loop

    void SendFrame(uint8_t* const data, uint8_t len, uint16_t tmo_ms)
    {
        sx_amp_transmit();
        Sx127xDriverCommon::SendFrame(data, len, tmo_ms);
        delay_us(125); // may not be needed
    }

    void SetToRx(void)
    {
        sx_amp_receive();
        Sx127xDriverCommon::SetToRx();
        delay_us(125); // may not be needed
    }
};


//-------------------------------------------------------
// Driver for SX2
//-------------------------------------------------------
#if defined DEVICE_HAS_DIVERSITY || defined DEVICE_HAS_DIVERSITY_SINGLE_SPI

#ifndef SX2_RESET
  #error SX2 must have a RESET pin!
#endif

// map the irq bits
typedef enum {
    SX2_IRQ_TX_DONE = SX1276_IRQ_TX_DONE,
    SX2_IRQ_RX_DONE = SX1276_IRQ_RX_DONE,
    SX2_IRQ_TIMEOUT = SX1276_IRQ_RX_TIMEOUT,
    SX2_IRQ_ALL     = SX1276_IRQ_ALL,
} SX2_IRQ_ENUM;


class Sx127xDriver2 : public Sx127xDriverCommon
{
  public:

    //-- interface to SPI peripheral

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

#ifndef DEVICE_HAS_DIVERSITY_SINGLE_SPI
    void SpiTransfer(uint8_t* dataout, uint8_t* datain, uint8_t len) override
    {
        spib_transfer(dataout, datain, len);
    }

    void SpiRead(uint8_t* datain, uint8_t len) override
    {
        spib_read(datain, len);
    }

    void SpiWrite(uint8_t* dataout, uint8_t len) override
    {
        spib_write(dataout, len);
    }
#else
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
#endif

    //-- RF power interface

    void _rfpower_calc(int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm) override
    {
#if defined DEVICE_HAS_I2C_DAC || defined DEVICE_HAS_INTERNAL_DAC_TWOCHANNELS
        rfpower_calc(power_dbm, sx_power, actual_power_dbm, &dac);
#elif defined POWER_USE_DEFAULT_RFPOWER_CALC
        sx1276_rfpower_calc_default(power_dbm, sx_power, actual_power_dbm, POWER_GAIN_DBM, POWER_SX1276_MAX);
#else
        sx1276_rfpower_calc(power_dbm, sx_power, actual_power_dbm);
#endif
    }

    //-- init API functions

    void _reset(void)
    {
        gpio_low(SX2_RESET);
        delay_ms(5); // datasheet says > 100 us
        gpio_high(SX2_RESET);
        delay_ms(50); // datasheet says 5 ms
    }

    void Init(void)
    {
        Sx127xDriverCommon::Init();

#ifndef DEVICE_HAS_DIVERSITY_SINGLE_SPI
        spib_init();
        spib_setnop(0x00); // 0x00 = NOP
#else
        // spi init done already by driver1
#endif
        sx2_init_gpio();
        sx2_dio_exti_isr_clearflag();
        sx2_dio_init_exti_isroff();
#ifdef DEVICE_HAS_SX127x_FSK
        sx2_dio1_exti_isr_clearflag();
        sx2_dio1_init_exti_isroff();
#endif

        // no idea how long the SX1276 takes to boot up, so give it some good time
        delay_ms(300);
        _reset(); // this is super crucial ! was so for SX1280, is it also for the SX1276 ??

        // this is not nice, figure out where to place
#if defined DEVICE_HAS_I2C_DAC || defined DEVICE_HAS_INTERNAL_DAC_TWOCHANNELS
        dac.Init();
#endif

        SetStandby(); // should be in STDBY after reset
        delay_us(1000); // is this needed ????
    }

    //-- high level API functions

    void StartUp(tSxGlobalConfig* const global_config)
    {
        if (gconfig) return; // has been started up already

        Configure(global_config);
        delay_us(125); // may not be needed
        sx2_dio_enable_exti_isr();
#ifdef DEVICE_HAS_SX127x_FSK
        sx2_dio1_enable_exti_isr();
#endif
    }

    //-- these are the API functions used in the loop

    void SendFrame(uint8_t* const data, uint8_t len, uint16_t tmo_ms)
    {
        sx2_amp_transmit();
        Sx127xDriverCommon::SendFrame(data, len, tmo_ms);
        delay_us(125); // may not be needed
    }

    void SetToRx(void)
    {
        sx2_amp_receive();
        Sx127xDriverCommon::SetToRx();
        delay_us(125); // may not be needed
    }
};

#endif


#endif // SX1276_DRIVER_H
