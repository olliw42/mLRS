//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// SX127x standard interface
//*******************************************************

#include "sx127x.h"


// low level

void Sx127xDriverBase::WriteRegister(uint16_t adr, uint8_t* data, uint8_t len)
{
uint8_t in_buf[SX127X_SPI_BUF_SIZE];

    SpiSelect();
    SpiTransfer((adr & 0x7F) | 0x80); // write bit set
    SpiTransfer(data, in_buf, len);
    SpiDeselect();
}


void Sx127xDriverBase::ReadRegister(uint16_t adr, uint8_t* data, uint8_t len)
{
uint8_t out_buf[SX127X_SPI_BUF_SIZE];

    for (uint8_t i = 0; i < len; i++) out_buf[i] = 0; // NOP

    SpiSelect();
    SpiTransfer((adr & 0x7F) | 0x00); // write bit cleared to read
    SpiTransfer(out_buf, data, len);
    SpiDeselect();
}


void Sx127xDriverBase::ReadWriteRegister(uint16_t adr, uint8_t mask, uint8_t data)
{
    uint8_t reg = ReadRegister(adr);
    reg &=~ mask;
    reg |= data;
    WriteRegister(adr, reg);
}


void Sx127xDriverBase::WriteBuffer(uint8_t offset, uint8_t* data, uint8_t len)
{
    WriteRegister(SX1276_REG_FifoAddrPtr, offset);
    WriteRegister(SX1276_REG_Fifo, data, len);
}


void Sx127xDriverBase::ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len)
{
    WriteRegister(SX1276_REG_FifoAddrPtr, offset);
    ReadRegister(SX1276_REG_Fifo, data, len);
}


// common

uint8_t Sx127xDriverBase::GetStatus(void)
{
    return ReadRegister(SX1276_REG_ModemStat);
}


void Sx127xDriverBase::SetSleep(void)
{
    // 7 LongRangeMode, 6 AccessSharedReg, 3 LowFrequencyModeOn, 2-0 Mode
    // we assume we are not in sleep, and do not want access to FSK or LF test registers
    WriteRegister(SX1276_REG_OpMode, SX1276_MODE_SLEEP);
}


void Sx127xDriverBase::SetStandby(void)
{
    // 7 LongRangeMode, 6 AccessSharedReg, 3 LowFrequencyModeOn, 2-0 Mode
    // ReadWriteRegister(SX1276_REG_OpMode, 0x07, SX1276_MODE_STDBY);
    // we assume we are not in sleep, and do not want access to FSK or LF test registers
    WriteRegister(SX1276_REG_OpMode, SX1276_MODE_STDBY);
}


void Sx127xDriverBase::SetOperationMode(uint8_t PacketType, uint8_t LowFrequencyMode)
{
    SetSleep(); // must be in sleep to switch to LoRa mode
    ReadWriteRegister(SX1276_REG_OpMode, 0x84, PacketType | LowFrequencyMode);
    SetStandby(); // return to normal
}


void Sx127xDriverBase::SetRfFrequency(uint32_t RfFrequency) // 23 bits only
{
uint8_t buf[3];

    buf[0] = (uint8_t)((RfFrequency & 0xFF0000) >> 16);
    buf[1] = (uint8_t)((RfFrequency & 0x00FF00) >> 8);
    buf[2] = (uint8_t)(RfFrequency & 0x0000FF);

    WriteRegister(SX1276_REG_FrMsb, buf, 3); // SX1276_REG_FrMsb, SX1276_REG_FrMid, SX1276_REG_FrLsb are consecutive
}


void Sx127xDriverBase::SetLnaParams(uint8_t LnaGain, uint8_t LnaBoostHf)
{
    // 7-5 LnaGain, 4-3 LnaBoostLf, 1-0 LnaBoostHf
    WriteRegister(SX1276_REG_Lna, LnaGain | SX1276_LNA_BOOST_LF_DEFAULT | LnaBoostHf);
}


void Sx127xDriverBase::OptimizeSensitivity(uint8_t Bandwidth)
{
    // see errata SX1276_77_8_ErrataNote_1.1_STD.pdf, 2.1, page 4

    if (Bandwidth == SX1276_LORA_BW_500) {
        WriteRegister(SX1276_REG_HighBWOptimize1, 0x02); // reg 0x36
        WriteRegister(SX1276_REG_HighBWOptimize2, 0x64); // reg 0x3A
    } else {
        WriteRegister(SX1276_REG_HighBWOptimize1, 0x03); // 0x36
    }
}


void Sx127xDriverBase::OptimizeReceiverResponse(uint8_t Bandwidth)
{
    // see errata SX1276_77_8_ErrataNote_1.1_STD.pdf, 2.3, page 5

    //TODO: offset Frf ???

    if (Bandwidth == SX1276_LORA_BW_500) {
        ReadWriteRegister(SX1276_REG_DetectOptimize, 0x80, SX1276_LORA_AUTOMATIC_IF_ON);
    } else {
        ReadWriteRegister(SX1276_REG_DetectOptimize, 0x80, SX1276_LORA_AUTOMATIC_IF_OFF);
    }

    switch (Bandwidth) {
    case SX1276_LORA_BW_7p8:
        WriteRegister(SX1276_REG_0x2F, 0x48);
        break;
    case SX1276_LORA_BW_10p4:
    case SX1276_LORA_BW_15p6:
    case SX1276_LORA_BW_20p8:
    case SX1276_LORA_BW_31p25:
    case SX1276_LORA_BW_41p7:
        WriteRegister(SX1276_REG_0x2F, 0x44);
        break;
    case SX1276_LORA_BW_62p5:
    case SX1276_LORA_BW_125:
    case SX1276_LORA_BW_250:
        WriteRegister(SX1276_REG_0x2F, 0x40);
        break;
    }

    if (Bandwidth != SX1276_LORA_BW_500) {
        WriteRegister(SX1276_REG_0x30, 0x00);
    }
}


void Sx127xDriverBase::SetBufferBaseAddress(uint8_t txBaseAdress, uint8_t rxBaseAdress)
{
    WriteRegister(SX1276_REG_FifoTxBaseAddr, txBaseAdress);
    WriteRegister(SX1276_REG_FifoRxBaseAddr, rxBaseAdress);
}


void Sx127xDriverBase::SetModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate)
{
    // 7-4 Bw, 3-1 CodingRate, 0 ImplicitHeaderModeOn
    if (SpreadingFactor == SX1276_LORA_SF6) {
        // special treatment for SF6, see datasheet, p.27,  for SF6 the header must be implicit
        WriteRegister(SX1276_REG_ModemConfig1, Bandwidth | CodingRate | SX1276_LORA_HEADER_IMPLICIT);
    } else {
        ReadWriteRegister(SX1276_REG_ModemConfig1, 0xFE, Bandwidth | CodingRate);
    }

    // 7-4 SpreadingFactor, 3 TxContinuousMode, 2 RxPayloadCrcOn, 1-0 SymbTimeout(9:8)
    if (SpreadingFactor == SX1276_LORA_SF6) {
        // special treatment for SF6, see datasheet, p.27, for SF6 the crc must be disabled
        ReadWriteRegister(SX1276_REG_ModemConfig2, 0xF4, SpreadingFactor| SX1276_LORA_CRC_DISABLE);
    } else {
        ReadWriteRegister(SX1276_REG_ModemConfig2, 0xF0, SpreadingFactor);
    }

    // special treatment for SF6, see datasheet, p.27
    if (SpreadingFactor == SX1276_LORA_SF6) {
          // 7 AutomaticIFOn, 2-0 DetectionOptimize
          ReadWriteRegister(SX1276_REG_DetectOptimize, 0x07, SX1276_LORA_DETECTION_OPTIMIZE_SF_6);
          WriteRegister(SX1276_REG_DetectionThreshold, SX1276_LORA_DETECTION_TRESHOLD_SF_6);
    } else {
          // 7 AutomaticIFOn, 2-0 DetectionOptimize
          ReadWriteRegister(SX1276_REG_DetectOptimize, 0x07, SX1276_LORA_DETECTION_OPTIMIZE_SF_7_12);
          WriteRegister(SX1276_REG_DetectionThreshold, SX1276_LORA_DETECTION_TRESHOLD_SF_7_12);
    }
}


void Sx127xDriverBase::SetPacketParams(uint8_t PreambleLength, uint8_t HeaderType, uint8_t PayloadLength, uint8_t Crc, uint8_t InvertIQRxTx)
{
    // preamble length can be > 255, but we don't support this
    WriteRegister(SX1276_REG_PreambleMsb, 0);
    WriteRegister(SX1276_REG_PreambleLsb, PreambleLength);

    // 7-4 Bw, 3-1 CodingRate, 0 ImplicitHeaderModeOn
    // ATTENTION: for SF6 the header must be implicit
    ReadWriteRegister(SX1276_REG_ModemConfig1, 0x01, HeaderType);

    // 7-4 SpreadingFactor, 3 TxContinuousMode, 2 RxPayloadCrcOn, 1-0 SymbTimeout(9:8)
    // ATTENTION: for SF6 the crc must be off
    ReadWriteRegister(SX1276_REG_ModemConfig2, 0x04, Crc);

    WriteRegister(SX1276_REG_PayloadLength, PayloadLength);

    // 6 InvertIQ RX, 0 InvertIQ TX
    ReadWriteRegister(SX1276_REG_InvertIQ, 0x41, InvertIQRxTx);

    //RegInvertIQ2         ????????????????????????  really? how to deal with rx and tx?
    // Set to 0x19 for inverted IQ
    //WriteRegister(SX1276_REG_InvertIQ2, (InvertIQRxTx) ? SX1276_LORA_IQ2_INVERTED : SX1276_LORA_IQ2_NORMAL);

    // ELRS does not use this,
    // also ELRS does not set InvertIQ TX ?!?!?
}


void Sx127xDriverBase::SetDioIrqParams(uint8_t IrqMask, uint8_t Dio0Mask, uint8_t Dio1Mask)
{
    // setting a bit masks the corresponding IRQ in RegIrqFlags
    WriteRegister(SX1276_REG_IrqFlagsMask, ~IrqMask);

    // 7-6 Dio0Mapping, 5-4 Dio1Mapping, 3-2 Dio2Mapping, 1-0 Dio3Mapping
    WriteRegister(SX1276_REG_DioMapping1, Dio0Mask | Dio1Mask);

    // 7-6 Dio4Mapping, 5-4 Dio5Mapping, 0 MapPreambleDetect
    WriteRegister(SX1276_REG_DioMapping2, 0);
}


uint16_t Sx127xDriverBase::GetIrqStatus(void)
{
    return ReadRegister(SX1276_REG_IrqFlags);
}


void Sx127xDriverBase::ClearIrqStatus(uint8_t IrqMask)
{
    // writing a 1 clears the IRQ
    WriteRegister(SX1276_REG_IrqFlags, IrqMask);
}


// Tx

void Sx127xDriverBase::SetPowerParams(uint8_t PaSelect, uint8_t MaxPower, uint8_t Power, uint8_t RampTime)
{
    // 7 PaSelect, 6-4 MaxPower, 3-0 OutputPower
    WriteRegister(SX1276_REG_PaConfig, PaSelect | MaxPower | (Power & 0x0F));

    // 3-0 PaRamp(3:0)
    ReadWriteRegister(SX1276_REG_PaRamp, 0x0F, RampTime);
}


void Sx127xDriverBase::SetTx(void)
{
    // 7 LongRangeMode, 6 AccessSharedReg, 3 LowFrequencyModeOn, 2-0 Mode
    // we assume we are not in sleep, and do not want access to FSK or LF test registers
    WriteRegister(SX1276_REG_OpMode, SX1276_MODE_TX);
}


// Rx

void Sx127xDriverBase::SetRxSingle(void)
{
    // 7 LongRangeMode, 6 AccessSharedReg, 3 LowFrequencyModeOn, 2-0 Mode
    // we assume we are not in sleep, and do not want access to FSK or LF test registers
    WriteRegister(SX1276_REG_OpMode, SX1276_MODE_RX_SINGLE);
}


void Sx127xDriverBase::SetRxContinuous(void)
{
    // 7 LongRangeMode, 6 AccessSharedReg, 3 LowFrequencyModeOn, 2-0 Mode
    // we assume we are not in sleep, and do not want access to FSK or LF test registers
    WriteRegister(SX1276_REG_OpMode, SX1276_MODE_RX_CONTINUOUS);
}


void Sx127xDriverBase::SetRxTimeout(uint16_t tmo_symbols)
{
    // 7-4 SpreadingFactor, 3 TxContinuousMode, 2 RxPayloadCrcOn, 1-0 SymbTimeout(9:8)
    ReadWriteRegister(SX1276_REG_ModemConfig2, 0x03, (tmo_symbols >> 8));
    WriteRegister(SX1276_REG_SymbTimeoutLsb, (tmo_symbols & 0x00FF));
}


void Sx127xDriverBase::GetPacketStatus(int8_t* RssiSync, int8_t* Snr)
{
    *RssiSync = -157 + ReadRegister(SX1276_REG_PktRssiValue);

    *Snr = (int8_t)ReadRegister(SX1276_REG_PktSnrValue) / 4;
}


void Sx127xDriverBase::GetRxBufferStatus(uint8_t* rxPayloadLength, uint8_t* rxStartBufferPointer)
{
    *rxPayloadLength = ReadRegister(SX1276_REG_RxNbBytes);
    *rxStartBufferPointer = ReadRegister(SX1276_REG_FifoRxCurrentAddr);
}


// auxiliary

uint8_t Sx127xDriverBase::GetFirmwareRev(void)
{
    return ReadRegister(SX1276_REG_Version);
}


void Sx127xDriverBase::SetSyncWord(uint8_t SyncWord)
{
    WriteRegister(SX1276_REG_SyncWord, SyncWord);
}





