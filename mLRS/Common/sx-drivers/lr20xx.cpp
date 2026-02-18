//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// LR20XX standard interface
//*******************************************************
// contributed by JLP, OlliW42
//*******************************************************

#include "lr20xx.h"

// spi methods

void Lr20xxDriverBase::SpiRead(uint8_t* datain, uint8_t len)
{
uint8_t dummy = 0; // NOP

    while (len) {
        SpiTransfer(dummy, datain);
        datain++;
        len--;
    }
}

void Lr20xxDriverBase::SpiWrite(uint8_t* dataout, uint8_t len)
{
uint8_t dummy;

    while (len) {
        SpiTransfer(*dataout, &dummy);
        dataout++;
        len--;
    }
}

// low level methods

void Lr20xxDriverBase::WriteCommand(uint16_t opcode, uint8_t* data, uint8_t len)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer((uint8_t)((opcode & 0xFF00) >> 8), &_status1);
    SpiTransfer((uint8_t)(opcode & 0x00FF), &_status2);
    // note: the next 4 bytes give the uint32_t IrqStatus
    // we could read them in, but let's ignore them, since len is not guaranteed to be >= 4
    if (len > 0) SpiWrite(data, len);
    SpiDeselect();
}

void Lr20xxDriverBase::ReadCommand(uint16_t opcode, uint8_t* data, uint8_t len)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer((uint8_t)((opcode & 0xFF00) >> 8), &_status1);
    SpiTransfer((uint8_t)(opcode & 0x00FF), &_status2);
    if (opcode != LR20XX_CMD_GET_STATUS) { // not needed for get status
        SpiDeselect();
        WaitOnBusy();
        SpiSelect();
    }
    SpiRead(data, len);
    SpiDeselect();
}


void Lr20xxDriverBase::WriteRadioTxFifo(uint8_t* data, uint8_t len)
{
    WriteCommand(LR20XX_CMD_WRITE_RADIO_TX_FIFO, data, len);
}

void Lr20xxDriverBase::ReadRadioRxFifo(uint8_t* data, uint8_t len)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer((uint8_t)((LR20XX_CMD_READ_RADIO_RX_FIFO & 0xFF00) >> 8), &_status1);
    SpiTransfer((uint8_t)(LR20XX_CMD_READ_RADIO_RX_FIFO & 0x00FF), &_status2);
    SpiRead(data, len);
    SpiDeselect();
}

void Lr20xxDriverBase::WriteRegMem32(uint32_t addr, uint32_t* data, uint8_t len)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer((uint8_t)((LR20XX_CMD_WRITE_REG_MEM_32 & 0xFF00) >> 8), &_status1);
    SpiTransfer((uint8_t)(LR20XX_CMD_WRITE_REG_MEM_32 & 0x00FF), &_status2);
    addr <<= 8;
    SpiWrite((uint8_t*)&(addr), 3);
    if (len > 0) SpiWrite((uint8_t*)data, 4*len);
    SpiDeselect();
}

void Lr20xxDriverBase::WriteRegMemMask32(uint32_t addr, uint32_t mask, uint32_t data)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer((uint8_t)((LR20XX_CMD_WRITE_REG_MEM_MASK_32 & 0xFF00) >> 8), &_status1);
    SpiTransfer((uint8_t)(LR20XX_CMD_WRITE_REG_MEM_MASK_32 & 0x00FF), &_status2);
    addr <<= 8;
    SpiWrite((uint8_t*)&(addr), 3);
    SpiWrite((uint8_t*)&mask, 4);
    SpiWrite((uint8_t*)&data, 4);
    SpiDeselect();
}

void Lr20xxDriverBase::ReadRegMem32(uint32_t addr, uint32_t* data, uint8_t len)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer((uint8_t)((LR20XX_CMD_READ_REG_MEM_32 & 0xFF00) >> 8), &_status1);
    SpiTransfer((uint8_t)(LR20XX_CMD_READ_REG_MEM_32 & 0x00FF), &_status2);
    addr <<= 8;
    SpiWrite((uint8_t*)&(addr), 3);
    // TODO: is a deselect, wait, select block required ??
    if (len > 0) SpiRead((uint8_t*)data, 4*len);
    SpiDeselect();
}






// TODO: is this really doing the right thing ??
// Dow ealso want the IrqStatus ??
void Lr20xxDriverBase::GetStatus(uint8_t* Status1, uint8_t* Status2)
{
    WriteCommand(LR20XX_CMD_GET_STATUS); // don't need a response, so don't need to use ReadCommand

    *Status1 = _status1;
    *Status2 = _status2;
}

void Lr20xxDriverBase::GetLastStatus(uint8_t* Status1, uint8_t* Status2)
{
    *Status1 = _status1;
    *Status2 = _status2;
}

void Lr20xxDriverBase::GetVersion(uint8_t* FwMajor, uint8_t* FwMinor)
{
uint8_t buf[2];

    ReadCommand(LR20XX_CMD_GET_VERSION, buf, 2);

    *FwMajor = buf[0];
    *FwMinor = buf[1];
}

void Lr20xxDriverBase::GetErrors(uint16_t* ErrorStat)
{
uint8_t buf[4];

    ReadCommand(LR20XX_CMD_GET_ERRORS, buf, 4);

    *ErrorStat = ((uint16_t)buf[2] << 8) + buf[3];
}

void Lr20xxDriverBase::ClearErrors(void)
{
    WriteCommand(LR20XX_CMD_CLEAR_ERRORS);
}

void Lr20xxDriverBase::SetDioFunction(uint8_t Dio, uint8_t Func, uint8_t PullDrive)
{
uint8_t buf[2];

    buf[0] = Dio; // allowed values are 5 - 11
    buf[1] = ((Func & 0x07) << 3) + (PullDrive & 0x7); // TODO: is this correct ???

    WriteCommand(LR20XX_CMD_SET_DIO_FUNCTION, buf, 2);
}

void Lr20xxDriverBase::SetDioRfSwitchConfig(uint8_t Dio, uint8_t Config)
{
uint8_t buf[1];

    buf[0] = (Config & 0x1F);

    WriteCommand(LR20XX_CMD_SET_DIO_RF_SWITCH_CONFIG, buf, 1);
}

void Lr20xxDriverBase::SetDioIrqConfig(uint8_t Dio, uint32_t Irq)
{
uint8_t buf[5];

    buf[0] = Dio;
    buf[1] = (uint8_t)((Irq & 0xFF000000) >> 24);
    buf[2] = (uint8_t)((Irq & 0x00FF0000) >> 16);
    buf[3] = (uint8_t)((Irq & 0x0000FF00) >> 8);
    buf[4] = (uint8_t) (Irq & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_DIO_IRQ_CONFIG, buf, 5);
}

void Lr20xxDriverBase::ClearIrq(uint32_t IrqsToClear)
{
uint8_t buf[4];

    buf[0] = (uint8_t)((IrqsToClear & 0xFF000000) >> 24);
    buf[1] = (uint8_t)((IrqsToClear & 0x00FF0000) >> 16);
    buf[2] = (uint8_t)((IrqsToClear & 0x0000FF00) >> 8);
    buf[3] = (uint8_t) (IrqsToClear & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_DIO_IRQ_CONFIG, buf, 5);
}

uint32_t Lr20xxDriverBase::GetAndClearIrqStatus(void)
{
uint8_t buf[6];

    ReadCommand(LR20XX_CMD_GET_AND_CLEAR_IRQ_STATUS, buf, 6);

    return ((uint32_t)buf[2] << 24) + ((uint32_t)buf[3] << 16) + ((uint32_t)buf[4] << 8) + (uint32_t)buf[5];
}

void Lr20xxDriverBase::ConfigLfClock(uint8_t LfClock)
{
uint8_t buf[1];

    buf[0] = (LfClock & 0x03);

    WriteCommand(LR20XX_CMD_CONFIG_LF_CLOCK, buf, 1);
}

void Lr20xxDriverBase::SetTcxoMode(uint8_t Tune, uint32_t StartTime)
{
uint8_t buf[5];

    buf[0] = Tune;
    buf[1] = (uint8_t)((StartTime & 0xFF000000) >> 24);
    buf[2] = (uint8_t)((StartTime & 0x00FF0000) >> 16);
    buf[3] = (uint8_t)((StartTime & 0x0000FF00) >> 8);
    buf[4] = (uint8_t) (StartTime & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_TCXO_MODE, buf, 5);
}

void Lr20xxDriverBase::SetRegMode(uint8_t SimoUsage)
{
uint8_t buf[1];

    buf[0] = SimoUsage;

    WriteCommand(LR20XX_CMD_SET_REG_MODE, buf, 1);
}

void Lr20xxDriverBase::Calibrate(uint8_t BlocksToCalibrate)
{
uint8_t buf[1];

    buf[0] = BlocksToCalibrate;

    WriteCommand(LR20XX_CMD_CALIBRATE, buf, 1);
}

void Lr20xxDriverBase::CalibFE(uint16_t Freq1, uint16_t Freq2, uint16_t Freq3)
{
uint8_t buf[6];

    buf[0] = (uint8_t)((Freq1 & 0xFF00) >> 8);
    buf[1] = (uint8_t) (Freq1 & 0x00FF);
    buf[2] = (uint8_t)((Freq2 & 0xFF00) >> 8);
    buf[3] = (uint8_t) (Freq2 & 0x00FF);
    buf[4] = (uint8_t)((Freq3 & 0xFF00) >> 8);
    buf[5] = (uint8_t) (Freq3 & 0x00FF);

    WriteCommand(LR20XX_CMD_CALIB_FE, buf, 6);
}

void Lr20xxDriverBase::SetStandby(uint8_t StandbyMode)
{
uint8_t buf[1];

    buf[0] = StandbyMode;

    WriteCommand(LR20XX_CMD_SET_STANDBY, buf, 1);
}

void Lr20xxDriverBase::SetFs(void)
{
    WriteCommand(LR20XX_CMD_SET_FS);
}

void Lr20xxDriverBase::SetRfFrequency(uint32_t RfFreq)
{
uint8_t buf[4];

    buf[0] = (uint8_t)((RfFreq & 0xFF000000) >> 24);
    buf[1] = (uint8_t)((RfFreq & 0x00FF0000) >> 16);
    buf[2] = (uint8_t)((RfFreq & 0x0000FF00) >> 8);
    buf[3] = (uint8_t) (RfFreq & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_RF_FREQUENCY, buf, 4);
}

void Lr20xxDriverBase::SetRxPath(uint8_t RxPath, uint8_t RxBoost)
{
uint8_t buf[2];

    buf[0] = (RxPath & 0x01);
    buf[1] = (RxBoost & 0x07);

    WriteCommand(LR20XX_CMD_SET_RX_PATH, buf, 2);
}

void Lr20xxDriverBase::SetPaConfig(uint8_t PaSel, uint8_t PaLfMode, uint8_t PaLfDutyCycle, uint8_t PaLfSlices, uint8_t PaHfDutyCycle)
{
uint8_t buf[3];

    buf[0] = (PaLfMode & 0x03) + ((PaSel & 0x01) << 7);
    buf[1] = (PaLfSlices & 0x0F) + ((PaLfDutyCycle & 0x0F) << 4);
    buf[2] = (PaHfDutyCycle & 0x0F);

    WriteCommand(LR20XX_CMD_SET_PA_CONFIG, buf, 3);
}

void Lr20xxDriverBase::SetTxParams(uint8_t Power, uint8_t RampTime)
{
uint8_t buf[2];

    buf[0] = Power;
    buf[1] = RampTime;

    WriteCommand(LR20XX_CMD_SET_TX_PARAMS, buf, 2);
}

void Lr20xxDriverBase::SetRssiCalibration(uint8_t RxPathHf, uint8_t RxPathLf, uint8_t* table)
{
/* // TODO
uint8_t buf[1 + 2*(3*81)];

    buf[0] = ((RxPathHf & 0x01) << 1) + (RxPathLf & 0x01);
    for (uint8_t i = 0; i < 2*(3*81); i++) buf[i+1] = table[i];

    WriteCommand(LR20XX_CMD_SET_RSSI_CALIBRATION, buf, 1 + 2*(3*81)); */
}

void Lr20xxDriverBase::SetRxTxFallbackMode(uint8_t FallbackMode)
{
uint8_t buf[1];

    buf[0] = FallbackMode;

    WriteCommand(LR20XX_CMD_SET_RXTX_FALLBACK_MODE, buf, 1);
}

void Lr20xxDriverBase::SetPacketType(uint8_t PacketType)
{
uint8_t buf[1];

    buf[0] = PacketType;

    WriteCommand(LR20XX_CMD_SET_PACKET_TYPE, buf, 1);
}

uint8_t Lr20xxDriverBase::GetPacketType(void)
{
uint8_t buf[3];

    ReadCommand(LR20XX_CMD_GET_PACKET_TYPE, buf, 3);

    return buf[2];
}

void Lr20xxDriverBase::ResetRxStats(void)
{
    WriteCommand(LR20XX_CMD_RESET_RX_STATS);
}

void Lr20xxDriverBase::SetRx(uint32_t RxTimeout) // 24 bits only, similar to sx126x
{
uint8_t buf[3];

    buf[0] = (uint8_t)((RxTimeout & 0x00FF0000) >> 16);
    buf[1] = (uint8_t)((RxTimeout & 0x0000FF00) >> 8);
    buf[2] = (uint8_t) (RxTimeout & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_RX, buf, 3);
}

void Lr20xxDriverBase::SetRx(void) // timeout set with SetDefaultRxTxTimeout() is used
{
    WriteCommand(LR20XX_CMD_SET_RX);
}

void Lr20xxDriverBase::SetTx(uint32_t TxTimeout) // 24 bits only, similar to sx126x
{
uint8_t buf[3];

    buf[0] = (uint8_t)((TxTimeout & 0x00FF0000) >> 16);
    buf[1] = (uint8_t)((TxTimeout & 0x0000FF00) >> 8);
    buf[2] = (uint8_t) (TxTimeout & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_TX, buf, 3);
}

void Lr20xxDriverBase::SetTx(void) // timeout set with SetDefaultRxTxTimeout() is used
{
    WriteCommand(LR20XX_CMD_SET_TX);
}

void Lr20xxDriverBase::SelPa(uint8_t PaSel)
{
uint8_t buf[1];

    buf[0] = (PaSel & 0x01);

    WriteCommand(LR20XX_CMD_SEL_PA, buf, 1);
}

uint16_t Lr20xxDriverBase::GetRxPktLength(void)
{
uint8_t buf[4];

    ReadCommand(LR20XX_CMD_GET_RX_PKT_LENGTH, buf, 4);

    return ((uint16_t)buf[2] << 8) + (uint16_t)buf[3];
}

void Lr20xxDriverBase::SetDefaultRxTxTimeout(uint32_t RxTimeout, uint32_t TxTimeout) // 24 bits only
{
uint8_t buf[6];

    buf[0] = (uint8_t)((RxTimeout & 0x00FF0000) >> 16);
    buf[1] = (uint8_t)((RxTimeout & 0x0000FF00) >> 8);
    buf[2] = (uint8_t) (RxTimeout & 0x000000FF);

    buf[3] = (uint8_t)((TxTimeout & 0x00FF0000) >> 16);
    buf[4] = (uint8_t)((TxTimeout & 0x0000FF00) >> 8);
    buf[5] = (uint8_t) (TxTimeout & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_DEFAULT_RX_TX_TIMEOUT, buf, 6);
}

void Lr20xxDriverBase::SetAgcGainManual(uint8_t GainStep)
{
uint8_t buf[1];

    buf[0] = (GainStep & 0x0F);

    WriteCommand(LR20XX_CMD_SET_AGC_GAIN_MANUAL, buf, 1);
}




void Lr20xxDriverBase::SetLoraModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate, uint8_t LowDataRateOptimize)
{
uint8_t buf[2];

    buf[0] = ((SpreadingFactor & 0x0F) << 4) + (Bandwidth & 0x0F);
    buf[1] = ((CodingRate & 0x0F) << 4) + ((CodingRate & 0x03) << 2) + (LowDataRateOptimize & 0x03);

    WriteCommand(LR20XX_CMD_SET_LORA_MODULATION_PARAMS, buf, 1);
}

void Lr20xxDriverBase::SetLoraPacketParams(uint16_t PreambleLength, uint8_t PayloadLength, uint8_t HeaderType, uint8_t Crc, uint8_t InvertIQ)
{
uint8_t buf[4];

    buf[0] = (uint8_t)((PreambleLength & 0xFF00) >> 8);
    buf[1] = (uint8_t) (PreambleLength & 0x00FF);
    buf[2] = PayloadLength;
    buf[3] = ((HeaderType & 0x01) << 2) + ((Crc & 0x01) << 1) + (InvertIQ & 0x01);

    WriteCommand(LR20XX_CMD_SET_LORA_PACKET_PARAMS, buf, 4);
}

void Lr20xxDriverBase::GetLoraRxStats(
      uint16_t* pkt_rx, uint16_t* pkt_crc_error, uint16_t* header_crc_error, uint16_t* false_synch)
{
uint8_t buf[10];

     ReadCommand(LR20XX_CMD_GET_LORA_RX_STATS, buf, 10);

     *pkt_rx = ((uint16_t)buf[2] << 8) + buf[3];
     *pkt_crc_error = ((uint16_t)buf[4] << 8) + buf[5];
     *header_crc_error = ((uint16_t)buf[6] << 8) + buf[7];
     *false_synch = ((uint16_t)buf[8] << 8) + buf[9];
}


void Lr20xxDriverBase::GetPacketStatus(
      int16_t* Rssi, int16_t* RssiSignal, int8_t* Snr,
      uint8_t* Crc, uint8_t* CR, uint16_t* PktLen, uint8_t* Detector)
{
uint8_t buf[8];

     ReadCommand(LR20XX_CMD_GET_LORA_PACKET_STATUS, buf, 8);

     *Crc = ((buf[2] & 0x10) << 1);
     *CR = (buf[2] & 0x0F);
     *PktLen = buf[3];
     *Snr = buf[4];
     *Rssi = ((int16_t)buf[5] << 1) + (((int16_t)buf[7] & 0x02) >> 1);
     *RssiSignal = ((int16_t)buf[6] << 1) + ((int16_t)buf[7] & 0x01);
     *Detector = (buf[7] & 0x3D) >> 2;
}

void Lr20xxDriverBase::GetPacketStatus(int16_t* Rssi, int16_t* RssiSignal, int8_t* Snr)
{
 uint8_t buf[8];

      ReadCommand(LR20XX_CMD_GET_LORA_PACKET_STATUS, buf, 8);

      *Snr = buf[4];
      *Rssi = ((int16_t)buf[5] << 1) + (((int16_t)buf[7] & 0x02) >> 1);
      *RssiSignal = ((int16_t)buf[6] << 1) + ((int16_t)buf[7] & 0x01);
}



// FIFO methods

uint16_t Lr20xxDriverBase::GetRxFifoLevel(void)
{
uint8_t buf[4];

    ReadCommand(LR20XX_CMD_GET_RX_FIFO_LEVEL, buf, 4);

    return ((uint16_t)buf[2] << 8) + (uint16_t)buf[3];
}

uint16_t Lr20xxDriverBase::GetTxFifoLevel(void)
{
uint8_t buf[4];

    ReadCommand(LR20XX_CMD_GET_TX_FIFO_LEVEL, buf, 4);

    return ((uint16_t)buf[2] << 8) + (uint16_t)buf[3];
}

void Lr20xxDriverBase::ClearRxFifo(void)
{
    WriteCommand(LR20XX_CMD_CLEAR_RX_FIFO);
}

void Lr20xxDriverBase::ClearTxFifo(void)
{
    WriteCommand(LR20XX_CMD_CLEAR_TX_FIFO);
}


// GFSK methods

void Lr20xxDriverBase::SetModulationParamsFSK(uint32_t BitRate, uint8_t PulseShape, uint8_t Bandwidth, uint32_t Fdev_hz)
{
uint8_t buf[9];

    buf[0] = (uint8_t)((BitRate & 0xFF000000) >> 24);
    buf[1] = (uint8_t)((BitRate & 0x00FF0000) >> 16);
    buf[2] = (uint8_t)((BitRate & 0x0000FF00) >> 8);
    buf[3] = (uint8_t) (BitRate & 0x000000FF);
    buf[4] = PulseShape;
    buf[5] = Bandwidth;
    buf[6] = (uint8_t)((Fdev_hz & 0x00FF0000) >> 16);
    buf[7] = (uint8_t)((Fdev_hz & 0x0000FF00) >> 8);
    buf[8] = (uint8_t) (Fdev_hz & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_FSK_MODULATION_PARAMS, buf, 9);
}

void Lr20xxDriverBase::SetPacketParamsFSK(
      uint16_t PreambleLength, uint8_t PreambleDetectorLength,
      uint8_t long_preamble_mode, uint8_t pld_lenUnit, uint8_t addr_comp,
      uint8_t PacketFormat, uint16_t PayloadLength, uint8_t Crc,
      uint8_t dc_free)
{
uint8_t buf[7];

    buf[0] = (uint8_t)((PreambleLength & 0xFF00) >> 8);
    buf[1] = (uint8_t) (PreambleLength & 0x00FF);
    buf[2] = PreambleDetectorLength;
    buf[3] = ((long_preamble_mode & 0x01) << 5) + ((pld_lenUnit & 0x01) << 4) +
             ((addr_comp & 0x02) << 2) + (PacketFormat & 0x03);
    buf[4] = (uint8_t)((PayloadLength & 0xFF00) >> 8);
    buf[5] = (uint8_t) (PayloadLength & 0x00FF);
    buf[6] = ((Crc & 0x0F) >> 4) + (dc_free & 0x0F);

    WriteCommand(LR20XX_CMD_SET_FSK_PACKET_PARAMS, buf, 7);
}

void Lr20xxDriverBase::SetWhiteningParamsFSK(uint8_t WhitenType, uint16_t Init)
{
uint8_t buf[2];

    buf[0] = ((WhitenType & 0x0F) << 4) + (uint8_t)(Init & 0x0FFF);
    buf[1] = Init;

    WriteCommand(LR20XX_CMD_SET_FSK_WHITENING_PARAMS, buf, 2);
}

void Lr20xxDriverBase::SetCrcParamsFSK(uint32_t Polynom, uint32_t Init)
{
uint8_t buf[8];

    buf[0] = (uint8_t)((Polynom & 0xFF000000) >> 24);
    buf[1] = (uint8_t)((Polynom & 0x00FF0000) >> 16);
    buf[2] = (uint8_t)((Polynom & 0x0000FF00) >> 8);
    buf[3] = (uint8_t) (Polynom & 0x000000FF);

    buf[4] = (uint8_t)((Init & 0xFF000000) >> 24);
    buf[5] = (uint8_t)((Init & 0x00FF0000) >> 16);
    buf[6] = (uint8_t)((Init & 0x0000FF00) >> 8);
    buf[7] = (uint8_t) (Init & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_FSK_CRC_PARAMS, buf, 8);
}

void Lr20xxDriverBase::SetSyncWordFSK(uint64_t SyncWord, uint8_t bit_order, uint8_t nb_bits)
{
uint8_t buf[9];

    buf[0] = (uint8_t)((SyncWord & 0xFF00000000000000) >> 56);
    buf[1] = (uint8_t)((SyncWord & 0x00FF000000000000) >> 48);
    buf[2] = (uint8_t)((SyncWord & 0x0000FF0000000000) >> 40);
    buf[3] = (uint8_t)((SyncWord & 0x000000FF00000000) >> 32);
    buf[4] = (uint8_t)((SyncWord & 0x00000000FF000000) >> 24);
    buf[5] = (uint8_t)((SyncWord & 0x0000000000FF0000) >> 16);
    buf[6] = (uint8_t)((SyncWord & 0x000000000000FF00) >> 8);
    buf[7] = (uint8_t) (SyncWord & 0x00000000000000FF);

    buf[8] = ((nb_bits & 0x01) << 7) + (bit_order & 0x7F);

    WriteCommand(LR20XX_CMD_SET_FSK_SYNC_WORD, buf, 9);
}

void Lr20xxDriverBase::SetAddressFSK(uint8_t addr_node, uint8_t addr_bcast)
{
uint8_t buf[2];

    buf[0] = addr_node;
    buf[1] = addr_bcast;

    WriteCommand(LR20XX_CMD_SET_FSK_ADDRESS, buf, 2);
}

void Lr20xxDriverBase::GetRxStatsFSK(
      uint16_t* pkt_rx, uint16_t* pkt_crc_error, uint16_t* len_error, uint16_t* pbl_det,
      uint16_t* sync_ok, uint16_t* sync_fail, uint16_t* timeout)
{
uint8_t buf[16];

    ReadCommand(LR20XX_CMD_GET_FSK_RX_STATS, buf, 16);

    *pkt_rx = ((uint16_t)buf[2] << 8) + buf[3];
    *pkt_crc_error = ((uint16_t)buf[4] << 8) + buf[5];
    *len_error = ((uint16_t)buf[6] << 8) + buf[7];
    *pbl_det = ((uint16_t)buf[8] << 8) + buf[9];
    *sync_ok = ((uint16_t)buf[10] << 8) + buf[11];
    *sync_fail = ((uint16_t)buf[12] << 8) + buf[13];
    *timeout = ((uint16_t)buf[14] << 8) + buf[15];
}

void Lr20xxDriverBase::GetPacketStatusFSK(
      uint16_t* PktLen, uint16_t* RssiAvg, uint16_t* RssiSync, uint8_t* AddrMatchBcast, uint8_t* AddrMatchNode, uint8_t* Lqi)
{
uint8_t buf[8];

    ReadCommand(LR20XX_CMD_GET_FSK_PACKET_STATUS, buf, 8);

    *PktLen = ((uint16_t)buf[2] << 8) + buf[3];
    *RssiAvg = ((uint16_t)buf[4] << 1) + ((buf[6] & 0x04) >> 2);
    *RssiSync = ((uint16_t)buf[5] << 1) + (buf[6] & 0x01);
    *AddrMatchBcast = (buf[6] & 0x20) >> 5;
    *AddrMatchNode = (buf[6] & 0x10) >> 4;
    *Lqi = buf[7];
}

void Lr20xxDriverBase::GetPacketStatusFSK(uint16_t* RssiAvg, uint16_t* RssiSync, uint8_t* Lqi)
{
uint8_t buf[8];

    ReadCommand(LR20XX_CMD_GET_FSK_PACKET_STATUS, buf, 8);

    *RssiAvg = ((uint16_t)buf[4] << 1) + ((buf[6] & 0x04) >> 2);
    *RssiSync = ((uint16_t)buf[5] << 1) + (buf[6] & 0x01);
    *Lqi = buf[7];
}


