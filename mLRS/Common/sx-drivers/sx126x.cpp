//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// SX126x standard interface
//*******************************************************
// contributed by jinchuuriki
//*******************************************************

#include "sx126x.h"


// low level

void Sx126xDriverBase::WriteCommand(uint8_t opcode, uint8_t* data, uint8_t len)
{
uint8_t in_buf[SX126X_SPI_BUF_SIZE];

    WaitOnBusy();
    SpiSelect();
    SpiTransfer(opcode, &_status);
    if (len > 0) SpiTransfer(data, in_buf, len);
    SpiDeselect();
    SetDelay(12); // semtech driver says 12 us
}


void Sx126xDriverBase::ReadCommand(uint8_t opcode, uint8_t* data, uint8_t len)
{
uint8_t out_buf[SX126X_SPI_BUF_SIZE];

    for (uint8_t i = 0; i < len; i++) out_buf[i] = 0; // NOP

    WaitOnBusy();
    SpiSelect();
    SpiTransfer(opcode, &_status);
    SpiTransfer(0); // NOP
    SpiTransfer(out_buf, data, len);
    SpiDeselect();
    // no delay according to semtech driver
}


void Sx126xDriverBase::WriteRegister(uint16_t adr, uint8_t* data, uint8_t len)
{
uint8_t in_buf[SX126X_SPI_BUF_SIZE];

    WaitOnBusy();
    SpiSelect();
    SpiTransfer(SX126X_CMD_WRITE_REGISTER, &_status);
    SpiTransfer((adr & 0xFF00) >> 8);
    SpiTransfer(adr & 0x00FF);
    SpiTransfer(data, in_buf, len);
    SpiDeselect();
    SetDelay(12); // semtech driver says 12 us
}


void Sx126xDriverBase::ReadRegister(uint16_t adr, uint8_t* data, uint8_t len)
{
uint8_t out_buf[SX126X_SPI_BUF_SIZE];

    for (uint8_t i = 0; i < len; i++) out_buf[i] = 0; // NOP

    WaitOnBusy();
    SpiSelect();
    SpiTransfer(SX126X_CMD_READ_REGISTER, &_status);
    SpiTransfer((adr & 0xFF00) >> 8);
    SpiTransfer(adr & 0x00FF);
    SpiTransfer(0); // NOP
    SpiTransfer(out_buf, data, len);
    SpiDeselect();
    // no delay according to semtech driver
}


void Sx126xDriverBase::WriteBuffer(uint8_t offset, uint8_t* data, uint8_t len)
{
uint8_t in_buf[SX126X_SPI_BUF_SIZE];

    WaitOnBusy();
    SpiSelect();
    SpiTransfer(SX126X_CMD_WRITE_BUFFER, &_status);
    SpiTransfer(offset);
    SpiTransfer(data, in_buf, len);
    SpiDeselect();
    SetDelay(12); // semtech driver says 12 us
}


void Sx126xDriverBase::ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len)
{
uint8_t out_buf[SX126X_SPI_BUF_SIZE];

    for (uint8_t i = 0; i < len; i++) out_buf[i] = 0; // NOP

    WaitOnBusy();
    SpiSelect();
    SpiTransfer(SX126X_CMD_READ_BUFFER, &_status);
    SpiTransfer(offset);
    SpiTransfer(0); // NOP
    SpiTransfer(out_buf, data, len);
    SpiDeselect();
    // no delay according to semtech driver
    ClearRxEvent();
}


// common

uint8_t Sx126xDriverBase::GetStatus(void)
{
    WriteCommand(SX126X_CMD_GET_STATUS);    // yes, this is correct !
    return _status;
}


void Sx126xDriverBase::SetStandby(uint8_t StandbyConfig)
{
    WriteCommand(SX126X_CMD_SET_STANDBY, StandbyConfig);
    switch (StandbyConfig) {
    case SX126X_STDBY_CONFIG_STDBY_XOSC:
        SetDelay(50); // semtech driver says 50 us
        break;
    default:
        SetDelay(1500); // semtech driver says 1500 us
    }
}


void Sx126xDriverBase::SetPacketType(uint8_t PacketType)
{
    WriteCommand(SX126X_CMD_SET_PACKET_TYPE, PacketType);
}


void Sx126xDriverBase:: SetRfFrequency(uint32_t RfFrequency) // 24 bit sx1280, 32 bit sx1262
{
uint8_t buf[4];

    buf[0] = (uint8_t)((RfFrequency & 0xFF000000) >> 24);
    buf[1] = (uint8_t)((RfFrequency & 0x00FF0000) >> 16);
    buf[2] = (uint8_t)((RfFrequency & 0x0000FF00) >> 8);
    buf[3] = (uint8_t) (RfFrequency & 0x000000FF);

    WriteCommand(SX126X_CMD_SET_RF_FREQUENCY, buf, 4);
}


void Sx126xDriverBase::SetBufferBaseAddress(uint8_t txBaseAdress, uint8_t rxBaseAdress)
{
uint8_t buf[2];

    buf[0] = txBaseAdress;
    buf[1] = rxBaseAdress;

    WriteCommand(SX126X_CMD_SET_BUFFER_BASEADDRESS, buf, 2);
}

void Sx126xDriverBase::SetModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate)
{
uint8_t buf[4];

   buf[0] = SpreadingFactor;
   buf[1] = Bandwidth;
   buf[2] = CodingRate;
   buf[3] = SX126X_LORA_LDRO_OFF;

   WriteCommand(SX126X_CMD_SET_MODULATION_PARAMS, buf, 4);

   _lora_bandwidth = Bandwidth;
}

void Sx126xDriverBase::SetPacketParams(uint8_t PreambleLength, uint8_t HeaderType, uint8_t PayloadLength, uint8_t Crc, uint8_t InvertIQ)
{
uint8_t buf[6];

    buf[0] = 0;
    buf[1] = PreambleLength;
    buf[2] = HeaderType;
    buf[3] = PayloadLength;
    buf[4] = Crc;
    buf[5] = InvertIQ;

    WriteCommand(SX126X_CMD_SET_PACKET_PARAMS, buf, 6);

    // section 15.4 Optimizing the Inverted IQ Operation p. 103-104
    // 15.4.1 Description
    // When exchanging LoRa® packets with inverted IQ polarity
    // some packet losses may be observed for longer packets.
    buf[0] = ReadRegister(SX126X_REG_IQ_CONFIG);
    if (InvertIQ) {
        buf[0] &= ~0x04;
    } else {
        buf[0] |= 0x04;
    }
    WriteRegister(SX126X_REG_IQ_CONFIG, buf[0]);

    _header_type = HeaderType;
}


void Sx126xDriverBase::SetDioIrqParams(uint16_t IrqMask, uint16_t Dio1Mask, uint16_t Dio2Mask, uint16_t Dio3Mask)
{
uint8_t buf[8];

    buf[0] = (uint8_t)((IrqMask & 0xFF00) >> 8);
    buf[1] = (uint8_t)(IrqMask & 0x00FF);

    buf[2] = (uint8_t)((Dio1Mask & 0xFF00) >> 8);
    buf[3] = (uint8_t)(Dio1Mask & 0x00FF);

    buf[4] = (uint8_t)((Dio2Mask & 0xFF00) >> 8);
    buf[5] = (uint8_t)(Dio2Mask & 0x00FF);

    buf[6] = (uint8_t)((Dio3Mask & 0xFF00) >> 8);
    buf[7] = (uint8_t)(Dio3Mask & 0x00FF);

    WriteCommand(SX126X_CMD_SET_DIOIRQ_PARAMS, buf, 8);
}


uint16_t Sx126xDriverBase::GetIrqStatus(void)
{
uint8_t status[2];

    ReadCommand(SX126X_CMD_GET_IRQ_STATUS, status, 2);
    return ((uint16_t)status[0] << 8) + status[1];
}


void Sx126xDriverBase::ClearIrqStatus(uint16_t IrqMask)
{
uint8_t buf[2];

    buf[0] = (uint8_t)((IrqMask & 0xFF00) >> 8);
    buf[1] = (uint8_t)(IrqMask & 0x00FF);

    WriteCommand(SX126X_CMD_CLR_IRQ_STATUS, buf, 2);
}


// Tx

void Sx126xDriverBase::SetTxParams(uint8_t Power, uint8_t RampTime)
{
uint8_t buf[2];

    buf[0] = Power;
    buf[1] = RampTime;

    WriteCommand(SX126X_CMD_SET_TX_PARAMS, buf, 2);
}

void Sx126xDriverBase::SetTx(uint32_t tmo_periodbase)
{
uint8_t buf[3];

    // 15.1 Modulation Quality with 500 kHz LoRa® Bandwidth
    // 15.1.1 Description
    // Some sensitivity degradation may be observed on any LoRa device, when receiving signals transmitted by the SX1261/2
    // with a LoRa BW of 500 kHz.
    // 15.1.2 Workaround
    // Before any packet transmission, bit #2 at address 0x0889 shall be set to:
    // • 0 if the LoRa BW = 500 kHz
    // • 1 for any other LoRa BW
    buf[0] = ReadRegister(SX126X_REG_SENSITIVITY_CONFIG);
    if (_lora_bandwidth == SX126X_LORA_BW_500) {
        buf[0] &= 0xFB;
    } else {
        buf[0] |= 0x04;
    }
    WriteRegister(SX126X_REG_SENSITIVITY_CONFIG, buf[0]);

    // 24 bits time out with base of 15.625us
    // TimeOut duration (us) = 15.625 * timeOut
    if (tmo_periodbase > 0xFFFFFF) tmo_periodbase = 0xFFFFFF;

    buf[0] = (uint8_t)((tmo_periodbase & 0xFF0000) >> 16);
    buf[1] = (uint8_t)((tmo_periodbase & 0x00FF00) >> 8);
    buf[2] = (uint8_t) (tmo_periodbase & 0x0000FF);
    WriteCommand(SX126X_CMD_SET_TX, buf, 3);

    SetDelay(100); // semtech driver says 100 us
}


// Rx

void Sx126xDriverBase::SetRx(uint32_t tmo_periodbase)
{
uint8_t buf[3];

    // 24 bits time out with base of 15.625us
    // TimeOut duration (us) = 15.625 * timeOut
    if (tmo_periodbase > 0xFFFFFF) tmo_periodbase = 0xFFFFFF;

    buf[0] = (uint8_t)((tmo_periodbase & 0xFF0000) >> 16);
    buf[1] = (uint8_t)((tmo_periodbase & 0x00FF00) >> 8);
    buf[2] = (uint8_t) (tmo_periodbase & 0x0000FF);
    WriteCommand(SX126X_CMD_SET_RX, buf, 3);

    SetDelay(100); // semtech driver says 100 us
}


void Sx126xDriverBase::GetPacketStatus(int8_t* RssiSync, int8_t* Snr)
{
uint8_t status[3];

    ReadCommand(SX126X_CMD_GET_PACKET_STATUS, status, 3);

    *RssiSync = -(int8_t)(status[0] / 2);
    *Snr = (int8_t)(status[1] / 4);
}


void Sx126xDriverBase::GetRxBufferStatus(uint8_t* rxPayloadLength, uint8_t* rxStartBufferPointer)
{
uint8_t status[2];

    ReadCommand(SX126X_CMD_GET_RX_BUFFER_STATUS, status, 2);

    *rxPayloadLength = status[0];
    *rxStartBufferPointer = status[1];
}


void Sx126xDriverBase::ClearRxEvent(void)
{
	// 15.3 Implicit Header Mode Timeout Behavior   p. 103
	// 15.3.1 Description
	// When receiving LoRa® packets in Rx mode with Timeout active, and no header (Implicit Mode), the timer responsible for
	// generating the Timeout (based on the RTC timer) is not stopped on RxDone event. Therefore, it may trigger an unexpected
	// timeout in any subsequent mode where the RTC isn’t re-invoked, and therefore reset and re-programmed.
	if (_header_type == SX126X_LORA_HEADER_IMPLICIT) {
		uint8_t buf = ReadRegister(SX126X_REG_RTC_STOP);
	    WriteRegister(SX126X_REG_RTC_STOP, 0x00); // stop the timer
	    uint8_t data = ReadRegister(SX126X_REG_RTC_EVENT);
	    data |= 0x02;
	    WriteRegister(SX126X_REG_RTC_EVENT, data);
	    WriteRegister(SX126X_REG_RTC_STOP, buf);  // restart the timer
	}
}

// auxiliary

void Sx126xDriverBase::SetRegulatorMode(uint8_t RegModeParam)
{
    WriteCommand(SX126X_CMD_SET_REGULATOR_MODE, RegModeParam);
}


void Sx126xDriverBase::SetAutoFs(uint8_t flag)
{
uint8_t buf;

    buf = (flag) ? SX126X_RX_TX_FALLBACK_MODE_FS : SX126X_RX_TX_FALLBACK_MODE_STDBY_RC;
    WriteCommand(SX126X_CMD_SET_RX_TX_FALLBACK_MODE, buf);
}


void Sx126xDriverBase::SetFs(void)
{
    WriteCommand(SX126X_CMD_SET_FS);
    SetDelay(70); // semtech driver says 70 us
}


void Sx126xDriverBase::SetLnaGainMode(uint8_t LnaGainMode)
{
uint8_t reg_rxgain;

    reg_rxgain = (LnaGainMode) ? SX126X_BOOST_GAIN_REG_VALUE : SX126X_SAVING_GAIN_REG_VALUE;
    WriteRegister(SX126X_REG_RX_GAIN, reg_rxgain);
}


uint16_t Sx126xDriverBase::GetFirmwareRev(void)
{
    return GetStatus();
}


void Sx126xDriverBase::SetSyncWord(uint16_t SyncWord)
{
uint8_t buf[2];

    buf[0] = (uint8_t)(SyncWord & 0x00FF);
    buf[1] = (uint8_t)((SyncWord & 0xFF00) >> 8);

    WriteRegister(SX126X_REG_LORA_SYNC_WORD_MSB, buf, 2);
}


void Sx126xDriverBase::ClearDeviceError(void)
{
uint8_t buf[] = {0, 0};

    WriteCommand(SX126X_CMD_CLEAR_DEVICE_ERRORS, buf, 2);
}


uint16_t Sx126xDriverBase::GetDeviceError(void)
{
uint8_t buf[2];

    ReadCommand(SX126X_CMD_GET_DEVICE_ERRORS, buf, 2);

    return ((uint16_t)buf[0] << 8) | buf[1];
}


// set DIO 3 as TCXO control, DIO 3 control output voltage to TCXO oscilqtor
void Sx126xDriverBase::SetDio3AsTcxoControl(uint8_t OutputVoltage, uint32_t delay_us)
{
uint8_t buf[4];

    buf[0] = OutputVoltage;
    buf[1] = (uint8_t)((delay_us >> 16) & 0xFF);
    buf[2] = (uint8_t)((delay_us >> 8) & 0xFF);
    buf[3] = (uint8_t)(delay_us & 0xFF);

    WriteCommand(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, buf, 4);
}


// set DIO 2 as RF switching control
void Sx126xDriverBase::SetDio2AsRfSwitchControl(uint8_t Mode)
{
    WriteCommand(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, Mode);
}


// set lora symbol number time out for prevent of false lora preamble detection
void Sx126xDriverBase::SetSymbNumTimeout(uint8_t tmo_symbnum)
{
uint8_t mant = (((tmo_symbnum > SX126X_MAX_LORA_SYMB_NUM_TIMEOUT) ? SX126X_MAX_LORA_SYMB_NUM_TIMEOUT : tmo_symbnum) + 1) >> 1;
uint8_t exp  = 0;
uint8_t reg  = 0;

    while (mant > 31) {
        mant = ( mant + 3 ) >> 2;
        exp++;
    }
    reg = mant << (2 * exp + 1);

    WriteCommand(SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT, reg);

    if (tmo_symbnum != 0) {
        reg = exp + (mant << 3);
        WriteRegister(SX126X_REG_SYNCH_TIMEOUT, reg);
    }
}


void Sx126xDriverBase::SetPaConfig_22dbm(void)
{
uint8_t buf[4];

    buf[0] = SX126X_PA_CONFIG_PA_DUTY_CYCLE;
    buf[1] = SX126X_PA_CONFIG_HP_MAX;
    buf[2] = SX126X_PA_CONFIG_DEVICE;
    buf[3] = SX126X_PA_CONFIG_PA_LUT;

    WriteCommand(SX126X_CMD_SET_PA_CONFIG, buf, 4);
}




