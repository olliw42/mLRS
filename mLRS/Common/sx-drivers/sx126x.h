//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// SX126x library
//*******************************************************
// contributed by jinchuuriki
//*******************************************************
#ifndef SX126X_LIB_H
#define SX126X_LIB_H
#pragma once

#include <inttypes.h>

#define SX126X_FREQ_XTAL_HZ               32000000


#define SX126X_FREQ_MHZ_TO_REG(f_mhz)     (uint32_t)((double)f_mhz*1.0E6*(double)(1 << 25)/(double)SX126X_FREQ_XTAL_HZ)


// not documented in the datasheet
// TODO: is this correct? where does this info come from?
#define SX126X_REG_FIRMWARE_VERSION_MSB   0x0153 // address of the register holding firmware version MSB

#define SX126X_MAX_LORA_SYMB_NUM_TIMEOUT  248


#ifndef ALIGNED
#define ALIGNED  __attribute__((aligned(4)))
#endif


#define SX126X_SPI_BUF_SIZE               256 // this must hold the max payload plus additional bytes


//-------------------------------------------------------
// Base Class
//-------------------------------------------------------

class Sx126xDriverBase
{
  public:
      Sx126xDriverBase() {} // constructor

    // this you will have to fill in the derived class

    void Init(void) {};

    // these you have to supply in the derived class

    virtual void SpiSelect(void) = 0;
    virtual void SpiDeselect(void) = 0;
    virtual void SpiTransfer(uint8_t* dataout, uint8_t* datain, uint8_t len) = 0;

    virtual void WaitOnBusy(void) {};
    virtual void SetDelay(uint16_t tmo_us) { (void)tmo_us; };

    // low level methods, usually no need to use them

    void SpiTransfer(uint8_t data, uint8_t* datain) { SpiTransfer(&data, datain, 1); }
    void SpiTransfer(uint8_t data) { uint8_t dummy; SpiTransfer(&data, &dummy, 1); }

    void WriteCommand(uint8_t opcode, uint8_t* data, uint8_t len);
    void ReadCommand(uint8_t opcode, uint8_t* data, uint8_t len);
    void WriteRegister(uint16_t adr, uint8_t* data, uint8_t len);
    void ReadRegister(uint16_t adr, uint8_t* data, uint8_t len);
    void WriteBuffer(uint8_t offset, uint8_t* data, uint8_t len);
    void ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len);

    void WriteCommand(uint8_t opcode) { WriteCommand(opcode, nullptr, 0); }
    void WriteCommand(uint8_t opcode, uint8_t data) { WriteCommand(opcode, &data, 1); }
    void WriteRegister(uint16_t adr, uint8_t data) { WriteRegister(adr, &data, 1); }
    uint8_t ReadRegister(uint16_t adr) { uint8_t data; ReadRegister(adr, &data, 1); return data; }

    // common methods

    uint8_t GetStatus(void);
    void SetStandby(uint8_t StandbyConfig);
    void SetPacketType(uint8_t PacketType);
    void SetRfFrequency(uint32_t RfFrequency); // 24 bits only
    void SetDio3AsTcxoControl(uint8_t OutputVoltage, uint32_t delay_us);
    void SetDio2AsRfSwitchControl(uint8_t Mode);
    void SetPaConfig_22dbm(void);
    void SetBufferBaseAddress(uint8_t txBaseAdress, uint8_t rxBaseAdress);
    void SetModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate);
    void SetPacketParams(uint8_t PreambleLength, uint8_t HeaderType, uint8_t PayloadLength, uint8_t Crc, uint8_t InvertIQ);
    void SetDioIrqParams(uint16_t IrqMask, uint16_t Dio1Mask, uint16_t Dio2Mask, uint16_t Dio3Mask);

    uint16_t GetIrqStatus(void);
    void ClearIrqStatus(uint16_t IrqMask);

    // Tx methods
    void SetTxParams(uint8_t Power, uint8_t RampTime);
    void SetTx(uint32_t tmo_periodbase); // max 24 bit

    // Rx methods
    void SetRx(uint32_t tmo_periodbase); // max 24 bit
    void GetPacketStatus(int8_t* RssiSync, int8_t* Snr);
    void GetRxBufferStatus(uint8_t* rxPayloadLength, uint8_t* rxStartBufferPointer);
    void ClearRxEvent(void); // clear any event, if any
    // auxiliary methods

    void ClearDeviceError(void);
    uint16_t GetDeviceError(void);
    void SetSymbNumTimeout(uint8_t tmo_symbnum);

    void SetRegulatorMode(uint8_t RegModeParam);
    void SetAutoFs(uint8_t flag);
    void SetFs(void);
    void SetLnaGainMode(uint8_t LnaGainMode);

    // what else we like to have

    uint16_t GetFirmwareRev(void);
    void SetSyncWord(uint16_t SyncWord);
    uint32_t GetFrequencyErrorIndicator(void);

    uint8_t GetLastStatus(void) { return _status; }

  private:
    uint8_t _status; // all spi transfers yield the status, so we can just get it

    uint8_t _header_type; // keep HeaderType, is required in SetRx()
    uint8_t _lora_bandwidth; // keep lora bandwidth, is required in SetTx()
};


//-------------------------------------------------------
// Enum Definitions
//-------------------------------------------------------

// WriteCommand(uint8_t opcode, uint8_t* data, uint8_t len)
typedef enum {
    // SX126X SPI commands
    // operational modes commands
    SX126X_CMD_NOP                        = 0x00,
    SX126X_CMD_SET_SLEEP                  = 0x84,
    SX126X_CMD_SET_STANDBY                = 0x80,
    SX126X_CMD_SET_FS                     = 0xC1,
    SX126X_CMD_SET_TX                     = 0x83,
    SX126X_CMD_SET_RX                     = 0x82,
    SX126X_CMD_STOP_TIMER_ON_PREAMBLE     = 0x9F,
    SX126X_CMD_SET_RX_DUTY_CYCLE          = 0x94,
    SX126X_CMD_SET_CAD                    = 0xC5,
    SX126X_CMD_SET_TX_CONTINUOUSWAVE      = 0xD1,
    SX126X_CMD_SET_TX_CONTINUOUSPREAMBLE  = 0xD2,
    SX126X_CMD_SET_REGULATOR_MODE         = 0x96,
    SX126X_CMD_CALIBRATE                  = 0x89,
    SX126X_CMD_CALIBRATE_IMAGE            = 0x98,
    SX126X_CMD_SET_PA_CONFIG              = 0x95,
    SX126X_CMD_SET_RX_TX_FALLBACK_MODE    = 0x93,

    // register and buffer access commands
    SX126X_CMD_WRITE_REGISTER             = 0x0D,
    SX126X_CMD_READ_REGISTER              = 0x1D,
    SX126X_CMD_WRITE_BUFFER               = 0x0E,
    SX126X_CMD_READ_BUFFER                = 0x1E,

    // DIO and IRQ control
    SX126X_CMD_SET_DIOIRQ_PARAMS          = 0x08,
    SX126X_CMD_GET_IRQ_STATUS             = 0x12,
    SX126X_CMD_CLR_IRQ_STATUS             = 0x02,
    SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D,
    SX126X_CMD_SET_DIO3_AS_TCXO_CTRL      = 0x97,

    // RF, modulation and packet commands
    SX126X_CMD_SET_RF_FREQUENCY           = 0x86,
    SX126X_CMD_SET_PACKET_TYPE            = 0x8A,
    SX126X_CMD_GET_PACKET_TYPE            = 0x11,
    SX126X_CMD_SET_TX_PARAMS              = 0x8E,
    SX126X_CMD_SET_MODULATION_PARAMS      = 0x8B,
    SX126X_CMD_SET_PACKET_PARAMS          = 0x8C,
    SX126X_CMD_SET_CAD_PARAMS             = 0x88,
    SX126X_CMD_SET_BUFFER_BASEADDRESS     = 0x8F,
    SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT  = 0x0A,

    // status commands
    SX126X_CMD_GET_STATUS                 = 0xC0,
    SX126X_CMD_GET_RSSI_INST              = 0x15,
    SX126X_CMD_GET_RX_BUFFER_STATUS       = 0x13,
    SX126X_CMD_GET_PACKET_STATUS          = 0x14,
    SX126X_CMD_GET_DEVICE_ERRORS          = 0x17,
    SX126X_CMD_CLEAR_DEVICE_ERRORS        = 0x07,
    SX126X_CMD_GET_STATS                  = 0x10,
    SX126X_CMD_RESET_STATS                = 0x00
} SX126X_CMD_ENUM;


// cmd 0x0d WriteRegister(uint16_t adr, uint8_t data, uint8_t len)
// cmd 0x1d ReadRegister(uint16_t adr, uint8_t* data, uint8_tlen)
typedef enum {
    // SX126X register map
    SX126X_REG_WHITENING_INITIAL_MSB      = 0x06B8, // FSK
    SX126X_REG_WHITENING_INITIAL_LSB      = 0x06B9, // FSK
    SX126X_REG_CRC_INITIAL_MSB            = 0x06BC, // FSK
    SX126X_REG_CRC_INITIAL_LSB            = 0x06BD, // FSK
    SX126X_REG_CRC_POLYNOMIAL_MSB         = 0x06BE, // FSK
    SX126X_REG_CRC_POLYNOMIAL_LSB         = 0x06BF, // FSK
    SX126X_REG_SYNC_WORD_0                = 0x06C0, // FSK
    SX126X_REG_SYNC_WORD_1                = 0x06C1, // FSK
    SX126X_REG_SYNC_WORD_2                = 0x06C2, // FSK
    SX126X_REG_SYNC_WORD_3                = 0x06C3, // FSK
    SX126X_REG_SYNC_WORD_4                = 0x06C4, // FSK
    SX126X_REG_SYNC_WORD_5                = 0x06C5, // FSK
    SX126X_REG_SYNC_WORD_6                = 0x06C6, // FSK
    SX126X_REG_SYNC_WORD_7                = 0x06C7, // FSK
    SX126X_REG_NODE_ADDRESS               = 0x06CD, // FSK
    SX126X_REG_BROADCAST_ADDRESS          = 0x06CE, // FSK
    SX126X_REG_LORA_SYNC_WORD_MSB         = 0x0740, // LORA
    SX126X_REG_LORA_SYNC_WORD_LSB         = 0x0741, // LORA
    SX126X_REG_RANDOM_NUMBER_0            = 0x0819, // Random number generator
    SX126X_REG_RANDOM_NUMBER_1            = 0x081A,
    SX126X_REG_RANDOM_NUMBER_2            = 0x081B,
    SX126X_REG_RANDOM_NUMBER_3            = 0x081C,
    SX126X_REG_RX_GAIN                    = 0x08AC, // boosted gain RX; Rx power saving : 0x94; Rx Boosted gain : 0x96
    SX126X_REG_OCP_CONFIGURATION          = 0x08E7, // over current protection
    SX126X_REG_XTA_TRIM                   = 0x0911,
    SX126X_REG_XTB_TRIM                   = 0x0912,

    // undocumented registers
    SX126X_REG_SYNCH_TIMEOUT              = 0x0706, // undocumented, get from semtech example
    SX126X_REG_SENSITIVITY_CONFIG         = 0x0889, // SX1268 datasheet v1.1, section 15.1
    SX126X_REG_TX_CLAMP_CONFIG            = 0x08D8, // SX1268 datasheet v1.1, section 15.2
    SX126X_REG_RTC_STOP                   = 0x0920, // SX1268 datasheet v1.1, section 15.3
    SX126X_REG_RTC_EVENT                  = 0x0944, // SX1268 datasheet v1.1, section 15.3
    SX126X_REG_IQ_CONFIG                  = 0x0736, // SX1268 datasheet v1.1, section 15.4
    SX126X_REG_RX_GAIN_RETENTION_0        = 0x029F, // SX1268 datasheet v1.1, section 9.6
    SX126X_REG_RX_GAIN_RETENTION_1        = 0x02A0, // SX1268 datasheet v1.1, section 9.6
    SX126X_REG_RX_GAIN_RETENTION_2        = 0x02A1, // SX1268 datasheet v1.1, section 9.6
} SX126X_REG_ENUM;


// cmd 0xC0 uint8_t GetStatus(void)
typedef enum {
    SX126X_STATUS_MODE_STDBY_RC           = 0b00100000, // table 13-76, p. 95
    SX126X_STATUS_MODE_STDBY_XOSC         = 0b00110000,
    SX126X_STATUS_MODE_FS                 = 0b01000000,
    SX126X_STATUS_MODE_RX                 = 0b01010000,
    SX126X_STATUS_MODE_TX                 = 0b01100000,
    SX126X_STATUS_MODE_MASK               = 0b01110000,
} SX126X_STATUS_MODE_ENUM;

typedef enum {
    SX126X_STATUS_CMD_DATA_AVAILABLE      = 0b00000100 , // table 13-76, p. 95
    SX126X_STATUS_CMD_TIMEOUT             = 0b00000110,
    SX126X_STATUS_CMD_PROCESSING_ERROR    = 0b00001000,
    SX126X_STATUS_CMD_EXEC_FAILURE        = 0b00001010,
    SX126X_STATUS_CMD_TX_DONE             = 0b00001100,
    SX126X_STATUS_CMD_MASK                = 0b00001110  ,
} SX126X_STATUS_CMD_ENUM;


//-------------------------------------------------------
// Enum Definitions Common
//-------------------------------------------------------

// cmd 0x80 SetStandby(uint8_t StandbyConfig)
typedef enum {
    SX126X_STDBY_CONFIG_STDBY_RC          = 0x00, // table 13-4, p. 68
    SX126X_STDBY_CONFIG_STDBY_XOSC        = 0x01
} SX126X_STDBY_CONFIG_ENUM;


// cmd 0x8A SetPacketType(uint8_t PacketType)
typedef enum {
    SX126X_PACKET_TYPE_GFSK               = 0x00, // table 13-38, p. 83
    SX126X_PACKET_TYPE_LORA               = 0x01,
} SX126X_PACKET_TYPE_ENUM;


// cmd 0x86 SetRfFrequency(uint32_t RfFrequency) // 32 bits
// cmd 0x8F SetBufferBaseAddress(uint8_t txBaseAdress, uint8_t rxBaseAdress)
// cmd 0x8B SetModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate, uint8_t LowDataRateOptimize)
typedef enum {
    SX126X_LORA_SF5                       = 0x05, // table 13-47, p. 87
    SX126X_LORA_SF6                       = 0x06,
    SX126X_LORA_SF7                       = 0x07,
    SX126X_LORA_SF8                       = 0x08,
    SX126X_LORA_SF9                       = 0x09,
    SX126X_LORA_SF10                      = 0x0A,
    SX126X_LORA_SF11                      = 0x0B,
    SX126X_LORA_SF12                      = 0x0C,
} SX126X_LORA_SF_ENUM;

typedef enum {
    SX126X_LORA_BW_62P5                   = 0x03, // table 13-48, p. 87
    SX126X_LORA_BW_125                    = 0x04,
    SX126X_LORA_BW_250                    = 0x05,
    SX126X_LORA_BW_500                    = 0x06,
} SX126X_LORA_BW_ENUM;

typedef enum {
    SX126X_LORA_CR_4_5                    = 0x01, // table 13-49, p. 87
    SX126X_LORA_CR_4_6                    = 0x02,
    SX126X_LORA_CR_4_7                    = 0x03,
    SX126X_LORA_CR_4_8                    = 0x04,
} SX126X_LORA_CR_ENUM;

typedef enum {
    SX126X_LORA_LDRO_OFF                  = 0x00, // table 13-50, 0 88
    SX126X_LORA_LDRO_ON                   = 0x01
} SX126X_LORA_LDRO_ENUM;

// cmd 0x8C SetPacketParams(uint8_t PreambleLength, uint8_t HeaderType, uint8_t PayloadLength, uint8_t CRC, uint8_t InvertIQ)
typedef enum {
    SX126X_LORA_HEADER_EXPLICIT           = 0x00, // table 13-67, p. 92
    SX126X_LORA_HEADER_IMPLICIT           = 0x01,
    SX126X_LORA_HEADER_DISABLE            = SX126X_LORA_HEADER_IMPLICIT,
    SX126X_LORA_HEADER_ENABLE             = SX126X_LORA_HEADER_EXPLICIT,
} SX126X_LORA_HEADER_ENUM;

typedef enum {
    SX126X_LORA_CRC_DISABLE               = 0x00, // table 14-69, p. 92
    SX126X_LORA_CRC_ENABLE                = 0x01,
} SX126X_LORA_CRC_ENUM;

typedef enum {
    SX126X_LORA_IQ_NORMAL                 = 0x00, // table 13-70, p. 92
    SX126X_LORA_IQ_INVERTED               = 0x01,
} SX126X_LORA_IQMODE_ENUM;


// cmd 0x08 SetDioIrqParams(uint16_t IrqMask, uint16_t Dio1Mask, uint16_t Dio2Mask, uint16_t Dio3Mask)
typedef enum {
    SX126X_IRQ_NONE                       = 0x0000, // table 13-29, p 80
    SX126X_IRQ_TX_DONE                    = 0x0001,
    SX126X_IRQ_RX_DONE                    = 0x0002,
    SX126X_IRQ_PREAMBLE_DETECTED          = 0x0004,
    SX126X_IRQ_SYNCWORD_VALID             = 0x0008, // not LORA
    SX126X_IRQ_HEADER_VALID               = 0x0010,
    SX126X_IRQ_HEADER_ERROR               = 0x0020,
    SX126X_IRQ_CRC_ERROR                  = 0x0040,
    SX126X_IRQ_CAD_DONE                   = 0x0080,
    SX126X_IRQ_CAD_DETECTED               = 0x0100,
    SX126X_IRQ_RX_TX_TIMEOUT              = 0x0200,
    SX126X_IRQ_ALL                        = 0xFFFF,
} SX126X_IRQ_ENUM;

// cmd 0x9D SetDIO2AsRfSwitchCtrl(uint8_t Dio2Mode)
typedef enum {
    SX126X_DIO2_AS_IRQ                    = 0x00, // table 13-33 p. 81
    SX126X_DIO2_AS_RF_SWITCH              = 0x01
} SX126X_DIO2_MODE_ENUM;

// cmd 0x97 SetDIO3AsTCXOCtrl(uint8_t tcxoVoltage, uint32_t delay)
typedef enum {                                   // table 13-35 p. 82
    SX126X_DIO3_OUTPUT_1_6                = 0x00, // 1.6V
    SX126X_DIO3_OUTPUT_1_7                = 0x01, // 1.7V
    SX126X_DIO3_OUTPUT_1_8                = 0x02, // 1.8V
    SX126X_DIO3_OUTPUT_2_2                = 0x03, // 2.2V
    SX126X_DIO3_OUTPUT_2_4                = 0x04, // 2.4V
    SX126X_DIO3_OUTPUT_2_7                = 0x05, // 2.7V
    SX126X_DIO3_OUTPUT_3_0                = 0x06, // 3.0V
    SX126X_DIO3_OUTPUT_3_3                = 0x07, // 3.3V
} SX126X_DIO3_OUTPUT_ENUM;


// cmd 0x83 SetTx(uint32_t timeOut)
// cmd 0x82 SetRx(uint32_t timeOut)

typedef enum {
    SX126X_TIMEOUT_TX_NONE                = 0,
} SX126X_TIMEOUT_TX_ENUM;

typedef enum {
    SX126X_TIMEOUT_RX_SINGLE              = 0x0000,
    SX126X_TIMEOUT_RX_CONTINUOUS          = 0xFFFF,
} SX126X_TIMEOUT_RX_ENUM;


//-------------------------------------------------------
// Enum Definitions Tx
//-------------------------------------------------------

// cmd 0x8E SetTxParams(uint8_t Power, uint8_t RampTime)
typedef enum {
    SX126X_RAMPTIME_10_US                 = 0x00, // table 13-41, p. 84
    SX126X_RAMPTIME_20_US                 = 0x01,
    SX126X_RAMPTIME_40_US                 = 0x02,
    SX126X_RAMPTIME_80_US                 = 0x03,
    SX126X_RAMPTIME_200_US                = 0x04,
    SX126X_RAMPTIME_800_US                = 0x05,
    SX126X_RAMPTIME_1700_US               = 0x06,
    SX126X_RAMPTIME_3400_US               = 0x07,
} SX126X_RAMPTIME_ENUM;

// added for our convenience
// -9 (0xF7) to +22 (0x16) dBm by step of 1 dB if high power PA is selected
typedef enum {
    SX126X_POWER_m9_DBM                   = -9, // 0.12 mW
    SX126X_POWER_0_DBM                    = 0, // 1 mW
    SX126X_POWER_10_DBM                   = 10, // 10 mW
    SX126X_POWER_17_DBM                   = 17, // 50 mW
    SX126X_POWER_20_DBM                   = 20, // 100 mW
    SX126X_POWER_22_DBM                   = 22, // 158 mW
    SX126X_POWER_MIN                      = SX126X_POWER_m9_DBM,
    SX126X_POWER_MAX                      = SX126X_POWER_22_DBM,
} SX126X_POWER_ENUM;


//-------------------------------------------------------
// Enum Definitions Rx
//-------------------------------------------------------

// cmd 0x14 GetPacketStatus(uint8_t* RssiSync, uint8_t* Snr)
// cmd 0x13 GetRxBufferStatus(uint8_t* rxPayloadLength, uint8_t* rxStartBufferPointer)
// cmd 0x1E ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len)


//-------------------------------------------------------
// Calibration
//-------------------------------------------------------
typedef enum {                                           // table 13-18 p. 75
    SX126X_CALIBRATE_IMAGE_OFF            = 0b00000000,  //  6     6     image calibration: disabled
    SX126X_CALIBRATE_IMAGE_ON             = 0b01000000,  //  6     6                        enabled
    SX126X_CALIBRATE_ADC_BULK_P_OFF       = 0b00000000,  //  5     5     ADC bulk P calibration: disabled
    SX126X_CALIBRATE_ADC_BULK_P_ON        = 0b00100000,  //  5     5                             enabled
    SX126X_CALIBRATE_ADC_BULK_N_OFF       = 0b00000000,  //  4     4     ADC bulk N calibration: disabled
    SX126X_CALIBRATE_ADC_BULK_N_ON        = 0b00010000,  //  4     4                             enabled
    SX126X_CALIBRATE_ADC_PULSE_OFF        = 0b00000000,  //  3     3     ADC pulse calibration: disabled
    SX126X_CALIBRATE_ADC_PULSE_ON         = 0b00001000,  //  3     3                            enabled
    SX126X_CALIBRATE_PLL_OFF              = 0b00000000,  //  2     2     PLL calibration: disabled
    SX126X_CALIBRATE_PLL_ON               = 0b00000100,  //  2     2                      enabled
    SX126X_CALIBRATE_RC13M_OFF            = 0b00000000,  //  1     1     13 MHz RC osc. calibration: disabled
    SX126X_CALIBRATE_RC13M_ON             = 0b00000010,  //  1     1                                 enabled
    SX126X_CALIBRATE_RC64K_OFF            = 0b00000000,  //  0     0     64 kHz RC osc. calibration: disabled
    SX126X_CALIBRATE_RC64K_ON             = 0b00000001,  //  0     0                                 enabled
    SX126X_CALIBRATE_ALL                  = 0b01111111,  //  6     0     calibrate all blocks
} SX126X_CALIBRATE_ENUM;

// cmd 0x89 CalibrateFunction (uint8_t* CalibrationParam)


//-------------------------------------------------------
// Image Calibration on specific frequency band
//-------------------------------------------------------
// cmd 0x98 CalibrateImage (uint8_t freq1, uint8_t freq2)
typedef enum {
    SX126X_CAL_IMG_430_MHZ_1             = 0x6B, // table 9-2 p. 57
    SX126X_CAL_IMG_430_MHZ_2             = 0x6F,
    SX126X_CAL_IMG_470_MHZ_1             = 0x75,
    SX126X_CAL_IMG_470_MHZ_2             = 0x81,
    SX126X_CAL_IMG_779_MHZ_1             = 0xC1,
    SX126X_CAL_IMG_779_MHZ_2             = 0xC5,
    SX126X_CAL_IMG_863_MHZ_1             = 0xD7,
    SX126X_CAL_IMG_863_MHZ_2             = 0xDB,
    SX126X_CAL_IMG_902_MHZ_1             = 0xE1,
    SX126X_CAL_IMG_902_MHZ_2             = 0xE9,
} SX126X_CALIBRATE_IMAGE_ENUM;


//-------------------------------------------------------
// PA config enum definition
//-------------------------------------------------------

// set power to max 22 dbm
// cmd 0x95 setPaConfig (uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut)
typedef enum {
    SX126X_PA_CONFIG_HP_MAX              = 0x07, // table 13-21 p. 77
    SX126X_PA_CONFIG_PA_LUT              = 0x01,
    SX126X_PA_CONFIG_DEVICE              = 0x00, // 1262 selected
    SX126X_PA_CONFIG_PA_DUTY_CYCLE       = 0x04
} SX126X_PA_CONFIG_ENUM;


//-------------------------------------------------------
// Enum Definitions Auxiliary
//-------------------------------------------------------

// cmd 0x96 SetRegulatorMode(uint8_t RegModeParam)
typedef enum {
    SX126X_REGULATOR_MODE_LDO             = 0x00, // table 13-16, p. 74
    SX126X_REGULATOR_MODE_DCDC            = 0x01
} SX126X_REGULATOR_MODE_ENUM;


// cmd 0x93 SetRxTxFallbackMode(uint8_t FallBackMode)
// After TX/RX operation, mode change to
typedef enum {
    SX126X_RX_TX_FALLBACK_MODE_FS         = 0x40, // table 13-23, p. 77
    SX126X_RX_TX_FALLBACK_MODE_STDBY_XOSC = 0x30,
    SX126X_RX_TX_FALLBACK_MODE_STDBY_RC   = 0x20
} SX126X_FALLBACK_MODE_ENUM;


// reg 0x8AC SetLnaGainMode(uint8_t LnaGainMode)
typedef enum {
  SX126X_LNA_GAIN_MODE_LOW_POWER          = 0x00, // table 9-3, p. 58
  SX126X_LNA_GAIN_MODE_HIGH_SENSITIVITY   = 0x01,
  SX126X_BOOST_GAIN_REG_VALUE             = 0x96,
  SX126X_SAVING_GAIN_REG_VALUE            = 0x94,
} SX126X_LNAGAIN_MODE_ENUM;


#endif // SX126X_LIB_H










