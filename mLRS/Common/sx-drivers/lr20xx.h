//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// LR2021 library
//*******************************************************
// contributed by JLP, OlliW42
//*******************************************************
#ifndef LR20XX_LIB_H
#define LR20XX_LIB_H
#pragma once

#include <inttypes.h>

#define LR20XX_FREQ_XTAL_HZ               32000000

#define LR20XX_FREQ_MHZ_TO_REG(f_mhz)     (uint32_t)((double)f_mhz * 1.0E6)
#define LR20XX_FREQ_GHZ_TO_REG(f_ghz)     (uint32_t)((double)f_ghz * 1.0E9)

#ifndef ALIGNED
#define ALIGNED __attribute__((aligned(4)))
#endif


//-------------------------------------------------------
// Base Class
//-------------------------------------------------------

class Lr20xxDriverBase {
public:
    Lr20xxDriverBase() {} // constructor

    // this you will have to fill in the derived class

    void Init(void) {}

    // these you have to supply in the derived class

    virtual void SpiSelect(void) = 0;
    virtual void SpiDeselect(void) = 0;
    virtual void SpiTransfer(uint8_t* dataout, uint8_t* datain, uint8_t len) = 0;

    virtual void SpiRead(uint8_t* datain, uint8_t len);
    virtual void SpiWrite(uint8_t* dataout, uint8_t len);

    virtual void WaitOnBusy(void) = 0;
    virtual void SetDelay(uint16_t tmo_us) { (void)tmo_us; }

    // spi methods

    void SpiTransfer(uint8_t dataout, uint8_t* datain) { SpiTransfer(&dataout, datain, 1); }
    void SpiRead(uint8_t* datain) { SpiRead(datain, 1); }
    void SpiWrite(uint8_t dataout) { SpiWrite(&dataout, 1); }

    // low level methods, usually no need to use them

    void WriteCommand(uint16_t opcode, uint8_t* data, uint8_t len);
    void ReadCommand(uint16_t opcode, uint8_t* data, uint8_t len);

    void WriteCommand(uint16_t opcode) { WriteCommand(opcode, nullptr, 0); }
    void WriteCommand(uint16_t opcode, uint8_t data) { WriteCommand(opcode, &data, 1); }
    uint8_t ReadCommand(uint16_t opcode) { uint8_t data; ReadCommand(opcode, &data, 1); return data; }

    void WriteRadioTxFifo(uint8_t* data, uint8_t len);
    void ReadRadioRxFifo(uint8_t* data, uint8_t len);

    void WriteRegMem32(uint32_t addr, uint32_t* data, uint8_t len); // len is number of uint32_t
    void WriteRegMemMask32(uint32_t addr, uint32_t mask, uint32_t data);
    void ReadRegMem32(uint32_t addr, uint32_t* data, uint8_t len); // len is number of uint32_t

    // System Configuration Commands

    void GetStatus(uint8_t* Status1, uint8_t* Status2);
    uint16_t GetStatus(void);
    uint16_t GetLastStatus(void);
    uint8_t GetLastStatusCmd(void);
    uint8_t GetLastStatusChipMode(void);
    void GetVersion(uint8_t* FwMajor, uint8_t* FwMinor);
    uint16_t GetErrors(void);
    void ClearErrors(void);
    void SetDioFunction(uint8_t Dio, uint8_t Func, uint8_t PullDrive);
    void SetDioRfSwitchConfig(uint8_t Dio, uint8_t Config);
    // ClearFifoIrqFLags()
    void SetDioIrqConfig(uint8_t Dio, uint32_t Irq);
    void ClearIrq(uint32_t IrqsToClear);
    uint32_t GetAndClearIrqStatus(void);
    // ConfigLfClock(), ConfigFifoIrq(), GetFifoIrqFlags(), GetFifoIrqFlags()
    uint16_t GetRxFifoLevel(void);
    uint16_t GetTxFifoLevel(void);
    void ClearRxFifo(void);
    void ClearTxFifo(void);
    void SetTcxoMode(uint8_t Tune, uint32_t StartTime);
    void SetRegMode(uint8_t SimoUsage);
    void Calibrate(uint8_t BlocksToCalibrate);
    void CalibFE(uint16_t Freq1, uint16_t Freq2, uint16_t Freq3);
    // GetVbat(), GetTemp(), GetRandomNumber(), SetSleep()
    void SetStandby(uint8_t StandbyMode);
    void SetFs(void);
    // SetAdditionalRegToRetain(), SetLbdCfg(), SetXosxCpTrim(), SetTempCompCfg(), SetNtcParams()

    // Common Radio Commands

    void SetRfFrequency(uint32_t RfFreq);
    void SetRxPath(uint8_t RxPath, uint8_t RxBoost);
    void SetPaConfig(uint8_t PaSel, uint8_t PaLfMode, uint8_t PaLfDutyCycle, uint8_t PaLfSlices, uint8_t PaHfDutyCycle);
    void SetPaConfig915Mhz(int8_t Power);
    void SetPaConfig2p4Ghz(int8_t Power);
    void SetPaConfig433Mhz(int8_t Power);
    void SetTxParams(int8_t Power, uint8_t RampTime);
    void SetRssiCalibration(uint8_t RxPathHf, uint8_t RxPathLf, uint8_t* table); // TODO: get this right
    void SetRxTxFallbackMode(uint8_t FallbackMode);
    void SetPacketType(uint8_t PacketType);
    uint8_t GetPacketType(void);
    // StopTimeoutOnPreamble()
    void ResetRxStats(void);
    int16_t GetRssiInst(void);
    void SetRx(uint32_t RxTimeout); // 24 bits only
    void SetRx(void); // timeout set with SetDefaultRxTxTimeout() is used
    void SetTx(uint32_t TxTimeout); // 24 bits only
    void SetTx(void); // timeout set with SetDefaultRxTxTimeout() is used
    // SetTxTestMode()
    void SelPa(uint8_t PaSel);
    // SetRxDutyCycle(), SetAutoRxTx()
    uint16_t GetRxPktLength(void);
    void SetDefaultRxTxTimeout(uint32_t RxTimeout, uint32_t TxTimeout); // 24 bits only
    // SetTimestampSource(), GetTimestampValue(), SetCca(), GetCcaResult()
    void SetAgcGainManual(uint8_t GainStep);
    // SetCadParams(), SetCad()

    // LoRa Packet Radio Commands

    void SetLoraModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate, uint8_t LowDataRateOptimize);
    void SetLoraPacketParams(uint16_t PreambleLength, uint8_t HeaderType, uint8_t PayloadLength, uint8_t Crc, uint8_t InvertIQ);
    // SetLoraSynchTimeout(), SetLoraSyncword()
    // SetLoraSideDetConfig(), SetLoraSideDetSyncword()
    // SetLoraCadParams(), SetLoraCAD()
    void GetLoraRxStats(uint16_t* pkt_rx, uint16_t* pkt_crc_error, uint16_t* header_crc_error, uint16_t* false_synch);
    void GetLoraPacketStatus(int16_t* Rssi, int16_t* RssiSignal, int8_t* Snr,
                             uint8_t* Crc, uint8_t* CR, uint16_t* PktLen, uint8_t* Detector);
    void GetLoraPacketStatus(int16_t* Rssi, int16_t* RssiSignal, int8_t* Snr);
    // SetLoraAddress(), SetLoraHopping(), SetLoraSideDetCad()

    // FSK Packet Radio Commands

    void SetModulationParamsFSK(uint32_t BitRate, uint8_t PulseShape, uint8_t Bandwidth, uint32_t Fdev_hz);
    void SetPacketParamsFSK(uint16_t PreambleLength, uint8_t PreambleDetectorLength,
                            uint8_t long_preamble_mode, uint8_t pld_lenUnit, uint8_t addr_comp,
                            uint8_t PacketFormat, uint16_t PayloadLength, uint8_t Crc, uint8_t dc_free);
    void SetWhiteningParamsFSK(uint8_t WhitenType, uint16_t Init);
    void SetCrcParamsFSK(uint32_t Polynom, uint32_t Init);
    void SetSyncWordFSK(uint64_t SyncWord, uint8_t bit_order, uint8_t nb_bits);
    void SetAddressFSK(uint8_t addr_node, uint8_t addr_bcast);
    void GetRxStatsFSK(uint16_t* pkt_rx, uint16_t* pkt_crc_error, uint16_t* len_error, uint16_t* pbl_det,
                       uint16_t* sync_ok, uint16_t* sync_fail, uint16_t* timeout);
    void GetPacketStatusFSK(uint16_t* PktLen, uint16_t* RssiAvg, uint16_t* RssiSync, uint8_t* AddrMatchBcast,
                            uint8_t* AddrMatchNode, uint8_t* Lqi);
    void GetPacketStatusFSK(uint16_t* RssiAvg, uint16_t* RssiSync, uint8_t* Lqi);

    // FLRC Packet Radio Commands

    void SetModulationParamsFLRC(uint8_t BitrateBw, uint8_t CodingRate, uint8_t PulseShape);
    void SetPacketParamsFLRC(uint8_t AgcPblLen, uint8_t SyncWordLength,
                             uint8_t SyncWordTx, uint8_t SyncWordMatch,
                             uint8_t PacketFormat, uint8_t CrcLength,
                             uint16_t PayloadLength);
    void GetRxStatsFLRC(int16_t* stats); /* ?? */
    void GetPacketStatusFLRC(int16_t* RssiSync); /*??*/
    void SetSyncWordFLRC(uint8_t SyncWordNum, uint32_t SyncWord);

    // auxiliary methods

    void EnableSx127xCompatibility() {}

  private:
    uint8_t _status1; // status is two bytes
    uint8_t _status2;
};

//-------------------------------------------------------
// Enum Definitions
//-------------------------------------------------------

// WriteCommand(uint16_t opcode, uint8_t* data, uint16_t len)
typedef enum {
    // FIFO Read/Write Commands (Table 5-1)
    LR20XX_CMD_READ_RADIO_RX_FIFO               = 0x0001,
    LR20XX_CMD_WRITE_RADIO_TX_FIFO              = 0x0002,
    // Register/Memory Access (Table 5-2)
    LR20XX_CMD_WRITE_REG_MEM_32                 = 0x0104,
    LR20XX_CMD_WRITE_REG_MEM_MASK_32            = 0x0105,
    LR20XX_CMD_READ_REG_MEM_32                  = 0x0106,
    // System Configuration (Table 5-3)
    LR20XX_CMD_GET_STATUS                       = 0x0100,
    LR20XX_CMD_GET_VERSION                      = 0x0101,
    LR20XX_CMD_GET_ERRORS                       = 0x0110,
    LR20XX_CMD_CLEAR_ERRORS                     = 0x0111,
    LR20XX_CMD_SET_DIO_FUNCTION                 = 0x0112,
    LR20XX_CMD_SET_DIO_RF_SWITCH_CONFIG         = 0x0113,
    _LR20XX_CMD_CLEAR_FIFO_IRQ_FLAGS             = 0x0114, // nu
    LR20XX_CMD_SET_DIO_IRQ_CONFIG               = 0x0115,
    LR20XX_CMD_CLEAR_IRQ                        = 0x0116,
    LR20XX_CMD_GET_AND_CLEAR_IRQ_STATUS         = 0x0117,
    _LR20XX_CMD_CONFIG_LF_CLOCK                  = 0x0118, // nu
    _LR20XX_CMD_CONFIG_CLK_OUTPUTS               = 0x0119, // nu
    _LR20XX_CMD_CONFIG_FIFO_IRQ                  = 0x011A, // nu
    _LR20XX_CMD_GET_FIFO_IRQ_FLAGS               = 0x011B, // nu
    LR20XX_CMD_GET_RX_FIFO_LEVEL                = 0x011C,
    LR20XX_CMD_GET_TX_FIFO_LEVEL                = 0x011D,
    LR20XX_CMD_CLEAR_RX_FIFO                    = 0x011E,
    LR20XX_CMD_CLEAR_TX_FIFO                    = 0x011F,
    LR20XX_CMD_SET_TCXO_MODE                    = 0x0120,
    LR20XX_CMD_SET_REG_MODE                     = 0x0121,
    LR20XX_CMD_CALIBRATE                        = 0x0122,
    LR20XX_CMD_CALIB_FE                         = 0x0123,
    _LR20XX_CMD_GET_VBAT                         = 0x0124, // nu
    _LR20XX_CMD_GET_TEMP                         = 0x0125, // nu
    _LR20XX_CMD_GET_RANDOM_NUMBER                = 0x0126, // nu
    _LR20XX_CMD_SET_SLEEP                        = 0x0127, // nu
    LR20XX_CMD_SET_STANDBY                      = 0x0128,
    LR20XX_CMD_SET_FS                           = 0x0129,
    _LR20XX_CMD_ADD_REGISTER_TO_RETENTION_MEM    = 0x012A, // nu
    _LR20XX_CMD_SET_EOL_CFG                      = 0x0130, // nu
    _LR20XX_CMD_CONFIGURE_XOSC                   = 0x0131, // nu
    _LR20XX_CMD_SET_TEMP_COMP_CFG                = 0x0132, // nu
    _LR20XX_CMD_SET_NTC_PARAMS                   = 0x0133, // nu
    // Common Radio Commands (Table 5-4)
    LR20XX_CMD_SET_RF_FREQUENCY                 = 0x0200,
    LR20XX_CMD_SET_RX_PATH                      = 0x0201,
    LR20XX_CMD_SET_PA_CONFIG                    = 0x0202,
    LR20XX_CMD_SET_TX_PARAMS                    = 0x0203,
    LR20XX_CMD_SET_RSSI_CALIBRATION             = 0x0205,
    LR20XX_CMD_SET_RXTX_FALLBACK_MODE           = 0x0206,
    LR20XX_CMD_SET_PACKET_TYPE                  = 0x0207,
    LR20XX_CMD_GET_PACKET_TYPE                  = 0x0208,
    _LR20XX_CMD_STOP_TIMEOUT_ON_PREAMBLE         = 0x0209, // nu
    LR20XX_CMD_RESET_RX_STATS                   = 0x020A,
    LR20XX_CMD_GET_RSSI_INST                    = 0x020B,
    LR20XX_CMD_SET_RX                           = 0x020C,
    LR20XX_CMD_SET_TX                           = 0x020D,
    _LR20XX_CMD_SET_TX_TEST_MODE                 = 0x020E, // nu
    LR20XX_CMD_SEL_PA                           = 0x020F,
    _LR20XX_CMD_SET_RX_DUTY_CYCLE                = 0x0210, // nu
    _LR20XX_CMD_AUTO_TXRX                        = 0x0211, // nu
    LR20XX_CMD_GET_RX_PKT_LENGTH                = 0x0212,
    LR20XX_CMD_SET_DEFAULT_RX_TX_TIMEOUT        = 0x0215,
    _LR20XX_CMD_SET_TIMESTAMP_SOURCE             = 0x0216, // nu
    _LR20XX_CMD_GET_TIMESTAMP_VALUE              = 0x0217, // nu
    _LR20XX_CMD_SET_CCA                          = 0x0218, // nu
    _LR20XX_CMD_GET_CCA_RESULT                   = 0x0219, // nu
    LR20XX_CMD_SET_AGC_GAIN_MANUAL              = 0x021A,
    _LR20XX_CMD_SET_CAD_PARAMS                   = 0x021B, // nu
    _LR20XX_CMD_SET_CAD                          = 0x021C, // nu
    // LoRa Commands (Table 5-5)
    LR20XX_CMD_SET_LORA_MODULATION_PARAMS       = 0x0220,
    LR20XX_CMD_SET_LORA_PACKET_PARAMS           = 0x0221,
    _LR20XX_CMD_SET_LORA_SYNCH_TIMEOUT           = 0x0222, // nu
    _LR20XX_CMD_SET_LORA_SYNC_WORD               = 0x0223, // nu
    _LR20XX_CMD_SET_LORA_SIDE_DET_CONFIG         = 0x0224, // nu
    _LR20XX_CMD_SET_LORA_SIDE_DET_SYNCWORD       = 0x0225, // nu
    _LR20XX_CMD_SET_LORA_CAD_PARAMS              = 0x0227, // nu
    _LR20XX_CMD_SET_LORA_CAD                     = 0x0228, // nu
    LR20XX_CMD_GET_LORA_RX_STATS                = 0x0229,
    LR20XX_CMD_GET_LORA_PACKET_STATUS           = 0x022A,
    _LR20XX_CMD_SET_LORA_ADDRESS                 = 0x022B, // nu
    _LR20XX_CMD_SET_LORA_HOPPING                 = 0x022C, // nu
    _LR20XX_CMD_SET_LORA_SIDE_DET_CAD            = 0x021E, // nu
    // FSK Commands (Table 5-6)
    LR20XX_CMD_SET_FSK_MODULATION_PARAMS        = 0x0240,
    LR20XX_CMD_SET_FSK_PACKET_PARAMS            = 0x0241,
    LR20XX_CMD_SET_FSK_WHITENING_PARAMS         = 0x0242,
    LR20XX_CMD_SET_FSK_CRC_PARAMS               = 0x0243,
    LR20XX_CMD_SET_FSK_SYNC_WORD                = 0x0244,
    LR20XX_CMD_SET_FSK_ADDRESS                  = 0x0245,
    LR20XX_CMD_GET_FSK_RX_STATS                 = 0x0246,
    LR20XX_CMD_GET_FSK_PACKET_STATUS            = 0x0247,
    // FLRC Commands (Table 5-7)
    LR20XX_CMD_SET_FLRC_MODULATION_PARAMS       = 0x0248,
    LR20XX_CMD_SET_FLRC_PACKET_PARAMS           = 0x0249,
    LR20XX_CMD_GET_FLRC_RX_STATS                = 0x024A,
    LR20XX_CMD_GET_FLRC_PACKET_STATUS           = 0x024B,
    LR20XX_CMD_SET_FLRC_SYNCWORD                = 0x024C,
} LR20XX_CMD_ENUM;

typedef enum {
    LR20XX_IRQ_NONE                         = 0x00000000, // table 5-17, page 91f
    LR20XX_IRQ_RX_FIFO                      = 0x00000001, // bit 0
    LR20XX_IRQ_TX_FIFO                      = 0x00000002, // bit 1
    LR20XX_IRQ_RNG_REQ_VLD                  = 0x00000004, // bit 2
    LR20XX_IRQ_TX_TIMESTAMP                 = 0x00000008, // bit 3
    LR20XX_IRQ_RX_TIMESTAMP                 = 0x00000010, // bit 4
    LR20XX_IRQ_PREAMBLE_DETECTED            = 0x00000020, // bit 5
    LR20XX_IRQ_SYNC_WORD_VALID              = 0x00000040, // bit 6
    LR20XX_IRQ_CAD_DETECTED                 = 0x00000080, // bit 7
    LR20XX_IRQ_LORA_HDR_TIMESTAMP           = 0x00000100, // bit 8
    LR20XX_IRQ_LORA_HEADER_ERR              = 0x00000200, // bit 9
    LR20XX_IRQ_LOW_BATTERY                  = 0x00000400, // bit 10
    LR20XX_IRQ_PA_OCP_OVP                   = 0x00000800, // bit 11
    LR20XX_IRQ_LORA_TX_RX_HOP               = 0x00001000, // bit 12
    LR20XX_IRQ_SYNC_FAIL                    = 0x00002000, // bit 13
    LR20XX_IRQ_LORA_SYMBOL_END              = 0x00004000, // bit 14
    LR20XX_IRQ_LORA_TIMESTAMP_STAT          = 0x00008000, // bit 15
    LR20XX_IRQ_ERROR                        = 0x00010000, // bit 16
    LR20XX_IRQ_CMD_ERROR                    = 0x00020000, // bit 17
    LR20XX_IRQ_RX_DONE                      = 0x00040000, // bit 18
    LR20XX_IRQ_TX_DONE                      = 0x00080000, // bit 19
    LR20XX_IRQ_CAD_DONE                     = 0x00100000, // bit 20
    LR20XX_IRQ_TIMEOUT                      = 0x00200000, // bit 21
    LR20XX_IRQ_CRC_ERROR                    = 0x00400000, // bit 22
    LR20XX_IRQ_LEN_ERROR                    = 0x00800000, // bit 23
    LR20XX_IRQ_ADDR_ERROR                   = 0x01000000, // bit 24
    LR20XX_IRQ_FHSS                         = 0x02000000, // bit 25
    LR20XX_IRQ_INTER_PACKET_1               = 0x04000000, // bit 26
    LR20XX_IRQ_INTER_PACKET_2               = 0x08000000, // bit 27
    LR20XX_IRQ_RNG_RESP_DONE                = 0x10000000, // bit 28
    LR20XX_IRQ_RNG_REQ_DIS                  = 0x20000000, // bit 29
    LR20XX_IRQ_RNG_EXCH_VLD                 = 0x40000000, // bit 30
    LR20XX_IRQ_RNG_TIMEOUT                  = 0x80000000, // bit 31
    LR20XX_IRQ_ALL                          = 0xFFFFFFFF,
} LR20XX_IRQ_ENUM;

//-------------------------------------------------------
// Enum Definitions, System Configuration Commands
//-------------------------------------------------------

// cmd 0x0100 void GetStatus(uint8_t* Status1, uint8_t* Status2)
#define LR20XX_STATUS_CMD_BIT               9
#define LR20XX_STATUS_CMD_MASK              0x07
#define LR20XX_STATUS_ISR_BIT               8
#define LR20XX_STATUS_ISR_MASK              0x01
#define LR20XX_STATUS_RESET_SOURCE_BIT      4
#define LR20XX_STATUS_RESET_SOURCE_MASK     0x0F
#define LR20XX_STATUS_CHIP_MODE_BIT         0
#define LR20XX_STATUS_CHIP_MODE_MASK        0x07

typedef enum {
    LR20XX_STATUS_CMD_FAIL                  = 0, // table 6-37, page 112, bits 11:9, ((status >> 9) & 0x07)
    LR20XX_STATUS_CMD_PERR                  = 1,
    LR20XX_STATUS_CMD_OK                    = 2,
    LR20XX_STATUS_CMD_DAT                   = 3,
} LR20XX_STATUS_CMD_ENUM;

typedef enum {
    LR20XX_STATUS_ISR_NO_ACTIVE             = 0, // table 6-37, page 112, bit 8, ((status >> 8) & 0x01)
    LR20XX_STATUS_ISR_AT_LEAST_ONE_ACTIVE   = 1,
} LR20XX_STATUS_ISR_ENUM;

typedef enum {
    LR20XX_STATUS_RESET_SOURCE_CLEARED      = 0, // table 6-37, page 112, bits 7:4, ((status >> 4) & 0x0F)
    LR20XX_STATUS_RESET_SOURCE_ANALOG       = 1,
    LR20XX_STATUS_RESET_SOURCE_NRESET_PIN   = 2,
} LR20XX_STATUS_RESET_SOURCE_ENUM;

typedef enum {
    LR20XX_STATUS_CHIP_MODE_SLEEP           = 0, // table 6-37, page 112, bits 2:0, ((status >> 0) & 0x07)
    LR20XX_STATUS_CHIP_MODE_STDBY           = 1,
    LR20XX_STATUS_CHIP_MODE_STDBY_XOSC      = 2,
    LR20XX_STATUS_CHIP_MODE_FS              = 3,
    LR20XX_STATUS_CHIP_MODE_RX              = 4,
    LR20XX_STATUS_CHIP_MODE_TX              = 5,
} LR20XX_STATUS_CHIP_MODE_ENUM;

// cmd 0x0110 void GetErrors(uint16_t* ErrorStat)
typedef enum {
    LR20XX_ERROR_HF_XOSC_START_ERR          = 0x0001, // table 6-43, page 114
    LR20XX_ERROR_LF_XOSC_START_ERR          = 0x0002,
    LR20XX_ERROR_PLL_LOCK_ERR               = 0x0004,
    LR20XX_ERROR_LF_RC_CALIB_ERR            = 0x0008,
    LR20XX_ERROR_HF_RC_CALIB_ERR            = 0x0010,
    LR20XX_ERROR_PLL_CALIB_ERR              = 0x0020,
    LR20XX_ERROR_AAF_CALIB_ERR              = 0x0040,
    LR20XX_ERROR_IMG_CALIB_ERR              = 0x0080,
    LR20XX_ERROR_CHIP_BUSY                  = 0x0100,
    LR20XX_ERROR_RXFREQ_NO_FE_CAL_ERR       = 0x0200,
    LR20XX_ERROR_MEAS_UNIT_ADC_CALIB_ERR    = 0x0400,
    LR20XX_ERROR_PA_OFFSET_CALIB_ERR        = 0x0800,
} LR20XX_ERROR_ENUM;

// cmd 0x0112 void SetDioFunction(uint8_t Dio, uint8_t Func, uint8_t PullDrive)
typedef enum {
    LR20XX_DIO_FUNCTION_NONE                = 0, // table 6-44, page 115
    LR20XX_DIO_FUNCTION_IRQ                 = 1,
    LR20XX_DIO_FUNCTION_RF_SWITCH           = 2,
    LR20XX_DIO_FUNCTION_GPIO_OUTPUT_LOW     = 5,
    LR20XX_DIO_FUNCTION_GPIO_OUTPUT_HIGH    = 6,
    LR20XX_DIO_FUNCTION_HF_CLK_OUT          = 7,
    LR20XX_DIO_FUNCTION_LF_CLK_OUT          = 8, // for DIO 7 - 11 only
    LR20XX_DIO_FUNCTION_TX_TRIGGER          = 9,
    LR20XX_DIO_FUNCTION_RX_TRIGGER          = 10,
} LR20XX_DIO_FUNCTION_ENUM;

typedef enum {
    LR20XX_DIO_SLEEP_PULL_NONE              = 0,
    LR20XX_DIO_SLEEP_PULL_DOWN              = 1,
    LR20XX_DIO_SLEEP_PULL_UP                = 2,
    LR20XX_DIO_SLEEP_PULL_AUTO              = 3,
} LR20XX_DIO_SLEEP_PULL_ENUM;

// cmd 0x0113 void SetDioRfSwitchConfig(uint8_t Dio, uint8_t Config)
typedef enum {
    LR20XX_DIO_RF_SWITCH_CONFIG_STANDBY     = 0x01, // table 6-45, page 116
    LR20XX_DIO_RF_SWITCH_CONFIG_RX_LF       = 0x02,
    LR20XX_DIO_RF_SWITCH_CONFIG_TX_LF       = 0x04,
    LR20XX_DIO_RF_SWITCH_CONFIG_RX_HF       = 0x08,
    LR20XX_DIO_RF_SWITCH_CONFIG_TX_HF       = 0x10,
} LR20XX_DIO_RF_SWITCH_CONFIG_ENUM;

// cmd 0x0120 void SetTcxoMode(uint8_t Tune, uint32_t StartTime)
typedef enum {
    LR20XX_TCXO_SUPPLY_VOLTAGE_1_6V         = 0, // table 6-65, page 123, 1.6 V
    LR20XX_TCXO_SUPPLY_VOLTAGE_1_7          = 1, // 1.7 V
    LR20XX_TCXO_SUPPLY_VOLTAGE_1_8          = 2, // 1.8 V
    LR20XX_TCXO_SUPPLY_VOLTAGE_2_2          = 3, // 2.2 V
    LR20XX_TCXO_SUPPLY_VOLTAGE_2_4          = 4, // 2.4 V
    LR20XX_TCXO_SUPPLY_VOLTAGE_2_7          = 5, // 2.7 V
    LR20XX_TCXO_SUPPLY_VOLTAGE_3_0          = 6, // 3.0 V
    LR20XX_TCXO_SUPPLY_VOLTAGE_3_3          = 7, // 3.3 V
} LR20XX_TCXO_SUPPLY_VOLTAGE_ENUM;

// cmd 0x0121 void SetRegMode(uint8_t SimoUsage)
typedef enum {
    LR20XX_SIMO_USAGE_OFF                   = 0, // table 6-26, page 105
    LR20XX_SIMO_USAGE_NORMAL                = 2,
} LR20XX_SIMO_USAGE_ENUM;

// cmd 0x0122 void Calibrate(uint8_t BlocksToCalibrate)
typedef enum {
    LR20XX_CALIBRATE_LF_RC                  = 0x01, // table 6-28, page 106
    LR20XX_CALIBRATE_HF_RC                  = 0x02,
    LR20XX_CALIBRATE_PLL                    = 0x04,
    LR20XX_CALIBRATE_AAF                    = 0x08,
    LR20XX_CALIBRATE_MU                     = 0x20,
    LR20XX_CALIBRATE_PA_OFF                 = 0x40,
} LR20XX_CALIBRATE_ENUM;

// cmd 0x0123 void CalibFE(uint16_t Freq1, uint16_t Freq2, uint16_t Freq3)
typedef enum {
    LR20XX_CAL_FE_LF_PATH_BIT               = 0x0000, // table 6-29, page 107
    LR20XX_CAL_FE_HF_PATH_BIT               = 0x8000,
} LR20XX_CAL_FE_PATH_ENUM;

typedef enum {
    // TODO ?????
} LR20XX_CALIB_FE_ENUM;

// cmd 0x0128 void SetStandby(uint8_t StandbyMode)
typedef enum {
    LR20XX_STANDBY_MODE_RC                  = 0, // table 6-8, page 96
    LR20XX_STANDBY_MODE_XOSC                = 1,
} LR20XX_STANDBY_MODE_ENUM;

// cmd 0x0201 void SetRxPath(uint8_t RxPath, uint8_t RxBoost)
typedef enum {
    LR20XX_RX_PATH_LF                       = 0, // table 7-2, page 127
    LR20XX_RX_PATH_HF                       = 1,
} LR20XX_RX_PATH_ENUM;

typedef enum {
    LR20XX_RX_BOOST_0_LF                    = 0, // table 7-2, page 127, recommended value for LF
    LR20XX_RX_BOOST_4_HF                    = 4, // recommended value for HF
} LR20XX_RX_BOOST_ENUM;

//-------------------------------------------------------
// Enum Definitions, Common Radio Commands
//-------------------------------------------------------

// cmd 0x0206 void SetRxTxFallbackMode(uint8_t FallbackMode)
typedef enum {
    LR20XX_RX_TX_FALLBACK_MODE_STDBY_RC     = 1, // table 6-13, page 98
    LR20XX_RX_TX_FALLBACK_MODE_STDBY_XOSC   = 2,
    LR20XX_RX_TX_FALLBACK_MODE_FS           = 3,
} LR20XX_FALLBACK_MODE_ENUM;

// cmd 0x0202 void SetPaConfig(uint8_t PaSel, uint8_t PaLfMode, uint8_t PaLfDutyCycle, uint8_t PaLfSlices, uint8_t PaHfDutyCycle)
typedef enum {
    LR20XX_PA_LF_MODE_IF_PA_HF              = 0, // helper to handle defaults according to datasheet
    LR20XX_PA_LF_DUTY_CYCLE_IF_PA_HF        = 6,
    LR20XX_PA_LF_SLICE_IF_PA_HF             = 7,
    LR20XX_PA_HF_DUTY_CYCLE_IF_PA_LF        = 16,
} LR20XX_PA_AUXILLIARY_ENUM;

typedef enum {
    LR20XX_PA_SEL_LF                        = 0, // table 7-15, page 132
    LR20XX_PA_SEL_HF                        = 1,
} LR20XX_PA_SEL_ENUM;

typedef enum {
    LR20XX_PA_LF_MODE_FSM                   = 0,
} LR20XX_PA_SLF_MODE_ENUM;

// cmd 0x0203 void SetTxParams(uint8_t Power, uint8_t RampTime)
// LF: -19 .. 44 = -9.5 dBm ... 22 dBm in 0.5 dBm steps
typedef enum {
    LR20XX_POWER_LF_m9_DBM                  = -18, // 0.12 mW
    LR20XX_POWER_LF_0_DBM                   = 0,   // 1 mW
    LR20XX_POWER_LF_10_DBM                  = 20, // 10 mW
    LR20XX_POWER_LF_17_DBM                  = 34, // 50 mW
    LR20XX_POWER_LF_20_DBM                  = 40, // 100 mW
    LR20XX_POWER_LF_22_DBM                  = 44, // 158 mW
    LR20XX_POWER_LF_MIN                     = LR20XX_POWER_LF_m9_DBM,
    LR20XX_POWER_LF_MAX                     = LR20XX_POWER_LF_22_DBM,
} LR20XX_POWER_LF_ENUM;

// HF: -39 .. 24 = -19.5 dBm ... 12 dBm in 0.5 dBm steps
typedef enum {
    LR20XX_POWER_HF_m19_5_DBM               = -39,
    LR20XX_POWER_HF_m18_DBM                 = -36,
    LR20XX_POWER_HF_m10_DBM                 = -20, // 0.1 mW
    LR20XX_POWER_HF_0_DBM                   = 0,   // 1 mW
    LR20XX_POWER_HF_3_DBM                   = 6,   // 1.995 mW
    LR20XX_POWER_HF_6_DBM                   = 12,  // 3.981 mW
    LR20XX_POWER_HF_10_DBM                  = 20,  // 10 mW
    LR20XX_POWER_HF_12_DBM                  = 24,
    LR20XX_POWER_HF_MIN                     = LR20XX_POWER_HF_m18_DBM,
    LR20XX_POWER_HF_MAX                     = LR20XX_POWER_HF_12_DBM,
} LR20XX_POWER_HF_ENUM;

typedef enum {
    LR20XX_RAMPTIME_2_US                    = 0, // table 7-21, page 136
    LR20XX_RAMPTIME_4_US                    = 1,
    LR20XX_RAMPTIME_8_US                    = 2,
    LR20XX_RAMPTIME_16_US                   = 3,
    LR20XX_RAMPTIME_32_US                   = 4,
    LR20XX_RAMPTIME_48_US                   = 5,
    LR20XX_RAMPTIME_64_US                   = 6,
    LR20XX_RAMPTIME_80_US                   = 7,
    LR20XX_RAMPTIME_96_US                   = 8,
    LR20XX_RAMPTIME_112_US                  = 9,
    LR20XX_RAMPTIME_128_US                  = 10,
    LR20XX_RAMPTIME_144_US                  = 11,
    LR20XX_RAMPTIME_160_US                  = 12,
    LR20XX_RAMPTIME_176_US                  = 13,
    LR20XX_RAMPTIME_192_US                  = 14,
    LR20XX_RAMPTIME_208_US                  = 15,
    LR20XX_RAMPTIME_240_US                  = 16,
    LR20XX_RAMPTIME_272_US                  = 17,
    LR20XX_RAMPTIME_304_US                  = 18,
} LR20XX_RAMPTIME_ENUM;

// cmd 0x0207 void SetPacketType(uint8_t PacketType)
typedef enum {
    LR20XX_PACKET_TYPE_LORA                 = 0, // table 8-1, page 138
    LR20XX_PACKET_TYPE_FSK                  = 2,
    LR20XX_PACKET_TYPE_BLE                  = 3,
    LR20XX_PACKET_TYPE_RTTOF                = 4,
    LR20XX_PACKET_TYPE_FLRC                 = 5,
    LR20XX_PACKET_TYPE_BPSK                 = 6,
    LR20XX_PACKET_TYPE_LR_FHSS              = 7,
    LR20XX_PACKET_TYPE_WMBUS                = 8,
    LR20XX_PACKET_TYPE_WISUN                = 9,
    LR20XX_PACKET_TYPE_OOK                  = 10,
    LR20XX_PACKET_TYPE_ZWAVE                = 12,
    LR20XX_PACKET_TYPE_OQPSK                = 13,
} LR20XX_PACKET_TYPE_ENUM;

// cmd 0x0212 void SetAgcGainManual(uint8_t GainStep)
typedef enum {
    LR20XX_AGC_GAIN_MANUAL_AFC              = 0, // enables AFC
    LR20XX_AGC_GAIN_MANUAL_1                = 1,
    LR20XX_AGC_GAIN_MANUAL_2                = 2,
    LR20XX_AGC_GAIN_MANUAL_3                = 3,
    LR20XX_AGC_GAIN_MANUAL_4                = 4,
    LR20XX_AGC_GAIN_MANUAL_5                = 5,
    LR20XX_AGC_GAIN_MANUAL_6                = 6,
    LR20XX_AGC_GAIN_MANUAL_7                = 7,
    LR20XX_AGC_GAIN_MANUAL_8                = 8,
    LR20XX_AGC_GAIN_MANUAL_9                = 9,
    LR20XX_AGC_GAIN_MANUAL_10               = 10,
    LR20XX_AGC_GAIN_MANUAL_11               = 11,
    LR20XX_AGC_GAIN_MANUAL_12               = 12,
    LR20XX_AGC_GAIN_MANUAL_13               = 13,
} LR20XX_AGC_GAIN_MANUAL_ENUM;

//-------------------------------------------------------
// Enum Definitions, LoRa Packet Radio Commands
//-------------------------------------------------------

// cmd 0x0220 void SetLoraModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth,
//                   uint8_t CodingRate, uint8_t LowDataRateOptimize)
typedef enum {
    LR20XX_LORA_SF5                         = 5, // table 9-2, page 145
    LR20XX_LORA_SF6                         = 6,
    LR20XX_LORA_SF7                         = 7,
    LR20XX_LORA_SF8                         = 8,
    LR20XX_LORA_SF9                         = 9,
    LR20XX_LORA_SF10                        = 10,
    LR20XX_LORA_SF11                        = 11,
    LR20XX_LORA_SF12                        = 12,
} LR20XX_LORA_SF_ENUM;

typedef enum {
    LR20XX_LORA_BW_31                       = 2, // table 9-3, page 145
    LR20XX_LORA_BW_62                       = 3,
    LR20XX_LORA_BW_125                      = 4,
    LR20XX_LORA_BW_250                      = 5,
    LR20XX_LORA_BW_500                      = 6,
    LR20XX_LORA_BW_1000                     = 7,
    LR20XX_LORA_BW_41                       = 10,
    LR20XX_LORA_BW_83                       = 11,
    LR20XX_LORA_BW_101                      = 12,
    LR20XX_LORA_BW_203                      = 13,
    LR20XX_LORA_BW_406                      = 14,
    LR20XX_LORA_BW_812                      = 15,
} LR20XX_LORA_BW_ENUM;

typedef enum {
    LR20XX_LORA_CR_4_5                      = 1, // table 9-4, page 146
    LR20XX_LORA_CR_4_6                      = 2,
    LR20XX_LORA_CR_4_7                      = 3,
    LR20XX_LORA_CR_4_8                      = 4,
    LR20XX_LORA_CR_LI_4_5                   = 5,
    LR20XX_LORA_CR_LI_4_6                   = 6,
    LR20XX_LORA_CR_LI_4_8                   = 7,
    LR20XX_LORA_CR_LI_CONV_4_6              = 8,
    LR20XX_LORA_CR_LI_CONV_4_8              = 9,
} LR20XX_LORA_CR_ENUM;

typedef enum {
    LR20XX_LORA_LDRO_OFF                    = 0, // recommended for: SF5-10, BW250 / SF11, BW500, BW1000
    LR20XX_LORA_LDRO_ON                     = 1, // all other BW and SF11/12 (including SX1280 compatible bandwidths BW800/400/200)
} LR20XX_LORA_LDRO_ENUM;

// cmd 0x0221 void SetLoraPacketParams(uint16_t PreambleLength, uint8_t HeaderType, uint8_t PayloadLength,
//                   uint8_t Crc, uint8_t InvertIQ)
typedef enum {
    LR20XX_LORA_HEADER_EXPLICIT             = 0, // table 9-5, page 147
    LR20XX_LORA_HEADER_IMPLICIT             = 1,
    LR20XX_LORA_HEADER_ENABLE               = LR20XX_LORA_HEADER_EXPLICIT,
    LR20XX_LORA_HEADER_DISABLE              = LR20XX_LORA_HEADER_IMPLICIT,
} LR20XX_LORA_HEADER_ENUM;

typedef enum {
    LR20XX_LORA_CRC_DISABLE                 = 0,
    LR20XX_LORA_CRC_ENABLE                  = 1,
} LR20XX_LORA_CRC_ENUM;

typedef enum {
    LR20XX_LORA_IQ_NORMAL                   = 0,
    LR20XX_LORA_IQ_INVERTED                 = 1,
} LR20XX_LORA_IQMODE_ENUM;

//-------------------------------------------------------
// Enum Definitions, FSK Packet Radio Commands
//-------------------------------------------------------

// cmd 0x0240 void SetModulationParamsFSK(uint32_t BitRate, uint8_t PulseShape, uint8_t Bandwidth, uint32_t Fdev_hz)
typedef enum {
    LR20XX_FSK_BITRATE_DIRECT               = 0x00000000, // table 11-1, page 161
    LR20XX_FSK_BITRATE_FRACTIONAL           = 0x80000000,
} LR20XX_FSK_BITRATE_ENUM;

typedef enum {
    LR20XX_FSK_PULSESHAPE_OFF               = 0, // table 11-1, page 161
    LR20XX_FSK_PULSESHAPE_BT_20             = 2, // Gaussian Bandwidth-Time bit period 2.0
    LR20XX_FSK_PULSESHAPE_BT_03             = 4, // Gaussian Bandwidth-Time bit period 0.3
    LR20XX_FSK_PULSESHAPE_BT_05             = 5, // Gaussian Bandwidth-Time bit period 0.5
    LR20XX_FSK_PULSESHAPE_BT_07             = 6, // Gaussian Bandwidth-Time bit period BT 0.7
    LR20XX_FSK_PULSESHAPE_BT_1              = 7, // Gaussian Bandwidth-Time bit period BT 1.0
} LR20XX_FSK_PULSESHAPE_ENUM;

typedef enum {
    LR20XX_FSK_BW_3076                      = 0, // table 11-2, page 162
} LR20XX_FSK_BANDWIDTH_ENUM;

//cmd 0x0241 void SetPacketParamsFSK(uint16_t PreambleLength, uint8_t PreambleDetectorLength,
//                  uint8_t long_preamble_mode, uint8_t pld_lenUnit, uint8_t addr_comp,
//                  uint8_t PacketFormat, uint16_t PayloadLength, uint8_t Crc, uint8_t dc_free)
typedef enum {
    LR20XX_FSK_PREAMBLE_DETECTOR_LENGT_OFF          = 0x00, // table 11-4, page 163
    LR20XX_FSK_PREAMBLE_DETECTOR_LENGTH_8BITS       = 0x08,
    LR20XX_FSK_PREAMBLE_DETECTOR_LENGTH_16BITS      = 0x10,
    LR20XX_FSK_PREAMBLE_DETECTOR_LENGTH_24BITS      = 0x18,
    LR20XX_FSK_PREAMBLE_DETECTOR_LENGTH_32BITS      = 0x20,
} LR20XX_FSK_PREAMBLE_DETECTOR_LENGTH_ENUM;

typedef enum {
    LR20XX_FSK_LONG_PREAMBLE_MODE_OFF       = 0,
    LR20XX_FSK_LONG_PREAMBLE_MODE_ON        = 1,
} LR20XX_FSK_LONG_PREAMBLE_MODE_ENUM;

typedef enum {
    LR20XX_FSK_PREAMBLE_LEN_UNIT_BYTES      = 0,
    LR20XX_FSK_PREAMBLE_LEN_UNIT_BITS       = 1,
} LR20XX_FSK_PREAMBLE_LEN_UNIT_ENUM;

typedef enum {
    LR20XX_FSK_ADDR_COMP_DISABLE                    = 0, // table 11-5, page 164
    LR20XX_FSK_ADDR_COMP_NODE_ADDR                  = 1,
    LR20XX_FSK_ADDR_COMP_NODE_AND_BROADCAST_ADDR    = 2,
} LR20XX_FSK_ADDR_COMP_ENUM;

typedef enum {
    LR20XX_FSK_PKT_FORMAT_FIXED_LEN                 = 0, // table 11-6, page 164
    LR20XX_FSK_PKT_FORMAT_VARIABLE_LEN_8BIT_HEADER  = 1, // SX126x/SX127x compatible
    LR20XX_FSK_PKT_FORMAT_VARIABLE_LEN_9BIT_HEADER  = 2, // SX128x compatible
    LR20XX_FSK_PKT_FORMAT_VARIABLE_LEN_16BIT_HEADER = 3,
} LR20XX_FSK_PKT_FORMAT_ENUM;

typedef enum {
    LR20XX_FSK_CRC_OFF                      = 0, // table 11-7, page 165
    LR20XX_FSK_CRC_1_BYTE                   = 1,
    LR20XX_FSK_CRC_2_BYTES                  = 2,
    LR20XX_FSK_CRC_3_BYTES                  = 3,
    LR20XX_FSK_CRC_4_BYTES                  = 4,
    LR20XX_FSK_CRC_1_BYTE_INV               = 9,
    LR20XX_FSK_CRC_2_BYTES_INV              = 10,
    LR20XX_FSK_CRC_3_BYTES_INV              = 10,
    LR20XX_FSK_CRC_4_BYTES_INV              = 10,
} LR20XX_FSK_CRC_ENUM;

typedef enum {
    LR20XX_FSK_WHITEN_TYPE_SX126x_SX127x    = 0, // table 11-9, page 166
    LR20XX_FSK_WHITEN_TYPE_SX128x           = 1,
} LR20XX_FSK_WHITEN_TYPE_ENUM;

//-------------------------------------------------------
// Enum Definitions, FLRC Packet Radio Commands
//-------------------------------------------------------

// cmd 0x0248 SetModulationParamsFLRC(uint8_t BitrateBw, uint8_t CodingRate, uint8_t PulseShape)
typedef enum {
    LR20XX_FLRC_BR_2600_BW_2666             = 0, // table 18-4, page 203
    LR20XX_FLRC_BR_2080_BW_2222             = 1,
    LR20XX_FLRC_BR_1300_BW_1333             = 2,
    LR20XX_FLRC_BR_1040_BW_1333             = 3,
    LR20XX_FLRC_BR_650_BW_888               = 4,
    LR20XX_FLRC_BR_520_BW_769               = 5,
    LR20XX_FLRC_BR_325_BW_444               = 6,
    LR20XX_FLRC_BR_260_BW_444               = 7,
} LR20XX_FLRC_BR_BW_ENUM;

typedef enum {
    LR20XX_FLRC_CR_1_2                      = 0, // table 18-4, page 203, CR = 1/2
    LR20XX_FLRC_CR_3_4                      = 1, // CR = 3/4
    LR20XX_FLRC_CR_1_0                      = 2, // CR = 1
    LR20XX_FLRC_CR_2_3                      = 3, // CR = 2/3
} LR20XX_FLRC_CR_ENUM;

typedef enum {
    LR20XX_FLRC_PULSESHAPE_OFF              = 0, // table 18-4, page 203, same as for FSK
    LR20XX_FLRC_PULSESHAPE_BT_20            = 2,
    LR20XX_FLRC_PULSESHAPE_BT_03            = 4,
    LR20XX_FLRC_PULSESHAPE_BT_05            = 5,
    LR20XX_FLRC_PULSESHAPE_BT_07            = 6,
    LR20XX_FLRC_PULSESHAPE_BT_1             = 7,
} LR20XX_FLRC_PULSESHAPE_ENUM;

// cmd 0x0249 SetPacketParamsFLRC(uint8_t AgcPblLen, uint8_t SyncWordLength,
//   uint8_t SyncWordTx, uint8_t SyncWordMatch, uint8_t PacketType,
//   uint8_t CrcLength, uint16_t PayloadLength)
typedef enum {
    LR20XX_FLRC_PREAMBLE_LENGTH_4_BITS      = 0, // table 18-5, page 204
    LR20XX_FLRC_PREAMBLE_LENGTH_8_BITS      = 1,
    LR20XX_FLRC_PREAMBLE_LENGTH_12_BITS     = 2,
    LR20XX_FLRC_PREAMBLE_LENGTH_16_BITS     = 3,
    LR20XX_FLRC_PREAMBLE_LENGTH_20_BITS     = 4,
    LR20XX_FLRC_PREAMBLE_LENGTH_24_BITS     = 5,
    LR20XX_FLRC_PREAMBLE_LENGTH_28_BITS     = 6,
    LR20XX_FLRC_PREAMBLE_LENGTH_32_BITS     = 7,
} LR20XX_FLRC_PREAMBLE_LENGTH_ENUM;

typedef enum {
    LR20XX_FLRC_SYNC_LEN_NONE               = 0, // table 18-5, page 204
    LR20XX_FLRC_SYNC_LEN_16_BITS            = 1,
    LR20XX_FLRC_SYNC_LEN_32_BITS            = 2,
} LR20XX_FLRC_SYNC_LEN_ENUM;

typedef enum {
    LR20XX_FLRC_SYNC_TX_NONE                = 0, // table 18-5, page 204
    LR20XX_FLRC_SYNC_TX_1                   = 1,
    LR20XX_FLRC_SYNC_TX_2                   = 2,
    LR20XX_FLRC_SYNC_TX_3                   = 3,
} LR20XX_FLRC_SYNC_TX_ENUM;

typedef enum {
    LR20XX_FLRC_SYNC_MATCH_OFF              = 0, // table 18-5, page 204
    LR20XX_FLRC_SYNC_MATCH_1                = 1,
    LR20XX_FLRC_SYNC_MATCH_2                = 2,
    LR20XX_FLRC_SYNC_MATCH_1_OR_2           = 3,
    LR20XX_FLRC_SYNC_MATCH_3                = 4,
    LR20XX_FLRC_SYNC_MATCH_1_OR_3           = 5,
    LR20XX_FLRC_SYNC_MATCH_2_OR_3           = 6,
    LR20XX_FLRC_SYNC_MATCH_1_2_OR_3         = 7,
} LR20XX_FLRC_SYNC_MATCH_ENUM;

typedef enum {
    LR20XX_FLRC_PKT_FORMAT_VAR_LEN          = 0, // table 18-5, page 204
    LR20XX_FLRC_PKT_FORMAT_FIX_LEN          = 1,
} LR20XX_FLRC_PKT_FORMAT_ENUM;

typedef enum {
    LR20XX_FLRC_CRC_OFF                     = 0, // table 18-5, page 204
    LR20XX_FLRC_CRC_16_BITS                 = 1,
    LR20XX_FLRC_CRC_24_BITS                 = 2,
    LR20XX_FLRC_CRC_32_BITS                 = 3,
} LR20XX_FLRC_CRC_ENUM;


#endif // LR20XX_LIB_H
