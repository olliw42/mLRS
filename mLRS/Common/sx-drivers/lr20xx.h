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

    virtual void WaitOnBusy(void) {}
    virtual void SetDelay(uint16_t tmo_us) { (void)tmo_us; }

    // spi methods

    void SpiTransfer(uint8_t dataout, uint8_t* datain) { SpiTransfer(&dataout, datain, 1); }
    void SpiRead(uint8_t* datain) { SpiRead(datain, 1); }
    void SpiWrite(uint8_t dataout) { SpiWrite(&dataout, 1); }

    // low level methods, usually no need to use them

    void WriteCommand(uint16_t opcode, uint8_t* data, uint8_t len);
    void ReadCommand(uint16_t opcode, uint8_t* data, uint8_t len);
    void WriteBuffer(uint8_t* data, uint8_t len);
    void ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len);

    void WriteCommand(uint16_t opcode) { WriteCommand(opcode, nullptr, 0); }
    void WriteCommand(uint16_t opcode, uint8_t data) { WriteCommand(opcode, &data, 1); }
    uint8_t ReadCommand(uint16_t opcode) { uint8_t data; ReadCommand(opcode, &data, 1); return data; }


    void WriteRegMem32(uint16_t opcode, uint8_t* data, uint8_t len);
    void WriteRegMemMask32(uint16_t opcode, uint8_t* data, uint8_t len);
    void ReadRegMem32(uint16_t opcode, uint8_t* data, uint8_t len);

    // common methods

    void GetStatus(uint8_t* Status1, uint8_t* Status2);
    void SetDioFunction(uint8_t Dio, uint8_t Func, uint8_t pull_drive);
    void SetDioRfSwitchConfig(uint8_t Dio, uint8_t tx_hf, uint8_t rx_hf, uint8_t tx_lf, uint8_t rx_lf, uint8_t standby);
    void SetDioIrqConfig(uint8_t Dio, uint32_t Irq);
    void ClearIrq(uint32_t IrqsToClear);
    uint32_t GetAndClearIrqStatus(void);

    void ConfigLfClock(uint8_t lf_clock);
    void ConfigClockOutputs(uint8_t hf_clk_out_scaling);
    void SetTcxoMode(uint8_t tune, uint32_t start_time);
    void SetRegMode(uint8_t simo_usage);
    void Calibrate(uint8_t blocks_to_calibrate);
    void CalibFE(uint16_t Freq1, uint16_t Freq2, uint16_t Freq3);
    void SetStandby(uint8_t standby_mode);
    void SetFs(void);
    void SetRfFrequency(uint32_t RfFreq);
    void SetRxPath(uint8_t rx_path, uint8_t rx_boost);
    void SetPaConfig(uint8_t pa_sel, uint8_t pa_lf_mode, uint8_t pa_lf_duty_cycle,
                     uint8_t pa_lf_slices,
                     uint8_t pa_hf_duty_cycle);
    void SetTxParams(uint8_t Power, uint8_t RampTime);
    void SetRssiCalibration(uint8_t rx_path_hf, uint8_t rx_path_lf, uint32_t* /*??*/ table);
    void SetRxTxFallbackMode(uint8_t FallbackMode);
    void SetPacketType(uint8_t PacketType);
    uint8_t /*??*/ GetPacketType(void);

    void ResetRxStats(void);
    void SetRx(uint32_t RxTimeout); // 24 bits only, similar to sx126x
    void SetTx(uint32_t TxTimeout); // 24 bits only, similar to sx126x
    void SelPa(uint8_t pa_sel);
    uint32_t /*??*/ GetRxPktLength(void);
    void SetDefaultRxTxTimeout(uint32_t rx_timeout, uint32_t tx_timeout);
    void SetAgcGainManual(uint8_t GainStep);
    void SetLoraModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate, uint8_t LowDataRateOptimize);
    void SetLoraPacketParams(uint16_t PreambleLength, uint8_t PayloadLength, uint8_t HeaderType, uint8_t Crc, uint8_t InvertIQ);
    void SetLoraSyncword(uint8_t Syncword);
    void GetLoraRxStats(uint16_t* pkt_rx, uint16_t* pkt_crc_error, uint16_t* header_crc_error, uint16_t* false_synch);
    uint32_t /*??*/ GetPacketStatus(void);
    void SetLoraAddress(uint8_t addr_comp_len, uint8_t addr_comp_pos, uint8_t addr);


    // Tx methods


    // Rx methods


    // auxiliary methods

    void EnableSx127xCompatibility(void);



    // GFSK methods

    void SetModulationParamsFSK(uint32_t br_bps, uint8_t PulseShape, uint8_t Bandwidth, uint32_t Fdev_hz);
    void SetPacketParamsFSK(/* uint16_t PreambleLength,
                             uint8_t PreambleDetectorLength,
                             uint8_t SyncWordLength,
                             uint8_t AddrComp,
                             uint8_t PacketType,
                             uint8_t PayloadLength,
                             uint8_t CRCType,
                             uint8_t Whitening); */
        uint16_t pbl_len_tx, uint8_t pbl_detect, uint8_t long_preamble_mode, uint8_t pld_lenUnit, uint8_t addr_comp,
        uint8_t pkt_format, uint16_t pld_len, uint8_t Crc, uint8_t dc_free);
    void SetWhiteningParamsFSK(uint8_t whiten_type, uint16_t Init);
    void SetCrcParamsFSK(uint32_t polynom, uint32_t Init);
    void SetSyncWordFSK(uint64_t SyncWord, uint8_t bit_order, uint8_t nb_bits);
    void SetAddressFSK(uint8_t addr_node, uint8_t addr_bcast);
    void GetRxStatsFSK(int16_t* stats); /* ?? */
    void GetPacketStatusFSK(int16_t* RssiSync); /* ?? */

    // FLRC methods

    void SetModulationParamsFLRC(uint8_t BitrateBw, uint8_t CodingRate, uint8_t PulseShape);
    void SetPacketParamsFLRC(uint8_t AgcPblLen, uint8_t SyncWordLength,
                             uint8_t SyncWordTx, uint8_t SyncWordMatch,
                             uint8_t PacketType, uint8_t CrcLength,
                             uint16_t PayloadLength);
    void SetSyncWordFLRC(uint8_t SyncWordNum, uint32_t SyncWord);
    void GetPacketStatusFLRC(int16_t* RssiSync); /*??*/
    void GetRxStatsFLRC(int16_t* stats); /* ?? */


    // FIFO methods

    void WriteRadioTxFifo(uint8_t* data, uint8_t len);
    void ReadRadioRxFifo(uint8_t* data, uint8_t len);
    void ClearFifoIrqFlags(uint8_t RxFifoFlagsToClear, uint8_t TxFifoFlagsToCleadata);
    void ConfigFifoIrq(uint8_t rx_fifo_irq_enable, uint8_t tx_fifo_irq_enable,
            uint16_t rx_high_threshold, uint16_t tx_low_threshold, uint16_t rx_low_threshold, uint16_t tx_high_threshold);
    uint32_t /*??*/ void GetFifoIrqFlags(void);
    uint32_t /*??*/ GetAndClearFifoIrqFlags(void);
    uint16_t /*??*/ GetRxFifoLevel(void);
    uint16_t GetTxFifoLevel(void);
    void ClearRxFifo(void);
    void ClearTxFifo(void);





    // other methods

    void GetVersion(uint8_t* HwVersion, uint8_t* UseCase, uint8_t* FwMajor, uint8_t* FwMinor);
    void GetErrors(uint8_t* HwVersion, uint8_t* UseCase, uint8_t* FwMajor, uint8_t* FwMinor);
    void ClearErrors(uint8_t* HwVersion, uint8_t* UseCase, uint8_t* FwMajor, uint8_t* FwMinor);


  private:
    uint8_t _status1; // status is now two bytes
    uint8_t _status2;
};

//-------------------------------------------------------
// Enum Definitions
//-------------------------------------------------------

// WriteCommand(uint16_t opcode, uint8_t* data, uint8_t len)
typedef enum {
    // FIFO Read/Write Commands (Table 5-1)
    LR20XX_CMD_READ_RADIO_RX_FIFO               = 0x0001,
    LR20XX_CMD_WRITE_RADIO_TX_FIFO              = 0x0002,
    // Register/Memory Access (Table 5-2)
    LR20XX_CMD_WRITE_REG_MEM_32                 = 0x0104,
    LR20XX_CMD_WRITE_REG_MEM_MASK_32            = 0x0105,
    LR20XX_CMD_READ_REG_MEM_32                  = 0x0106,
    // System Configuration (Table 5-3)
    LR20XX_CMD_GET_STATUS = 0x0100,
    LR20XX_CMD_GET_VERSION = 0x0101,
    LR20XX_CMD_GET_ERRORS = 0x0110,
    LR20XX_CMD_CLEAR_ERRORS = 0x0111,
    LR20XX_CMD_SET_DIO_FUNCTION = 0x0112,
    LR20XX_CMD_SET_DIO_AS_RF_SWITCH = 0x0113,
    LR20XX_CMD_CLEAR_FIFO_IRQ_FLAGS = 0x0114,
    LR20XX_CMD_SET_DIO_IRQ_CONFIG = 0x0115,
    LR20XX_CMD_CLEAR_IRQ = 0x0116,
    LR20XX_CMD_GET_AND_CLEAR_IRQ_STATUS = 0x0117,
    LR20XX_CMD_CONFIG_LF_CLOCK = 0x0118,
    LR20XX_CMD_CONFIG_CLK_OUTPUTS = 0x0119,
    LR20XX_CMD_CONFIG_FIFO_IRQ = 0x011A,
    LR20XX_CMD_GET_FIFO_IRQ_FLAGS = 0x011B,
    LR20XX_CMD_GET_RX_FIFO_LEVEL = 0x011C,
    LR20XX_CMD_GET_TX_FIFO_LEVEL = 0x011D,
    LR20XX_CMD_CLEAR_RX_FIFO = 0x011E,
    LR20XX_CMD_CLEAR_TX_FIFO = 0x011F,
    LR20XX_CMD_SET_TCXO_MODE = 0x0120,
    LR20XX_CMD_SET_REG_MODE = 0x0121,
    LR20XX_CMD_CALIBRATE = 0x0122,
    LR20XX_CMD_CALIB_FE = 0x0123,
    LR20XX_CMD_GET_VBAT = 0x0124,
    LR20XX_CMD_GET_TEMP = 0x0125,
    LR20XX_CMD_GET_RANDOM_NUMBER = 0x0126,
    LR20XX_CMD_SET_SLEEP = 0x0127,
    LR20XX_CMD_SET_STANDBY = 0x0128,
    LR20XX_CMD_SET_FS                           = 0x0129,
    LR20XX_CMD_ADD_REGISTER_TO_RETENTION_MEM    = 0x012A,
    LR20XX_CMD_SET_EOL_CFG                      = 0x0130,
    LR20XX_CMD_CONFIGURE_XOSC                   = 0x0131,
    LR20XX_CMD_SET_TEMP_COMP_CFG                = 0x0132,
    LR20XX_CMD_SET_NTC_PARAMS                   = 0x0133,
    // Common Radio Commands (Table 5-4)
    LR20XX_CMD_SET_RF_FREQUENCY                 = 0x0200,
    LR20XX_CMD_SET_RX_PATH                      = 0x0201,
    LR20XX_CMD_SET_PA_CONFIG                    = 0x0202,
    LR20XX_CMD_SET_TX_PARAMS                    = 0x0203,
    LR20XX_CMD_SET_RSSI_CALIBRATION             = 0x0205,
    LR20XX_CMD_SET_RXTX_FALLBACK_MODE           = 0x0206,
    LR20XX_CMD_SET_PACKET_TYPE                  = 0x0207,
    LR20XX_CMD_GET_PACKET_TYPE                  = 0x0208,
    LR20XX_CMD_STOP_TIMEOUT_ON_PREAMBLE         = 0x0209,
    LR20XX_CMD_RESET_STATS                      = 0x020A,
    LR20XX_CMD_GET_RSSI_INST                    = 0x020B,
    LR20XX_CMD_SET_RX                           = 0x020C,
    LR20XX_CMD_SET_TX                           = 0x020D,
    LR20XX_CMD_SET_TX_TEST_MODE                 = 0x020E,
    LR20XX_CMD_SEL_PA                           = 0x020F,
    LR20XX_CMD_SET_RX_DUTY_CYCLE                = 0x0210,
    LR20XX_CMD_AUTO_TXRX                        = 0x0211,
    LR20XX_CMD_GET_RX_PKT_LENGTH                = 0x0212,
    LR20XX_CMD_SET_DEFAULT_RX_TX_TIMEOUT        = 0x0215,
    LR20XX_CMD_SET_TIMESTAMP_SOURCE             = 0x0216,
    LR20XX_CMD_GET_TIMESTAMP_VALUE              = 0x0217,
    LR20XX_CMD_SET_CCA                          = 0x0218,
    LR20XX_CMD_GET_CCA_RESULT                   = 0x0219,
    LR20XX_CMD_SET_AGC_GAIN_MANUAL = 0x021A,
    LR20XX_CMD_SET_CAD_PARAMS = 0x021B,
    LR20XX_CMD_SET_CAD = 0x021C,
    LR20XX_CMD_SET_LORA_SIDE_DET_CAD = 0x021E,
    // LoRa Commands (Table 5-5)
    LR20XX_CMD_SET_MODULATION_PARAMS = 0x0220,
    LR20XX_CMD_SET_PACKET_PARAMS = 0x0221,
    LR20XX_CMD_SET_LORA_SYNCH_TIMEOUT = 0x0222,
    LR20XX_CMD_SET_LORA_SYNC_WORD = 0x0223,
    LR20XX_CMD_SET_LORA_SIDE_DET_CONFIG = 0x0224,
    LR20XX_CMD_SET_LORA_SIDE_DET_SYNCWORD = 0x0225,
    LR20XX_CMD_SET_LORA_CAD_PARAMS = 0x0227,
    LR20XX_CMD_SET_LORA_CAD = 0x0228,
    LR20XX_CMD_GET_LORA_RX_STATS = 0x0229,
    LR20XX_CMD_GET_PACKET_STATUS = 0x022A,
    LR20XX_CMD_SET_LORA_ADDRESS = 0x022B,
    LR20XX_CMD_SET_LORA_HOPPING = 0x022C,
    // FSK Commands (Table 5-6)
    LR20XX_CMD_SET_MODULATION_PARAMS_GFSK = 0x0240,
    LR20XX_CMD_SET_PACKET_PARAMS_GFSK = 0x0241,
    LR20XX_CMD_SET_GFSK_WHIT_PARAMS = 0x0242,
    LR20XX_CMD_SET_GFSK_CRC_PARAMS = 0x0243,
    LR20XX_CMD_SET_GFSK_SYNC_WORD = 0x0244,
    LR20XX_CMD_SET_GFSK_ADDRESS = 0x0245,
    LR20XX_CMD_GET_GFSK_RX_STATS = 0x0246,
    LR20XX_CMD_GET_PACKET_STATUS_GFSK = 0x0247,
    // FLRC Commands (Table 5-7)
    LR20XX_CMD_SET_MODULATION_PARAMS_FLRC = 0x0248,
    LR20XX_CMD_SET_PACKET_PARAMS_FLRC = 0x0249,
    LR20XX_CMD_GET_FLRC_RX_STATS = 0x024A,
    LR20XX_CMD_GET_PACKET_STATUS_FLRC = 0x024B,
    LR20XX_CMD_SET_FLRC_SYNCWORD = 0x024C,
    // LR-FHSS Commands (Table 5-9)
    LR20XX_CMD_LR_FHSS_BUILD_FRAME = 0x0256,
    LR20XX_CMD_LR_FHSS_SET_SYNC_WORD = 0x0257,
} LR20XX_CMD_ENUM;

// cmd 0x0100 void GetStatus(uint8_t* Stat1, uint8_t* Stat2, uint32_t*
// IrqStatus)
typedef enum {
  LR20XX_STATUS_CMD_FAIL = 0x00 << 1, // table 3-2, page 28
  LR20XX_STATUS_CMD_PERR = 0x01 << 1,
  LR20XX_STATUS_CMD_OK = 0x02 << 1,
  LR20XX_STATUS_CMD_DAT = 0x03 << 1,
} LR20XX_STATUS_COMMAND_ENUM;

typedef enum {
  LR20XX_STATUS_INTERRUPT_INACTIVE = 0x00, // table 3-2, page 28
  LR20XX_STATUS_INTERRUPT_ACTIVE = 0x01,
} LR20XX_STATUS_INTERRUPT_ENUM;

typedef enum {
  LR20XX_STATUS_RESET_CLEARED = 0x00 << 4, // table 3-3, page 29
  LR20XX_STATUS_RESET_ANALOG = 0x01 << 4,
  LR20XX_STATUS_RESET_EXTERNAL = 0x02 << 4,
  LR20XX_STATUS_RESET_SYSTEM = 0x03 << 4,
  LR20XX_STATUS_RESET_WATCHDOG = 0x04 << 4,
  LR20XX_STATUS_RESET_WAKEUP = 0x05 << 4,
  LR20XX_STATUS_RESET_RTC = 0x06 << 4,
} LR20XX_STATUS_RESET_ENUM;

typedef enum {
  LR20XX_STATUS_MODE_SLEEP = 0x00, // table 6-38
  LR20XX_STATUS_MODE_STANDBY_RC = 0x01,
  LR20XX_STATUS_MODE_STANDBY_EXT = 0x02,
  LR20XX_STATUS_MODE_FS = 0x03,
  LR20XX_STATUS_MODE_RX = 0x04,
  LR20XX_STATUS_MODE_TX = 0x05,
} LR20XX_STATUS_MODE_ENUM;

typedef enum {
  LR20XX_STATUS_BOOTLOADER_BOOTLOADER = 0x00, // table 3-3, page 29
  LR20XX_STATUS_BOOTLOADER_FLASH = 0x01,
} LR20XX_STATUS_BOOTLOADER_ENUM;

//-------------------------------------------------------
// Enum Definitions Common
//-------------------------------------------------------

// cmd 0x011C void SetStandby(uint8_t StandbyConfig)
typedef enum {
    LR20XX_STDBY_CONFIG_STDBY_RC = 0x00, // table 2-1, page 15
    LR20XX_STDBY_CONFIG_STDBY_XOSC = 0x01
} LR20XX_STDBY_CONFIG_ENUM;

// cmd 0x020E void SetPacketType(uint8_t PacketType)
typedef enum {
    LR20XX_PACKET_TYPE_LORA = 0x00,
    LR20XX_PACKET_TYPE_RFU_1 = 0x01,
    LR20XX_PACKET_TYPE_FSK = 0x02,
    LR20XX_PACKET_TYPE_BLE = 0x03,
    LR20XX_PACKET_TYPE_RTTOF = 0x04,
    LR20XX_PACKET_TYPE_FLRC = 0x05,
    LR20XX_PACKET_TYPE_BPSK = 0x06,
    LR20XX_PACKET_TYPE_LR_FHSS = 0x07,
    LR20XX_PACKET_TYPE_WMBUS = 0x08,
    LR20XX_PACKET_TYPE_WISUN = 0x09,
    LR20XX_PACKET_TYPE_OOK = 0x0A,
    LR20XX_PACKET_TYPE_RFU_B = 0x0B,
    LR20XX_PACKET_TYPE_ZWAVE = 0x0C,
    LR20XX_PACKET_TYPE_OQPSK = 0x0D,
    LR20XX_PACKET_TYPE_NONE = 0xFF, // Helper
} LR20XX_PACKET_TYPE_ENUM;

// cmd 0x020B void SetRfFrequency(uint32_t RfFrequency)

// cmd 0x0112 void SetDioAsRfSwitch(uint8_t RfSwEnable, uint8_t RfSwStbyCfg,
// uint8_t RfSwRxCfg, uint8_t RfSwTxCfg, uint8_t TxHPCfg, uint8_t RfSwTxHfCfg)
typedef enum {
    LR20XX_RF_SW_CFG_STANDBY = 0x01, // Bit 0
    LR20XX_RF_SW_CFG_RX_LF = 0x02,   // Bit 1
    LR20XX_RF_SW_CFG_TX_LF = 0x04,   // Bit 2
    LR20XX_RF_SW_CFG_RX_HF = 0x08,   // Bit 3
    LR20XX_RF_SW_CFG_TX_HF = 0x10,   // Bit 4
} LR20XX_RF_SW_CFG_ENUM;

typedef enum {
    LR20XX_SYSTEM_LFCLK_RC = 0x00,  // Use internal RC 32kHz (Default)
    LR20XX_SYSTEM_LFCLK_EXT = 0x02, // Use externally provided 32kHz signal on DIO11
} LR20XX_SYSTEM_LFCLK_CFG_ENUM;

// cmd 0x0117 void SetTcxoMode(uint8_t OutputVoltage, uint32_t Delay)
typedef enum {
    LR20XX_TCXO_OUTPUT_1_6 = 0x00, // // table 6-3, page 48, 1.6V
    LR20XX_TCXO_OUTPUT_1_7 = 0x01, // 1.7V
    LR20XX_TCXO_OUTPUT_1_8 = 0x02, // 1.8V
    LR20XX_TCXO_OUTPUT_2_2 = 0x03, // 2.2V
    LR20XX_TCXO_OUTPUT_2_4 = 0x04, // 2.4V
    LR20XX_TCXO_OUTPUT_2_7 = 0x05, // 2.7V
    LR20XX_TCXO_OUTPUT_3_0 = 0x06, // 3.0V
    LR20XX_TCXO_OUTPUT_3_3 = 0x07, // 3.3V
} LR20XX_TCXO_OUTPUT_ENUM;

// cmd 0x020F void SetModulationParams(uint8_t SpreadingFactor, uint8_t
// Bandwidth, uint8_t CodingRate, uint8_t LowDataRateOptimize)
typedef enum {
    LR20XX_LORA_SF5 = 0x05, // table 8-4, page 67
    LR20XX_LORA_SF6 = 0x06,
    LR20XX_LORA_SF7 = 0x07,
    LR20XX_LORA_SF8 = 0x08,
    LR20XX_LORA_SF9 = 0x09,
    LR20XX_LORA_SF10 = 0x0A,
    LR20XX_LORA_SF11 = 0x0B,
    LR20XX_LORA_SF12 = 0x0C,
} LR20XX_LORA_SF_ENUM;

typedef enum {
    LR20XX_LORA_BW_31 = 0x02,
    LR20XX_LORA_BW_62 = 0x03,
    LR20XX_LORA_BW_125 = 0x04,
    LR20XX_LORA_BW_250 = 0x05,
    LR20XX_LORA_BW_500 = 0x06,
    LR20XX_LORA_BW_1000 = 0x07,
    LR20XX_LORA_BW_41 = 0x0A,
    LR20XX_LORA_BW_83 = 0x0B,
    LR20XX_LORA_BW_101 = 0x0C,
    LR20XX_LORA_BW_203 = 0x0D,
    LR20XX_LORA_BW_406 = 0x0E,
    LR20XX_LORA_BW_812 = 0x0F,
} LR20XX_LORA_BW_ENUM;

typedef enum {
    LR20XX_LORA_CR_4_5 = 0x01, // table 8-4, page 67
    LR20XX_LORA_CR_4_6 = 0x02,
    LR20XX_LORA_CR_4_7 = 0x03,
    LR20XX_LORA_CR_4_8 = 0x04,
    LR20XX_LORA_CR_LI_4_5 = 0x05,
    LR20XX_LORA_CR_LI_4_6 = 0x06,
    LR20XX_LORA_CR_LI_4_8 = 0x07,
    LR20XX_LORA_CR_LI_CONV_4_6 = 0x08,
    LR20XX_LORA_CR_LI_CONV_4_8 = 0x09,
} LR20XX_LORA_CR_ENUM;

typedef enum {
    LR20XX_LORA_LDR_OFF = 0x00, // table 8-4, page 67
    LR20XX_LORA_LDR_ON = 0x01,
} LR20XX_LORA_LDR_ENUM;

// cmd 0x0210 void SetPacketParams(uint16_t PreambleLength, uint8_t HeaderType,
// uint8_t PayloadLength, uint8_t Crc, uint8_t InvertIQ)
typedef enum {
    LR20XX_LORA_HEADER_EXPLICIT = 0x00, // table 8-5, page 68
    LR20XX_LORA_HEADER_IMPLICIT = 0x01,
    LR20XX_LORA_HEADER_ENABLE = LR20XX_LORA_HEADER_EXPLICIT,  // compatiblility?
    LR20XX_LORA_HEADER_DISABLE = LR20XX_LORA_HEADER_IMPLICIT, // compatiblility?
} LR20XX_LORA_HEADER_ENUM;

typedef enum {
    LR20XX_LORA_CRC_DISABLE = 0x00, // table 8-5, page 68
    LR20XX_LORA_CRC_ENABLE = 0x01,
} LR20XX_LORA_CRC_ENUM;

typedef enum {
    LR20XX_LORA_IQ_NORMAL = 0x00, // table 8-5, page 68
    LR20XX_LORA_IQ_INVERTED = 0x01,
} LR20XX_LORA_IQMODE_ENUM;

// cmd 0x0115 void SetDioIrqParams(uint32_t Irq1ToEnable, uint32_t Irq2ToEnable)
// IRQ bits per LR2021 datasheet Table 5-17
typedef enum {
    LR20XX_IRQ_NONE = 0x00000000,
    LR20XX_IRQ_RX_FIFO = 0x00000001,             // bit 0: Rx FIFO threshold
    LR20XX_IRQ_TX_FIFO = 0x00000002,             // bit 1: Tx FIFO threshold
    LR20XX_IRQ_RNG_REQ_VLD = 0x00000004,         // bit 2: Ranging request valid
    LR20XX_IRQ_TX_TIMESTAMP = 0x00000008,        // bit 3: Tx timestamp
    LR20XX_IRQ_RX_TIMESTAMP = 0x00000010,        // bit 4: Rx timestamp
    LR20XX_IRQ_PREAMBLE_DETECTED = 0x00000020,   // bit 5: Preamble detected
    LR20XX_IRQ_SYNC_WORD_VALID = 0x00000040,     // bit 6: LoRa header/Syncword valid
    LR20XX_IRQ_CAD_DETECTED = 0x00000080,        // bit 7: Channel activity detected
    LR20XX_IRQ_LORA_HDR_TIMESTAMP = 0x00000100,  // bit 8: LoRa header timestamp
    LR20XX_IRQ_HEADER_ERROR = 0x00000200,        // bit 9: LoRa header CRC error
    LR20XX_IRQ_LOW_BATTERY = 0x00000400,         // bit 10: Low battery
    LR20XX_IRQ_PA_OCP_OVP = 0x00000800,          // bit 11: PA OCP/OVP
    LR20XX_IRQ_LORA_TX_RX_HOP = 0x00001000,      // bit 12: LoRa intra-packet hopping
    LR20XX_IRQ_SYNC_FAIL = 0x00002000,           // bit 13: Syncword match failed
    LR20XX_IRQ_LORA_SYMBOL_END = 0x00004000,     // bit 14: End of LoRa symbol
    LR20XX_IRQ_LORA_TIMESTAMP_STAT = 0x00008000, // bit 15: Timestamp stats available
    LR20XX_IRQ_ERROR = 0x00010000,               // bit 16: General error
    LR20XX_IRQ_CMD_ERROR = 0x00020000,           // bit 17: Command error
    LR20XX_IRQ_RX_DONE = 0x00040000,             // bit 18: Packet reception done
    LR20XX_IRQ_TX_DONE = 0x00080000,             // bit 19: Packet transmission done
    LR20XX_IRQ_CAD_DONE = 0x00100000,            // bit 20: CAD finished
    LR20XX_IRQ_TIMEOUT = 0x00200000,             // bit 21: Rx or Tx timeout
    LR20XX_IRQ_CRC_ERROR = 0x00400000,           // bit 22: CRC error
    LR20XX_IRQ_LEN_ERROR = 0x00800000,           // bit 23: Length error
    LR20XX_IRQ_ADDR_ERROR = 0x01000000,          // bit 24: Address error
    LR20XX_IRQ_FHSS = 0x02000000,                // bit 25: FHSS ramp-up
    LR20XX_IRQ_INTER_PACKET_1 = 0x04000000,      // bit 26: Inter-packet 1
    LR20XX_IRQ_INTER_PACKET_2 = 0x08000000,      // bit 27: Inter-packet 2
    LR20XX_IRQ_RNG_RESP_DONE = 0x10000000,       // bit 28: Ranging response done
    LR20XX_IRQ_RNG_REQ_DIS = 0x20000000,         // bit 29: Ranging request discarded
    LR20XX_IRQ_RNG_EXCH_VLD = 0x40000000,        // bit 30: Ranging exchange valid
    LR20XX_IRQ_RNG_TIMEOUT = 0x80000000,         // bit 31: Ranging timeout
    LR20XX_IRQ_ALL = 0xFFFFFFFF,
} LR20XX_IRQ_ENUM;

// cmd 0x020A void SetTx(uint32_t TxTimeout)
// cmd 0x0209 void SetRx(uint32_t RxTimeout)

typedef enum {
    LR20XX_TIMEOUT_TX_NONE = 0x000000,
} LR20XX_TIMEOUT_TX_ENUM;

typedef enum {
    LR20XX_TIMEOUT_RX_SINGLE = 0x000000,
    LR20XX_TIMEOUT_RX_CONTINUOUS = 0xFFFFFF,
} LR20XX_TIMEOUT_RX_ENUM;

//-------------------------------------------------------
// Enum Definitions Tx
//-------------------------------------------------------

// cmd 0x0215 void SetPaConfig(uint8_t PaSel, uint8_t RegPaSupply, uint8_t
// PaDutyCycle, uint8_t PaHPSel)
typedef enum {
    LR20XX_PA_SEL_LF = 0x00,
    LR20XX_PA_SEL_HF = 0x01,
} LR20XX_PA_SEL_ENUM;

typedef enum {
    LR20XX_PA_DUTY_CYCLE_22_DBM = 0x04, // to be used with high power PA
    LR20XX_PA_DUTY_CYCLE_14_DBM = 0x07, // to be used with low power PA
} LR20XX_PA_DUTY_CYCLE_ENUM;

typedef enum {
    LR20XX_PA_HP_SEL_22_DBM = 0x07, // this setting only affects the high power PA
} LR20XX_PA_HP_SEL_ENUM;

// cmd 0x0211 void SetTxParams(uint8_t Power, uint8_t RampTime)
typedef enum {
    LR20XX_RAMPTIME_2_US = 0x00,
    LR20XX_RAMPTIME_4_US = 0x01,
    LR20XX_RAMPTIME_8_US = 0x02,
    LR20XX_RAMPTIME_16_US = 0x03,
    LR20XX_RAMPTIME_32_US = 0x04,
    LR20XX_RAMPTIME_48_US = 0x05,
    LR20XX_RAMPTIME_64_US = 0x06,
    LR20XX_RAMPTIME_80_US = 0x07,
    LR20XX_RAMPTIME_96_US = 0x08,
    LR20XX_RAMPTIME_112_US = 0x09,
    LR20XX_RAMPTIME_128_US = 0x0A,
    LR20XX_RAMPTIME_144_US = 0x0B,
    LR20XX_RAMPTIME_160_US = 0x0C,
    LR20XX_RAMPTIME_176_US = 0x0D,
    LR20XX_RAMPTIME_192_US = 0x0E,
    LR20XX_RAMPTIME_208_US = 0x0F,
    LR20XX_RAMPTIME_240_US = 0x10,
    LR20XX_RAMPTIME_272_US = 0x11,
    LR20XX_RAMPTIME_304_US = 0x12,
} LR20XX_RAMPTIME_ENUM;

// added for convenience
// -9 (0xF7) to +22 (0x16) dBm by step of 1 dB if high power PA is selected
typedef enum {
    LR20XX_POWER_m9_DBM = -9, // 0.12 mW
    LR20XX_POWER_0_DBM = 0,   // 1 mW
    LR20XX_POWER_10_DBM = 10, // 10 mW
    LR20XX_POWER_17_DBM = 17, // 50 mW
    LR20XX_POWER_20_DBM = 20, // 100 mW
    LR20XX_POWER_22_DBM = 22, // 158 mW
    LR20XX_POWER_MIN = LR20XX_POWER_m9_DBM,
    LR20XX_POWER_MAX = LR20XX_POWER_22_DBM,
} LR20XX_POWER_ENUM;

//-------------------------------------------------------
// Enum Definitions Rx
//-------------------------------------------------------

// cmd void 0x0204 GetPacketStatus(int16_t* RssiSync, int8_t* Snr)
// cmd void 0x010A ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len)

//-------------------------------------------------------
// Enum Definitions Auxiliary
//-------------------------------------------------------

// cmd 0x0110 void SetRegMode(uint8_t RegModeParam)
typedef enum {
    LR20XX_REGULATOR_MODE_LDO = 0x00, // table 5-3, page 42
    LR20XX_REGULATOR_MODE_DCDC = 0x02,
} LR20XX_REGULATOR_MODE_ENUM;

// cmd 0x0213 void SetRxTxFallbackMode(uint8_t FallbackMode)
typedef enum {
    LR20XX_RX_TX_FALLBACK_MODE_STDBY_RC = 0x01, // table 7-5, page 53
    LR20XX_RX_TX_FALLBACK_MODE_STDBY_XOSC = 0x02,
    LR20XX_RX_TX_FALLBACK_MODE_FS = 0x03,
} LR20XX_FALLBACK_MODE_ENUM;

// cmd 0x011D void SetFs(void)

// cmd 0x0227 void SetRxBoosted(uint8_t RxBoosted)
typedef enum {
    LR20XX_RX_BOOST_NONE = 0x00, // table 7-15, page 58
    LR20XX_RX_BOOST_EN = 0x04,
} LR20XX_LNAGAIN_MODE_ENUM;

// cmd 0x0111 void CalibImage(uint8_t Freq1, uint8_t Freq2)
typedef enum {
    LR20XX_CAL_FE_430_MHZ_1 = 0x6B, // table 2-3, page 16
    LR20XX_CAL_FE_430_MHZ_2 = 0x6F,
    LR20XX_CAL_FE_470_MHZ_1 = 0x75,
    LR20XX_CAL_FE_470_MHZ_2 = 0x81,
    LR20XX_CAL_FE_779_MHZ_1 = 0xC1,
    LR20XX_CAL_FE_779_MHZ_2 = 0xC5,
    LR20XX_CAL_FE_863_MHZ_1 = 0xD7,
    LR20XX_CAL_FE_863_MHZ_2 = 0xDB,
    LR20XX_CAL_FE_902_MHZ_1 = 0xE1,
    LR20XX_CAL_FE_902_MHZ_2 = 0xE9,
} LR20XX_CALIB_FE_ENUM;

//-------------------------------------------------------
// Enum Definitions GFSK
//-------------------------------------------------------

// cmd 0x020F SetModulationParamsGFSK(uint32_t br_bps, uint8_t PulseShape,
// uint8_t Bandwidth, uint32_t Fdev_hz)
typedef enum {
    LR20XX_GFSK_PULSESHAPE_OFF = 0x00,    // Table 11-1 (SetFskModulationParams)
    LR20XX_GFSK_PULSESHAPE_BT_03 = 0x04,  // Gaussian BT 0.3
    LR20XX_GFSK_PULSESHAPE_BT_05 = 0x05,  // Gaussian BT 0.5
    LR20XX_GFSK_PULSESHAPE_BT_07 = 0x06,  // Gaussian BT 0.7
    LR20XX_GFSK_PULSESHAPE_BT_1 = 0x07,   // Gaussian BT 1.0
} LR20XX_GFSK_PULSESHAPE_ENUM;

typedef enum {
    LR20XX_GFSK_BW_4800 = 0x27, // 39
    LR20XX_GFSK_BW_5800 = 0xD7, // 215
    LR20XX_GFSK_BW_7300 = 0x57, // 87
    LR20XX_GFSK_BW_9700 = 0x26, // 38
    LR20XX_GFSK_BW_11700 = 0x5E, // 94
    LR20XX_GFSK_BW_14600 = 0x56, // 86
    LR20XX_GFSK_BW_19500 = 0x25, // 37
    LR20XX_GFSK_BW_23400 = 0xD5, // 213
    LR20XX_GFSK_BW_29300 = 0x55, // 85
    LR20XX_GFSK_BW_39000 = 0x24, // 36
    LR20XX_GFSK_BW_46900 = 0xD4, // 212
    LR20XX_GFSK_BW_58600 = 0x54, // 84
    LR20XX_GFSK_BW_78200 = 0x23, // 35
    LR20XX_GFSK_BW_93800 = 0xD3, // 211
    LR20XX_GFSK_BW_117300 = 0x53, // 83
    LR20XX_GFSK_BW_156200 = 0x22, // 34
    LR20XX_GFSK_BW_187200 = 0xD2, // 210
    LR20XX_GFSK_BW_234300 = 0x52, // 82
    LR20XX_GFSK_BW_312000 = 0x21, // 33
    LR20XX_GFSK_BW_373600 = 0xD1, // 209
    LR20XX_GFSK_BW_467000 = 0x51, // 81
} LR20XX_GFSK_BANDWIDTH_ENUM;

// cmd 0x0210 SetPacketParamsGFSK(uint16_t PreambleLength, uint8_t
// PreambleDetectorLength, uint8_t SyncWordLength, uint8_t AddrComp,
//            uint8_t PacketType, uint8_t PayloadLength, uint8_t CRCType,
//            uint8_t Whitening);
typedef enum {
  LR20XX_GFSK_PREAMBLE_DETECTOR_OFF = 0x00,          // Table 11-4
  LR20XX_GFSK_PREAMBLE_DETECTOR_LENGTH_8BITS = 0x08,
  LR20XX_GFSK_PREAMBLE_DETECTOR_LENGTH_16BITS = 0x10,
  LR20XX_GFSK_PREAMBLE_DETECTOR_LENGTH_24BITS = 0x18,
  LR20XX_GFSK_PREAMBLE_DETECTOR_LENGTH_32BITS = 0x20,
} LR20XX_GFSK_PREAMBLE_DETECTOR_LENGTH_ENUM;

typedef enum {
  LR20XX_GFSK_ADDRESS_FILTERING_DISABLE = 0x00,
  LR20XX_GFSK_ADDRESS_FILTERING_NODE_ADDRESS = 0x01,
  LR20XX_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES = 0x02,
} LR20XX_GFSK_ADDRESS_FILTERING_ENUM;

typedef enum {
  LR20XX_GFSK_PKT_FIX_LEN = 0x00,
  LR20XX_GFSK_PKT_VAR_LEN = 0x01,
  LR20XX_GFSK_PKT_VAR_LEN_SX128X_COMPAT = 0x02,
} LR20XX_GFSK_PKT_LEN_ENUM;

typedef enum {
  LR20XX_GFSK_CRC_OFF = 0x01,
  LR20XX_GFSK_CRC_1_BYTE = 0x00,
  LR20XX_GFSK_CRC_2_BYTES = 0x02,
  LR20XX_GFSK_CRC_1_BYTE_INV = 0x04,
  LR20XX_GFSK_CRC_2_BYTES_INV = 0x06,
} LR20XX_GFSK_CRC_TYPES_ENUM;

typedef enum {
  LR20XX_GFSK_WHITENING_OFF = 0x00,
  LR20XX_GFSK_WHITENING_ENABLE = 0x01,
  LR20XX_GFSK_WHITENING_ENABLE_SX128X_COMPAT = 0x03,
} LR20XX_GFSK_WHITENING_TYPES_ENUM;

//-------------------------------------------------------
// Enum Definitions FLRC
//-------------------------------------------------------

// cmd 0x0248 SetModulationParamsFLRC(uint8_t BitrateBw, uint8_t CodingRate,
// uint8_t PulseShape)
typedef enum {
  LR20XX_FLRC_BR_2600_BW_2666 = 0x00, // Table 18-4, datasheet
  LR20XX_FLRC_BR_2080_BW_2222 = 0x01,
  LR20XX_FLRC_BR_1300_BW_1333 = 0x02,
  LR20XX_FLRC_BR_1040_BW_1333 = 0x03,
  LR20XX_FLRC_BR_650_BW_888 = 0x04,
  LR20XX_FLRC_BR_520_BW_769 = 0x05,
  LR20XX_FLRC_BR_325_BW_444 = 0x06,
  LR20XX_FLRC_BR_260_BW_444 = 0x07,
} LR20XX_FLRC_BITRATE_BW_ENUM;

typedef enum {
  LR20XX_FLRC_CR_1_2 = 0x00, // Coding rate = 1/2
  LR20XX_FLRC_CR_3_4 = 0x01, // Coding rate = 3/4
  LR20XX_FLRC_CR_1_0 = 0x02, // Coding rate = 1 (no FEC)
  LR20XX_FLRC_CR_2_3 = 0x03, // Coding rate = 2/3
} LR20XX_FLRC_CR_ENUM;

typedef enum {
  LR20XX_FLRC_PULSESHAPE_OFF = 0x00,    // No pulse shaping
  LR20XX_FLRC_PULSESHAPE_BT_03 = 0x04,  // Gaussian BT 0.3
  LR20XX_FLRC_PULSESHAPE_BT_05 = 0x05,  // Gaussian BT 0.5
  LR20XX_FLRC_PULSESHAPE_BT_07 = 0x06,  // Gaussian BT 0.7
  LR20XX_FLRC_PULSESHAPE_BT_1 = 0x07,   // Gaussian BT 1.0
} LR20XX_FLRC_PULSESHAPE_ENUM;

// cmd 0x0249 SetPacketParamsFLRC(uint8_t AgcPblLen, uint8_t SyncWordLength,
// uint8_t SyncWordTx, uint8_t SyncWordMatch, uint8_t PacketType,
// uint8_t CrcLength, uint16_t PayloadLength)
typedef enum {
  LR20XX_FLRC_PREAMBLE_LENGTH_4_BITS = 0x00,
  LR20XX_FLRC_PREAMBLE_LENGTH_8_BITS = 0x01,
  LR20XX_FLRC_PREAMBLE_LENGTH_12_BITS = 0x02,
  LR20XX_FLRC_PREAMBLE_LENGTH_16_BITS = 0x03,
  LR20XX_FLRC_PREAMBLE_LENGTH_20_BITS = 0x04,
  LR20XX_FLRC_PREAMBLE_LENGTH_24_BITS = 0x05,
  LR20XX_FLRC_PREAMBLE_LENGTH_28_BITS = 0x06,
  LR20XX_FLRC_PREAMBLE_LENGTH_32_BITS = 0x07,
} LR20XX_FLRC_PREAMBLE_LENGTH_ENUM;

typedef enum {
  LR20XX_FLRC_SYNCWORD_LEN_NONE = 0x00, // No syncword (0 bits)
  LR20XX_FLRC_SYNCWORD_LEN_16 = 0x01,   // 16 bits (2 bytes)
  LR20XX_FLRC_SYNCWORD_LEN_32 = 0x02,   // 32 bits (4 bytes)
} LR20XX_FLRC_SYNCWORD_LENGTH_ENUM;

typedef enum {
  LR20XX_FLRC_SYNCWORD_TX_NONE = 0x00, // No syncword in Tx
  LR20XX_FLRC_SYNCWORD_TX_1 = 0x01,    // Use syncword 1
  LR20XX_FLRC_SYNCWORD_TX_2 = 0x02,    // Use syncword 2
  LR20XX_FLRC_SYNCWORD_TX_3 = 0x03,    // Use syncword 3
} LR20XX_FLRC_SYNCWORD_TX_ENUM;

typedef enum {
  LR20XX_FLRC_SYNCWORD_MATCH_OFF = 0x00,       // Detection disabled
  LR20XX_FLRC_SYNCWORD_MATCH_1 = 0x01,         // Match syncword 1
  LR20XX_FLRC_SYNCWORD_MATCH_2 = 0x02,         // Match syncword 2
  LR20XX_FLRC_SYNCWORD_MATCH_1_OR_2 = 0x03,    // Match 1 or 2
  LR20XX_FLRC_SYNCWORD_MATCH_3 = 0x04,         // Match syncword 3
  LR20XX_FLRC_SYNCWORD_MATCH_1_OR_3 = 0x05,    // Match 1 or 3
  LR20XX_FLRC_SYNCWORD_MATCH_2_OR_3 = 0x06,    // Match 2 or 3
  LR20XX_FLRC_SYNCWORD_MATCH_1_2_OR_3 = 0x07,  // Match 1, 2, or 3
} LR20XX_FLRC_SYNCWORD_MATCH_ENUM;

typedef enum {
  LR20XX_FLRC_PKT_VAR_LEN = 0x00, // Variable/Dynamic payload length
  LR20XX_FLRC_PKT_FIX_LEN = 0x01, // Fixed payload length
} LR20XX_FLRC_PKT_LEN_ENUM;

typedef enum {
  LR20XX_FLRC_CRC_OFF = 0x00,        // CRC disabled
  LR20XX_FLRC_CRC_2_BYTES = 0x01,    // 16-bit CRC
  LR20XX_FLRC_CRC_3_BYTES = 0x02,    // 24-bit CRC
  LR20XX_FLRC_CRC_4_BYTES = 0x03,    // 32-bit CRC
} LR20XX_FLRC_CRC_LENGTH_ENUM;

#endif // LR20XX_LIB_H
