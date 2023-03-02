//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Setup
//*******************************************************
#ifndef SETUP_H
#define SETUP_H
#pragma once


#include "setup_types.h"
#include "hal\hal.h"


tSetupMetaData SetupMetaData;
tSetup Setup;
tGlobalConfig Config;


//-------------------------------------------------------
// Configure Setup MetaData
//-------------------------------------------------------

void setup_configure_metadata(void)
{
    SetupMetaData = {0};

    //-- FrequencyBand: "2.4,915 FCC,868,433,70"
#ifdef FREQUENCY_BAND_2P4_GHZ
    SetupMetaData.FrequencyBand_allowed_mask = 0b00001; // only 2.4 GHz, not editable
#elif defined FREQUENCY_BAND_915_MHZ_FCC && defined FREQUENCY_BAND_868_MHZ && \
      !defined FREQUENCY_BAND_433_MHZ && !defined FREQUENCY_BAND_70_CM_HAM
    SetupMetaData.FrequencyBand_allowed_mask = 0b00110; // only 915 FCC, 868
#elif !defined FREQUENCY_BAND_915_MHZ_FCC && !defined FREQUENCY_BAND_868_MHZ && \
      defined FREQUENCY_BAND_433_MHZ && defined FREQUENCY_BAND_70_CM_HAM
    SetupMetaData.FrequencyBand_allowed_mask = 0b11000; // only 433 MHz, 70 cm HAM
#elif defined FREQUENCY_BAND_915_MHZ_FCC && defined FREQUENCY_BAND_868_MHZ && \
      defined FREQUENCY_BAND_433_MHZ && defined FREQUENCY_BAND_70_CM_HAM
      SetupMetaData.FrequencyBand_allowed_mask = 0b11110; // only 915 FCC, 868, 433 MHz, 70 cm HAM
#elif defined FREQUENCY_BAND_915_MHZ_FCC
    SetupMetaData.FrequencyBand_allowed_mask = 0b00010; // only 915 MHz FCC, not editable
#elif defined FREQUENCY_BAND_868_MHZ
    SetupMetaData.FrequencyBand_allowed_mask = 0b00100; // only 868 MHz, not editable
#elif defined FREQUENCY_BAND_433_MHZ
    SetupMetaData.FrequencyBand_allowed_mask = 0b01000; // only 433 MHz, not editable
#elif defined FREQUENCY_BAND_70_CM_HAM
    SetupMetaData.FrequencyBand_allowed_mask = 0b10000; // only 70 cm HAM, not editable
#endif

    //-- Mode: "50 Hz,31 Hz,19 Hz"
#ifdef DEVICE_HAS_SX128x
    SetupMetaData.Mode_allowed_mask = UINT16_MAX; // all
#elif defined DEVICE_HAS_SX126x
    SetupMetaData.Mode_allowed_mask = 0b0110; // only 31 Hz, 19 Hz
#elif defined DEVICE_HAS_SX127x
    SetupMetaData.Mode_allowed_mask = 0b0100; // only 19 Hz, not editable
#endif

    //-- Tx:

    power_optstr_from_rfpower_list(SetupMetaData.Tx_Power_optstr, rfpower_list, RFPOWER_LIST_NUM, 44);

    // Diversity: "on,antenna1,antenna2"
#ifdef DEVICE_HAS_DIVERSITY
    SetupMetaData.Tx_Diversity_allowed_mask = UINT16_MAX; // all
#else
    SetupMetaData.Tx_Diversity_allowed_mask = 0b0010; // only antenna1, not editable
#endif

    // Tx ChannelSource: "none,mbridge,in,crsf"
#if defined DEVICE_HAS_JRPIN5 && defined USE_IN
    SetupMetaData.Tx_ChannelsSource_allowed_mask = UINT16_MAX; // all
#elif defined DEVICE_HAS_JRPIN5
    SetupMetaData.Tx_ChannelsSource_allowed_mask = 0b1011; // only none, mbridge, crsf
#elif defined USE_IN
    SetupMetaData.Tx_ChannelsSource_allowed_mask = 0b0101; // only none, in
#else
    SetupMetaData.Tx_ChannelsSource_allowed_mask = 0b0001; // only none, not editable
#endif

    // Tx InMode: "sbus,sbus inv"
#ifdef DEVICE_HAS_IN
    SetupMetaData.Tx_InMode_allowed_mask = UINT16_MAX; // all
#elif defined DEVICE_HAS_IN_NORMAL
    SetupMetaData.Tx_InMode_allowed_mask = 0b0010; // only sbus inv, not editable
#elif defined DEVICE_HAS_IN_INVERTED
    SetupMetaData.Tx_InMode_allowed_mask = 0b0001; // only sbus, not editable
#else
    SetupMetaData.Tx_InMode_allowed_mask = 0; // not available, do not display
#endif

    // Tx SerialDestination: "serial,mbridge,serial2"
#if defined DEVICE_HAS_JRPIN5 && defined USE_SERIAL2
    SetupMetaData.Tx_SerialDestination_allowed_mask = UINT16_MAX; // all
#elif defined DEVICE_HAS_JRPIN5
    SetupMetaData.Tx_SerialDestination_allowed_mask = 0b0011; // only serial, mbridge
#elif defined USE_SERIAL2
    SetupMetaData.Tx_SerialDestination_allowed_mask = 0b0101; // only serial, serial2
#else
    SetupMetaData.Tx_SerialDestination_allowed_mask = 0b0001; // only serial, not editable
#endif

    // Tx Buzzer: ""off,LP,rxLQ"
#ifdef DEVICE_HAS_BUZZER
    SetupMetaData.Tx_Buzzer_allowed_mask = UINT16_MAX; // all
#else
    SetupMetaData.Tx_Buzzer_allowed_mask = 0; // not available, do not display
#endif

    //-- Rx:

    power_optstr_from_rfpower_list(SetupMetaData.Rx_Power_optstr, rfpower_list, RFPOWER_LIST_NUM, 44);

    // Rx Diversity: "on,antenna1,antenna2"
#ifdef DEVICE_HAS_DIVERSITY
    SetupMetaData.Rx_Diversity_allowed_mask = UINT16_MAX; // all
#else
    SetupMetaData.Rx_Diversity_allowed_mask = 0b0010; // only antenna1, not editable
#endif

    // Rx OutMode: "sbus,crsf,sbus inv"
#ifdef DEVICE_HAS_OUT
    SetupMetaData.Rx_OutMode_allowed_mask = UINT16_MAX; // all
#elif defined DEVICE_HAS_OUT_NORMAL
    SetupMetaData.Rx_OutMode_allowed_mask = 0b0110; // crsf,sbus inv
#elif defined DEVICE_HAS_OUT_INVERTED
    SetupMetaData.Rx_OutMode_allowed_mask = 0b001; // sbus, not editable
#else
    SetupMetaData.Rx_OutMode_allowed_mask = 0;  // not available, do not display
#endif

    // Rx Buzzer: ""off,LP"
#ifdef DEVICE_HAS_BUZZER
    SetupMetaData.Rx_Buzzer_allowed_mask = UINT16_MAX; // all
#else
    SetupMetaData.Rx_Buzzer_allowed_mask = 0; // not available, do not display
#endif

    //-- Tx: Receiver setup meta data

    SetupMetaData.rx_available = false;

    SetupMetaData.rx_firmware_version = 0;
    SetupMetaData.rx_setup_layout = 0;
    strcpy(SetupMetaData.rx_device_name, "");
    SetupMetaData.rx_actual_power_dbm = INT8_MAX;
    SetupMetaData.rx_actual_diversity = 3;
}


//-------------------------------------------------------
// Setup
//-------------------------------------------------------

void setup_default(void)
{
    strcpy(Setup.BindPhrase, BIND_PHRASE);
    Setup.FrequencyBand = SETUP_RF_BAND;
    Setup.Mode = SETUP_MODE;

    Setup.Tx.Power = SETUP_TX_POWER;
    Setup.Tx.Diversity = SETUP_TX_DIVERSITY;
    Setup.Tx.SerialDestination = SETUP_TX_SERIAL_DESTINATION;
    Setup.Tx.ChannelsSource = SETUP_TX_CHANNELS_SOURCE;
    Setup.Tx.ChannelOrder = SETUP_TX_CHANNEL_ORDER;
    Setup.Tx.InMode = SETUP_TX_IN_MODE;
    Setup.Tx.SerialBaudrate = SETUP_TX_SERIAL_BAUDRATE;
    Setup.Tx.SerialLinkMode = SETUP_TX_SERIAL_LINK_MODE;
    Setup.Tx.SendRadioStatus = SETUP_TX_SEND_RADIO_STATUS;
    Setup.Tx.Buzzer = SETUP_TX_BUZZER;
    Setup.Tx.CliLineEnd = SETUP_TX_CLI_LINE_END;

    Setup.Rx.Power = SETUP_RX_POWER;
    Setup.Rx.Diversity = SETUP_RX_DIVERSITY;
    Setup.Rx.ChannelOrder = SETUP_RX_CHANNEL_ORDER;
    Setup.Rx.OutMode = SETUP_RX_OUT_MODE;
    Setup.Rx.OutRssiChannelMode = SETUP_RX_OUT_RSSI_CHANNEL;
    Setup.Rx.FailsafeMode = SETUP_RX_FAILSAFE_MODE;
    Setup.Rx.SerialBaudrate = SETUP_RX_SERIAL_BAUDRATE;
    Setup.Rx.SerialLinkMode = SETUP_RX_SERIAL_LINK_MODE;
    Setup.Rx.SendRadioStatus = SETUP_RX_SEND_RADIO_STATUS;
    Setup.Rx.Buzzer = SETUP_RX_BUZZER;
    Setup.Rx.SendRcChannels = SETUP_RX_SEND_RC_CHANNELS;

    for (uint8_t ch = 0; ch < 12; ch++) { Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[ch] = 0; }
    for (uint8_t ch = 0; ch < 4; ch++) { Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[ch] = 1; }

    Setup.Rx.__RadioStatusMethod = 0xFF; // deprecated
}


void setup_sanitize(void)
{
// note: if allowed_mask = 0, this also triggers
// we may have to distinguish between not editable and not displayed, but currently
// not displayed applies only to settings for which the default is ok
#define SETUP_TST_NOTALLOWED(amask,pfield) ((SetupMetaData.amask & (1 << Setup.pfield)) == 0)

    //-- BindPhrase, FrequencyBand, Mode
    sanitize_bindphrase(Setup.BindPhrase);

#ifdef FREQUENCY_BAND_2P4_GHZ
    uint8_t frequency_band_default = SETUP_FREQUENCY_BAND_2P4_GHZ;
#elif defined FREQUENCY_BAND_915_MHZ_FCC && defined FREQUENCY_BAND_868_MHZ
    uint8_t frequency_band_default = SETUP_FREQUENCY_BAND_868_MHZ; // my privilege to be in the EU :)
#elif defined FREQUENCY_BAND_915_MHZ_FCC
    uint8_t frequency_band_default = SETUP_FREQUENCY_BAND_915_MHZ_FCC;
#elif defined FREQUENCY_BAND_868_MHZ
    uint8_t frequency_band_default = SETUP_FREQUENCY_BAND_868_MHZ;
#elif defined FREQUENCY_BAND_433_MHZ
    uint8_t frequency_band_default = SETUP_FREQUENCY_BAND_433_MHZ;
#elif defined FREQUENCY_BAND_70_CM_HAM
    uint8_t frequency_band_default = SETUP_FREQUENCY_BAND_70_CM_HAM;
#else
    #error Unknown Frequencyband !
#endif
    if (Setup.FrequencyBand >= SETUP_FREQUENCY_BAND_NUM) Setup.FrequencyBand = frequency_band_default;
    if (SETUP_TST_NOTALLOWED(FrequencyBand_allowed_mask,FrequencyBand)) Setup.FrequencyBand = frequency_band_default;

    if (Setup.Mode >= MODE_NUM) Setup.Mode = MODE_19HZ;
    if (SETUP_TST_NOTALLOWED(Mode_allowed_mask,Mode)) Setup.Mode = MODE_19HZ;

    //-- Tx:

    if (Setup.Tx.Power >= RFPOWER_LIST_NUM) Setup.Tx.Power = RFPOWER_LIST_NUM - 1;

    if (Setup.Tx.Diversity >= DIVERSITY_NUM) Setup.Tx.Diversity = DIVERSITY_DEFAULT;
    if (SETUP_TST_NOTALLOWED(Tx_Diversity_allowed_mask,Tx.Diversity)) Setup.Tx.Diversity = DIVERSITY_ANTENNA1;

    if (Setup.Tx.ChannelsSource >= CHANNEL_SOURCE_NUM) Setup.Tx.ChannelsSource = CHANNEL_SOURCE_NONE;
    if (SETUP_TST_NOTALLOWED(Tx_ChannelsSource_allowed_mask,Tx.ChannelsSource)) Setup.Tx.ChannelsSource = CHANNEL_SOURCE_NONE;

    if (Setup.Tx.InMode >= IN_CONFIG_NUM) Setup.Tx.InMode = IN_CONFIG_SBUS;
    if (SETUP_TST_NOTALLOWED(Tx_InMode_allowed_mask,Tx.InMode)) Setup.Tx.InMode = IN_CONFIG_SBUS;

    if (Setup.Tx.SerialDestination >= SERIAL_DESTINATION_NUM) Setup.Tx.SerialDestination = SERIAL_DESTINATION_SERIAL;
    if (SETUP_TST_NOTALLOWED(Tx_SerialDestination_allowed_mask,Tx.SerialDestination)) Setup.Tx.SerialDestination = SERIAL_DESTINATION_SERIAL;

    if (Setup.Tx.ChannelOrder >= CHANNEL_ORDER_NUM) Setup.Tx.ChannelOrder = CHANNEL_ORDER_AETR;
    if (Setup.Tx.SerialBaudrate >= SERIAL_BAUDRATE_NUM) Setup.Tx.SerialBaudrate = SERIAL_BAUDRATE_57600;
    if (Setup.Tx.SerialLinkMode >= SERIAL_LINK_MODE_NUM) Setup.Tx.SerialLinkMode = SERIAL_LINK_MODE_TRANSPARENT;
    if (Setup.Tx.SendRadioStatus >= TX_SEND_RADIO_STATUS_NUM) Setup.Tx.SendRadioStatus = TX_SEND_RADIO_STATUS_OFF;

    if (Setup.Tx.Buzzer >= BUZZER_NUM) Setup.Tx.Buzzer = BUZZER_OFF;
    if (SETUP_TST_NOTALLOWED(Tx_Buzzer_allowed_mask,Tx.Buzzer)) Setup.Tx.Buzzer = BUZZER_OFF;

    if (Setup.Tx.CliLineEnd >= CLI_LINE_END_NUM) Setup.Tx.CliLineEnd = CLI_LINE_END_CR;

    // device cannot use mBridge (pin5) and CRSF (pin5) at the same time !
    if ((Setup.Tx.SerialDestination == SERIAL_DESTINATION_MBRDIGE) && (Setup.Tx.ChannelsSource == CHANNEL_SOURCE_CRSF)) {
        Setup.Tx.ChannelsSource = CHANNEL_SOURCE_NONE;
    }

    //-- Rx:

    if (Setup.Rx.Power >= RFPOWER_LIST_NUM) Setup.Rx.Power = RFPOWER_LIST_NUM - 1;

    if (Setup.Rx.Diversity >= DIVERSITY_NUM) Setup.Rx.Diversity = DIVERSITY_DEFAULT;
    if (SETUP_TST_NOTALLOWED(Rx_Diversity_allowed_mask,Rx.Diversity)) Setup.Rx.Diversity = DIVERSITY_ANTENNA1;

    if (Setup.Rx.ChannelOrder >= CHANNEL_ORDER_NUM) Setup.Rx.ChannelOrder = CHANNEL_ORDER_AETR;

    if (Setup.Rx.OutMode >= OUT_CONFIG_NUM) Setup.Rx.OutMode = OUT_CONFIG_SBUS;
    if (SETUP_TST_NOTALLOWED(Rx_OutMode_allowed_mask,Rx.OutMode)) Setup.Rx.OutMode = OUT_CONFIG_SBUS;

    if (Setup.Rx.OutRssiChannelMode >= OUT_RSSI_CHANNEL_NUM) Setup.Rx.OutRssiChannelMode = 0;
    if (Setup.Rx.FailsafeMode >= FAILSAFE_MODE_NUM) Setup.Rx.FailsafeMode = FAILSAFE_MODE_NO_SIGNAL;

    if (Setup.Rx.Buzzer > BUZZER_LOST_PACKETS) Setup.Rx.Buzzer = BUZZER_OFF;
    if (SETUP_TST_NOTALLOWED(Rx_Buzzer_allowed_mask,Rx.Buzzer)) Setup.Rx.Buzzer = BUZZER_OFF;

    for (uint8_t ch = 0; ch < 12; ch++) {
        if (Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[ch] < -120) Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[ch] = 0;
        if (Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[ch] > 120) Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[ch] = 0;
    }
    for (uint8_t ch = 0; ch < 4; ch++) {
        if (Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[ch] > 2) Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[ch] = 1;
    }

    if (Setup.Rx.SerialBaudrate >= SERIAL_BAUDRATE_NUM) Setup.Rx.SerialBaudrate = SERIAL_BAUDRATE_57600;
    if (Setup.Rx.SerialLinkMode >= SERIAL_LINK_MODE_NUM) Setup.Rx.SerialLinkMode = SERIAL_LINK_MODE_TRANSPARENT;
    if (Setup.Rx.SendRadioStatus >= RX_SEND_RADIO_STATUS_NUM) Setup.Rx.SendRadioStatus = RX_SEND_RADIO_STATUS_OFF;
    // deprecated if (Setup.Rx.RadioStatusMethod >= RADIO_STATUS_METHOD_NUM) Setup.Rx.RadioStatusMethod = RADIO_STATUS_METHOD_W_TXBUF;
    //Setup.Rx.RadioStatusMethod = RADIO_STATUS_METHOD_W_TXBUF; // for the moment we fix it to this setting
    if (Setup.Rx.SendRcChannels >= SEND_RC_CHANNELS_NUM) Setup.Rx.SendRcChannels = SEND_RC_CHANNELS_OFF;
}


//-------------------------------------------------------
// Configure
//-------------------------------------------------------

void configure_mode(uint8_t mode)
{
    Config.Mode = mode;

    switch (Config.Mode) {
    case MODE_50HZ:
        Config.frame_rate_ms = 20; // 20 ms = 50 Hz
        Config.frame_rate_hz = 50;
        Config.LoraConfigIndex = SX128x_LORA_CONFIG_BW800_SF5_CRLI4_5;
        Config.lora_send_frame_tmo = MODE_50HZ_SEND_FRAME_TMO; // 10;
        break;
    case MODE_31HZ:
        Config.frame_rate_ms = 32; // 32 ms = 31.25 Hz
        Config.frame_rate_hz = 31;
#ifdef DEVICE_HAS_SX128x
        Config.LoraConfigIndex = SX128x_LORA_CONFIG_BW800_SF6_CRLI4_5;
#else
        Config.LoraConfigIndex = SX126x_LORA_CONFIG_BW500_SF5_CR4_5;
#endif
        Config.lora_send_frame_tmo = MODE_31HZ_SEND_FRAME_TMO; // 15
        break;
  case MODE_19HZ:
        Config.frame_rate_ms = 53; // 53 ms = 18.9 Hz
        Config.frame_rate_hz = 19;
#ifdef DEVICE_HAS_SX128x
        Config.LoraConfigIndex = SX128x_LORA_CONFIG_BW800_SF7_CRLI4_5;
#elif defined DEVICE_HAS_SX126x
        Config.LoraConfigIndex = SX126x_LORA_CONFIG_BW500_SF6_CR4_5;
#else
        Config.LoraConfigIndex = SX127x_LORA_CONFIG_BW500_SF6_CR4_5;
#endif
        Config.lora_send_frame_tmo = MODE_19HZ_SEND_FRAME_TMO; // 25;
        break;
    default:
        while (1) {} // must not happen, should have been resolved in setup_sanitize()

    }
}


void setup_configure(void)
{
    //-- SyncWord

    uint32_t bind_dblword = u32_from_bindphrase(Setup.BindPhrase);

    Config.FrameSyncWord = fmav_crc_calculate((uint8_t*)&bind_dblword, 4); // condense it into a u16

    //-- Power

    // note: the actually used power will be determined later when the SX are set up
#ifdef DEVICE_IS_TRANSMITTER
    Config.Power_dbm = rfpower_list[Setup.Tx.Power].dbm;
#endif
#ifdef DEVICE_IS_RECEIVER
    Config.Power_dbm = rfpower_list[Setup.Rx.Power].dbm;
#endif

  //-- Diversity

#ifdef DEVICE_HAS_DIVERSITY
#ifdef DEVICE_IS_TRANSMITTER
    switch (Setup.Tx.Diversity) {
#endif
#ifdef DEVICE_IS_RECEIVER
    switch (Setup.Rx.Diversity) {
#endif
    case DIVERSITY_DEFAULT:
        Config.UseAntenna1 = true;
        Config.UseAntenna2 = true;
        break;
    case DIVERSITY_ANTENNA1:
        Config.UseAntenna1 = true;
        Config.UseAntenna2 = false;
        break;
    case DIVERSITY_ANTENNA2:
        Config.UseAntenna1 = false;
        Config.UseAntenna2 = true;
        break;
    default:
        while (1) {} // must not happen, should have been resolved in setup_sanitize()
    }
#else
    Config.UseAntenna1 = true;
    Config.UseAntenna2 = false;
#endif

    //-- Mode, Mode dependent settings, LoRa configuration

    configure_mode(Setup.Mode);

    //-- Fhss

    //Config.FhssSeed = bind_dblword;
    // this is much better for narrow bands, like 868 MHz
    // we could make it dependable with a #ifdef, but what's the point
    Config.FhssSeed = Config.FrameSyncWord;

    switch (Setup.FrequencyBand) {
    case SETUP_FREQUENCY_BAND_2P4_GHZ:
        switch (Config.Mode) {
        case MODE_50HZ: Config.FhssNum = FHSS_NUM_BAND_2P4_GHZ; break;
        case MODE_31HZ: Config.FhssNum = FHSS_NUM_BAND_2P4_GHZ_31HZ_MODE; break;
        case MODE_19HZ: Config.FhssNum = FHSS_NUM_BAND_2P4_GHZ_19HZ_MODE; break;
        default:
            while (1) {} // must not happen, should have been resolved in setup_sanitize()
        }
        break;
    case SETUP_FREQUENCY_BAND_915_MHZ_FCC:
        switch (Config.Mode) {
        case MODE_31HZ: Config.FhssNum = FHSS_NUM_BAND_915_MHZ_FCC; break;
        case MODE_19HZ: Config.FhssNum = FHSS_NUM_BAND_915_MHZ_FCC_19HZ_MODE; break;
        default:
            while (1) {} // must not happen, should have been resolved in setup_sanitize()
        }
        break;
    case SETUP_FREQUENCY_BAND_868_MHZ:
        Config.FhssNum = FHSS_NUM_BAND_868_MHZ;
        break;
    case SETUP_FREQUENCY_BAND_433_MHZ:
        Config.FhssNum = FHSS_NUM_BAND_433_MHZ;
        break;
    case SETUP_FREQUENCY_BAND_70_CM_HAM:
        switch (Config.Mode) {
        case MODE_31HZ: Config.FhssNum = FHSS_NUM_BAND_70_CM_HAM; break;
        case MODE_19HZ: Config.FhssNum = FHSS_NUM_BAND_70_CM_HAM_19HZ_MODE; break;
        default:
            while (1) {} // must not happen, should have been resolved in setup_sanitize()
        }
        break;
    default:
        while (1) {} // must not happen, should have been resolved in setup_sanitize()
    }

    //-- More Config, may depend on above config settings

    Config.connect_tmo_systicks = SYSTICK_DELAY_MS(CONNECT_TMO_MS);
    Config.connect_listen_hop_cnt = (uint8_t)(1.5f * Config.FhssNum);

    Config.LQAveragingPeriod = (LQ_AVERAGING_MS/Config.frame_rate_ms);

    //-- Serial
#ifdef DEVICE_IS_TRANSMITTER
    switch (Setup.Tx.SerialBaudrate) {
#endif
#ifdef DEVICE_IS_RECEIVER
    switch (Setup.Rx.SerialBaudrate) {
#endif
    case SERIAL_BAUDRATE_9600: Config.SerialBaudrate = 9600; break;
    case SERIAL_BAUDRATE_19200: Config.SerialBaudrate = 19200; break;
    case SERIAL_BAUDRATE_38400: Config.SerialBaudrate = 38400; break;
    case SERIAL_BAUDRATE_57600: Config.SerialBaudrate = 57600; break;
    case SERIAL_BAUDRATE_115200: Config.SerialBaudrate = 115200; break;
    case SERIAL_BAUDRATE_230400: Config.SerialBaudrate = 230400; break;
    default:
        Config.SerialBaudrate = 57600;
    }

    //-- Mbridge, Crsf

    Config.UseMbridge = false;
    Config.UseCrsf = false;
#if defined DEVICE_IS_TRANSMITTER && defined DEVICE_HAS_JRPIN5
    if ((Setup.Tx.SerialDestination == SERIAL_DESTINATION_MBRDIGE) || (Setup.Tx.ChannelsSource == CHANNEL_SOURCE_MBRIDGE)) {
        Config.UseMbridge = true;
    }
    if ((Setup.Tx.SerialDestination != SERIAL_DESTINATION_MBRDIGE) && (Setup.Tx.ChannelsSource == CHANNEL_SOURCE_CRSF)) {
        Config.UseCrsf = true;
    }
    if (Config.UseMbridge && Config.UseCrsf) {
        while (1) {} // mBridge and CRSF cannot be used simultaneously, must not happen, should have been resolved in setup_sanitize()
    }
#endif
}


//-------------------------------------------------------
// helper
//-------------------------------------------------------

void setup_clear(void)
{
    memset(&Setup, 0xff, sizeof(tSetup));
}


EE_STATUS_ENUM setup_store_to_EEPROM(void)
{
    return ee_writedata(&Setup, sizeof(tSetup));
}


EE_STATUS_ENUM setup_retrieve_from_EEPROM(void)
{
    return ee_readdata(&Setup, sizeof(tSetup));
}


void setup_reload(void)
{
    setup_retrieve_from_EEPROM();
    setup_sanitize();
}


//-------------------------------------------------------
// Init
//-------------------------------------------------------

void setup_init(void)
{
EE_STATUS_ENUM ee_status;
bool doEEPROMwrite;

    setup_clear();
    ee_status = ee_init();
    if (ee_status == EE_STATUS_OK) { ee_status = setup_retrieve_from_EEPROM(); }
    if (ee_status != EE_STATUS_OK) { // try it a 2nd time
        setup_clear();
        ee_status = ee_init();
        if (ee_status == EE_STATUS_OK) { ee_status = setup_retrieve_from_EEPROM(); }
    }
    if (ee_status != EE_STATUS_OK) setup_clear();

    doEEPROMwrite = false;
    if (Setup.Layout != SETUPLAYOUT) {
        setup_default();
        Setup.Layout = SETUPLAYOUT;
        doEEPROMwrite = true;
    }
    if (Setup.Version != VERSION) { // do after Layout, ensures that these flags are correct irrespective of Layout handling
        Setup.Version = VERSION;
        doEEPROMwrite = true;
    }
    if ((strncmp(Setup.MarkerStr, SETUP_MARKER_STR, 16) != 0)) {
        strbufstrcpy((char*)Setup.MarkerStr, SETUP_MARKER_STR, 16);
        strbufstrcpy((char*)Setup.MarkerEnd, SETUP_MARKEREND_STR, 8);
        doEEPROMwrite = true;
    }
    if (doEEPROMwrite) {
       setup_store_to_EEPROM();
    }

    setup_configure_metadata();

#ifdef SETUP_FORCE_COMMON_CONF
setup_default();
#endif

    setup_sanitize();

    setup_configure();
}


#endif // SETUP_H
