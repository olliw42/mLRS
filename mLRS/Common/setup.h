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
#include "hal/device_conf.h"
#include "hal/hal.h"


tSetupMetaData SetupMetaData;
tSetup Setup;
tGlobalConfig Config;


//-------------------------------------------------------
// Configure Setup MetaData
//-------------------------------------------------------

void setup_configure_metadata(void)
{
    SetupMetaData = {};

    //-- FrequencyBand: "2.4,915 FCC,868,433,70,866 IN"
#if defined FREQUENCY_BAND_2P4_GHZ && defined FREQUENCY_BAND_915_MHZ_FCC && defined FREQUENCY_BAND_868_MHZ
    // DUALBAND!
    SetupMetaData.FrequencyBand_allowed_mask = 0b000110; // 915 FCC, 868
#elif defined FREQUENCY_BAND_2P4_GHZ
    SetupMetaData.FrequencyBand_allowed_mask = 0b000001; // only 2.4 GHz, not editable
#elif defined FREQUENCY_BAND_915_MHZ_FCC && defined FREQUENCY_BAND_868_MHZ && \
      defined FREQUENCY_BAND_433_MHZ && defined FREQUENCY_BAND_70_CM_HAM
    SetupMetaData.FrequencyBand_allowed_mask = 0b011110; // 915 FCC, 868, 433, 70
#elif defined FREQUENCY_BAND_915_MHZ_FCC && defined FREQUENCY_BAND_868_MHZ && defined FREQUENCY_BAND_866_MHZ_IN
    SetupMetaData.FrequencyBand_allowed_mask = 0b100110; // 915 FCC, 868, 866 IN
#elif defined FREQUENCY_BAND_915_MHZ_FCC && defined FREQUENCY_BAND_868_MHZ
    SetupMetaData.FrequencyBand_allowed_mask = 0b000110; // 915 FCC, 868
#elif defined FREQUENCY_BAND_433_MHZ && defined FREQUENCY_BAND_70_CM_HAM
    SetupMetaData.FrequencyBand_allowed_mask = 0b011000; // 433, 70
#elif defined FREQUENCY_BAND_915_MHZ_FCC
    SetupMetaData.FrequencyBand_allowed_mask = 0b000010; // only 915 MHz FCC, not editable
#elif defined FREQUENCY_BAND_868_MHZ
    SetupMetaData.FrequencyBand_allowed_mask = 0b000100; // only 868 MHz, not editable
#elif defined FREQUENCY_BAND_433_MHZ
    SetupMetaData.FrequencyBand_allowed_mask = 0b001000; // only 433 MHz, not editable
#elif defined FREQUENCY_BAND_70_CM_HAM
    SetupMetaData.FrequencyBand_allowed_mask = 0b010000; // only 70 cm HAM, not editable
#elif defined FREQUENCY_BAND_866_MHZ_IN
    SetupMetaData.FrequencyBand_allowed_mask = 0b100000; // only 866 MHz IN, not editable
#endif

    //-- Mode: "50 Hz,31 Hz,19 Hz,FLRC"
#ifdef DEVICE_HAS_SX128x
#ifndef MLRS_DEV_FEATURE_FLRC
    SetupMetaData.Mode_allowed_mask = 0b0111; // only 50 Hz, 31 Hz, 19 Hz
#else
    SetupMetaData.Mode_allowed_mask = UINT16_MAX; // all
#endif
#elif defined DEVICE_HAS_SX126x || defined DEVICE_HAS_DUAL_SX126x_SX128x
    SetupMetaData.Mode_allowed_mask = 0b0110; // only 31 Hz, 19 Hz
#elif defined DEVICE_HAS_SX127x
    SetupMetaData.Mode_allowed_mask = 0b0100; // only 19 Hz, not editable
#endif

    //-- Ortho: "off,1/3,2/3,3/3"
    // we cannot work out all cases here, since it also depends on the actual selection, so we just do what we can do
#if defined FREQUENCY_BAND_2P4_GHZ || defined FREQUENCY_BAND_915_MHZ_FCC || defined FREQUENCY_BAND_70_CM_HAM
    SetupMetaData.Ortho_allowed_mask = UINT16_MAX; // all
#else
    SetupMetaData.Ortho_allowed_mask = 0b0001; // only off, not editable
#endif
#ifndef MLRS_DEV_FEATURE_ORTHO
    SetupMetaData.Ortho_allowed_mask = 0; // not available, do not display
#endif

    //-- Tx:

    power_optstr_from_rfpower_list(SetupMetaData.Tx_Power_optstr, rfpower_list, RFPOWER_LIST_NUM, 44);

    // Diversity: "enabled,antenna1,antenna2,r:e t:a1,r:e t:a2"
#ifdef DEVICE_HAS_DIVERSITY
    SetupMetaData.Tx_Diversity_allowed_mask = UINT16_MAX; // all
#elif defined DEVICE_HAS_DUAL_SX126x_SX128x
    SetupMetaData.Tx_Diversity_allowed_mask = 0b00001; // only enabled, not editable
#else
    SetupMetaData.Tx_Diversity_allowed_mask = 0b00010; // only antenna1, not editable
#endif

    // Tx ChannelSource: "none,crsf,in,mbridge"
#if defined DEVICE_HAS_JRPIN5 && defined USE_IN
    SetupMetaData.Tx_ChannelsSource_allowed_mask = UINT16_MAX; // all
#elif defined DEVICE_HAS_JRPIN5
    SetupMetaData.Tx_ChannelsSource_allowed_mask = 0b1011; // only none, crsf, mbridge
#elif defined USE_IN
    SetupMetaData.Tx_ChannelsSource_allowed_mask = 0b0101; // only none, in
#else
    SetupMetaData.Tx_ChannelsSource_allowed_mask = 0b0001; // only none, not editable
#endif

    // Tx InMode: "sbus,sbus inv"
#ifdef DEVICE_HAS_IN
    SetupMetaData.Tx_InMode_allowed_mask = UINT16_MAX; // all
#elif defined DEVICE_HAS_IN_NORMAL
    SetupMetaData.Tx_InMode_allowed_mask = 0b10; // only sbus inv, not editable
#elif defined DEVICE_HAS_IN_INVERTED
    SetupMetaData.Tx_InMode_allowed_mask = 0b01; // only sbus, not editable
#else
    SetupMetaData.Tx_InMode_allowed_mask = 0; // not available, do not display
#endif

    // Tx SerialDestination: "serial,serial2,mbridge"
#if defined DEVICE_HAS_JRPIN5 && defined USE_SERIAL2
    SetupMetaData.Tx_SerialDestination_allowed_mask = UINT16_MAX; // all
#elif defined DEVICE_HAS_JRPIN5
    SetupMetaData.Tx_SerialDestination_allowed_mask = 0b101; // only serial, mbridge
#elif defined USE_SERIAL2
    SetupMetaData.Tx_SerialDestination_allowed_mask = 0b011; // only serial, serial2
#else
    SetupMetaData.Tx_SerialDestination_allowed_mask = 0b001; // only serial, not editable
#endif

    // Tx Buzzer: ""off,LP,rxLQ"
#ifdef DEVICE_HAS_BUZZER
    SetupMetaData.Tx_Buzzer_allowed_mask = UINT16_MAX; // all
#else
    SetupMetaData.Tx_Buzzer_allowed_mask = 0; // not available, do not display
#endif

    //-- Rx:

    power_optstr_from_rfpower_list(SetupMetaData.Rx_Power_optstr, rfpower_list, RFPOWER_LIST_NUM, 44);

    // Rx Diversity: "enabled,antenna1,antenna2,r:e t:a1,r:e t:a2"
#ifdef DEVICE_HAS_DIVERSITY
    SetupMetaData.Rx_Diversity_allowed_mask = UINT16_MAX; // all
#elif defined DEVICE_HAS_DUAL_SX126x_SX128x
    SetupMetaData.Rx_Diversity_allowed_mask = 0b00001; // only enabled, not editable
#else
    SetupMetaData.Rx_Diversity_allowed_mask = 0b00010; // only antenna1, not editable
#endif

    // Rx OutMode: "sbus,crsf,sbus inv"
#ifdef DEVICE_HAS_OUT
    SetupMetaData.Rx_OutMode_allowed_mask = UINT16_MAX; // all
#elif defined DEVICE_HAS_OUT_NORMAL
    SetupMetaData.Rx_OutMode_allowed_mask = 0b110; // crsf,sbus inv
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
    SetupMetaData.rx_actual_diversity = DIVERSITY_NUM;
}


//-------------------------------------------------------
// Setup
//-------------------------------------------------------

void inc_bindphrase_char(char* s, uint8_t pos)
{
    char* cptr = strchr(bindphrase_chars, s[pos]);
    uint8_t n = (cptr) ? cptr - bindphrase_chars + 1 : 0; // must not happen that c is not found, but play it safe
    if (n >= strlen(bindphrase_chars)) n = 0;
    s[pos] = bindphrase_chars[n];
}


void setup_default_bindphrase(char* s, uint8_t config_id, const char* bindphrase_default)
{
    strcpy(s, bindphrase_default);
    if (config_id == 0) return;

#ifdef FREQUENCY_BAND_2P4_GHZ
    switch (config_id) {
        case 1: s[5] = '5'; break;
        case 2: s[5] = 'a'; break;
        case 3: s[5] = 'f'; break;
        case 4: s[5] = 'k'; break;
        case 5: s[5] = 'p'; break;
        case 6: s[5] = 'u'; break;
        case 7: s[5] = 'z'; break;
        case 8: s[5] = '1'; break;
        case 9: s[5] = '2'; break;
    }
    if (s[5] == bindphrase_default[5]) inc_bindphrase_char(s, 5);
#else
    for (uint8_t cnt = 0; cnt < config_id; cnt++) { // inc s[5] config_id many times
        inc_bindphrase_char(s, 5);
    }
#endif
}


void setup_default(uint8_t config_id)
{
    char bind_phrase[6+1];
    setup_default_bindphrase(bind_phrase, config_id, BIND_PHRASE);
    strcpy(Setup.Common[config_id].BindPhrase, bind_phrase);

    Setup.Common[config_id].FrequencyBand = SETUP_RF_BAND;
    Setup.Common[config_id].Mode = SETUP_MODE;
    Setup.Common[config_id].Ortho = SETUP_RF_ORTHO;

    Setup.Tx[config_id].Power = SETUP_TX_POWER;
    Setup.Tx[config_id].Diversity = SETUP_TX_DIVERSITY;
    Setup.Tx[config_id].SerialDestination = SETUP_TX_SERIAL_DESTINATION;
    Setup.Tx[config_id].ChannelsSource = SETUP_TX_CHANNELS_SOURCE;
    Setup.Tx[config_id].ChannelOrder = SETUP_TX_CHANNEL_ORDER;
    Setup.Tx[config_id].InMode = SETUP_TX_IN_MODE;
    Setup.Tx[config_id].SerialBaudrate = SETUP_TX_SERIAL_BAUDRATE;
    Setup.Tx[config_id].SendRadioStatus = SETUP_TX_SEND_RADIO_STATUS;
    Setup.Tx[config_id].Buzzer = SETUP_TX_BUZZER;
    Setup.Tx[config_id].CliLineEnd = SETUP_TX_CLI_LINE_END;

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
}


void setup_sanitize_config(uint8_t config_id)
{
// note: if allowed_mask = 0, this also triggers
// we may have to distinguish between not editable and not displayed, but currently
// not displayed applies only to settings for which the default is ok
#define SETUP_TST_NOTALLOWED(amask,pfield) ((SetupMetaData.amask & (1 << Setup.pfield)) == 0)

#define TST_NOTALLOWED(amask,pfield,val) \
    if ((SetupMetaData.amask & (1 << Setup.pfield)) == 0) { Setup.pfield = val; } \
    if (SetupMetaData.amask != 0 && (SetupMetaData.amask & (1 << Setup.pfield)) == 0) { \
      for (uint8_t i = 0; i < 16; i++) { if (SetupMetaData.amask & (1 << i)) { Setup.pfield = i; break; } } \
    }

#define SANITIZE(pfield,max,setupval,val) \
    if (Setup.pfield >= max) { Setup.pfield = setupval; } \
    if (Setup.pfield >= max) { Setup.pfield = val; }

    //-- BindPhrase, FrequencyBand, Mode, Ortho

    char bind_phrase[6+1];
    setup_default_bindphrase(bind_phrase, config_id, BIND_PHRASE);
    sanitize_bindphrase(Setup.Common[config_id].BindPhrase, bind_phrase);

#if defined FREQUENCY_BAND_2P4_GHZ && defined FREQUENCY_BAND_915_MHZ_FCC && defined FREQUENCY_BAND_868_MHZ
    // DUALBAND!
    uint8_t frequency_band_default = SETUP_FREQUENCY_BAND_868_MHZ;
#elif defined FREQUENCY_BAND_2P4_GHZ
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
#elif defined FREQUENCY_BAND_866_MHZ_IN
    uint8_t frequency_band_default = SETUP_FREQUENCY_BAND_866_MHZ_IN;
#else
    #error Unknown Frequencyband !
#endif
    if (Setup.Common[config_id].FrequencyBand >= SETUP_FREQUENCY_BAND_NUM) {
        Setup.Common[config_id].FrequencyBand = frequency_band_default;
    }
    TST_NOTALLOWED(FrequencyBand_allowed_mask, Common[config_id].FrequencyBand, frequency_band_default);

    SANITIZE(Common[config_id].Mode, MODE_NUM, SETUP_MODE, MODE_19HZ);
    TST_NOTALLOWED(Mode_allowed_mask, Common[config_id].Mode, MODE_19HZ);

    SANITIZE(Common[config_id].Ortho, ORTHO_NUM, SETUP_RF_ORTHO, ORTHO_NONE);
    TST_NOTALLOWED(Ortho_allowed_mask, Common[config_id].Ortho, ORTHO_NONE);
    switch (Setup.Common[config_id].FrequencyBand) { // restrict ortho to 2.4GHz, 915FCC, 70CM
    case SETUP_FREQUENCY_BAND_2P4_GHZ:
    case SETUP_FREQUENCY_BAND_915_MHZ_FCC:
    case SETUP_FREQUENCY_BAND_70_CM_HAM:
        break;
    default:
        Setup.Common[config_id].Ortho = ORTHO_NONE;
    }

    //-- Tx:

    SANITIZE(Tx[config_id].Power, RFPOWER_LIST_NUM, SETUP_TX_POWER, RFPOWER_LIST_NUM - 1);

    SANITIZE(Tx[config_id].Diversity, DIVERSITY_NUM, SETUP_TX_DIVERSITY, DIVERSITY_DEFAULT);
    TST_NOTALLOWED(Tx_Diversity_allowed_mask, Tx[config_id].Diversity, DIVERSITY_ANTENNA1);

    SANITIZE(Tx[config_id].ChannelsSource, CHANNEL_SOURCE_NUM, SETUP_TX_CHANNELS_SOURCE, CHANNEL_SOURCE_NONE);
    TST_NOTALLOWED(Tx_ChannelsSource_allowed_mask, Tx[config_id].ChannelsSource, CHANNEL_SOURCE_NONE);

    SANITIZE(Tx[config_id].InMode, IN_CONFIG_NUM, SETUP_TX_IN_MODE, IN_CONFIG_SBUS);
    TST_NOTALLOWED(Tx_InMode_allowed_mask, Tx[config_id].InMode, IN_CONFIG_SBUS);

    SANITIZE(Tx[config_id].SerialDestination, SERIAL_DESTINATION_NUM, SETUP_TX_SERIAL_DESTINATION, SERIAL_DESTINATION_SERIAL);
    TST_NOTALLOWED(Tx_SerialDestination_allowed_mask, Tx[config_id].SerialDestination, SERIAL_DESTINATION_SERIAL);

    SANITIZE(Tx[config_id].ChannelOrder, CHANNEL_ORDER_NUM, SETUP_TX_CHANNEL_ORDER, CHANNEL_ORDER_AETR);

    SANITIZE(Tx[config_id].SerialBaudrate, SERIAL_BAUDRATE_NUM, SETUP_TX_SERIAL_BAUDRATE, SERIAL_BAUDRATE_115200);

    SANITIZE(Tx[config_id].SendRadioStatus, TX_SEND_RADIO_STATUS_NUM, SETUP_TX_SEND_RADIO_STATUS, TX_SEND_RADIO_STATUS_OFF);

    SANITIZE(Tx[config_id].Buzzer, BUZZER_NUM, SETUP_TX_BUZZER, BUZZER_OFF);
    TST_NOTALLOWED(Tx_Buzzer_allowed_mask, Tx[config_id].Buzzer, BUZZER_OFF);

    SANITIZE(Tx[config_id].CliLineEnd, CLI_LINE_END_NUM, SETUP_TX_CLI_LINE_END, CLI_LINE_END_CR);

    // device cannot use mBridge (pin5) and CRSF (pin5) at the same time !
    // dest\src | NONE    | CRSF    | INPORT  | MBRIDGE
    // -------------------------------------------------
    //  SERIAL  |  -      | crsf    | -       | mbridge
    //  SERIAL2 |  -      | crsf    | -       | mbridge
    //  MBRDIGE | mbridge | crsf !! | mbridge | mbridge
    if ((Setup.Tx[config_id].ChannelsSource == CHANNEL_SOURCE_CRSF) &&
        (Setup.Tx[config_id].SerialDestination == SERIAL_DESTINATION_MBRDIGE)) {
        Setup.Tx[config_id].SerialDestination = SERIAL_DESTINATION_SERIAL;
    }

    //-- Rx:

    SANITIZE(Rx.Power, RFPOWER_LIST_NUM, SETUP_RX_POWER, RFPOWER_LIST_NUM - 1);

    SANITIZE(Rx.Diversity, DIVERSITY_NUM, SETUP_RX_DIVERSITY, DIVERSITY_DEFAULT);
    TST_NOTALLOWED(Rx_Diversity_allowed_mask, Rx.Diversity,  DIVERSITY_ANTENNA1);

    SANITIZE(Rx.ChannelOrder, CHANNEL_ORDER_NUM, SETUP_RX_CHANNEL_ORDER, CHANNEL_ORDER_AETR);

    SANITIZE(Rx.OutMode, OUT_CONFIG_NUM, SETUP_RX_OUT_MODE, OUT_CONFIG_SBUS);
    TST_NOTALLOWED(Rx_OutMode_allowed_mask, Rx.OutMode, OUT_CONFIG_SBUS);

    SANITIZE(Rx.OutRssiChannelMode, OUT_RSSI_LQ_CHANNEL_NUM, SETUP_RX_OUT_RSSI_CHANNEL, 0);

    SANITIZE(Rx.OutLqChannelMode, OUT_RSSI_LQ_CHANNEL_NUM, SETUP_RX_OUT_LQ_CHANNEL, 0);

    SANITIZE(Rx.FailsafeMode, FAILSAFE_MODE_NUM, SETUP_RX_FAILSAFE_MODE, FAILSAFE_MODE_NO_SIGNAL);

    SANITIZE(Rx.Buzzer, BUZZER_LOST_PACKETS + 1, SETUP_RX_BUZZER, BUZZER_OFF);
    TST_NOTALLOWED(Rx_Buzzer_allowed_mask, Rx.Buzzer, BUZZER_OFF);

    for (uint8_t ch = 0; ch < 12; ch++) {
        if (Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[ch] < -120) Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[ch] = 0;
        if (Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[ch] > 120) Setup.Rx.FailsafeOutChannelValues_Ch1_Ch12[ch] = 0;
    }
    for (uint8_t ch = 0; ch < 4; ch++) {
        if (Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[ch] > 2) Setup.Rx.FailsafeOutChannelValues_Ch13_Ch16[ch] = 1;
    }

    SANITIZE(Rx.SerialBaudrate, SERIAL_BAUDRATE_NUM, SETUP_RX_SERIAL_BAUDRATE, SERIAL_BAUDRATE_57600);

    SANITIZE(Rx.SerialLinkMode, SERIAL_LINK_MODE_NUM, SETUP_RX_SERIAL_LINK_MODE, SERIAL_LINK_MODE_TRANSPARENT);

    SANITIZE(Rx.SendRadioStatus, RX_SEND_RADIO_STATUS_NUM, SETUP_RX_SEND_RADIO_STATUS, RX_SEND_RADIO_STATUS_OFF);

    SANITIZE(Rx.SendRcChannels, SEND_RC_CHANNELS_NUM, SETUP_RX_SEND_RC_CHANNELS, SEND_RC_CHANNELS_OFF);

    //-- Spares and deprecated options:
    // should be 0xFF'ed

    Setup.Tx[config_id].__SerialLinkMode = 0xFF;
    Setup.Rx.__RadioStatusMethod = 0xFF;

    for (uint8_t n = 0; n < sizeof(Setup.spare)/sizeof(Setup.spare[0]); n++) Setup.spare[n] = 0xFF;
    for (uint8_t n = 0; n < sizeof(Setup.Common[config_id].spare)/sizeof(Setup.Common[config_id].spare[0]); n++) Setup.Common[config_id].spare[n] = 0xFF;
    for (uint8_t n = 0; n < sizeof(Setup.Tx[config_id].spare)/sizeof(Setup.Tx[config_id].spare[0]); n++) Setup.Tx[config_id].spare[n] = 0xFF;
    for (uint8_t n = 0; n < sizeof(Setup.Rx.spare)/sizeof(Setup.Rx.spare[0]); n++) Setup.Rx.spare[n] = 0xFF;
}


//-------------------------------------------------------
// Configure
//-------------------------------------------------------

// also called by bind
void configure_mode(uint8_t mode)
{
    Config.Mode = mode;

    switch (Config.Mode) {
    case MODE_50HZ:
        Config.frame_rate_ms = 20; // 20 ms = 50 Hz
        Config.frame_rate_hz = 50;
        Config.Sx.LoraConfigIndex = SX128x_LORA_CONFIG_BW800_SF5_CRLI4_5;
        Config.send_frame_tmo_ms = MODE_50HZ_SEND_FRAME_TMO_MS; // 10;
        break;

    case MODE_31HZ:
        Config.frame_rate_ms = 32; // 32 ms = 31.25 Hz
        Config.frame_rate_hz = 31;
#ifdef DEVICE_HAS_SX128x
        Config.Sx.LoraConfigIndex = SX128x_LORA_CONFIG_BW800_SF6_CRLI4_5;
#else
        Config.Sx.LoraConfigIndex = SX126x_LORA_CONFIG_BW500_SF5_CR4_5;
#endif
        Config.send_frame_tmo_ms = MODE_31HZ_SEND_FRAME_TMO_MS; // 15
        break;

  case MODE_19HZ:
        Config.frame_rate_ms = 53; // 53 ms = 18.9 Hz
        Config.frame_rate_hz = 19;
#ifdef DEVICE_HAS_SX128x
        Config.Sx.LoraConfigIndex = SX128x_LORA_CONFIG_BW800_SF7_CRLI4_5;
#elif defined DEVICE_HAS_SX126x || defined DEVICE_HAS_DUAL_SX126x_SX128x
        Config.Sx.LoraConfigIndex = SX126x_LORA_CONFIG_BW500_SF6_CR4_5;
#else
        Config.Sx.LoraConfigIndex = SX127x_LORA_CONFIG_BW500_SF6_CR4_5;
#endif
        Config.send_frame_tmo_ms = MODE_19HZ_SEND_FRAME_TMO_MS; // 25;
        break;

    case MODE_FLRC_DEV:
        Config.frame_rate_ms = 7; // 7 ms = 143 Hz
        Config.frame_rate_hz = 143;
        Config.Sx.LoraConfigIndex = 0;
        Config.send_frame_tmo_ms = MODE_FLRC_SEND_FRAME_TMO_MS; // 3;
        break;

    default:
        while (1) {} // must not happen, should have been resolved in setup_sanitize()

    }

    Config.Sx2.LoraConfigIndex = Config.Sx.LoraConfigIndex;
#ifdef DEVICE_HAS_DUAL_SX126x_SX128x
    switch (Config.Mode) {
    case MODE_31HZ:
        Config.Sx2.LoraConfigIndex = SX128x_LORA_CONFIG_BW800_SF6_CRLI4_5;
        break;
  case MODE_19HZ:
        Config.Sx2.LoraConfigIndex = SX128x_LORA_CONFIG_BW800_SF7_CRLI4_5;
        break;
    default:
        while (1) {} // must not happen, should have been resolved in setup_sanitize()
    }
#endif

    // helper for sx drivers
    Config.Sx.is_lora = (Config.Mode != MODE_FLRC_DEV);
    Config.Sx2.is_lora = Config.Sx.is_lora;
}


void setup_configure_config(uint8_t config_id)
{
    //-- SyncWord
    // note: FrameSyncWord will be modified below by Ortho

    uint32_t bind_dblword = u32_from_bindphrase(Setup.Common[config_id].BindPhrase);

    Config.FrameSyncWord = fmav_crc_calculate((uint8_t*)&bind_dblword, 4); // condense it into a u16

    Config.Sx.FlrcSyncWord = bind_dblword;
    Config.Sx2.FlrcSyncWord = Config.Sx.FlrcSyncWord;

    //-- Power

    // note: the actually used power will be determined later when the SX are set up
#ifdef DEVICE_IS_TRANSMITTER
    Config.Sx.Power_dbm = rfpower_list[Setup.Tx[config_id].Power].dbm;
#endif
#ifdef DEVICE_IS_RECEIVER
    Config.Sx.Power_dbm = rfpower_list[Setup.Rx.Power].dbm;
#endif
    Config.Sx2.Power_dbm = Config.Sx.Power_dbm;

  //-- Diversity

#ifdef DEVICE_HAS_DIVERSITY
  #ifdef DEVICE_IS_TRANSMITTER
    switch (Setup.Tx[config_id].Diversity) {
  #endif
  #ifdef DEVICE_IS_RECEIVER
    switch (Setup.Rx.Diversity) {
  #endif
    case DIVERSITY_DEFAULT:
        Config.Diversity = DIVERSITY_DEFAULT;
        Config.ReceiveUseAntenna1 = true;
        Config.ReceiveUseAntenna2 = true;
        Config.TransmitUseAntenna1 = true;
        Config.TransmitUseAntenna2 = true;
        break;
    case DIVERSITY_ANTENNA1:
        Config.Diversity = DIVERSITY_ANTENNA1;
        Config.ReceiveUseAntenna1 = true;
        Config.ReceiveUseAntenna2 = false;
        Config.TransmitUseAntenna1 = true;
        Config.TransmitUseAntenna2 = false;
        break;
    case DIVERSITY_ANTENNA2:
        Config.Diversity = DIVERSITY_ANTENNA2;
        Config.ReceiveUseAntenna1 = false;
        Config.ReceiveUseAntenna2 = true;
        Config.TransmitUseAntenna1 = false;
        Config.TransmitUseAntenna2 = true;
        break;
    case DIVERSITY_R_ENABLED_T_ANTENNA1:
        Config.Diversity = DIVERSITY_R_ENABLED_T_ANTENNA1;
        Config.ReceiveUseAntenna1 = true;
        Config.ReceiveUseAntenna2 = true;
        Config.TransmitUseAntenna1 = true;
        Config.TransmitUseAntenna2 = false;
        break;
    case DIVERSITY_R_ENABLED_T_ANTENNA2:
        Config.Diversity = DIVERSITY_R_ENABLED_T_ANTENNA2;
        Config.ReceiveUseAntenna1 = true;
        Config.ReceiveUseAntenna2 = true;
        Config.TransmitUseAntenna1 = false;
        Config.TransmitUseAntenna2 = true;
        break;
    default:
        while (1) {} // must not happen, should have been resolved in setup_sanitize()
    }
#elif defined DEVICE_HAS_DUAL_SX126x_SX128x
    Config.Diversity = DIVERSITY_DEFAULT;
    Config.ReceiveUseAntenna1 = true;
    Config.ReceiveUseAntenna2 = true;
    Config.TransmitUseAntenna1 = true;
    Config.TransmitUseAntenna2 = true;
#else
    Config.Diversity = DIVERSITY_ANTENNA1;
    Config.ReceiveUseAntenna1 = true;
    Config.ReceiveUseAntenna2 = false;
    Config.TransmitUseAntenna1 = true;
    Config.TransmitUseAntenna2 = false;
#endif

    //-- FrequencyBand

    Config.FrequencyBand = Setup.Common[config_id].FrequencyBand;

    //-- Mode, Mode dependent settings, LoRa configuration

    configure_mode(Setup.Common[config_id].Mode);

    Config.Sx.FrequencyBand = Config.FrequencyBand;
    Config.Sx2.FrequencyBand = Config.Sx.FrequencyBand;

    //-- Fhss

    Config.Fhss.FrequencyBand = Config.FrequencyBand;
    Config.Fhss.FrequencyBand_allowed_mask = SetupMetaData.FrequencyBand_allowed_mask;

    //Config.FhssSeed = bind_dblword;
    // this is much better for narrow bands, like 868 MHz
    // we could make it dependable with a #ifdef, but what's the point
    Config.Fhss.Seed = Config.FrameSyncWord;

    Config.Fhss.Except = EXCEPT_NONE;
    if (Config.Fhss.FrequencyBand == SETUP_FREQUENCY_BAND_2P4_GHZ) {
        Config.Fhss.Except = except_from_bindphrase(Setup.Common[config_id].BindPhrase);
    }

    Config.Fhss.Ortho = Setup.Common[config_id].Ortho;
    // modify also FrameSyncWord
    if (Config.Fhss.Ortho > ORTHO_NONE) {
        Config.FrameSyncWord += 0x1111 * Config.Fhss.Ortho;
        Config.Sx.FlrcSyncWord += 0x11111111 * Config.Fhss.Ortho;
        Config.Sx2.FlrcSyncWord = Config.Sx.FlrcSyncWord;
    }

    switch (Config.Fhss.FrequencyBand) {
    case SETUP_FREQUENCY_BAND_2P4_GHZ:
        switch (Config.Mode) {
        case MODE_50HZ: Config.Fhss.Num = FHSS_NUM_BAND_2P4_GHZ; break;
        case MODE_31HZ: Config.Fhss.Num = FHSS_NUM_BAND_2P4_GHZ_31HZ_MODE; break;
        case MODE_19HZ: Config.Fhss.Num = FHSS_NUM_BAND_2P4_GHZ_19HZ_MODE; break;
        case MODE_FLRC_DEV: Config.Fhss.Num = FHSS_NUM_BAND_2P4_GHZ; break;
        default:
            while (1) {} // must not happen, should have been resolved in setup_sanitize()
        }
        break;
    case SETUP_FREQUENCY_BAND_915_MHZ_FCC:
        Config.Fhss.Num = FHSS_NUM_BAND_915_MHZ_FCC;
        break;
    case SETUP_FREQUENCY_BAND_868_MHZ:
        Config.Fhss.Num = FHSS_NUM_BAND_868_MHZ;
        break;
    case SETUP_FREQUENCY_BAND_433_MHZ:
        Config.Fhss.Num = FHSS_NUM_BAND_433_MHZ;
        break;
    case SETUP_FREQUENCY_BAND_70_CM_HAM:
        switch (Config.Mode) {
        case MODE_31HZ: Config.Fhss.Num = FHSS_NUM_BAND_70_CM_HAM; break;
        case MODE_19HZ: Config.Fhss.Num = FHSS_NUM_BAND_70_CM_HAM_19HZ_MODE; break;
        default:
            while (1) {} // must not happen, should have been resolved in setup_sanitize()
        }
        break;
    case SETUP_FREQUENCY_BAND_866_MHZ_IN:
        Config.Fhss.Num = FHSS_NUM_BAND_866_MHZ_IN;
        break;
    default:
        while (1) {} // must not happen, should have been resolved in setup_sanitize()
    }

    Config.Fhss2 = Config.Fhss;

#ifdef DEVICE_HAS_DUAL_SX126x_SX128x
    Config.Fhss2.FrequencyBand = SETUP_FREQUENCY_BAND_2P4_GHZ;
    Config.Fhss2.FrequencyBand_allowed_mask = (1 << SETUP_FREQUENCY_BAND_2P4_GHZ);
    switch (Config.Fhss2.FrequencyBand) {
    case SETUP_FREQUENCY_BAND_2P4_GHZ:
        switch (Config.Mode) {
        case MODE_31HZ: Config.Fhss2.Num = FHSS_NUM_BAND_2P4_GHZ_31HZ_MODE; break;
        case MODE_19HZ: Config.Fhss2.Num = FHSS_NUM_BAND_2P4_GHZ_19HZ_MODE; break;
        default:
            while (1) {} // must not happen, should have been resolved in setup_sanitize()
        }
        break;
    default:
        while (1) {} // must not happen, should have been resolved in setup_sanitize()
    }
#endif

    //-- More Config, may depend on above config settings

    Config.connect_tmo_systicks = SYSTICK_DELAY_MS(CONNECT_TMO_MS);
    Config.connect_listen_hop_cnt = (uint8_t)(1.5f * Config.Fhss.Num);

    Config.LQAveragingPeriod = (LQ_AVERAGING_MS/Config.frame_rate_ms);

    //-- Serial

#ifdef DEVICE_IS_TRANSMITTER
    switch (Setup.Tx[config_id].SerialBaudrate) {
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
#ifdef DEVICE_IS_TRANSMITTER
        Config.SerialBaudrate = 115200;
#else
        Config.SerialBaudrate = 57600;
#endif
    }

    //-- Mbridge, Crsf

    Config.UseMbridge = false;
    Config.UseCrsf = false;
#if defined DEVICE_IS_TRANSMITTER && defined DEVICE_HAS_JRPIN5
    // conflicts must have been sorted out before in setup_sanitize()
    if ((Setup.Tx[config_id].ChannelsSource == CHANNEL_SOURCE_MBRIDGE) ||
        (Setup.Tx[config_id].SerialDestination == SERIAL_DESTINATION_MBRDIGE)) {
        Config.UseMbridge = true;
    }
    if (Setup.Tx[config_id].ChannelsSource == CHANNEL_SOURCE_CRSF) {
        Config.UseCrsf = true;
    }
    if (Config.UseMbridge && Config.UseCrsf) {
        while (1) {} // mBridge and CRSF cannot be used simultaneously, must not happen
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
    setup_sanitize_config(Config.ConfigId);
}


//-------------------------------------------------------
// Init
//-------------------------------------------------------

#define SETUPLAYOUT_L0_0_02    2 // layout version up to v0.3.28
#define SETUPLAYOUT_L0_3_29  329 // layout version from v0.3.29 - v0.3.34
#define SETUPLAYOUT_L0_3_35  335 // layout version from v0.3.35 onwards


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
        if (Setup.Layout < SETUPLAYOUT_L0_3_29) {
            strstrbufcpy(Setup.Common[0].BindPhrase, Setup.__BindPhrase, 6);
            Setup.Common[0].FrequencyBand = Setup.__FrequencyBand;
            Setup.Common[0].Mode = Setup.__Mode;
            for (uint8_t id = 1; id < SETUP_CONFIG_LEN; id++) setup_default(id); // default all other tables
            Setup._ConfigId = 0;
        } else
        if (Setup.Layout < SETUPLAYOUT_L0_3_35) {
            for (uint8_t id = 0; id < SETUP_CONFIG_LEN; id++) {
                // Tx ChannelSource rearranged
                uint8_t tx_channel_source = Setup.Tx[id].ChannelsSource;
                switch (tx_channel_source) {
                case L0329_CHANNEL_SOURCE_MBRIDGE: Setup.Tx[id].ChannelsSource = CHANNEL_SOURCE_MBRIDGE; break;
                case L0329_CHANNEL_SOURCE_CRSF: Setup.Tx[id].ChannelsSource = CHANNEL_SOURCE_CRSF; break;
                }
                // Tx SerialDestination rearranged
                uint8_t tx_serial_destination = Setup.Tx[id].SerialDestination;
                switch (tx_serial_destination) {
                case L0329_SERIAL_DESTINATION_MBRDIGE: Setup.Tx[id].SerialDestination = SERIAL_DESTINATION_MBRDIGE; break;
                case L0329_SERIAL_DESTINATION_SERIAL2: Setup.Tx[id].SerialDestination = SERIAL_DESTINATION_SERIAL2; break;
                }
            }
        } else {
            for (uint8_t id = 0; id < SETUP_CONFIG_LEN; id++) setup_default(id);
            Setup._ConfigId = 0;
        }
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

#ifdef DEVICE_IS_TRANSMITTER
    if (Setup._ConfigId >= SETUP_CONFIG_LEN) Setup._ConfigId = 0;

    if (Setup.Tx[Setup._ConfigId].ChannelsSource == CHANNEL_SOURCE_MBRIDGE ||
        Setup.Tx[Setup._ConfigId].ChannelsSource == CHANNEL_SOURCE_CRSF) {
        // config id is supported
        // ensure that they all have the same channel_source
        for (uint8_t id = 0; id < SETUP_CONFIG_LEN; id++) {
            Setup.Tx[id].ChannelsSource = Setup.Tx[Setup._ConfigId].ChannelsSource;
        }
    } else {
        // config id is not supported, so force config_id = 0 page
        if (Setup._ConfigId > 0) Setup.Tx[0] = Setup.Tx[Setup._ConfigId]; // copy all settings to page 0
        Setup._ConfigId = 0;
    }
#else
    Setup._ConfigId = 0;
#endif

    Config.ConfigId = Setup._ConfigId;

    setup_configure_metadata();

#ifdef SETUP_FORCE_COMMON_CONF
    setup_default(Config.ConfigId);
#endif

    setup_sanitize_config(Config.ConfigId);

    setup_configure_config(Config.ConfigId);
}


#endif // SETUP_H
