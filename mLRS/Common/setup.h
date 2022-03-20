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


tSetupMetaData SetupMetaData;

tSetup Setup;
tGlobalConfig Config;


void setup_configure_metadata(void)
{
    SetupMetaData = {0};

    // "50 Hz,31 Hz,19 Hz"
#ifdef DEVICE_HAS_SX128x
    SetupMetaData.Mode_allowed_mask = UINT16_MAX; // all
#elif defined DEVICE_HAS_SX126x
    SetupMetaData.Mode_allowed_mask = 0b0110; // only 31 Hz, 19 Hz
#elif defined DEVICE_HAS_SX127x
    SetupMetaData.Mode_allowed_mask = 0b0100; // only 19 Hz
#endif

    //-- Tx:

#ifdef DEVICE_IS_TRANSMITTER
    strcpy(SetupMetaData.Tx_Power_optstr, RFPOWER_OPTSTR);
#else
    strcpy(SetupMetaData.Tx_Power_optstr, "min");
#endif

    // "on,antenna1,antenna2"
#ifdef DEVICE_HAS_DIVERSITY
    SetupMetaData.Tx_Diversity_allowed_mask = UINT16_MAX; // all
#else
    SetupMetaData.Tx_Diversity_allowed_mask = 0b0010; // only antenna1
#endif

    // "none,mbridge,in,crsf"
#if defined DEVICE_HAS_JRPIN5 && defined DEVICE_HAS_IN
    SetupMetaData.Tx_ChannelsSource_allowed_mask = UINT16_MAX; // all
#elif defined DEVICE_HAS_JRPIN5
    SetupMetaData.Tx_ChannelsSource_allowed_mask = 0b1011; // only none, mbridge, crsf
#elif defined DEVICE_HAS_IN
    SetupMetaData.Tx_ChannelsSource_allowed_mask = 0b0101; // only none, in
#else
    SetupMetaData.Tx_ChannelsSource_allowed_mask = 0b0001; // only none
#endif

    // "sbus,sbus inv"
#ifdef DEVICE_HAS_IN
    SetupMetaData.Tx_InMode_allowed_mask = UINT16_MAX; // all
#else
    SetupMetaData.Tx_InMode_allowed_mask = 0b0001; // default to sbus
#endif

    // "serial,mbridge"
#ifdef DEVICE_HAS_JRPIN5
    SetupMetaData.Tx_SerialDestination_allowed_mask = UINT16_MAX; // all
#else
    SetupMetaData.Tx_SerialDestination_allowed_mask = 0b0001; // only serial
#endif

    //-- Rx:

#ifdef DEVICE_IS_RECEIVER
    strcpy(SetupMetaData.Rx_Power_optstr, RFPOWER_OPTSTR);
#else
    strcpy(SetupMetaData.Rx_Power_optstr, "min");
#endif

    // "on,antenna1,antenna2"
#ifdef DEVICE_HAS_DIVERSITY
    SetupMetaData.Rx_Diversity_allowed_mask = UINT16_MAX; // all
#else
    SetupMetaData.Rx_Diversity_allowed_mask = 0b0010; // only antenna1
#endif

    SetupMetaData.rx_available = true; //false;
    SetupMetaData.rx_firmware_version = 211;
    SetupMetaData.rx_device_name[0] = 'a';
    SetupMetaData.rx_device_name[1] = 'b';
    SetupMetaData.rx_device_name[2] = '\0';
    SetupMetaData.rx_actual_power_dbm = INT8_MAX;
    SetupMetaData.rx_actual_diversity = 3;
}


void setup_default(void)
{
    strncpy_x(Setup.BindPhrase, BIND_PHRASE, 6); // 6 chars
    Setup.Mode = SETUP_MODE;

    Setup.Tx.Power = 0;
    Setup.Tx.Diversity = SETUP_TX_DIVERSITY;
    Setup.Tx.SerialDestination = SETUP_TX_SERIAL_DESTINATION;
    Setup.Tx.ChannelsSource = SETUP_TX_CHANNELS_SOURCE;
    Setup.Tx.ChannelOrder = SETUP_TX_CHANNEL_ORDER;
    Setup.Tx.InMode = SETUP_TX_IN_MODE;
    Setup.Tx.SerialBaudrate = SERIAL_BAUDRATE_57600;
    Setup.Tx.SerialLinkMode = SETUP_TX_SERIAL_LINK_MODE;
    Setup.Tx.SendRadioStatus = SETUP_TX_SEND_RADIO_STATUS;

    Setup.Rx.Power = 0;
    Setup.Rx.Diversity = SETUP_RX_DIVERSITY;
    Setup.Rx.ChannelOrder = SETUP_RX_CHANNEL_ORDER;
    Setup.Rx.OutMode = SETUP_RX_OUT_MODE;
    Setup.Rx.OutRssiChannelMode = SETUP_RX_OUT_RSSI_CHANNEL;
    Setup.Rx.FailsafeMode = SETUP_RX_FAILSAFE_MODE;
    for (uint8_t ch = 0; ch < 16; ch++) { Setup.Rx.FailsafeOutChannelValues[ch] = 0; }
    Setup.Rx.SerialBaudrate = SERIAL_BAUDRATE_57600;
    Setup.Rx.SerialLinkMode = SETUP_RX_SERIAL_LINK_MODE;
    Setup.Rx.SendRadioStatus = SETUP_RX_SEND_RADIO_STATUS;
}


void setup_sanitize(void)
{
#define SETUP_TST_ALLOWED(x,y) (SetupMetaData.x & (1 << Setup.y))

    sanitize_bind_phrase(Setup.BindPhrase);

    if (Setup.Mode >= MODE_NUM) Setup.Mode = MODE_19HZ;
    if (SETUP_TST_ALLOWED(Mode_allowed_mask,Mode) == 0) Setup.Mode = MODE_19HZ;

    //-- Tx:

#ifdef DEVICE_IS_TRANSMITTER
    if (Setup.Tx.Power >= RFPOWER_LIST_NUM) Setup.Tx.Power = 0;
#else
    Setup.Tx.Power = 0;
#endif

    if (Setup.Tx.Diversity >= DIVERSITY_NUM) Setup.Tx.Diversity = DIVERSITY_DEFAULT;
    if (SETUP_TST_ALLOWED(Tx_Diversity_allowed_mask,Tx.Diversity) == 0) Setup.Tx.Diversity = DIVERSITY_ANTENNA1;

    if (Setup.Tx.ChannelsSource >= CHANNEL_SOURCE_NUM) Setup.Tx.ChannelsSource = CHANNEL_SOURCE_NONE;
    if (SETUP_TST_ALLOWED(Tx_ChannelsSource_allowed_mask,Tx.ChannelsSource) == 0) Setup.Tx.ChannelsSource = CHANNEL_SOURCE_NONE;

    if (Setup.Tx.InMode >= IN_CONFIG_NUM) Setup.Tx.InMode = IN_CONFIG_SBUS;
    if (SETUP_TST_ALLOWED(Tx_InMode_allowed_mask,Tx.InMode) == 0) Setup.Tx.InMode = IN_CONFIG_SBUS;

    if (Setup.Tx.SerialDestination >= SERIAL_DESTINATION_NUM) Setup.Tx.SerialDestination = SERIAL_DESTINATION_SERIAL_PORT;
    if (SETUP_TST_ALLOWED(Tx_SerialDestination_allowed_mask,Tx.SerialDestination) == 0) Setup.Tx.Diversity = DIVERSITY_ANTENNA1;

    if (Setup.Tx.ChannelOrder >= CHANNEL_ORDER_NUM) Setup.Tx.ChannelOrder = CHANNEL_ORDER_AETR;
    if (Setup.Tx.SerialBaudrate >= SERIAL_BAUDRATE_NUM) Setup.Tx.SerialBaudrate = SERIAL_BAUDRATE_57600;
    if (Setup.Tx.SerialLinkMode >= SERIAL_LINK_MODE_NUM) Setup.Tx.SerialLinkMode = SERIAL_LINK_MODE_TRANSPARENT;
    if (Setup.Tx.SendRadioStatus >= SEND_RADIO_STATUS_NUM) Setup.Tx.SendRadioStatus = SEND_RADIO_STATUS_OFF;

    // device cannot use mBridge (pin5) and CRSF (pin5) at the same time !
    if ((Setup.Tx.SerialDestination == SERIAL_DESTINATION_MBRDIGE) && (Setup.Tx.ChannelsSource == CHANNEL_SOURCE_CRSF)) {
        Setup.Tx.ChannelsSource = CHANNEL_SOURCE_NONE;
    }

    //-- Rx:

#ifdef DEVICE_IS_RECEIVER
    if (Setup.Rx.Power >= RFPOWER_LIST_NUM) Setup.Rx.Power = 0;
#else
    Setup.Rx.Power = 0;
#endif

    if (Setup.Rx.Diversity >= DIVERSITY_NUM) Setup.Rx.Diversity = DIVERSITY_DEFAULT;
    if (SETUP_TST_ALLOWED(Rx_Diversity_allowed_mask,Rx.Diversity) == 0) Setup.Rx.Diversity = DIVERSITY_ANTENNA1;

    if (Setup.Rx.ChannelOrder >= CHANNEL_ORDER_NUM) Setup.Rx.ChannelOrder = CHANNEL_ORDER_AETR;
    if (Setup.Rx.OutMode >= OUT_CONFIG_NUM) Setup.Rx.OutMode = OUT_CONFIG_SBUS;
    if (Setup.Rx.OutRssiChannelMode >= OUT_RSSI_CHANNEL_NUM) Setup.Rx.OutRssiChannelMode = 0;
    if (Setup.Rx.FailsafeMode >= FAILSAFE_MODE_NUM) Setup.Rx.FailsafeMode = FAILSAFE_MODE_NO_SIGNAL;
    for (uint8_t ch = 0; ch < 16; ch++) {
        if (Setup.Rx.FailsafeOutChannelValues[ch] < -120) Setup.Rx.FailsafeOutChannelValues[ch] = 0;
        if (Setup.Rx.FailsafeOutChannelValues[ch] > 120) Setup.Rx.FailsafeOutChannelValues[ch] = 0;
    }
    if (Setup.Rx.SerialBaudrate >= SERIAL_BAUDRATE_NUM) Setup.Rx.SerialBaudrate = SERIAL_BAUDRATE_57600;
    if (Setup.Rx.SerialLinkMode >= SERIAL_LINK_MODE_NUM) Setup.Rx.SerialLinkMode = SERIAL_LINK_MODE_TRANSPARENT;
    if (Setup.Rx.SendRadioStatus >= SEND_RADIO_STATUS_NUM) Setup.Rx.SendRadioStatus = SEND_RADIO_STATUS_OFF;
}


void setup_configure(void)
{
    //-- SyncWord
    uint32_t bind_dblword = u32_from_bind_phrase(Setup.BindPhrase);

    Config.FrameSyncWord = (uint16_t)(bind_dblword & 0x0000FFFF);

    //-- Frequency band

#ifdef FREQUENCY_BAND_2P4_GHZ
    Config.FrequencyBand = CONFIG_FREQUENCY_BAND_2P4_GHZ;
#elif defined FREQUENCY_BAND_915_MHZ_FCC
    Config.FrequencyBand = CONFIG_FREQUENCY_BAND_915_MHZ_FCC;
#elif defined FREQUENCY_BAND_868_MHZ
    Config.FrequencyBand = CONFIG_FREQUENCY_BAND_868_MHZ;
#else
    #error Unknown Frequencyband !
#endif

    //-- Power

    // TODO: we momentarily use the POWER values, but eventually we need to use the power_list[] array and setup Rx/Tx power
    // note: the actually used power will be determined later when the SX are set up
#ifdef DEVICE_IS_TRANSMITTER
    Config.Power = SETUP_TX_POWER; //power_list[Setup.Tx.Power];
#endif
#ifdef DEVICE_IS_RECEIVER
    Config.Power = SETUP_RX_POWER; //power_list[Setup.Rx.Power];
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
    }
#else
    Config.UseAntenna1 = true;
    Config.UseAntenna2 = false;
#endif

    //-- Mode dependent setting, LoRa configuration

    switch (Setup.Mode) {
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
        while (1) {} // must not happen
    }

    //-- Fhss

    Config.FhssSeed = bind_dblword;

#if defined FREQUENCY_BAND_868_MHZ
    Config.FhssNum = FHSS_NUM_BAND_868_MHZ;
#elif defined FREQUENCY_BAND_915_MHZ_FCC
    Config.FhssNum = FHSS_NUM_BAND_915_MHZ_FCC;
#else
    switch (Setup.Mode) {
    case MODE_50HZ:
        Config.FhssNum = FHSS_NUM_BAND_2P4_GHZ;
        break;
    case MODE_31HZ:
        Config.FhssNum = FHSS_NUM_BAND_2P4_GHZ_31HZ_MODE;
        break;
    case MODE_19HZ:
        Config.FhssNum = FHSS_NUM_BAND_2P4_GHZ_19HZ_MODE;
        break;
    }
#endif

    //-- More Config, may depend on above config settings

    Config.connect_tmo_systicks = SYSTICK_DELAY_MS((uint16_t)( (float)CONNECT_TMO_MS + 0.75f * Config.frame_rate_ms));
    Config.connect_listen_hop_cnt = (uint8_t)(1.5f * Config.FhssNum);

    Config.LQAveragingPeriod = (LQ_AVERAGING_MS/Config.frame_rate_ms);
}


void setup_init(void)
{
    setup_configure_metadata();

    setup_default();

    setup_sanitize();

    setup_configure();
}


#endif // SETUP_H
