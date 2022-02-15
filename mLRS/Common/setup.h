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


tSetup Setup;
tGlobalConfig Config;


void setup_default(void)
{
  Setup.Tx.SerialDestination = SETUP_TX_SERIAL_DESTINATION;
  Setup.Tx.ChannelsSource = SETUP_TX_CHANNELS_SOURCE;
  Setup.Tx.ChannelOrder = SETUP_TX_CHANNEL_ORDER;
  Setup.Tx.Power = 0;
  Setup.Tx.SendRadioStatus = SETUP_TX_SEND_RADIO_STATUS;
  Setup.Tx.Diversity = SETUP_TX_DIVERSITY;

  Setup.Rx.ChannelOrder = SETUP_RX_CHANNEL_ORDER;
  Setup.Rx.OutMode = SETUP_RX_OUT_MODE;
  Setup.Rx.FailsafeMode = SETUP_RX_FAILSAFE_MODE;
  Setup.Rx.Power = 0;
  Setup.Rx.SerialBaudrate = SETUP_RX_SERIAL_BAUDRATE;
  Setup.Rx.SendRadioStatus = SETUP_RX_SEND_RADIO_STATUS;
  Setup.Rx.Diversity = SETUP_RX_DIVERSITY;

  Setup.BindDblWord = BIND_DBLWORD;

  Setup.Mode = SETUP_MODE;
}


void setup_sanitize(void)
{
  // device cannot use mBridge (pin5) and CRSF (pin5) at the same time !
  if ((Setup.Tx.SerialDestination == SERIAL_DESTINATION_MBRDIGE) && (Setup.Tx.ChannelsSource == CHANNEL_SOURCE_CRSF)) {
    Setup.Tx.ChannelsSource = CHANNEL_SOURCE_NONE;
  }

#ifdef DEVICE_IS_TRANSMITTER
#ifndef DEVICE_HAS_MBRIDGE
  // device doesn't support half-duplex pin 5
  if (Setup.Tx.SerialDestination == SERIAL_DESTINATION_MBRDIGE) Setup.Tx.SerialDestination = SERIAL_DESTINATION_SERIAL_PORT;
  if (Setup.Tx.ChannelsSource == CHANNEL_SOURCE_MBRIDGE) Setup.Tx.ChannelsSource = CHANNEL_SOURCE_NONE;
  if (Setup.Tx.ChannelsSource == CHANNEL_SOURCE_CRSF) Setup.Tx.ChannelsSource = CHANNEL_SOURCE_NONE;
#else
  if (Setup.Tx.SerialDestination == SERIAL_DESTINATION_MBRDIGE) {
    // mBridge & CRSF cannot be used simultaneously
    if (Setup.Tx.ChannelsSource == CHANNEL_SOURCE_CRSF) Setup.Tx.ChannelsSource = CHANNEL_SOURCE_NONE;
  }
#endif

#ifndef DEVICE_HAS_IN
  if (Setup.Tx.ChannelsSource == CHANNEL_SOURCE_SPORT) Setup.Tx.ChannelsSource = CHANNEL_SOURCE_NONE;
#endif
#endif

#ifndef DEVICE_HAS_DIVERSITY
#ifdef DEVICE_IS_TRANSMITTER
  if (Setup.Tx.Diversity >= DIVERSITY_ANTENNA2) Setup.Tx.Diversity = DIVERSITY_DEFAULT;
#endif
#ifdef DEVICE_IS_RECEIVER
  if (Setup.Rx.Diversity >= DIVERSITY_ANTENNA2) Setup.Rx.Diversity = DIVERSITY_DEFAULT;
#endif
#endif

#if defined DEVICE_HAS_SX126x || defined DEVICE_HAS_SX127x
  Setup.Mode = MODE_19HZ; // only 19 Hz mode allowed
#endif
}


void setup_configure(void)
{
  // TODO: we momentarily use the POWER values, but eventually we need to use the power_list[] array and setup Rx/Tx power
#ifdef DEVICE_IS_TRANSMITTER
  Config.Power = SETUP_TX_POWER;
#endif
#ifdef DEVICE_IS_RECEIVER
  Config.Power = SETUP_RX_POWER;
#endif

  switch (Setup.Mode) {
  case MODE_50HZ:
  default:
    Config.frame_rate_ms = 20; // 20 ms = 50 Hz
    Config.LoraConfigIndex = 0;
    Config.lora_send_frame_tmo = MODE_50HZ_SEND_FRAME_TMO; // 10;
    Config.lora_set_to_rx_tmo = MODE_50HZ_TX_SET_RX_TMO; // 11;
    break;
  case MODE_19HZ:
    Config.frame_rate_ms = 53; // 53 ms = 18.9 Hz
    Config.LoraConfigIndex = 1;
    Config.lora_send_frame_tmo = MODE_19HZ_SEND_FRAME_TMO; // 25;
    Config.lora_set_to_rx_tmo = MODE_19HZ_TX_SET_RX_TMO; // 30;
    break;
  }

  Config.FrameSyncWord = (uint16_t)(Setup.BindDblWord & 0x0000FFFF);
  Config.FhssSeed = Setup.BindDblWord;
  Config.FhssNum = FHSS_NUM;
  Config.LQAveragingPeriod = (LQ_AVERAGING_MS/Config.frame_rate_ms);

  Config.connect_tmo_systicks = SYSTICK_DELAY_MS((uint16_t)( (float)CONNECT_TMO_MS + 0.75f * Config.frame_rate_ms));
  Config.connect_listen_hop_cnt = (uint8_t)(1.5f * Config.FhssNum);

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
}


void setup_init(void)
{
  setup_default();

  setup_sanitize();

  setup_configure();
}



#endif // SETUP_H
