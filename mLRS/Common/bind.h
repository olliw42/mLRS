//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Bind
//*******************************************************
#ifndef BIND_H
#define BIND_H
#pragma once


#include <stdint.h>
#include "common_conf.h"
#include "hal/device_conf.h"


extern volatile uint32_t millis32(void);
extern bool connected(void);
#ifdef DEVICE_IS_RECEIVER
extern void clock_reset(void);
#endif

extern SX_DRIVER sx;
extern SX2_DRIVER sx2;

void sxReadFrame(uint8_t antenna, void* const data, void* const data2, uint8_t len);
void sxSendFrame(uint8_t antenna, void* const data, uint8_t len, uint16_t tmo_ms);
void sxGetPacketStatus(uint8_t antenna, tStats* const stats);

extern tStats stats;


//-------------------------------------------------------
// Bind Class
//-------------------------------------------------------

#define BIND_SIGNATURE_TX_STR     "mLRS\x01\x02\x03\x04"
#define BIND_SIGNATURE_RX_STR     "mLRS\x04\x03\x02\x01"
#define BIND_BUTTON_DEBOUNCE_MS   50
#define BIND_BUTTON_TMO_MS        4000


typedef enum {
    BIND_TASK_NONE = 0,
    BIND_TASK_CHANGED_TO_BIND,
    BIND_TASK_RX_STORE_PARAMS,
    BIND_TASK_TX_RESTART_CONTROLLER,
} BIND_TASK_ENUM;


class tBindBase
{
  public:
    void Init(void);
    bool IsInBind(void) { return is_in_binding; }
    void StartBind(void) { if (!is_in_binding) binding_requested = true; }
    void StopBind(void) { if (is_in_binding) binding_stop_requested = true; }
    void ConfigForBind(void);
    void HopToNextBind(uint16_t frequency_band); // SETUP_FREQUENCY_BAND_ENUM
    void Tick_ms(void);
    void Do(void);
    uint8_t Task(void);

    void AutoBind(void); // only for receiver, call every ms
    uint32_t auto_bind_tmo_ms;

    bool is_in_binding; // is in sync with link loop
    bool binding_requested;
    bool binding_stop_requested;
    uint32_t button_tlast_ms;
    uint8_t task;
    bool is_connected;

    uint64_t TxSignature; // 8 bytes, signature of Tx module
    uint64_t RxSignature; // 8 bytes, signature of Rx module

    void handle_receive(uint8_t antenna, uint8_t rx_status);
    void do_transmit(uint8_t antenna);
    uint8_t do_receive(uint8_t antenna, bool do_clock_reset);
    void config_rf(void);

    bool is_pressed;
    int8_t pressed_cnt;

  private:
    tSxGlobalConfig* gconfig;
    uint16_t mode_mask; // mask to handle toggling between 19 Hz and 19 Hz7x mode
};


void tBindBase::Init(void)
{
    is_in_binding = false;
    binding_requested = false;
    binding_stop_requested = false;
    task = BIND_TASK_NONE;
    is_connected = false;

    button_tlast_ms = millis32();
    is_pressed = false;
    pressed_cnt = 0;

    memcpy(&TxSignature, BIND_SIGNATURE_TX_STR, 8);
    memcpy(&RxSignature, BIND_SIGNATURE_RX_STR, 8);

    auto_bind_tmo_ms = 1000 * RX_BIND_MODE_AFTER_POWERUP_TIME_SEC;

    mode_mask = 0;
}


void tBindBase::ConfigForBind(void)
{
    // used by both the Tx and Rx, switch to 19 Hz mode, select lowest possible power
    // we technically have to distinguish between MODE_19HZ or MODE_19HZ_7X
    // configure_mode() however does currently do the same for both cases
    if (Config.Mode == MODE_19HZ_7X) {
        mode_mask = 0xFFFF;
        configure_mode(MODE_19HZ_7X, Config.FrequencyBand);
        mode_mask &=~ (1 << Config.FrequencyBand); // clear bit for current frequency band
    } else {
        mode_mask = 0;
        configure_mode(MODE_19HZ, Config.FrequencyBand);
        mode_mask |= (1 << Config.FrequencyBand); // set bit for current frequency band
    }

    config_rf();
}


void tBindBase::HopToNextBind(uint16_t frequency_band) // SETUP_FREQUENCY_BAND_ENUM
{
    // not nice
    // used only by Rx
    // we would need SetupMetaData.Mode_allowed_mask before it is adjusted for the selected frequency band
    // we could keep a copy of the un-adjusted SetupMetaData.Mode_allowed_mask
    // we also could provide a function to do it both in setup.h and here
    // for the moment reconstruct the info by explicit defines
#if defined DEVICE_HAS_LR11xx
    uint16_t mode_allowed_mask = 0b110111; // 50 Hz, 31 Hz, 19 Hz, 19 Hz 7x, FSK // only important that both 19Hz and 19Hz7x are set
#elif defined DEVICE_HAS_SX127x
    uint16_t mode_allowed_mask = 0b100000; // 19 Hz 7x, not editable // only important that only 19Hz7x is set
#else
    uint16_t mode_allowed_mask = 0b000111; // 50 Hz, 31 Hz, 19 Hz // only important that only 19Hz is set
#endif

    // if both 19Hz and 19Hz7X are set, we need to cycle with toggles
    if ((mode_allowed_mask & (1 << MODE_19HZ)) && (mode_allowed_mask & (1 << MODE_19HZ_7X))) {
        if (frequency_band == SETUP_FREQUENCY_BAND_2P4_GHZ) {
            configure_mode(MODE_19HZ, frequency_band);
        } else {
            if (mode_mask & (1 << frequency_band)) {
                configure_mode(MODE_19HZ_7X, frequency_band);
                mode_mask &=~ (1 << frequency_band); // clear bit
            } else {
                configure_mode(MODE_19HZ, frequency_band);
                mode_mask |= (1 << frequency_band); // set bit
            }
        }
        config_rf();
    }
}


void tBindBase::config_rf(void)
{
    sx.SetToIdle();
    sx2.SetToIdle();
    sx.SetRfPower_dbm(rfpower_list[0].dbm);
    sx2.SetRfPower_dbm(rfpower_list[0].dbm);
    IF_SX(sx.ResetToLoraConfiguration();)
    IF_SX2(sx2.ResetToLoraConfiguration();)
    sx.SetToIdle();
    sx2.SetToIdle();
}


// called each ms
void tBindBase::Tick_ms(void)
{
    // a not so efficient but simple debounce
    if (!is_pressed) {
        if (button_pressed()) { pressed_cnt++; } else { pressed_cnt = 0; }
        if (pressed_cnt >= BIND_BUTTON_DEBOUNCE_MS) is_pressed = true;
    } else {
        if (!button_pressed()) { pressed_cnt--; } else { pressed_cnt = BIND_BUTTON_DEBOUNCE_MS; }
        if (pressed_cnt <= 0) is_pressed = false;
    }
}


// called in each doPreTransmit or doPostReceive cycle
void tBindBase::Do(void)
{
    uint32_t tnow = millis32();

    if (is_pressed) {
        if (tnow - button_tlast_ms > BIND_BUTTON_TMO_MS) {
            binding_requested = true;
        }
    } else {
        button_tlast_ms = tnow;
    }

#ifdef DEVICE_IS_TRANSMITTER
    if (is_in_binding) {
        if (is_connected && !connected()) { // we just lost connection
            task = BIND_TASK_TX_RESTART_CONTROLLER;
        }
        is_connected = connected();

        if (binding_stop_requested) {
            task = BIND_TASK_TX_RESTART_CONTROLLER;
        }
    }
#endif

    if (!is_in_binding && binding_requested) {
        is_in_binding = true;
        task = BIND_TASK_CHANGED_TO_BIND;
    }
}


// called directly after bind.Do()
uint8_t tBindBase::Task(void)
{
    switch (task) {
    case BIND_TASK_TX_RESTART_CONTROLLER:
    case BIND_TASK_RX_STORE_PARAMS:
        // postpone until button is released, prevents jumping to RESTART while button is till pressed by user
        if (is_pressed) return BIND_TASK_NONE;
        break;
    }

    uint8_t ret = task;
    task = BIND_TASK_NONE;
    return ret;
}


void tBindBase::AutoBind(void) // only for receiver, call every ms
{
#if defined DEVICE_IS_RECEIVER && defined RX_BIND_MODE_AFTER_POWERUP
    if (!auto_bind_tmo_ms) return;

    auto_bind_tmo_ms--;

    if (auto_bind_tmo_ms == 0) {
        binding_requested = true;
    }
#endif
}


tTxBindFrame txBindFrame;
tRxBindFrame rxBindFrame;


#ifdef DEVICE_IS_TRANSMITTER

void tBindBase::handle_receive(uint8_t antenna, uint8_t rx_status)
{
    if (rx_status == RX_STATUS_INVALID) return;

    // do stuff
}


void tBindBase::do_transmit(uint8_t antenna)
{
    memset((uint8_t*)&txBindFrame, 0, sizeof(txBindFrame));
    txBindFrame.bind_signature = TxSignature;

    txBindFrame.connected = connected();

    strbufstrcpy(txBindFrame.BindPhrase_6, Setup.Common[Config.ConfigId].BindPhrase, 6);
    // TODO txBindFrame.FrequencyBand = Setup.Common[Config.ConfigId].FrequencyBand;
    txBindFrame.Mode = Setup.Common[Config.ConfigId].Mode;
    txBindFrame.Ortho = Setup.Common[Config.ConfigId].Ortho;

    txBindFrame.crc = fmav_crc_calculate((uint8_t*)&txBindFrame, FRAME_TX_RX_LEN - 2);
}


uint8_t tBindBase::do_receive(uint8_t antenna, bool do_clock_reset)
{
    sxReadFrame(antenna, &rxBindFrame, &rxBindFrame, FRAME_TX_RX_LEN);

    bool ok = (rxBindFrame.bind_signature == RxSignature);
    if (ok) {
        uint16_t crc = fmav_crc_calculate((uint8_t*)&rxBindFrame, FRAME_TX_RX_LEN - 2);
        ok = (crc == rxBindFrame.crc);
    }

    sxGetPacketStatus(antenna, &stats);

    if (ok) return RX_STATUS_VALID;

    return RX_STATUS_INVALID;
}

#endif
#ifdef DEVICE_IS_RECEIVER

void tBindBase::handle_receive(uint8_t antenna, uint8_t rx_status)
{
    if (rx_status == RX_STATUS_INVALID) return;

    strstrbufcpy(Setup.Common[0].BindPhrase, txBindFrame.BindPhrase_6, 6);
    // TODO Setup.Common[0].FrequencyBand = txBindFrame.FrequencyBand;
    Setup.Common[0].Mode = txBindFrame.Mode;
    Setup.Common[0].Ortho = txBindFrame.Ortho;

    if (txBindFrame.connected) {
        task = BIND_TASK_RX_STORE_PARAMS;
    }
}


void tBindBase::do_transmit(uint8_t antenna)
{
    memset((uint8_t*)&rxBindFrame, 0, sizeof(rxBindFrame));
    rxBindFrame.bind_signature = RxSignature;

    rxBindFrame.connected = connected();

    rxBindFrame.firmware_version = VERSION;
    strbufstrcpy(rxBindFrame.device_name_20, DEVICE_NAME, 20);

    rxBindFrame.crc = fmav_crc_calculate((uint8_t*)&rxBindFrame, FRAME_TX_RX_LEN - 2);
    sxSendFrame(antenna, &rxBindFrame, FRAME_TX_RX_LEN, SEND_FRAME_TMO_MS);
}


uint8_t tBindBase::do_receive(uint8_t antenna, bool do_clock_reset)
{
    sxReadFrame(antenna, &txBindFrame, &txBindFrame, FRAME_TX_RX_LEN);

    bool ok = (txBindFrame.bind_signature == TxSignature);
    if (ok) {
        uint16_t crc = fmav_crc_calculate((uint8_t*)&txBindFrame, FRAME_TX_RX_LEN - 2);
        ok = (crc == txBindFrame.crc);
    }

    if (ok && do_clock_reset) clock_reset();

    sxGetPacketStatus(antenna, &stats);

    if (ok) return RX_STATUS_VALID;

    return RX_STATUS_INVALID;
}

#endif


#endif // BIND_H
