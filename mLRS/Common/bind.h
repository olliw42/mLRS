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


uint32_t millis32(void);
static inline bool connected(void);


//-------------------------------------------------------
// Bind Class
//-------------------------------------------------------

#define BIND_SIGNATURE_TX_STR  "mLRS\x01\x02\x03\x04"
#define BIND_SIGNATURE_RX_STR  "mLRS\x04\x03\x02\x01"
#define BIND_BUTTON_TMO        2000


typedef enum {
    BIND_TASK_NONE = 0,
    BIND_TASK_CHANGED_TO_BIND,
    BIND_TASK_RX_STORE_PARAMS,
    BIND_TASK_TX_RESTART_CONTROLLER,
} BIND_TASK_ENUM;


class BindBase
{
  public:
    void Init(void);
    bool IsInBind(void) { return is_in_binding; }
    void SetToBind(void) { binding_requested = true; }
    void ConfigForBind(void);
    void Do(void);
    uint8_t Task(void);

    bool is_in_binding;
    bool binding_requested;
    uint32_t button_tlast_ms;
    uint8_t task;
    bool is_connected;

    uint64_t TxSignature; // 8 bytes
    uint64_t RxSignature; // 8 bytes

    void handle_receive(uint8_t antenna, uint8_t rx_status);
    void do_transmit(uint8_t antenna);
    uint8_t do_receive(uint8_t antenna, bool do_clock_reset);
};


void BindBase::Init(void)
{
    is_in_binding = false;
    binding_requested = false;
    task = BIND_TASK_NONE;
    is_connected = false;

    button_init();
    button_tlast_ms = millis32();

    memcpy(&TxSignature, BIND_SIGNATURE_TX_STR, 8);
    memcpy(&RxSignature, BIND_SIGNATURE_RX_STR, 8);
}


void BindBase::ConfigForBind(void)
{
    // switch to 19 Mode, select lowest possible power
    configure_mode(MODE_19HZ);

    sx.SetToIdle();
    sx2.SetToIdle();
    sx.SetRfPower_dbm(rfpower_list[0].dbm);
    sx2.SetRfPower_dbm(rfpower_list[0].dbm);
    sx.SetLoraConfigurationByIndex(Config.LoraConfigIndex);
    sx2.SetLoraConfigurationByIndex(Config.LoraConfigIndex);
    sx.ClearIrqStatus(SX1280_IRQ_ALL);
    sx2.ClearIrqStatus(SX1280_IRQ_ALL);
    sx.SetToIdle();
    sx2.SetToIdle();
}


void BindBase::Do(void)
{
    uint32_t tnow = millis32();

    if (button_pressed()) {
        if (tnow - button_tlast_ms > BIND_BUTTON_TMO) {
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
    }
#endif

   if (!is_in_binding && binding_requested) {
       is_in_binding = true;
       task = BIND_TASK_CHANGED_TO_BIND;
   }
}


uint8_t BindBase::Task(void)
{
    uint8_t ret = task;
    task = BIND_TASK_NONE;
    return ret;
}



tTxBindFrame txBindFrame;
tRxBindFrame rxBindFrame;

#ifdef DEVICE_IS_TRANSMITTER

void BindBase::handle_receive(uint8_t antenna, uint8_t rx_status)
{
dbg.puts("\ntx HANDLE REC");

    if (rx_status == RX_STATUS_INVALID) return;

    // do stuff
dbg.puts(" DO");

}


void BindBase::do_transmit(uint8_t antenna)
{
//dbg.puts("\ntx do trns");
    memset(&txBindFrame, 0, sizeof(txBindFrame));
    txBindFrame.bind_signature = TxSignature;

    txBindFrame.connected = connected();

    strncpy_x(txBindFrame.BindPhrase, Setup.BindPhrase, 6);
    txBindFrame.FrequencyBand = Setup.FrequencyBand;
    txBindFrame.Mode = Setup.Mode;

    txBindFrame.crc = fmav_crc_calculate((uint8_t*)&txBindFrame, FRAME_TX_RX_LEN - 2);
    sxSendFrame(antenna, &txBindFrame, &txBindFrame, FRAME_TX_RX_LEN, SEND_FRAME_TMO);
}


uint8_t BindBase::do_receive(uint8_t antenna, bool do_clock_reset)
{
dbg.puts("\ntx do rec");

    sxReadFrame(antenna, &rxBindFrame, &rxBindFrame, FRAME_TX_RX_LEN);
    bool ok = (rxBindFrame.bind_signature == RxSignature);
    if (ok) {
        uint16_t crc = fmav_crc_calculate((uint8_t*)&rxBindFrame, FRAME_TX_RX_LEN - 2);
        ok = (crc == rxBindFrame.crc);
    }
    sxGetPacketStatus(antenna, &stats);
    return (ok) ? RX_STATUS_VALID : RX_STATUS_INVALID;
}

#endif
#ifdef DEVICE_IS_RECEIVER


void BindBase::handle_receive(uint8_t antenna, uint8_t rx_status)
{
dbg.puts("\nrx HANLDE REC");

    if (rx_status == RX_STATUS_INVALID) return;

    // do stuff
dbg.puts(" DO");

    strncpy_x(Setup.BindPhrase, txBindFrame.BindPhrase, 6);
    Setup.FrequencyBand = txBindFrame.FrequencyBand;
    Setup.Mode = txBindFrame.Mode;

    if (txBindFrame.connected) {
        task = BIND_TASK_RX_STORE_PARAMS;
    }
}


void BindBase::do_transmit(uint8_t antenna)
{
dbg.puts("\nrx do trns");
    memset(&rxBindFrame, 0, sizeof(rxBindFrame));
    rxBindFrame.bind_signature = RxSignature;

    rxBindFrame.connected = connected();

    rxBindFrame.firmware_version = VERSION;
    strncpy_x(rxBindFrame.device_name, DEVICE_NAME, 20);

    rxBindFrame.crc = fmav_crc_calculate((uint8_t*)&rxBindFrame, FRAME_TX_RX_LEN - 2);
    sxSendFrame(antenna, &rxBindFrame, &rxBindFrame, FRAME_TX_RX_LEN, SEND_FRAME_TMO);
}


void clock_reset(void);


uint8_t BindBase::do_receive(uint8_t antenna, bool do_clock_reset)
{
dbg.puts("\nrx do rec");

    sxReadFrame(antenna, &txBindFrame, &txBindFrame, FRAME_TX_RX_LEN);
    bool ok = (txBindFrame.bind_signature == TxSignature);
    if (ok) {
        uint16_t crc = fmav_crc_calculate((uint8_t*)&txBindFrame, FRAME_TX_RX_LEN - 2);
        ok = (crc == txBindFrame.crc);
    }

    if (ok && do_clock_reset) clock_reset();

    sxGetPacketStatus(antenna, &stats);
    return (ok) ? RX_STATUS_VALID : RX_STATUS_INVALID;
}

#endif


#endif // BIND_H
