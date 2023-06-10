//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// mLRS RX
/********************************************************

v0.0.00:
*/

#define DBG_MAIN(x)
#define DBG_MAIN_SLIM(x)
#define DEBUG_ENABLED
#define FAIL_ENABLED


// we set the priorities here to have an overview, SysTick is at 15
#define CLOCK_IRQ_PRIORITY          10
#define UARTB_IRQ_PRIORITY          11 // serial
#define UART_IRQ_PRIORITY           12 // out pin
#define UARTC_IRQ_PRIORITY          11 // debug
#define SX_DIO_EXTI_IRQ_PRIORITY    13
#define SX2_DIO_EXTI_IRQ_PRIORITY   13
#define SWUART_TIM_IRQ_PRIORITY     11 // debug on swuart
#define BUZZER_TIM_IRQ_PRIORITY     14

#include "../Common/common_conf.h"
#include "../Common/common_types.h"
#include "../Common/hal/glue.h"
#include "../modules/stm32ll-lib/src/stdstm32.h"
#include "../modules/stm32ll-lib/src/stdstm32-peripherals.h"
#include "../Common/thirdparty/stdstm32-mcu.h"
#ifdef STM32WL
#include "../modules/stm32ll-lib/src/stdstm32-subghz.h"
#endif
#include "../Common/hal/hal.h"
#include "../Common/sx-drivers/sx12xx.h"
#include "../modules/stm32ll-lib/src/stdstm32-delay.h" // these are dependent on hal
#include "../modules/stm32ll-lib/src/stdstm32-eeprom.h"
#include "../modules/stm32ll-lib/src/stdstm32-spi.h"
#ifdef DEVICE_HAS_DIVERSITY
#include "../modules/stm32ll-lib/src/stdstm32-spib.h"
#endif
#ifdef USE_SERIAL
#include "../modules/stm32ll-lib/src/stdstm32-uartb.h"
#endif
#ifdef USE_OUT
#include "../modules/stm32ll-lib/src/stdstm32-uart.h"
#endif
#ifdef USE_DEBUG
#ifdef DEVICE_HAS_DEBUG_SWUART
#include "../modules/stm32ll-lib/src/stdstm32-uart-sw.h"
#else
#include "../modules/stm32ll-lib/src/stdstm32-uartc.h"
#endif
#endif
#ifdef USE_I2C
#include "../modules/stm32ll-lib/src/stdstm32-i2c.h"
#endif
#define FASTMAVLINK_IGNORE_WADDRESSOFPACKEDMEMBER
#include "../Common/mavlink/out/mlrs_all/mlrs_all.h"
#include "../Common/setup.h"
#include "../Common/common.h"
#include "../Common/micros.h"
//#include "../Common/test.h" // un-comment if you want to compile for board test

#include "clock.h"
#include "out.h"
#include "rxstats.h"


ClockBase clock;
RxStatsBase rxstats;


// is required in bind.h
void clock_reset(void) { clock.Reset(); }


class Out : public OutBase
{
  public:
#ifdef USE_OUT
    void Init(void)
    {
        OutBase::Init(&Setup.Rx);
        out_init_gpio();
        uart_init_isroff();
    }

#if defined DEVICE_HAS_OUT || defined DEVICE_HAS_OUT_NORMAL
    bool config_crsf(bool enable_flag) override
    {
        if (enable_flag) {
            uart_setprotocol(416666, XUART_PARITY_NO, UART_STOPBIT_1);
            out_set_normal();
        }
        return true;
    }

    bool config_sbus_inverted(bool enable_flag) override
    {
        if (enable_flag) {
            uart_setprotocol(100000, XUART_PARITY_EVEN, UART_STOPBIT_2);
            out_set_normal();
        }
        return true;
    }
#endif

#if defined DEVICE_HAS_OUT || defined DEVICE_HAS_OUT_INVERTED
    bool config_sbus(bool enable_flag) override
    {
        if (enable_flag) {
            uart_setprotocol(100000, XUART_PARITY_EVEN, UART_STOPBIT_2);
            out_set_inverted();
        }
        return true;
    }
#endif

    void putc(char c) override { uart_putc(c); }

    void SendLinkStatistics(void)
    {
        tOutLinkStats lstats = {
          .receiver_rssi1 = stats.last_rssi1,
          .receiver_rssi2 = stats.last_rssi2,
          .receiver_LQ = rxstats.GetLQ(),
          .receiver_snr = stats.GetLastSnr(),
          .receiver_antenna = stats.last_antenna,
          .receiver_transmit_antenna = stats.last_transmit_antenna,
          .receiver_power_dbm = sx.RfPower_dbm(),
          .transmitter_rssi = stats.received_rssi,
          .transmitter_LQ = stats.received_LQ,
          .transmitter_snr = 0,
          .transmitter_antenna = stats.received_antenna,
          .transmitter_transmit_antenna = stats.received_transmit_antenna,
          .mode = (uint8_t)Config.Mode,
        };
        if (USE_ANTENNA1 && USE_ANTENNA2) {
            lstats.antenna_config = 3;
        } else if (USE_ANTENNA2) {
            lstats.antenna_config = 2;
        } else {
            lstats.antenna_config = 1;
        }
        OutBase::SendLinkStatistics(&lstats);
    }
#else
    void Init(void) {}
    void SendLinkStatistics(void) {}
#endif
};

Out out;


//-------------------------------------------------------
// Mavlink
//-------------------------------------------------------

#include "mavlink_interface_rx.h"

MavlinkBase mavlink;

#include "sx_serial_interface_rx.h"

tRxSxSerial sx_serial;


//-------------------------------------------------------
// Init
//-------------------------------------------------------

void init(void)
{
    systembootloader_init();
    leds_init();
    button_init();

    delay_init();
    micros_init();

    systembootloader_do(); // after delay_init() since it may need delay

    serial.Init();
    out.Init();

    buzzer.Init();
    fan.Init();
    dbg.Init();

    setup_init();

    clock.Init(Config.frame_rate_ms); // clock needs Config, so call after setup_init()

    sx.Init(); // sx needs Config, so call after setup_init()
    sx2.Init();
}


//-------------------------------------------------------
// SX12xx
//-------------------------------------------------------

volatile uint16_t irq_status;
volatile uint16_t irq2_status;

IRQHANDLER(
void SX_DIO_EXTI_IRQHandler(void)
{
    sx_dio_exti_isr_clearflag();
    irq_status = sx.GetAndClearIrqStatus(SX12xx_IRQ_ALL);
    if (irq_status & SX12xx_IRQ_RX_DONE) {
        if (bind.IsInBind()) {
            uint64_t bind_signature;
            sx.ReadBuffer(0, (uint8_t*)&bind_signature, 8);
            if (bind_signature != bind.TxSignature) irq_status = 0; // not binding frame, so ignore it
        } else {
            uint16_t sync_word;
            sx.ReadBuffer(0, (uint8_t*)&sync_word, 2); // rxStartBufferPointer is always 0, so no need for sx.GetRxBufferStatus()
            if (sync_word != Config.FrameSyncWord) irq_status = 0; // not for us, so ignore it
        }
    }
})
#ifdef DEVICE_HAS_DIVERSITY
IRQHANDLER(
void SX2_DIO_EXTI_IRQHandler(void)
{
    sx2_dio_exti_isr_clearflag();
    irq2_status = sx2.GetAndClearIrqStatus(SX12xx_IRQ_ALL);
    if (irq2_status & SX12xx_IRQ_RX_DONE) {
        if (bind.IsInBind()) {
            uint64_t bind_signature;
            sx2.ReadBuffer(0, (uint8_t*)&bind_signature, 8);
            if (bind_signature != bind.TxSignature) irq2_status = 0;
        } else {
            uint16_t sync_word;
            sx2.ReadBuffer(0, (uint8_t*)&sync_word, 2);
            if (sync_word != Config.FrameSyncWord) irq2_status = 0;
        }
    }
})
#endif


uint8_t link_rx1_status;
uint8_t link_rx2_status;


//-- Tx/Rx cmd frame handling

uint8_t link_task;
uint8_t transmit_frame_type;
bool doParamsStore;


void link_task_init(void)
{
    link_task = LINK_TASK_NONE;
    transmit_frame_type = TRANSMIT_FRAME_TYPE_NORMAL;

    doParamsStore = false;
}


void link_task_set(uint8_t task)
{
    link_task = task;
    transmit_frame_type = TRANSMIT_FRAME_TYPE_CMD;
}


// we clear then we receive a non-cmd frame, as this indicates that the tx has gotten the response
// we also should clear then disconnected
void link_task_reset(void)
{
    link_task = LINK_TASK_NONE;
    transmit_frame_type = TRANSMIT_FRAME_TYPE_NORMAL;
}


void process_received_txcmdframe(tTxFrame* frame)
{
tCmdFrameHeader* head = (tCmdFrameHeader*)(frame->payload);

    switch (head->cmd) {
    case FRAME_CMD_GET_RX_SETUPDATA:
        // request to send setup data, trigger sending RX_SETUPDATA in next transmission
        link_task_set(LINK_TASK_RX_SEND_RX_SETUPDATA);
        break;
    case FRAME_CMD_SET_RX_PARAMS:
        // received rx params, trigger sending RX_SETUPDATA in next transmission
        unpack_txcmdframe_setrxparams(frame);
        link_task_set(LINK_TASK_RX_SEND_RX_SETUPDATA);
        break;
    case FRAME_CMD_STORE_RX_PARAMS:
        // got request to store rx params
        doParamsStore = true;
        break;
    case FRAME_CMD_GET_RX_SETUPDATA_WRELOAD:
        setup_reload();
        // request to send setup data, trigger sending RX_SETUPDATA in next transmission
        link_task_set(LINK_TASK_RX_SEND_RX_SETUPDATA);
        break;
    }
}


void pack_rxcmdframe(tRxFrame* frame, tFrameStats* frame_stats)
{
    switch (link_task) {
    case LINK_TASK_RX_SEND_RX_SETUPDATA:
        // send rx setup data
        pack_rxcmdframe_rxsetupdata(frame, frame_stats);
        break;
    }
}


//-- normal Tx, Rx frames handling

void prepare_transmit_frame(uint8_t antenna, uint8_t ack)
{
uint8_t payload[FRAME_RX_PAYLOAD_LEN];
uint8_t payload_len = 0;

    if (transmit_frame_type == TRANSMIT_FRAME_TYPE_NORMAL) {

        // read data from serial
        if (connected()) {
            for (uint8_t i = 0; i < FRAME_RX_PAYLOAD_LEN; i++) {
                if (!sx_serial.available()) break;
                payload[payload_len] = sx_serial.getc();
//dbg.putc(payload[payload_len]);
                payload_len++;
            }

            stats.bytes_transmitted.Add(payload_len);
            stats.serial_data_transmitted.Inc();
        } else {
            sx_serial.flush();
        }
    }

    stats.last_transmit_antenna = antenna;

    tFrameStats frame_stats;
    frame_stats.seq_no = stats.transmit_seq_no;
    frame_stats.ack = ack;
    frame_stats.antenna = stats.last_antenna;
    frame_stats.transmit_antenna = antenna;
    frame_stats.rssi = stats.GetLastRssi();
    frame_stats.LQ = rxstats.GetLQ();
    frame_stats.LQ_serial_data = rxstats.GetLQ_serial_data();

    if (transmit_frame_type == TRANSMIT_FRAME_TYPE_NORMAL) {
        pack_rxframe(&rxFrame, &frame_stats, payload, payload_len);
    } else {
        pack_rxcmdframe(&rxFrame, &frame_stats);
    }
}


void process_received_frame(bool do_payload, tTxFrame* frame)
{
    stats.received_antenna = frame->status.antenna;
    stats.received_transmit_antenna = frame->status.transmit_antenna;
    stats.received_rssi = rssi_i8_from_u7(frame->status.rssi_u7);
    stats.received_LQ = frame->status.LQ;
    stats.received_LQ_serial_data = frame->status.LQ_serial_data;

    // copy rc data
    if (!do_payload) {
        // copy only channels 1-4,12,13 and jump out
        rcdata_rc1_from_txframe(&rcData, frame);
        return;
    }

    rcdata_from_txframe(&rcData, frame);

    // handle cmd frame
    if (frame->status.frame_type == FRAME_TYPE_TX_RX_CMD) {
        process_received_txcmdframe(frame);
        return;
    }

    link_task_reset(); // clear it if non-cmd frame is received

    // output data on serial, but only if connected
    if (connected()) {
        for (uint8_t i = 0; i < frame->status.payload_len; i++) {
            uint8_t c = frame->payload[i];
            sx_serial.putc(c);
//dbg.putc(c);
        }

        stats.bytes_received.Add(frame->status.payload_len);
        stats.serial_data_received.Inc();
    }
}


//-- receive/transmit handling api

void handle_receive(uint8_t antenna)
{
uint8_t rx_status;
tTxFrame* frame;

    if (antenna == ANTENNA_1) {
        rx_status = link_rx1_status;
        frame = &txFrame;
    } else {
        rx_status = link_rx2_status;
        frame = &txFrame2;
    }

    if (bind.IsInBind()) {
        bind.handle_receive(antenna, rx_status);
        return;
    }

    if (rx_status != RX_STATUS_INVALID) { // RX_STATUS_CRC1_VALID, RX_STATUS_VALID

        bool do_payload = (rx_status == RX_STATUS_VALID);

        process_received_frame(do_payload, frame);

        rxstats.doValidCrc1FrameReceived();
        if (rx_status == RX_STATUS_VALID) rxstats.doValidFrameReceived(); // should we count valid payload only if tx frame ?

        stats.received_seq_no = frame->status.seq_no;
        stats.received_ack = frame->status.ack;

    } else { // RX_STATUS_INVALID
        stats.received_seq_no = UINT8_MAX;
        stats.received_ack = 0;
    }

    // we set it for all received frames
    stats.last_antenna = antenna;

    // we count all received frames
    rxstats.doFrameReceived();
}


void handle_receive_none(void) // RX_STATUS_NONE
{
    stats.received_seq_no = UINT8_MAX;
    stats.received_ack = 0;
}


void do_transmit(uint8_t antenna) // we send a frame to transmitter
{
uint8_t ack = 1;

    if (bind.IsInBind()) {
        bind.do_transmit(antenna);
        return;
    }

    stats.transmit_seq_no++;

    prepare_transmit_frame(antenna, ack);

    // to test asymmetric connection, fake rxFrame, to no send doesn't work as it blocks the sx
    sxSendFrame(antenna, &rxFrame, FRAME_TX_RX_LEN, SEND_FRAME_TMO); // 10ms tmo
}


uint8_t do_receive(uint8_t antenna, bool do_clock_reset) // we receive a frame from receiver
{
uint8_t res;
uint8_t rx_status = RX_STATUS_INVALID; // this also signals that a frame was received

    if (bind.IsInBind()) {
        return bind.do_receive(antenna, do_clock_reset);
    }

    // we don't need to read sx.GetRxBufferStatus(), but hey
    // we could save 2 byte's time by not reading sync_word again, but hey
    sxReadFrame(antenna, &txFrame, &txFrame2, FRAME_TX_RX_LEN);
    res = (antenna == ANTENNA_1) ? check_txframe(&txFrame) : check_txframe(&txFrame2);

    if (res) {
        DBG_MAIN(dbg.puts("fail ");dbg.putc('\n');)
dbg.puts("fail a");dbg.putc(antenna+'0');dbg.puts(" ");dbg.puts(u8toHEX_s(res));dbg.putc('\n');
    }

    // must not happen !
    // it can happen though, I've observed it on R9, maybe if in the ca 1 ms after receive the sx starts receiving something?
    if (res == CHECK_ERROR_SYNCWORD) { FAIL("do_receive() CHECK_ERROR_SYNCWORD"); return RX_STATUS_INVALID; }

    if (res == CHECK_OK || res == CHECK_ERROR_CRC) {

        if (do_clock_reset) clock.Reset();

        rx_status = (res == CHECK_OK) ? RX_STATUS_VALID : RX_STATUS_CRC1_VALID;
    }

    // we want to have the rssi,snr stats even if it's a bad packet
    sxGetPacketStatus(antenna, &stats);

    return rx_status;
}


//##############################################################################################################
//*******************************************************
// MAIN routine
//*******************************************************

uint16_t led_blink;
uint16_t tick_1hz;
uint16_t tick_1hz_commensurate;

uint8_t link_state;
uint8_t connect_state;
uint16_t connect_tmo_cnt;
uint8_t connect_sync_cnt;
uint8_t connect_listen_cnt;
bool connect_occured_once;

uint8_t doPostReceive2_cnt;
bool doPostReceive2;
bool frame_missed;


static inline bool connected(void)
{
  return (connect_state == CONNECT_STATE_CONNECTED);
}


int main_main(void)
{
#ifdef BOARD_TEST_H
  main_test();
#endif
RESTARTCONTROLLER:
  init();
  DBG_MAIN(dbg.puts("\n\n\nHello\n\n");)

  serial.SetBaudRate(Config.SerialBaudrate);

  // startup sign of life
  LED_RED_OFF;
  for (uint8_t i = 0; i < 7; i++) { LED_RED_TOGGLE; delay_ms(50); }

  // start up sx
  if (!sx.isOk()) { FAILALWAYS(GR_OFF_RD_BLINK, "Sx not ok"); } // fail!
  if (!sx2.isOk()) { FAILALWAYS(RD_OFF_GR_BLINK, "Sx2 not ok"); } // fail!
  irq_status = irq2_status = 0;
  IF_ANTENNA1(sx.StartUp());
  IF_ANTENNA2(sx2.StartUp());
  bind.Init();
  fhss.Init(Config.FhssNum, Config.FhssSeed, Setup.FrequencyBand);
  fhss.Start();

  sx.SetRfFrequency(fhss.GetCurrFreq());
  sx2.SetRfFrequency(fhss.GetCurrFreq());

  link_state = LINK_STATE_RECEIVE;
  connect_state = CONNECT_STATE_LISTEN;
  connect_tmo_cnt = 0;
  connect_listen_cnt = 0;
  connect_sync_cnt = 0;
  connect_occured_once = false;
  link_rx1_status = RX_STATUS_NONE;
  link_rx2_status = RX_STATUS_NONE;
  link_task_init();
  doPostReceive2_cnt = 0;
  doPostReceive2 = false;
  frame_missed = false;

  rxstats.Init(Config.LQAveragingPeriod);

  out.Configure(Setup.Rx.OutMode);
  mavlink.Init();
  sx_serial.Init();
  fan.SetPower(sx.RfPower_dbm());

  led_blink = 0;
  tick_1hz = 0;
  tick_1hz_commensurate = 0;
  doSysTask = 0; // helps in avoiding too short first loop
  while (1) {

    //-- SysTask handling

    if (doSysTask) {
        doSysTask = 0;

        if (connect_tmo_cnt) {
            connect_tmo_cnt--;
        }

        if (connected()) {
            DECc(led_blink, SYSTICK_DELAY_MS(500));
        } else {
            DECc(led_blink, SYSTICK_DELAY_MS(200));
        }

        if (bind.IsInBind()) {
            if (!led_blink) { LED_GREEN_TOGGLE; LED_RED_TOGGLE; }
        } else
        if (connected()) {
            if (!led_blink) LED_GREEN_TOGGLE;
            LED_RED_OFF;
        } else {
            LED_GREEN_OFF;
            if (!led_blink) LED_RED_TOGGLE;
        }

        DECc(tick_1hz, SYSTICK_DELAY_MS(1000));

        if (!connect_occured_once) bind.AutoBind();

        if (!tick_1hz) {
            dbg.puts(".");
/*            dbg.puts("\nRX: ");
            dbg.puts(u8toBCD_s(rxstats.GetLQ())); dbg.putc(',');
            dbg.puts(u8toBCD_s(rxstats.GetLQ_serial_data()));
            dbg.puts(" (");
            dbg.puts(u8toBCD_s(stats.frames_received.GetLQ())); dbg.putc(',');
            dbg.puts(u8toBCD_s(stats.valid_crc1_received.GetLQ())); dbg.putc(',');
            dbg.puts(u8toBCD_s(stats.valid_frames_received.GetLQ()));
            dbg.puts("),");
            dbg.puts(u8toBCD_s(stats.received_LQ)); dbg.puts(", ");

            dbg.puts(s8toBCD_s(stats.last_rssi1)); dbg.putc(',');
            dbg.puts(s8toBCD_s(stats.received_rssi)); dbg.puts(", ");
            dbg.puts(s8toBCD_s(stats.last_snr1)); dbg.puts("; ");

            dbg.puts(u16toBCD_s(stats.bytes_transmitted.GetBytesPerSec())); dbg.puts(", ");
            dbg.puts(u16toBCD_s(stats.bytes_received.GetBytesPerSec())); dbg.puts("; "); */
        }
    }

    //-- SX handling

    switch (link_state) {
    case LINK_STATE_RECEIVE: {
        if (connect_state >= CONNECT_STATE_SYNC) { // we hop only if not in listen
            fhss.HopToNext();
        }
        sx.SetRfFrequency(fhss.GetCurrFreq());
        sx2.SetRfFrequency(fhss.GetCurrFreq());
        IF_ANTENNA1(sx.SetToRx(0)); // single without tmo
        IF_ANTENNA2(sx2.SetToRx(0));
        link_state = LINK_STATE_RECEIVE_WAIT;
        link_rx1_status = RX_STATUS_NONE;
        link_rx2_status = RX_STATUS_NONE;
        irq_status = 0;
        irq2_status = 0;
        DBG_MAIN_SLIM(dbg.puts("\n>");)
        }break;

    case LINK_STATE_TRANSMIT: {
        // TODO: transmit antenna diversity
        do_transmit((USE_ANTENNA1) ? ANTENNA_1 : ANTENNA_2);
        link_state = LINK_STATE_TRANSMIT_WAIT;
        irq_status = 0; // important, in low connection condition, RxDone isr could trigger
        irq2_status = 0;
        }break;
    }//end of switch(link_state)

IF_ANTENNA1(
    if (irq_status) {
        if (link_state == LINK_STATE_TRANSMIT_WAIT) {
            if (irq_status & SX12xx_IRQ_TX_DONE) {
                irq_status = 0;
                link_state = LINK_STATE_RECEIVE;
                DBG_MAIN_SLIM(dbg.puts("<");)
            }
        } else
        if (link_state == LINK_STATE_RECEIVE_WAIT) {
            if (irq_status & SX12xx_IRQ_RX_DONE) {
                irq_status = 0;
                bool do_clock_reset = (link_rx2_status == RX_STATUS_NONE);
                link_rx1_status = do_receive(ANTENNA_1, do_clock_reset);
                if (link_rx1_status == RX_STATUS_VALID) sx.HandleAFC();
                DBG_MAIN_SLIM(dbg.puts("!");)
            }
        }

        if (irq_status & SX12xx_IRQ_RX_DONE) { // R, T, TW
            FAILALWAYS(GR_OFF_RD_BLINK, "IRQ RX DONE FAIL");
        }
        if (irq_status & SX12xx_IRQ_TX_DONE) {
            FAILALWAYS(RD_OFF_GR_BLINK, "IRQ TX DONE FAIL");
        }
        if (irq_status & SX12xx_IRQ_TIMEOUT) {
            FAILALWAYS(BLINK_COMMON, "IRQ TMO FAIL");
        }
    }//end of if(irq_status)
);
IF_ANTENNA2(
    if (irq2_status) {
        if (link_state == LINK_STATE_TRANSMIT_WAIT) {
            if (irq2_status & SX12xx_IRQ_TX_DONE) {
                irq2_status = 0;
                link_state = LINK_STATE_RECEIVE;
            }
        } else
        if (link_state == LINK_STATE_RECEIVE_WAIT) {
            if (irq2_status & SX12xx_IRQ_RX_DONE) {
                irq2_status = 0;
                bool do_clock_reset = (link_rx1_status == RX_STATUS_NONE);
                link_rx2_status = do_receive(ANTENNA_2, do_clock_reset);
                if (link_rx2_status == RX_STATUS_VALID) sx2.HandleAFC();
            }
        }

        if (irq2_status & SX12xx_IRQ_RX_DONE) { // R, T, TW
            FAILALWAYS(GR_ON_RD_BLINK, "IRQ2 RX DONE FAIL");
        }
        if (irq2_status & SX12xx_IRQ_TX_DONE) {
            FAILALWAYS(RD_ON_GR_BLINK, "IRQ2 TX DONE FAIL");
        }
        if (irq2_status & SX12xx_IRQ_TIMEOUT) {
            FAILALWAYS(BLINK_ALTERNATE, "IRQ2 TMO FAIL");
        }
    }//end of if(irq2_status)
);

    // this happens ca 1 ms after a frame was or should have been received
    if (doPostReceive) {
        doPostReceive = false;

        bool frame_received, valid_frame_received, invalid_frame_received;
        frame_received = valid_frame_received = invalid_frame_received = false; // to make compiler happy
        if (USE_ANTENNA1 && USE_ANTENNA2) {
            frame_received = (link_rx1_status > RX_STATUS_NONE) || (link_rx2_status > RX_STATUS_NONE);
            valid_frame_received = (link_rx1_status > RX_STATUS_INVALID) || (link_rx2_status > RX_STATUS_INVALID);
            invalid_frame_received = frame_received && !valid_frame_received;
        } else if (USE_ANTENNA1) {
            frame_received = (link_rx1_status > RX_STATUS_NONE);
            valid_frame_received = (link_rx1_status > RX_STATUS_INVALID);
            invalid_frame_received = (link_rx1_status == RX_STATUS_INVALID); // frame_received && !valid_frame_received;
        } else if (USE_ANTENNA2) {
            frame_received = (link_rx2_status > RX_STATUS_NONE);
            valid_frame_received = (link_rx2_status > RX_STATUS_INVALID);
            invalid_frame_received = (link_rx2_status == RX_STATUS_INVALID);
        }

/*dbg.puts("\n> 1: ");
dbg.puts(s8toBCD_s(stats.last_rssi1));
dbg.puts(" 2: ");
dbg.puts(s8toBCD_s(stats.last_rssi2));*/

        if (frame_received) { // frame received
            uint8_t antenna = ANTENNA_1;

            if (USE_ANTENNA1 && USE_ANTENNA2) {
                // work out which antenna we choose
                //            |   NONE   |  INVALID  | CRC1_VALID | VALID
                // --------------------------------------------------------
                // NONE       |          |   1 or 2  |     1      |  1
                // INVALID    |  1 or 2  |   1 or 2  |     1      |  1
                // CRC1_VALID |    2     |     2     |   1 or 2   |  1
                // VALID      |    2     |     2     |     2      |  1 or 2

                if (link_rx1_status == link_rx2_status) {
                    // we can choose either antenna, so select the one with the better rssi
                    antenna = (stats.last_rssi1 > stats.last_rssi2) ? ANTENNA_1 : ANTENNA_2;
                } else
                if (link_rx1_status == RX_STATUS_VALID) {
                    antenna = ANTENNA_1;
                } else
                if (link_rx2_status == RX_STATUS_VALID) {
                    antenna = ANTENNA_2;
                } else
                if (link_rx1_status == RX_STATUS_CRC1_VALID) {
                    antenna = ANTENNA_1;
                } else
                if (link_rx2_status == RX_STATUS_CRC1_VALID) {
                    antenna = ANTENNA_2;
                } else {
                    // we can choose either antenna, so select the one with the better rssi
                    antenna = (stats.last_rssi1 > stats.last_rssi2) ? ANTENNA_1 : ANTENNA_2;
                }
            } else if (USE_ANTENNA2) {
                antenna = ANTENNA_2;
            }

            handle_receive(antenna);
//dbg.puts(" a "); dbg.puts((antenna == ANTENNA_1) ? "1 " : "2 ");
        } else {
            handle_receive_none();
        }

        if (valid_frame_received) { // valid frame received
            switch (connect_state) {
            case CONNECT_STATE_LISTEN:
                connect_state = CONNECT_STATE_SYNC;
                connect_sync_cnt = 0;
                break;
            case CONNECT_STATE_SYNC:
                connect_sync_cnt++;
                if (connect_sync_cnt >= CONNECT_SYNC_CNT) {
                    connect_state = CONNECT_STATE_CONNECTED;
                    connect_occured_once = true;
                }
                break;
            }
            connect_tmo_cnt = CONNECT_TMO_SYSTICKS;

            link_state = LINK_STATE_TRANSMIT; // switch to TX
        }

        // when in listen: we received something, but something wrong, so we need go back to RX
        if ((connect_state == CONNECT_STATE_LISTEN) && invalid_frame_received) {
            link_state = LINK_STATE_RECEIVE;
        }

        // when in listen, slowly loop through frequencies
        if (connect_state == CONNECT_STATE_LISTEN) {
            connect_listen_cnt++;
            if (connect_listen_cnt >= CONNECT_LISTEN_HOP_CNT) {
                fhss.HopToNext();
                connect_listen_cnt = 0;
                link_state = LINK_STATE_RECEIVE; // switch back to RX
            }
            if (fhss.HopToNextBind()) { link_state = LINK_STATE_RECEIVE; } // switch back to RX
        }

        // we just disconnected, or are in sync but don't receive anything
        if ((connect_state >= CONNECT_STATE_SYNC) && !connect_tmo_cnt) {
            // switch to listen state
            // only do it if not in listen, since otherwise it never could reach receive wait and hence never could connect
            connect_state = CONNECT_STATE_LISTEN;
            connect_listen_cnt = 0;
            link_state = LINK_STATE_RECEIVE; // switch back to RX
        }

        // we didn't receive a valid frame
        frame_missed = false;
        if ((connect_state >= CONNECT_STATE_SYNC) && !valid_frame_received) {
            frame_missed = true;
            // reset sync counter, relevant if in sync
            // connect_sync_cnt = 0; // NO!! when in sync this means that we need to get five in a row, right!?!
            // switch to transmit state
            // only do it if receiving, else keep it in RX mode, otherwise chances to connect are dim
            // we are on the correct frequency, so no need to hop
            link_state = LINK_STATE_TRANSMIT;
        }

#ifdef DEVICE_HAS_SX127x
        if ((connect_state >= CONNECT_STATE_SYNC) || (link_state == LINK_STATE_RECEIVE)) {
#else
        if (connect_state >= CONNECT_STATE_SYNC) {
#endif
            sx.SetToIdle();
            sx2.SetToIdle();
        }

        DECc(tick_1hz_commensurate, Config.frame_rate_hz);
        if (!tick_1hz_commensurate) {
            rxstats.Update1Hz();
        }
        rxstats.Next();
        if (!connected()) rxstats.Clear();

        if (connect_state == CONNECT_STATE_LISTEN) link_task_reset();

        if (Setup.Rx.Buzzer == BUZZER_LOST_PACKETS && connect_occured_once && !bind.IsInBind()) {
            if (!valid_frame_received) buzzer.BeepLP();
        }

        bind.Do();
        switch (bind.Task()) {
        case BIND_TASK_CHANGED_TO_BIND:
            bind.ConfigForBind();
            CLOCK_PERIOD_10US = ((uint16_t)Config.frame_rate_ms * 100);
            clock.Reset();
            fhss.SetToBind(Config.frame_rate_ms);
            LED_GREEN_ON;
            LED_RED_OFF;
            connect_state = CONNECT_STATE_LISTEN;
            link_state = LINK_STATE_RECEIVE;
            break;
        case BIND_TASK_RX_STORE_PARAMS:
            Setup.FrequencyBand = fhss.GetCurrFrequencyBand();
            doParamsStore = true;
            break;
        }

        doPostReceive2_cnt = 5; // allow link_state changes to be handled, so postpone this few loops
    }//end of if(doPostReceive)

    //-- Update channels, Out handling, etc

    if (doPostReceive2_cnt) {
        doPostReceive2_cnt--;
        if (!doPostReceive2_cnt) doPostReceive2 = true;
    }
    if (doPostReceive2) {
        doPostReceive2 = false;

        out.SetChannelOrder(Setup.Rx.ChannelOrder);
        if (connected()) {
            out.SendRcData(&rcData, frame_missed, false, stats.GetLastRssi());
            out.SendLinkStatistics();
            mavlink.SendRcData(out.GetRcDataPtr(), false);
        } else {
            if (connect_occured_once) {
                // generally output a signal only if we had a connection at least once
                out.SendRcData(&rcData, true, true, RSSI_MIN);
                out.SendLinkStatisticsDisconnected();
                mavlink.SendRcData(out.GetRcDataPtr(), true);
            }
        }
    }//end of if(doPostReceive2)

    out.Do(micros());

    //-- Do mavlink

    mavlink.Do();

    //-- Store parameters

    if (doParamsStore) {
        sx.SetToIdle();
        sx2.SetToIdle();
        LED_RED_ON; LED_GREEN_ON;
        setup_store_to_EEPROM();
        goto RESTARTCONTROLLER;
    }

  }//end of while(1) loop

}//end of main

