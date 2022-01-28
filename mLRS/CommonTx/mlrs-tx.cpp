//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// mLRS TX
/********************************************************

v0.0.00:
*/

#define DBG_MAIN(x)
#define DBG_MAIN_SLIM(x)


// we set the priorities here to have an overview
#define SX_DIO1_EXTI_IRQ_PRIORITY   11
#define UART_IRQ_PRIORITY           10 // mbridge, this needs to be high, when lower than DIO1, the module could stop sending via the bridge
#define UARTB_IRQ_PRIORITY          14 // serial
#define UARTC_IRQ_PRIORITY          14

#include "..\Common\common_conf.h"
#include "..\Common\common_types.h"
#include "..\Common\hal\glue.h"
#include "..\modules\stm32ll-lib\src\stdstm32.h"
#include "..\modules\stm32ll-lib\src\stdstm32-peripherals.h"
#include "..\Common\hal\hal.h"
#include "..\modules\stm32ll-lib\src\stdstm32-delay.h"
#include "..\modules\stm32ll-lib\src\stdstm32-spi.h"
#include "..\modules\sx12xx-lib\src\sx128x.h"
#include "..\modules\stm32ll-lib\src\stdstm32-uartb.h"
#include "..\modules\stm32ll-lib\src\stdstm32-uartc.h"
#include "..\Common\fhss.h"
#define FASTMAVLINK_IGNORE_WADDRESSOFPACKEDMEMBER
#include "..\Common\mavlink\out\mlrs\mlrs.h"
#include "..\Common\common.h"
//#include "..\Common\test.h" // un-comment if you want to compile for board test

#ifdef DEVICE_HAS_IN
#include "..\modules\stm32ll-lib\src\stdstm32-uarte.h"
#endif
#include "in.h"
#include "txstats.h"


#define CLOCK_TIMx                  TIM3

void clock_init(void)
{
  tim_init_1us_freerunning(CLOCK_TIMx);
}

uint16_t micros(void)
{
  return CLOCK_TIMx->CNT;
}


class In : public InBase
{
#ifdef DEVICE_HAS_IN
public:
  void Init(void)
  {
    InBase::Init();
    in_init_gpio();
    uarte_init_isroff();
  }

  void config_sbus(void) override
  {
    uarte_setprotocol(100000, XUART_PARITY_EVEN, UART_STOPBIT_2);
    in_set_inverted();
    gpio_init_af(UARTE_RX_IO, IO_MODE_INPUT_PD, UARTE_IO_AF, IO_SPEED_VERYFAST);
    uarte_rx_enableisr(ENABLE);
  }

  bool available(void) override { return uarte_rx_available(); }
  char getc(void) override { return uarte_getc(); }
  uint16_t tim_1us(void) override { return micros(); }
#endif
};

In in;


class ChannelOrder
{
public:
  ChannelOrder(void)
  {
    channel_order = UINT8_MAX;
    for (uint8_t n = 0; n < 4; n++) channel_map[n] = n;
  }

  void Set(uint8_t new_channel_order)
  {
    if (new_channel_order == channel_order) return;
    channel_order = new_channel_order;

    switch (channel_order) {
      case CHANNEL_ORDER_AETR:
        // nothing to do
        break;
      case CHANNEL_ORDER_TAER:
        // TODO
        break;
      case CHANNEL_ORDER_ETAR:
        channel_map[0] = 2;
        channel_map[1] = 0;
        channel_map[2] = 1;
        channel_map[3] = 3;
        break;
    }
  }

  void Apply(tRcData* rc)
  {
    uint16_t ch[4] = { rc->ch[0], rc->ch[1], rc->ch[2], rc->ch[3] };
    for (uint8_t n = 0; n < 4; n++) {
      rc->ch[n] = ch[channel_map[n]];
    }
  }

private:
  uint8_t channel_order;
  uint8_t channel_map[4];
};

ChannelOrder channelOrder;


void init(void)
{
  leds_init();
  button_init();
  pos_switch_init();

  delay_init();
  clock_init();
  serial.Init();

  in.Init();

  uartc_init();

  sx.Init();
}


//-------------------------------------------------------
// Statistics for Transmitter
//-------------------------------------------------------

static inline bool connected(void);

class TxStats : public TxStatsBase
{
  bool is_connected(void) override { return connected(); }
};

TxStats txstats;


//-------------------------------------------------------
// mbridge
//-------------------------------------------------------

#include "mbridge_interface.h" // this includes uart.h as it needs callbacks
#include "crsf_interface.h" // this includes uart.h as it needs callbacks


//-------------------------------------------------------
// mavlink
//-------------------------------------------------------

#include "mavlink_interface.h"


//-------------------------------------------------------
// SX1280
//-------------------------------------------------------

volatile uint16_t irq_status;

IRQHANDLER(
void SX_DIO1_EXTI_IRQHandler(void)
{
  LL_EXTI_ClearFlag_0_31(SX_DIO1_EXTI_LINE_x);
  //LED_RIGHT_RED_TOGGLE;
  irq_status = sx.GetIrqStatus();
  sx.ClearIrqStatus(SX1280_IRQ_ALL);
  if (irq_status & SX1280_IRQ_RX_DONE) {
    sx.ReadFrame((uint8_t*)&rxFrame, FRAME_TX_RX_LEN);
  }
})


typedef enum {
    CONNECT_STATE_LISTEN = 0,
    CONNECT_STATE_SYNC,
    CONNECT_STATE_CONNECTED,
} CONNECT_STATE_ENUM;

typedef enum {
    LINK_STATE_IDLE = 0,
    LINK_STATE_TRANSMIT,
    LINK_STATE_TRANSMIT_WAIT,
    LINK_STATE_RECEIVE,
    LINK_STATE_RECEIVE_WAIT,
    LINK_STATE_RECEIVE_DONE,
} LINK_STATE_ENUM;


uint8_t payload[FRAME_TX_PAYLOAD_LEN] = {0};
uint8_t payload_len = 0;


void process_transmit_frame(uint8_t ack)
{
  // read data from serial
  if (connected()) {
    memset(payload, 0, FRAME_TX_PAYLOAD_LEN);
    payload_len = 0;

    for (uint8_t i = 0; i < FRAME_TX_PAYLOAD_LEN; i++) {
#if (SETUP_TX_SERIAL_DESTINATION == 1)
      if (!bridge.available()) break;
      payload[payload_len] = bridge.getc();
#else
      if (!serial.available()) break;
      payload[payload_len] = serial.getc();
#endif
      payload_len++;
    }

    stats.bytes_transmitted.Add(payload_len);
    stats.fresh_serial_data_transmitted.Inc();

  } else {
#if (SETUP_TX_SERIAL_DESTINATION == 1)
    bridge.flush();
#else
    serial.flush();
#endif
    memset(payload, 0, FRAME_TX_PAYLOAD_LEN);
    payload_len = 0;
  }

  tFrameStats frame_stats;
  frame_stats.seq_no = stats.transmit_seq_no;
  frame_stats.ack = ack;
  frame_stats.antenna = ANTENNA_1;
  frame_stats.rssi = stats.last_rx_rssi;
  frame_stats.LQ = txstats.GetLQ();
  frame_stats.LQ_serial_data = txstats.GetLQ_serial_data();

  pack_tx_frame(&txFrame, &frame_stats, &rcData, payload, payload_len);
  sx.SendFrame((uint8_t*)&txFrame, FRAME_TX_RX_LEN, 10); // 10 ms tmo
}


void process_received_frame(bool do_payload)
{
  stats.received_antenna = rxFrame.status.antenna;
  stats.received_rssi = -(rxFrame.status.rssi_u7);
  stats.received_LQ = rxFrame.status.LQ;
  stats.received_LQ_serial_data = rxFrame.status.LQ_serial_data;

  if (!do_payload) return;

  // output data on serial
  for (uint8_t i = 0; i < rxFrame.status.payload_len; i++) {
    uint8_t c = rxFrame.payload[i];
#if (SETUP_TX_SERIAL_DESTINATION == 1)
    bridge.putc(c); // send to radio
#else
    serial.putc(c); // send to serial
#endif

    // parse stream, and inject radio status
#if (SETUP_TX_SEND_RADIO_STATUS > 0)
    uint8_t res = fmav_parse_to_frame_buf(&f_result, f_buf, &f_status, c);
    if (res == FASTMAVLINK_PARSE_RESULT_OK) { // we have a complete mavlink frame
      if (inject_radio_status) {
        inject_radio_status = false;
#  if (SETUP_TX_SEND_RADIO_STATUS == 1)
        send_radio_status();
#  elif (SETUP_TX_SEND_RADIO_STATUS == 2)
        send_radio_status_v2();
#  endif
      }
    }
#endif
  }

  stats.bytes_received.Add(rxFrame.status.payload_len);
  stats.fresh_serial_data_received.Inc();

  DBG_MAIN(uartc_puts("got "); uartc_puts(": ");
  for (uint8_t i = 0; i < rxFrame.status.payload_len; i++) uartc_putc(rxFrame.payload[i]);
  uartc_putc('\n');)
}


void do_transmit(void) // we send a TX frame to receiver
{
  uint8_t ack = 0;

  stats.transmit_seq_no++;

  process_transmit_frame(ack);
}


bool do_receive(void) // we receive a RX frame from receiver
{
bool ok = false;

  uint8_t res = check_rx_frame(&rxFrame);
  if (res) {
    DBG_MAIN(uartc_puts("fail "); uartc_putc('\n');)
uartc_puts("fail "); uartc_puts(u8toHEX_s(res));uartc_putc('\n');
  }

  if (res == CHECK_OK) {
    bool do_payload = true;

    process_received_frame(do_payload);

    txstats.doValidFrameReceived();

    stats.last_received_seq_no = rxFrame.status.seq_no;
    stats.last_received_ack = rxFrame.status.ack;

	  ok = true;
  } else {
    stats.last_received_seq_no = UINT8_MAX;
  }

  if (res != CHECK_ERROR_SYNCWORD) {
    // read it here, we want to have it even if it's a bad packet, but it should be for us
    sx.GetPacketStatus(&stats.last_rx_rssi, &stats.last_rx_snr);

    // we count all received frames, which are at least for us
    txstats.doFrameReceived();
  }

  return ok;
}


//##############################################################################################################
//*******************************************************
// MAIN routine
//*******************************************************

uint16_t led_blink;
uint16_t tick_1hz;

uint16_t tx_tick;
uint16_t link_state;
uint8_t connect_state;
uint16_t connect_tmo_cnt;
uint8_t connect_sync_cnt;

bool crsf_telemetry_tick_start; // called at 50Hz, in sync with transmit
bool crsf_telemetry_tick_next; // called at 1 ms
uint16_t crsf_telemetry_state;


static inline bool connected(void)
{
  return (connect_state == CONNECT_STATE_CONNECTED);
}


int main_main(void)
{
#ifdef BOARD_TEST_H
  main_test();
#endif
  init();
#if (defined USE_MBRIDGE)
  bridge.Init();
#endif
#if (SETUP_TX_CHANNELS_SOURCE == 3)
  crsf.Init();
#endif

  DBG_MAIN(uartc_puts("\n\n\nHello\n\n");)

  // startup sign of life
  LED_RED_OFF;
  for (uint8_t i = 0; i < 7; i++) { LED_RED_TOGGLE; delay_ms(50); }

  // start up sx
  if (!sx.isOk()) {
    while (1) { LED_RED_TOGGLE; delay_ms(50); } // fail!
  }
  sx.StartUp();
  fhss.Init(FHSS_SEED);
  fhss.StartTx();
  sx.SetRfFrequency(fhss.GetCurrFreq());

//  for (uint8_t i = 0; i < fhss.Cnt(); i++) {
//    uartc_puts("c = "); uartc_puts(u8toBCD_s(fhss.ch_list[i])); uartc_puts(" f = "); uartc_puts(u32toBCD_s(fhss.fhss_list[i])); uartc_puts("\n"); delay_ms(50);
//  }

  tx_tick = 0;
  link_state = LINK_STATE_IDLE;
  connect_state = CONNECT_STATE_LISTEN;
  connect_tmo_cnt = 0;
  connect_sync_cnt = 0;

  txstats.Init(LQ_AVERAGING_PERIOD);

  in.Configure(IN_CONFIG_SBUS);

  f_init();

  crsf_telemetry_tick_start = false;
  crsf_telemetry_tick_next = false;
  crsf_telemetry_state = 0;

  led_blink = 0;
  tick_1hz = 0;
  doSysTask = 0; // helps in avoiding too short first loop
  while (1) {

    //-- SysTask handling

    if (doSysTask) {
      doSysTask = 0;

      if (connect_tmo_cnt) {
        connect_tmo_cnt--;
      }

      DECc(tick_1hz, SYSTICK_DELAY_MS(1000));
      DECc(tx_tick, SYSTICK_DELAY_MS(FRAME_RATE_MS));
      if (connected()) {
        DECc(led_blink, SYSTICK_DELAY_MS(500));
      } else {
        DECc(led_blink, SYSTICK_DELAY_MS(200));
      }

      if (!led_blink) {
        if (connected()) LED_GREEN_TOGGLE; else LED_RED_TOGGLE;
      }
      if (connected()) { LED_RED_OFF; } else { LED_GREEN_OFF; }

      if (!connected()) {
        stats.Clear();
        f_init();
      }

      if (!tick_1hz) {
        txstats.Update1Hz();
        if (connected()) inject_radio_status = true;

        uartc_puts("TX: ");
        uartc_puts(u8toBCD_s(txstats.GetLQ_serial_data()));
        uartc_puts(" (");
        uartc_puts(u8toBCD_s(stats.frames_received.GetLQ())); uartc_putc(',');
        uartc_puts(u8toBCD_s(stats.valid_frames_received.GetLQ()));
        uartc_puts("),");
        uartc_puts(u8toBCD_s(stats.received_LQ)); uartc_puts(", ");

        uartc_puts(s8toBCD_s(stats.last_rx_rssi)); uartc_putc(',');
        uartc_puts(s8toBCD_s(stats.received_rssi)); uartc_puts(", ");
        uartc_puts(s8toBCD_s(stats.last_rx_snr)); uartc_puts("; ");

        uartc_puts(u16toBCD_s(stats.bytes_transmitted.GetBytesPerSec())); uartc_puts(", ");
        uartc_puts(u16toBCD_s(stats.bytes_received.GetBytesPerSec())); uartc_puts("; ");
        uartc_putc('\n');
      }

      if (!tx_tick) {
        // trigger next cycle
        if (connected() && !connect_tmo_cnt) {
          connect_state = CONNECT_STATE_LISTEN;
        }
        if (connected() && (link_state != LINK_STATE_RECEIVE_DONE)) {
          // frame missed
          connect_sync_cnt = 0;
        }
/*
        static uint16_t tlast_us = 0;
        uint16_t tnow_us = micros();
        uint16_t dt = tnow_us - tlast_us;
        tlast_us = tnow_us;

        uartc_puts(" ");
        uartc_puts(u16toBCD_s(tnow_us)); uartc_puts(", "); uartc_puts(u16toBCD_s(dt)); uartc_puts("; ");
        switch (link_state) {
        case LINK_STATE_IDLE: uartc_puts("i  "); break;
        case LINK_STATE_TRANSMIT: uartc_puts("t  "); break;
        case LINK_STATE_TRANSMIT_WAIT: uartc_puts("tw "); break;
        case LINK_STATE_RECEIVE: uartc_puts("r  "); break;
        case LINK_STATE_RECEIVE_WAIT: uartc_puts("rw "); break;
        case LINK_STATE_RECEIVE_DONE: uartc_puts("rd "); break;
        }
        switch (connect_state) {
        case CONNECT_STATE_LISTEN: uartc_puts("L "); break;
        case CONNECT_STATE_SYNC: uartc_puts("S "); break;
        case CONNECT_STATE_CONNECTED: uartc_puts("C "); break;
        }
        uartc_puts(connected() ? "c " : "d ");
        uartc_puts("\n");
*/
        txstats.Next();
        link_state = LINK_STATE_TRANSMIT;
        crsf_telemetry_tick_start = true;
      }

      crsf_telemetry_tick_next = true;
    }

    //-- SX handling

    switch (link_state) {
    case LINK_STATE_IDLE:
    case LINK_STATE_RECEIVE_DONE:
      break;

    case LINK_STATE_TRANSMIT:
      fhss.HopToNext();
      sx.SetRfFrequency(fhss.GetCurrFreq());
      do_transmit();
      link_state = LINK_STATE_TRANSMIT_WAIT;
      irq_status = 0;
      DBG_MAIN_SLIM(uartc_puts(">");)
      break;

    case LINK_STATE_RECEIVE:
      // datasheet says "As soon as a packet is detected, the timer is automatically
      // disabled to allow complete reception of the packet." Why does then 5 ms not work??
      sx.SetToRx(10); // we wait 10 ms for the start for the frame, 5 ms does not work ??
      link_state = LINK_STATE_RECEIVE_WAIT;
      irq_status = 0;
      break;
    }

    if (irq_status) {
      if (link_state == LINK_STATE_TRANSMIT_WAIT) {
        if (irq_status & SX1280_IRQ_TX_DONE) {
          irq_status = 0;
          link_state = LINK_STATE_RECEIVE;
          DBG_MAIN_SLIM(uartc_puts("!");)
        }
      }
      else
      if (link_state == LINK_STATE_RECEIVE_WAIT) {
        if (irq_status & SX1280_IRQ_RX_DONE) {
          irq_status = 0;
          if (do_receive()) {
            if (connect_state == CONNECT_STATE_LISTEN) {
              connect_state = CONNECT_STATE_SYNC;
              connect_sync_cnt = 0;
            } else
            if (connect_state == CONNECT_STATE_SYNC) {
              connect_sync_cnt++;
              if (connect_sync_cnt >= CONNECT_SYNC_CNT) connect_state = CONNECT_STATE_CONNECTED;
            } else {
              connect_state = CONNECT_STATE_CONNECTED;
            }
            connect_tmo_cnt = CONNECT_TMO_SYSTICKS;
            link_state = LINK_STATE_RECEIVE_DONE; // ready for next frame
          } else {
            link_state = LINK_STATE_IDLE; // ready for next frame
          }
          DBG_MAIN_SLIM(uartc_puts("<\n");)
        }
      }

      if (irq_status & SX1280_IRQ_RX_TX_TIMEOUT) {
        irq_status = 0;
        link_state = LINK_STATE_IDLE;
      }

      if (irq_status & SX1280_IRQ_RX_DONE) {
        LED_GREEN_OFF;
        while (1) { LED_RED_ON; delay_ms(5); LED_RED_OFF; delay_ms(5); }
      }
      if (irq_status & SX1280_IRQ_TX_DONE) {
        LED_RED_OFF;
        while (1) { LED_GREEN_ON; delay_ms(5); LED_GREEN_OFF; delay_ms(5); }
      }
    }


    // update channels
    channelOrder.Set(SETUP_TX_CHANNEL_ORDER); //TODO: find proper place
    //-- MBridge handling
#if (defined USE_MBRIDGE)
    if (bridge.channels_received) {
      bridge.channels_received = false;
#  if (SETUP_TX_CHANNELS_SOURCE == 1)
      // update channels
      fill_rcdata_from_mbridge(&rcData, &(bridge.channels));
      channelOrder.Apply(&rcData);
#  endif
      // when we receive channels packet from transmitter, we send link stats to transmitter
      mbridge_send_LinkStats();
    }

    if (bridge.cmd_received) {
      bridge.cmd_received = false;
      uint8_t cmd;
      uint8_t payload[MBRIDGE_R2M_COMMAND_PAYLOAD_LEN_MAX];
      bridge.GetCommand(&cmd, payload);
    }
#elif (SETUP_TX_CHANNELS_SOURCE == 2) && (defined DEVICE_HAS_IN)
    // update channels
    in.Update(&rcData);
    channelOrder.Apply(&rcData);
#elif (SETUP_TX_CHANNELS_SOURCE == 3)
    if (crsf_telemetry_tick_start) {
      crsf_telemetry_tick_start = false;
      crsf_telemetry_state = 1;
    }
    if (crsf_telemetry_state && crsf_telemetry_tick_next && crsf.IsEmpty()) {
      crsf_telemetry_tick_next = false;
      switch (crsf_telemetry_state) {
      case 1: crsf_send_LinkStatistics(); break;
      case 2: crsf_send_LinkStatisticsTx(); break;
      case 3: crsf_send_LinkStatisticsRx(); break;
      }
      crsf_telemetry_state++;
    }

    if (crsf.frame_received) {
      crsf.frame_received = false;
      // update channels
      if (crsf.IsChannelData()) {
        fill_rcdata_from_crsf(&rcData, crsf.frame);
        channelOrder.Apply(&rcData);
      }
    }
#endif

  }//end of while(1) loop

}//end of main

