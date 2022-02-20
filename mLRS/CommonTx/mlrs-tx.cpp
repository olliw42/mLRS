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
#define DEBUG_ENABLED


// we set the priorities here to have an overview
#define UART_IRQ_PRIORITY           10 // mbridge, this needs to be high, when lower than DIO1, the module could stop sending via the bridge
#define UARTB_IRQ_PRIORITY          11 // serial
#define UARTC_IRQ_PRIORITY          11 // debug
#define SX_DIO_EXTI_IRQ_PRIORITY    13
#define SX2_DIO_EXTI_IRQ_PRIORITY   13

#include "..\Common\common_conf.h"
#include "..\Common\common_types.h"
#include "..\Common\hal\glue.h"
#include "..\modules\stm32ll-lib\src\stdstm32.h"
#include "..\modules\stm32ll-lib\src\stdstm32-peripherals.h"
#include "..\Common\sx-drivers\sx12xx.h"
#include "..\Common\hal\hal.h"
#include "..\modules\stm32ll-lib\src\stdstm32-delay.h" // these are dependent on hal
#include "..\modules\stm32ll-lib\src\stdstm32-spi.h"
#ifdef DEVICE_HAS_DIVERSITY
#include "..\modules\stm32ll-lib\src\stdstm32-spib.h"
#endif
#ifndef DEVICE_HAS_NO_SERIAL
#include "..\modules\stm32ll-lib\src\stdstm32-uartb.h"
#endif
#ifndef DEVICE_HAS_NO_DEBUG
#include "..\modules\stm32ll-lib\src\stdstm32-uartc.h"
#endif
#ifdef DEVICE_HAS_I2C
#include "..\Common\stdstm32-i2c.h"
#endif
#define FASTMAVLINK_IGNORE_WADDRESSOFPACKEDMEMBER
#include "..\Common\mavlink\out\mlrs\mlrs.h"
#include "..\Common\fhss.h"
#include "..\Common\setup.h"
#include "..\Common\common.h"
#include "..\Common\micros.h"
//#include "..\Common\test.h" // un-comment if you want to compile for board test

#ifdef DEVICE_HAS_IN
#include "..\modules\stm32ll-lib\src\stdstm32-uarte.h"
#endif
#include "in.h"
#include "txstats.h"


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

  void config_sbus(bool inverted) override
  {
    uarte_setprotocol(100000, XUART_PARITY_EVEN, UART_STOPBIT_2);
    if (!inverted) {
      in_set_inverted();
      gpio_init_af(UARTE_RX_IO, IO_MODE_INPUT_PD, UARTE_IO_AF, IO_SPEED_VERYFAST);
    } else {
      in_set_normal();
      gpio_init_af(UARTE_RX_IO, IO_MODE_INPUT_PU, UARTE_IO_AF, IO_SPEED_VERYFAST);
    }
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
  micros_init();
  serial.Init();

  in.Init();

  dbg.Init();

  setup_init();

  sx.Init();
  sx2.Init();
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
volatile uint16_t irq2_status;

IRQHANDLER(
void SX_DIO_EXTI_IRQHandler(void)
{
  LL_EXTI_ClearFlag_0_31(SX_DIO_EXTI_LINE_x);
  //LED_RIGHT_RED_TOGGLE;
  irq_status = sx.GetAndClearIrqStatus(SX12xx_IRQ_ALL);
  if (irq_status & SX12xx_IRQ_RX_DONE) {
    uint16_t sync_word;
    sx.ReadBuffer(0, (uint8_t*)&sync_word, 2); // rxStartBufferPointer is always 0, so no need for sx.GetRxBufferStatus()
    if (sync_word != Config.FrameSyncWord) irq_status = 0; // not for us, so ignore it
  }
})
#ifdef DEVICE_HAS_DIVERSITY
IRQHANDLER(
void SX2_DIO_EXTI_IRQHandler(void)
{
  LL_EXTI_ClearFlag_0_31(SX2_DIO_EXTI_LINE_x);
  //LED_RIGHT_RED_TOGGLE;
  irq2_status = sx2.GetAndClearIrqStatus(SX12xx_IRQ_ALL);
  if (irq2_status & SX12xx_IRQ_RX_DONE) {
    uint16_t sync_word;
    sx2.ReadBuffer(0, (uint8_t*)&sync_word, 2); // rxStartBufferPointer is always 0, so no need for sx.GetRxBufferStatus()
    if (sync_word != Config.FrameSyncWord) irq2_status = 0; // not for us, so ignore it
  }
})
#endif


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

typedef enum {
    RX_STATUS_NONE = 0, // no frame received
    RX_STATUS_INVALID,
    RX_STATUS_VALID,
} RX_STATUS_ENUM;

uint8_t link_rx1_status;
uint8_t link_rx2_status;


uint8_t payload[FRAME_TX_PAYLOAD_LEN] = {0};
uint8_t payload_len = 0;


void process_transmit_frame(uint8_t antenna, uint8_t ack)
{
  // read data from serial
  if (connected()) {
    memset(payload, 0, FRAME_TX_PAYLOAD_LEN);
    payload_len = 0;

    for (uint8_t i = 0; i < FRAME_TX_PAYLOAD_LEN; i++) {
      if (Setup.Tx.SerialDestination == SERIAL_DESTINATION_SERIAL_PORT) { // don't use a switch, since we want to break out of the for loop
        if (!serial.available()) break;
        payload[payload_len] = serial.getc();
      } else
      if (Setup.Tx.SerialDestination == SERIAL_DESTINATION_MBRDIGE) {
        if (!bridge.available()) break;
        payload[payload_len] = bridge.getc();
      }
      payload_len++;
    }

    stats.bytes_transmitted.Add(payload_len);
    stats.fresh_serial_data_transmitted.Inc();

  } else {
    if (Setup.Tx.SerialDestination == SERIAL_DESTINATION_SERIAL_PORT) serial.flush();
    if (Setup.Tx.SerialDestination == SERIAL_DESTINATION_MBRDIGE) bridge.flush();
    memset(payload, 0, FRAME_TX_PAYLOAD_LEN);
    payload_len = 0;
  }

  stats.last_tx_antenna = antenna;

  tFrameStats frame_stats;
  frame_stats.seq_no = stats.transmit_seq_no;
  frame_stats.ack = ack;
  frame_stats.antenna = stats.last_rx_antenna;
  frame_stats.transmit_antenna = antenna;
  frame_stats.rssi = (antenna == ANTENNA_1) ? stats.last_rx_rssi1 : stats.last_rx_rssi2;
  frame_stats.LQ = txstats.GetLQ();
  frame_stats.LQ_serial_data = txstats.GetLQ_serial_data();

  pack_tx_frame(&txFrame, &frame_stats, &rcData, payload, payload_len);

  if (antenna == ANTENNA_1) {
    sx.SendFrame((uint8_t*)&txFrame, FRAME_TX_RX_LEN, SEND_FRAME_TMO); // 10 ms tmo
  } else {
    sx2.SendFrame((uint8_t*)&txFrame, FRAME_TX_RX_LEN, SEND_FRAME_TMO); // 10 ms tmo
  }
}


void process_received_frame(bool do_payload, tRxFrame* frame)
{
  stats.received_antenna = frame->status.antenna;
  stats.received_transmit_antenna = frame->status.transmit_antenna;
  stats.received_rssi = -(frame->status.rssi_u7);
  stats.received_LQ = frame->status.LQ;
  stats.received_LQ_serial_data = frame->status.LQ_serial_data;

  if (!do_payload) return;

  // output data on serial
  for (uint8_t i = 0; i < frame->status.payload_len; i++) {
    uint8_t c = frame->payload[i];
    if (Setup.Tx.SerialDestination == SERIAL_DESTINATION_SERIAL_PORT) serial.putc(c); // send to serial
    if (Setup.Tx.SerialDestination == SERIAL_DESTINATION_MBRDIGE) bridge.putc(c); // send to radio
    // parse stream, and inject radio status
    if (Setup.Tx.SendRadioStatus) {
      uint8_t res = fmav_parse_to_frame_buf(&f_result, f_buf, &f_status, c);
      if (res == FASTMAVLINK_PARSE_RESULT_OK && inject_radio_status) { // we have a complete mavlink frame
        inject_radio_status = false;
        if (Setup.Tx.SendRadioStatus == SEND_RADIO_STATUS_ON) send_radio_status();
        if (Setup.Tx.SendRadioStatus == SEND_RADIO_STATUS_V2_ON) send_radio_status_v2();
      }
    }
  }

  stats.bytes_received.Add(frame->status.payload_len);
  stats.fresh_serial_data_received.Inc();
}


void handle_receive(uint8_t antenna)
{
uint8_t rx_status;
tRxFrame* frame;

  if (antenna == ANTENNA_1) {
    rx_status = link_rx1_status;
    frame = &rxFrame;
  } else {
    rx_status = link_rx2_status;
    frame = &rxFrame2;
  }

  if (rx_status != RX_STATUS_INVALID) {
    bool do_payload = true;

    process_received_frame(do_payload, frame);

    txstats.doValidFrameReceived();

    stats.received_seq_no_last = frame->status.seq_no;
    stats.received_ack_last = frame->status.ack;

  } else {
    stats.received_seq_no_last = UINT8_MAX;
    stats.received_ack_last = 0;
  }

  // we set it for all received frames
  stats.last_rx_antenna = antenna;

  // we count all received frames
  txstats.doFrameReceived();
}


void do_transmit(uint8_t antenna) // we send a TX frame to receiver
{
  uint8_t ack = 1;

  stats.transmit_seq_no++;

  process_transmit_frame(antenna, ack);
}


uint8_t do_receive(uint8_t antenna) // we receive a RX frame from receiver
{
uint8_t res;
uint8_t rx_status = RX_STATUS_INVALID; // this also signals that a frame was received

  // we don't need to read sx.GetRxBufferStatus(), but hey
  // we could save 2 byte's time by not reading sync_word again, but hey
  if (antenna == ANTENNA_1) {
    sx.ReadFrame((uint8_t*)&rxFrame, FRAME_TX_RX_LEN);
    res = check_rx_frame(&rxFrame);
  } else {
    sx2.ReadFrame((uint8_t*)&rxFrame2, FRAME_TX_RX_LEN);
    res = check_rx_frame(&rxFrame2);
  }

  if (res) {
    DBG_MAIN(dbg.puts("fail ");dbg.putc('\n');)
dbg.puts("fail ");dbg.puts(u8toHEX_s(res));dbg.putc('\n');
  }

  if (res == CHECK_ERROR_SYNCWORD) return false; // must not happen !

  if (res == CHECK_OK) {
    rx_status = RX_STATUS_VALID;
  }

  // we want to have it even if it's a bad packet
  if (antenna == ANTENNA_1) {
    sx.GetPacketStatus(&stats.last_rx_rssi1, &stats.last_rx_snr1);
  } else {
    sx2.GetPacketStatus(&stats.last_rx_rssi2, &stats.last_rx_snr2);
  }

  return rx_status;
}


//##############################################################################################################
//*******************************************************
// MAIN routine
//*******************************************************

uint16_t led_blink;
uint16_t tick_1hz;

uint16_t tx_tick;
bool doPreTransmit;

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
  bridge.Init();
#ifdef USE_CRSF
  crsf.Init();
#endif

  DBG_MAIN(dbg.puts("\n\n\nHello\n\n");)

  // startup sign of life
  LED_RED_OFF;
  for (uint8_t i = 0; i < 7; i++) { LED_RED_TOGGLE; delay_ms(50); }

  // start up sx
  if (!sx.isOk()) {
    while (1) { LED_RED_TOGGLE; delay_ms(25); } // fail!
  }
  if (!sx2.isOk()) {
    while (1) { LED_GREEN_TOGGLE; delay_ms(25); } // fail!
  }
  IF_ANTENNA1(sx.StartUp());
  IF_ANTENNA2(sx2.StartUp());
  fhss.Init(Config.FhssNum, Config.FhssSeed);
  fhss.StartTx();

  sx.SetRfFrequency(fhss.GetCurrFreq());
  sx2.SetRfFrequency(fhss.GetCurrFreq());

//  for (uint8_t i = 0; i < fhss.Cnt(); i++) {
//    dbg.puts("c = ");dbg.puts(u8toBCD_s(fhss.ch_list[i]));dbg.puts(" f = ");dbg.puts(u32toBCD_s(fhss.fhss_list[i]));dbg.puts("\n"); delay_ms(50);
//  }

  tx_tick = 0;
  doPreTransmit = false;
  link_state = LINK_STATE_IDLE;
  connect_state = CONNECT_STATE_LISTEN;
  connect_tmo_cnt = 0;
  connect_sync_cnt = 0;
  link_rx1_status = RX_STATUS_NONE;
  link_rx2_status = RX_STATUS_NONE;

  txstats.Init(Config.LQAveragingPeriod);

  in.Configure(Setup.Tx.InMode);

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
      DECc(tx_tick, SYSTICK_DELAY_MS(Config.frame_rate_ms));
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
        f_init();
      }

      if (!tick_1hz) {
        txstats.Update1Hz();
        if (connected()) inject_radio_status = true;

        dbg.puts("\nTX: ");
        dbg.puts(u8toBCD_s(txstats.GetLQ_serial_data()));
        dbg.puts(" (");
        dbg.puts(u8toBCD_s(stats.frames_received.GetLQ())); dbg.putc(',');
        dbg.puts(u8toBCD_s(stats.valid_frames_received.GetLQ()));
        dbg.puts("),");
        dbg.puts(u8toBCD_s(stats.received_LQ)); dbg.puts(", ");

        dbg.puts(s8toBCD_s(stats.last_rx_rssi1)); dbg.putc(',');
        dbg.puts(s8toBCD_s(stats.received_rssi)); dbg.puts(", ");
        dbg.puts(s8toBCD_s(stats.last_rx_snr1)); dbg.puts("; ");

        dbg.puts(u16toBCD_s(stats.bytes_transmitted.GetBytesPerSec())); dbg.puts(", ");
        dbg.puts(u16toBCD_s(stats.bytes_received.GetBytesPerSec())); dbg.puts("; ");
      }

      if (!tx_tick) {
        // trigger next cycle
        doPreTransmit = true;
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
      sx2.SetRfFrequency(fhss.GetCurrFreq());
      do_transmit((USE_ANTENNA1) ? ANTENNA_1 : ANTENNA_2);
      link_state = LINK_STATE_TRANSMIT_WAIT;
      irq_status = 0;
      irq2_status = 0;
      DBG_MAIN_SLIM(dbg.puts("\n>");)
      break;

    case LINK_STATE_RECEIVE:
      // for sx1280: datasheet says "As soon as a packet is detected, the timer is automatically
      // disabled to allow complete reception of the packet." Why does then 5 ms not work??
      IF_ANTENNA1(sx.SetToRx(TX_SET_RX_TMO)); // we wait 10 ms for the start for the frame, 5 ms does not work ??
      IF_ANTENNA2(sx2.SetToRx(TX_SET_RX_TMO));
      link_state = LINK_STATE_RECEIVE_WAIT;
      irq_status = 0;
      irq2_status = 0;
      break;
    }//end of switch(link_state)

IF_ANTENNA1(
    if (irq_status) {
      if (link_state == LINK_STATE_TRANSMIT_WAIT) {
        if (irq_status & SX12xx_IRQ_TX_DONE) {
          irq_status = 0;
          link_state = LINK_STATE_RECEIVE;
          DBG_MAIN_SLIM(dbg.puts("!");)
        }
      } else
      if (link_state == LINK_STATE_RECEIVE_WAIT) {
        if (irq_status & SX12xx_IRQ_RX_DONE) {
          irq_status = 0;
          link_rx1_status = do_receive(ANTENNA_1);
          DBG_MAIN_SLIM(dbg.puts("<");)
        }
      }

      if (irq_status & SX12xx_IRQ_TIMEOUT) {
        irq_status = 0;
        link_state = LINK_STATE_IDLE;
        link_rx1_status = RX_STATUS_NONE;
        link_rx2_status = RX_STATUS_NONE;
      }

      if (irq_status & SX12xx_IRQ_RX_DONE) {
        LED_GREEN_OFF;
        while (1) { LED_RED_ON; delay_ms(25); LED_RED_OFF; delay_ms(25); }
      }
      if (irq_status & SX12xx_IRQ_TX_DONE) {
        LED_RED_OFF;
        while (1) { LED_GREEN_ON; delay_ms(25); LED_GREEN_OFF; delay_ms(25); }
      }
    }//end of if(irq_status)
);
IF_ANTENNA2(
    if (irq2_status) {
      if (link_state == LINK_STATE_TRANSMIT_WAIT) {
        if (irq2_status & SX12xx_IRQ_TX_DONE) {
          irq2_status = 0;
          link_state = LINK_STATE_RECEIVE;
          DBG_MAIN_SLIM(dbg.puts("!");)
        }
      } else
      if (link_state == LINK_STATE_RECEIVE_WAIT) {
        if (irq2_status & SX12xx_IRQ_RX_DONE) {
          irq2_status = 0;
          link_rx2_status = do_receive(ANTENNA_2);
          DBG_MAIN_SLIM(dbg.puts("<");)
        }
      }

      if (irq2_status & SX12xx_IRQ_TIMEOUT) { // ??????????????????
        irq2_status = 0;
        link_state = LINK_STATE_IDLE;
        link_rx1_status = RX_STATUS_NONE;
        link_rx2_status = RX_STATUS_NONE;
      }

      if (irq2_status & SX12xx_IRQ_RX_DONE) {
        LED_GREEN_ON; //LED_GREEN_OFF;
        while (1) { LED_RED_ON; delay_ms(25); LED_RED_OFF; delay_ms(25); }
      }
      if (irq2_status & SX12xx_IRQ_TX_DONE) {
        LED_RED_ON; //LED_RED_OFF;
        while (1) { LED_GREEN_ON; delay_ms(25); LED_GREEN_OFF; delay_ms(25); }
      }
    }//end of if(irq2_status)
);

    // this happens before switching to transmit, i.e. after a frame was or should have been received
    if (doPreTransmit) {
      doPreTransmit = false;

      bool frame_received = false;
      bool valid_frame_received = false;
      if (USE_ANTENNA1 && USE_ANTENNA1) {
        frame_received = (link_rx1_status > RX_STATUS_NONE) || (link_rx2_status > RX_STATUS_NONE);
        valid_frame_received = (link_rx1_status > RX_STATUS_INVALID) || (link_rx2_status > RX_STATUS_INVALID);
      } else if (USE_ANTENNA1) {
        frame_received = (link_rx1_status > RX_STATUS_NONE);
        valid_frame_received = (link_rx1_status > RX_STATUS_INVALID);
      } else if (USE_ANTENNA2) {
        frame_received = (link_rx2_status > RX_STATUS_NONE);
        valid_frame_received = (link_rx2_status > RX_STATUS_INVALID);
      }

      if (frame_received) { // frame received
        uint8_t antenna = ANTENNA_1;

        if (USE_ANTENNA1 && USE_ANTENNA1) {
          // work out which antenna we choose
          //            |   NONE   |  INVALID  | VALID
          // --------------------------------------------------------
          // NONE       |          |   1 or 2  |  1
          // INVALID    |  1 or 2  |   1 or 2  |  1
          // VALID      |    2     |     2     |  1 or 2

          if (link_rx1_status == link_rx2_status) {
            // we can choose either antenna, so select the one with the better rssi
            antenna = (stats.last_rx_rssi1 > stats.last_rx_rssi2) ? ANTENNA_1 : ANTENNA_2;
          } else
          if (link_rx1_status == RX_STATUS_VALID) {
            antenna = ANTENNA_1;
          } else
          if (link_rx2_status == RX_STATUS_VALID) {
            antenna = ANTENNA_2;
          } else {
            // we can choose either antenna, so select the one with the better rssi
            antenna = (stats.last_rx_rssi1 > stats.last_rx_rssi2) ? ANTENNA_1 : ANTENNA_2;
          }
        } else if (USE_ANTENNA2) {
          antenna = ANTENNA_2;
        }

        handle_receive(antenna);
      }

      if (valid_frame_received) { // valid frame received
        switch (connect_state) {
        case CONNECT_STATE_LISTEN:
          connect_state = CONNECT_STATE_SYNC;
          connect_sync_cnt = 0;
          break;
        case CONNECT_STATE_SYNC:
          connect_sync_cnt++;
          if (connect_sync_cnt >= CONNECT_SYNC_CNT) connect_state = CONNECT_STATE_CONNECTED;
          break;
        default:
          connect_state = CONNECT_STATE_CONNECTED;
        }
        connect_tmo_cnt = CONNECT_TMO_SYSTICKS;
      }

      if (connected() && !connect_tmo_cnt) {
        connect_state = CONNECT_STATE_LISTEN;
      }

      // we didn't receive a valid frame
      if (connected() && !valid_frame_received) {
        // reset sync counter, relevant if in sync
        connect_sync_cnt = 0;
      }

      link_state = LINK_STATE_TRANSMIT;
      link_rx1_status = RX_STATUS_NONE;
      link_rx2_status = RX_STATUS_NONE;

      if (!connected()) stats.Clear();
      txstats.Next();
    }//end of if(doPreTransmit)


    //-- Update channels, MBridge handling, Crsf handling, In handling

    channelOrder.Set(Setup.Tx.ChannelOrder); //TODO: find proper place

#if (defined USE_MBRIDGE)
    // when mBridge is enabled on Tx, it sends channels in regular intervals, this we can used as sync
    if (bridge.channels_received) {
      bridge.channels_received = false;
      if (Setup.Tx.ChannelsSource == CHANNEL_SOURCE_MBRIDGE) {
        // update channels
        fill_rcdata_from_mbridge(&rcData, &(bridge.channels));
        channelOrder.Apply(&rcData);
      }
      // when we receive channels packet from transmitter, we send link stats to transmitter
      mbridge_send_LinkStats();
    }

    if (bridge.cmd_received) {
      bridge.cmd_received = false;
      uint8_t cmd;
      uint8_t payload[MBRIDGE_R2M_COMMAND_PAYLOAD_LEN_MAX];
      bridge.GetCommand(&cmd, payload);
    }
#endif
#if (defined DEVICE_HAS_IN)
    if (Setup.Tx.ChannelsSource == CHANNEL_SOURCE_INPORT) {
      // update channels
      if (in.Update(&rcData)) {
        channelOrder.Apply(&rcData);
      }
    }
#endif
#ifdef USE_CRSF
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
      if (crsf.IsChannelData() && (Setup.Tx.ChannelsSource == CHANNEL_SOURCE_CRSF)) {
        fill_rcdata_from_crsf(&rcData, crsf.frame);
        channelOrder.Apply(&rcData);
      }
    }
#endif


  }//end of while(1) loop

}//end of main

