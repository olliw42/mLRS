//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// mLRS RX
/********************************************************

v0.0.00:
- in sync do not try to catch 1.5 fhss sequence, we rely entirely on syncword and crc to identify valid packet
  we now just expect 5 consequent packets, should we be happy with just one, as it was before?

TODO:
effect of USE_DCDC? where to place it??

can one put now ReadfRame out of isr? would allow using one spi for two sx

should we do a delay between rxdone and transmitting?
technically yes, but it seems no need to explicitly do it

in listen, cycle frequency slowly

retransmissions
*/

#define DBG_MAIN(x)
#define DBG_MAIN_SLIM(x)
#define DBG_STATUS(x)


// we set the priorities here to have an overview
#define CLOCK_IRQ_PRIORITY          10
#define SX_DIO1_EXTI_IRQ_PRIORITY   11
#define UARTB_IRQ_PRIORITY          15 // serial
#define UARTC_IRQ_PRIORITY          15
#define UART_IRQ_PRIORITY           14 // SBus out pin

#include "..\Common\common_conf.h"
#include "..\Common\common_types.h"
#include "..\Common\hal\glue.h"
#include "..\modules\stm32ll-lib\src\stdstm32.h"
#include "..\modules\stm32ll-lib\src\stdstm32-peripherals.h"
#include "..\modules\sx12xx-lib\src\sx128x.h"
#include "..\Common\hal\hal.h"
#include "..\modules\stm32ll-lib\src\stdstm32-delay.h"
#include "..\modules\stm32ll-lib\src\stdstm32-spi.h"
#include "..\modules\stm32ll-lib\src\stdstm32-uartb.h"
#include "..\modules\stm32ll-lib\src\stdstm32-uartc.h"
#include "..\modules\stm32ll-lib\src\stdstm32-uart.h"
#define FASTMAVLINK_IGNORE_WADDRESSOFPACKEDMEMBER
#include "..\Common\mavlink\out\storm32\storm32.h"
#include "..\Common\common.h"
//#include "..\Common\test.h" // un-comment if you want to compile for board test

#include "clock.h"
#include "out.h"
#include "rxstats.h"


ClockBase clock;


class Out : public OutBase
{
public:
  void Init(void)
  {
    OutBase::Init();
    out_init_gpio();
    uart_init_isroff();
  }

  void config_sbus(void) override
  {
    uart_setprotocol(100000, XUART_PARITY_EVEN, UART_STOPBIT_2);
    out_set_inverted();
    uart_rx_enableisr(ENABLE);
  }

  void putc(char c) override { uart_putc(c); }
};

Out out;


void init(void)
{
  leds_init();

  delay_init();
  serial.Init(); //uartb_setprotocol(SETUP_RX_SERIAL_BAUDRATE, XUART_PARITY_NO, UART_STOPBIT_1);
  out.Init();

  uartc_init();

  clock.Init();
  doPostReceive = false;

  sx.Init();
}


//-------------------------------------------------------
// Statistics for Receiver
//-------------------------------------------------------

static inline bool connected(void);

class RxStats : public RxStatsBase
{
  bool is_connected(void) override { return connected(); }
};

RxStats rxstats;


//-------------------------------------------------------
// SX1280
//-------------------------------------------------------

volatile uint16_t irq_status;

IRQHANDLER(
void SX_DIO1_EXTI_IRQHandler(void)
{
  LL_EXTI_ClearFlag_0_31(SX_DIO1_EXTI_LINE_x);
  //LED_RED_TOGGLE;
  irq_status = sx.GetIrqStatus();
  sx.ClearIrqStatus(SX1280_IRQ_ALL);
  if (irq_status & SX1280_IRQ_RX_DONE) {
    sx.ReadFrame((uint8_t*)&txFrame, FRAME_TX_RX_LEN);
  }
})


typedef enum {
    CONNECT_STATE_LISTEN = 0,
    CONNECT_STATE_SYNC,
    CONNECT_STATE_CONNECTED,
} CONNECT_STATE_ENUM;

typedef enum {
    LINK_STATE_RECEIVE = 0,
    LINK_STATE_RECEIVE_WAIT,
    LINK_STATE_TRANSMIT,
    LINK_STATE_TRANSMIT_WAIT,
} LINK_STATE_ENUM;

#define IS_RECEIVE_STATE    (link_state == LINK_STATE_RECEIVE || link_state == LINK_STATE_RECEIVE_WAIT)
#define IS_TRANSMIT_STATE   (link_state == LINK_STATE_TRANSMIT || link_state == LINK_STATE_TRANSMIT_WAIT)


uint8_t payload[FRAME_RX_PAYLOAD_LEN] = {0};
uint8_t payload_len = 0;


void process_transmit_frame(uint8_t ack)
{
  // read data from serial
  if (connected()) {
    memset(payload, 0, FRAME_TX_PAYLOAD_LEN);
    payload_len = 0;

    for (uint8_t i = 0; i < FRAME_RX_PAYLOAD_LEN; i++) {
      if (!serial.available()) break;
      payload[payload_len] = serial.getc();
      payload_len++;
    }

    stats.AddBytesTransmitted(payload_len);
  } else {
    serial.flush();
    memset(payload, 0, FRAME_TX_PAYLOAD_LEN);
    payload_len = 0;
  }

  tFrameStats frame_stats;
  frame_stats.seq_no = stats.transmit_seq_no;
  frame_stats.ack = ack;
  frame_stats.antenna = ANTENNA_1;
  frame_stats.rssi = rxstats.GetRssi();
  frame_stats.LQ = rxstats.GetLQ();
  frame_stats.LQ_serial_data = rxstats.GetLQ_serial_data();

  pack_rx_frame(&rxFrame, &frame_stats, payload, payload_len);
  sx.SendFrame((uint8_t*)&rxFrame, FRAME_TX_RX_LEN, 10); // 10ms tmo
}


void process_received_frame(bool do_payload)
{
  stats.received_antenna = txFrame.status.antenna;
  stats.received_rssi = -(txFrame.status.rssi_u7);
  stats.received_LQ = txFrame.status.LQ;
  stats.received_LQ_serial_data = txFrame.status.LQ_serial_data;

  // copy rc data
  if (!do_payload) {
    // copy only channels 1-4, and jump out
    rcdata_ch0to3_from_txframe(&rcData, &txFrame);
    return;
  }

  rcdata_from_txframe(&rcData, &txFrame);

  // output data on serial, but only if connected
  if (connected()) {
    for (uint8_t i = 0; i < txFrame.status.payload_len; i++) {
      uint8_t c = txFrame.payload[i];
      serial.putc(c); // send to serial
      DBG_MAIN(uartc_putc(c);)
    }

    stats.AddBytesReceived(txFrame.status.payload_len);
  }

  DBG_MAIN(uartc_puts("got "); uartc_puts(": ");
  uartc_putc('\n');)
}


void do_transmit(void) // we send a RX frame to transmitter
{
  uint8_t ack = 0;

  stats.transmit_seq_no++;

  process_transmit_frame(ack);
}


bool do_receive(void) // we receive a TX frame from receiver
{
bool ok = false;

  uint8_t res = check_tx_frame(&txFrame);
  if (res) {
    DBG_MAIN(uartc_puts("fail "); uartc_putc('\n');)
uartc_puts("fail "); uartc_puts(u8toHEX_s(res));uartc_putc('\n');
  }

  if (res == CHECK_OK || res == CHECK_ERROR_CRC) {
    clock.Reset();

    bool do_payload = (res == CHECK_OK);

    process_received_frame(do_payload);

    rxstats.doValidCrc1FrameReceived();
    if (res == CHECK_OK) rxstats.doValidFrameReceived();

    stats.last_received_seq_no = txFrame.status.seq_no;
    stats.last_received_ack = txFrame.status.ack;

    ok = true;
  } else {
    stats.last_received_seq_no = UINT8_MAX;
  }

  if (res != CHECK_ERROR_SYNCWORD) {
    // read it here, we want to have it even if it's a bad packet, but it should be for us
    sx.GetPacketStatus(&stats.last_rx_rssi, &stats.last_rx_snr);

    // we count all received frames, which are at least for us
    rxstats.doFrameReceived();
  }

  return ok;
}


void rescue(uint8_t t_ms)
{
  gpio_low(SX_RESET);
  delay_us(50); // 10 us works, play it safe
  gpio_high(SX_RESET);
  delay_ms(t_ms); // 2ms works, 1ms doesn't, play it safe and give it 5ms

  sx.Configure();
}


//##############################################################################################################
//*******************************************************
// MAIN routine
//*******************************************************

uint16_t led_blink;
uint16_t tick_1hz;

uint8_t link_state;
uint8_t connect_state;
uint16_t connect_tmo_cnt;
uint8_t connect_sync_cnt;
uint8_t connect_listen_cnt;
bool connect_occured_once;

uint32_t link_rescue_cnt; // rescue in state transmit


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
  fhss.StartRx();
  fhss.HopToConnect();
  sx.SetRfFrequency(fhss.GetCurrFreq());

//  for (uint8_t i = 0; i < fhss.Cnt(); i++) {
//    uartc_puts("c = "); uartc_puts(u8toBCD_s(fhss.ch_list[i])); uartc_puts(" f = "); uartc_puts(u32toBCD_s(fhss.fhss_list[i])); uartc_puts("\n"); delay_ms(50);
//  }

  link_state = LINK_STATE_RECEIVE;
  connect_state = CONNECT_STATE_LISTEN;
  connect_tmo_cnt = 0;
  connect_listen_cnt = 0;
  connect_sync_cnt = 0;
  connect_occured_once = false;
  link_rescue_cnt = 0;

  rxstats.Init(LQ_AVERAGING_PERIOD);

  out.Configure(OUT_CONFIG_SBUS);

  led_blink = 0;
  tick_1hz = 0;
  doSysTask = 0; // helps in avoiding too short first loop
  while (1) {

    if (doSysTask) {
      doSysTask = 0;

      if (connect_tmo_cnt) {
        connect_tmo_cnt--;
      }

      DECc(tick_1hz, SYSTICK_DELAY_MS(1000));
      if (connected()) {
        DECc(led_blink, SYSTICK_DELAY_MS(500));
      } else
      if (IS_RECEIVE_STATE) {
        DECc(led_blink, SYSTICK_DELAY_MS(200));
      } else {
        DECc(led_blink, SYSTICK_DELAY_MS(50));
      }

      if (!led_blink) {
        if (connected()) LED_GREEN_TOGGLE; else LED_RED_TOGGLE;
      }
      if (connected()) { LED_RED_OFF; } else { LED_GREEN_OFF; }

      if (!connected()) stats.Clear();

      if (!tick_1hz) {
        rxstats.Update1Hz();

        uartc_puts("RX: ");
//broken        uartc_puts(u8toBCD_s(rxstats.GetRawLQ())); uartc_putc(',');
        uartc_puts(u8toBCD_s(stats.GetLQ()));
        uartc_puts(" (");
        uartc_puts(u8toBCD_s(stats.LQ_frames_received)); uartc_putc(',');
        uartc_puts(u8toBCD_s(stats.LQ_valid_crc1_received)); uartc_putc(',');
        uartc_puts(u8toBCD_s(stats.LQ_valid_frames_received));
        uartc_puts("),");
        uartc_puts(u8toBCD_s(stats.received_LQ)); uartc_puts(", ");

        uartc_puts(s8toBCD_s(stats.last_rx_rssi)); uartc_putc(',');
        uartc_puts(s8toBCD_s(stats.received_rssi)); uartc_puts(", ");
        uartc_puts(s8toBCD_s(stats.last_rx_snr)); uartc_puts("; ");

        uartc_puts(u16toBCD_s(stats.bytes_per_sec_transmitted)); uartc_puts(", ");
        uartc_puts(u16toBCD_s(stats.bytes_per_sec_received)); uartc_puts("; ");
        uartc_putc('\n');
      }
    }

    switch (link_state) {
    case LINK_STATE_RECEIVE: {
      DBG_STATUS(uint8_t status = sx.GetStatus();
      if ((status & SX1280_STATUS_MODE_MASK) != SX1280_STATUS_MODE_FS) {
        uartc_puts("$$"); uartc_puts(u8toHEX_s(status>>5));
        break;
      })
      if (connect_state == CONNECT_STATE_LISTEN) {
        fhss.HopToConnect();
      } else {
        fhss.HopToNext();
      }
      sx.SetRfFrequency(fhss.GetCurrFreq());
      sx.SetToRx(0); // single without tmo
      link_state = LINK_STATE_RECEIVE_WAIT;
      DBG_STATUS(status = sx.GetStatus();
      if ((status & SX1280_STATUS_MODE_MASK) != SX1280_STATUS_MODE_RX) {
        uartc_puts("$"); uartc_puts(u8toHEX_s(status>>5));
      })
      DBG_MAIN_SLIM(uartc_puts(">");)
      }break;

    case LINK_STATE_TRANSMIT: {
      DBG_STATUS(uint8_t status = sx.GetStatus();
      if ((status & SX1280_STATUS_MODE_MASK) != SX1280_STATUS_MODE_FS) {
        uartc_puts("§§"); uartc_puts(u8toHEX_s(status>>5));
        break;
      })
      do_transmit();
      DBG_STATUS(status = sx.GetStatus();
      if ((status & SX1280_STATUS_MODE_MASK) != SX1280_STATUS_MODE_TX) {
        uartc_puts("§"); uartc_puts(u8toHEX_s(status>>5));
        rescue(4);
        uartc_puts(u8toHEX_s(sx.GetStatus()>>5));
        // rescue takes a couple of ms, so we miss sending this packet
        link_state = LINK_STATE_RECEIVE;
        link_rescue_cnt++;
        break;
      })
      link_state = LINK_STATE_TRANSMIT_WAIT;
      }break;
    }

    if (irq_status) {
      if (link_state == LINK_STATE_RECEIVE_WAIT) {
        if (irq_status & SX1280_IRQ_RX_DONE) {
          irq_status = 0;
          if (do_receive()) { // also calls clock.Reset() if valid packet
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
            connect_occured_once = true;
            link_state = LINK_STATE_TRANSMIT; // switch to TX
          } else {
            // we received something, but something wrong, so we need go back to RX
            if (connect_state == CONNECT_STATE_LISTEN) {
              link_state = LINK_STATE_RECEIVE;
            }
          }
          DBG_MAIN_SLIM(uartc_puts("!");)
        }
      } else
      if (link_state == LINK_STATE_TRANSMIT_WAIT) {
        if (irq_status & SX1280_IRQ_TX_DONE) {
          irq_status = 0;
          link_state = LINK_STATE_RECEIVE; // switch back to RX
          DBG_MAIN_SLIM(uartc_puts("<\n");)
        }
      }

      if (irq_status & SX1280_IRQ_RX_DONE) {
        LED_GREEN_OFF;
        while (1) { LED_RED_ON; delay_ms(5); LED_RED_OFF; delay_ms(5); }
      }
      if (irq_status & SX1280_IRQ_TX_DONE) {
        LED_RED_OFF;
        while (1) { LED_GREEN_ON; delay_ms(5); LED_GREEN_OFF; delay_ms(5); }
      }
      if (irq_status & SX1280_IRQ_RX_TX_TIMEOUT) {
        while (1) { LED_RED_ON; LED_GREEN_ON; delay_ms(50); LED_RED_OFF; LED_GREEN_OFF; delay_ms(50); }
      }
    }

    // this happens ca 1 ms after a frame was or should have been received
    if (doPostReceive) {
      doPostReceive = false;

      if (connect_state == CONNECT_STATE_LISTEN) {
        connect_listen_cnt++;
        if (connect_listen_cnt >= CONNECT_LISTEN_HOP_CNT) {
          fhss.HopToNext();
          connect_listen_cnt = 0;
          link_state = LINK_STATE_RECEIVE; // switch back to RX
        }
      }

      // we just disconnected, or are in sync but don't receive anything
      if ((connect_state >= CONNECT_STATE_SYNC) && !connect_tmo_cnt) {
        // switch to listen state
        // only do it if not in listen, since otherwise it never could reach receive wait and hence never could connect
        connect_state = CONNECT_STATE_LISTEN;
        connect_listen_cnt = 0;
        link_state = LINK_STATE_RECEIVE; // switch back to RX
      }

      rxstats.Next();

      // we missed the receive frame
      bool missed = false;
      if ((connect_state >= CONNECT_STATE_SYNC) && !IS_TRANSMIT_STATE) {
        missed = true;

        // reset sync counter, relevant if in sync
        connect_sync_cnt = 0;

        // switch to transmit state
        // only do it if receiving, else keep it in RX mode, otherwise chances to connect are dim
        // we are on the correct frequency, so no need to hop
        sx.SetFs();
        link_state = LINK_STATE_TRANSMIT;
      }

      out.SetChannelOrder(SETUP_RX_CHANNEL_ORDER);
      if (connected()) {
        out.SendRcData(&rcData, missed, false);
      } else {
#if SETUP_RX_FAILSAFE_MODE == 1
        if (connect_occured_once) {
          tRcData rc;
          memcpy(&rc, &rcData, sizeof(tRcData));
          for (uint8_t n = 0; n < 3; n++) rc.ch[n] = 1024;
          out.SendRcData(&rc, true, true);
        }
#else
        // no signal, so do nothing
#endif
      }
/*
      static uint16_t tlast_10us = 0;
      uint16_t tnow_10us = clock.tim_10us();
      uint16_t dt = tnow_10us - tlast_10us;
      tlast_10us = tnow_10us;

      uartc_puts(" ");
      uartc_puts(u16toBCD_s(tnow_10us)); uartc_puts(", "); uartc_puts(u16toBCD_s(dt)); uartc_puts("; ");
      uartc_puts(missed ? "m " : "o ");
      switch (link_state) {
      case LINK_STATE_TRANSMIT: uartc_puts("t  "); break;
      case LINK_STATE_TRANSMIT_WAIT: uartc_puts("tw "); break;
      case LINK_STATE_RECEIVE: uartc_puts("r  "); break;
      case LINK_STATE_RECEIVE_WAIT: uartc_puts("rw "); break;
      }
      switch (connect_state) {
      case CONNECT_STATE_LISTEN: uartc_puts("L "); break;
      case CONNECT_STATE_SYNC: uartc_puts("S "); break;
      case CONNECT_STATE_CONNECTED: uartc_puts("C "); break;
      }
      uartc_puts(connected() ? "c " : "d ");
      uartc_puts("\n");
*/
    }

  }//end of while(1) loop

}//end of main

