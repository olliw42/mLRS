//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
//
/********************************************************

v0.0.00:

TODO:
we need multiple "connected" time scales
- for deciding to output sbus
- for deciding when pll locks and unlocks, so it re-locks
- for deciding when to stay in RX, i.e, we shouldn't send TX if we want to reconnect
- for deciding when to try a "fast" reconnect
- for deciding when to go back to "full" connect
- for deciding if it got a signal
- for deciding when/how to calculate LQ
??

effect of USE_DCDC? where to place it??

can one put now ReadfRame out of isr? would allow using one spi for two sx

should we do a delay between rxdone and transmitting?
technically yes, but it seems no need to be done explicitly
*/

#define DBG_MAIN(x)
#define DBG_MAIN_SLIM(x)
#define DBG_MAIN_FULL(x)
#define DBG_STATUS(x)


// we set the priorities here to have an overview
#define CLOCK_IRQ_PRIORITY          10
#define SX_DIO1_EXTI_IRQ_PRIORITY   11
#define UARTB_IRQ_PRIORITY          15 // serial
#define UARTC_IRQ_PRIORITY          15
#define UART_IRQ_PRIORITY           14 // SBus out pin

#include "..\Common\common_conf.h"
#include "..\Common\hal\glue.h"
#include "..\modules\stm32ll-lib\src\stdstm32.h"
#include "..\modules\stm32ll-lib\src\stdstm32-peripherals.h"
#include "..\Common\hal\hal.h"
#include "..\modules\stm32ll-lib\src\stdstm32-delay.h"
#include "..\modules\stm32ll-lib\src\stdstm32-spi.h"
#include "..\modules\sx12xx-lib\src\sx128x.h"
#include "..\modules\stm32ll-lib\src\stdstm32-uartb.h"
#include "..\modules\stm32ll-lib\src\stdstm32-uartc.h"
#include "..\modules\stm32ll-lib\src\stdstm32-uart.h"
#define FASTMAVLINK_IGNORE_WADDRESSOFPACKEDMEMBER
#include "..\Common\mavlink\out\storm32\storm32.h"
#include "..\Common\common.h"

#include "clock.h"
#include "pll.h"
#include "out.h"
#include "rxstats.h"


class Pll : public PllBase
{
public:
  uint16_t tim_10us(void) override { return CLOCK_TIMx->CNT; }
  void tim_set_period_10us(uint16_t period_10us) override { clock_oc_period = period_10us; }
};


Pll pll;
volatile bool do_post_receive;
ClockBase clock;

// PLL:
// we lock on RxDone
// that is, the events pll.tick() and pll.rxdone() are brought into sync
// pll.update() is called in middle of cycle and does the pll update
// do_post_receive is triggered 1 ms after pll.tick(), which is ca 1 ms
// after a packet was or was supposed to be received

IRQHANDLER(
void CLOCK_IRQHandler(void)
{
  if (LL_TIM_IsActiveFlag_CC1(CLOCK_TIMx)) { // this is at about when RX was or was supposed to be received
    LL_TIM_ClearFlag_CC1(CLOCK_TIMx);
    CLOCK_TIMx->CCR2 = CLOCK_TIMx->CCR1 + (clock_oc_period/2);
    CLOCK_TIMx->CCR3 = CLOCK_TIMx->CCR1 + CLOCK_OC3_SHIFT_10US;
    CLOCK_TIMx->CCR1 = CLOCK_TIMx->CCR1 + clock_oc_period;
    pll.tick();
    //LED_GREEN_ON;
  }
  if (LL_TIM_IsActiveFlag_CC2(CLOCK_TIMx)) { // pll update must be in the middle of the cycle
    LL_TIM_ClearFlag_CC2(CLOCK_TIMx);
    pll.update();
    //LED_GREEN_OFF;
  }
  if (LL_TIM_IsActiveFlag_CC3(CLOCK_TIMx)) { // this is 1 ms after RX was or was supposed to be received
    LL_TIM_ClearFlag_CC3(CLOCK_TIMx);
    do_post_receive = true;
  }
})


class Out : public OutBase
{
public:
  void Init(void)
  {
    OutBase::Init();
    out_init_gpio();
    uart_init_isroff();
  }

  virtual void config_sbus(void) override
  {
    uart_setprotocol(100000, XUART_PARITY_EVEN, UART_STOPBIT_2);
    out_set_inverted();
    uart_rx_enableisr(ENABLE);
  }

  virtual void putc(char c) override { uart_putc(c); }
};

Out out;


void init(void)
{
  leds_init();

  delay_init();
  serial.Init(); //uartb_setprotocol(SETUP_RX_SERIAL_BAUDRATE, XUART_PARITY_NO, UART_STOPBIT_1);
  out.Init();

  uartc_init();

  clock.InitIsrOff(FRAME_RATE_MS*1000);
  pll.Init(FRAME_RATE_MS*1000);
  clock.EnableIsr();
  do_post_receive = false;

  sx.Init();
}


//-------------------------------------------------------
// Statistics for Receiver
//-------------------------------------------------------
// we also handle the stats field with this class, this is somewhat dirty

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
  //pll.rx_done1(); // we need to check correctness of packed before accepting it
})


typedef enum {
    LINK_STATE_CONNECT = 0,

    LINK_STATE_RECEIVE,
    LINK_STATE_RECEIVE_WAIT,
    LINK_STATE_TRANSMIT,
    LINK_STATE_TRANSMIT_WAIT,
} LINK_STATE_ENUM;

#define IS_CONNECT_STATE   (link_state == LINK_STATE_CONNECT)
#define IS_RECEIVE_STATE   (link_state == LINK_STATE_RECEIVE || link_state == LINK_STATE_RECEIVE_WAIT)
#define IS_TRANSMIT_STATE  (link_state == LINK_STATE_TRANSMIT || link_state == LINK_STATE_TRANSMIT_WAIT)

static inline bool connected(void);


void do_transmit(bool set_ack) // we send a RX frame to transmitter
{
  uint8_t payload[FRAME_RX_PAYLOAD_LEN] = {0};
  uint8_t payload_len = 0;

  for (uint8_t i = 0; i < FRAME_RX_PAYLOAD_LEN; i++) {
    if (!serial.available()) break;
    payload[payload_len] = serial.getc();
    payload_len++;
  }

  tFrameStats frame_stats;
  frame_stats.seq_no = stats.tx_seq_no; stats.tx_seq_no++;
  frame_stats.ack = (set_ack) ? 1 : 0;
  frame_stats.rssi = stats.last_rx_rssi;
  frame_stats.snr = stats.last_rx_snr;
  frame_stats.LQ = rxstats.GetLQ(connected());

  pack_rx_frame(&rxFrame, &frame_stats, payload, payload_len);
  sx.SendFrame((uint8_t*)&rxFrame, FRAME_TX_RX_LEN, 10); // 10ms tmo
}


uint8_t check_received_frame(void) // we received a TX frame from transmitter, 0: fail, 1: crc1 ok, 2: full ok
{
  uint16_t err = check_tx_frame(&txFrame);

  if (err) {
    DBG_MAIN(uartc_puts("fail "); uartc_putc('\n');)
uartc_puts("fail "); uartc_puts(u16toHEX_s(err));uartc_putc('\n');

    if (err == 0x0010) { // everything except full crc is ok
      return 1;
    }
    return 0;
  }
  return 2;
}


// process what we have received
void process_received_frame(bool full)
{
  DBG_MAIN(char s[16];
  uartc_puts("got "); uartc_puts(s); uartc_puts(": ");)

  stats.received_rssi = -(txFrame.status.rssi_u8);
  stats.received_LQ = txFrame.status.LQ;

  stats.received_seq_no = txFrame.status.seq_no;
  stats.received_ack = txFrame.status.ack;

  // copy rc data
  if (!full) {
    rcdata_ch0to3_from_txframe(&rcData, &txFrame);
    return;
  }
  rcdata_from_txframe(&rcData, &txFrame);

  // forward data to serial
  for (uint8_t i = 0; i < txFrame.status.payload_len; i++) {
    serial.putc(txFrame.payload[i]);
    DBG_MAIN(uartc_putc(txFrame.payload[i]);)
  }

  DBG_MAIN(uartc_putc('\n');)
}


bool do_receive(void)
{
bool ok = false;

  uint8_t res = check_received_frame(); // returns 0, 1, 2

  if (res > 0) {
    pll.rx_done(); // we need to check correctness of packed before accepting this
    bool full = (res > 1);

    process_received_frame(full);

    rxstats.doValidCrc1FrameReceived();
    if (full) rxstats.doValidFrameReceived();

    ok = true;
  }

  // read it here, we want to have it even if it's a bad packet, but we want to do it after pll.rx_done();
  sx.GetPacketStatus(&stats.last_rx_rssi, &stats.last_rx_snr);

  // we count all received frames
  rxstats.doFrameReceived();

  return ok;
}


void rescue(uint8_t t_ms)
{
  gpio_low(SX_RESET);
  delay_us(50); // 10 us works, play it safe
  gpio_high(SX_RESET);
  delay_ms(t_ms); // 2ms works, 1ms doesn't, play it safe and give it 5ms

  sx.Configure();

  //sx.SetRfFrequency(...) do in main
}


//##############################################################################################################
//*******************************************************
// MAIN routine
//*******************************************************

uint16_t led_blink;
uint16_t tick_1hz;

uint8_t link_state;
uint16_t connected_tmo_cnt;

uint32_t link_rescue_cnt; // rescue in state transmit


// receiving means:
//   we are receiving valid frames, but we may miss some occasionally
//   the receiving flag is lost when too many (by whatever criterion) frames are missed
// connected means:
//   we are receiving, but in addition further health conditions are met
//   only when connected we send out e.g. pulses on the sbus pin, etc

static inline bool receiving(void)
{
  return (connected_tmo_cnt > 0);
}

static inline bool connected(void)
{
  return (connected_tmo_cnt > 0) && (pll.locked);
}


int main_main(void)
{
  init();

  DBG_MAIN(uartc_puts("\n\n\nHello\n\n");)

  // startup sign of life
  LED_RED_OFF;
  for (uint8_t i = 0; i < 7; i++) { LED_RED_TOGGLE; delay_ms(50); }

  // start up sx
  if (!sx.isOk()) {
    DBG_MAIN(uartc_puts("fail\n");)
    while (1) { LED_RED_TOGGLE; delay_ms(50); } // fail!
  } else {
    DBG_MAIN(uartc_puts("ok\n");)
  }
  sx.StartUp();
  fhss.Init(SEEDDBLWORD);
  fhss.StartRx();
  sx.SetRfFrequency(fhss.GetCurr());

//  for (uint8_t n = 0; n < FHSS_MAX_NUM; n++) {
//    uartc_puts("f = "); uartc_puts(u32toBCD_s(fhss.fhss_list[n])); uartc_puts("\n");
//  }

  link_state = LINK_STATE_CONNECT; //RECEIVE;
  connected_tmo_cnt = 0;
  link_rescue_cnt = 0;
  rxstats.Init(LQ_AVERAGING_PERIOD);

  out.Configure(OUT_CONFIG_SBUS);

  led_blink = 0;
  tick_1hz = 0;
  doSysTask = 0; // helps in avoiding too short first loop
  while (1) {

    if (doSysTask) {
      doSysTask = 0;

      if (connected_tmo_cnt) {
        connected_tmo_cnt--;
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

      if (!tick_1hz) {
        rxstats.Update1Hz();
        //uartc_puts(" . ");

        uartc_puts("RX: ");
        uartc_puts(u8toBCD_s(rxstats.GetRawLQ())); uartc_putc(',');
        uartc_puts(u8toBCD_s(stats.rx_LQ));
        uartc_puts(" (");
        uartc_puts(u8toBCD_s(stats.LQ_received)); uartc_putc(',');
        uartc_puts(u8toBCD_s(stats.LQ_valid_crc1_received)); uartc_putc(',');
        uartc_puts(u8toBCD_s(stats.LQ_valid_received));
        uartc_puts("),");
        uartc_puts(u8toBCD_s(stats.received_LQ)); uartc_puts(", ");

        uartc_puts(s8toBCD_s(stats.last_rx_rssi)); uartc_putc(',');
        uartc_puts(s8toBCD_s(stats.received_rssi)); uartc_puts(", ");
        uartc_puts(s8toBCD_s(stats.last_rx_snr));
        uartc_putc('\n');

      }
    }

    switch (link_state) {
    case LINK_STATE_CONNECT:
    case LINK_STATE_RECEIVE: {
DBG_STATUS(uint8_t status = sx.GetStatus();
      if ((status & SX1280_STATUS_MODE_MASK) != SX1280_STATUS_MODE_FS) {
        uartc_puts("$$"); uartc_puts(u8toHEX_s(status>>5));
        break;
      })
      //if (link_state == LINK_STATE_RECEIVE) fhss.HopToNext();
      fhss.HopToNext();
      sx.SetRfFrequency(fhss.GetCurr());
      rxstats.Clear();
      sx.SetToRx(0); // single without tmo
      link_state = LINK_STATE_RECEIVE_WAIT;
DBG_STATUS(status = sx.GetStatus();
      if ((status & SX1280_STATUS_MODE_MASK) != SX1280_STATUS_MODE_RX) {
        uartc_puts("$"); uartc_puts(u8toHEX_s(status>>5));
      })
      DBG_MAIN_FULL(uartc_puts("RX: rx\n");)
      DBG_MAIN_SLIM(uartc_puts(">");)
      }break;

    case LINK_STATE_TRANSMIT: {
DBG_STATUS(uint8_t status = sx.GetStatus();
      if ((status & SX1280_STATUS_MODE_MASK) != SX1280_STATUS_MODE_FS) {
        uartc_puts("!!"); uartc_puts(u8toHEX_s(status>>5));
        break;
      })
      do_transmit(false);
DBG_STATUS(status = sx.GetStatus();
      if ((status & SX1280_STATUS_MODE_MASK) != SX1280_STATUS_MODE_TX) {
        uartc_puts("!"); uartc_puts(u8toHEX_s(status>>5));
        rescue(4);
        uartc_puts(u8toHEX_s(sx.GetStatus()>>5));
        // rescue takes a couple of ms, so we miss sending this packet
        link_state = LINK_STATE_RECEIVE;
        link_rescue_cnt++;
        break;
      })
      link_state = LINK_STATE_TRANSMIT_WAIT;
      DBG_MAIN_FULL(uartc_puts("RX: tx\n");)
      }break;
    }

    if (irq_status) {
      if (link_state == LINK_STATE_RECEIVE_WAIT) {
        if (irq_status & SX1280_IRQ_RX_DONE) {
          irq_status = 0;
          if (do_receive()) {
            connected_tmo_cnt = CONNECT_TMO_SYSTICKS;
            link_state = LINK_STATE_TRANSMIT; // switch to TX
          }
          DBG_MAIN_FULL(uartc_puts("RX: rx done\n");)
          DBG_MAIN_SLIM(uartc_puts("!");)
        }
      } else
      if (link_state == LINK_STATE_TRANSMIT_WAIT) {
        if (irq_status & SX1280_IRQ_TX_DONE) {
          irq_status = 0;
          link_state = LINK_STATE_RECEIVE; // switch back to RX
          DBG_MAIN_FULL(uartc_puts("RX: tx done\n");)
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
    if (do_post_receive) {
      do_post_receive = false;

      if (!IS_CONNECT_STATE) {
        rxstats.Next();
        rxstats.Set();
      }
      rxstats.Clear();

      if (connected() && !IS_TRANSMIT_STATE) { // only do this if connected, else keep it in RX mode, otherwise chances to connect are dim
        sx.SetFs();
        link_state = LINK_STATE_TRANSMIT;
      }

      if (connected()) {
        out.send_rcdata(&rcData);
      }
    }

/*
    if (pll.is_updated()) {
      uartc_puts(" ");
      uartc_puts(u8toBCD_s(pll.snap_idx)); uartc_puts(": ");
      uartc_puts(u16toBCD_s(pll.snap_tlast_rx_done)); uartc_puts(", ");
      uartc_puts(u16toBCD_s(pll.snap_tlast_tick)); uartc_puts(", ");
      uartc_puts(u16toBCD_s(pll.snap_period)); uartc_puts(", ");
      uartc_puts(s16toBCD_s(pll.snap_phase)); uartc_puts("; ");
      //uartc_puts(u16toBCD_s(pll.expected_period)); uartc_puts("; ");
      uartc_puts(u16toBCD_s(pll.phase_abs_filt_n/pll.phase_abs_filt_fact)); uartc_puts("; ");
      uartc_puts(u8toBCD_s(pll.obtained)); uartc_puts(", ");
      uartc_puts(u8toBCD_s(pll.missed)); uartc_puts("; ");
//      uartc_puts(pll.locked ? "l " : "u ");
//      uartc_puts(connected() ? "c " : "d ");
//      uartc_puts(u8toBCD_s(rxstats.GetRawLQ())); uartc_puts("; ");
      if (pll.snap_phase == INT16_MAX)  uartc_puts(" !!");
      uartc_puts("\n");
    }
*/

  }//end of while(1) loop

}//end of main

