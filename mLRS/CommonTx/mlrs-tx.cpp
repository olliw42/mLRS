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
#include "..\modules\stm32ll-lib\src\stdstm32-eeprom.h"
#include "..\modules\stm32ll-lib\src\stdstm32-spi.h"
#ifdef DEVICE_HAS_DIVERSITY
#include "..\modules\stm32ll-lib\src\stdstm32-spib.h"
#endif
#ifdef USE_SERIAL
#include "..\modules\stm32ll-lib\src\stdstm32-uartb.h"
#endif
#if defined USE_COM_ON_C || defined USE_DEBUG
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
#include "cli.h"


TxStatsBase txstats;


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


tComPort com;

tTxCli cli;

tSerialBase* serialport;


void init(void)
{
    leds_init();
    button_init();
    pos_switch_init();

    delay_init();
    micros_init();

    bind.Init();
    serial.Init();
    in.Init();

    com.Init();
    cli.Init(&com);
    dbg.Init();

    setup_init();

    sx.Init(); // sx needs Config, so call after setup_init()
    sx2.Init();
}


//-------------------------------------------------------
// mbridge
//-------------------------------------------------------

uint8_t mavlink_vehicle_state(void);

#include "mbridge_interface.h" // this includes uart.h as it needs callbacks
#include "crsf_interface.h" // this includes uart.h as it needs callbacks


//-------------------------------------------------------
// mavlink
//-------------------------------------------------------

#include "mavlink_interface_tx.h"

MavlinkBase mavlink;


uint8_t mavlink_vehicle_state(void)
{
    return mavlink.VehicleState();
}


void init_serialport(void)
{
    switch (Setup.Tx.SerialDestination) {
    case SERIAL_DESTINATION_MBRDIGE:
        serialport = &mbridge;
        break;
    case SERIAL_DESTINATION_SERIAL_PORT:
        serialport = &serial;
        break;
    default:
        serialport = nullptr;
    }
}


//-------------------------------------------------------
// While transmit/receive tasks
//-------------------------------------------------------
// we may want to add some timer to do more than one task in the transmit/receive period
// this would help a lot with the different available periods depending on the mode

#include "..\Common\while.h"

WhileTransmit whileTransmit;


void WhileTransmit::handle(void)
{
    cli.Do();
}


void WhileTransmit::handle_tasks(void)
{
}


//-------------------------------------------------------
// SX12xx
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
        if (bind.IsInBind()) {
            uint64_t bind_signature;
            sx.ReadBuffer(0, (uint8_t*)&bind_signature, 8);
            if (bind_signature != bind.RxSignature) irq_status = 0; // not binding frame, so ignore it
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
    LL_EXTI_ClearFlag_0_31(SX2_DIO_EXTI_LINE_x);
    //LED_RIGHT_RED_TOGGLE;
    irq2_status = sx2.GetAndClearIrqStatus(SX12xx_IRQ_ALL);
    if (irq2_status & SX12xx_IRQ_RX_DONE) {
        if (bind.IsInBind()) {
            uint64_t bind_signature;
            sx2.ReadBuffer(0, (uint8_t*)&bind_signature, 8);
            if (bind_signature != bind.RxSignature) irq2_status = 0;
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
uint16_t link_task_tmo;
bool doParamsStore;


void link_task_init(void)
{
    link_task = LINK_TASK_NONE;
    link_task_tmo = 0;
    transmit_frame_type = TRANSMIT_FRAME_TYPE_NORMAL;

    doParamsStore = false;
}


bool link_task_set(uint8_t task)
{
    if (link_task != LINK_TASK_NONE) return false; // a task is running

    link_task = task;
    transmit_frame_type = TRANSMIT_FRAME_TYPE_CMD;

    // set a timeout if relevant
    link_task_tmo = 0;
    switch (link_task) {
    case LINK_TASK_TX_STORE_RX_PARAMS: // store rx parameters
        link_task_tmo = 6 * Config.frame_rate_ms;
        break;
    }

    return true;
}


void link_task_reset(void)
{
    link_task = LINK_TASK_NONE;
    link_task_tmo = 0;
    transmit_frame_type = TRANSMIT_FRAME_TYPE_NORMAL;
}


void link_task_tick_ms(void)
{
    // if a timeout has been set, count it down
    if (link_task_tmo) {
        link_task_tmo--;
        if (!link_task_tmo) {
            switch (link_task) {
            case LINK_TASK_TX_STORE_RX_PARAMS: doParamsStore = true; break;
            }
            link_task_reset();
            mbridge.Unlock();
        }
    }
}


void process_received_rxcmdframe(tRxFrame* frame)
{
tCmdFrameHeader* head = (tCmdFrameHeader*)(frame->payload);

    switch (head->cmd) {
    case FRAME_CMD_RX_SETUPDATA:
        // received rx setup data
        unpack_rxcmdframe_rxsetupdata(frame);
        link_task_reset();
        mbridge.Unlock();
        break;
  }
}


void pack_txcmdframe(tTxFrame* frame, tFrameStats* frame_stats, tRcData* rc)
{
    switch (link_task) {
    case LINK_TASK_TX_GET_RX_SETUPDATA:
        pack_txcmdframe_cmd(frame, frame_stats, rc, FRAME_CMD_GET_RX_SETUPDATA);
        break;
    case LINK_TASK_TX_SET_RX_PARAMS:
        pack_txcmdframe_setrxparams(frame, frame_stats, rc);
        break;
    case LINK_TASK_TX_STORE_RX_PARAMS:
        pack_txcmdframe_cmd(frame, frame_stats, rc, FRAME_CMD_STORE_RX_PARAMS);
        transmit_frame_type = TRANSMIT_FRAME_TYPE_NORMAL;
        break;
    }
}


//-- normal Tx, Rx frames handling

uint8_t payload[FRAME_TX_PAYLOAD_LEN] = {0};
uint8_t payload_len = 0;


void process_transmit_frame(uint8_t antenna, uint8_t ack)
{
    if (transmit_frame_type == TRANSMIT_FRAME_TYPE_NORMAL) {
      memset(payload, 0, FRAME_TX_PAYLOAD_LEN);
      payload_len = 0;

      // read data from serial port
      if (connected()) {
        if (serialport) {
          for (uint8_t i = 0; i < FRAME_TX_PAYLOAD_LEN; i++) {
            if (Setup.Rx.SerialLinkMode == SERIAL_LINK_MODE_MAVLINK) {
              if (!mavlink.available()) break; // get from serial port via mavlink parser
              payload[payload_len] = mavlink.getc();
            } else {
              if (!serialport->available()) break; // get from serial port
              payload[payload_len] = serialport->getc();
            }
//dbg.putc(payload[payload_len]);
            payload_len++;
          }
        }

        stats.bytes_transmitted.Add(payload_len);
        stats.fresh_serial_data_transmitted.Inc();
      } else {
        if (serialport) mavlink.flush(); // we don't distinguish here, can't harm to always flush mavlink hanlder
      }
    }

    stats.last_tx_antenna = antenna;

    tFrameStats frame_stats;
    frame_stats.seq_no = stats.transmit_seq_no;
    frame_stats.ack = ack;
    frame_stats.antenna = stats.last_rx_antenna;
    frame_stats.transmit_antenna = antenna;
    frame_stats.rssi = stats.GetLastRxRssi();
    frame_stats.LQ = txstats.GetLQ();
    frame_stats.LQ_serial_data = txstats.GetLQ_serial_data();

    if (transmit_frame_type == TRANSMIT_FRAME_TYPE_NORMAL) {
        pack_txframe(&txFrame, &frame_stats, &rcData, payload, payload_len);
    } else {
        pack_txcmdframe(&txFrame, &frame_stats, &rcData);
    }
}


void process_received_frame(bool do_payload, tRxFrame* frame)
{
    stats.received_antenna = frame->status.antenna;
    stats.received_transmit_antenna = frame->status.transmit_antenna;
    stats.received_rssi = rssi_i8_from_u7(frame->status.rssi_u7);
    stats.received_LQ = frame->status.LQ;
    stats.received_LQ_serial_data = frame->status.LQ_serial_data;

    if (!do_payload) return;

    if (frame->status.frame_type == FRAME_TYPE_TX_RX_CMD) { //!= FRAME_TYPE_RX) {
        process_received_rxcmdframe(frame);
        return;
    }

    // output data on serial
    if (serialport) {
        for (uint8_t i = 0; i < frame->status.payload_len; i++) {
            uint8_t c = frame->payload[i];
            if (Setup.Tx.SerialLinkMode == SERIAL_LINK_MODE_MAVLINK) {
                mavlink.putc(c);
            } else {
              serialport->putc(c);
            }
//dbg.putc(c);
        }
    }

    stats.bytes_received.Add(frame->status.payload_len);
    stats.fresh_serial_data_received.Inc();
}


//-- receive/transmit handling api

void handle_receive(uint8_t antenna)
{
uint8_t rx_status;
tRxFrame* frame;

    if (bind.IsInBind()) {
        bind.handle_receive(antenna, (antenna == ANTENNA_1) ? link_rx1_status : link_rx2_status);
        return;
    }

    if (antenna == ANTENNA_1) {
        rx_status = link_rx1_status;
        frame = &rxFrame;
    } else {
        rx_status = link_rx2_status;
        frame = &rxFrame2;
    }

    if (rx_status != RX_STATUS_INVALID) { // RX_STATUS_CRC1_VALID, RX_STATUS_VALID
        bool do_payload = true;

        process_received_frame(do_payload, frame);

        txstats.doValidFrameReceived(); // should we count valid payload only if rx frame ?

        stats.received_seq_no_last = frame->status.seq_no;
        stats.received_ack_last = frame->status.ack;

    } else { // RX_STATUS_INVALID
        stats.received_seq_no_last = UINT8_MAX;
        stats.received_ack_last = 0;
    }

    // we set it for all received frames
    stats.last_rx_antenna = antenna;

    // we count all received frames
    txstats.doFrameReceived();
}


void handle_receive_none(void) // RX_STATUS_NONE
{
    stats.received_seq_no_last = UINT8_MAX;
    stats.received_ack_last = 0;
}


void do_transmit(uint8_t antenna) // we send a TX frame to receiver
{
uint8_t ack = 1;

    if (bind.IsInBind()) {
        bind.do_transmit(antenna);
        return;
    }

    stats.transmit_seq_no++;

    process_transmit_frame(antenna, ack);

    sxSendFrame(antenna, &txFrame, &txFrame2, FRAME_TX_RX_LEN, SEND_FRAME_TMO); // 10 ms tmo
}


uint8_t do_receive(uint8_t antenna) // we receive a RX frame from receiver
{
uint8_t res;
uint8_t rx_status = RX_STATUS_INVALID; // this also signals that a frame was received

    if (bind.IsInBind()) {
        return bind.do_receive(antenna, false);
    }

    // we don't need to read sx.GetRxBufferStatus(), but hey
    // we could save 2 byte's time by not reading sync_word again, but hey
    sxReadFrame(antenna, &rxFrame, &rxFrame2, FRAME_TX_RX_LEN);
    res = (antenna == ANTENNA_1) ? check_rxframe(&rxFrame) : check_rxframe(&rxFrame2);

    if (res) {
        DBG_MAIN(dbg.puts("fail ");dbg.putc('\n');)
//dbg.puts("fail a");dbg.putc(antenna+'0');dbg.puts(" ");dbg.puts(u8toHEX_s(res));dbg.putc('\n');
    }

    if (res == CHECK_ERROR_SYNCWORD) return false; // must not happen !

    if (res == CHECK_OK) {
        rx_status = RX_STATUS_VALID;
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

uint16_t tx_tick;
uint16_t tick_1hz_commensurate;
bool doPreTransmit;

uint16_t link_state;
uint8_t connect_state;
uint16_t connect_tmo_cnt;
uint8_t connect_sync_cnt;


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

  mbridge.Init(Config.UseMbridge);
  crsf.Init(Config.UseCrsf);
  serial.SetBaudRate(Config.SerialBaudrate);
  init_serialport();

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

  tx_tick = 0;
  doPreTransmit = false;
  link_state = LINK_STATE_IDLE;
  connect_state = CONNECT_STATE_LISTEN;
  connect_tmo_cnt = 0;
  connect_sync_cnt = 0;
  link_rx1_status = RX_STATUS_NONE;
  link_rx2_status = RX_STATUS_NONE;
  link_task_init();
  link_task_set(LINK_TASK_TX_GET_RX_SETUPDATA); // we start with wanting to get rx setup data

  txstats.Init(Config.LQAveragingPeriod);

  in.Configure(Setup.Tx.InMode);
  mavlink.Init();
  whileTransmit.Init();

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

      if (!tick_1hz) {
/*        dbg.puts("\nTX: ");
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
        dbg.puts(u16toBCD_s(stats.bytes_received.GetBytesPerSec())); dbg.puts("; "); */
      }

      DECc(tx_tick, SYSTICK_DELAY_MS(Config.frame_rate_ms));

      if (!tx_tick) {
        doPreTransmit = true; // trigger next cycle
        crsf.TelemetryStart();
      }

      mbridge.TelemetryTick_ms();
      crsf.TelemetryTick_ms();
      link_task_tick_ms();
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
      whileTransmit.Trigger();
      break;

    case LINK_STATE_RECEIVE:
      IF_ANTENNA1(sx.SetToRx(0));
      IF_ANTENNA2(sx2.SetToRx(0));
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
      if (USE_ANTENNA1 && USE_ANTENNA2) {
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

        if (USE_ANTENNA1 && USE_ANTENNA2) {
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
      } else {
        handle_receive_none();
      }

      txstats.fhss_curr_i = fhss.curr_i;
      txstats.rx1_valid = (link_rx1_status > RX_STATUS_INVALID);
      txstats.rx2_valid = (link_rx2_status > RX_STATUS_INVALID);

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

      // we are connected but tmo ran out
      if (connected() && !connect_tmo_cnt) {
        // so disconnect
        connect_state = CONNECT_STATE_LISTEN;
        link_task_set(LINK_TASK_TX_GET_RX_SETUPDATA);
      }

      // we are connected but didn't receive a valid frame
      if (connected() && !valid_frame_received) {
        // reset sync counter, relevant if in sync
        connect_sync_cnt = 0;
      }

      link_state = LINK_STATE_TRANSMIT;
      link_rx1_status = RX_STATUS_NONE;
      link_rx2_status = RX_STATUS_NONE;

      DECc(tick_1hz_commensurate, Config.frame_rate_hz);
      if (!tick_1hz_commensurate) {
        txstats.Update1Hz();
      }

      if (!connected()) stats.Clear();
      txstats.Next();

      // store parameters
      if (doParamsStore) {
        setup_store_to_EEPROM();
        goto RESTARTCONTROLLER;
      }

      bind.Do();
      switch (bind.Task()) {
      case BIND_TASK_CHANGED_TO_BIND:
        bind.ConfigForBind();
        fhss.SetToBind();
        LED_GREEN_OFF;
        LED_RED_OFF;
        connect_state = CONNECT_STATE_LISTEN;
        break;
      case BIND_TASK_TX_RESTART_CONTROLLER: goto RESTARTCONTROLLER; break;
      }

//dbg.puts((valid_frame_received) ? "\nvalid" : "\ninval");
    }//end of if(doPreTransmit)


    //-- Update channels, MBridge handling, Crsf handling, In handling, etc

#ifdef DEVICE_HAS_JRPIN5
    uint8_t mbstate, mbcmd; // for some reason it gives an error when put inside IF_MBRIDGE()
IF_MBRIDGE(
    // mBridge sends channels in regular 20 ms intervals, this we can use as sync
    if (mbridge.ChannelsUpdated(&rcData)) {
      // update channels
      if (Setup.Tx.ChannelsSource == CHANNEL_SOURCE_MBRIDGE) {
        channelOrder.Set(Setup.Tx.ChannelOrder); //TODO: better than before, but still better place!?
        channelOrder.Apply(&rcData);
      }
      // when we receive channels packet from transmitter, we send link stats to transmitter
      mbridge.TelemetryStart();
    }
    // we send a mbridge cmd twice per 20 ms cycle, we can't send too fast, in otx the receive buffer can hold 64 cmds
    if (mbridge.TelemetryUpdateState(&mbstate)) {
      switch (mbstate) {
      case 1: mbridge_send_LinkStats(); break;
      case 6: case 9:
        if (mbridge.CommandInFifo(&mbcmd)) {
          switch (mbcmd) {
          case MBRIDGE_CMD_DEVICE_ITEM_TX: mbridge_send_DeviceItemTx(); break;
          case MBRIDGE_CMD_DEVICE_ITEM_RX: mbridge_send_DeviceItemRx(); break;
          case MBRIDGE_CMD_PARAM_ITEM: mbridge_send_ParamItem(); break;
          case MBRIDGE_CMD_INFO: mbridge_send_Info(); break;
          }
        }
        break;
      }
    }
    if (mbridge.CommandReceived(&mbcmd)) {
      switch (mbcmd) {
      case MBRIDGE_CMD_DEVICE_REQUEST_ITEMS:
        mbridge.cmd_fifo.Put(MBRIDGE_CMD_DEVICE_ITEM_TX);
        mbridge.cmd_fifo.Put(MBRIDGE_CMD_DEVICE_ITEM_RX);
        break;
      case MBRIDGE_CMD_PARAM_REQUEST_LIST: mbridge_start_ParamRequestList(); break;
      case MBRIDGE_CMD_REQUEST_CMD: mbridge_send_RequestCmd(mbridge.GetPayloadPtr()); break;
      case MBRIDGE_CMD_PARAM_SET: {
        bool rx_param_changed;
        if (mbridge_do_ParamSet(mbridge.GetPayloadPtr(), &rx_param_changed)) {
            if (rx_param_changed) {
                if (connected()) {
                    // set parameter on Rx side
                    link_task_set(LINK_TASK_TX_SET_RX_PARAMS);
                    mbridge.Lock(MBRIDGE_CMD_PARAM_SET); // lock mbridge
                }
            }
        }
        }break;
      case MBRIDGE_CMD_PARAM_STORE:
        if (connected()) {
          link_task_set(LINK_TASK_TX_STORE_RX_PARAMS);
          mbridge.Lock(MBRIDGE_CMD_PARAM_STORE); // lock mbridge
        } else {
          doParamsStore = true;
        }
        break;
      case MBRIDGE_CMD_BIND: bind.SetToBind(); break;
      }
    }
);
IF_CRSF(
    uint8_t packet_idx;
    if (crsf.TelemetryUpdate(&packet_idx)) {
      switch (packet_idx) {
      case 1: crsf_send_LinkStatistics(); break;
      case 2: crsf_send_LinkStatisticsTx(); break;
      case 3: crsf_send_LinkStatisticsRx(); break;
      case 4:
        if (Setup.Tx.SerialLinkMode == SERIAL_LINK_MODE_MAVLINK) crsf.SendTelemetryFrame();
        break;
      }
    }

    if (Setup.Tx.ChannelsSource == CHANNEL_SOURCE_CRSF) {
      // update channels
      if (crsf.Update(&rcData)) {
        channelOrder.Set(Setup.Tx.ChannelOrder); //TODO: better than before, but still better place!?
        channelOrder.Apply(&rcData);
      }
    }
);
#endif
#if (defined DEVICE_HAS_IN)
    if (Setup.Tx.ChannelsSource == CHANNEL_SOURCE_INPORT) {
      // update channels
      if (in.Update(&rcData)) {
        channelOrder.Set(Setup.Tx.ChannelOrder); //TODO: better than before, but still better place!?
        channelOrder.Apply(&rcData);
      }
    }
#endif

    //-- do mavlink

    mavlink.Do();

    //-- do WhileTransmit stuff

    whileTransmit.Do();

    //-- cli task

    switch (cli.Task()) {
    case CLI_TASK_RX_PARAM_SET:
      if (connected()) {
        link_task_set(LINK_TASK_TX_SET_RX_PARAMS);
        mbridge.Lock(MBRIDGE_CMD_PARAM_SET); // lock mbridge
      }
      break;
    case CLI_TASK_PARAM_STORE:
      if (connected()) {
        link_task_set(LINK_TASK_TX_STORE_RX_PARAMS);
        mbridge.Lock(MBRIDGE_CMD_PARAM_STORE); // lock mbridge
      } else {
        doParamsStore = true;
      }
      break;
    case CLI_TASK_BIND: bind.SetToBind(); break;
    }

  }//end of while(1) loop

}//end of main

