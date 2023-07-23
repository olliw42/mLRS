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
#define FAIL_ENABLED


// we set the priorities here to have an overview, SysTick is at 15
#define UART_IRQ_PRIORITY           10 // mbridge, this needs to be high, when lower than DIO1, the module could stop sending via the bridge
#define UARTB_IRQ_PRIORITY          11 // serial
#define UARTC_IRQ_PRIORITY          11 // com
#define UARTD_IRQ_PRIORITY          15 // serial2/ESP/BT
#define UARTF_IRQ_PRIORITY          15 // debug
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
#include "../Common/thirdparty/stdstm32-adc.h"
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
#ifdef USE_COM
#ifndef USE_USB
#include "../modules/stm32ll-lib/src/stdstm32-uartc.h"
#endif
#endif
#ifdef USE_SERIAL2
#include "../modules/stm32ll-lib/src/stdstm32-uartd.h"
#endif
#ifdef USE_USB
#include "../modules/stm32-usb-device/stdstm32-usb-vcp.h"
#endif
#ifdef USE_IN
#include "../modules/stm32ll-lib/src/stdstm32-uarte.h"
#endif
#ifdef USE_DEBUG
#ifdef DEVICE_HAS_DEBUG_SWUART
#include "../modules/stm32ll-lib/src/stdstm32-uart-sw.h"
#else
#include "../modules/stm32ll-lib/src/stdstm32-uartf.h"
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
#include "../Common/channel_order.h"
//#include "../Common/test.h" // un-comment if you want to compile for board test

#include "in.h"
#include "txstats.h"
#include "cli.h"
#include "mbridge_interface.h" // this includes uart.h as it needs callbacks, declares tMBridge mbridge
#include "crsf_interface_tx.h" // this includes uart.h as it needs callbacks, declares tTxCrsf crsf


TxStatsBase txstats;
tComPort com;
tTxCli cli;
ChannelOrder channelOrder(ChannelOrder::DIRECTION_TX_TO_MLRS);


class In : public InBase
{
#ifdef USE_IN
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
            gpio_init_af(UARTE_RX_IO, IO_MODE_INPUT_PD, UARTE_IO_AF, IO_SPEED_VERYFAST); //TODO: should go to hal, but UARTE_RX_IO not known to hal at the time
        } else {
            in_set_normal();
            gpio_init_af(UARTE_RX_IO, IO_MODE_INPUT_PU, UARTE_IO_AF, IO_SPEED_VERYFAST);
        }
        uarte_rx_enableisr(ENABLE);
    }

    bool available(void) override { return uarte_rx_available(); }
    char getc(void) override { return uarte_getc(); }
#endif
};

In in;


//-------------------------------------------------------
// Mavlink
//-------------------------------------------------------

#include "mavlink_interface_tx.h"

MavlinkBase mavlink;


uint8_t mavlink_vehicle_state(void)
{
    return mavlink.VehicleState();
}


#include "sx_serial_interface_tx.h"

tTxSxSerial sx_serial;


//-------------------------------------------------------
// Display
//-------------------------------------------------------
// doing one, draw or update, every cycle in 50 Hz mode works, but
// doing both every cycle does not work! why ???

#include "../CommonTx/disp.h"

tTxDisp disp;


//-------------------------------------------------------
// Wifi Bridge
//-------------------------------------------------------

#include "esp.h"

tTxEspWifiBridge esp;


//-------------------------------------------------------
// While transmit/receive tasks
//-------------------------------------------------------

#include "../Common/while.h"


class WhileTransmit : public WhileBase
{
  public:
    int32_t dtmax_us(void) override { return sx.TimeOverAir_us() - 1000; }
    void handle_once(void) override;
};

WhileTransmit whileTransmit;


void WhileTransmit::handle_once(void)
{
    cli.Set(Setup.Tx.CliLineEnd);
    cli.Do();

#ifdef USE_DISPLAY
    uint32_t tnow_ms = millis32();

    static uint32_t main_tlast_ms = 0;
    if (tnow_ms - main_tlast_ms >= 250) { // Update Main page at 4 Hz
        main_tlast_ms = tnow_ms;
        disp.UpdateMain();
    }

    if (bind.IsInBind()) disp.SetBind();

    static uint32_t draw_tlast_ms = 0;
    if (tnow_ms - draw_tlast_ms >= 30) { // effectively slows down if in 50 Hz mode (Draw takes time, ca 30 ms on G4)
        draw_tlast_ms = tnow_ms;
        disp.Draw();
    }
#endif
}


//-------------------------------------------------------
// Some helper
//-------------------------------------------------------

void start_bind(void)
{
    if (!bind.IsInBind()) bind.StartBind();
}


void stop_bind(void)
{
    if (bind.IsInBind()) bind.StopBind();
}


void enter_system_bootloader(void)
{
    disp.DrawBoot();
    BootLoaderInit();
}


void enter_flash_esp(void)
{
    disp.DrawFlashEsp();
    flashesp_do(&com);
}


//-------------------------------------------------------
// Init
//-------------------------------------------------------

void init_once(void)
{
    com.InitOnce();
}


void init(void)
{
    // disable all interrupts, they may be enabled with restart
    __disable_irq();

    systembootloader_init();
    leds_init();
    button_init();
    esp_init();

    delay_init();
    micros_init();

    systembootloader_do(); // after delay_init() since it may need delay
    fiveway_init();

    serial.Init();
    serial2.Init();
    in.Init();

    com.Init();
    cli.Init(&com);
    esp.Init(&com, &serial2);
    buzzer.Init();
    fan.Init();
    dbg.Init();

    sx.Init();
    sx2.Init();

    setup_init();

    mbridge.Init(Config.UseMbridge, Config.UseCrsf); // these affect peripherals, hence do here
    crsf.Init(Config.UseCrsf);

    __enable_irq();
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
    sx2_dio_exti_isr_clearflag();
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
uint16_t link_task_delay_ms;
bool doParamsStore;


void link_task_init(void)
{
    link_task = LINK_TASK_NONE;
    link_task_delay_ms = 0;
    transmit_frame_type = TRANSMIT_FRAME_TYPE_NORMAL;

    doParamsStore = false;
}


bool link_task_set(uint8_t task)
{
    if (link_task != LINK_TASK_NONE) return false; // a task is running

    link_task = task;
    transmit_frame_type = TRANSMIT_FRAME_TYPE_CMD;

    link_task_delay_ms = 0;

    // set a timeout if relevant, or do other things as needed
    switch (link_task) {
    case LINK_TASK_TX_GET_RX_SETUPDATA:
    case LINK_TASK_TX_GET_RX_SETUPDATA_WRELOAD:
        SetupMetaData.rx_available = false;
        break;
    case LINK_TASK_TX_STORE_RX_PARAMS: // store rx parameters
        link_task_delay_ms = 500; // we set a delay, the actual store is triggered when it expires
        break;
    }

    return true;
}


void link_task_reset(void)
{
    link_task = LINK_TASK_NONE;
    link_task_delay_ms = 0;
    transmit_frame_type = TRANSMIT_FRAME_TYPE_NORMAL;
}


void link_task_tick_ms(void)
{
    // if a delay has been set, count it down
    if (link_task_delay_ms) {
        link_task_delay_ms--;
        if (!link_task_delay_ms) {
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
#if (defined DEVICE_HAS_JRPIN5)
        switch (mbridge.cmd_in_process) {
        case MBRIDGE_CMD_REQUEST_INFO: mbridge.HandleCmd(MBRIDGE_CMD_REQUEST_INFO); break;
        }
        mbridge.Unlock();
#endif
        break;
    }
}


void pack_txcmdframe(tTxFrame* frame, tFrameStats* frame_stats, tRcData* rc)
{
    switch (link_task) {
    case LINK_TASK_TX_GET_RX_SETUPDATA:
        pack_txcmdframe_cmd(frame, frame_stats, rc, FRAME_CMD_GET_RX_SETUPDATA);
        break;
    case LINK_TASK_TX_GET_RX_SETUPDATA_WRELOAD:
        pack_txcmdframe_cmd(frame, frame_stats, rc, FRAME_CMD_GET_RX_SETUPDATA_WRELOAD);
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

void prepare_transmit_frame(uint8_t antenna, uint8_t ack)
{
uint8_t payload[FRAME_TX_PAYLOAD_LEN];
uint8_t payload_len = 0;

    if (transmit_frame_type == TRANSMIT_FRAME_TYPE_NORMAL) {

        // read data from serial port
        if (connected()) {
            if (sx_serial.IsEnabled()) {
                for (uint8_t i = 0; i < FRAME_TX_PAYLOAD_LEN; i++) {
                    if (!sx_serial.available()) break;
                    payload[payload_len] = sx_serial.getc();
                    payload_len++;
                }
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

    if (frame->status.frame_type == FRAME_TYPE_TX_RX_CMD) {
        process_received_rxcmdframe(frame);
        return;
    }

    // output data on serial
    if (sx_serial.IsEnabled()) {
        for (uint8_t i = 0; i < frame->status.payload_len; i++) {
            uint8_t c = frame->payload[i];
            sx_serial.putc(c);
        }
    }

    stats.bytes_received.Add(frame->status.payload_len);
    stats.serial_data_received.Inc();
}


//-- receive/transmit handling api

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

    if (bind.IsInBind()) {
        bind.handle_receive(antenna, rx_status);
        return;
    }

    if (rx_status != RX_STATUS_INVALID) { // RX_STATUS_VALID
        bool do_payload = true;

        process_received_frame(do_payload, frame);

        txstats.doValidFrameReceived(); // should we count valid payload only if rx frame ?

        stats.received_seq_no = frame->status.seq_no;
        stats.received_ack = frame->status.ack;

    } else { // RX_STATUS_INVALID
        stats.received_seq_no = UINT8_MAX;
        stats.received_ack = 0;
    }

    // we set it for all received frames
    stats.last_antenna = antenna;

    // we count all received frames
    txstats.doFrameReceived();
}


void handle_receive_none(void) // RX_STATUS_NONE
{
    stats.received_seq_no = UINT8_MAX;
    stats.received_ack = 0;
}


void do_transmit(uint8_t antenna) // we send a TX frame to receiver
{
uint8_t ack = 1;

    if (bind.IsInBind()) {
        bind.do_transmit(antenna);
        return;
    }

    stats.transmit_seq_no++;

    prepare_transmit_frame(antenna, ack);

    sxSendFrame(antenna, &txFrame, FRAME_TX_RX_LEN, SEND_FRAME_TMO_MS); // 10 ms tmo
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

    if (res == CHECK_ERROR_SYNCWORD) return RX_STATUS_INVALID; // must not happen !

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
bool connect_occured_once;


static inline bool connected(void)
{
    return (connect_state == CONNECT_STATE_CONNECTED);
}

static inline bool connected_and_rx_setup_available(void)
{
    return (connected() && SetupMetaData.rx_available);
}


int main_main(void)
{
#ifdef BOARD_TEST_H
  main_test();
#endif
  init_once();
RESTARTCONTROLLER:
  init();
  DBG_MAIN(dbg.puts("\n\n\nHello\n\n");)

  serial.SetBaudRate(Config.SerialBaudrate);
  serial2.SetBaudRate(Config.SerialBaudrate);

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
  fhss.Init(Config.FhssNum, Config.FhssSeed, Setup.FrequencyBand, Config.FhssOrtho, Config.FhssExcept);
  fhss.Start();

  sx.SetRfFrequency(fhss.GetCurrFreq());
  sx2.SetRfFrequency(fhss.GetCurrFreq());

  tx_tick = 0;
  doPreTransmit = false;
  link_state = LINK_STATE_IDLE;
  connect_state = CONNECT_STATE_LISTEN;
  connect_tmo_cnt = 0;
  connect_sync_cnt = 0;
  connect_occured_once = false;
  link_rx1_status = link_rx2_status = RX_STATUS_NONE;
  link_task_init();
  link_task_set(LINK_TASK_TX_GET_RX_SETUPDATA); // we start with wanting to get rx setup data

  txstats.Init(Config.LQAveragingPeriod);

  in.Configure(Setup.Tx.InMode);
  mavlink.Init();
  sx_serial.Init(&serial, &mbridge, &serial2);
  fan.SetPower(sx.RfPower_dbm());
  whileTransmit.Init();

  disp.Init();

  led_blink = 0;
  tick_1hz = 0;
  tick_1hz_commensurate = 0;
  doSysTask = 0; // helps in avoiding too short first loop
  while (1) {

    //-- SysTask handling

    if (doSysTask) {
        // when we do long tasks, like display transfer, we miss ticks, so we need to catch up
        // the commands below must not be sensitive to strict ms timing
        doSysTask--; // doSysTask = 0;

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
            if (Setup.Tx.Buzzer == BUZZER_RX_LQ && connect_occured_once) {
                buzzer.BeepLQ(stats.received_LQ);
            }
        }

        DECc(tx_tick, SYSTICK_DELAY_MS(Config.frame_rate_ms));

        if (!tx_tick) {
            doPreTransmit = true; // trigger next cycle
            crsf.TelemetryStart();
        }

        mbridge.TelemetryTick_ms();
        crsf.TelemetryTick_ms();
        link_task_tick_ms();

        disp.Tick_ms();

        if (!tick_1hz) {
            dbg.puts(".");
/*            dbg.puts("\nTX: ");
            dbg.puts(u8toBCD_s(txstats.GetLQ()));
            dbg.puts("(");
            dbg.puts(u8toBCD_s(stats.frames_received.GetLQ())); dbg.putc(',');
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
    case LINK_STATE_IDLE:
    case LINK_STATE_RECEIVE_DONE:
        break;

    case LINK_STATE_TRANSMIT:
        fhss.HopToNext();
        sx.SetRfFrequency(fhss.GetCurrFreq());
        sx2.SetRfFrequency(fhss.GetCurrFreq());
        do_transmit((USE_ANTENNA1) ? ANTENNA_1 : ANTENNA_2);
        link_state = LINK_STATE_TRANSMIT_WAIT;
        irq_status = irq2_status = 0;
        DBG_MAIN_SLIM(dbg.puts("\n>");)
        whileTransmit.Trigger();
        break;

    case LINK_STATE_RECEIVE:
        IF_ANTENNA1(sx.SetToRx(0));
        IF_ANTENNA2(sx2.SetToRx(0));
        link_state = LINK_STATE_RECEIVE_WAIT;
        link_rx1_status = link_rx2_status = RX_STATUS_NONE;
        irq_status = irq2_status = 0;
        break;
    }//end of switch(link_state)

IF_ANTENNA1(
    if (irq_status) {
        if (link_state == LINK_STATE_TRANSMIT_WAIT) {
            if (irq_status & SX12xx_IRQ_TX_DONE) {
                irq_status = 0;
                link_state = LINK_STATE_RECEIVE;
                DBG_MAIN_SLIM(dbg.puts("1!");)
            }
        } else
        if (link_state == LINK_STATE_RECEIVE_WAIT) {
            if (irq_status & SX12xx_IRQ_RX_DONE) {
                irq_status = 0;
                link_rx1_status = do_receive(ANTENNA_1);
                DBG_MAIN_SLIM(dbg.puts("1<");)
            }
        }

        if (irq_status & SX12xx_IRQ_TIMEOUT) {
            irq_status = 0;
            link_state = LINK_STATE_IDLE;
            link_rx1_status = RX_STATUS_NONE;
            link_rx2_status = RX_STATUS_NONE;
            DBG_MAIN_SLIM(dbg.puts("1?");)
        }

        if (irq_status & SX12xx_IRQ_RX_DONE) {
            FAILALWAYS(GR_OFF_RD_BLINK, "IRQ RX DONE FAIL");
        }
        if (irq_status & SX12xx_IRQ_TX_DONE) {
            FAILALWAYS(RD_OFF_GR_BLINK, "IRQ TX DONE FAIL");
        }
    }//end of if(irq_status)
);
IF_ANTENNA2(
    if (irq2_status) {
        if (link_state == LINK_STATE_TRANSMIT_WAIT) {
            if (irq2_status & SX12xx_IRQ_TX_DONE) {
                irq2_status = 0;
                link_state = LINK_STATE_RECEIVE;
                DBG_MAIN_SLIM(dbg.puts("2!");)
            }
        } else
        if (link_state == LINK_STATE_RECEIVE_WAIT) {
            if (irq2_status & SX12xx_IRQ_RX_DONE) {
                irq2_status = 0;
                link_rx2_status = do_receive(ANTENNA_2);
                DBG_MAIN_SLIM(dbg.puts("2<");)
            }
        }

        if (irq2_status & SX12xx_IRQ_TIMEOUT) { // ??????????????????
            irq2_status = 0;
            link_state = LINK_STATE_IDLE;
            link_rx1_status = RX_STATUS_NONE;
            link_rx2_status = RX_STATUS_NONE;
            DBG_MAIN_SLIM(dbg.puts("2?");)
        }

        if (irq2_status & SX12xx_IRQ_RX_DONE) {
            FAILALWAYS(GR_ON_RD_BLINK, "IRQ2 RX DONE FAIL");
        }
        if (irq2_status & SX12xx_IRQ_TX_DONE) {
            FAILALWAYS(RD_ON_GR_BLINK, "IRQ2 TX DONE FAIL");
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
                    antenna = (stats.last_rssi1 > stats.last_rssi2) ? ANTENNA_1 : ANTENNA_2;
                } else
                if (link_rx1_status == RX_STATUS_VALID) {
                    antenna = ANTENNA_1;
                } else
                if (link_rx2_status == RX_STATUS_VALID) {
                    antenna = ANTENNA_2;
                } else {
                    // we can choose either antenna, so select the one with the better rssi
                    antenna = (stats.last_rssi1 > stats.last_rssi2) ? ANTENNA_1 : ANTENNA_2;
                }
            } else if (USE_ANTENNA2) {
                antenna = ANTENNA_2;
            }

            handle_receive(antenna);
        } else {
            handle_receive_none();
        }

        txstats.fhss_curr_i = fhss.CurrI();
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
                if (connect_sync_cnt >= CONNECT_SYNC_CNT) {
                    if (!SetupMetaData.rx_available && !bind.IsInBind()) {
                        // should not have happen, but does very occasionally happen, so let's cope with
                        // we must have gotten it at least once, on first connect, since we need it
                        // later on we can accept to be gentle and be ok with not getting it again
                        // bottom line: the receiver must not change after first connection
                        if (connect_occured_once) {
                            link_task_reset();
                            SetupMetaData.rx_available = true;
                        } else {
                            // we could be more gentle and postpone connection by one cnt
                            FAILALWAYS(BLINK_3, "rx_available not true");
                        }
                    }
                    connect_state = CONNECT_STATE_CONNECTED;
                    connect_occured_once = true;
                }
                break;
            }
            connect_tmo_cnt = CONNECT_TMO_SYSTICKS;
        }

        // we are connected but tmo ran out
        if (connected() && !connect_tmo_cnt) {
            // so disconnect
            connect_state = CONNECT_STATE_LISTEN;
            // link_state will be set to LINK_STATE_TRANSMIT below
        }

        // we are connected but didn't receive a valid frame
        if (connected() && !valid_frame_received) {
            // reset sync counter, relevant if in sync
            //connect_sync_cnt = 0; //isn't needed, right? since when connected we can't be in sync
        }

        link_state = LINK_STATE_TRANSMIT;
        link_rx1_status = RX_STATUS_NONE;
        link_rx2_status = RX_STATUS_NONE;

        if (connect_state == CONNECT_STATE_LISTEN) {
            link_task_reset(); // to ensure that the following set is enforced
            link_task_set(LINK_TASK_TX_GET_RX_SETUPDATA);
        }

        DECc(tick_1hz_commensurate, Config.frame_rate_hz);
        if (!tick_1hz_commensurate) {
            txstats.Update1Hz();
        }
        txstats.Next();
        if (!connected()) txstats.Clear();

        if (Setup.Tx.Buzzer == BUZZER_LOST_PACKETS && connect_occured_once && !bind.IsInBind()) {
            if (!valid_frame_received) buzzer.BeepLP();
        }

        // store parameters
        if (doParamsStore) {
            sx.SetToIdle();
            sx2.SetToIdle();
            setup_store_to_EEPROM();
            goto RESTARTCONTROLLER;
        }

        bind.Do();
        switch (bind.Task()) {
        case BIND_TASK_CHANGED_TO_BIND:
            bind.ConfigForBind();
            fhss.SetToBind();
            LED_GREEN_ON;
            LED_RED_OFF;
            connect_state = CONNECT_STATE_LISTEN;
            // link_state was set to LINK_STATE_TRANSMIT already
            break;
        case BIND_TASK_TX_RESTART_CONTROLLER: goto RESTARTCONTROLLER; break;
        }

//dbg.puts((valid_frame_received) ? "\nvalid" : "\ninval");
    }//end of if(doPreTransmit)


    //-- Update channels, MBridge handling, Crsf handling, In handling, etc

#ifdef DEVICE_HAS_JRPIN5
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
    uint8_t mbtask; uint8_t mbcmd;
    if (mbridge.TelemetryUpdate(&mbtask)) {
        switch (mbtask) {
        case TXBRIDGE_SEND_LINK_STATS: mbridge_send_LinkStats(); break;
        case TXBRIDGE_SEND_CMD:
            if (mbridge.CommandInFifo(&mbcmd)) mbridge_send_cmd(mbcmd);
            break;
        }
    }
);
IF_MBRIDGE_OR_CRSF( // to allow crsf mbridge emulation
    // handle an incoming command
    uint8_t mbcmd;
    if (mbridge.CommandReceived(&mbcmd)) {
        switch (mbcmd) {
        case MBRIDGE_CMD_REQUEST_INFO:
            setup_reload();
            if (connected()) {
                link_task_set(LINK_TASK_TX_GET_RX_SETUPDATA_WRELOAD);
                mbridge.Lock(MBRIDGE_CMD_REQUEST_INFO); // lock mbridge
            } else {
                mbridge.HandleCmd(MBRIDGE_CMD_REQUEST_INFO);
            }
            break;
        case MBRIDGE_CMD_PARAM_REQUEST_LIST: mbridge.HandleCmd(MBRIDGE_CMD_PARAM_REQUEST_LIST); break;
        case MBRIDGE_CMD_REQUEST_CMD: mbridge.HandleRequestCmd(mbridge.GetPayloadPtr()); break;
        case MBRIDGE_CMD_PARAM_SET: {
            bool rx_param_changed;
            bool param_changed = mbridge_do_ParamSet(mbridge.GetPayloadPtr(), &rx_param_changed);
            if (param_changed && rx_param_changed && connected()) {
                link_task_set(LINK_TASK_TX_SET_RX_PARAMS); // set parameter on Rx side
                mbridge.Lock(MBRIDGE_CMD_PARAM_SET); // lock mbridge
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
        case MBRIDGE_CMD_BIND_START: start_bind(); break;
        case MBRIDGE_CMD_BIND_STOP: stop_bind(); break;
        case MBRIDGE_CMD_SYSTEM_BOOTLOADER: enter_system_bootloader(); break;
        case MBRIDGE_CMD_MODELID_SET: {
//dbg.puts("\nmbridge model id "); dbg.puts(u8toBCD_s(mbridge.GetPayloadPtr()[0]));
            }break;
        }
    }
);
IF_CRSF(
    if (crsf.ChannelsUpdated(&rcData)) {
        // update channels
        if (Setup.Tx.ChannelsSource == CHANNEL_SOURCE_CRSF) {
            channelOrder.Set(Setup.Tx.ChannelOrder); //TODO: better than before, but still better place!?
            channelOrder.Apply(&rcData);
        }
    }
    uint8_t crsftask; uint8_t crsfcmd;
    uint8_t mbcmd; static uint8_t do_cnt = 0; // if it's to fast lua script gets out of sync
    uint8_t* buf; uint8_t len;
    if (crsf.TelemetryUpdate(&crsftask, Config.frame_rate_ms)) {
        switch (crsftask) {
        case TXCRSF_SEND_LINK_STATISTICS: crsf_send_LinkStatistics(); do_cnt = 2; break;
        case TXCRSF_SEND_LINK_STATISTICS_TX: crsf_send_LinkStatisticsTx(); break;
        case TXCRSF_SEND_LINK_STATISTICS_RX: crsf_send_LinkStatisticsRx(); break;
        case TXCRSF_SEND_TELEMETRY_FRAME:
            if (do_cnt && mbridge.CommandInFifo(&mbcmd)) {
                mbridge_send_cmd(mbcmd);
            }
            if (mbridge.CrsfFrameAvailable(&buf, &len)) {
                crsf.SendMBridgeFrame(buf, len);
            } else
            if (connected_and_rx_setup_available() && Setup.Rx.SerialLinkMode == SERIAL_LINK_MODE_MAVLINK) {
                crsf.SendTelemetryFrame();
            }
            DECl(do_cnt);
            break;
        }
    }
    if (crsf.CommandReceived(&crsfcmd)) {
        switch (crsfcmd) {
        case TXCRSF_CMD_MODELID_SET:
//dbg.puts("\ncrsf model select id "); dbg.puts(u8toBCD_s(crsf.GetCmdDataPtr()[0]));
            break;
        case TXCRSF_CMD_MBRIDGE_IN:
//dbg.puts("\ncrsf mbridge ");
            mbridge.ParseCrsfFrame(crsf.GetPayloadPtr(), crsf.GetPayloadLen(), micros());
            break;
        }
    }
);
#endif
#ifdef USE_IN
    if (Setup.Tx.ChannelsSource == CHANNEL_SOURCE_INPORT) {
        // update channels
        if (in.Update(&rcData)) {
            channelOrder.Set(Setup.Tx.ChannelOrder); //TODO: better than before, but still better place!?
            channelOrder.Apply(&rcData);
        }
    }
#endif

    //-- Do mavlink

    mavlink.Do();

    //-- Do WhileTransmit stuff

    whileTransmit.Do();

    //-- Handle display or cli task

    uint8_t cli_task = disp.Task();
    if (cli_task == CLI_TASK_NONE) cli_task = cli.Task();

    switch (cli_task) {
    case CLI_TASK_RX_PARAM_SET:
        if (connected()) {
            link_task_set(LINK_TASK_TX_SET_RX_PARAMS);
            mbridge.Lock(); // lock mbridge
        }
        break;
    case CLI_TASK_PARAM_STORE:
        if (connected()) {
            link_task_set(LINK_TASK_TX_STORE_RX_PARAMS);
            mbridge.Lock(); // lock mbridge
        } else {
            doParamsStore = true;
        }
        break;
    case CLI_TASK_PARAM_RELOAD:
        setup_reload();
        if (connected()) {
            link_task_set(LINK_TASK_TX_GET_RX_SETUPDATA_WRELOAD);
            mbridge.Lock(); // lock mbridge
        }
        break;
    case CLI_TASK_BIND: start_bind(); break;
    case CLI_TASK_BOOT: enter_system_bootloader(); break;
    case CLI_TASK_FLASH_ESP: enter_flash_esp(); break;
    }

    //-- Handle esp wifi bridge

    esp.Do();
    uint8_t esp_task = esp.Task();
    if (esp_task == ESP_TASK_RESTART_CONTROLLER) goto RESTARTCONTROLLER;

  }//end of while(1) loop

}//end of main

