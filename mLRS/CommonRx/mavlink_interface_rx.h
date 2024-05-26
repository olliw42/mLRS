//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Mavlink Interface RX Side
//*******************************************************
#ifndef MAVLINK_INTERFACE_RX_H
#define MAVLINK_INTERFACE_RX_H
#pragma once


#include "../Common/mavlink/fmav_extension.h"
#include "../Common/libs/filters.h"
#ifdef USE_FEATURE_MAVLINKX
#include "../Common/thirdparty/mavlinkx.h"
#include "../Common/libs/fifo.h"
#endif


extern volatile uint32_t millis32(void);
static inline bool connected(void);


#define RADIO_LINK_SYSTEM_ID          51 // SiK uses 51, 68
#define GCS_SYSTEM_ID                 255 // default of MissionPlanner, QGC
#define REBOOT_SHUTDOWN_MAGIC         1234321
#if defined ESP8266 || defined ESP32
  #define REBOOT_SHUTDOWN_MAGIC_ACK   (REBOOT_SHUTDOWN_MAGIC + 1) // to indicate ESP
#else
  #define REBOOT_SHUTDOWN_MAGIC_ACK   REBOOT_SHUTDOWN_MAGIC
#endif

#define MAVLINK_BUF_SIZE              300 // needs to be larger than max MAVLink frame size = 280 bytes

#define MAVLINK_OPT_FAKE_PARAMFTP     2 // 0: off, 1: always, 2: determined from mode & baudrate


class tRxMavlink
{
  public:
    void Init(void);
    void Do(void);
    void SendRcData(tRcData* rc_out, bool frame_missed, bool failsafe);
    void FrameLost(void);

    void putc(char c);
    bool available(void);
    uint8_t getc(void);
    void flush(void);

  private:
    void send_msg_serial_out(void);
    void generate_radio_status(void);
    void generate_rc_channels_override(void);
    void generate_radio_rc_channels(void);
    void generate_radio_link_stats(void);
    void generate_radio_link_information(void);
    void generate_radio_link_flow_control(void);

    uint16_t serial_in_available(void);
    bool handle_txbuf_ardupilot(uint32_t tnow_ms);
    bool handle_txbuf_method_b(uint32_t tnow_ms); // for PX4, aka "brad"

    // fields for link in -> parser -> serial out
    fmav_status_t status_link_in;
    uint8_t buf_link_in[MAVLINK_BUF_SIZE]; // buffer for link in parser
    fmav_status_t status_serial_out; // not needed, status_link_in could be used, but clearer so
    fmav_message_t msg_serial_out; // could be avoided by more efficient coding, is used only momentarily/locally

    // fields for serial in -> parser -> link out
#ifdef USE_FEATURE_MAVLINKX
    fmav_status_t status_serial_in;
    uint8_t buf_serial_in[MAVLINK_BUF_SIZE]; // buffer for serial in parser
    fmav_message_t msg_link_out; // could be avoided by more efficient coding, is used only momentarily/locally
    FifoBase<char,512> fifo_link_out; // needs to be at least 82 + 280
    uint32_t bytes_parser_in; // bytes in the parser
#endif

    // to inject RADIO_STATUS or RADIO_LINK_FLOW_CONTROL
    uint32_t radio_status_tlast_ms;
    uint32_t bytes_link_out; // bytes grabbed by link out
    uint8_t radio_status_txbuf;

    uint32_t bytes_link_out_cnt; // for rate filter
    LPFilterRate bytes_link_out_rate_filt;

    typedef enum {
        TXBUF_STATE_NORMAL = 0,
        TXBUF_STATE_BURST,
        TXBUF_STATE_BURST_HIGH,
        TXBUF_STATE_PX4_RECOVER, // for PX4, buffer draining, resume bulk download
    } TXBUF_STATE_ENUM;
    uint8_t txbuf_state;

    // to inject RC_CHANNELS_OVERRIDE or RADIO_RC_CHANNELS & RADIO_LINK_STATS
    bool inject_rc_channels;
    uint16_t rc_chan[16]; // holds the rc data in MAVLink format
    int16_t rc_chan_13b[16]; // holds the rc data in MAVLink RADIO_RC_CHANNELS format
    bool rc_failsafe;
    uint32_t rc_channels_tupdated_ms; // time of last update of RC channels values
    bool rc_channels_uptodate;

    // to handle command PREFLIGHT_REBOOT_SHUTDOWN, START_RX_PAIR and to inject CMD_ACK response
    bool inject_cmd_ack;
    struct {
        uint16_t command;
        uint8_t cmd_src_sysid;
        uint8_t cmd_src_compid;
        uint8_t state; // 0: not armed, 1: armed, 2: going to be executed
        uint32_t texe_ms;
    } cmd_ack;

    void handle_cmd_long(void);
    void generate_cmd_ack(void);

    uint8_t _buf[MAVLINK_BUF_SIZE]; // temporary working buffer, to not burden stack
};


void tRxMavlink::Init(void)
{
    fmav_init();

    status_link_in = {};
    status_serial_out = {};

#ifdef USE_FEATURE_MAVLINKX
    fmavX_init();
    fmavX_config_compression((Config.Mode == MODE_19HZ) ? 1 : 0); // use compression only in 19 Hz mode

    status_serial_in = {};
    fifo_link_out.Init();
    bytes_parser_in = 0;
#endif

    radio_status_tlast_ms = millis32() + 1000;
    radio_status_txbuf = 0;
    txbuf_state = TXBUF_STATE_NORMAL;

    bytes_link_out = 0;
    bytes_link_out_cnt = 0;
    bytes_link_out_rate_filt.Reset();

    inject_rc_channels = false;
    for (uint8_t i = 0; i < 16; i++) { rc_chan[i] = 0; rc_chan_13b[i] = 0; }
    rc_failsafe = false;
    rc_channels_tupdated_ms = 0;
    rc_channels_uptodate = false;

    inject_cmd_ack = false;
    cmd_ack.cmd_src_sysid = 0;
    cmd_ack.cmd_src_compid = 0;
    cmd_ack.state = 0;
    cmd_ack.texe_ms = 0;
}


// rc_out is the rc data stored in out class
// after handling of channel order and failsafes.
// Need to take care of failsafe flag however.
void tRxMavlink::SendRcData(tRcData* rc_out, bool frame_missed, bool failsafe)
{
    if (Setup.Rx.SendRcChannels == SEND_RC_CHANNELS_OFF) return;

    uint8_t failsafe_mode = Setup.Rx.FailsafeMode;

    if (failsafe) {
        switch (failsafe_mode) {
        case FAILSAFE_MODE_NO_SIGNAL:
            // do not output anything, so jump out
            return;
        case FAILSAFE_MODE_CH1CH4_CENTER:
            // in this mode do not report bad signal
            failsafe = false;
            break;
        }
    }

    for (uint8_t i = 0; i < 16; i++) {
        rc_chan[i] = rc_to_mavlink(rc_out->ch[i]);
        rc_chan_13b[i] = rc_to_mavlink_13bcentered(rc_out->ch[i]);
    }

    rc_channels_tupdated_ms = millis32();
    rc_failsafe = failsafe;
    rc_channels_uptodate = !frame_missed;

    inject_rc_channels = true;
}


void tRxMavlink::Do(void)
{
    uint32_t tnow_ms = millis32();
    bool inject_radio_link_stats = false;
    bool inject_radio_status = false;

    if (!connected()) {
        //Init();
        //radio_status_tlast_ms = tnow_ms + 1000;
#ifdef USE_FEATURE_MAVLINKX
        fifo_link_out.Flush();
#endif
    }

    if (!SERIAL_LINK_MODE_IS_MAVLINK(Setup.Rx.SerialLinkMode)) return;

    // parse serial in -> link out
#ifdef USE_FEATURE_MAVLINKX
    fmav_result_t result;
    if (fifo_link_out.HasSpace(290)) { // we have space for a full MAVLink message, so can safely parse
        while (serial.available()) {
            char c = serial.getc();
            bytes_parser_in++; // memorize it is still in processing
            fmav_parse_and_check_to_frame_buf(&result, buf_serial_in, &status_serial_in, c);
            if (result.res == FASTMAVLINK_PARSE_RESULT_OK) {

                // TODO: this could be be done more efficiently by not going via msg_link_out
                // but by directly going buf_serial_in -> _buf

                fmav_frame_buf_to_msg(&msg_link_out, &result, buf_serial_in); // requires RESULT_OK

                uint16_t len;
                if (Setup.Rx.SerialLinkMode == SERIAL_LINK_MODE_MAVLINK_X) {
                    len = fmavX_msg_to_frame_buf(_buf, &msg_link_out); // X frame now in _buf
                } else {
                    len = fmav_msg_to_frame_buf(_buf, &msg_link_out);
                }

                fifo_link_out.PutBuf(_buf, len);
                bytes_parser_in = 0;

                if (msg_link_out.msgid == FASTMAVLINK_MSG_ID_COMMAND_LONG) {
                    handle_cmd_long();
                }

                break; // give the loop a chance before handling a further message
            }
        }
    }
#endif

    if (Setup.Rx.SendRadioStatus && connected()) {
        // we currently know that if we determine inject_radio_status here it will be executed immediately
        switch (Setup.Rx.SendRadioStatus) {
        case RX_SEND_RADIO_STATUS_METHOD_ARDUPILOT_1:
            inject_radio_status = handle_txbuf_ardupilot(tnow_ms);
            break;
        case RX_SEND_RADIO_STATUS_METHOD_PX4:
            inject_radio_status = handle_txbuf_method_b(tnow_ms);
            break;
        }
    } else if (Setup.Rx.SendRadioStatus && !connected()) {
        if ((tnow_ms - radio_status_tlast_ms) >= 1000) {
            radio_status_tlast_ms = tnow_ms;
            inject_radio_status = true;
            radio_status_txbuf = 50; // ArduPilot: 50-90, PX4: 35-50  -> no change
        }
        bytes_link_out_rate_filt.Reset();
    } else {
        radio_status_tlast_ms = tnow_ms;
        bytes_link_out_rate_filt.Reset();
    }

    // TODO: either the buffer must be guaranteed to be large, or we need to check filling

    if (inject_rc_channels) { // give it priority // && serial.tx_is_empty()) // check available size!?
        inject_rc_channels = false;
        switch (Setup.Rx.SendRcChannels) {
        case SEND_RC_CHANNELS_RCCHANNELSOVERRIDE:
            generate_rc_channels_override();
            send_msg_serial_out();
            break;
        case SEND_RC_CHANNELS_RADIORCCHANNELS:
            generate_radio_rc_channels();
            send_msg_serial_out();
            inject_radio_link_stats = true;
            break;
        }
    }

    if (inject_radio_link_stats) { // check available size!?
        inject_radio_link_stats = false;
        generate_radio_link_stats();
        send_msg_serial_out();
    }

    if (inject_radio_status) { // check available size!?
        inject_radio_status = false;
/* only for deving, always send radio status
        if (Setup.Rx.SendRcChannels == SEND_RC_CHANNELS_RADIORCCHANNELS) {
            generate_radio_link_flow_control();
        } else {
            generate_radio_status();
        }
*/
        generate_radio_status();
        send_msg_serial_out();
    }

    if (inject_cmd_ack) {
        inject_cmd_ack = false;
        generate_cmd_ack();
        send_msg_serial_out();
    }

    if (cmd_ack.state == 2 && (tnow_ms - cmd_ack.texe_ms) > 1000) {
        switch (cmd_ack.command) {
        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            BootLoaderInit(); // jump to system bootloader
            break;
        case MAV_CMD_START_RX_PAIR:
            bind.StartBind(); // start binding
            break;
        }
        cmd_ack.command = 0;
        cmd_ack.state = 0;
    }
}


void tRxMavlink::FrameLost(void)
{
#ifdef USE_FEATURE_MAVLINKX
    // reset parser link in -> serial out
    fmav_parse_reset(&status_link_in);
#endif
}


typedef enum {
    MAVFTP_OPCODE_OpenFileRO = 4,
} MAVFTPOPCODEENUM;


void tRxMavlink::putc(char c)
{
    // parse link in -> serial out
    fmav_result_t result;
#ifdef USE_FEATURE_MAVLINKX
    if (Setup.Rx.SerialLinkMode == SERIAL_LINK_MODE_MAVLINK_X) {
        fmavX_parse_and_check_to_frame_buf(&result, buf_link_in, &status_link_in, c);
    } else {
        fmav_parse_and_check_to_frame_buf(&result, buf_link_in, &status_link_in, c);
    }
#else
    fmav_parse_and_check_to_frame_buf(&result, buf_link_in, &status_link_in, c);
#endif
    if (result.res == FASTMAVLINK_PARSE_RESULT_OK) {
        fmav_frame_buf_to_msg(&msg_serial_out, &result, buf_link_in); // requires RESULT_OK

#if MAVLINK_OPT_FAKE_PARAMFTP > 0
#if MAVLINK_OPT_FAKE_PARAMFTP > 1
        bool force_param_list = true;
        switch (Config.Mode) {
        case MODE_FLRC_111HZ: force_param_list = (Config.SerialBaudrate > 230400); break; // 230400 bps and lower is ok for mftp
        case MODE_50HZ:
        case MODE_FSK_50HZ: force_param_list = (Config.SerialBaudrate > 115200); break; // 115200 bps and lower is ok for mftp
        case MODE_31HZ: force_param_list = (Config.SerialBaudrate > 57600); break; // 57600 bps and lower is ok for mftp
        case MODE_19HZ: force_param_list = (Config.SerialBaudrate > 38400); break; // 38400 bps and lower is ok for mftp
        }
        if (force_param_list)
#endif
        // if it's a mavftp call to @PARAM/param.pck we fake the url
        // this will make ArduPilot to response with a NACK:FileNotFound
        // which will make MissionPlanner (any GCS?) to fallback to normal parameter upload
        if (msg_serial_out.msgid == FASTMAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL) {
            uint8_t target_component = msg_serial_out.payload[2];
            uint8_t opcode = msg_serial_out.payload[6];
            char* url = (char*)(msg_serial_out.payload + 15);
            if (((target_component == MAV_COMP_ID_AUTOPILOT1) || (target_component == MAV_COMP_ID_ALL)) &&
                (opcode == MAVFTP_OPCODE_OpenFileRO)) {
                if (!strncmp(url, "@PARAM/param.pck", 16)) {
                    url[1] = url[7] = url[13] = 'x'; // now fake it to "@xARAM/xaram.xck"
                    fmav_msg_recalculate_crc(&msg_serial_out); // we need to recalculate CRC, requires RESULT_OK
                }
            }
        }
#endif

        send_msg_serial_out();
    }
}


bool tRxMavlink::available(void)
{
#ifdef USE_FEATURE_MAVLINKX
    return fifo_link_out.Available();
#else
    return serial.available();
#endif
}


uint8_t tRxMavlink::getc(void)
{
    bytes_link_out++;
    bytes_link_out_cnt++;

#ifdef USE_FEATURE_MAVLINKX
    return fifo_link_out.Get();
#else
    return serial.getc();
#endif
}


void tRxMavlink::flush(void)
{
#ifdef USE_FEATURE_MAVLINKX
    fifo_link_out.Flush();
#endif
    serial.flush();
}


void tRxMavlink::send_msg_serial_out(void)
{
    uint16_t len = fmav_msg_to_frame_buf(_buf, &msg_serial_out);

    serial.putbuf(_buf, len);
}


//-------------------------------------------------------
// Handle txbuf
//-------------------------------------------------------
// for the txbuf rate-based mechanism see design_decissions.h for details

uint16_t tRxMavlink::serial_in_available(void)
{
#ifdef USE_FEATURE_MAVLINKX
    // count all bytes still in processing
    return fifo_link_out.Available() + serial.bytes_available() + bytes_parser_in;
#else
    return serial.bytes_available();
#endif
}


bool tRxMavlink::handle_txbuf_ardupilot(uint32_t tnow_ms)
{
    // work out state
    bool inject_radio_status = false;
    uint8_t txbuf_state_last = txbuf_state; // to track changes in txbuf_state

    if ((tnow_ms - radio_status_tlast_ms) >= 1000) {
        //txbuf_state = TXBUF_STATE_NORMAL; // ??? or should we enter this case only if in normal?? does it matter ??
        radio_status_tlast_ms = tnow_ms;
        inject_radio_status = true;
    } else
    if ((tnow_ms - radio_status_tlast_ms) >= 100) { // limit to 10 Hz
        switch (txbuf_state) {
        case TXBUF_STATE_NORMAL:
            if (serial_in_available() > 1024) { // ups, suddenly lots of traffic
                txbuf_state = TXBUF_STATE_BURST;
                radio_status_tlast_ms = tnow_ms;
                inject_radio_status = true;
            }
            break;
        case TXBUF_STATE_BURST:
            if (serial_in_available() > 1024) { // it hasn't depleted, so raise alarm high
                txbuf_state = TXBUF_STATE_BURST_HIGH;
                radio_status_tlast_ms = tnow_ms;
                inject_radio_status = true;
            } else
            if (serial_in_available() < 384) { // quite empty, so we can go back and try normal
                txbuf_state = TXBUF_STATE_NORMAL;
                radio_status_tlast_ms = tnow_ms;
                inject_radio_status = true;
            }
            break;
        case TXBUF_STATE_BURST_HIGH:
            if (serial_in_available() < 1024) { // it has depleted, we can go back and try burst
                txbuf_state = TXBUF_STATE_BURST;
            }
            radio_status_tlast_ms = tnow_ms;
            inject_radio_status = true;
            break;
        }
    }

    if (!inject_radio_status) return false;

    // now calculate txbuf
    uint8_t txbuf = 100;

    // method C, with improvements
    // assumes 1 sec delta time
    uint32_t rate_max = ((uint32_t)1000 * FRAME_RX_PAYLOAD_LEN) / Config.frame_rate_ms; // theoretical rate, bytes per sec
    uint32_t rate_percentage = (bytes_link_out * 100) / rate_max;

    // https://github.com/ArduPilot/ardupilot/blob/fa6441544639bd5dc84c3e6e3d2f7bfd2aecf96d/libraries/GCS_MAVLink/GCS_Common.cpp#L782-L801
    // aim at 75%..85% rate usage in steady state
    if (rate_percentage > 95) {
        txbuf = 0;                        // ArduPilot:  0-19  -> +60 ms,    PX4:  0-24  -> *0.8
    } else if (rate_percentage > 85) {
        txbuf = 30;                       // ArduPilot: 20-49  -> +20 ms,    PX4: 25-34  -> *0.975
    } else if (rate_percentage < 60) {
        txbuf = 100;                      // ArduPilot: 96-100 -> -40 ms,    PX4: 51-100 -> *1.025
    } else if (rate_percentage < 75) {
        txbuf = 91;                       // ArduPilot: 91-95  -> -20 ms,    PX4: 51-100 -> *1.025
    } else {
        txbuf = 50;                       // ArduPilot: 50-90  -> no change, PX4: 35-50  -> no change
    }

    if (txbuf_state == TXBUF_STATE_BURST_HIGH) {
        txbuf = 0; // try to slow down as much as possible
    } else
    if (txbuf_state == TXBUF_STATE_BURST) {
        txbuf = 50; // cut out PARAMS but don't change stream rate
    } else
    if ((txbuf_state == TXBUF_STATE_NORMAL) && (txbuf_state_last > TXBUF_STATE_NORMAL)) { // has changed back to NORMAL
        txbuf = 51; // allow PARAMS but don't change stream rate
    }
    txbuf_state_last = txbuf_state;

    // only for "educational" purposes currently
    bytes_link_out_rate_filt.Update(tnow_ms, bytes_link_out_cnt, 1000);
#if 0 // Debug
static uint32_t t_last = 0;
uint32_t t = millis32(), dt = t - t_last; t_last = t;
dbg.puts("\nMa: ");
dbg.puts(u16toBCD_s(t));dbg.puts(" (");dbg.puts(u16toBCD_s(dt));dbg.puts("), ");
//dbg.puts(u16toBCD_s(stats.GetTransmitBandwidthUsage()*41));dbg.puts(", ");
dbg.puts(u16toBCD_s(bytes_link_out));dbg.puts(" (");
dbg.puts(u16toBCD_s(bytes_link_out_rate_filt.Get()));dbg.puts("), ");
dbg.puts(u16toBCD_s(serial_in_available()));dbg.puts(", ");
dbg.puts(u8toBCD_s((rate_percentage<256)?rate_percentage:255));dbg.puts(", ");
if(txbuf_state==2) dbg.puts("high, "); else
if(txbuf_state==1) dbg.puts("brst, "); else dbg.puts("norm, ");
dbg.puts(u8toBCD_s(txbuf));dbg.puts(", ");
if(txbuf<20) dbg.puts("+60 "); else
if(txbuf<50) dbg.puts("+20 "); else
if(txbuf>95) dbg.puts("-40 "); else
if(txbuf>90) dbg.puts("-20 "); else dbg.puts("0   ");
#endif
    if ((txbuf_state == TXBUF_STATE_NORMAL) && (txbuf == 100)) {
        radio_status_tlast_ms -= 666; // do again in 1/3 sec
        bytes_link_out = (bytes_link_out * 2)/3; // approximate by 2/3 of what was received in the last 1 sec
    } else {
        bytes_link_out = 0; // reset, to restart rate measurement
    }

    radio_status_txbuf = txbuf;
    return true;
}


// this method should be selected for PX4 and currently may be a useful alternative for Ardupilot
bool tRxMavlink::handle_txbuf_method_b(uint32_t tnow_ms)
{
    // work out state
    bool inject_radio_status = false;

    if ((tnow_ms - radio_status_tlast_ms) >= 1000) {
        radio_status_tlast_ms = tnow_ms;
        inject_radio_status = true;
    } else
    switch (txbuf_state) {
        case TXBUF_STATE_NORMAL:
            if (serial_in_available() > 800) { // oops, buffer filling
                txbuf_state = TXBUF_STATE_BURST;
                radio_status_tlast_ms = tnow_ms;
                inject_radio_status = true;
            }
            break;
        case TXBUF_STATE_BURST:
            if (serial_in_available() > 1400) { // still growing, so raise alarm high
                txbuf_state = TXBUF_STATE_BURST_HIGH;
                radio_status_tlast_ms = tnow_ms;
                inject_radio_status = true;
            } else
            if (serial_in_available() < FRAME_RX_PAYLOAD_LEN*2) { // less than 2 radio messages remain, back to normal
                txbuf_state = TXBUF_STATE_PX4_RECOVER;
                radio_status_tlast_ms = tnow_ms;
                inject_radio_status = true;
            }
            break;
        case TXBUF_STATE_BURST_HIGH:
            if ((tnow_ms - radio_status_tlast_ms) >= 100) { // limit to 10 Hz
                if (serial_in_available() < 1400) { // it has stopped growing, we can go back and try burst
                    txbuf_state = TXBUF_STATE_BURST;
                }
                radio_status_tlast_ms = tnow_ms;
                inject_radio_status = true;
            }
            break;
        case TXBUF_STATE_PX4_RECOVER: // transient state so we don't need txbuf_state_last
            txbuf_state = TXBUF_STATE_NORMAL;
            break;
    }

    if (!inject_radio_status) return false;

    // now calculate txbuf
    uint8_t txbuf = 100;

    // method C, with improvements
    // assumes 1 sec delta time
    uint32_t rate_max = ((uint32_t)1000 * FRAME_RX_PAYLOAD_LEN) / Config.frame_rate_ms; // theoretical rate, bytes per sec
    uint32_t rate_percentage = (bytes_link_out * 100) / rate_max;

    // https://github.com/ArduPilot/ardupilot/blob/fa6441544639bd5dc84c3e6e3d2f7bfd2aecf96d/libraries/GCS_MAVLink/GCS_Common.cpp#L782-L801
    // https://github.com/PX4/PX4-Autopilot/blob/fe80e7aa468a50bec6b035d0e8e4e37e516c84ff/src/modules/mavlink/mavlink_main.cpp#L1436-L1463
    // https://github.com/PX4/PX4-Autopilot/blob/fe80e7aa468a50bec6b035d0e8e4e37e516c84ff/src/modules/mavlink/mavlink_main.h#L690
    // PX4 is less bursty for normal streams, we might be able to sustain higher rates of 80%..90%
    switch (txbuf_state) {
        case TXBUF_STATE_NORMAL:
            if (rate_percentage > 95) {
                txbuf = 0;                        // ArduPilot:  0-19  -> +60 ms,    PX4:  0-24  -> *0.8
            } else if (rate_percentage > 85) {
                txbuf = 30;                       // ArduPilot: 20-49  -> +20 ms,    PX4: 25-34  -> *0.975
            } else if (rate_percentage < 60) {
                txbuf = 100;                      // ArduPilot: 96-100 -> -40 ms,    PX4: 51-100 -> *1.025
            } else if (rate_percentage < 75) {
                txbuf = 91;                       // ArduPilot: 91-95  -> -20 ms,    PX4: 51-100 -> *1.025
            } else {
                txbuf = 50;                       // ArduPilot: 50-90  -> no change, PX4: 35-50  -> no change
            }
            break;

        case TXBUF_STATE_BURST:
            txbuf = 33; // just enough to stop parameter flow
            break;

        case TXBUF_STATE_BURST_HIGH:
            txbuf = 0; // slow down as much as possible
            break;

        case TXBUF_STATE_PX4_RECOVER:
            txbuf = 93; // restart data flow
            break;
    }

    // only for "educational" purposes currently
    bytes_link_out_rate_filt.Update(tnow_ms, bytes_link_out_cnt, 1000);
#if 0 // Debug
static uint32_t t_last = 0;
uint32_t t = millis32(), dt = t - t_last; t_last = t;
dbg.puts("\nMp: ");
dbg.puts(u16toBCD_s(t));dbg.puts(" (");dbg.puts(u16toBCD_s(dt));dbg.puts("), ");
//dbg.puts(u16toBCD_s(stats.GetTransmitBandwidthUsage()*41));dbg.puts(", ");
dbg.puts(u16toBCD_s(bytes_link_out));dbg.puts(", ");
dbg.puts(u16toBCD_s(serial_in_available()));dbg.puts(", ");
dbg.puts(u8toBCD_s((rate_percentage<256)?rate_percentage:255));dbg.puts(", ");
if(txbuf_state==1) dbg.puts("brst, "); else
if(txbuf_state==2) dbg.puts("high, "); else
if(txbuf_state==3) dbg.puts("recv, "); else dbg.puts("norm, ");
dbg.puts(u8toBCD_s(txbuf));dbg.puts(", ");
if(txbuf<25) dbg.puts("*0.8 "); else
if(txbuf<35) dbg.puts("*0.975 "); else
if(txbuf>50) dbg.puts("*1.025 "); else dbg.puts("*1 ");
#endif
    // increase rate faster after transient traffic since PX4 currently has no fast recovery. Could also try 100ms
    if ((txbuf_state == TXBUF_STATE_NORMAL) && (txbuf == 100)) {
        radio_status_tlast_ms -= 800; // do again in 200ms
        bytes_link_out = (bytes_link_out * 4)/5; // rolling average
    } else {
        bytes_link_out = 0; // reset, to restart rate measurement
    }

    radio_status_txbuf = txbuf;
    return true;
}


//-------------------------------------------------------
// Handle Messages
//-------------------------------------------------------

// nothing yet


//-------------------------------------------------------
// Generate Messages
//-------------------------------------------------------

// see design_decissions.h for details
void tRxMavlink::generate_radio_status(void)
{
uint8_t rssi, remrssi, txbuf, noise;

    rssi = rssi_i8_to_ap(stats.GetLastRssi());
    remrssi = rssi_i8_to_ap(stats.received_rssi);

    // we don't have a reasonable noise measurement, but can use this field to report on the snr
    // the snr can be positive and negative however, so we artificially set snr = 10 to zero
    int16_t snr = -stats.GetLastSnr() + 10;
    noise = (snr < 0) ? 0 : (snr > 127) ? 127 : snr;

    txbuf = radio_status_txbuf;

    fmav_msg_radio_status_pack(
        &msg_serial_out,
        RADIO_LINK_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO, // SiK uses 51, 68
        rssi, remrssi, txbuf, noise, UINT8_MAX, 0, 0,
        //uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed,
        &status_serial_out);
}


void tRxMavlink::generate_rc_channels_override(void)
{
    fmav_msg_rc_channels_override_pack(
        &msg_serial_out,
        GCS_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO, // ArduPilot accepts it only if it comes from its GCS sysid
        0, 0, // we do not know the sysid, compid of the flight controller
        rc_chan[0], rc_chan[1], rc_chan[2], rc_chan[3], rc_chan[4], rc_chan[5], rc_chan[6], rc_chan[7],
        rc_chan[8], rc_chan[9], rc_chan[10], rc_chan[11], rc_chan[12], rc_chan[13], rc_chan[14], rc_chan[15],
        0, 0,
        // uint8_t target_system, uint8_t target_component,
        // uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw,
        // uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw,
        // uint16_t chan17_raw, uint16_t chan18_raw,
        &status_serial_out);
}


void tRxMavlink::generate_radio_rc_channels(void)
{
int16_t channels[32]; // FASTMAVLINK_MSG_RADIO_RC_CHANNELS_FIELD_CHANNELS_NUM = 32

    memcpy(channels, rc_chan_13b, 16*2); // for (uint8_t n = 0; n < 16; n++) channels[n] = rc_chan_13b[n];
    for (uint8_t n = 16; n < 32; n++) channels[n] = 0;

    uint8_t flags = 0;
    if (rc_failsafe) flags |= RADIO_RC_CHANNELS_FLAGS_FAILSAFE;
    if (!rc_channels_uptodate) flags |= RADIO_RC_CHANNELS_FLAGS_OUTDATED;

    fmav_msg_radio_rc_channels_pack(
        &msg_serial_out,
        RADIO_LINK_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO,
        0, 0, // targets
        rc_channels_tupdated_ms, flags,
        16, channels,
        //uint8_t target_system, uint8_t target_component,
        //uint32_t time_last_update_ms, uint16_t flags,
        //uint8_t count, const int16_t* channels,
        &status_serial_out);
}


void tRxMavlink::generate_radio_link_stats(void)
{
uint8_t flags, rx_rssi1, rx_rssi2;
int8_t rx_snr1, rx_snr2;

    // Rssi are in negative dBm. Values 0..253 corresponds to 0..-253 dBm. 254 = no link connection, 255 = unknown
    flags = RADIO_LINK_STATS_FLAGS_RSSI_DBM_DEV;

    if (USE_ANTENNA1 && USE_ANTENNA2) {
        rx_rssi1 = rssi_i8_to_mavradio(stats.last_rssi1, connected());
        rx_rssi2 = rssi_i8_to_mavradio(stats.last_rssi2, connected());
        rx_snr1 = stats.last_snr1;
        rx_snr2 = stats.last_snr2;
    } else if (USE_ANTENNA2) {
        rx_rssi1 = rssi_i8_to_mavradio(stats.last_rssi2, connected());
        rx_rssi2 = UINT8_MAX; // invalidate so that rx_rssi1 is used
        rx_snr1 = stats.last_snr2;
        rx_snr2 = INT8_MAX; // invalidate so that rx_snr1 is used
    } else {
        rx_rssi1 = rssi_i8_to_mavradio(stats.last_rssi1, connected());
        rx_rssi2 = UINT8_MAX;
        rx_snr1 = stats.last_snr1;
        rx_snr2 = INT8_MAX;
    }

#define MAVLINK_USE_RADIO_LINK_STATS_MLRS
#ifdef MAVLINK_USE_RADIO_LINK_STATS_MLRS
    float freq1 = fhss.GetCurrFreq_Hz();
    float freq2 = fhss.GetCurrFreq2_Hz();

    fmav_msg_radio_link_stats_mlrs_pack(
#else
    fmav_msg_radio_link_stats_dev_pack(
#endif
        &msg_serial_out,
        RADIO_LINK_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO,
        0, 0, // targets

        flags,

        // rx stats
        rxstats.GetLQ_rc(), // uint8_t rx_LQ_rc
        rxstats.GetLQ_serial(), // uint8_t rx_LQ_ser
        rx_rssi1, // uint8_t rx_rssi1
        rx_snr1, // int8_t rx_snr1
        rx_rssi2, // uint8_t rx_rssi2
        rx_snr2, // int8_t rx_snr2

        // tx stats
        (connected()) ? stats.received_LQ_serial : 0, // uint8_t tx_LQ_ser
        rssi_i8_to_mavradio(stats.received_rssi, connected()), // uint8_t tx_rssi1
        INT8_MAX, // int8_t tx_snr1
        UINT8_MAX, // uint8_t tx_rssi2, we don't know from which antenna, thus invalidate so that rssi1 is used
        INT8_MAX, // int8_t tx_snr2, we don't know from which antenna, thus invalidate so that snr1 is used

        // antenna
        stats.last_antenna, // uint8_t rx_receive_antenna
        stats.last_transmit_antenna, // uint8_t rx_transmit_antenna
        stats.received_antenna, // uint8_t tx_receive_antenna
        stats.received_transmit_antenna, // uint8_t tx_transmit_antenna

#ifdef MAVLINK_USE_RADIO_LINK_STATS_MLRS
        // frequencies in Hz
        freq1, freq2,
#endif

        //uint8_t target_system, uint8_t target_component,
        //uint8_t flags,
        //uint8_t rx_LQ_rc, uint8_t rx_LQ_ser, uint8_t rx_rssi1, int8_t rx_snr1, uint8_t rx_rssi2, int8_t rx_snr2,
        //uint8_t tx_LQ_ser, uint8_t tx_rssi1, int8_t tx_snr1, uint8_t tx_rssi2, int8_t tx_snr2,
        //uint8_t rx_receive_antenna, uint8_t rx_transmit_antenna,
        //uint8_t tx_receive_antenna, uint8_t tx_transmit_antenna,
        &status_serial_out);
}


void tRxMavlink::generate_radio_link_information(void)
{
    uint16_t tx_ser_data_rate = 0; // ignore/unknown
    uint16_t rx_ser_data_rate = 0; // ignore/unknown

    switch (Config.Mode) {
    case MODE_50HZ: case MODE_FSK_50HZ:
        tx_ser_data_rate = 3200;
        rx_ser_data_rate = 4100;
        break;
    case MODE_31HZ:
        tx_ser_data_rate = 2000;
        rx_ser_data_rate = 2562;
        break;
    case MODE_19HZ:
        tx_ser_data_rate = 1207;
        rx_ser_data_rate = 1547;
        break;
    case MODE_FLRC_111HZ:
        tx_ser_data_rate = 7111;
        rx_ser_data_rate = 9111;
        break;
    }

    fmav_msg_radio_link_information_dev_pack(
        &msg_serial_out,
        RADIO_LINK_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO,
        0, 0, // targets

        RADIO_LINK_TYPE_MLRS_DEV, // uint8_t type
        Config.Mode, // uint8_t mode
        INT8_MAX, sx.RfPower_dbm(), // int8_t tx_power, int8_t rx_power
        Config.frame_rate_hz, Config.frame_rate_hz, // uint16_t tx_packet_rate, uint16_t rx_packet_rate,
        tx_ser_data_rate, rx_ser_data_rate, //uint16_t tx_ser_data_rate, uint16_t rx_ser_data_rate,
        sx.ReceiverSensitivity_dbm(), sx.ReceiverSensitivity_dbm(), // is equal for Tx and Rx // uint8_t tx_receive_sensitivity, uint8_t rx_receive_sensitivity

        //uint8_t target_system, uint8_t target_component,
        //uint8_t type, uint8_t mode,
        //int8_t tx_power, int8_t rx_power,
        //uint16_t tx_packet_rate, uint16_t rx_packet_rate,
        //uint16_t tx_ser_data_rate, uint16_t rx_ser_data_rate,
        //uint8_t tx_receive_sensitivity, uint8_t rx_receive_sensitivity,
        &status_serial_out);
}


void tRxMavlink::generate_cmd_ack(void)
{
    uint8_t result = MAV_RESULT_ACCEPTED;
    if (cmd_ack.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN) {
        if (!serial.has_systemboot()) result = MAV_RESULT_DENIED;
    }

    fmav_msg_command_ack_pack(
        &msg_serial_out,
        RADIO_LINK_SYSTEM_ID, MAV_COMP_ID_TELEMETRY_RADIO,
        cmd_ack.command,
        result, // result
        cmd_ack.state, // progress
        REBOOT_SHUTDOWN_MAGIC_ACK, // result_param2, set it to magic value
        cmd_ack.cmd_src_sysid,
        cmd_ack.cmd_src_compid,
        //uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2,
        //uint8_t target_system, uint8_t target_component,
        &status_serial_out);
}


void tRxMavlink::handle_cmd_long(void)
{
#ifdef USE_FEATURE_MAVLINKX
fmav_command_long_t payload;

    fmav_msg_command_long_decode(&payload, &msg_link_out);

    // check if it is for us, only allow targeted commands
    if (payload.target_system != RADIO_LINK_SYSTEM_ID) return;
    if (payload.target_component != MAV_COMP_ID_TELEMETRY_RADIO) return;

    // do we want to handle this command?
    if (payload.command != MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN &&
        payload.command != MAV_CMD_START_RX_PAIR) return;

    // now handle it

    if (payload.command != cmd_ack.command) { // a new sequence is started, so reset
        cmd_ack.state = 0;
    }

    inject_cmd_ack = true;
    cmd_ack.command = payload.command;
    cmd_ack.cmd_src_sysid = msg_link_out.sysid;
    cmd_ack.cmd_src_compid = msg_link_out.compid;

    bool cmd_valid = false;
    switch (payload.command) {
        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (!serial.has_systemboot()) break; // can't do uart flashing on this serial
            cmd_valid = (payload.param3 == 3.0f &&
                         payload.param4 == MAV_COMP_ID_TELEMETRY_RADIO &&
                         payload.param7 == (float)REBOOT_SHUTDOWN_MAGIC);
            break;
        case MAV_CMD_START_RX_PAIR:
            cmd_valid = (payload.param7 == (float)REBOOT_SHUTDOWN_MAGIC);
            break;
    }

    if (cmd_valid && payload.confirmation == 1) {
        cmd_ack.state = 1;
    } else if (cmd_valid && payload.confirmation == 2 && cmd_ack.state) {
        cmd_ack.state = 2;
        cmd_ack.texe_ms = millis32();
    } else {
        cmd_ack.state = 0;
    }
#endif
}


#endif // MAVLINK_INTERFACE_RX_H
