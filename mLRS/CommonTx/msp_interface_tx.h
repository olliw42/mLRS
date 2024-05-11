//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// MSP Interface TX Side
//*******************************************************
#ifndef MSP_INTERFACE_TX_H
#define MSP_INTERFACE_TX_H
#pragma once


#ifdef USE_FEATURE_MAVLINKX
#if 1
//#define MSP_PAYLOAD_LEN_MAX  511 // INAV7.1 uses up to 340 !!!
//#include "../Common/protocols/msp_protocol.h"
#include "../Common/thirdparty/mspx.h"


extern volatile uint32_t millis32(void);
static inline bool connected_and_rx_setup_available(void);


#define MSP_BUF_SIZE  (MSP_FRAME_LEN_MAX + 16) //300 // needs to be larger than max supported msp frame size


class tTxMsp
{
  public:
    void Init(tSerialBase* _serialport, tSerialBase* _serial2port);
    void Do(void);
    void FrameLost(void);

    void putc(char c);
    bool available(void);
    uint8_t getc(void);
    void flush(void);

  private:
    void handle_msg_serial_out(msp_message_t* msg);

    tSerialBase* ser;

    msp_status_t status_link_in;
    msp_message_t msp_msg_link_in;

    uint8_t _buf[MSP_BUF_SIZE]; // temporary working buffer, to not burden stack
};


void tTxMsp::Init(tSerialBase* _serialport, tSerialBase* _serial2port)
{
    switch (Setup.Tx[Config.ConfigId].SerialDestination) {
    case SERIAL_DESTINATION_SERIAL:
        ser = _serialport;
        break;
    case SERIAL_DESTINATION_SERIAL2:
        ser = _serial2port;
        break;
    case SERIAL_DESTINATION_MBRDIGE:
        ser = nullptr;
        break;
    default:
        while (1) {} // must not happen
    }

    msp_init();

    status_link_in = {};
}


void tTxMsp::FrameLost(void)
{
}



void tTxMsp::Do(void)
{
    if (!connected_and_rx_setup_available()) {
    }
}


void tTxMsp::putc(char c)
{
    if (!ser) return;

    ser->putc(c);

    // parse link in -> serial out
    if (msp_parse_to_msg(&msp_msg_link_in, &status_link_in, c)) {

        // allow crsf to capture it
        crsf.TelemetryHandleMspMsg(&msp_msg_link_in);

/*
        dbg.puts("\nMSP: ");
        dbg.putc(msp_msg_link_in.magic2);
        dbg.putc(msp_msg_link_in.type);
        //dbg.puts(" ");
        //dbg.puts(u16toBCD_s(msp_msg_link_in.function));
        dbg.puts(" ");
        dbg.puts(u16toBCD_s(msp_msg_link_in.len));
*/
        //dbg.puts(" ");
        dbg.puts("\n");
        char s[32]; msp_function_str(s, &msp_msg_link_in); dbg.puts(s);
        //dbg.puts(" ");
        //dbg.puts(u16toBCD_s(msp_msg_link_in.len));

    }
}


bool tTxMsp::available(void)
{
    if (!ser) return false;

    return ser->available();
}


uint8_t tTxMsp::getc(void)
{
    if (!ser) return 0;

    return ser->getc();
}


void tTxMsp::flush(void)
{
    if (!ser) return;

    ser->flush();
}


//-------------------------------------------------------
// Handle Messages
//-------------------------------------------------------

void tTxMsp::handle_msg_serial_out(msp_message_t* msg)
{
}




#else
class tTxMsp
{
  public:
    void Init(tSerialBase* _serialport, tSerialBase* _serial2port) {}
    void Do(void) {}
    void FrameLost(void) {}

    void putc(char c) { serial.putc(c); }
    bool available(void) { return serial.available(); }
    uint8_t getc(void) { return serial.getc(); }
    void flush(void) {}
};
#endif
#else // !USE_FEATURE_MAVLINKX

class tTxMsp
{
  public:
    void Init(tSerialBase* _serialport, tSerialBase* _serial2port) {}
    void Do(void) {}
    void FrameLost(void) {}

    void putc(char c) {}
    bool available(void) { return false; }
    uint8_t getc(void) { return 0; }
    void flush(void) {}
};

#endif // USE_FEATURE_MAVLINKX


#endif // MSP_INTERFACE_TX_H

