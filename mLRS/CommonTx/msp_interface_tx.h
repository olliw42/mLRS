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


// TODO: share fixed buffers with mavlink interface


#ifdef USE_FEATURE_MAVLINKX
#include "../Common/protocols/msp_protocol.h"
#include "../Common/thirdparty/mspx.h"


extern volatile uint32_t millis32(void);
extern bool connected_and_rx_setup_available(void);


#define MSP_BUF_SIZE  (MSP_FRAME_LEN_MAX + 16) // needs to be larger than max supported MSP frame size


class tTxMsp
{
  public:
    void Init(tSerialBase* const _serialport, tSerialBase* const _serial2port);
    void Do(void);
    void FrameLost(void);

    void putc(char c);
    bool available(void);
    uint8_t getc(void);
    void flush(void);

  private:

    tSerialBase* ser;

    // fields for link in -> parser -> serial out
    msp_status_t status_link_in;
    msp_message_t msp_msg_link_in;

    // fields for serial in -> parser -> link out
    msp_status_t status_ser_in;
    msp_message_t msp_msg_ser_in;
    tFifo<char,2*512> fifo_link_out; // needs to be at least ??

    uint8_t _buf[MSP_BUF_SIZE]; // temporary working buffer, to not burden stack
};


void tTxMsp::Init(tSerialBase* const _serialport, tSerialBase* const _serial2port)
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
        while(1){} // must not happen
    }

    msp_init();

    status_link_in = {};
    status_ser_in = {};
    fifo_link_out.Init();
}


void tTxMsp::FrameLost(void)
{
    msp_parse_reset(&status_link_in);
}


void tTxMsp::Do(void)
{
    if (!connected_and_rx_setup_available()) {
        fifo_link_out.Flush();
    }

    if (!SERIAL_LINK_MODE_IS_MSP(Setup.Rx.SerialLinkMode)) return;

    // parse serial in -> link out
    if (fifo_link_out.HasSpace(MSP_FRAME_LEN_MAX + 16)) { // we have space for a full MSP message, so can safely parse
        while (ser->available()) {
            char c = ser->getc();
            if (msp_parse_to_msg(&msp_msg_ser_in, &status_ser_in, c)) {
                uint16_t len = msp_msg_to_frame_bufX(_buf, &msp_msg_ser_in); // converting to mspX
                fifo_link_out.PutBuf(_buf, len);
            }
        }
    }
}


void tTxMsp::putc(char c)
{
    if (!ser) return;

    // parse link in -> serial out
    if (msp_parseX_to_msg(&msp_msg_link_in, &status_link_in, c)) { // converting from mspX
        bool send = true;

        if (msp_msg_link_in.type == MSP_TYPE_RESPONSE) {
            switch (msp_msg_link_in.function) {
            case MSP_BOXNAMES:
//dbg.puts("\nMBC "); for(uint16_t i= 0; i< msp_msg_link_in.len; i++) dbg.puts(u8toHEX_s(msp_msg_link_in.payload[i]));
                msp_msg_link_in.flag = MSP_FLAG_NONE; // it should be set to MSP_FLAG_SOURCE_ID_RC_LINK, not what we want
                // decompress
                mspX_boxnames_payload_decompress(&msp_msg_link_in, _buf); // we can use the buffer
//dbg.puts("\nMB ");for(uint16_t i = 0; i < msp_msg_link_in.len; i++) dbg.putc(msp_msg_link_in.payload[i]);
                break;
            case MSPX_STATUS:
                // it's our home brew, catch it
                send = false;
                break;
            }

            if (msp_msg_link_in.flag & MSP_FLAG_SOURCE_ID_RC_LINK) {
                // it's requested by the receiver, catch it
                send = false;
            }
        }

        if (send) {
            uint16_t len = msp_msg_to_frame_buf(_buf, &msp_msg_link_in);
            ser->putbuf(_buf, len);
        }

        // allow crsf class to capture it
        if (msp_msg_link_in.type == MSP_TYPE_RESPONSE) {
            crsf.TelemetryHandleMspMsg(&msp_msg_link_in);
        }

/*
dbg.puts("\n");
dbg.putc(msp_msg_link_in.type);
char s[32]; msp_function_str_from_msg(s, &msp_msg_link_in); dbg.puts(s);
dbg.puts(" ");
dbg.puts(u16toBCD_s(msp_msg_link_in.len));
if(msp_msg_link_in.len>64){
dbg.puts(" ");for (uint16_t n=0;n<((msp_msg_link_in.len<100)?msp_msg_link_in.len:50); n++) dbg.putc(msp_msg_link_in.payload[n]);}
*/
/*dbg.puts(" ");
dbg.puts(u8toHEX_s(msp_msg_link_in.checksum));
uint16_t _len = msp_msg_to_frame_buf(_buf, &msp_msg_link_in);
uint8_t crc8 = crsf_crc8_update(0, &(_buf[3]), _len - 4);
dbg.puts(" ");
dbg.puts(u8toHEX_s(crc8));
*/
    }
}


bool tTxMsp::available(void)
{
    if (!ser) return false;

    return fifo_link_out.Available();
    //return ser->available();
}


uint8_t tTxMsp::getc(void)
{
    if (!ser) return 0;

    return fifo_link_out.Get();
    //return ser->getc();
}


void tTxMsp::flush(void)
{
    if (!ser) return;

    fifo_link_out.Flush();
    ser->flush();
}


#else // !USE_FEATURE_MAVLINKX

class tTxMsp
{
  public:
    void Init(tSerialBase* const _serialport, tSerialBase* const _serial2port) {}
    void Do(void) {}
    void FrameLost(void) {}

    void putc(char c) {}
    bool available(void) { return false; }
    uint8_t getc(void) { return 0; }
    void flush(void) {}
};

#endif // USE_FEATURE_MAVLINKX


#endif // MSP_INTERFACE_TX_H

