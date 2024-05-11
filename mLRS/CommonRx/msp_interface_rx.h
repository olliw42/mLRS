//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// MSP Interface RX Side
//*******************************************************
#ifndef MSP_INTERFACE_RX_H
#define MSP_INTERFACE_RX_H
#pragma once


#ifdef USE_FEATURE_MAVLINKX
#if 0
#include "../Common/libs/fifo.h"
#define MSP_PAYLOAD_LEN_MAX  511 // INAV7.1 uses up to 340 !!!
#include "../Common/thirdparty/mspx.h"


extern volatile uint32_t millis32(void);
static inline bool connected(void);


#define MSP_BUF_SIZE  (MSP_FRAME_LEN_MAX + 16) //300 // needs to be larger than max supported msp frame size


class tRxMsp
{
  public:
    void Init(void);
    void Do(void);
    void FrameLost(void);

    void putc(char c);
    bool available(void);
    uint8_t getc(void);
    void flush(void);

  private:

    msp_status_t status_ser_in;
    msp_message_t msp_msg_ser_in;

    msp_status_t status_link_out;
    msp_message_t msp_msg_link_out;

    FifoBase<char,2*512> fifo_link_out; // needs to be at least 82 + 280

    uint8_t _buf[MSP_BUF_SIZE]; // temporary working buffer, to not burden stack
};


void tRxMsp::Init(void)
{
    msp_init();

    status_ser_in = {};
    status_link_out = {};

    fifo_link_out.Init();
}


void tRxMsp::Do(void)
{
    if (!connected()) {
        fifo_link_out.Flush();
    }

    if (fifo_link_out.HasSpace(MSP_FRAME_LEN_MAX + 16)) { // we have space for a full MSP message, so can safely parse
        while (serial.available()) {
            char c = serial.getc();
//            fifo_link_out.Put(c);

            if (msp_parse_to_msg(&msp_msg_ser_in, &status_ser_in, c)) {
                uint16_t len = msp_msg_to_frame_buf(_buf, &msp_msg_ser_in);
                fifo_link_out.PutBuf(_buf, len);

/*
                dbg.puts("\nMSP: ");
                dbg.putc(msp_msg_ser_in.magic2);
                dbg.putc(msp_msg_ser_in.type);
                dbg.puts(" ");
                dbg.puts(u16toBCD_s(msp_msg_ser_in.function));
                dbg.puts(" ");
                dbg.puts(u16toBCD_s(msp_msg_ser_in.len));
*/
            }

        }
    }
}


void tRxMsp::FrameLost(void)
{
}


void tRxMsp::putc(char c)
{
    serial.putc(c);
}


bool tRxMsp::available(void)
{
    return fifo_link_out.Available();
    //return serial.available();
}


uint8_t tRxMsp::getc(void)
{
    return fifo_link_out.Get();
    //return serial.getc();
}


void tRxMsp::flush(void)
{
    fifo_link_out.Flush();
    // serial is flushed by caller
}


#else
class tRxMsp
{
  public:
    void Init(void) {}
    void Do(void) {}
    void FrameLost(void) {}

    void putc(char c) { serial.putc(c); }
    bool available(void) { return serial.available(); }
    uint8_t getc(void) { return serial.getc(); }
    void flush(void) {}
};
#endif
#else //!USE_FEATURE_MAVLINKX

#endif //USE_FEATURE_MAVLINKX


#endif // MSP_INTERFACE_RX_H
