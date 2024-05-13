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
#if 1
#include "../Common/libs/fifo.h"
#include "../Common/thirdparty/mspx.h"


extern volatile uint32_t millis32(void);
static inline bool connected(void);


#define MSP_BUF_SIZE  (MSP_FRAME_LEN_MAX + 16) // needs to be larger than max supported msp frame size


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

    // fields for link in -> parser -> serial out
    msp_status_t status_link_in;
    msp_message_t msp_msg_link_in;

    // fields for serial in -> parser -> link out
    msp_status_t status_ser_in;
    msp_message_t msp_msg_ser_in;
    FifoBase<char,2*512> fifo_link_out; // needs to be at least ??

    // to inject requests if no requests from a gcs
    uint32_t msp_request_tlast_ms;
    uint32_t tick_tlast_ms;
    uint8_t tick;

    uint8_t _buf[MSP_BUF_SIZE]; // temporary working buffer, to not burden stack
};


void tRxMsp::Init(void)
{
    msp_init();

    status_link_in = {};
    status_ser_in = {};
    fifo_link_out.Init();

    msp_request_tlast_ms = 0;
    tick_tlast_ms = 0;
    tick = 0;
}


void tRxMsp::Do(void)
{
    if (!connected()) {
        fifo_link_out.Flush();
    }

    // parse serial in -> link out
    if (fifo_link_out.HasSpace(MSP_FRAME_LEN_MAX + 16)) { // we have space for a full MSP message, so can safely parse
        while (serial.available()) {
            char c = serial.getc();
            if (msp_parse_to_msg(&msp_msg_ser_in, &status_ser_in, c)) {
#ifdef USE_MSPX
                uint16_t len = msp_msg_to_frame_bufX(_buf, &msp_msg_ser_in); // converting to mspX !!
#else
                uint16_t len = msp_msg_to_frame_buf(_buf, &msp_msg_ser_in);
#endif

                fifo_link_out.PutBuf(_buf, len);

/*
dbg.puts("\n");
dbg.putc(msp_msg_ser_in.type);
char s[32]; msp_function_str_from_msg(s, &msp_msg_ser_in); dbg.puts(s);
dbg.puts(" ");
dbg.puts(u16toBCD_s(msp_msg_ser_in.len));
*/
            }

        }
    }


    uint32_t tnow_ms = millis32();
    bool ticked = false;
    if ((tnow_ms - tick_tlast_ms) >= 100) {
        tick_tlast_ms = tnow_ms;
        INCc(tick, 20);
        ticked = true;
    }

    if ((tnow_ms - msp_request_tlast_ms) < 1500) return; // got a request recently

    if (ticked)
    switch (tick) {
    case 0: case 5: case 10: case 15: {
        uint16_t len = msp_generate_request_to_frame_buf(_buf, MSP_TYPE_REQUEST, MSP_ATTITUDE);
        serial.putbuf(_buf, len);
        }break;

    case 2: case 6: case 11: case 16: {
        uint16_t len = msp_generate_request_to_frame_buf(_buf, MSP_TYPE_REQUEST, MSP_INAV_STATUS);
        serial.putbuf(_buf, len);
        }break;

    case 3: case 7: case 12: case 17: {
        uint16_t len = msp_generate_request_to_frame_buf(_buf, MSP_TYPE_REQUEST, MSP_ALTITUDE);
        serial.putbuf(_buf, len);
        }break;

    }
}


void tRxMsp::FrameLost(void)
{
    msp_parse_reset(&status_link_in);
}


void tRxMsp::putc(char c)
{
    // parse link in -> serial out
#ifdef USE_MSPX
    if (msp_parseX_to_msg(&msp_msg_link_in, &status_link_in, c)) { // converting from mspX
#else
    if (msp_parse_to_msg(&msp_msg_link_in, &status_link_in, c)) {
#endif
        uint16_t len = msp_msg_to_frame_buf(_buf, &msp_msg_link_in);
        serial.putbuf(_buf, len);

        if (msp_msg_link_in.type == MSP_TYPE_REQUEST) {
            msp_request_tlast_ms = millis32();
        }

/*
dbg.puts("\n");
dbg.putc(msp_msg_link_in.type);
char s[32]; msp_function_str_from_msg(s, &msp_msg_link_in); dbg.puts(s);
dbg.puts(" ");
dbg.puts(u16toBCD_s(msp_msg_link_in.len));
dbg.puts(" ");
dbg.puts(u8toHEX_s(msp_msg_link_in.checksum));
uint8_t crc8 = crsf_crc8_update(0, &(_buf[3]), len - 4);
dbg.puts(" ");
dbg.puts(u8toHEX_s(crc8));
*/
    }
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
