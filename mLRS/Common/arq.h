//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// ARQ Transmit/Receive
//*******************************************************
#ifndef ARQ_H
#define ARQ_H
#pragma once

#define USE_ARQ
#define USE_ARQ_TX_SIM_MISS 0 //9
#define USE_ARQ_RX_SIM_MISS 0 //5

#ifdef USE_ARQ

//-------------------------------------------------------
// Transmit
//-------------------------------------------------------

class tTransmitArq
{
  public:
    void Init(void);

    typedef enum {
        ARQ_TX_IDLE = 0,
        ARQ_TX_FRAME_MISSED,
        ARQ_TX_NACK,
        ARQ_TX_ACK,
    } ARQ_TX_ENUM;

    void Disconnected(void);
    void FrameMissed(void);
    void NAck(void);
    void Ack(void);

    void Clear(void);
    bool GetFreshPayload(void);
    void PutC(char c);
    uint8_t SeqNo(void);

    uint8_t status;

    uint8_t payload_seq_no; // the seq_no associated to this payload
    uint8_t bytes_in_payload;
    uint8_t payload[FRAME_RX_PAYLOAD_LEN];

    uint8_t payload_len; // not necessarily the same as bytes_in_payload
};


void tTransmitArq::Init(void)
{
    status = ARQ_TX_IDLE;
    payload_seq_no = 0;
    bytes_in_payload = 0;
    payload_len = 0;
}


void tTransmitArq::Disconnected(void)
{
    status = ARQ_TX_IDLE;
}


void tTransmitArq::FrameMissed(void)
{
    status = ARQ_TX_FRAME_MISSED;
}


void tTransmitArq::NAck(void)
{
    status = ARQ_TX_NACK;
}


void tTransmitArq::Ack(void)
{
    status = ARQ_TX_ACK;
}


void tTransmitArq::Clear(void)
{
    payload_len = 0;
}


bool tTransmitArq::GetFreshPayload(void)
{
    if (status == ARQ_TX_IDLE) { while(1); } // must not happen, should have been called after Missed,NAck,Ack

    if (status == ARQ_TX_ACK) {
        payload_seq_no++; // give this payload the next seq_no
        bytes_in_payload = 0; // to prepare for PutC()
        return true;
    }

    return false;
}


void tTransmitArq::PutC(char c)
{
    payload[bytes_in_payload++] = c;
    payload_len = bytes_in_payload;
}


uint8_t tTransmitArq::SeqNo(void)
{
    return payload_seq_no;
}


//-------------------------------------------------------
// Receive
//-------------------------------------------------------

class tReceiveArq
{
  public:
    void Init(void);

    typedef enum {
        ARQ_RX_IDLE = 0,
        ARQ_RX_FRAME_MISSED,
        ARQ_RX_RECEIVED_WAS_IDLE,
        ARQ_RX_RECEIVED,
    } ARQ_RX_ENUM;

    void Disconnected(void);
    void FrameMissed(void);
    void Received(uint8_t _seq_no);
    bool AcceptPayload(void);

    uint8_t Ack(void);

    uint8_t status;

    uint8_t received_seq_no_last;
    uint8_t received_seq_no;

    uint8_t ack;
};


void tReceiveArq::Init(void)
{
    status = ARQ_RX_IDLE;
    received_seq_no = 0;
    received_seq_no_last = 0;
    ack = 1;
}


void tReceiveArq::Disconnected(void)
{
    status = ARQ_RX_IDLE;
}


void tReceiveArq::FrameMissed(void)
{
    status = ARQ_RX_FRAME_MISSED;
    ack = 0; // AcceptPayload() is not called if FRAME_MISSED
}


void tReceiveArq::Received(uint8_t _seq_no)
{
    received_seq_no = _seq_no;
    status = (status == ARQ_RX_IDLE) ? ARQ_RX_RECEIVED_WAS_IDLE : ARQ_RX_RECEIVED;
    // AcceptPayload() is called
}


bool tReceiveArq::AcceptPayload(void)
{
    if (status == ARQ_RX_IDLE) { while(1); } // must not happen, should have been called after Missed,Received

    switch (status) {
    case ARQ_RX_RECEIVED_WAS_IDLE:
        ack = 1;
        received_seq_no_last = received_seq_no;
        return true;
    case ARQ_RX_RECEIVED:
        ack = 1; // we always need to ack when we received a frame, to tell sender we want a new frame
        if (received_seq_no == received_seq_no_last) { // that's the same as before, reject
            return false;
        }
        received_seq_no_last = received_seq_no;
        return true;
    case ARQ_RX_FRAME_MISSED: // is not called if FRAME_MISSED !!
    default:
        ack = 0; // respond with NAck
        return false;
    }
}


uint8_t tReceiveArq::Ack(void)
{
    return ack;
}


#else
// these should be nfc, i.e., result in exactly the same behavior as before

class tTransmitArq
{
  public:
    void Init(void) { seq_no = 0; }

    void Disconnected(void) {}
    void FrameMissed(void) {}
    void NAck(void) {}
    void Ack(void) {}

    void Clear(void) { payload_len = 0; }
    bool GetFreshPayload(void) { return true; }
    void PutC(char c) { payload[payload_len++] = c; }
    uint8_t SeqNo(void) { seq_no++; return seq_no; }

    uint8_t payload_len;
    uint8_t payload[FRAME_RX_PAYLOAD_LEN];
    uint8_t seq_no;
};


class tReceiveArq
{
  public:
    void Init(void) {}

    void Disconnected(void) {}
    void FrameMissed(void) {}
    void Received(uint8_t _seq_no) {}
    bool AcceptPayload(void) { return true; }

    uint8_t Ack(void) { return 1; }
};

#endif

#endif // ARQ_H
