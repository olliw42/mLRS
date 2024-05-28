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
#define USE_ARQ_TX_SIM_MISS 4 //9
#define USE_ARQ_RX_SIM_MISS 5 //5

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
        ARQ_TX_RECEIVED,
    } ARQ_TX_ENUM;

    void Disconnected(void);
    void FrameMissed(void);
    void Received(uint8_t seq_no);

    bool GetFreshPayload(void);

    uint8_t SeqNo(void);

    uint8_t status;
    uint8_t received_seq_no; // attention: is 0/1 only, 0 = even, 1 = odd
    uint8_t payload_seq_no; // the seq_no associated to this payload
};


void tTransmitArq::Init(void)
{
    status = ARQ_TX_IDLE;
    received_seq_no = 0;
    payload_seq_no = 0;
}


void tTransmitArq::Disconnected(void)
{
    status = ARQ_TX_IDLE;
}


void tTransmitArq::FrameMissed(void)
{
    status = ARQ_TX_FRAME_MISSED;
}


void tTransmitArq::Received(uint8_t seq_no)
{
    received_seq_no = seq_no;
    status = ARQ_TX_RECEIVED;
}


// called at begin of prepare_transmit_frame()
bool tTransmitArq::GetFreshPayload(void)
{
    switch (status) {
    case ARQ_TX_IDLE:
        payload_seq_no++;
        return true;
    case ARQ_TX_RECEIVED:
        // keep previous payload if received_seq_no != payload_seq_no
        // attention: received_seq_no is 1 bit!
        // next = (received_seq_no & 0x01) == (payload_seq_no & 0x01)
        if ((received_seq_no & 0x01) != (payload_seq_no & 0x01)) return false;

        payload_seq_no++; // give this payload the next seq_no
        return true;
    case ARQ_TX_FRAME_MISSED:
        return false;
    }
    return false;
}


uint8_t tTransmitArq::SeqNo(void)
{
    return payload_seq_no; // will be converted by 3 bit to 0...7
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
    void Received(uint8_t seq_no);

    bool AcceptPayload(void);

    uint8_t AckSeqNo(void);

    uint8_t status;
    uint8_t received_seq_no_last; // attention: the seq no here are all 3 bit,  0..7
    uint8_t received_seq_no;
    uint8_t ack_seq_no;
};


void tReceiveArq::Init(void)
{
    status = ARQ_RX_IDLE;
    received_seq_no = 0;
    received_seq_no_last = 0;
    ack_seq_no = 0;
}


void tReceiveArq::Disconnected(void)
{
    status = ARQ_RX_IDLE;
}


void tReceiveArq::FrameMissed(void)
{
    status = ARQ_RX_FRAME_MISSED;
}


void tReceiveArq::Received(uint8_t seq_no)
{
    received_seq_no = seq_no;
    status = (status == ARQ_RX_IDLE) ? ARQ_RX_RECEIVED_WAS_IDLE : ARQ_RX_RECEIVED;
    // AcceptPayload() is called
}


bool tReceiveArq::AcceptPayload(void)
{
    if (status == ARQ_RX_IDLE) { while(1); } // must not happen, should have been called after Missed,Received

    switch (status) {
    case ARQ_RX_RECEIVED_WAS_IDLE: {
        bool accept = true;
        received_seq_no_last = received_seq_no;
        ack_seq_no = received_seq_no;
        return accept; }
    case ARQ_RX_RECEIVED: {
        bool accept = (received_seq_no != received_seq_no_last); // new seq no received, so accept it
        received_seq_no_last = received_seq_no;
        ack_seq_no = received_seq_no;
        return accept; }
    case ARQ_RX_FRAME_MISSED: // is not called if handle_receive_none(), RX_STATUS_NONE !
        return false;
    }
    return false;
}


uint8_t tReceiveArq::AckSeqNo(void)
{
    return ack_seq_no; // will be converted by 1 bit to 0/1
}


#else
// these should be nfc, i.e., result in exactly the same behavior as before

class tTransmitArq
{
  public:
    void Init(void) { seq_no = 0; }

    void Disconnected(void) {}
    void FrameMissed(void) {}
    void Received(uint8_t seq_no) {}

    bool GetFreshPayload(void) { return true; }
    uint8_t SeqNo(void) { seq_no++; return seq_no; }

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

    uint8_t AckSeqNo(void) { return 1; }
};

#endif

#endif // ARQ_H
