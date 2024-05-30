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
#define USE_ARQ_DBG
#define USE_ARQ_TX_SIM_MISS 4 //9 // 0 = off
#define USE_ARQ_RX_SIM_MISS 5 //5 // 0 = off

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
    void Received(uint8_t ack_seq_no);

    bool GetFreshPayload(void);
    uint8_t SeqNo(void);
    void SetRetryCnt(uint8_t retry_cnt);

    uint8_t status;
    uint8_t received_ack_seq_no; // attention: is 0/1 only, 0 = even, 1 = odd
    uint8_t payload_seq_no; // the seq_no associated to this payload
    uint8_t payload_retry_cnt; // the retry counts for this payload

    bool SimulateMiss(void);
};


void tTransmitArq::Init(void)
{
    status = ARQ_TX_IDLE;
    received_ack_seq_no = 0;
    payload_seq_no = 0;
    payload_retry_cnt = UINT8_MAX; // 255 = infinite
}


// methods called upon receive or expected receive (in doPostReceive)
// the calling sequence is:
// 1. Received(seq_no) or FrameMissed() (in handle_receive() or handle_receive_none())
// 2. Disconnected()

void tTransmitArq::Disconnected(void)
{
    status = ARQ_TX_IDLE;
}


void tTransmitArq::FrameMissed(void)
{
    status = ARQ_TX_FRAME_MISSED;
}


void tTransmitArq::Received(uint8_t ack_seq_no)
{
    received_ack_seq_no = ack_seq_no; // is 0/1
    status = ARQ_TX_RECEIVED;
}


// methods called for transmit

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
        if ((received_ack_seq_no & 0x01) != (payload_seq_no & 0x01)) return false;

        payload_seq_no++; // give this payload the next seq_no
        return true;
    case ARQ_TX_FRAME_MISSED:
        return false;
    }
    return false;
}


uint8_t tTransmitArq::SeqNo(void)
{
    return payload_seq_no; // will be converted to 3 bit or 0...7
}


void tTransmitArq::SetRetryCnt(uint8_t retry_cnt)
{
    payload_retry_cnt = UINT8_MAX; // 255 = infinite
}


// miscellaneous

bool tTransmitArq::SimulateMiss(void)
{
#if USE_ARQ_RX_SIM_MISS
    static uint8_t miss_cnt = 0;
    DECc(miss_cnt, USE_ARQ_RX_SIM_MISS);
    return (miss_cnt == 0) ? true : false;
#endif
    return false;
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
    bool FrameLost(void);

    uint8_t AckSeqNo(void);

    uint8_t status;
    uint8_t received_seq_no_last; // all seq_no here are 3 bit,  0..7
    uint8_t received_seq_no;
    uint8_t ack_seq_no;
    bool accept_received_payload; // maybe a state?
    bool frame_lost; // maybe a state?

    void spin(void);

    bool SimulateMiss(void);
};


void tReceiveArq::Init(void)
{
    status = ARQ_RX_IDLE;
    received_seq_no = 0;
    received_seq_no_last = 0;
    accept_received_payload = false;
    frame_lost = false;
    ack_seq_no = 0;
}


// methods called upon receive or expected receive (in doPreTransmit)
// the calling sequence is:
// 1. Received(seq_no) or FrameMissed() (in handle_receive() or handle_receive_none())
// 2. AcceptPayload() (in process_received_frame())
// 3. FrameLost()
// 4. Disconnected()

void tReceiveArq::Disconnected(void)
{
    status = ARQ_RX_IDLE;
}


void tReceiveArq::spin(void)
{
    if (status == ARQ_RX_IDLE) { while(1); } // must not happen, should have been called after Missed,Received

    switch (status) {
    case ARQ_RX_RECEIVED_WAS_IDLE:
        accept_received_payload = true;
        received_seq_no_last = received_seq_no;
        ack_seq_no = received_seq_no;
        break;
    case ARQ_RX_RECEIVED:
        accept_received_payload = (received_seq_no != received_seq_no_last); // new seq no received, so accept it
        received_seq_no_last = received_seq_no;
        ack_seq_no = received_seq_no;
        break;
    case ARQ_RX_FRAME_MISSED: // is not called if handle_receive_none(), RX_STATUS_NONE !
        accept_received_payload = false;
        break;
    default:
        accept_received_payload = false;
    }

    frame_lost = false;
}


void tReceiveArq::FrameMissed(void)
{
    status = ARQ_RX_FRAME_MISSED;
    spin();
}


void tReceiveArq::Received(uint8_t seq_no)
{
    received_seq_no = seq_no;
    status = (status == ARQ_RX_IDLE) ? ARQ_RX_RECEIVED_WAS_IDLE : ARQ_RX_RECEIVED;
    spin();
}


bool tReceiveArq::AcceptPayload(void)
{
    return accept_received_payload;
}


bool tReceiveArq::FrameLost(void) // called to check if parsers need to be reset
{
    return frame_lost;
}


// methods called for transmit

uint8_t tReceiveArq::AckSeqNo(void)
{
    return ack_seq_no; // will be converted by 1 bit to 0/1
}


// miscellaneous

bool tReceiveArq::SimulateMiss(void)
{
#if USE_ARQ_TX_SIM_MISS
    static uint8_t miss_cnt = 0;
    DECc(miss_cnt, USE_ARQ_TX_SIM_MISS);
    return (miss_cnt == 0) ? true : false;
#endif
    return false;
}


#else
// these should be nfc, i.e., result in exactly the same behavior as before

class tTransmitArq
{
  public:
    void Init(void) { seq_no = 0; }

    void Disconnected(void) {}
    void FrameMissed(void) {}
    void Received(uint8_t ack_seq_no) {}

    bool GetFreshPayload(void) { return true; }
    uint8_t SeqNo(void) { seq_no++; return seq_no; }
    void SetRetryCnt(uint8_t retry_cnt) {}

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
    bool FrameLost(void) { return false; }

    uint8_t AckSeqNo(void) { return 1; }
};

#undef USE_ARQ_DBG
#endif

#endif // ARQ_H
