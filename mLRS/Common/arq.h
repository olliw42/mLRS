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

#define USE_ARQ // just for the moment, eventually should go
//#define USE_ARQ_DBG
#define USE_ARQ_RETRY_CNT     -1 // -1: set by SetRetryCnt(), 0 = off, 255 = infinite,
#define USE_ARQ_TX_SIM_MISS   4 //9 // 0 = off
#define USE_ARQ_RX_SIM_MISS   3 //5 //5 // 0 = off

#ifdef USE_ARQ

/*
theory of operation
- data frame has a seq_no field of 3 bits
  seq_no numbers the serial data payload
  this allows the recipient to identify which payload it got, and decide if it is a
  fresh payload or an old
- response frame has an ack field of 1 bit
  ack holds the seq_no of the last received payload
  this allows the sender to determine if the current payload has been successfully
  delivered

a diagram showing the communication is here https://github.com/olliw42/mLRS/pull/185#issuecomment-2134020873
*/

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
    void AckReceived(uint8_t ack_seq_no);

    bool GetFreshPayload(void);
    uint8_t SeqNo(void);
    void SetRetryCnt(uint8_t retry_cnt);

    void SetRetryCntAuto(int32_t _frame_cnt, uint8_t mode);

    uint8_t status;
    uint8_t received_ack_seq_no;  // attention: is 0/1 only, 0 = even, 1 = odd
    uint8_t payload_seq_no;       // the seq_no associated to this payload
    uint8_t payload_retry_cnt;    // maximum number of allowed retries for this payload, 0 = off, 255 = infinite
    uint8_t payload_retries;      // number of retries for this payload

    bool SimulateMiss(void);
};


void tTransmitArq::Init(void)
{
    status = ARQ_TX_IDLE;
    received_ack_seq_no = 0;
    payload_seq_no = 0;
    payload_retry_cnt = UINT8_MAX; // 0 = off, 255 = infinite
    payload_retries = 0;
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


void tTransmitArq::AckReceived(uint8_t ack_seq_no)
{
    received_ack_seq_no = ack_seq_no; // is 0/1
    status = ARQ_TX_RECEIVED;
}


// methods called for transmit

// called at begin of prepare_transmit_frame()
// return true: send new payload
//        false: resend previous payload
bool tTransmitArq::GetFreshPayload(void)
{
    if (payload_retry_cnt == 0) { // ARQ disabled, new payload each time
        payload_seq_no++;
        payload_retries = 0;
        return true;
    }

    switch (status) {
    case ARQ_TX_IDLE:
        // we have no history info
        // so send new payload
        payload_seq_no++;
        payload_retries = 0;
        return true;

    case ARQ_TX_RECEIVED:
        // frame received, hence we got ack/nack
        // if received_ack_seq_no == payload_seq_no
        //   => the recipient has acked the reception => send new payload
        // else
        //   => the recipient wants the previous payload again => no new payload
        // attention: received_ack_seq_no is 1 bit!
        // next = (received_ack_seq_no & 0x01) == (payload_seq_no & 0x01)
        if ((received_ack_seq_no & 0x01) != (payload_seq_no & 0x01)) {
            // nack, recipient wants the previous payload again
            if (payload_retry_cnt == UINT8_MAX) { // ARQ with infinite retries, so never next
                payload_retries = 0;
                return false;
            } else { // ARQ with finite retries
                payload_retries++;
                if (payload_retries <= payload_retry_cnt) { // we still can retry
                    return false;
                }
            }
        }
        // recipient acked the previous payload or too many retries, shall send new payload
        payload_seq_no++; // give this payload the next seq_no
        payload_retries = 0;
        return true;

    case ARQ_TX_FRAME_MISSED:
        // no frame or invalid frame received, hence no ack/nack received
        // needs thus to be treated as nack
        // => no new payload, unless too many retries
        if (payload_retry_cnt == UINT8_MAX) { // ARQ with infinite retries, so never next
            payload_retries = 0;
            return false;
        } else { // ARQ with finite retries
            payload_retries++;
            if (payload_retries > payload_retry_cnt) { // too many retries, send new payload
                payload_seq_no++;
                payload_retries = 0;
                return true;
            }
        }
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
#ifndef USE_ARQ_DBG
    payload_retry_cnt = retry_cnt;
#else
  #if USE_ARQ_RETRY_CNT < 0
    payload_retry_cnt = retry_cnt;
  #else
    payload_retry_cnt = USE_ARQ_RETRY_CNT;
  #endif
#endif
}


void tTransmitArq::SetRetryCntAuto(int32_t _frame_cnt, uint8_t mode)
{
    switch (mode) {
    case MODE_FLRC_111HZ: // 3 -> 2 -> 1
        if (_frame_cnt >= 800) {
            SetRetryCnt(3);
        } else if (_frame_cnt >= 700) {
            SetRetryCnt(2);
        } else {
            SetRetryCnt(1);
        }
        break;
    case MODE_FSK_50HZ: // 2 -> 1
    case MODE_50HZ:
    case MODE_31HZ:
    case MODE_19HZ:
        SetRetryCnt((_frame_cnt >= 800) ? 2 : 1);
    }

    SetRetryCnt(1); // should never be called
}


// miscellaneous

bool tTransmitArq::SimulateMiss(void)
{
#if defined USE_ARQ_DBG && USE_ARQ_RX_SIM_MISS > 0
    static uint8_t miss_cnt = 0;
    DECc(miss_cnt, USE_ARQ_RX_SIM_MISS);
    if (miss_cnt == 0) return true;
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
    uint8_t received_seq_no_last;   // all seq_no here are 3 bit,  0..7
    uint8_t received_seq_no;
    uint8_t ack_seq_no;
    bool accept_received_payload;   // maybe a state?
    bool frame_lost;                // maybe a state?

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
    frame_lost = false;

    switch (status) {
    case ARQ_RX_RECEIVED_WAS_IDLE:
        // we got a frame with valid payload, but have no history info
        // so accept it
        // let's also indicate that we have had lost frames
        accept_received_payload = true;
        frame_lost = true;
        received_seq_no_last = received_seq_no;
        ack_seq_no = received_seq_no;
        break;

    case ARQ_RX_RECEIVED:
        // we got a frame with valid payload
        // if payload's seq_no is different from the last => we got a new payload
        accept_received_payload = (received_seq_no != received_seq_no_last); // new seq no received, so accept it

        // the received seq_no is 3 bits
        // we can check if we lost a frame if the received seq no is larger than just +1
        // this fails if it had been 8-1 = 7 missed frames
        if (((received_seq_no - received_seq_no_last) & 0x07) > 1) frame_lost = true;

        received_seq_no_last = received_seq_no;
        ack_seq_no = received_seq_no;
        break;

    case ARQ_RX_FRAME_MISSED:
        // no frame or invalid frame received, hence no new payload received
        accept_received_payload = false;
        break;

    default: // ARQ_RX_IDLE
        while(1){} // must not happen, should have been called after FrameMissed(), Received()
    }
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


// called to check if parsers need to be reset
// must be called after FrameMissed(), Received(), but before payload is passed on
bool tReceiveArq::FrameLost(void)
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
#if defined USE_ARQ_DBG && USE_ARQ_TX_SIM_MISS > 0
    static uint8_t miss_cnt = 0;
    DECc(miss_cnt, USE_ARQ_TX_SIM_MISS);
    if (miss_cnt == 0) return true;
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
    void AckReceived(uint8_t ack_seq_no) {}

    bool GetFreshPayload(void) { return true; }
    uint8_t SeqNo(void) { seq_no++; return seq_no; }
    void SetRetryCnt(uint8_t retry_cnt) {}
    void SetRetryCntAuto(int32_t _frame_cnt, uint8_t mode) {}

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
