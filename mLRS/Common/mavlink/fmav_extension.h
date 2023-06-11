//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Mavlink
//*******************************************************
#pragma once
#ifndef FASTMAVLINK_EXTENSION_H
#define FASTMAVLINK_EXTENSION_H

#include "../thirdparty/thirdparty.h"


//-------------------------------------------------------
// helper
//-------------------------------------------------------

void fmav_msg_recalculate_crc(fmav_message_t* msg)
{
    uint16_t crc = fmav_crc_calculate(&(msg->len), 1);
    fmav_crc_accumulate(&crc, msg->incompat_flags);
    fmav_crc_accumulate(&crc, msg->compat_flags);
    fmav_crc_accumulate(&crc, msg->seq);
    fmav_crc_accumulate(&crc, msg->sysid);
    fmav_crc_accumulate(&crc, msg->compid);
    fmav_crc_accumulate_buf(&crc, msg->msgid_a, 3);

    fmav_crc_accumulate_buf(&crc, msg->payload, msg->len);
    fmav_crc_accumulate(&crc, msg->crc_extra);
    msg->checksum = crc;
}


//-------------------------------------------------------
// Component List Class
//-------------------------------------------------------

typedef struct {
    uint8_t sys_id;
    uint8_t comp_id;
    uint8_t seq_last;
} tComponent;


template <int LIST_SIZE>
class ComponentList
{
  public:
    ComponentList() // constructor
    {
        Init();
    }

    void Init(void)
    {
        for (uint8_t i = 0; i < LIST_SIZE; i++) buf[i].empty = true;
    }

    uint8_t FindComp(uint8_t sys_id, uint8_t comp_id)
    {
        for (uint8_t i = 0; i < LIST_SIZE; i++) {
            if (buf[i].empty) continue;
            if (buf[i].element.sys_id == sys_id && buf[i].element.comp_id == comp_id) return i;
        }
        return UINT8_MAX;
    }

    uint8_t AddComp(uint8_t sys_id, uint8_t comp_id)
    {
        for (uint8_t i = 0; i < LIST_SIZE; i++) {
            if (!buf[i].empty) continue;
            buf[i].element.sys_id = sys_id;
            buf[i].element.comp_id = comp_id;
            buf[i].empty = false;
            return i;
        }
        return UINT8_MAX;
    }

    // this ensures, possibly the hard way, that a valid index is returned
    uint8_t FindAndAdd(uint8_t sys_id, uint8_t comp_id, uint8_t seq)
    {
        uint8_t i = FindComp(sys_id, comp_id);

        if (i != UINT8_MAX) { // is in list, so return the index
            return i;
        }

        // not in list, so add it
        i = AddComp(sys_id, comp_id);

        if (i != UINT8_MAX) { // is now in list, so set the seq no. and return the index
            buf[i].element.seq_last = seq;
            return i;
        }

        // list is full, so replace last element
        i = LIST_SIZE - 1;
        buf[i].element.sys_id = sys_id;
        buf[i].element.comp_id = comp_id;
        buf[i].element.seq_last = seq;
        buf[i].empty = true;
        return i;
    }

    uint8_t GetSeq(uint8_t i)
    {
      if (i >= LIST_SIZE) return 0; // just to play it safe, must not happen
      if (buf[i].empty) return 0;

      return buf[i].element.seq_last;
    }

    bool SetSeq(uint8_t i, uint8_t seq)
    {
        if (i >= LIST_SIZE) return false;
        if (buf[i].empty) return false;

        buf[i].element.seq_last = seq;
        return true;
    }

    typedef struct {
        bool empty;
        tComponent element;
    } tListElement;

    const uint8_t size = LIST_SIZE;
    tListElement buf[LIST_SIZE];
};


//-------------------------------------------------------
// Mavlink X
//-------------------------------------------------------
// first attempt
// we parse the stream and simply substitute the header

// the original fmav construct for sending looks like:
//
//    if (fmav_parse_and_check_to_frame_buf(&result_link_in, buf_link_in, &status_link_in, c)) {
//        fmav_frame_buf_to_msg(&msg_serial_out, &result_link_in, buf_link_in);
//        uint16_t len = fmav_msg_to_frame_buf(_buf, &msg_serial_out);
//        serial.putbuf(_buf, len);
//    }
//
// this reads the complete frame, so adds latency, but for the moment we do it to keep it simple
// should be ok for testing and dev-ing

#define MAVLINKX_MAGIC_1            'o' // TODO: do we really need two STX bytes?
#define MAVLINKX_MAGIC_2            'w'
#define MAVLINKX_HEADER_LEN_MAX     13 // STX1 STX2 flags len seq sysid compid msgid1 msgid2 msgid3 tsysid tcompid crc8
#define MAVLINKX_FRAME_LEN_MAX      (MAVLINKX_HEADER_LEN_MAX+FASTMAVLINK_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


typedef enum {
    MAVLINKX_FLAGS_IS_V1            = 0x01,
    MAVLINKX_FLAGS_HAS_SIGNATURE    = 0x02,
    MAVLINKX_FLAGS_IS_TARGETED      = 0x04,
    MAVLINKX_FLAGS_HAS_MSGID16      = 0x08,
    MAVLINKX_FLAGS_HAS_SYSID16      = 0x10, // not used, it's more to indicate the possibility
    MAVLINKX_FLAGS_IS_COMPRESSED    = 0x20, // not used currently
} fmavx_flags_e;


typedef enum {
    FASTMAVLINK_PARSE_STATE_MAGIC_2 = 100,
    FASTMAVLINK_PARSE_STATE_FLAGS,
    FASTMAVLINK_PARSE_STATE_TARGET_SYSID,
    FASTMAVLINK_PARSE_STATE_TARGET_COMPID,
    FASTMAVLINK_PARSE_STATE_CRC8,
} fmavx_parse_state_e;


typedef struct _fmavx_status {
    uint8_t flags;
    uint8_t header[16];
    uint8_t pos;
    uint8_t target_sysid;
    uint8_t target_compid;
} fmavx_status_t;


void fmavX_status_reset(fmavx_status_t* status)
{
    status->flags = 0;
    status->pos = 0;
}


uint8_t fmavX_crc8_calculate(const uint8_t* buf, uint16_t len)
{
    // TODO: what would be the best crc8 for our purpose?
    return crc8_update(0, buf, len, 0xD5); // CRSF's crc8
}


fmavx_status_t fmavx_status = {0};


// convert fmav msg structure into fmavX packet
// fmav_message_t has all info on the packet which we need, so it's quite straightforward to do

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmavX_msg_to_frame_buf(uint8_t* buf, fmav_message_t* msg)
{
    uint16_t pos = 0;

    // new magic
    // we want to identify something which is really rare
    buf[0] = MAVLINKX_MAGIC_1;
    buf[1] = MAVLINKX_MAGIC_2;

    // flags
    buf[2] = 0;
    if (msg->magic == FASTMAVLINK_MAGIC_V1) {
        buf[2] |= MAVLINKX_FLAGS_IS_V1;
    }
    if (msg->incompat_flags & FASTMAVLINK_INCOMPAT_FLAGS_SIGNED) {
        buf[2] |= MAVLINKX_FLAGS_HAS_SIGNATURE;
    }
    if (msg->msgid < 65536) {
        buf[2] |= MAVLINKX_FLAGS_HAS_MSGID16;
    }
    if (msg->target_sysid > 0 || msg->target_compid > 0) {
        buf[2] |= MAVLINKX_FLAGS_IS_TARGETED;
    }

    // more
    buf[3] = msg->len;
    buf[4] = msg->seq;
    buf[5] = msg->sysid;
    buf[6] = msg->compid;

    // msgid
    buf[7] = (uint8_t)msg->msgid;
    buf[8] = (uint8_t)((msg->msgid) >> 8);
    pos = 9;
    if (!(buf[2] & MAVLINKX_FLAGS_HAS_MSGID16)) {
        buf[pos++] = (uint8_t)((msg->msgid) >> 16);
    }

    // targets
    if (buf[2] & MAVLINKX_FLAGS_IS_TARGETED) {
        buf[pos++] = msg->target_sysid;
        buf[pos++] = msg->target_compid;
    }

    // crc8
    uint8_t crc8 = fmavX_crc8_calculate(buf, pos);
    buf[pos++] = crc8;

    // payload
    // we should want to remove the targets if there are any, for the moment we just don't
    memcpy(&(buf[pos]), msg->payload, msg->len);
    pos += msg->len;

    // crc16
    buf[pos++] = (uint8_t)msg->checksum;
    buf[pos++] = (uint8_t)((msg->checksum) >> 8);

    // signature
    if (msg->incompat_flags & FASTMAVLINK_INCOMPAT_FLAGS_SIGNED) {
        memcpy(&(buf[pos]), msg->signature_a, FASTMAVLINK_SIGNATURE_LEN);
        pos += FASTMAVLINK_SIGNATURE_LEN;
    }

    return pos;
}


// parse the stream into regular fmav msg
// returns NONE, HAS_HEADER, or OK
FASTMAVLINK_FUNCTION_DECORATOR void _fmavX_parse_header_to_frame_buf(fmav_result_t* result, uint8_t* buf, fmav_status_t* status, uint8_t c)
{
    fmavx_status.header[fmavx_status.pos++] = c; // memorize

    switch (status->rx_state) {
    case FASTMAVLINK_PARSE_STATE_MAGIC_2:
        if (c == MAVLINKX_MAGIC_2) {
            status->rx_state = FASTMAVLINK_PARSE_STATE_FLAGS;
        } else {
            fmav_parse_reset(status);
        }
        return;

    case FASTMAVLINK_PARSE_STATE_FLAGS:{
        fmavx_status.flags = c;

        uint8_t magic = (fmavx_status.flags & MAVLINKX_FLAGS_IS_V1) ? FASTMAVLINK_MAGIC_V1 : FASTMAVLINK_MAGIC_V2;
        buf[status->rx_cnt++] = magic; // STX

        status->rx_state = FASTMAVLINK_PARSE_STATE_LEN;
        }return;

    case FASTMAVLINK_PARSE_STATE_LEN:
        buf[status->rx_cnt++] = c; // len

        if (fmavx_status.flags & MAVLINKX_FLAGS_IS_V1) {
            status->rx_header_len = FASTMAVLINK_HEADER_V1_LEN;
            status->rx_frame_len = (uint16_t)c + FASTMAVLINK_HEADER_V1_LEN + FASTMAVLINK_CHECKSUM_LEN;
        } else {
            status->rx_header_len = FASTMAVLINK_HEADER_V2_LEN;
            status->rx_frame_len = (uint16_t)c + FASTMAVLINK_HEADER_V2_LEN + FASTMAVLINK_CHECKSUM_LEN;

            uint8_t incompat_flags = (fmavx_status.flags & MAVLINKX_FLAGS_HAS_SIGNATURE) ? FASTMAVLINK_INCOMPAT_FLAGS_SIGNED : 0;
            buf[status->rx_cnt++] = incompat_flags; // incompat_flags
            buf[status->rx_cnt++] = 0; // compat_flags

            if (incompat_flags & FASTMAVLINK_INCOMPAT_FLAGS_SIGNED) {
                status->rx_frame_len += FASTMAVLINK_SIGNATURE_LEN;
            }
        }

        status->rx_state = FASTMAVLINK_PARSE_STATE_SEQ;
        return;

    case FASTMAVLINK_PARSE_STATE_SEQ:
    case FASTMAVLINK_PARSE_STATE_SYSID:
    case FASTMAVLINK_PARSE_STATE_COMPID:
    case FASTMAVLINK_PARSE_STATE_MSGID_1:
        buf[status->rx_cnt++] = c; // seq, sysid, compid, msgid1

        status->rx_state++;
        return;

    case FASTMAVLINK_PARSE_STATE_MSGID_2:
        buf[status->rx_cnt++] = c; // msgid2

        if (fmavx_status.flags & MAVLINKX_FLAGS_HAS_MSGID16) { // has no msgid3
            buf[status->rx_cnt++] = 0; // msgid3

            if (fmavx_status.flags & MAVLINKX_FLAGS_IS_TARGETED) { // has targets
                status->rx_state = FASTMAVLINK_PARSE_STATE_TARGET_SYSID;
                return;
            }
            status->rx_state = FASTMAVLINK_PARSE_STATE_CRC8;
            return;
        }

        status->rx_state = FASTMAVLINK_PARSE_STATE_MSGID_3;
        return;

    case FASTMAVLINK_PARSE_STATE_MSGID_3:
        buf[status->rx_cnt++] = c; // msgid3

        if (fmavx_status.flags & MAVLINKX_FLAGS_IS_TARGETED) { // has targets
            status->rx_state = FASTMAVLINK_PARSE_STATE_TARGET_SYSID;
            return;
        }
        status->rx_state = FASTMAVLINK_PARSE_STATE_CRC8;
        return;

    case FASTMAVLINK_PARSE_STATE_TARGET_SYSID:
        // we don't do anything with it currently
        fmavx_status.target_sysid = c;
        status->rx_state = FASTMAVLINK_PARSE_STATE_TARGET_COMPID;
        return;

    case FASTMAVLINK_PARSE_STATE_TARGET_COMPID:
        // we don't do anything with it currently
        fmavx_status.target_compid = c;
        status->rx_state = FASTMAVLINK_PARSE_STATE_CRC8;
        return;
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmavX_parse_to_frame_buf(fmav_result_t* result, uint8_t* buf, fmav_status_t* status, uint8_t c)
{
    if (status->rx_cnt >= MAVLINKX_FRAME_LEN_MAX) { // this should never happen, but play it safe
        status->rx_state = FASTMAVLINK_PARSE_STATE_IDLE;
    }

    switch (status->rx_state) {
    case FASTMAVLINK_PARSE_STATE_IDLE:
        status->rx_cnt = 0;
        if (c == MAVLINKX_MAGIC_1) {
            fmavX_status_reset(&fmavx_status);
            fmavx_status.header[fmavx_status.pos++] = MAVLINKX_MAGIC_1; // memorize
            status->rx_state = FASTMAVLINK_PARSE_STATE_MAGIC_2;
        }
        result->res = FASTMAVLINK_PARSE_RESULT_NONE;
        return FASTMAVLINK_PARSE_RESULT_NONE;

    case FASTMAVLINK_PARSE_STATE_MAGIC_2:
    case FASTMAVLINK_PARSE_STATE_FLAGS:
    case FASTMAVLINK_PARSE_STATE_LEN:
    case FASTMAVLINK_PARSE_STATE_SEQ:
    case FASTMAVLINK_PARSE_STATE_SYSID:
    case FASTMAVLINK_PARSE_STATE_COMPID:
    case FASTMAVLINK_PARSE_STATE_MSGID_1:
    case FASTMAVLINK_PARSE_STATE_MSGID_2:
    case FASTMAVLINK_PARSE_STATE_MSGID_3:
    case FASTMAVLINK_PARSE_STATE_TARGET_SYSID:
    case FASTMAVLINK_PARSE_STATE_TARGET_COMPID:
        _fmavX_parse_header_to_frame_buf(result, buf, status, c);
        result->res = FASTMAVLINK_PARSE_RESULT_NONE;
        return FASTMAVLINK_PARSE_RESULT_NONE;

    case FASTMAVLINK_PARSE_STATE_CRC8: {
        uint8_t crc8 = fmavX_crc8_calculate(fmavx_status.header, fmavx_status.pos);

        if (c != crc8) { // error, try to reset properly
            fmav_parse_reset(status);

            // search for another 'O'
            uint8_t next_magic_pos = 0;
            for (uint8_t n = 2; n < fmavx_status.pos; n++) { // sufficient to start with flags field
                if (fmavx_status.header[n] == MAVLINKX_MAGIC_1) { next_magic_pos = n; break; }
            }

            // there is another 'O' in the header, so backtrack
            if (next_magic_pos) {
                fmavx_status.header[fmavx_status.pos++] = c; // memorize also crc8

                uint8_t len = fmavx_status.pos;
                uint8_t head[16];
                memcpy(head, fmavx_status.header, len);

                fmavX_status_reset(&fmavx_status);
                fmavx_status.header[fmavx_status.pos++] = MAVLINKX_MAGIC_1; // memorize
                status->rx_state = FASTMAVLINK_PARSE_STATE_MAGIC_2;

                for (uint8_t n = next_magic_pos + 1; n < len; n++) {
                    _fmavX_parse_header_to_frame_buf(result, buf, status, head[n]);
                }
            }

            result->res = FASTMAVLINK_PARSE_RESULT_NONE;
            return FASTMAVLINK_PARSE_RESULT_NONE;
        }

        status->rx_state = FASTMAVLINK_FASTPARSE_STATE_FRAME;
        result->res = FASTMAVLINK_PARSE_RESULT_HAS_HEADER;
        return FASTMAVLINK_PARSE_RESULT_HAS_HEADER; }

    case FASTMAVLINK_FASTPARSE_STATE_FRAME:
        buf[status->rx_cnt++] = c; // payload, crc16, signature
        if (status->rx_cnt >= status->rx_frame_len) {
            status->rx_cnt = 0;
            status->rx_state = FASTMAVLINK_PARSE_STATE_IDLE;
            result->res = FASTMAVLINK_PARSE_RESULT_OK;
            result->frame_len = status->rx_frame_len;
            return FASTMAVLINK_PARSE_RESULT_OK;
        }
        result->res = FASTMAVLINK_PARSE_RESULT_HAS_HEADER;
        return FASTMAVLINK_PARSE_RESULT_HAS_HEADER;
    }

    // should never get to here
    fmav_parse_reset(status);
    result->res = FASTMAVLINK_PARSE_RESULT_NONE;
    return FASTMAVLINK_PARSE_RESULT_NONE;
}


// convenience wrapper
// returns 0, or 1
FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmavX_parse_and_check_to_frame_buf(fmav_result_t* result, uint8_t* buf, fmav_status_t* status, uint8_t c)
{
    uint8_t res;

    res = fmavX_parse_to_frame_buf(result, buf, status, c);
    // result can be NONE, HAS_HEADER, or OK
    if (res != FASTMAVLINK_PARSE_RESULT_OK) return 0;

    res = fmav_check_frame_buf(result, buf);
    // result can be MSGID_UNKNOWN, LENGTH_ERROR, CRC_ERROR, SIGNATURE_ERROR, or OK
    if (res == FASTMAVLINK_PARSE_RESULT_MSGID_UNKNOWN || res == FASTMAVLINK_PARSE_RESULT_OK) {
        return 1;
    }

    return 0;
}


#endif // FASTMAVLINK_EXTENSION_H
