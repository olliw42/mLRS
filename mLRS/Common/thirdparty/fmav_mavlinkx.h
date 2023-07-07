//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------
// MavlinkX
//------------------------------
#pragma once
#ifndef FASTMAVLINK_MAVLINKX_H
#define FASTMAVLINK_MAVLINKX_H

#include "../thirdparty/thirdparty.h"


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

// STX1                       STX
// STX2                       len
// flags                      incompat_flag
//   flagsext                 compat_flag
// len                        seq
// seq                        sysid
// sysid                      compid
//   sysid2                   msgid1
// compid                     msgid2
//   tsysid                   msgid3
//     tsysid2
//   tcompid
// msgid1
//   msgid2
//   msgid3
//   crcextra
// crc8

// TODO: do we really need two STX bytes?
// log analysis suggests that all bytes occur, but most have probabilities below 1%, but not less than ca 0.15%
// this means that a byte occurs once in about ca 500 bytes
// which is rare but not super rare
// since we backtrack the header this still might be ok

// TODO: should we backtrack if the crc16 doesn't match?

#define MAVLINKX_MAGIC_1              'o'
#define MAVLINKX_MAGIC_2              'w'
#define MAVLINKX_HEADER_LEN_MAX       16 // can be 9 to 16
#define MAVLINKX_FRAME_LEN_MAX        (MAVLINKX_HEADER_LEN_MAX+FASTMAVLINK_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


typedef enum {
    MAVLINKX_FLAGS_IS_V1              = 0x01,
    MAVLINKX_FLAGS_HAS_TARGETS        = 0x02,
    MAVLINKX_FLAGS_HAS_MSGID16        = 0x04,
    MAVLINKX_FLAGS_HAS_MSGID24        = 0x08,
    MAVLINKX_FLAGS_IS_COMPRESSED      = 0x40, // not used currently
    MAVLINKX_FLAGS_HAS_EXTENSION      = 0x80,
} fmavx_flags_e;


typedef enum {
    MAVLINKX_FLAGS_EXT_HAS_SIGNATURE  = 0x01,
    MAVLINKX_FLAGS_EXT_HAS_SYSID16    = 0x02, // not yet used, it's more to indicate the possibility
    MAVLINKX_FLAGS_EXT_NO_CRC_EXTRA   = 0x04, // always used currently, unknown messages are not yet supported
} fmavx_flags_ext_e;


typedef enum {
    FASTMAVLINK_PARSE_STATE_MAGIC_2 = 100,
    FASTMAVLINK_PARSE_STATE_FLAGS,
    FASTMAVLINK_PARSE_STATE_FLAGS_EXTENSION,
    FASTMAVLINK_PARSE_STATE_TARGET_SYSID,
    FASTMAVLINK_PARSE_STATE_TARGET_COMPID,
    FASTMAVLINK_PARSE_STATE_CRC_EXTRA,
    FASTMAVLINK_PARSE_STATE_CRC8,
} fmavx_parse_state_e;


typedef struct _fmavx_status {
    uint8_t flags;
    uint8_t flags_ext;
    uint8_t header[MAVLINKX_HEADER_LEN_MAX + 2];
    uint8_t pos;
    uint8_t target_sysid;
    uint8_t target_compid;
    uint8_t crc_extra;
} fmavx_status_t;


void fmavX_status_reset(fmavx_status_t* status)
{
    status->flags = 0;
    status->flags_ext = 0;
    status->pos = 0;
}


uint8_t fmavX_crc8_calculate(const uint8_t* buf, uint16_t len)
{
    // TODO: what would be the best crc8 for our purpose?
    return crc8_update(0, buf, len, 0xD5); // CRSF's crc8
}


fmavx_status_t fmavx_status = {};


// convert fmav msg structure into fmavX packet
// fmav_message_t has all info on the packet which we need, so it's quite straightforward to do

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmavX_msg_to_frame_buf(uint8_t* buf, fmav_message_t* msg)
{
    uint16_t pos = 0;
    uint8_t flags_ext = 0;

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
        buf[2] |= MAVLINKX_FLAGS_HAS_EXTENSION;
        flags_ext |= MAVLINKX_FLAGS_EXT_HAS_SIGNATURE;
    }
    if (msg->target_sysid > 0 || msg->target_compid > 0) {
        buf[2] |= MAVLINKX_FLAGS_HAS_TARGETS;
    }
    if (msg->msgid < 256) {
        // no flag needed
    } else
    if (msg->msgid < 65536) {
        buf[2] |= MAVLINKX_FLAGS_HAS_MSGID16;
    } else {
        buf[2] |= MAVLINKX_FLAGS_HAS_MSGID24;
    }

    // flags extension
    pos = 3;
    if (buf[2] & MAVLINKX_FLAGS_HAS_EXTENSION) {
        buf[pos++] = flags_ext;
    }

    // more
    buf[pos++] = msg->len;
    buf[pos++] = msg->seq;
    buf[pos++] = msg->sysid;
    buf[pos++] = msg->compid;

    // targets
    if (buf[2] & MAVLINKX_FLAGS_HAS_TARGETS) {
        buf[pos++] = msg->target_sysid;
        buf[pos++] = msg->target_compid;
    }

    // msgid
    buf[pos++] = (uint8_t)msg->msgid;
    if (msg->msgid >= 256) {
       buf[pos++] = (uint8_t)((msg->msgid) >> 8);
    }
    if (msg->msgid >= 65536) {
        buf[pos++] = (uint8_t)((msg->msgid) >> 16);
    }

    // extra crc
    if (!(flags_ext & MAVLINKX_FLAGS_EXT_NO_CRC_EXTRA)) {
        buf[pos++] = msg->crc_extra;
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
FASTMAVLINK_FUNCTION_DECORATOR void _fmavX_parse_header_to_frame_buf(uint8_t* buf, fmav_status_t* status, uint8_t c)
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
        fmavx_status.flags_ext = 0; // ensure it's zero, makes ifs below simpler as we don't have to check for HAS_EXTENSION

        uint8_t magic = (fmavx_status.flags & MAVLINKX_FLAGS_IS_V1) ? FASTMAVLINK_MAGIC_V1 : FASTMAVLINK_MAGIC_V2;
        buf[status->rx_cnt++] = magic; // 0: STX

        if (fmavx_status.flags & MAVLINKX_FLAGS_HAS_EXTENSION) {
            status->rx_state = FASTMAVLINK_PARSE_STATE_FLAGS_EXTENSION;
        } else {
            status->rx_state = FASTMAVLINK_PARSE_STATE_LEN;
        }
        }return;

    case FASTMAVLINK_PARSE_STATE_FLAGS_EXTENSION:
        fmavx_status.flags_ext = c;

        status->rx_state = FASTMAVLINK_PARSE_STATE_LEN;
        return;

    case FASTMAVLINK_PARSE_STATE_LEN:
        buf[status->rx_cnt++] = c; // 1: len

        if (fmavx_status.flags & MAVLINKX_FLAGS_IS_V1) {
            status->rx_header_len = FASTMAVLINK_HEADER_V1_LEN;
            status->rx_frame_len = (uint16_t)c + FASTMAVLINK_HEADER_V1_LEN + FASTMAVLINK_CHECKSUM_LEN;
        } else {
            status->rx_header_len = FASTMAVLINK_HEADER_V2_LEN;
            status->rx_frame_len = (uint16_t)c + FASTMAVLINK_HEADER_V2_LEN + FASTMAVLINK_CHECKSUM_LEN;

            uint8_t incompat_flags = (fmavx_status.flags_ext & MAVLINKX_FLAGS_EXT_HAS_SIGNATURE) ? FASTMAVLINK_INCOMPAT_FLAGS_SIGNED : 0;
            buf[status->rx_cnt++] = incompat_flags; // 2: incompat_flags
            buf[status->rx_cnt++] = 0; // 3: compat_flags

            if (incompat_flags & FASTMAVLINK_INCOMPAT_FLAGS_SIGNED) {
                status->rx_frame_len += FASTMAVLINK_SIGNATURE_LEN;
            }
        }

        status->rx_state = FASTMAVLINK_PARSE_STATE_SEQ;
        return;

    case FASTMAVLINK_PARSE_STATE_SEQ:
    case FASTMAVLINK_PARSE_STATE_SYSID:
      buf[status->rx_cnt++] = c; // 4: seq, 5: sysid

      status->rx_state++;
      return;

    case FASTMAVLINK_PARSE_STATE_COMPID:
      buf[status->rx_cnt++] = c; // 6: compid

      if (fmavx_status.flags & MAVLINKX_FLAGS_HAS_TARGETS) { // has targets
          status->rx_state = FASTMAVLINK_PARSE_STATE_TARGET_SYSID;
          return;
      }

      status->rx_state = FASTMAVLINK_PARSE_STATE_MSGID_1;
      return;

    case FASTMAVLINK_PARSE_STATE_TARGET_SYSID:
        // we don't do anything with it currently
        fmavx_status.target_sysid = c;
        status->rx_state = FASTMAVLINK_PARSE_STATE_TARGET_COMPID;
        return;

    case FASTMAVLINK_PARSE_STATE_TARGET_COMPID:
        // we don't do anything with it currently
        fmavx_status.target_compid = c;
        status->rx_state = FASTMAVLINK_PARSE_STATE_MSGID_1;
        return;

    case FASTMAVLINK_PARSE_STATE_MSGID_1:
        buf[status->rx_cnt++] = c; // 7: msgid1

        if ( !(fmavx_status.flags & MAVLINKX_FLAGS_HAS_MSGID16) &&
             !(fmavx_status.flags & MAVLINKX_FLAGS_HAS_MSGID24)    ) { // has no msgid2, msgid3

            buf[status->rx_cnt++] = 0; // 8: msgid2
            buf[status->rx_cnt++] = 0; // 9: msgid3

            if (fmavx_status.flags_ext & MAVLINKX_FLAGS_EXT_NO_CRC_EXTRA) { // has no crcextra
                status->rx_state = FASTMAVLINK_PARSE_STATE_CRC8;
                return;
            }

            status->rx_state = FASTMAVLINK_PARSE_STATE_CRC_EXTRA;
            return;
        }

        status->rx_state = FASTMAVLINK_PARSE_STATE_MSGID_2;
        return;

    case FASTMAVLINK_PARSE_STATE_MSGID_2:
        buf[status->rx_cnt++] = c; // 8: msgid2

        if (!(fmavx_status.flags & MAVLINKX_FLAGS_HAS_MSGID24)) { // has no msgid3

            buf[status->rx_cnt++] = 0; // 9: msgid3

            if (fmavx_status.flags_ext & MAVLINKX_FLAGS_EXT_NO_CRC_EXTRA) { // has no crcextra
                status->rx_state = FASTMAVLINK_PARSE_STATE_CRC8;
                return;
            }

            status->rx_state = FASTMAVLINK_PARSE_STATE_CRC_EXTRA;
            return;
        }

        status->rx_state = FASTMAVLINK_PARSE_STATE_MSGID_3;
        return;

    case FASTMAVLINK_PARSE_STATE_MSGID_3:
        buf[status->rx_cnt++] = c; // 9: msgid3

        if (fmavx_status.flags_ext & MAVLINKX_FLAGS_EXT_NO_CRC_EXTRA) { // has no crcextra
            status->rx_state = FASTMAVLINK_PARSE_STATE_CRC8;
            return;
        }

        status->rx_state = FASTMAVLINK_PARSE_STATE_CRC_EXTRA;
        return;

    case FASTMAVLINK_PARSE_STATE_CRC_EXTRA:
        // we don't do anything with it currently
        fmavx_status.crc_extra = c;
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
    case FASTMAVLINK_PARSE_STATE_FLAGS_EXTENSION:
    case FASTMAVLINK_PARSE_STATE_LEN:
    case FASTMAVLINK_PARSE_STATE_SEQ:
    case FASTMAVLINK_PARSE_STATE_SYSID:
    case FASTMAVLINK_PARSE_STATE_COMPID:
    case FASTMAVLINK_PARSE_STATE_TARGET_SYSID:
    case FASTMAVLINK_PARSE_STATE_TARGET_COMPID:
    case FASTMAVLINK_PARSE_STATE_MSGID_1:
    case FASTMAVLINK_PARSE_STATE_MSGID_2:
    case FASTMAVLINK_PARSE_STATE_MSGID_3:
    case FASTMAVLINK_PARSE_STATE_CRC_EXTRA:
        _fmavX_parse_header_to_frame_buf(buf, status, c);
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
                    _fmavX_parse_header_to_frame_buf(buf, status, head[n]);
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


#endif // FASTMAVLINK_MAVLINKX_H
