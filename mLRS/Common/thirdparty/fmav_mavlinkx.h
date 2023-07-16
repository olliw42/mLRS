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
// 4th attempt
// we parse the stream and simply substitute the header

// the original fmav construct for sending looks like:
//
//    if (fmav_parse_and_check_to_frame_buf(&result_link_in, buf_link_in, &status_link_in, c)) {
//        fmav_frame_buf_to_msg(&msg_serial_out, &result_link_in, buf_link_in);
//        uint16_t len = fmav_msg_to_frame_buf(_buf, &msg_serial_out);
//        serial.putbuf(_buf, len);
//    }
//
// this reads the complete frame, so adds a bit latency, but for the moment we do it to keep it simple
// should be ok for testing and dev-ing, and also good enough for application

// header structure (indent indicates field depends on flags)
//
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
// we instead reset parser on frame lost, which in our case equally resolves the issue

#define MAVLINKX_MAGIC_1              'o'
#define MAVLINKX_MAGIC_2              'w'
#define MAVLINKX_HEADER_LEN_MAX       17 // can be 9 to 17
#define MAVLINKX_FRAME_LEN_MAX        (MAVLINKX_HEADER_LEN_MAX+FASTMAVLINK_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


typedef enum {
    MAVLINKX_FLAGS_IS_V1              = 0x01,
    MAVLINKX_FLAGS_HAS_TARGETS        = 0x02,
    MAVLINKX_FLAGS_HAS_MSGID16        = 0x04,
    MAVLINKX_FLAGS_HAS_MSGID24        = 0x08,
    MAVLINKX_FLAGS_IS_COMPRESSED      = 0x40,
    MAVLINKX_FLAGS_HAS_EXTENSION      = 0x80,
} fmavx_flags_e;


typedef enum {
    MAVLINKX_FLAGS_EXT_HAS_SIGNATURE  = 0x01,
    MAVLINKX_FLAGS_EXT_HAS_SYSID16    = 0x02, // not yet used, it's more to indicate the possibility
    MAVLINKX_FLAGS_EXT_NO_CRC_EXTRA   = 0x04, // always used currently, unknown messages are not supported anyways
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
    uint8_t pos_of_len;
    uint8_t target_sysid;
    uint8_t target_compid;
    uint8_t crc_extra;
    uint16_t rx_payload_len;
} fmavx_status_t;


void fmavX_status_reset(fmavx_status_t* xstatus)
{
    xstatus->flags = 0;
    xstatus->flags_ext = 0;
    xstatus->pos = 0;
}


uint8_t fmavX_crc8_calculate(const uint8_t* buf, uint16_t len)
{
    // TODO: what would be the best crc8 for our purpose?
    return crc8_update(0, buf, len, 0xD5); // CRSF's crc8
}


// TODO: shouldn't be global
fmavx_status_t fmavx_status = {};


uint8_t _fmavX_payload_compress(uint8_t* payload_out, uint8_t* len_out, uint8_t* payload, uint8_t len);
void _fmavX_payload_decompress(uint8_t* payload_out, uint8_t* len_out, uint8_t len);


//-------------------------------------------------------
// Converter
//-------------------------------------------------------
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
    uint8_t pos_of_len = pos;
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
    // we postpone crc8 calculation to after payload compression, since flags may change
    //uint8_t crc8 = fmavX_crc8_calculate(buf, pos);
    //buf[pos++] = crc8;
    pos++;

    // payload
    // we should want to remove the targets if there are any, for the moment we just don't
    // do compression, but do not advance pos since we need to do crc8
    uint8_t len;
    if (_fmavX_payload_compress(&(buf[pos]), &len, msg->payload, msg->len)) {
        buf[2] |= MAVLINKX_FLAGS_IS_COMPRESSED;
        buf[pos_of_len] = len;
    } else {
        memcpy(&(buf[pos]), msg->payload, msg->len);
        len = msg->len;
    }

    // now we can do crc8
    uint8_t crc8 = fmavX_crc8_calculate(buf, pos-1);
    buf[pos-1] = crc8;

    pos += len;

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


//-------------------------------------------------------
// Parser
//-------------------------------------------------------
// parse the stream into regular fmav frame_buf
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
        fmavx_status.pos_of_len = status->rx_cnt;
        buf[status->rx_cnt++] = c; // 1: len
        fmavx_status.rx_payload_len = c;

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

        status->rx_state = FASTMAVLINK_PARSE_STATE_PAYLOAD;
        result->res = FASTMAVLINK_PARSE_RESULT_HAS_HEADER;
        return FASTMAVLINK_PARSE_RESULT_HAS_HEADER; }

    case FASTMAVLINK_PARSE_STATE_PAYLOAD:
        buf[status->rx_cnt++] = c; // payload
        if (status->rx_cnt >= status->rx_header_len + (uint16_t)fmavx_status.rx_payload_len) {

            if (fmavx_status.flags & MAVLINKX_FLAGS_IS_COMPRESSED) {
                uint8_t len;
                _fmavX_payload_decompress(&(buf[status->rx_header_len]), &len, fmavx_status.rx_payload_len);
                uint8_t delta_len = (len - fmavx_status.rx_payload_len);
                status->rx_cnt += delta_len;
                fmavx_status.rx_payload_len += delta_len;
                status->rx_frame_len += delta_len;
                buf[fmavx_status.pos_of_len] = len;
            }

            status->rx_state = FASTMAVLINK_FASTPARSE_STATE_FRAME;
        }
        return FASTMAVLINK_PARSE_RESULT_HAS_HEADER;

    case FASTMAVLINK_FASTPARSE_STATE_FRAME:
        buf[status->rx_cnt++] = c; // crc16, signature
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


//-------------------------------------------------------
// Compression
//-------------------------------------------------------
/*
Compression scheme

The concept developed here is based on an observation of how the bytes in the payload of mavlink messages
are distributed. If the actual distribution is very different, the scheme becomes less effective, obviously.
The observations are:
- ca. 20% - 25% of the bytes in the payload are 0 (after trailing zero removal in v2)
- ca. 3% - 4% of the bytes in the payload are 255
- all other bytes are present "nearly" similarly with low probability, ranging from ca. 1% - 0.15%
- the bytes 1 - 16 are present with about 10% probability
This distribution means that beyond dealing with the many 0 and 255 there is actually not so much to compress,
since the entropy is already relatively high.

As a rough estimate for what can be achieved let's assume that all 0 and 255 can be compressed away, which
gives a payload compression of ca 25%. Taking into account the header, we can't get more than ca 15% overall
compression.

Plenty of schemes were tried, this was (by far) the "best" one:

00              -> 0
010             -> 255
0110 + 8 bits   -> RLE for 0: the code is followed by 8 bits to encode 1-255 0 bytes
0111 + 8 bits   -> RLE for 255: the code is followed by 8 bits to encode 1-255 255 bytes
1    + 8 bits   -> codes for remaining bytes not covered by the other branches

For encoding the tree we follow the method of gzip, see:
https://commandlinefanatic.com/cgi-bin/showarticle.cgi?article=art007
*/


#define FMAX_DBG(x)

//-- compression
// we can use payload_out buffer as working buffer

static uint8_t out_bit;


// len_out & pos_out index the next bit position to write to
void _fmavX_encode_add(uint8_t* payload_out, uint8_t* len_out, uint16_t code, uint8_t code_len)
{
    // 1 -> 0x0001, 2 -> 0x0002, ..., 8 -> 0x0080
    uint16_t cur_code_bit = (uint16_t)1 << (code_len - 1);

    for (uint8_t i = 0; i < code_len; i++) {
        if (!(code & cur_code_bit)) { // clear the bit
            payload_out[*len_out] &=~ out_bit;
        }
        cur_code_bit >>= 1; // next bit from code
        out_bit >>= 1; // next bit pos in out bytes
        if (out_bit == 0) { // out byte completely done, go to next out byte
            (*len_out)++;
            out_bit = 0x80;
            payload_out[*len_out] = 0xFF;
        }
    }
}


uint8_t _fmavX_payload_compress(uint8_t* payload_out, uint8_t* len_out, uint8_t* payload, uint8_t len)
{
    //if (len > 20) return 0;


FMAX_DBG(dbg.puts("\n\ncompress");
dbg.puts("\n payload ");
for (uint16_t i = 0; i < len; i++) {
    dbg.puts("x");dbg.puts(u8toHEX_s(payload[i]));
})

    uint16_t code; // can be 2 bits - 12 bits

    *len_out = 0;
    out_bit = 0x80;
    payload_out[0] = 0xFF; // we use 1's as stop marker in last byte, so it's easier to fill with 0xFF

    for (uint8_t n = 0; n < len; n++) {
        uint8_t c = payload[n];

        // handle c
            if (c == 0) {
                code = (uint16_t)0b00; // 2 bit code = 2 bits length
                _fmavX_encode_add(payload_out, len_out, code, 2);
            } else
            if (c == 255) {
                code = (uint16_t)0b010; // 3 bit code = 3 bits length
                _fmavX_encode_add(payload_out, len_out, code, 3);
            } else {
                code = (uint16_t)0b1 << 8; // 1 bit code + room for 8 bits = 9 bits length
                code += c;
                _fmavX_encode_add(payload_out, len_out, code, 9);
            }

        // compression doesn't reduce payload len
        // check it inside loop to prevent to write into memory outside of payload buffer
        if (*len_out >= len) return 0;
    }

    if (out_bit != 0) { // handle last out byte if not completed
        (*len_out)++; // count it
    }

    // compression didn't reduce payload len
    if (*len_out >= len) return 0;

FMAX_DBG(dbg.puts("\n pay out ");
for (uint16_t i = 0; i < *len_out; i++) {
    dbg.puts("x");dbg.puts(u8toHEX_s(payload_out[i]));
})


    return 1;
}


//-- decompression

typedef enum {
    MAVLINKX_CODE_0 = 0,
    MAVLINKX_CODE_255,
    MAVLINKX_CODE_0_RLE,
    MAVLINKX_CODE_255_RLE,
    MAVLINKX_CODE_ELSE
} fmavx_code_e;


typedef struct {
    uint8_t len; // bit length of the code
    uint8_t id; // number to tell what it represents
} fmavx_code_entry_t;


#define MAVLINKX_CODE_LIST_LEN  16 // 4 bits maximal code length

const fmavx_code_entry_t fmavx_code_list[MAVLINKX_CODE_LIST_LEN] = {
    { .len = 2, .id = MAVLINKX_CODE_0       }, // 00|00   -> 0
    { .len = 2, .id = MAVLINKX_CODE_0       }, // 00|01
    { .len = 2, .id = MAVLINKX_CODE_0       }, // 00|10
    { .len = 2, .id = MAVLINKX_CODE_0       }, // 00|11
    { .len = 3, .id = MAVLINKX_CODE_255     }, // 010|0   -> 255
    { .len = 3, .id = MAVLINKX_CODE_255     }, // 010|1
    { .len = 4, .id = MAVLINKX_CODE_0_RLE   }, // 0110    -> 0 RLE
    { .len = 4, .id = MAVLINKX_CODE_255_RLE }, // 0111    -> 255 RLE
    { .len = 1, .id = MAVLINKX_CODE_ELSE    }, // 1|000   -> rest
    { .len = 1, .id = MAVLINKX_CODE_ELSE    }, // 1|001
    { .len = 1, .id = MAVLINKX_CODE_ELSE    }, // 1|010
    { .len = 1, .id = MAVLINKX_CODE_ELSE    }, // 1|011
    { .len = 1, .id = MAVLINKX_CODE_ELSE    }, // 1|100
    { .len = 1, .id = MAVLINKX_CODE_ELSE    }, // 1|101
    { .len = 1, .id = MAVLINKX_CODE_ELSE    }, // 1|110
    { .len = 1, .id = MAVLINKX_CODE_ELSE    }, // 1|111
};


static uint8_t in_pos;
static uint8_t in_bit;
uint8_t in_buf[280];


uint8_t _fmavX_decode_next_bits(uint8_t* code, uint8_t len, uint8_t bits_len)
{
    *code = 0;

    for (uint8_t i = 0; i < bits_len; i++) {

        if (in_pos >= len) return 0;

        *code <<= 1;
        if (in_buf[in_pos] & in_bit) { // set the bit
            *code |= 0x01;
        }

        in_bit >>= 1; // next bit in in byte
        if (in_bit == 0) { // in byte completely done, go to next in byte
            in_pos++;
            in_bit = 0x80;
        }
    }

    return 1;
}


void _fmavX_decode_back_bits(uint8_t bits_len)
{
    for (uint8_t i = 0; i < bits_len; i++) {
        in_bit <<= 1;
        if (in_bit == 0) {
            in_pos--;
            in_bit = 0x01;
        }
    }
}



void _fmavX_payload_decompress(uint8_t* payload_out, uint8_t* len_out, uint8_t len)
{
//    *len_out = len;
//    return;

FMAX_DBG(dbg.puts("\ndecompress");
dbg.puts("\n pay in  ");
for (uint16_t i = 0; i < len; i++) {
    dbg.puts("x");dbg.puts(u8toHEX_s(payload_out[i]));
})

    memcpy(in_buf, payload_out, len);

    *len_out = 0;

    in_pos = 0;
    in_bit = 0x80;

    while (in_pos < len) {
        uint8_t c;

        // get next 4 bits
        if (!_fmavX_decode_next_bits(&c, len, 4)) break;

FMAX_DBG(dbg.puts("\n   c x");dbg.puts(u8toHEX_s(c));)

        fmavx_code_entry_t code = fmavx_code_list[c];

        // put back unused bits
        _fmavX_decode_back_bits(4 - code.len);

        switch (code.id) {
            case MAVLINKX_CODE_0:
FMAX_DBG(dbg.puts(" -> code_0");)
                c = 0;
                payload_out[(*len_out)++] = c;
                break;
            case MAVLINKX_CODE_255:
FMAX_DBG(dbg.puts(" -> code_255");)
                c = 0xFF;
                payload_out[(*len_out)++] = c;
                break;
            case MAVLINKX_CODE_ELSE:
FMAX_DBG(dbg.puts(" -> code_else");)
                if (!_fmavX_decode_next_bits(&c, len, 8)) break;
FMAX_DBG(dbg.puts(" -> c x");dbg.puts(u8toHEX_s(c));)
                payload_out[(*len_out)++] = c;
                break;
        }
    }


FMAX_DBG(dbg.puts("\n payload ");
for (uint16_t i = 0; i < *len_out; i++) {
    dbg.puts("x");dbg.puts(u8toHEX_s(payload_out[i]));
})

}


#endif // FASTMAVLINK_MAVLINKX_H
