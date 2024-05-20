//------------------------------
// The MavlinkX library
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// All Rights Reserved.
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies, and
// that the use of this software in a product is acknowledged in the product
// documentation or a location viewable to users.
//------------------------------
// MavlinkX
// builds on top of fastMavlink library, https://github.com/olliw42/fastmavlink
//------------------------------
#pragma once
#ifndef MAVLINKX_H
#define MAVLINKX_H


#define MAVLINKX_CRC8_LOOKUP_TABLE
#define MAVLINKX_COMPRESSION // compression with O3 costs ca 8 kB flash
#define MAVLINKX_O3


//-------------------------------------------------------
// MavlinkX
//-------------------------------------------------------
// we parse the stream and simply substitute the header

// the original fmav construct for sending looks like:
//
//    if (fmav_parse_and_check_to_frame_buf(&result_link_in, buf_link_in, &status_link_in, c)) {
//        fmav_frame_buf_to_msg(&msg_serial_out, &result_link_in, buf_link_in);
//        uint16_t len = fmav_msg_to_frame_buf(_buf, &msg_serial_out);
//        serial.putbuf(_buf, len);
//    }
//
// this reads the complete frame, so adds a bit latency
// should be good enough for applications

// header structure (indentation indicates that field depends on flags)
// the header is designed to be extensible, and yet be backwards and forward compatible
//
// STX1                       STX                 STXV1
// STX2                       len                 len
// flags                      incompat_flag       seq
//   flagsext                 compat_flag         sysid
// len                        seq                 compid
// seq                        sysid               msgid
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
// we instead reset parser on frame lost, equally resolves the issue


#ifdef MAVLINKX_O3
  #ifdef __GNUC__
    #pragma GCC push_options
    #pragma GCC optimize ("O3")
  #endif
#endif


#define MAVLINKX_MAGIC_1              0x6F // 'o'
#define MAVLINKX_MAGIC_2              0x77 // 'w'
#define MAVLINKX_HEADER_LEN_MAX       17 // can be 9 to 17
#define MAVLINKX_FRAME_LEN_MAX        287 // =  HEADER_LEN_MAX + PAYLOAD_LEN_MAX + CHECKSUM_LEN + SIGNATURE_LEN


typedef enum {
    MAVLINKX_FLAGS_IS_V1              = 0x01,
    MAVLINKX_FLAGS_HAS_MSGID16        = 0x02,
    MAVLINKX_FLAGS_HAS_TARGETS        = 0x08,
    MAVLINKX_FLAGS_HAS_CRC_EXTRA      = 0x10, // not yet used, indicates the possibility
    MAVLINKX_FLAGS_IS_COMPRESSED      = 0x40,
    MAVLINKX_FLAGS_HAS_EXTENSION      = 0x80,
} fmavx_flags_e;


typedef enum {
    MAVLINKX_FLAGS_EXT_HAS_SIGNATURE  = 0x01,
    MAVLINKX_FLAGS_EXT_HAS_MSGID24    = 0x02,
    MAVLINKX_FLAGS_EXT_HAS_SYSID16    = 0x04, // not yet used, indicates the possibility
} fmavx_flags_ext_e;


typedef enum { // extend fastMavlink's FASTMAVLINK_PARSE_STATE enum
    FASTMAVLINK_PARSE_STATE_MAGIC_2 = 100,
    FASTMAVLINK_PARSE_STATE_FLAGS,
    FASTMAVLINK_PARSE_STATE_FLAGS_EXTENSION,
    FASTMAVLINK_PARSE_STATE_TARGET_SYSID,
    FASTMAVLINK_PARSE_STATE_TARGET_COMPID,
    FASTMAVLINK_PARSE_STATE_CRC_EXTRA,
    FASTMAVLINK_PARSE_STATE_CRC8,
} fmavx_parse_state_e;


typedef struct
{
    uint8_t flags;
    uint8_t flags_ext;
    uint8_t header[MAVLINKX_HEADER_LEN_MAX + 2];
    uint8_t pos;
    uint8_t pos_of_len;
    uint8_t target_sysid;
    uint8_t target_compid;
    uint8_t crc_extra;
    uint8_t rx_payload_len;

    // for compression
    uint8_t out_bit;
    uint8_t in_pos;
    uint8_t in_bit;
} fmavx_status_t;


void fmavX_status_reset(fmavx_status_t* xstatus)
{
    xstatus->flags = 0;
    xstatus->flags_ext = 0;
    xstatus->pos = 0;
}


typedef struct
{
    uint8_t compression_enabled;
} fmavx_config_t;
static fmavx_config_t fmavx_config_g = {}; // we use a global here


void fmavX_config_compression(uint8_t compression_flag)
{
    fmavx_config_g.compression_enabled = (compression_flag) ? 1 : 0;
}


// TODO: shouldn't be global
fmavx_status_t fmavx_status = {};


#ifdef MAVLINKX_COMPRESSION
uint8_t _fmavX_payload_compress(uint8_t* payload_out, uint8_t* len_out, uint8_t* payload, uint8_t len);
void _fmavX_payload_decompress(uint8_t* payload_out, uint8_t* len_out, uint8_t len);
#endif


//-------------------------------------------------------
// Init
//-------------------------------------------------------

// call it once before using the library
FASTMAVLINK_FUNCTION_DECORATOR void fmavX_init(void)
{
    memset(&fmavx_status, 0, sizeof(fmavx_status));

    fmavx_config_g.compression_enabled = 0; // disable it per default
}


//-------------------------------------------------------
// Crc8
//-------------------------------------------------------
// catalogue of quite many crcs
// - https://reveng.sourceforge.io/crc-catalogue/1-15.htm
// standard reference for "best" crcs
// - https://users.ece.cmu.edu/~koopman/crc/index.html
// good source, explains also Koopman representation well (ch. 7.2):
// - http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
//
// CRSF's crc8: 0xD5 = 0x1D5 = 0xEA in Koopman notation, used in CRC-8/DVB-S2
//
// based on Koopman, we choose 0x83 (in Koopman notation) = 0x107 = 0x07
// note: 0x07 appears to be quite popular, used in CRC-8/ROHC, CRC-8/SMBUS, CRC-8/I-432-1
// note: 0x07 is amenable for fast crc calculation, see
// https://www.dsp-weimich.com/programming/cyclic-redundancy-check-crc-bitwise-lookup-table-fast-crc-without-lookup-table-reversing-crc-c-implementation-using-the-octave-gnu-tool/
// fmavx_crc8_accumulate() is derived following this reference.
//
// brute force algorithm:
// *crc ^= data;
// for (uint8_t i = 0; i < 8; i++) {
//     *crc = (*crc & 0x80) ? (*crc << 1) ^ 0x07 : (*crc << 1);
// }
//
// with init 0x00 it corresponds to CRC-8 (see https://crccalc.com/), and 123456789 -> 0xF4
//
// since the first byte is always the STX and thus not zero, we could init the crc with 0. But
// we prefer to follow common practice and init with 0xFF.
//
// check "123456789" -> 0xFB
//
// interestingly, using the lookup table requires not more flash since the code becomes that much smaller!
// => use it always


#define MAVLINKX_CRC8_INIT  0xFF
#define MAVLINKX_CRC8_POLY  0x07


#ifdef MAVLINKX_CRC8_LOOKUP_TABLE

#ifdef __GNUC__
  #pragma GCC push_options
  #pragma GCC optimize ("O3")
#endif

// generated from https://crccalc.com/, for CRC-8
const uint8_t fmavx_crc8_table[256] = {
    0x00 , 0x07 , 0x0e , 0x09 , 0x1c , 0x1b , 0x12 , 0x15,
    0x38 , 0x3f , 0x36 , 0x31 , 0x24 , 0x23 , 0x2a , 0x2d,
    0x70 , 0x77 , 0x7e , 0x79 , 0x6c , 0x6b , 0x62 , 0x65,
    0x48 , 0x4f , 0x46 , 0x41 , 0x54 , 0x53 , 0x5a , 0x5d,
    0xe0 , 0xe7 , 0xee , 0xe9 , 0xfc , 0xfb , 0xf2 , 0xf5,
    0xd8 , 0xdf , 0xd6 , 0xd1 , 0xc4 , 0xc3 , 0xca , 0xcd,
    0x90 , 0x97 , 0x9e , 0x99 , 0x8c , 0x8b , 0x82 , 0x85,
    0xa8 , 0xaf , 0xa6 , 0xa1 , 0xb4 , 0xb3 , 0xba , 0xbd,
    0xc7 , 0xc0 , 0xc9 , 0xce , 0xdb , 0xdc , 0xd5 , 0xd2,
    0xff , 0xf8 , 0xf1 , 0xf6 , 0xe3 , 0xe4 , 0xed , 0xea,
    0xb7 , 0xb0 , 0xb9 , 0xbe , 0xab , 0xac , 0xa5 , 0xa2,
    0x8f , 0x88 , 0x81 , 0x86 , 0x93 , 0x94 , 0x9d , 0x9a,
    0x27 , 0x20 , 0x29 , 0x2e , 0x3b , 0x3c , 0x35 , 0x32,
    0x1f , 0x18 , 0x11 , 0x16 , 0x03 , 0x04 , 0x0d , 0x0a,
    0x57 , 0x50 , 0x59 , 0x5e , 0x4b , 0x4c , 0x45 , 0x42,
    0x6f , 0x68 , 0x61 , 0x66 , 0x73 , 0x74 , 0x7d , 0x7a,
    0x89 , 0x8e , 0x87 , 0x80 , 0x95 , 0x92 , 0x9b , 0x9c,
    0xb1 , 0xb6 , 0xbf , 0xb8 , 0xad , 0xaa , 0xa3 , 0xa4,
    0xf9 , 0xfe , 0xf7 , 0xf0 , 0xe5 , 0xe2 , 0xeb , 0xec,
    0xc1 , 0xc6 , 0xcf , 0xc8 , 0xdd , 0xda , 0xd3 , 0xd4,
    0x69 , 0x6e , 0x67 , 0x60 , 0x75 , 0x72 , 0x7b , 0x7c,
    0x51 , 0x56 , 0x5f , 0x58 , 0x4d , 0x4a , 0x43 , 0x44,
    0x19 , 0x1e , 0x17 , 0x10 , 0x05 , 0x02 , 0x0b , 0x0c,
    0x21 , 0x26 , 0x2f , 0x28 , 0x3d , 0x3a , 0x33 , 0x34,
    0x4e , 0x49 , 0x40 , 0x47 , 0x52 , 0x55 , 0x5c , 0x5b,
    0x76 , 0x71 , 0x78 , 0x7f , 0x6a , 0x6d , 0x64 , 0x63,
    0x3e , 0x39 , 0x30 , 0x37 , 0x22 , 0x25 , 0x2c , 0x2b,
    0x06 , 0x01 , 0x08 , 0x0f , 0x1a , 0x1d , 0x14 , 0x13,
    0xae , 0xa9 , 0xa0 , 0xa7 , 0xb2 , 0xb5 , 0xbc , 0xbb,
    0x96 , 0x91 , 0x98 , 0x9f , 0x8a , 0x8d , 0x84 , 0x83,
    0xde , 0xd9 , 0xd0 , 0xd7 , 0xc2 , 0xc5 , 0xcc , 0xcb,
    0xe6 , 0xe1 , 0xe8 , 0xef , 0xfa , 0xfd , 0xf4 , 0xf3
};


FASTMAVLINK_FUNCTION_DECORATOR void fmavX_crc8_accumulate(uint8_t* crc, uint8_t data)
{
    *crc = fmavx_crc8_table[*crc ^ data];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmavX_crc8_calculate(const uint8_t* buf, uint16_t len)
{
    uint8_t crc = MAVLINKX_CRC8_INIT;
    while (len--) {
        crc = fmavx_crc8_table[crc ^ *buf++];
    }
    return crc;
}

#ifdef __GNUC__
  #pragma GCC pop_options
#endif

#else

FASTMAVLINK_FUNCTION_DECORATOR void fmavX_crc8_accumulate(uint8_t* crc, uint8_t data)
{
uint8_t tmp1, tmp2;

    *crc ^= data;
    tmp1 = *crc << 1;
    if (*crc & 0x80) tmp1 ^= MAVLINKX_CRC8_POLY;
    tmp2 = tmp1 << 1;
    if (tmp1 & 0x80) tmp2 ^= MAVLINKX_CRC8_POLY;
    *crc = tmp2 ^ tmp1 ^ *crc;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmavX_crc8_calculate(const uint8_t* buf, uint16_t len)
{
    uint8_t crc = MAVLINKX_CRC8_INIT;
    while (len--) {
        fmavX_crc8_accumulate(&crc, *buf++);
    }
    return crc;
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR void fmavX_crc8_init(uint8_t* crc)
{
    *crc = MAVLINKX_CRC8_INIT;
}


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
    // can be > 0 only if we know the message, so this implicitly does it only if we know the message
    if (msg->target_sysid > 0 || msg->target_compid > 0) {
        buf[2] |= MAVLINKX_FLAGS_HAS_TARGETS;
    }
    if (msg->msgid < 256) {
        // no flag needed
    } else
    if (msg->msgid < 65536) {
        buf[2] |= MAVLINKX_FLAGS_HAS_MSGID16;
    } else {
        buf[2] |= MAVLINKX_FLAGS_HAS_EXTENSION;
        flags_ext |= MAVLINKX_FLAGS_EXT_HAS_MSGID24;
    }

    // we should do it based on whether we know the message, but we don't currently have that info easily available in msg
    // if (known) buf[2] |= MAVLINKX_FLAGS_HAS_CRC_EXTRA;

    pos = 3;

    // flags extension
    if (buf[2] & MAVLINKX_FLAGS_HAS_EXTENSION) {
        buf[pos++] = flags_ext;
    }

    // more
#ifdef MAVLINKX_COMPRESSION
    uint8_t pos_of_len = pos;
#endif
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
    if (buf[2] & MAVLINKX_FLAGS_HAS_MSGID16) {
        buf[pos++] = (uint8_t)((msg->msgid) >> 8);
    }
    if (flags_ext & MAVLINKX_FLAGS_EXT_HAS_MSGID24) {
        buf[pos++] = (uint8_t)((msg->msgid) >> 16);
    }

    // extra crc
    if (buf[2] & MAVLINKX_FLAGS_HAS_CRC_EXTRA) {
        buf[pos++] = msg->crc_extra;
    }

#ifdef MAVLINKX_COMPRESSION
    // crc8
    // we postpone crc8 calculation to after payload compression, since flags may change
    //uint8_t crc8 = fmavX_crc8_calculate(buf, pos);
    //buf[pos++] = crc8;

    // payload
    // we should want to remove the targets if there are any, for the moment we just don't
    // do compression, but do not advance pos since we need to do crc8
    uint8_t len;
    if (fmavx_config_g.compression_enabled &&
        _fmavX_payload_compress(&(buf[pos + 1]), &len, msg->payload, msg->len)) {
        buf[2] |= MAVLINKX_FLAGS_IS_COMPRESSED;
        buf[pos_of_len] = len;
    } else {
        memcpy(&(buf[pos + 1]), msg->payload, msg->len);
        len = msg->len;
    }

    // now we can do crc8
    uint8_t crc8 = fmavX_crc8_calculate(buf, pos);
    buf[pos++] = crc8;

    pos += len;
#else
    // crc8
    uint8_t crc8 = fmavX_crc8_calculate(buf, pos);
    buf[pos++] = crc8;

    // payload
    memcpy(&(buf[pos]), msg->payload, msg->len);
    pos += msg->len;
#endif

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
// parse the fmavX stream into regular fmav frame_buf
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

    case FASTMAVLINK_PARSE_STATE_FLAGS: {
        fmavx_status.flags = c;
        fmavx_status.flags_ext = 0; // ensure it's zero, makes ifs below simpler as we don't have to check for HAS_EXTENSION

        uint8_t magic = (fmavx_status.flags & MAVLINKX_FLAGS_IS_V1) ? FASTMAVLINK_MAGIC_V1 : FASTMAVLINK_MAGIC_V2;
        buf[status->rx_cnt++] = magic; // 0: STX

        if (fmavx_status.flags & MAVLINKX_FLAGS_HAS_EXTENSION) {
            status->rx_state = FASTMAVLINK_PARSE_STATE_FLAGS_EXTENSION;
            return;
        }

        status->rx_state = FASTMAVLINK_PARSE_STATE_LEN;
        return; }

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
            status->rx_frame_len = (uint16_t)c + (FASTMAVLINK_HEADER_V1_LEN + FASTMAVLINK_CHECKSUM_LEN);
        } else {
            status->rx_header_len = FASTMAVLINK_HEADER_V2_LEN;
            status->rx_frame_len = (uint16_t)c + (FASTMAVLINK_HEADER_V2_LEN + FASTMAVLINK_CHECKSUM_LEN);

            uint8_t incompat_flags = 0;
            if (fmavx_status.flags_ext & MAVLINKX_FLAGS_EXT_HAS_SIGNATURE) incompat_flags |= FASTMAVLINK_INCOMPAT_FLAGS_SIGNED;
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
             !(fmavx_status.flags_ext & MAVLINKX_FLAGS_EXT_HAS_MSGID24) ) { // has no msgid2, msgid3

            buf[status->rx_cnt++] = 0; // 8: msgid2
            buf[status->rx_cnt++] = 0; // 9: msgid3

            if (fmavx_status.flags & MAVLINKX_FLAGS_HAS_CRC_EXTRA) { // has crcextra
                status->rx_state = FASTMAVLINK_PARSE_STATE_CRC_EXTRA;
                return;
            }

            status->rx_state = FASTMAVLINK_PARSE_STATE_CRC8;
            return;
        }

        status->rx_state = FASTMAVLINK_PARSE_STATE_MSGID_2;
        return;

    case FASTMAVLINK_PARSE_STATE_MSGID_2:
        buf[status->rx_cnt++] = c; // 8: msgid2

        if (!(fmavx_status.flags_ext & MAVLINKX_FLAGS_EXT_HAS_MSGID24)) { // has no msgid3

            buf[status->rx_cnt++] = 0; // 9: msgid3

            if (fmavx_status.flags & MAVLINKX_FLAGS_HAS_CRC_EXTRA) { // has crcextra
                status->rx_state = FASTMAVLINK_PARSE_STATE_CRC_EXTRA;
                return;
            }

            status->rx_state = FASTMAVLINK_PARSE_STATE_CRC8;
            return;
        }

        status->rx_state = FASTMAVLINK_PARSE_STATE_MSGID_3;
        return;

    case FASTMAVLINK_PARSE_STATE_MSGID_3:
        buf[status->rx_cnt++] = c; // 9: msgid3

        if (fmavx_status.flags & MAVLINKX_FLAGS_HAS_CRC_EXTRA) { // has crcextra
            status->rx_state = FASTMAVLINK_PARSE_STATE_CRC_EXTRA;
            return;
        }

        status->rx_state = FASTMAVLINK_PARSE_STATE_CRC8;
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

            // there is another 'o' in the header, so backtrack
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
        if (status->rx_cnt >= status->rx_header_len + fmavx_status.rx_payload_len) {

#ifdef MAVLINKX_COMPRESSION
            if (fmavx_status.flags & MAVLINKX_FLAGS_IS_COMPRESSED) {
                uint8_t len;
                _fmavX_payload_decompress(&(buf[status->rx_header_len]), &len, fmavx_status.rx_payload_len);
                uint8_t delta_len = (len - fmavx_status.rx_payload_len);
                status->rx_cnt += delta_len;
                fmavx_status.rx_payload_len += delta_len;
                status->rx_frame_len += delta_len;
                buf[fmavx_status.pos_of_len] = len;
            }
#endif

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
Compression scheme X4
Copyright (c) OlliW, OlliW42, www.olliw.eu
For terms of use/license see above permission notice.

The concept developed here is based on observations of the distribution of the bytes in the payload of
MAVLink messages. If the actual distribution is very different, the scheme becomes less effective, obviously.
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

Plenty of schemes were tried, this was (so far by far) the "best" one:

000             -> 0
0010            -> 255
00110 + 8 bits  -> RLE for 0: the code is followed by 8 bits to encode 1-255 0 bytes
00111 + 8 bits  -> RLE for 255: the code is followed by 8 bits to encode 1-255 255 bytes
10    + 6 bits  -> 1 .. 64
11    + 6 bits  -> 191 .. 254
01    + 7 bits  -> codes for remaining bytes not covered by the other branches

In decompression, detection of the end in the last byte is a problem.
For the scheme however, it is easily possible by noting that the bit sequences
  1111111
   111111
    11111
     1111
      111
       11
        1
are all not possible. The end can thus be marked by simply setting the last bits in
the last byte all to 1.
*/
#ifdef MAVLINKX_COMPRESSION

//-- compression
// we can use payload_out buffer as working buffer

// len_out & out_bit index the next bit position to write to
void _fmavX_encode_put_bits(uint8_t* payload_out, uint8_t* len_out, uint16_t code, uint8_t code_bit_len)
{
    uint16_t cur_code_bit = (uint16_t)1 << (code_bit_len - 1);

    for (uint8_t i = 0; i < code_bit_len; i++) {
        if (!(code & cur_code_bit)) { // clear the bit
            payload_out[*len_out] &=~ fmavx_status.out_bit;
        }
        cur_code_bit >>= 1; // next bit from code
        fmavx_status.out_bit >>= 1; // next bit pos in out bytes
        if (fmavx_status.out_bit == 0) { // out byte completely done, go to next out byte
            (*len_out)++;
            fmavx_status.out_bit = 0x80;
            payload_out[*len_out] = 0xFF;
        }
    }
}


void _fmavX_encode_rle(uint8_t* payload_out, uint8_t* len_out, uint8_t c, uint8_t RLE_cnt)
{
uint16_t code;

    if (c == 0x00) {
        if (RLE_cnt <= 4) { // 3 * x <= 13 => x <= 4
            for (uint8_t i = 0; i < RLE_cnt; i++) {
                code = (uint16_t)0b000; // 3 bit code = 3 bits length
                _fmavX_encode_put_bits(payload_out, len_out, code, 3);
            }
        } else {
            code = (uint16_t)0b00110 << 8; // 5 bit code + 8 bits = 13 bits length
            code += RLE_cnt;
            _fmavX_encode_put_bits(payload_out, len_out, code, 13);
        }
    } else
    if (c == 0xFF) {
        if (RLE_cnt <= 3) { // 4 * x <= 13 -> x <= 3
            for (uint8_t i = 0; i < RLE_cnt; i++) {
                code = (uint16_t)0b0010; // 4 bit code = 4 bits length
                _fmavX_encode_put_bits(payload_out, len_out, code, 4);
            }
        } else {
            code = (uint16_t)0b00111 << 8; // 5 bit code + 8 bits = 13 bits length
            code += RLE_cnt;
            _fmavX_encode_put_bits(payload_out, len_out, code, 13);
        }
    }
}


uint8_t _fmavX_payload_compress(uint8_t* payload_out, uint8_t* len_out, uint8_t* payload, uint8_t len)
{
uint16_t code; // can be 2 - 13 bits
uint8_t is_in_RLE;
uint8_t RLE_char;
uint8_t RLE_cnt;

    *len_out = 0;
    fmavx_status.out_bit = 0x80;
    payload_out[0] = 0xFF; // we use 1's as stop marker in last byte, so it's easier to fill with 0xFF

    is_in_RLE = 0;

    for (uint8_t n = 0; n < len; n++) {
        uint8_t c = payload[n];

        if (is_in_RLE) {
            if (c == RLE_char) {
                RLE_cnt++;
            } else {
                is_in_RLE = 0;
                _fmavX_encode_rle(payload_out, len_out, RLE_char, RLE_cnt);
            }
        }

        // handle c
        if (!is_in_RLE) {
            if (c == 0x00 || c == 0xFF) {
                is_in_RLE = 1;
                RLE_char = c;
                RLE_cnt = 1;
            } else
            if (c >= 1 && c <= 64) {
                code = (uint16_t)0b10 << 6; // 10: 2 bit code + 6 bits = 8 bits length
                code += (c - 1); // 0 .. 63
                _fmavX_encode_put_bits(payload_out, len_out, code, 8);
            } else
            if (c >= 191 && c <= 254) {
                code = (uint16_t)0b11 << 6; // 11: 2 bit code + 6 bits = 8 bits length
                code += (c - 191); // 0 .. 63
                _fmavX_encode_put_bits(payload_out, len_out, code, 8);
            } else {
                // 65 .. 190
                code = (uint16_t)0b01 << 7; // 01: 2 bit code + 7 bits = 9 bits length
                code += (c - 65); // 0 .. 125
                _fmavX_encode_put_bits(payload_out, len_out, code, 9);
            }
        }

        // compression doesn't reduce payload len
        // check it inside loop to prevent to write into memory outside of payload buffer
        if (*len_out >= len) return 0;
    }

    if (is_in_RLE) { // handle pending RLE
        _fmavX_encode_rle(payload_out, len_out, RLE_char, RLE_cnt);
    }

    if (fmavx_status.out_bit != 0x80) { // handle last out byte if not completed
        (*len_out)++; // count it
    }

    // compression didn't reduce payload len
    if (*len_out >= len) return 0;

    return 1;
}


//-- decompression

typedef enum {
    MAVLINKX_CODE_0 = 0,
    MAVLINKX_CODE_255,
    MAVLINKX_CODE_0_RLE,
    MAVLINKX_CODE_255_RLE,
    MAVLINKX_CODE_1_64,
    MAVLINKX_CODE_191_254,
    MAVLINKX_CODE_65_190,
    MAVLINKX_CODE_UNDEFINED,
} fmavx_code_e;


// TODO: shouldn't be global
uint8_t fmavx_in_buf[300];


uint8_t _fmavX_decode_get_bits(uint8_t* code, uint8_t len, uint8_t bits_len)
{
    *code = 0;
    for (uint8_t i = 0; i < bits_len; i++) {
        if (fmavx_status.in_pos >= len) { // reached end
            return 0;
        }
        *code <<= 1;
        if (fmavx_in_buf[fmavx_status.in_pos] & fmavx_status.in_bit) { // set the bit
            *code |= 0x01;
        }
        fmavx_status.in_bit >>= 1; // next bit in in byte
        if (fmavx_status.in_bit == 0) { // in byte completely done, go to next in byte
            fmavx_status.in_pos++;
            fmavx_status.in_bit = 0x80;
        }
    }
    return 1;
}


void _fmavX_payload_decompress(uint8_t* payload_out, uint8_t* len_out, uint8_t len)
{
    memcpy(fmavx_in_buf, payload_out, len); // copy current payload into work buffer

    *len_out = 0;

    fmavx_status.in_pos = 0;
    fmavx_status.in_bit = 0x80;

    while (1) {
        uint8_t c;

        // get next code
        uint8_t code = MAVLINKX_CODE_UNDEFINED;

        if (_fmavX_decode_get_bits(&c, len, 2)) {
            if (c == 0b10) { // 10
                code = MAVLINKX_CODE_1_64;
            } else
            if (c == 0b11) { // 11
                code = MAVLINKX_CODE_191_254;
            } else
            if (c == 0b01) { // 01
                code = MAVLINKX_CODE_65_190;
            } else
            if (_fmavX_decode_get_bits(&c, len, 1)) {
                if (c == 0b0) { // 000
                    code = MAVLINKX_CODE_0;
                } else
                if (_fmavX_decode_get_bits(&c, len, 1)) {
                    if (c == 0b0) { // 0010
                        code = MAVLINKX_CODE_255;
                    } else
                    if (_fmavX_decode_get_bits(&c, len, 1)) {
                        if (c == 0b0) { // 00110
                            code = MAVLINKX_CODE_0_RLE;
                        } else { // 00111
                            code = MAVLINKX_CODE_255_RLE;
                        }
                    }
                }
            }
        }

        if (code == MAVLINKX_CODE_UNDEFINED) return; // end

        switch (code) {
            case MAVLINKX_CODE_0:
                payload_out[(*len_out)++] = 0x00;
                break;
            case MAVLINKX_CODE_255:
                payload_out[(*len_out)++] = 0xFF;
                break;
            case MAVLINKX_CODE_0_RLE:
                if (!_fmavX_decode_get_bits(&c, len, 8)) return; // end
                for (uint8_t i = 0; i < c; i++) payload_out[(*len_out)++] = 0;
                break;
            case MAVLINKX_CODE_255_RLE:
                if (!_fmavX_decode_get_bits(&c, len, 8)) return; // end
                for (uint8_t i = 0; i < c; i++) payload_out[(*len_out)++] = 0xFF;
                break;
            case MAVLINKX_CODE_1_64:
                 if (!_fmavX_decode_get_bits(&c, len, 6)) return; // end
                 payload_out[(*len_out)++] = c + 1;
                break;
            case MAVLINKX_CODE_191_254:
                 if (!_fmavX_decode_get_bits(&c, len, 6)) return; // end
                 payload_out[(*len_out)++] = c + 191;
                break;
            case MAVLINKX_CODE_65_190:
                if (!_fmavX_decode_get_bits(&c, len, 7)) return; // end
                payload_out[(*len_out)++] = c + 65;
                break;
        }
    }
}


#endif // MAVLINKX_COMPRESSION


#ifdef MAVLINKX_O3
  #ifdef __GNUC__
    #pragma GCC pop_options
  #endif
#endif


#endif // MAVLINKX_H

