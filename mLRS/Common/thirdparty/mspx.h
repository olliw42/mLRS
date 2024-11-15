//------------------------------
// The MSP X library
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// All Rights Reserved.
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies, and
// that the use of this software in a product is acknowledged in the product
// documentation or a location viewable to users.
//------------------------------
#pragma once
#ifndef MSPX_H
#define MSPX_H

// STX1                       '$'                     '$'
// STX2                       'X'                     'M'
// flags                      type '<', '>', '!'      type '<', '>', '!'
// function_1                 flag 0                  len
// function_2                 function_1              function
// len_1                      function_2              payload
// len_2                      len_1                   crc8
// crc8                       len_2
// paylaod                    payload
// crc16_1                    crc8
// crc16_2


#define MSPX_MAGIC_2              'o'
#define MSPX_HEADER_LEN           8
#define MSPX_FRAME_LEN_MAX        (8 + MSP_PAYLOAD_LEN_MAX + 2) // =  HEADER_LEN_MAX + PAYLOAD_LEN_MAX + CHECKSUM_LEN


typedef enum {
    MSP_PARSE_STATE_IDLE = 0,
    MSP_PARSE_STATE_MAGIC_2,
    MSP_PARSE_STATE_TYPE,
    // v2
    MSP_PARSE_STATE_FLAG,
    MSP_PARSE_STATE_FUNCTION_1,
    MSP_PARSE_STATE_FUNCTION_2,
    MSP_PARSE_STATE_LEN_1,
    MSP_PARSE_STATE_LEN_2,
    // v1
    MSP_PARSE_STATE_V1_LEN,
    MSP_PARSE_STATE_V1_FUNCTION,
    // v1 & v2
    MSP_PARSE_STATE_PAYLOAD,
    MSP_PARSE_STATE_CHECKSUM,
} msp_parse_state_e;


typedef enum {
    MSP_PARSE_RESULT_NONE = 0,
    MSP_PARSE_RESULT_HAS_HEADER,
    MSP_PARSE_RESULT_OK,
    MSP_PARSE_RESULT_CRC_ERROR,
} msp_parse_result_e;


typedef struct {
    uint8_t state;
    uint16_t cnt;
} msp_status_t;


void msp_status_reset(msp_status_t* const status)
{
    status->state = MSP_PARSE_STATE_IDLE;
    status->cnt = 0;
}


void msp_parse_reset(msp_status_t* const status)
{
    status->state = MSP_PARSE_STATE_IDLE;
}


uint8_t msp_parse_to_msg(msp_message_t* const msg, msp_status_t* const status, char c)
{
    if (status->cnt >= MSP_FRAME_LEN_MAX) { // this should never happen, but play it safe
        status->state = MSP_PARSE_STATE_IDLE;
    }

    switch (status->state) {
    case MSP_PARSE_STATE_IDLE:
        msp_status_reset(status);
        if (c == MSP_MAGIC_1) {
            status->state = MSP_PARSE_STATE_MAGIC_2;
        }
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

    case MSP_PARSE_STATE_MAGIC_2:
        if (c == MSP_MAGIC_2_V1 || c == MSP_MAGIC_2_V2) {
            msg->magic2 = c;
            status->state = MSP_PARSE_STATE_TYPE;
        } else {
            status->state = MSP_PARSE_STATE_IDLE;
        }
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

    case MSP_PARSE_STATE_TYPE:
        if (c == MSP_TYPE_REQUEST || c == MSP_TYPE_RESPONSE || c == MSP_TYPE_ERROR) {
            msg->type = c;
            status->state = (msg->magic2 == MSP_MAGIC_2_V1) ? MSP_PARSE_STATE_V1_LEN : MSP_PARSE_STATE_FLAG;
            msg->flag = 0; // for v1 it will not be set, so ensure it is set
        } else {
            status->state = MSP_PARSE_STATE_IDLE;
        }
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

    // v2
    case MSP_PARSE_STATE_FLAG:
        msg->flag = c;
        status->state = MSP_PARSE_STATE_FUNCTION_1;
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

    case MSP_PARSE_STATE_FUNCTION_1:
        msg->function = c;
        status->state = MSP_PARSE_STATE_FUNCTION_2;
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

    case MSP_PARSE_STATE_FUNCTION_2:
        msg->function += ((uint16_t)c << 8);
        status->state = MSP_PARSE_STATE_LEN_1;
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

    case MSP_PARSE_STATE_LEN_1:
        msg->len = c;
        status->state = MSP_PARSE_STATE_LEN_2;
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

    case MSP_PARSE_STATE_LEN_2:
        msg->len += ((uint16_t)c << 8);
        if (msg->len == 0) {
            status->state = MSP_PARSE_STATE_CHECKSUM;
        } else {
            status->state = MSP_PARSE_STATE_PAYLOAD;
        }
        status->cnt = 0;
        msg->res = MSP_PARSE_RESULT_HAS_HEADER;
        if (msg->len > MSP_PAYLOAD_LEN_MAX) {
            msp_status_reset(status);
        }
        return 0;

    // v1
    case MSP_PARSE_STATE_V1_LEN:
        msg->len = c;
        status->state = MSP_PARSE_STATE_V1_FUNCTION;
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

    case MSP_PARSE_STATE_V1_FUNCTION:
        msg->function = c;
        status->state = MSP_PARSE_STATE_PAYLOAD;
        msg->res = MSP_PARSE_RESULT_HAS_HEADER;
        if (msg->len > MSP_PAYLOAD_LEN_MAX) { // should never happen for v1
            msp_status_reset(status);
        }
        return 0;

    // v1 & v2
    case MSP_PARSE_STATE_PAYLOAD:
        if (status->cnt < MSP_PAYLOAD_LEN_MAX) msg->payload[status->cnt] = c; // payload
        status->cnt++;
        if (status->cnt >= msg->len) {
            status->state = MSP_PARSE_STATE_CHECKSUM;
        }
        msg->res = MSP_PARSE_RESULT_HAS_HEADER;
        return 0;

    case MSP_PARSE_STATE_CHECKSUM:
        msg->checksum = c;
        msp_status_reset(status);
        msg->res = MSP_PARSE_RESULT_OK;
        return 1;
    }

    // should never get to here
    msp_status_reset(status);
    return 0;
}


uint16_t msp_msg_to_frame_buf(uint8_t* const buf, msp_message_t* const msg)
{
    if (msg->len > MSP_PAYLOAD_LEN_MAX) { // can't handle it, so couldn't have received it completely
        return 0;
    }

    buf[0] = MSP_MAGIC_1;
    buf[1] = msg->magic2;
    buf[2] = msg->type;

    uint16_t pos = 8;
    uint16_t header_len = 8;

    if (msg->magic2 == MSP_MAGIC_2_V1) {
        buf[3] = msg->len;
        buf[4] = msg->function;
        pos = header_len = 5;
    } else {
        buf[3] = msg->flag;
        buf[4] = msg->function;
        buf[5] = msg->function >> 8;
        buf[6] = msg->len;
        buf[7] = msg->len >> 8;
        pos = header_len = 8;
    }

    for (uint16_t i = 0; i < msg->len; i++) buf[pos++] = msg->payload[i];

    buf[pos] = msg->checksum;

    return msg->len + header_len + 1;
}


uint16_t msp_frame_buf_to_msg(msp_message_t* const msg, uint8_t* const buf)
{
    msg->magic2 = buf[1];
    msg->type = buf[2];

    uint16_t pos = 8;
    uint16_t header_len = 8;

    if (msg->magic2 == MSP_MAGIC_2_V1) {
        msg->flag = 0;
        msg->len = buf[3];
        msg->function = buf[4];
        pos = header_len = 5;
    } else {
        msg->flag = buf[3];
        msg->function = (uint16_t)(buf[4]) + ((uint16_t)(buf[5]) << 8);
        msg->len = (uint16_t)(buf[6]) + ((uint16_t)(buf[7]) << 8);
        pos = header_len = 8;
    }

    if (msg->len > MSP_PAYLOAD_LEN_MAX) { // can't handle it, so couldn't have received it completely
        return 0;
    }

    for (uint16_t i = 0; i < msg->len; i++) msg->payload[i] = buf[pos++];

    msg->checksum = buf[pos];

    return msg->len + header_len + 1;
}


//-------------------------------------------------------
// MspX
//-------------------------------------------------------

typedef enum {
    MSPX_PARSE_STATE_FLAGS = 100,
    MSPX_PARSE_STATE_CRC8,
    MSPX_PARSE_STATE_CRC16_1,
    MSPX_PARSE_STATE_CRC16_2,
} mspx_parse_state_e;


typedef enum {
    MSPX_FLAGS_REQUEST            = 0x01,
    MSPX_FLAGS_ERROR              = 0x02,
    MSPX_FLAGS_V1                 = 0x04,
    MSPX_FLAGS_NO_RESPONSE        = 0x08,
    MSPX_FLAGS_SOURCE_ID_RC_LINK  = 0x10,
} mspx_flags_e;


typedef struct
{
    uint8_t flags;
    uint8_t crc8;
    uint16_t crc16;
} mspx_status_t;


// TODO: shouldn't be global
mspx_status_t mspx_status = {};


// converting from MSP X
// result in msg needs to be correct MSP message
uint8_t msp_parseX_to_msg(msp_message_t* const msg, msp_status_t* const status, char c)
{
    if (status->cnt >= MSP_FRAME_LEN_MAX) { // this should never happen, but play it safe
        status->state = MSP_PARSE_STATE_IDLE;
    }

    switch (status->state) {
    case MSP_PARSE_STATE_IDLE:
        msp_status_reset(status);
        if (c == MSP_MAGIC_1) {
            status->state = MSP_PARSE_STATE_MAGIC_2;
        }
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

    case MSP_PARSE_STATE_MAGIC_2:
        if (c == MSPX_MAGIC_2) {
            status->state = MSPX_PARSE_STATE_FLAGS;
        } else {
            status->state = MSP_PARSE_STATE_IDLE;
        }
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

    case MSPX_PARSE_STATE_FLAGS:
        mspx_status.flags = c;
        msg->magic2 = (mspx_status.flags & MSPX_FLAGS_V1) ? MSP_MAGIC_2_V1 : MSP_MAGIC_2_V2;
        if (mspx_status.flags & MSPX_FLAGS_REQUEST) {
            msg->type = MSP_TYPE_REQUEST;
        } else if (mspx_status.flags & MSPX_FLAGS_ERROR) {
            msg->type = MSP_TYPE_ERROR;
        } else {
            msg->type = MSP_TYPE_RESPONSE;
        }
        msg->flag = 0;
        if (mspx_status.flags & MSPX_FLAGS_NO_RESPONSE) {
            msg->flag |= MSP_FLAG_NO_RESPONSE;
        }
        if (mspx_status.flags & MSPX_FLAGS_SOURCE_ID_RC_LINK) {
            msg->flag |= MSP_FLAG_SOURCE_ID_RC_LINK;
        }
        status->state = MSP_PARSE_STATE_FUNCTION_1;
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

    case MSP_PARSE_STATE_FUNCTION_1:
        msg->function = c;
        status->state = MSP_PARSE_STATE_FUNCTION_2;
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

    case MSP_PARSE_STATE_FUNCTION_2:
        msg->function += ((uint16_t)c << 8);
        status->state = MSP_PARSE_STATE_LEN_1;
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

    case MSP_PARSE_STATE_LEN_1:
        msg->len = c;
        status->state = MSP_PARSE_STATE_LEN_2;
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

    case MSP_PARSE_STATE_LEN_2:
        msg->len += ((uint16_t)c << 8);
        status->state = MSPX_PARSE_STATE_CRC8;
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

    case MSPX_PARSE_STATE_CRC8: {
        mspx_status.crc8 = c;
        uint8_t crc8;
        fmavX_crc8_init(&crc8);
        fmavX_crc8_accumulate(&crc8, mspx_status.flags);
        fmavX_crc8_accumulate(&crc8, msg->function);
        fmavX_crc8_accumulate(&crc8, msg->function >> 8);
        fmavX_crc8_accumulate(&crc8, msg->len);
        fmavX_crc8_accumulate(&crc8, msg->len >> 8);
        if (mspx_status.crc8 != crc8) {
            msp_status_reset(status);
            msg->res = MSP_PARSE_RESULT_CRC_ERROR;
            return 0;
        }
        if (msg->len == 0) {
            status->state = MSPX_PARSE_STATE_CRC16_1;
        } else {
            status->state = MSP_PARSE_STATE_PAYLOAD;
        }
        msg->res = MSP_PARSE_RESULT_HAS_HEADER;
        status->cnt = 0;
        if (msg->len > MSP_PAYLOAD_LEN_MAX) {
            msp_status_reset(status);
        }
        return 0; }

    case MSP_PARSE_STATE_PAYLOAD:
        if (status->cnt < MSP_PAYLOAD_LEN_MAX) msg->payload[status->cnt] = c; // payload
        status->cnt++;
        if (status->cnt >= msg->len) {
            status->state = MSPX_PARSE_STATE_CRC16_1;
        }
        msg->res = MSP_PARSE_RESULT_HAS_HEADER;
        return 0;

    case MSPX_PARSE_STATE_CRC16_1:
        mspx_status.crc16 = c;
        status->state = MSPX_PARSE_STATE_CRC16_2;
        msg->res = MSP_PARSE_RESULT_HAS_HEADER;
        return 0;

    case MSPX_PARSE_STATE_CRC16_2: {
        mspx_status.crc16 += ((uint16_t)c << 8);
        uint16_t crc16;
        fmav_crc_init(&crc16);
        fmav_crc_accumulate(&crc16, mspx_status.flags);
        fmav_crc_accumulate(&crc16, msg->function);
        fmav_crc_accumulate(&crc16, msg->function >> 8);
        fmav_crc_accumulate(&crc16, msg->len);
        fmav_crc_accumulate(&crc16, msg->len >> 8);
        fmav_crc_accumulate(&crc16, mspx_status.crc8);
        fmav_crc_accumulate_buf(&crc16, msg->payload, msg->len);
        if (mspx_status.crc16 != crc16) {
            msp_status_reset(status);
            msg->res = MSP_PARSE_RESULT_CRC_ERROR;
            return 0;
        }

        // fake correct MSP checksum
        if (mspx_status.flags & MSPX_FLAGS_V1) {
            msg->checksum = 0;
            msg->checksum ^= msg->len;
            msg->checksum ^= msg->function;
            for (uint16_t n = 0; n < msg->len; n++) msg->checksum ^= msg->payload[n];
        } else {
            msg->checksum = crsf_crc8_calc(0, msg->flag);
            msg->checksum = crsf_crc8_calc(msg->checksum, msg->function);
            msg->checksum = crsf_crc8_calc(msg->checksum, msg->function >> 8);
            msg->checksum = crsf_crc8_calc(msg->checksum, msg->len);
            msg->checksum = crsf_crc8_calc(msg->checksum, msg->len >> 8);
            msg->checksum = crsf_crc8_update(msg->checksum, msg->payload, msg->len);
        }

        msp_status_reset(status);
        msg->res = MSP_PARSE_RESULT_OK;
        return 1; }
    }

    // should never get to here
    msp_status_reset(status);
    return 0;
}


// converting to MSP X !
uint16_t msp_msg_to_frame_bufX(uint8_t* const buf, msp_message_t* const msg)
{
    if (msg->len > MSP_PAYLOAD_LEN_MAX) { // can't handle it, so couldn't have received it completely
        return 0;
    }

    uint8_t flags = 0;
    if (msg->type == MSP_TYPE_REQUEST) {
        flags |= MSPX_FLAGS_REQUEST;
    } else if (msg->type == MSP_TYPE_ERROR) {
        flags |= MSPX_FLAGS_ERROR;
    }
    if (msg->magic2 == MSP_MAGIC_2_V1) {
        flags |= MSPX_FLAGS_V1;
    }

    if (msg->flag & MSP_FLAG_NO_RESPONSE) {
        flags |= MSPX_FLAGS_NO_RESPONSE;
    }
    if (msg->flag & MSP_FLAG_SOURCE_ID_RC_LINK) {
        flags |= MSPX_FLAGS_SOURCE_ID_RC_LINK;
    }

    buf[0] = MSP_MAGIC_1;
    buf[1] = MSPX_MAGIC_2;
    buf[2] = flags;
    buf[3] = msg->function;
    buf[4] = msg->function >> 8;
    buf[5] = msg->len;
    buf[6] = msg->len >> 8;
    buf[7] = fmavX_crc8_calculate(&(buf[2]), 5);

    uint16_t pos = 8;

    for (uint16_t i = 0; i < msg->len; i++) buf[pos++] = msg->payload[i];

    uint16_t crc16 = fmav_crc_calculate(&(buf[2]), msg->len + 6);
    buf[pos++] = crc16;
    buf[pos++] = crc16 >> 8;

    return msg->len + 8 + 2;
}


// generate a MSPX frame holding a MSP V2 message
uint16_t msp_generate_v2_frame_bufX(uint8_t* const buf, uint8_t type, uint8_t flag, uint16_t function, uint8_t* const payload, uint16_t len)
{
    if (len > MSP_PAYLOAD_LEN_MAX) { // can't handle it, so couldn't have received it completely
        return 0;
    }

    uint8_t flags = 0;
    if (type == MSP_TYPE_REQUEST) {
        flags |= MSPX_FLAGS_REQUEST;
    } else if (type == MSP_TYPE_ERROR) {
        flags |= MSPX_FLAGS_ERROR;
    }

    if (flag & MSP_FLAG_NO_RESPONSE) {
        flags |= MSPX_FLAGS_NO_RESPONSE;
    }
    if (flag & MSP_FLAG_SOURCE_ID_RC_LINK) {
        flags |= MSPX_FLAGS_SOURCE_ID_RC_LINK;
    }

    buf[0] = MSP_MAGIC_1;
    buf[1] = MSPX_MAGIC_2;
    buf[2] = flags;
    buf[3] = function;
    buf[4] = function >> 8;
    buf[5] = len;
    buf[6] = len >> 8;
    buf[7] = fmavX_crc8_calculate(&(buf[2]), 5);

    uint16_t pos = 8;

    for (uint16_t i = 0; i < len; i++) buf[pos++] = payload[i];

    uint16_t crc16 = fmav_crc_calculate(&(buf[2]), len + 6);
    buf[pos++] = crc16;
    buf[pos++] = crc16 >> 8;

    return len + 8 + 2;
}


//-------------------------------------------------------
// Init
//-------------------------------------------------------

void msp_init(void)
{
    // empty, nothing to do
}


//-------------------------------------------------------
// Helper
//-------------------------------------------------------

void msp_msg_recalculate_crc(msp_message_t* const msg)
{
    msg->checksum = crsf_crc8_calc(0, msg->flag);
    msg->checksum = crsf_crc8_calc(msg->checksum, msg->function);
    msg->checksum = crsf_crc8_calc(msg->checksum, msg->function >> 8);
    msg->checksum = crsf_crc8_calc(msg->checksum, msg->len);
    msg->checksum = crsf_crc8_calc(msg->checksum, msg->len >> 8);
    msg->checksum = crsf_crc8_update(msg->checksum, msg->payload, msg->len);
}


/* not used
uint16_t msp_generate_v2_request_to_msg(msp_message_t* const msg, uint8_t type, uint8_t flag, uint16_t function)
{
    msg->magic2 = MSP_MAGIC_2_V2;
    msg->type = type;
    msg->flag = flag;
    msg->function = function;
    msg->len = 0;
    msg->checksum = crsf_crc8_calc(0, msg->flag);
    msg->checksum = crsf_crc8_calc(msg->checksum, msg->function);
    msg->checksum = crsf_crc8_calc(msg->checksum, msg->function >> 8);
    msg->checksum = crsf_crc8_calc(msg->checksum, msg->len);
    msg->checksum = crsf_crc8_calc(msg->checksum, msg->len >> 8);
    return 8 + 1;
}
*/


// generate a MSP v2 request frame (simpler since no payload)
uint16_t msp_generate_v2_request_to_frame_buf(uint8_t* const buf, uint8_t type, uint8_t flag, uint16_t function)
{
    buf[0] = MSP_MAGIC_1;
    buf[1] = MSP_MAGIC_2_V2;
    buf[2] = type;
    buf[3] = flag;
    buf[4] = function;
    buf[5] = function >> 8;
    buf[6] = 0;
    buf[7] = 0;
    buf[8] = crsf_crc8_update(0, &(buf[3]), 5);
    return 8 + 1;
}


// generate a MSP v2 frame
uint16_t msp_generate_v2_frame_buf(uint8_t* const buf, uint8_t type, uint8_t flag, uint16_t function, uint8_t* const payload, uint16_t len)
{
    if (len > MSP_PAYLOAD_LEN_MAX) { // can't handle it, so couldn't have received it completely
        return 0;
    }

    buf[0] = MSP_MAGIC_1;
    buf[1] = MSP_MAGIC_2_V2;
    buf[2] = type;
    buf[3] = flag;
    buf[4] = function;
    buf[5] = function >> 8;
    buf[6] = len;
    buf[7] = len >> 8;

    uint16_t pos = 8;

    for (uint16_t i = 0; i < len; i++) buf[pos++] = payload[i];

    buf[pos] = crsf_crc8_update(0, &(buf[3]), len + 5);

    return len + 8 + 1;
}


void msp_type_str(char* s, msp_message_t* const msg)
{
    switch (msg->type) {
    case MSP_TYPE_REQUEST: strcpy(s, "req"); break;
    case MSP_TYPE_RESPONSE: strcpy(s, "rsp"); break;
    case MSP_TYPE_ERROR: strcpy(s, "err"); break;
    default:
        strcpy(s, u8toBCD_s(msg->type));
    }
}


void msp_function_str(char* s, uint16_t function)
{
    switch (function) {

    case MSP_STATUS: strcpy(s, "STATUS"); break;
    case 102: strcpy(s, "RAW_IMU"); break;
    case 106: strcpy(s, "RAW_GPS"); break;
    case 107: strcpy(s, "COMP_GPS"); break;
    case 108: strcpy(s, "ATTITUDE"); break;
    case 109: strcpy(s, "ALTITUDE"); break;
    case 110: strcpy(s, "ANALOG"); break;
    case 113: strcpy(s, "ACTIVEBOXES"); break;
    case 114: strcpy(s, "MISC"); break;
    case 121: strcpy(s, "NAV_STATUS"); break;
    case 130: strcpy(s, "BATTERY_STATE"); break;
    case 150: strcpy(s, "STATUS_EX"); break;
    case 151: strcpy(s, "SENSOR_STATUS"); break;

    case MSP_SET_RAW_RC: strcpy(s, "SET_RAW_RC"); break;

    case 0x2000: strcpy(s, "INAV_STATUS"); break;
    case 0x2002: strcpy(s, "INAV_ANALOG"); break;
    case 0x2003: strcpy(s, "INAV_MISC"); break;

    case 0x1F01: strcpy(s, "RANGEFINDER"); break;
    case 0x1F02: strcpy(s, "OPTIC_FLOW"); break;
    case 0x1F03: strcpy(s, "GPS"); break;
    case 0x1F04: strcpy(s, "COMPASS"); break;
    case 0x1F05: strcpy(s, "BAROMETER"); break;
    case 0x1F06: strcpy(s, "AIRSPEED"); break;

    default:
        strcpy(s, u16toBCD_s(function));
    }
}


void msp_function_str_from_msg(char* s, msp_message_t* const msg)
{
    msp_function_str(s, msg->function);
}


//-------------------------------------------------------
// Compression
//-------------------------------------------------------

// compress
// also set inav_flight_modes_box_mode_flags[]
void mspX_boxnames_payload_compress(uint8_t* const payload_out, uint16_t* const len_out, uint8_t* const payload, uint16_t len, uint8_t* const inav_flight_modes_box_mode_flags)
{
    char s[48];
    uint8_t pos = 0;
    uint8_t state = 0;
    uint8_t box = 0; // inavFlightModes.boxModeFlag handling

    *len_out = 0;

    for (uint16_t i = 0; i < len; i++) {
        if (payload[i] != ';') {
            s[pos++] = payload[i];
            if (pos >= 32) pos = 0;
        } else {
            s[pos++] = '\0';
            bool found = false;
            for (uint8_t n = 0; n < INAV_BOXES_COUNT; n++) {
                if (!strcmp(s, inavBoxes[n].boxName)) {
                    if (state != 0xFF) { state = 0xFF; payload_out[(*len_out)++] = 0xFF; }
                    payload_out[(*len_out)++] = n;
                    found = true;

                    if (inavBoxes[n].flightModeFlag < INAV_FLIGHT_MODES_COUNT) { // is a flight mode we want  to record im MSPX_STATUS
                        inav_flight_modes_box_mode_flags[inavBoxes[n].flightModeFlag] = box; // inav_flight_modes_box_mode_flag handling
                    }

                    break; // found, no need to look further
                }
            }
            if (!found) {
                if (state != 0xFE) { state = 0xFE; payload_out[(*len_out)++] = 0xFE; }
                for (uint8_t n = 0; n < strlen(s); n++) payload_out[(*len_out)++] = s[n];
                payload_out[(*len_out)++] = ';';
            }
            pos = 0;

            box++; // inav_flight_modes_box_mode_flags handling
        }
    }
}


// decompress
void mspX_boxnames_payload_decompress(msp_message_t* const msg, uint8_t* const buf)
{
    uint8_t payload_len = msg->len;
    memcpy(buf, msg->payload, msg->len);

    msg->len = 0;
    uint8_t state = 0;

    for (uint16_t i = 0; i < payload_len; i++) {
        uint8_t c = buf[i];
        if (c == 0xFF) {
            state = 0xFF;
        } else
        if (c == 0xFE) {
            state = 0xFE;
        } else
        if (state == 0xFF) {
            if (c < INAV_BOXES_COUNT) { // protect against nonsense
                for (uint8_t n = 0; n < strlen(inavBoxes[c].boxName); n++) {
                    msg->payload[msg->len++] = (inavBoxes[c].boxName)[n];
                }
                msg->payload[msg->len++] = ';';
            }
        } else
        if (state == 0xFE) {
            msg->payload[msg->len++] = c;
        }
    }

    // we now need to recalculate the crc
    msp_msg_recalculate_crc(msg);
}


#endif // MSPX_H

/*
INAV7.1
fc -> gcs:

connect:
-          116: len = 340     MSP_BOXNAMES  all chars, compressed to ca 45

MIXER
- x2020 / 8224: len = 384     MSP2_INAV_SERVO_MIXER               2*16 * 6 = 192 + 2*16 * 6 = 192 = 384

- x1005 / 4101: len = 192     MSP2_COMMON_MOTOR_MIXER

- x2026 / 8230: len = 256     MSP2_INAV_LOGIC_CONDITIONS_STATUS   64*4 = 256

OUTPUTS
- x2020 / 8224: len = 384

- x1005 / 4101: len = 192

-          120: len = 224     MSP_SERVO_CONFIGURATIONS

PORTS:
CONFIGURATION:
- x1007 / 4103: len = varies  MSP2_COMMON_SETTING_INFO            contains chars

FAILSAFE:
- x1007 / 4103: len = varies

PID TUNING
- x1007 / 4103: len = varies

ADVANCED TUNNING:
- x1007 / 4103: len = varies

PROGRAMMING:
- x2026 / 8230: len = 256

RECEIVER:
- x1007 / 4103: len = varies

MODES:
-          116: len = 340

-           34: len = 160     MSP_MODE_RANGES

ADJUSTMENTS:
-          116: len = 340

-           52: len = 120

GPS:
ALIGNMENT TOOL:
MISSION CONTROL:

OSD
- x2012 / 8120: len = 340     MSP2_INAV_OSD_LAYOUTS
- x1007 / 4103: len = varies
- x2020 / 8224: len = 384
LED STRIP
- x2048 / 8264: len = 510     MSP2_INAV_LED_STRIP_CONFIG_EX     128*5 = 640



*/
