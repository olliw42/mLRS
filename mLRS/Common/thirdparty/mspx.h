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

#define USE_MSPX

// STX1                       '$'
// STX2                       'X', 'M'
// flags                      type '<', '>', '!'
// function_1                 flag 0
// function_2                 function_1
// len_1                      function_2
// len_2                      len_1
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
    MSP_PARSE_STATE_FLAG,
    MSP_PARSE_STATE_FUNCTION_1,
    MSP_PARSE_STATE_FUNCTION_2,
    MSP_PARSE_STATE_LEN_1,
    MSP_PARSE_STATE_LEN_2,
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


void msp_status_reset(msp_status_t* status)
{
    status->state = MSP_PARSE_STATE_IDLE;
    status->cnt = 0;
}


void msp_parse_reset(msp_status_t* status)
{
    status->state = MSP_PARSE_STATE_IDLE;
}


uint8_t msp_parse_to_msg(msp_message_t* msg, msp_status_t* status, char c)
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
            status->state = MSP_PARSE_STATE_FLAG;
        } else {
            status->state = MSP_PARSE_STATE_IDLE;
        }
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

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


uint16_t msp_msg_to_frame_buf(uint8_t* buf, msp_message_t* msg)
{
    if (msg->len > MSP_PAYLOAD_LEN_MAX) { // can't handle it, so couldn't have received it completely
        return 0;
    }

    buf[0] = MSP_MAGIC_1;
    buf[1] = msg->magic2;
    buf[2] = msg->type;
    buf[3] = 0;
    buf[4] = msg->function;
    buf[5] = msg->function >> 8;
    buf[6] = msg->len;
    buf[7] = msg->len >> 8;

    uint16_t pos = 8;

    for (uint16_t i = 0; i < msg->len; i++) buf[pos++] = msg->payload[i];

    buf[pos] = msg->checksum;

    return msg->len + 8 + 1;
}


uint16_t msp_frame_buf_to_msg(msp_message_t* msg, uint8_t* buf)
{
    msg->magic2 = buf[1];
    msg->type = buf[2];
    msg->flag = 0; // buf[3]
    msg->function = (uint16_t)(buf[4]) + ((uint16_t)(buf[5]) << 8);
    msg->len = (uint16_t)(buf[6]) + ((uint16_t)(buf[7]) << 8);

    if (msg->len > MSP_PAYLOAD_LEN_MAX) { // can't handle it, so couldn't have received it completely
        return 0;
    }

    uint16_t pos = 8;

    for (uint16_t i = 0; i < msg->len; i++) msg->payload[i] = buf[pos++];

    msg->checksum = buf[pos];

    return msg->len + 8 + 1;
}


//-------------------------------------------------------
// MspX
//-------------------------------------------------------

typedef enum {
    MSP_PARSE_STATE_FLAGS = 100,
    MSP_PARSE_STATE_CRC8,
    MSP_PARSE_STATE_CRC16_1,
    MSP_PARSE_STATE_CRC16_2,
} mspx_parse_state_e;


typedef enum {
    MSP_FLAGS_REQUEST   = 0x01,
    MSP_FLAGS_ERROR     = 0x02,
} mspx_flags_e;


typedef struct
{
    uint8_t flags;
    uint8_t crc8;
    uint16_t crc16;
} mspx_status_t;


// TODO: shouldn't be global
mspx_status_t mspx_status = {};


// converting from mspX
// result in msg needs to be correct msp message
uint8_t msp_parseX_to_msg(msp_message_t* msg, msp_status_t* status, char c)
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
            status->state = MSP_PARSE_STATE_FLAGS;
        } else {
            status->state = MSP_PARSE_STATE_IDLE;
        }
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

    case MSP_PARSE_STATE_FLAGS:
        mspx_status.flags = c;
        // fake correct msp header
        msg->magic2 = MSP_MAGIC_2_V2;
        if (mspx_status.flags & MSP_FLAGS_REQUEST) {
            msg->type = MSP_TYPE_REQUEST;
        } else if (mspx_status.flags & MSP_FLAGS_ERROR) {
            msg->type = MSP_TYPE_ERROR;
        } else {
            msg->type = MSP_TYPE_RESPONSE;
        }
        msg->flag = 0;
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
        status->state = MSP_PARSE_STATE_CRC8;
        msg->res = MSP_PARSE_RESULT_NONE;
        return 0;

    case MSP_PARSE_STATE_CRC8: {
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
            status->state = MSP_PARSE_STATE_CRC16_1;
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
            status->state = MSP_PARSE_STATE_CRC16_1;
        }
        msg->res = MSP_PARSE_RESULT_HAS_HEADER;
        return 0;

    case MSP_PARSE_STATE_CRC16_1:
        mspx_status.crc16 = c;
        status->state = MSP_PARSE_STATE_CRC16_2;
        msg->res = MSP_PARSE_RESULT_HAS_HEADER;
        return 0;

    case MSP_PARSE_STATE_CRC16_2: {
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

        // fake correct msp checksum
        msg->checksum = crsf_crc8_calc(0, msg->flag);
        msg->checksum = crsf_crc8_calc(msg->checksum, msg->function);
        msg->checksum = crsf_crc8_calc(msg->checksum, msg->function >> 8);
        msg->checksum = crsf_crc8_calc(msg->checksum, msg->len);
        msg->checksum = crsf_crc8_calc(msg->checksum, msg->len >> 8);
        msg->checksum = crsf_crc8_update(msg->checksum, msg->payload, msg->len);

        msp_status_reset(status);
        msg->res = MSP_PARSE_RESULT_OK;
        return 1; }
    }

    // should never get to here
    msp_status_reset(status);
    return 0;
}


// converting to mspX !!
uint16_t msp_msg_to_frame_bufX(uint8_t* buf, msp_message_t* msg)
{
    if (msg->len > MSP_PAYLOAD_LEN_MAX) { // can't handle it, so couldn't have received it completely
        return 0;
    }

    uint8_t flags = 0;
    if (msg->type == MSP_TYPE_REQUEST) {
        flags |= MSP_FLAGS_REQUEST;
    } else if (msg->type == MSP_TYPE_ERROR) {
        flags |= MSP_FLAGS_ERROR;
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


uint16_t msp_generate_frame_bufX(uint8_t* buf, uint8_t type, uint16_t function, uint8_t* payload, uint16_t len)
{
    if (len > MSP_PAYLOAD_LEN_MAX) { // can't handle it, so couldn't have received it completely
        return 0;
    }

    uint8_t flags = 0;
    if (type == MSP_TYPE_REQUEST) {
        flags |= MSP_FLAGS_REQUEST;
    } else if (type == MSP_TYPE_ERROR) {
        flags |= MSP_FLAGS_ERROR;
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

void msp_msg_recalculate_crc(msp_message_t* msg)
{
    msg->checksum = crsf_crc8_calc(0, msg->flag);
    msg->checksum = crsf_crc8_calc(msg->checksum, msg->function);
    msg->checksum = crsf_crc8_calc(msg->checksum, msg->function >> 8);
    msg->checksum = crsf_crc8_calc(msg->checksum, msg->len);
    msg->checksum = crsf_crc8_calc(msg->checksum, msg->len >> 8);
    for (uint16_t i = 0; i < msg->len; i++) msg->checksum = crsf_crc8_calc(msg->checksum, msg->payload[i]);
}


uint16_t msp_generate_request_to_msg(msp_message_t* msg, uint8_t type, uint16_t function)
{
    msg->magic2 = MSP_MAGIC_2_V2;
    msg->type = type;
    msg->flag = 0;
    msg->function = function;
    msg->len = 0;
    msg->checksum = crsf_crc8_calc(0, msg->flag);
    msg->checksum = crsf_crc8_calc(msg->checksum, msg->function);
    msg->checksum = crsf_crc8_calc(msg->checksum, msg->function >> 8);
    msg->checksum = crsf_crc8_calc(msg->checksum, msg->len);
    msg->checksum = crsf_crc8_calc(msg->checksum, msg->len >> 8);
    return 8 + 1;
}


uint16_t msp_generate_request_to_frame_buf(uint8_t* buf, uint8_t type, uint16_t function)
{
    buf[0] = MSP_MAGIC_1;
    buf[1] = MSP_MAGIC_2_V2;
    buf[2] = type;
    buf[3] = 0;
    buf[4] = function;
    buf[5] = function >> 8;
    buf[6] = 0;
    buf[7] = 0;
    buf[8] = crsf_crc8_update(0, &(buf[3]), 5);
    return 8 + 1;
}


uint16_t msp_generate_frame_buf(uint8_t* buf, uint8_t type, uint16_t function, uint8_t* payload, uint16_t len)
{
    if (len > MSP_PAYLOAD_LEN_MAX) { // can't handle it, so couldn't have received it completely
        return 0;
    }

    buf[0] = MSP_MAGIC_1;
    buf[1] = MSP_MAGIC_2_V2;
    buf[2] = type;
    buf[3] = 0;
    buf[4] = function;
    buf[5] = function >> 8;
    buf[6] = len;
    buf[7] = len >> 8;

    uint16_t pos = 8;

    for (uint16_t i = 0; i < len; i++) buf[pos++] = payload[i];

    buf[pos] = crsf_crc8_update(0, &(buf[3]), len + 5);

    return len + 8 + 1;
}


void msp_type_str(char* s, msp_message_t* msg)
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


void msp_function_str_from_msg(char* s, msp_message_t* msg)
{
    msp_function_str(s, msg->function);
}



#endif // MSPX_H
