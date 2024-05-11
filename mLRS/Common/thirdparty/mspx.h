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

// https://github.com/iNavFlight/inav/wiki/MSP-V2

// STX1                       '$'
// STX2                       'X', 'M'
// flags                      type '<', '>', '!'
// function_1                 flag 0
// function_2                 function_1
// len                        function_2
// crc8                       len_1
// payload                    len_2
//                            payload
// crc16                      crc8


#define MSPX_MAGIC_1              '$'
#define MSPX_MAGIC_2              'o'
#define MSPX_HEADER_LEN           7
#define MSPX_FRAME_LEN_MAX        (7 + MSP_PAYLOAD_LEN_MAX + 1) // =  HEADER_LEN_MAX + PAYLOAD_LEN_MAX + CHECKSUM_LEN


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
    uint16_t frame_len;
} msp_status_t;


void msp_status_reset(msp_status_t* status)
{
    status->state = MSP_PARSE_STATE_IDLE;
    status->cnt = 0;
    status->frame_len = 0;
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
        if (c == MSP_MAGIC_2_V1 || c == MSP_MAGIC_2_V2 ) {
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



uint8_t msp_parse_to_msgX(msp_message_t* msg, msp_status_t* status, char c)
{
    return 0;
}


uint8_t msp_msgX_to_frame_buf(uint8_t* buf, msp_message_t* msg)
{
    return 0;
}




uint8_t msp_parse_and_check_to_frame_buf(uint8_t* buf, msp_status_t* status, char c)
{
     buf[0] = c;
     return 1;
}


uint16_t msp_frame_buf_to_mspX(uint8_t* buf_out, uint8_t* buf_in)
{
    buf_out[0] = buf_in[0];
    return 1;
}


uint8_t mspX_parse_and_check_to_frame_buf(uint8_t* buf, uint16_t* len, msp_status_t* status, char c)
{
    buf[0] = c;
    *len = 1;
    return 1;
}



void msp_init(void)
{
    // empty, nothing to do
}






void msp_function_str(char* s, msp_message_t* msg)
{
    switch (msg->function) {

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
        strcpy(s, u16toBCD_s(msg->function));
    }
}





#endif // MSPX_H
