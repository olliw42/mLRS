//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Mavlink
//*******************************************************
#pragma once
#ifndef FASMAVLINK_EXTENSION_H
#define FASMAVLINK_EXTENSION_H


FASTMAVLINK_FUNCTION_DECORATOR void fmav_frame_buf_set_seq(uint8_t* buf, uint8_t crc_extra, uint8_t new_seq)
{
    buf[4] = new_seq;

    uint8_t len = buf[1];
    if (buf[0] == FASTMAVLINK_MAGIC_V2) {
        uint16_t crc = fmav_crc_calculate(&(buf[1]), FASTMAVLINK_HEADER_V2_LEN - 1);
        fmav_crc_accumulate_buf(&crc, &buf[FASTMAVLINK_HEADER_V2_LEN], len);
        fmav_crc_accumulate(&crc, crc_extra);
        buf[len + FASTMAVLINK_HEADER_V2_LEN] = (uint8_t)crc;
        buf[len + FASTMAVLINK_HEADER_V2_LEN + 1] = (uint8_t)(crc >> 8);
    } else {
        uint16_t crc = fmav_crc_calculate(&(buf[1]), FASTMAVLINK_HEADER_V1_LEN - 1);
        fmav_crc_accumulate_buf(&crc, &buf[FASTMAVLINK_HEADER_V1_LEN], len);
        fmav_crc_accumulate(&crc, crc_extra);
        buf[len + FASTMAVLINK_HEADER_V1_LEN] = (uint8_t)crc;
        buf[len + FASTMAVLINK_HEADER_V1_LEN + 1] = (uint8_t)(crc >> 8);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_set_seq(fmav_message_t* msg, uint8_t new_seq)
{
    msg->seq = new_seq;

    uint16_t crc;
    fmav_crc_init(&crc);
    if (msg->magic == FASTMAVLINK_MAGIC_V2) {
        fmav_crc_accumulate(&crc, msg->len);
        fmav_crc_accumulate(&crc, msg->incompat_flags);
        fmav_crc_accumulate(&crc, msg->compat_flags);
        fmav_crc_accumulate(&crc, msg->seq);
        fmav_crc_accumulate(&crc, msg->sysid);
        fmav_crc_accumulate(&crc, msg->compid);
        fmav_crc_accumulate_buf(&crc, &(msg->msgid_a[0]), 3);
    } else {
        fmav_crc_accumulate(&crc, msg->len);
        fmav_crc_accumulate(&crc, msg->seq);
        fmav_crc_accumulate(&crc, msg->sysid);
        fmav_crc_accumulate(&crc, msg->compid);
        fmav_crc_accumulate(&crc, msg->msgid);
    }
    fmav_crc_accumulate_buf(&crc, msg->payload, msg->len);
    fmav_crc_accumulate(&crc, msg->crc_extra);

    msg->checksum = crc;
}


uint8_t fmav_frame_buf_reconstruct_crc_extra(uint8_t* buf)
{
    uint16_t crc, crc_msg;

    uint8_t len = buf[1];
    if (buf[0] == FASTMAVLINK_MAGIC_V2) {
        crc = fmav_crc_calculate(&(buf[1]), FASTMAVLINK_HEADER_V2_LEN - 1);
        fmav_crc_accumulate_buf(&crc, &buf[FASTMAVLINK_HEADER_V2_LEN], len);
        crc_msg = buf[len + FASTMAVLINK_HEADER_V2_LEN] + (buf[len + FASTMAVLINK_HEADER_V2_LEN + 1] << 8);

    } else {
        crc = fmav_crc_calculate(&(buf[1]), FASTMAVLINK_HEADER_V1_LEN - 1);
        fmav_crc_accumulate_buf(&crc, &buf[FASTMAVLINK_HEADER_V1_LEN], len);
        crc_msg = buf[len + FASTMAVLINK_HEADER_V1_LEN] + (buf[len + FASTMAVLINK_HEADER_V1_LEN + 1] << 8);
    }

    for (uint16_t crc_extra = 0; crc_extra < 256; crc_extra++) {
      uint16_t crc_test = crc;
      fmav_crc_accumulate(&crc_test, crc_extra);
      if (crc_test == crc_msg) return crc_extra;
    }

    return 0; //must not happen
}


uint8_t fmav_msg_reconstruct_crc_extra(fmav_message_t* msg)
{
uint16_t crc;

    fmav_crc_init(&crc);
    if (msg->magic == FASTMAVLINK_MAGIC_V2) {
        fmav_crc_accumulate(&crc, msg->len);
        fmav_crc_accumulate(&crc, msg->incompat_flags);
        fmav_crc_accumulate(&crc, msg->compat_flags);
        fmav_crc_accumulate(&crc, msg->seq);
        fmav_crc_accumulate(&crc, msg->sysid);
        fmav_crc_accumulate(&crc, msg->compid);
        fmav_crc_accumulate_buf(&crc, &(msg->msgid_a[0]), 3);
    } else {
        fmav_crc_accumulate(&crc, msg->len);
        fmav_crc_accumulate(&crc, msg->seq);
        fmav_crc_accumulate(&crc, msg->sysid);
        fmav_crc_accumulate(&crc, msg->compid);
        fmav_crc_accumulate(&crc, msg->msgid);
    }
    fmav_crc_accumulate_buf(&crc, msg->payload, msg->len);

    for (uint16_t crc_extra = 0; crc_extra < 256; crc_extra++) {
      uint16_t crc_test = crc;
      fmav_crc_accumulate(&crc_test, crc_extra);
      if (crc_test == msg->checksum) return crc_extra;
    }

    return 0; //must not happen
}


#endif // FASMAVLINK_EXTENSION_H
