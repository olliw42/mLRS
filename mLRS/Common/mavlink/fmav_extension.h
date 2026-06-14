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


//-------------------------------------------------------
// helper
//-------------------------------------------------------

void fmav_msg_recalculate_crc(fmav_message_t* const msg)
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


#endif // FASTMAVLINK_EXTENSION_H
