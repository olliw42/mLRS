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

#define LIST_SIZE  16

//template <int LIST_SIZE>
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
        new_added = false;
    }

    uint8_t FindComp(uint8_t sys_id, uint8_t comp_id)
    {
        for (uint8_t i = 0; i < LIST_SIZE; i++) {
            if (buf[i].empty) continue;
            if (buf[i].sys_id == sys_id && buf[i].comp_id == comp_id) return i;
        }
        return UINT8_MAX;
    }

    uint8_t AddComp(uint8_t sys_id, uint8_t comp_id, uint32_t tnow_ms)
    {
        for (uint8_t i = 0; i < LIST_SIZE; i++) {
            if (!buf[i].empty) continue;
            buf[i].sys_id = sys_id;
            buf[i].comp_id = comp_id;
            buf[i].empty = false;
            new_added = true;
            return i;
        }
        return UINT8_MAX;
    }

    // this ensures, possibly the hard way, that a valid index is returned
    uint8_t FindAndAdd(uint8_t sys_id, uint8_t comp_id, uint32_t tnow_ms)
    {
        uint8_t i = FindComp(sys_id, comp_id);

        if (i != UINT8_MAX) { // is in list, so return the index
            buf[i].tlast_ms = tnow_ms;
            return i;
        }

        // not in list, so add it
        i = AddComp(sys_id, comp_id, tnow_ms);

        if (i != UINT8_MAX) { // is now in list, so return the index
            buf[i].tlast_ms = tnow_ms;
            return i;
        }

        // list is full, so replace last element
        i = LIST_SIZE - 1;
        buf[i].sys_id = sys_id;
        buf[i].comp_id = comp_id;
        buf[i].tlast_ms = tnow_ms;
        buf[i].empty = true;
        new_added = true;
        return i;
    }

    typedef struct {
        bool empty;
        uint8_t sys_id;
        uint8_t comp_id;
        uint32_t tlast_ms;
    } tListElement;

    tListElement buf[LIST_SIZE];

    bool new_added;
};


#endif // FASTMAVLINK_EXTENSION_H
