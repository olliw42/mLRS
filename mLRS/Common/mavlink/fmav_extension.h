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


#endif // FASTMAVLINK_EXTENSION_H
