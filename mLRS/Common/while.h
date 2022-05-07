//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// While Transmit/Receive
//*******************************************************
#ifndef WHILE_H
#define WHILE_H
#pragma once


//-------------------------------------------------------
// While transmit/receive tasks
//-------------------------------------------------------

class WhileBase
{
  public:
    void Init(void);
    void Trigger(void);
    void Do(void);

    virtual void handle_once(void) {};
    virtual void handle(void) {};

    virtual uint16_t tnow_us(void);
    virtual int32_t dtmax_us(void) { return 1; }

    uint16_t do_cnt;
    uint16_t tstart_us;
    int32_t tremaining_us;
};


#endif // WHILE_H
