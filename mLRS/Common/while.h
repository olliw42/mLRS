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
// we may want to add some timer to do more than one task in the transmit/receive period
// this would help a lot with the different available periods depending on the mode

class WhileBase
{
  public:
    void Init(void);
    void Trigger(void);
    void Do(void);
    void SetTask(uint16_t task);
    virtual void handle(void) {};
    virtual void handle_tasks(void) {};

    uint8_t tasks;
    uint16_t do_cnt;
};


class WhileTransmit : public WhileBase
{
  public:
    void handle(void) override;
    void handle_tasks(void) override;
};


#endif // WHILE_H
