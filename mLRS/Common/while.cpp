//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// While Transmit/Receive
//*******************************************************


#include <stdint.h>
#include "while.h"


void WhileBase::Init(void)
{
    do_cnt = 0;
    tasks = WHILE_TASK_NONE;
}


void WhileBase::Trigger(void)
{
    do_cnt = 5; // postpone the action by few loops
}


void WhileBase::Do(void)
{
    if (!do_cnt) return; // 0 = not triggered -> jump out
    do_cnt--; // count down
    if (do_cnt) return; // !0 = we still postpone -> jump out

    if (!tasks) return; // no task to do -> jump out
    handle_tasks();
}


void WhileBase::SetTask(uint16_t task)
{
    tasks |= task;
}


