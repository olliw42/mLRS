//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Stack check
//*******************************************************
#ifndef ESPLIB_STACK_H
#define ESPLIB_STACK_H
#ifdef __cplusplus
extern "C" {
#endif


#include <inttypes.h>


//-------------------------------------------------------
//
//-------------------------------------------------------


uint32_t stack_check_used(void)
{
    return 1;
}


void stack_check_init(void)
{
}


//-------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif // ESPLIB_STACK_H
