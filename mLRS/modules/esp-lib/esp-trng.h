//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP TRNG
//********************************************************
#ifndef ESPLIB_TRNG_H
#define ESPLIB_MCU_H


void trng_init(void)
{
}


uint32_t trng_get32(void)
{
    return UINT32_MAX; // 0xFFFFFFFF;
}


#endif // ESPLIB_TRNG_H
