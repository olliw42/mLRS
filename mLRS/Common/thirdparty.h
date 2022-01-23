//*******************************************************
// This holds code taken from other open source projects
// For the licences, please look up in the original
// project repositories.
//********************************************************
#ifndef THIRDARTY_H
#define THIRDARTY_H
#pragma once


#include <stdint.h>


// This code is from betaflight
// https://github.com/betaflight/betaflight/blob/master/src/main/common/crc.c

uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly);
uint8_t crc8_update(uint8_t crc, const void *data, uint32_t length, uint8_t poly);



#endif // THIRDARTY_H
