//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// CRYPTO
//*******************************************************
// Based on Monocypher, https://github.com/LoupVaillant/monocypher
/*
SecretKey handling:
- on bind, a secret key root is exchanged, which is based bind phrase, tx uid, rx uid
- on first connection, a session secret key is generated, which is based on
  secret key root plus some randomized data (e.g. from a trng, or startup ms counter)
- the nonce is build, which is based on a 3 bytes partial nonce + header data, such as
  seq_no : 3, rssi_u7 : 7, LQ_serial : 7
*/
//*******************************************************
#ifndef CRYPTO_H
#define CRYPTO_H
#pragma once


#include <inttypes.h>


#define NONCE_LEN   3


class tCrypto
{
  public:
    void Init(char* bind_phrase, uint8_t tx_uid[12], uint8_t rx_uid[12]);

    void SetSessionKey(uint64_t random);
    void SetSessionKey(uint8_t random[8]) { uint64_t r; memcpy(&r, random, 8); SetSessionKey(r); }

    void Disconnected(void) { _random = 0; }

    void Encrypt(uint8_t* const payload, uint8_t* len);
    void Decrypt(uint8_t* const payload, uint8_t* len);

    uint64_t Random(void) { return _random; }

  private:
    uint8_t _static_key[64];
    uint64_t _random;

    uint8_t _key[32];
    uint8_t _nonce[12];
    uint32_t _nonce_u32;

    void _crypt_it(uint8_t* payload, uint16_t len);
};


#endif // CRYPTO_H
