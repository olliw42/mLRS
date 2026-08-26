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
#include <string.h>


#define ARRAY_LEN(x)  sizeof(x)/sizeof(x[0])

#ifndef PACKED
  #define PACKED(__Declaration__)  __Declaration__ __attribute__((packed)) // that's for __GNUC__
#endif


#define NONCE_LEN   3


class tCrypto
{
  public:
    void Init(void);

    void SetKey(char* bind_phrase, uint8_t tx_uid[12], uint8_t rx_uid[12]);

    void Encrypt(uint8_t* const payload, uint8_t* len);
    void Decrypt(uint8_t* const payload, uint8_t* len);

  private:
    uint8_t _key[32];
    uint8_t _nonce[12];
    uint32_t _nonce_u32;

    void _crypt_it(uint8_t* payload, uint16_t len);
};






#endif // CRYPTO_H
