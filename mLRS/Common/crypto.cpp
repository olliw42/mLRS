//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// CRYPTO
//*******************************************************

#include <stdlib.h>
#include <string.h>
#include "crypto.h"
#include "thirdparty/monocypher/src/monocypher.h"



void tCrypto::Init(void)
{
}


void tCrypto::SetKey(char* bind_phrase, uint8_t tx_uid[12], uint8_t rx_uid[12], uint64_t random)
{
    uint8_t key_source[64]; // 16 + 6 + 12 + 12 + 8 = 54

    memcpy(key_source,                    "mLRS key @!%&$?&", 16);  // 16 bytes
    memcpy(key_source + 16,               bind_phrase,         6);  //  6 bytes
    memcpy(key_source + 16 + 6,           tx_uid,             12);  // 12 bytes
    memcpy(key_source + 16 + 6 + 12,      rx_uid,             12);  // 12 bytes
    memcpy(key_source + 16 + 6 + 12 + 12, &random, sizeof(random)); //  8 bytes

    crypto_blake2b(_key, 32, key_source, 54);
}


void tCrypto::Encrypt(uint8_t* const payload, uint16_t len)
{
    uint32_t counter = 0;

    crypto_chacha20_ietf(
        payload,    // cipher_text,
        NULL,       // plain_text, in-place encoding
        len,        // text_size,
        _key,       // key[32],
        _nonce,     // nonce[12],
        counter);   // ctr
}


void tCrypto::Decrypt(uint8_t* const payload, uint16_t len)
{
    uint32_t counter = 0;

    crypto_chacha20_ietf(
        payload,    // cipher_text,
        NULL,       // plain_text, in-place encoding
        len,        // text_size,
        _key,       // key[32],
        _nonce,     // nonce[12],
        counter);   // ctr
}

