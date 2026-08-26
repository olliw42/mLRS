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
    memset(_key, 0, sizeof(_key));
    memset(_nonce, 0, sizeof(_nonce));
    _nonce_u32 = 0;
}


void tCrypto::SetKey(char* bind_phrase, uint8_t tx_uid[12], uint8_t rx_uid[12])
{
    uint8_t key_source[64]; // 8 + 6 + 12 + 12 + 8 = 46

    memcpy(key_source,                "mLRS key",   8);  //  8 bytes
    memcpy(key_source + 8,            bind_phrase,  6);  //  6 bytes
    memcpy(key_source + 8 + 6,        tx_uid,       12); // 12 bytes
    memcpy(key_source + 8 + 6 + 12,   rx_uid,       12); // 12 bytes // sum = 38 bytes

    crypto_blake2b(_key, 32, key_source, 38);
}


void tCrypto::Encrypt(uint8_t* const payload, uint8_t* len)
{
    _nonce_u32++;

    _nonce[0] = (_nonce_u32 & 0x00FF0000) >> 16;
    _nonce[1] = (_nonce_u32 & 0x0000FF00) >> 8;
    _nonce[2] = (_nonce_u32 & 0x000000FF);

    // encrypt data at payload
    _crypt_it(payload, *len);

    // move data to payload + 3
    memmove(payload + NONCE_LEN, payload, *len); // NOT memcpy(), needs to copy from end towards beginning !!

    // correct len for the nonce
    *len += NONCE_LEN;

    // copy nonce into payload
    payload[0] = _nonce[0];
    payload[1] = _nonce[1];
    payload[2] = _nonce[2];
}


void tCrypto::Decrypt(uint8_t* const payload, uint8_t* len)
{
    _nonce[0] = payload[0];
    _nonce[1] = payload[1];
    _nonce[2] = payload[2];

    *len -= NONCE_LEN;

    memmove(payload, payload + NONCE_LEN, *len);

    _crypt_it(payload, *len);
}


void tCrypto::_crypt_it(uint8_t* payload, uint16_t len)
{
    uint32_t counter = 0;

    while (len > 0) {
        uint16_t chunk_len = (len > 64) ? 64 : len;

        crypto_chacha20_ietf(
            payload,      // cipher_text,
            payload,      // plain_text, same as cipher = in-place encoding
            chunk_len,    // text_size,
            _key,         // key[32],
            _nonce,       // nonce[12],
            counter);     // ctr

        payload += chunk_len;
        len -= chunk_len;
        counter++;
    }
}

