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


void tCrypto::SetKey(uint8_t tx_uid[12], uint8_t rx_uid[12])
{
}


void tCrypto::Encrypt(uint8_t* const payload, uint16_t len)
{
    uint32_t counter = 0;

    memcpy(_buf, payload, len);

    crypto_chacha20_ietf(
        payload, // cipher_text,
        _buf, // plain_text,
        len, // text_size,
        _key, // key[32],
        _nonce, // nonce[12],
        counter); // ctr
}


void tCrypto::Decrypt(uint8_t* const payload, uint16_t len)
{
    uint32_t counter = 0;

    memcpy(_buf, payload, len);

    crypto_chacha20_ietf(
        payload, // cipher_text,
        _buf, // plain_text,
        len, // text_size,
        _key, // key[32],
        _nonce, // nonce[12],
        counter); // ctr
}

