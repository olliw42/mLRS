//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// CRYPTO
//*******************************************************


#define PRIVACY_LEVEL   2 // 0: nothing, 1: encryption, 2: encryption + authentication


#include <stdlib.h>
#include <string.h>
#include "crypto.h"
#include "thirdparty/monocypher/src/monocypher.h"


#define NONCE_LEN   3
#define MAC_LEN     3


void tCrypto::Init(char* bind_phrase, uint8_t tx_uid[12], uint8_t rx_uid[12])
{
    _random = 0; // 0 means session key not yet set

    memset(_key, 0, sizeof(_key));
    memset(_nonce, 0, sizeof(_nonce));
    _nonce_u32 = 0;

    memset(_static, 0, sizeof(_static));
    memcpy(_static,               "mLRS key",    8); //  8 bytes
    memcpy(_static + 8,           bind_phrase,   6); //  6 bytes
    memcpy(_static + 8 + 6,       tx_uid,       12); // 12 bytes
    memcpy(_static + 8 + 6 + 12,  rx_uid,       12); // 12 bytes // sum 38 bytes

    crypto_blake2b(_static_key, 32, _static, 38);
    memcpy(_key, _static_key, 32);
}


void tCrypto::SetSessionKey(uint64_t random)
{
uint8_t key_source[64]; // 38 + 8 = 46

    if (_random != 0) return; // has been set already

    _random = random;

    memcpy(key_source, _static,      38); // 38 bytes
    memcpy(key_source + 38, &_random, 8); // 8 bytes // sum = 46 bytes

    crypto_blake2b(_key, 32, key_source, 46);
}


void tCrypto::GetEncryptedRandom(uint8_t random[16])
{
uint8_t nonce_buf[12];
uint8_t poly1305_key[32];
uint8_t mac[16];

    if (_random == 0) while(1){} // must have been set before, must not happen

    memset(nonce_buf, 0, 12);
    memcpy(nonce_buf, &_static_nonce_u32, 4);
    _static_nonce_u32++; // ready it for next use

    crypto_chacha20_ietf(random, (uint8_t*)&_random, 8, _static_key, nonce_buf, 1234567);

    memcpy(random + 8, nonce_buf, 4); // random[8] ... random[11]

    crypto_chacha20_ietf(poly1305_key, NULL, 32, _static_key, nonce_buf, 1234568);
    crypto_poly1305(mac, random, 12, poly1305_key);

    memcpy(random + 12, mac, 4); // random[12] ... random[15]
}


void tCrypto::SetSessionKeyFromEncryptedRandom(uint8_t random[16])
{
uint8_t nonce_buf[12];
uint8_t poly1305_key[32];
uint8_t mac[16];
uint64_t rand;

    memset(nonce_buf, 0, 12);
    memcpy(nonce_buf, random + 8, 4); // random[8] ... random[11]

    crypto_chacha20_ietf(poly1305_key, NULL, 32, _static_key, nonce_buf, 1234568);
    crypto_poly1305(mac, random, 12, poly1305_key);
    for (uint8_t i = 0; i < 4; i++) if (random[12 + i] != mac[i]) return; // fail

    crypto_chacha20_ietf((uint8_t*)&rand, random, 8, _static_key, nonce_buf, 1234567);

    SetSessionKey(rand);
}


uint16_t tCrypto::NonceLen(void)
{
#if PRIVACY_LEVEL == 1
    return NONCE_LEN;
#elif PRIVACY_LEVEL == 2
    return MAC_LEN + NONCE_LEN;
#endif
    return 0;
}


void tCrypto::Encrypt(uint8_t* const payload, uint8_t* len)
{
#if PRIVACY_LEVEL == 1
    _encrypt(payload, len);
#elif PRIVACY_LEVEL == 2
    _encrypt_w_auth(payload, len);
#endif
}


void tCrypto::Decrypt(uint8_t* const payload, uint8_t* len)
{
#if PRIVACY_LEVEL == 1
    _decrypt(payload, len);
#elif PRIVACY_LEVEL == 2
    _decrypt_w_auth(payload, len);
#endif
}


void tCrypto::_encrypt(uint8_t* const payload, uint8_t* len)
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


void tCrypto::_decrypt(uint8_t* const payload, uint8_t* len)
{
    if (*len < NONCE_LEN) {
        *len = 0; // TODO: what should we do ?
        return;
    }

    _nonce[0] = payload[0];
    _nonce[1] = payload[1];
    _nonce[2] = payload[2];

    *len -= NONCE_LEN;

    memmove(payload, payload + NONCE_LEN, *len);

    _crypt_it(payload, *len);
}


void tCrypto::_encrypt_w_auth(uint8_t* const payload, uint8_t* len)
{
uint8_t mac[16];

    _nonce_u32++;

    _nonce[0] = (_nonce_u32 & 0x00FF0000) >> 16;
    _nonce[1] = (_nonce_u32 & 0x0000FF00) >> 8;
    _nonce[2] = (_nonce_u32 & 0x000000FF);

    // encrypt data at payload
    _crypt_it(payload, *len);

    // MAC = poly1305(nonce || ciphertext)
    _mac_it(mac, payload, *len);

    // move data to payload + 6
    memmove(payload + MAC_LEN + NONCE_LEN, payload, *len); // NOT memcpy(), needs to copy from end towards beginning !!

    // correct len for the nonce
    *len += MAC_LEN + NONCE_LEN;

    // copy mac into payload
    payload[0] = mac[0];
    payload[1] = mac[1];
    payload[2] = mac[2];

    // copy nonce into payload
    payload[3] = _nonce[0];
    payload[4] = _nonce[1];
    payload[5] = _nonce[2];
}


void tCrypto::_decrypt_w_auth(uint8_t* const payload, uint8_t* len)
{
uint8_t received_mac[MAC_LEN];
uint8_t mac[16];

    if (*len < MAC_LEN + NONCE_LEN) {
        *len = 0; // TODO: what should we do ?
        return;
    }

    received_mac[0] = payload[0];
    received_mac[1] = payload[1];
    received_mac[2] = payload[2];

    _nonce[0] = payload[3];
    _nonce[1] = payload[4];
    _nonce[2] = payload[5];

    *len -= MAC_LEN + NONCE_LEN;

    memmove(payload, payload + MAC_LEN + NONCE_LEN, *len);

    // calculate MAC over nonce + payload
    _mac_it(mac, payload, *len);

    // comparison of 3-byte mac
    bool ok = true;
    for (uint8_t i = 0; i < MAC_LEN; i++) if (mac[i] != received_mac[i]) ok = false;

    if (!ok) { // authentication failed
        *len = 0; // pretend we didn't got data at all // TODO: what should we do ?
        return;
    }

    _crypt_it(payload, *len);
}


void tCrypto::_crypt_it(uint8_t* payload, uint16_t len)
{
// Note: the counter does not have to start at 0, one just needs to use
// different counter for each block, so always starting with 1 is fine

    crypto_chacha20_ietf(
        payload,      // cipher_text,
        payload,      // plain_text, same as cipher = in-place encoding
        len,          // text_size,
        _key,         // key[32],
        _nonce,       // nonce[12],
        1);           // ctr
}


void tCrypto::_mac_it(uint8_t mac[16], uint8_t* const payload, uint16_t len)
{
uint8_t poly1305_key[32];
crypto_poly1305_ctx ctx;

// Note: the ChaCha20 keystream of the first block is used as key for poly1305 (which wants 32 bytes key)
// so, we use counter = 0
// it is important that for the data then a different counter is used, so use counter = 1 there

    crypto_chacha20_ietf(
        poly1305_key, // cipher_text,
        NULL,         // plain_text, NULL = returns ChaCha20 keystream
        32,           // text_size,
        _key,         // key[32],
        _nonce,       // nonce[12],
        0);           // ctr

    crypto_poly1305_init(&ctx, poly1305_key);
    crypto_poly1305_update(&ctx, _nonce, NONCE_LEN);
    crypto_poly1305_update(&ctx, payload, len);
    crypto_poly1305_final(&ctx, mac);
}

