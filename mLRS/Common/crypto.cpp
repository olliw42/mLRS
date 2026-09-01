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


#define NONCE_LEN       3
#define MAC_LEN         3
#define LVL3_NONCE_LEN  4
#define LVL3_MAC_LEN    8


typedef struct {
    uint8_t nonce_len;
    uint8_t mac_len;
} crypto_level_t;


const crypto_level_t crypto_list[] = {
    { .nonce_len = 0,               .mac_len = 0            }, // nothing
    { .nonce_len = NONCE_LEN,       .mac_len = 0            }, // level 1: only encryption
    { .nonce_len = NONCE_LEN,       .mac_len = MAC_LEN      }, // level 2: encryption + authentication
    { .nonce_len = LVL3_NONCE_LEN,  .mac_len = LVL3_MAC_LEN }, // level 3: stronger encryption + authentication
};


#define PRIVACY_LEVEL_NUM  (sizeof(crypto_list)/sizeof(crypto_level_t))


//-------------------------------------------------------
// Crypto API
//-------------------------------------------------------

void tCrypto::Init(char* const bind_phrase, uint8_t tx_uid[12], uint8_t rx_uid[12], uint64_t tx_random)
{
    _privacy_level = 0;

    memset(_static, 0, sizeof(_static));
    memcpy(_static,                   "mLRS key",    8); //  8 bytes
    memcpy(_static + 8,               bind_phrase,   6); //  6 bytes
    memcpy(_static + 8 + 6,           tx_uid,       12); // 12 bytes
    memcpy(_static + 8 + 6 + 12,      rx_uid,       12); // 12 bytes
    memcpy(_static + 8 + 6 + 12 +12,  &tx_random,    8); //  8 bytes // sum 46 bytes

    // is reused on each Tx power cycle, if there is concern, re-bind. TODO: should we set it randomly?
    _static_nonce_u32 = 0;

    _random = 0;
    _random_valid = false; // session key not yet set

    memset(_key, 0, sizeof(_key));
    memset(_nonce, 0, sizeof(_nonce));
    _nonce_len = 0;
    _nonce_u32 = 0;

    _nonce_u32_last_received = 0;

    // construct static key
    crypto_blake2b(_static_key, 32, _static, 46);

    // set key to static key to have some default
    memcpy(_key, _static_key, 32);
}


void tCrypto::SetPrivacyLevel(uint8_t privacy_level)
{
    if (privacy_level >= PRIVACY_LEVEL_NUM) return;

    _privacy_level = privacy_level;
}


//-- handle session random and session key
// The session random is transmitted encrypted, in the following format:
//  0 ..  7: 8 bytes random
//  8 .. 11: 4 bytes nonce, starts with 0
// 12 .. 15: 4 bytes mac

// Tx: called in init sequence
// Rx: called by SetSessionKeyFromEncryptedRandom() when a FRAME_CMD_GET_RX_SETUPDATA frame is received
void tCrypto::SetSessionKey(uint64_t random)
{
uint8_t key_source[64]; // 46 + 8 = 54

    if (random == 0 || random == UINT64_MAX) return; // don't accept these, should not happen TODO: what to do if it does?

    _random = random;
    _random_valid = true;

    memcpy(key_source,      _static,  46); // 46 bytes
    memcpy(key_source + 46, &_random,  8); //  8 bytes // sum = 54 bytes

    crypto_blake2b(_key, 32, key_source, 54);
}


// only Tx: send along with a FRAME_CMD_GET_RX_SETUPDATA frame
void tCrypto::GetEncryptedRandom(uint8_t random[16])
{
uint8_t nonce_buf[12];
uint8_t poly1305_key[32];
uint8_t mac[16];

    if (!_random_valid) while(1){} // must not happen, must have been set before, just to ensure proper code flow

    memset(nonce_buf, 0, 12);
    memcpy(nonce_buf, &_static_nonce_u32, 4);

    _static_nonce_u32++; // ready it for next use

    crypto_chacha20_ietf(random, (uint8_t*)&_random, 8, _static_key, nonce_buf, 1);

    memcpy(random + 8, nonce_buf, 4); // random[8] ... random[11]

    crypto_chacha20_ietf(poly1305_key, NULL, 32, _static_key, nonce_buf, 0);
    crypto_poly1305(mac, random, 12, poly1305_key);

    memcpy(random + 12, mac, 4); // random[12] ... random[15]
}


// only Rx: called upon receive of a FRAME_CMD_GET_RX_SETUPDATA frame
void tCrypto::SetSessionKeyFromEncryptedRandom(uint8_t random[16])
{
uint8_t nonce_buf[12];
uint8_t poly1305_key[32];
uint8_t mac[16];
uint64_t rand;

    if (_random_valid) return; // has been set already

    memset(nonce_buf, 0, 12);
    memcpy(nonce_buf, random + 8, 4); // random[8] ... random[11]

    crypto_chacha20_ietf(poly1305_key, NULL, 32, _static_key, nonce_buf, 0);
    crypto_poly1305(mac, random, 12, poly1305_key);
    for (uint8_t i = 0; i < 4; i++) { if (random[12 + i] != mac[i]) return; } // authentication failed

    crypto_chacha20_ietf((uint8_t*)&rand, random, 8, _static_key, nonce_buf, 1);

    SetSessionKey(rand);
}


// only Rx: called when receiver is disconnected
void tCrypto::Disconnected(void)
{
    // TODO: this needs carefully thinking through.
    // one needs to consider differences between re-powered, reconnected
    // currently: for privacy level >= 2, session key stays always persistent

    if (_privacy_level <= 1) { // accept potentially new session random/session key
        _random_valid = false;
    }
}


//-- API miscellaneous

uint16_t tCrypto::NonceLen(void)
{
    if (!_privacy_level) return 0; // no encryption

    return crypto_list[_privacy_level].nonce_len + crypto_list[_privacy_level].mac_len;
}


void tCrypto::Encrypt(uint8_t* const data, uint8_t len, uint8_t* payload_len)
{
    if (!_privacy_level) return; // no encryption

    _encrypt_it(data, len, payload_len);
}


void tCrypto::Decrypt(uint8_t* const data, uint8_t len, uint8_t* payload_len)
{
    if (!_privacy_level) return; // no encryption

    _decrypt_it(data, len, payload_len);
}


//-------------------------------------------------------
// Encryption handlers
//-------------------------------------------------------

// The data is transmitted encrypted, in the following format:
//   3/4 bytes nonce
//   0/3/8 bytes mac
//   data

void tCrypto::_encrypt_it(uint8_t* const data, uint8_t len, uint8_t* payload_len)
{
uint8_t mac[16];
uint8_t mac_len = crypto_list[_privacy_level].mac_len;

    // update nonce
    _nonce_u32++;
    _nonce_len = crypto_list[_privacy_level].nonce_len;
    memcpy(_nonce, &_nonce_u32, _nonce_len); // _nonce[0] ... _nonce[nonce_len-1] = _nonce_u32

    // encrypt data at data[0]
    _crypt_it(data, len);

    if (mac_len) {
        // MAC = poly1305(nonce || ciphertext)
        _mac_it(mac, data, len);
    }

    // move data to payload + mac_len + nonce_len
    memmove(data + mac_len + _nonce_len, data, len); // NOT memcpy(), needs to copy from end towards beginning !!

    // correct payload len for the mac and nonce
    *payload_len += mac_len + _nonce_len;

    // copy mac into data
    memcpy(data, mac, mac_len); // data[0] ... data[mac_len-1]

    // copy nonce into data
    memcpy(data + mac_len, _nonce, _nonce_len); // data[mac_len] ... data[mac_len+nonce_len-1]
}


void tCrypto::_decrypt_it(uint8_t* const data, uint8_t len, uint8_t* payload_len)
{
uint8_t received_mac[LVL3_MAC_LEN];
uint32_t received_nonce_u32;
uint8_t mac[16];
uint8_t mac_len = crypto_list[_privacy_level].mac_len;

    _nonce_len = crypto_list[_privacy_level].nonce_len;

    if (len < mac_len + _nonce_len) {
        *payload_len = 0; // TODO: what should we do ?
        return;
    }

    // get mac
    memcpy(received_mac, data, mac_len); // data[0] ... data[mac_len-1]

    // get nonce
    memcpy(_nonce, data + mac_len, _nonce_len); // data[mac_len] ... data[mac_len+nonce_len-1]

    // correct len, payload_len for the mac and nonce
    *payload_len -= mac_len + _nonce_len;
    len -= mac_len + _nonce_len;

    // move data to data[0]
    memmove(data, data + mac_len + _nonce_len, len); // NOT memcpy(), needs to copy from beginning towards end !!

    if (mac_len) {
        // calculate MAC over nonce + payload
        _mac_it(mac, data, len);

        // comparison of mac_len byte mac
        bool ok = true;
        for (uint8_t i = 0; i < mac_len; i++) { if (mac[i] != received_mac[i]) ok = false; }

        if (!ok) { // authentication failed
            *payload_len = 0; // pretend we didn't got data at all // TODO: what should we do ?
            return;
        }
    }

    // check nonce, don't accept previously seen nonces, to prevent replay attacks
    // do only for privacy levels > 1
    // TODO: what needs to be done upon connection loss? does it play well with ARQ?
    memcpy(&received_nonce_u32, _nonce, _nonce_len); // _nonce_u32 = _nonce[0] ... _nonce[nonce_len-1]
    if (_privacy_level >= 2 && received_nonce_u32 <= _nonce_u32_last_received) {
    //    *len = 0;
    //    return;
    }
    _nonce_u32_last_received = received_nonce_u32;

    // decrypt data at data[0]
    _crypt_it(data, len);
}


//-------------------------------------------------------
// Monocypher interface
//-------------------------------------------------------

void tCrypto::_crypt_it(uint8_t* data, uint16_t len)
{
// Note: the counter does not have to start at 0, one just needs to use
// different counter for each block, so always starting with 1 is fine

    crypto_chacha20_ietf(
        data,     // cipher_text,
        data,     // plain_text, same as cipher = in-place encoding
        len,      // text_size,
        _key,     // key[32],
        _nonce,   // nonce[12],
        1);       // ctr
}


void tCrypto::_mac_it(uint8_t mac[16], uint8_t* const data, uint16_t len)
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
    crypto_poly1305_update(&ctx, _nonce, _nonce_len);
    crypto_poly1305_update(&ctx, data, len);
    crypto_poly1305_final(&ctx, mac);
}

