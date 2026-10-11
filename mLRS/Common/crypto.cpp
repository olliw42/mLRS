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


#if LVL3_NONCE_LEN + LVL3_MAC_LEN != CRYPTO_NONCE_MAX_LEN
#error CRYPTO_NONCE_MAX_LEN incompatible with nonce and MAC defines!
#endif


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

void tCrypto::Init(
    uint8_t role,
    char* const bind_phrase, uint8_t tx_uid[12], uint8_t rx_uid[12], uint64_t tx_random,
    uint8_t privacy_level)
{
    _role = role;

    _privacy_level = 0;
    if (privacy_level < PRIVACY_LEVEL_NUM) _privacy_level = privacy_level;

    _nonce_len = crypto_list[_privacy_level].nonce_len;
    _mac_len = crypto_list[_privacy_level].mac_len;

    // static secrets

    _static_random = tx_random;

    memset(_static_source, 0, sizeof(_static_source));
    memcpy(_static_source,                   "mLRS key",    8); //  8 bytes
    memcpy(_static_source + 8,               bind_phrase,   6); //  6 bytes
    memcpy(_static_source + 8 + 6,           tx_uid,       12); // 12 bytes
    memcpy(_static_source + 8 + 6 + 12,      rx_uid,       12); // 12 bytes
    memcpy(_static_source + 8 + 6 + 12 +12,  &tx_random,    8); //  8 bytes // sum 46 bytes

    crypto_blake2b(_static_key, 32, _static_source, 46); // construct static key

    _startup_nonce_u32 = 0;

    // session secrets

    _session_random = 0;
    _session_key_has_been_set = false; // session key not yet set

    memset(_session_key, 0, sizeof(_session_key));
    memcpy(_session_key, _static_key, 32); // set key to static key to have some default
    _nonce_u32 = 0;

    // auxiliary

    _nonce_u32_last_received = 0;

    // statistics
    mac_errors = 0;
    replay_counts = 0;
}


bool tCrypto::ValidKeys(void) // to tell Tx or Rx if they can connect
{
//    return (_privacy_level == 0 ||(_static_random != UINT64_MAX && _session_random != UINT64_MAX));
    return !(_privacy_level > 0 && (_static_random == UINT64_MAX || _session_random == UINT64_MAX));
}


//-- handle session random and session key

// Tx: called in init sequence
// Rx: called by SetSessionKeyFromEncryptedRandom() when a FRAME_CMD_GET_RX_SETUPDATA_STARTUP frame is received
void tCrypto::SetSessionKey(uint64_t session_random)
{
uint8_t key_source[64]; // 46 + 8 = 54

    _session_random = session_random;

    memcpy(key_source,      _static_source,   46); // 46 bytes
    memcpy(key_source + 46, &_session_random,  8); //  8 bytes // sum = 54 bytes

    crypto_blake2b(_session_key, 32, key_source, 54);

    _session_key_has_been_set = true;
}

// The session random is transmitted encrypted, in the following format:
//  0 ..  7:  8 bytes session-random
//  8 .. 19: 12 bytes startup-random | bind-random | startup-nonce
// 20 .. 27:  8 bytes mac

// only Tx: send along with a FRAME_CMD_GET_RX_SETUPDATA_STARTUP frame
void tCrypto::EncryptSessionRandom(uint8_t* const buf28, uint64_t startup_random, uint64_t bind_random)
{
uint8_t nonce[12];
uint8_t poly1305_key[32];
uint8_t mac[16];

    if (!_session_key_has_been_set) while(1){} // must not happen, SetSessionKey() must be called before

    bind_random += _startup_nonce_u32;
    _startup_nonce_u32++; // ready it for next use

    memset(nonce, 0, 12);
    memcpy(nonce, &startup_random, 8);
    memcpy(nonce + 8, &bind_random, 4);

    crypto_chacha20_ietf(buf28, (uint8_t*)&_session_random, 8, _static_key, nonce, 1); // random[0] ... random[8 - 1]

    memcpy(buf28 + 8, nonce, 12); // random[8] ... random[20 - 1]

    crypto_chacha20_ietf(poly1305_key, NULL, 32, _static_key, nonce, 0);
    crypto_poly1305(mac, buf28, 20, poly1305_key); // mac over session random & nonce

    memcpy(buf28 + 20, mac, 8); // random[20] ... random[28 - 1]
}


// only Rx: called upon receive of a FRAME_CMD_GET_RX_SETUPDATA frame
void tCrypto::SetSessionKeyFromEncryptedRandomBuf(uint8_t* const buf28)
{
uint8_t nonce[12];
uint8_t poly1305_key[32];
uint8_t mac[16];
uint64_t session_random;

    if (_session_key_has_been_set) return; // has already been set

    memset(nonce, 0, 12);
    memcpy(nonce, buf28 + 8, 12); // random[8] ... random[20 -1]

    crypto_chacha20_ietf(poly1305_key, NULL, 32, _static_key, nonce, 0);
    crypto_poly1305(mac, buf28, 20, poly1305_key); // mac over session random & nonce
    for (uint8_t i = 0; i < 8; i++) { if (buf28[20 + i] != mac[i]) return; } // authentication failed

    crypto_chacha20_ietf((uint8_t*)&session_random, buf28, 8, _static_key, nonce, 1);

    SetSessionKey(session_random);
}


//-- API miscellaneous

uint16_t tCrypto::NonceLen(void)
{
    if (!_privacy_level) return 0; // no encryption

    return _nonce_len + _mac_len;
}


void tCrypto::Encrypt(void* const header, uint8_t header_len, void* const data, uint8_t len)
{
    if (!_privacy_level) return; // no encryption

    _encrypt_it((uint8_t*)header, header_len, (uint8_t*)data, len);
}


void tCrypto::RecalculateMac(void* const header, uint8_t header_len, void* const data, uint8_t len)
{
    if (!_privacy_level) return; // no encryption

    _remac_it((uint8_t*)header, header_len, (uint8_t*)data, len);
}


bool tCrypto::Decrypt(void* const header, uint8_t header_len, void* const data, uint8_t len)
{
    if (!_privacy_level) return true; // no encryption

    return _decrypt_it((uint8_t*)header, header_len, (uint8_t*)data, len);
}


//-------------------------------------------------------
// Encryption handlers
//-------------------------------------------------------

// The data is transmitted encrypted, in the following format:
//   0/3/8 bytes mac
//   3/4 bytes nonce
//   data

// Note: The outside code must ensure that payload_len is adjusted correct,
// so that payload_len + nonce_len + mac_len never exceeds the size of the payload buffer.
void tCrypto::_encrypt_it(uint8_t* const header, uint8_t header_len, uint8_t* const data, uint8_t len)
{
uint8_t nonce[12];
uint8_t mac[16];

    // update nonce
    _nonce_u32++;

    // create 12-byte nonce
    memset(nonce, 0, 12);
    memcpy(nonce, &_nonce_u32, _nonce_len); // _nonce[0] ... _nonce[nonce_len - 1] = _nonce_u32

    // fake the nonce for role
    nonce[11] = (_role == RX) ? 0xAA : 0x55;

    // encrypt data at data[0]
    _crypt_it(data, len, nonce);

    if (_mac_len) {
        // MAC = poly1305(header || nonce || ciphertext)
        _mac_it(mac, header, header_len, data, len, nonce, _nonce_len);
    }

    // move data to data + mac_len + nonce_len
    memmove(data + _mac_len + _nonce_len, data, len); // NOT memcpy(), needs to copy from end towards beginning !!

    // copy mac into data
    memcpy(data, mac, _mac_len); // data[0] ... data[mac_len - 1]

    // copy nonce into data
    memcpy(data + _mac_len, nonce, _nonce_len); // data[mac_len] ... data[mac_len + nonce_len - 1]
}


void tCrypto::_remac_it(uint8_t* const header, uint8_t header_len, uint8_t* const data, uint8_t len)
{
uint8_t nonce[12];
uint8_t mac[16];

    if (!_mac_len) return; // nothing to do

    if (len < _mac_len + _nonce_len) return; // should not happen, note: len is here NOT the payload len!

    // get nonce from data
    memset(nonce, 0, 12);
    memcpy(nonce, data + _mac_len, _nonce_len);

    // fake the nonce for role
    nonce[11] = (_role == RX) ? 0xAA : 0x55;

    // MAC the existing encrypted data, not plain palyoad
    _mac_it(mac, header, header_len, data + _mac_len + _nonce_len, len - _mac_len - _nonce_len, nonce, _nonce_len);

    // copy new mac into data; leave nonce and ciphertext untouched
    memcpy(data, mac, _mac_len); // data[0] ... data[mac_len - 1]
}


bool tCrypto::_decrypt_it(uint8_t* const header, uint8_t header_len, uint8_t* const data, uint8_t len)
{
uint8_t received_mac[LVL3_MAC_LEN];
uint8_t nonce[12];
uint32_t nonce_u32;
uint8_t mac[16];

    if (len < _mac_len + _nonce_len) { // higher level code should prevent this, play it safe
        return false;
    }

    // get mac from data
    memcpy(received_mac, data, _mac_len); // data[0] ... data[mac_len - 1]

    // get nonce from data
    memset(nonce, 0, 12);
    nonce_u32 = 0;
    memcpy(nonce, data + _mac_len, _nonce_len); // data[mac_len] ... data[mac_len + nonce_len - 1]
    memcpy(&nonce_u32, nonce, _nonce_len);      // _nonce_u32 = _nonce[0] ... _nonce[nonce_len - 1]

    // correct len for the mac and nonce
    len -= _mac_len + _nonce_len;

    // move data to data[0]
    memmove(data, data + _mac_len + _nonce_len, len); // NOT memcpy(), needs to copy from beginning towards end !!

    // fake the nonce for role
    nonce[11] = (_role == TX) ? 0xAA : 0x55;

    if (_mac_len) {
        // calculate MAC over header + nonce + payload
        _mac_it(mac, header, header_len, data, len, nonce, _nonce_len);

        // comparison of mac_len byte mac
        bool ok = true;
        for (uint8_t i = 0; i < _mac_len; i++) { if (mac[i] != received_mac[i]) ok = false; }

        if (!ok) { // authentication failed
            mac_errors++;
            return false;
        }
    }

    // check nonce, don't accept previously seen nonces, to prevent replay attacks
    // do only for privacy levels > 1
    // TODO: what needs to be done upon connection loss? does it play well with ARQ?
    if (_privacy_level >= 2 && nonce_u32 <= _nonce_u32_last_received) {
        replay_counts++;
        //return false;
    }
    _nonce_u32_last_received = nonce_u32;

    // decrypt data at data[0]
    _crypt_it(data, len, nonce);

    return true;
}


//-------------------------------------------------------
// Monocypher interface
//-------------------------------------------------------

void tCrypto::_crypt_it(uint8_t* data, uint16_t len, uint8_t nonce[12])
{
// Note: the counter does not have to start at 0, one just needs to use
// different counter for each block, so always starting with 1 is fine

    crypto_chacha20_ietf(
        data,         // cipher_text,
        data,         // plain_text, same as cipher = in-place encoding
        len,          // text_size,
        _session_key, // key[32],
        nonce,        // nonce[12],
        1);           // ctr
}


void tCrypto::_mac_it(uint8_t mac[16], uint8_t* const header, uint8_t header_len, uint8_t* const data, uint16_t len, uint8_t nonce[12], uint8_t nonce_len)
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
        _session_key, // key[32],
        nonce,        // nonce[12],
        0);           // ctr

    crypto_poly1305_init(&ctx, poly1305_key);
    crypto_poly1305_update(&ctx, header, header_len);
    crypto_poly1305_update(&ctx, nonce, nonce_len);
    crypto_poly1305_update(&ctx, data, len);
    crypto_poly1305_final(&ctx, mac);
}

