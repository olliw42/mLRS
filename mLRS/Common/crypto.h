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
- on bind, a key root is exchanged, which is based on bind phrase, tx uid, rx uid, 8 byte random number
  from that a secret static key is generated
- on first connection two things happen
    - a 8 byte random number from a TRNG is exchanged; the exchange is encrypted and
      authenticated with 4 byte nonce and 4 byte mac using the static key
    - a secret session key is generated, which is based on
      the key root data plus the 8-byte random value
- depending on the privacy level, the nonce is 3 or 4 bytes, and a mac for authentication is 0, 3, or 8 bytes
- replay attacks can be prevented by requiring the nonce to monotonously increase
- privacy levels
    off: nothing
    level 1: only encryption                      (3 bytes nonce, no authentication, no replay attack prevention)
    level 2: encryption + authentication          (3 bytes nonce, 3 bytes mac, replay attack prevention)
    level 3: stronger encryption + authentication (4 bytes nonce, 8 bytes mac, replay attack prevention)
*/
//*******************************************************
#ifndef CRYPTO_H
#define CRYPTO_H
#pragma once


#include <inttypes.h>


class tCrypto
{
  public:
    void Init(char* const bind_phrase, uint8_t tx_uid[12], uint8_t rx_uid[12], uint64_t tx_random);
    void SetPrivacyLevel(uint8_t privacy_level);

    void SetSessionKey(uint64_t random);
    void GetEncryptedRandom(uint8_t random[16]);

    void SetSessionKeyFromEncryptedRandom(uint8_t random[16]);
    void Disconnected(void);

    uint8_t PrivacyLevel(void) { return _privacy_level; }
    uint16_t NonceLen(void);
    void Encrypt(uint8_t* const data, uint8_t len, uint8_t* payload_len);
    void Decrypt(uint8_t* const data, uint8_t len, uint8_t* payload_len);

    uint64_t Random(void) { return (_random_valid) ? _random : 0; }

  private:
    uint8_t _privacy_level;

    uint8_t _static[64];
    uint8_t _static_key[32];
    uint32_t _static_nonce_u32;
    uint64_t _random;
    bool _random_valid;
    uint8_t _key[32];
    uint32_t _nonce_u32;
    uint8_t _nonce[12];
    uint8_t _nonce_len;

    uint32_t _nonce_u32_last_received;

    void _encrypt_it(uint8_t* const data, uint8_t len, uint8_t* payload_len);
    void _decrypt_it(uint8_t* const data, uint8_t len, uint8_t* payload_len);

    void _crypt_it(uint8_t* data, uint16_t len);
    void _mac_it(uint8_t mac[16], uint8_t* const data, uint16_t len);
};


#endif // CRYPTO_H
