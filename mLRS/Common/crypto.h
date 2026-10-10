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
- on bind, a key root is exchanged, which is based on bind phrase, tx uid, rx uid, 8-byte random number
  from that a static key is generated
  Note: the material used for constructing the static key is exchanged during binding in plain text.
  Binding must thus be performed in a secure environment. If there is any suspicion that the static key
  has been compromised, a new binding should be performed.
- on first connection this happen
    - a fresh 8-byte session random number is exchanged; the exchange is encrypted and
      authenticated with a 12-byte random nonce and 8-byte mac using the static key
    - a secret session key is generated, which is based on
      the key root data plus the 8-byte session random number
- depending on the privacy level, the nonce is 3 or 4 bytes, and a mac for authentication is 0, 3, or 8 bytes
- replay attacks can be prevented by requiring the nonce to monotonously increase (not yet implemented)
- privacy levels
    off: nothing
    level 1: only encryption                       only payload  (3 bytes nonce, no authentication, no replay attack prevention)
    level 2: encryption + authentication           RC + payload  (3 bytes nonce, 3 bytes mac, replay attack prevention)
    level 3: stronger encryption + authentication  RC + payload  (4 bytes nonce, 8 bytes mac, replay attack prevention)
*/
//*******************************************************
#ifndef CRYPTO_H
#define CRYPTO_H
#pragma once


#include <inttypes.h>


#define CRYPTO_STARTUP_RANDOM_BUF_LEN  28 // length of encrypted session random, nonce, mac
#define CRYPTO_NONCE_MAX_LEN  12 // maximum length of nonce and mac


class tCrypto
{
  public:
    typedef enum {
        TX = 0,
        RX,
    } ROLE_ENUM;

    void Init(
        uint8_t role,
        char* const bind_phrase, uint8_t tx_uid[12], uint8_t rx_uid[12], uint64_t tx_random,
        uint8_t privacy_level);

    void SetSessionKey(uint64_t session_random); // Tx only
    void EncryptSessionRandom(uint8_t* const buf28, uint64_t startup_random, uint64_t bind_random); // Tx only
    void SetSessionKeyFromEncryptedRandomBuf(uint8_t* const buf28); // Rx only

    bool ValidKeys(void);

    uint8_t PrivacyLevel(void) { return _privacy_level; }
    bool IsAuthenticated(void) { return (_privacy_level >= 2); }
    uint16_t NonceLen(void);
    void Encrypt(void* const header, uint8_t header_len, void* const data, uint8_t len);
    void RecalculateMac(void* const header, uint8_t header_len, void* const data, uint8_t len);
    bool Decrypt(void* const header, uint8_t header_len, void* const data, uint8_t len);

    uint64_t SessionRandom(void) { return (_session_key_has_been_set) ? _session_random : 0; } // Rx only, only for reporting, no function

    uint32_t mac_errors;
    uint32_t replay_counts;

  private:
    uint8_t _role;
    uint8_t _privacy_level;

    uint8_t _nonce_len;
    uint8_t _mac_len;

    uint64_t _static_random;
    uint8_t _static_source[64];
    uint8_t _static_key[32];

    uint32_t _startup_nonce_u32;

    uint64_t _session_random;
    uint8_t _session_key[32];
    bool _session_key_has_been_set;
    uint32_t _nonce_u32;

    uint32_t _nonce_u32_last_received;

    void _encrypt_it(uint8_t* const header, uint8_t header_len, uint8_t* const data, uint8_t len);
    void _remac_it(uint8_t* const header, uint8_t header_len, uint8_t* const data, uint8_t len);
    bool _decrypt_it(uint8_t* const header, uint8_t header_len, uint8_t* const data, uint8_t len);

    void _crypt_it(uint8_t* data, uint16_t len, uint8_t nonce[12]);
    void _mac_it(uint8_t mac[16], uint8_t* const header, uint8_t header_len, uint8_t* const data, uint16_t len, uint8_t nonce[12], uint8_t nonce_len);
};


#endif // CRYPTO_H
