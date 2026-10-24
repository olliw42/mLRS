//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// OTA, shared definitions
// image info, protocol
//*******************************************************
#ifndef OTA_PROTOCOL_H
#define OTA_PROTOCOL_H
#pragma once

#include <inttypes.h>


// tx: relays the update to the receiver, rx: can be updated over the air
#if defined DEVICE_IS_TRANSMITTER
  #define USE_RX_OTA
#endif
#if defined DEVICE_IS_RECEIVER && (defined ESP32 || defined ESP8266)
  #define USE_RX_OTA
#endif


#define OTA_PROTOCOL_VERSION      1


//-------------------------------------------------------
// App image info
//-------------------------------------------------------
// sits somewhere in the image, for the host tool, to find out which receiver the image is for
// the receiver is identified by its device name, as in the bind and setup frames

#define OTA_APP_INFO_MAGIC        0x4F4C524D // 'MRLO'

typedef struct
{
    uint32_t magic;
    uint32_t version;
    char device_name[20+1]; // DEVICE_NAME
} tOtaAppInfo;


//-------------------------------------------------------
// Protocol
//-------------------------------------------------------
// Receiver pull, strict request-response. The rx asks, the host answers, the tx only relays.
// The rx owns all timing: if no answer comes in time it repeats the request, after some tries, or if no
// answer came for some seconds, it gives up and leaves. Host and tx have no state, an answer which comes
// too late is just ignored by the rx.
// rx -> host: cmd, session_id[2], payload
// host -> rx: cmd | 0x80, session_id[2], payload
// session_id is filled in by the tx, values are little endian
//
// INFO     rx: tOtaInfoRequest
//          host: tOtaInfoResponse      status not ok: the rx leaves, e.g. the image is not for it
// DATA     rx: tOtaDataRequest
//          host: tOtaDataResponse      1 <= n <= max_len data bytes, the rx advances its offset by n
// RESULT   rx: status                  the rx leaves when it gets the answer, or after some tries
//          host: -
//
// length is what is transferred, image_length what it becomes in flash, they differ only with compression
// crc is CRC-32/MPEG-2 over what is transferred, as the ESP32 rom and the ESP8266 core do it

#define OTA_PACKET_LEN_MAX        255 // a LoRa frame
#define OTA_PACKET_HEADER_LEN     3
#define OTA_DATA_LEN_MAX          (OTA_PACKET_LEN_MAX - OTA_PACKET_HEADER_LEN - 4)

typedef enum {
    OTA_CMD_INFO = 1,
    OTA_CMD_DATA,
    OTA_CMD_RESULT,
    OTA_CMD_RESPONSE = 0x80,
} OTA_CMD_ENUM;

// info: what the rx can take, and what the host sends
typedef enum {
    OTA_FLAG_DEFLATE = 0x01, // data is a zlib stream
    OTA_FLAG_GZIP = 0x02, // data is a gzip stream, which the rx stores as it is
} OTA_FLAG_ENUM;

typedef enum {
    OTA_STATUS_OK = 0,
    OTA_STATUS_ERR_TARGET,
    OTA_STATUS_ERR_LENGTH,
    OTA_STATUS_ERR_VERSION,
    OTA_STATUS_ERR_FLASH,
    OTA_STATUS_ERR_IMAGE,
    OTA_STATUS_ERR_UNSUPPORTED,
    OTA_STATUS_ERR_DATA,
} OTA_STATUS_ENUM;

PACKED(
typedef struct
{
    uint8_t version; // OTA_PROTOCOL_VERSION
    uint32_t space; // the largest image it can take
    uint8_t flags; // OTA_FLAG_ENUM, what it can take
    char device_name_20[20]; // not zero terminated if it is 20 chars long
}) tOtaInfoRequest;

PACKED(
typedef struct
{
    uint8_t status; // OTA_STATUS_ENUM
    uint32_t length;
    uint8_t flags; // OTA_FLAG_ENUM, what is sent
    uint32_t image_length;
    uint32_t crc;
}) tOtaInfoResponse;

PACKED(
typedef struct
{
    uint32_t offset;
    uint8_t max_len;
}) tOtaDataRequest;

PACKED(
typedef struct
{
    uint32_t offset;
    uint8_t data[OTA_DATA_LEN_MAX];
}) tOtaDataResponse;


#endif // OTA_PROTOCOL_H
