
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "../../../../modules/stm32-dronecan-lib/libcanard/canard.h" //XX#include <canard.h>




#define DRONECAN_PROTOCOL_CANCONFIG_MAX_SIZE 5
#define DRONECAN_PROTOCOL_CANCONFIG_SIGNATURE (0x29C5251BF4F06CA4ULL)

#define DRONECAN_PROTOCOL_CANCONFIG_ID 345



#define DRONECAN_PROTOCOL_CANCONFIG_CAN_BITRATE_1MBPS 1000000

#define DRONECAN_PROTOCOL_CANCONFIG_CAN_BITRATE_2MBPS 2000000

#define DRONECAN_PROTOCOL_CANCONFIG_CAN_BITRATE_4MBPS 4000000

#define DRONECAN_PROTOCOL_CANCONFIG_CAN_BITRATE_5MBPS 5000000

#define DRONECAN_PROTOCOL_CANCONFIG_CAN_BITRATE_8MBPS 8000000

#define DRONECAN_PROTOCOL_CANCONFIG_CAN_CLASSIC 0

#define DRONECAN_PROTOCOL_CANCONFIG_CAN_FD 1





#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class dronecan_protocol_CanConfig_cxx_iface;
#endif


struct dronecan_protocol_CanConfig {

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = dronecan_protocol_CanConfig_cxx_iface;
#endif




    uint32_t bit_rate;



    uint8_t variant;



    uint8_t reserved;



};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t dronecan_protocol_CanConfig_encode(struct dronecan_protocol_CanConfig* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool dronecan_protocol_CanConfig_decode(const CanardRxTransfer* transfer, struct dronecan_protocol_CanConfig* msg);

#if defined(CANARD_DSDLC_INTERNAL)

static inline void _dronecan_protocol_CanConfig_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dronecan_protocol_CanConfig* msg, bool tao);
static inline bool _dronecan_protocol_CanConfig_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dronecan_protocol_CanConfig* msg, bool tao);
void _dronecan_protocol_CanConfig_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dronecan_protocol_CanConfig* msg, bool tao) {

    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;






    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->bit_rate);

    *bit_ofs += 32;






    canardEncodeScalar(buffer, *bit_ofs, 3, &msg->variant);

    *bit_ofs += 3;






    canardEncodeScalar(buffer, *bit_ofs, 5, &msg->reserved);

    *bit_ofs += 5;





}

/*
 decode dronecan_protocol_CanConfig, return true on failure, false on success
*/
bool _dronecan_protocol_CanConfig_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dronecan_protocol_CanConfig* msg, bool tao) {

    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;





    canardDecodeScalar(transfer, *bit_ofs, 32, false, &msg->bit_rate);

    *bit_ofs += 32;







    canardDecodeScalar(transfer, *bit_ofs, 3, false, &msg->variant);

    *bit_ofs += 3;







    canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->reserved);

    *bit_ofs += 5;





    return false; /* success */

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct dronecan_protocol_CanConfig sample_dronecan_protocol_CanConfig_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>


BROADCAST_MESSAGE_CXX_IFACE(dronecan_protocol_CanConfig, DRONECAN_PROTOCOL_CANCONFIG_ID, DRONECAN_PROTOCOL_CANCONFIG_SIGNATURE, DRONECAN_PROTOCOL_CANCONFIG_MAX_SIZE);


#endif
#endif
