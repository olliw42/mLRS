//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// MSP Protocol
//
// MSP v2: https://github.com/iNavFlight/inav/wiki/MSP-V2
//
// STX1           '$'
// STX2           'X', 'M'
// type           '<', '>', '!'
// flag           0
// function_1     LSB
// function_2     MSB
// len_1          LSB
// len_2          MSB
// payload
// crc8
//
//  https://github.com/iNavFlight/inav/blob/master/src/main/fc/fc_msp.c
//
//*******************************************************
#ifndef MSP_PROTOCOL_H
#define MSP_PROTOCOL_H
#pragma once


#ifndef PACKED
  #define MSP_PACKED(__Declaration__)  __Declaration__ __attribute__((packed)) // that's for __GNUC__
#else
  #define MSP_PACKED  PACKED
#endif


#ifndef MSP_PAYLOAD_LEN_MAX
#define MSP_PAYLOAD_LEN_MAX       512 // INAV uses some quite big messages
#endif


#define MSP_MAGIC_1               '$'
#define MSP_MAGIC_2_V1            'M'
#define MSP_MAGIC_2_V2            'X'
#define MSP_TYPE_REQUEST          '<'
#define MSP_TYPE_RESPONSE         '>'
#define MSP_TYPE_ERROR            '!'
#define MSP_HEADER_LEN            8
#define MSP_FRAME_LEN_MAX         (8 + MSP_PAYLOAD_LEN_MAX + 1) // =  HEADER_LEN_MAX + PAYLOAD_LEN_MAX + CHECKSUM_LEN


typedef struct {
    uint8_t magic2;
    uint8_t type;
    uint8_t flag;
    uint16_t function;
    uint16_t len;
    uint8_t payload[MSP_PAYLOAD_LEN_MAX];
    uint8_t checksum;
//helper fields
    uint8_t res; // see MSP_PARSE_RESULT
} msp_message_t;




typedef enum {
    MSP_STATUS                    = 101,
    MSP_RAW_IMU                   = 102,
    MSP_RAW_GPS                   = 106,
    MSP_COMP_GPS                  = 107,
    MSP_ATTITUDE                  = 108,
    MSP_ALTITUDE                  = 109,
    MSP_ANALOG                    = 110,
    MSP_ACTIVEBOXES               = 113,
    MSP_MISC                      = 114,
    MSP_NAV_STATUS                = 121,
    MSP_BATTERY_STATE             = 130,
    MSP_STATUS_EX                 = 150,
    MSP_SENSOR_STATUS             = 151,

    MSP_INAV_STATUS               = 0x2000,
    MSP_INAV_ANALOG               = 0x2002,
    MSP_INAV_MISC                 = 0x2003,

    MSP_SENSOR_RANGEFINDER        = 0x1F01,
    MSP_SENSOR_OPTIC_FLOW         = 0x1F02,
    MSP_SENSOR_GPS                = 0x1F03,
    MSP_SENSOR_COMPASS            = 0x1F04,
    MSP_SENSOR_BAROMETER          = 0x1F05,
    MSP_SENSOR_AIRSPEED           = 0x1F06,
} MSP_FUNCTION_ENUM;



//-- Telemetry data frames


// MSP_STATUS  101
//      cycleTime   UINT 16   unit: microseconds
//      i2c_errors_count  UINT 16
//      sensor  UINT 16   BARO<<1|MAG<<2|GPS<<3|SONAR<<4
//      flag  UINT 32   a bit variable to indicate which BOX are active, the bit position depends on the BOX which are configured
//      global_conf.currentSet  UINT 8  to indicate the current configuration setting


// MSP_RAW_IMU  102
//      accx  INT 16  unit: it depends on ACC sensor and is based on ACC_1G definition
//                    MMA7455 64 / MMA8451Q 512 / ADXL345 265 / BMA180 255 / BMA020 63 / NUNCHUCK 200 / LIS3LV02 256 / LSM303DLx_ACC 256 / MPU6050 512 / LSM330 256
//      accy  INT 16
//      accz  INT 16
//      gyrx  INT 16  unit: it depends on GYRO sensor. For MPU6050, 1 unit = 1/4.096 deg/s
//      gyry  INT 16
//      gyrz  INT 16
//      magx  INT 16  unit: it depends on MAG sensor.
//      magy  INT 16
//      magz  INT 16
MSP_PACKED(
typedef struct
{
    int16_t acc[3];
    uint16_t gyro[3];
    uint16_t mag[3];
}) tMspRawImu;

#define MSP_RAW_IMU_LEN  6


// MSP_RAW_GPS  106
//      GPS_FIX   UINT 8  0 or 1
//      GPS_numSat  UINT 8
//      GPS_coord[LAT]  UINT 32   1 / 10 000 000 deg
//      GPS_coord[LON]  UINT 32   1 / 10 000 000 deg
//      GPS_altitude  UINT 16   meter
//      GPS_speed   UINT 16   cm/s
//      GPS_ground_course   UINT 16   unit: degree*10
MSP_PACKED(
typedef struct
{
    uint8_t fixType;
    uint8_t numSat;
    uint32_t lat;
    uint32_t lon;
    uint16_t alt; // meters
    uint16_t ground_speed;
    uint16_t ground_course;
    uint16_t hdop;
}) tMspRawGps; // 18 bytes

#define MSP_RAW_GPS_LEN  18


// MSP_COMP_GPS  107
//      GPS_distanceToHome  UINT 16   unit: meter
//      GPS_directionToHome   UINT 16   unit: degree (range [-180;+180])
//      GPS_update  UINT 8  a flag to indicate when a new GPS frame is received (the GPS fix is not dependent of this)
MSP_PACKED(
typedef struct
{
    uint16_t distance_to_home;
    uint16_t direction_to_home;
    uint8_t gps_heartbeat;
}) tMspCompGps; // 5 bytes

#define MSP_COMP_GPS_LEN  5


// MSP_ATTIUDE  108
//      angx  INT 16  Range [-1800;1800] (unit: 1/10 degree)
//      angy  INT 16  Range [-900;900] (unit: 1/10 degree)
//      heading   INT 16  Range [-180;180]
MSP_PACKED(
typedef struct
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
}) tMspAttitude; // 6 bytes

#define MSP_ATTITUDE_LEN  6


// MSP_ALTITUDE  109
//      EstAlt  INT 32  cm
//      vario   INT 16  cm/s
MSP_PACKED(
typedef struct
{
    int32_t estimated_position_z;
    int16_t estimated_velocity_z;
    uint32_t baro_altitude;
}) tMspAltitude; // 10 bytes

#define MSP_ALTIFUDE_LEN  10


// MSP_ANALOG  110
//      vbat  UINT 8  unit: 1/10 volt
//      intPowerMeterSum  UINT 16
//      rssi  UINT 16   range: [0;1023]
//      amperage  UINT 16
MSP_PACKED(
typedef struct
{
    uint8_t battery_voltage;
    uint16_t mAh_drawn; // milliamp hours drawn from battery
    uint16_t rssi;
    uint16_t amperage; // send amperage in 0.01 A steps, range is -320A to 320A
}) tMspAnalog; // 7 bytes

#define MSP_ANALOG_LEN  7


// MSP_SENSOR_STATUS  151
MSP_PACKED(
typedef struct
{
    uint8_t is_hardware_healty;
    uint8_t hw_gyro_status;
    uint8_t hw_AccelerometerStatus;
    uint8_t hw_CompassStatus;
    uint8_t hw_BarometerStatus;
    uint8_t hw_GPSStatus;
    uint8_t hw_RangefinderStatus;
    uint8_t hw_PitotmeterStatus;
    uint8_t hw_OpticalFlowStatus;
}) tMspSensorStatus; // 9 bytes

#define MSP_SENSOR_STATUS_LEN  9


// MSP_INAV_STATUS  0x2000
MSP_PACKED(
typedef struct
{
    uint16_t cycle_time;
    uint16_t i2c_error_counter;
    uint16_t sensor_status;
    uint16_t average_system_load_percent;
    uint8_t config_profile : 4;
    uint8_t config_battery_profile : 4;
    uint32_t arming_flags;
    // sbufWriteData(dst, &mspBoxModeFlags, sizeof(mspBoxModeFlags));
    // uint8_t getConfigMixerProfile;
}) tMspInavStatus; // >= 13 bytes


// MSP_INAV_ANALOG  0x2002
MSP_PACKED(
typedef struct
{
    // Bit 1: battery full, Bit 2: use capacity threshold, Bit 3-4: battery state, Bit 5-8: battery cell count
    uint8_t battery_was_full_when_plugged_in : 1;
    uint8_t battery_uses_capacity_thresholds : 1;
    uint8_t battery_state : 2;
    uint8_t battery_cell_count : 4;
    uint16_t battery_voltage;
    uint16_t amperage; // send amperage in 0.01 A steps
    uint32_t power; // power draw
    uint32_t mAh_drawn; // milliamp hours drawn from battery
    uint32_t mWh_drawn; // milliWatt hours drawn from battery
    uint32_t battery_remaining_capacity;
    uint8_t battery_percentage;
    uint16_t rssi;
}) tMspInavAnalog; // 24 bytes

#define MSP_INAV_ANALOG_LEN  24



#endif // MSP_PROTOCOL_H

