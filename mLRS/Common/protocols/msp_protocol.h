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
// crc8           starting from flag up to inclusive payload
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
// helper fields
    uint8_t res; // see MSP_PARSE_RESULT
} msp_message_t;


typedef enum {
    MSP_SONAR_ALTITUDE            = 58,
    MSP_STATUS                    = 101,
    MSP_RAW_IMU                   = 102,
    MSP_RAW_GPS                   = 106,
    MSP_COMP_GPS                  = 107,
    MSP_ATTITUDE                  = 108,
    MSP_ALTITUDE                  = 109,
    MSP_ANALOG                    = 110, // better use MSP_INAV_ANALOG
    MSP_ACTIVEBOXES               = 113,
    MSP_MISC                      = 114, // better use MSP_INAV_MISC
    MSP_BOXNAMES                  = 116,
    MSP_BOXIDS                    = 119,
    MSP_NAV_STATUS                = 121,
    MSP_BATTERY_STATE             = 130, // DJI googles fc battery info
    MSP_STATUS_EX                 = 150,
    MSP_SENSOR_STATUS             = 151,

    MSP_SET_RAW_RC                = 200,

    MSP_INAV_STATUS               = 0x2000,
    MSP_INAV_ANALOG               = 0x2002,
    MSP_INAV_MISC                 = 0x2003,
    MSP_INAV_AIR_SPEED            = 0x2009,
    MSP_INAV_MISC2                = 0x203A,

    MSP_SENSOR_RANGEFINDER        = 0x1F01,
    MSP_SENSOR_OPTIC_FLOW         = 0x1F02,
    MSP_SENSOR_GPS                = 0x1F03,
    MSP_SENSOR_COMPASS            = 0x1F04,
    MSP_SENSOR_BAROMETER          = 0x1F05,
    MSP_SENSOR_AIRSPEED           = 0x1F06,
} MSP_FUNCTION_ENUM;


//-- Telemetry data frames
// sources:
// - http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
// - https://github.com/iNavFlight/inav/blob/master/src/main/fc/fc_msp.c


// MSP_SONAR_ALTITUDE  58 //out message get surface altitude [cm]
MSP_PACKED(
typedef struct
{
    uint32_t rangefinder_altitude; // surface altitude [cm]
}) tMspSonarAltitude;

#define MSP_SONAR_ALTITUDE_LEN  4


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

#define MSP_RAW_IMU_LEN  18


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
    uint8_t fixType;          // 0 or 1
    uint8_t numSat;
    uint32_t lat;             // 1 / 10 000 000 deg
    uint32_t lon;             // 1 / 10 000 000 deg
    uint16_t alt;             // meters
    uint16_t ground_speed;    // cm/s
    uint16_t ground_course;   // degree*10 // gpsSol.groundCourse is 0.1 degrees
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
    uint16_t distance_to_home;  // unit: meter
    uint16_t direction_to_home; // unit: degree (range [-180;+180])
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
    int16_t roll;     // range [-1800;1800] (unit: 1/10 degree)
    int16_t pitch;    // range [-900;900] (unit: 1/10 degree)
    int16_t yaw;      // range [-180;180]
}) tMspAttitude; // 6 bytes

#define MSP_ATTITUDE_LEN  6


// MSP_ALTITUDE  109
//      EstAlt  INT 32  cm
//      vario   INT 16  cm/s
MSP_PACKED(
typedef struct
{
    int32_t estimated_position_z; // cm
    int16_t estimated_velocity_z; // cm/s
    int32_t baro_altitude;
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
    uint8_t battery_voltage;  // unit: 1/10 volt
    uint16_t mAh_drawn;       // milliamp hours drawn from battery
    uint16_t rssi;            // range: [0;1023]
    uint16_t amperage;        // send amperage in 0.01 A steps, range is -320A to 320A
}) tMspAnalog; // 7 bytes

#define MSP_ANALOG_LEN  7


// MSP_ACTIVEBOXES  113
MSP_PACKED(
typedef struct
{
    uint8_t msp_box_mode_flags[1]; // to have a reference pointer
}) tMspActiveBoxes;


// MSP_BOXIDS  119
MSP_PACKED(
typedef struct
{
    uint8_t msp_box_ids[1]; // to have a reference pointer
}) tMspBoxIds;


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
typedef enum {
    MSP_INAV_STATUS_ARMING_FLAGS_ARMED  = (uint32_t)0x04,
} MSP_INAV_STATUS_ARMING_FLAGS_ENUM;

// see src/main/fc_fc_masp_box.c, uint16_t packSensorStatus(void)
typedef enum {
    INAV_SENSOR_STATUS_ACC          = 0,
    INAV_SENSOR_STATUS_BARO         = 1,
    INAV_SENSOR_STATUS_MAG          = 2,
    INAV_SENSOR_STATUS_GPS          = 3,
    INAV_SENSOR_STATUS_RANGEFINDER  = 4,
    INAV_SENSOR_STATUS_OPFLOW       = 5,
    INAV_SENSOR_STATUS_PITOT        = 6,
    INAV_SENSOR_STATUS_TEMP         = 7,
    INAV_SENSOR_STATUS_HW_FAILURE   = 15,
} INAV_SENSOR_STATUS_FLAGS_ENUM;

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
    uint8_t msp_box_mode_flags[1]; // to have a reference pointer
    // sbufWriteData(dst, &mspBoxModeFlags, sizeof(mspBoxModeFlags));
    // the length of the box mode flags can be determined form the length of the payload, = len - 14
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
    uint16_t battery_voltage;             // seems to be in 0.01 V steps ???
    uint16_t amperage;                    // send amperage in 0.01 A steps
    uint32_t power;                       // power draw
    uint32_t mAh_drawn;                   // milliamp hours drawn from battery
    uint32_t mWh_drawn;                   // milliWatt hours drawn from battery
    uint32_t battery_remaining_capacity;
    uint8_t battery_percentage;
    uint16_t rssi;
}) tMspInavAnalog; // 24 bytes

#define MSP_INAV_ANALOG_LEN  24


// MSP2_INAV_MISC2  0x203A
MSP_PACKED(
typedef struct
{
    uint32_t ontime;                            // on time (seconds)
    uint32_t flight_time;                       // flight time (seconds)
    uint8_t throttle_percent;                   // throttle percent
    uint8_t navigation_is_controlling_throttle; // auto throttle flag (0 or 1)
}) tMspInavMisc2; // 10 bytes

#define MSP_INAV_MISC2_LEN  10


//-- Control/Command data frames

// MSP_SET_RAW_RC  200
MSP_PACKED(
typedef struct
{
    uint16_t rc[16];                            // range [1000;2000]
}) tMspSetRawRc; // 32 bytes

#define MSP_SET_RAW_RC_LEN  32


//-------------------------------------------------------
// MspX Messages
//-------------------------------------------------------

typedef enum {
    MSPX_STATUS = 0x2FFF,
} MSPX_FUNCTION_ENUM;


typedef struct {
    const char *boxName;
    uint8_t boxModeFlag;
    const char *telName;
} tMspBoxArray;


typedef enum {
    MSP_BOXARRAY_ARM = 0,
    MSP_BOXARRAY_FAILSAFE, // put it first so it has priority
    MSP_BOXARRAY_ANGLE,
    MSP_BOXARRAY_HORIZON,
    MSP_BOXARRAY_NAV_ALTHOLD,
    MSP_BOXARRAY_HEADING_HOLD,
    MSP_BOXARRAY_HEADFREE,
    MSP_BOXARRAY_NAV_RTH,
    MSP_BOXARRAY_NAV_POSHOLD,
    MSP_BOXARRAY_MANUAL,
    MSP_BOXARRAY_AUTO_TUNE,
    MSP_BOXARRAY_NAV_WP,
    MSP_BOXARRAY_AIR_MODE,
    MSP_BOXARRAY_FLAPERON,
    MSP_BOXARRAY_TURN_ASSIST,
    MSP_BOXARRAY_NAV_LAUNCH,
    MSP_BOXARRAY_NAV_COURSE_HOLD,
    MSP_BOXARRAY_NAV_CRUISE,
    MSP_BOXARRAY_ANGLE_HOLD,
    MSP_BOXARRAY_COUNT,
} MSP_BOXARRAY_ENUM;


static tMspBoxArray boxarray[MSP_BOXARRAY_COUNT] = {
    { .boxName = "ARM",               .boxModeFlag = 255,   .telName = "ARM" },
    { .boxName = "FAILSAFE",          .boxModeFlag = 255,   .telName = "!FS!" },
    { .boxName = "ANGLE",             .boxModeFlag = 255,   .telName = "ANGL" },
    { .boxName = "HORIZON",           .boxModeFlag = 255,   .telName = "HOR" },
    { .boxName = "NAV ALTHOLD",       .boxModeFlag = 255,   .telName = "ALTH" },
    { .boxName = "HEADING HOLD",      .boxModeFlag = 255,   .telName = "HHLD" },
    { .boxName = "HEADFREE",          .boxModeFlag = 255,   .telName = "HFRE" },
    { .boxName = "NAV RTH",           .boxModeFlag = 255,   .telName = "RTH" },
    { .boxName = "NAV POSHOLD",       .boxModeFlag = 255,   .telName = "HOLD" },
    { .boxName = "MANUAL",            .boxModeFlag = 255,   .telName = "MAN" },
    { .boxName = "AUTO TUNE",         .boxModeFlag = 255,   .telName = "ATUN" },
    { .boxName = "NAV WP",            .boxModeFlag = 255,   .telName = "WP" },
    { .boxName = "AIR MODE",          .boxModeFlag = 255,   .telName = "AIR" },
    { .boxName = "FLAPERON",          .boxModeFlag = 255,   .telName = "FLAP" },
    { .boxName = "TURN ASSIST",       .boxModeFlag = 255,   .telName = "TURN" },
    { .boxName = "NAV LAUNCH",        .boxModeFlag = 255,   .telName = "LNCH" },
    { .boxName = "NAV COURSE HOLD",   .boxModeFlag = 255,   .telName = "CRSH" },
    { .boxName = "NAV CRUISE",        .boxModeFlag = 255,   .telName = "CRUS" },
    { .boxName = "ANGLE HOLD",        .boxModeFlag = 255,   .telName = "ANGH" },
};


#endif // MSP_PROTOCOL_H



/*
https://github.com/iNavFlight/inav/blob/master/src/main/fc/fc_msp.c
static bool mspFcProcessOutCommand(uint16_t cmdMSP, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn)

151    case MSP_SENSOR_STATUS:
        sbufWriteU8(dst, isHardwareHealthy() ? 1 : 0);
        sbufWriteU8(dst, getHwGyroStatus());
        sbufWriteU8(dst, getHwAccelerometerStatus());
        sbufWriteU8(dst, getHwCompassStatus());
        sbufWriteU8(dst, getHwBarometerStatus());
        sbufWriteU8(dst, getHwGPSStatus());
        sbufWriteU8(dst, getHwRangefinderStatus());
        sbufWriteU8(dst, getHwPitotmeterStatus());
        sbufWriteU8(dst, getHwOpticalFlowStatus());
        break;


0x2000    case MSP2_INAV_STATUS:
        {
            // Preserves full arming flags and box modes
            boxBitmask_t mspBoxModeFlags;
            packBoxModeFlags(&mspBoxModeFlags);

            sbufWriteU16(dst, (uint16_t)cycleTime);
#ifdef USE_I2C
            sbufWriteU16(dst, i2cGetErrorCounter());
#else
            sbufWriteU16(dst, 0);
#endif
            sbufWriteU16(dst, packSensorStatus());
            sbufWriteU16(dst, averageSystemLoadPercent);
            sbufWriteU8(dst, (getConfigBatteryProfile() << 4) | getConfigProfile());
            sbufWriteU32(dst, armingFlags);
            sbufWriteData(dst, &mspBoxModeFlags, sizeof(mspBoxModeFlags));
            sbufWriteU8(dst, getConfigMixerProfile());
        }
        break;


102    case MSP_RAW_IMU:
        {
            for (int i = 0; i < 3; i++) {
                sbufWriteU16(dst, (int16_t)lrintf(acc.accADCf[i] * 512));
            }
            for (int i = 0; i < 3; i++) {
                sbufWriteU16(dst, gyroRateDps(i));
            }
            for (int i = 0; i < 3; i++) {
#ifdef USE_MAG
                sbufWriteU16(dst, lrintf(mag.magADC[i]));
#else
                sbufWriteU16(dst, 0);
#endif
            }
        }
        break;


108    case MSP_ATTITUDE:
        sbufWriteU16(dst, attitude.values.roll);
        sbufWriteU16(dst, attitude.values.pitch);
        sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
        break;


109    case MSP_ALTITUDE:
        sbufWriteU32(dst, lrintf(getEstimatedActualPosition(Z)));
        sbufWriteU16(dst, lrintf(getEstimatedActualVelocity(Z)));
#if defined(USE_BARO)
        sbufWriteU32(dst, baroGetLatestAltitude());
#else
        sbufWriteU32(dst, 0);
#endif
        break;


//#define MSP_SONAR_ALTITUDE              58 //out message get surface altitude [cm]
58    case MSP_SONAR_ALTITUDE:
#ifdef USE_RANGEFINDER
        sbufWriteU32(dst, rangefinderGetLatestAltitude());
#else
        sbufWriteU32(dst, 0);
#endif
        break;


0x2002    case MSP2_INAV_ANALOG:
        // Bit 1: battery full, Bit 2: use capacity threshold, Bit 3-4: battery state, Bit 5-8: battery cell count
        sbufWriteU8(dst, batteryWasFullWhenPluggedIn() | (batteryUsesCapacityThresholds() << 1) | (getBatteryState() << 2) | (getBatteryCellCount() << 4));
        sbufWriteU16(dst, getBatteryVoltage());
        sbufWriteU16(dst, getAmperage()); // send amperage in 0.01 A steps
        sbufWriteU32(dst, getPower());    // power draw
        sbufWriteU32(dst, getMAhDrawn()); // milliamp hours drawn from battery
        sbufWriteU32(dst, getMWhDrawn()); // milliWatt hours drawn from battery
        sbufWriteU32(dst, getBatteryRemainingCapacity());
        sbufWriteU8(dst, calculateBatteryPercentage());
        sbufWriteU16(dst, getRSSI());
        break;


0x203A    case MSP2_INAV_MISC2:
        // Timers
        sbufWriteU32(dst, micros() / 1000000); // On time (seconds)
        sbufWriteU32(dst, getFlightTime()); // Flight time (seconds)

        // Throttle
        sbufWriteU8(dst, getThrottlePercent(true)); // Throttle Percent
        sbufWriteU8(dst, navigationIsControllingThrottle() ? 1 : 0); // Auto Throttle Flag (0 or 1)

        break;


106    case MSP_RAW_GPS:
        sbufWriteU8(dst, gpsSol.fixType);
        sbufWriteU8(dst, gpsSol.numSat);
        sbufWriteU32(dst, gpsSol.llh.lat);
        sbufWriteU32(dst, gpsSol.llh.lon);
        sbufWriteU16(dst, gpsSol.llh.alt/100); // meters
        sbufWriteU16(dst, gpsSol.groundSpeed);
        sbufWriteU16(dst, gpsSol.groundCourse);
        sbufWriteU16(dst, gpsSol.hdop);
        break;


107    case MSP_COMP_GPS:
        sbufWriteU16(dst, GPS_distanceToHome);
        sbufWriteU16(dst, GPS_directionToHome);
        sbufWriteU8(dst, gpsSol.flags.gpsHeartbeat ? 1 : 0);
        break;


0x2009    case MSP2_INAV_AIR_SPEED:
#ifdef USE_PITOT
        sbufWriteU32(dst, getAirspeedEstimate());
#else
        sbufWriteU32(dst, 0);
#endif
        break;






MSP_SET_RAW_RC  200   → FC  rcData[RC_CHANS]  16 x UINT 16  Range [1000;2000]

ROLL/PITCH/YAW/THROTTLE/AUX1/AUX2/AUX3AUX4

This request is used to inject RC channel via MSP. Each chan overrides legacy RX as long as it is refreshed at least every second. See UART radio projects for more details.

*/
