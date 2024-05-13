//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Passthrough Protocol Header
//
// ArduPilot Passthrough Protocol
// https://docs.google.com/spreadsheets/d/1oFv5zSl10wyR0LOcCChDdy9peIRNkbaEBrsHDQ0ZdmE/edit
// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.h
// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp
//*******************************************************
/*
  SYS_STATUS            #1    -> 0x5003 BATT_1                            SRx_EXT_STAT    not used, replaced by BATTERY_STATUS
  GPS_RAW_INT           #24   -> 0x5002 GPS STATUS                        SRx_EXT_STAT
  SCALED_IMU            #26   -> 0x5001 AP_STATUS                         -               not used, replaced by RAW_IMU
  RAW_IMU               #27   -> 0x5001 AP_STATUS                         SRx_RAW_SENS
  ATTITUDE              #30   -> 0x5006 ATTITUDE_RANGE                    SRx_EXTRA1
  GLOBAL_POSITION_INT   #33   -> 0x800 (2x), 0x5004 HOME                  SRx_POSITION
  MISSION_CURRENT       #42   -> 0x500D WAYPOINT_V2                       SRx_EXT_STAT
  NAV_CONTROLLER_OUTPUT #62   -> 0x5009 WAYPOINT_V1, 0x500D WAYPOINT V2   SRx_EXT_STAT
  VFR_HUD               #74   -> 0x5005 VEL_YAW, 0x50F2 VFR_HUD           SRx_EXTRA2
  TERRAIN_REPORT        #136  -> 0x500B TERRAIN                           -
  BATTERY_STATUS        #147  -> 0x5003 BATT_1, 0x5008 BATT_2             SRx_EXTRA3
  FENCE_STATUS          #162  -> 0x5001 AP_STATUS                         SRx_EXT_STAT
  // #173, #181, #226 are ArduPilot dialect specific
  RANGEFINDER           #173  -> 0x5006 ATTITUDE_RANGE                    SRx_EXTRA3
  BATTERY2              #181  -> 0x5008 BATT_2                            SRx_EXTRA3      not used, replaced by BATTERY_STATUS
  RPM                   #226  -> 0x500A RPM                               SRx_EXTRA3
  HOME_POSITION         #242  -> 0x5004 HOME                              -

  DISTANCE_SENSOR       #132                                              SRx_EXTRA3
  EKF_STATUS_REPORT     #193                                              SRx_EXTRA3
  WIND_COV              #231                                              -
  EXTENDED_SYS_STATE    #245                                              -

  // #168 is ArduPilot dialect specific
  WIND                  #168                                              SRx_EXTRA3

  GPS2_RAW              #124
  HIGH_LATENCY          #234
  HIGH_LATENCY2         #235

=> we want these streams:
                            HEARTBEAT               -> AP_STATUS_0x5001, PARAM_0x5007
  SRx_EXT_STAT (1/2 Hz) ->  SYS_STATUS              -> (VEL_YAW_0x5005)
                            GPS_RAW_INT             -> GPS_STATUS_0x5002 (GPS_LAT_0x800, GPS_LON_0x800)
                            MISSION_CURRENT         -> WAYPOINT_V2_0x500D
                            NAV_CONTROLLER_OUTPUT   -> WAYPOINT_V2_0x500D
                            FENCE_STATUS            -> (AP_STATUS_0x5001)
  SRx_EXTRA1   (4 Hz)   ->  ATTITUDE                -> ATTITUDE_RANGE_0x5006
  SRx_EXTRA2   (4 Hz)   ->  VFR_HUD                 -> VEL_YAW_0x5005, VEL_YAW_0x5005_AIR (AP_STATUS_0x5001)
  SRx_EXTRA3   (1/2 Hz) ->  RANGEFINDER             -> ATTITUDE_RANGE_0x5006
                            BATTERY_STATUS          -> BATT_1_0x5003, BATT_2_0x5008
                            TERRAIN_REPORT          -> TERRAIN_0x500B
                            RPM                     -> RPM_0x500A
                            WIND ?useful?
                            DISTANCE_SENSOR ?useful? yes, we want to digest it if we get it
  SRx_POSITION (2 Hz)   ->  GLOBAL_POSITION_INT     -> GPS_LAT_0x800, GPS_LON_0x800, HOME_0x5004
  SRx_RAW_SENS (0 Hz)   ->  RAW_IMU                 -> (AP_STATUS_0x5001)

  missing                   HOME_POSITION           -> HOME_0x5004
*/
/*
  MavToPass uses
  - SYS_STATUS to get BATT1 volt and curr, and BATTERY_STATUS to get BATT1 mAh
  - BATTERY2 to get BATT2 volt and curr, and BATTERY_STATUS to get BATT2 mAh
  we replace it with only using BATTERY_STATUS, see also MAVlink for OpenTx

  MavToPass uses
  - SCALED_IMU to get IMU temperature
  we replace it with RAW_IMU, as it is streamed
*/
#ifndef PASSTHROUGH_PROTOCOL_H
#define PASSTHROUGH_PROTOCOL_H
#pragma once


#include "../thirdparty/thirdparty.h"


//-------------------------------------------------------
// Interface Implementation

// TODO:
// for passthrough messages which involve date from several mavlink messages, we need to avoid that invalid data is used

#define RAD2DEGF                    5.729577951E+01f
#define DEG2RADF                    1.745329252E-02f


class tPassThrough
{
  public:
    void Init(void);

    bool GetTelemetryFrameSingle(uint8_t packet_type, uint8_t* data, uint8_t* len);
    bool GetTelemetryFrameMulti(uint8_t* data, uint8_t* len);

    enum {
        GPS_LAT_0x800 = 0,        // 0x800 GPS lat
        GPS_LON_0x800,            // 0x800 GPS lon
        TEXT_0x5000,              // 0x5000 status text
        AP_STATUS_0x5001,         // 0x5001 AP status
        GPS_STATUS_0x5002,        // 0x5002 GPS status (sats,fix,hdop,altitude_MSL)
        BATT_1_0x5003,            // 0x5003 battery 1 (voltage,current,mAh)
        HOME_0x5004,              // 0x5004 home (distance,altitude_rel,direction)
        VEL_YAW_0x5005,           // 0x5005 velocity and yaw (climbspeed,speed,yaw,flag for groundspeed/airspeed)
        ATTITUDE_RANGE_0x5006,    // 0x5006 attitude and range (roll,pitch,rangefinder_distance)
        PARAM_0x5007,             // 0x5007 parameters
        BATT_2_0x5008,            // 0x5008 battery 2 (voltage,current,mAh)
        WAYPOINT_V1_0x5009,       // 0x5009 waypoint data v1 // not used by ArduPilot
        RPM_0x500A,               // 0x500A rpm (rpm sensor 1, rpm sensor 2)
        TERRAIN_0x500B,           // 0x500B terrain data (height above terrain, flag for unhealthy)
        WIND_0x500C,              // 0x500C wind data (true direction, true windspeed, apparent direction, apparent windspeed)
        WAYPOINT_V2_0x500D,       // 0x500D waypoint data v2 (number,distance,yaw)

        SERVO_OUTPUT_RAW_0x50F1,  // 0x50F1 // not used by ArduPilot
        VFR_HUD_0x50F2,           // 0x50F2 (airspeed,throttle,altitude_MSL) // not used by ArduPilot
        RSSI_0xF101,              // 0xF101 // not used by ArduPilot

        // own additions
        VEL_YAW_0x5005_AIR,       // 0x5005 velocity and yaw (climbspeed,speed,yaw,flag for groundspeed/airspeed)

        PASSTHROUGH_PACKET_TYPE_NUM
    } PASSTHROUGHPACKETTYPEENUM;

    // these read a mavlink message and convert data into passthrough data fields
    
    void handle_mavlink_msg_passthrough_array(fmav_frsky_passthrough_array_t* payload);
    void handle_mavlink_msg_passthrough_array_tunnel(fmav_tunnel_t* payload);
    void decode_passthrough_array(uint8_t count, uint8_t* buf);
    bool passthrough_array_is_receiving;

    void handle_mavlink_msg_heartbeat(fmav_heartbeat_t* payload);           // #0
    void handle_mavlink_msg_sys_status(fmav_sys_status_t* payload);         // #1
    void handle_mavlink_msg_gps_raw_int(fmav_gps_raw_int_t* payload);       // #24
    void handle_mavlink_msg_raw_imu(fmav_raw_imu_t* payload);               // #27
    void handle_mavlink_msg_attitude(fmav_attitude_t* payload);             // #30
    void handle_mavlink_msg_global_position_int(fmav_global_position_int_t* payload);     // #33
    void handle_mavlink_msg_mission_current(fmav_mission_current_t* payload);             // #42
    void handle_mavlink_msg_nav_controller_output(fmav_nav_controller_output_t* payload); // #62
    void handle_mavlink_msg_vfr_hud(fmav_vfr_hud_t* payload);               // #74
    void handle_mavlink_msg_terrain_report(fmav_terrain_report_t* payload); // #136
    void handle_mavlink_msg_battery_status(fmav_battery_status_t* payload); // #147
    void handle_mavlink_msg_fence_status(fmav_fence_status_t* payload);     // #162
    void handle_mavlink_msg_rangefinder(fmav_rangefinder_t* payload);       // #173, ArduPilot dialect specific
    void handle_mavlink_msg_rpm(fmav_rpm_t* payload);                       // #226, ArduPilot dialect specific
    void handle_mavlink_msg_home_position(fmav_home_position_t* payload);   // #242
    void handle_mavlink_msg_statustext(fmav_statustext_t* payload);         // #253

    // these read a msp message and convert data into passthrough data fields

    void handle_msp_msg_attitude(tMspAttitude* payload);
    void handle_msp_inav_status(tMspInavStatus* payload);
    void handle_msp_inav_analog(tMspInavAnalog* payload);
    void handle_msp_inav_misc2(tMspInavMisc2* payload);

    // methods to convert mavlink data to passthrough (OpenTX) format

    bool get_GpsLat_0x800(uint32_t* data);
    bool get_GpsLon_0x800(uint32_t* data);
    bool get_Text_0x5000(uint32_t* data);
    bool get_ApStatus_0x5001(uint32_t* data);
    bool get_GpsStatus_0x5002(uint32_t* data);
    bool get_Battery1_0x5003(uint32_t* data);
    bool get_Home_0x5004(uint32_t* data);
    bool get_VelocityYaw_0x5005(uint32_t* data);
    bool get_AttitudeRange_0x5006(uint32_t* data);
    bool get_Param_0x5007(uint32_t* data);
    bool get_Battery2_0x5008(uint32_t* data);
    bool get_Rpm_0x500A(uint32_t* data);
    bool get_Terrain_0x500B(uint32_t* data);
    bool get_Wind_0x500C(uint32_t* data);
    bool get_WayPointV2_0x500D(uint32_t* data);
    bool get_VfrHud_0x50F2(uint32_t* data);
    bool get_VelocityYaw_0x5005_Air(uint32_t* data);

    bool get_packet_data(uint8_t packet_type, uint32_t* data);

  private:

    uint16_t pt_id[PASSTHROUGH_PACKET_TYPE_NUM] = {
        0x800, 0x800,
        0x5000, 0x5001, 0x5002, 0x5003, 0x5004, 0x5005, 0x5006, 0x5007, 0x5008, 0x5009,
        0x500A, 0x500B, 0x500C, 0x500D,
        0x50F1, 0x50F2, 0xF101,
        0x5005,
    };

    bool pt_update[PASSTHROUGH_PACKET_TYPE_NUM];
    uint32_t pt_data[PASSTHROUGH_PACKET_TYPE_NUM];
    
    fmav_heartbeat_t heartbeat = {};
    fmav_sys_status_t sys_status = {};
    fmav_gps_raw_int_t gps_raw_int = {};
    fmav_raw_imu_t raw_imu = {};
    fmav_attitude_t attitude = {};
    fmav_global_position_int_t global_position_int = {};
    fmav_nav_controller_output_t nav_controller_output = {};
    fmav_mission_current_t mission_current = {};
    fmav_vfr_hud_t vfr_hud = {};
    fmav_terrain_report_t terrain_report = {};
    fmav_battery_status_t battery_status_id0 = {};
    fmav_battery_status_t battery_status_id1 = {};
    fmav_fence_status_t fence_status = {};
    fmav_rangefinder_t rangefinder = {};
    fmav_rpm_t rpm = {};
    fmav_home_position_t home_position = {};
    fmav_statustext_t statustext = {};

    // we double-buffer it, tried 8-size fifo, didn't appear to make a difference
    fmav_statustext_t statustext_cur = {};
    bool statustext_cur_inprocess;
    uint8_t statustext_cur_chunk_index;

    bool global_position_int_received_once;
    bool fence_status_received_once;
    bool home_position_received_once;

    bool vehicle_is_armed;

    uint8_t param_id_to_send;
};


void tPassThrough::Init(void)
{
    for (uint8_t n = 0; n < PASSTHROUGH_PACKET_TYPE_NUM; n++) pt_update[n] = false;

    statustext_cur_inprocess = false;
    statustext_cur_chunk_index = 0;

    global_position_int_received_once = false;
    fence_status_received_once = false;
    home_position_received_once = false;

    vehicle_is_armed = false;

    param_id_to_send = 1;

    passthrough_array_is_receiving = false;
}


//-------------------------------------------------------
// Mavlink Handlers

void tPassThrough::decode_passthrough_array(uint8_t count, uint8_t* buf)
{
    for (uint8_t i = 0; i < count; i++) {
        uint16_t id;
        uint32_t data;
        memcpy(&id, &(buf[i*6]), 2);
        memcpy(&data, &(buf[i*6 + 2]), 4);

        switch (id) {
        case 0x800:
            if (data & (1 << 31)) { // lon
                pt_data[GPS_LON_0x800] = data;
                pt_update[GPS_LON_0x800] = true;
            } else {
              pt_data[GPS_LAT_0x800] = data;
              pt_update[GPS_LAT_0x800] = true;
            }
            break;
//NO, we ignore it        case 0x5000: pt_data[TEXT_0x5000] = data; pt_update[TEXT_0x5000] = true; break;
        case 0x5001: pt_data[AP_STATUS_0x5001] = data; pt_update[AP_STATUS_0x5001] = true; break;
        case 0x5002: pt_data[GPS_STATUS_0x5002] = data; pt_update[GPS_STATUS_0x5002] = true; break;
        case 0x5003: pt_data[BATT_1_0x5003] = data; pt_update[BATT_1_0x5003] = true; break;
        case 0x5004: pt_data[HOME_0x5004] = data; pt_update[HOME_0x5004] = true; break;
        case 0x5005:
            if (data & (1 << 28)) { // airspeed
                pt_data[VEL_YAW_0x5005_AIR] = data;
                pt_update[VEL_YAW_0x5005_AIR] = true;
            } else { // groundspeed
                pt_data[VEL_YAW_0x5005] = data;
                pt_update[VEL_YAW_0x5005] = true;
            }
            break;
        case 0x5006: pt_data[ATTITUDE_RANGE_0x5006] = data; pt_update[ATTITUDE_RANGE_0x5006] = true; break;
        case 0x5007: pt_data[PARAM_0x5007] = data; pt_update[PARAM_0x5007] = true; break;
        case 0x5008: pt_data[BATT_2_0x5008] = data; pt_update[BATT_2_0x5008] = true; break;
        case 0x500A: pt_data[RPM_0x500A] = data; pt_update[RPM_0x500A] = true; break;
        case 0x500B: pt_data[TERRAIN_0x500B] = data; pt_update[TERRAIN_0x500B] = true; break;
        case 0x500C: pt_data[WIND_0x500C] = data; pt_update[WIND_0x500C] = true; break;
        case 0x500D: pt_data[WAYPOINT_V2_0x500D] = data; pt_update[WAYPOINT_V2_0x500D] = true; break;
        }
    }
}


void tPassThrough::handle_mavlink_msg_passthrough_array_tunnel(fmav_tunnel_t* payload)
{
    if (payload->payload_type != 34567) return;
    passthrough_array_is_receiving = true;
    decode_passthrough_array(payload->payload_length / 6, payload->payload);
}


void tPassThrough::handle_mavlink_msg_passthrough_array(fmav_frsky_passthrough_array_t* payload)
{
    passthrough_array_is_receiving = true;
    decode_passthrough_array(payload->count, payload->packet_buf);
}


void tPassThrough::handle_mavlink_msg_heartbeat(fmav_heartbeat_t* payload) // #0 -> 0x5001 AP_STATUS, 0x5007 PARAM
{
    memcpy(&heartbeat, payload, sizeof(fmav_heartbeat_t));
    pt_update[AP_STATUS_0x5001] = true;
    pt_update[PARAM_0x5007] = true;

    vehicle_is_armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
}


void tPassThrough::handle_mavlink_msg_sys_status(fmav_sys_status_t* payload) // #1 -> 0x5005 VEL_YAW
{
    memcpy(&sys_status, payload, sizeof(fmav_sys_status_t));
    // we do not update, VEL_YAW reads sys_status.onboard_sensor flags, but no urgent reason to update
    // pt_update[VEL_YAW_0x5005] = true;
}


void tPassThrough::handle_mavlink_msg_gps_raw_int(fmav_gps_raw_int_t* payload) // #24 -> 0x5002 GPS STATUS
{
    memcpy(&gps_raw_int, payload, sizeof(fmav_gps_raw_int_t));
    // we do not update, GPS_LAT/GPS_LON read gps_raw_int.fix_type, but no urgent reason to update
    // pt_update[GPS_LAT_0x800] = true;
    // pt_update[GPS_LON_0x800] = true;
    pt_update[GPS_STATUS_0x5002] = true;
}


void tPassThrough::handle_mavlink_msg_raw_imu(fmav_raw_imu_t* payload) // #27 -> 0x5001 AP_STATUS
{
    memcpy(&raw_imu, payload, sizeof(fmav_raw_imu_t));
    // we do not update, AP_STATUS reads raw_imu.temperature, but no urgent reason to update
    // pt_update[AP_STATUS_0x5001] = (raw_imu.temperature != 0); // 0 = invalid, so we update this only if valid
}


void tPassThrough::handle_mavlink_msg_attitude(fmav_attitude_t* payload) // #30 -> 0x5006 ATTITUDE_RANGE
{
    memcpy(&attitude, payload, sizeof(fmav_attitude_t));
    pt_update[ATTITUDE_RANGE_0x5006] = true;
}


void tPassThrough::handle_mavlink_msg_global_position_int(fmav_global_position_int_t* payload) // #33 -> 0x800 (2x), 0x5004 HOME
{
    memcpy(&global_position_int, payload, sizeof(fmav_global_position_int_t));
    pt_update[GPS_LAT_0x800] = true;
    pt_update[GPS_LON_0x800] = true;
    pt_update[HOME_0x5004] = true;
    global_position_int_received_once = true;

    // we fake here a home position
    // if not armed and no home_position yet received then we set it to current
    if (!vehicle_is_armed && !home_position_received_once) {
        home_position.longitude = global_position_int.lon;
        home_position.latitude = global_position_int.lat;
    }
}


void tPassThrough::handle_mavlink_msg_mission_current(fmav_mission_current_t* payload) // #42 -> 0x5009 WAYPOINT_V1, 0x500D WAYPOINT V2
{
    memcpy(&mission_current, payload, sizeof(fmav_mission_current_t));
    // pt_update[WAYPOINT_V1_0x5009] = true; // not supported
    // pt_update[WAYPOINT_V2_0x500D] = true; // TODO
}


void tPassThrough::handle_mavlink_msg_nav_controller_output(fmav_nav_controller_output_t* payload) // #62 -> 0x5009 WAYPOINT_V1, 0x500D WAYPOINT V2
{
    memcpy(&nav_controller_output, payload, sizeof(fmav_nav_controller_output_t));
    // pt_update[WAYPOINT_V1_0x5009] = true; // not supported
    // pt_update[WAYPOINT_V2_0x500D] = true; // TODO
}


void tPassThrough::handle_mavlink_msg_vfr_hud(fmav_vfr_hud_t* payload) // #74 -> 0x5005 VEL_YAW, 0x5001 AP_STATUS, 0x50F2 VFR_HUD
{
    memcpy(&vfr_hud, payload, sizeof(fmav_vfr_hud_t));
    pt_update[VEL_YAW_0x5005] = true;
    // we send it only if we have a sensor
    pt_update[VEL_YAW_0x5005_AIR] =
        ((sys_status.onboard_control_sensors_present & MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE) &&
         (sys_status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE) &&
         (sys_status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
    // we do not update, AP_STATUS reads vfr_hud.throttle, but no urgent reason to update IS THIS WHAT WE WANT ???
    // pt_update[AP_STATUS_0x5001] = true;
    // pt_update[VFR_HUD_0x50F2] = true; // we don't use it
}


void tPassThrough::handle_mavlink_msg_terrain_report(fmav_terrain_report_t* payload) // #136 -> 0x500B TERRAIN
{
    memcpy(&terrain_report, payload, sizeof(fmav_terrain_report_t));
    // pt_update[TERRAIN_0x500B] = true; // TODO
}


void tPassThrough::handle_mavlink_msg_battery_status(fmav_battery_status_t* payload) // #147 -> 0x5003 BATT_1, 0x5008 BATT_2
{
    if (payload->id == 0) {
        memcpy(&battery_status_id0, payload, sizeof(fmav_battery_status_t));
        pt_update[BATT_1_0x5003] = true;
    }
    if (payload->id == 1) {
        memcpy(&battery_status_id1, payload, sizeof(fmav_battery_status_t));
        pt_update[BATT_2_0x5008] = true;
    }
}


void tPassThrough::handle_mavlink_msg_fence_status(fmav_fence_status_t* payload) // #162 -> 0x5001 AP_STATUS
{
    memcpy(&fence_status, payload, sizeof(fmav_fence_status_t));
    // we do not update, AP_STATUS reads vfr_hud.throttle, but no urgent reason to update IS THIS WHAT WE WANT ???
    // pt_update[AP_STATUS_0x5001] = true;
    fence_status_received_once = true;
}


void tPassThrough::handle_mavlink_msg_rangefinder(fmav_rangefinder_t* payload) // #173 -> 0x5006 ATTITUDE_RANGE
{
    memcpy(&rangefinder, payload, sizeof(fmav_rangefinder_t));
    pt_update[ATTITUDE_RANGE_0x5006] = true; // TODO
}


void tPassThrough::handle_mavlink_msg_rpm(fmav_rpm_t* payload) // #226 -> 0x500A RPM
{
    memcpy(&rpm, payload, sizeof(fmav_rpm_t));
    pt_update[RPM_0x500A] = true; // TODO
}


void tPassThrough::handle_mavlink_msg_home_position(fmav_home_position_t* payload) // #242 -> 0x5004 HOME
{
    memcpy(&home_position, payload, sizeof(fmav_home_position_t));
    pt_update[HOME_0x5004] = true; // TODO
    home_position_received_once = true;
}


void tPassThrough::handle_mavlink_msg_statustext(fmav_statustext_t* payload) // #253 -> 0x5000 TEXT
{
    memcpy(&statustext, payload, sizeof(fmav_statustext_t));
    pt_update[TEXT_0x5000] = true;
}


//-------------------------------------------------------
// Msp Handlers

void tPassThrough::handle_msp_msg_attitude(tMspAttitude* payload)
{
    // only roll, pitch is used in AttitudeRange_0x5006
    attitude.pitch = (DEG2RADF * 0.1f) * payload->pitch; // cdeg -> rad
    attitude.roll = (DEG2RADF * 0.1f) * payload->roll; // cdeg -> rad
    pt_update[ATTITUDE_RANGE_0x5006] = true;

    vfr_hud.heading = payload->yaw; // deg -> deg
    pt_update[VEL_YAW_0x5005] = true;
}


void tPassThrough::handle_msp_inav_status(tMspInavStatus* payload)
{
    vehicle_is_armed = (payload->arming_flags & MSP_INAV_STATUS_ARMING_FLAGS_ARMED);
    pt_update[AP_STATUS_0x5001] = true;
}


void tPassThrough::handle_msp_inav_analog(tMspInavAnalog* payload)
{
    //int32_t voltage = mav_battery_voltage(&battery_status_id0);
    battery_status_id0.current_battery = payload->amperage; // 0.01 A ->
    battery_status_id0.current_consumed = payload->mAh_drawn; // mAh ->
    pt_update[BATT_1_0x5003] = true;
}


void tPassThrough::handle_msp_inav_misc2(tMspInavMisc2* payload)
{
    vfr_hud.throttle = payload->throttle_percent;
    pt_update[VFR_HUD_0x50F2] = true;
}


//-------------------------------------------------------
// Passthrough Converters

void pt_pack32(uint32_t* value, uint32_t data, uint8_t pos, uint8_t len)
{
    uint32_t mask = 0;
    for (uint32_t i = 0; i < len; i++) mask |= (1 << i);

    (*value) |= (data & mask) << pos;
}


bool tPassThrough::get_GpsLat_0x800(uint32_t* data)
{
    if (!pt_update[GPS_LAT_0x800]) return false;
    pt_update[GPS_LAT_0x800] = false;

    if (passthrough_array_is_receiving) { *data = pt_data[GPS_LAT_0x800]; return true; }

    if (gps_raw_int.fix_type < GPS_FIX_TYPE_3D_FIX) return false; // don't send if no valid data

    *data = 0;

    int32_t lat = global_position_int.lat;
    uint32_t pt_lat = (lat < 0) ? lat : -lat;
    pt_lat = (pt_lat / 50) * 3; // * 60/1000

    pt_pack32(data, pt_lat, 0, 30);
    pt_pack32(data, (global_position_int.lat < 0) ? 1 : 0, 30, 2);

    return true;
}


bool tPassThrough::get_GpsLon_0x800(uint32_t* data)
{
    if (!pt_update[GPS_LON_0x800]) return false;
    pt_update[GPS_LON_0x800] = false;

    if (passthrough_array_is_receiving) { *data = pt_data[GPS_LON_0x800]; return true; }

    if (gps_raw_int.fix_type < GPS_FIX_TYPE_3D_FIX) return false; // don't send if no valid data

    int32_t lon = global_position_int.lon;
    uint32_t pt_lon = (lon < 0) ? lon : -lon;
    pt_lon = (pt_lon / 50) * 3; // * 60/1000

    *data = 0;
    pt_pack32(data, pt_lon, 0, 30);
    pt_pack32(data, (global_position_int.lon < 0) ? 3 : 2, 30, 2);

    return true;
}


// this needs a special treatments as we send it in chunks
// we double buffer the statustext message
bool tPassThrough::get_Text_0x5000(uint32_t* data)
{
/*// NO: even with passthrough array we do it by reading the statustext, we ignore 0x5000 data from passthrough array
    if (passthrough_array_is_receiving) {
        if (!pt_update[TEXT_0x5000]) return false;
        pt_update[TEXT_0x5000] = false;
        *data = pt_data[TEXT_0x5000];
        return true;
    }
*/

    if (!statustext_cur_inprocess) { // idle, so we can check for a new statustext
        if (!pt_update[TEXT_0x5000]) return false; // nothing to do
        pt_update[TEXT_0x5000] = false;

        memcpy(&statustext_cur, &statustext, sizeof(fmav_statustext_t));
        statustext_cur_inprocess = true;
        statustext_cur_chunk_index = 0;
    }

    *data = 0;

    char c = '\0';
    for (uint8_t i = 0; i < 4; i++) {
        if (statustext_cur_chunk_index >= sizeof(statustext_cur.text)) break; // last char reached
        c = statustext_cur.text[statustext_cur_chunk_index++];
        if (c == '\0') break; // terminating char reached
        pt_pack32(data, c, (3 - i) * 8, 7); // 24, 16, 8, 0
    }
    // text is of size 50, which is not divisible by 4
    // hence, we always will have a terminating '\0' in a chunk, also when the string ends because of reaching 50
    if ((c == '\0') || (statustext_cur_chunk_index >= sizeof(statustext_cur.text))) {
        // we reached end of text
        // severity is sent as MSB of last three bytes of the last chunk (bits 24, 16, and 8)
        // works since characters are 7 bits
        pt_pack32(data, (statustext_cur.severity & 0x01), 7, 1);
        pt_pack32(data, (statustext_cur.severity & 0x02) >> 1, 15, 1);
        pt_pack32(data, (statustext_cur.severity & 0x04) >> 2, 23, 1);

        statustext_cur_inprocess = false;
    }

    return true;
}


bool tPassThrough::get_ApStatus_0x5001(uint32_t* data)
{
    if (!pt_update[AP_STATUS_0x5001]) return false;
    pt_update[AP_STATUS_0x5001] = false;

    if (passthrough_array_is_receiving) { *data = pt_data[AP_STATUS_0x5001]; return true; }

    uint32_t pt_flight_mode = heartbeat.custom_mode + 1;
    uint32_t pt_simple = 0;

    uint32_t pt_land_complete = vehicle_is_armed; // just make it equal to armed state
    uint32_t pt_armed = vehicle_is_armed;

    uint32_t pt_bat_fs = 0; // ?
    uint32_t pt_ekf_fs = 0; // ?
    uint32_t pt_fs = 0; // ?

    uint32_t pt_fence_present = (fence_status_received_once) ? 1 : 0;
    uint32_t pt_fence_breached = (fence_status.breach_status) ? 1 : 0;

    int32_t pt_throt = vfr_hud.throttle; // ArduPilot VFR_HUD sends abs(throttle)!!
    if (pt_throt <= 0) {
        pt_throt = 0;
    } else
    if (pt_throt >= 100) {
        pt_throt = 63;
    } else {
        pt_throt = (pt_throt * 63  + 49) / 100;
    }

    int32_t pt_imu_temp = 0;
    if (raw_imu.temperature != 0) { // 0 = invalid
        pt_imu_temp = raw_imu.temperature / 100; // cdegC -> degC
        if (pt_imu_temp < 19) pt_imu_temp = 19;
        if (pt_imu_temp > 82) pt_imu_temp = 82;
        pt_imu_temp -= 19;
    }

    *data = 0;
    pt_pack32(data, pt_flight_mode, 0, 5);
    pt_pack32(data, pt_simple, 5, 2);
    pt_pack32(data, pt_land_complete, 7, 1);
    pt_pack32(data, pt_armed, 8, 1);
    pt_pack32(data, pt_bat_fs, 9, 1);
    pt_pack32(data, pt_ekf_fs, 10, 2);
    pt_pack32(data, pt_fs, 12, 1);
    pt_pack32(data, pt_fence_present, 13, 1);
    pt_pack32(data, pt_fence_breached, 14, 1);
    pt_pack32(data, pt_throt, 19, 7);
    pt_pack32(data, pt_imu_temp, 26, 6);

    return true;
}


bool tPassThrough::get_GpsStatus_0x5002(uint32_t* data)
{
    if (!pt_update[GPS_STATUS_0x5002]) return false;
    pt_update[GPS_STATUS_0x5002] = false;

    if (passthrough_array_is_receiving) { *data = pt_data[GPS_STATUS_0x5002]; return true; }

    uint32_t pt_sats = gps_raw_int.satellites_visible;
    if (pt_sats == UINT8_MAX) pt_sats = 0; // UINT8_MAX = invalid
    if (pt_sats > 15) pt_sats = 15;

    uint32_t pt_fix, pt_fix2;
    if (gps_raw_int.fix_type <= GPS_FIX_TYPE_3D_FIX) { // 0,1,2,3
        pt_fix = gps_raw_int.fix_type;
        pt_fix2 = 0;
    } else { // 4,5,6
        pt_fix = 3;
        pt_fix2 = gps_raw_int.fix_type - GPS_FIX_TYPE_3D_FIX;
        if (pt_fix2 > 6) pt_fix2 = 6;
    }

    int32_t pt_hdop = 127;
    if (gps_raw_int.fix_type >= GPS_FIX_TYPE_3D_FIX) {
        pt_hdop = gps_raw_int.eph / 10; // cm -> dm
        if (pt_hdop > 127) pt_hdop = 127; // this also catches gps_raw_int.eph = UINT16_MAX for invalid
    }

    int32_t pt_altitude_MSL = 0;
    if (gps_raw_int.fix_type >= GPS_FIX_TYPE_3D_FIX) {
        pt_altitude_MSL = gps_raw_int.alt / 100; // mm -> cm
        if (pt_altitude_MSL > 127000) {
            pt_altitude_MSL = 127000;
        } else
        if (pt_altitude_MSL < -127000) {
            pt_altitude_MSL = -127000;
        }
    }

    *data = 0;
    pt_pack32(data, pt_sats, 0, 4);
    pt_pack32(data, pt_fix, 4, 2);
    pt_pack32(data, prep_number(pt_hdop, 2, 1), 6, 8);
    pt_pack32(data, pt_fix2, 14, 2);
    pt_pack32(data, prep_number(pt_altitude_MSL, 2, 2), 22, 9+1); // + sign bit

    return true;
}


int32_t mav_battery_voltage(fmav_battery_status_t* payload);


bool tPassThrough::get_Battery1_0x5003(uint32_t* data)
{
    if (!pt_update[BATT_1_0x5003]) return false;
    pt_update[BATT_1_0x5003] = false;

    if (passthrough_array_is_receiving) { *data = pt_data[BATT_1_0x5003]; return true; }

    int32_t voltage = mav_battery_voltage(&battery_status_id0);
    int32_t current = battery_status_id0.current_battery;
    int32_t consumed = battery_status_id0.current_consumed;

    int32_t pt_volt = voltage / 100; // mV -> dV
    int32_t pt_curr = (current < 0) ? 0 : current / 10; // cA -> dA // -1 = invalid
    int32_t pt_mAh = (consumed > INT16_MAX) ? INT16_MAX : consumed; // mAh -> mAh // range is limited to [0..INT16_MAX]

    *data = 0;
    pt_pack32(data, pt_volt, 0, 9);
    pt_pack32(data, prep_number(pt_curr, 2, 1), 9, 8);
    pt_pack32(data, pt_mAh, 17, 15);

    return true;
}


bool tPassThrough::get_Home_0x5004(uint32_t* data)
{
    if (!pt_update[HOME_0x5004]) return false;
    pt_update[HOME_0x5004] = false;

    if (passthrough_array_is_receiving) { *data = pt_data[HOME_0x5004]; return true; }

    // this is called only if global_position_int or home_position has been received once
    // we need to check if we really have got one global_position_int
    // since this might be have been called from home_position
    if (!global_position_int_received_once) return false;

    float lon1 = (float)home_position.longitude * 1.0E-7f * DEG2RADF; // home position
    float lat1 = (float)home_position.latitude * 1.0E-7f * DEG2RADF;
    float lon2 = (float)global_position_int.lon * 1.0E-7f * DEG2RADF; // current position
    float lat2 = (float)global_position_int.lat * 1.0E-7f * DEG2RADF;

    float cos_lat1 = cosf(lat1);
    float cos_lat2 = cosf(lat2);

    float direction = atan2f( sinf(lon2 - lon1) * cos_lat2 ,
                              cos_lat1 * sinf(lat2) - sinf(lat1) * cos_lat2 * cosf(lon2 - lon1)
                             );

    float direction_deg = direction * RAD2DEGF;
    if (direction_deg < 0.0f) direction_deg += 360.0f;

    int16_t direction_i16 = (int16_t)direction_deg - 180;
    if (direction_i16 < 0) direction_i16 += 360;
    if (direction_i16 > 359) direction_i16 -= 360;
    direction_i16 /= 3;

    float sin_dlat_half = sinf((lat2 - lat1) * 0.5f);
    float sin_dlon_half = sinf((lon2 - lon1) * 0.5f);

    float a = sin_dlat_half * sin_dlat_half + sin_dlon_half*sin_dlon_half * cos_lat1 * cos_lat2;
    float distance = 6371.0E+3f * 2.0f * asinf(sqrtf(a));

    int32_t pt_distance = distance;
    int32_t pt_altitude_rel = global_position_int.relative_alt / 100;
    int32_t pt_direction = direction_i16;

    *data = 0;
    pt_pack32(data, prep_number(roundf(pt_distance), 3, 2), 0, 12);
    pt_pack32(data, prep_number(roundf(pt_altitude_rel), 3, 2), 12, 12+1); // + sign bit
    pt_pack32(data, pt_direction, 25, 7);

    return true;
}

// Copter:
// vfr_hud groundspeed is ahrs.groundspeed()
// vfr_hud airspeed is copter.airspeed.get_airspeed() or ahrs().airspeed_vector_true(airspeed_vec_bf).length() or gps().ground_speed()
// not so clear what they are
// is there a way to figure out if there is a valid airspeed? => SYS_STATUS SENSOR STATUS flags!?
// Plane:
// vfr_hud groundspeed is ahrs.groundspeed()
// vfr_hud airspeed is plane.airspeed.get_airspeed() or ahrs().airspeed_estimate(aspeed) or 0
// ArduPilotPT sends ahrs().airspeed_estimate_true(airspeed_m) if true&wished or ahrs.groundspeed()
// so, the vfr_hud's groundspeed is ok

bool tPassThrough::get_VelocityYaw_0x5005(uint32_t* data)
{
    if (!pt_update[VEL_YAW_0x5005]) return false;
    pt_update[VEL_YAW_0x5005] = false;

    if (passthrough_array_is_receiving) { *data = pt_data[VEL_YAW_0x5005]; return true; }

    int32_t pt_climbspeed = roundf(vfr_hud.climb * 10.0f); // m/s -> dm/s
    int32_t pt_groundspeed = roundf(vfr_hud.groundspeed * 10.0f); // m/s -> dm/s
    int32_t pt_yaw = vfr_hud.heading * 5; // deg -> 0.2 deg

    *data = 0;
    pt_pack32(data, prep_number(pt_climbspeed, 2, 1), 0, 8+1); // sign bit
    pt_pack32(data, prep_number(pt_groundspeed, 2, 1), 9, 8);
    pt_pack32(data, pt_yaw, 17, 11);
    pt_pack32(data, 0, 28, 1); // mark it as groundspeed

    return true;
}

bool tPassThrough::get_VelocityYaw_0x5005_Air(uint32_t* data)
{
    if (!pt_update[VEL_YAW_0x5005_AIR]) return false;
    pt_update[VEL_YAW_0x5005_AIR] = false;

    if (passthrough_array_is_receiving) { *data = pt_data[VEL_YAW_0x5005_AIR]; return true; }

    int32_t pt_climbspeed = roundf(vfr_hud.climb * 10.0f); // m/s -> dm/s
    int32_t pt_groundspeed = roundf(vfr_hud.airspeed * 10.0f); // m/s -> dm/s
    int32_t pt_yaw = vfr_hud.heading * 5; // deg -> 0.2 deg

    *data = 0;
    pt_pack32(data, prep_number(pt_climbspeed, 2, 1), 0, 8+1); // sign bit
    pt_pack32(data, prep_number(pt_groundspeed, 2, 1), 9, 8);
    pt_pack32(data, pt_yaw, 17, 11);
    pt_pack32(data, 1, 28, 1); // mark it as airspeed

    return true;
}


bool tPassThrough::get_AttitudeRange_0x5006(uint32_t* data)
{
    if (!pt_update[ATTITUDE_RANGE_0x5006]) return false;
    pt_update[ATTITUDE_RANGE_0x5006] = false;

    if (passthrough_array_is_receiving) { *data = pt_data[ATTITUDE_RANGE_0x5006]; return true; }

    int32_t pt_roll = roundf((RAD2DEGF * attitude.roll * 5.0f) + 900.0f); // rad -> 0.2 deg
    int32_t pt_pitch = roundf((RAD2DEGF * attitude.pitch * 5.0f) + 450.0f); // rad ->  0.2 deg
    int32_t pt_dist = roundf(rangefinder.distance * 100.0f); // m -> cm

    *data = 0;
    pt_pack32(data, pt_roll, 0, 11);
    pt_pack32(data, pt_pitch, 11, 10);
    pt_pack32(data, prep_number(pt_dist, 3, 1), 21, 11);

    return true;
}


// since it is linked to heartbeat, it is implicitly called every 1 Hz
bool tPassThrough::get_Param_0x5007(uint32_t* data)
{
    if (!pt_update[PARAM_0x5007]) return false;
    pt_update[PARAM_0x5007] = false;

    if (passthrough_array_is_receiving) { *data = pt_data[PARAM_0x5007]; return true; }

    uint32_t pt_param_id = param_id_to_send;

    param_id_to_send++;
    if (param_id_to_send > 4) param_id_to_send = 1;

    uint32_t pt_param_value = 0;
    switch (pt_param_id) {
    case 1:
        pt_param_value = heartbeat.type;
        break;
    default:
        return false;
    }

    *data = 0;
    pt_pack32(data, pt_param_value, 0, 24);
    pt_pack32(data, pt_param_id, 24, 4);

    return true;
}


bool tPassThrough::get_Battery2_0x5008(uint32_t* data)
{
    if (!pt_update[BATT_2_0x5008]) return false;
    pt_update[BATT_2_0x5008] = false;

    if (passthrough_array_is_receiving) { *data = pt_data[BATT_2_0x5008]; return true; }

    int32_t voltage = mav_battery_voltage(&battery_status_id1);
    int32_t current = battery_status_id1.current_battery;
    int32_t consumed = battery_status_id1.current_consumed;

    int32_t pt_volt = voltage / 100; // mV -> dV
    int32_t pt_curr = (current < 0) ? 0 : current / 10; // cA -> dA // -1 = invalid
    int32_t pt_mAh = (consumed > INT16_MAX) ? INT16_MAX : consumed; // mAh -> mAh // range is limited to [0..INT16_MAX]

    *data = 0;
    pt_pack32(data, pt_volt, 0, 9);
    pt_pack32(data, prep_number(pt_curr, 2, 1), 9, 8);
    pt_pack32(data, pt_mAh, 17, 15);

    return true;
}


bool tPassThrough::get_Rpm_0x500A(uint32_t* data)
{
    if (!pt_update[RPM_0x500A]) return false;
    pt_update[RPM_0x500A] = false;

    if (passthrough_array_is_receiving) { *data = pt_data[RPM_0x500A]; return true; }

    *data = 0;

    return true;
}


bool tPassThrough::get_Terrain_0x500B(uint32_t* data)
{
    if (!pt_update[TERRAIN_0x500B]) return false;
    pt_update[TERRAIN_0x500B] = false;

    if (passthrough_array_is_receiving) { *data = pt_data[TERRAIN_0x500B]; return true; }

    *data = 0;

    return true;
}


bool tPassThrough::get_Wind_0x500C(uint32_t* data)
{
    if (!pt_update[WIND_0x500C]) return false;
    pt_update[WIND_0x500C] = false;

    if (passthrough_array_is_receiving) { *data = pt_data[WIND_0x500C]; return true; }

    *data = 0;

    return true;
}


bool tPassThrough::get_WayPointV2_0x500D(uint32_t* data)
{
    if (!pt_update[WAYPOINT_V2_0x500D]) return false;
    pt_update[WAYPOINT_V2_0x500D] = false;

    if (passthrough_array_is_receiving) { *data = pt_data[WAYPOINT_V2_0x500D]; return true; }

    *data = 0;

    return true;
}


bool tPassThrough::get_VfrHud_0x50F2(uint32_t* data)
{
    if (!pt_update[VFR_HUD_0x50F2]) return false;
    pt_update[VFR_HUD_0x50F2] = false;

    if (passthrough_array_is_receiving) { *data = pt_data[VFR_HUD_0x50F2]; return true; }

    int32_t pt_airspeed = roundf(vfr_hud.airspeed * 10.0f); // m/s -> dm/s
    int32_t pt_throttle = vfr_hud.throttle; // 0...100
    int32_t pt_altitude_MSL = roundf(vfr_hud.alt * 10.0f); // m -> dm

    *data = 0;
    pt_pack32(data, prep_number(pt_airspeed, 2, 1), 0, 8);
    pt_pack32(data, pt_throttle, 8, 7);
    pt_pack32(data, prep_number(pt_altitude_MSL, 3, 2), 15, 12+1); // + sign bit

    return true;
}


//-------------------------------------------------------
// Main Functions

bool tPassThrough::get_packet_data(uint8_t packet_type, uint32_t* data)
{
    switch (packet_type) {
    case TEXT_0x5000: return get_Text_0x5000(data);
    case AP_STATUS_0x5001: return get_ApStatus_0x5001(data);
    case GPS_STATUS_0x5002: return get_GpsStatus_0x5002(data);
    case BATT_1_0x5003: return get_Battery1_0x5003(data);
    case HOME_0x5004: return get_Home_0x5004(data);
    case VEL_YAW_0x5005: return get_VelocityYaw_0x5005(data);
    case ATTITUDE_RANGE_0x5006: return get_AttitudeRange_0x5006(data);
    case PARAM_0x5007: return get_Param_0x5007(data);
    case BATT_2_0x5008: return get_Battery2_0x5008(data);
    // skip WAYPOINT_V1_0x5009, deprecated
    case RPM_0x500A: return get_Rpm_0x500A(data);
    case TERRAIN_0x500B: return get_Terrain_0x500B(data);
    case WIND_0x500C: return get_Wind_0x500C(data);
    case WAYPOINT_V2_0x500D:  return get_WayPointV2_0x500D(data);
    // skip SERVO_OUTPUT_RAW_0x50F1, deprecated
    // case VFR_HUD_0x50F2: return get_VfrHud_0x50F2(data); // we do not use it
    // skip RSSI_0xF101, deprecated
    case VEL_YAW_0x5005_AIR: return get_VelocityYaw_0x5005_Air(data);
    };

    *data = 0;
    return false;
}


bool tPassThrough::GetTelemetryFrameSingle(uint8_t packet_type, uint8_t* data, uint8_t* len)
{
//pt_updated[packet_type] = true;

    uint32_t d;
    if (!get_packet_data(packet_type, &d)) return false;

    tCrsfPassthroughSingle p;

    p.sub_type = CRSF_AP_CUSTOM_TELEM_TYPE_SINGLE_PACKET_PASSTHROUGH;
    p.packet_type = pt_id[packet_type];
    p.data = d;

    memcpy(data, &p, CRSF_PASSTHROUGH_SINGLE_LEN);
    *len = CRSF_PASSTHROUGH_SINGLE_LEN;

    return true;
}


// this captures up to nine passthrough frames
bool tPassThrough::GetTelemetryFrameMulti(uint8_t* data, uint8_t* len)
{
    // current stable Yaapu lua script wants to see more than 8 packets in the multi
    // dev version does not
    // we should send round robin
    // but for the moment we just go through them, the chances should be good enough

    tCrsfPassthroughMulti pm;
    pm.sub_type = CRSF_AP_CUSTOM_TELEM_TYPE_MULTI_PACKET_PASSTHROUGH;
    pm.count = 0;

    uint32_t pd;

    if (get_ApStatus_0x5001(&pd)) {
        pm.packet[pm.count].packet_type = pt_id[AP_STATUS_0x5001];
        pm.packet[pm.count].data = pd;
        pm.count++;
    }

    if (get_VelocityYaw_0x5005(&pd)) {
        pm.packet[pm.count].packet_type = pt_id[VEL_YAW_0x5005];
        pm.packet[pm.count].data = pd;
        pm.count++;
    }
    if (get_VelocityYaw_0x5005_Air(&pd)) {
        pm.packet[pm.count].packet_type = pt_id[VEL_YAW_0x5005_AIR];
        pm.packet[pm.count].data = pd;
        pm.count++;
    }

    if (get_AttitudeRange_0x5006(&pd)) {
        pm.packet[pm.count].packet_type = pt_id[ATTITUDE_RANGE_0x5006];
        pm.packet[pm.count].data = pd;
        pm.count++;
    }

    if (get_GpsStatus_0x5002(&pd)) {
        pm.packet[pm.count].packet_type = pt_id[GPS_STATUS_0x5002];
        pm.packet[pm.count].data = pd;
        pm.count++;
    }

    if (get_GpsLat_0x800(&pd)) {
        pm.packet[pm.count].packet_type = pt_id[GPS_LAT_0x800];
        pm.packet[pm.count].data = pd;
        pm.count++;
    }
    if (get_GpsLon_0x800(&pd)) {
        pm.packet[pm.count].packet_type = pt_id[GPS_LON_0x800];
        pm.packet[pm.count].data = pd;
        pm.count++;
    }

    if ((pm.count < CRSF_PASSTHROUGH_MULTI_COUNT_MAX) && get_Battery1_0x5003(&pd)) {
        pm.packet[pm.count].packet_type = pt_id[BATT_1_0x5003];
        pm.packet[pm.count].data = pd;
        pm.count++;
    }

    if ((pm.count < CRSF_PASSTHROUGH_MULTI_COUNT_MAX) && get_Battery2_0x5008(&pd)) {
        pm.packet[pm.count].packet_type = pt_id[BATT_2_0x5008];
        pm.packet[pm.count].data = pd;
        pm.count++;
    }

    if ((pm.count < CRSF_PASSTHROUGH_MULTI_COUNT_MAX) && get_Home_0x5004(&pd)) {
        pm.packet[pm.count].packet_type = pt_id[HOME_0x5004];
        pm.packet[pm.count].data = pd;
        pm.count++;
    }

    if ((pm.count < CRSF_PASSTHROUGH_MULTI_COUNT_MAX) && get_Param_0x5007(&pd)) {
        pm.packet[pm.count].packet_type = pt_id[PARAM_0x5007];
        pm.packet[pm.count].data = pd;
        pm.count++;
    }

    if ((pm.count < CRSF_PASSTHROUGH_MULTI_COUNT_MAX) && get_Text_0x5000(&pd)) {
        pm.packet[pm.count].packet_type = pt_id[TEXT_0x5000];
        pm.packet[pm.count].data = pd;
        pm.count++;
    }

    if (!pm.count) return false; // nothing to send

    *len = 2 + pm.count * 6;
    memcpy(data, &pm, *len);
    return true;
}


#endif // PASSTHROUGH_PROTOCOL_H


