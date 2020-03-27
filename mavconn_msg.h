#pragma once

#include <mavconn/interface.h>
#include <opencv2/core/matx.hpp>

using namespace mavconn;
using namespace cv;

const uint16_t leapseconds = 18;
const uint32_t sec_per_week = 7 * 86400;

// int32_t lat = 425633500;   // Terni
// int32_t lon = 126432900;   // Terni
// int32_t alt = 163000;      // Terni

const double origin_lat = 42.56335;
const double origin_lon = 12.64329;
const double origin_alt = 163.0;

const double radius_of_earth = 6378100.0;

void send_heartbeat(MAVConnInterface *ip);
void send_gps_global_origin(MAVConnInterface *ip);
void send_set_home_position(MAVConnInterface *ip);
void send_vision_position_estimate(MAVConnInterface *ip, Vec3d tra, Vec3d rot, uint64_t micros);
void send_timesync(MAVConnInterface *ip, uint64_t tc1, uint64_t ts1);
void send_system_time(MAVConnInterface *ip, uint64_t time_unix_usec);
void send_gps_input(MAVConnInterface *ip, uint64_t time_unix_usec, Vec3d pos_ned, Vec3d vel_ned, uint16_t yaw_cd);
