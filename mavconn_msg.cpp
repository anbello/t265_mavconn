#include <iostream>
#include <chrono>

#include <time.h>
#include <mavconn/interface.h>
#include <opencv2/core/matx.hpp>

#include "mavconn_msg.h"

using namespace mavconn;
using namespace mavlink;

void send_heartbeat(MAVConnInterface *ip) {
	using mavlink::common::MAV_TYPE;
	using mavlink::common::MAV_AUTOPILOT;
	using mavlink::common::MAV_MODE;
	using mavlink::common::MAV_STATE;

	mavlink::common::msg::HEARTBEAT hb {};
	hb.type = int(MAV_TYPE::ONBOARD_CONTROLLER);
	hb.autopilot = int(MAV_AUTOPILOT::INVALID);
	hb.base_mode = int(MAV_MODE::MANUAL_ARMED);
	hb.custom_mode = 0;
	hb.system_status = int(MAV_STATE::ACTIVE);

	ip->send_message_ignore_drop(hb);
}

void send_gps_global_origin(MAVConnInterface *ip) {
	mavlink_message_t msg {};
	mavlink::MsgMap map(msg);

	mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN ggo {};

	int32_t lat = 425633500;   // Terni
	int32_t lon = 126432900;   // Terni
	int32_t alt = 163000;      // Terni

	ggo.latitude = lat;
	ggo.longitude = lon;	
	ggo.altitude = alt;

	ggo.serialize(map);
	mavlink::mavlink_finalize_message(&msg, ip->get_system_id(), ip->get_component_id(), ggo.MIN_LENGTH, ggo.LENGTH, ggo.CRC_EXTRA);

	ip->send_message_ignore_drop(&msg);
}

void send_set_home_position(MAVConnInterface *ip) {
	mavlink_message_t msg {};
	mavlink::MsgMap map(msg);

	mavlink::common::msg::SET_HOME_POSITION shp {};

	int32_t lat = 425633500;   // Terni
	int32_t lon = 126432900;   // Terni
	int32_t alt = 163000;      // Terni
	//array<float, 4> q = {1, 0, 0, 0};   // w x y z

	shp.latitude = lat;
	shp.longitude = lon;	
	shp.altitude = alt;
	shp.x = 0.0;
	shp.y = 0.0;
	shp.z = 0.0;
	shp.q = {1.0, 0.0, 0.0, 0.0};   // w x y z
	shp.approach_x = 0.0;
	shp.approach_y = 0.0;
	shp.approach_z = 1.0;

	shp.serialize(map);
	mavlink::mavlink_finalize_message(&msg, ip->get_system_id(), ip->get_component_id(), shp.MIN_LENGTH, shp.LENGTH, shp.CRC_EXTRA);

	ip->send_message_ignore_drop(&msg);
}

void send_vision_position_estimate(MAVConnInterface *ip, Vec3d tra, Vec3d rot, uint64_t micros) {
	mavlink_message_t msg {};
	mavlink::MsgMap map(msg);

	mavlink::common::msg::VISION_POSITION_ESTIMATE vpe {};

    vpe.usec = micros;
	vpe.x = tra[0];
	vpe.y = tra[1];
	vpe.z = tra[2];
	vpe.roll = rot[0];
	vpe.pitch = rot[1];
	vpe.yaw = rot[2];

	vpe.serialize(map);
	mavlink::mavlink_finalize_message(&msg, ip->get_system_id(), ip->get_component_id(), vpe.MIN_LENGTH, vpe.LENGTH, vpe.CRC_EXTRA);

	ip->send_message_ignore_drop(&msg);
}

void send_timesync(MAVConnInterface *ip, uint64_t tc1, uint64_t ts1) {
	mavlink_message_t msg {};
	mavlink::MsgMap map(msg);

	mavlink::common::msg::TIMESYNC tms {};

    tms.tc1 = tc1;
    tms.ts1 = ts1;

	tms.serialize(map);
	mavlink::mavlink_finalize_message(&msg, ip->get_system_id(), ip->get_component_id(), tms.MIN_LENGTH, tms.LENGTH, tms.CRC_EXTRA);

	ip->send_message_ignore_drop(&msg);
}

void send_system_time(MAVConnInterface *ip, uint64_t time_unix_usec) {
	mavlink_message_t msg {};
	mavlink::MsgMap map(msg);

	mavlink::common::msg::SYSTEM_TIME smt {};

    smt.time_unix_usec = time_unix_usec;

	smt.serialize(map);
	mavlink::mavlink_finalize_message(&msg, ip->get_system_id(), ip->get_component_id(), smt.MIN_LENGTH, smt.LENGTH, smt.CRC_EXTRA);

	ip->send_message_ignore_drop(&msg);
}

void send_gps_input(MAVConnInterface *ip, uint64_t time_unix_usec, Vec3d pos_ned, Vec3d vel_ned, uint16_t yaw_cd) {
	mavlink_message_t msg {};
	mavlink::MsgMap map(msg);

	mavlink::common::msg::GPS_INPUT gps {};

    float north = pos_ned[0];
    float east = pos_ned[1];
    float down = pos_ned[2];

    float vn = vel_ned[0];
    float ve = vel_ned[1];
    float vd = vel_ned[2];

	// gps_time: tnow -> (week, week_ms)
    double tnow = (double)time_unix_usec / 1000000.0;
    uint32_t epoch = 86400*(10*365 + (1980-1969)/4 + 1 + 6 - 2) - leapseconds;
    uint32_t epoch_seconds = (uint32_t)tnow - epoch;
    uint16_t week = (uint16_t)floor(epoch_seconds / sec_per_week);
    uint32_t t_ms = (uint32_t)(tnow * 1000) % 1000;
    uint32_t week_ms = (epoch_seconds % sec_per_week) * 1000 + ((uint32_t)floor(t_ms/200) * 200);

	// gps_offset: (east, north) -> (gps_lat, gps_lon)
    double bearing = atan2(east, north) * 180.0 / M_PI;
    double distance = sqrt(east * east + north * north);

    double lat1 = origin_lat * M_PI / 180.0;
    double lon1 = origin_lon * M_PI / 180.0;
    double brng = bearing * M_PI / 180.0;
    double dr = distance / radius_of_earth;

    double lat2 = asin(sin(lat1) * cos(dr) + cos(lat1) * sin(dr) * cos(brng));
    double lon2 = lon1 + atan2(sin(brng) * sin(dr) * cos(lat1), cos(dr) - sin(lat1) * sin(lat2));
    
    double gps_lat = lat2 * 180.0 / M_PI;
    double gps_lon = lon2 * 180.0 / M_PI;
    gps_lon = fmod((gps_lon + 180.0), 360.0) - 180.0;
    double gps_alt = origin_alt - down;


    gps.time_usec = time_unix_usec;
    gps.gps_id = 0;
    gps.ignore_flags = 0;
    gps.time_week_ms = week_ms;   // uint32_t
    gps.time_week = week;   // uint16_t
    gps.fix_type = 3;
    gps.lat = (int32_t)(gps_lat * 1.0e7);
    gps.lon = (int32_t)(gps_lon * 1.0e7);
    gps.alt = gps_alt;
    gps.hdop = 1.0;
    gps.vdop = 1.0;
    gps.vn = vn;
    gps.ve = ve;
    gps.vd = vd;
    gps.speed_accuracy = 0.2;
    gps.horiz_accuracy = 1.0;
    gps.vert_accuracy = 1.0;
    gps.satellites_visible = 16;
    gps.yaw = yaw_cd;

    //cout << "lat: " << gps.lat << " lon: " << gps.lon << endl;;

	gps.serialize(map);
	mavlink::mavlink_finalize_message(&msg, ip->get_system_id(), ip->get_component_id(), gps.MIN_LENGTH, gps.LENGTH, gps.CRC_EXTRA);

	ip->send_message_ignore_drop(&msg);
}
