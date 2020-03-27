#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <vector>
#include <iostream>
#include <chrono>
#include <time.h>
#include <mavconn/interface.h>

#include "mavconn_msg.h"

#define MAV 1

using namespace std;
using namespace cv;

using namespace mavconn;
using namespace mavlink;

using namespace std::chrono;

namespace {
const char* about = "Pose estimation using T265 VIO stereo camera";
const char* keys  =
        "{lp       |       | File with launch parameters }";
}

Matx33d quaternionToRotationMatrix(rs2_quaternion& q)
{
    // Set the matrix as column-major for convenient work with OpenGL and rotate by 180 degress (by negating 1st and 3rd columns) <- NO
    Matx33d mat((1 - 2 * q.y*q.y - 2 * q.z*q.z),  (2 * q.x*q.y - 2 * q.z*q.w),      (2 * q.x*q.z + 2 * q.y*q.w),
                (2 * q.x*q.y + 2 * q.z*q.w),      (1 - 2 * q.x*q.x - 2 * q.z*q.z),  (2 * q.y*q.z - 2 * q.x*q.w),
                (2 * q.x*q.z - 2 * q.y*q.w),      (2 * q.y*q.z + 2 * q.x*q.w),      (1 - 2 * q.x*q.x - 2 * q.y*q.y));

    return mat;
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Matx33d &R)
{
    Matx33d Rt;
    transpose(R, Rt);
    Matx33d shouldBeIdentity = Rt * R;
    Matx33d I = Matx33d::eye();

    return  norm(I, shouldBeIdentity) < 2e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3d rotationMatrixToEulerAngles(Matx33d &R)
{
    assert(isRotationMatrix(R));

    float sy = sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R(2,1) , R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2(R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return Vec3d(x, y, z);
}

int main(int argc, char *argv[])
{
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if (argc < 2) {
        parser.printMessage();
        return 0;
    }

    String filename = parser.get<string>("lp");
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened()) {
        cerr << "Invalid launch file" << endl;
        return 0;
    }

    int camera_orientation = fs["camera_orientation"];
    int pose_msg_rate = fs["pose_msg_rate"];
    int vision_gps_msg = fs["vision_gps_msg"];
    float offset_x = fs["offset_x"];
    float offset_y = fs["offset_y"];
    float offset_z = fs["offset_z"];
    float scale_factor = fs["scale_factor"];
    String mavconn_url = fs["mavconn_url"];

    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }

#ifdef MAV
    // Mavlink Interface
    MAVConnInterface::Ptr client;

	// Mavlink connection from url
    client = MAVConnInterface::open_url(mavconn_url, 1, 240);

    // Mavlink message receive callback
	client->message_received_cb = [&](const mavlink_message_t * msg, const Framing framing) {
		int msgid = int(msg->msgid);

		if (msgid == mavlink::common::msg::STATUSTEXT::MSG_ID) {
			mavlink::common::msg::STATUSTEXT stt {};
			mavlink::MsgMap map(msg);

			stt.deserialize(map);
			cout << "STATUSTEXT: " << to_string(stt.text) << endl;
		} else if (msgid == mavlink::common::msg::TIMESYNC::MSG_ID) {
            mavlink::common::msg::TIMESYNC tms {};
            mavlink::MsgMap map(msg);

            tms.deserialize(map);

            uint64_t now_ns = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();

            // TODO: timesync handling
            if (tms.tc1 == 0) {
                send_timesync(client.get(), now_ns, tms.ts1);
                //cout << "TIMESYNC tc: " << to_string(tms.tc1) <<  " ts: " << to_string(tms.ts1) << endl;
            } else if (tms.tc1 > 0) {
                //cout << "TIMESYNC offset: " << to_string((tms.ts1 + now_ns - tms.tc1) / 2) << " tc: " << to_string(tms.tc1) << " ts: " << to_string(tms.ts1) << endl;
                //cout << "TIMESYNC RTT: " << to_string(now_ns - tms.ts1) << " tc: " << to_string(tms.tc1) << " ts: " << to_string(tms.ts1) << endl;
            }
        }
	};
#endif // MAV

    bool first = true;

    u_int64_t now_micros = 0;
    u_int64_t now_nanos = 0;
    double now, prev_send_pose, prev_heartbeat;
    double prev_iter = 0.0;
    float dt = 0.0;

    auto now_epoch = system_clock::now().time_since_epoch();
    now_micros = duration_cast<microseconds>(now_epoch).count();
    now = (double)now_micros / 1000000.0;
    prev_send_pose = (double)now_micros / 1000000.0;
    prev_heartbeat = (double)now_micros / 1000000.0;

    double pose_msg_period = 1.0 / (double)pose_msg_rate;

    Vec3d rvec = Vec3d(0.0, 0.0, 0.0);
    Vec3d tvec = Vec3d(0.0, 0.0, 0.0);
    Vec3d rvec_prev = Vec3d(0.0, 0.0, 0.0);
    Vec3d tvec_prev = Vec3d(0.0, 0.0, 0.0);
    Vec3d tvec_vel = Vec3d(0.0, 0.0, 0.0);
    Vec3d tvec_vel_prev = Vec3d(0.0, 0.0, 0.0);
    Vec3d tvec_vel_filt = Vec3d(0.0, 0.0, 0.0);

    Affine3d H_T265Ref_T265body;
    Affine3d H_aeroRef_T265Ref;
    Affine3d H_T265body_aeroBody;
    Affine3d H_aeroRef_aeroBody;
    Affine3d H_body_camera;
    Affine3d H_camera_body;

    Vec3d TraVec;
    Vec3d VelVec;
    Vec3d VelVecPrev;
    Vec3d VelVecFilt;
    Matx33d RotMat;

    float yaw_deg = 0.0;
    uint16_t yaw_cd = 36000;

    RotMat = Matx33d(0.0,  0.0, -1.0, 
                     1.0,  0.0,  0.0, 
                     0.0, -1.0,  0.0);

    H_aeroRef_T265Ref = Affine3d(RotMat, Vec3d(0.0, 0.0, 0.0));

    if (camera_orientation == 0) {
        // Forward, USB port to the right
        H_T265body_aeroBody = H_aeroRef_T265Ref.inv();
    }
    else if (camera_orientation == 1) {
        // Downfacing, USB port to the right
        RotMat = Matx33d(0.0, 1.0, 0.0, 
                         1.0, 0.0, 0.0, 
                         0.0, 0.0,-1.0);

        H_T265body_aeroBody = Affine3d(RotMat, Vec3d(0.0, 0.0, 0.0));
    }
    else if (camera_orientation == 2) {
        // 45degree forward
        RotMat = Matx33d(0.0       ,  1.0,  0.0       ,
                        -0.70710676, -0.0, -0.70710676,
                        -0.70710676,  0.0,  0.70710676);

        H_T265body_aeroBody = Affine3d(RotMat, Vec3d(0.0, 0.0, 0.0));
    }
    else {
        // Default
        H_T265body_aeroBody = H_aeroRef_T265Ref.inv();
    }

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    pipe.start(cfg);

    // Main loop
    while (true)
    {
        now_epoch = system_clock::now().time_since_epoch();
        now_micros = duration_cast<microseconds>(now_epoch).count();
        now_nanos = duration_cast<nanoseconds>(now_epoch).count();
        now = (double)now_micros / 1000000.0;

        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Retrieve the pose frame
        auto pose = frames.get_pose_frame();
        
        if (pose) {
            // Retrieve the pose data from T265 position tracking sensor
            auto pose_data = pose.get_pose_data();

            // ******************
            // Transform
            // ******************

            TraVec = Vec3d(pose_data.translation.x, pose_data.translation.y, pose_data.translation.z);
            RotMat = quaternionToRotationMatrix(pose_data.rotation);
            H_T265Ref_T265body = Affine3d(RotMat, TraVec);

            H_aeroRef_aeroBody = H_aeroRef_T265Ref * H_T265Ref_T265body * H_T265body_aeroBody;

            // ******************
            // Offset
            // ******************

            H_body_camera = Affine3d();   // Default constructor. It represents a 4x4 identity matrix.
            H_body_camera.translation(Vec3d(offset_x, offset_y, offset_z));
            H_camera_body = H_body_camera.inv();

            H_aeroRef_aeroBody = H_body_camera * H_aeroRef_aeroBody * H_camera_body;

            // ******************
            // Pose for msg
            // ******************

            RotMat = H_aeroRef_aeroBody.rotation();
            tvec = H_aeroRef_aeroBody.translation();
            rvec = rotationMatrixToEulerAngles(RotMat);

            // ****************************
            // Transform velocity from T265
            // ****************************

            VelVec = Vec3d(pose_data.velocity.x, pose_data.velocity.y, pose_data.velocity.z);
            H_T265Ref_T265body = Affine3d();   // Default constructor. It represents a 4x4 identity matrix.
            H_T265Ref_T265body.translation(VelVec);

            H_aeroRef_aeroBody = H_aeroRef_T265Ref * H_T265Ref_T265body * H_T265body_aeroBody;

            VelVec = H_aeroRef_aeroBody.translation();

            // ***********************************************
            // Yaw in cdeg and velocity from position derivate
            // ***********************************************

            yaw_deg = rvec[2] * 180.0 / M_PI;
            yaw_deg = (yaw_deg < 0.0) ? yaw_deg + 360.0 : yaw_deg;
            yaw_deg = (yaw_deg >= 360.0) ? yaw_deg - 360.0 : yaw_deg;

            yaw_cd = (int)(yaw_deg * 100);
            if (yaw_cd == 0)
                yaw_cd = 36000;

            if (prev_iter > 0.0) {
                dt = (now - prev_iter);
            } else {
                dt = 0.0;
            }
            prev_iter = now;

            if (dt > 0.0) {
                tvec_vel = (tvec - tvec_prev) / dt;
            } else {
                tvec_vel = Vec3d(0.0, 0.0, 0.0);
            }
            
            if (first) {
                tvec_vel_prev = tvec_vel;
                VelVecPrev = VelVec;
                first = false;
            }

            tvec_vel_filt = tvec_vel_prev * 0.9 + tvec_vel * 0.1;
            VelVecFilt = VelVecPrev * 0.75 + VelVec * 0.25;

            tvec_prev = tvec;
            rvec_prev = rvec;

            tvec_vel_prev = tvec_vel_filt;
            VelVecPrev = VelVecFilt;

            if ((now - prev_send_pose) > pose_msg_period) {
                //cout << "Tick Send Pose = " << (now - prev_send_pose) << endl;
                prev_send_pose = now;

                //cout << tvec_vel_filt[0] << " " << tvec_vel_filt[1] << " " << tvec_vel_filt[2] << " " 
                //        << VelVec[0] << " " << VelVec[1] << " " << VelVec[2] << " " 
                //        << VelVecFilt[0] << " " << VelVecFilt[1] << " " << VelVecFilt[2] << endl;
#ifdef MAV
                if (vision_gps_msg == 1) {
                    send_vision_position_estimate(client.get(), tvec, rvec, now_micros);
                    //cout << "R: " << rvec[0] << " P: " << rvec[1] << " Y: " << rvec[2] <<  endl;
                }
                else if (vision_gps_msg == 2) {
                    send_gps_input(client.get(), now_micros, tvec, VelVecFilt, yaw_cd);
                    //send_gps_input(client.get(), now_micros, tvec, tvec_vel_filt, yaw_cd);
                }
#endif
            }
        }

        if ((now - prev_heartbeat) > 1.0) {
            cout << "Tick Heart Beat = " << (now - prev_heartbeat) << endl;
            prev_heartbeat = now;
#ifdef MAV
			send_heartbeat(client.get());
            send_system_time(client.get(), now_micros);
            send_timesync(client.get(), 0, now_nanos);
			send_gps_global_origin(client.get());
			send_set_home_position(client.get());
#endif
            if (pose) {
                cout << "tra:" << tvec << endl;
                cout << "rot:" << rvec << endl;
            }
        }
    }

    return EXIT_SUCCESS;
}
