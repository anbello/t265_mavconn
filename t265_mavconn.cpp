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

Matx33f quaternionToRotationMatrix(rs2_quaternion& q)
{
    // Set the matrix as column-major for convenient work with OpenGL and rotate by 180 degress (by negating 1st and 3rd columns) <- NO
    Matx33f mat((1 - 2 * q.y*q.y - 2 * q.z*q.z),  (2 * q.x*q.y - 2 * q.z*q.w),      (2 * q.x*q.z + 2 * q.y*q.w),
                (2 * q.x*q.y + 2 * q.z*q.w),      (1 - 2 * q.x*q.x - 2 * q.z*q.z),  (2 * q.y*q.z - 2 * q.x*q.w),
                (2 * q.x*q.z - 2 * q.y*q.w),      (2 * q.y*q.z + 2 * q.x*q.w),      (1 - 2 * q.x*q.x - 2 * q.y*q.y));

    return mat;
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Matx33f &R)
{
    Matx33f Rt;
    transpose(R, Rt);
    Matx33f shouldBeIdentity = Rt * R;
    Matx33f I = Matx33f::eye();

    return  norm(I, shouldBeIdentity) < 2e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Matx33f &R)
{
    // assert(isRotationMatrix(R));

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
    return Vec3f(x, y, z);
}

Vec3f rotationMatrixToEulerAngles2(Matx33f & rotationMatrix)
{
    Vec3f euler;

    double m00 = rotationMatrix(0,0);
    double m02 = rotationMatrix(0,2);
    double m10 = rotationMatrix(1,0);
    double m11 = rotationMatrix(1,1);
    double m12 = rotationMatrix(1,2);
    double m20 = rotationMatrix(2,0);
    double m22 = rotationMatrix(2,2);

    double bank, attitude, heading;

    // Assuming the angles are in radians.
    if (m10 > 0.998) { // singularity at north pole
        bank = 0;
        attitude = CV_PI/2;
        heading = atan2(m02,m22);
    }
    else if (m10 < -0.998) { // singularity at south pole
        bank = 0;
        attitude = -CV_PI/2;
        heading = atan2(m02,m22);
    }
    else
    {
        bank = atan2(-m12,m11);
        attitude = asin(m10);
        heading = atan2(-m20,m00);
    }

    euler(0) = bank;
    euler(2) = attitude;
    euler(1) = heading;

    return euler;
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
    bool send_origin = (int)fs["send_origin"] == 1;

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
    u_int32_t delta_micros = 0;
    double now, prev_send_pose, prev_heartbeat;

    auto now_epoch = system_clock::now().time_since_epoch();
    now_micros = duration_cast<microseconds>(now_epoch).count();
    now = (double)now_micros / 1000000.0;
    prev_send_pose = (double)now_micros / 1000000.0;
    prev_heartbeat = (double)now_micros / 1000000.0;

    double pose_msg_period = 1.0 / (double)pose_msg_rate;

    Vec3f rpyvec = Vec3f(0.0, 0.0, 0.0);
    Vec3f rpyvec2 = Vec3f(0.0, 0.0, 0.0);
    Vec3f xyzvec = Vec3f(0.0, 0.0, 0.0);

    Affine3f H_T265Ref_T265body;
    Affine3f H_aeroRef_T265Ref;
    Affine3f H_T265body_aeroBody;
    Affine3f H_aeroRef_aeroBody;
    Affine3f H_aeroRef_PrevAeroBody;
    Affine3f H_aeroRef_VelAeroBody;
    Affine3f H_body_camera;
    Affine3f H_camera_body;

    Vec3f TraVec = Vec3f(0.0, 0.0, 0.0);
    Vec3f VelVec = Vec3f(0.0, 0.0, 0.0);
    Vec3f deltaRot = Vec3f(0.0, 0.0, 0.0);
    Vec3f deltaTra = Vec3f(0.0, 0.0, 0.0);
    Matx33f RotMat;

    float yaw_deg = 0.0;
    uint16_t yaw_cd = 36000;

    RotMat = Matx33f(0.0,  0.0, -1.0, 
                     1.0,  0.0,  0.0, 
                     0.0, -1.0,  0.0);

    H_aeroRef_T265Ref = Affine3f(RotMat, Vec3f(0.0, 0.0, 0.0));

    if (camera_orientation == 0) {
        // Forward, USB port to the right
        H_T265body_aeroBody = H_aeroRef_T265Ref.inv();
    }
    else if (camera_orientation == 1) {
        // Downfacing, USB port to the right
        RotMat = Matx33f(0.0, 1.0, 0.0, 
                         1.0, 0.0, 0.0, 
                         0.0, 0.0,-1.0);

        H_T265body_aeroBody = Affine3f(RotMat, Vec3f(0.0, 0.0, 0.0));
    }
    else if (camera_orientation == 2) {
        // 45degree forward
        RotMat = Matx33f(0.0       ,  1.0,  0.0       ,
                        -0.70710676, -0.0, -0.70710676,
                        -0.70710676,  0.0,  0.70710676);

        H_T265body_aeroBody = Affine3f(RotMat, Vec3f(0.0, 0.0, 0.0));
    }
    else {
        // Default
        H_T265body_aeroBody = H_aeroRef_T265Ref.inv();
    }

    H_aeroRef_PrevAeroBody = Affine3f();

    double pose_timestamp;
    u_int64_t frame_number;
    float confidence = 0.0;

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
            pose_timestamp = pose.get_timestamp();
            frame_number = pose.get_frame_number();
            confidence = pose_data.tracker_confidence * 100.0 / 3.0;

            // ******************
            // Transform
            // ******************

            TraVec = Vec3f(pose_data.translation.x, pose_data.translation.y, pose_data.translation.z);
            RotMat = quaternionToRotationMatrix(pose_data.rotation);
            H_T265Ref_T265body = Affine3f(RotMat, TraVec);

            H_aeroRef_aeroBody = H_aeroRef_T265Ref * H_T265Ref_T265body * H_T265body_aeroBody;

            // ******************
            // Offset
            // ******************

            H_body_camera = Affine3f();   // Default constructor. It represents a 4x4 identity matrix.
            H_body_camera.translation(Vec3f(offset_x, offset_y, offset_z));
            H_camera_body = H_body_camera.inv();

            H_aeroRef_aeroBody = H_body_camera * H_aeroRef_aeroBody * H_camera_body;

            // ******************
            // Pose for msg
            // ******************

            RotMat = H_aeroRef_aeroBody.rotation();
            xyzvec = H_aeroRef_aeroBody.translation();
            rpyvec = rotationMatrixToEulerAngles(RotMat);
            rpyvec2 = rotationMatrixToEulerAngles2(RotMat);

            // ****************************
            // Transform velocity from T265
            // ****************************

            VelVec = Vec3f(pose_data.velocity.x, pose_data.velocity.y, pose_data.velocity.z);
            H_T265Ref_T265body = Affine3f();   // Default constructor. It represents a 4x4 identity matrix.
            H_T265Ref_T265body.translation(VelVec);

            H_aeroRef_VelAeroBody = H_aeroRef_T265Ref * H_T265Ref_T265body * H_T265body_aeroBody;

            VelVec = H_aeroRef_VelAeroBody.translation();

            // ***********************************************
            // Yaw in cdeg and velocity from position derivate
            // ***********************************************

            yaw_deg = rpyvec[2] * 180.0 / M_PI;
            yaw_deg = (yaw_deg < 0.0) ? yaw_deg + 360.0 : yaw_deg;
            yaw_deg = (yaw_deg >= 360.0) ? yaw_deg - 360.0 : yaw_deg;

            yaw_cd = (int)(yaw_deg * 100);
            if (yaw_cd == 0)
                yaw_cd = 36000;

            if ((now - prev_send_pose) > pose_msg_period) {
                //cout << "Tick Send Pose = " << (now - prev_send_pose) << endl;

                //cout << "fn:" << frame_number << " dt:" << (now - pose_timestamp * 1000.0) << endl;

                //cout << xyzvec_vel_filt[0] << " " << xyzvec_vel_filt[1] << " " << xyzvec_vel_filt[2] << " " 
                //        << VelVec[0] << " " << VelVec[1] << " " << VelVec[2] << " " 
                //        << VelVecFilt[0] << " " << VelVecFilt[1] << " " << VelVecFilt[2] << endl;
#ifdef MAV
                if (vision_gps_msg == 1) {
                    send_vision_position_estimate(client.get(), now_micros, xyzvec, rpyvec);
                    //cout << "R: " << rpyvec[0] << " P: " << rpyvec[1] << " Y: " << rpyvec[2] <<  endl;
                }
                else if (vision_gps_msg == 2) {
                    send_gps_input(client.get(), now_micros, xyzvec, VelVec, yaw_cd);
                }
                else if (vision_gps_msg == 3) {
                    delta_micros = (uint64_t)((now - prev_send_pose) * 1000000.0);

                    Affine3f H_PrevAeroBody_CurrAeroBody = H_aeroRef_PrevAeroBody.inv() * H_aeroRef_aeroBody;
                    deltaTra = H_PrevAeroBody_CurrAeroBody.translation();
                    Matx33f RotM = H_PrevAeroBody_CurrAeroBody.rotation();
                    deltaRot = rotationMatrixToEulerAngles(RotM);

                    send_vision_position_delta(client.get(), now_micros, delta_micros, deltaRot, deltaTra, confidence);
                }
                else if (vision_gps_msg == 4) {
                    delta_micros = (uint64_t)((now - prev_send_pose) * 1000000.0);

                    Affine3f H_PrevAeroBody_CurrAeroBody = H_aeroRef_PrevAeroBody.inv() * H_aeroRef_aeroBody;
                    deltaTra = H_PrevAeroBody_CurrAeroBody.translation();
                    Matx33f RotM = H_PrevAeroBody_CurrAeroBody.rotation();
                    deltaRot = rotationMatrixToEulerAngles(RotM);

                    send_vision_position_delta(client.get(), now_micros, delta_micros, deltaRot, deltaTra, confidence);
                    send_gps_input(client.get(), now_micros, xyzvec, VelVec, yaw_cd);
                }
#endif
                H_aeroRef_PrevAeroBody = H_aeroRef_aeroBody;
                prev_send_pose = now;
            }
        }

        if ((now - prev_heartbeat) > 1.0) {
            cout << "Tick Heart Beat = " << (now - prev_heartbeat) << endl;
            prev_heartbeat = now;
#ifdef MAV
			send_heartbeat(client.get());
            send_system_time(client.get(), now_micros);
            send_timesync(client.get(), 0, now_nanos);
            if (send_origin)
            {
                send_gps_global_origin(client.get());
                send_set_home_position(client.get());
            }
#endif
            if (pose) {
                cout << "tra:" << xyzvec << endl;
                cout << "rot:" << rpyvec << endl;
                cout << "rt2:" << rpyvec2 << endl;
                cout << "fn:" << frame_number << " dt:" << (now - pose_timestamp * 1000.0) << " conf:" << confidence << endl;
            }
        }
    }

    return EXIT_SUCCESS;
}
