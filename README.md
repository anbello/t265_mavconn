# t265_mavconn
t265_mavconn is the porting to C/C++ of the Python script written by Thien Nguyen (@LuckyBird) to send VISION_POSITION_ESTIMATE messages from the T265 VIO device (https://www.intelrealsense.com/tracking-camera-t265) to Flight Controller with ArduPilot after transforming the reference frame to NED.

Mavlink communications are managed through MAVCONN library (libmavconn - https://github.com/mavlink/mavros/tree/master/libmavconn).

The work done by Thien Nguyen is documented in this wiki page (https://ardupilot.org/copter/docs/common-vio-tracking-camera.html).

More info on this blog post:
https://discuss.ardupilot.org/t/using-mavconn-library-for-sending-non-gps-navigation-messages-in-c-c/54105
