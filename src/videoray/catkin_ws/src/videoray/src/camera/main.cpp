//
// All this node does is call the system command:
// v4l2-ctl -s ntsc
//
// This is required to ensure that the analog camera on the VideoRay Pro 4
// is properly intialized to NTSC mode.
// In the launch script for the camera, camera.launch,
// the GSCAM_CONFIG variable is configured before gscam is launched.
// The GSCAM_CONFIG variable is set to:
// "v4l2src device=/dev/video0 ! video/x-raw-rgb,framerate=30/1  ! ffmpegcolorspace"
//
// The v4l2-ctl command is part of the Ubuntu v4l-utils package.
// $ sudo apt-get install v4l-utils
//

#include "ros/ros.h"
#include <iostream> // stringstream

using std::cout;
using std::endl;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_NTSC_init");

    cout << "Initializing analog camera to NTSC mode..." << endl;

    system("v4l2-ctl -s ntsc");
    
    return 0;
}

