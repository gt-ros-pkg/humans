#!/bin/bash

# Initialize analog camera for NTSC mode.
# You may need to install v4l-utils for this command...
v4l2-ctl -s ntsc --device=1
#v4l2-ctl -s ntsc --device=0

# Ensure that ttyUSB serial ports are writable
sudo chmod 666 /dev/ttyUSB*

if [ "$1" == "--enable-sonar" ]; then
    # Launch the VideoRay ROS Package with the BlueView Sonar nodes
    roslaunch videoray deploy_with_sonar.launch
else
    # Launch the VideoRay ROS Package
    roslaunch videoray deploy.launch
fi
