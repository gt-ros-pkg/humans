#!/bin/bash

# Initialize analog camera for NTSC mode.
# You may need to install v4l-utils for this command...
v4l2-ctl -s ntsc

# Ensure that ttyUSB serial ports are writable
sudo chmod 666 /dev/ttyUSB*

# Launch the VideoRay ROS Package with the BlueView Sonar nodes
roslaunch videoray deploy_with_sonar.launch
