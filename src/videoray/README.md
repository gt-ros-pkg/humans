videoray ROS package
=============

Description
-----------

The videoray ROS packages provides control of the VideoRay Pro 4 and provides
several launch scripts for mission execution.

Authors
-------------
Kevin DeMarco <demarco@gatech.edu> http://www.kevindemarco.com

Quickstart
==========
    1. Build the videoray package and dependent projects
    2. Connect VideoRay's USB devices (USB/DVR camera, RS-485/USB converter,
       joystick input).
    3. Power on the VideoRay.
    4. Start the VideoRay deploy run program:

$ rosrun videoray deploy.sh

If you have a BlueView sonar as well, you can launch the deploy.sh script with
the --enable-sonar argument:

$ rosrun videoray deploy.sh --enable-sonar

You may have to allow up to 30 seconds after powering on the BlueView sonar for
the sonar to be detectable on the network.

Packages
==================

comm
----

The comm package creates a library that allows a ROS node to send high-level
commands (thruster control, light control, camera control) to the VideoRay and
receive sensor data from the VideoRay.

control
-------

The control package uses the comm package to create a ROS node that
communicates directly with the VideoRay over the RS-485 communication
interface. The control node exposes the VideoRay's control inputs and sensor
outputs to other ROS nodes through the publish-and-subscribe interface.

sim
---

The sim package contains several different C++ implementations of the
VideoRay's dynamics for simulation purposes. This package is under heavy
development.

Notes
=====

What is this bash script?
-------------------------

You may be wondering by a bash script is executed with the rosrun command
instead of just using the roslaunch command. The deploy.sh script is used
because it initializes the VideoRay's analog camera for use in Linux and it
ensures that the /dev/ttyUSB* serial devices can be read and written to
(permissions) from the ROS node. I had to do these tasks manually everytime I
unplugged the VideoRay from the computer, but I wanted a single command to
initialize the system and launch the deploy scripts.

What's up with this analog camera?
----------------------------------

In order to get the VideoRay Pro 4's analog camera working in Linux (v4l2
driver) with the use of the KWorld DVR/USB device, the following approach was taken.

First, tell the video for linux driver (v4l2) that the analog camera is NTSC
compliant. You will need the v4l-utils package to issue the command.
       
$ sudo apt-get install v4l-utils
$ v4l2-ctl -s ntsc

Second, use the standard ROS node, gscam, to connect to the camera and begin
publishing camera images. However, you need to set the GSCAM_CONFIG environment
variable before launching gscam, but this can be done in the ROS launch
file. In the case of this camera, the following GSCAM_CONFIG string was used:

"v4l2src device=/dev/video0 ! video/x-raw-rgb,framerate=30/1  ! ffmpegcolorspace"

Which looks like the following in the launch file (it should be placed before
the <node> tag that launches the gscam node).

<env name="GSCAM_CONFIG" value="v4l2src device=/dev/video0 !
video/x-raw-rgb,framerate=30/1 ! ffmpegcolorspace" />

Look at the videoray/launch/camera.launch launch file for a full example.
