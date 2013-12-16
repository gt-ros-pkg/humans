syllo_rqt ROS package
=============

Description
-------------
A collection of rqt GUI plugins.


Authors
-------------
Kevin DeMarco <demarco@gatech.edu> http://www.kevindemarco.com


Notes
-----------------
If you know you built your rqt plugin correctly, but it's not appearing,
try to start rqt with the force discover option, so it doesn't rely on it's
cached values.

$ rqt --force-discover


Packages
==================

rqt_autopilot
-----------------

A simple GUI that allows the user to set the desired heading and desired depth
of the VideoRay Pro 4's internal controller.

rqt_blueview
-----------------

A control GUI for the BlueView 2D Imaging sonar that allows the user to set
maximum and minimum sonar ranges, amplitude threshold, and log raw sonar data.

rqt_compass
-----------------

A simple GUI for displaying the vehicle's compass heading.

rqt_rov_status
-----------------

A GUI that displays the ROV's health status in terms of temperatures, operating
voltages, humidity, etc.

rqt_thrust_monitor
-----------------

A GUI that monitors the joystick's thruster command values.
