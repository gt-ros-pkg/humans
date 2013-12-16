syllo ROS package
=============

Description
------------- 

A collection of utility functions and wrapper classes to decrease development
time.

Authors
-------------
Kevin DeMarco <demarco@gatech.edu> http://www.kevindemarco.com

Packages
==================

syllo_common
-----------------

The syllo_common package contains common tools for euler to quaternion
conversion, filtering, time functions, etc. Also, syllo_common contains the
SylloNode class, which is used as a wrapper for most ROS commands. This is to
reduce the dependency on ROS for SylloNode executables in future development.

All SylloNodes require the definition of a tick_rate parameter in the launch
file for node. The tick_rate defines the rate at which each ROS loop is
executed. Example tick_rate definitions can be found in
blueview/catkin_ws/src/sonar_2d_node/launch/*.launch.

syllo_serial
-----------------

The syllo_serial package implements a very simple serial port C++ library.

syllo_blueview
-----------------

The syllo_blueview package implements a C++ interface to the BlueView SDK. In
order to use this package, you must have access to the BlueView SDK
libraries. Currently, at Georgia Tech, we only have the 32-bit Linux BlueView
SDK libraries, so all of our development is on 32-bit Ubuntu 12.04. You should
make sure that you have the appropriate binaries for your system.

Since BlueView doesn't provide a nice build / discovery system for their SDK,
you will need to provide the location of the root BlueView SDK directory in the
BLUEVIEW_SDK_ROOT environment variable. For example, the BLUEVIEW_SDK_ROOT
variable is commonly defined in the user's .bashrc (or similar) configuration
file in this way:

export BLUEVIEW_SDK_ROOT=/path/to/bvtsdk

In the BlueView SDK library that was delivered to Georgia Tech, the bvtsdk
directory has the following contents: "colormaps" "data" "doc" "examples"
"include" "lib". If those directories aren't present, you will have issues with
building the syllo_blueview library.

The syllo_blueview cmake package was designed such that if you don't have the
BlueView SDK, the rest of the syllo packages that don't depend on
syllo_blueview will be compiled without issues.
    
