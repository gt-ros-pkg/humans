blueview ROS package
=============

Description
-----------

This package implements a ROS node to publish 2D imaging sonar data from a
BlueView sonar.

Authors
-------------
Kevin DeMarco <demarco@gatech.edu> http://www.kevindemarco.com


Packages
==================

sonar_2d_node
-------------

A ROS node that connects to live BlueView sonar over the BlueView SDK ethernet
interface and publishes sonar data in the form of an OpenCV image or a point
cloud (future, soon). The sonar_2d_node can also be used to playback raw sonar
data from previously saved .son files. The sonar_2d_node requires a number of
input parameters, so it is recommended that the sonar node is started with the
help of a launch file. A number of example launch files are provided in the
launch directory, but the following is a description of the required input
parameters.

    * net_or_file : string : This parameter should be set to either "net" or
      "file" depending on whether this node is connecting to a real sonar over
      ethernet ("net") or if the node is playing back a previously recorded
      .son file ("file").
    * ip_addr : string : This parameter can be set to an IP address if the user
      knows the specific IP address of the sonar when the net_or_file variable
      is set to "net". If this parameter is not set or set to "0.0.0.0", then
      the node will automatically try to search for the sonar.
    * sonar_file : string : When net_or_file is set to "file", this parameter
      defines the full path to the .son file for playback.
    * min_dist : double : The minimum sonar distance on startup.
    * max_dist : double : The maximum sonar distance on startup.
    * mode : string : This parameter can be set to "image", "range", or "both"
      to define the type of sonar data to publish. Currently, only "image" is
      supported.
    * color_map : string : The full path of the BlueView SDK color map to use
      when generating the sonar image. For example,
      "/path/to/bvtsdk/colormaps/jet.cmap".
    * save_directory : string : The directory in which raw sonar data is logged
      to when enabled. For example, "~/sonar_log". If the directory doesn't
      exist, it will be created if the user has proper permissions.
