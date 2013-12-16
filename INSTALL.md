HumansLab ROS Repository
--------------------

Installation
--------------------

1. Build all ROS catkin packages:

$ ./build.sh

2. After the build is complete, the build script will output a sequence of
   commands that you should place in your .bashrc (or similar) file to ensure
   that your system can "see" the newly built components. The following is an
   example sequence of commands that you should place in your .bashrc file:

CATKIN_WS1_SETUP=/path/to/gt-ros-pkg.humans/setenv.sh
if [ -f ${CATKIN_WS1_SETUP} ]; then
source ${CATKIN_WS1_SETUP}
fi

