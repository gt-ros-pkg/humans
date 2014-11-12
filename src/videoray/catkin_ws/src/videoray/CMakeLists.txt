cmake_minimum_required(VERSION 2.8.3)
project(videoray)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  message_generation 
  sensor_msgs 
  cv_bridge
  syllo_common
  syllo_serial
  )

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Throttle.msg
  DesiredTrajectory.msg
  AccessoryControl.msg
  Status.msg
  UHRIComm.msg
  Notes.msg
  )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES videoray
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(videoray
#   src/${PROJECT_NAME}/videoray.cpp
# )

## Declare a cpp executable
# add_executable(videoray_node src/videoray_node.cpp)
add_executable(videoray_control src/sim/videoray_control.cpp)
add_executable(videoray_sim src/sim/videoray_sim.cpp)
add_executable(videoray_control_p src/sim/videoray_control_p.cpp)
add_executable(videoray_sim_and_control src/sim/videoray_sim_and_control.cpp)
add_executable(videoray_moos src/sim/videoray_moos.cpp)
add_executable(cam_sim src/sim/cam_sim.cpp)

add_executable(control 
  src/control/main.cpp 
  src/comm/Packetizer.cpp 
  src/comm/VideoRayComm.cpp  
  )

add_executable(camera_NTSC_init
  src/camera/main.cpp
  )


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(videoray_node videoray_generate_messages_cpp)
add_dependencies(videoray_control videoray_generate_messages_cpp videoray_gencpp)
add_dependencies(videoray_sim videoray_generate_messages_cpp videoray_gencpp)
add_dependencies(videoray_sim_and_control videoray_generate_messages_cpp videoray_gencpp)
add_dependencies(videoray_moos videoray_generate_messages_cpp videoray_gencpp)
add_dependencies(videoray_control_p videoray_generate_messages_cpp videoray_gencpp)
add_dependencies(cam_sim videoray_generate_messages_cpp videoray_gencpp)

add_dependencies(control videoray_generate_messages_cpp videoray_gencpp)
add_dependencies(camera_NTSC_init videoray_generate_messages_cpp videoray_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(videoray_control
  ${catkin_LIBRARIES}
)

target_link_libraries(videoray_sim
  ${catkin_LIBRARIES}
)

target_link_libraries(videoray_sim_and_control
  ${catkin_LIBRARIES}
)

target_link_libraries(videoray_moos
  ${catkin_LIBRARIES}
)

target_link_libraries(videoray_control_p
  ${catkin_LIBRARIES}
)

target_link_libraries(control
  ${catkin_LIBRARIES}
)

target_link_libraries(camera_NTSC_init
  ${catkin_LIBRARIES}
)

find_package(OpenCV REQUIRED)

target_link_libraries(cam_sim
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS videoray videoray_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_videoray.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
