cmake_minimum_required(VERSION 2.8.3)

project(rqt_compass)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp syllo_common)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

# Find Qwt
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules/")
find_package(Qwt REQUIRED)

include(${QT_USE_FILE})
include(${QWT_INCLUDE_DIRS})

set(rqt_compass_SRCS
  src/rqt_compass/compass.cpp
  src/rqt_compass/ratio_layouted_frame.cpp
)

set(rqt_compass_HDRS
  include/rqt_compass/compass.h
  include/rqt_compass/ratio_layouted_frame.h
)

set(rqt_compass_UIS
  src/rqt_compass/compass.ui
)

set(rqt_compass_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${QWT_INCLUDE_DIRS}
)

catkin_package(
	INCLUDE_DIRS ${rqt_compass_INCLUDE_DIRECTORIES}
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS rqt_gui rqt_gui_cpp syllo_common
)
catkin_python_setup()

qt4_wrap_cpp(rqt_compass_MOCS ${rqt_compass_HDRS})
qt4_wrap_ui(rqt_compass_UIS_H ${rqt_compass_UIS})

include_directories(${rqt_compass_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_compass_SRCS} ${rqt_compass_MOCS} ${rqt_compass_UIS_H})
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${QT_QTCORE_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QWT_LIBRARIES}
)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
 
install(PROGRAMS scripts/rqt_compass
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
