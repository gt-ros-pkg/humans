cmake_minimum_required(VERSION 2.8.3)

project(rqt_rov_status)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp videoray)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(rqt_rov_status_SRCS
  src/rqt_rov_status/rov_status.cpp
  src/rqt_rov_status/ratio_layouted_frame.cpp
)

set(rqt_rov_status_HDRS
  include/rqt_rov_status/rov_status.h
  include/rqt_rov_status/ratio_layouted_frame.h
)

set(rqt_rov_status_UIS
  src/rqt_rov_status/rov_status.ui
)

set(rqt_rov_status_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
	INCLUDE_DIRS ${rqt_rov_status_INCLUDE_DIRECTORIES}
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS rqt_gui rqt_gui_cpp videoray
)
catkin_python_setup()

qt4_wrap_cpp(rqt_rov_status_MOCS ${rqt_rov_status_HDRS})
qt4_wrap_ui(rqt_rov_status_UIS_H ${rqt_rov_status_UIS})

include_directories(${rqt_rov_status_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_rov_status_SRCS} ${rqt_rov_status_MOCS} ${rqt_rov_status_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
 
install(PROGRAMS scripts/rqt_rov_status
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
