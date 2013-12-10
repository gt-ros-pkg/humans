cmake_minimum_required(VERSION 2.8.3)

project(rqt_autopilot)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp videoray)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(rqt_autopilot_SRCS
  src/rqt_autopilot/autopilot.cpp
  src/rqt_autopilot/ratio_layouted_frame.cpp
)

set(rqt_autopilot_HDRS
  include/rqt_autopilot/autopilot.h
  include/rqt_autopilot/ratio_layouted_frame.h
)

set(rqt_autopilot_UIS
  src/rqt_autopilot/autopilot.ui
)

set(rqt_autopilot_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
	INCLUDE_DIRS ${rqt_autopilot_INCLUDE_DIRECTORIES}
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS rqt_gui rqt_gui_cpp videoray
)
catkin_python_setup()

qt4_wrap_cpp(rqt_autopilot_MOCS ${rqt_autopilot_HDRS})
qt4_wrap_ui(rqt_autopilot_UIS_H ${rqt_autopilot_UIS})

include_directories(${rqt_autopilot_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_autopilot_SRCS} ${rqt_autopilot_MOCS} ${rqt_autopilot_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
 
install(PROGRAMS scripts/rqt_autopilot
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
