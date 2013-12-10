cmake_minimum_required(VERSION 2.8.3)

project(rqt_(>>>APP-NAME<<<))
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(rqt_(>>>APP-NAME<<<)_SRCS
  src/rqt_(>>>APP-NAME<<<)/(>>>APP-NAME<<<).cpp
  src/rqt_(>>>APP-NAME<<<)/ratio_layouted_frame.cpp
)

set(rqt_(>>>APP-NAME<<<)_HDRS
  include/rqt_(>>>APP-NAME<<<)/(>>>APP-NAME<<<).h
  include/rqt_(>>>APP-NAME<<<)/ratio_layouted_frame.h
)

set(rqt_(>>>APP-NAME<<<)_UIS
  src/rqt_(>>>APP-NAME<<<)/(>>>APP-NAME<<<).ui
)

set(rqt_(>>>APP-NAME<<<)_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
	INCLUDE_DIRS ${rqt_(>>>APP-NAME<<<)_INCLUDE_DIRECTORIES}
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS rqt_gui rqt_gui_cpp
)
catkin_python_setup()

qt4_wrap_cpp(rqt_(>>>APP-NAME<<<)_MOCS ${rqt_(>>>APP-NAME<<<)_HDRS})
qt4_wrap_ui(rqt_(>>>APP-NAME<<<)_UIS_H ${rqt_(>>>APP-NAME<<<)_UIS})

include_directories(${rqt_(>>>APP-NAME<<<)_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_(>>>APP-NAME<<<)_SRCS} ${rqt_(>>>APP-NAME<<<)_MOCS} ${rqt_(>>>APP-NAME<<<)_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
 
install(PROGRAMS scripts/rqt_(>>>APP-NAME<<<)
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
