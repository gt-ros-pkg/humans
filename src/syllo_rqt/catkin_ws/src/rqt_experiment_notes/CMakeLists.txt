cmake_minimum_required(VERSION 2.8.3)

project(rqt_experiment_notes)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  rqt_gui 
  rqt_gui_cpp
  )

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(rqt_experiment_notes_SRCS
  src/rqt_experiment_notes/experiment_notes.cpp
  src/rqt_experiment_notes/ratio_layouted_frame.cpp
)

set(rqt_experiment_notes_HDRS
  include/rqt_experiment_notes/experiment_notes.h
  include/rqt_experiment_notes/ratio_layouted_frame.h
)

set(rqt_experiment_notes_UIS
  src/rqt_experiment_notes/experiment_notes.ui
)

set(rqt_experiment_notes_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
	INCLUDE_DIRS ${rqt_experiment_notes_INCLUDE_DIRECTORIES}
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS rqt_gui rqt_gui_cpp
)
catkin_python_setup()

qt4_wrap_cpp(rqt_experiment_notes_MOCS ${rqt_experiment_notes_HDRS})
qt4_wrap_ui(rqt_experiment_notes_UIS_H ${rqt_experiment_notes_UIS})

include_directories(${rqt_experiment_notes_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_experiment_notes_SRCS} ${rqt_experiment_notes_MOCS} ${rqt_experiment_notes_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
 
install(PROGRAMS scripts/rqt_experiment_notes
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
