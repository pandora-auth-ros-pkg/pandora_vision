cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_annotator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy 
  rosbag
  camera_calibration_parsers
  roslib
  cv_bridge
  sensor_msgs
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt4 REQUIRED COMPONENTS 
  QtCore 
  QtGui 
  QtUiTools
)
catkin_python_setup()

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

include(${QT_USE_FILE})

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  DEPENDS   
    QtCore 
    QtGui 
    QtUiTools
  CATKIN_DEPENDS
    roscpp
    rospy
    rosbag
    roslib
    sensor_msgs
    cv_bridge
  INCLUDE_DIRS include
  LIBRARIES 
)

#-----------------------------------------------------------------------------#
set(QT_FORMS 
  ui/pandora_annotator.ui 
)

QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

QT_WRAP_CPP(pandora_vision_annotator_node SAMPLE_MOC_SRCS 
  include/pandora_vision_annotator/pandora_vision_annotator_connector.h
  include/pandora_vision_annotator/pandora_vision_annotator_controller.h
  )
#-----------------------------------------------------------------------------#

#~ set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

add_executable(pandora_vision_annotator_node
  src/pandora_vision_annotator/pandora_vision_annotator_node.cpp
  src/pandora_vision_annotator/pandora_vision_annotator_application.cpp
  src/pandora_vision_annotator/pandora_vision_annotator_connector.cpp
  src/pandora_vision_annotator/pandora_vision_annotator_controller.cpp
  src/pandora_vision_annotator/pandora_vision_annotator_loader.cpp
  src/pandora_vision_annotator/pandora_vision_annotator_tools.cpp
  ${QT_FORMS_HPP} ${SAMPLE_MOC_SRCS}
)
target_link_libraries(pandora_vision_annotator_node
  ${catkin_LIBRARIES} 
  ${QT_LIBRARIES}
)
install(PROGRAMS scripts/images_to_bag
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


