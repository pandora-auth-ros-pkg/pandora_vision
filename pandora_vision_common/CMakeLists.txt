cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
  pandora_common_msgs
  sensor_processor
  pcl_ros
  urdf
  roslint
  )

catkin_package(
  CATKIN_DEPENDS
    roscpp
    cv_bridge
    image_transport
    sensor_msgs
    pandora_common_msgs
    sensor_processor
    pcl_ros
    urdf
  INCLUDE_DIRS
    include
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

################################ Utils library #################################
add_library(${PROJECT_NAME}_utils
  src/pandora_vision_utilities/blob_detection.cpp
  src/pandora_vision_utilities/bounding_box_detection.cpp
  src/pandora_vision_utilities/edge_detection.cpp
  src/pandora_vision_utilities/histogram.cpp
  src/pandora_vision_utilities/hole_filters.cpp
  src/pandora_vision_utilities/holes_conveyor.cpp
  src/pandora_vision_utilities/message_conversions.cpp
  src/pandora_vision_utilities/morphological_operators.cpp
  src/pandora_vision_utilities/noise_elimination.cpp
  src/pandora_vision_utilities/outline_discovery.cpp
  src/pandora_vision_utilities/parameters.cpp
  src/pandora_vision_utilities/timer.cpp
  src/pandora_vision_utilities/visualization.cpp
  src/pandora_vision_utilities/wavelets.cpp
  )
target_link_libraries(${PROJECT_NAME}_utils
  ${catkin_LIBRARIES}
  )

##################################### Testing ##################################
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
