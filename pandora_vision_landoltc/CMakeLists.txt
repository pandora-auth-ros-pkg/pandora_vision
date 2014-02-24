cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_landoltc)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    vision_communications
    state_manager
    cv_bridge
    image_transport
    sensor_msgs
)


catkin_package(
    DEPENDS
    CATKIN_DEPENDS
      roscpp
      vision_communications
      state_manager
      cv_bridge
      image_transport
      sensor_msgs
    INCLUDE_DIRS
      include
    LIBRARIES 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(landoltC_node 
  src/landoltC_detection.cpp
)
target_link_libraries(landoltC_node ${catkin_LIBRARIES} )
