cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_datamatrix)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    state_manager
    state_manager_communications
    cv_bridge
    image_transport
    sensor_msgs
)

message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(DMTX REQUIRED)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
      roscpp
      state_manager
      state_manager_communications
      cv_bridge
      image_transport
      sensor_msgs
    INCLUDE_DIRS
      include
    LIBRARIES 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${DMTX_INCLUDE_DIRS}
)

add_executable(datamatrix_node 
                  src/datamatrix_node.cpp
                  src/datamatrix_detection.cpp
                  src/datamatrix_detector.cpp
)
add_dependencies(datamatrix_node 
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(datamatrix_node 
    ${catkin_LIBRARIES} 
    ${DMTX_LIBRARIES}
)




