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
#~ find_package(DMTX REQUIRED)

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
target_link_libraries(datamatrix_node dmtx ${catkin_LIBRARIES} ${DMTX_LIBRARIES})




