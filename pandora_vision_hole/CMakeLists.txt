cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_hole)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    vision_communications
    state_manager
    cv_bridge
    image_transport
    sensor_msgs
)

find_package(PkgConfig)
pkg_check_modules(GSL gsl)

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

########################### holeFindNode ############################

add_executable(hole_find_node
						src/hole_detection.cpp  
						src/hole_detector.cpp 
						src/textureFilter.cpp 
						src/tracker.cpp 
						src/trackerChain.cpp 
						src/pattern.cpp 
						src/thing.cpp
					   )
target_link_libraries(hole_find_node 
  ${catkin_LIBRARIES}
  gsl
  gslcblas
)
add_dependencies(hole_find_node vision_communications_gencpp)
