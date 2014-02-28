cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_hazmat)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    vision_communications
    state_manager
    state_manager_communications
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
)

add_executable(hazmat_node 
						src/hazmat_detection.cpp
						src/hazmat_detector.cpp
						src/utils.cpp 
						src/sift.cpp 
						src/kdtree.cpp 
						src/minpq.cpp 
						src/xform.cpp 
						src/imgfeatures.cpp)
target_link_libraries(hazmat_node ${catkin_LIBRARIES} )
add_dependencies(hazmat_node 
    vision_communications_gencpp
    ${catkin_EXPORTED_TARGETS}
)

########################### hazmatTrainNode ############################
if(HAZMAT_ENABLE_TRAIN)
  add_executable(hazmat_train_node src/siftfeat.cpp
						src/utils.cpp 
						src/sift.cpp  
						src/imgfeatures.cpp)
  target_link_libraries(hazmat_train_node ${catkin_LIBRARIES} )
endif()

