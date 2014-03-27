cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_victim)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    vision_communications
    state_manager
    state_manager_communications
    cv_bridge
    image_transport
    sensor_msgs
    roslint
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
      roslint
    INCLUDE_DIRS
      include
    LIBRARIES 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(victim_node 
    src/victim_node.cpp
    src/victim_detection.cpp
    src/color_detection.cpp
)
target_link_libraries(victim_node ${catkin_LIBRARIES} )
add_dependencies(victim_node 
    vision_communications_gencpp
    ${catkin_EXPORTED_TARGETS}
)

########################### modelTRainer ############################
if(MODEL_ENABLE_TRAIN)
  add_executable(model_train 
                  src/model_trainer.cpp
  )
  target_link_libraries(model_train ${catkin_LIBRARIES} )
endif()

