cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_obstacle)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslint
    cv_bridge
    pcl_conversions
    sensor_msgs
    pandora_vision_msgs
    pandora_vision_common
    sensor_processor
    state_manager
    state_manager_msgs
    pandora_cmake_tools
)

catkin_package(
    #DEPENDS
    CATKIN_DEPENDS
      roscpp
      cv_bridge
      pcl_conversions
      sensor_msgs
      pandora_vision_msgs
      pandora_vision_common
      sensor_processor
      state_manager
      state_manager_msgs
    INCLUDE_DIRS
    LIBRARIES
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

############################### barrel preprocessor ##############################
add_library(${PROJECT_NAME}_barrel_preprocessor
  src/barrel_detection/barrel_preprocessor.cpp
)
target_link_libraries(${PROJECT_NAME}_barrel_preprocessor
  ${catkin_LIBRARIES}
)

############################### barrel processor ##############################
add_library(${PROJECT_NAME}_barrel_processor
  src/barrel_detection/barrel_processor.cpp
)
target_link_libraries(${PROJECT_NAME}_barrel_processor
  ${catkin_LIBRARIES}
)

############################### barrel postprocessor ##############################
add_library(${PROJECT_NAME}_barrel_postprocessor
  src/barrel_detection/barrel_postprocessor.cpp
)
target_link_libraries(${PROJECT_NAME}_barrel_postprocessor
  ${catkin_LIBRARIES}
)

############################### barrel handler ##############################
add_library(${PROJECT_NAME}_barrel_handler
  src/barrel_detection/barrel_handler.cpp
)
target_link_libraries(${PROJECT_NAME}_barrel_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_barrel_preprocessor
  ${PROJECT_NAME}_barrel_processor
  ${PROJECT_NAME}_barrel_postprocessor
)

############################ barrel node ###############################
add_executable(${PROJECT_NAME}_barrel_node
  src/barrel_detection/barrel_node.cpp
)
add_dependencies(${PROJECT_NAME}_barrel_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_barrel_node 
  ${catkin_LIBRARIES} 
  ${PROJECT_NAME}_barrel_handler
)


#~ To check lint : catkin_make roslint_pandora_vision_obstacle
set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line, -build/include_what_you_use")

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  include/*.h
  src/*.cpp
  test/*.cpp

)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

##################################### Testing ##################################

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
