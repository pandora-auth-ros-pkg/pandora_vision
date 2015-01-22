cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_hazmat)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pandora_vision_msgs
    state_manager
    state_manager_msgs
    cv_bridge
    image_transport
    sensor_msgs
    dynamic_reconfigure
    roslint
    urdf
    pandora_cmake_tools
)

#generate_dynamic_reconfigure_options(
#  config/hazmat_cfg.cfg
#)

catkin_package(
    #DEPENDS
    CATKIN_DEPENDS
      roscpp
      pandora_vision_msgs
      state_manager
      state_manager_msgs
      cv_bridge
      image_transport
      sensor_msgs
      dynamic_reconfigure
      roslint
      urdf
    #INCLUDE_DIRS
    #LIBRARIES
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}_filters
  src/filters/histogram_mask.cpp
  src/filters/image_signature.cpp)
  )
target_link_libraries(
  ${PROJECT_NAME}_filters
  ${catkin_LIBRARIES}
  )

add_executable(hazmat_node
      src/hazmat_node.cpp
      src/hazmat_detection.cpp
      src/hazmat_detector.cpp
      src/simple_hazmat_detector.cpp
      src/sift_hazmat_detector.cpp
      src/surf_hazmat_detector.cpp
      src/orb_hazmat_detector.cpp)
target_link_libraries(hazmat_node
    ${PROJECT_NAME}_filters
    ${catkin_LIBRARIES} )
add_dependencies(hazmat_node
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# Download necessary data to run
download_extra_data(${PROJECT_NAME}_download_training_data
                    EXTRA_FILES patterns/extra_files.yaml)

########################### hazmatTrainNode ############################
#if(HAZMAT_ENABLE_TRAIN)
#  add_executable(hazmat_train src/hazmat_trainer.cpp
#          src/utils.cpp
#          src/sift.cpp
#          src/imgfeatures.cpp)
#  target_link_libraries(hazmat_train ${catkin_LIBRARIES} )
#endif()


#~ To check lint : catkin_make roslint_pandora_vision_hazmat
set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string")

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  // files for lint
)


roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
