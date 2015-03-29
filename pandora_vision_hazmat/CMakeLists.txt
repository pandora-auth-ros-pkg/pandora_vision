cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_hazmat)

find_package(Boost REQUIRED)

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
      urdf
    #INCLUDE_DIRS
    LIBRARIES
      Boost
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}_filters
  src/filters/histogram_mask.cpp
  src/filters/image_signature.cpp)
target_link_libraries(
  ${PROJECT_NAME}_filters
  ${catkin_LIBRARIES}
  )

add_library(
  ${PROJECT_NAME}_detectors
  src/detection/planar_object_detector.cpp
  src/detection/feature_matching_detector.cpp
  src/detection/sift_detector.cpp
  src/detection/surf_detector.cpp
  src/detection/orb_detector.cpp
  )

target_link_libraries(
  ${PROJECT_NAME}_detectors
  ${PROJECT_NAME}_filters
  ${catkin_LIBRARIES}
  )

add_library(
  ${PROJECT_NAME}_detector_factory
  src/detection/detector_factory.cpp)
target_link_libraries(
  ${PROJECT_NAME}_detector_factory
  ${PROJECT_NAME}_detectors
  ${catkin_LIBRARIES}
  )

add_library(${PROJECT_NAME}_interface
      src/detection/hazmat_detection.cpp
      )
target_link_libraries(${PROJECT_NAME}_interface
    ${PROJECT_NAME}_detector_factory
    ${catkin_LIBRARIES}
    )
add_executable(${PROJECT_NAME}_node
      src/detection/hazmat_node.cpp
      )
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}_interface
    ${catkin_LIBRARIES} )
add_dependencies(${PROJECT_NAME}_node
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# Download necessary data to run
download_extra_data(${PROJECT_NAME}_download_training_results
                    EXTRA_FILES data/extra_files.yaml)

option(${PROJECT_NAME}_training "Build flag for the training module")
########################### hazmatTrainNode ############################
if(${PROJECT_NAME}_training)
  add_executable(${PROJECT_NAME}_trainer_node 
          src/training/hazmat_trainer_node.cpp
          src/training/planar_pattern_trainer.cpp 
          src/training/factory.cpp
          src/training/sift_trainer.cpp
          src/training/surf_trainer.cpp
          src/training/orb_trainer.cpp)
        target_link_libraries(${PROJECT_NAME}_trainer_node
          ${catkin_LIBRARIES} )
endif()


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
