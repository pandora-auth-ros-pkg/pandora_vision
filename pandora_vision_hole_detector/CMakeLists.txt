cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_hole_detector)

#~ set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    vision_communications
    state_manager
    state_manager_communications
    cv_bridge
    image_transport
    sensor_msgs
    dynamic_reconfigure
    pcl_ros
    message_filters
    roslint
)

generate_dynamic_reconfigure_options(
  cfg/hole_fusion_cfg.cfg
  cfg/depth_cfg.cfg
  cfg/rgb_cfg.cfg
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
      dynamic_reconfigure
      pcl_ros
      message_filters
      roslint
    INCLUDE_DIRS
      include
    LIBRARIES
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)


########################### DepthNode ############################
add_library(depth_node_lib
  src/depth_node/depth.cpp
  src/message_conversions/message_conversions.cpp
)
target_link_libraries(depth_node_lib
  ${catkin_LIBRARIES}
)

add_executable(depth_node
  src/depth_node/depth_node.cpp
  src/depth_node/hole_detector.cpp
  src/depth_node/hole_filters.cpp
  src/depth_node/noise_elimination.cpp
  src/depth_node/edge_detection.cpp
  src/depth_node/blob_detection.cpp
  src/depth_node/bounding_box_detection.cpp
  src/depth_node/morphological_operators.cpp
  src/depth_node/visualization.cpp
  src/depth_node/depth_parameters.cpp
  src/depth_node/timer.cpp
)
set (DEPTH_COMPILE_FLAGS "-Wno-deprecated")

target_link_libraries(depth_node
  depth_node_lib
  ${catkin_LIBRARIES}
)

set_target_properties(depth_node
  PROPERTIES COMPILE_FLAGS
  ${DEPTH_COMPILE_FLAGS}
)

add_dependencies(depth_node
  vision_gencfg
  vision_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)


########################### RgbDepthSynchronizerNode ###########################
add_library(synchronizer_lib
  src/message_conversions/message_conversions.cpp
  src/depth_node/timer.cpp
)
target_link_libraries(synchronizer_lib
  ${catkin_LIBRARIES}
)

add_executable(synchronizer_node
  src/synchronizer_node/rgb_depth_synchronizer.cpp
  src/synchronizer_node/rgb_depth_synchronizer_node.cpp
)

target_link_libraries(synchronizer_node
  synchronizer_lib
  ${catkin_LIBRARIES}
)

add_dependencies(synchronizer_node
  vision_gencfg
  vision_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)


############################## HoleFusionNode ##################################
add_library(hole_fusion_lib
  src/message_conversions/message_conversions.cpp
  src/depth_node/blob_detection.cpp
  src/depth_node/bounding_box_detection.cpp
  src/depth_node/depth_parameters.cpp
  src/depth_node/edge_detection.cpp
  src/depth_node/hole_filters.cpp
  src/depth_node/morphological_operators.cpp
  src/depth_node/timer.cpp
  src/depth_node/visualization.cpp
)

target_link_libraries(hole_fusion_lib
  ${catkin_LIBRARIES}
)
add_executable(hole_fusion_node
  src/hole_fusion_node/hole_fusion.cpp
  src/hole_fusion_node/hole_fusion_node.cpp
  src/hole_fusion_node/hole_fusion_parameters.cpp
  src/hole_fusion_node/generic_filters.cpp
  src/hole_fusion_node/rgb_filters.cpp
  src/hole_fusion_node/depth_filters.cpp
  src/hole_fusion_node/planes_detection.cpp
)

target_link_libraries(hole_fusion_node
  hole_fusion_lib
  ${catkin_LIBRARIES}
)

add_dependencies(hole_fusion_node
  vision_gencfg
  vision_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)


################################# RgbDummyNode #################################
add_executable(rgb_dummy_node
  src/rgb_dummy_node/rgb_dummy_node.cpp
  src/rgb_dummy_node/rgb_dummy.cpp
  src/depth_node/hole_filters.cpp
  src/message_conversions/message_conversions.cpp
  src/depth_node/noise_elimination.cpp
  src/depth_node/edge_detection.cpp
  src/depth_node/blob_detection.cpp
  src/depth_node/bounding_box_detection.cpp
  src/depth_node/timer.cpp
  src/depth_node/depth_parameters.cpp
  src/depth_node/morphological_operators.cpp
  src/depth_node/visualization.cpp
  )

target_link_libraries(rgb_dummy_node ${catkin_LIBRARIES} )

add_dependencies(rgb_dummy_node
    vision_communications_gencpp
    ${catkin_EXPORTED_TARGETS}
)

################################# RgbNode ######################################
add_executable(rgb_node
    src/rgb_node/hole_node.cpp
    src/rgb_node/hole_detection.cpp
    src/rgb_node/hole_detector.cpp
    src/rgb_node/texture_filter.cpp

    src/rgb_node/hole_filters.cpp
    src/rgb_node/noise_elimination.cpp
    src/rgb_node/edge_detection.cpp
    src/rgb_node/blob_detection.cpp
    src/rgb_node/bounding_box_detection.cpp
    src/rgb_node/morphological_operators.cpp
    src/rgb_node/visualization.cpp
    src/rgb_node/rgb_parameters.cpp
    src/rgb_node/timer.cpp

)
target_link_libraries(rgb_node ${catkin_LIBRARIES} )

add_dependencies(rgb_node
    vision_communications_gencpp
    ${catkin_EXPORTED_TARGETS}
)

#~ To check lint : catkin_make roslint_pandora_vision_hole_detector
set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string, -runtime/printf, -readability/streams")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  include/rgb_node/hole_detection.h
  include/rgb_node/hole_detector.h
  include/rgb_node/edge_detection.h
  include/rgb_node/texture_filter.h
  include/rgb_node/blob_extraction.h
  include/rgb_node/rgb_constants.h
  src/rgb_node/hole_detection.cpp
  src/rgb_node/hole_detector.cpp
  src/rgb_node/edge_detection.cpp
  src/rgb_node/texture_filter.cpp
  src/rgb_node/blob_extraction.cpp
  src/rgb_node/rgb_constants.cpp
)

################################# ROSLINT ######################################
# Execute via lint : catkin_make roslint_{PACKAGE_NAME}

set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string, -runtime/printf, -readability/streams")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

## depth_node ##

include/depth_node/blob_detection.h
include/depth_node/bounding_box_detection.h
include/depth_node/defines.h
include/depth_node/depth.h
include/depth_node/depth_parameters.h
include/depth_node/edge_detection.h
include/depth_node/hole_detector.h
include/depth_node/hole_filters.h
include/depth_node/morphological_operators.h
include/depth_node/noise_elimination.h
#include/depth_node/timer.h
include/depth_node/visualization.h

src/depth_node/blob_detection.cpp
src/depth_node/bounding_box_detection.cpp
src/depth_node/depth.cpp
src/depth_node/depth_node.cpp
src/depth_node/depth_parameters.cpp
src/depth_node/edge_detection.cpp
src/depth_node/hole_detector.cpp
src/depth_node/hole_filters.cpp
src/depth_node/morphological_operators.cpp
src/depth_node/noise_elimination.cpp
#src/depth_node/timer.cpp
src/depth_node/visualization.cpp


## hole_fusion_node ##

include/hole_fusion_node/depth_filters.h
include/hole_fusion_node/generic_filters.h
include/hole_fusion_node/hole_fusion.h
include/hole_fusion_node/hole_fusion_parameters.h
include/hole_fusion_node/planes_detection.h
include/hole_fusion_node/rgb_filters.h

src/hole_fusion_node/depth_filters.cpp
src/hole_fusion_node/generic_filters.cpp
src/hole_fusion_node/hole_fusion.cpp
src/hole_fusion_node/hole_fusion_node.cpp
src/hole_fusion_node/hole_fusion_parameters.cpp
src/hole_fusion_node/planes_detection.cpp
src/hole_fusion_node/rgb_filters.cpp


## message_conversions ##

include/message_conversions.h
src/message_conversions.cpp


## synchronizer_node ##

include/synchronizer_node/rgb_depth_synchronizer.h
src/synchronizer_node/rgb_depth_synchronizer.cpp
src/synchronizer_node/rgb_depth_synchronizer_node.cpp
)

roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
