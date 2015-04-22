cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_victim)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pandora_common_msgs
    pandora_vision_msgs
    state_manager
    state_manager_msgs
    pandora_cmake_tools
    cv_bridge
    image_transport
    dynamic_reconfigure
    sensor_msgs
    roslint
    urdf
)

generate_dynamic_reconfigure_options(
  config/victim_dyn_reconf.config
  )

catkin_package(
    #DEPENDS
    CATKIN_DEPENDS
      roscpp
      pandora_common_msgs
      pandora_vision_msgs
      state_manager
      state_manager_msgs
      cv_bridge
      image_transport
      dynamic_reconfigure
      sensor_msgs
      urdf
    #INCLUDE_DIRS
    #LIBRARIES
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

############################ File Utilities Library ############################
add_library(${PROJECT_NAME}_utilities
            src/utilities/file_utilities.cpp)
target_link_libraries(${PROJECT_NAME}_utilities ${catkin_LIBRARIES})

########################## Victim_Parameters library ###############################

add_library(${PROJECT_NAME}_victim_parameters src/victim_parameters.cpp)
target_link_libraries(${PROJECT_NAME}_victim_parameters ${catkin_LIBRARIES})

################################ Channels Statistics Feature Extractors library #################################
add_library(${PROJECT_NAME}_channels_statistics_feature_extractors
            src/channels_statistics_feature_extractors/mean_std_dev_extractor.cpp
            src/channels_statistics_feature_extractors/dominant_color_extractor.cpp
            src/channels_statistics_feature_extractors/dft_coeffs_extractor.cpp
            src/channels_statistics_feature_extractors/color_angles_extractor.cpp
)

target_link_libraries(${PROJECT_NAME}_channels_statistics_feature_extractors ${catkin_LIBRARIES} ${PROJECT_NAME}_victim_parameters)

################################ Feature Extractors library #################################
add_library(${PROJECT_NAME}_feature_extractors
            src/feature_extractors/channels_statistics_extractor.cpp
            src/feature_extractors/edge_orientation_extractor.cpp
            src/feature_extractors/haralickfeature_extractor.cpp
            src/feature_extractors/depth_feature_extraction.cpp
            src/feature_extractors/rgb_feature_extraction.cpp
)


target_link_libraries(${PROJECT_NAME}_feature_extractors ${catkin_LIBRARIES} ${PROJECT_NAME}_channels_statistics_feature_extractors
${PROJECT_NAME}_victim_parameters ${PROJECT_NAME}_utilities)

########################## SVM Classifier Library###############################
add_library(${PROJECT_NAME}_svm_classifier
            src/svm_classifier/svm_training.cpp
            src/svm_classifier/rgb_svm_training.cpp
            #src/svm_classifier/depth_svm_training.cpp
)

target_link_libraries(${PROJECT_NAME}_svm_classifier ${catkin_LIBRARIES} ${PROJECT_NAME}_feature_extractors
  ${PROJECT_NAME}_victim_parameters ${PROJECT_NAME}_utilities)

#############################  victim node ##################################################

add_executable(victim_node
    src/victim_node.cpp
    src/victim_detection.cpp
    src/svm_classifier/rgb_system_validator.cpp
    src/svm_classifier/depth_system_validator.cpp
    src/victim_vj_detector.cpp
)

target_link_libraries(victim_node ${catkin_LIBRARIES}
                      ${PROJECT_NAME}_feature_extractors
                      ${PROJECT_NAME}_channels_statistics_feature_extractors
                      ${PROJECT_NAME}_victim_parameters)

add_dependencies(victim_node
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

########################### victimTrainer ############################
if(VICTIM_ENABLE_TRAIN)

  add_executable(victim_rgb_svm_training_node
                  #src/svm_classifier/training.cpp
                  src/victim_rgb_svm_training_node.cpp
                  #src/svm_classifier/training_parameters.cpp
  		)


  target_link_libraries(victim_rgb_svm_training_node ${catkin_LIBRARIES}
                      ${PROJECT_NAME}_svm_classifier
                      ${PROJECT_NAME}_feature_extractors
                      ${PROJECT_NAME}_channels_statistics_feature_extractors
                      ${PROJECT_NAME}_victim_parameters)

  add_dependencies(victim_rgb_svm_training_node
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
endif()

########################### modelTRainer ############################
if(MODEL_ENABLE_TRAIN)
  add_executable(model_train
                  src/model_trainer.cpp
  )
  target_link_libraries(model_train ${catkin_LIBRARIES} )
endif()

download_extra_data(
  ${PROJECT_NAME}_download_training_files
  EXTRA_FILES data/extra_files.yaml
  )

#~ To check lint : catkin_make roslint_pandora_vision_victim
set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string")

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  ## victim_node ##

  include/victim_detection.h
  include/victim_vj_detector.h
  include/svm_classifier/rgb_system_validator.h
  include/svm_classifier/depth_system_validator.h
  include/svm_classifier/training.h
  include/svm_classifier/training_parameters.h
  include/victim_parameters.h

  src/victim_detection.cpp
  src/victim_vj_detector.cpp
  src/svm_classifier/rgb_system_validator.cpp
  src/svm_classifier/depth_system_validator.cpp
  src/svm_classifier/training.cpp
  src/svm_classifier/training_parameters.cpp
  src/victim_parameters.cpp

  ## feature_extractors ##

  include/feature_extractors/channels_statistics_extractor.h
  include/feature_extractors/edge_orientation_extractor.h
  include/feature_extractors/haralickfeature_extractor.h



  src/feature_extractors/channels_statistics_extractor.cpp
  src/feature_extractors/edge_orientation_extractor.cpp
  src/feature_extractors/haralickfeature_extractor.cpp


  ## channels_statistics_feature_extractors ##

  include/channels_statistics_feature_extractors/mean_std_dev.cpp
  include/channels_statistics_feature_extractors/dominant_color.cpp
  include/channels_statistics_feature_extractors/dft_coeffs.cpp
  include/channels_statistics_feature_extractors/color_angles.cpp

  src/channels_statistics_feature_extractors/mean_std_dev.cpp
  src/channels_statistics_feature_extractors/dominant_color.cpp
  src/channels_statistics_feature_extractors/dft_coeffs.cpp
  src/channels_statistics_feature_extractors/color_angles.cpp

)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

##################################### Testing ##################################

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
