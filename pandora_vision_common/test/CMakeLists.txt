find_package(roslib REQUIRED)
find_package(rostest REQUIRED)
find_package(pandora_testing_tools REQUIRED)
find_package(pandora_cmake_tools REQUIRED)

include_directories(
${pandora_testing_tools_INCLUDE_DIRS}
${roslib_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS})

###############################  Unit Tests ####################################

#################################### utils #####################################

catkin_add_gtest(${PROJECT_NAME}_outline_discovery_test
  unit/pandora_vision_utilities/outline_discovery_test.cpp)
target_link_libraries(${PROJECT_NAME}_outline_discovery_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(${PROJECT_NAME}_bounding_box_detection_test
  unit/pandora_vision_utilities/bounding_box_detection_test.cpp)
target_link_libraries(${PROJECT_NAME}_bounding_box_detection_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(${PROJECT_NAME}_edge_detection_test
  unit/pandora_vision_utilities/edge_detection_test.cpp)
target_link_libraries(${PROJECT_NAME}_edge_detection_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(${PROJECT_NAME}_hole_filters_test
  unit/pandora_vision_utilities/hole_filters_test.cpp)
target_link_libraries(${PROJECT_NAME}_hole_filters_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(${PROJECT_NAME}_holes_conveyor_test
  unit/pandora_vision_utilities/holes_conveyor_test.cpp)
target_link_libraries(${PROJECT_NAME}_holes_conveyor_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(${PROJECT_NAME}_message_conversions_test
  unit/pandora_vision_utilities/message_conversions_test.cpp)
target_link_libraries(${PROJECT_NAME}_message_conversions_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(${PROJECT_NAME}_morphological_operators_test
  unit/pandora_vision_utilities/morphological_operators_test.cpp)
target_link_libraries(${PROJECT_NAME}_morphological_operators_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(${PROJECT_NAME}_noise_elimination_test
  unit/pandora_vision_utilities/noise_elimination_test.cpp)
target_link_libraries(${PROJECT_NAME}_noise_elimination_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(${PROJECT_NAME}_visualization_test
  unit/pandora_vision_utilities/visualization_test.cpp)
target_link_libraries(${PROJECT_NAME}_visualization_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(${PROJECT_NAME}_wavelets_test
  unit/pandora_vision_utilities/wavelets_test.cpp)
target_link_libraries(${PROJECT_NAME}_wavelets_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

