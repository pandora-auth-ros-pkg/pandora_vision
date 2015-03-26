find_package(rostest REQUIRED)

################# Unit Tests #################

catkin_add_gtest(landoltc_detector_test
  unit/landoltc_2d/landoltc_detector_test.cpp)	
  
target_link_libraries(landoltc_detector_test
    ${catkin_LIBRARIES} ${PROJECT_NAME}_2d_lib gtest_main)

catkin_add_gtest(landoltc3d_detector_test
  unit/landoltc_3d/landoltc3d_detector_test.cpp)	
  
target_link_libraries(landoltc3d_detector_test
  ${catkin_LIBRARIES} ${PROJECT_NAME}_3d_lib gtest_main)
