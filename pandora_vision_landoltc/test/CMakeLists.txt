find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  rostest
  roslint
  )

################# Unit Tests #################

catkin_add_gtest(landoltc_detector_test
  unit/landoltc_2d/landoltc_detector_test.cpp)	
  
add_library(landoltc_lib
  ../src/landoltc_2d/landoltc_detector.cpp)
  
target_link_libraries(landoltc_detector_test
    ${catkin_LIBRARIES} landoltc_lib gtest_main)

catkin_add_gtest(landoltc3d_detector_test
  unit/landoltc_3d/landoltc3d_detector_test.cpp)	
  
add_library(landoltc3d_lib
  ../src/landoltc_3d/landoltc3d_detector.cpp)
  
target_link_libraries(landoltc3d_detector_test
    ${catkin_LIBRARIES} landoltc3d_lib gtest_main)
