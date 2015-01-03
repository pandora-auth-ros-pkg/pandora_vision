find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  rostest
  roslint
  )

link_directories(${catkin_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

################# Unit Tests #################

catkin_add_gtest(landoltc_detector_test
  unit/landoltc_2d/landoltc_detector_test.cpp)	
  
target_link_libraries(landoltc_detector_test
    ${catkin_LIBRARIES} landoltc_lib gtest_main)

catkin_add_gtest(landoltc3d_detector_test
  unit/landoltc_3d/landoltc3d_detector_test.cpp)	
  
target_link_libraries(landoltc3d_detector_test
    ${catkin_LIBRARIES} landoltc3d_lib gtest_main)
