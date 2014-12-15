find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  rostest
  roslint
)
link_directories(${catkin_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

################# Unit Tests #################
catkin_add_gtest(motion_detector_test
  unit/motion_detector_test.cpp
)
target_link_libraries(motion_detector_test
  ${catkin_LIBRARIES}
  motion_node_lib
  gtest_main
)
