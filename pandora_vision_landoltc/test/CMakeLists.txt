find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  roslint
  )


###############################  Unit Tests ####################################

################################# landoltc node #####################################
catkin_add_gtest(landoltc_detector_test
  unit/landoltc_detector_test.cpp)

add_library(landoltc_node_lib
../src/landoltc_detector.cpp
)




target_link_libraries(landoltc_detector_test landoltc_node_lib  ${catkin_LIBRARIES}  gtest_main)
