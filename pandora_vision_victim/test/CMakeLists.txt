find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  #node_tests
  #node_tests_msgs
  rostest
  roslint
  )

link_directories(${catkin_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

###############################  Unit Tests ####################################

#################################### feature extractors #####################################

catkin_add_gtest(mean_std_dev_test unit/feature_extractors/mean_std_dev_test.cpp)

target_link_libraries(mean_std_dev_test ${catkin_LIBRARIES} feature_extractors_lib gtest_main)

#catkin_add_gtest(dominant_color_test unit/feature_extractors/dominant_color_test.cpp)
#target_link_libraries(dominant_color_test ${catkin_LIBRARIES} feature_extractors_lib gtest_main)




############## Functional Tests ###############

#catkin_download_test_data(
#    hole_detector_test.bag
#    https://pandora.ee.auth.gr/rosbags/2014_bags/hole_detector_test.bag
#    DESTINATION functional
#    )
#add_rostest(functional/hole_detector_test.launch)
