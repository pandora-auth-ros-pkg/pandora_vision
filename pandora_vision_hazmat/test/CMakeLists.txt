find_package(roslib REQUIRED)
find_package(rostest REQUIRED)
find_package(pandora_testing_tools REQUIRED)
find_package(pandora_cmake_tools REQUIRED)

include_directories(
${pandora_testing_tools_INCLUDE_DIRS}
${roslib_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS})

################################################################################
#                               Benchmark Tests                                #
################################################################################

add_rostest(benchmark/hazmat_benchmark_test.launch)

#~ catkin_add_gtest(histogram_mask_test unit/histogram_mask_test.cpp)
#~ target_link_libraries(histogram_mask_test
  #~ ${catkin_LIBRARIES}
  #~ ${PROJECT_NAME}_filters
  #~ gtest_main
  #~ )
#~ 
#~ catkin_add_gtest(image_signature_test unit/image_signature_test.cpp)
#~ target_link_libraries(image_signature_test
  #~ ${catkin_LIBRARIES}
  #~ ${PROJECT_NAME}_filters
  #~ gtest_main
  #~ )
