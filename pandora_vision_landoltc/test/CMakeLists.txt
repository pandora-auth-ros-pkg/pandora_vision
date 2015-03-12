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

add_rostest(benchmark/landoltc_benchmark_test.launch)

################################################################################
#                               Functional Tests                               #
################################################################################

#~ download_extra_data(
  #~ ${PROJECT_NAME}_landoltc_test_images
  #~ EXTRA_FILES test/functional/images/extra_files.yaml
#~ )
#~ add_dependencies(${PROJECT_NAME}_landoltc_test_images tests)
#~ 
#~ add_rostest(functional/landoltc_test.launch)
