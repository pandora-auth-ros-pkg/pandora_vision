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

#download_extra_data(
  #${PROJECT_NAME}_qr_test_images
  #EXTRA_FILES test/functional/images/extra_files.yaml
  #)
#add_dependencies(${PROJECT_NAME}_qr_test_images tests)

add_rostest(benchmark/qr_benchmark_test.launch)
