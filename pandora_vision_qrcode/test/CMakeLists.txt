find_package(roslib REQUIRED)
find_package(rostest REQUIRED)
find_package(pandora_testing_tools REQUIRED)
find_package(pandora_cmake_tools REQUIRED)

include_directories(
${pandora_testing_tools_INCLUDE_DIRS}
${roslib_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS})

################# Unit Tests #################

download_extra_data(
  ${PROJECT_NAME}_download_qr_images_unit_testing
  EXTRA_FILES test/unit/data/extra_files.yaml
  )

catkin_add_gtest(${PROJECT_NAME}_detector_test
  unit/qrCode_detector_test.cpp
  )
target_link_libraries(${PROJECT_NAME}_detector_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_detector
  gtest_main
  )
add_dependencies(${PROJECT_NAME}_detector_test
  ${PROJECT_NAME}_download_qr_images_unit_testing
  )

##############################################################################
#                             Download Benchmark Data                        #
##############################################################################
if (${PROJECT_NAME}_download_testing_data)

  # Download the images for the kinect/xtion benchmarking.
  download_extra_data(
    ${PROJECT_NAME}_benchmark_images_480p
    EXTRA_FILES test/benchmark/images/images_480p/extra_files.yaml
    )
  # Download the images for the 720p web cam benchmarking.
  download_extra_data(
    ${PROJECT_NAME}_benchmark_images_720p
    EXTRA_FILES test/benchmark/images/images_720p/extra_files.yaml
    )
endif()
################################################################################
#                               Benchmark Tests                                #
################################################################################

if (${PROJECT_NAME}_benchmark)
  add_rostest(benchmark/qrcode_benchmark_test.launch
    ARGS xtion:=true)
  add_rostest(benchmark/qrcode_benchmark_test.launch
    ARGS usb_cam:=true)
endif()
