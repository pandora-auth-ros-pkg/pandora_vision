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

################################################################################
#                               Functional Tests                               #
################################################################################

download_extra_data(
  ${PROJECT_NAME}_qr_test_images
  EXTRA_FILES test/functional/images/extra_files.yaml
  )
add_dependencies(${PROJECT_NAME}_qr_test_images tests)

add_rostest(functional/qr_test.launch)

################# Unit Tests #################
catkin_add_gtest(qrCode_detector_test
  unit/qrCode_detector_test.cpp
)
find_package(PkgConfig)
pkg_check_modules(ZBAR zbar)
target_link_libraries(qrCode_detector_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_qrcode_node
  gtest_main
  zbar
)

download_extra_data(
    ${PROJECT_NAME}_download_qr_images_unit_testing
    EXTRA_FILES test/unit/data/extra_files.yaml
    )


