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
  qrcode_node_lib
  gtest_main
  zbar
)

################# Download necessary images in /home/v/Downloads folder #################

set(MY_URL "https://copy.com/weAYQs6IXAUoQdKa")
set(MY_SRC_PATH "$ENV{HOME}/Downloads")
set(MY_DOWNLOAD_PATH "${MY_SRC_PATH}/Qr_Datamatrix_Testing.tar.gz")
set(MY_EXTRACTION_PATH "${MY_SRC_PATH}/Qr_Datamatrix_Testing")

if (NOT EXISTS "${MY_DOWNLOAD_PATH}")
  file(DOWNLOAD "${MY_URL}" "${MY_DOWNLOAD_PATH}")
endif()

file(MAKE_DIRECTORY ${MY_EXTRACTION_PATH})

execute_process(
  COMMAND ${CMAKE_COMMAND} -E tar xzf ${MY_SRC_PATH}/Qr_Datamatrix_Testing.tar.gz
  WORKING_DIRECTORY ${MY_EXTRACTION_PATH}
)



