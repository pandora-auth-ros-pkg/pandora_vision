find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  rostest
  roslint
)
link_directories(${catkin_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

################# Unit Tests #################
catkin_add_gtest(datamatrix_detector_test
  unit/datamatrix_detector_test.cpp
)
find_package(PkgConfig)
find_package(DMTX REQUIRED)

target_link_libraries(datamatrix_detector_test
  ${catkin_LIBRARIES}
  datamatrix_node_lib
  gtest_main
  ${DMTX_LIBRARIES}
)
