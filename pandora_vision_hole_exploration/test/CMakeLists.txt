###############################  Unit Tests ####################################

#################################### utils #####################################


catkin_add_gtest(holes_conveyor_test
  unit/utils/holes_conveyor_test.cpp)
target_link_libraries(holes_conveyor_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(message_conversions_test
  unit/utils/message_conversions_test.cpp)
target_link_libraries(message_conversions_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(visualization_test
  unit/utils/visualization_test.cpp)
target_link_libraries(visualization_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)


################################ depth node ####################################
catkin_add_gtest(depth_processor_test
  unit/depth_node/depth_processor_test.cpp)

target_link_libraries(depth_processor_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_depth_processor
  gtest_main)


################################# rgb node #####################################
catkin_add_gtest(rgb_processor_test
  unit/rgb_node/rgb_processor_test.cpp)

target_link_libraries(rgb_processor_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_rgb_processor
  gtest_main)

############################# hole fusion node #################################
catkin_add_gtest(hole_fusion_test
  unit/hole_fusion_node/hole_fusion_test.cpp)

target_link_libraries(hole_fusion_test
 ${PROJECT_NAME}_hole_fusion
 gtest_main)

############### Functional Tests ###############

find_package(rostest REQUIRED)

#catkin_download_test_data(
#    ${PROJECT_NAME}_hole_exploration_test.bag
#    http://downloads.pandora.ee.auth.gr/bags/hole_detector_test.bag
#    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test/functional
#    MD5 1844a18c09ee3f7b5725bb788fa43ac1
#    )
#add_rostest(functional/hole_exploration_test.launch)

################################################################################
#                               Benchmark Tests                                #
################################################################################

download_extra_data(
  ${PROJECT_NAME}_benchmark_images_bags_pcl
  EXTRA_FILES test/benchmark/images/bags_pcl/extra_files.yaml
  #EXCLUDE_FROM_ALL
  )
add_dependencies(${PROJECT_NAME}_benchmark_images_bags_pcl tests)

# add_rostest(benchmark/hole_exploration_benchmark_test.launch)
# add_rostest(benchmark/hole_benchmark_test.launch DEPENDENCIES
            # ${PROJECT_NAME}_benchmark_images_bags_pcl)
