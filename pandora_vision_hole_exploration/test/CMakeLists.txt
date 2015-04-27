###############################  Unit Tests ####################################

#################################### utils #####################################


#catkin_add_gtest(bounding_box_detection_test
#  unit/utils/bounding_box_detection_test.cpp)
#target_link_libraries(bounding_box_detection_test
#  ${catkin_LIBRARIES}
#  ${PROJECT_NAME}_utils
#  gtest_main)


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
catkin_add_gtest(depth_test
  unit/depth_node/depth_test.cpp)

target_link_libraries(depth_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_depth
  gtest_main)


################################# rgb node #####################################
catkin_add_gtest(rgb_test
  unit/rgb_node/rgb_test.cpp)

target_link_libraries(rgb_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_rgb
  gtest_main)

############################# hole fusion node #################################
catkin_add_gtest(hole_fusion_test
  unit/hole_fusion_node/hole_fusion_test.cpp)

target_link_libraries(hole_fusion_test
 ${PROJECT_NAME}_hole_fusion
 gtest_main)

############### Functional Tests ###############
#
#find_package(rostest REQUIRED)

#catkin_download_test_data(
#    ${PROJECT_NAME}_hole_detector_test.bag
#    http://downloads.pandora.ee.auth.gr/bags/hole_detector_test.bag
#    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test/functional
#    MD5 1844a18c09ee3f7b5725bb788fa43ac1
#    )
#add_rostest(functional/hole_detector_test.launch)
