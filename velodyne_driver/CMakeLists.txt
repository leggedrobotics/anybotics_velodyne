cmake_minimum_required(VERSION 3.5.1)
project(any_velodyne_driver)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  any_velodyne_msgs
  diagnostic_updater
  dynamic_reconfigure
  nodelet
  roscpp
  tf
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)
find_package(Boost REQUIRED COMPONENTS thread)

# libpcap provides no pkg-config or find_package module:
set(libpcap_LIBRARIES -lpcap)

# Generate dynamic_reconfigure server
generate_dynamic_reconfigure_options(
  cfg/VelodyneNode.cfg
)

# objects needed by other ROS packages that depend on this one
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_socket
    ${PROJECT_NAME}
    ${PROJECT_NAME}_nodelet
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

include_directories(
  include 
  SYSTEM
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

#############
# Libraries #
#############
# Socket library.
add_library(${PROJECT_NAME}_socket 
  src/lib/input.cc
)
add_dependencies(${PROJECT_NAME}_socket
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(${PROJECT_NAME}_socket
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# ROS wrapper.
add_library(${PROJECT_NAME}
  src/driver/driver.cc
)
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_socket
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

##########################
##  Nodes and nodelets  ##
##########################
# Node.
add_executable(${PROJECT_NAME}_node
  src/velodyne_driver_node.cc
)
add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_socket
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# Nodelet.
add_library(${PROJECT_NAME}_nodelet
  src/velodyne_driver_nodelet.cc
)
add_dependencies(${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}_socket
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

###########
# Install #
###########
install(
  TARGETS
    ${PROJECT_NAME}_socket
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  FILES
    nodelet_velodyne.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  PROGRAMS
    src/vdump
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# if (CATKIN_ENABLE_TESTING)

#   # these dependencies are only needed for unit testing
#   find_package(roslaunch REQUIRED)
#   find_package(rostest REQUIRED)

#   # Download packet capture (PCAP) files containing test data.
#   # Store them in devel-space, so rostest can easily find them.
#   catkin_download_test_data(
#     ${PROJECT_NAME}_tests_class.pcap
#     http://download.ros.org/data/velodyne/class.pcap
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
#     MD5 65808d25772101358a3719b451b3d015)
#   catkin_download_test_data(
#     ${PROJECT_NAME}_tests_32e.pcap
#     http://download.ros.org/data/velodyne/32e.pcap
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
#     MD5 e41d02aac34f0967c03a5597e1d554a9)
#   catkin_download_test_data(
#     ${PROJECT_NAME}_tests_vlp16.pcap
#     http://download.ros.org/data/velodyne/vlp16.pcap
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
#     MD5 f45c2bb1d7ee358274e423ea3b66fd73)
  
#   # unit tests
#   add_rostest(tests/pcap_node_hertz.test)
#   add_rostest(tests/pcap_nodelet_hertz.test)
#   add_rostest(tests/pcap_32e_node_hertz.test)
#   add_rostest(tests/pcap_32e_nodelet_hertz.test)
#   add_rostest(tests/pcap_vlp16_node_hertz.test)
#   add_rostest(tests/pcap_vlp16_nodelet_hertz.test)
  
#   # parse check all the launch/*.launch files
#   roslaunch_add_file_check(launch)

#   # unit test
#   catkin_add_gtest(time_test tests/timeconversiontest.cpp)
#   target_link_libraries(time_test
#     ${catkin_LIBRARIES}
#     ${Boost_LIBRARIES})
# endif (CATKIN_ENABLE_TESTING)

##############
##   Test   ##
##############
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    tests/empty_test.cpp
  )
  add_dependencies(test_${PROJECT_NAME}
    ${PROJECT_NAME}_node
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    include
  )
  target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${PROJECT_NAME}
    ${PROJECT_NAME}_socket
    ${PROJECT_NAME}_nodelet
    ${catkin_LIBRARIES}
  )

  ###################
  ## Code_coverage ##
  ###################
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS
        test_${PROJECT_NAME}
    )
  endif()
endif()

#################
## Clang_tools ##
#################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling(
    DISABLE_CLANG_FORMAT
  )
endif(cmake_clang_tools_FOUND)
