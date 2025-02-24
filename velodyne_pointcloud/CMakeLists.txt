cmake_minimum_required(VERSION 3.5.1)
project(any_velodyne_pointcloud)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  angles
  any_velodyne_driver
  any_velodyne_msgs
  diagnostic_updater
  dynamic_reconfigure
  nodelet
  roscpp
  roslib
  sensor_msgs
  tf
  rosbag
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

# Generate dynamic_reconfigure server
generate_dynamic_reconfigure_options(
  cfg/CloudNode.cfg
  cfg/TransformNode.cfg
)

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    any_velodyne_rawdata
    any_velodyne_cloud_nodelet
    any_velodyne_transform_nodelet
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

#############
# Libraries #
#############
add_library(any_velodyne_rawdata
  src/lib/rawdata.cc
  src/lib/calibration.cc
)
add_dependencies(any_velodyne_rawdata
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(any_velodyne_rawdata 
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

######################
# Nodes and nodelets #
######################
# Point cloud.
add_executable(any_velodyne_cloud_node src/conversions/cloud_node.cc
  src/conversions/convert.cc
  src/conversions/pointcloudXYZIRT.cc
  src/conversions/organized_cloudXYZIRT.cc
)
add_dependencies(any_velodyne_cloud_node
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(any_velodyne_cloud_node 
  any_velodyne_rawdata
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_library(any_velodyne_cloud_nodelet src/conversions/cloud_nodelet.cc
  src/conversions/convert.cc
  src/conversions/pointcloudXYZIRT.cc
  src/conversions/organized_cloudXYZIRT.cc
)
add_dependencies(any_velodyne_cloud_nodelet
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(any_velodyne_cloud_nodelet
  any_velodyne_rawdata
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# Transform.
add_executable(any_velodyne_transform_node src/conversions/transform_node.cc
  src/conversions/transform.cc
  src/conversions/pointcloudXYZIRT.cc
  src/conversions/organized_cloudXYZIRT.cc
)
add_dependencies(any_velodyne_transform_node
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(any_velodyne_transform_node
  any_velodyne_rawdata
  ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES}
)

add_library(any_velodyne_transform_nodelet src/conversions/transform_nodelet.cc
  src/conversions/transform.cc
  src/conversions/pointcloudXYZIRT.cc
  src/conversions/organized_cloudXYZIRT.cc
)
add_dependencies(any_velodyne_transform_nodelet
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(any_velodyne_transform_nodelet
  any_velodyne_rawdata
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)


###########
# Install #
###########
install(TARGETS
    any_velodyne_rawdata
    any_velodyne_cloud_node
    any_velodyne_cloud_nodelet
    any_velodyne_transform_node
    any_velodyne_transform_nodelet
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  FILES
    nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  DIRECTORY
    launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(
  DIRECTORY
    params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
)
install(
  PROGRAMS 
    scripts/gen_calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
##############
##   Test   ##
##############
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    tests/empty_test.cpp
  )
  add_dependencies(test_${PROJECT_NAME}
    any_velodyne_cloud_node
    any_velodyne_transform_node
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    include
  )
  target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    any_velodyne_rawdata
    any_velodyne_cloud_nodelet
    any_velodyne_transform_nodelet
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
