cmake_minimum_required(VERSION 3.5.1)
project(any_velodyne_laserscan)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  nodelet
  sensor_msgs
  dynamic_reconfigure
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

generate_dynamic_reconfigure_options(
  cfg/VelodyneLaserScan.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_nodelet
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

#############
# Libraries #
#############
add_library(${PROJECT_NAME}
  src/velodyne_laserscan.cpp
)
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

######################
# Nodes and nodelets #
######################
add_library(${PROJECT_NAME}_nodelet
  src/nodelet.cpp
)
add_dependencies(${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_node
  src/node.cpp
)
add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

###########
# Install #
###########
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_nodelet
    ${PROJECT_NAME}_node
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
