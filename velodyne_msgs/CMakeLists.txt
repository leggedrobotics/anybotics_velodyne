cmake_minimum_required(VERSION 3.5.1)
project(any_velodyne_msgs)

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
)

# Declare the service and message files to be built
add_message_files(
  DIRECTORY
    msg
  FILES
    VelodynePacket.msg
    VelodyneScan.msg
    VelodyneSyncStatus.msg
    VelodyneInterfaceStatus.msg
)
generate_messages(
  DEPENDENCIES
    std_msgs
)

# Catkin package macro
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
