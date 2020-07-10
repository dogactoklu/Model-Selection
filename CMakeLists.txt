cmake_minimum_required(VERSION 2.8.3)
project(battery_arena)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs

  gazebo_ros
)


catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    message_runtime
)


# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# For battery plugin
add_library(BatteryPlugin plugins/BatteryPlugin.cc)
add_dependencies(BatteryPlugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(BatteryPlugin ${roscpp_LIBRARIES} ${GAZEBO_LIBRARIES})

# For better reset plugin
add_library(BetterReset plugins/BetterReset.cc)
add_dependencies(BetterReset ${catkin_EXPORTED_TARGETS})
target_link_libraries(BetterReset ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)
