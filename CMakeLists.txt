cmake_minimum_required(VERSION 3.0.2)
project(rosbag_test_pkg)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
)


catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


