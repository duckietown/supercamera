cmake_minimum_required(VERSION 2.8.3)
project(histogram_perception)

find_package(catkin REQUIRED COMPONENTS
  rospy
  duckietown_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
