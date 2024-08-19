cmake_minimum_required(VERSION 3.5.1)
project(sst_sensor_ring)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  SSTSensorRing.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  #INCLUDE_DIRS 
  #LIBRARIES 
  #CATKIN_DEPENDS
  #DEPENDS 
)
