cmake_minimum_required(VERSION 2.8.3)
project(im_msgs)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
  visualization_msgs
  message_generation
)



## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   BumperState.msg
   Bumper.msg
 )
 
add_service_files(
  FILES
  SetRGB.srv
)


 generate_messages(
   DEPENDENCIES
   geometry_msgs   nav_msgs   sensor_msgs   std_msgs   trajectory_msgs   visualization_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES im_msgs
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs trajectory_msgs visualization_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
