cmake_minimum_required(VERSION 2.8.3)
project(evarobot_controller)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES evarobot_controller
#  CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


 add_executable(evarobot_controller src/evarobot_controller.cpp)
 target_link_libraries(evarobot_controller
   ${catkin_LIBRARIES}
)

