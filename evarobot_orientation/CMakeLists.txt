cmake_minimum_required(VERSION 2.8.3)
project(evarobot_orientation)

set(orientation_PATH "./sdk/")
set(orientation_INCLUDE_DIRS "${orientation_PATH}/include")
FILE(GLOB orientation_LIBRARIES "${orientation_PATH}/*.so")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES evarobot_orientation
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${orientation_INCLUDE_DIRS}
  include
)

 add_executable(evarobot_orientation src/evarobot_orientation.cpp)
 target_link_libraries(evarobot_orientation ${catkin_LIBRARIES} ${orientation_LIBRARIES})

