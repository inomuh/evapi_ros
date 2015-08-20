cmake_minimum_required(VERSION 2.8.3)
project(evarobot_odometry)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  geometry_msgs
  roscpp
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES evarobot_odometry
#  CATKIN_DEPENDS nav_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


add_executable(evarobot_odometry src/evarobot_odometry.cpp)
target_link_libraries(evarobot_odometry
   ${catkin_LIBRARIES}
 )

