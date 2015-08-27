cmake_minimum_required(VERSION 2.8.3)
project(evarobot_bumper)

set(bumper_PATH "./sdk/")
set(bumper_INCLUDE_DIRS "${bumper_PATH}/include")
FILE(GLOB bumper_LIBRARIES "${bumper_PATH}/")

find_package(catkin REQUIRED COMPONENTS
  im_msgs
  roscpp
  rospy
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES evarobot_bumper
#  CATKIN_DEPENDS im_msgs roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${bumper_INCLUDE_DIRS}
  ${bumper_LIBRARIES}
  include
)

 add_executable(evarobot_bumper src/evarobot_bumper.cpp)
 target_link_libraries(evarobot_bumper
   ${catkin_LIBRARIES}
   pthread
   eio
 )
