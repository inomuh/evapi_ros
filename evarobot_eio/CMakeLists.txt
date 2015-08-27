cmake_minimum_required(VERSION 2.8.3)
project(evarobot_eio)

set(eio_PATH "./sdk/")
set(eio_INCLUDE_DIRS "${eio_PATH}/include")
FILE(GLOB eio_LIBRARIES "${eio_PATH}/")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)


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
  ${eio_INCLUDE_DIRS}
  ${eio_LIBRARIES}
  include
)

 add_executable(evarobot_eio src/evarobot_eio.cpp)
 target_link_libraries(evarobot_eio 
   ${catkin_LIBRARIES}
   pthread
   eio
 )
