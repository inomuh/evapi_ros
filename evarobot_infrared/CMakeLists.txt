cmake_minimum_required(VERSION 2.8.3)
project(evarobot_infrared)

set(adc_PATH "sdk/")
set(adc_INCLUDE_DIRS "${adc_PATH}/include")
set(adc_LIBRARIES "${adc_PATH}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES evarobot_infrared
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${adc_INCLUDE_DIRS}
  include
)

link_directories(${adc_LIBRARIES})

add_executable(evarobot_infrared src/evarobot_infrared.cpp)
target_link_libraries(evarobot_infrared ${catkin_LIBRARIES} adc)

