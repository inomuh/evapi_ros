cmake_minimum_required(VERSION 2.8.3)
project(evarobot_infrared)

set(EVAPI_LIB_SDK_PATH "../sdk/")
FILE(GLOB EVAPI_LIB_SDK_SRC "${EVAPI_LIB_SDK_PATH}/src/*.cpp")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  dynamic_reconfigure
  diagnostic_updater
)

generate_dynamic_reconfigure_options(cfg/evarobot_infrared.cfg)

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
  ${EVAPI_LIB_SDK_PATH}/include
  ${EVAPI_LIB_SDK_PATH}/src  
  include
)

add_executable(evarobot_infrared src/evarobot_infrared.cpp ${EVAPI_LIB_SDK_SRC})
add_dependencies(evarobot_infrared evarobot_infrared_gencfg)
target_link_libraries(evarobot_infrared ${catkin_LIBRARIES})

