cmake_minimum_required(VERSION 2.8.3)
project(evarobot_pozyx)

set(EVAPI_LIB_SDK_PATH "../sdk/")
FILE(GLOB EVAPI_LIB_SDK_SRC "${EVAPI_LIB_SDK_PATH}/src/*.cpp")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  diagnostic_updater
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES evarobot_pozyx
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EVAPI_LIB_SDK_PATH}/include
  ${EVAPI_LIB_SDK_PATH}/src  
  include
)

 add_executable(evarobot_pozyx src/evarobot_pozyx.cpp ${EVAPI_LIB_SDK_SRC})
# add_dependencies(evarobot_driver im_msgs_generate_messages_cpp)
# add_dependencies(evarobot_driver evarobot_driver_gencfg)
 target_link_libraries(evarobot_pozyx ${catkin_LIBRARIES})
