cmake_minimum_required(VERSION 2.8.3)
project(evarobot_controller)

set(EVAPI_LIB_SDK_PATH "../sdk/")
FILE(GLOB EVAPI_LIB_SDK_SRC "${EVAPI_LIB_SDK_PATH}/src/*.cpp")

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  diagnostic_updater
  realtime_tools
)

generate_dynamic_reconfigure_options(cfg/evarobot_controller.cfg)

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
  ${EVAPI_LIB_SDK_PATH}/include
  include
)




 add_executable(evarobot_controller src/evarobot_controller.cpp ${EVAPI_LIB_SDK_SRC})
 add_dependencies(evarobot_controller im_msgs_generate_messages_cpp)
 add_dependencies(evarobot_controller evarobot_controller_gencfg)
 target_link_libraries(evarobot_controller
   ${catkin_LIBRARIES}
)
