cmake_minimum_required(VERSION 2.8.3)
project(evarobot_odometry)

set(EVAPI_LIB_SDK_PATH "../sdk/")
FILE(GLOB EVAPI_LIB_SDK_SRC "${EVAPI_LIB_SDK_PATH}/src/*.cpp")

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  geometry_msgs
  roscpp
  dynamic_reconfigure
  std_msgs
  diagnostic_updater
  realtime_tools
)

generate_dynamic_reconfigure_options(cfg/evarobot_odometry.cfg)


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
  ${EVAPI_LIB_SDK_PATH}/include
  include
)


add_executable(evarobot_odometry src/evarobot_odometry.cpp ${EVAPI_LIB_SDK_SRC})
add_dependencies(evarobot_odometry im_msgs_generate_messages_cpp)
add_dependencies(evarobot_odometry evarobot_odometry_gencfg)
target_link_libraries(evarobot_odometry
   ${catkin_LIBRARIES}
 )

