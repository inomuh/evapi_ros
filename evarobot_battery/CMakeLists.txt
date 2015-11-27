cmake_minimum_required(VERSION 2.8.3)
project(evarobot_battery)

set(EVAPI_LIB_SDK_PATH "../sdk/")
FILE(GLOB EVAPI_LIB_SDK_SRC "${EVAPI_LIB_SDK_PATH}/src/*.cpp")

find_package(catkin REQUIRED COMPONENTS
	im_msgs
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  diagnostic_updater
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS im_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EVAPI_LIB_SDK_PATH}/include
  ${EVAPI_LIB_SDK_PATH}/src  
  include
)

 add_executable(evarobot_battery src/evarobot_battery.cpp ${EVAPI_LIB_SDK_SRC} src/LTC2943.cpp)
  add_dependencies(evarobot_battery im_msgs_generate_messages_cpp)
 target_link_libraries(evarobot_battery
   ${catkin_LIBRARIES}
   pthread
 )
