cmake_minimum_required(VERSION 2.8.3)
project(evarobot_driver)

set(EVAPI_LIB_SDK_PATH "../sdk/")
FILE(GLOB EVAPI_LIB_SDK_SRC "${EVAPI_LIB_SDK_PATH}/src/*.cpp")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(cfg/evarobot_driver.cfg)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES evarobot_driver
#  CATKIN_DEPENDS geometry_msgs roscpp rospy
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

 add_executable(evarobot_driver src/evarobot_driver.cpp ${EVAPI_LIB_SDK_SRC})
 add_dependencies(evarobot_driver evarobot_driver_gencfg)
 target_link_libraries(evarobot_driver ${catkin_LIBRARIES})
