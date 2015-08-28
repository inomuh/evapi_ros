cmake_minimum_required(VERSION 2.8.3)
project(evarobot_bumper)

set(EVAPI_LIB_SDK_PATH "../sdk/")
FILE(GLOB EVAPI_LIB_SDK_SRC "${EVAPI_LIB_SDK_PATH}/src/*.cpp")

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
  CATKIN_DEPENDS im_msgs #roscpp rospy
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

 add_executable(evarobot_bumper src/evarobot_bumper.cpp ${EVAPI_LIB_SDK_SRC})
 target_link_libraries(evarobot_bumper
   ${catkin_LIBRARIES}
   pthread
 )
