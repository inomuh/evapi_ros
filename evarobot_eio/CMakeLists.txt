cmake_minimum_required(VERSION 2.8.3)
project(evarobot_eio)

set(EVAPI_LIB_SDK_PATH "../sdk/")
FILE(GLOB EVAPI_LIB_SDK_SRC "${EVAPI_LIB_SDK_PATH}/src/*.cpp")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  im_msgs
  diagnostic_updater
)


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

add_executable(evarobot_eio src/evarobot_eio.cpp ${EVAPI_LIB_SDK_SRC})
 target_link_libraries(evarobot_eio 
   ${catkin_LIBRARIES}
   pthread
 )

add_executable(evarobot_rgb src/evarobot_rgb.cpp ${EVAPI_LIB_SDK_SRC})
add_dependencies(evarobot_rgb im_msgs_generate_messages_cpp)
 target_link_libraries(evarobot_rgb 
   ${catkin_LIBRARIES}
   pthread
 )
