cmake_minimum_required(VERSION 2.8.3)
project(ekb_test)

set(EVAPI_LIB_SDK_PATH "../sdk/")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  diagnostic_updater
  im_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES evarobot_sonar
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
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


add_executable(ekb_test src/ekb_test.cpp)
add_dependencies(ekb_test ekb_test_gencfg)
add_dependencies(ekb_test im_msgs_generate_messages_cpp)
target_link_libraries(ekb_test
   ${catkin_LIBRARIES}
)

add_executable(evakit src/evakit.cpp)
add_dependencies(evakit ekb_test_gencfg)
add_dependencies(evakit im_msgs_generate_messages_cpp)
target_link_libraries(evakit
   ${catkin_LIBRARIES}
)
