cmake_minimum_required(VERSION 2.8.3)
project(evarobot_sonar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  dynamic_reconfigure
  diagnostic_updater
)

generate_dynamic_reconfigure_options(cfg/evarobot_sonar.cfg)

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
  include
)


add_executable(evarobot_sonar src/evarobot_sonar.cpp)
add_dependencies(evarobot_sonar evarobot_sonar_gencfg)
target_link_libraries(evarobot_sonar
   ${catkin_LIBRARIES}
)
