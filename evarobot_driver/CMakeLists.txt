cmake_minimum_required(VERSION 2.8.3)
project(evarobot_driver)

set(driver_PATH "sdk/")
set(driver_INCLUDE_DIRS "${driver_PATH}/include")
set(driver_LIBRARIES "${driver_PATH}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
)



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
	${driver_INCLUDE_DIRS}
  include
)

link_directories(${driver_LIBRARIES})

 add_executable(evarobot_driver src/evarobot_driver.cpp)
 target_link_libraries(evarobot_driver ${catkin_LIBRARIES} driver)
