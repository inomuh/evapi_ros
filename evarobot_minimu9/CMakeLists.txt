cmake_minimum_required(VERSION 2.8.3)
project(evarobot_minimu9)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

set(minimu_PATH "./sdk/")
set(minimu_INCLUDE_DIRS "${minimu_PATH}/include")
FILE(GLOB minimu_LIBRARIES "${minimu_PATH}")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES evarobot_minimu9
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${minimu_INCLUDE_DIRS}
  ${minimu_LIBRARIES}
  include
)

add_executable(evarobot_minimu9 src/evarobot_minimu9.cpp)
target_link_libraries(evarobot_minimu9
   ${catkin_LIBRARIES}
   minimu9
   boost_program_options
   pthread
 )

