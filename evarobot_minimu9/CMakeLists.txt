cmake_minimum_required(VERSION 2.8.3)
project(evarobot_minimu9)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

set(MINIMU9_LIB_SDK_PATH "../minimu9/")
FILE(GLOB MINIMU9_LIB_SDK_SRC "${MINIMU9_LIB_SDK_PATH}/src/*.cpp")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  diagnostic_updater
)

generate_dynamic_reconfigure_options(cfg/evarobot_minimu9.cfg)

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
  ${MINIMU9_LIB_SDK_PATH}/include
  ${MINIMU9_LIB_SDK_PATH}/src  
  include
)


add_executable(evarobot_minimu9 src/evarobot_minimu9.cpp ${MINIMU9_LIB_SDK_SRC})
add_dependencies(evarobot_minimu9 evarobot_minimu9_gencfg)
target_link_libraries(evarobot_minimu9
   ${catkin_LIBRARIES}
   boost_program_options
   pthread
 )

