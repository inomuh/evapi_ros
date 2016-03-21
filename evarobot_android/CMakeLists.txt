cmake_minimum_required(VERSION 2.8.3)
project(evarobot_android)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(evarobot_android src/evarobot_android.cpp)

target_link_libraries(evarobot_android
  ${catkin_LIBRARIES}
)
