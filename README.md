evapi_ros
=========

Before evapi_ros
================

 * install ros for raspberry pi (http://wiki.ros.org/ROSberryPi)
 * install i2c-dev library
 > sudo apt-get install libi2c-dev
 * install eigen library
 > sudo apt-get install libeigen3-dev

Create workspace
================
> mkdir -p ~/catkin_ws/src
> cd ~/catkin_ws/src
> catkin_init_workspace
> cd ~/catkin_ws
> catkin_make
> source devel/setup.bash

Install evapi_ros
=================
> cd ~/catkin_ws/src
> git clone https://github.com/inomuh/evapi_ros.git
> cd ~/catkin_ws
> catkin_make
