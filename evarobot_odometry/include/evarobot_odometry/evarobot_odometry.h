#ifdef English_dox
/**
 * \file   evarobot_odometry.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Calculates and published position using encoder sensor.
 * \detail
 */
#endif

#ifndef INCLUDE_EVAROBOT_ODOMETRY_H_
#define INCLUDE_EVAROBOT_ODOMETRY_H_

#include "ros/ros.h"
#include <realtime_tools/realtime_publisher.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose2D.h"

#include "geometry_msgs/PointStamped.h"
#include "std_srvs/Empty.h"
#include "im_msgs/WheelVel.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <string>    
#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <math.h>

#include <ros/console.h>
#include <ErrorCodes.h>
#include <dynamic_reconfigure/server.h>
#include <evarobot_odometry/ParamsConfig.h>

#ifdef English_dox
//! pi number
#endif
#define PI 3.1416

using namespace std;

#ifdef English_dox
//! Controls new parameters are set or not.
#endif
bool b_is_received_params = false;

#ifdef English_dox
//! Controls evarobot_odometry/reset_odom service is called or not.
#endif
bool b_reset_odom = false;

#ifdef English_dox
//! Separation between left and right wheels.
#endif
double d_wheel_separation;

#ifdef English_dox
//! The height of the robot's center.
#endif
double d_height;

#ifdef English_dox
//! The Gear Ratio is defined as the input speed relative to the output speed.
#endif
int i_gear_ratio;

#ifdef English_dox
//! Cycles per revolution (CPR).
#endif
int i_cpr;

#ifdef English_dox
//! Diameter of the wheels.
#endif
double d_wheel_diameter;

#ifdef English_dox
//! If an error occurs, shows its code number.
#endif
int i_error_code = 0;

#endif
