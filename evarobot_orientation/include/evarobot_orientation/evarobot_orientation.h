#ifndef INCLUDE_EVAROBOT_ORIENTATION_H_
#define INCLUDE_EVAROBOT_ORIENTATION_H_

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

#include "nav_msgs/Odometry.h"

#include "IMUM6.h"

#include <sstream>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <ros/console.h>
#include <ErrorCodes.h>

#ifdef English_dox
//! Driver path of um6 IMU
#endif
#define SERIAL_PATH "/dev/ttyAMA0"

#ifdef English_dox
//! Baudrate of um6
#endif
#define BAUDRATE B115200

#ifdef English_dox
//! Databits of um6 IMU
#endif
#define DATABITS CS8

#ifdef English_dox
//! Parity mode selection for message control. 0: No Parity, 1: ODD Parity, 2: Even Parity.
#endif
#define PARITY 0 

#ifdef English_dox
//! Stop bits for message control
#endif
#define STOP_BITS 0

using namespace std;

#ifdef English_dox
//! If an error occurs, shows its code number.
#endif
int i_error_code = 0;

#endif
