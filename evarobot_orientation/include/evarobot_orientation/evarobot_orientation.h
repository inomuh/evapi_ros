//! Bu sınıfa ait dokümantasyon evapi_ros 85rpm altındaki dokümantasyon ile aynıdır.

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

//#ifndef DEBUG
//#define DEBUG
//#endif

//#ifndef TEST
//#define TEST
//#endif

#define SERIAL_PATH "/dev/ttyAMA0"
#define BAUDRATE B115200
#define DATABITS CS8

// 0: No Parity; 
// 1: ODD Parity; 
// 2: Even Parity
#define PARITY 0 
#define STOP_BITS 0

using namespace std;

#endif
