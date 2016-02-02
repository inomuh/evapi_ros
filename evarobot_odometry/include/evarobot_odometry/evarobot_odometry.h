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

#define PI 3.1416

using namespace std;

#endif
