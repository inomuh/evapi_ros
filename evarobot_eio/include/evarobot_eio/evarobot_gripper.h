#ifndef INCLUDE_EVAROBOT_GRIPPER_H_
#define INCLUDE_EVAROBOT_GRIPPER_H_

/**
 * \file   evarobot_gripper.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2016
 * \brief  Control evarobot gripper and pan tilt.
 * \details
 *
 */

#include "ros/ros.h"
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>		/* ioctl */
#include "IMEIO.h"
#include "IMServoPWM.h"


#include <ros/console.h>
#include "ErrorCodes.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include "std_srvs/Empty.h"

char SEM_NAME[]= "i2c";

using namespace std;

#endif
