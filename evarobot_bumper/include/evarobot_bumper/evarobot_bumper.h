#ifdef English_dox
/**
 * \file   evarobot_bumper.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Reads collision information from bumper sensors and publishes topic.
 * \details
 */
#endif

#ifndef INCLUDE_EVAROBOT_BUMPER_H_
#define INCLUDE_EVAROBOT_BUMPER_H_

#include "ros/ros.h"

#include "std_msgs/Bool.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <vector>
#include <sstream>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <evarobot_bumper/ParamsConfig.h>
#include "IMEIO.h"
#include "ErrorCodes.h"
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

#ifdef English_dox
//! Maximum number of bumper sensors.
#endif
#define MAX_BUMPER 1

using namespace std;

#ifdef English_dox
//! Semaphore name to prevent deadlock.
#endif
char SEM_NAME[]= "i2c";

#ifdef English_dox
//! If an error occurs, shows its code number.
#endif
int i_error_code = 0;

#ifdef English_dox
//! If this variable false; bumper topic is published only there is at least one subscriber. If this variable true; bumper topic is published in all conditions.
#endif
bool b_always_on;

#ifdef English_dox
//! Controls new parameters are set or not.
#endif
bool b_is_received_params = false;

#ifdef English_dox
//! Values of bumper sensors.
#endif
bool b_collision = true;

#endif
