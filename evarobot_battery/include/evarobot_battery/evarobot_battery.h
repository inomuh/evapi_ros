#ifdef English_dox
/**
 * \file   evarobot_battery.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Reads battery information from LTC2943 and publishes topic.
 * \details
 */
#endif

#include "ros/ros.h"

#include <iostream>
#include <string>
#include <stdio.h>
#include <iomanip>
#include <stdlib.h>

#include <sys/ipc.h>
#include <semaphore.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "im_msgs/Battery.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <dynamic_reconfigure/server.h>
#include <evarobot_minimu9/ParamsConfig.h>

#include "ErrorCodes.h"
#include "IMI2C.h"
#include "evarobot_battery/LTC2943.h"

using namespace std;

#ifdef English_dox
//! If an error occurs, shows its code number.
#endif
int i_error_code = 0;

#ifdef English_dox
//! Semaphore name to prevent deadlock.
#endif
char SEM_NAME[]= "i2c";
