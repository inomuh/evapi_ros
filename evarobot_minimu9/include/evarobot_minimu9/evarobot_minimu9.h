#ifdef English_dox
/**
 * \file   evarobot_minimu9.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Controls and publishes IMU data.
 * \details
 */
#endif

#ifndef INCLUDE_EVAROBOT_MINIMU_H_
#define INCLUDE_EVAROBOT_MINIMU_H_

#include "ros/ros.h"
#include <realtime_tools/realtime_publisher.h>

#include "vector.h"
#include "MinIMU9.h"
#include "version.h"

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <sys/time.h>
#include <system_error>
#include <boost/program_options.hpp>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include "sensor_msgs/Imu.h"

#include <sys/types.h>
#include <sys/ipc.h>
#include <semaphore.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <sstream>

#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <evarobot_minimu9/ParamsConfig.h>
#include "ErrorCodes.h"

#ifdef English_dox
//! Semaphore name to prevent deadlock.
#endif
char SEM_NAME[]= "i2c";

namespace opts = boost::program_options;

#ifdef English_dox
//! An Euler angle could take 8 chars: -234.678, but usually we only need 6.
#endif
float field_width = 6;

#ifdef English_dox
//! Defines FLOAT_FORMAT with precision 3.
#endif
#define FLOAT_FORMAT std::fixed << std::setprecision(3) << std::setw(field_width)

#ifdef English_dox
//! If an error occurs, shows its code number.
#endif
int i_error_code = 0;

#ifdef English_dox
//! Controls new parameters are set or not.
#endif
bool b_is_received_params = false;

#endif
