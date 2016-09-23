#ifdef English_dox
/**
 * \file   evarobot_eio.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Controls eio pins over i2c.
 * \details
 */
#endif

#ifndef INCLUDE_EVAROBOT_EIO_H_
#define INCLUDE_EVAROBOT_EIO_H_

#include "ros/ros.h"
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "IMEIO.h"

#include <ros/console.h>
#include "ErrorCodes.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#ifdef English_dox
//! Semaphore name to prevent deadlock.
#endif
char SEM_NAME[]= "i2c";

using namespace std;

#ifdef English_dox
//! If an error occurs, shows its code number.
#endif
int i_error_code = 0;

#endif
