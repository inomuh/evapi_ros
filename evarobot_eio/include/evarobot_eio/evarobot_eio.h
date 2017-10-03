#ifndef INCLUDE_EVAROBOT_EIO_H_
#define INCLUDE_EVAROBOT_EIO_H_

#include "ros/ros.h"
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>		/* ioctl */
#include "IMEIO.h"

#include <ros/console.h>
#include "ErrorCodes.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

char SEM_NAME[]= "i2c";

using namespace std;

#endif
