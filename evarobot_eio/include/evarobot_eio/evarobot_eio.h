//! Bu sınıfa ait dokümantasyon evapi_ros 85rpm altındaki dokümantasyon ile aynıdır.

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
#include "im_msgs/EIOTest.h"

char SEM_NAME[]= "i2c";

using namespace std;

#endif
