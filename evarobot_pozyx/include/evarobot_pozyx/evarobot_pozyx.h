#ifndef INCLUDE_EVAROBOT_POZYX_H_
#define INCLUDE_EVAROBOT_POZYX_H_

#define POZYX_POS_ALL 0xFF
#define POZYX_POS_ERR_ALL 0xFE
#define POZYX_QUAT_ALL 0xFD

#include "ros/ros.h"
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>		/* ioctl */
#include "IMSerial.h"
#include <iterator>
#include <list>
#include <sstream>

#include "nav_msgs/Odometry.h"

#include <ros/console.h>
#include "ErrorCodes.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>


/* Begin auto generated defines */ 
/* POZYX REGISTERS firmware version v0.9 */ 

/* Status registers */

#define POZYX_WHO_AM_I 0x0 /* Returns the constant value 0x43. */
#define POZYX_FIRMWARE_VER 0x1 /* Returns the POZYX firmware version. */
#define POZYX_HARDWARE_VER 0x2 /* Returns the POZYX hardware version. */
#define POZYX_ST_RESULT 0x3 /* Returns the self-test result */
#define POZYX_ERRORCODE 0x4 /* Describes a possibly system error. */
#define POZYX_INT_STATUS 0x5 /* Indicates the source of the interrupt. */
#define POZYX_CALIB_STATUS 0x6 /* Returns the calibration status. */

/* Configuration registers */

#define POZYX_INT_MASK 0x10 /* Indicates which interrupts are enabled. */
#define POZYX_CONFIG_GPIO1 0x11 /* Configure GPIO pin 1. */
#define POZYX_CONFIG_GPIO2 0x12 /* Configure GPIO pin 2. */
#define POZYX_CONFIG_GPIO3 0x13 /* Configure GPIO pin 3. */
#define POZYX_CONFIG_GPIO4 0x14 /* Configure GPIO pin 4. */
#define POZYX_CONFIG_LEDS0 x15 /* Configure the LEDs */
#define POZYX_POS_ALG 0x16 /* Algorithm used for positioning */
#define POZYX_POS_NUM_ANCHORS 0x17 /* Configure the number of anchors and selection procedure */
#define POZYX_POS_INTERVAL 0x18 /* Defines the update interval in ms in continuous positioning. */
#define POZYX_NETWORK_ID 0x1A /* The network id.  */
#define POZYX_UWB_CHANNEL 0x1C /* UWB channel number. */
#define POZYX_UWB_RATES 0x1D /* Configure the UWB datarate and pulse repetition frequency (PRF) */
#define POZYX_UWB_PLEN 0x1E /* Configure the UWB preamble length.  */
#define POZYX_UWB_GAIN 0x1F /* Configure the power gain for the UWB transmitter */
#define POZYX_UWB_XTALTRIM 0x20 /* Trimming value for the uwb crystal. */
#define POZYX_OPERATION_MODE 0x22 /* Configure the mode of operation of the pozyx device */
#define POZYX_SENSORS_MODE 0x23 /* Configure the mode of operation of the sensors */

/* Positioning data */

#define POZYX_POS_X 0x30 /* x-coordinate of the device in mm. */
#define POZYX_POS_Y 0x34 /* y-coordinate of the device in mm. */
#define POZYX_POS_Z 0x38 /* z-coordinate of the device in mm. */
#define POZYX_POS_ERR_X 0x3C /* estimated error covariance of x */
#define POZYX_POS_ERR_Y 0x3E /* estimated error covariance of y */
#define POZYX_POS_ERR_Z 0x40 /* estimated error covariance of z */
#define POZYX_POS_ERR_XY 0x42 /* estimated covariance of xy */
#define POZYX_POS_ERR_XZ 0x44 /* estimated covariance of xz */
#define POZYX_POS_ERR_YZ 0x46 /* estimated covariance of yz */

/* Sensor data */

#define POZYX_PRESSURE 0x50 /* Pressure data */
#define POZYX_ACCEL_X 0x54 /* Accelerometer data (in mg) */
#define POZYX_ACCEL_Y 0x56 /*  */
#define POZYX_ACCEL_Z 0x58 /*  */
#define POZYX_MAGN_X 0x5A /* Magnemtometer data */
#define POZYX_MAGN_Y 0x5C /*  */
#define POZYX_MAGN_Z 0x5E /*  */
#define POZYX_GYRO_X 0x60 /* Gyroscope data */
#define POZYX_GYRO_Y 0x62 /*  */
#define POZYX_GYRO_Z 0x64 /*  */
#define POZYX_EUL_HEADING 0x66 /* Euler angles heading (or yaw) */
#define POZYX_EUL_ROLL 0x68 /* Euler angles roll */
#define POZYX_EUL_PITCH 0x6A /* Euler angles pitch */
#define POZYX_QUAT_W 0x6C /* Weight of quaternion. */
#define POZYX_QUAT_X 0x6E /* x of quaternion */
#define POZYX_QUAT_Y 0x70 /* y of quaternion */
#define POZYX_QUAT_Z 0x72 /* z of quaternion */
#define POZYX_LIA_X 0x74 /* Linear acceleration in x-direction */
#define POZYX_LIA_Y 0x76 /*  */
#define POZYX_LIA_Z 0x78 /*  */
#define POZYX_GRAV_X 0x7A /* x-component of gravity vector  */
#define POZYX_GRAV_Y 0x7C /* y-component of gravity vector  */
#define POZYX_GRAV_Z 0x7E /* z-component of gravity vector  */
#define POZYX_TEMPERATURE 0x80 /* Temperature */

/* General data */

#define POZYX_DEVICE_LIST_SIZE 0x81 /* Returns the number of devices stored internally */
#define POZYX_RX_NETWORK_ID 0x82 /* The network id of the latest received message */
#define POZYX_RX_DATA_LEN 0x84 /* The length of the latest received message */
#define POZYX_GPIO1 0x85 /* Value of the GPIO pin 1 */
#define POZYX_GPIO2 0x86 /* Value of the GPIO pin 2 */
#define POZYX_GPIO3 0x87 /* Value of the GPIO pin 3 */
#define POZYX_GPIO4 0x88 /* Value of the GPIO pin 4 */

/*Functions*/

#define POZYX_RESET_SYS 0xB0 /* Reset the Pozyx device */
#define POZYX_LED_CTRL 0xB1 /* Control LEDS 1 to 4 on the board */
#define POZYX_TX_DATA 0xB2 /* Write data in the UWB transmit (TX) buffer */
#define POZYX_TX_SEND 0xB3 /* Transmit the TX buffer to some other pozyx device */
#define POZYX_RX_DATA 0xB4 /* Read data from the UWB receive (RX) buffer */
#define POZYX_DO_RANGING 0xB5 /* Initiate ranging measurement */
#define POZYX_DO_POSITIONING 0xB6 /* Initiate the positioning process.  */
#define POZYX_POS_SET_ANCHOR_IDS 0xB7 /* Set the list of anchor ID's used for positioning.  */
#define POZYX_POS_GET_ANCHOR_IDS 0xB8 /* Read the list of anchor ID's used for positioning. */

/*Device list functions*/

#define POZYX_DEVICES_GETIDS 0xB9 /* Get all the network IDs's of devices in the device list. */
#define POZYX_DEVICES_DISCOVER 0xBA /* Obtain the network ID's of all pozyx devices within range. */
#define POZYX_DEVICES_CALIBRATE 0xBB /* Obtain the coordinates of the pozyx (anchor) devices within range. */
#define POZYX_DEVICES_CLEAR 0xBC /* Clear the list of all pozyx devices. */
#define POZYX_DEVICE_ADD 0xBD /* Add a pozyx device to the devices list */
#define POZYX_DEVICE_GETINFO 0xBE /* Get the stored device information for a given pozyx device */
#define POZYX_DEVICE_GETCOORDS 0xBF /* Get the stored coordinates of a given pozyx device */
#define POZYX_DEVICE_GETRANGEINFO 0xC0 /* Get the stored range inforamation of a given pozyx device */
#define POZYX_CIR_DATA 0xC1 /* Get the channel impulse response (CIR) coefficients */

/* Macro's to test if registers are readable/writable */

#define IS_REG_READABLE(x)  ((((x)==0x0)||((x)==0x1)||((x)==0x2)||((x)==0x3)||((x)==0x4)||((x)==0x5)||((x)==0x6)||((x)==0x10)||((x)==0x11)||((x)==0x12)||((x)==0x13)||((x)==0x14)||((x)==0x15)||((x)==0x16)||((x)==0x17)||((x)==0x18)||((x)==0x1A)||((x)==0x1C)||((x)==0x1D)||((x)==0x1E)||((x)==0x1F)||((x)==0x20)||((x)==0x22)||((x)==0x23)||((x)==0x30)||((x)==0x34)||((x)==0x38)||((x)==0x3C)||((x)==0x3E)||((x)==0x40)||((x)==0x42)||((x)==0x44)||((x)==0x46)||((x)==0x50)||((x)==0x54)||((x)==0x56)||((x)==0x58)||((x)==0x5A)||((x)==0x5C)||((x)==0x5E)||((x)==0x60)||((x)==0x62)||((x)==0x64)||((x)==0x66)||((x)==0x68)||((x)==0x6A)||((x)==0x6C)||((x)==0x6E)||((x)==0x70)||((x)==0x72)||((x)==0x74)||((x)==0x76)||((x)==0x78)||((x)==0x7A)||((x)==0x7C)||((x)==0x7E)||((x)==0x80)||((x)==0x81)||((x)==0x82)||((x)==0x84)||((x)==0x85)||((x)==0x86)||((x)==0x87)||((x)==0x88))?1:0) 
#define IS_REG_WRITABLE(x)  ((((x)==0x10)||((x)==0x11)||((x)==0x12)||((x)==0x13)||((x)==0x14)||((x)==0x15)||((x)==0x16)||((x)==0x17)||((x)==0x18)||((x)==0x1A)||((x)==0x1C)||((x)==0x1D)||((x)==0x1E)||((x)==0x1F)||((x)==0x20)||((x)==0x22)||((x)==0x23)||((x)==0x30)||((x)==0x34)||((x)==0x38)||((x)==0x85)||((x)==0x86)||((x)==0x87)||((x)==0x88))?1:0) 
#define IS_FUNCTIONCALL(x)  ((((x)==0xB0)||((x)==0xB1)||((x)==0xB2)||((x)==0xB3)||((x)==0xB4)||((x)==0xB5)||((x)==0xB6)||((x)==0xB7)||((x)==0xB8)||((x)==0xB9)||((x)==0xBA)||((x)==0xBB)||((x)==0xBC)||((x)==0xBD)||((x)==0xBE)||((x)==0xBF)||((x)==0xC0)||((x)==0xC1))?1:0) 

/* End of auto generated defines */ 



using namespace std;

/*
class IMPOZYX
{
 public:


 private:
};
*/


#endif // INCLUDE_EVAROBOT_POZYX_H_
