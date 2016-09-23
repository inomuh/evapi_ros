#ifdef English_dox
/**
 * \file   evarobot_pozyx.h
 * \brief  Calculates position using POZYX sensor.
 * \detail
 */
#endif

#ifndef INCLUDE_EVAROBOT_POZYX_H_
#define INCLUDE_EVAROBOT_POZYX_H_

#define POZYX_POS_ALL 0xFF
#define POZYX_POS_ERR_ALL 0xFE
#define POZYX_QUAT_ALL 0xFD

#include "ros/ros.h"
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "IMSerial.h"
#include <iterator>
#include <list>
#include <vector>
#include <sstream>

#include "nav_msgs/Odometry.h"

#include <ros/console.h>
#include "ErrorCodes.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>


/* Begin auto generated defines */ 
/* POZYX REGISTERS firmware version v0.9 */ 

/* Status registers */
#ifdef English_dox
//! Returns the constant value 0x43.
#endif
#define POZYX_WHO_AM_I 0x0
#ifdef English_dox
//! Returns the POZYX firmware version.
#endif
#define POZYX_FIRMWARE_VER 0x1
#ifdef English_dox
//! Returns the POZYX hardware version.
#endif
#define POZYX_HARDWARE_VER 0x2
#ifdef English_dox
//! Returns the self-test result.
#endif
#define POZYX_ST_RESULT 0x3
#ifdef English_dox
//! Describes a possibly system error.
#endif
#define POZYX_ERRORCODE 0x4
#ifdef English_dox
//! Indicates the source of the interrupt.
#endif
#define POZYX_INT_STATUS 0x5
#ifdef English_dox
//!Returns the calibration status.
#endif
#define POZYX_CALIB_STATUS 0x6

/* Configuration registers */
#ifdef English_dox
//! Indicates which interrupts are enabled.
#endif
#define POZYX_INT_MASK 0x10
#ifdef English_dox
//! Configure GPIO pin 1.
#endif
#define POZYX_CONFIG_GPIO1 0x11
#ifdef English_dox
//! Configure GPIO pin 2.
#endif
#define POZYX_CONFIG_GPIO2 0x12
#ifdef English_dox
//! Configure GPIO pin 3.
#endif
#define POZYX_CONFIG_GPIO3 0x13
#ifdef English_dox
//! Configure GPIO pin 4.
#endif
#define POZYX_CONFIG_GPIO4 0x14
#ifdef English_dox
//! Configure the LEDs
#endif
#define POZYX_CONFIG_LEDS0 x15
#ifdef English_dox
//! Algorithm used for positioning
#endif
#define POZYX_POS_ALG 0x16
#ifdef English_dox
//! Configure the number of anchors and selection procedure
#endif
#define POZYX_POS_NUM_ANCHORS 0x17
#ifdef English_dox
//! Defines the update interval in ms in continuous positioning.
#endif
#define POZYX_POS_INTERVAL 0x18
#ifdef English_dox
//! The network id.
#endif
#define POZYX_NETWORK_ID 0x1A
#ifdef English_dox
//! UWB channel number.
#endif
#define POZYX_UWB_CHANNEL 0x1C
#ifdef English_dox
//! Configure the UWB datarate and pulse repetition frequency (PRF)
#endif
#define POZYX_UWB_RATES 0x1D
#ifdef English_dox
//! Configure the UWB preamble length.
#endif
#define POZYX_UWB_PLEN 0x1E
#ifdef English_dox
//! Configure the power gain for the UWB transmitter
#endif
#define POZYX_UWB_GAIN 0x1F
#ifdef English_dox
//! Trimming value for the uwb crystal.
#endif
#define POZYX_UWB_XTALTRIM 0x20
#ifdef English_dox
//! Configure the mode of operation of the pozyx device
#endif
#define POZYX_OPERATION_MODE 0x22
#ifdef English_dox
//! Configure the mode of operation of the sensors
#endif
#define POZYX_SENSORS_MODE 0x23

/* Positioning data */
#ifdef English_dox
//! x-coordinate of the device in mm.
#endif
#define POZYX_POS_X 0x30
#ifdef English_dox
//! y-coordinate of the device in mm.
#endif
#define POZYX_POS_Y 0x34
#ifdef English_dox
//! z-coordinate of the device in mm.
#endif
#define POZYX_POS_Z 0x38
#ifdef English_dox
//! estimated error covariance of x
#endif
#define POZYX_POS_ERR_X 0x3C
#ifdef English_dox
//! estimated error covariance of y
#endif
#define POZYX_POS_ERR_Y 0x3E
#ifdef English_dox
//! estimated error covariance of z
#endif
#define POZYX_POS_ERR_Z 0x40
#ifdef English_dox
//! estimated covariance of xy
#endif
#define POZYX_POS_ERR_XY 0x42
#ifdef English_dox
//! estimated covariance of xz
#endif
#define POZYX_POS_ERR_XZ 0x44
#ifdef English_dox
//! estimated covariance of yz
#endif
#define POZYX_POS_ERR_YZ 0x46

/* Sensor data */
#ifdef English_dox
//! Pressure data
#endif
#define POZYX_PRESSURE 0x50
#ifdef English_dox
//! Accelerometer data (in mg) in x-direction
#endif
#define POZYX_ACCEL_X 0x54
#ifdef English_dox
//! Accelerometer data (in mg) in y-direction
#endif
#define POZYX_ACCEL_Y 0x56
#ifdef English_dox
//! Accelerometer data (in mg) in z-direction
#endif
#define POZYX_ACCEL_Z 0x58
#ifdef English_dox
//! Magnemtometer data in x-direction
#endif
#define POZYX_MAGN_X 0x5A
#ifdef English_dox
//! Magnemtometer data in y-direction
#endif
#define POZYX_MAGN_Y 0x5C
#ifdef English_dox
//! Magnemtometer data in z-direction
#endif
#define POZYX_MAGN_Z 0x5E
#ifdef English_dox
//! Gyroscope data in x-direction
#endif
#define POZYX_GYRO_X 0x60
#ifdef English_dox
//! Gyroscope data in y-direction
#endif
#define POZYX_GYRO_Y 0x62
#ifdef English_dox
//! Gyroscope data in z-direction
#endif
#define POZYX_GYRO_Z 0x64
#ifdef English_dox
//! Euler angles heading (or yaw)
#endif
#define POZYX_EUL_HEADING 0x66
#ifdef English_dox
//! Euler angles roll
#endif
#define POZYX_EUL_ROLL 0x68
#ifdef English_dox
//! Euler angles pitch
#endif
#define POZYX_EUL_PITCH 0x6A
#ifdef English_dox
//! Weight of quaternion.
#endif
#define POZYX_QUAT_W 0x6C
#ifdef English_dox
//! x of quaternion
#endif
#define POZYX_QUAT_X 0x6E
#ifdef English_dox
//! y of quaternion
#endif
#define POZYX_QUAT_Y 0x70
#ifdef English_dox
//! z of quaternion
#endif
#define POZYX_QUAT_Z 0x72
#ifdef English_dox
//! Linear acceleration in x-direction
#endif
#define POZYX_LIA_X 0x74
#ifdef English_dox
//! Linear acceleration in y-direction
#endif
#define POZYX_LIA_Y 0x76
#ifdef English_dox
//! Linear acceleration in z-direction
#endif
#define POZYX_LIA_Z 0x78
#ifdef English_dox
//! x-component of gravity vector
#endif
#define POZYX_GRAV_X 0x7A
#ifdef English_dox
//! y-component of gravity vector
#endif
#define POZYX_GRAV_Y 0x7C
#ifdef English_dox
//! z-component of gravity vector
#endif
#define POZYX_GRAV_Z 0x7E
#ifdef English_dox
//! Temperature
#endif
#define POZYX_TEMPERATURE 0x80

/* General data */
#ifdef English_dox
//! Returns the number of devices stored internally
#endif
#define POZYX_DEVICE_LIST_SIZE 0x81
#ifdef English_dox
//! The network id of the latest received message
#endif
#define POZYX_RX_NETWORK_ID 0x82
#ifdef English_dox
//! The length of the latest received message
#endif
#define POZYX_RX_DATA_LEN 0x84
#ifdef English_dox
//! Value of the GPIO pin 1
#endif
#define POZYX_GPIO1 0x85
#ifdef English_dox
//! Value of the GPIO pin 2
#endif
#define POZYX_GPIO2 0x86
#ifdef English_dox
//! Value of the GPIO pin 3
#endif
#define POZYX_GPIO3 0x87
#ifdef English_dox
//! Value of the GPIO pin 4
#endif
#define POZYX_GPIO4 0x88

/*Functions*/
#ifdef English_dox
//! Reset the Pozyx device
#endif
#define POZYX_RESET_SYS 0xB0
#ifdef English_dox
//! Control LEDS 1 to 4 on the board
#endif
#define POZYX_LED_CTRL 0xB1
#ifdef English_dox
//! Write data in the UWB transmit (TX) buffer
#endif
#define POZYX_TX_DATA 0xB2
#ifdef English_dox
//! Transmit the TX buffer to some other pozyx device
#endif
#define POZYX_TX_SEND 0xB3
#ifdef English_dox
//! Read data from the UWB receive (RX) buffer
#endif
#define POZYX_RX_DATA 0xB4
#ifdef English_dox
//! Initiate ranging measurement
#endif
#define POZYX_DO_RANGING 0xB5
#ifdef English_dox
//! Initiate the positioning process.
#endif
#define POZYX_DO_POSITIONING 0xB6
#ifdef English_dox
//! Set the list of anchor ID's used for positioning.
#endif
#define POZYX_POS_SET_ANCHOR_IDS 0xB7
#ifdef English_dox
//! Read the list of anchor ID's used for positioning.
#endif
#define POZYX_POS_GET_ANCHOR_IDS 0xB8

/*Device list functions*/
#ifdef English_dox
//! Get all the network IDs's of devices in the device list.
#endif
#define POZYX_DEVICES_GETIDS 0xB9
#ifdef English_dox
//! Obtain the network ID's of all pozyx devices within range.
#endif
#define POZYX_DEVICES_DISCOVER 0xBA
#ifdef English_dox
//! Obtain the coordinates of the pozyx (anchor) devices within range.
#endif
#define POZYX_DEVICES_CALIBRATE 0xBB
#ifdef English_dox
//! Clear the list of all pozyx devices.
#endif
#define POZYX_DEVICES_CLEAR 0xBC
#ifdef English_dox
//! Add a pozyx device to the devices list
#endif
#define POZYX_DEVICE_ADD 0xBD
#ifdef English_dox
//! Get the stored device information for a given pozyx device
#endif
#define POZYX_DEVICE_GETINFO 0xBE
#ifdef English_dox
//! Get the stored coordinates of a given pozyx device
#endif
#define POZYX_DEVICE_GETCOORDS 0xBF
#ifdef English_dox
//! Get the stored range information of a given pozyx device
#endif
#define POZYX_DEVICE_GETRANGEINFO 0xC0
#ifdef English_dox
//! Get the channel impulse response (CIR) coefficients
#endif
#define POZYX_CIR_DATA 0xC1

#ifdef English_dox
//! Macro's to test if registers are readable/writable
#endif
#define IS_REG_READABLE(x)  ((((x)==0x0)||((x)==0x1)||((x)==0x2)||((x)==0x3)||((x)==0x4)||((x)==0x5)||((x)==0x6)||((x)==0x10)||((x)==0x11)||((x)==0x12)||((x)==0x13)||((x)==0x14)||((x)==0x15)||((x)==0x16)||((x)==0x17)||((x)==0x18)||((x)==0x1A)||((x)==0x1C)||((x)==0x1D)||((x)==0x1E)||((x)==0x1F)||((x)==0x20)||((x)==0x22)||((x)==0x23)||((x)==0x30)||((x)==0x34)||((x)==0x38)||((x)==0x3C)||((x)==0x3E)||((x)==0x40)||((x)==0x42)||((x)==0x44)||((x)==0x46)||((x)==0x50)||((x)==0x54)||((x)==0x56)||((x)==0x58)||((x)==0x5A)||((x)==0x5C)||((x)==0x5E)||((x)==0x60)||((x)==0x62)||((x)==0x64)||((x)==0x66)||((x)==0x68)||((x)==0x6A)||((x)==0x6C)||((x)==0x6E)||((x)==0x70)||((x)==0x72)||((x)==0x74)||((x)==0x76)||((x)==0x78)||((x)==0x7A)||((x)==0x7C)||((x)==0x7E)||((x)==0x80)||((x)==0x81)||((x)==0x82)||((x)==0x84)||((x)==0x85)||((x)==0x86)||((x)==0x87)||((x)==0x88))?1:0) 
#define IS_REG_WRITABLE(x)  ((((x)==0x10)||((x)==0x11)||((x)==0x12)||((x)==0x13)||((x)==0x14)||((x)==0x15)||((x)==0x16)||((x)==0x17)||((x)==0x18)||((x)==0x1A)||((x)==0x1C)||((x)==0x1D)||((x)==0x1E)||((x)==0x1F)||((x)==0x20)||((x)==0x22)||((x)==0x23)||((x)==0x30)||((x)==0x34)||((x)==0x38)||((x)==0x85)||((x)==0x86)||((x)==0x87)||((x)==0x88))?1:0) 
#define IS_FUNCTIONCALL(x)  ((((x)==0xB0)||((x)==0xB1)||((x)==0xB2)||((x)==0xB3)||((x)==0xB4)||((x)==0xB5)||((x)==0xB6)||((x)==0xB7)||((x)==0xB8)||((x)==0xB9)||((x)==0xBA)||((x)==0xBB)||((x)==0xBC)||((x)==0xBD)||((x)==0xBE)||((x)==0xBF)||((x)==0xC0)||((x)==0xC1))?1:0) 

#ifdef English_dox
//! Maximum frame size that can be sent one time.
#endif
#define RX_BUFFER_SIZE 500

#ifdef English_dox
//! Structure of a packet.
#endif
struct PACKET_STRUCT
{
  char address;
  char packet_type;
  char checksum;

  uint8_t data_length;
  char data[120];
};

/* End of auto generated defines */ 

using namespace std;

#endif // INCLUDE_EVAROBOT_POZYX_H_
