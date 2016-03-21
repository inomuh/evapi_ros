/**
 * \file   IMServoPWM.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2016
 * \brief  Control evarobot gripper and pan tilt.
 * \details
 *
 */

#ifndef INCLUDE_IMSERVOPWM_H_
#define INCLUDE_IMSERVOPWM_H_

#include "IMI2C.h"
#include <cmath>
#include <inttypes.h>

using namespace std;

class IMServoPWM
{
	public:
		// Registers/etc.
		static const char __MODE1;
		static const char __MODE2;
		static const char __SUBADR1;
		static const char __SUBADR2;
		static const char __SUBADR3;
		static const char __PRESCALE;
		static const char __LED0_ON_L;
		static const char __LED0_ON_H;
		static const char __LED0_OFF_L;
		static const char __LED0_OFF_H;
		static const char __ALL_LED_ON_L;
		static const char __ALL_LED_ON_H;
		static const char __ALL_LED_OFF_L;
		static const char __ALL_LED_OFF_H;

		// Bits
		static const char __RESTART;
		static const char __SLEEP;
		static const char __ALLCALL;
		static const char __INVRT;
		static const char __OUTDRV;
		
		//! Constructor
		/**
		* \param u_c_device_address address of the device connected to i2c
		* \param str_i2c_file_name i2c driver name
		* \param mutex mutex to use i2c driver simultaneously
		*/
		IMServoPWM(unsigned char u_c_device_address, string str_i2c_file_name, sem_t * mutex);
		~IMServoPWM();
		//void Reset();
		void setPWMFreq(double freq);
		void setPWM(char channel, uint32_t on, uint32_t off);
		void setAllPWM(uint32_t on, uint32_t off);
	
	private:
	
		IMI2C * i2c;
	
};

#endif /* INCLUDE_IMSERVOPWM_H_ */
