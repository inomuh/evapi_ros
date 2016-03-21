/*
 * IMServoPWM.cpp
 *
 *  Created on: Mar 16, 2016
 *      Author: makcakoca
 */
 
#include "../include/IMServoPWM.h"

// Registers/etc.
const char IMServoPWM::__MODE1              = 0x00;
const char IMServoPWM::__MODE2              = 0x01;
const char IMServoPWM::__SUBADR1            = 0x02;
const char IMServoPWM::__SUBADR2            = 0x03;
const char IMServoPWM::__SUBADR3            = 0x04;
const char IMServoPWM::__PRESCALE           = 0xFE;
const char IMServoPWM::__LED0_ON_L          = 0x06;
const char IMServoPWM::__LED0_ON_H          = 0x07;
const char IMServoPWM::__LED0_OFF_L         = 0x08;
const char IMServoPWM::__LED0_OFF_H         = 0x09;
const char IMServoPWM::__ALL_LED_ON_L       = 0xFA;
const char IMServoPWM::__ALL_LED_ON_H       = 0xFB;
const char IMServoPWM::__ALL_LED_OFF_L      = 0xFC;
const char IMServoPWM::__ALL_LED_OFF_H      = 0xFD;

// Bits
const char IMServoPWM::__RESTART            = 0x80;
const char IMServoPWM::__SLEEP              = 0x10;
const char IMServoPWM::__ALLCALL            = 0x01;
const char IMServoPWM::__INVRT              = 0x10;
const char IMServoPWM::__OUTDRV             = 0x04;

// Constructor
IMServoPWM::IMServoPWM(unsigned char u_c_device_address, string str_i2c_file_name, sem_t * mutex)
{
	char mode1;
	
	try
	{
		this->i2c = new IMI2C(u_c_device_address, str_i2c_file_name, mutex);
		
		this->setAllPWM(0, 0);
		this->i2c->WriteDataByte(IMServoPWM::__MODE2, IMServoPWM::__OUTDRV);
		this->i2c->WriteDataByte(IMServoPWM::__MODE1, IMServoPWM::__ALLCALL);
		
		usleep(5000);                                       // wait for oscillator
		
		
		this->i2c->ReadDataByte(IMServoPWM::__MODE1, mode1);
		mode1 = mode1 & ~IMServoPWM::__SLEEP;             // wake up (reset sleep)
		this->i2c->WriteDataByte(IMServoPWM::__MODE1, mode1);
		usleep(5000); // wait for oscillator	
	}
	catch(int i)
	{
		throw i;
	}
                            
}

IMServoPWM::~IMServoPWM()
{
	delete this->i2c;
}


// Sets a all PWM channels
void IMServoPWM::setAllPWM(uint32_t on, uint32_t off)
{
	try
	{
		this->i2c->WriteDataByte(IMServoPWM::__ALL_LED_ON_L, on & 0xFF);
		this->i2c->WriteDataByte(IMServoPWM::__ALL_LED_ON_H, on >> 8);
		this->i2c->WriteDataByte(IMServoPWM::__ALL_LED_OFF_L, off & 0xFF);
		this->i2c->WriteDataByte(IMServoPWM::__ALL_LED_OFF_H, off >> 8);
	}
	catch(int i)
	{
		throw i;
	}

}

// Sets a single PWM channel
void IMServoPWM::setPWM(char channel, uint32_t on, uint32_t off)
{
	try
	{
		this->i2c->WriteDataByte(IMServoPWM::__LED0_ON_L+4*channel, on & 0xFF);
		this->i2c->WriteDataByte(IMServoPWM::__LED0_ON_H+4*channel, on >> 8);
		this->i2c->WriteDataByte(IMServoPWM::__LED0_OFF_L+4*channel, off & 0xFF);
		this->i2c->WriteDataByte(IMServoPWM::__LED0_OFF_H+4*channel, off >> 8);
	}
	catch(int i)
	{
		throw i;
	}

}

// Sets the PWM frequency
void IMServoPWM::setPWMFreq(double freq)
{
	int prescale;
	char oldmode, newmode;
	double prescaleval = 25000000.0;    // 25MHz
	prescaleval /= 4096.0;       // 12-bit
	prescaleval /= freq;
	prescaleval -= 1.0;
	
	prescale = floor(prescaleval + 0.5);

	try
	{
		this->i2c->ReadDataByte(IMServoPWM::__MODE1, oldmode);
		newmode = (oldmode & 0x7F) | 0x10;             // sleep
		this->i2c->WriteDataByte(IMServoPWM::__MODE1, newmode); // go to sleep

		this->i2c->WriteDataByte(IMServoPWM::__PRESCALE, char(prescale));
		this->i2c->WriteDataByte(IMServoPWM::__MODE1, oldmode);
		
		usleep(5000);
		this->i2c->WriteDataByte(IMServoPWM::__MODE1, oldmode | 0x80);
	}
	catch(int i)
	{
		throw i;
	}

}
