/*
 * LTC2943.cpp
 *
 *  Created on: Oct 20, 2015
 *      Author: veli
 */

#include "evarobot_battery/LTC2943.h"
#include <inttypes.h>
#include <stdint.h>

using namespace std;

int LTC2943::resetAlertStatus()
{
	int returnVal = -1;

	returnVal = batteryReader->WriteByte(LTC2943_ALERT_RESPONSE_ADDRESS);

	return returnVal;
}

int LTC2943::setTemperatureThresholdHigh(uint16_t _maxTemp)
{
	int i_status = -1;
	uint16_t maxTemperature;

	maxTemperature = (_maxTemp + 273.15)*(0xFFFF)/(FULLSCALE_TEMPERATURE);

	union
	{
		uint8_t b[2];
		uint16_t w;
	} data;

	data.w = maxTemperature;

	i_status = batteryReader->WriteTwoDataByte(TEMPERATURE_THRESH_HIGH, data.b[1], data.b[0]);

	if(i_status < 0)
	{
		perror("Writing to temperature high threshold register failed.");
		exit(1);
	}

	return i_status;
}

int LTC2943::setTemperatureThresholdLow(uint16_t _minTemp)
{
	int i_status = -1;
	uint16_t minTemperature;

	minTemperature = (_minTemp + 273.15)*(0xFFFF)/(FULLSCALE_TEMPERATURE);

	union
	{
		uint8_t b[2];
		uint16_t w;
	} data;

	data.w = minTemperature;

	i_status = batteryReader->WriteTwoDataByte(TEMPERATURE_THRESH_LOW, data.b[1], data.b[0]);

	if(i_status < 0)
	{
		perror("Writing to temperature low threshold register failed.");
		exit(1);
	}

	return i_status;
}

int LTC2943::setCurrentThresholdHigh(uint16_t _maxCurrent)
{
	int i_status = -1;
	uint16_t maxCurrentCode;

	maxCurrentCode = this->senseResistorValue * _maxCurrent *(0x7FFF)/(FULLSCALE_CURRENT) + 0x7FFF;

	union
	{
		uint8_t b[2];
		uint16_t w;
	} data;

	data.w = maxCurrentCode;

	i_status = batteryReader->WriteTwoDataByte(CURRENT_THRESH_HIGH_MSB, data.b[1], data.b[0]);

	if(i_status < 0)
	{
		perror("Writing to current high threshold register failed.");
		exit(1);
	}

	return i_status;
}

int LTC2943::setCurrentThresholdLow(uint16_t _minCurrent)
{
	int i_status = -1;
	uint16_t minCurrentCode;

	minCurrentCode = this->senseResistorValue * _minCurrent *(0x7FFF)/(FULLSCALE_CURRENT) + 0x7FFF;

	union
	{
		uint8_t b[2];
		uint16_t w;
	} data;

	data.w = minCurrentCode;

	i_status = batteryReader->WriteTwoDataByte(CURRENT_THRESH_LOW_MSB, data.b[1], data.b[0]);

	if(i_status < 0)
	{
		perror("Writing to current low threshold register failed.");
		exit(1);
	}

	return i_status;
}

int LTC2943::setChargeThresholdHigh(uint16_t _maxCharge)
{
	int i_status = -1;

	union
	{
		uint8_t b[2];
		uint16_t w;
	} data;

	data.w = _maxCharge;

	i_status = batteryReader->WriteTwoDataByte(CHARGE_THRESH_HIGH_MSB, data.b[1], data.b[0]);

	if(i_status < 0)
	{
		perror("Writing to charge high threshold register failed.");
		exit(1);
	}

	return i_status;
}

int LTC2943::setChargeThresholdLow(uint16_t _minCharge)
{
	int i_status = -1;

	union
	{
		uint8_t b[2];
		uint16_t w;
	} data;

	data.w = _minCharge;

	i_status = batteryReader->WriteTwoDataByte(CHARGE_THRESH_LOW_MSB, data.b[1], data.b[0]);

	if(i_status < 0)
	{
		perror("Writing to charge low threshold register failed.");
		exit(1);
	}

	return i_status;
}

int LTC2943::setVoltageThresholdHigh(float _maxVoltage)
{
	int i_status = -1;
	uint16_t maxVoltageCode;

	maxVoltageCode = _maxVoltage *(65535.0) / (FULLSCALE_VOLTAGE);

	//cout << "max voltage code: " << endl;
	//cout << maxVoltageCode << endl;


	union
	{
		uint8_t b[2];
		uint16_t w;
	} data;

	data.w = maxVoltageCode;

	i_status = batteryReader->WriteTwoDataByte(VOLTAGE_THRESH_HIGH_MSB, data.b[1], data.b[0]);

	if(i_status < 0)
	{
		perror("Writing to voltage high threshold register failed.");
		exit(1);
	}

	return i_status;
}

int LTC2943::setVoltageThresholdLow(float _minVoltage)
{
	int i_status = -1;
	uint16_t minVoltageCode;

	minVoltageCode = _minVoltage * 65535.0 / (FULLSCALE_VOLTAGE);

	union
	{
		uint8_t b[2];
		uint16_t w;
	} data;

	data.w = minVoltageCode;

	i_status = batteryReader->WriteTwoDataByte(VOLTAGE_THRESH_LOW_MSB, data.b[1], data.b[0]);

	if(i_status < 0)
	{
		perror("Writing to voltage low threshold register failed.");
		exit(1);
	}

	return i_status;
}

int LTC2943::readStatusRegister()
{
	unsigned char statusReg;
	int i_status = -1;

	i_status = batteryReader->ReadDataByte(CONTROL, statusReg);

	if(i_status < 0)
	{
		perror("Read from status register failed.");
		exit(1);
	}

	
	return (int)statusReg;
}

float LTC2943::readTemperature()
{
	float f_temperature = 0.0;
	uint16_t temperatureADCCode;
	int i_status = -1;

	unsigned char u_c_temp_msb_byte = 0;
	unsigned char u_c_temp_lsb_byte = 0;

	i_status = batteryReader->ReadTwoDataByte(TEMPERATURE_MSB, u_c_temp_msb_byte, u_c_temp_lsb_byte);

	if(i_status < 0)
	{
		perror("Read from temperature register failed.");
		exit(1);
	}

	temperatureADCCode = (uint16_t)u_c_temp_msb_byte * 256 + (uint16_t)u_c_temp_lsb_byte;

	f_temperature = calculateTemperatureFromADCCode(temperatureADCCode);

	return f_temperature;
}

float LTC2943::calculateTemperatureFromADCCode(uint16_t _tempCode)
{
	float calculatedTempValue = 0.0;

	calculatedTempValue = _tempCode * ((float)(FULLSCALE_TEMPERATURE)/65535) - 273.15;

	return calculatedTempValue;
}

float LTC2943::readCurrent()
{
	float f_current = 0.0;
	uint16_t currentADCCode;
	int i_status = -1;

	unsigned char u_c_current_msb_byte = 0;
	unsigned char u_c_current_lsb_byte = 0;

	i_status = batteryReader->ReadTwoDataByte(CURRENT_MSB, u_c_current_msb_byte, u_c_current_lsb_byte);

	if(i_status < 0)
	{
		perror("Read from current register failed.");
		exit(1);
	}

	currentADCCode = (uint16_t)u_c_current_msb_byte * 256 + (uint16_t)u_c_current_lsb_byte;

	f_current = calculateCurrentFromADCCode(currentADCCode);

	return f_current;
}

float LTC2943::calculateCurrentFromADCCode(uint16_t _currentCode)
{
	float calculatedCurrentValue = 0.0;

	calculatedCurrentValue = (((float)_currentCode-32767)/(32767))*((float)(FULLSCALE_CURRENT)/this->senseResistorValue);

	return calculatedCurrentValue;
}

float LTC2943::readVoltage()
{
	float f_voltage = 0.0;
	uint16_t voltageADCCode;
	int i_status = -1;

	unsigned char u_c_voltage_msb_byte = 0;
	unsigned char u_c_voltage_lsb_byte = 0;

	i_status = batteryReader->ReadTwoDataByte(VOLTAGE_MSB, u_c_voltage_msb_byte, u_c_voltage_lsb_byte);

	if(i_status < 0)
	{
		perror("Read from voltage register failed.");
		exit(1);
	}

	voltageADCCode = (uint16_t)u_c_voltage_msb_byte * 256 + (uint16_t)u_c_voltage_lsb_byte;

	f_voltage = calculateVoltageFromADCCode(voltageADCCode);

	return f_voltage;

}

float LTC2943::calculateVoltageFromADCCode(uint16_t _voltageCode)
{
	float calculatedVoltageValue = 0.0;

	calculatedVoltageValue = ((float)_voltageCode / (65535)) * FULLSCALE_VOLTAGE;

	return calculatedVoltageValue;
}

float LTC2943::readAccumulatedCharge()
{
	float f_accumulatedCharge = 0.0;
	uint16_t accumulatedChargeADCCode;
	int i_status = -1;

	unsigned char u_c_accumulated_msb_byte = 0;
	unsigned char u_c_accumulated_lsb_byte = 0;

	i_status = batteryReader->ReadTwoDataByte(ACC_CHARGE_MSB, u_c_accumulated_msb_byte, u_c_accumulated_lsb_byte);

	if(i_status < 0)
	{
		perror("Read from accumulated register failed.");
		exit(1);
	}

	accumulatedChargeADCCode = (uint16_t)u_c_accumulated_msb_byte * 256 + (uint16_t)u_c_accumulated_lsb_byte;

	f_accumulatedCharge = calculateAccumulatedChargeFromADCCode(accumulatedChargeADCCode);

	return f_accumulatedCharge;
}

float LTC2943::calculateAccumulatedChargeFromADCCode(uint16_t _accChargeCode)
{
	float calculatedAccValue = 0.0;

	calculatedAccValue = 1000 * (float)(_accChargeCode * CHARGE_LEAST * this->prescalerValue * 0.05) / (this->senseResistorValue * 4096);

	return calculatedAccValue;
}

int LTC2943::setControlRegister(unsigned char _mode, unsigned char _prescalerCode, unsigned char _alccConfiguration)
{
	int i_status = -1;
	int8_t configuration = 0x00;

	configuration = _mode | _prescalerCode | _alccConfiguration;

	i_status = batteryReader->WriteDataByte(CONTROL, configuration);

	if(i_status < 0)
	{
		perror("Writing to control register failed.");
		exit(1);
	}

	switch(int(_prescalerCode))
	{
		case 0x00:
			this->prescalerValue = 1;
			break;
		case 0x08:
			this->prescalerValue = 4;
			break;
		case 0x10:
			this->prescalerValue = 16;
			break;
		case 0x18:
			this->prescalerValue = 64;
			break;
		case 0x20:
			this->prescalerValue = 256;
			break;
		case 0x28:
			this->prescalerValue = 1024;
			break;
		case 0x30:
			this->prescalerValue = 4096;
			break;
		case 0x31:
			this->prescalerValue = 4096;
			break;
		default:
			this->prescalerValue = 4096;
	}

	return i_status;
}

LTC2943::LTC2943(unsigned char u_c_device_address,
				 string str_i2c_file_name,
				 sem_t * mutex)
{
	// TODO Auto-generated constructor stub
	senseResistorValue = 0.05; // 50 mohm

	prescalerValue = 4096; // varsayılan entegre değeri
	prescalerMode = PRESCALER_4096;
	alccMode = ALERT_MODE;

	this->batteryReader = new IMI2C(u_c_device_address, str_i2c_file_name, mutex);

}

LTC2943::~LTC2943()
{
	// TODO Auto-generated destructor stub
	delete this->batteryReader;
}

