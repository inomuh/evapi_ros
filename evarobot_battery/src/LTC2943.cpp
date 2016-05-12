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

void LTC2943::resetAlertStatus()
{
	try
	{
		this->batteryReader->WriteByte(LTC2943_ALERT_RESPONSE_ADDRESS);
	}catch(int e)
	{
		throw e;
	} 
}

void LTC2943::setTemperatureThresholdHigh(uint16_t _maxTemp)
{
	uint16_t maxTemperature;

	maxTemperature = (_maxTemp + 273.15)*(0xFFFF)/(FULLSCALE_TEMPERATURE);

	union
	{
		uint8_t b[2];
		uint16_t w;
	} data;

	data.w = maxTemperature;

	try
	{
		this->batteryReader->WriteTwoDataByte(TEMPERATURE_THRESH_HIGH, data.b[1], data.b[0]);
	}catch(int e)
	{
		throw e;
	} 

}

void LTC2943::setTemperatureThresholdLow(uint16_t _minTemp)
{
	uint16_t minTemperature;

	minTemperature = (_minTemp + 273.15)*(0xFFFF)/(FULLSCALE_TEMPERATURE);

	union
	{
		uint8_t b[2];
		uint16_t w;
	} data;

	data.w = minTemperature;

	try
	{
		this->batteryReader->WriteTwoDataByte(TEMPERATURE_THRESH_LOW, data.b[1], data.b[0]);
	}catch(int e)
	{
		throw e;
	}

}

void LTC2943::setCurrentThresholdHigh(uint16_t _maxCurrent)
{
	uint16_t maxCurrentCode;

	maxCurrentCode = this->senseResistorValue * _maxCurrent *(0x7FFF)/(FULLSCALE_CURRENT) + 0x7FFF;

	union
	{
		uint8_t b[2];
		uint16_t w;
	} data;

	data.w = maxCurrentCode;

	try
	{
		this->batteryReader->WriteTwoDataByte(CURRENT_THRESH_HIGH_MSB, data.b[1], data.b[0]);
	}catch(int e)
	{
		throw e;
	}

}

void LTC2943::setCurrentThresholdLow(uint16_t _minCurrent)
{
	uint16_t minCurrentCode;

	minCurrentCode = this->senseResistorValue * _minCurrent *(0x7FFF)/(FULLSCALE_CURRENT) + 0x7FFF;

	union
	{
		uint8_t b[2];
		uint16_t w;
	} data;

	data.w = minCurrentCode;

	try
	{
		this->batteryReader->WriteTwoDataByte(CURRENT_THRESH_LOW_MSB, data.b[1], data.b[0]);
	}catch(int e)
	{
		throw e;
	}
}

void LTC2943::setChargeThresholdHigh(uint16_t _maxCharge)
{
	union
	{
		uint8_t b[2];
		uint16_t w;
	} data;

	data.w = _maxCharge;

	try
	{
		this->batteryReader->WriteTwoDataByte(CHARGE_THRESH_HIGH_MSB, data.b[1], data.b[0]);
	}catch(int e)
	{
		throw e;
	}
}

void LTC2943::setChargeThresholdLow(uint16_t _minCharge)
{
	union
	{
		uint8_t b[2];
		uint16_t w;
	} data;

	data.w = _minCharge;

	try
	{
		this->batteryReader->WriteTwoDataByte(CHARGE_THRESH_LOW_MSB, data.b[1], data.b[0]);
	}catch(int e)
	{
		throw e;
	}
}

void LTC2943::setVoltageThresholdHigh(float _maxVoltage)
{
	uint16_t maxVoltageCode;

	maxVoltageCode = _maxVoltage *(65535.0) / (FULLSCALE_VOLTAGE);

	union
	{
		uint8_t b[2];
		uint16_t w;
	} data;

	data.w = maxVoltageCode;

	try
	{
		this->batteryReader->WriteTwoDataByte(VOLTAGE_THRESH_HIGH_MSB, data.b[1], data.b[0]);
	}catch(int e)
	{
		throw e;
	}
}

void LTC2943::setVoltageThresholdLow(float _minVoltage)
{
	uint16_t minVoltageCode;

	minVoltageCode = _minVoltage * 65535.0 / (FULLSCALE_VOLTAGE);

	union
	{
		uint8_t b[2];
		uint16_t w;
	} data;

	data.w = minVoltageCode;

	try
	{
		this->batteryReader->WriteTwoDataByte(VOLTAGE_THRESH_LOW_MSB, data.b[1], data.b[0]);
	}catch(int e)
	{
		throw e;
	}
}

int LTC2943::readStatusRegister()
{
	char statusReg;
	int i_status = 1;
	
	statusReg = c_register_data[STATUS];
	
	return (int)statusReg;
}

float LTC2943::readTemperature()
{
	float f_temperature = 0.0;
	uint16_t temperatureADCCode;

	temperatureADCCode = (uint16_t)c_register_data[TEMPERATURE_MSB] * 256 + (uint16_t)c_register_data[TEMPERATURE_LSB];

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

	currentADCCode = (uint16_t)c_register_data[CURRENT_MSB] * 256 + (uint16_t)c_register_data[CURRENT_LSB];

	f_current = calculateCurrentFromADCCode(currentADCCode);
	
	return f_current*10.0;
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

	voltageADCCode = (uint16_t)c_register_data[VOLTAGE_MSB] * 256 + (uint16_t)c_register_data[VOLTAGE_LSB];

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
	int i_status = 1;

	accumulatedChargeADCCode = (uint16_t)c_register_data[ACC_CHARGE_MSB] * 256 + (uint16_t)c_register_data[ACC_CHARGE_LSB];

	f_accumulatedCharge = calculateAccumulatedChargeFromADCCode(accumulatedChargeADCCode);

	return f_accumulatedCharge;
}

float LTC2943::calculateAccumulatedChargeFromADCCode(uint16_t _accChargeCode)
{
	float calculatedAccValue = 0.0;

	calculatedAccValue = 1000 * (float)(_accChargeCode * CHARGE_LEAST * this->prescalerValue * 0.05) / (this->senseResistorValue * 4096);

	return calculatedAccValue;
}

void LTC2943::setControlRegister(unsigned char _mode, unsigned char _prescalerCode, unsigned char _alccConfiguration)
{
	int8_t configuration = 0x00;

	configuration = _mode | _prescalerCode | _alccConfiguration;

	try
	{
		this->batteryReader->WriteDataByte(CONTROL, configuration);
	}catch(int e)
	{
		throw e;
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

void LTC2943::readRegisters()
{
	try
	{
		this->batteryReader->ReadMultipleRegister(c_register_data, 24);	
	}catch(int e)
	{
		throw e;
	} 	
}
