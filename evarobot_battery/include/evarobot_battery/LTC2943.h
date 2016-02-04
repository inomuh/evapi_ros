/*
 * LTC2943.h
 *
 *  Created on: Oct 20, 2015
 *      Author: veli
 */

#ifndef LIB_LTC2943_H_
#define LIB_LTC2943_H_

#include "IMI2C.h"

/* LTC2943 donanım adres tanımlamaları */
#define LTC2943_HARDWARE_ADDRESS 		0x64
#define LTC2943_ALERT_RESPONSE_ADDRESS 	0x0C

/* LTC2943 yazmaç haritası tanımlamaları */
#define STATUS 							0x00
#define CONTROL 						0x01
#define ACC_CHARGE_MSB 					0x02
#define ACC_CHARGE_LSB					0x03
#define CHARGE_THRESH_HIGH_MSB 			0x04
#define CHARGE_THRESH_HIGH_LSB			0x05
#define CHARGE_THRESH_LOW_MSB			0x06
#define CHARGE_THRESH_LOW_LSB			0x07
#define VOLTAGE_MSB						0x08
#define VOLTAGE_LSB						0x09
#define VOLTAGE_THRESH_HIGH_MSB			0x0A
#define VOLTAGE_THRESH_HIGH_LSB			0x0B
#define VOLTAGE_THRESH_LOW_MSB			0x0C
#define VOLTAGE_THRESH_LOW_LSB			0x0D
#define CURRENT_MSB						0x0E
#define CURRENT_LSB						0x0F
#define CURRENT_THRESH_HIGH_MSB			0x10
#define CURRENT_THRESH_HIGH_LSB			0x11
#define CURRENT_THRESH_LOW_MSB 			0x12
#define CURRENT_THRESH_LOW_LSB			0x13
#define TEMPERATURE_MSB					0x14
#define TEMPERATURE_LSB					0x15
#define TEMPERATURE_THRESH_HIGH			0x16
#define TEMPERATURE_THRESH_LOW			0x17

/* Kontrol yazmacı komut kodlaro tanımlamaları
 *
 * B[7:6] - ADC mode
 * B[5:3] - Prescaler
 * B[2:1] - ALCC Configure
 * B[0]	  - Shut down
 * */

#define AUTOMATIC_MODE 					0xC0
#define SCAN_MODE						0x80
#define MANUAL_MODE						0x40
#define SLEEP_MODE						0x00

#define PRESCALER_1						0x00
#define PRESCALER_4						0x08
#define PRESCALER_16					0x10
#define PRESCALER_64					0x18
#define PRESCALER_256					0x20
#define PRESCALER_1024					0x28
#define PRESCALER_4096					0x30
#define PRESCALER_4096_2				0x31

#define ALERT_MODE						0x04
#define CHARGE_COMPLETE_MODE			0x02
#define DISABLE_ALCC_PIN				0x00

#define SHUTDOWN_MODE					0x01

/* Dönüşüm sabitleri */
const float CHARGE_LEAST = 0.00034; // 0.34 mAh
const float VOLTAGE_LEAST = 0.00144; // 1.44 mV
const float CURRENT_LEAST = 0.0000293; // 29.3 uV
const float TEMPERATURE_LEAST = 0.25; // 0.25 C
const float FULLSCALE_VOLTAGE = 23.6; // 23.6 V
const float FULLSCALE_CURRENT = 0.06; // 60 mV
const float FULLSCALE_TEMPERATURE = 510; // 510 K


class LTC2943
{

public:
	float senseResistorValue;
	uint16_t prescalerValue;
	uint16_t prescalerMode;
	uint16_t alccMode;


	float readAccumulatedCharge();
	float readVoltage();
	float readCurrent();
	float readTemperature();

	int readStatusRegister();

	void setChargeThresholdHigh(uint16_t _maxCharge);
	void setChargeThresholdLow(uint16_t _minCharge);
	void setVoltageThresholdHigh(float _maxVoltage);
	void setVoltageThresholdLow(float _minVoltage);


	void setCurrentThresholdHigh(uint16_t _maxCurrent);
	void setCurrentThresholdLow(uint16_t _minCurrent);
	void setTemperatureThresholdHigh(uint16_t _maxTemp);
	void setTemperatureThresholdLow(uint16_t _minTemp);

	int getVoltageThresholdLow();

	void resetAlertStatus();

	void setControlRegister(unsigned char _mode, unsigned char _prescaler, unsigned char _alccConfiguration);

	void readRegisters();
	LTC2943(unsigned char u_c_device_address,
			 string str_i2c_file_name,
			 sem_t * mutex);

	virtual ~LTC2943();

private:
	IMI2C * batteryReader;
	char c_register_data[24];

	float calculateAccumulatedChargeFromADCCode(uint16_t _accumulatedChargeADCCode);
	float calculateVoltageFromADCCode(uint16_t _voltageCode);
	float calculateCurrentFromADCCode(uint16_t _currentCode);
	float calculateTemperatureFromADCCode(uint16_t _temperatureCode);
};

#endif /* LIB_LTC2943_H_ */
