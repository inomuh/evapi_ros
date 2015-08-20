/*
 * IMEIO.cpp
 *
 *  Created on: Jun 1, 2015
 *      Author: makcakoca
 */

#include "../include/IMEIO.h"

const bool IMEIO::OUTPUT = false;
const bool IMEIO::INPUT = true;

const bool IMEIO::HIGH = true;
const bool IMEIO::LOW = false;

// PORTA

const int IMEIO::RGB_LED1 = 0;
const int IMEIO::RGB_LED2 = 1;
const int IMEIO::RGB_LED3 = 2;
const int IMEIO::BUZZER = 3;

const int IMEIO::BUMPER0 = 4;
const int IMEIO::BUMPER1 = 5;
const int IMEIO::BUMPER2 = 6;

const int IMEIO::USER_LED = 7;

// PORTB
const int IMEIO::SWR_RESET = 0;
const int IMEIO::EIO0 = 1;
const int IMEIO::EIO1 = 2;
const int IMEIO::EIO2 = 3;
const int IMEIO::EIO3 = 4;
const int IMEIO::EIO4 = 5;
const int IMEIO::EIO5 = 6;
const int IMEIO::EIO6 = 7;


IMEIO::IMEIO(unsigned char u_c_device_address, string str_i2c_file_name, sem_t * mutex)
{
	this->u_c_pins_directions_A = 0x00;
	this->u_c_pins_data_A = 0x00;

	this->u_c_pins_directions_B = 0x00;
	this->u_c_pins_data_B = 0x00;

	this->i2c = new IMI2C(u_c_device_address, str_i2c_file_name, mutex);
}

IMEIO::~IMEIO()
{
	delete this->i2c;
}

int IMEIO::SetPinADirection(int i_pin_number, bool b_direction)
{
	int i_status = -1;

	unsigned char u_c_mask = ~(0x01 << i_pin_number);

	this->u_c_pins_directions_A &= u_c_mask;
	this->u_c_pins_directions_A |= (b_direction << i_pin_number);

	i_status = this->i2c->WriteDataByte(IMEIO::IODIRA, this->u_c_pins_directions_A);

	return i_status;
}

int IMEIO::SetPinBDirection(int i_pin_number, bool b_direction)
{
	int i_status = -1;

	unsigned char u_c_mask =  ~(0x01 << i_pin_number);

	this->u_c_pins_directions_B &= u_c_mask;
	this->u_c_pins_directions_B |= (b_direction << i_pin_number);

	i_status = this->i2c->WriteDataByte(IMEIO::IODIRB, this->u_c_pins_directions_B);

	return i_status;
}

int IMEIO::SetPinAValue(int i_pin_number, bool b_value)
{
	int i_status = -1;

	printf("pin_number(A): %d \n", i_pin_number);

	unsigned char u_c_mask =  ~(0x01 << i_pin_number);

	this->u_c_pins_data_A &= u_c_mask;

	this->u_c_pins_data_A |= (b_value << i_pin_number);

	printf("u_c_pins_data(A): %x \n", this->u_c_pins_data_A);

	i_status = this->i2c->WriteDataByte(IMEIO::GPIOA, this->u_c_pins_data_A);

	return i_status;

}

int IMEIO::SetPinBValue(int i_pin_number, bool b_value)
{
	int i_status = -1;

	printf("pin_number: %d \n", i_pin_number);

	unsigned char u_c_mask =  ~(0x01 << i_pin_number);

	this->u_c_pins_data_B &= u_c_mask;
	this->u_c_pins_data_B |= (b_value << i_pin_number);

	printf("u_c_pins_data(B): %x \n", u_c_pins_data_B);

	i_status = this->i2c->WriteDataByte(IMEIO::GPIOB, this->u_c_pins_data_B);

	return i_status;
}

int IMEIO::GetPinAValue(int i_pin_number, bool & b_value)
{
	int i_status = -1;

	i_status = this->i2c->ReadDataByte(IMEIO::GPIOA, this->u_c_pins_data_A);

	b_value = (this->u_c_pins_data_A >> i_pin_number) & 0x01;

	return i_status;

}

int IMEIO::GetPinBValue(int i_pin_number, bool & b_value)
{
	int i_status = -1;

	i_status = this->i2c->ReadDataByte(IMEIO::GPIOB, this->u_c_pins_data_B);

	b_value = (this->u_c_pins_data_B >> i_pin_number) & 0x01;

	return i_status;

}





