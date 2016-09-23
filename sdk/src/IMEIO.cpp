#include "../include/IMEIO.h"

/**
 * IMEIO constructor. Initializes required variables.
 */
IMEIO::IMEIO(unsigned char u_c_device_address, string str_i2c_file_name, sem_t * mutex)
{
	this->c_pins_directions_A = 0x00;
	this->c_pins_data_A = 0x00;

	this->c_pins_directions_B = 0x00;
	this->c_pins_data_B = 0x00;

	try{
		this->i2c = new IMI2C(u_c_device_address, str_i2c_file_name, mutex);
	}catch(int i){
		throw i;
	}
}

IMEIO::~IMEIO()
{
	delete this->i2c;
}

/**
 * Sets pin direction as input or output for PORTA
 */
int IMEIO::SetPinADirection(int i_pin_number, bool b_direction)
{
	int i_status = -1;

	/**
	 * Pin number shifted, masked and prepared for operation.
	 */
	char c_mask = ~(0x01 << i_pin_number);
	this->c_pins_directions_A &= c_mask;
	this->c_pins_directions_A |= (b_direction << i_pin_number);

	/**
	 * Writes pin direction to IMEIO::IODIRA register using I2C.
	 */
	try{
		i_status = this->i2c->WriteDataByte(IMEIO::IODIRA, this->c_pins_directions_A);
	}catch(int i){
		throw i;
	}
	
	return i_status;
}

/**
 * Sets pin direction as input or output for PORTB
 */
int IMEIO::SetPinBDirection(int i_pin_number, bool b_direction)
{
	int i_status = -1;

	/**
	 * Pin number shifted, masked and prepared for operation.
	 */
	char c_mask =  ~(0x01 << i_pin_number);
	this->c_pins_directions_B &= c_mask;
	this->c_pins_directions_B |= (b_direction << i_pin_number);

	/**
	 * Writes pin direction to IMEIO::IODIRB register using I2C.
	 */
	try{
		i_status = this->i2c->WriteDataByte(IMEIO::IODIRB, this->c_pins_directions_B);
	}catch(int i){
		throw i;
	}

	return i_status;
}

/**
 * Sets pin value as input or output for PORTA
 */
int IMEIO::SetPinAValue(int i_pin_number, bool b_value)
{
	int i_status = -1;

	/**
	 * Pin number shifted, masked and prepared for operation.
	 */
	char c_mask =  ~(0x01 << i_pin_number);
	this->c_pins_data_A &= c_mask;
	this->c_pins_data_A |= (b_value << i_pin_number);

	/**
	 * Writes pin data to IMEIO::GPIOA register using I2C.
	 */
	try{
		i_status = this->i2c->WriteDataByte(IMEIO::GPIOA, this->c_pins_data_A);
	}catch(int i){
		throw i;
	}

	return i_status;
}

/**
 * Sets pin value as input or output for PORTB
 */
int IMEIO::SetPinBValue(int i_pin_number, bool b_value)
{
	int i_status = -1;

	/**
	 * Pin number shifted, masked and prepared for operation.
	 */
	char c_mask =  ~(0x01 << i_pin_number);
	this->c_pins_data_B &= c_mask;
	this->c_pins_data_B |= (b_value << i_pin_number);

	/**
	 * Writes pin data to IMEIO::GPIOB register using I2C.
	 */
	try{
		i_status = this->i2c->WriteDataByte(IMEIO::GPIOB, this->c_pins_data_B);
	}catch(int i){
		throw i;
	}

	return i_status;
}

/**
 * Gets pin value as input or output for PORTA
 */
int IMEIO::GetPinAValue(int i_pin_number, bool & b_value)
{
	int i_status = -1;

	/**
	 * Reads IMEIO::GPIOA register data using I2C.
	 */
	try{
		i_status = this->i2c->ReadDataByte(IMEIO::GPIOA, this->c_pins_data_A);
	}catch(int i){
		throw i;
	}

	b_value = (this->c_pins_data_A >> i_pin_number) & 0x01;

	return i_status;

}

/**
 * Gets pin value as input or output for PORTB
 */
int IMEIO::GetPinBValue(int i_pin_number, bool & b_value)
{
	int i_status = -1;

	/**
	 * Reads IMEIO::GPIOB register data using I2C.
	 */
	try{
		i_status = this->i2c->ReadDataByte(IMEIO::GPIOB, this->c_pins_data_B);
	}catch(int i){
		throw i;
	}

	b_value = (this->c_pins_data_B >> i_pin_number) & 0x01;

	return i_status;

}
