/*
 * IMI2C.cpp
 *
 *  Created on: Mar 23, 2015
 *      Author: makcakoca
 */


#include "../include/IMI2C.h"

/*****************************************************************
 * This is the default constructor for the class. It assigns
 * all private variables to default values and calls the openI2C()
 * function to open the default I2C device "/dev/i2c-0".
 *****************************************************************/
IMI2C::IMI2C()
{
	this->str_i2c_file_name = "/dev/i2c-0";
	this->u_c_device_adress= 0;
	//this->i_i2cfd = -1;
//	this->OpenI2C();

}

/*******************************************************************
 * This is the overloaded constructor. It allows the programmer to
 * specify a custom I2C device & device address
 * The device descriptor is determined by the openI2C() private member
 * function call.
 * *****************************************************************/

IMI2C::IMI2C(unsigned char u_c_device_address, string str_i2c_file_name, sem_t * mutex)
{
	this->str_i2c_file_name = str_i2c_file_name;
	this->u_c_device_adress = u_c_device_address;
	this->mutex = mutex;
	//this->i_i2cfd = -1;
//	this->OpenI2C();
}
/**********************************************************************
 * This is the class destructor it simply closes the open I2C device
 * by calling the closeI2C() which in turn calls the close() system call
 * *********************************************************************/

IMI2C::~IMI2C(void)
{
	//this->CloseI2C();
}

/**********************************************************************
 * This function opens the I2C device by simply calling the open system
 * call on the I2C device specified in the i2cFileName string. The I2C
 * device is opened for writing and reading. The i2cDescriptor private
 * variable is set by the return value of the open() system call.
 * This variable will be used to reference the opened I2C device by the
 * ioctl() & close() system calls.
 * ********************************************************************/

int IMI2C::OpenI2C()
{
	int i_status_value = -1;

	//sem_wait(this->mutex);

	i_status_value = open(str_i2c_file_name.c_str(), O_RDWR);
	if(i_status_value < 0)
	{
		perror("IMI2C: Could not open file.");
		exit(1);
	}

	return i_status_value;
}

/*********************************************************************
 * This function closes the I2C device by calling the close() system call
 * on the I2C device decriptor.
 * *******************************************************************/

int IMI2C::CloseI2C(int i_i2cfd)
{
	int i_status_value = -1;
	i_status_value = close(i_i2cfd);

	if(i_status_value < 0)
	{
		perror("Could not close file");
		exit(1);
	}

	//sem_post(this->mutex);

	return i_status_value;
}
/********************************************************************
 *This function writes a byte of data "data" to a specific register
 *"reg_addr" in the I2C device This involves sending these two bytes
 *in order to the i2C device by means of the ioctl() command. Since
 *both bytes are written (no read/write switch), both pieces
 *of information can be sent in a single message (i2c_msg structure)
 ********************************************************************/
int IMI2C::WriteDataByte(unsigned char u_c_register_address, unsigned char u_c_data)
{

	unsigned char u_c_buffer[2];
	//unsigned char u_c_buffer[1];
	int i_status_value = -1;
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[1];
	int i_i2cfd = -1;

	i_i2cfd = this->OpenI2C();

	u_c_buffer[0] = u_c_register_address;
	u_c_buffer[1] = u_c_data;
	//u_c_buffer[0] = u_c_data;

	messages[0].addr = this->u_c_device_adress;
	messages[0].flags = 0;
	messages[0].len = sizeof(u_c_buffer);
	messages[0].buf = (char *)u_c_buffer;

	packets.msgs = messages;
	packets.nmsgs = 1;

	i_status_value = ioctl(i_i2cfd, I2C_RDWR, &packets);
	if(i_status_value < 0)
	{
		perror("Write to I2C Device failed");
		exit(1);
	}

	this->CloseI2C(i_i2cfd);

	return i_status_value;
}

/********************************************************************
 *This function reads a byte of data "data" from a specific register
 *"reg_addr" in the I2C device. This involves sending the register
 *byte "reg_Addr" with "write" asserted and then instructing the
 *I2C device to read a byte of data from that address ("read asserted").
 *This necessitates the use of two i2c_msg structs. One for the register
 *address write and another for the read from the I2C device i.e.
 *I2C_M_RD flag is set. The read data is then saved into the reference
 *variable "data".
 ********************************************************************/

int IMI2C::ReadDataByte(unsigned char u_c_register_address, unsigned char & u_c_data)
{

    unsigned char *p_u_c_inbuff, u_c_outbuff;
	int i_status_value = -1;
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[2];
	//struct i2c_msg messages[1];
	int i_i2cfd = -1;

	i_i2cfd = this->OpenI2C();

	u_c_outbuff = u_c_register_address;
	messages[0].addr = this->u_c_device_adress;
	messages[0].flags= 0;
	messages[0].len = sizeof(u_c_outbuff);
	messages[0].buf = (char*)&u_c_outbuff;

	p_u_c_inbuff = &u_c_data;
	messages[1].addr = this->u_c_device_adress;
	messages[1].flags = I2C_M_RD;
	messages[1].len = sizeof(*p_u_c_inbuff); // size of value pointed to by inbuff to size of pointer inbuff
	messages[1].buf = (char*)p_u_c_inbuff;

/*	p_u_c_inbuff = &u_c_data;
	messages[0].addr = this->u_c_device_adress;
	messages[0].flags = I2C_M_RD;
	messages[0].len = sizeof(*p_u_c_inbuff); // size of value pointed to by inbuff to size of pointer inbuff
	messages[0].buf = p_u_c_inbuff;
*/

	packets.msgs = messages;
//	packets.nmsgs = 1;
	packets.nmsgs = 2;

	i_status_value = ioctl(i_i2cfd, I2C_RDWR, &packets);
	if(i_status_value < 0)
	{
		perror("Read from I2C Device failed");
		exit(1);
	}

	this->CloseI2C(i_i2cfd);

	return i_status_value;
}

