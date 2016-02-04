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
}
/**********************************************************************
 * This is the class destructor it simply closes the open I2C device
 * by calling the closeI2C() which in turn calls the close() system call
 * *********************************************************************/

IMI2C::~IMI2C(void){}

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

	sem_wait(this->mutex);

	i_status_value = open(str_i2c_file_name.c_str(), O_RDWR);
	if(i_status_value < 0)
	{
		throw -22;
	}
	
	if(ioctl(i_status_value, I2C_SLAVE, this->u_c_device_adress) < 0)
	{
		throw -130;
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
		throw -23;
	}

	sem_post(this->mutex);

	return i_status_value;
}
/********************************************************************
 *This function writes a byte of data "data" to a specific register
 *"reg_addr" in the I2C device This involves sending these two bytes
 *in order to the i2C device by means of the ioctl() command. Since
 *both bytes are written (no read/write switch), both pieces
 *of information can be sent in a single message (i2c_msg structure)
 ********************************************************************/
int IMI2C::WriteDataByte(char c_register_address, char c_data)
{
	char c_buffer[2];
	int i_status_value = -1;
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[1];
	int i_i2cfd = -1;

	try{
		i_i2cfd = this->OpenI2C();
	}catch(int i){
		throw i;
	}

	c_buffer[0] = c_register_address;
	c_buffer[1] = c_data;

	messages[0].addr = this->u_c_device_adress;
	messages[0].flags = 0;
	messages[0].len = sizeof(c_buffer);
	messages[0].buf = c_buffer;

	packets.msgs = messages;
	packets.nmsgs = 1;

	i_status_value = ioctl(i_i2cfd, I2C_RDWR, &packets);
	if(i_status_value < 0)
	{
		try
		{
			this->CloseI2C(i_i2cfd);
		}catch(int i){
						throw i;
		}

		throw -24;
	}

	try{
		this->CloseI2C(i_i2cfd);
	}catch(int i){
		throw i;
	}

	return i_status_value;
}

/********************************************************************
 *This function writes only one byte.
 ********************************************************************/
int IMI2C::WriteByte(unsigned char u_c_byte)
{
	int i_i2cfd = -1;

	try{
		i_i2cfd = this->OpenI2C();
	}catch(int i){
		throw i;
	}

	if (ioctl(i_i2cfd, I2C_SLAVE, u_c_byte) < 0)
	{
	    throw -25;
	}

	int returnval = i2c_smbus_read_byte(i_i2cfd);
	if(returnval < 0)
	{
		throw -25;
	}

	try{
		this->CloseI2C(i_i2cfd);
	}catch(int i){
		throw i;
	}

	return returnval;
}

/********************************************************************
 *This function writes two bytes of data "data" to a specific register
 *"reg_addr" in the I2C device This involves sending these two bytes
 *in order to the i2C device by means of the ioctl() command. Since
 *both bytes are written (no read/write switch), both pieces
 *of information can be sent in a single message (i2c_msg structure)
 * register -> low byte -> high byte
 ********************************************************************/
int IMI2C::WriteTwoDataByte(char c_register_address, char c_msb_data, char c_lsb_data)
{

	char c_buffer[2];
	int i_status_value = -1;
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[2];
	int i_i2cfd = -1;

	try{
		i_i2cfd = this->OpenI2C();
	}catch(int e){
		throw e;
	}

	c_buffer[0] = c_register_address;
	c_buffer[1] = c_lsb_data;

	messages[0].addr = this->u_c_device_adress;
	messages[0].flags = 0;
	messages[0].len = sizeof(c_buffer);
	messages[0].buf = c_buffer;

	c_buffer[0] = c_register_address;
	c_buffer[1] = c_msb_data;

	messages[1].addr = this->u_c_device_adress;
	messages[1].flags = 0;
	messages[1].len = sizeof(c_buffer);
	messages[1].buf = c_buffer;

	packets.msgs = messages;
	packets.nmsgs = 2;

	i_status_value = ioctl(i_i2cfd, I2C_RDWR, &packets);
	if(i_status_value < 0)
	{
		throw -24;
	}

	try{
		this->CloseI2C(i_i2cfd);
	}catch(int i){
		throw i;
	}

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

int IMI2C::ReadDataByte(char c_register_address, char & c_data)
{

  char *p_c_inbuff, c_outbuff;
	int i_status_value = -1;
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[2];
	int i_i2cfd = -1;

	try{
		i_i2cfd = this->OpenI2C();
	}catch(int i){
		throw i;
	}

	c_outbuff = c_register_address;
	messages[0].addr = this->u_c_device_adress;
	messages[0].flags= 0;
	messages[0].len = sizeof(c_outbuff);
	messages[0].buf = &c_outbuff;

	p_c_inbuff = &c_data;
	messages[1].addr = this->u_c_device_adress;
	messages[1].flags = I2C_M_RD;
	messages[1].len = sizeof(*p_c_inbuff); // size of value pointed to by inbuff to size of pointer inbuff
	messages[1].buf = p_c_inbuff;

	packets.msgs = messages;
	packets.nmsgs = 2;

	i_status_value = ioctl(i_i2cfd, I2C_RDWR, &packets);
	if(i_status_value < 0)
	{
		try{
						this->CloseI2C(i_i2cfd);
		}catch(int i){
						throw i;
		}

		throw -25;
	}

	try{
		this->CloseI2C(i_i2cfd);
	}catch(int i){
		throw i;
	}

	return i_status_value;
}

 /*
  * TODO: Check this function.
  */
int IMI2C::ReadTwoDataByte(char c_register_address, char & c_msb_data, char & c_lsb_data)
{
   char c_outbuff;
   char *p_c_inbuff;

	int i_status_value = -1;
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[3];
	int i_i2cfd = -1;

	try{
		i_i2cfd = this->OpenI2C();
	}catch(int i){
		throw i;
	}

	c_outbuff = c_register_address;
	messages[0].addr = this->u_c_device_adress;
	messages[0].flags= 0;
	messages[0].len = sizeof(c_outbuff);
	messages[0].buf = &c_outbuff;

	p_c_inbuff = &c_msb_data;
	messages[1].addr = this->u_c_device_adress;
	messages[1].flags = I2C_M_RD;
	messages[1].len = sizeof(*p_c_inbuff); // size of value pointed to by inbuff to size of pointer inbuff
	messages[1].buf = p_c_inbuff;

	p_c_inbuff = &c_lsb_data;
	messages[2].addr = this->u_c_device_adress;
	messages[2].flags = I2C_M_RD;
	messages[2].len = sizeof(*p_c_inbuff);
	messages[2].buf = p_c_inbuff;

	packets.msgs = messages;
	packets.nmsgs = 3;

	i_status_value = ioctl(i_i2cfd, I2C_RDWR, &packets);
	if(i_status_value < 0)
	{
		throw -25;
	}

	try{
		this->CloseI2C(i_i2cfd);
	}catch(int i){
		throw i;
	}

	return i_status_value;

}

void IMI2C::ReadMultipleRegister(char * c_data, int i_reg_size)
{
	int i_i2cfd = -1;
	
	try{
		i_i2cfd = this->OpenI2C();
	}catch(int i){
		throw i;
	}
	
	if(read(i_i2cfd, c_data, i_reg_size) != i_reg_size)
	{
		try{
			this->CloseI2C(i_i2cfd);
		}catch(int i){
			throw i;
		}		
		throw -128;
	}

	try{
		this->CloseI2C(i_i2cfd);
	}catch(int i){
		throw i;
	}
}
