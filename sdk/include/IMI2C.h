#ifdef English_dox
/**
 * \file   IMI2C.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Controls I2C of Raspberry Pi.
 * \details
 *
 */
#endif
#ifdef Turkish_dox
/**
 * \file   IMI2C.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Raspberry Pi üzerindeki I2C'yi kontrol eder.
 * \details
 *
 */
#endif

#ifndef INCLUDE_IMI2C_H_
#define INCLUDE_IMI2C_H_

#include <string>
#include <stdio.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>

#include <sys/ipc.h>
#include <semaphore.h>
#include <sys/stat.h>
#include <inttypes.h>
#include <stdint.h>

using namespace std;

//! Controls I2C of Raspberry Pi.
class IMI2C
{
public:

	#ifdef English_dox
	//! Default Constructor
	#endif

	#ifdef Turkish_dox
	//! Default Constructor
	#endif
	IMI2C();

	#ifdef English_dox
	//! Constructor
	/**
	 * \param u_c_device_address i2c device address
	 * \param str_i2c_file_name i2c device driver name
	 * \param *mutex mutex to synchronize i2c device driver
	 */
	#endif

	#ifdef Turkish_dox
	//! Constructor
	/**
	 * \param u_c_device_address i2c aygıt adresi
	 * \param str_i2c_file_name i2c aygıt sürücüsü adı
	 * \param *mutex i2c sürücüsünün kullanımını senkronlaştırmak için mutex
	 */
	#endif
	IMI2C(unsigned char u_c_device_address, string str_i2c_file_name, sem_t * mutex);


	~IMI2C(void);

	#ifdef English_dox
	//! Function to write byte data into a register of an I2C device
	/**
	 * \param c_register_address Register address in the I2C device to write data
	 * \param c_data Data to write specified register
	 * \retval <0 if there is an error
	 * \retval >=0 no error
	 */
	#endif
	int WriteDataByte(char c_register_address, char c_data);

	#ifdef English_dox
	//! Function to write two bytes of data into a register of an I2C device
	/**
	 * \param c_register_address Register address in the I2C device to write data
	 * \param c_msb_data Data to write low byte of specified register
	 * \param c_lsb_data Data to write high byte of specified register
	 * \retval <0 if there is an error
	 * \retval >=0 no error
	 */
	#endif
	int WriteTwoDataByte(char c_register_address, char c_msb_data, char c_lsb_data);

	#ifdef English_dox
	//! Function to write only one byte data.
	/**
	 * This function writes only one byte.
	 * \param u_c_byte Data to write
	 * \retval <0 if there is an error
	 * \retval >=0 no error
	 */
	#endif
	int WriteByte(unsigned char u_c_register_address);
	
	#ifdef English_dox
	//! Function to read byte data from a register of an I2C device
	/**
	 * \param c_register_address Register address in the I2C device to read data
	 * \param &c_data Data of specified register.
	 * \retval <0 if there ise an error
	 * \retval >=0 no error
	 */
	#endif
	int ReadDataByte(char c_register_address, char & c_data);

	#ifdef English_dox
	//! Function to read multiple bytes of data from I2C device
	/**
	 * \param *c_data Data of registers.
	 * \param i_reg_size Number of registers to read.
	 */
	#endif
	void ReadMultipleRegister(char * c_data, int i_reg_size);

	int ReadTwoDataByte(char c_register_address, char & c_msb_data, char & c_lsb_data);
	int WriteDataByte(char c_register_address, char c_data);

private:

	#ifdef English_dox
	//! Open an I2C device.
	/**
	 * \retval <0 if there ise an error
	 * \retval >=0 no error
	 */
	#endif
	#ifdef Turkish_dox
	//! I2C aygıtını açar.
	/**
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int OpenI2C();

	#ifdef English_dox
	//! Close an I2C device.
	/**
	 * \param i_i2cfd I2C file descriptor to close.
	 * \retval <0 if there ise an error
	 * \retval >=0 no error
	 */
	#endif
	#ifdef Turkish_dox
	//! I2C aygıtını kapatır.
	/**
	 * \param i_i2cfd Kapatılacak I2C'nin dosya tanımlayıcısıdır.
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int CloseI2C(int i_i2cfd);

	#ifdef English_dox
		//! Name of i2c device. e.g."/dev/i2c-0" or "/dev/i2c-1"
	#endif
	string  str_i2c_file_name;

	#ifdef English_dox
		//! Address of i2c device.
	#endif
	unsigned char u_c_device_adress;

	#ifdef English_dox
		//! mutex to synchronize i2c device driver
	#endif
	sem_t * mutex;
};



#endif /* INCLUDE_IMI2C_H_ */
