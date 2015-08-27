#ifdef English_dox
/**
 * \file   IMI2C.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief
 * \details
 *
 */
#endif
#ifdef Turkish_dox
/**
 * \file   IMI2C.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief
 * \details
 *
 */
#endif

#ifndef INCLUDE_IMI2C_H_
#define INCLUDE_IMI2C_H_

#include <string>
#include <stdio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>

//#include <sys/types.h>
#include <sys/ipc.h>
#include <semaphore.h>
#include <sys/stat.h>


using namespace std;

class IMI2C
{
public:

	#ifdef English_dox
	//! Constructor
	/**
	 *
	 */
	#endif

	#ifdef Turkish_dox
	//! Constructor
	/**
	 *
	 */
	#endif
	IMI2C();

	#ifdef English_dox
	//! Constructor
	/**
	 * \param u_c_device_address i2c device address
	 * \param str_i2c_file_name i2c device driver name
	 * \param * mutex
	 */
	#endif

	#ifdef Turkish_dox
	//! Constructor
	/**
	 * \param u_c_device_address i2c aygıt adresi
	 * \param str_i2c_file_name i2c aygıt sürücüsü adı
	 * \param * mutex i2c sürücüsünün kullanımını senkronlaştırmak için mutex.
	 */
	#endif
	IMI2C(unsigned char u_c_device_address, string str_i2c_file_name, sem_t * mutex);


	~IMI2C(void);

	#ifdef English_dox
	//! Function to write byte data into a register of an I2C device
	/**
	 * \param u_c_register_address
	 * \param u_c_data
	 * \retvat <0 if there ise an error
	 * \retval >=0 no error
	 */
	#endif
	#ifdef Turkish_dox
	//! I2C aygıtının ilgili registerına veri yazan fonksiyondur.
	/**
	 * \param u_c_register_address
	 * \param u_c_data
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int WriteDataByte(unsigned char u_c_register_address, unsigned char u_c_data);


	#ifdef English_dox
	//! Function to read byte data from a register of an I2C device
	/**
	 * Detailed
	 * \param u_c_register_address
	 * \param & u_c_data Okunan değerdir.
	 * \retvat <0 if there ise an error
	 * \retval >=0 no error
	 */
	#endif
	#ifdef Turkish_dox
	//! I2C aygıtının ilgili registerından veri okuyan fonksiyondur.
	/**
	 * Detaylı bilgi
	 * \param u_c_register_address
	 * \param & u_c_data Okunan değerdir.
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int ReadDataByte(unsigned char u_c_register_address, unsigned char & u_c_data);

private:

	#ifdef English_dox
	//! Open an I2C device.
	/**
	 * \retvat <0 if there ise an error
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
	 * \param i_i2cfd
	 * \retvat <0 if there ise an error
	 * \retval >=0 no error
	 */
	#endif
	#ifdef Turkish_dox
	//! I2C aygıtını kapatır.
	/**
	 * \param i_i2cfd
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int CloseI2C(int i_i2cfd);

	//! i2c device name e.g."/dev/i2c-0" or "/dev/i2c-1"
	string  str_i2c_file_name;
	//int i_i2cfd;  // i2c device descriptor

	unsigned char u_c_device_adress;

	sem_t * mutex;
};



#endif /* INCLUDE_IMI2C_H_ */
