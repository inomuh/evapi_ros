#ifdef English_dox
/**
 * \file   IMSPI.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Controls SPI communication.
 * \details
 *
 */
#endif
#ifdef Turkish_dox
/**
 * \file   IMSPI.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  SPI üzerinden haberleşmeyi kontrol eder.
 * \details
 *
 */
#endif

#ifndef INCLUDE_IMSPI_H_
#define INCLUDE_IMSPI_H_

#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <iostream>

//! Controls SPI communication.
class IMSPI
{

public:

	#ifdef English_dox
	//! Default Constructor
	#endif
	#ifdef Turkish_dox
	//! Default Constructor
	#endif
	IMSPI();

	#ifdef English_dox
	//! Constructor
	/**
	 * \param str_devspi SPI driver name
	 * \param u_c_spi_mode SPI_MODE_0 - SPI_MODE_3
	 * \param u_i_spi_speed Communication speed
	 * \param u_c_spi_bits_per_word Number of bits per data
	 */
	#endif
	#ifdef Turkish_dox
	//! Constructor
	/**
	 * \param str_devspi SPI sürücü adı
	 * \param u_c_spi_mode SPI_MODE_0 - SPI_MODE_3
	 * \param u_i_spi_speed Haberleşme hızı
	 * \param u_c_spi_bits_per_word Her verideki bit sayısı
	 */
	#endif
	IMSPI(std::string str_devspi, unsigned char u_c_spi_mode, unsigned int u_i_spi_speed,
			unsigned char u_c_spi_bits_per_word);

    ~IMSPI();

	#ifdef English_dox
	//! Writes data and reads the data returned back spi peripheral.
	/**
	 * This function writes data "data" of length "length" to the spidev
	 * device. Data shifted in from the spidev device is saved back into "data".
	 * \param *p_u_c_data This data is written to spi and response is written to this variable.
	 * \param i_length Size of data.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	#ifdef Turkish_dox
	//! 'p_u_c_data' verisini spi'ya yazar ve dönen cevabı yine aynı değişkene kaydeder.
	/**
	 * \param *p_u_c_data Bu veri spi'ya yazılır ve cevap bu değişkene yazılır.
	 * \param i_length Verinin boyutudur.
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
    int SpiWriteRead(unsigned char *p_u_c_data, int i_length);

private:
	#ifdef Turkish_dox
	//! SPI_MODE_0 - SPI_MODE_3
	#endif
	unsigned char u_c_mode;
	#ifdef Turkish_dox
	//! Number of bits per data
	#endif
	unsigned char u_c_bits_per_word;
	#ifdef Turkish_dox
	//! Communication speed
	#endif
	unsigned int u_i_speed;
	#ifdef Turkish_dox
	//! File descriptor for spi.
	#endif
	int i_spifd;

	#ifdef English_dox
	//! It is responsible for opening the spidev device "devspi" and then setting up the spidev interface.
	/**
	 * \param devspi SPI Device name
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	#ifdef Turkish_dox
	//! SPI sürücü açma ve ilklendirme ile görevlidir.
	/**
	 * \param devspi SPI aygıtı sürücü adı
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int SpiOpen(std::string devspi);

	#ifdef English_dox
	//! Responsible for closing the spidev interface. Called in destructor
	/**
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	#ifdef Turkish_dox
	//! spidev arayüzünü kapatmak ile görevlidir. Destructor içerisinde çağrılır.
	/**
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int SpiClose();

};


#endif /* INCLUDE_IMSPI_H_ */
