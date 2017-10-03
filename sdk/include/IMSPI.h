#ifdef English_dox
/**
 * \file   IMSPI.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief
 * \details
 *
 */
#endif
#ifdef Turkish_dox
/**
 * \file   IMSPI.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief
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


class IMSPI
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
	IMSPI();

	#ifdef English_dox
	//! Constructor
	/**
	 * \param str_devspi SPI driver name
	 * \param u_c_spi_mode SPI_MODE_0 - SPI_MODE_3
	 * \param u_i_spi_speed
	 * \param u_c_spi_bits_per_word
	 */
	#endif

	#ifdef Turkish_dox
	//! Constructor
	/**
	 * \param str_devspi SPI sürücü adı
	 * \param u_c_spi_mode SPI_MODE_0 - SPI_MODE_3
	 * \param u_i_spi_speed
	 * \param u_c_spi_bits_per_word
	 */
	#endif
	IMSPI(std::string str_devspi, unsigned char u_c_spi_mode, unsigned int u_i_spi_speed,
			unsigned char u_c_spi_bits_per_word);

    ~IMSPI();

	#ifdef English_dox
	//! Writes data and reads the data returned back spi peripheral.
	/**
	 * This function writes data "data" of length "length" to the spidev
	 * device. Data shifted in from the spidev device is saved back into
	 * "data".
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! 'p_u_c_data' verisini spi'ya yazar ve dönen cevabı yine aynı değişkene kaydeder.
	/**
	 * \param p_u_c_data
	 * \param i_length
	 * \return
	 */
	#endif
    int SpiWriteRead(unsigned char *p_u_c_data, int i_length);

private:
	unsigned char u_c_mode;
	unsigned char u_c_bits_per_word;
	unsigned int u_i_speed;
	int i_spifd;

	#ifdef English_dox
	//! It is responsible for opening the spidev device "devspi" and then setting up the spidev interface.
	/**
	 * \param devspi SPI Device name
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! SPI sürücü açma ve ilklendirme ile görevlidir.
	/**
	 * \param devspi SPI aygıtı sürücü adı
	 * \return
	 */
	#endif
	int SpiOpen(std::string devspi);

	#ifdef English_dox
	//! Responsible for closing the spidev interface. Called in destructor
	/**
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! spidev arayüzünü kapatmak ile görevlidir. Destructor içerisinde çağrılır.
	/**
	 * \return
	 */
	#endif
	int SpiClose();

};


#endif /* INCLUDE_IMSPI_H_ */
