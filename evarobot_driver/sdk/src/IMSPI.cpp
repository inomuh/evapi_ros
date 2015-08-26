/*
 * IMSPI.cpp
 *
 *  Created on: Mar 23, 2015
 *      Author: makcakoca
 */

#include "../include/IMSPI.h"

using namespace std;
/**********************************************************
 * spiOpen() :function is called by the constructor.
 * It is responsible for opening the spidev device
 * "devspi" and then setting up the spidev interface.
 * private member variables are used to configure spidev.
 * They must be set appropriately by constructor before calling
 * this function.
 * *********************************************************/
int IMSPI::SpiOpen(std::string devspi)
{
    int i_status_value = -1;
    this->i_spifd = open(devspi.c_str(), O_RDWR);
    if(this->i_spifd < 0)
    {
        perror("IMSPI: SPI aygiti acilamadi.");
        exit(1);
    }

    i_status_value = ioctl (this->i_spifd, SPI_IOC_WR_MODE, &(this->u_c_mode));
    if(i_status_value < 0)
    {
        perror("IMSPI: SPIMode (WR) Atanamadi...ioctl hatasi");
        exit(1);
    }

    i_status_value = ioctl (this->i_spifd, SPI_IOC_RD_MODE, &(this->u_c_mode));
    if(i_status_value < 0)
    {
      perror("IMSPI: SPIMode (RD) Atanamadi...ioctl hatasi");
      exit(1);
    }

    i_status_value = ioctl (this->i_spifd, SPI_IOC_WR_BITS_PER_WORD, &(this->u_c_bits_per_word));
    if(i_status_value < 0)
    {
      perror("IMSPI: SPI bitsPerWord (WR) atanamadi...ioctl hatasi");
      exit(1);
    }

    i_status_value = ioctl (this->i_spifd, SPI_IOC_RD_BITS_PER_WORD, &(this->u_c_bits_per_word));
    if(i_status_value < 0)
    {
      perror("IMSPI: SPI bitsPerWord(RD) atanamadi...ioctl hatasi");
      exit(1);
    }

    i_status_value = ioctl (this->i_spifd, SPI_IOC_WR_MAX_SPEED_HZ, &(this->u_i_speed));
    if(i_status_value < 0)
    {
      perror("IMSPI: SPI speed (WR) atanamadi...ioctl hatasi");
      exit(1);
    }

    i_status_value = ioctl (this->i_spifd, SPI_IOC_RD_MAX_SPEED_HZ, &(this->u_i_speed));
    if(i_status_value < 0)
    {
      perror("IMSPI: SPI speed (RD) atanamadi...ioctl hatasi");
      exit(1);
    }

    return i_status_value;
}

/***********************************************************
 * spiClose(): Responsible for closing the spidev interface.
 * Called in destructor
 * *********************************************************/

int IMSPI::SpiClose()
{
    int i_status_value = -1;
    i_status_value = close(this->i_spifd);
    if(i_status_value < 0)
    {
      perror("IMSPI: SPI aygiti kapatilamadi.");
      exit(1);
    }

    return i_status_value;
}

/********************************************************************
 * This function writes data "data" of length "length" to the spidev
 * device. Data shifted in from the spidev device is saved back into
 * "data".
 * ******************************************************************/
int IMSPI::SpiWriteRead(unsigned char *p_u_c_data, int i_length)
{

  struct spi_ioc_transfer spi[i_length];
  int i = 0;
  int i_return_value = -1;

  bzero(spi, sizeof spi); // ioctl struct must be zeroed

// one spi transfer for each byte

  for (i = 0 ; i < i_length ; i++)
  {

    spi[i].tx_buf        = (unsigned long)(p_u_c_data + i); // transmit from "data"
    spi[i].rx_buf        = (unsigned long)(p_u_c_data + i) ; // receive into "data"
    spi[i].len           = sizeof(*(p_u_c_data + i)) ;
    spi[i].delay_usecs   = 0 ;
    spi[i].speed_hz      = this->u_i_speed ;
    spi[i].bits_per_word = this->u_c_bits_per_word ;
    spi[i].cs_change = 0;
}

  i_return_value = ioctl (this->i_spifd, SPI_IOC_MESSAGE(i_length), &spi) ;

 if(i_return_value < 0)
 {
    perror("IMSPI: SPI verisini transfer ederken hata olustu...ioctl hatasi");
    exit(1);
 }

return i_return_value;

}

/*************************************************
 * Default constructor. Set member variables to
 * default values and then call spiOpen()
 * ***********************************************/

IMSPI::IMSPI()
{
    this->u_c_mode = SPI_MODE_0 ;
    this->u_c_bits_per_word = 8;
    this->u_i_speed = 1000000;
    this->i_spifd = -1;

    this->SpiOpen(std::string("/dev/spidev0.0"));
}

/*************************************************
 * overloaded constructor. let user set member variables to
 * and then call spiOpen()
 * ***********************************************/
IMSPI::IMSPI(std::string str_devspi, unsigned char u_c_spi_mode, unsigned int u_i_spi_speed,
		unsigned char u_c_spi_bits_per_word)
{
    this->u_c_mode = u_c_spi_mode ;
    this->u_c_bits_per_word = u_c_spi_bits_per_word;
    this->u_i_speed = u_i_spi_speed;
    this->i_spifd = -1;

    this->SpiOpen(str_devspi);

}

/**********************************************
 * Destructor: calls spiClose()
 * ********************************************/
IMSPI::~IMSPI()
{
    this->SpiClose();
}


