#ifdef English_dox
/**
 * \file   IMSerial.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief
 * \details
 *
 */
#endif
#ifdef Turkish_dox
/**
 * \file   IMSerial.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief
 * \details
 *
 */
#endif

#ifndef INCLUDE_IMSERIAL_H_
#define INCLUDE_IMSERIAL_H_

#include <termios.h>
#include <string.h>
#include <string>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <cstdlib>

using namespace std;

//! Callback function pointer.
typedef void(*CallbackFunctionPtr)(void*);

class IMSerial
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
	IMSerial();

	#ifdef English_dox
	//! Constructor
	/**
	 * \param str_device_name
	 * \param baudrate i.e. B115200
	 * \param data_bits i.e. CS8
	 * \param parity i.e. PARODD
	 * \param parity_on i.e. PARENB
	 * \param stop_bits i.e. 0
	 */
	#endif

	#ifdef Turkish_dox
	//! Constructor
	/**
	 * \param str_device_name
	 * \param baudrate ör. B115200
	 * \param data_bits ör. CS8
	 * \param parity ör. PARODD
	 * \param parity_on ör. PARENB
	 * \param stop_bits ör. 0
	 */
	#endif
	IMSerial(string str_device_name, speed_t baudrate, tcflag_t data_bits,
			tcflag_t parity, tcflag_t parity_on, tcflag_t stop_bits);

	#ifdef English_dox
	//! Sets Baudrate
	/**
	 * \param baudrate i.e. B115200
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Baudrate değerini atar.
	/**
	 * \param baudrate ör. B115200
	 * \return
	 */
	#endif
	int SetBaudrate(speed_t baudrate);

	#ifdef English_dox
	//! Sets DataBits
	/**
	 * \param data_bits i.e. CS8
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! DataBits değerini atar.
	/**
	 * \param data_bits ör. CS8
	 * \return
	 */
	#endif
	int SetDataBits(tcflag_t data_bits);

	#ifdef English_dox
	//! Sets Parity
	/**
	 * \param parity i.e. PARODD
	 * \param parity_on i.e. PARENB
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Parity değerini atar
	/**
	 * \param parity ör. PARODD
	 * \param parity_on ör. PARENB
	 * \return
	 */
	#endif
	int SetParity(tcflag_t parity, tcflag_t parity_on);

	#ifdef English_dox
	//! Sets Stop Bits
	/**
	 * \param stop_bits
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetStopBits(tcflag_t stop_bits);

	#ifdef English_dox
	//! Returns Baudrate
	/**
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Baudrate'i döndürür.
	/**
	 * \return
	 */
	#endif
	speed_t GetBaudrate() const;

	#ifdef English_dox
	//! Returns DataBits
	/**
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! DataBits değerini döndürür.
	/**
	 * \return
	 */
	#endif
	tcflag_t GetDataBits() const;

	#ifdef English_dox
	//! Returns Parity
	/**
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Parity değerinin döndürür.
	/**
	 * \return
	 */
	#endif
	tcflag_t GetParity() const;

	#ifdef English_dox
	//! Returns StopBits
	/**
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! StopBit değerini döndürür.
	/**
	 * \return
	 */
	#endif
	tcflag_t GetStopBits() const;

	#ifdef English_dox
	//! Reads serial data.
	/**
	 * \param * p_c_data
	 * \param u_i_size
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Seri porttan gelen veriyi okur.
	/**
	 * \param * p_c_data okunan veri
	 * \param u_i_size okunacak veri boyutu
	 * \return
	 */
	#endif
	int ReadData(char * p_c_data, unsigned int u_i_size);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int ReadDataAsync(char * p_c_data, unsigned int u_i_size);

	//int WriteData(const char * p_c_data, unsigned int u_i_size);

	#ifdef English_dox
	//! Writes serial data.
	/**
	 * \param * p_c_data
	 * \param u_i_size
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Seri porta veri yazar.
	/**
	 * \param * p_c_data yazılacak veri
	 * \param u_i_size yazılacak veri boyutu
	 * \return
	 */
	#endif
	int WriteData(const void * p_data, size_t size);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	bool isReadyRead() const;

private:

	#ifdef English_dox
	//! Opens serial port
	/**
	 * \param _str_name serial driver name
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Seri bağlantıyı açar.
	/**
	 * \param _str_name seri sürücünün adı
	 * \return
	 */
	#endif
	int SerialOpen(std::string _str_name);

	#ifdef English_dox
	//! Closes serial port
	/**
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Seri bağlantıyı kapatır.
	/**
	 * \return
	 */
	#endif
	int SerialClose();

	#ifdef English_dox
	//! Function to configurate and initialize Serial Communication Peripheral
	/**
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Seri ahberleşmenin ayarlarını ve ilklendirmesi yapan fpnksiyondur.
	/**
	 * \return
	 */
	#endif
	int SerialConfig();

	#ifdef English_dox
	//! Signal handler
	/**
	 * \param i_status
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//!
	/**
	 * \return
	 */
	#endif
	static void SignalHandlerIO(int i_status);

	std::string str_name;
	speed_t baudrate;
	tcflag_t data_bits;
	tcflag_t parity;
	tcflag_t parity_on;
	tcflag_t stop_bits;

	int i_serial_fd;

	bool b_read_data_available;

	struct sigaction signal_action_io;
	struct termios port_settings;

	static bool b_signal_received;

};



#endif /* INCLUDE_IMSERIAL_H_ */
