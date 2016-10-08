#ifdef English_dox
/**
 * \file   IMSerial.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Controls serial communication.
 * \details
 *
 */
#endif
#ifdef Turkish_dox
/**
 * \file   IMSerial.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Seri haberleşmeyi kontrol eder.
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
#include <stdint.h>
using namespace std;

//! Controls serial communication.
class IMSerial
{
public:

	#ifdef English_dox
	//! Default Constructor
	#endif

	#ifdef Turkish_dox
	//! Default Constructor
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
	//! Destructor
	#endif

	#ifdef Turkish_dox
	//! Destructor
	#endif
	~IMSerial();

	#ifdef English_dox
	//! Sets Baudrate
	/**
	 * \param baudrate i.e. B115200
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	#ifdef Turkish_dox
	//! Baudrate değerini atar.
	/**
	 * \param baudrate ör. B115200
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int SetBaudrate(speed_t baudrate);

	#ifdef English_dox
	//! Sets DataBits
	/**
	 * \param data_bits i.e. CS8
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	#ifdef Turkish_dox
	//! DataBits değerini atar.
	/**
	 * \param data_bits ör. CS8
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int SetDataBits(tcflag_t data_bits);

	#ifdef English_dox
	//! Sets Parity
	/**
	 * \param parity i.e. PARODD
	 * \param parity_on i.e. PARENB
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	#ifdef Turkish_dox
	//! Parity değerini atar
	/**
	 * \param parity ör. PARODD
	 * \param parity_on ör. PARENB
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int SetParity(tcflag_t parity, tcflag_t parity_on);

	#ifdef English_dox
	//! Sets Stop Bits
	/**
	 * \param stop_bits Stop bit value.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	#ifdef Turkish_dox
	//! Stop bitini ayarlar
	/**
	 * \param stop_bits Stop biti değeridir.
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int SetStopBits(tcflag_t stop_bits);

	#ifdef English_dox
	//! Returns Baudrate
	/**
	 * \return Baudrate
	 */
	#endif
	#ifdef Turkish_dox
	//! Baudrate'i döndürür.
	/**
	 * \return Baudrate
	 */
	#endif
	speed_t GetBaudrate() const;

	#ifdef English_dox
	//! Returns DataBits
	/**
	 * \return DataBits
	 */
	#endif
	#ifdef Turkish_dox
	//! DataBits değerini döndürür.
	/**
	 * \return DataBits
	 */
	#endif
	tcflag_t GetDataBits() const;

	#ifdef English_dox
	//! Returns Parity
	/**
	 * \return Parity
	 */
	#endif
	#ifdef Turkish_dox
	//! Parity değerinin döndürür.
	/**
	 * \return Parity
	 */
	#endif
	tcflag_t GetParity() const;

	#ifdef English_dox
	//! Returns StopBits
	/**
	 * \return StopBits
	 */
	#endif
	#ifdef Turkish_dox
	//! StopBit değerini döndürür.
	/**
	 * \return StopBits
	 */
	#endif
	tcflag_t GetStopBits() const;

	#ifdef English_dox
	//! Reads serial data.
	/**
	 * \param *p_c_data Data from serial port
	 * \param u_i_size Size of data
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	#ifdef Turkish_dox
	//! Seri porttan gelen veriyi okur.
	/**
	 * \param *p_c_data okunan veri
	 * \param u_i_size okunacak veri boyutu
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int ReadData(char * p_c_data, unsigned int u_i_size);

	#ifdef English_dox
	//! Reads serial data asynchronously
	/**
	 * \param *p_c_data Data from serial port
	 * \param u_i_size Size of data
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	#ifdef Turkish_dox
	//! Asenkron olarak veriyi okur.
	/**
	 * \param *p_c_data okunan veri
	 * \param u_i_size okunacak verinin boyutu
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int ReadDataAsync(char * p_c_data, unsigned int u_i_size);

	#ifdef English_dox
	//! Writes data to serial port.
	/**
	 * \param *p_data Data to write
	 * \param size Size of data
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	#ifdef Turkish_dox
	//! Seri porta veri yazar.
	/**
	 * \param *p_data yazılacak veri
	 * \param size yazılacak veri boyutu
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int WriteData(const void * p_data, size_t size);

	#ifdef English_dox
	//! Returns data is ready or not.
	/**
	 * \return data ready or not
	 */
	#endif
	#ifdef Turkish_dox
	//! Verinin hazır olup olmadığını döndürür.
	/**
	 * \return veri hazır ya da değil
	 */
	#endif
	bool isReadyRead() const;

private:
	#ifdef English_dox
	//! Opens serial port
	/**
	 * \param _str_name serial driver name
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	#ifdef Turkish_dox
	//! Seri bağlantıyı açar.
	/**
	 * \param _str_name seri sürücünün adı
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int SerialOpen(std::string _str_name);

	#ifdef English_dox
	//! Closes serial port
	/**
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	#ifdef Turkish_dox
	//! Seri bağlantıyı kapatır.
	/**
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int SerialClose();

	#ifdef English_dox
	//! Function to configurate and initialize Serial Communication Peripheral
	/**
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	#ifdef Turkish_dox
	//! Seri ahberleşmenin ayarlarını ve ilklendirmesi yapan fpnksiyondur.
	/**
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int SerialConfig();

	#ifdef English_dox
	//! Signal handler
	#endif
	#ifdef Turkish_dox
	//! Sinyal yakalayıcı
	#endif
	static void SignalHandlerIO(int i_status);

	#ifdef Turkish_dox
	//! Name of serial port
	#endif
	std::string str_name;
	#ifdef Turkish_dox
	//! Specifies how fast data is sent over a serial line.
	#endif
	speed_t baudrate;
	#ifdef Turkish_dox
	//! A bit mask specifying flag for data bits.
	#endif
	tcflag_t data_bits;
	#ifdef Turkish_dox
	//! A bit mask specifying flag for parity.
	#endif
	tcflag_t parity;
	#ifdef Turkish_dox
	//! A bit mask specifying flag for parity on.
	#endif
	tcflag_t parity_on;
	#ifdef Turkish_dox
	//! A bit mask specifying flag for stop bits.
	#endif
	tcflag_t stop_bits;

	#ifdef Turkish_dox
	//! File descriptor for serial port.
	#endif
	int i_serial_fd;

	#ifdef Turkish_dox
	//! Signal handler.
	#endif
	struct sigaction signal_action_io;
	#ifdef Turkish_dox
	//! Port setting for serial communication.
	#endif
	struct termios port_settings;

	#ifdef Turkish_dox
	//! Stores new signal is handled or not.
	#endif
	static bool b_signal_received;
};

#endif /* INCLUDE_IMSERIAL_H_ */
