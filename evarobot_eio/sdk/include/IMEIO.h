#ifdef English_dox
/**
 * \file   IMEIO.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Jun, 2015
 * \brief  Reads analog inputs through MCP3208 on evarobot sensorboard.
 * \details
 *
 */
#endif
#ifdef Turkish_dox
/**
 * \file   IMEIO.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Haz, 2015
 * \brief
 * \details
 *
 */
#endif

#ifndef INCLUDE_IMEIO_H_
#define INCLUDE_IMEIO_H_


#include "IMI2C.h"

using namespace std;

class IMEIO
{
public:
	#ifdef English_dox
	//! Constructor
	/**
	 * \param u_c_device_address address of the device connected to i2c
	 * \param str_i2c_file_name i2c driver name
	 * \param mutex mutex to use i2c driver simultaneously
	 */
	#endif

	#ifdef Turkish_dox
	//! Constructor
	/**
	 * \param u_c_device_address i2c^ye bağlanan aygıtın adresi
	 * \param str_i2c_file_name i2c sürücüsünün adı
	 * \param mutex eş zamanlı olarak i2c sürücüsüne erişebilmek için mutex
	 */
	#endif
	IMEIO(unsigned char u_c_device_address, string str_i2c_file_name, sem_t * mutex);

	~IMEIO();

	#ifdef English_dox
	//! Set pin direction as input or output for PORTA
	/**
	 *
	 * \param i_pin_number extended io no OR defined variables i.e. IMEIO::RGB_LED1
	 * \param b_direction IMEIO::OUTPUT or IMEIO::INPUT
	 * \retval <0 if there is an error
	 * \retval >0 if everthing is OK.
	 */
	#endif
	#ifdef Turkish_dox
	//! PORTA için pin yönelimini(girdi/çıktı) ayarlayan fonksiyondur.
	/**
	 * Detaylı bilgi
	 * \param i_pin_number genişletilmiş girdi/çıktı pin no YA DA tanımlanmış değişkenler örneğin, IMEIO::RGB_LED1
	 * \param b_direction IMEIO::OUTPUT YA DA IMEIO::INPUT
	 * \retval <0 hata varsa
	 * \retval >0 hata yoksa.
	 */
	#endif
	int SetPinADirection(int i_pin_number, bool b_direction);

	#ifdef English_dox
	//! Set pin direction as input or output for PORTB
	/**
	 * \param i_pin_number extended io no OR defined variables i.e. IMEIO::EIO0
	 * \param b_direction IMEIO::OUTPUT or IMEIO::INPUT
	 * \retval <0 if there is an error
	 * \retval >0 if everthing is OK.
	 * 	 */
	#endif
	#ifdef Turkish_dox
	//! PORTB için pin yönelimini(girdi/çıktı) ayarlayan fonksiyondur.
	/**
	 * Detaylı bilgi
	 * \param i_pin_number genişletilmiş girdi/çıktı pin no YA DA tanımlanmış değişkenler örneğin, IMEIO::EIO0
	 * \param b_direction IMEIO::OUTPUT YA DA IMEIO::INPUT
	 * \retval <0 hata varsa
	 * \retval >0 hata yoksa.
	 */
	#endif
	int SetPinBDirection(int i_pin_number, bool b_direction);

	#ifdef English_dox
	//! Set pin value as input or output for PORTA
	/**
	 * \param i_pin_number extended io no OR defined variables i.e. IMEIO::RGB_LED1
	 * \param b_value IMEIO::HIGH or IMEIO::LOW
	 * \retval <0 if there is an error
	 * \retval >0 if everthing is OK.
	 */
	#endif
	#ifdef Turkish_dox
	//! PORTA için pin değerini(1/0) ayarlayan fonksiyondur.
	/**
	 * \param i_pin_number genişletilmiş pin no YA DA tanımlanmış değişkenler örneğin, IMEIO::RGB_LED1
	 * \param b_value IMEIO::HIGH YA DA IMEIO::LOW
	 * \retval <0 hata varsa
	 * \retval >0 hata yoksa.
	 */
	#endif
	int SetPinAValue(int i_pin_number, bool b_value);

	#ifdef English_dox
	//! Set pin value as input or output for PORTB
	/**
	 * \param i_pin_number extended io no OR defined variables i.e. IMEIO::EIO0
	 * \param b_value IMEIO::HIGH or IMEIO::LOW
	 * \retval <0 if there is an error
	 * \retval >0 if everthing is OK.
	 */
	#endif
	#ifdef Turkish_dox
	//! PORTB için pin değerini(1/0) ayarlayan fonksiyondur.
	/**
	 * \param i_pin_number genişletilmiş pin no YA DA tanımlanmış değişkenler örneğin, IMEIO::EIO0
	 * \param b_value IMEIO::HIGH YA DA IMEIO::LOW
	 * \retval <0 hata varsa
	 * \retval >0 hata yoksa.
	 */
	#endif
	int SetPinBValue(int i_pin_number, bool b_value);

	#ifdef English_dox
	//! Get pin value as input or output for PORTA
	/**
	 * Detailed
	 * \param i_pin_number extended io no OR defined variables i.e. IMEIO::RGB_LED1
	 * \param b_value value of pin number (IMEIO::HIGH or IMEIO::LOW)
	 * \retval <0 if there is an error
	 * \retval >0 if everthing is OK.
	 */
	#endif
	#ifdef Turkish_dox
	//! PORTA için pin değerini okuyan fonksiyondur.
	/**
	 * Detaylı bilgi
	 * \param i_pin_number genişletilmiş pin no YA DA tanımlanmış değişkenler örneğin, IMEIO::RGB_LED1
	 * \param b_value pinin okunan değeridir.
	 * \retval <0 hata varsa
	 * \retval >0 hata yoksa.
	 */
	#endif
	int GetPinAValue(int i_pin_number, bool & b_value);

	#ifdef English_dox
	//! Get pin value as input or output for PORTB
	/**
	 * Detailed
	 * \param i_pin_number extended io no OR defined variables i.e. IMEIO::EIO0
	 * \param b_value value of pin number (IMEIO::HIGH or IMEIO::LOW)
	 * \retval <0 if there is an error
	 * \retval >0 if everthing is OK.
	 */
	#endif
	#ifdef Turkish_dox
	//! PORTB için pin değerini okuyan fonksiyondur.
	/**
	 * Detaylı bilgi
	 * \param i_pin_number genişletilmiş pin no YA DA tanımlanmış değişkenler örneğin, IMEIO::EIO0
	 * \param b_value pinin okunan değeridir.
	 * \retval <0 hata varsa
	 * \retval >0 hata yoksa.
	 */
	#endif
	int GetPinBValue(int i_pin_number, bool & b_value);

	static const bool OUTPUT;
	static const bool INPUT;

	static const bool HIGH;
	static const bool LOW;

	// PORTA
	static const int RGB_LED1;
	static const int RGB_LED2;
	static const int RGB_LED3;
	static const int BUZZER;

	static const int BUMPER0;
	static const int BUMPER1;
	static const int BUMPER2;

	static const int USER_LED;


	//PORTB

	static const int SWR_RESET;

	static const int EIO0;
	static const int EIO1;
	static const int EIO2;
	static const int EIO3;
	static const int EIO4;
	static const int EIO5;
	static const int EIO6;



private:
	unsigned char u_c_pins_directions_A;
	unsigned char u_c_pins_data_A;

	unsigned char u_c_pins_directions_B;
	unsigned char u_c_pins_data_B;

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param str_pin_number
	 * \param u_c_pin
	 * \param u_c_port
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param str_pin_number
	 * \param u_c_pin
	 * \param u_c_port
	 * \return
	 */
	#endif

	IMI2C * i2c;

	static const unsigned char IODIRA = 0x00;
	static const unsigned char IODIRB = 0x01;

	static const unsigned char GPIOA = 0x12;
	static const unsigned char GPIOB = 0x13;

	static const unsigned char OLATA = 0x14;
	static const unsigned char OLATB = 0x15;


};



#endif /* INCLUDE_IMEIO_H_ */
