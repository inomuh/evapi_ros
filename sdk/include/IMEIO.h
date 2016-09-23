#ifdef English_dox
/**
 * \file   IMEIO.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Jun, 2015
 * \brief  Interfacing an I2C GPIO expander (MCP23017, https://cdn-shop.adafruit.com/datasheets/mcp23017.pdf) to the Raspberry Pi.
 * \details
 *
 */
#endif
#ifdef Turkish_dox
/**
 * \file   IMEIO.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Haz, 2015
 * \brief  Raspberry Pi için I2C GPIO genişletici (MCP23017) arayüzüdür.
 * \details
 *
 */
#endif

#ifndef INCLUDE_IMEIO_H_
#define INCLUDE_IMEIO_H_

#include "IMI2C.h"

using namespace std;

//! Interfacing an I2C GPIO expander (MCP23017, https://cdn-shop.adafruit.com/datasheets/mcp23017.pdf) to the Raspberry Pi.
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
	//! Sets pin direction as input or output for PORTA
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
	 * \param i_pin_number genişletilmiş girdi/çıktı pin no YA DA tanımlanmış değişkenler örneğin, IMEIO::RGB_LED1
	 * \param b_direction IMEIO::OUTPUT YA DA IMEIO::INPUT
	 * \retval <0 hata varsa
	 * \retval >0 hata yoksa.
	 */
	#endif
	int SetPinADirection(int i_pin_number, bool b_direction);

	#ifdef English_dox
	//! Sets pin direction as input or output for PORTB
	/**
	 * \param i_pin_number extended io no OR defined variables i.e. IMEIO::EIO0
	 * \param b_direction IMEIO::OUTPUT or IMEIO::INPUT
	 * \retval <0 if there is an error
	 * \retval >0 if everthing is OK.
	 */
	#endif
	#ifdef Turkish_dox
	//! PORTB için pin yönelimini(girdi/çıktı) ayarlayan fonksiyondur.
	/**
	 * \param i_pin_number genişletilmiş girdi/çıktı pin no YA DA tanımlanmış değişkenler örneğin, IMEIO::EIO0
	 * \param b_direction IMEIO::OUTPUT YA DA IMEIO::INPUT
	 * \retval <0 hata varsa
	 * \retval >0 hata yoksa.
	 */
	#endif
	int SetPinBDirection(int i_pin_number, bool b_direction);

	#ifdef English_dox
	//! Sets pin value as input or output for PORTA
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
	//! Sets pin value as input or output for PORTB
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
	//! Gets pin value as input or output for PORTA
	/**
	 * \param i_pin_number extended io no OR defined variables i.e. IMEIO::RGB_LED1
	 * \param &b_value value of pin number (IMEIO::HIGH or IMEIO::LOW)
	 * \retval <0 if there is an error
	 * \retval >0 if everthing is OK.
	 */
	#endif
	#ifdef Turkish_dox
	//! PORTA için pin değerini okuyan fonksiyondur.
	/**
	 * \param i_pin_number genişletilmiş pin no YA DA tanımlanmış değişkenler örneğin, IMEIO::RGB_LED1
	 * \param &b_value pinin okunan değeridir.
	 * \retval <0 hata varsa
	 * \retval >0 hata yoksa.
	 */
	#endif
	int GetPinAValue(int i_pin_number, bool & b_value);

	#ifdef English_dox
	//! Gets pin value as input or output for PORTB
	/**
	 * \param i_pin_number extended io no OR defined variables i.e. IMEIO::EIO0
	 * \param &b_value value of pin number (IMEIO::HIGH or IMEIO::LOW)
	 * \retval <0 if there is an error
	 * \retval >0 if everthing is OK.
	 */
	#endif
	#ifdef Turkish_dox
	//! PORTB için pin değerini okuyan fonksiyondur.
	/**
	 * \param i_pin_number genişletilmiş pin no YA DA tanımlanmış değişkenler örneğin, IMEIO::EIO0
	 * \param &b_value pinin okunan değeridir.
	 * \retval <0 hata varsa
	 * \retval >0 hata yoksa.
	 */
	#endif
	int GetPinBValue(int i_pin_number, bool & b_value);

	#ifdef English_dox
		//! Extended IO variable to set a pin as output.
	#endif
	static const bool OUTPUT = false;
	#ifdef English_dox
		//! Extended IO variable to set a pin as input.
	#endif
	static const bool INPUT = true;

	#ifdef English_dox
		//! Extended IO variable to set a pin high.
	#endif
	static const bool HIGH = true;
	#ifdef English_dox
		//! Extended IO variable to set a pin low.
	#endif
	static const bool LOW = false;

	// PORTA
	#ifdef English_dox
		//! Pin number of RGB_LED1 on PORTA.
	#endif
	static const int RGB_LED1 = 0;
	#ifdef English_dox
		//! Pin number of RGB_LED2 on PORTA.
	#endif
	static const int RGB_LED2 = 1;
	#ifdef English_dox
		//! Pin number of RGB_LED3 on PORTA.
	#endif
	static const int RGB_LED3 = 2;
	#ifdef English_dox
		//! Pin number of BUZZER on PORTA.
	#endif
	static const int BUZZER = 3;

	#ifdef English_dox
		//! Pin number of BUMPER0 on PORTA.
	#endif
	static const int BUMPER0 = 4;
	#ifdef English_dox
		//! Pin number of BUMPER1 on PORTA.
	#endif
	static const int BUMPER1 = 5;
	#ifdef English_dox
		//! Pin number of BUMPER2 on PORTA.
	#endif
	static const int BUMPER2 = 6;

	#ifdef English_dox
		//! Pin number of USER_LED on PORTA.
	#endif
	static const int USER_LED = 7;


	//PORTB
	#ifdef English_dox
		//! Pin number of SWR_RESET on PORTB.
	#endif
	static const int SWR_RESET = 0;

	#ifdef English_dox
		//! Pin number of EIO0 on PORTB.
	#endif
	static const int EIO0 = 1;
	#ifdef English_dox
		//! Pin number of EIO1 on PORTB.
	#endif
	static const int EIO1 = 2;
	#ifdef English_dox
		//! Pin number of EIO2 on PORTB.
	#endif
	static const int EIO2 = 3;
	#ifdef English_dox
		//! Pin number of EIO3 on PORTB.
	#endif
	static const int EIO3 = 4;
	#ifdef English_dox
		//! Pin number of EIO4 on PORTB.
	#endif
	static const int EIO4 = 5;
	#ifdef English_dox
		//! Pin number of EIO5 on PORTB.
	#endif
	static const int EIO5 = 6;
	#ifdef English_dox
		//! Pin number of EIO6 on PORTB.
	#endif
	static const int EIO6 = 7;


private:
	#ifdef English_dox
		//! Value of IODIRA register.
	#endif
	char c_pins_directions_A;
	#ifdef English_dox
		//! Value of GPIOA register.
	#endif
	char c_pins_data_A;

	#ifdef English_dox
		//! Value of IODIRB register.
	#endif
	char c_pins_directions_B;
	#ifdef English_dox
		//! Value of GPIOB register.
	#endif
	char c_pins_data_B;

	#ifdef English_dox
	//! Class IMI2C is used for communicating with I2C device.
	#endif
	IMI2C * i2c;

	#ifdef English_dox
		//! Address of IODIRA register.
	#endif
	static const char IODIRA = 0x00;
	#ifdef English_dox
		//! Address of IODIRB register.
	#endif
	static const char IODIRB = 0x01;

	#ifdef English_dox
		//! Address of GPIOA register.
	#endif
	static const char GPIOA = 0x12;
	#ifdef English_dox
		//! Address of GPIOB register.
	#endif
	static const char GPIOB = 0x13;

	#ifdef English_dox
		//! Address of OLATA register.
	#endif
	static const char OLATA = 0x14;
	#ifdef English_dox
		//! Address of OLATB register.
	#endif
	static const char OLATB = 0x15;
};

#endif /* INCLUDE_IMEIO_H_ */
