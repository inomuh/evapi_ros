#ifdef English_dox
/**
 * \file   IMGPIO.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief Controls GPIO's of Raspberry Pi.
 *
 * \details
 *
 * This class uses the "Linux approach" (sysfs).
 * Linux already has a built-in driver for safely accessing the GPIOs.
 * It basically views each property of each GPIO pin as a file.
 * This is the preferred method of GPIO access.
 * One disadvantage of this approach is that it tends to make for slower (but safer) GPIO pin toggling.
 *
 */
#endif
#ifdef Turkish_dox
/**
 * \file   IMGPIO.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief Raspberry Pi üzerindeki GPIO'ları kontrol eden sınıftır.
 * \details
 * Bu sınıf "Linux yaklaşımı" adı verilen yöntem kullanılmaktadır.
 * Linux'te GPIO'lara güvenli erişebilmek için driver yazılmıştır.
 * Her bir GPIO için birer dosya üzerinden kontrol edilebilmektedir.
 * Bu yöntemin dezavantjı, GPIO seviye değiştirme hızının düşük olmasıdır.
 *
 */
#endif

#ifndef INCLUDE_IMGPIO_H_
#define INCLUDE_IMGPIO_H_


#include <string>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <string>
#include <signal.h>

using namespace std;

//! Controls GPIO's of Raspberry Pi.
class IMGPIO
{
public:
	#ifdef English_dox
	//! Default Constructor
	#endif

	#ifdef Turkish_dox
	//! Default Constructor
	#endif
	IMGPIO();

	#ifdef English_dox
	//! Constructor
	/**
	 * \param str_pin_number Pin number of GPIO.
	 */
	#endif

	#ifdef Turkish_dox
	//! Constructor
	/**
	 * \param str_pin_number GPIO pin numarasıdır.
	 */
	#endif
	IMGPIO(string str_pin_number);

	//! Destructor
	~IMGPIO();

	#ifdef English_dox
	//! Sets pin direction
	/**
	 * \param str_direction IMGPIO::INPUT or IMGPIO::OUTPUT
	 * \retval <0 if there is an error
	 * \retval >=0 no error
	 */
	#endif
	#ifdef Turkish_dox
	//! Costructor'da girilen pini girdi yada çıktı olarak ayarlayan fonksiyondur.
	/**
	 * \param str_direction Giriş için; IMGPIO::INPUT ya da Çıkış için; IMGPIO::OUTPUT
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int SetPinDirection(string str_direction);

	#ifdef English_dox
	//! Sets pin value
	/**
	 * \param str_value IMGPIO::HIGH or IMGPIO::LOW
	 * \retvat <0 if there ise an error
	 * \retval >=0 no error
	 */
	#endif
	#ifdef Turkish_dox
	//! Pin değeri atayan fonksiyondur.
	/**
	 *
	 * \param str_value IMGPIO::HIGH ya da IMGPIO::LOW
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int SetPinValue(string str_value);

	#ifdef English_dox
	//! Gets pin value
	/**
	 * \param &str_value pin value.  (IMGPIO::HIGH or IMGPIO::LOW)
	 * \retvat <0 if there ise an error
	 * \retval >=0 no error
	 */
	#endif
	#ifdef Turkish_dox
	//! Pin değerini okuyan fonksiyondur.
	/**
	 * \param & str_value Okunan pin değeridir. (IMGPIO::HIGH ya da IMGPIO::LOW)
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int GetPinValue(string & str_value);

	#ifdef English_dox
	//! Returns pin number which set in Constructor.
	/**
	 *
	 * \return pin number
	 */
	#endif
	#ifdef Turkish_dox
	//! Constructor'da girilen pin numarasını döndüren fonksiyondur.
	/**
	 * \return pin değeridir.
	 */
	#endif
	string GetPinNumber() const;

	static const string RISING;
	static const string FALLING;
	static const string BOTH;

	static const string OUTPUT;
	#ifdef English_dox
		//! Value to set GPIO pin as input.
	#endif
	static const string INPUT;

	#ifdef English_dox
		//! GPIO pin value for high.
	#endif
	static const string HIGH;
	#ifdef English_dox
		//! GPIO pin value for low.
	#endif
	static const string LOW;

	#ifdef English_dox
		//! GPIO0 pin number.
	#endif
	static const string GPIO0;
	#ifdef English_dox
		//! GPIO1 pin number.
	#endif
	static const string GPIO1;
	#ifdef English_dox
		//! GPIO2 pin number.
	#endif
	static const string GPIO2;
	#ifdef English_dox
		//! GPIO3 pin number.
	#endif
	static const string GPIO3;
	#ifdef English_dox
		//! GPIO4 pin number.
	#endif
	static const string GPIO4;
	#ifdef English_dox
		//! GPIO5 pin number.
	#endif
	static const string GPIO5;
	#ifdef English_dox
		//! GPIO6 pin number.
	#endif
	static const string GPIO6;
	#ifdef English_dox
		//! GPIO7 pin number.
	#endif
	static const string GPIO7;
	#ifdef English_dox
		//! GPIO8 pin number.
	#endif
	static const string GPIO8;
	#ifdef English_dox
		//! GPIO9 pin number.
	#endif
	static const string GPIO9;
	#ifdef English_dox
		//! GPIO10 pin number.
	#endif
	static const string GPIO10;
	#ifdef English_dox
		//! GPIO11 pin number.
	#endif
	static const string GPIO11;
	#ifdef English_dox
		//! GPIO12 pin number.
	#endif
	static const string GPIO12;
	#ifdef English_dox
		//! GPIO13 pin number.
	#endif
	static const string GPIO13;

private:

	#ifdef English_dox
	//! Exports GPIO pin
	/**
	 * \retvat <0 if there is an error
	 * \retval >=0 no error
	 */
	#endif
	#ifdef Turkish_dox
	//! GPIO pinini kullanıma açar.
	/**
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
    int ExportGpio();

	#ifdef English_dox
	//! Unexports GPIO pin
	/**
	 * \retvat <0 if there ise an error
	 * \retval >=0 no error
	 */
	#endif
	#ifdef Turkish_dox
	//! GPIO pinini kullanıma kapatır.
	/**
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int UnexportGpio();

	#ifdef English_dox
		//! File description of file to write value.
	#endif
	int i_value_fd;
	#ifdef English_dox
		//! File description of file to set direction.
	#endif
	int i_direction_fd;
	#ifdef English_dox
		//! File description of file to export.
	#endif
	int i_export_fd;
	#ifdef English_dox
		//! File description of file to import.
	#endif
	int i_unexport_fd;
	#ifdef English_dox
		//! File description of file to set signal edge.
	#endif
	int i_edge_fd;
	#ifdef English_dox
		//! GPIO pin number.
	#endif
	string str_pin_number;
};

#endif /* INCLUDE_IMGPIO_H_ */
