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


//bool g_b_exit_signal = false;

// Callback function pointer.
typedef void(*CallbackFunctionPtr)(void*, int);
void CatchSignal(int i_sig);

class IMGPIO
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
	IMGPIO();

	#ifdef English_dox
	//! Constructor
	/**
	 * \param str_pin_number
	 */
	#endif

	#ifdef Turkish_dox
	//! Constructor
	/**
	 * \param str_pin_number
	 */
	#endif
	IMGPIO(string str_pin_number);

	~IMGPIO();

	#ifdef English_dox
	//! Sets pin direction
	/**
	 * Detailed
	 * \param str_direction IMGPIO::INPUT or IMGPIO::OUTPUT
	 * \retval <0 if there is an error
	 * \retval >=0 no error
	 */
	#endif
	#ifdef Turkish_dox
	//! Costructor'da girilen pin girdi yada çıktı olarak ayarlayan fonksiyondur.
	/**
	 *
	 * \param str_direction Giriş için; IMGPIO::INPUT ya da Çıkış için; IMGPIO::OUTPUT
	 * \retval <0 hata varsa
	 * \retval >=0 hata yoksa
	 */
	#endif
	int SetPinDirection(string str_direction);

//	int SetInterruptMode(string str_int_mode);

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
	 * Detailed
	 * \param & str_value is pin value read.  (IMGPIO::HIGH or IMGPIO::LOW)
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
	//! Returns pin number is set in Constructor.
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

//	void SetCallback(CallbackFunctionPtr _callback_function, void *p);

	//! Constant Değişkenler.

	static const string RISING;
	static const string FALLING;
	static const string BOTH;

	static const string OUTPUT;
	static const string INPUT;

	static const string HIGH;
	static const string LOW;

	static const string GPIO0;
	static const string GPIO1;
	static const string GPIO2;
	static const string GPIO3;
	static const string GPIO4;
	static const string GPIO5;
	static const string GPIO6;
	static const string GPIO7;
	static const string GPIO8;
	static const string GPIO9;
	static const string GPIO10;
	static const string GPIO11;
	static const string GPIO12;
	static const string GPIO13;

private:

	#ifdef English_dox
	//! Exports GPIO pin
	/**
	 * \retvat <0 if there ise an error
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

//	int PollGpio();

	int i_value_fd;
	int i_direction_fd;
	int i_export_fd;
	int i_unexport_fd;
	int i_edge_fd;
	string str_pin_number;
//	CallbackFunctionPtr callback_function;
//	void *p_v_callback_pointer;
//	string str_interrupt_mode;

//	bool b_exit_signal;
//	void CatchSignal(int i_sig);


};

#endif /* INCLUDE_IMGPIO_H_ */
