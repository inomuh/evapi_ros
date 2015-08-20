#ifdef English_dox
/**
 * \file   IMPWM.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief
 * \details
 *
 * Source: This code can access to the PWM1 and PWM2 subsystems which output the PWM signal on
 * GPIO18(ALT5) and GPIO13(ALT0) respectively. Only the code(http://hertaville.com/rpipwm.html)
 * for PWM1 subsystem was upgrated.
 *
 */
#endif
#ifdef Turkish_dox
/**
 * \file   IMPWM.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief
 * \details
 *
 */
#endif

#ifndef INCLUDE_IMPWM_H_
#define INCLUDE_IMPWM_H_

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

class IMPWM
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
	IMPWM();

	#ifdef English_dox
	//! Constructor
	/**
	 * \param d_Hz Frequency
	 * \param u_i_counts PWM resolution
	 * \param d_duty duty cycle
	 * \param i_m IMPWM::MSMODE or IMPWM::PWMMODE
	 */
	#endif

	#ifdef Turkish_dox
	//! Constructor
	/**
	 * \param d_Hz Frekans
	 * \param u_i_counts PWM çözünürlüğü
	 * \param d_duty Duty cycle
	 * \param i_m IMPWM::MSMODE ya da IMPWM::PWMMODE
	 */
	#endif
	IMPWM(double d_Hz, unsigned int u_i_counts, double d_duty,  int i_m);

	~IMPWM();

	#ifdef English_dox
	//! Sets Frequency and reinitializes PWM peripheral
	/**
	 * \param & c_d_hz Frequency
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Frekansı atar ve PWM'yi yeniden başlatır.
	/**
	 * \param & c_d_hz Frekans
	 * \return
	 */
	#endif
	unsigned int SetFrequency(const double & c_d_hz);

	#ifdef English_dox
	//! Sets PWM resolution (counts) and reinitializes PWM peripheral
	/**
	 * \param & c_u_i_counts PWM resolution
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! PWM çözünürlüğünü atar ve PWM'i yeniden başlatır.
	/**
	 * \param & c_u_i_counts PWM çözünürlüğü
	 * \return
	 */
	#endif
	unsigned int SetCounts(const unsigned int & c_u_i_counts);

	#ifdef English_dox
	//! Sets Duty Cycle as a Percentage (Fast)
	/**
	 * \param &c_d_duty Duty cycle
	 * \param i_pwm_no 0: PWM1 1:PWM2
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Duty Cycle'yı yüzdelik olarak atar (Hızlı).
	/**
	 * \param &c_d_duty Duty cycle
	 * \param i_pwm_no 0: PWM1 1:PWM2
	 * \return
	 */
	#endif
	unsigned int SetDutyCycle(const double &c_d_duty, int i_pwm_no);

	#ifdef English_dox
	//! Sets Duty Cycle as a count value (Fast) i.e. if counts is 1024 and 'duty' is set to 512, a 50% duty cycle is achieved
	/**
	 * \param &c_u_i_counts
	 * \param i_pwm_no 0: PWM1 1:PWM2
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//!
	/**
	 * \param &c_u_i_counts
	 * \param i_pwm_no 0: PWM1 1:PWM2
	 * \return
	 */
	#endif
	unsigned int SetDutyCycleCount(const unsigned int &c_u_i_counts, int i_pwm_no);

	#ifdef English_dox
	//! Disables PWM, sets duty cycle and enable PWM respectively.
	/**
	 * \param &c_d_duty Duty cycle
	 * \param &c_i_m IMPWM::MSMODE or IMPWM::PWMMODE
	 * \param i_pwm_no  0: PWM1 1:PWM2
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Sırasıyla PWM'i etkisizleştirir, duty cycle atar ve PWM'mi etkinleştirir.
	/**
	 * \param &c_d_duty Duty cycle
	 * \param &c_i_m IMPWM::MSMODE ya da IMPWM::PWMMODE
	 * \param i_pwm_no 0: PWM1 1:PWM2
	 * \return
	 */
	#endif
	unsigned int SetDutyCycleForce(const double &c_d_duty, const  int &c_i_m, int i_pwm_no);


	#ifdef English_dox
	//! sets PWM model with 'setDutyCycleForce()'
	/**
	 * \param &c_i_m IMPWM::MSMODE ya da IMPWM::PWMMODE
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! 'setDutyCycleForce()' fonksiyonunu kullanarak PWM Modunu atar.
	/**
	 * \param &c_i_m IMPWM::MSMODE ya da IMPWM::PWMMODE
	 * \return
	 */
	#endif
	unsigned int SetMode(const  int &c_i_m);


	#ifdef English_dox
	//! Returns current Frequency of PWM waveform
	/**
	 *
	 */
	#endif
	#ifdef Turkish_dox
	//! PWM sinyalinin frekansını döndürür.
	/**
	 *
	 */
	#endif
	double GetFrequency() const;

	#ifdef English_dox
	//! Returns current DutyCycle (as a %) of PWM waveform
	/**
	 *
	 */
	#endif
	#ifdef Turkish_dox
	//! PWM sinyalinin Duty Cycle'ını yüzdelik olarak döndürür.
	/**
	 *
	 */
	#endif
	double GetDutyCycle() const;

	#ifdef English_dox
	//! Returns PWM resolution
	/**
	 *
	 */
	#endif
	#ifdef Turkish_dox
	//! PWM çözünürlüğünü döndürür.
	/**
	 *
	 */
	#endif
	int GetCounts() const;

	#ifdef English_dox
	//! Returns Divisor value used to set the period per count.
	/**
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Bölücünün sayısını döndürür.
	/**
	 * \return
	 */
	#endif
	int GetDivisor() const;

	#ifdef English_dox
	//! Return PWMMODE (IMPWM::MSMODE or IMPWM::PWMMODE)
	/**
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! PWMMODE (IMPWM::MSMODE or IMPWM::PWMMODE)'unu döndürür.
	/**
	 * \return
	 */
	#endif
	int GetMode() const;


	//Public constants

	// PWM MODES
	static const int PWMMODE = 1;
	static const int MSMODE = 2;

	// Error Codes
	static const int ERRFREQ = 1;
	static const int ERRCOUNT = 2;
	static const int ERRDUTY = 3;
	static const int ERRMODE = 4;

private:

	//Private constants

	//Base register addresses
	static const int BCM2708_PERI_BASE = 0x3F000000;//0x20000000;
	static const int PWM_BASE = (BCM2708_PERI_BASE + 0x20C000); /* PWM controller */
	static const int CLOCK_BASE = (BCM2708_PERI_BASE + 0x101000); /* Clock controller */
	static const int GPIO_BASE = (BCM2708_PERI_BASE + 0x200000); /* GPIO controller */

	static const int PWM_CTL = 0;
	static const int PWM_RNG1 = 4;
	static const int PWM_DAT1 = 5;
	static const int PWM_RNG2 = 8;
	static const int PWM_DAT2 = 9;
	static const int PWMCLK_CNTL= 40;
	static const int PWMCLK_DIV = 41;

	// Register addresses offsets divided by 4 (register addresses are word (32-bit) aligned
	static const int BLOCK_SIZE = 4096;
	// Block size.....every time mmap() is called a 4KB
	//section of real physical memory is mapped into the memory of
	//the process

	#ifdef English_dox
	//! Function to map physical memory
	/**
	 * \param u_l_base_address
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Fiziksel hafızaya eşleme yapan fonksiyondur.
	/**
	 * \param u_l_base_address
	 * \return
	 */
	#endif
	volatile unsigned *MapRegisterAddres(unsigned long u_l_base_address);

	#ifdef English_dox
	//! This function sets GPIO18 to the alternat function 5 (ALT5) and GPIO13 to ALT0 to enable the pin to output the PWM waveforms generated by PWM1 and PWM2
	/**
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! GPIO18'i ALT5 ve GPIO13'u ALT0 alternatif fonksiyonlara PWM sinyali çıktısını üretebilmek için ayarlamaktadır.
	/**
	 * \return
	 */
	#endif
	void ConfigPWMPin();


	#ifdef English_dox
	//! This function is responsible for the global configuration and initialization
	//! of the the PWM1 and PWM2 peripheral
	/**
	 *
	 */
	#endif
	#ifdef Turkish_dox
	//! PWM1 ve PWM2 için global ayarlama ve ilklendirme işlemlerinden sorumlu fonksiyondur.
	/**
	 *
	 */
	#endif
	void ConfigPWM();


	double d_frequency; // PWM frequency
	double d_dutyCycle; //PWM duty Cycle (%)
	unsigned int u_i_counts; // PWM resolution
	unsigned int u_i_divisor; // divisor value
	int i_mode;  // PWM mode
	volatile unsigned *p_v_u_clk, *p_v_u_pwm, *p_v_u_gpio; // pointers to the memory mapped sections
	//of our process memory

};



#endif /* INCLUDE_IMPWM_H_ */
