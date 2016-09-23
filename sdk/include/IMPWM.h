#ifdef English_dox
/**
 * \file   IMPWM.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Controls PWM of Raspberry Pi.
 * \details
 *
 * Source: This code can access to the PWM1 and PWM2 subsystems which output the PWM signal on
 * GPIO18(ALT5) and GPIO13(ALT0) respectively. Only the code(http://hertaville.com/rpipwm.html)
 * for PWM1 subsystem was upgraded.
 *
 */
#endif
#ifdef Turkish_dox
/**
 * \file   IMPWM.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Raspberry Pi üzerindeki PWM'i kontrol eder.
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

//! Controls PWM of Raspberry Pi.
class IMPWM
{

public:
	#ifdef English_dox
	//! Default Constructor
	#endif

	#ifdef Turkish_dox
	//! Default Constructor
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
	 * \param &c_d_hz Frequency
	 */
	#endif
	#ifdef Turkish_dox
	//! Frekansı atar ve PWM'yi yeniden başlatır.
	/**
	 * \param &c_d_hz Frekans
	 */
	#endif
	void SetFrequency(const double & c_d_hz);

	#ifdef English_dox
	//! Sets PWM resolution (counts) and reinitializes PWM peripheral
	/**
	 * \param &c_u_i_counts PWM resolution
	 */
	#endif
	#ifdef Turkish_dox
	//! PWM çözünürlüğünü atar ve PWM'i yeniden başlatır.
	/**
	 * \param &c_u_i_counts PWM çözünürlüğü
	 */
	#endif
	void SetCounts(const unsigned int & c_u_i_counts);

	#ifdef English_dox
	//! Sets Duty Cycle as a Percentage (Fast)
	/**
	 * \param &c_d_duty Duty cycle
	 * \param i_pwm_no 0: PWM1 1:PWM2
	 */
	#endif
	#ifdef Turkish_dox
	//! Duty Cycle'yı yüzdelik olarak ayarlar (Hızlı)
	/**
	 * \param &c_d_duty Duty cycle
	 * \param i_pwm_no 0: PWM1 1:PWM2
	 */
	#endif
	void SetDutyCycle(const double &c_d_duty, int i_pwm_no);

	#ifdef English_dox
	//! Sets Duty Cycle as a count value (Fast) i.e. if counts is 1024 and 'duty' is set to 512, a 50% duty cycle is achieved
	/**
	 * \param &c_u_i_counts PWM resolution
	 * \param i_pwm_no 0: PWM1 1:PWM2
	 */
	#endif
	#ifdef Turkish_dox
	//!
	/**
	 * \param &c_u_i_counts PWM çözünürlüğü
	 * \param i_pwm_no 0: PWM1 1:PWM2
	 */
	#endif
	void SetDutyCycleCount(const unsigned int &c_u_i_counts, int i_pwm_no);

	#ifdef English_dox
	//! Disables PWM, sets duty cycle and enable PWM respectively.
	/**
	 * \param &c_d_duty Duty cycle
	 * \param &c_i_m IMPWM::MSMODE or IMPWM::PWMMODE
	 * \param i_pwm_no  0: PWM1 1:PWM2
	 */
	#endif
	#ifdef Turkish_dox
	//! Sırasıyla PWM'i etkisizleştirir, duty cycle atar ve PWM'mi etkinleştirir.
	/**
	 * \param &c_d_duty Duty cycle
	 * \param &c_i_m IMPWM::MSMODE ya da IMPWM::PWMMODE
	 * \param i_pwm_no 0: PWM1 1:PWM2
	 */
	#endif
	void SetDutyCycleForce(const double &c_d_duty, const  int &c_i_m, int i_pwm_no);


	#ifdef English_dox
	//! sets PWM model with 'setDutyCycleForce()'
	/**
	 * \param &c_i_m IMPWM::MSMODE or IMPWM::PWMMODE
	 */
	#endif
	#ifdef Turkish_dox
	//! 'setDutyCycleForce()' fonksiyonunu kullanarak PWM Modunu atar.
	/**
	 * \param &c_i_m IMPWM::MSMODE ya da IMPWM::PWMMODE
	 */
	#endif
	void SetMode(const int &c_i_m);


	#ifdef English_dox
	//! Returns current Frequency of PWM waveform
	/**
	 * \return current Frequency of PWM waveform
	 */
	#endif
	#ifdef Turkish_dox
	//! PWM sinyalinin frekansını döndürür.
	/**
	 * \return PWM sinyalinin frekansı
	 */
	#endif
	double GetFrequency() const;

	#ifdef English_dox
	//! Returns current DutyCycle (as a %) of PWM waveform
	/**
	 * \return current DutyCycle (as a %) of PWM waveform
	 */
	#endif
	#ifdef Turkish_dox
	//! PWM sinyalinin Duty Cycle'ını yüzdelik olarak döndürür.
	/**
	 * \return Yüzdelik olarak PWM sinyalinin Duty Cycle'ı
	 */
	#endif
	double GetDutyCycle() const;

	#ifdef English_dox
	//! Returns PWM resolution
	/**
	 * \return PWM resolution
	 */
	#endif
	#ifdef Turkish_dox
	//! PWM çözünürlüğünü döndürür.
	/**
	 * \return PWM çözünürlüğü
	 */
	#endif
	int GetCounts() const;

	#ifdef English_dox
	//! Returns Divisor value used to set the period per count.
	/**
	 * \return Divisor value
	 */
	#endif
	#ifdef Turkish_dox
	//! Bölücünün sayısını döndürür.
	/**
	 * \return Bölücünün sayısı
	 */
	#endif
	int GetDivisor() const;

	#ifdef English_dox
	//! Return PWMMODE (IMPWM::MSMODE or IMPWM::PWMMODE)
	/**
	 * \return PWMMODE (IMPWM::MSMODE or IMPWM::PWMMODE)
	 */
	#endif
	#ifdef Turkish_dox
	//! PWMMODE (IMPWM::MSMODE or IMPWM::PWMMODE)'unu döndürür.
	/**
	 * \return PWMMODE (IMPWM::MSMODE or IMPWM::PWMMODE)
	 */
	#endif
	int GetMode() const;

	//Public constants

	// PWM MODES
	#ifdef English_dox
		//! Continuous PWM mode
	#endif
	static const int PWMMODE = 1;
	#ifdef English_dox
		//! Sectional PWM mode
	#endif
	static const int MSMODE = 2;

private:
	//Base register addresses
	#ifdef English_dox
		//! BCM2708 peripherals base register address
	#endif
	static const int BCM2708_PERI_BASE = 0x3F000000;
	#ifdef English_dox
		//! BCM2708 PWM controller base register address
	#endif
	static const int PWM_BASE = (BCM2708_PERI_BASE + 0x20C000);
	#ifdef English_dox
		//! BCM2708 Clock controller base register address
	#endif
	static const int CLOCK_BASE = (BCM2708_PERI_BASE + 0x101000);
	#ifdef English_dox
		//! BCM2708 GPIO controller base register address
	#endif
	static const int GPIO_BASE = (BCM2708_PERI_BASE + 0x200000);

	#ifdef English_dox
		//! CTL - PWM Control shift value
	#endif
	static const int PWM_CTL = 0;
	#ifdef English_dox
		//! RNG1 - PWM Control shift value
	#endif
	static const int PWM_RNG1 = 4;
	#ifdef English_dox
		//! DAT1 - PWM Control shift value
	#endif
	static const int PWM_DAT1 = 5;
	#ifdef English_dox
		//! RNG2 - PWM Control shift value
	#endif
	static const int PWM_RNG2 = 8;
	#ifdef English_dox
		//! DAT2 - PWM Control shift value
	#endif
	static const int PWM_DAT2 = 9;
	#ifdef English_dox
		//! CLK_CNTL - PWM Control shift value
	#endif
	static const int PWMCLK_CNTL= 40;
	#ifdef English_dox
		//! CLK_DIV - PWM Control shift value
	#endif
	static const int PWMCLK_DIV = 41;

	#ifdef English_dox
		//! This specifies the default block size of real physical memory
	#endif
	static const int BLOCK_SIZE = 4096;


	#ifdef English_dox
	//! Function to map physical memory
	/**
	 * \param u_l_base_address Offset to base address
	 * \return Register address map
	 */
	#endif
	volatile unsigned *MapRegisterAddres(unsigned long u_l_base_address);

	#ifdef English_dox
		//! This function sets GPIO18 to the alternate function 5 (ALT5) and GPIO13 to ALT0 to enable the pin to output the PWM waveforms generated by PWM1 and PWM2
	#endif
	#ifdef Turkish_dox
	//! GPIO18'i ALT5 ve GPIO13'u ALT0 alternatif fonksiyonlara PWM sinyali çıktısını üretebilmek için ayarlamaktadır.
	#endif
	void ConfigPWMPin();


	#ifdef English_dox
	//! This function is responsible for the global configuration and initialization
	//! of the the PWM1 and PWM2 peripheral
	#endif
	#ifdef Turkish_dox
	//! PWM1 ve PWM2 için global ayarlama ve ilklendirme işlemlerinden sorumlu fonksiyondur.
	#endif
	void ConfigPWM();

	#ifdef Turkish_dox
	//! PWM frequency
	#endif
	double d_frequency;
	#ifdef Turkish_dox
	//! PWM duty cycle (%)
	#endif
	double d_dutyCycle;
	#ifdef Turkish_dox
	//! PWM resolution
	#endif
	unsigned int u_i_counts;
	#ifdef Turkish_dox
	//! PWM divisor value
	#endif
	unsigned int u_i_divisor;
	#ifdef Turkish_dox
	//! PWM mode
	#endif
	int i_mode;
	#ifdef Turkish_dox
	//! pointers to the memory mapped sections of our process memory
	#endif
	volatile unsigned *p_v_u_clk, *p_v_u_pwm, *p_v_u_gpio;
};

#endif /* INCLUDE_IMPWM_H_ */
