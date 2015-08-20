#ifdef English_dox
/**
 * \file   IMADC.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Reads analog inputs through MCP3208 on evarobot sensorboard.
 * \details
 * There are two types reading mode which are single-ended and differantial.
 * Reference voltage in single-ended mode is 5V.
 * Max. of adc bits is 16 bits.
 * It is used for reading infrared sensors on evarobot.
 */
#endif
#ifdef Turkish_dox
/**
 * \file   IMADC.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Sensör Kartında bulunan analog girdileri okumaktadır.
 * \details
 * MCP3208 ADC üzerinden analog bilgileri okuyan sınıftır.
 * Sensör Kartında bulunan analog girdileri okumaktadır.
 * Maksimum 8 kanal okuyabilmektedir.
 * Referans değeri 5V'tur.
 * Pinler teker teker veya differansiyel olarak okunabilmektedir.
 * Entegre ile SPI üzerinden haberleşilmektedir.
 * Evarobot üzerindeki kızılötesi sensörleri okuyan sınıftır.
 */
#endif


#ifndef INCLUDE_IMADC_H_
#define INCLUDE_IMADC_H_

#include "IMSPI.h"

class IMADC
{
public:

	#ifdef English_dox
	//! Constructor
	/**
	 * \param p_imspi pointer of IMSPI class.
	 * \param i_adc_bits number of adc bits. It depends ADC hardware.
	 */
	#endif

	#ifdef Turkish_dox
	//! Constructor
	/**
	 * \param p_imspi IMSPI sınıfı işaretçisi
	 * \param i_adc_bits ADC bit sayısı. ADC entegresine bağlı bir değerdir.
	 */
	#endif

	IMADC(IMSPI * p_imspi, int i_adc_bits=12);

	#ifdef English_dox
	//! Reads single-ended Analog to Digital Converter Channel (MCP3208).
	/**
	 * \param i_channel_no is channel number to read in range of 0-7
	 * \return value of data on the channel
	 */
	#endif
	#ifdef Turkish_dox
	//! Pinleri 5V referans değerine göre okuyan fonksiyondur.
	/**
	 * Sensör Kartında bulunan Analog - Dijital Çevirici (MCP3208)
	 * entegresi kanallarını tek taraflı okumaktadır.
	 *
	 * \param i_channel_no 0-7 arasında girilen, okunacak kanal numarısıdır.
	 * \return kanal üzerinde okunan değerdir.
	 */
	#endif
	int ReadChannel(int i_channel_no);


	#ifdef English_dox
	//! Reads differential Analog to Digital Converter Channel (MCP3208).
	/**
	 * \param i_channel_plus is channel number to read in range of 0-7
	 * \param i_channel_negative
	 * \return value of data on the difference(i_channel_plus - i_channel_negative)
	 */
	#endif
	#ifdef Turkish_dox
	//! Pinleri diferansiyel olarak okuyan fonksiyondur.
	/**
	 * Sensör kartında bulunan analog giriş bölümündeki iki kanal arasındaki
	 * farkı okumak için kullanılmaktadır.
	 *
	 * \param i_channel_plus 0-7 arasında girilen, differansiyel olarak okunacak kanal numarasıdır.
	 * \param i_channel_negative 0-7 arasında girilen, referans olarak kullanılacak kanaldır.
	 * \return iki kanal arasındaki analog değerin dijital değeridir. (i_channel_plus - i_channel_negative)
	 */
	#endif
	int ReadChannel(int i_channel_plus, int i_channel_negative);

private:
	#ifdef English_dox
	//! Class IMSPI is used for communicating with MCP3208.
	#endif

	#ifdef Turkish_dox
	//! ADC entegresi ile haberleşebilmek için IMSPI sınıfı kullanılmaktadır.
	#endif
	IMSPI * p_imspi;

	#ifdef English_dox
	//! Masking variable.
	#endif

	#ifdef Turkish_dox
	//! Okunan değeri maskelemek için kullanılanılan değişkendir.
	#endif
	int i_mask;
};

#endif /* INCLUDE_IMADC_H_ */
