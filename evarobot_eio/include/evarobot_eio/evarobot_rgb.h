#ifdef English_dox
/**
 * \file   evarobot_rgb.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Controls rgb leds over i2c.
 * \details
 */
#endif

#ifndef INCLUDE_EVAROBOT_RGB_H_
#define INCLUDE_EVAROBOT_RGB_H_

#include "ros/ros.h"
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "IMEIO.h"
#include <vector>
#include <sstream>

#include "im_msgs/SetRGB.h"


#include <ros/console.h>
#include "ErrorCodes.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#ifdef English_dox
//! Semaphore name to prevent deadlock.
#endif
char SEM_NAME[]= "i2c";

#ifdef English_dox
//! If an error occurs, shows its code number.
#endif
int i_error_code;

using namespace std;

#ifdef English_dox
//! Controls RGB leds.
#endif
class IMRGB
{
public:
	#ifdef English_dox
	//! Constructor
	/**
	 * \param *eio IMEIO class reference.
	 */
	#endif
	IMRGB(IMEIO * eio);

	#ifdef English_dox
	//! Destructor
	#endif
	~IMRGB();
	#ifdef English_dox
	//! Turns on leds red and blue respectively and repetitiously.
	#endif
	void RunAutonomousMode();

	#ifdef English_dox
	//! Turns on leds blue in flip flip mode.
	#endif
	void RunTeleopMode();

	#ifdef English_dox
	//! Turns on leds turquoise in flip flip mode.
	#endif
	void RunWanderMode();

	#ifdef English_dox
	//! Turns on leds i_color in flash mode.
	#endif
	void RunManualMode();

	#ifdef English_dox
	//! Turns on leds green in flash mode.
	#endif
	void RunChargingMode();

	#ifdef English_dox
	//! Turns on leds green in continuous mode.
	#endif
	void RunChargedMode();

	#ifdef English_dox
	//! Turns on leds red in flash mode.
	#endif
	void RunErrorMode();

	#ifdef English_dox
	//! Turns on leds yellow in flash mode.
	#endif
	void RunLowBatteryMode();

	#ifdef English_dox
	//! Turns on leds in all colors.
	/**
	 * Turns on leds red, green, yellow, blue, pink, turquoise, white, turquoise, ping,
	 * blue, yellow, green, red respectively. Then, turns off leds.
	 */
	#endif
	void RunOpeningMode();

	#ifdef English_dox
	//! Turns off leds.
	#endif
	void TurnLedOff();
	
	#ifdef English_dox
	//! Controls new request is taken or not.
	/**
	 * \return new request is taken or not.
	 */
	#endif
	bool isNewRequest() const;

	#ifdef English_dox
	//! Returns frequency.
	/**
	 * \return frequency.
	 */
	#endif
	float GetFrequency() const;

	#ifdef English_dox
	//! Returns mode number.
	/**
	 * const int IMRGB::MANUAL      = 0;
	 * const int IMRGB::ON          = 1;
	 * const int IMRGB::AUTONOMOUS  = 2;
	 * const int IMRGB::TELEOP      = 3;
	 * const int IMRGB::WANDER      = 4;
	 * const int IMRGB::LOW_BATTERY = 5;
	 * const int IMRGB::CHARGING    = 6;
	 * const int IMRGB::CHARGED     = 7;
	 * const int IMRGB::ERROR       = 8;
	 * \return mode number.
	 */
	#endif
	int GetMode() const;

	#ifdef English_dox
	//! Returns how many times current mode will repeat.
	/**
	 * \return how many times current mode will repeat
	 */
	#endif
	int GetTimesValue() const;
	
	#ifdef English_dox
	//! Callback function for evarobot_rgb/SetRGB service.
	/**
	 * \param &request Input data includes color, count, mode and frequency.
	 * \param &response Return value 1 if there is no error, 0 if any error occurs.
	 * \retval false If there are any errors.
	 * \retval true If there is no error.
	 */
	#endif
	bool callbackSetRGB(im_msgs::SetRGB::Request& request, im_msgs::SetRGB::Response& response);
	
	#ifdef English_dox
	//! Color codes.
	#endif
	static const int LEDOFF;
	static const int RED;
	static const int GREEN;
	static const int YELLOW;
	static const int BLUE;
	static const int PINK;
	static const int TURQUOISE;
	static const int WHITE;

	#ifdef English_dox
	//! Mode codes.
	#endif
	static const int MANUAL;
	static const int ON;
	static const int AUTONOMOUS;
	static const int TELEOP;
	static const int WANDER;
	static const int LOW_BATTERY;
	static const int CHARGING;
	static const int CHARGED;
	static const int ERROR;
		
private:
	#ifdef English_dox
	//! IMEIO class reference to control RGB leds.
	#endif
	IMEIO * eio;
	
	#ifdef English_dox
	//! New request is taken or not.
	#endif
	bool b_new_data;

	#ifdef English_dox
	//! Current run mode.
	#endif
	int i_mode;

	#ifdef English_dox
	//! Run frequency.
	#endif
	float f_frequency;

	#ifdef English_dox
	//! How many times current mode will repeat.
	#endif
	int i_times;

	#ifdef English_dox
	//! Current color of RGB leds.
	#endif
	int i_color;
	
	#ifdef English_dox
	//! Turns on leds in input color.
	/**
	 * \param i_color Color code to turn on leds.
	 */
	#endif
	void TurnLedOn(int i_color);
	
	#ifdef English_dox
	//! Turns on leds in input color and then turns off leds repetitiously.
	/**
	 * \param i_color Color code to turn on leds.
	 */
	#endif
	void FlipFlip(int i_color);

	#ifdef English_dox
	//! Turns on leds in input color and then turns off leds.
	/**
	 * \param i_color Color code to turn on leds.
	 * \param f_frequency Flash frequency
	 */
	#endif
	void Flash(int i_color, float f_frequency);

	#ifdef English_dox
	//! Decreases i_times by 1.
	/**
	 * \retval false There is no remaining count.
	 * \retval true There is remaining count.
	 */
	#endif
	bool DecreaseTimesValue();
};

#endif
