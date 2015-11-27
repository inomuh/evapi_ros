#ifndef INCLUDE_EVAROBOT_RGB_H_
#define INCLUDE_EVAROBOT_RGB_H_

#include "ros/ros.h"
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>		/* ioctl */
#include "IMEIO.h"
#include <vector>
#include <sstream>

#include "im_msgs/SetRGB.h"

char SEM_NAME[]= "i2c";

using namespace std;

class IMRGB
{
public:
	IMRGB(IMEIO * eio);
	~IMRGB();
	void RunAutonomousMode();
	void RunTeleopMode();
	void RunWanderMode();
	void RunManualMode();
	void RunChargingMode();
	void RunChargedMode();
	void RunErrorMode();
	void RunLowBatteryMode();
	void RunOpeningMode();
	void TurnLedOff();
	
	bool isNewRequest() const;
	float GetFrequency() const;
	int GetMode() const;
	int GetTimesValue() const;
	
	// Callback Fınction
	bool callbackSetRGB(im_msgs::SetRGB::Request& request, im_msgs::SetRGB::Response& response);
	
	// COLORS
	static const int LEDOFF;
	static const int RED;
	static const int GREEN;
	static const int YELLOW;
	static const int BLUE;
	static const int PINK;
	static const int TURQUOISE;
	static const int WHITE;
	
	// MODES
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
	IMEIO * eio;
	
	bool b_new_data;
	int i_mode;
	float f_frequency;
	int i_times;
	int i_color;
	
	void TurnLedOn(int i_color);
	
	void FlipFlip(int i_color);
	void Flash(int i_color, float f_frequency);
	bool DecreaseTimesValue();
};

#endif
