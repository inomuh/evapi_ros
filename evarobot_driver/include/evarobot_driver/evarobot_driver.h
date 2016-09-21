//! Bu sınıfa ait dokümantasyon evapi_ros 85rpm altındaki dokümantasyon ile aynıdır.

#ifndef INCLUDE_EVAROBOT_DRIVER_H_
#define INCLUDE_EVAROBOT_DRIVER_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


#include "IMPWM.h"
#include "IMGPIO.h"
#include "IMADC.h"

#include <stdio.h> 
#include <math.h> 
#include <sstream>

#include <dynamic_reconfigure/server.h>
#include <evarobot_driver/ParamsConfig.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include "im_msgs/SetRGB.h"
#include "im_msgs/Voltage.h"
#include "im_msgs/WheelVel.h"

#include <ros/console.h>
#include "ErrorCodes.h"
#include "im_msgs/EKBTest.h"

#include <boost/shared_ptr.hpp>

using namespace std;


bool b_is_received_params = false;

double g_d_max_lin;
double g_d_max_ang;
double g_d_wheel_separation;
double g_d_wheel_diameter;

class IMDRIVER
{
public:
	IMDRIVER(double d_limit_voltage,
			 float f_max_lin_vel, 
			 float f_max_ang_vel, 
			 float f_wheel_separation, 
			 float f_wheel_diameter,
			 double d_frequency, 
			 unsigned int u_i_counts, 
			 double d_duty, 
			 int i_mode,
			 boost::shared_ptr< IMGPIO > * m1_in, 
			 boost::shared_ptr< IMGPIO > * m2_in,
			 IMGPIO * m1_en,
			 IMGPIO * m2_en,
			 IMADC * adc,
			 ros::ServiceClient & client,
			 double _timeout);
			 
	~IMDRIVER();
			 
	void CallbackWheelVel(const im_msgs::WheelVel::ConstPtr & msg);
	void ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
	im_msgs::Voltage GetMotorVoltage() const;

	bool CheckMotorCurrent();
	bool CheckTimeout();
	
	void UpdateParams();
	void ApplyVel(float f_left_wheel_velocity, float f_right_wheel_velocity);
	void Enable();
	void Disable();
	
	int CheckError();
	
private:
	IMPWM * pwm;
	boost::shared_ptr< IMGPIO > * m1_in;
	boost::shared_ptr< IMGPIO > * m2_in;
	IMGPIO * m1_en;
	IMGPIO * m2_en;
	
	IMADC * adc;
	ros::ServiceClient client;
	
	float f_max_lin_vel; 
	float f_max_ang_vel; 
	float f_wheel_separation; 
	float f_wheel_diameter;

	float f_left_motor_voltage;
	float f_right_motor_voltage;

	double d_frequency;
	double d_duty;
	double d_limit_voltage;
	
	double d_timeout;
	
	bool b_motor_error, b_left_motor_error, b_right_motor_error;
	bool b_timeout_err;
	
	unsigned int u_i_counts;
	int i_mode;    
	int i_const_count;
	
	ros::Time curr_vel_time;
};

#endif

