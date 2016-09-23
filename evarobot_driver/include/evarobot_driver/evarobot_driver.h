#ifdef English_dox
/**
 * \file   evarobot_driver.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Apr, 2015
 * \brief  Generates PWM to drive motors with desired velocities.
 * \details
 */
#endif

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

#include <boost/shared_ptr.hpp>

using namespace std;

#ifdef English_dox
//! Controls new parameters are set or not.
#endif
bool b_is_received_params = false;

#ifdef English_dox
//! Maximum linear velocity. Linear velocity cannot be higher than this value.
#endif
double g_d_max_lin;

#ifdef English_dox
//! Maximum angular velocity. Angular velocity cannot be higher than this value.
#endif
double g_d_max_ang;

#ifdef English_dox
//! Separation between left and right wheels.
#endif
double g_d_wheel_separation;

#ifdef English_dox
//! Diameter of the wheels.
#endif
double g_d_wheel_diameter;

#ifdef English_dox
//! If an error occurs, shows its code number.
#endif
int i_error_code;

#ifdef English_dox
//! Driver class for evarobot.
#endif
class IMDRIVER
{
public:
	#ifdef English_dox
	//! Constructor
	/**
	 * \param d_limit_voltage Maximum voltage variable.
	 * \param f_max_lin_vel Maximum linear velocity variable.
	 * \param f_max_ang_vel Maximum angular velocity variable.
	 * \param f_wheel_separation Separation between left and right wheels.
	 * \param f_wheel_diameter Diameter of wheels.
	 * \param d_frequency PWM frequency.
	 * \param u_i_counts PWM resolution.
	 * \param d_duty PWN duty cycle.
	 * \param i_mode PWM mode (IMPWM::MSMODE or IMPWM::PWMMODE).
	 * \param *m1_in INPUTA value for motor driver vnh5019.
	 * \param *m2_in INPUTB value for motor driver vnh5019.
	 * \param *m1_en ENABLEA value for motor driver vnh5019.
	 * \param *m2_en ENABLEB value for motor driver vnh5019.
	 * \param *adc ADC class reference.
	 * \param &client ServiceClient for evarobot_rgb/SetRGB service.
	 * \param _timeout Time limit between two consecutive data from cntr_wheel_vel topic.
	 */
	#endif
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

	#ifdef English_dox
	//! Destructor.
	#endif
	~IMDRIVER();

	#ifdef English_dox
	//! Callback funstion for cntr_wheel_vel topic.
	/**
	 * \param &msg Message from cntr_wheel_vel topic in type of im_msgs::WheelVel::ConstPtr.
	 */
	#endif
	void CallbackWheelVel(const im_msgs::WheelVel::ConstPtr & msg);

	#ifdef English_dox
	//! Handles errors and publish them to diagnostics topic.
	/**
	 * \param &stat Diagnostics updater variable. Errors are added to this variable.
	 */
	#endif
	void ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

	#ifdef English_dox
	//! Returns left and right motor voltages.
	/**
	 * \return left and right motor voltages in type of im_msgs::Voltage.
	 */
	#endif
	im_msgs::Voltage GetMotorVoltage() const;

	#ifdef English_dox
	//! Controls motor current is in limits or not.
	/**
	 * \return Motor current is in limits or not.
	 */
	#endif
	bool CheckMotorCurrent();

	#ifdef English_dox
	//! Controls time difference between two consecutive data from cntr_wheel_vel topic.
	/**
	 * \return time difference between two consecutive data from cntr_wheel_vel topic is overflow limit or not.
	 */
	#endif
	bool CheckTimeout();
	
	#ifdef English_dox
	//! Updates driver parameters.
	#endif
	void UpdateParams();

	#ifdef English_dox
	//! Applies input velocities to motors if velocities are in limits.
	/**
	 * \param f_left_wheel_velocity Left wheel velocity.
	 * \param f_right_wheel_velocity Right wheel velocity.
	 */
	#endif
	void ApplyVel(float f_left_wheel_velocity, float f_right_wheel_velocity);

	#ifdef English_dox
	//! Enables motor driver pins.
	#endif
	void Enable();

	#ifdef English_dox
	//! Disables motor driver pins.
	#endif
	void Disable();
	
	#ifdef English_dox
	//! Controls are there any error in related GPIO pins.
	/**
	 * \retval <0 if there is an error
	 * \retval >=0 no error
	 */
	#endif
	int CheckError();
	
private:
	#ifdef English_dox
	//! PWM class reference.
	#endif
	IMPWM * pwm;

	#ifdef English_dox
	//! INPUTA value for motor driver vnh5019.
	#endif
	boost::shared_ptr< IMGPIO > * m1_in;

	#ifdef English_dox
	//! INPUTB value for motor driver vnh5019.
	#endif
	boost::shared_ptr< IMGPIO > * m2_in;

	#ifdef English_dox
	//! ENABLEA value for motor driver vnh5019.
	#endif
	IMGPIO * m1_en;

	#ifdef English_dox
	//! ENABLEB value for motor driver vnh5019.
	#endif
	IMGPIO * m2_en;
	
	#ifdef English_dox
	//! ADC class reference.
	#endif
	IMADC * adc;

	#ifdef English_dox
	//! ServiceClient for evarobot_rgb/SetRGB service.
	#endif
	ros::ServiceClient client;
	
	#ifdef English_dox
	//! Maximum linear velocity variable.
	#endif
	float f_max_lin_vel; 

	#ifdef English_dox
	//! Maximum angular velocity variable.
	#endif
	float f_max_ang_vel; 

	#ifdef English_dox
	//! Separation between left and right wheels.
	#endif
	float f_wheel_separation; 

	#ifdef English_dox
	//! Diameter of wheels.
	#endif
	float f_wheel_diameter;

	#ifdef English_dox
	//! Left motor voltage.
	#endif
	float f_left_motor_voltage;

	#ifdef English_dox
	//! Right motor voltage.
	#endif
	float f_right_motor_voltage;

	#ifdef English_dox
	//! PWM frequency.
	#endif
	double d_frequency;

	#ifdef English_dox
	//! PWM duty cycle.
	#endif
	double d_duty;

	#ifdef English_dox
	//! Voltage limit for both left and right wheels.
	#endif
	double d_limit_voltage;
	
	#ifdef English_dox
	//! Time limit between two consecutive data from cntr_wheel_vel topic.
	#endif
	double d_timeout;
	
	#ifdef English_dox
	//! Are there any limit overflow in left or right wheels.
	#endif
	bool b_motor_error;

	#ifdef English_dox
	//! Are there any limit overflow in left wheel.
	#endif
	bool b_left_motor_error;

	#ifdef English_dox
	//! Are there any limit overflow in right wheel.
	#endif
	bool b_right_motor_error;

	#ifdef English_dox
	//! There is a timeout error or not.
	#endif
	bool b_timeout_err;
	
	#ifdef English_dox
	//! PWM resolution.
	#endif
	unsigned int u_i_counts;

	#ifdef English_dox
	//! PWM mode (IMPWM::MSMODE or IMPWM::PWMMODE)
	#endif
	int i_mode;
	
	#ifdef English_dox
	//! Last time data taken from cntr_wheel_vel topic.
	#endif
	ros::Time curr_vel_time;

	int rightPWM, leftPWM;
};

#endif

