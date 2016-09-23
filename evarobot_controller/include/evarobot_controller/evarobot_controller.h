#ifdef English_dox
/**
 * \file   evarobot_controller.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  2 DOF pid controller.
 * \details
 */
#endif

#ifndef INCLUDE_EVAROBOT_CONTROLLER_H_
#define INCLUDE_EVAROBOT_CONTROLLER_H_



#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <stdio.h>
#include <stdlib.h>

#include <ros/console.h>
#include <realtime_tools/realtime_publisher.h>

#include <string>
#include <sstream>

#include "ros/ros.h"


#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"

#include "std_srvs/Empty.h"
#include "im_msgs/WheelVel.h"

#include "nav_msgs/Odometry.h"
#include <dynamic_reconfigure/server.h>
#include <evarobot_controller/ParamsConfig.h>
#include <ErrorCodes.h>

using namespace std;

#ifdef English_dox
//! Desired left and right wheel speeds.
#endif
float g_f_left_desired = 0.0;
float g_f_right_desired = 0.0;

#ifdef English_dox
//! Desired linear and angular speeds.
#endif
float g_f_linear_desired = 0.0;
float g_f_angular_desired = 0.0;

#ifdef English_dox
//! Measured left and right wheel speeds.
#endif
float g_f_left_measured = 0.0;
float g_f_right_measured = 0.0;

#ifdef English_dox
//! Measured linear and angular speeds.
#endif
float g_f_linear_measured = 0.0;
float g_f_angular_measured = 0.0;

#ifdef English_dox
//! Separation between left and right wheels.
#endif
double g_d_wheel_separation;

#ifdef English_dox
//! Controls new parameters are set or not.
#endif
bool b_is_received_params = false;

#ifdef English_dox
//! Controls reset controller callback is called or not.
#endif
bool b_reset_controller = false;

#ifdef English_dox
//! Controller pid parameters.
#endif
double g_d_p_1;
double g_d_i_1;
double g_d_d_1;
double g_d_w_1;
double g_d_p_2;
double g_d_i_2;
double g_d_d_2;
double g_d_w_2;

#ifdef English_dox
//! Time difference between current and previous data.
#endif
double g_d_dt = 0.0;

#ifdef English_dox
//! Controller Type: 0->Decoupled, 1->Coupled
#endif
int g_i_controller_type;

#ifdef English_dox
//! If an error occurs, shows its code number.
#endif
int i_error_code = 0;

#ifdef English_dox
//! Controller for PID parameters.
#endif
class PIDController {
public:
	#ifdef English_dox
	//! Default Constructor
	#endif

	PIDController();
	#ifdef English_dox
	//! Constructor
	/**
	 * \param d_proportional_constant Proportional constant variable.
	 * \param d_integral_constant Integral constant variable.
	 * \param d_derivative_constant Derivative constant variable.
	 * \param d_windup_constant Windup constant variable.
	 * \param _d_max_vel Maximum velocity variable.
	 * \param _str_name Name for pid controller.
	 */
	#endif
	PIDController(double d_proportional_constant,
				double d_integral_constant,
				double d_derivative_constant,
				double d_windup_constant,
				double _d_max_vel,
				string _str_name);

	#ifdef English_dox
	//! Updates pid parameters.
	/**
	 * \param d_proportional_constant Proportional constant variable.
	 * \param d_integral_constant Integral constant variable.
	 * \param d_derivative_constant Derivative constant variable.
	 * \param d_windup_constant Windup constant variable.
	 */
	#endif
	void UpdateParams(double d_proportional_constant,
					double d_integral_constant,
					double d_derivative_constant,
					double d_windup_constant);

	#ifdef English_dox
	//! Handles errors and publish them to diagnostics topic.
	/**
	 * \param &stat Diagnostics updater variable. Errors are added to this variable.
	 */
	#endif
	void ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

	#ifdef English_dox
	//! Destructor.
	#endif
	~PIDController();

	#ifdef English_dox
	//! Runs controller to reach desired value.
	/**
	 * \param f_desired Desired value.
	 * \param f_measured Measured value.
	 * \return value should be applied.
	 */
	#endif
	float RunController(float f_desired, float f_measured);

	#ifdef English_dox
	//! Resets pid parameters to zero.
	#endif
	void Reset();

private:
	#ifdef English_dox
	//! PID error parameters.
	#endif
	double d_integral_error;
	double d_derivative_error;
	double d_proportional_error;
	double d_windup_error;

	#ifdef English_dox
	//! Previous proportional error.
	#endif
	double d_pre_error;

	#ifdef English_dox
	//! PID parameter constants.
	#endif
	double d_integral_constant;
	double d_derivative_constant;
	double d_proportional_constant;
	double d_windup_constant;

	#ifdef English_dox
	//! Previous filtered data.
	#endif
	float f_filtered_data_pre;

	#ifdef English_dox
	//! Name for controller.
	#endif
	string str_name;

	#ifdef English_dox
	//! Maximum velocity. Velocity cannot be higher than this value.
	#endif
	double d_max_vel;

	#ifdef English_dox
	//! Previous run time of RunController function.
	#endif
	ros::Time read_time;

	#ifdef English_dox
	//! Difference between current time and previous time.
	#endif
	ros::Duration dur_time;
};

#endif  // INCLUDE_EVAROBOT_CONTROLLER_H_
