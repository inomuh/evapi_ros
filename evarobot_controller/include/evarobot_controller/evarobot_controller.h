#ifndef INCLUDE_EVAROBOT_CONTROLLER_H_
#define INCLUDE_EVAROBOT_CONTROLLER_H_

#include "ros/ros.h"
#include <realtime_tools/realtime_publisher.h>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include "std_srvs/Empty.h"
#include "im_msgs/WheelVel.h"

#include <stdio.h>
#include <stdlib.h>

#include <string>    
#include <sstream>

float g_f_left_desired = 0.0;
float g_f_right_desired = 0.0;

float g_f_pre_angular_desired = 0.0;
float g_f_pre_linear_desired = 0.0;

bool b_is_new_measured = false;
bool b_is_new_desired = false;

bool g_b_only_ang_vel = false;

float g_f_left_measured = 0.0;
float g_f_right_measured = 0.0;

double g_d_wheel_separation;
bool b_is_received_params = false;

bool b_reset_controller = false;

using namespace std;

double g_d_p_left;
double g_d_i_left;
double g_d_d_left;

double g_d_p_right;
double g_d_i_right;
double g_d_d_right;

double g_d_dt = 0;

class PIDController
{
public:
		PIDController();
		PIDController(double d_proportional_constant, 
									double d_integral_constant, 
									double d_derivative_constant,
									double _d_max_vel,
									string _str_name);
									
		void UpdateParams(double d_proportional_constant, 
											double d_integral_constant, 
											double d_derivative_constant);
											
		void ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
		
		~PIDController();
		
		float RunController(float f_desired, float f_measured);
		void Reset();
		
private:
	double d_integral_error;
	double d_derivative_error;
	double d_proportional_error;
	double d_pre_error;
	
	double d_integral_constant;
	double d_derivative_constant;
	double d_proportional_constant;
	
	string str_name;
	
	double d_max_vel;
	
	ros::Time read_time;
	ros::Duration dur_time;
	
};

#endif
