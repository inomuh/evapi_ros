#ifndef INCLUDE_EVAROBOT_CONTROLLER_H_
#define INCLUDE_EVAROBOT_CONTROLLER_H_

#include "ros/ros.h"

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"

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

float g_f_left_measured = 0.0;
float g_f_right_measured = 0.0;

double g_d_wheel_separation;


using namespace std;


class PIDController
{
public:
		PIDController();
		PIDController(double d_proportional_constant, 
					  double d_integral_constant, 
					  double d_derivative_constant);
		
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
	
	float max_vel;
	
	ros::Time read_time;
	ros::Duration dur_time;
	
};

#endif
