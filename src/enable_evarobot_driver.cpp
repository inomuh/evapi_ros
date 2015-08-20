/**
* @file evarobot_driver.cpp
* @Author Me (mehmet.akcakoca@inovasyonmuhendislik.com)
* @date April 2, 2015
*
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "IMPWM.h"
#include "IMGPIO.h"

#include <stdio.h> 
#include <math.h> 

using namespace std;

class IMDRIVER
{
public:
	IMDRIVER(float f_max_lin_vel, 
			 float f_max_ang_vel, 
			 float f_wheel_separation, 
			 float f_wheel_diameter,
			 double d_frequency, 
			 unsigned int u_i_counts, 
			 double d_duty, 
			 int i_mode,
			 IMGPIO * gpio_ins);
			 
	~IMDRIVER()
	{
		printf("driver kapatiliyor\n");
		delete pwm;
	}
			 
	void CallbackVel(const geometry_msgs::Twist::ConstPtr & msg);
	void CallbackWheelVel(const geometry_msgs::Twist::ConstPtr & msg);
	
private:
	IMPWM * pwm;
	IMGPIO * gpio_ins;

	
	float f_max_lin_vel; 
    float f_max_ang_vel; 
    float f_wheel_separation; 
    float f_wheel_diameter;

	double d_frequency;
	double d_duty;

	unsigned int u_i_counts;
	int i_mode;    
	int i_const_count;
};

IMDRIVER::IMDRIVER(float f_max_lin_vel, 
				   float f_max_ang_vel, 
				   float f_wheel_separation, 
				   float f_wheel_diameter,
				   double d_frequency, 
				   unsigned int u_i_counts, 
				   double d_duty, 
				   int i_mode,
				   IMGPIO * gpio_ins)
{
	this->f_max_lin_vel = f_max_lin_vel; 
    this->f_max_ang_vel = f_max_ang_vel; 
    this->f_wheel_separation = f_wheel_separation; 
    this->f_wheel_diameter = f_wheel_diameter;

	this->i_const_count =  (u_i_counts - 1) / this->f_max_lin_vel;
		
	this->pwm = new IMPWM(d_frequency, u_i_counts, d_duty, i_mode);
	
	this->gpio_ins = gpio_ins;
	
	for(int i = 0; i < 4; i++)
		this->gpio_ins[i].SetPinDirection(IMGPIO::OUTPUT);
	
}

void IMDRIVER::CallbackVel(const geometry_msgs::Twist::ConstPtr & msg)
{
	float f_linear_vel = msg->linear.x;
	float f_angular_vel = msg->angular.z;

	float f_left_wheel_velocity = ((2 * f_linear_vel) - f_angular_vel * f_wheel_separation) / 2; // m/s
	float f_right_wheel_velocity = ((2 * f_linear_vel) + f_angular_vel * f_wheel_separation) / 2; // m/s

	if(f_left_wheel_velocity < 0)
	{
		this->gpio_ins[0].SetPinValue(IMGPIO::LOW);
		this->gpio_ins[1].SetPinValue(IMGPIO::HIGH);
	}
	else if(f_left_wheel_velocity > 0)
	{
		this->gpio_ins[0].SetPinValue(IMGPIO::HIGH);
		this->gpio_ins[1].SetPinValue(IMGPIO::LOW);
	}
	else
	{
		this->gpio_ins[0].SetPinValue(IMGPIO::LOW);
		this->gpio_ins[1].SetPinValue(IMGPIO::LOW);
	}
	
	if(f_right_wheel_velocity < 0)
	{
		this->gpio_ins[2].SetPinValue(IMGPIO::LOW);
		this->gpio_ins[3].SetPinValue(IMGPIO::HIGH);
	}
	else if(f_right_wheel_velocity > 0)
	{
		this->gpio_ins[2].SetPinValue(IMGPIO::HIGH);
		this->gpio_ins[3].SetPinValue(IMGPIO::LOW);
	}
	else
	{
		this->gpio_ins[2].SetPinValue(IMGPIO::LOW);
		this->gpio_ins[3].SetPinValue(IMGPIO::LOW);
	}
	
	int i_left_wheel_duty = int(fabs(f_left_wheel_velocity) * this->i_const_count);
	int i_right_wheel_duty = int(fabs(f_right_wheel_velocity) * this->i_const_count); 

	i_left_wheel_duty = i_left_wheel_duty>=u_i_counts ? u_i_counts - 1:i_left_wheel_duty;
	
	i_right_wheel_duty = i_right_wheel_duty>=u_i_counts ? u_i_counts - 1:i_right_wheel_duty;

	cout << "left_duty: " << i_left_wheel_duty << " right duty: " << i_right_wheel_duty << endl;

	if(i_left_wheel_duty != 0)
		pwm->SetDutyCycleCount(i_left_wheel_duty, 0);
//	pwm->SetDutyCycleCount(this->i_const_count - 1, 0);
//	cout << this->i_const_count << endl;
	
	if(i_right_wheel_duty != 0)
		pwm->SetDutyCycleCount(i_right_wheel_duty, 1);
//	pwm->SetDutyCycleCount(this->i_const_count - 1, 1);
}



void IMDRIVER::CallbackWheelVel(const geometry_msgs::Twist::ConstPtr & msg)
{
	float f_left_wheel_velocity = msg->linear.x;
	float f_right_wheel_velocity = msg->linear.y;

	if(f_left_wheel_velocity < 0)
	{
		this->gpio_ins[0].SetPinValue(IMGPIO::LOW);
		this->gpio_ins[1].SetPinValue(IMGPIO::HIGH);
	}
	else if(f_left_wheel_velocity > 0)
	{
		this->gpio_ins[0].SetPinValue(IMGPIO::HIGH);
		this->gpio_ins[1].SetPinValue(IMGPIO::LOW);
	}
	else
	{
		this->gpio_ins[0].SetPinValue(IMGPIO::LOW);
		this->gpio_ins[1].SetPinValue(IMGPIO::LOW);
	}
	
	if(f_right_wheel_velocity < 0)
	{
		this->gpio_ins[2].SetPinValue(IMGPIO::LOW);
		this->gpio_ins[3].SetPinValue(IMGPIO::HIGH);
	}
	else if(f_right_wheel_velocity > 0)
	{
		this->gpio_ins[2].SetPinValue(IMGPIO::HIGH);
		this->gpio_ins[3].SetPinValue(IMGPIO::LOW);
	}
	else
	{
		this->gpio_ins[2].SetPinValue(IMGPIO::LOW);
		this->gpio_ins[3].SetPinValue(IMGPIO::LOW);
	}
	
	int i_left_wheel_duty = int(fabs(f_left_wheel_velocity) * this->i_const_count);
	int i_right_wheel_duty = int(fabs(f_right_wheel_velocity) * this->i_const_count); 

	i_left_wheel_duty = i_left_wheel_duty>=u_i_counts ? u_i_counts - 1:i_left_wheel_duty;
	
	i_right_wheel_duty = i_right_wheel_duty>=u_i_counts ? u_i_counts - 1:i_right_wheel_duty;

	cout << "left_duty: " << i_left_wheel_duty << " right duty: " << i_right_wheel_duty << endl;


	if(i_left_wheel_duty != 0)
		pwm->SetDutyCycleCount(i_left_wheel_duty, 0);
//	pwm->SetDutyCycleCount(this->i_const_count - 1, 0);
//	cout << this->i_const_count << endl;
	
	if(i_right_wheel_duty != 0)
		pwm->SetDutyCycleCount(i_right_wheel_duty, 1);
//	pwm->SetDutyCycleCount(this->i_const_count - 1, 1);
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "evarobot_driver");
  ros::NodeHandle n;
  
  string str_topic;
  
  double d_max_lin_vel;
  double d_max_ang_vel;
  
  double d_wheel_diameter;
  double d_wheel_separation;
  
  double d_frequency;
  double d_duty;
  
  int i_counts;
  
  int i_mode;

  
  n.param<string>("evarobot_driver/commandTopic", str_topic, "cntr_wheel_vel");
  
  if(!n.getParam("evarobot_driver/maxLinearVel", d_max_lin_vel))
  {
	  ROS_ERROR("Failed to get param 'maxLinearVel'");
  }

  if(!n.getParam("evarobot_driver/maxAngularVel", d_max_ang_vel))
  {
	  ROS_ERROR("Failed to get param 'maxAngularVel'");
  }  

  if(!n.getParam("evarobot_driver/wheelSeparation", d_wheel_separation))
  {
	  ROS_ERROR("Failed to get param 'wheelSeparation'");
  } 

  if(!n.getParam("evarobot_driver/wheelDiameter", d_wheel_diameter))
  {
	  ROS_ERROR("Failed to get param 'wheelDiameter'");
  }     
  
  if(!n.getParam("evarobot_driver/pwmFrequency", d_frequency))
  {
	  ROS_ERROR("Failed to get param 'pwmFrequency'");
  }  
  
  if(!n.getParam("evarobot_driver/pwmDuty", d_duty))
  {
	  ROS_ERROR("Failed to get param 'pwmDuty'");
  } 
  
  if(!n.getParam("evarobot_driver/pwmCounts", i_counts))
  {
	  ROS_ERROR("Failed to get param 'pwmCounts'");
  } 
  
  if(!n.getParam("evarobot_driver/pwmMode", i_mode))
  {
	  ROS_ERROR("Failed to get param 'pwmMode'");
  } 
  
  // PWMMODE = 1; --- MSMODE = 2
  if(i_mode != IMPWM::PWMMODE && i_mode != IMPWM::MSMODE)
  {
	  ROS_ERROR("Undefined PWM Mode.");
  }
  
  
  
  IMGPIO gpio[4] = {IMGPIO(IMGPIO::GPIO4), IMGPIO(IMGPIO::GPIO12), IMGPIO(IMGPIO::GPIO11), IMGPIO(IMGPIO::GPIO13)};
     
  IMDRIVER imdriver((float)d_max_lin_vel, (float)d_max_ang_vel, (float)d_wheel_separation, (float)d_wheel_diameter,
					d_frequency, i_counts, d_duty, i_mode, gpio);
  
  //ros::Subscriber sub = n.subscribe(str_topic.c_str(), 2, &IMDRIVER::CallbackVel, &imdriver);
  ros::Subscriber sub = n.subscribe(str_topic.c_str(), 2, &IMDRIVER::CallbackWheelVel, &imdriver);
  
  
  if(sub.getNumPublishers() < 1)
  {
	  ROS_WARN("No appropriate subscriber.");
  }

  ros::spin();

 

  return 0;
}
