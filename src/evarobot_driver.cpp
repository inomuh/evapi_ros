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
			 IMGPIO * m1_in, 
			 IMGPIO * m2_in,
			 IMGPIO * m1_en,
			 IMGPIO * m2_en);
			 
	~IMDRIVER()
	{
		printf("driver kapatiliyor\n");
		this->Disable();
		delete pwm;
	}
			 
	void CallbackVel(const geometry_msgs::Twist::ConstPtr & msg);
	void CallbackWheelVel(const geometry_msgs::Twist::ConstPtr & msg);
	void Enable()
	{
		this->m1_en->SetPinDirection(IMGPIO::OUTPUT);	
		this->m2_en->SetPinDirection(IMGPIO::OUTPUT);
		
		this->m1_en->SetPinValue(IMGPIO::HIGH);
		this->m2_en->SetPinValue(IMGPIO::HIGH);
		
/*		this->m1_en->SetPinDirection(IMGPIO::INPUT);	
		this->m2_en->SetPinDirection(IMGPIO::INPUT);*/
	}
	
	void Disable()
	{
		this->m1_en->SetPinDirection(IMGPIO::OUTPUT);	
		this->m2_en->SetPinDirection(IMGPIO::OUTPUT);
		
		this->m1_en->SetPinValue(IMGPIO::LOW);
		this->m2_en->SetPinValue(IMGPIO::LOW);
		
	}
	
	int CheckError()
	{
		int i_ret = 0;
		
		string str_m1_data;
		string str_m2_data;
		
		this->m1_en->GetPinValue(str_m1_data);
		this->m2_en->GetPinValue(str_m2_data);
		
		if(str_m1_data == IMGPIO::LOW)
		{
			--i_ret;
		}
		
		if(str_m2_data == IMGPIO::LOW)
		{
			--i_ret;
		}
		
		return i_ret;
	}
	
private:
	IMPWM * pwm;
	IMGPIO * m1_in;
	IMGPIO * m2_in;
	IMGPIO * m1_en;
	IMGPIO * m2_en;

	
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
				   IMGPIO * m1_in, 
				   IMGPIO * m2_in,
				   IMGPIO * m1_en,
				   IMGPIO * m2_en)
{
	this->f_max_lin_vel = f_max_lin_vel; 
    this->f_max_ang_vel = f_max_ang_vel; 
    this->f_wheel_separation = f_wheel_separation; 
    this->f_wheel_diameter = f_wheel_diameter;

	this->u_i_counts = u_i_counts;
	this->i_const_count =  (u_i_counts - 1) / this->f_max_lin_vel;
		
	this->pwm = new IMPWM(d_frequency, u_i_counts, d_duty, i_mode);
	
	this->m1_in = m1_in;
	this->m2_in = m2_in;
	this->m1_en = m1_en;
	this->m2_en = m2_en;
	
	for(int i = 0; i < 2; i++)
		this->m1_in[i].SetPinDirection(IMGPIO::OUTPUT);
	
	for(int i = 0; i < 2; i++)
		this->m2_in[i].SetPinDirection(IMGPIO::OUTPUT);	
		
	// enable motors
	this->Enable();
	
}

void IMDRIVER::CallbackVel(const geometry_msgs::Twist::ConstPtr & msg)
{
	float f_linear_vel = msg->linear.x;
	float f_angular_vel = msg->angular.z;

	float f_left_wheel_velocity = ((2 * f_linear_vel) - f_angular_vel * f_wheel_separation) / 2; // m/s
	float f_right_wheel_velocity = ((2 * f_linear_vel) + f_angular_vel * f_wheel_separation) / 2; // m/s

	if(f_left_wheel_velocity < 0)
	{
		this->m1_in[0].SetPinValue(IMGPIO::LOW);
		this->m1_in[1].SetPinValue(IMGPIO::HIGH);
	}
	else if(f_left_wheel_velocity > 0)
	{
		this->m1_in[0].SetPinValue(IMGPIO::HIGH);
		this->m1_in[1].SetPinValue(IMGPIO::LOW);
	}
	else
	{
		this->m1_in[0].SetPinValue(IMGPIO::LOW);
		this->m1_in[1].SetPinValue(IMGPIO::LOW);
	}
	
	if(f_right_wheel_velocity < 0)
	{
		this->m2_in[0].SetPinValue(IMGPIO::LOW);
		this->m2_in[1].SetPinValue(IMGPIO::HIGH);
	}
	else if(f_right_wheel_velocity > 0)
	{
		this->m2_in[0].SetPinValue(IMGPIO::HIGH);
		this->m2_in[1].SetPinValue(IMGPIO::LOW);
	}
	else
	{
		this->m2_in[0].SetPinValue(IMGPIO::LOW);
		this->m2_in[1].SetPinValue(IMGPIO::LOW);
	}
	
	
	int i_left_wheel_duty = int(fabs(f_left_wheel_velocity) * this->i_const_count);
	int i_right_wheel_duty = int(fabs(f_right_wheel_velocity) * this->i_const_count); 

	i_left_wheel_duty = i_left_wheel_duty>=this->u_i_counts ? this->u_i_counts - 1:i_left_wheel_duty;
	
	i_right_wheel_duty = i_right_wheel_duty>=this->u_i_counts ? this->u_i_counts - 1:i_right_wheel_duty;

//	cout << "left_duty: " << i_left_wheel_duty << " right duty: " << i_right_wheel_duty << endl;

	if(i_left_wheel_duty != 0)
		pwm->SetDutyCycleCount(i_left_wheel_duty, 0);
	
	if(i_right_wheel_duty != 0)
		pwm->SetDutyCycleCount(i_right_wheel_duty, 1);
}



void IMDRIVER::CallbackWheelVel(const geometry_msgs::Twist::ConstPtr & msg)
{
	float f_left_wheel_velocity = msg->linear.x;
	float f_right_wheel_velocity = msg->linear.y;

	if(f_left_wheel_velocity < 0)
	{
		this->m1_in[0].SetPinValue(IMGPIO::LOW);
		this->m1_in[1].SetPinValue(IMGPIO::HIGH);
	}
	else if(f_left_wheel_velocity > 0)
	{
		this->m1_in[0].SetPinValue(IMGPIO::HIGH);
		this->m1_in[1].SetPinValue(IMGPIO::LOW);
	}
	else
	{
		this->m1_in[0].SetPinValue(IMGPIO::LOW);
		this->m1_in[1].SetPinValue(IMGPIO::LOW);
	}
	
	if(f_right_wheel_velocity < 0)
	{
		this->m2_in[0].SetPinValue(IMGPIO::HIGH);
		this->m2_in[1].SetPinValue(IMGPIO::LOW);
	}
	else if(f_right_wheel_velocity > 0)
	{
		this->m2_in[0].SetPinValue(IMGPIO::LOW);
		this->m2_in[1].SetPinValue(IMGPIO::HIGH);
	}
	else
	{
		this->m2_in[0].SetPinValue(IMGPIO::LOW);
		this->m2_in[1].SetPinValue(IMGPIO::LOW);
	}
	

	int i_left_wheel_duty = int(fabs(f_left_wheel_velocity) * this->i_const_count);
	int i_right_wheel_duty = int(fabs(f_right_wheel_velocity) * this->i_const_count); 

//	cout << "left_duty----: " << i_left_wheel_duty << " right duty-----: " << i_right_wheel_duty << endl;

	i_left_wheel_duty = i_left_wheel_duty>=this->u_i_counts ? this->u_i_counts - 1:i_left_wheel_duty;
	
	i_right_wheel_duty = i_right_wheel_duty>=this->u_i_counts ? this->u_i_counts - 1:i_right_wheel_duty;
	
//	cout << "u_i_counts " << this->u_i_counts << endl;

//	cout << "left_duty: " << i_left_wheel_duty << " right duty: " << i_right_wheel_duty << endl;


	if(i_left_wheel_duty != 0)
		pwm->SetDutyCycleCount(i_left_wheel_duty, 0);
	
	if(i_right_wheel_duty != 0)
		pwm->SetDutyCycleCount(i_right_wheel_duty, 1);

	if(this->CheckError() == -1)
	{
		ROS_ERROR("Failure at Left Motor.");
	}
	else if(this->CheckError() == -2)
	{
		ROS_ERROR("Failure at Right Motor.");
	}
	else if(this->CheckError() == -3)
	{
		ROS_ERROR("Failure at Both of Motors.");
	}

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
  
  int i_m1_in1, i_m1_in2, i_m1_en;
  int i_m2_in1, i_m2_in2, i_m2_en;

  n.param<int>("evarobot_driver/M1_IN1", i_m1_in1, 1);
  n.param<int>("evarobot_driver/M1_IN2", i_m1_in2, 12);
  n.param<int>("evarobot_driver/M1_EN", i_m1_en, 5);

  
  n.param<int>("evarobot_driver/M2_IN1", i_m2_in1, 0);
  n.param<int>("evarobot_driver/M2_IN2", i_m2_in2, 19);
  n.param<int>("evarobot_driver/M2_EN", i_m2_en, 6);

  
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
  
  stringstream ss1, ss2;
  
  ss1 << i_m1_in1;
  ss2 << i_m1_in2;
  
  IMGPIO gpio_m1_in[2] = {IMGPIO(ss1.str()), IMGPIO(ss2.str())};
  
  ss1.str("");
  ss2.str("");
  ss1 << i_m2_in1;
  ss2 << i_m2_in2;
  IMGPIO gpio_m2_in[2] = {IMGPIO(ss1.str()), IMGPIO(ss2.str())};
  
  ss1.str("");
  ss2.str("");
  ss1 << i_m1_en;
  ss2 << i_m2_en;
  
  IMGPIO gpio_m1_en = IMGPIO(ss1.str());
  IMGPIO gpio_m2_en = IMGPIO(ss2.str());
     
  IMDRIVER imdriver((float)d_max_lin_vel, (float)d_max_ang_vel, 
				  (float)d_wheel_separation, (float)d_wheel_diameter,
					d_frequency, i_counts, d_duty, i_mode, 
					gpio_m1_in, gpio_m2_in, &gpio_m1_en, &gpio_m2_en);
  
  //ros::Subscriber sub = n.subscribe(str_topic.c_str(), 2, &IMDRIVER::CallbackVel, &imdriver);
  ros::Subscriber sub = n.subscribe(str_topic.c_str(), 2, &IMDRIVER::CallbackWheelVel, &imdriver);
  
  
  if(sub.getNumPublishers() < 1)
  {
	  ROS_WARN("No appropriate subscriber.");
  }

  ros::spin();

 

  return 0;
}
