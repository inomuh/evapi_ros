#include "evarobot_controller/evarobot_controller.h"

#include <dynamic_reconfigure/server.h>
#include <evarobot_controller/ParamsConfig.h>

void PIDController::Reset()
{
	this->d_proportional_error = 0.0;
	this->d_derivative_error = 0.0;
	this->d_integral_error = 0.0;

}

PIDController::PIDController()
{
	this->d_integral_error = 0.0;
	this->d_derivative_error = 0.0;
	this->d_proportional_error = 0.0;
	this->d_pre_error = 0.0;
	
	this->d_integral_constant = 0.0;
	this->d_derivative_constant = 0.0;
	this->d_proportional_constant = 0.0;
	
	this->read_time = ros::Time::now();
	this->max_vel = 1.1;
		
}

PIDController::PIDController(double d_proportional_constant, 
			  double d_integral_constant, 
			  double d_derivative_constant)
{
	this->d_integral_error = 0.0;
	this->d_derivative_error = 0.0;
	this->d_proportional_error = 0.0;
	this->d_pre_error = 0.0;
	
	this->d_integral_constant = d_integral_constant;
	this->d_derivative_constant = d_derivative_constant;
	this->d_proportional_constant = d_proportional_constant;
	
	this->read_time = ros::Time::now();
	this->max_vel = 1.1;
}

PIDController::~PIDController(){}

float PIDController::RunController(float f_desired, float f_measured)
{
	float f_ret = 0;
	
	double d_current_error = (double)(f_desired - f_measured);
	
	this->dur_time = ros::Time::now() - this->read_time;
	this->read_time = ros::Time::now();

	if(f_desired >= -0.001 && f_desired <= 0.001)
	{
		this->d_proportional_error = 0.0;
		this->d_derivative_error = 0.0;
		this->d_integral_error = 0.0;

//		printf("\n");

		f_ret = 0.0;
		
		return f_ret;
	}
	
	this->d_proportional_error = d_current_error;
	this->d_derivative_error = (d_current_error - d_pre_error) / dur_time.toSec();
	this->d_integral_error += d_current_error * dur_time.toSec();
	
	f_ret = this->d_proportional_constant *  this->d_proportional_error 
			+ this->d_integral_constant *  this->d_integral_error
			+ this->d_derivative_constant * this->d_derivative_error;
			
	if( fabs(f_ret) > max_vel )
	{
		f_ret = (f_ret * this->max_vel) / fabs(f_ret);
	}
			
//	printf("%lf, %lf, %lf, %f \n", this->d_proportional_error, this->d_integral_error, this->d_derivative_error, f_ret);
	
	return f_ret;
	
}

void PIDController::UpdateParams(double d_proportional_constant, 
								 double d_integral_constant, 
								 double d_derivative_constant)
{
	this->d_integral_constant = d_integral_constant;
	this->d_derivative_constant = d_derivative_constant;
	this->d_proportional_constant = d_proportional_constant;
}



void CallbackMeasured(const geometry_msgs::PointStamped::ConstPtr & msg)
{
	b_is_new_measured = true;
	g_f_left_measured = msg->point.x;
	g_f_right_measured = msg->point.y;
}

void CallbackDesired(const geometry_msgs::Twist::ConstPtr & msg)
{
//	b_is_new_desired = true;
	
	if(fabs(g_f_pre_linear_desired - msg->linear.x) > 0.1 || fabs(g_f_pre_angular_desired - msg->angular.z) > 0.1)
	{
		b_is_new_desired = true;
		g_f_pre_linear_desired = msg->linear.x;
		g_f_pre_angular_desired = msg->angular.z;
	}


	float f_linear_vel = msg->linear.x;
	float f_angular_vel = msg->angular.z;

	g_f_left_desired = ((2 * f_linear_vel) - f_angular_vel * g_d_wheel_separation) / 2; // m/s
	g_f_right_desired = ((2 * f_linear_vel) + f_angular_vel * g_d_wheel_separation) / 2; // m/s
	
	
}

void CallbackReconfigure(evarobot_controller::ParamsConfig &config, uint32_t level)
{
   b_is_received_params = true;        
   g_d_wheel_separation = config.wheelSeparation;       
   
   g_d_p_left = config.proportionalConstLeft;
   g_d_i_left = config.integralConstLeft;
   g_d_d_left = config.derivativeConstLeft;

   g_d_p_right = config.proportionalConstRight;
   g_d_i_right = config.integralConstRight;
   g_d_d_right = config.derivativeConstRight;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "evarobot_controller");
  ros::NodeHandle n;
  
  // ROS PARAMS
  string str_desired_topic;
  string str_measured_topic;
  string str_controller_topic;
  
  double d_integral_constant_left;
  double d_derivative_constant_left;
  double d_proportional_constant_left;

  double d_integral_constant_right;
  double d_derivative_constant_right;
  double d_proportional_constant_right;
  
  bool b_always_on;
  
  double d_frequency;
  
  //---------------
  
  n.param<string>("evarobot_controller/desiredTopic", str_desired_topic, "cmd_vel");  
  n.param<string>("evarobot_controller/measuredTopic", str_measured_topic, "wheel_vel");
  n.param<string>("evarobot_controller/controllerTopic", str_controller_topic, "cntr_wheel_vel");
  
  n.param("evarobot_odometry/alwaysOn", b_always_on, false);
  
  if(!n.getParam("evarobot_controller/wheelSeparation", g_d_wheel_separation))
  {
	  ROS_ERROR("Failed to get param 'wheelSeparation'");
  }
  if(!n.getParam("evarobot_controller/integralConstLeft", d_integral_constant_left))
  {
	  ROS_ERROR("Failed to get param 'integralConstLeft'");
  }

  if(!n.getParam("evarobot_controller/derivativeConstLeft", d_derivative_constant_left))
  {
	  ROS_ERROR("Failed to get param 'derivativeConstLeft'");
  }  

  if(!n.getParam("evarobot_controller/proportionalConstLeft", d_proportional_constant_left))
  {
	  ROS_ERROR("Failed to get param 'proportionalConstLeft'");
  } 

  if(!n.getParam("evarobot_controller/integralConstRight", d_integral_constant_right))
  {
	  ROS_ERROR("Failed to get param 'integralConstRight'");
  }

  if(!n.getParam("evarobot_controller/derivativeConstRight", d_derivative_constant_right))
  {
	  ROS_ERROR("Failed to get param 'derivativeConstRight'");
  }  

  if(!n.getParam("evarobot_controller/proportionalConstRight", d_proportional_constant_right))
  {
	  ROS_ERROR("Failed to get param 'proportionalConstRight'");
  } 

   
  
  if(!n.getParam("evarobot_controller/Frequency", d_frequency))
  {
	  ROS_ERROR("Failed to get param 'Frequency'");
  }  
  
  
  
  PIDController left_controller = PIDController(d_proportional_constant_left, d_integral_constant_left, d_derivative_constant_left);
  PIDController right_controller = PIDController(d_proportional_constant_right, d_integral_constant_right, d_derivative_constant_right);
     
  
  ros::Subscriber measured_pub = n.subscribe(str_measured_topic.c_str(), 2, CallbackMeasured);
  ros::Subscriber desired_pub = n.subscribe(str_desired_topic.c_str(), 2, CallbackDesired);
  ros::Publisher ctrl_pub = n.advertise<geometry_msgs::Twist>(str_controller_topic.c_str(), 10);
  
  // Dynamic Reconfigure
  dynamic_reconfigure::Server<evarobot_controller::ParamsConfig> srv;
  dynamic_reconfigure::Server<evarobot_controller::ParamsConfig>::CallbackType f;
  f = boost::bind(&CallbackReconfigure, _1, _2);
  srv.setCallback(f);
  ///////////////
  
  ros::Rate loop_rate(d_frequency);

  geometry_msgs::Twist msg;
  
  while(ros::ok())
  {
	  //if(b_is_new_desired && b_is_new_measured)
	 // {
		if(b_is_new_desired)
		{
			//left_controller.Reset();
			//right_controller.Reset();
		}
		
		if(b_is_received_params)
		{
			ROS_INFO("Updating Controller Params...");
			ROS_INFO("%f, %f, %f", g_d_p_left, g_d_i_left, g_d_d_left);
			ROS_INFO("%f, %f, %f", g_d_p_right, g_d_i_right, g_d_d_right);
			
			left_controller.UpdateParams(g_d_p_left, g_d_i_left, g_d_d_left);
			right_controller.UpdateParams(g_d_p_right, g_d_i_right, g_d_d_right);
			b_is_received_params = false;
		}

//		printf("left:  ");
//		printf("left_desired: %f\n", g_f_left_desired);
//		printf("left_measured: %f\n", g_f_left_measured);
		msg.linear.x = left_controller.RunController(g_f_left_desired, g_f_left_measured);
		
//		printf("right:  ");
//		printf("right_desired: %f\n", g_f_right_desired);
//		printf("right_measured: %f\n", g_f_right_measured);
		msg.linear.y = right_controller.RunController(g_f_right_desired, g_f_right_measured);
		
		b_is_new_desired = false;
		b_is_new_measured = false;
		
		if(ctrl_pub.getNumSubscribers() > 0 || b_always_on)
		{
			ctrl_pub.publish(msg);
		}
		
	 // }
	  
	  ros::spinOnce();
	  loop_rate.sleep();
  }

  return 0;
}
