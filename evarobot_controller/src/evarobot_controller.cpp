//! Bu sınıfa ait dokümantasyon evapi_ros 85rpm altındaki dokümantasyon ile aynıdır.

#include "evarobot_controller/evarobot_controller.h"

#include <dynamic_reconfigure/server.h>
#include <evarobot_controller/ParamsConfig.h>


int i_error_code = 0;

bool b_ekb_test_status = false;


void PIDController::Reset()
{
	this->d_proportional_error = 0.0;
	this->d_derivative_error = 0.0;
	this->d_integral_error = 0.0;
	this->d_pre_error = 0.0;
//	ROS_INFO("Reseting PID Error");
}

PIDController::PIDController()
{
	this->Reset();
	
	this->d_integral_constant = 0.0;
	this->d_derivative_constant = 0.0;
	this->d_proportional_constant = 0.0;
	
	this->read_time = ros::Time::now();
	this->d_max_vel = 0.9;
		
}

PIDController::PIDController(double d_proportional_constant, 
														 double d_integral_constant, 
														 double d_derivative_constant,
														 double _d_max_vel,
														 string _str_name):d_max_vel(_d_max_vel), str_name(_str_name)
{
	this->Reset();
	
	this->d_integral_constant = d_integral_constant;
	this->d_derivative_constant = d_derivative_constant;
	this->d_proportional_constant = d_proportional_constant;
	
	this->read_time = ros::Time::now();
}

PIDController::~PIDController(){}

float PIDController::RunController(float f_desired, float f_measured)
{
	float f_ret = 0;
	
	double d_current_error = (double)(f_desired - f_measured);

	if( (this->d_integral_error < 0.0 && f_desired >= 0.0) || (this->d_integral_error > 0.0 && f_desired <= 0.0) )
	{
		this->d_integral_error = 0.0;
	}

	if(fabs(d_current_error) <= 0.01)
	{
		d_current_error = 0.0;
		//ROS_DEBUG("Steady state error is in the boundary.");
	}

	
	this->dur_time = ros::Time::now() - this->read_time;
	this->read_time = ros::Time::now();

	this->d_proportional_error = d_current_error;
	this->d_derivative_error = (d_current_error - this->d_pre_error) / dur_time.toSec();
	this->d_integral_error += d_current_error * dur_time.toSec();
	
	f_ret = this->d_proportional_constant *  this->d_proportional_error 
			+ this->d_integral_constant *  this->d_integral_error
			+ this->d_derivative_constant * this->d_derivative_error;
	
	this->d_pre_error = d_current_error;
			
	if( fabs(f_ret) > this->d_max_vel )
	{
		ROS_DEBUG("EvarobotController: MAX VEL: %f", this->d_max_vel);
		f_ret = (f_ret * this->d_max_vel) / fabs(f_ret);
	}

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

void PIDController::ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	if(i_error_code<0)
    	{
            stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "%s", GetErrorDescription(i_error_code).c_str());
    	    i_error_code = 0;
        }
	else
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "%s is OK!", this->str_name.c_str());

		stat.add("Proportional Constant", this->d_proportional_constant);
		stat.add("Integral Constant", this->d_integral_constant);
		stat.add("Derivative Constant", this->d_derivative_constant);

		stat.add("Proportional Error", this->d_proportional_error);
		stat.add("Integral Error", this->d_integral_error);
		stat.add("Derivative Error", this->d_derivative_error);
	}
}

void CallbackMeasured(const im_msgs::WheelVel::ConstPtr & msg)
{
	g_d_dt = (ros::Time::now() - msg->header.stamp).toSec();
	g_f_left_measured = msg->left_vel;
	g_f_right_measured = msg->right_vel;
}

void CallbackDesired(const geometry_msgs::Twist::ConstPtr & msg)
{
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

bool CallbackResetController(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	ROS_DEBUG("EvarobotContoller: Reset Controller");
	b_reset_controller = true;
	return true;
}

bool CallbackEKBTest(im_msgs::EKBTest::Request& request, im_msgs::EKBTest::Response& response)
{
	ROS_DEBUG("EvarobotController: EKB Test");
	b_ekb_test_status = request.b_on_off;
	response.b_success = true;
	return true;
}


int main(int argc, char **argv)
{

  // ROS PARAMS
  double d_integral_constant_left;
  double d_derivative_constant_left;
  double d_proportional_constant_left;

  double d_integral_constant_right;
  double d_derivative_constant_right;
  double d_proportional_constant_right;
    
  double d_max_vel;
  double d_frequency;
  double d_max_freq, d_min_freq;
  
  double d_timeout;
  
  //---------------
  
  ros::init(argc, argv, "/evarobot_controller");
  ros::NodeHandle n;

  
  n.param<double>("evarobot_controller/maxVel", d_max_vel, 0.9);
  n.param("evarobot_controller/minFreq", d_min_freq, 0.2);
  n.param("evarobot_controller/maxFreq", d_max_freq, 10.0);
  n.param("evarobot_controller/timeout", d_timeout, 0.5);
  
  if(!n.getParam("evarobot_controller/wheelSeparation", g_d_wheel_separation))
  {
	  ROS_INFO(GetErrorDescription(-28).c_str());
  	  i_error_code = -28;
  }
  if(!n.getParam("evarobot_controller/integralConstLeft", d_integral_constant_left))
  {
	  ROS_INFO(GetErrorDescription(-65).c_str());
	  i_error_code = -65;
  }

  if(!n.getParam("evarobot_controller/derivativeConstLeft", d_derivative_constant_left))
  {
	  ROS_INFO(GetErrorDescription(-69).c_str());
	  i_error_code = -69;
  }  

  if(!n.getParam("evarobot_controller/proportionalConstLeft", d_proportional_constant_left))
  {
	  ROS_INFO(GetErrorDescription(-70).c_str());
	  i_error_code = -70;
  } 

  if(!n.getParam("evarobot_controller/integralConstRight", d_integral_constant_right))
  {
	  ROS_INFO(GetErrorDescription(-71).c_str());
	  i_error_code = -71;
  }

  if(!n.getParam("evarobot_controller/derivativeConstRight", d_derivative_constant_right))
  {
	  ROS_INFO(GetErrorDescription(-72).c_str());
	  i_error_code = -72;
  }  

  if(!n.getParam("evarobot_controller/proportionalConstRight", d_proportional_constant_right))
  {
	  ROS_INFO(GetErrorDescription(-73).c_str());
  	  i_error_code = -73;
  }   
  
  if(!n.getParam("evarobot_controller/Frequency", d_frequency))
  {
	  ROS_INFO(GetErrorDescription(-74).c_str());
	  i_error_code = -74;
  }  
    
  PIDController left_controller = PIDController(d_proportional_constant_left, 
																								d_integral_constant_left, 
																								d_derivative_constant_left,
																								d_max_vel,
																								"LeftController");
																								
  PIDController right_controller = PIDController(d_proportional_constant_right, 
																								 d_integral_constant_right, 
																								 d_derivative_constant_right,
																								 d_max_vel,
																								 "RightController");
  
  ros::Subscriber measured_sub = n.subscribe("wheel_vel", 1, CallbackMeasured);
  ros::Subscriber desired_sub = n.subscribe("cmd_vel", 1, CallbackDesired);
  realtime_tools::RealtimePublisher<im_msgs::WheelVel> * ctrl_pub = new realtime_tools::RealtimePublisher<im_msgs::WheelVel>(n, "cntr_wheel_vel", 10);
  
  ros::ServiceServer service = n.advertiseService("evarobot_controller/reset_controller", CallbackResetController);
  
  // Dynamic Reconfigure
  dynamic_reconfigure::Server<evarobot_controller::ParamsConfig> srv;
  dynamic_reconfigure::Server<evarobot_controller::ParamsConfig>::CallbackType f;
  f = boost::bind(&CallbackReconfigure, _1, _2);
  srv.setCallback(f);
  ///////////////
  
  // Diagnostics
  diagnostic_updater::Updater updater;
	updater.setHardwareID("None");
	updater.add("LeftController", &left_controller, &PIDController::ProduceDiagnostics);
	updater.add("RightController", &right_controller, &PIDController::ProduceDiagnostics);
		
	diagnostic_updater::HeaderlessTopicDiagnostic pub_freq("cntr_wheel_vel", updater,
											diagnostic_updater::FrequencyStatusParam(&d_min_freq, &d_max_freq, 0.1, 10));
  		
  
	ros::Rate loop_rate(d_frequency);
  
	ros::ServiceServer service_ = n.advertiseService("evarobot_controller/ekb_test", CallbackEKBTest);
  
	while(!b_ekb_test_status && ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
  
  while(ros::ok())
  {
		
		if(b_is_received_params)
		{
			ROS_DEBUG("EvarobotController: Updating Controller Params...");
			ROS_DEBUG("EvarobotController: %f, %f, %f", g_d_p_left, g_d_i_left, g_d_d_left);
			ROS_DEBUG("EvarobotController: %f, %f, %f", g_d_p_right, g_d_i_right, g_d_d_right);
			
			left_controller.UpdateParams(g_d_p_left, g_d_i_left, g_d_d_left);
			right_controller.UpdateParams(g_d_p_right, g_d_i_right, g_d_d_right);
			b_is_received_params = false;
		}

		if(b_reset_controller)
		{
			left_controller.Reset();
			right_controller.Reset();
			b_reset_controller = false;
		}

		ctrl_pub->msg_.header.stamp = ros::Time::now();
		ctrl_pub->msg_.left_vel = left_controller.RunController(g_f_left_desired, g_f_left_measured);
		ctrl_pub->msg_.right_vel = right_controller.RunController(g_f_right_desired, g_f_right_measured);
		
		if(g_d_dt > d_timeout)
		{
			ROS_INFO(GetErrorDescription(-75).c_str());
			i_error_code = -75;
			ctrl_pub->msg_.left_vel = 0.0;
			ctrl_pub->msg_.right_vel = 0.0;
			left_controller.Reset();
			right_controller.Reset();
		}
		else if(g_d_dt < 0)
		{
			ROS_INFO(GetErrorDescription(-76).c_str());
			i_error_code = -76;
			ctrl_pub->msg_.left_vel = 0.0;
			ctrl_pub->msg_.right_vel = 0.0;
			left_controller.Reset();
			right_controller.Reset();
		}
		
		if (ctrl_pub->trylock())
		{
			ctrl_pub->unlockAndPublish();
		}			
		pub_freq.tick();
		

	  updater.update();
	  ros::spinOnce();
	  loop_rate.sleep();
  }

  return 0;
}
