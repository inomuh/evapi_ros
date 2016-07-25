// "Copyright 2015 <Mehmet Akcakoca>"
#include "evarobot_controller/evarobot_controller.h"

#include <dynamic_reconfigure/server.h>
#include <evarobot_controller/ParamsConfig.h>


int i_error_code = 0;

void PIDController::Reset() {
  this->d_proportional_error = 0.0;
  this->d_derivative_error = 0.0;
  this->d_integral_error = 0.0;
  this->d_windup_error = 0.0;
  this->d_pre_error = 0.0;
}

PIDController::PIDController() {
  this->Reset();

  this->d_integral_constant = 0.0;
  this->d_derivative_constant = 0.0;
  this->d_proportional_constant = 0.0;
  this->d_windup_constant = 0.0;

  this->read_time = ros::Time::now();
  this->d_max_vel = 0.7;
  this->f_filtered_data_pre = 0.0;
}

PIDController::PIDController(double d_proportional_constant,
                             double d_integral_constant,
                             double d_derivative_constant,
			     double d_windup_constant,
                             double _d_max_vel,
                             string _str_name):
    d_max_vel(_d_max_vel), str_name(_str_name) {
  this->Reset();

  this->d_integral_constant = d_integral_constant;
  this->d_derivative_constant = d_derivative_constant;
  this->d_proportional_constant = d_proportional_constant;
  this->d_windup_constant = d_windup_constant;

  this->read_time = ros::Time::now();

  this->f_filtered_data_pre = 0.0;
}

PIDController::~PIDController() {}

float PIDController::RunController(float f_desired, float f_measured) {
  float f_ret = 0;
  float f_ret_w = 0.0;
  float f_filtered_data = 0.0, f_raw_data = 0.0;
  float f_cutoff = 0.85;	

  double d_current_error = (double)(f_desired - f_measured);
  /*
  if ( (this->d_integral_error < 0.0 && f_desired >= 0.0)
       || (this->d_integral_error > 0.0 && f_desired <= 0.0) ) {
    this->d_integral_error = 0.0;
  }
  */
  if ( fabs(d_current_error) <= 0.10 ) {
    d_current_error = 0.0;
  }

  this->dur_time = ros::Time::now() - this->read_time;
  this->read_time = ros::Time::now();

  this->d_proportional_error = d_current_error;
  this->d_derivative_error = -(f_measured - d_pre_error) / dur_time.toSec();
  this->d_integral_error += d_current_error * dur_time.toSec();

  f_ret = this->d_proportional_constant *  this->d_proportional_error
      + this->d_integral_constant *  this->d_integral_error
      + this->d_derivative_constant * this->d_derivative_error;

  this->d_pre_error = f_measured;
  
  if ( fabs(f_ret) > this->d_max_vel ) {
    f_ret_w = (f_ret * this->d_max_vel) / fabs(f_ret);
    this->d_windup_error = f_ret - f_ret_w;
    f_ret += this->d_windup_constant * this->d_windup_error * this->d_integral_error;
  }

  if ( fabs(f_ret) > this->d_max_vel ) {
    ROS_INFO("EvarobotController: MAX VEL: %f", this->d_max_vel);
    f_ret = (f_ret * this->d_max_vel) / fabs(f_ret);  
  }

  // First Order Low-Pass Filter
  f_raw_data = f_ret;
  f_filtered_data = (1 - f_cutoff)*f_raw_data + f_cutoff*this->f_filtered_data_pre;
  this->f_filtered_data_pre = f_filtered_data;

  return f_filtered_data;
}

void PIDController::UpdateParams(double d_proportional_constant,
                                 double d_integral_constant,
                                 double d_derivative_constant,
				 double d_windup_constant) {
  this->d_integral_constant = d_integral_constant;
  this->d_derivative_constant = d_derivative_constant;
  this->d_proportional_constant = d_proportional_constant;
  this->d_windup_constant = d_windup_constant;
}

void PIDController::ProduceDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if ( i_error_code < 0 ) {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                  "%s",
                  GetErrorDescription(i_error_code).c_str());
    i_error_code = 0;
  } else {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
                  "%s is OK!",
                  this->str_name.c_str());

    stat.add("Proportional Constant", this->d_proportional_constant);
    stat.add("Integral Constant", this->d_integral_constant);
    stat.add("Derivative Constant", this->d_derivative_constant);
    stat.add("Windup Constant", this->d_windup_constant);

    stat.add("Proportional Error", this->d_proportional_error);
    stat.add("Integral Error", this->d_integral_error);
    stat.add("Derivative Error", this->d_derivative_error);
    stat.add("Windup Error", this->d_windup_error);
  }
}

void CallbackMeasuredOdom(const nav_msgs::Odometry::ConstPtr & msg) {
  if (g_i_controller_type == 1) {
    g_d_dt = (ros::Time::now() - msg->header.stamp).toSec();
  }
  g_f_linear_measured = msg->twist.twist.linear.x;
  g_f_angular_measured = msg->twist.twist.angular.z;
}

void CallbackMeasuredWheel(const im_msgs::WheelVel::ConstPtr & msg) {
  if (g_i_controller_type == 0) {
    g_d_dt = (ros::Time::now() - msg->header.stamp).toSec();
  }
  g_f_left_measured = msg->left_vel;
  g_f_right_measured = msg->right_vel;
}

void CallbackDesired(const geometry_msgs::Twist::ConstPtr & msg) {
  float f_linear_vel = msg->linear.x;
  float f_angular_vel = msg->angular.z;

  g_f_linear_desired = f_linear_vel;
  g_f_angular_desired = f_angular_vel;

  g_f_left_desired = 0.5*(2.0 * f_linear_vel
                          - f_angular_vel * g_d_wheel_separation);  // m/s
  g_f_right_desired = 0.5*(2.0 * f_linear_vel
                           + f_angular_vel * g_d_wheel_separation);  // m/s
}

void CallbackReconfigure(evarobot_controller::ParamsConfig &config, uint32_t level) {
  b_is_received_params = true;
  g_d_wheel_separation = config.wheelSeparation;

  g_d_p_1 = config.proportionalConst1;
  g_d_i_1 = config.integralConst1;
  g_d_d_1 = config.derivativeConst1;
  g_d_w_1 = config.windupConst1;

  g_d_p_2 = config.proportionalConst2;
  g_d_i_2 = config.integralConst2;
  g_d_d_2 = config.derivativeConst2;
  g_d_w_2 = config.windupConst2;
}

bool CallbackResetController(std_srvs::Empty::Request& request,
                             std_srvs::Empty::Response& response) {
  ROS_DEBUG("EvarobotContoller: Reset Controller");
  b_reset_controller = true;
  return true;
}

int main(int argc, char **argv) {
  // ROS PARAMS
  double d_integral_constant_1;
  double d_derivative_constant_1;
  double d_proportional_constant_1;
  double d_windup_constant_1;

  double d_integral_constant_2;
  double d_derivative_constant_2;
  double d_proportional_constant_2;
  double d_windup_constant_2;

  double d_max_vel_1, d_max_vel_2;
  double d_frequency;
  double d_max_freq, d_min_freq;

  string str_cntr_1, str_cntr_2;

  double d_timeout;
  //---------------

  ros::init(argc, argv, "/evarobot_controller");
  ros::NodeHandle n;


  n.param<double>("evarobot_controller/maxVel1", d_max_vel_1, 0.7);
  n.param<double>("evarobot_controller/maxVel2", d_max_vel_2, 0.7);

  n.param<double>("evarobot_controller/minFreq", d_min_freq, 0.2);
  n.param<double>("evarobot_controller/maxFreq", d_max_freq, 10.0);
  n.param<double>("evarobot_controller/timeout", d_timeout, 0.5);

  n.param<double>("evarobot_controller/wheelSeparation", g_d_wheel_separation, 0.34);

  n.param<double>("evarobot_controller/integralConst1", d_integral_constant_1, 0.1);
  n.param<double>("evarobot_controller/derivativeConst1", d_derivative_constant_1, 0.1);
  n.param<double>("evarobot_controller/proportionalConst1", d_proportional_constant_1, 0.1);
  n.param<double>("evarobot_controller/windupConst1", d_windup_constant_1, 0.1);

  n.param<double>("evarobot_controller/integralConst2", d_integral_constant_2, 0.1);
  n.param<double>("evarobot_controller/derivativeConst2", d_derivative_constant_2, 0.1);
  n.param<double>("evarobot_controller/proportionalConst2", d_proportional_constant_2, 0.1);
  n.param<double>("evarobot_controller/windupConst2", d_windup_constant_2, 0.1);

  n.param<double>("evarobot_controller/Frequency", d_frequency, 10.0);

  n.param<string>("evarobot_controller/cntr1Name", str_cntr_1, "Controller_1");
  n.param<string>("evarobot_controller/cntr2Name", str_cntr_2, "Controller_2");

  // Controller Type: 0->Decoupled, 1->Coupled
  n.param<int>("evarobot_controller/controllerType", g_i_controller_type, 0);

  PIDController controller1 = PIDController(d_proportional_constant_1, 
					    d_integral_constant_1,
                                            d_derivative_constant_1,
					    d_windup_constant_1,
                                            d_max_vel_1,
                                            str_cntr_1);
  PIDController controller2 = PIDController(d_proportional_constant_2,
					    d_integral_constant_2,
					    d_derivative_constant_2,
					    d_windup_constant_2,
					    d_max_vel_2,
					    str_cntr_2);

  ros::Subscriber measured_sub_odom = n.subscribe("odom", 1, CallbackMeasuredOdom);
  ros::Subscriber measured_sub_wheel = n.subscribe("wheel_vel", 1, CallbackMeasuredWheel);
  ros::Subscriber desired_sub = n.subscribe("cmd_vel",
                                            1,
                                            CallbackDesired);
  realtime_tools::RealtimePublisher<im_msgs::WheelVel> * ctrl_pub =
      new realtime_tools::RealtimePublisher<im_msgs::WheelVel>
      (n, "cntr_wheel_vel", 10);

  ros::ServiceServer service = n.advertiseService(
      "evarobot_controller/reset_controller",
      CallbackResetController);

  // Dynamic Reconfigure
  dynamic_reconfigure::Server<evarobot_controller::ParamsConfig> srv;
  dynamic_reconfigure::Server
      <evarobot_controller::ParamsConfig>::CallbackType f;
  f = boost::bind(&CallbackReconfigure, _1, _2);
  srv.setCallback(f);
  ///////////////
  // Diagnostics
  diagnostic_updater::Updater updater;
  updater.setHardwareID("None");
  updater.add(str_cntr_1,
              &controller1,
              &PIDController::ProduceDiagnostics);
  updater.add(str_cntr_2,
              &controller2,
              &PIDController::ProduceDiagnostics);

  diagnostic_updater::HeaderlessTopicDiagnostic pub_freq(
      "cntr_wheel_vel",
      updater,
      diagnostic_updater::FrequencyStatusParam(
          &d_min_freq,
          &d_max_freq,
          0.1,
          10));

  ros::Rate loop_rate(d_frequency);

  while ( ros::ok() ) {
    if ( b_is_received_params ) {
      ROS_DEBUG("EvarobotController: Updating Controller Params...");
      ROS_DEBUG("EvarobotController: %f, %f, %f, %f",
                g_d_p_1, g_d_i_1, g_d_d_1, g_d_w_1);
      ROS_DEBUG("EvarobotController: %f, %f, %f, %f",
                g_d_p_2, g_d_i_2, g_d_d_2, g_d_w_2);

      controller1.UpdateParams(g_d_p_1, g_d_i_1, g_d_d_1, g_d_w_1);
      controller2.UpdateParams(g_d_p_2, g_d_i_2, g_d_d_2, g_d_w_2);
      b_is_received_params = false;
    }

    if ( b_reset_controller ) {
      controller1.Reset();
      controller2.Reset();
      b_reset_controller = false;
    }

    if ( g_i_controller_type == 1) {

      float f_angVel = 0.0, f_linVel = 0.0;

      f_linVel = controller1.RunController(g_f_linear_desired, g_f_linear_measured);
      f_angVel = controller2.RunController(g_f_angular_desired, g_f_angular_measured);

      ctrl_pub->msg_.header.stamp = ros::Time::now();
      ctrl_pub->msg_.left_vel = 0.5*(2.0 * f_linVel 
				     - f_angVel * g_d_wheel_separation);  // m/s
      ctrl_pub->msg_.right_vel = 0.5*(2.0 * f_linVel 
				      + f_angVel * g_d_wheel_separation);  // m/s
    } else {
      ctrl_pub->msg_.header.stamp = ros::Time::now();
      ctrl_pub->msg_.left_vel = controller1.RunController(g_f_left_desired, g_f_left_measured);
      ctrl_pub->msg_.right_vel = controller2.RunController(g_f_right_desired, g_f_right_measured);
    }

    if ( g_d_dt > d_timeout ) {
      ROS_INFO(GetErrorDescription(-75).c_str());
      i_error_code = -75;
      ctrl_pub->msg_.left_vel = 0.0;
      ctrl_pub->msg_.right_vel = 0.0;
      controller1.Reset();
      controller2.Reset();
    } else if ( g_d_dt < 0 ) {
      ROS_INFO(GetErrorDescription(-76).c_str());
      i_error_code = -76;
      ctrl_pub->msg_.left_vel = 0.0;
      ctrl_pub->msg_.right_vel = 0.0;
      controller1.Reset();
      controller2.Reset();
    }

    if ( ctrl_pub->trylock() ) {
      ctrl_pub->unlockAndPublish();
    }
    pub_freq.tick();


    updater.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
