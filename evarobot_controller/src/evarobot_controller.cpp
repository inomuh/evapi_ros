#include "evarobot_controller/evarobot_controller.h"

/**
 * Resets pid parameters to zero.
 */
void PIDController::Reset() {
  this->d_proportional_error = 0.0;
  this->d_derivative_error = 0.0;
  this->d_integral_error = 0.0;
  this->d_windup_error = 0.0;
  this->d_pre_error = 0.0;
}

/**
 * Default Constructor
 */
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

/**
 * Constructor with custom parameters.
 */
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

/**
 * Destructor
 */
PIDController::~PIDController() {}

/**
 * Runs controller to reach desired value.
 * Takes desired and measured values as parameter.
 * Returns value should be applied.
 */
float PIDController::RunController(float f_desired, float f_measured) {
	/**
	 * Output of pid controller
	 */
	float f_ret = 0;

	/**
	 * Difference between desired and measured values.
	 */
	double d_current_error = (double)(f_desired - f_measured);

	/**
	 * If error is smaller than or equal to 0.10, it can be ignored.
	 */
	if ( fabs(d_current_error) <= 0.10 ) {
		d_current_error = 0.0;
	}

	/**
	 * Clears integral error if direction of wheel need to change.
	 */
	if ( (this->d_integral_error < 0.0 && f_desired > 0.0) 
	   || (this->d_integral_error > 0.0 && f_desired < 0.0) ) {
		this->d_integral_error = 0;
	}

	/**
	 * Difference between current time and previous time.
	 */
	this->dur_time = ros::Time::now() - this->read_time;

	/**
	 * Current time is assigned to read_time for future usage.
	 */
	this->read_time = ros::Time::now();

	/**
	 * Proportional error, derivative error and integral error are calculated using current error.
	 */
	this->d_proportional_error = d_current_error;
	this->d_derivative_error = -(f_measured - d_pre_error) / dur_time.toSec();
	this->d_integral_error += d_current_error * dur_time.toSec();

	/**
	 * Output of pid is calculated.
	 */
	f_ret = this->d_proportional_constant *  this->d_proportional_error
			+ this->d_integral_constant *  this->d_integral_error
			+ this->d_derivative_constant * this->d_derivative_error;

	/**
	 * Previous proportional error is set as measured value.
	 */
	this->d_pre_error = f_measured;

	/**
	 * If calculated output value is higher than maximum velocity, normalizes it.
	 */
	if ( fabs(f_ret) > this->d_max_vel ) {
		float f_ret_w = (f_ret * this->d_max_vel) / fabs(f_ret);
		this->d_windup_error = f_ret - f_ret_w;
		f_ret += this->d_windup_constant * this->d_windup_error * this->d_integral_error;
	}

	/**
	 * If normalized output is higher than maximum velocity, normalizes it again.
	 */
	if ( fabs(f_ret) > this->d_max_vel ) {
		ROS_INFO("EvarobotController: MAX VEL: %f", this->d_max_vel);
		f_ret = (f_ret * this->d_max_vel) / fabs(f_ret);
	}

	/**
	 * Low pass filter is applied to output.
	 */

	if ( f_desired == 0.0 ) {
		this->Reset();
		f_raw_data = 0.0;
	} else {
		f_raw_data = f_ret;
	}

	float f_cutoff = 0.85;
	float f_raw_data = f_ret;
	float f_filtered_data = (1 - f_cutoff)*f_raw_data + f_cutoff*this->f_filtered_data_pre;
	this->f_filtered_data_pre = f_filtered_data;
	return f_filtered_data;
}

/**
 * Updates pid parameters.
 */
void PIDController::UpdateParams(double d_proportional_constant,
                                 double d_integral_constant,
                                 double d_derivative_constant,
				 double d_windup_constant) {
  this->d_integral_constant = d_integral_constant;
  this->d_derivative_constant = d_derivative_constant;
  this->d_proportional_constant = d_proportional_constant;
  this->d_windup_constant = d_windup_constant;
}

/**
 *  If an error occurs publishes it.
 *  Else publishes current pid constants and pid errors.
 */
void PIDController::ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
	if ( i_error_code < 0 ) {
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
				  "%s",
				  GetErrorDescription(i_error_code).c_str());
		i_error_code = 0;
	}
	else {
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

/**
 * Callback of odom topic.
 */
void CallbackMeasuredOdom(const nav_msgs::Odometry::ConstPtr & msg) {
  if (g_i_controller_type == 1) {
    g_d_dt = (ros::Time::now() - msg->header.stamp).toSec();
  }
  g_f_linear_measured = msg->twist.twist.linear.x;
  g_f_angular_measured = msg->twist.twist.angular.z;
}

/**
 * Callback of wheel_vel topic
 */
void CallbackMeasuredWheel(const im_msgs::WheelVel::ConstPtr & msg) {
  if (g_i_controller_type == 0) {
    g_d_dt = (ros::Time::now() - msg->header.stamp).toSec();
  }
  g_f_left_measured = msg->left_vel;
  g_f_right_measured = msg->right_vel;
}

/**
 * Callback of cmd_vel topic
 */
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

/**
 * Callback function to change pid parameters at runtime.
 */
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

/**
 * Service callback to reset controller parameters.
 */
bool CallbackResetController(std_srvs::Empty::Request& request,
                             std_srvs::Empty::Response& response) {
  ROS_DEBUG("EvarobotContoller: Reset Controller");
  b_reset_controller = true;
  return true;
}

/**
 * Program starts from here.
 */
int main(int argc, char **argv) {
	/**
	 * Decoupled mode pid parameters.
	 */
	double d_integral_constant_1;
	double d_derivative_constant_1;
	double d_proportional_constant_1;
	double d_windup_constant_1;

	/**
	 * Coupled mode pid parameters.
	 */
	double d_integral_constant_2;
	double d_derivative_constant_2;
	double d_proportional_constant_2;
	double d_windup_constant_2;

	/**
	 * Maximum velocity for decoupled and coupled modes respectively.
	 */
	double d_max_vel_1, d_max_vel_2;

	/**
	 * Frequency, minimum frequency and maximum frequency variables.
	 */
	double d_frequency;
	double d_max_freq, d_min_freq;

	/**
	 * Controller name for decoupled and coupled modes respectively.
	 */
	string str_cntr_1, str_cntr_2;

	/**
	 * Timout duration for sensor subscribe.
	 */
	double d_timeout;

	/**
	 * Initializes ROS node with evarobot_controller name.
	 */
	ros::init(argc, argv, "/evarobot_controller");

	/**
	 * Creates ROS node handler.
	 */
	ros::NodeHandle n;

	/**
	 * Gets parameters from configuration file.
	 */
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
	n.param<int>("evarobot_controller/controllerType", g_i_controller_type, 0);

	/**
	 * Create decoupled mode pid controller
	 */
	PIDController controller1 = PIDController(d_proportional_constant_1,
						d_integral_constant_1,
						d_derivative_constant_1,
						d_windup_constant_1,
						d_max_vel_1,
						str_cntr_1);

	/**
	 * Create coupled mode pid controller
	 */
	PIDController controller2 = PIDController(d_proportional_constant_2,
						d_integral_constant_2,
						d_derivative_constant_2,
						d_windup_constant_2,
						d_max_vel_2,
						str_cntr_2);

	/**
	 * Subscribers for odom, wheel_vel, cmd_vel topics respectively.
	 */
	ros::Subscriber measured_sub_odom = n.subscribe("odom", 1, CallbackMeasuredOdom);
	ros::Subscriber measured_sub_wheel = n.subscribe("wheel_vel", 1, CallbackMeasuredWheel);
	ros::Subscriber desired_sub = n.subscribe("cmd_vel", 1,	CallbackDesired);

	/**
	 * Publisher topic is created with cntr_wheel_vel topic name and im_msgs::WheelVel message type.
	 */
	realtime_tools::RealtimePublisher<im_msgs::WheelVel> * ctrl_pub =
	  new realtime_tools::RealtimePublisher<im_msgs::WheelVel>
	  (n, "cntr_wheel_vel", 10);

	/**
	 * Service is created to reset controller parameters.
	 */
	ros::ServiceServer service = n.advertiseService(
	  "evarobot_controller/reset_controller",
	  CallbackResetController);

	/**
	 * Dynamic reconfigure is set to provide changing variables at runtime.
	 */
	dynamic_reconfigure::Server<evarobot_controller::ParamsConfig> srv;
	dynamic_reconfigure::Server
	  <evarobot_controller::ParamsConfig>::CallbackType f;
	f = boost::bind(&CallbackReconfigure, _1, _2);
	srv.setCallback(f);

	/**
	 * Set diagnostics to handle and publish error.
	 */
	diagnostic_updater::Updater updater;
	updater.setHardwareID("None");
	updater.add(str_cntr_1, &controller1, &PIDController::ProduceDiagnostics);
	updater.add(str_cntr_2, &controller2, &PIDController::ProduceDiagnostics);
	diagnostic_updater::HeaderlessTopicDiagnostic pub_freq(
	  "cntr_wheel_vel", updater, diagnostic_updater::FrequencyStatusParam(&d_min_freq, &d_max_freq, 0.1, 10));

	/**
	 * Define frequency
	 */
	ros::Rate loop_rate(d_frequency);

	while ( ros::ok() ) {
		/**
    	 * If new parameters are set, information message is written and controller parameters are updated.
    	 */
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

		/**
		 * If reset controller service is called, controller parameters are reset.
		 */
		if ( b_reset_controller ) {
			controller1.Reset();
			controller2.Reset();
			b_reset_controller = false;
		}

		/**
		 * If decoupled mode is selected.
		 */
		if ( g_i_controller_type == 1) {

			/**
			 * Angular velocity and linear velocity are calculated.
			 */
			float f_angVel = 0.0, f_linVel = 0.0;
			f_linVel = controller1.RunController(g_f_linear_desired, g_f_linear_measured);
			f_angVel = controller2.RunController(g_f_angular_desired, g_f_angular_measured);

			ctrl_pub->msg_.header.stamp = ros::Time::now();
			ctrl_pub->msg_.left_vel = 0.5*(2.0 * f_linVel
						 - f_angVel * g_d_wheel_separation);  // m/s
			ctrl_pub->msg_.right_vel = 0.5*(2.0 * f_linVel
						  + f_angVel * g_d_wheel_separation);  // m/s
		}
		/**
		 * Else, coupled mode selected.
		 */
		else {
			/**
			 * Left wheel velocity and right wheel velocity are calculated.
			 */
			ctrl_pub->msg_.header.stamp = ros::Time::now();
			ctrl_pub->msg_.left_vel = controller1.RunController(g_f_left_desired, g_f_left_measured);
			ctrl_pub->msg_.right_vel = controller2.RunController(g_f_right_desired, g_f_right_measured);
		}

		/**
		 * If sensor data is not taken for a long time error occurs.
		 * In this case, reset controller parameters and output velocities.
		 */
		if ( g_d_dt > d_timeout ) {
			ROS_INFO(GetErrorDescription(-75).c_str());
			i_error_code = -75;
			ctrl_pub->msg_.left_vel = 0.0;
			ctrl_pub->msg_.right_vel = 0.0;
			controller1.Reset();
			controller2.Reset();
		}
		/**
		 * Else if time difference is negative, time goes backward error occurs.
		 * In this case, reset controller parameters and output velocities.
		 */
		else if ( g_d_dt < 0 ) {
			ROS_INFO(GetErrorDescription(-76).c_str());
			i_error_code = -76;
			ctrl_pub->msg_.left_vel = 0.0;
			ctrl_pub->msg_.right_vel = 0.0;
			controller1.Reset();
			controller2.Reset();
		}

		/**
		 * Calculated values is published.
		 */
		if ( ctrl_pub->trylock() ) {
			ctrl_pub->unlockAndPublish();
		}
		pub_freq.tick();

		/**
		 * Loop is slept to hold frequency.
		 */
		updater.update();
		ros::spinOnce();
		loop_rate.sleep();
	}

  return 0;
}
