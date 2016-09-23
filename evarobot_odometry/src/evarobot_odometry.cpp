#include "evarobot_odometry/evarobot_odometry.h"

/**
 * Callback function to change variables at runtime.
 */
void CallbackReconfigure(evarobot_odometry::ParamsConfig &config, uint32_t level)
{
   b_is_received_params = true;        
   d_wheel_separation = config.wheelSeparation;
   d_height = config.height;
   i_gear_ratio = config.gearRatio;
   i_cpr = config.CPR;
   d_wheel_diameter = config.wheelDiameter;
}

/**
 * Callback function to resetting odometry.
 */
bool CallbackResetOdom(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	ROS_DEBUG("EvarobotOdometry: Reset Odometry");
	b_reset_odom = true;
	return true;
}

/**
 * If an error occurs publishes it.
 * Else publishes "EvarobotOdometry: No problem." message.
 */
void ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    if(i_error_code<0)
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "%s", GetErrorDescription(i_error_code).c_str());
        i_error_code = 0;
    }
    else
    {
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "EvarobotOdometry: No problem.");
    }
}

/**
 * Program starts here.
 */
int main(int argc, char **argv)
{
	/**
	 * File descriptor for encoder operations.
	 */
	int i_fd;

	/**
	 * Frequency, minimum frequency and maximum frequency variables.
	 */
	double d_frequency;
	double d_min_freq, d_max_freq;

    stringstream ss;
	
    /**
     * Encoder device path.
     */
	string str_device_path;

	/**
	 * Previous linear velocity of right and left wheels respectively.
	 */
	float f_lin_velR_pre = 0.0, f_lin_velL_pre = 0.0;

	/**
	 * Initializes ROS node with evarobot_odometry name.
	 */
	ros::init(argc, argv, "/evarobot_odometry");

	/**
	 * Creates ROS node handler.
	 */
	ros::NodeHandle n;

	/**
	 * Gets parameters from configuration file.
	 */
	n.param<string>("evarobot_odometry/devicePath", str_device_path, "/dev/evarobotEncoder");
	n.param("evarobot_odometry/minFreq", d_min_freq, 0.2);
	n.param("evarobot_odometry/maxFreq", d_max_freq, 10.0);
	
	if(!n.getParam("evarobot_odometry/wheelSeparation", d_wheel_separation))
	{
	  ROS_INFO(GetErrorDescription(-108).c_str());
        i_error_code = -108;
	}
	
	if(!n.getParam("evarobot_odometry/height", d_height))
	{
	  ROS_INFO(GetErrorDescription(-109).c_str());
        i_error_code = -109;
	} 	
	
	if(!n.getParam("evarobot_odometry/frequency", d_frequency))
	{
	  ROS_INFO(GetErrorDescription(-110).c_str());
        i_error_code = -110;
	} 	
	
	if(!n.getParam("evarobot_odometry/gearRatio", i_gear_ratio))
	{
	  ROS_INFO(GetErrorDescription(-111).c_str());
        i_error_code = -111;
	} 	
	
	if(!n.getParam("evarobot_odometry/CPR", i_cpr))
	{
	  ROS_INFO(GetErrorDescription(-112).c_str());
        i_error_code = -112;
	} 	
	
	if(!n.getParam("evarobot_odometry/wheelDiameter", d_wheel_diameter))
	{
	  ROS_INFO(GetErrorDescription(-113).c_str());
        i_error_code = -113;
	}
		
	/**
	 * Publisher topic is created with odom topic name and nav_msgs::Odometry message type.
	 * Publisher topic is created with wheel_vel topic name and im_msgs::WheelVel message type.
	 */
	realtime_tools::RealtimePublisher<nav_msgs::Odometry> * pose_pub = new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(n, "odom", 10);
	realtime_tools::RealtimePublisher<im_msgs::WheelVel> * vel_pub = new realtime_tools::RealtimePublisher<im_msgs::WheelVel>(n, "wheel_vel", 10);

	/**
	 * Service is created to reset odometry parameters.
	 */
	ros::ServiceServer service = n.advertiseService("evarobot_odometry/reset_odom", CallbackResetOdom);
	
	/**
	 * Define frequency
	 */
	ros::Rate loop_rate(d_frequency);

	/**
	 * Read time is initialized as current time.
	 */
	ros::Time read_time = ros::Time::now();

	/**
	 * Difference between current time and previous read time.
	 */
	ros::Duration dur_time;

	/**
	 * Dynamic reconfigure is set to provide changing variables at runtime.
	 */
	dynamic_reconfigure::Server<evarobot_odometry::ParamsConfig> srv;
	dynamic_reconfigure::Server<evarobot_odometry::ParamsConfig>::CallbackType f;
	f = boost::bind(&CallbackReconfigure, _1, _2);
	srv.setCallback(f);

	/**
	 * Set diagnostics to handle and publish error.
	 */
	diagnostic_updater::Updater updater;
	updater.setHardwareID("none");
	updater.add("evarobot_odometry", &ProduceDiagnostics);
		
	diagnostic_updater::HeaderlessTopicDiagnostic pub_freq("odom", updater,
											diagnostic_updater::FrequencyStatusParam(&d_min_freq, &d_max_freq, 0.1, 10));
    
	/**
	 * Raw encoder data.
	 */
	char c_read_buf[31];
	string str_data;

	/**
	 * Current pose of the robot.
	 */
	geometry_msgs::Pose2D odom_pose;
	
	/**
	 * Difference between current pose and previous pose.
	 */
	geometry_msgs::Pose2D delta_odom_pose;

	/**
	 * Pulse number of left and right wheel encoders respectively.
	 */
	float f_left_read = 0.0, f_right_read = 0.0;

	/**
	 * Last known pulse number of left and right wheel encoders respectively.
	 */
	float f_left_read_last = 0.0, f_right_read_last = 0.0; 

	/**
	 * Covariance matrix
	 */										
	float covariance[36] =	{0.05, 0, 0, 0, 0, 0,  // covariance on gps_x
				 0, 0.05, 0, 0, 0, 0,  // covariance on gps_y
				 0, 0, 0.05, 0, 0, 0,  // covariance on gps_z
				 0, 0, 0, 0.3, 0, 0,  // large covariance on rot x
				 0, 0, 0, 0, 0.3, 0,  // large covariance on rot y
				 0, 0, 0, 0, 0, 0.3};  // large covariance on rot z	
	
	/**
	 * Covariance matrix is added to ROS message
	 */
	for(int i = 0; i < 36; i++)
	{
		pose_pub->msg_.pose.covariance[i] = covariance[i];
	}		
	
	/**
	 * Opening the encoder file.
	 */
	i_fd = open(str_device_path.c_str(), O_RDWR);
	if(i_fd < 0)
	{
		ROS_INFO(GetErrorDescription(-114).c_str());
		i_error_code = -114;
		exit(-1);
	}

	/**
	 * Cleaning the encoder.
	 */
	write(i_fd, "clean", 5);

	while (ros::ok())
	{
		/**
		 * If new parameters are set, information message is written.
		 */
		if(b_is_received_params)
		{
			ROS_DEBUG("EvarobotOdometry: Updating Odometry Params...");
			b_is_received_params = false;
		}
		
		/**
		 * If reset_odom service is called, odometry parameters are reset.
		 */
		if(b_reset_odom)
		{
			ROS_DEBUG("EvarobotOdometry: Resetting Odometry");
			
			write(i_fd, "clean", 5);
			
			f_left_read = 0.0;
			f_right_read = 0.0;
			f_left_read_last = 0.0;
			f_right_read_last = 0.0;
			
			odom_pose.x = 0.0;
			odom_pose.y = 0.0;
			odom_pose.theta = 0.0;
			
			delta_odom_pose.x = 0.0;
			delta_odom_pose.y = 0.0;
			delta_odom_pose.theta = 0.0;
			
			b_reset_odom = false;
		}
		
		/**
		 * Reading encoder.
		 */
		read(i_fd, c_read_buf, sizeof(c_read_buf));
		str_data = c_read_buf; 

		/**
		 * Difference between current time and previous read time.
		 */
		dur_time = ros::Time::now() - read_time;

		/**
		 * Read time is set as current time for next usages.
		 */
		read_time = ros::Time::now();

		/**
		 * Parse left and right rpms.
		 */
		std::size_t found = str_data.find("_");
		if (found != std::string::npos)
		{
			f_left_read = atof(str_data.substr(0, found).c_str());
			f_right_read = atof(str_data.substr(found+1).c_str());
			str_data = "";
		}	

		/**
		 * Difference between current number of rotation and previous number of rotation is calculated.
		 */
		float f_delta_sr = 0.0, f_delta_sl = 0.0, f_delta_s = 0.0; 
		f_delta_sr =  PI * d_wheel_diameter * (f_right_read - f_right_read_last) / (i_gear_ratio * i_cpr);
		f_delta_sl =  PI * d_wheel_diameter * (f_left_read - f_left_read_last) / (i_gear_ratio * i_cpr);

		/**
		 * Difference between current orientation and previous orientation is calculated.
		 */
		delta_odom_pose.theta = (f_delta_sr - f_delta_sl) / d_wheel_separation;
		f_delta_s = (f_delta_sr + f_delta_sl) / 2;

		/**
		 * Displacement in x and y axes are calculated.
		 */
		delta_odom_pose.x = f_delta_s * cos(odom_pose.theta + delta_odom_pose.theta * 0.5);
		delta_odom_pose.y = f_delta_s * sin(odom_pose.theta + delta_odom_pose.theta * 0.5);

		/**
		 * Calculate new positions.
		 */
		odom_pose.x += delta_odom_pose.x;
		odom_pose.y += delta_odom_pose.y;
		odom_pose.theta += delta_odom_pose.theta;

		/**
		 * New values are set as last known values for next usages.
		 */
		f_right_read_last = f_right_read;
		f_left_read_last = f_left_read;

		/**
		 * ROS message content is filled
		 */
		pose_pub->msg_.pose.pose.position.x = odom_pose.x;
		pose_pub->msg_.pose.pose.position.y = odom_pose.y;
		pose_pub->msg_.pose.pose.position.z = (float)d_height;

		pose_pub->msg_.pose.pose.orientation.x = 0.0;
		pose_pub->msg_.pose.pose.orientation.y = 0.0;
		pose_pub->msg_.pose.pose.orientation.z = sin(0.5 * odom_pose.theta); 
		pose_pub->msg_.pose.pose.orientation.w = cos(0.5 * odom_pose.theta);

		/**
		 * Linear and angular velocity variables.
		 */
		float f_lin_vel = 0, f_ang_vel = 0;
		/**
		 * Filtered right and left wheel velocity variables.
		 */
		float f_lin_vel_right = 0, f_lin_vel_left = 0;
		/**
		 * Raw right and left wheel velocity variables.
		 */
		float f_raw_right_vel = 0.0, f_raw_left_vel = 0.0;

		ROS_DEBUG_STREAM("EvarobotOdometry: dur_time: " << dur_time.toSec());

		if(dur_time.toSec() > 0)
		{		
			/**
			 *  Raw right and left wheel velocities calculated.
			 */
			f_raw_right_vel = f_delta_sr / dur_time.toSec();
			f_raw_left_vel = f_delta_sl / dur_time.toSec();

			/**
			 * left and right wheel velocities are filtered with Low-Pass Filter.
			 */
			f_lin_vel_left = (1 - 0.35)*f_raw_left_vel + 0.35*f_lin_velL_pre;
			f_lin_velL_pre = f_lin_vel_left;

			f_lin_vel_right = (1 - 0.35)*f_raw_right_vel + 0.35*f_lin_velR_pre;
			f_lin_velR_pre = f_lin_vel_right;

			ROS_DEBUG_STREAM("EvarobotOdometry: VEl_LEFT: " << f_lin_vel_left << "  Vel_right: " << f_lin_vel_right << " dur: " << dur_time.toSec());
			
			/**
			 * Linear and angular velocities are calculated using left and right wheel velocities.
			 */
			f_lin_vel = (f_lin_vel_right + f_lin_vel_left) / 2.0;
			f_ang_vel = (f_lin_vel_right - f_lin_vel_left) / d_wheel_separation;
		}
		else
		{
			ROS_INFO(GetErrorDescription(-115).c_str());
			i_error_code = -115;
		}
		
		/**
		 * ROS message content is filled for wheel_vel topic.
		 */
		ss.str("");
		ss << n.resolveName(n.getNamespace(), true) << "/wheel_link";		
		vel_pub->msg_.header.frame_id = ss.str();
		vel_pub->msg_.header.stamp = ros::Time::now();
		vel_pub->msg_.left_vel = f_lin_vel_left;
		vel_pub->msg_.right_vel = f_lin_vel_right;

		/**
		 * wheel_vel topic is published.
		 */
		if (vel_pub->trylock())
		{
			vel_pub->unlockAndPublish();
		}			
		
		/**
		 * ROS message content is filled for odom topic.
		 */
		pose_pub->msg_.twist.twist.linear.x = f_lin_vel;
		pose_pub->msg_.twist.twist.linear.y = 0.0;
		pose_pub->msg_.twist.twist.linear.z = 0.0;
		pose_pub->msg_.twist.twist.angular.x = 0.0;
		pose_pub->msg_.twist.twist.angular.y = 0.0;
		pose_pub->msg_.twist.twist.angular.z = f_ang_vel;
		pose_pub->msg_.header.stamp = ros::Time::now();
		ss.str("");
		ss << n.resolveName(n.getNamespace(), true) << "/odom";
		pose_pub->msg_.header.frame_id = ss.str();
		ss.str("");
		ss << n.resolveName(n.getNamespace(), true) << "/base_link";
		pose_pub->msg_.child_frame_id = ss.str();
		
		/**
		 * odom topic is published.
		 */
		if (pose_pub->trylock())
		{
			pose_pub->unlockAndPublish();
		}			
		pub_freq.tick();

		/**
		 * Loop is slept to hold frequency.
		 */
		ros::spinOnce();
		updater.update();
		loop_rate.sleep();
	}

	/**
	 * Closing the encoder file.
	 */
	close(i_fd);

	return 0;
}
