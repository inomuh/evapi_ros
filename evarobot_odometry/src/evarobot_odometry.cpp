#include "evarobot_odometry/evarobot_odometry.h"

#include <dynamic_reconfigure/server.h>
#include <evarobot_odometry/ParamsConfig.h>

bool b_is_received_params = false;

bool b_reset_odom = false;
double d_wheel_separation;
double d_height;
int i_gear_ratio;
int i_cpr;
double d_wheel_diameter;

int i_error_code = 0;
	
void CallbackReconfigure(evarobot_odometry::ParamsConfig &config, uint32_t level)
{
   b_is_received_params = true;        
   
   d_wheel_separation = config.wheelSeparation;
   d_height = config.height;
   i_gear_ratio = config.gearRatio;
   i_cpr = config.CPR;
   d_wheel_diameter = config.wheelDiameter;
}

bool CallbackResetOdom(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	ROS_DEBUG("EvarobotOdometry: Reset Odometry");
	b_reset_odom = true;
	return true;
}



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

int main(int argc, char **argv)
{
	int i_fd;
	double d_frequency;
	double d_min_freq, d_max_freq;
	stringstream ss;
	
	string str_device_path;
	
	ros::init(argc, argv, "/evarobot_odometry");
	ros::NodeHandle n;

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
		
	
  realtime_tools::RealtimePublisher<nav_msgs::Odometry> * pose_pub = new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(n, "odom", 10);
  realtime_tools::RealtimePublisher<im_msgs::WheelVel> * vel_pub = new realtime_tools::RealtimePublisher<im_msgs::WheelVel>(n, "wheel_vel", 10);
  realtime_tools::RealtimePublisher<im_msgs::WheelVel> * rotation_pub = new realtime_tools::RealtimePublisher<im_msgs::WheelVel>(n, "rotation", 10);


	
	ros::ServiceServer service = n.advertiseService("evarobot_odometry/reset_odom", CallbackResetOdom);
	
	ros::Rate loop_rate(d_frequency);

	ros::Time read_time = ros::Time::now();
	ros::Duration dur_time;

  // Dynamic Reconfigure
  dynamic_reconfigure::Server<evarobot_odometry::ParamsConfig> srv;
  dynamic_reconfigure::Server<evarobot_odometry::ParamsConfig>::CallbackType f;
  f = boost::bind(&CallbackReconfigure, _1, _2);
  srv.setCallback(f);
  ///////////////
  
  // Diagnostics
	diagnostic_updater::Updater updater;
	updater.setHardwareID("none");
	updater.add("evarobot_odometry", &ProduceDiagnostics);
		
	diagnostic_updater::HeaderlessTopicDiagnostic pub_freq("odom", updater,
											diagnostic_updater::FrequencyStatusParam(&d_min_freq, &d_max_freq, 0.1, 10));
    
	char c_read_buf[31], c_write_buf[100];
	

	geometry_msgs::Pose2D odom_pose;
	geometry_msgs::Pose2D delta_odom_pose;

	string str_data;

	float f_left_read = 0.0, f_right_read = 0.0;
	float f_left_read_last = 0.0, f_right_read_last = 0.0; 

										
	float covariance[36] =	{0.01, 0, 0, 0, 0, 0,  // covariance on gps_x
										0, 0.01, 0, 0, 0, 0,  // covariance on gps_y
										0, 0, 99999, 0, 0, 0,  // covariance on gps_z
										0, 0, 0, 99999, 0, 0,  // large covariance on rot x
										0, 0, 0, 0, 99999, 0,  // large covariance on rot y
										0, 0, 0, 0, 0, 0.01};  // large covariance on rot z	
	for(int i = 0; i < 36; i++)
	{
		pose_pub->msg_.pose.covariance[i] = covariance[i];
	}		
	
	// Opening the encoder.
	i_fd = open(str_device_path.c_str(), O_RDWR);
	
	// Cleaning the encoder.
	//    write(fd, write_buf, sizeof(write_buf));
	write(i_fd, "clean", 5);
	
	
	if(i_fd < 0)
	{
		ROS_INFO(GetErrorDescription(-114).c_str());
        i_error_code = -114;
		exit(-1);
	}

        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;


	while (ros::ok())
	{
		if(b_is_received_params)
		{
			ROS_DEBUG("EvarobotOdometry: Updating Odometry Params...");
			b_is_received_params = false;
		}
		
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
		
		// Reading encoder.
		read(i_fd, c_read_buf, sizeof(c_read_buf));
		str_data = c_read_buf; 

		dur_time = ros::Time::now() - read_time;

		read_time = ros::Time::now();

		// Parse left and right rpms.
		std::size_t found = str_data.find("_");
		if (found != std::string::npos)
		{
			f_left_read = atof(str_data.substr(0, found).c_str());
			f_right_read = atof(str_data.substr(found+1).c_str());
			str_data = "";
		}	

		float f_delta_sr = 0.0, f_delta_sl = 0.0, f_delta_s = 0.0; 
		float f_rotvel_left = 0.0, f_rotvel_right = 0.0;

		// Dönüş sayısı değişimi hesaplanıyor.
		f_delta_sr =  PI * d_wheel_diameter * (f_right_read - f_right_read_last) / (i_gear_ratio * i_cpr);
		f_delta_sl =  PI * d_wheel_diameter * (f_left_read - f_left_read_last) / (i_gear_ratio * i_cpr);

		f_rotvel_left = 2*PI * (f_left_read - f_left_read_last) / (i_gear_ratio * i_cpr);
		f_rotvel_right = 2*PI * (f_right_read - f_right_read_last) / (i_gear_ratio * i_cpr);

		// Oryantasyondaki değişim hesaplanıyor.
		delta_odom_pose.theta = (f_delta_sr - f_delta_sl) / d_wheel_separation;
		f_delta_s = (f_delta_sr + f_delta_sl) / 2;

		// x ve y eksenlerindeki yer değiştirme hesaplanıyor.
		delta_odom_pose.x = f_delta_s * cos(odom_pose.theta + delta_odom_pose.theta * 0.5);
		delta_odom_pose.y = f_delta_s * sin(odom_pose.theta + delta_odom_pose.theta * 0.5);

		// Calculate new positions.
		odom_pose.x += delta_odom_pose.x;
		odom_pose.y += delta_odom_pose.y;
		odom_pose.theta += delta_odom_pose.theta;

		// Yeni dönüş değerleri son okunan olarak atanıyor.
		f_right_read_last = f_right_read;
		f_left_read_last = f_left_read;

			
		// Yayınlanacak Posizyon Verisi dolduruluyor.
		pose_pub->msg_.pose.pose.position.x = odom_pose.x;
		pose_pub->msg_.pose.pose.position.y = odom_pose.y;
		pose_pub->msg_.pose.pose.position.z = (float)d_height;


		roll = 0.0;
		pitch = 0.0;
		yaw = odom_pose.theta;
			
		pose_pub->msg_.pose.pose.orientation.x = sin(roll*0.5) * cos(pitch*0.5) * cos(yaw*0.5) - cos(roll*0.5) * sin(pitch*0.5) * sin(yaw*0.5);
		pose_pub->msg_.pose.pose.orientation.y = cos(roll*0.5) * sin(pitch*0.5) * cos(yaw*0.5) + sin(roll*0.5) * cos(pitch*0.5) * sin(yaw*0.5);
		pose_pub->msg_.pose.pose.orientation.z = cos(roll*0.5) * cos(pitch*0.5) * sin(yaw*0.5) - sin(roll*0.5) * sin(pitch*0.5) * cos(yaw*0.5);
		pose_pub->msg_.pose.pose.orientation.w = cos(roll*0.5) * cos(pitch*0.5) * cos(yaw*0.5) + sin(roll*0.5) * sin(pitch*0.5) * sin(yaw*0.5);


//		pose_pub->msg_.pose.pose.orientation.x = 0.0;
//		pose_pub->msg_.pose.pose.orientation.y = 0.0;
//		pose_pub->msg_.pose.pose.orientation.z = sin(0.5 * odom_pose.theta); 
//		pose_pub->msg_.pose.pose.orientation.w = cos(0.5 * odom_pose.theta);

		float f_lin_vel = 0, f_ang_vel = 0;
		float f_lin_vel_right = 0, f_lin_vel_left = 0;

		ROS_DEBUG_STREAM("EvarobotOdometry: dur_time: " << dur_time.toSec());

		if(dur_time.toSec() > 0)
		{		
			f_lin_vel_right = f_delta_sr / dur_time.toSec();
			f_lin_vel_left = f_delta_sl / dur_time.toSec();
			
			ROS_DEBUG_STREAM("EvarobotOdometry: VEl_LEFT: " << f_lin_vel_left << "  Vel_right: " << f_lin_vel_right << " dur: " << dur_time.toSec());

			f_rotvel_left /= dur_time.toSec();
	                f_rotvel_right /= dur_time.toSec();
			
			f_lin_vel = (f_lin_vel_right + f_lin_vel_left) / 2.0;
			f_ang_vel = (f_lin_vel_right - f_lin_vel_left) / d_wheel_separation;
		}
		else
		{
			ROS_INFO(GetErrorDescription(-115).c_str());
			i_error_code = -115;
		}
		
		ss.str("");
		ss << n.resolveName(n.getNamespace(), true) << "/wheel_link";
		
		vel_pub->msg_.header.frame_id = ss.str();
		vel_pub->msg_.header.stamp = ros::Time::now();
		vel_pub->msg_.left_vel = f_lin_vel_left;
		vel_pub->msg_.right_vel = f_lin_vel_right;

		rotation_pub->msg_.header.frame_id = ss.str();
		rotation_pub->msg_.header.stamp = ros::Time::now();
                rotation_pub->msg_.left_vel = f_rotvel_left;
                rotation_pub->msg_.right_vel = f_rotvel_right;

		if (vel_pub->trylock())
		{
			vel_pub->unlockAndPublish();
		}			
		
		if (rotation_pub->trylock())
                {
                        rotation_pub->unlockAndPublish();
                }

		// Yayınlacak Hız Verisi dolduruluyor.
		pose_pub->msg_.twist.twist.linear.x = f_lin_vel;
		pose_pub->msg_.twist.twist.linear.y = 0.0;
		pose_pub->msg_.twist.twist.linear.z = 0.0;

		pose_pub->msg_.twist.twist.angular.x = 0.0;
		pose_pub->msg_.twist.twist.angular.y = 0.0;
		pose_pub->msg_.twist.twist.angular.z = f_ang_vel;

		//uint32
		//msg.header.seq
		// ROS Zaman etiketi topiğe basılıyor. (time)
		pose_pub->msg_.header.stamp = ros::Time::now();

		// Odometry verisinin frame id'si yazılıyor. (string)
		ss.str("");
		ss << n.resolveName(n.getNamespace(), true) << "/odom";
		pose_pub->msg_.header.frame_id = ss.str();
		
		ss.str("");
		ss << n.resolveName(n.getNamespace(), true) << "/base_link";
		pose_pub->msg_.child_frame_id = ss.str();
		
		// Veri topikten basılıyor.
		if (pose_pub->trylock())
		{
			pose_pub->unlockAndPublish();
		}			
		pub_freq.tick();

		ros::spinOnce();
		updater.update();
		// Frekansı tutturmak için uyutuluyor.
		loop_rate.sleep();
	}

	// Enkoder sürücü dosyası kapatılıyor.
	close(i_fd);

	return 0;
}
