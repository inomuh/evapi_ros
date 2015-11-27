#include "evarobot_odometry/evarobot_odometry.h"

#include <dynamic_reconfigure/server.h>
#include <evarobot_odometry/ParamsConfig.h>

bool b_is_received_params = false;

bool b_always_on;
bool b_reset_odom = false;
double d_wheel_separation;
double d_height;
int i_gear_ratio;
int i_cpr;
double d_wheel_diameter;


	
void CallbackReconfigure(evarobot_odometry::ParamsConfig &config, uint32_t level)
{
   b_is_received_params = true;        
   
   b_always_on = config.alwaysOn;
   d_wheel_separation = config.wheelSeparation;
   d_height = config.height;
   i_gear_ratio = config.gearRatio;
   i_cpr = config.CPR;
   d_wheel_diameter = config.wheelDiameter;
}

bool CallbackResetOdom(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	ROS_INFO("Reset Odometry");
	b_reset_odom = true;
	return true;
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
	n.param("evarobot_odometry/alwaysOn", b_always_on, false);
	n.param("evarobot_odometry/minFreq", d_min_freq, 0.2);
	n.param("evarobot_odometry/maxFreq", d_max_freq, 10.0);
	
	if(!n.getParam("evarobot_odometry/wheelSeparation", d_wheel_separation))
	{
	  ROS_ERROR("Failed to get param 'wheelSeparation'");
	}
	
	if(!n.getParam("evarobot_odometry/height", d_height))
	{
	  ROS_ERROR("Failed to get param 'height'");
	} 	
	
	if(!n.getParam("evarobot_odometry/frequency", d_frequency))
	{
	  ROS_ERROR("Failed to get param 'frequency'");
	} 	
	
	if(!n.getParam("evarobot_odometry/gearRatio", i_gear_ratio))
	{
	  ROS_ERROR("Failed to get param 'gearRatio'");
	} 	
	
	if(!n.getParam("evarobot_odometry/CPR", i_cpr))
	{
	  ROS_ERROR("Failed to get param 'CPR'");
	} 	
	
	if(!n.getParam("evarobot_odometry/wheelDiameter", d_wheel_diameter))
	{
	  ROS_ERROR("Failed to get param 'wheelDiameter'");
	}
		
	ros::Publisher pose_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
	ros::Publisher vel_pub = n.advertise<geometry_msgs::PointStamped>("wheel_vel", 10);
	
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
	updater.setHardwareID("None");
		
	diagnostic_updater::HeaderlessTopicDiagnostic pub_freq("odom", updater,
											diagnostic_updater::FrequencyStatusParam(&d_min_freq, &d_max_freq, 0.1, 10));
    
	char c_read_buf[100], c_write_buf[100];
	

	nav_msgs::Odometry msg;
	geometry_msgs::PointStamped wheel_vel;
	geometry_msgs::Pose2D odom_pose;
	geometry_msgs::Pose2D delta_odom_pose;

	string str_data;

	float f_left_read = 0.0, f_right_read = 0.0;
	float f_left_read_last = 0.0, f_right_read_last = 0.0; 

										
	float covariance[36] =	{1, 0, 0, 0, 0, 0,  // covariance on gps_x
										0, 1, 0, 0, 0, 0,  // covariance on gps_y
										0, 0, 1, 0, 0, 0,  // covariance on gps_z
										0, 0, 0, 1, 0, 0,  // large covariance on rot x
										0, 0, 0, 0, 1, 0,  // large covariance on rot y
										0, 0, 0, 0, 0, 1};  // large covariance on rot z	
	for(int i = 0; i < 36; i++)
	{
		msg.pose.covariance[i] = covariance[i];
	}		
	
	// Opening the encoder.
	i_fd = open(str_device_path.c_str(), O_RDWR);
	
	// Cleaning the encoder.
	//    write(fd, write_buf, sizeof(write_buf));
	write(i_fd, "clean", 5);
	
	
	if(i_fd < 0)
	{
		ROS_ERROR("File %s either does not exist or has been locked by another process\n", str_device_path.c_str());
		exit(-1);
	}

	while (ros::ok())
	{
		if(b_is_received_params)
		{
			ROS_INFO("Updating Odometry Params...");
			b_is_received_params = false;
		}
		
		if(b_reset_odom)
		{
			ROS_INFO("Resetting Odometry");
			
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
		
//		cout << str_data << endl;

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

//		cout << "left_read: " << f_left_read << " right_read: " << f_right_read << endl;

		float f_delta_sr = 0.0, f_delta_sl = 0.0, f_delta_s = 0.0; 

		// Dönüş sayısı değişimi hesaplanıyor.
		f_delta_sr =  PI * d_wheel_diameter * (f_right_read - f_right_read_last) / (i_gear_ratio * i_cpr);
		f_delta_sl =  PI * d_wheel_diameter * (f_left_read - f_left_read_last) / (i_gear_ratio * i_cpr);

//		cout << "deltaSr: " << f_delta_sr << " deltaSl: " << f_delta_sl << endl;

		// Oryantasyondaki değişim hesaplanıyor.
		delta_odom_pose.theta = (f_delta_sr - f_delta_sl) / d_wheel_separation;
		f_delta_s = (f_delta_sr + f_delta_sl) / 2;

//		cout << "delta_teta: " << delta_odom_pose.theta << " deltaS: " << f_delta_s << endl;

		// x ve y eksenlerindeki yer değiştirme hesaplanıyor.
		delta_odom_pose.x = f_delta_s * cos(odom_pose.theta + delta_odom_pose.theta * 0.5);
		delta_odom_pose.y = f_delta_s * sin(odom_pose.theta + delta_odom_pose.theta * 0.5);

//		cout << "delta_odom x: " <<  delta_odom_pose.x << " y: " <<  delta_odom_pose.y << endl;

		// Yeni pozisyonlar hesaplanıyor.
		odom_pose.x += delta_odom_pose.x;
		odom_pose.y += delta_odom_pose.y;
		odom_pose.theta += delta_odom_pose.theta;

		// Yeni dönüş değerleri son okunan olarak atanıyor.
		f_right_read_last = f_right_read;
		f_left_read_last = f_left_read;

	//	cout << "read_last ri: " << f_right_read_last << " le: " << f_left_read_last << endl;
	
			
		// Yayınlanacak Posizyon Verisi dolduruluyor.
		msg.pose.pose.position.x = odom_pose.x;
		msg.pose.pose.position.y = odom_pose.y;
		msg.pose.pose.position.z = (float)d_height;

		msg.pose.pose.orientation.x = 0.0;
		msg.pose.pose.orientation.y = 0.0;
		msg.pose.pose.orientation.z = sin(0.5 * odom_pose.theta); 
		msg.pose.pose.orientation.w = cos(0.5 * odom_pose.theta);
		
													
		//cout << "Pose.X: " << odom_pose.x << "  Pose.Y: " << odom_pose.y << "  Orientation: " << odom_pose.theta << endl;

		float f_lin_vel = 0, f_ang_vel = 0;
		float f_lin_vel_right = 0, f_lin_vel_left = 0;

//		cout << "dur_time: " << dur_time.toSec() << endl;

		if(dur_time.toSec() > 0)
		{
//			f_lin_vel = sqrt(pow(delta_odom_pose.x, 2) + pow(delta_odom_pose.y, 2)) / dur_time.toSec();
//			f_ang_vel = delta_odom_pose.theta / dur_time.toSec();
			
			f_lin_vel_right = f_delta_sr / dur_time.toSec();
			f_lin_vel_left = f_delta_sl / dur_time.toSec();
			
//			cout << "VEl_LEFT: " << f_lin_vel_left << "  Vel_right: " << f_lin_vel_right << " dur: " << dur_time.toSec() << endl;

			
			f_lin_vel = (f_lin_vel_right + f_lin_vel_left) / 2.0;
			f_ang_vel = (f_lin_vel_right - f_lin_vel_left) / d_wheel_separation;

		}
		else
		{
			ROS_ERROR("Division by Zero");
		}
		
		ss.str("");
		ss << n.resolveName(n.getNamespace(), true) << "/wheel_link";
		
		wheel_vel.header.frame_id = ss.str();
		wheel_vel.header.stamp = ros::Time::now();
		wheel_vel.point.x = f_lin_vel_left;
		wheel_vel.point.y = f_lin_vel_right;
		
		if(vel_pub.getNumSubscribers() > 0 || b_always_on)
		{
			vel_pub.publish(wheel_vel);
		}
		
		// Yayınlacak Hız Verisi dolduruluyor.
		msg.twist.twist.linear.x = f_lin_vel;
		msg.twist.twist.linear.y = 0.0;
		msg.twist.twist.linear.z = 0.0;

		msg.twist.twist.angular.x = 0.0;
		msg.twist.twist.angular.y = 0.0;
		msg.twist.twist.angular.z = f_ang_vel;

		//uint32
		//msg.header.seq
		// ROS Zaman etiketi topiğe basılıyor. (time)
		msg.header.stamp = ros::Time::now();

		// Odometry verisinin frame id'si yazılıyor. (string)
		ss.str("");
		ss << n.resolveName(n.getNamespace(), true) << "/odom";
		msg.header.frame_id = ss.str();
		
		ss.str("");
		ss << n.resolveName(n.getNamespace(), true) << "/base_link";
		msg.child_frame_id = ss.str();
		
		// Veri topikten basılıyor.
		if(pose_pub.getNumSubscribers() > 0 || b_always_on)
		{
			pose_pub.publish(msg);
			pub_freq.tick();
		}

		ros::spinOnce();
		updater.update();
		// Frekansı tutturmak için uyutuluyor.
		loop_rate.sleep();
	}

	// Enkoder sürücü dosyası kapatılıyor.
	close(i_fd);

	return 0;
}
