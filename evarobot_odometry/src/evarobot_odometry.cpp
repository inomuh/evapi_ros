#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose2D.h"

#include <string>    
#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <math.h>

#define PI 3.1416

using namespace std;

int main(int argc, char **argv)
{
	double d_wheel_separation;
	double d_height;
	double d_frequency;
	double d_wheel_diameter;
	int i_gear_ratio;
	int i_cpr;
	
	string str_device_path;
	string str_frame_id;
	string str_topic;
	
	bool b_always_on;
	
	ros::init(argc, argv, "evarobot_odometry");
	ros::NodeHandle n;

	n.param<string>("evarobot_odometry/devicePath", str_device_path, "/dev/evarobotEncoder");
	n.param<string>("evarobot_odometry/odomTopic", str_topic, "odom");
	n.param<string>("evarobot_odometry/odomFrame", str_frame_id, "odom");
	
	n.param("evarobot_odometry/alwaysOn", b_always_on, false);

	
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
	

	

	ros::Publisher pose_pub = n.advertise<nav_msgs::Odometry>(str_topic.c_str(), 10);

	ros::Rate loop_rate(d_frequency);

	ros::Time read_time = ros::Time::now();
	ros::Duration dur_time;


	char c_read_buf[100], c_write_buf[100];
	int i_fd;

	nav_msgs::Odometry msg;
	geometry_msgs::Pose2D odom_pose;
	geometry_msgs::Pose2D delta_odom_pose;

	string str_data;

	float f_left_read = 0.0, f_right_read = 0.0;
	float f_left_read_last = 0.0, f_right_read_last = 0.0; 

	
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

		cout << "left_read: " << f_left_read << " right_read: " << f_right_read << endl;

		float f_delta_sr, f_delta_sl, f_delta_s; 

		// Dönüş sayısı değişimi hesaplanıyor.
		f_delta_sr =  PI * d_wheel_diameter * (f_right_read - f_right_read_last) / (i_gear_ratio * i_cpr);
		f_delta_sl =  PI * d_wheel_diameter * (f_left_read - f_left_read_last) / (i_gear_ratio * i_cpr);

		cout << "deltaSr: " << f_delta_sr << " deltaSl: " << f_delta_sl << endl;

		// Oryantasyondaki değişim hesaplanıyor.
		delta_odom_pose.theta = (f_delta_sr - f_delta_sl) / d_wheel_separation;
		f_delta_s = (f_delta_sr + f_delta_sl) / 2;

		cout << "delta_teta: " << delta_odom_pose.theta << " deltaS: " << f_delta_s << endl;

		// x ve y eksenlerindeki yer değiştirme hesaplanıyor.
		delta_odom_pose.x = f_delta_s * cos(odom_pose.theta + delta_odom_pose.theta * 0.5);
		delta_odom_pose.y = f_delta_s * sin(odom_pose.theta + delta_odom_pose.theta * 0.5);

		cout << "delta_odom x: " <<  delta_odom_pose.x << " y: " <<  delta_odom_pose.y << endl;

		// Yeni pozisyonlar hesaplanıyor.
		odom_pose.x += delta_odom_pose.x;
		odom_pose.y += delta_odom_pose.y;
		odom_pose.theta += delta_odom_pose.theta;

		// Yeni dönüş değerleri son okunan olarak atanıyor.
		f_right_read_last = f_right_read;
		f_left_read_last = f_left_read;

		cout << "read_last ri: " << f_right_read_last << " le: " << f_left_read_last << endl;
			
		// Yayınlanacak Posizyon Verisi dolduruluyor.
		msg.pose.pose.position.x = odom_pose.x;
		msg.pose.pose.position.y = odom_pose.y;
		msg.pose.pose.position.z = (float)d_height;

		msg.pose.pose.orientation.x = 0.0;
		msg.pose.pose.orientation.y = 0.0;
		msg.pose.pose.orientation.z = sin(odom_pose.theta); 
		msg.pose.pose.orientation.w = cos(odom_pose.theta);

		float f_lin_vel = 0, f_ang_vel = 0;

		cout << "dur_time: " << dur_time.toSec() << endl;

		if(dur_time.toSec() > 0)
		{
			f_lin_vel = sqrt(pow(delta_odom_pose.x, 2) + pow(delta_odom_pose.y, 2)) / dur_time.toSec();
			f_ang_vel = delta_odom_pose.theta / dur_time.toSec();
		}
		else
		{
			ROS_ERROR("Division by Zero");
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
		msg.header.frame_id = str_frame_id;

		// Veri topikten basılıyor.
		if(pose_pub.getNumSubscribers() > 0 || b_always_on)
		{
			pose_pub.publish(msg);
		}

		ros::spinOnce();

		// Frekansı tutturmak için uyutuluyor.
		loop_rate.sleep();
	}

	// Enkoder sürücü dosyası kapatılıyor.
	close(i_fd);

	return 0;
}
