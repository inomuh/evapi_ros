#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#include <vector>
#include <sstream>

#include "IMADC.h"

//#ifndef DEBUG
//#define DEBUG
//#endif

#define MAX_IR 7

using namespace std;

int main(int argc, char **argv)
{
	unsigned char u_c_spi_mode;
	
	// ROS PARAMS
	vector<int> T_i_channels;
	vector<double> T_d_posX, T_d_posY, T_d_posZ, T_d_angle_yaw;
	vector<double> T_d_min_ranges, T_d_max_ranges;
	vector<double> T_d_field_of_views;
	
	vector<double> T_d_par_k, T_d_par_a, T_d_par_b;
		
	double d_frequency;
	
	string str_driver_path;
	string str_namespace;
	string str_frame_id;
	
	int i_spi_mode;
	int i_spi_speed;
	int i_spi_bits;
	int i_adc_bits;
	
	bool b_always_on;
	// rosparams end
	
	vector<ros::Publisher> T_pub_ir;
	vector<sensor_msgs::Range> T_irs;
	
	ros::init(argc, argv, "evarobot_infrared");
	ros::NodeHandle n;
	
	n.param<string>("evarobot_infrared/driverPath", str_driver_path, "/dev/spidev0.0");
	n.param<string>("evarobot_infrared/namespace", str_namespace, "gazebo");
	n.param<string>("evarobot_infrared/frameNamespace", str_frame_id, "infrared");
	n.param("evarobot_infrared/alwaysOn", b_always_on, false);
	n.param("evarobot_infrared/spiMode", i_spi_mode, 0);
	n.param("evarobot_infrared/spiSpeed", i_spi_speed, 1000000);
	n.param("evarobot_infrared/spiBits", i_spi_bits, 8);
	n.param("evarobot_infrared/adcBits", i_adc_bits, 12);
	

		
	if(!n.getParam("evarobot_infrared/frequency", d_frequency))
	{
		ROS_ERROR("Failed to get param 'frequency'");
	} 

	if(i_adc_bits > 16)
	{
		ROS_ERROR("adcBits mustn't be greater than 16");
	}
	
	n.getParam("evarobot_infrared/parK", T_d_par_k); // 0.42
	n.getParam("evarobot_infrared/parB", T_d_par_b); // -0.0013461
	n.getParam("evarobot_infrared/parA", T_d_par_a); // 0.0000473

	n.getParam("evarobot_infrared/field_of_view", T_d_field_of_views);
	n.getParam("evarobot_infrared/minRange", T_d_min_ranges);
	n.getParam("evarobot_infrared/maxRange", T_d_max_ranges);

	n.getParam("evarobot_infrared/channelNo", T_i_channels);
		
	n.getParam("evarobot_infrared/posX", T_d_posX);
	n.getParam("evarobot_infrared/posY", T_d_posY);
	n.getParam("evarobot_infrared/posZ", T_d_posZ);
	n.getParam("evarobot_infrared/angleYaw", T_d_angle_yaw);
	
	if( !(T_d_par_k.size() == T_d_par_a.size()
	&& T_d_par_a.size() == T_d_par_b.size()
	&& T_d_par_b.size() == T_d_field_of_views.size()
	&& T_d_field_of_views.size() == T_d_min_ranges.size()
	&& T_d_min_ranges.size() == T_d_max_ranges.size()
	&& T_d_min_ranges.size() == T_i_channels.size()
	&& T_i_channels.size() == T_d_posX.size()
	&& T_d_posX.size() == T_d_posY.size()
	&& T_d_posY.size() == T_d_posZ.size()
	&& T_d_posZ.size() == T_d_angle_yaw.size())	)
	{
		ROS_ERROR("Size of params are not same.");
	}
	else if(T_i_channels.size() > MAX_IR)
	{
		ROS_ERROR("Number of infrared sensors mustn't be greater than %d", MAX_IR);
	}
	
	#ifdef DEBUG
		ROS_INFO("Number of ir sensors: %d", T_d_posZ.size());
	#endif
	
	
	
	// Set publishers
	for(uint i = 0; i < T_i_channels.size(); i++)
	{
		stringstream ss_topic;
		ss_topic << str_namespace << "/ir" << i;
		ros::Publisher dummy_publisher = n.advertise<sensor_msgs::Range>(ss_topic.str().c_str(), 10);
		T_pub_ir.push_back(dummy_publisher);
	}

	// Define frequency
	ros::Rate loop_rate(d_frequency);

	
	
	// init sonar variables
	for(uint i = 0; i < T_i_channels.size(); i++)
	{
		
		stringstream ss_frame;
		ss_frame << str_frame_id << "/ir" << i;
		
		sensor_msgs::Range dummy_ir;
		
		dummy_ir.header.frame_id = ss_frame.str();
		dummy_ir.radiation_type = 1;
		dummy_ir.field_of_view = T_d_field_of_views[i];
		dummy_ir.min_range = T_d_min_ranges[i];
		dummy_ir.max_range = T_d_max_ranges[i];
		
		T_irs.push_back(dummy_ir);
	}
	
	switch(i_spi_mode)
	{
		case 0:
		{
			u_c_spi_mode = SPI_MODE_0;
			break;
		}
		
		case 1:
		{
			u_c_spi_mode = SPI_MODE_1;
			break;
		}
		
		case 2:
		{
			u_c_spi_mode = SPI_MODE_2;
			break;
		}
		
		case 3:
		{
			u_c_spi_mode = SPI_MODE_3;
			break;
		}
		
		default:
		{
			ROS_ERROR("Wrong spiMode. It should be 0-3.");
		}
	}
	
	// Creating spi and adc objects.
	IMSPI * p_im_spi = new IMSPI(str_driver_path, u_c_spi_mode, i_spi_speed, i_spi_bits);
	IMADC * p_im_adc = new IMADC(p_im_spi, i_adc_bits);
	
	while(ros::ok())
	{		
		for(uint i = 0; i < T_pub_ir.size(); i++)
		{
			// Read sensors
			int i_raw_data = 0;
			
			i_raw_data = p_im_adc->ReadChannel(T_i_channels[i]);
			
			#ifdef DEBUG
				ROS_INFO("Raw data %f \n", (double)i_raw_data);
				ROS_INFO("Par_A %f \n", T_d_par_a[i]);
				ROS_INFO("Par_B %f \n", T_d_par_b[i]);
				ROS_INFO("Par_K %f \n", T_d_par_k[i]);
			#endif
			
			
			T_irs[i].range = 0.01 * ((1 / (T_d_par_a[i] * (double)i_raw_data + T_d_par_b[i])) - T_d_par_k[i]);
			T_irs[i].header.stamp = ros::Time::now();
					
			// Publish Data
			if(T_pub_ir[i].getNumSubscribers() > 0 || b_always_on)
			{
				T_pub_ir[i].publish(T_irs[i]);
			}
			
		}
		
		loop_rate.sleep();	
	
	}
	
	return 0;
}

