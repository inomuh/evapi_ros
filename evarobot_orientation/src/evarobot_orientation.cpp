//! Bu sınıfa ait dokümantasyon evapi_ros 85rpm altındaki dokümantasyon ile aynıdır.

#include "evarobot_orientation/evarobot_orientation.h"

int i_error_code = 0;

void ParseRegisters(int32_t i_register_data, int & i_data_l, int & i_data_h)
{
	IMUM6::UNION32 union32;

	union32.i = 0x00000000;
	union32.i |= (((signed int)i_register_data & 0x000000FF) << 8) | (((signed int)i_register_data & 0x0000FF00) >> 8);
	i_data_l = union32.i;
	
	union32.i = 0x00000000;
	union32.i |= ((((signed int)i_register_data >> 16) & 0x000000FF) << 8) | ((((signed int)i_register_data >> 16) & 0x0000FF00) >> 8);
	i_data_h = union32.i;
}

void ParseRegisters(int32_t i_register_data, int & i_data_h)
{
	IMUM6::UNION32 union32;
	
	union32.i = 0x00000000;
	union32.i |= ((((signed int)i_register_data >> 16) & 0x000000FF) << 8) | ((((signed int)i_register_data >> 16) & 0x0000FF00) >> 8);
	i_data_h = union32.i;
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
	unsigned char u_c_spi_mode;
	tcflag_t parity;
	tcflag_t parity_on;
	
	vector<int> T_i_registers;
	
	IMUM6::UM6_DATA data;
	
	// ROS PARAMS
	
	double d_frequency;
		
	bool b_always_on;
	// rosparams end
	
	ros::Publisher pub_um6;
	sensor_msgs::Imu imu_packet;
		
	ros::init(argc, argv, "/evarobot_orientation");
	ros::NodeHandle n;
	
	n.param("evarobot_orientation/alwaysOn", b_always_on, false);
			
	if(!n.getParam("evarobot_orientation/frequency", d_frequency))
	{
		//ROS_ERROR("Failed to get param 'frequency'");
		ROS_INFO(GetErrorDescription(-116).c_str());
        i_error_code = -116;
	} 
	
	
	// Set publisher
	pub_um6 = n.advertise<sensor_msgs::Imu>("imu", 1);
	#ifdef TEST
	ros::Publisher pub_pose_demo = n.advertise<nav_msgs::Odometry>("odom_demo", 1);
	#endif

	// Define frequency
	ros::Rate loop_rate(d_frequency);

	stringstream ss_frame;
	ss_frame << n.resolveName(n.getNamespace(), true) << "/imu_link";

	// init imu packets
	imu_packet.header.frame_id = ss_frame.str();


	
	switch(PARITY)
	{
		case 0:
		{
			parity = 0;
			parity_on = 0;
			break;
		}
		
		case 1:
		{
			parity = PARODD;
			parity_on = PARENB;
			break;
		}
		
		case 2:
		{
			parity = ~PARODD;
			parity_on = PARENB;
			break;
		}
		
		default:
		{
			//ROS_ERROR("Wrong Parity Mode. It should be 0-2.");
			ROS_INFO(GetErrorDescription(-117).c_str());
			i_error_code = -117;
		}
	}
	
	// ROS PARAMS
    double d_min_freq = 0.2;
	double d_max_freq = 10.0;

	// Diagnostics
	diagnostic_updater::Updater updater;
	updater.setHardwareID("EvarobotOrientation");
	updater.add("orientation", &ProduceDiagnostics);

	diagnostic_updater::HeaderlessTopicDiagnostic pub_freq("orientation", updater,
            diagnostic_updater::FrequencyStatusParam(&d_min_freq, &d_max_freq, 0.1, 10));
	
	IMSerial * p_im_serial;
	IMUM6 * p_im_um6;
	try{
		// Creating serial and um6 objects.
		IMSerial * p_im_serial = new IMSerial(SERIAL_PATH, BAUDRATE, DATABITS, parity, parity_on, STOP_BITS);
		IMUM6 * p_im_um6 = new IMUM6(p_im_serial);
	}catch(int e){
		ROS_INFO(GetErrorDescription(e).c_str());
		i_error_code = e;
	}
	

	while(ros::ok())
	{
		try{		
			if(p_im_um6->GetRawData(data))
			{
			
				if(!p_im_um6->CheckData()){
					//ROS_ERROR("Checksum error in serial communication\n");
					ROS_INFO(GetErrorDescription(-118).c_str());
					i_error_code = -118;
				}

				
				p_im_um6->ProcessData(T_i_registers);
				
				#ifdef DEBUG
				
				ROS_DEBUG("EvarobotOdometry: Read Registers: \n");
				for(uint i = 0; i < T_i_registers.size(); i++)
				{
					ROS_DEBUG("EvarobotOdometry: %x_", T_i_registers[i]);
					ROS_DEBUG("EvarobotOdometry: ::%x_", p_im_um6->GetDataRegister(T_i_registers[i]) );
				}
				
				
				#endif
				
				geometry_msgs::Quaternion orientation;
				geometry_msgs::Vector3 linear_acceleration;
				
				#ifdef TEST
				nav_msgs::Odometry pose_demo;
				#endif
				
				int i_orientation_x, i_orientation_y, i_orientation_z, i_orientation_w;
				int i_linear_acceleration_x, i_linear_acceleration_y, i_linear_acceleration_z;
			
				ParseRegisters(p_im_um6->GetDataRegister(IMUM6::REGISTERS::UM6_QUAT_AB), i_orientation_y, i_orientation_x);
				 
				ParseRegisters(p_im_um6->GetDataRegister(IMUM6::REGISTERS::UM6_QUAT_CD), i_orientation_w, i_orientation_z);
				 
				ParseRegisters(p_im_um6->GetDataRegister(IMUM6::REGISTERS::UM6_ACCEL_PROC_XY), i_linear_acceleration_y, i_linear_acceleration_x);
				
				ParseRegisters(p_im_um6->GetDataRegister(IMUM6::REGISTERS::UM6_ACCEL_PROC_Z), i_linear_acceleration_z);
				
				linear_acceleration.x = 0.000183105 * (double)i_linear_acceleration_x;
				linear_acceleration.y = 0.000183105 * (double)i_linear_acceleration_y;
				linear_acceleration.z = 0.000183105 * (double)i_linear_acceleration_z;
				
				orientation.x = 0.0000335693 * (double)i_orientation_x;
				orientation.y = 0.0000335693 * (double)i_orientation_y;
				orientation.z = 0.0000335693 * (double)i_orientation_z;
				orientation.w = 0.0000335693 * (double)i_orientation_w;
				
				
				#ifdef TEST
				pose_demo.header.stamp = ros::Time::now();
				pose_demo.header.frame_id = "odom";
				
				
				pose_demo.pose.pose.position.x = 0.0;
				pose_demo.pose.pose.position.y = 0.0;
				pose_demo.pose.pose.position.z = 0.0;
				pose_demo.pose.pose.orientation = orientation;
				#endif
				
				imu_packet.orientation = orientation;
				imu_packet.linear_acceleration = linear_acceleration;
				imu_packet.header.stamp = ros::Time::now();
						
				// Publish Data
				if(pub_um6.getNumSubscribers() > 0 || b_always_on)
				{
					pub_um6.publish(imu_packet);
				}
				
				#ifdef TEST
				if(pub_um6.getNumSubscribers() > 0 || b_always_on)
				{
					pub_pose_demo.publish(pose_demo);
				}
				#endif
						
			
			}
		}catch(int e){
			ROS_INFO(GetErrorDescription(e).c_str());
			i_error_code = e;
		}
		updater.update();
		loop_rate.sleep();
	
	}
	
	return 0;
}
