#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"



#include "IMUM6.h"

#define SERIAL_PATH "/dev/ttyAMA0"
#define BAUDRATE B115200
#define DATABITS CS8

// 0: No Parity; 
// 1: ODD Parity; 
// 2: Even Parity
#define PARITY 0 
#define STOP_BITS 0

using namespace std;


void ParseRegisters(int32_t i_register_data, float & f_data_l, float & f_data_h)
{
	UNION32 union32;

	union32.i = 0x00000000;
	union32.i |= ((-(unsigned int)i_register_data) & 0x0000FFFF);
	f_data_l = union32.f;
	
	union32.i = 0x00000000;
	union32.i |= ( ((-(unsigned int)i_register_data) >> 16) & 0x0000FFFF );
	f_data_h = union32.f;
	
}

void ParseRegisters(int32_t i_register_data, float f_data_h)
{
	UNION32 union32;
	
	union32.i = 0x00000000;
	union32.i |= ( ((-(unsigned int)i_register_data) >> 16) & 0x0000FFFF );
	f_data_h = union32.f;
}

int main(int argc, char **argv)
{
	unsigned char u_c_spi_mode;
	tcflag_t parity;
	tcflag_t parity_on;
	
	vector<int> T_i_registers;
	
	// ROS PARAMS
	
	double d_frequency;
	
	string str_namespace;
	string str_frame_id;
	
	bool b_always_on;
	// rosparams end
	
	ros::Publisher pub_um6;
	sensor_msgs::Imu imu_packet;
	
	ros::init(argc, argv, "evarobot_orientation");
	ros::NodeHandle n;
	
	n.param<string>("evarobot_orientation/namespace", str_namespace, "gazebo");
	n.param<string>("evarobot_orientation/frameNamespace", str_frame_id, "imu");
	n.param("evarobot_orientation/alwaysOn", b_always_on, false);
			
	if(!n.getParam("evarobot_orientation/frequency", d_frequency))
	{
		ROS_ERROR("Failed to get param 'frequency'");
	} 

	
	
	
	// Set publisher
	stringstream ss_topic;
	ss_topic << str_namespace << "/imu";
	pub_um6 = n.advertise<sensor_msgs::Imu>(ss_topic.str().c_str(), 1);
	
	

	// Define frequency
	ros::Rate loop_rate(d_frequency);

	stringstream ss_frame;
	ss_frame << str_frame_id << "/imu";


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
			parity = PARODD
			parity_on = PARENB;
			break;
		}
		
		case 2:
		{
			parity = ~PARODD
			parity_on = PARENB;
			break;
		}
		
		default:
		{
			ROS_ERROR("Wrong Parity Mode. It should be 0-2.");
		}
	}
	
	
	// Creating serial and um6 objects.
	IMSerial * p_im_serial = new IMSerial(SERIAL_PATH, BAUDRATE, DATABITS, parity, parity_on, STOP_BITS);
	IMUM6 * p_im_um6 = new IMUM6(p_im_serial);
	

	while(ros::ok())
	{		
		if(p_im_um6->GetRawData(data))
		{
		
			if(!p_im_um6->CheckData())
				ROS_ERROR("Checksum error in serial communication\n");

			
			p_im_um6->ProcessData(T_i_registers);
			
			//printf("Read Registers: \n");
			//for(uint i = 0; i < T_i_registers.size(); i++)
			//{
				//printf("%x_", T_i_registers[i]);
				//printf("::%x_", imimu.GetDataRegister(T_i_registers[i]) );
			//}
			//printf("\n");
			
			
			geometry_msgs::Quaternion orientation;
			geometry_msgs::Vector3 linear_acceleration;
			
		
			ParseRegisters(p_im_um6->GetDataRegister(IMUM6::UM6_QUAT_AB),
			 orientation.y, orientation.x);
			 
			ParseRegisters(p_im_um6->GetDataRegister(IMUM6::UM6_QUAT_CD),
			 orientation.w, orientation.z);
			 
			ParseRegisters(p_im_um6->GetDataRegister(IMUM6::UM6_ACCEL_RAW_XY),
			 orientation.y, orientation.x);
			 
			ParseRegisters(p_im_um6->GetDataRegister(IMUM6::UM6_ACCEL_RAW_Z,
			linear_acceleration.z)
			
			imu_packet.orientation = orientation;
			imu_packet.linear_acceleration = linear_acceleration;
			imu_packet.header.stamp = ros::Time::now();
					
			// Publish Data
			if(pub_um6.getNumSubscribers() > 0 || b_always_on)
			{
				pub_um6.publish(imu_packet);
			}		
		
		}
			
		loop_rate.sleep();	
	
	}
	
	return 0;
}
