#include "evarobot_bumper/evarobot_bumper.h"

#include <dynamic_reconfigure/server.h>
#include <evarobot_bumper/ParamsConfig.h>


using namespace std;

bool b_always_on;
bool b_is_received_params = false;

void CallbackReconfigure(evarobot_bumper::ParamsConfig &config, uint32_t level)
{
   ROS_INFO("Bumper Param Callback Function...");
   b_is_received_params = true;        
   
   b_always_on = config.alwaysOn;
}

int main(int argc, char **argv)
{
	
	key_t key;
	sem_t *mutex;
	FILE * fd;

	key = 1005;

	mutex = sem_open(SEM_NAME,O_CREAT,0644,1);
	if(mutex == SEM_FAILED)
	{
		perror("unable to create semaphore");
		sem_unlink(SEM_NAME);

		return(-1);
	}


	// ROS PARAMS
	double d_frequency;
	
	string str_topic_name;
	string str_i2c_device_address;
	string str_i2c_path;
	
	// rosparams end
	
	ros::Publisher pub_bumper;
	im_msgs::Bumper bumpers;
	
		
	ros::init(argc, argv, "evarobot_bumper");
	ros::NodeHandle n;
	
	n.param<string>("evarobot_bumper/i2c_path", str_i2c_path, "/dev/i2c-1");

	n.param<string>("evarobot_bumper/commandTopic", str_topic_name, "bumper");
	n.param("evarobot_bumper/alwaysOn", b_always_on, false);

	
	if(!n.getParam("evarobot_bumper/frequency", d_frequency))
	{
		ROS_ERROR("Failed to get param 'frequency'");
	} 

		
	// Set publisher
	pub_bumper = n.advertise<im_msgs::Bumper>(str_topic_name.c_str(), 10);
	
	// Dynamic Reconfigure
    dynamic_reconfigure::Server<evarobot_bumper::ParamsConfig> srv;
    dynamic_reconfigure::Server<evarobot_bumper::ParamsConfig>::CallbackType f;
    f = boost::bind(&CallbackReconfigure, _1, _2);
    srv.setCallback(f);
    ///////////////
	
	// Define frequency
	ros::Rate loop_rate(d_frequency);
	
	IMEIO * eio = new IMEIO(0b00100000, string("/dev/i2c-1"),  mutex);
	
	eio->SetPinADirection(IMEIO::BUMPER0, IMEIO::INPUT);
	eio->SetPinADirection(IMEIO::BUMPER1, IMEIO::INPUT);
	eio->SetPinADirection(IMEIO::BUMPER2, IMEIO::INPUT);
	
	vector<int> T_i_bumper_no;
	
	T_i_bumper_no.resize(MAX_BUMPER);
	
	T_i_bumper_no[0] = IMEIO::BUMPER0;
	T_i_bumper_no[1] = IMEIO::BUMPER1;
	T_i_bumper_no[2] = IMEIO::BUMPER2;

	while(ros::ok())
	{
		if(b_is_received_params)
		{
			ROS_INFO("Updating Bumper Params...");
		
			b_is_received_params = false;
		}		
		
		for(uint i = 0; i < T_i_bumper_no.size(); i++)
		{
			im_msgs::BumperState bumper_state;
		
			bool b_data;
			eio->GetPinAValue(T_i_bumper_no[i], b_data);
			
			bumper_state.bumper_state = b_data;
			bumpers.state.push_back(bumper_state);

		}
		
		bumpers.header.stamp = ros::Time::now();

		// Publish Data
		if(pub_bumper.getNumSubscribers() > 0 || b_always_on)
		{
			pub_bumper.publish(bumpers);
		}
		
		bumpers.state.clear();
		
		ros::spinOnce();
		loop_rate.sleep();	
	
	}
	
	
	return 0;
}
