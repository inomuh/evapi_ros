#include "evarobot_eio/evarobot_gripper.h"

int i_error_code = 0;

bool b_open_gripper = false;
bool b_close_gripper = false;
bool b_tilt_up = false;
bool b_tilt_down = false;

bool CallbackOpenGripper(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	ROS_DEBUG("EvarobotGripper: Open Gripper");
	b_open_gripper = true;
	return true;
}

bool CallbackCloseGripper(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	ROS_DEBUG("EvarobotGripper: Close Gripper");
	b_close_gripper = true;
	return true;
}

bool CallbackTiltUp(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	ROS_DEBUG("EvarobotGripper: Tilt Up");
	b_tilt_up = true;
	return true;
}

bool CallbackTiltDown(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	ROS_DEBUG("EvarobotGripper: Tilt Down");
	b_tilt_down = true;
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
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "EvarobotEIO: No problem.");
    }
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
		ROS_INFO(GetErrorDescription(-98).c_str());
        i_error_code = -98;
		sem_unlink(SEM_NAME);

		return(-1);
	}


	// ROS PARAMS	
	string str_i2c_path;
	int servoMin, servoMax;
	double servoFreq;
	// rosparams end
	
	
		
	ros::init(argc, argv, "evarobot_gripper");
	ros::NodeHandle n;
	
	n.param<string>("evarobot_gripper/i2c_path", str_i2c_path, "/dev/i2c-1");
        n.param<int>("evarobot_gripper/servoMin", servoMin, 245);
        n.param<int>("evarobot_gripper/servoMax", servoMax, 460);
        n.param<double>("evarobot_gripper/servoFreq", servoFreq, 60.0);
	


	ros::ServiceServer service_og = n.advertiseService("evarobot_gripper/OpenGripper", CallbackOpenGripper);
	ros::ServiceServer service_cg = n.advertiseService("evarobot_gripper/CloseGripper", CallbackCloseGripper);
	ros::ServiceServer service_tu = n.advertiseService("evarobot_gripper/TiltUp", CallbackTiltUp);
	ros::ServiceServer service_td = n.advertiseService("evarobot_gripper/TiltDown", CallbackTiltDown);
	
		
	// Define frequency
	ros::Rate loop_rate(10.0);
	
	IMEIO * eio ;
	IMServoPWM * servo;
	
	try{
		eio = new IMEIO(0b00100000, str_i2c_path,  mutex);
		servo = new IMServoPWM(0x40, str_i2c_path,  mutex);
		
		// EIO0 -> INA
		// EIO1 -> INB
		eio->SetPinBDirection(IMEIO::EIO0, IMEIO::OUTPUT);
		eio->SetPinBDirection(IMEIO::EIO1, IMEIO::OUTPUT);

	}catch(int e){
		ROS_INFO(GetErrorDescription(e).c_str());
        i_error_code = e;
	}
     	
	
	// ROS PARAMS
  double d_min_freq = 0.2;
	double d_max_freq = 10.0;

	// Diagnostics
	diagnostic_updater::Updater updater;
	updater.setHardwareID("EvarobotGripper");
	updater.add("gripper", &ProduceDiagnostics);

	diagnostic_updater::HeaderlessTopicDiagnostic pub_freq("gripper", updater,
            diagnostic_updater::FrequencyStatusParam(&d_min_freq, &d_max_freq, 0.1, 10));

	servo->setPWMFreq(servoFreq);
	servo->setPWM(0, 0, servoMin);
	while(ros::ok())
	{		
		
		try
		{
			if(b_open_gripper)
			{
				eio->SetPinBValue(IMEIO::EIO0, true);
				eio->SetPinBValue(IMEIO::EIO1, false);
				usleep(1000000);
				eio->SetPinBValue(IMEIO::EIO0, false);
				b_open_gripper = false;
			}
			else if(b_close_gripper)
			{
				eio->SetPinBValue(IMEIO::EIO0, false);
				eio->SetPinBValue(IMEIO::EIO1, true);
				usleep(1000000);
				eio->SetPinBValue(IMEIO::EIO1, false);
				b_close_gripper = false;
			}
			else if(b_tilt_up)
			{
				servo->setPWM(0, 0, servoMin);
				b_tilt_up = false;
			}
			else if(b_tilt_down)
			{
				servo->setPWM(0, 0, servoMax);
				b_tilt_down = false;
			}

			
		}
		catch(int e)
		{
			ROS_INFO(GetErrorDescription(e).c_str());
			i_error_code = e;
		}

		ros::spinOnce();
		updater.update();
		loop_rate.sleep();	
	
	}
	servo->setPWM(0, 0, servoMin);
	
	return 0;
}
