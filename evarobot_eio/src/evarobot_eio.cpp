//! Bu sınıfa ait dokümantasyon evapi_ros 85rpm altındaki dokümantasyon ile aynıdır.

#include "evarobot_eio/evarobot_eio.h"

int i_error_code = 0;

int i_test_code = 0;

IMEIO * eio ;

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

bool CallbackEKBTest(im_msgs::EIOTest::Request& request, im_msgs::EIOTest::Response& response)
{
	ROS_DEBUG("EvarobotEIO: EKB Test");
	int i_pin = request.pin;
	int i_mod = request.mode;
	bool b_success = true;
	bool b_data = false;
	bool b_value = request.value;

	// porta
	if(request.portno == 0)
	{
	        try
		{
        	        eio->SetPinADirection(i_pin, IMEIO::OUTPUT);
                        eio->SetPinAValue(i_pin, b_value);
                }catch(int e){
                	b_success = false;
                }

	}
	// portb
	else
	{
		if(i_mod==0)
		{
			try{
				eio->SetPinBDirection(i_pin, IMEIO::INPUT);
				usleep(1000000);
				eio->GetPinBValue(i_pin, b_data);
			}catch(int e){
				b_success = false;
			}
		}
		else
		{
			try{
				eio->SetPinBDirection(i_pin, IMEIO::OUTPUT);
				eio->SetPinBValue(i_pin, b_value);
			}catch(int e){
				b_success = false;
			}
		}
	}

	response.b_data = b_data;
	response.b_success = b_success;
	return true;
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
	double d_frequency;
	
	string str_i2c_path;
	// rosparams end
	

		
	ros::init(argc, argv, "evarobot_eio");
	ros::NodeHandle n;
	
	n.param<string>("evarobot_eio/i2c_path", str_i2c_path, "/dev/i2c-1");
	
	if(!n.getParam("evarobot_eio/frequency", d_frequency))
	{
		ROS_INFO(GetErrorDescription(-99).c_str());
        i_error_code = -99;
	} 

	ros::ServiceServer service = n.advertiseService("evarobot_eio/ekb_test", CallbackEKBTest);	

	// Define frequency
	ros::Rate loop_rate(d_frequency);
	
	
	
	try{
		eio = new IMEIO(0b00100000, string("/dev/i2c-1"),  mutex);		
		eio->SetPinBDirection(IMEIO::EIO0, IMEIO::OUTPUT);
		eio->SetPinBDirection(IMEIO::EIO1, IMEIO::OUTPUT);
		eio->SetPinBDirection(IMEIO::EIO2, IMEIO::OUTPUT);
		eio->SetPinBDirection(IMEIO::EIO3, IMEIO::OUTPUT);
		eio->SetPinBDirection(IMEIO::EIO4, IMEIO::OUTPUT);
		eio->SetPinBDirection(IMEIO::EIO5, IMEIO::OUTPUT);
		eio->SetPinBDirection(IMEIO::EIO6, IMEIO::OUTPUT);
		eio->SetPinBDirection(IMEIO::SWR_RESET, IMEIO::INPUT);
		eio->SetPinADirection(IMEIO::BUZZER, IMEIO::OUTPUT);
		eio->SetPinADirection(IMEIO::USER_LED, IMEIO::OUTPUT);
	}catch(int e){
		ROS_INFO(GetErrorDescription(e).c_str());
        i_error_code = e;
	}
     
	bool b_data = false;
	
	
	// ROS PARAMS
    double d_min_freq = 0.2;
	double d_max_freq = 10.0;

	// Diagnostics
	diagnostic_updater::Updater updater;
	updater.setHardwareID("EvarobotEIO");
	updater.add("eio", &ProduceDiagnostics);

	diagnostic_updater::HeaderlessTopicDiagnostic pub_freq("eio", updater,
            diagnostic_updater::FrequencyStatusParam(&d_min_freq, &d_max_freq, 0.1, 10));
            
	while(ros::ok())
	{					
		updater.update();
		loop_rate.sleep();	
		ros::spinOnce();
	}
	
	
	return 0;
}
