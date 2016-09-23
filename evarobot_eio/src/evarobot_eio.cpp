#include "evarobot_eio/evarobot_eio.h"

/**
 *  If an error occurs publishes it.
 *  Else publishes both "EvarobotEIO: No problem" message.
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
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "EvarobotEIO: No problem.");
    }
}

/**
 * Program starts here.
 */
int main(int argc, char **argv)
{
	/**
	 * Semaphore variables.
	 */
	key_t key = 1005;
	sem_t *mutex;
	FILE * fd;

	/**
	 * create & initialize semaphore
	 */
	mutex = sem_open(SEM_NAME,O_CREAT,0644,1);
	if(mutex == SEM_FAILED)
	{
		ROS_INFO(GetErrorDescription(-98).c_str());
        i_error_code = -98;
		sem_unlink(SEM_NAME);

		return(-1);
	}

	/**
	 * Frequency, minimum frequency and maximum frequency variables.
	 */
	double d_frequency;
	double d_min_freq = 0.2;
	double d_max_freq = 10.0;
	
	/**
	 * i2c device path.
	 */
	string str_i2c_path;
	
	/**
	 * Initializes ROS node with evarobot_eio name.
	 */
	ros::init(argc, argv, "evarobot_eio");
	
	/**
	 * Creates ROS node handler.
	 */
	ros::NodeHandle n;
	
	/**
	 * Gets i2c device path and ROS frequency parameters from configuration file.
	 */
	n.param<string>("evarobot_eio/i2c_path", str_i2c_path, "/dev/i2c-1");
	
	if(!n.getParam("evarobot_eio/frequency", d_frequency))
	{
		ROS_INFO(GetErrorDescription(-99).c_str());
        i_error_code = -99;
	}

	/**
	 * Define frequency
	 */
	ros::Rate loop_rate(d_frequency);
	
	/**
	 * Creates IMEIO class reference and sets EIO pins as output.
	 */
	IMEIO * eio ;
	try{
		eio = new IMEIO(0b00100000, string("/dev/i2c-1"),  mutex);
		
		eio->SetPinBDirection(IMEIO::EIO0, IMEIO::OUTPUT);
		eio->SetPinBDirection(IMEIO::EIO1, IMEIO::OUTPUT);
		eio->SetPinBDirection(IMEIO::EIO2, IMEIO::OUTPUT);
		eio->SetPinBDirection(IMEIO::EIO3, IMEIO::OUTPUT);
		eio->SetPinBDirection(IMEIO::EIO4, IMEIO::OUTPUT);
		eio->SetPinBDirection(IMEIO::EIO5, IMEIO::OUTPUT);
		eio->SetPinBDirection(IMEIO::EIO6, IMEIO::OUTPUT);
		eio->SetPinBDirection(IMEIO::SWR_RESET, IMEIO::OUTPUT);

		eio->SetPinADirection(IMEIO::RGB_LED1, IMEIO::OUTPUT);
		eio->SetPinADirection(IMEIO::RGB_LED2, IMEIO::OUTPUT);
		eio->SetPinADirection(IMEIO::RGB_LED3, IMEIO::OUTPUT);
		eio->SetPinADirection(IMEIO::BUZZER, IMEIO::OUTPUT);
		eio->SetPinADirection(IMEIO::USER_LED, IMEIO::OUTPUT);
	}catch(int e){
		ROS_INFO(GetErrorDescription(e).c_str());
        i_error_code = e;
	}
     
	/**
	 * Set diagnostics to handle and publish error.
	 */
	diagnostic_updater::Updater updater;
	updater.setHardwareID("EvarobotEIO");
	updater.add("eio", &ProduceDiagnostics);

	diagnostic_updater::HeaderlessTopicDiagnostic pub_freq("eio", updater,
            diagnostic_updater::FrequencyStatusParam(&d_min_freq, &d_max_freq, 0.1, 10));
            
	while(ros::ok())
	{
		/**
		 * Loop is slept to hold frequency.
		 */
		updater.update();
		loop_rate.sleep();
	}
	
	
	return 0;
}
