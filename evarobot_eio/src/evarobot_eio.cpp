#include "evarobot_eio/evarobot_eio.h"


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
	
	string str_i2c_path;
	// rosparams end
	
	
	
	
		
	ros::init(argc, argv, "evarobot_eio");
	ros::NodeHandle n;
	
	n.param<string>("evarobot_eio/i2c_path", str_i2c_path, "/dev/i2c-1");
	
	if(!n.getParam("evarobot_eio/frequency", d_frequency))
	{
		ROS_ERROR("Failed to get param 'frequency'");
	} 

		

	// Define frequency
	ros::Rate loop_rate(d_frequency);
	
	IMEIO * eio = new IMEIO(0b00100000, string("/dev/i2c-1"),  mutex);
	
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
     
	bool b_data = false;
	
	while(ros::ok())
	{		
		

						
			b_data = !b_data;
			printf("value: %x \n", b_data);
			eio->SetPinBValue(IMEIO::EIO0, b_data);
			eio->SetPinBValue(IMEIO::EIO1, b_data);
			eio->SetPinBValue(IMEIO::EIO2, b_data);
			eio->SetPinBValue(IMEIO::EIO3, b_data);
			eio->SetPinBValue(IMEIO::EIO4, b_data);
			eio->SetPinBValue(IMEIO::EIO5, b_data);
			eio->SetPinBValue(IMEIO::EIO6, b_data);
			eio->SetPinBValue(IMEIO::SWR_RESET, b_data);

                      //  eio->SetPinAValue(IMEIO::RGB_LED1, b_data);
                        eio->SetPinAValue(IMEIO::RGB_LED2, b_data);
                      //  eio->SetPinAValue(IMEIO::RGB_LED3, b_data);
			eio->SetPinAValue(IMEIO::USER_LED, b_data);
                        eio->SetPinAValue(IMEIO::BUZZER, b_data);

			loop_rate.sleep();	
	
	}
	
	
	return 0;
}
