#include "evarobot_eio/evarobot_rgb.h"

const int IMRGB::LEDOFF    = 0;
const int IMRGB::RED       = 1;
const int IMRGB::GREEN     = 2;
const int IMRGB::YELLOW    = 3;
const int IMRGB::BLUE      = 4;
const int IMRGB::PINK      = 5;
const int IMRGB::TURQUOISE = 6;
const int IMRGB::WHITE     = 7;

const int IMRGB::MANUAL      = 0;
const int IMRGB::ON          = 1;
const int IMRGB::AUTONOMOUS  = 2;
const int IMRGB::TELEOP      = 3;
const int IMRGB::WANDER      = 4;
const int IMRGB::LOW_BATTERY = 5;
const int IMRGB::CHARGING    = 6;
const int IMRGB::CHARGED     = 7;
const int IMRGB::ERROR       = 8;

/**
 * Constructor with eio class reference parameter.
 * Sets default values.
 */
IMRGB::IMRGB(IMEIO * eio)
{
	this->eio = eio;
	this->b_new_data = false;
	this->i_mode = IMRGB::MANUAL;
	this->f_frequency = 10.0;
	this->i_times = 0;
	this->i_color = IMRGB::LEDOFF;

	/**
	 * Sets RGB led pins as output.
	 */
	try{
		this->eio->SetPinADirection(IMEIO::RGB_LED1, IMEIO::OUTPUT);
		this->eio->SetPinADirection(IMEIO::RGB_LED2, IMEIO::OUTPUT);
		this->eio->SetPinADirection(IMEIO::RGB_LED3, IMEIO::OUTPUT);
	}catch(int e){
		ROS_INFO(GetErrorDescription(e).c_str());
        i_error_code = e;
	}
	this->TurnLedOn(0);
}

/**
 * Destructor turns off leds.
 */
IMRGB::~IMRGB()
{
	this->TurnLedOff();
}

/**
 * Turns off leds.
 */
void IMRGB::TurnLedOff()
{
	this->TurnLedOn(0);
	this->f_frequency = 10.0;
}

/**
 * Turns on leds in input color.
 */
void IMRGB::TurnLedOn(int i_color)
{
	int i_dummies[3] = {1, 2, 4};
	bool b_rgb[3];
	
	if(i_color > 7)
	{
		ROS_INFO(GetErrorDescription(-92).c_str());
        i_error_code = -92;
	}
	
	/**
	 * Calculates R, B and G values according to input color code.
	 */
	for(int i = 0; i < 3; i++)
	{
		b_rgb[i] = (bool)(i_color & i_dummies[i]);
	}
	try{
		this->eio->SetPinAValue(IMEIO::RGB_LED1, b_rgb[0]);
		this->eio->SetPinAValue(IMEIO::RGB_LED2, b_rgb[1]);
		this->eio->SetPinAValue(IMEIO::RGB_LED3, b_rgb[2]);
	}catch(int e){
		ROS_INFO(GetErrorDescription(e).c_str());
                i_error_code = e;
	}
}

/**
 * Turns on leds red and blue respectively and repetitiously.
 */
void IMRGB::RunAutonomousMode()
{	
	int i_colors[]={IMRGB::RED, 0, 
					IMRGB::RED, 0,
					IMRGB::RED, 0, 
					IMRGB::RED, 0,
					-1, 
					IMRGB::BLUE, 0,
					IMRGB::BLUE, 0,
					IMRGB::BLUE, 0,
					IMRGB::BLUE, 0, 
					-1};

	for(int i = 0; i < (sizeof(i_colors)/sizeof(*i_colors)); i++)
	{
		if(i_colors[i] < 0)
		{
			usleep(200000);
			++i;
			if(i >= (sizeof(i_colors)/sizeof(*i_colors)))
				break;
		}

		this->TurnLedOn(i_colors[i]);
		usleep(50000);
	}
	
	this->DecreaseTimesValue();
}

/**
 * Turns on leds in input color and then turns off leds repetitiously.
 */
void IMRGB::FlipFlip(int i_color)
{
	int i_colors[]={i_color, 0,
				    i_color, 0,
				    i_color, 0,
				    i_color, 0,
				    i_color, 0,
				    i_color, 0,
				    i_color, 0,
				    -1};
				
	for(int i = 0; i < (sizeof(i_colors)/sizeof(*i_colors)); i++)
	{
		if(i_colors[i] < 0)
		{
			usleep(500000);
			++i;
			if(i >= (sizeof(i_colors)/sizeof(*i_colors)))
				break;
		}
		this->TurnLedOn(i_colors[i]);
		
		usleep(50000);
	}	
}

/**
 * Turns on leds blue in flip flip mode.
 */
void IMRGB::RunTeleopMode()
{
	this->FlipFlip(IMRGB::BLUE);
	this->DecreaseTimesValue();
}

/**
 * Turns on leds turquoise in flip flip mode.
 */
void IMRGB::RunWanderMode()
{
	this->FlipFlip(IMRGB::TURQUOISE);
	this->DecreaseTimesValue();
}

/**
 * Turns on leds in input color and then turns off leds.
 */
void IMRGB::Flash(int i_color, float f_frequency)
{
	int i_colors[]={i_color, 0};
	int i_sleep_time;
	
	if(f_frequency == 0.0)
	{
		ROS_INFO(GetErrorDescription(-93).c_str());
        i_error_code = -93;
		return;
	}
	
	if(f_frequency < 0)
	{
		this->TurnLedOn(i_color);
	}
	else
	{
		for(int i = 0; i < (sizeof(i_colors)/sizeof(*i_colors)); i++)
		{
			this->TurnLedOn(i_colors[i]);
			i_sleep_time = (int)(1000000.0 / (2.0 * f_frequency));
			usleep(i_sleep_time);
		}	
	}
}

/**
 * Turns on leds i_color in flash mode.
 */
void IMRGB::RunManualMode()
{
	this->Flash(this->i_color, this->f_frequency);
	this->DecreaseTimesValue();
}

/**
 * Turns on leds green in flash mode.
 */
void IMRGB::RunChargingMode()
{
	this->Flash(IMRGB::GREEN, 1.0);
	this->DecreaseTimesValue();
}

/**
 * Turns on leds green in continuous mode.
 */
void IMRGB::RunChargedMode()
{
	this->TurnLedOn(IMRGB::GREEN);
	this->DecreaseTimesValue();
}

/**
 * Turns on leds red in flash mode.
 */
void IMRGB::RunErrorMode()
{
	this->Flash(IMRGB::RED, 1.0);
	this->DecreaseTimesValue();
}

/**
 * Turns on leds yellow in flash mode.
 */
void IMRGB::RunLowBatteryMode()
{
	this->Flash(IMRGB::YELLOW, 1.0);
	this->DecreaseTimesValue();
}

/**
 * Turns on leds red, green, yellow, blue, pink, turquoise, white, turquoise, ping,
 * blue, yellow, green, red respectively. Then, turns off leds.
 */
void IMRGB::RunOpeningMode()
{
	int i_colors[]={IMRGB::RED, IMRGB::GREEN, IMRGB::YELLOW, IMRGB::BLUE, 
					IMRGB::PINK, IMRGB::TURQUOISE, IMRGB::WHITE, IMRGB::TURQUOISE, 
					IMRGB::PINK, IMRGB::BLUE, IMRGB::YELLOW, IMRGB::GREEN, 
					IMRGB::RED, IMRGB::LEDOFF};
					
	for(int i = 0; i < (sizeof(i_colors)/sizeof(*i_colors)); i++)
	{
		this->TurnLedOn(i_colors[i]);
		usleep(500000);
	}
	
	this->DecreaseTimesValue();
}

/**
 * Decreases i_times by 1.
 * Returns false if there is no remaining count.
 * Returns true if there is remaining count.
 */
bool IMRGB::DecreaseTimesValue()
{
	this->i_times--;
	this->b_new_data = false;
	
	if(this->i_times < 0)
	{
		this->i_times == 0;
		return false;
	}
	
	return true;
}

/**
 * Controls new request is taken or not.
 */
bool IMRGB::isNewRequest() const
{
	return this->b_new_data;
}

/**
 * Returns mode number.
 */
int IMRGB::GetMode() const
{
	return this->i_mode;
}

/**
 * Returns frequency.
 */
float IMRGB::GetFrequency() const
{
	return this->f_frequency;
}

/**
 * Returns how many times current mode will repeat.
 */
int IMRGB::GetTimesValue() const
{
	return this->i_times;
}

/**
 * Callback function for evarobot_rgb/SetRGB service.
 */
bool IMRGB::callbackSetRGB(im_msgs::SetRGB::Request& request, im_msgs::SetRGB::Response& response)
{
	this->b_new_data = true;
	response.ret = 1;
	
	this->i_times = request.times;
	
	if(request.mode < 0 || request.mode > 8)
	{
		response.ret = 0;
		ROS_INFO(GetErrorDescription(-94).c_str());
        i_error_code = -94;
	}
	else
	{
		this->i_mode = request.mode;
	}
	
	if(request.frequency == 0)
	{
		response.ret = 0;
		ROS_INFO(GetErrorDescription(-95).c_str());
        i_error_code = -95;
	}
	else
	{
		this->f_frequency = request.frequency;
	}
	
	if(request.color < 0 || request.color > 7)
	{
		response.ret = 0;
		ROS_INFO(GetErrorDescription(-96).c_str());
        i_error_code = -96;
	}
	else
	{
		this->i_color = request.color;
	}
	
	return true;
}

/**
 *  If an error occurs publishes it.
 *  Else publishes "EvarobotRGB: No problem." message.
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
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "EvarobotRGB: No problem.");
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
		ROS_INFO(GetErrorDescription(-97).c_str());
        i_error_code = -97;
		sem_unlink(SEM_NAME);

		return(-1);
	}

	/**
	 * i2c device path.
	 */
	string str_i2c_path;

	/**
	 * Initializes ROS node with evarobot_rgb name.
	 */
	ros::init(argc, argv, "/evarobot_rgb");
	
	/**
	 * Creates ROS node handler.
	 */
	ros::NodeHandle n;
	
	/**
	 * Gets i2c device path parameter from configuration file.
	 */
	n.param<string>("evarobot_eio/i2c_path", str_i2c_path, "/dev/i2c-1");
		
	/**
	 * Creates IMEIO class reference.
	 */
	IMEIO * eio;
	try{
		eio = new IMEIO(0b00100000, string("/dev/i2c-1"),  mutex);
	}catch(int e){
		ROS_INFO(GetErrorDescription(e).c_str());
        i_error_code = e;
	}
	
	/**
	 * Creates IMRGB class reference.
	 */
	IMRGB * rgb = new IMRGB(eio);
	
	/**
	 * Creates ROS service with name evarobot_rgb/SetRGB.
	 */
	ros::ServiceServer service = n.advertiseService("evarobot_rgb/SetRGB", &IMRGB::callbackSetRGB, rgb);
	
	/**
	 * Define frequency
	 */
	ros::Rate loop_rate(10.0);

	/**
	 * Set diagnostics to handle and publish error.
	 */
	diagnostic_updater::Updater updater;
	updater.setHardwareID("EvarobotRGB");
	updater.add("rgb", &ProduceDiagnostics);

	/**
	 * Start time and end time of operations in while loop.
	 */
	ros::Time start_time, end_time;

	/**
	 * Difference between end_time and start_time
	 */
	ros::Duration loop_duration;

	/**
	 * Remaining sleep time of loop to hold rgb frequency.
	 */
	ros::Duration sleep_duration;

	while(ros::ok())
	{	
		/**
		 * Gets start time of loop.
		 */
		start_time = ros::Time::now();
		
		/**
		 * Selected mode is applied.
		 */
		if(rgb->GetTimesValue() > 0 || rgb->GetTimesValue() < 0)
		{
			switch(rgb->GetMode())
			{
				case IMRGB::MANUAL:
				{
					rgb->RunManualMode();
					break;
				}
				
				case IMRGB::ON:
				{
					rgb->RunOpeningMode();
					break;
				}
				
				case IMRGB::AUTONOMOUS:
				{
					rgb->RunAutonomousMode();
					break;
				}
				
				case IMRGB::TELEOP:
				{
					rgb->RunTeleopMode();
					break;
				}
				
				case IMRGB::WANDER:
				{
					rgb->RunWanderMode();
					break;
				}
				
				case IMRGB::LOW_BATTERY:
				{
					rgb->RunLowBatteryMode();
					break;
				}
				
				case IMRGB::CHARGING:
				{
					rgb->RunChargingMode();
					break;
				}
				
				case IMRGB::CHARGED:
				{
					rgb->RunChargedMode();
					break;
				}
				
				case IMRGB::ERROR:
				{
					rgb->RunErrorMode();
					break;
				}
								
			}
		}
		else
		{
			rgb->TurnLedOff();
		}

		/**
		 * Loop is slept to hold frequency.
		 */
		updater.update();
		ros::spinOnce();
	
		/**
		 * Gets end time of loop.
		 */
		end_time = ros::Time::now();
		
		/**
		 * Remaining time is calculated.
		 * Loop is slept to hold rgb frequency.
		 */
		if(rgb->GetFrequency() > 0.0 && !(rgb->GetMode() == IMRGB::TELEOP || rgb->GetMode() == IMRGB::WANDER))
		{
			loop_duration =  end_time - start_time;
			sleep_duration = ros::Duration(1.0/rgb->GetFrequency()) - loop_duration;
			ros::Duration(sleep_duration).sleep();
		}
		else if(rgb->GetFrequency() < 0.0 && rgb->GetMode() == IMRGB::MANUAL)
		{
			loop_rate.sleep();
		}
	}
	
	return 0;
}
