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

IMRGB::IMRGB(IMEIO * eio)
{
	this->eio = eio;
	this->b_new_data = false;
	this->i_mode = IMRGB::MANUAL;
	this->f_frequency = 10.0;
	this->i_times = 0;
	this->i_color = IMRGB::LEDOFF;
	
	this->eio->SetPinADirection(IMEIO::RGB_LED1, IMEIO::OUTPUT);
	this->eio->SetPinADirection(IMEIO::RGB_LED2, IMEIO::OUTPUT);
    this->eio->SetPinADirection(IMEIO::RGB_LED3, IMEIO::OUTPUT);
    
    this->TurnLedOn(0);
}

IMRGB::~IMRGB()
{
	this->TurnLedOff();
}

void IMRGB::TurnLedOff()
{
	this->TurnLedOn(0);
	this->f_frequency = 10.0;
}

void IMRGB::TurnLedOn(int i_color)
{
	int i_dummies[3] = {1, 2, 4};
	bool b_rgb[3];
	
	if(i_color > 7)
	{
		ROS_ERROR("Invalid Color while turning led on");
	}
	
	for(int i = 0; i < 3; i++)
	{
		b_rgb[i] = (bool)(i_color & i_dummies[i]);
	}

	this->eio->SetPinAValue(IMEIO::RGB_LED1, b_rgb[0]);
	this->eio->SetPinAValue(IMEIO::RGB_LED2, b_rgb[1]);
	this->eio->SetPinAValue(IMEIO::RGB_LED3, b_rgb[2]);
}

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

void IMRGB::RunTeleopMode()
{
	this->FlipFlip(IMRGB::BLUE);
	this->DecreaseTimesValue();
}

void IMRGB::RunWanderMode()
{
	this->FlipFlip(IMRGB::TURQUOISE);
	this->DecreaseTimesValue();
}

void IMRGB::Flash(int i_color, float f_frequency)
{
	int i_colors[]={i_color, 0};
	int i_sleep_time;
	
	if(f_frequency == 0.0)
	{
		ROS_ERROR("Division by zero");
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

void IMRGB::RunManualMode()
{
	this->Flash(this->i_color, this->f_frequency);
	this->DecreaseTimesValue();
}

void IMRGB::RunChargingMode()
{
	this->Flash(IMRGB::GREEN, 1.0);
	this->DecreaseTimesValue();
}

void IMRGB::RunChargedMode()
{
	this->TurnLedOn(IMRGB::GREEN);
	this->DecreaseTimesValue();
}

void IMRGB::RunErrorMode()
{
	this->Flash(IMRGB::RED, 1.0);
	this->DecreaseTimesValue();
}

void IMRGB::RunLowBatteryMode()
{
	this->Flash(IMRGB::YELLOW, 1.0);
	this->DecreaseTimesValue();
}
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

bool IMRGB::isNewRequest() const
{
	return this->b_new_data;
}

int IMRGB::GetMode() const
{
	return this->i_mode;
}

float IMRGB::GetFrequency() const
{
	return this->f_frequency;
}

int IMRGB::GetTimesValue() const
{
	return this->i_times;
}

bool IMRGB::callbackSetRGB(im_msgs::SetRGB::Request& request, im_msgs::SetRGB::Response& response)
{
	
	this->b_new_data = true;
	response.ret = 1;
	
	
	this->i_times = request.times;
	
	
	if(request.mode < 0 || request.mode > 8)
	{
		response.ret = 0;
		ROS_ERROR("Invalid mode.");
	}
	else
	{
		this->i_mode = request.mode;
	}
	
	if(request.frequency == 0)
	{
		response.ret = 0;
		ROS_ERROR("Invalid frequency.");
	}
	else
	{
		this->f_frequency = request.frequency;
	}
	
	if(request.color < 0 || request.color > 7)
	{
		response.ret = 0;
		ROS_ERROR("Invalid color.");
	}
	else
	{
		this->i_color = request.color;
	}
	
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
		perror("unable to create semaphore");
		sem_unlink(SEM_NAME);

		return(-1);
	}

	// ROS PARAMS
	string str_i2c_path;
	// rosparams end
	
	ros::init(argc, argv, "/evarobot_rgb");
	ros::NodeHandle n;
	
	n.param<string>("evarobot_eio/i2c_path", str_i2c_path, "/dev/i2c-1");
		

	IMEIO * eio = new IMEIO(0b00100000, string("/dev/i2c-1"),  mutex);
	IMRGB * rgb = new IMRGB(eio);
	
	ros::ServiceServer service = n.advertiseService("evarobot_rgb/SetRGB", &IMRGB::callbackSetRGB, rgb);
	
	ros::Time start_time, end_time;
	ros::Duration sleep_duration, loop_duration;
	ros::Rate loop_rate(10.0);
	
	while(ros::ok())
	{	
		start_time = ros::Time::now();
		
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
				
		ros::spinOnce();
	
		end_time = ros::Time::now();
		
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
