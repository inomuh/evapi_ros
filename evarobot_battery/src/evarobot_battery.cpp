#include "evarobot_battery/evarobot_battery.h"

#include "IMI2C.h"
#include "evarobot_battery/LTC2943.h"

#define AUTOMATIC_MODE 					0xC0
#define SCAN_MODE						0x80
#define MANUAL_MODE						0x40
#define SLEEP_MODE						0x00

#define PRESCALER_1						0x00
#define PRESCALER_4						0x08
#define PRESCALER_16					0x10
#define PRESCALER_64					0x18
#define PRESCALER_256					0x20
#define PRESCALER_1024					0x28
#define PRESCALER_4096					0x30
#define PRESCALER_4096_2				0x31

#define ALERT_MODE						0x04
#define CHARGE_COMPLETE_MODE			0x02
#define DISABLE_ALCC_PIN				0x00

#define SHUTDOWN_MODE					0x01

using namespace std;

int main(int argc, char *argv[])
{
	// Semaphore
	key_t key;
	sem_t *mutex;
	FILE *fd;
	
	im_msgs::Battery msg;
	
	int i_status;
	
	// ROS PARAMS
	double d_frequency;
	double d_min_freq, d_max_freq;
	double d_high_voltage_th, d_low_voltage_th;
	double d_max_current;
	double d_bat_capacity, d_low_capacity;
	bool b_always_on;
	string str_i2cDevice;
	
	//name the shared memory segment
	key = 1005;

	//create & initialize semaphore
	mutex = sem_open(SEM_NAME,O_CREAT,0644,1);
	if(mutex == SEM_FAILED)
	{
		perror("unable to create semaphore");
		sem_unlink(SEM_NAME);
		
		return(-1);
	}
	
	ros::init(argc, argv, "evarobot_battery");
	ros::NodeHandle n;

	n.param("evarobot_battery/alwaysOn", b_always_on, false);
	n.param("evarobot_battery/minFreq", d_min_freq, 0.2);
	n.param("evarobot_battery/maxFreq", d_max_freq, 1.0);
	n.param("evarobot_battery/frequency", d_frequency, 1.0);
	n.param("evarobot_battery/highVTh", d_high_voltage_th, 13.0);
	n.param("evarobot_battery/lowVTh", d_low_voltage_th, 11.0);
	n.param("evarobot_battery/maxCurr", d_max_current, 7.0);
	n.param("evarobot_battery/batCapacity", d_bat_capacity, 7.0);
	n.param("evarobot_battery/lowCapacity", d_low_capacity, 3.0);
	n.param<std::string>("evarobot_minimu9/i2cDevice", str_i2cDevice, "/dev/i2c-1");

	ros::Publisher pub_bat = n.advertise<im_msgs::Battery>("battery", 10);

	LTC2943 batteryReader = LTC2943(0x64, "/dev/i2c-1", mutex);

	sem_wait(mutex);

	// create objects
	batteryReader.setControlRegister(AUTOMATIC_MODE,PRESCALER_4096, ALERT_MODE);

	
	i_status = batteryReader.setVoltageThresholdHigh(13.0);
	if(i_status < 0)
	{
		ROS_ERROR("Failed: setVoltageThresholdHigh");
	}
	
	i_status = batteryReader.setVoltageThresholdLow(4.5);
	if(i_status < 0)
	{
		ROS_ERROR("Failed: setVoltageThresholdLow");
	}

	sem_post(mutex);
	
	// Define frequency
	ros::Rate loop_rate(10.0);	
	while(ros::ok())
	{
		
		sem_wait(mutex);

		int i_status_register = 0;

//		i_status_register = batteryReader.readStatusRegister();
//		ROS_INFO("Status Register: %d", i_status_register);
//		i_status = batteryReader.resetAlertStatus();
		msg.voltage = batteryReader.readVoltage();
		msg.current = batteryReader.readCurrent();
		msg.charge = batteryReader.readAccumulatedCharge();
		msg.temperature = batteryReader.readTemperature();
			
		sem_post(mutex);
		
//		if(msg.voltage >= d_high_voltage_th)
//		{
			// High Voltage
//		}
//		else if(msg.voltage <= d_low_voltage_th)
//		{
			// Low voltage
//		}
		
//		if(msg.charge >= d_bat_capacity)
//		{
			// Charged
//		}
//		else if(msg.charge <= d_low_capacity)
//		{
			// Low battery
//		}
				
		
		if(pub_bat.getNumSubscribers() > 0 || b_always_on)
		{
			pub_bat.publish(msg);
			//pub_freq.tick();
		}
		
		ros::spinOnce();
		loop_rate.sleep();
		
	}

	return 0;
}
