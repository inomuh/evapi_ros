//! Bu sınıfa ait dokümantasyon evapi_ros 85rpm altındaki dokümantasyon ile aynıdır.

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

int i_error_code = 0;

bool b_ekb_test_status = false;

void ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    if(i_error_code<0)
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "%s", GetErrorDescription(i_error_code).c_str());
        i_error_code = 0;
    }
    else
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No collision!");
    }
}


bool CallbackEKBTest(im_msgs::EKBTest::Request& request, im_msgs::EKBTest::Response& response)
{
	ROS_DEBUG("EvarobotBattery: EKB Test");
	b_ekb_test_status = request.b_on_off;
	response.b_success = true;
	return true;
}

int main(int argc, char *argv[])
{
	// Semaphore
	key_t key;
	sem_t *mutex;
	FILE *fd;
	
	im_msgs::Battery msg;
	double d_charge_last = -1.0;
	
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
		ROS_INFO(GetErrorDescription(-125).c_str());
        i_error_code = -125;
        
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
	n.param("evarobot_battery/batCapacity", d_bat_capacity, 13000.0);
	n.param("evarobot_battery/lowCapacity", d_low_capacity, 7000.0);
	n.param<std::string>("evarobot_minimu9/i2cDevice", str_i2cDevice, "/dev/i2c-1");

	ros::Publisher pub_bat = n.advertise<im_msgs::Battery>("battery", 10);

	LTC2943 batteryReader = LTC2943(0x64, "/dev/i2c-1", mutex);

	//sem_wait(mutex);

	try
	{
		batteryReader.setControlRegister(AUTOMATIC_MODE,PRESCALER_4096, ALERT_MODE);
		batteryReader.setVoltageThresholdHigh(d_high_voltage_th);
		batteryReader.setVoltageThresholdLow(d_low_voltage_th);
	}catch(int e)
	{
		ROS_INFO(GetErrorDescription(e).c_str());
		i_error_code = e;
	}
	
	//sem_post(mutex);

	// Diagnostics
	diagnostic_updater::Updater updater;
	updater.setHardwareID("LTC2943");
	updater.add("evarobot_battery", &ProduceDiagnostics);

	diagnostic_updater::HeaderlessTopicDiagnostic pub_freq("battery", updater,
            diagnostic_updater::FrequencyStatusParam(&d_min_freq, &d_max_freq, 0.1, 10));
	
	// Define frequency
	ros::Rate loop_rate(10.0);	
	
	ros::ServiceServer service = n.advertiseService("evarobot_battery/ekb_test", CallbackEKBTest);
	
	while(!b_ekb_test_status && ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	
	while(ros::ok())
	{
		
		//sem_wait(mutex);

		int i_status_register = 0;

//		i_status_register = batteryReader.readStatusRegister();
//		ROS_INFO("Status Register: %d", i_status_register);
//		i_status = batteryReader.resetAlertStatus();

		try
		{
			batteryReader.readRegisters();
		}catch(int e)
		{
			ROS_INFO(GetErrorDescription(e).c_str());
			i_error_code = e;
		}

		msg.voltage = batteryReader.readVoltage();
		msg.current = batteryReader.readCurrent();
		msg.charge = batteryReader.readAccumulatedCharge();
		msg.temperature = batteryReader.readTemperature();
			
		//sem_post(mutex);
		
		if(msg.voltage >= d_high_voltage_th)
		{
			// High Voltage
			ROS_INFO(GetErrorDescription(-126).c_str());
			i_error_code = -126;
		}
		else if(msg.voltage <= d_low_voltage_th)
		{
			// Low voltage
			ROS_INFO(GetErrorDescription(-127).c_str());
			i_error_code = -127;
		}
		
		if(msg.charge <= d_low_capacity)
		{
			// Low battery
			ROS_INFO(GetErrorDescription(-129).c_str());
			i_error_code = -129;			
		}
		
		if(d_charge_last > 0 && d_charge_last > msg.charge)
		{
			// Charging
			msg.charge_state = 1;
		}
		else if(d_charge_last > 0 && d_charge_last < msg.charge)
		{
			// Discharging
			msg.charge_state = 0;
		}
		
		if(msg.charge >= d_bat_capacity)
		{
			// Charged
			msg.charge_state = 2;
		}
		
		d_charge_last = msg.charge;		
		
		if(pub_bat.getNumSubscribers() > 0 || b_always_on)
		{
			pub_bat.publish(msg);
			pub_freq.tick();
		}
		
		updater.update();
		ros::spinOnce();
		loop_rate.sleep();
		
	}

	return 0;
}
