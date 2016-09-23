#include "evarobot_battery/evarobot_battery.h"

/**
 * LTC2943 modes
 */
#define AUTOMATIC_MODE 					0xC0
#define SCAN_MODE						0x80
#define MANUAL_MODE						0x40
#define SLEEP_MODE						0x00
#define SHUTDOWN_MODE					0x01

/**
 * LTC2943 prescalers
 */
#define PRESCALER_1						0x00
#define PRESCALER_4						0x08
#define PRESCALER_16					0x10
#define PRESCALER_64					0x18
#define PRESCALER_256					0x20
#define PRESCALER_1024					0x28
#define PRESCALER_4096					0x30
#define PRESCALER_4096_2				0x31

/**
 * LTC29423 AL/CC pin mode enumeration
 */
#define ALERT_MODE						0x04 // Alert mode
#define CHARGE_COMPLETE_MODE			0x02 // Charge complete mode
#define DISABLE_ALCC_PIN				0x00 // AL/CC pin disabled

/**
 * If an error occurs publishes it.
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
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    }
}

/**
 * Program starts from here.
 */
int main(int argc, char *argv[])
{
	/**
	 * Semaphore variables.
	 */
	key_t key = 1005;
	sem_t *mutex;
	FILE *fd;
	
	/**
	 * Frequency, minimum frequency and maximum frequency variables.
	 */
	double d_frequency;
	double d_min_freq, d_max_freq;
	/**
	 * High voltage and low voltage threshold variables.
	 */
	double d_high_voltage_th, d_low_voltage_th;
	/**
	 * Maximum current variable.
	 */
	double d_max_current;
	/**
	 * Battery capacity and battery low capacity variables.
	 */
	double d_bat_capacity, d_low_capacity;
	/**
	 * If this variable false; battery topic is published only there is at least one subscriber.
	 * If this variable true; battery topic is published in all conditions.
	 */
	bool b_always_on;

	/**
	 * Battery message to publish.
	 */
	im_msgs::Battery msg;

	/**
	 * Current charge value.
	 */
	double d_charge_last = -1.0;

	/**
	 * i2c device to communicate battery device.
	 */
	string str_i2cDevice;

	/**
	 * create & initialize semaphore
	 */
	mutex = sem_open(SEM_NAME,O_CREAT,0644,1);
	if(mutex == SEM_FAILED)
	{
		ROS_INFO(GetErrorDescription(-125).c_str());
        i_error_code = -125;
		sem_unlink(SEM_NAME);
		return(-1);
	}
	
	/**
	 * Initializes ROS node with evarobot_battery name.
	 */
	ros::init(argc, argv, "evarobot_battery");

	/**
	 * Creates ROS node handler.
	 */
	ros::NodeHandle n;

	/**
	 * Gets parameters from configuration file.
	 */
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

	/**
	 * Publisher topic is created with battery topic name and std_msgs::Battery message type.
	 */
	ros::Publisher pub_bat = n.advertise<im_msgs::Battery>("battery", 10);

	/**
	 * Battery reader to get raw data.
	 */
	LTC2943 batteryReader = LTC2943(0x64, "/dev/i2c-1", mutex);

	/**
	 * Sets battery reader informations.
	 */
	try
	{
		batteryReader.setControlRegister(AUTOMATIC_MODE,PRESCALER_4096, ALERT_MODE);
		batteryReader.setVoltageThresholdHigh(d_high_voltage_th);
		batteryReader.setVoltageThresholdLow(d_low_voltage_th);
	}
	catch(int e)
	{
		ROS_INFO(GetErrorDescription(e).c_str());
		i_error_code = e;
	}
	
	/**
	 * Set diagnostics to handle and publish error.
	 */
	diagnostic_updater::Updater updater;
	updater.setHardwareID("LTC2943");
	updater.add("evarobot_battery", &ProduceDiagnostics);
	diagnostic_updater::HeaderlessTopicDiagnostic pub_freq("battery", updater,
            diagnostic_updater::FrequencyStatusParam(&d_min_freq, &d_max_freq, 0.1, 10));
	
	/**
	 * Define frequency
	 */
	ros::Rate loop_rate(10.0);	

	while(ros::ok())
	{
		/**
		 * Reads register from LTC29423.
		 */
		try
		{
			batteryReader.readRegisters();
		}
		catch(int e)
		{
			ROS_INFO(GetErrorDescription(e).c_str());
			i_error_code = e;
		}

		/**
		 * Gets voltage, current, accumulated charge and temperature values from LTC29423.
		 */
		msg.voltage = batteryReader.readVoltage();
		msg.current = batteryReader.readCurrent();
		msg.charge = batteryReader.readAccumulatedCharge();
		msg.temperature = batteryReader.readTemperature();

		/**
		 * Controls voltage is valid.
		 * Voltage is invalid if bigger than high voltage threshold or smaller than low voltage threshold.
		 */
		if(msg.voltage >= d_high_voltage_th) // High Voltage
		{
			ROS_INFO(GetErrorDescription(-126).c_str());
			i_error_code = -126;
		}
		else if(msg.voltage <= d_low_voltage_th) // Low voltage
		{
			ROS_INFO(GetErrorDescription(-127).c_str());
			i_error_code = -127;
		}
		
		/**
		 * Controls accumulated charge is valid.
		 * It is invalid If smaller than low capacity
		 */
		if(msg.charge <= d_low_capacity) // Low battery
		{
			ROS_INFO(GetErrorDescription(-129).c_str());
			i_error_code = -129;			
		}
		
		/**
		 * Determines charge state. (Charhing or discharging)
		 * If current charge is bigger than previous charge, battery is charging.
		 * If current charge is smaller than previous charge, battery is discharging.
		 */
		if(d_charge_last > 0 && d_charge_last > msg.charge) // Charging
		{
			msg.charge_state = 1;
		}
		else if(d_charge_last > 0 && d_charge_last < msg.charge) // Discharging
		{
			msg.charge_state = 0;
		}
		
		/**
		 * Determines battery is full or not.
		 * If current charge is bigger than or equal to battery capacity, battery is fully charged.
		 */
		if(msg.charge >= d_bat_capacity) // Fully charged
		{
			msg.charge_state = 2;
		}
		
		/**
		 * Last charge is equalized to current charge after operations are completed.
		 */
		d_charge_last = msg.charge;		
		
		/**
		 * If there are any subscriber or b_always_on is set as true, message is published.
		 * Else, message is not published.
		 */
		if(pub_bat.getNumSubscribers() > 0 || b_always_on)
		{
			pub_bat.publish(msg);
			pub_freq.tick();
		}
		
		/**
		 * Loop is slept to hold frequency.
		 */
		updater.update();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
