#include "evarobot_android/evarobot_android.h"

/**
 * Program starts from here.
 */
int main(int argc, char **argv)
{
	/**
	 * Initializes both sonar and infrared sensor range data.
	 */
	for(uint i=0; i<12; i++)
	{
		i_sonar_values[i] = -1;
		i_infrared_values[i] = -1;
	}
  
	/**
	 * Initializes ROS node with evarobot_android name.
	 */
	ros::init(argc, argv, "evarobot_android");

	/**
	 * Creates ROS node handler.
	 */
	ros::NodeHandle n;

	/**
	 * Gets SonarSlots parameter from configuration file.
	 * Slot numbers of sonar sensors are stored at this variable.
	 */
	n.getParam("evarobot_android/SonarSlots", T_i_sonar_slots);
	/**
	 * Gets InfraredSlots parameter from configuration file.
	 * Slot numbers of infrared sensors are stored at this variable.
	 */
	n.getParam("evarobot_android/InfraredSlots", T_i_infrared_slots);

	/**
	 * If there is no sonar and infrared sensor, a warning is created.
	 */
	if(T_i_sonar_slots.size() <= 0 && T_i_infrared_slots.size() <= 0)
	{
		ROS_WARN("evarobot_android: There are no sonar or infrared");
	}

	/**
	 * Sonar sensor subscribers are generated.
	 * Topic names of sonar sensors are sonar0, sonar1, ...
	 */
	std::ostringstream ss;
	for(uint i = 0; i < T_i_sonar_slots.size(); i++)
	{
		ss<<"/sonar"<<i;
		IMSUBSCRIBER* subscriber = new IMSUBSCRIBER(ss.str(), T_i_sonar_slots[i], n, true);
		subscriber->SubscribeToTopic();
		ss.str("");
	}

	/**
	 * Infrared sensor subscribers are generated.
	 * Topic names of infrared sensors are ir0, ir1, ...
	 */
	for(uint i = 0; i < T_i_infrared_slots.size(); i++)
	{
		ss<<"/ir"<<i;
		IMSUBSCRIBER* subscriber = new IMSUBSCRIBER(ss.str(), T_i_infrared_slots[i], n, false);
		subscriber->SubscribeToTopic();
		ss.str("");
	}

	/**
	 * Publisher topic is created with evamobil topic name and std_msgs::String message type.
	 */
	ros::Publisher evamobil_pub = n.advertise<std_msgs::String>("evamobil", 1000);

	/**
	 * Define frequency.
	 */
	ros::Rate loop_rate(5);

	while (ros::ok())
	{
		std_msgs::String msg;

		/**
		 * Sonar and infrared sensor data are converted to this format.
		 * range_from_slot_0; range_from_slot_1; range_from_slot_2; ...; range_from_slot_11;
		 * Minimum of sonar and infrared range data is taken.
		 */
		std::stringstream ss;
		for(uint i=0; i<12; i++)
		{
			if(i_sonar_values[i] != -1 || i_infrared_values[i] != -1)
			{
				if(i_sonar_values[i] != -1 && i_infrared_values[i] == -1)
					ss<<i_sonar_values[i]<<";";
				else if(i_sonar_values[i] == -1 && i_infrared_values[i] != -1)
					ss<<i_infrared_values[i]<<";";
				else if(i_infrared_values[i]>=800)
					ss<<i_sonar_values[i]<<";";
				else if(i_sonar_values[i] < i_infrared_values[i])
					ss<<i_sonar_values[i]<<";";
				else
					ss<<i_infrared_values[i]<<";";
			}
			else
			{
				ss << "0;";
			}
		}

		/**
		 * 2 is added to created string. The last data defines battery status.
		 * 0: empty, 1: half full, 2: full
		 */
		ss<<"2";

		/**
		 * Message set as created string and published.
		 */
		msg.data = ss.str();
		evamobil_pub.publish(msg);

		/**
		 * Loop is slept to hold frequency.
		 */
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
