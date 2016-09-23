#ifdef English_dox
/**
 * \file   evarobot_android.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2016
 * \brief  Subscribes sonar and infrared sensor data topics and publishes them in a specific format.
 * \details
 */
#endif

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

using namespace std;

#ifdef English_dox
//! Sonar sensors are located at which slots on the evarobot. There are 12 different directional slots.
#endif
vector<int> T_i_sonar_slots;

#ifdef English_dox
//! Infrared sensors are located at which slots on the evarobot. There are 12 different directional slots.
#endif
vector<int> T_i_infrared_slots;

#ifdef English_dox
//! Range values of sonar sensors which are subscribed. Maximum number of sonar sensors on the evarobot is 12.
#endif
float i_sonar_values[12];

#ifdef English_dox
//! Range values of infrared sensors which are subscribed. Maximum number of infrared sensors on the evarobot is 12.
#endif
float i_infrared_values[12];

#ifdef English_dox
//! Subscriber class for both sonar and infrared topics.
#endif
class IMSUBSCRIBER
{
	public:
		#ifdef English_dox
		//! Constructor
		/**
		 * \param str_topic_name Name of the topic to subscribe.
		 * \param i_slot_id ID of the slot where sensor is located.
		 * \param n ROS node handler.
		 * \param b_sonar This topic is sonar or not (infrared).
		 */
		#endif
		IMSUBSCRIBER(string str_topic_name, int i_slot_id, ros::NodeHandle n, bool b_sonar)
		{
			this->i_slot_id = i_slot_id;
			this->b_sonar = b_sonar;
			this->str_topic_name = str_topic_name;
			this->n = n;
		}

		#ifdef English_dox
		//! Creates subscriber to topic which is created in constructor.
		#endif
		void SubscribeToTopic()
		{
			sub = n.subscribe(str_topic_name.c_str(), 1000, &IMSUBSCRIBER::subscriberCallback, this);
		}

		#ifdef English_dox
		//! Callback function of subscriber.
		/**
		 * \param msg Message from subscribed topic. Range data of msg is saved to publish.
		 */
		#endif
		void subscriberCallback(const sensor_msgs::Range::ConstPtr& msg)
		{
			if(b_sonar)
				i_sonar_values[i_slot_id + ((i_slot_id-7)*(-2))] = msg->range * 1000;
			else
				i_infrared_values[i_slot_id + ((i_slot_id-7)*(-2))] = msg->range * 1000;
		}

		#ifdef English_dox
		//! ID of the slot where sensor is located.
		#endif
		int i_slot_id;

		#ifdef English_dox
		//! This topic is sonar or not (infrared).
		#endif
		bool b_sonar;
		#ifdef English_dox
		//! Name of the topic to subscribe.
		#endif
		string str_topic_name;
		#ifdef English_dox
		//! ROS node handler.
		#endif
		ros::NodeHandle n;
		#ifdef English_dox
		//! Subscriber to subscribe topic.
		#endif
		ros::Subscriber sub;
};
