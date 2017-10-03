#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

using namespace std;

vector<int> T_i_sonar_slots;

vector<int> T_i_infrared_slots;

float i_sonar_values[12];

float i_infrared_values[12];



class IMSUBSCRIBER
{
	public:
		IMSUBSCRIBER(string str_topic_name, int i_slot_id, ros::NodeHandle n, bool b_sonar)
		{
			this->i_slot_id = i_slot_id;
			this->b_sonar = b_sonar;	
			this->str_topic_name = str_topic_name;
			this->n = n;
		}
		
		void SubscribeToTopic()
		{
			sub = n.subscribe(str_topic_name.c_str(), 1000, &IMSUBSCRIBER::subscriberCallback, this);
		}

		void subscriberCallback(const sensor_msgs::Range::ConstPtr& msg)
		{
			if(b_sonar)
				i_sonar_values[i_slot_id + ((i_slot_id-7)*(-2))] = msg->range * 1000;
			else
				i_infrared_values[i_slot_id + ((i_slot_id-7)*(-2))] = msg->range * 1000;
		}
		
		int i_slot_id;
		bool b_sonar;
		string str_topic_name;
		ros::NodeHandle n;
		ros::Subscriber sub;
};

/*
void subscriberCallback(const sensor_msgs::Range::ConstPtr& msg)
{
	ROS_INFO("Sonar Seq: [%d]", msg->header.seq);
	ROS_INFO("Sonar Range: [%f]", msg->range);
}*/



int main(int argc, char **argv)
{
  for(uint i=0; i<12; i++)
  {
	  i_sonar_values[i] = -1;
	  i_infrared_values[i] = -1;
  }
  
  ros::init(argc, argv, "evarobot_android");

  ros::NodeHandle n;
  /*
  ros::Subscriber sub1 = n.subscribe("sonar0", 1000, subscriberCallback);
  ros::Subscriber sub2 = n.subscribe("sonar1", 1000, subscriberCallback);
  ros::Subscriber sub3 = n.subscribe("sonar2", 1000, subscriberCallback);
  ros::Subscriber sub4 = n.subscribe("sonar3", 1000, subscriberCallback);
  ros::Subscriber sub5 = n.subscribe("sonar4", 1000, subscriberCallback);
  ros::Subscriber sub6 = n.subscribe("sonar5", 1000, subscriberCallback);
  ros::Subscriber sub7 = n.subscribe("sonar6", 1000, subscriberCallback);
  ros::Subscriber sub8 = n.subscribe("sonar0", 1000, subscriberCallback);
  ros::Subscriber sub9 = n.subscribe("ir0", 1000, subscriberCallback);
  ros::Subscriber sub10 = n.subscribe("ir1", 1000, subscriberCallback);
  ros::Subscriber sub11 = n.subscribe("ir2", 1000, subscriberCallback);
  */

  n.getParam("evarobot_android/SonarSlots", T_i_sonar_slots);

  n.getParam("evarobot_android/InfraredSlots", T_i_infrared_slots);

  if(T_i_sonar_slots.size() <= 0 && T_i_infrared_slots.size() <= 0)
  {
	ROS_WARN("evarobot_android: There are no sonar or infrared");
  }

  std::ostringstream ss;
  
  for(uint i = 0; i < T_i_sonar_slots.size(); i++)
  {
	  ss<<"/sonar"<<i;
	  IMSUBSCRIBER* subscriber = new IMSUBSCRIBER(ss.str(), T_i_sonar_slots[i], n, true);
	  subscriber->SubscribeToTopic();
	  ss.str("");
  }
  
  for(uint i = 0; i < T_i_infrared_slots.size(); i++)
  {
	  ss<<"/ir"<<i;
	  IMSUBSCRIBER* subscriber = new IMSUBSCRIBER(ss.str(), T_i_infrared_slots[i], n, false);
	  subscriber->SubscribeToTopic();
	  ss.str("");
  }
  

  ros::Publisher evamobil_pub = n.advertise<std_msgs::String>("evamobil", 1000);

  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    std_msgs::String msg;

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
	
	ss<<"2";
    msg.data = ss.str();
    evamobil_pub.publish(msg);
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
