#include "ekb_test/ekb_test.h"

using namespace std;

/**
 * Number of tests for a sensor.
 */
int i_count = 0;

/**
 * Remaining number of tests for a sensor.
 */
int i_count_gec = 0;

/**
 * Number of messages from current sensor during motor test.
 */
int i_count_akim = 0;

/**
 * If true, test selection menu is shown.
 * Else test is in progress and menu is not shown.
 */
bool b_menu = true;

/**
 * Last time callback function is called for currently testing sensor.
 */
double d_secs_last_message;

/**
 * Current time in ROS.
 */
double d_secs_while;

/**
 * Determines timeout control is enabled or not.
 */
bool b_time_control = false;

/**
 * Name of sensor which is currently in test.
 */
string str_aktif_sensor = "";

/**
 * Stores encoder test results. These data is printed on the screen when the motor test is completed.
 */
std::vector<std::string> str_enkoder;

/**
 * Stores current test results. These data is printed on the screen when the motor test is completed.
 */
std::vector<std::string> str_akim;

/**
 * Current test is done or not during motor test.
 */
bool b_akim = false;

/**
 * Encoder test is done or not during motor test.
 */
bool b_enkoder = false;

/**
 * Service client to start sonar sensors.
 */
ros::ServiceClient client_sonar;

/**
 * Service client to start infrared sensors.
 */
ros::ServiceClient client_ir;

/**
 * Service client to start bumper sensors.
 */
ros::ServiceClient client_bumper;

/**
 * Service client to start evarobot controller.
 */
ros::ServiceClient client_motor;

/**
 * Service client to start encoder sensors.
 */
ros::ServiceClient client_encoder;

/**
 * Service client to start IMU sensor.
 */
ros::ServiceClient client_imu;

/**
 * Service client to start current sensor.
 */
ros::ServiceClient client_akim;

/**
 * Service client to start battery sensor.
 */
ros::ServiceClient client_batarya;

/**
 * Service client to start EIO pins.
 */
ros::ServiceClient client_eio;

/**
 * Publisher for cmd_vel topic to apply velocities.
 */
ros::Publisher pub_motor;

/**
 * Callback function for sonar0 topic in type of sensor_msgs::Range. 
 */
void sonar0Callback(const sensor_msgs::Range::ConstPtr& msg)
{
	/**
	 * If current sensor is sonar0 these block is run.
	 */
	if(str_aktif_sensor=="sonar0")
	{
		/**
		 * Test number and range data from sonar0 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			cout<<"Sonar0 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<msg->range<<endl;
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for sonar0.
		 * Information message is printed to prepare sonar1 for testing.
		 */
		else
		{
			b_time_control = false;
			i_count_gec = i_count;
			cout<<"Kabloyu sonar1 pinlerine takin ve enter tusuna basin.";
			cin.ignore();
			d_secs_last_message = ros::Time::now().toSec();
			b_time_control = true;
			str_aktif_sensor = "sonar1";
		}
	}
}

/**
 * Callback function for sonar1 topic in type of sensor_msgs::Range. 
 */
void sonar1Callback(const sensor_msgs::Range::ConstPtr& msg)
{
	/**
	 * If current sensor is sonar1 these block is run.
	 */
	if(str_aktif_sensor=="sonar1")
	{
		/**
		 * Test number and range data from sonar1 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			cout<<"Sonar1 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<msg->range<<endl;
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for sonar1.
		 * Information message is printed to prepare sonar2 for testing.
		 */
		else
		{
			b_time_control = false;
			i_count_gec = i_count;
			cout<<"Kabloyu sonar2 pinlerine takin ve enter tusuna basin.";
			cin.ignore();
			d_secs_last_message = ros::Time::now().toSec();
			b_time_control = true;
			str_aktif_sensor = "sonar2";
		}
	}
}

/**
 * Callback function for sonar2 topic in type of sensor_msgs::Range. 
 */
void sonar2Callback(const sensor_msgs::Range::ConstPtr& msg)
{
	/**
	 * If current sensor is sonar2 these block is run.
	 */
	if(str_aktif_sensor=="sonar2")
	{
		/**
		 * Test number and range data from sonar2 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			cout<<"Sonar2 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<msg->range<<endl;
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for sonar2.
		 * Information message is printed to prepare sonar3 for testing.
		 */
		else
		{
			b_time_control = false;
			i_count_gec = i_count;
			cout<<"Kabloyu sonar3 pinlerine takin ve enter tusuna basin.";
			cin.ignore();
			d_secs_last_message = ros::Time::now().toSec();
			b_time_control = true;
			str_aktif_sensor = "sonar3";
		}
	}
}

/**
 * Callback function for sonar3 topic in type of sensor_msgs::Range. 
 */
void sonar3Callback(const sensor_msgs::Range::ConstPtr& msg)
{
	/**
	 * If current sensor is sonar3 these block is run.
	 */
	if(str_aktif_sensor=="sonar3")
	{
		/**
		 * Test number and range data from sonar3 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			cout<<"Sonar3 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<msg->range<<endl;
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for sonar3.
		 * Information message is printed to prepare sonar4 for testing.
		 */
		else
		{
			b_time_control = false;
			i_count_gec = i_count;
			cout<<"Kabloyu sonar4 pinlerine takin ve enter tusuna basin.";
			cin.ignore();
			d_secs_last_message = ros::Time::now().toSec();
			b_time_control = true;
			str_aktif_sensor = "sonar4";
		}
	}
}

/**
 * Callback function for sonar4 topic in type of sensor_msgs::Range. 
 */
void sonar4Callback(const sensor_msgs::Range::ConstPtr& msg)
{
	/**
	 * If current sensor is sonar4 these block is run.
	 */
	if(str_aktif_sensor=="sonar4")
	{
		/**
		 * Test number and range data from sonar4 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			cout<<"Sonar4 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<msg->range<<endl;
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for sonar4.
		 * Information message is printed to prepare sonar5 for testing.
		 */
		else
		{
			b_time_control = false;
			i_count_gec = i_count;
			cout<<"Kabloyu sonar5 pinlerine takin ve enter tusuna basin.";
			cin.ignore();
			d_secs_last_message = ros::Time::now().toSec();
			b_time_control = true;
			str_aktif_sensor = "sonar5";
		}
	}
}

/**
 * Callback function for sonar5 topic in type of sensor_msgs::Range. 
 */
void sonar5Callback(const sensor_msgs::Range::ConstPtr& msg)
{
	/**
	 * If current sensor is sonar5 these block is run.
	 */
	if(str_aktif_sensor=="sonar5")
	{
		/**
		 * Test number and range data from sonar5 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			cout<<"Sonar5 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<msg->range<<endl;
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for sonar5.
		 * Information message is printed to prepare sonar6 for testing.
		 */
		else
		{
			b_time_control = false;
			i_count_gec = i_count;
			cout<<"Kabloyu sonar6 pinlerine takin ve enter tusuna basin.";
			cin.ignore();
			d_secs_last_message = ros::Time::now().toSec();
			b_time_control = true;
			str_aktif_sensor = "sonar6";
		}
	}
}

/**
 * Callback function for sonar6 topic in type of sensor_msgs::Range. 
 */
void sonar6Callback(const sensor_msgs::Range::ConstPtr& msg)
{
	/**
	 * If current sensor is sonar6 these block is run.
	 */
	if(str_aktif_sensor=="sonar6")
	{
		/**
		 * Test number and range data from sonar6 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			cout<<"Sonar6 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<msg->range<<endl;
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for sonar6.
		 * Tests done for sonar sensors.
		 * b_menu is set as true to show test selection menu.
		 */
		else
		{
			b_time_control = false;
			str_aktif_sensor = "";
			b_menu = true;
		}
	}
}

/**
 * Callback function for ir0 topic in type of sensor_msgs::Range. 
 */
void ir0Callback(const sensor_msgs::Range::ConstPtr& msg)
{
	/**
	 * If current sensor is ir0 these block is run.
	 */
	if(str_aktif_sensor=="ir0")
	{
		/**
		 * Test number and range data from ir0 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			cout<<"Kızılötesi0 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<msg->range<<endl;
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for ir0.
		 * Information message is printed to prepare ir1 for testing.
		 */
		else
		{
			b_time_control = false;
			i_count_gec = i_count;
			cout<<"Kabloyu ir1 pinlerine takin ve enter tusuna basin.";
			cin.ignore();
			d_secs_last_message = ros::Time::now().toSec();
			b_time_control = true;
			str_aktif_sensor = "ir1";
		}
	}
}

/**
 * Callback function for ir1 topic in type of sensor_msgs::Range. 
 */
void ir1Callback(const sensor_msgs::Range::ConstPtr& msg)
{
	/**
	 * If current sensor is ir1 these block is run.
	 */
	if(str_aktif_sensor=="ir1")
	{
		/**
		 * Test number and range data from ir1 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			cout<<"Kızılötesi1 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<msg->range<<endl;
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for ir1.
		 * Information message is printed to prepare ir2 for testing.
		 */
		else
		{
			b_time_control = false;
			i_count_gec = i_count;
			cout<<"Kabloyu ir2 pinlerine takin ve enter tusuna basin.";
			cin.ignore();
			d_secs_last_message = ros::Time::now().toSec();
			b_time_control = true;
			str_aktif_sensor = "ir2";
		}
	}
}

/**
 * Callback function for ir2 topic in type of sensor_msgs::Range. 
 */
void ir2Callback(const sensor_msgs::Range::ConstPtr& msg)
{
	/**
	 * If current sensor is ir2 these block is run.
	 */
	if(str_aktif_sensor=="ir2")
	{
		/**
		 * Test number and range data from ir2 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			cout<<"Kızılötesi2 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<msg->range<<endl;
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for ir2.
		 * Information message is printed to prepare ir3 for testing.
		 */
		else
		{
			b_time_control = false;
			i_count_gec = i_count;
			cout<<"Kabloyu ir3 pinlerine takin ve enter tusuna basin.";
			cin.ignore();
			d_secs_last_message = ros::Time::now().toSec();
			b_time_control = true;
			str_aktif_sensor = "ir3";
		}
	}
}

/**
 * Callback function for ir3 topic in type of sensor_msgs::Range. 
 */
void ir3Callback(const sensor_msgs::Range::ConstPtr& msg)
{
	/**
	 * If current sensor is ir0 these block is run.
	 */
	if(str_aktif_sensor=="ir3")
	{
		/**
		 * Test number and range data from ir3 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			cout<<"Kızılötesi3 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<msg->range<<endl;
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for ir3.
		 * Information message is printed to prepare ir4 for testing.
		 */
		else
		{
			b_time_control = false;
			i_count_gec = i_count;
			cout<<"Kabloyu ir4 pinlerine takin ve enter tusuna basin.";
			cin.ignore();
			d_secs_last_message = ros::Time::now().toSec();
			b_time_control = true;
			str_aktif_sensor = "ir4";
		}
	}
}

/**
 * Callback function for ir4 topic in type of sensor_msgs::Range. 
 */
void ir4Callback(const sensor_msgs::Range::ConstPtr& msg)
{
	/**
	 * If current sensor is ir4 these block is run.
	 */
	if(str_aktif_sensor=="ir4")
	{
		/**
		 * Test number and range data from ir4 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			cout<<"Kızılötesi4 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<msg->range<<endl;
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for ir4.
		 * Information message is printed to prepare ir5 for testing.
		 */
		else
		{
			b_time_control = false;
			i_count_gec = i_count;
			cout<<"Kabloyu ir5 pinlerine takin ve enter tusuna basin.";
			cin.ignore();
			d_secs_last_message = ros::Time::now().toSec();
			b_time_control = true;
			str_aktif_sensor = "ir5";
		}
	}
}

/**
 * Callback function for ir5 topic in type of sensor_msgs::Range. 
 */
void ir5Callback(const sensor_msgs::Range::ConstPtr& msg)
{
	/**
	 * If current sensor is ir5 these block is run.
	 */
	if(str_aktif_sensor=="ir5")
	{
		/**
		 * Test number and range data from ir5 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			cout<<"Kızılötesi5 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<msg->range<<endl;
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for ir5.
		 * Information message is printed to prepare ir6 for testing.
		 */
		else
		{
			b_time_control = false;
			i_count_gec = i_count;
			cout<<"Kabloyu ir6 pinlerine takin ve enter tusuna basin.";
			cin.ignore();
			d_secs_last_message = ros::Time::now().toSec();
			b_time_control = true;
			str_aktif_sensor = "ir6";
		}
	}
}

/**
 * Callback function for ir6 topic in type of sensor_msgs::Range. 
 */
void ir6Callback(const sensor_msgs::Range::ConstPtr& msg)
{
	/**
	 * If current sensor is ir6 these block is run.
	 */
	if(str_aktif_sensor=="ir6")
	{
		/**
		 * Test number and range data from ir6 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			cout<<"Kızılötesi6 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<msg->range<<endl;
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for ir6.
		 * Information message is printed to prepare ir7 for testing.
		 */
		else
		{
			b_time_control = false;
			i_count_gec = i_count;
			cout<<"Kabloyu ir7 pinlerine takin ve enter tusuna basin.";
			cin.ignore();
			d_secs_last_message = ros::Time::now().toSec();
			b_time_control = true;
			str_aktif_sensor = "ir7";
		}
	}
}

/**
 * Callback function for ir7 topic in type of sensor_msgs::Range. 
 */
void ir7Callback(const sensor_msgs::Range::ConstPtr& msg)
{
	/**
	 * If current sensor is ir7 these block is run.
	 */
	if(str_aktif_sensor=="ir7")
	{
		/**
		 * Test number and range data from ir7 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			cout<<"Kızılötesi7 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<msg->range<<endl;
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for ir7.
		 * Tests done for infrared sensors.
		 * b_menu is set as true to show test selection menu.
		 */
		else
		{
			b_time_control = false;
			str_aktif_sensor = "";
			b_menu = true;
		}
	}
}

/**
 * Callback function for bumper topic in type of im_msgs::Bumper.
 * Input message for this callback contains al of the three bumpers' data.
 */
void bumperCallback(const im_msgs::Bumper::ConstPtr& msg)
{
	/**
	 * If current sensor is bumper0 these block is run.
	 */
	if(str_aktif_sensor=="bumper0")
	{
		/**
		 * Test number and collicision data from bumper0 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			if(msg->state[0].bumper_state)
			{
				cout<<"Bumper0 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<"true"<<endl;
			}
			else
			{
				cout<<"Bumper0 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<"false"<<endl;
			}
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for bumper0.
		 * Information message is printed to prepare bumper1 for testing.
		 */
		else
		{
			b_time_control = false;
			i_count_gec = i_count;
			cout<<"Kabloyu bumper1 pinlerine takin ve enter tusuna basin.";
			cin.ignore();
			b_time_control = true;
			str_aktif_sensor = "bumper1";
		}
	}
	/**
	 * If current sensor is bumper1 these block is run.
	 */
	else if(str_aktif_sensor=="bumper1")
	{
		/**
		 * Test number and collicision data from bumper1 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			if(msg->state[1].bumper_state)
			{
				cout<<"Bumper1 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<"true"<<endl;
			}
			else
			{
				cout<<"Bumper1 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<"false"<<endl;
			}
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for bumper1.
		 * Information message is printed to prepare bumper2 for testing.
		 */
		else
		{
			b_time_control = false;
			i_count_gec = i_count;
			cout<<"Kabloyu bumper2 pinlerine takin ve enter tusuna basin.";
			cin.ignore();
			d_secs_last_message = ros::Time::now().toSec();
			b_time_control = true;
			str_aktif_sensor = "bumper2";
		}
	}
	/**
	 * If current sensor is bumper2 these block is run.
	 */
	else if(str_aktif_sensor=="bumper2")
	{
		/**
		 * Test number and collicision data from bumper2 sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			if(msg->state[2].bumper_state)
			{
				cout<<"Bumper2 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<"true"<<endl;
			}
			else
			{
				cout<<"Bumper2 -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<"false"<<endl;
			}
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for bumper2.
		 * Tests done for bumper sensors.
		 * b_menu is set as true to show test selection menu.
		 */
		else
		{
			b_time_control = false;
			str_aktif_sensor = "";
			b_menu = true;
		}
	}	
}

/**
 * Callback function for wheel_vel topic in type of im_msgs::WheelVel. 
 */
void encoderCallback(const im_msgs::WheelVel::ConstPtr& msg)
{
	/**
	 * If current sensor is encoder these block is run.
	 */
	if(str_aktif_sensor=="enkoder")
	{
		/**
		 * Test number and encoder data for both left and right wheels are printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			if(msg->right_vel != 0.0 && msg->left_vel != 0.0)
			{
				std::ostringstream stream;
				stream << "Enkoder Sensörü -> "<<"Test " << (i_count-i_count_gec)<<" -> \t" <<"Sol Teker : "<< fixed << setprecision(4) << msg->left_vel<<"\tSag teker : "<< fixed << setprecision(4)<< msg->right_vel<<endl;
				str_enkoder.push_back(stream.str());
				d_secs_last_message = ros::Time::now().toSec();
				i_count_gec--;
			}
		}
		/**
		 * Test is done for encoder.
		 */
		else
		{
			b_enkoder = true;
		}

		/**
		 * If tests are done for both current and encoder sensor, print data.
		 */
		if(b_akim && b_enkoder)
		{
			for(int t=0;t<str_akim.size();t++)
			{
				cout<<str_akim.at(t);
			}
			
            for(int t=0;t<str_enkoder.size();t++)
            {
				cout<<str_enkoder.at(t);
            }
            
            /**
             * Data are cleared.
             * b_menu is set as true to show test selection menu.
             */
			str_akim.clear();
			str_enkoder.clear();
			b_time_control = false;
			geometry_msgs::Twist msg;
			msg.angular.z = 0;
			msg.linear.x = 0;
			pub_motor.publish(msg);
			str_aktif_sensor = "";
			b_akim=false;
			b_enkoder=false;
			b_menu = true;
		}
	}
}

/**
 * Callback function for motor_voltages topic in type of im_msgs::Voltage. 
 */
void akimCallback(const im_msgs::Voltage::ConstPtr& msg)
{
	/**
	 * If current sensor is encoder these block is run.
	 * Current test and encoder test run together.
	 */
	if(str_aktif_sensor=="enkoder")
	{
		/**
		 * Test number and current data for both left and right wheels are printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_akim>0)
		{
			std::ostringstream stream;
			stream << "Akim Sensörü -> "<<"Test " << (i_count-i_count_akim)<<" -> \t" <<"Sol Teker : "<< fixed << setprecision(4) << msg->left_motor<<"\tSag teker : "<< fixed << setprecision(4)<< msg->right_motor<<endl;
			str_akim.push_back(stream.str());
			d_secs_last_message = ros::Time::now().toSec();
			i_count_akim--;
		}
		/**
		 * Test is done for current.
		 */
		else
		{
			b_akim = true;
		}
	}
}

/**
 * Callback function for imu topic in type of sensor_msgs::Imu. 
 */
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	/**
	 * If current sensor is imu these block is run.
	 */
	if(str_aktif_sensor=="imu")
	{
		/**
		 * Test number and imu data are printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			cout<<"IMU -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<"orientation.x : "<< fixed << setprecision(4) << msg->orientation.x<<"\torientation.y : "<< fixed << setprecision(4)<< msg->orientation.y<<"\torientation.z : "<< fixed << setprecision(4) <<msg->orientation.z<<"\torientation.w : "<< fixed << setprecision(4)<< msg->orientation.w<<endl;
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for imu.
		 * b_menu is set as true to show test selection menu.
		 */
		else
		{
			b_time_control = false;
			str_aktif_sensor = "";
			b_menu = true;
		}
	}
}

/**
 * Callback function for battery topic in type of im_msgs::Battery.
 */
void bataryaCallback(const im_msgs::Battery::ConstPtr& msg)
{
	/**
	 * If current sensor is battery these block is run.
	 */
	if(str_aktif_sensor=="batarya sensörü")
	{
		/**
		 * Test number, current and voltage data are printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			cout<<"Batarya -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<"voltaj : "<< fixed << setprecision(4) << msg->voltage<<"\tAkim : "<< fixed << setprecision(4)<< msg->current<<endl;
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for battery.
		 * b_menu is set as true to show test selection menu.
		 */
		else
		{
			b_time_control = false;
			str_aktif_sensor = "";
			b_menu = true;
		}
	}
}

/**
 * This function starts tests for sonar sensors. 
 */
void SonarTesti()
{
	/**
	 * Service client is called to start sonar sensors.
	 */
	im_msgs::EKBTest srv;
	srv.request.b_on_off = true;
	client_sonar.call(srv);
	
	/**
	 * Get number of tests for a sonar sensor.
	 */
	cout<<"Sonar sensörler için gerçekleştirilecek test sayisini [0 - 10] girin."<<endl;
	cin>>i_count;
	i_count_gec = i_count;
	
	/**
	 * Information message is printed to prepare sonar0 for testing.
	 */
	cout<<"Kabloyu sonar0 pinlerine takın ve enter tusuna basin.";
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(), '\n');
	cin.ignore();
	
	/**
	 * Current sensor is set as sonar0.
	 */
	d_secs_last_message = ros::Time::now().toSec();
	b_time_control = true;
	str_aktif_sensor = "sonar0";
}

/**
 * This function starts tests for infrared sensors.
 */
void KizilotesiTesti()
{
	/**
	 * Service client is called to start infrared sensors.
	 */
	im_msgs::EKBTest srv;
	srv.request.b_on_off = true;
	client_ir.call(srv);
	
	/**
	 * Get number of tests for a infrared sensor.
	 */
	cout<<"Kizilötesi sensörler için gerçeklestirilecek test sayisini [0 - 10] girin."<<endl;
	cin>>i_count;
	i_count_gec = i_count;
	
	/**
	 * Information message is printed to prepare ir0 for testing.
	 */
	cout<<"Kabloyu ir0 pinlerine takin ve enter tusuna basin.";
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(), '\n');
	cin.ignore();
	
	/**
	 * Current sensor is set as ir0.
	 */
	d_secs_last_message = ros::Time::now().toSec();
	b_time_control = true;
	str_aktif_sensor = "ir0";
}

/**
 * This function starts test for bumper sensor. 
 */
void BumperTesti()
{
	/**
	 * Service client is called to start bumper sensor.
	 */
	im_msgs::EKBTest srv;
	srv.request.b_on_off = true;
	client_bumper.call(srv);
	
	/**
	 * Get number of tests for bumper sensor.
	 */
	cout<<"Bumper sensörler için gerçekleştirilecek test sayısını [0 - 10] girin."<<endl;
	cin>>i_count;
	i_count_gec = i_count;
	
	/**
	 * Information message is printed to prepare bumper for testing.
	 */
	cout<<"Kabloyu bumper0 pinlerine takin ve enter tusuna basin.";
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(), '\n');
	cin.ignore();
	
	/**
	 * Current sensor is set as bumper.
	 */
	d_secs_last_message = ros::Time::now().toSec();
	b_time_control = true;
	str_aktif_sensor = "bumper0";
}

/**
 * This function starts tests for encoder sensor and motor. 
 */
void MotorVeEnkoderTesti(bool b_ileri)
{
	/**
	 * Service client is called to start encoder sensor and evarobot_controller.
	 */
	im_msgs::EKBTest srv;
	srv.request.b_on_off = true;
	client_motor.call(srv);
	client_encoder.call(srv);
	client_akim.call(srv);
	
	/**
	 * Get number of tests for encoder sensor.
	 */
	cout<<"Motor ve enkoder için gerçekleştirilecek test sayisini [0 - 10] girin."<<endl;
	cin>>i_count;
	i_count_gec = i_count;
	i_count_akim = i_count;
	
	/**
	 * Information message is printed to prepare motor for testing.
	 * If b_ileri is set as true move forward (linear velocity is positive). Else, move backward (linear velocity is negative).
	 */
	cout<<"Kabloyu motor pinlerine takin, motor güçlerini baglayin ve enter tusuna basin.";
	if(b_ileri)
	{
		cout<<"Test bittiginde enkoder sensorunden okunan sol ve sag teker degerlerini kontrol edin. Degerler 0.25 0.35 arasi olmalidir. Aksi takdirde motor arizalidir."<<endl;
		cin.clear();
		cin.ignore(numeric_limits<streamsize>::max(), '\n');
		cin.ignore();
		geometry_msgs::Twist msg;
		msg.angular.z = 0.0;
		msg.linear.x = 0.3;
		pub_motor.publish(msg);
	}
	else
	{
		cout<<"Test bittiginde enkoder sensorunden okunan sol ve sag teker degerlerini kontrol edin. Degerler -0.25 -0.35 arasi olmalidir. Aksi takdirde motor arizalidir."<<endl;
		cin.clear();
		cin.ignore(numeric_limits<streamsize>::max(), '\n');
		cin.ignore();
		geometry_msgs::Twist msg;
		msg.angular.z = 0;
		msg.linear.x = -0.35;
		pub_motor.publish(msg);
	}
	
	/**
	 * Current sensor is set as encoder.
	 */
	str_aktif_sensor = "enkoder";
	d_secs_last_message = ros::Time::now().toSec();
	b_time_control = true;
}

/**
 * This function starts tests for IMU sensor. 
 */
void IMUTesti()
{
	/**
	 * Get number of tests for IMU sensor.
	 */
	cout<<"IMU sensörü için gerçeklestirilecek test sayisini [0 - 10] girin."<<endl;
	cin>>i_count;
	i_count_gec = i_count;
	
	/**
	 * Information message is printed to prepare IMU for testing.
	 */
	cout<<"Kabloyu IMU pinlerine takin ve enter tusuna basin.";
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(), '\n');
	cin.ignore();
	
	/**
	 * Service client is called to start IMU sensor.
	 */
	im_msgs::EKBTest srv;
	srv.request.b_on_off = true;
	client_imu.call(srv);
	
	/**
	 * Current sensor is set as imu.
	 */
	str_aktif_sensor = "imu";
	d_secs_last_message = ros::Time::now().toSec();
	b_time_control = true;
}

/**
 * This function starts tests for battery sensor. 
 */
void BataryaTesti()
{
	/**
	 * Service client is called to start battery sensor.
	 */
	im_msgs::EKBTest srv;
	srv.request.b_on_off = true;
	client_batarya.call(srv);
	
	/**
	 * Get number of tests for battery sensor.
	 */
	cout<<"Batarya sensörü için gerçekleştirilecek test sayısını [0 - 10] girin."<<endl;
	cin>>i_count;
	i_count_gec = i_count;
	
	/**
	 * Information message is printed to prepare battery for testing.
	 */
	cout<<"Kabloyu batarya sensörü pinlerine takin ve enter tusuna basin.";
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(), '\n');
	cin.ignore();
	
	/**
	 * Current sensor is set as battery.
	 */
	str_aktif_sensor = "batarya sensörü";
	d_secs_last_message = ros::Time::now().toSec();
	b_time_control = true;
}

/**
 * This function start test for a EIO pin.
 * Pin is tested in outpud mode. A led is used to see output.
 */
void EIOPinModTest(int pin, string str, string str_basari)
{
	/**
	 * Information message is printed to prepare GPIO pin for testing.
	 */
	im_msgs::EIOTest srv;
	cout<<"Led'i " << str << " pinlerine takin ve enter tusuna basin.";
	cin.ignore();
	if(pin==1)
		cin.ignore();
		
	/**
	 * Service client is called to set GPIO pin as output.
	 */
	srv.request.pin = pin;
	srv.request.mode = 1; 
	srv.request.value = true; 
	srv.request.portno = 1; 
	client_eio.call(srv);
	if(!srv.response.b_success)
	{
		cout<< str << " arizali"<<endl;
	}
	else
	{
		cout<< str_basari << " enter tusuna basin."<<endl;
		cin.ignore();
	}
    srv.request.value = false; 
	srv.request.portno = 1; 
    client_eio.call(srv);
}

/**
 * This function start test for a EIO pin.
 * Pin in is tested in inpud mode. A button is used to get input.
 */
void EIOPinModTest2(int pin, string str)
{
	/**
	 * Service client is called to set GPIO pin as input.
	 */
	im_msgs::EIOTest srv;
	srv.request.pin = pin;
	srv.request.mode = 0; 
	srv.request.value = false;
	srv.request.portno = 1; 
	client_eio.call(srv);

	/**
	 * Information message is printed to prepare GPIO pin for testing.
	 */
	cout<<"Buton'u " << str << " pinlerine takın, butona basili tutun ve enter tusuna basin.";
	cin.ignore();
	client_eio.call(srv);
	if(!srv.response.b_success)
	{
		cout << str << " arızalı"<<endl;
	}
	else
	{
		if(srv.response.b_data)
		{
			cout << str << " pini başarı ile okundu." << endl;
		}
		else
		{
			cout << "[HATA]" << str << " pinden dogru deger okunmadi." << endl;
			cout << "Tekrar denediginizde butona basili oldugundan emin olunuz." << endl;
		}
	}
}

/**
 * This function starts tests for GPIO pins. 
 */
void EIOTesti()
{
	/**
	 * Test GPIO pins 1, 2, ..., 7 in output mode.
	 */
	EIOPinModTest(1, "EIO0", "Led yandiginda");
	EIOPinModTest(2, "EIO1", "Led yandiginda");
	EIOPinModTest(3, "EIO2", "Led yandiginda");
	EIOPinModTest(4, "EIO3", "Led yandiginda");
	EIOPinModTest(5, "EIO4", "Led yandiginda");
	EIOPinModTest(6, "EIO5", "Led yandiginda");
	EIOPinModTest(7, "EIO6", "Led yandiginda");
	
	/**
	 * Test for buzzer.
	 */
	im_msgs::EIOTest srv;
	srv.request.pin = 3;
	srv.request.mode = 1;
	srv.request.portno = 0; 
	srv.request.value = true;
	client_eio.call(srv);
	if(!srv.response.b_success)
	{
		cout<< "Buzzer arizali"<<endl;
	}
	else
	{
		cout<< "Buzzer öttügünde enter tusuna basin.";
		cin.ignore();
	}
    srv.request.value = false;
    client_eio.call(srv);
	
	/**
	 * Test LED on the EKB.
	 */
	srv.request.pin = 7;
	srv.request.mode = 1; 
	srv.request.portno = 0; 
	srv.request.value = true;
	client_eio.call(srv);
	if(!srv.response.b_success)
	{
		cout<< "EKB üzerindeki led arizali"<<endl;
	}
	else
	{
		cout<< "EKB üzerindeki led yandiginda enter tusuna basin.";
		cin.ignore();
	}
        srv.request.value = false;
        client_eio.call(srv);

	/**
	 * Test GPIO pins 1, 2, ..., 7 and reset button in input mode.
	 */
	EIOPinModTest2(0, "RESET");
	EIOPinModTest2(7, "EIO6");
	EIOPinModTest2(6, "EIO5");
	EIOPinModTest2(5, "EIO4");
	EIOPinModTest2(4, "EIO3");
	EIOPinModTest2(3, "EIO2");
	EIOPinModTest2(2, "EIO1");
	EIOPinModTest2(1, "EIO0");
	
	/**
	 * Test is done for GPIO pins.
	 * b_menu is set as true to show test selection menu.
	 */
	str_aktif_sensor = "";
	b_menu = true;
	d_secs_last_message = ros::Time::now().toSec();
}

/**
 * This function prints available tests on the screen.
 */
void MenuyuGoster()
{
	cout<<"************EKB TEST YAZILIMI***********"<<endl;
	cout<<"1. Sonar Testi"<<endl;
	cout<<"2. Kizilötesi Testi"<<endl;
	cout<<"3. Bumper Testi"<<endl;
	cout<<"4. Ileri Motor, Enkoder ve Akim Sensörü Testi"<<endl;
	cout<<"5. Geri Motor, Enkoder ve Akim Sensörü Testi"<<endl;
	cout<<"6. IMU Testi"<<endl;
	cout<<"7. Batarya Sensörü Testi"<<endl;
	cout<<"8. EIO Testi"<<endl;
	cout<<"Çikis için -1 giriniz."<<endl;
}

/**
 * Program starts here.
 */
int main(int argc, char **argv)
{
	/**
	 * Initializes ROS node with ekb_test name.
	 */
	ros::init(argc, argv, "/ekb_test");
	
	/**
	 * Creates ROS node handler.
	 */
	ros::NodeHandle n;

	/**
	 * Waits until ROS OK.
	 */
	while(!ros::ok()){}

	/**
	 * Service clients are created to start sensors.
	 */
	client_sonar = n.serviceClient<im_msgs::EKBTest>("evarobot_sonar/ekb_test");
	client_ir = n.serviceClient<im_msgs::EKBTest>("evarobot_infrared/ekb_test");
	client_bumper = n.serviceClient<im_msgs::EKBTest>("evarobot_bumper/ekb_test");
	client_motor = n.serviceClient<im_msgs::EKBTest>("evarobot_controller/ekb_test");
	client_encoder = n.serviceClient<im_msgs::EKBTest>("evarobot_odometry/ekb_test");
	client_imu = n.serviceClient<im_msgs::EKBTest>("evarobot_minimu9/ekb_test");
	client_akim = n.serviceClient<im_msgs::EKBTest>("evarobot_driver/ekb_test");
	client_batarya = n.serviceClient<im_msgs::EKBTest>("evarobot_battery/ekb_test");
	client_eio = n.serviceClient<im_msgs::EIOTest>("evarobot_eio/ekb_test");

	/**
	 *  Sonar sensor subscribers are created for sonar0, sonar1, ... and sonar6 topics.
	 */
	ros::Subscriber sub_sonar_0 = n.subscribe("sonar0", 1, sonar0Callback);
	ros::Subscriber sub_sonar_1 = n.subscribe("sonar1", 1, sonar1Callback);
	ros::Subscriber sub_sonar_2 = n.subscribe("sonar2", 1, sonar2Callback);
	ros::Subscriber sub_sonar_3 = n.subscribe("sonar3", 1, sonar3Callback);
	ros::Subscriber sub_sonar_4 = n.subscribe("sonar4", 1, sonar4Callback);
	ros::Subscriber sub_sonar_5 = n.subscribe("sonar5", 1, sonar5Callback);
	ros::Subscriber sub_sonar_6 = n.subscribe("sonar6", 1, sonar6Callback);
	
	/**
	 *  Infrared sensor subscribers are created for ir0, ir1, ... and ir7 topics.
	 */
	ros::Subscriber sub_ir_0 = n.subscribe("ir0", 1, ir0Callback);
	ros::Subscriber sub_ir_1 = n.subscribe("ir1", 1, ir1Callback);
	ros::Subscriber sub_ir_2 = n.subscribe("ir2", 1, ir2Callback);
	ros::Subscriber sub_ir_3 = n.subscribe("ir3", 1, ir3Callback);
	ros::Subscriber sub_ir_4 = n.subscribe("ir4", 1, ir4Callback);
	ros::Subscriber sub_ir_5 = n.subscribe("ir5", 1, ir5Callback);
	ros::Subscriber sub_ir_6 = n.subscribe("ir6", 1, ir6Callback);
	ros::Subscriber sub_ir_7 = n.subscribe("ir7", 1, ir7Callback);
	
	/**
	 * Bumper sensor subscriber is created for bumper topic.
	 */
	ros::Subscriber sub_bumper = n.subscribe("bumper", 1, bumperCallback);
	
	/**
	 * Encoder sensor subscriber is created for wheel_vel topic.
	 */
	ros::Subscriber sub_wheel_vel = n.subscribe("wheel_vel", 1, encoderCallback);
	
	/**
	 * IMU sensor subscriber is created for bumper topic.
	 */
	ros::Subscriber sub_imu = n.subscribe("imu", 1, imuCallback);
	
	/**
	 * Current sensor subscriber is created for bumper topic.
	 */
	ros::Subscriber sub_akim = n.subscribe("motor_voltages", 1, akimCallback);
	
	/**
	 * Battery sensor subscriber is created for bumper topic.
	 */
	ros::Subscriber sub_batarya = n.subscribe("battery", 1, bataryaCallback);

	/**
	 * Publisher topic is created to apply velocities over cmd_vel topic.
	 */
	pub_motor = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	/**
	 * Test number input variable.
	 */
	int i_secenek = 0;
	
	while(true)
	{
		/**
		 * If there is no test active, show menu.
		 */
		if(b_menu)
		{
			/**
			 * Show menu and get selection.
			 */
			b_menu = false;
			MenuyuGoster();
			cin>>i_secenek;
			
			/**
			 * Perform operation due to selection.
			 */
			switch(i_secenek)
			{
				case -1: cout<<"Çikis yapmak için ctrl+c tuslarina basin."<<endl; return 0; break;
				
				case 0: MenuyuGoster(); break;
				
				case 1: SonarTesti(); break;
				
				case 2: KizilotesiTesti(); break;
				
				case 3: BumperTesti(); break;
				
				case 4: MotorVeEnkoderTesti(true); break;

				case 5: MotorVeEnkoderTesti(false); break;

				case 6: IMUTesti(); break;
				
				case 7: BataryaTesti(); break;
				
				case 8: EIOTesti(); break;
				
				default: cout<<"Geçersiz Parametre Girdiniz."<<endl; break;
			}
		}
		/**
		 * If test is continuing, control for timeout.
		 */
		else if(b_time_control)
		{
			/**
			 * Control time difference betweeen current time and last taken message time is in limit (10 seconds).
			 * If timeout exception is occurs, stop the current test. Print the broken sensor information.
			 * If available print information message to prepare next sensor for testing.
			 * Else, show the menu.
			 */
			d_secs_while = ros::Time::now().toSec();
			if(d_secs_while-d_secs_last_message>10)
			{
				if(str_aktif_sensor=="enkoder")
				{
					b_time_control = false;
					str_aktif_sensor = "";
					b_menu = true;
					cout<<"motor/enkoder/akim sensörü arizali."<<endl;
					geometry_msgs::Twist msg;
					msg.angular.z = 0;
					msg.linear.x = 0;
					pub_motor.publish(msg);
				}
				else
				{
					cout<<str_aktif_sensor<<" arizali."<<endl;
					if(str_aktif_sensor=="sonar0")
					{
						
						i_count_gec = i_count;
						cout<<"Kabloyu sonar1 pinlerine takın ve enter tusuna basin.";
						cin.ignore();
						d_secs_last_message = ros::Time::now().toSec();
						str_aktif_sensor = "sonar1";
						b_time_control = true;
					}
					else if(str_aktif_sensor=="sonar1")
					{
						b_time_control = false;
						i_count_gec = i_count;
						cout<<"Kabloyu sonar2 pinlerine takin ve enter tusuna basin.";
						cin.ignore();
						d_secs_last_message = ros::Time::now().toSec();
						str_aktif_sensor = "sonar2";
						b_time_control = true;
					}
					else if(str_aktif_sensor=="sonar2")
					{
						b_time_control = false;
						i_count_gec = i_count;
						cout<<"Kabloyu sonar3 pinlerine takin ve enter tusuna basin.";
						cin.ignore();
						d_secs_last_message = ros::Time::now().toSec();
						str_aktif_sensor = "sonar3";
						b_time_control = true;
					}
					else if(str_aktif_sensor=="sonar3")
					{
						b_time_control = false;
						i_count_gec = i_count;
						cout<<"Kabloyu sonar4 pinlerine takin ve enter tusuna basin.";
						cin.ignore();
						d_secs_last_message = ros::Time::now().toSec();
						str_aktif_sensor = "sonar4";
						b_time_control = true;
					}
					else if(str_aktif_sensor=="sonar4")
					{
						b_time_control = false;
						i_count_gec = i_count;
						cout<<"Kabloyu sonar5 pinlerine takin ve enter tusuna basin.";
						cin.ignore();
						d_secs_last_message = ros::Time::now().toSec();
						str_aktif_sensor = "sonar5";
						b_time_control = true;
					}
					else if(str_aktif_sensor=="sonar5")
					{
						b_time_control = false;
						i_count_gec = i_count;
						cout<<"Kabloyu sonar6 pinlerine takin ve enter tusuna basin.";
						cin.ignore();
						d_secs_last_message = ros::Time::now().toSec();
						str_aktif_sensor = "sonar6";
						b_time_control = true;
					}
					else if(str_aktif_sensor=="sonar6")
					{
						b_time_control = false;
						str_aktif_sensor = "";
						b_menu = true;
					}
					else if(str_aktif_sensor=="ir0")
					{
						b_time_control = false;
						i_count_gec = i_count;
						cout<<"Kabloyu ir1 pinlerine takin ve enter tusuna basin.";
						cin.ignore();
						d_secs_last_message = ros::Time::now().toSec();
						str_aktif_sensor = "ir1";
						b_time_control = true;
					}
					else if(str_aktif_sensor=="ir1")
					{
						b_time_control = false;
						i_count_gec = i_count;
						cout<<"Kabloyu ir2 pinlerine takin ve enter tusuna basin.";
						cin.ignore();
						d_secs_last_message = ros::Time::now().toSec();
						str_aktif_sensor = "ir2";
						b_time_control = true;
					}
					else if(str_aktif_sensor=="ir2")
					{
						b_time_control = false;
						i_count_gec = i_count;
						cout<<"Kabloyu ir3 pinlerine takin ve enter tusuna basin.";
						cin.ignore();
						d_secs_last_message = ros::Time::now().toSec();
						str_aktif_sensor = "ir3";
						b_time_control = true;
					}
					else if(str_aktif_sensor=="ir3")
					{
						b_time_control = false;
						i_count_gec = i_count;
						cout<<"Kabloyu ir4 pinlerine takin ve enter tusuna basin.";
						cin.ignore();
						d_secs_last_message = ros::Time::now().toSec();
						str_aktif_sensor = "ir4";
						b_time_control = true;
					}
					else if(str_aktif_sensor=="ir4")
					{
						b_time_control = false;
						i_count_gec = i_count;
						cout<<"Kabloyu ir5 pinlerine takin ve enter tusuna basin.";
						cin.ignore();
						d_secs_last_message = ros::Time::now().toSec();
						str_aktif_sensor = "ir5";
						b_time_control = true;
					}
					else if(str_aktif_sensor=="ir5")
					{
						b_time_control = false;
						i_count_gec = i_count;
						cout<<"Kabloyu ir6 pinlerine takin ve enter tusuna basin.";
						cin.ignore();
						d_secs_last_message = ros::Time::now().toSec();
						str_aktif_sensor = "ir6";
						b_time_control = true;
					}
					else if(str_aktif_sensor=="ir6")
					{
						b_time_control = false;
						i_count_gec = i_count;
						cout<<"Kabloyu ir7 pinlerine takin ve enter tusuna basin.";
						cin.ignore();
						d_secs_last_message = ros::Time::now().toSec();
						str_aktif_sensor = "ir7";
						b_time_control = true;
					}
					else if(str_aktif_sensor=="ir7")
					{
						b_time_control = false;
						str_aktif_sensor = "";
						b_menu = true;
					}
					else if(str_aktif_sensor=="bumper0")
					{
						b_time_control = false;
						i_count_gec = i_count;
						cout<<"Kabloyu bumper1 pinlerine takin ve enter tusuna basin.";
						cin.ignore();
						d_secs_last_message = ros::Time::now().toSec();
						str_aktif_sensor = "bumper1";
						b_time_control = true;
					}
					else if(str_aktif_sensor=="bumper1")
					{
						b_time_control = false;
						i_count_gec = i_count;
						cout<<"Kabloyu bumper2 pinlerine takin ve enter tusuna basin.";
						cin.ignore();
						d_secs_last_message = ros::Time::now().toSec();
						str_aktif_sensor = "bumper2";
						b_time_control = true;
					}
					else if(str_aktif_sensor=="bumper2")
					{
						b_time_control = false;
						str_aktif_sensor = "";
						b_menu = true;
					}
					else if(str_aktif_sensor=="enkoder")
					{
						b_time_control = false;
						geometry_msgs::Twist msg;
						msg.angular.z = 0;
						msg.linear.x = 0;
						pub_motor.publish(msg);
						str_aktif_sensor = "";
						b_menu = true;
					}
					else if(str_aktif_sensor=="imu")
					{
						b_time_control = false;
						str_aktif_sensor = "";
						b_menu = true;
					}
					else if(str_aktif_sensor=="batarya sensörü")
					{
						b_time_control = false;
						str_aktif_sensor = "";
						b_menu = true;
					}
					
				}
			}
		}
		
		ros::spinOnce();
	}
	
	
	
	return 0;
}
