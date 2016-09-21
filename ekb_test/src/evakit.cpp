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
 * Service client to start sonar sensors.
 */
ros::ServiceClient client_sonar;

/**
 * Service client to start infrared sensors.
 */
ros::ServiceClient client_ir;

/**
 * Service client to start bumper sensor.
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
 */
void bumperCallback(const im_msgs::Bumper::ConstPtr& msg)
{
	/**
	 * If current sensor is bumper these block is run.
	 */
	if(str_aktif_sensor=="bumper")
	{
		/**
		 * Test number and collicision data from bumper sensor is printed on the screen. 
		 * Remaining number of tests decreased by 1.
		 */
		if(i_count_gec>0)
		{
			if(msg->state[0].bumper_state)
			{
				cout<<"Bumper -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<"true"<<endl;
			}
			else
			{
				cout<<"Bumper -> "<<"Test " << (i_count-i_count_gec)<<" -> " <<"false"<<endl;
			}
			d_secs_last_message = ros::Time::now().toSec();
			i_count_gec--;
		}
		/**
		 * Test is done for bumper.
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
				std::cout<< msg->right_vel <<std::endl;
				std::ostringstream stream;
				stream << "Enkoder Sensörü -> "<<"Test " << (i_count-i_count_gec)<<" -> \t" <<"Sol Teker : "<< fixed << setprecision(4) << msg->left_vel<<"\tSag teker : "<< fixed << setprecision(4)<< msg->right_vel<<endl;
				str_enkoder.push_back(stream.str());
				d_secs_last_message = ros::Time::now().toSec();
				i_count_gec--;
			}
		}
		/**
		 * Test is done for encoder.
		 * b_menu is set as true to show test selection menu.
		 */
		else
		{
			for(int t=0;t<str_enkoder.size();t++)
			{
				std::cout<<str_enkoder.at(t);
			}
			/*
			str_enkoder.clear();
			geometry_msgs::Twist msg;
			msg.angular.z = 0;
			msg.linear.x = 0;
			pub_motor.publish(msg);
			*/
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
	cout<<"Kabloyu bumper pinlerine takin ve enter tusuna basin.";
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(), '\n');
	cin.ignore();
	
	/**
	 * Current sensor is set as bumper.
	 */
	d_secs_last_message = ros::Time::now().toSec();
	b_time_control = true;
	str_aktif_sensor = "bumper";
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
	
	/**
	 * Get number of tests for encoder sensor.
	 */
	cout<<"Motor ve enkoder için gerçeklestirilecek test sayisini [50 - 100] girin."<<endl;
	cin>>i_count;
	i_count_gec = i_count;
	
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
 * This function prints available tests on the screen.
 */
void MenuyuGoster()
{
	cout<<"************EKB TEST YAZILIMI***********"<<endl;
	cout<<"1. Sonar Testi"<<endl;
	cout<<"2. Kizilötesi Testi"<<endl;
	cout<<"3. Bumper Testi"<<endl;
	cout<<"4. Ileri Motor ve Enkoder Sensörü Testi"<<endl;
	cout<<"5. Geri Motor ve Enkoder Sensörü Testi"<<endl;
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

	/**
	 * Sonar sensor subscribers are created for sonar0 and sonar1 topics.
	 */
	ros::Subscriber sub_sonar_0 = n.subscribe("sonar0", 1, sonar0Callback);
	ros::Subscriber sub_sonar_1 = n.subscribe("sonar1", 1, sonar1Callback);
	
	/**
	 * Infrared sensor subscribers are created for ir0 and ir1 topics.
	 */
	ros::Subscriber sub_ir_0 = n.subscribe("ir0", 1, ir0Callback);
	ros::Subscriber sub_ir_1 = n.subscribe("ir1", 1, ir1Callback);
	
	/**
	 * Bumper sensor subscriber is created for bumper topic.
	 */
	ros::Subscriber sub_bumper = n.subscribe("bumper", 1, bumperCallback);
	
	/**
	 * Encoder sensor subscriber is created for wheel_vel topic.
	 */
	ros::Subscriber sub_wheel_vel = n.subscribe("wheel_vel", 1, encoderCallback);
	
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
					cout<<"motor/enkoder sensörü arizali."<<endl;
					//geometry_msgs::Twist msg;
					//msg.angular.z = 0;
					//msg.linear.x = 0;
					//pub_motor.publish(msg);
				}
				else
				{
					cout<<str_aktif_sensor<<" arizali."<<endl;
					if(str_aktif_sensor=="sonar0")
					{
						
						i_count_gec = i_count;
						cout<<"Kabloyu sonar1 pinlerine takin ve enter tusuna basin.";
						cin.ignore();
						d_secs_last_message = ros::Time::now().toSec();
						str_aktif_sensor = "sonar1";
						b_time_control = true;
					}
					else if(str_aktif_sensor=="sonar1")
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
						str_aktif_sensor = "";
						b_menu = true;
					}
					else if(str_aktif_sensor=="bumper")
					{
						b_time_control = false;
						str_aktif_sensor = "";
						b_menu = true;
					}
					else if(str_aktif_sensor=="enkoder")
					{
						b_time_control = false;
						//geometry_msgs::Twist msg;
						//msg.angular.z = 0;
						//msg.linear.x = 0;
						//pub_motor.publish(msg);
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
