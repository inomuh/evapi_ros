#include "evarobot_teleop/ps3.h"

/**
 * Constructor. Sets default values.
 * Maximum linear velocity of robot is set as 1.0
 * Maximum angular velocity of robot is set as 3.0
 * Deadman button is set as unpressed.
 * Maximum linear velocity that can be applied by PS3 controller.
 * Maximum angular velocity that can be applied by PS3 controller.
 */
IMJoystick::IMJoystick()
{
	this->max_lin = 1.0;
	this->max_ang = 3.0;
	this->deadman = false;
	
	this->max_x = 0.2;
	this->max_z = 0.5;
}

/**
 * If an error occurs publishes it. Else publishes OK message.
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
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No collision!");
    }
}

/**
 * Initializes default values for buttons on PS3 controller. Creates publisher, subscriber and service client.
 */
void IMJoystick::init()
{
	this->deadman_button = 10;
	this->axr_upwards = 3;
	this->axl_leftwards = 0;

	this->triangle_button = 12;
	this->cross_button = 14;
	this->square_button = 15;
	this->circle_button = 13;
	
	this->vel_msg.linear.x = 0.0;
	this->vel_msg.angular.z = 0.0;
	
	/**
	 * Service client is created for evarobot_rgb/SetRGB service in type of im_msgs::SetRGB.
	 */
	this->client = this->n.serviceClient<im_msgs::SetRGB>("evarobot_rgb/SetRGB");
		
	/**
	 * cmd_vel publisher is created in type of geometry_msgs::Twist.
	 */
	this->vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	/**
	 * joy publisher is created in type of sensor_msgs::Joy.
	 */
	this->joy_sub_ = this->n.subscribe("joy", 10, &IMJoystick::CallbackJoy, this);
	
	this->n.param("evarobot_ps3/timeout", this->d_timeout, 1.0);
	ROS_DEBUG("EvarobotTeleop: timeout: %f", this->d_timeout);
		
	this->last_time = ros::Time::now();	
}

/**
 * Callback function for joy topic.
 */
void IMJoystick::CallbackJoy(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
	/**
	 * Determines deadman button is pressed or not.
	 */
	deadman = (((unsigned int)deadman_button < joy_msg->buttons.size()) && joy_msg->buttons[deadman_button]);

	this->last_time = ros::Time::now();	

	ROS_DEBUG("EvarobotTeleop: DEADMAN %d", joy_msg->buttons[deadman_button]);
	
	/**
	 * If deadman button is not pressed, no velocity applied.
	 */
	if (!deadman)
	{
		ROS_DEBUG("EvarobotTeleop: deadman is false");
		this->vel_msg.linear.x = 0.0;
		this->vel_msg.angular.z = 0.0;
		return;
	}

	/**
	 * If triangle_button is pressed, increase maximum linear velocity by 0.1.
	 */
	if(((unsigned int)triangle_button < joy_msg->buttons.size()) && joy_msg->buttons[triangle_button])
		this->max_x += 0.1;
	
	/**
	 * If cross_button is pressed, decrease maximum linear velocity by 0.1.
	 */
	if(((unsigned int)cross_button < joy_msg->buttons.size()) && joy_msg->buttons[cross_button])
					this->max_x -= 0.1;

	/**
	 * If max linear velocity drop to below zero, it is set as 0.1.
	 */
	if(this->max_x < 0.0)
		max_x = 0.1;

	/**
	 * If max linear velocity exceed max_lin, it is set as max_lin.
	 */
	if(this->max_x > this->max_lin)
		max_x = this->max_lin;

	/**
	 * If square_button is pressed, increase maximum angular velocity by 0.1.
	 */
	if(((unsigned int)square_button < joy_msg->buttons.size()) && joy_msg->buttons[square_button])
		this->max_z += 0.1;

	/**
	 * If square_button is pressed, decrease maximum angular velocity by 0.1.
	 */
	if(((unsigned int)circle_button < joy_msg->buttons.size()) && joy_msg->buttons[circle_button])
		this->max_z -= 0.1;

	/**
	 * If max angular velocity drop to below zero, it is set as 0.1.
	 */
	if(this->max_z < 0.0)
		max_z = 0.1;

	/**
	 * If max angular velocity exceed max_ang, it is set as max_ang.
	 */
	if(this->max_z > this->max_ang)
		max_z = this->max_ang;

	ROS_DEBUG("EvarobotTeleop: linear %f <-- %d", joy_msg->axes[axr_upwards], axr_upwards);
	ROS_DEBUG("EvarobotTeleop: angular %f <-- %d", joy_msg->axes[axl_leftwards], axl_leftwards);

	/**
	 * ROS message content is set.
	 */
	this->vel_msg.linear.x = this->max_x * joy_msg->axes[axr_upwards];
	this->vel_msg.angular.z = this->max_z * joy_msg->axes[axl_leftwards];
}

/**
 * Publishes velocity to cmd_vel topic.
 */
void IMJoystick::PublishVel()
{
	this->CheckTimeout();
	this->vel_pub.publish(this->vel_msg);
}

/**
 * Turns on leds TELEOP mode (Flipping blue).
 */
void IMJoystick::CallRGBService()
{
	im_msgs::SetRGB srv;

	srv.request.times = -1;
	srv.request.mode = 3;
	srv.request.frequency = -1;
	srv.request.color = 0;

	if(this->client.call(srv) == 0)
	{
		ROS_INFO(GetErrorDescription(-123).c_str());
        i_error_code = -123;
	}
}

/**
 * Control timeout exception.
 */
void IMJoystick::CheckTimeout()
{
	ros::Duration duration = ros::Time::now() - this->last_time;
	if(duration.toSec() > d_timeout)
	{
		this->vel_msg.linear.x = 0.0;
		this->vel_msg.angular.z = 0.0;
		ROS_INFO(GetErrorDescription(-123).c_str());
        i_error_code = -124;
	}
}

/**
 * Turns off rgb leds.
 */
void IMJoystick::TurnOffRGB()
{
	im_msgs::SetRGB srv;

	srv.request.times = -1;
	srv.request.mode = 0;
	srv.request.frequency = 1;
	srv.request.color = 0;
	if(this->client.call(srv) == 0)
	{
		ROS_INFO(GetErrorDescription(-123).c_str());
        i_error_code = -123;
	}
}

/**
 * Callback for terminate key combination (ctrl+c).
 */
void sighandler(int sig)
{
	g_b_terminate = true;
}

/**
 * Program starts here.
 */
int main(int argc, char **argv)
{
	/**
	 * Initializes ROS node with evarobot_ps3 name.
	 */
	ros::init(argc, argv, "evarobot_ps3");
	
	/**
	 * Creates and initializes PS3 controller.
	 */
	IMJoystick ps3;
	ps3.init();
	ps3.CallRGBService();
	
	/**
	 * Define frequency
	 */
	ros::Rate loop_rate(10.0);
	
	/**
	 * Terminate key combination is set.
	 */
	signal(SIGINT, &sighandler);

	/**
	 * Set diagnostics to handle and publish error.
	 */
    double d_min_freq = 0.2;
	double d_max_freq = 10.0;
	diagnostic_updater::Updater updater;
	updater.setHardwareID("EvarobotEIO");
	updater.add("eio", &ProduceDiagnostics);
	diagnostic_updater::HeaderlessTopicDiagnostic pub_freq("eio", updater,
            diagnostic_updater::FrequencyStatusParam(&d_min_freq, &d_max_freq, 0.1, 10));
            
	while (!g_b_terminate)
	{
		/**
		 * cmd_vel topic is published.
		 */
		ps3.PublishVel();

		/**
		 * Loop is slept to hold frequency.
		 */
		updater.update();
		ros::spinOnce();
		loop_rate.sleep();
	}

	/**
	 * Leds are turned off.
	 */
	ps3.TurnOffRGB();
	return 0;
}
