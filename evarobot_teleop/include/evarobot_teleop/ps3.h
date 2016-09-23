#ifdef English_dox
/**
 * \file   ps3.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Controls evarobot using PS3 controller.
 * \details
 */
#endif

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "im_msgs/SetRGB.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include "IMEIO.h"
#include "ErrorCodes.h"
#include <signal.h>

using namespace std;

#ifdef English_dox
//! If an error occurs, shows its code number.
#endif
int i_error_code = 0;

#ifdef English_dox
//! Terminate key combination (ctrl+c) is pressed or not.
#endif
bool g_b_terminate = false;

#ifdef English_dox
//! Takes commands from PS controller and publishes cmd_vel data.
#endif
class IMJoystick
{
	public:
		#ifdef English_dox
		//! Creates ROS node handler.
		#endif
		ros::NodeHandle n;

		#ifdef English_dox
		//! ROS message in type of geometry_msgs::Twist.
		#endif
		geometry_msgs::Twist cmd;

		#ifdef English_dox
		//! Maximum linear velocity.
		#endif
		double max_lin;

		#ifdef English_dox
		//! Maximum angular velocity.
		#endif
		double max_ang;

		#ifdef English_dox
		//! Maximum linear velocity that can be applied using PS3 controller.
		#endif
		double max_x;

		#ifdef English_dox
		//! Maximum angular velocity that can be applied using PS3 controller.
		#endif
		double max_z;

		#ifdef English_dox
		//! This button should be pressed to apply velocities, else no velocity applied.
		#endif
		bool deadman;

		#ifdef English_dox
		//! If data is not taken for a long time, connection dropped and no velocity applied.
		#endif
		double d_timeout;

		#ifdef English_dox
		//! cmd_vel topic publisher in type of geometry_msgs::Twist.
		#endif
		ros::Publisher vel_pub;

		#ifdef English_dox
		//! joy topic subscriber in type of sensor_msgs::Joy.
		#endif
		ros::Subscriber joy_sub_;

		#ifdef English_dox
		//! evarobot_rgb/SetRGB service client in type of im_msgs::SetRGB.
		#endif
		ros::ServiceClient client;
	
		#ifdef English_dox
		//! Constructor. Sets default values.
		#endif
		IMJoystick();

		#ifdef English_dox
		//! Destructor
		#endif
		~IMJoystick(){}

		#ifdef English_dox
		//! Initializes default values for buttons on PS3 controller. Creates publisher, subscriber and service client.
		#endif
		void init();

		#ifdef English_dox
		//! Callback function for joy topic.
		/**
		 * \param &joy_msg Message from PS3 controller.
		 */
		#endif
		void CallbackJoy(const sensor_msgs::Joy::ConstPtr& joy_msg);

		#ifdef English_dox
		//! Publishes velocity to cmd_vel topic.
		#endif
		void PublishVel();

		#ifdef English_dox
		//! Turns on leds TELEOP mode (Flipping blue).
		#endif
		void CallRGBService();

		#ifdef English_dox
		//! Turns off rgb leds.
		#endif
		void TurnOffRGB();
			
	private:
		#ifdef English_dox
		//! Id of the button. This button should be pressed to apply velocities, else no velocity applied.
		#endif
		int deadman_button;

		#ifdef English_dox
		//! Id of right analog control button to apply linear velocity.
		#endif
		int axr_upwards;

		#ifdef English_dox
		//! Id of left analog control to apply angular velocity.
		#endif
		int axl_leftwards;

		#ifdef English_dox
		//! Id of button to increase max_x value by 0.1.
		#endif
		int triangle_button;

		#ifdef English_dox
		//! Id of button to decrease max_x value by 0.1.
		#endif
		int cross_button;

		#ifdef English_dox
		//! Id of button to increase max_z value by 0.1.
		#endif
		int square_button;

		#ifdef English_dox
		//! Id of button to decrease max_z value by 0.1.
		#endif
		int circle_button;
	
		#ifdef English_dox
		//! Last time CallbackJoy callback function is called.
		#endif
		ros::Time last_time;

		#ifdef English_dox
		//! ROS message to send from cmd_vel topic.
		#endif
		geometry_msgs::Twist vel_msg;
	
		#ifdef English_dox
		//! Control timeout exception.
		#endif
		void CheckTimeout();
};
