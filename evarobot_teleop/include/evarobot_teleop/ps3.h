#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "im_msgs/SetRGB.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include "IMEIO.h"
#include "ErrorCodes.h"

using namespace std;

class IMJoystick
{
	public:
	ros::NodeHandle n;
	geometry_msgs::Twist cmd;
	
	double max_lin, max_ang;
	double max_x, max_z;
	bool deadman;
	
	double d_timeout;
	
	ros::Publisher vel_pub;
	ros::Subscriber joy_sub_;
	ros::ServiceClient client;

	IMJoystick();
	~IMJoystick(){}
	void init();
	void CallbackJoy(const sensor_msgs::Joy::ConstPtr& joy_msg);
	void PublishVel();
	void CallRGBService();
	void TurnOffRGB();
			
	private:
	int deadman_button;
	int axr_leftwards;
	int axr_upwards;
	
	int axl_leftwards;
	int axl_upwards;

  int triangle_button, cross_button, square_button, circle_button;

	ros::Time last_time;
	geometry_msgs::Twist vel_msg;
	
	void CheckTimeout();
};
