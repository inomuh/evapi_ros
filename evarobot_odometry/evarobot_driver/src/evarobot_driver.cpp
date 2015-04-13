#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

//#include "evarobot_driver/rpiPWM.h"

#include "IMPWM.h"

#include <stdio.h> 
#include <math.h> 

IMPWM pwm(1000.0, 256, 80.0, IMPWM::MSMODE);

using namespace std;

void CallbackVel(const geometry_msgs::Twist::ConstPtr& msg)
{
 
  float f_dogrusal_hiz = msg->linear.x;
  float f_acisal_hiz = msg->angular.z;
 
 cout << "D: " << f_dogrusal_hiz << " A: " << f_acisal_hiz << endl;
 

  
  float f_sol_teker_hiz = f_dogrusal_hiz - f_acisal_hiz * 0.34 / 2;
  float f_sag_teker_hiz = f_dogrusal_hiz + f_acisal_hiz * 0.34 / 2;

cout << "SOL: " << f_sol_teker_hiz << " SAG: " << f_sag_teker_hiz << endl;
  
    
  pwm.SetDirection(0, f_sol_teker_hiz<0 ? 0:1);
  pwm.SetDirection(1, f_sag_teker_hiz<0 ? 0:1);

  int i_sol_teker_duty = int(fabs(f_sol_teker_hiz) * 214);
  int i_sag_teker_duty = int(fabs(f_sag_teker_hiz) * 214); 
 
  i_sol_teker_duty = i_sol_teker_duty>=256 ? 255:i_sol_teker_duty;
  i_sag_teker_duty = i_sag_teker_duty>=256 ? 255:i_sag_teker_duty;


  pwm.SetDutyCycleCount(i_sol_teker_duty, 0);
  pwm.SetDutyCycleCount(i_sag_teker_duty, 1); 
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "evarobot_driver");
  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("cmd_vel", 10, CallbackVel);

  ros::spin();

  return 0;
}
