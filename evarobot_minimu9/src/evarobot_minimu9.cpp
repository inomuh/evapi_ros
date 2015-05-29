#include "ros/ros.h"

#include "minimu9/vector.h"
#include "minimu9/MinIMU9.h"
#include "minimu9/version.h"

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <sys/time.h>
#include <system_error>
#include <boost/program_options.hpp>

#include "sensor_msgs/Imu.h"

#include <sys/types.h>
#include <sys/ipc.h>
#include <semaphore.h>
#include <sys/stat.h>
#include <fcntl.h>

//#include <string.h>


char SEM_NAME[]= "i2c";

namespace opts = boost::program_options;

// TODO: print warning if accelerometer magnitude is not close to 1 when starting up

// An Euler angle could take 8 chars: -234.678, but usually we only need 6.
float field_width = 6;
#define FLOAT_FORMAT std::fixed << std::setprecision(3) << std::setw(field_width)

std::ostream & operator << (std::ostream & os, const vector & vector)
{
    return os << FLOAT_FORMAT << vector(0) << ' '
              << FLOAT_FORMAT << vector(1) << ' '
              << FLOAT_FORMAT << vector(2);
}

std::ostream & operator << (std::ostream & os, const matrix & matrix)
{
    return os << (vector)matrix.row(0) << ' '
              << (vector)matrix.row(1) << ' '
              << (vector)matrix.row(2);
}

std::ostream & operator << (std::ostream & os, const quaternion & quat)
{
    return os << FLOAT_FORMAT << quat.w() << ' '
              << FLOAT_FORMAT << quat.x() << ' '
              << FLOAT_FORMAT << quat.y() << ' '
              << FLOAT_FORMAT << quat.z();
}

typedef void rotation_output_function(quaternion& rotation);

void output_quaternion(quaternion & rotation)
{
    std::cout << rotation;
}

void output_matrix(quaternion & rotation)
{
    std::cout << rotation.toRotationMatrix();
}

void output_euler(quaternion & rotation)
{
    std::cout << (vector)(rotation.toRotationMatrix().eulerAngles(2, 1, 0) * (180 / M_PI));
}

int millis()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

void streamRawValues(IMU& imu)
{
    imu.enable();
    while(1)
    {
        imu.read();
        printf("%7d %7d %7d  %7d %7d %7d  %7d %7d %7d\n",
               imu.raw_m[0], imu.raw_m[1], imu.raw_m[2],
               imu.raw_a[0], imu.raw_a[1], imu.raw_a[2],
               imu.raw_g[0], imu.raw_g[1], imu.raw_g[2]
        );
        usleep(20*1000);
    }
}

//! Uses the acceleration and magnetic field readings from the compass
// to get a noisy estimate of the current rotation matrix.
// This function is where we define the coordinate system we are using
// for the ground coords:  North, East, Down.
matrix rotationFromCompass(const vector& acceleration, const vector& magnetic_field)
{
    vector down = -acceleration;     // usually true
    vector east = down.cross(magnetic_field); // actually it's magnetic east
    vector north = east.cross(down);

    east.normalize();
    north.normalize();
    down.normalize();

    matrix r;
    r.row(0) = north;
    r.row(1) = east;
    r.row(2) = down;
    return r;
}

typedef void fuse_function(quaternion& rotation, float dt, const vector& angular_velocity,
                  const vector& acceleration, const vector& magnetic_field);

void fuse_compass_only(quaternion& rotation, float dt, const vector& angular_velocity,
  const vector& acceleration, const vector& magnetic_field)
{
    // Implicit conversion of rotation matrix to quaternion.
    rotation = rotationFromCompass(acceleration, magnetic_field);
}

// Uses the given angular velocity and time interval to calculate
// a rotation and applies that rotation to the given quaternion.
// w is angular velocity in radians per second.
// dt is the time.
void rotate(quaternion& rotation, const vector& w, float dt)
{
    // Multiply by first order approximation of the
    // quaternion representing this rotation.
    rotation *= quaternion(1, w(0)*dt/2, w(1)*dt/2, w(2)*dt/2);
    rotation.normalize();
}

void fuse_gyro_only(quaternion& rotation, float dt, const vector& angular_velocity,
  const vector& acceleration, const vector& magnetic_field)
{
    rotate(rotation, angular_velocity, dt);
}

void fuse_default(quaternion& rotation, float dt, const vector& angular_velocity,
  const vector& acceleration, const vector& magnetic_field)
{
    vector correction = vector(0, 0, 0);

    if (abs(acceleration.norm() - 1) <= 0.3)
    {
        // The magnetidude of acceleration is close to 1 g, so
        // it might be pointing up and we can do drift correction.

        const float correction_strength = 1;

        matrix rotationCompass = rotationFromCompass(acceleration, magnetic_field);
        matrix rotationMatrix = rotation.toRotationMatrix();

        correction = (
            rotationCompass.row(0).cross(rotationMatrix.row(0)) +
            rotationCompass.row(1).cross(rotationMatrix.row(1)) +
            rotationCompass.row(2).cross(rotationMatrix.row(2))
          ) * correction_strength;

    }

    rotate(rotation, angular_velocity + correction, dt);
}

void ahrs(IMU & imu, fuse_function * fuse, rotation_output_function * output)
{
    imu.loadCalibration();
    imu.enable();
    imu.measureOffsets();
    
    // The quaternion that can convert a vector in body coordinates
    // to ground coordinates when it its changed to a matrix.
    quaternion rotation = quaternion::Identity();

    int start = millis(); // truncate 64-bit return value
    while(1)
    {
        int last_start = start;
        start = millis();
        float dt = (start-last_start)/1000.0;
        if (dt < 0){ throw std::runtime_error("Time went backwards."); }

        vector angular_velocity = imu.readGyro();
        vector acceleration = imu.readAcc();
        vector magnetic_field = imu.readMag();

        fuse(rotation, dt, angular_velocity, acceleration, magnetic_field);

        output(rotation);
        std::cout << "  " << acceleration << "  " << magnetic_field << std::endl << std::flush;

        // Ensure that each iteration of the loop takes at least 20 ms.
        while(millis() - start < 20)
        {
            usleep(1000);
        }
    }
}

int main(int argc, char *argv[])
{
	// Semaphore
	key_t key;
	sem_t *mutex;
	FILE * fd;
	
	
	// ROS PARAMS
	double d_frequency;
	std::string str_topic, str_frame_id;
	bool b_always_on;
	
	//name the shared memory segment
	key = 1005;
//	printf("create & initialize semaphore\n");
	
	
	//create & initialize semaphore
	mutex = sem_open(SEM_NAME,O_CREAT,0644,1);
	if(mutex == SEM_FAILED)
	{
		perror("unable to create semaphore");
		sem_unlink(SEM_NAME);
		
		return(-1);
	}
	
	// Define what all the command-line parameters are.
    std::string mode, output_mode, i2cDevice;
        
	
	// ROS PARAMS END
	
	ros::init(argc, argv, "evarobot_minimu9");
	ros::NodeHandle n;
	
	/*if(!n.getParam("evarobot_sonar/frequency", d_frequency))
	{
		ROS_INFO("Failed to get param 'frequency'");
		d_frequency = 10.0;
	}
	
	ros::Rate loop_rate(d_frequency);*/
	
	
	
	sensor_msgs::Imu msg;
	
	n.param<std::string>("evarobot_minimu9/i2cDevice", i2cDevice, "/dev/i2c-1");
	n.param<std::string>("evarobot_minimu9/topicName", str_topic, "imu_data");
	n.param<std::string>("evarobot_minimu9/frame_id", str_frame_id, "imu");
	
	n.param("evarobot_minimu9/alwaysOn", b_always_on, false);

	
	
	if(!n.getParam("evarobot_minimu9/frequency", d_frequency))
	{
	  ROS_ERROR("Failed to get param 'frequency'");
	} 	
	
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>(str_topic.c_str(), 10);
	
	
	output_mode = "matrix";
	mode = "normal";
    try
    {


        MinIMU9 imu(i2cDevice.c_str());


      // void ahrs(IMU & imu, fuse_function * fuse, rotation_output_function * output)
        imu.loadCalibration();
		imu.enable();
		imu.measureOffsets();

		// The quaternion that can convert a vector in body coordinates
		// to ground coordinates when it its changed to a matrix.
		quaternion rotation = quaternion::Identity();

		int start = millis(); // truncate 64-bit return value
		while(ros::ok())
		{
			int last_start = start;
			start = millis();
			float dt = (start-last_start)/1000.0;
			if (dt < 0){ throw std::runtime_error("Time went backwards."); }

			vector angular_velocity = imu.readGyro();
			vector acceleration = imu.readAcc();
			vector magnetic_field = imu.readMag();

			sem_wait(mutex);
			fuse_default(rotation, dt, angular_velocity, acceleration, magnetic_field);
			sem_post(mutex);
			
			msg.header.frame_id = str_frame_id;
			msg.header.stamp = ros::Time::now();
						
			msg.orientation.x = rotation.x();
			msg.orientation.y = rotation.y();
			msg.orientation.z = rotation.z();
			msg.orientation.w = rotation.w();
			
			if(imu_pub.getNumSubscribers() > 0 || b_always_on)
			{
				imu_pub.publish(msg);
			}

			ros::spinOnce();


			// Ensure that each iteration of the loop takes at least 20 ms.
			while(millis() - start < 20)
			{
				usleep(1000);
			}
		}
        
        //	sem_close(mutex);
		sem_unlink(SEM_NAME);
        
        return 0;
    }
    catch(const std::system_error & error)
    {
        std::string what = error.what();
        const std::error_code & code = error.code();
        std::cerr << "Error: " << what << "  " << code.message() << " (" << code << ")" << std::endl;
        return 2;
    }
    catch(const opts::multiple_occurrences & error)
    {
        std::cerr << "Error: " << error.what() << " of " << error.get_option_name() << " option." << std::endl;
        return 1;
    }
    catch(const std::exception & error)    
    {
        std::cerr << "Error: " << error.what() << std::endl;
        return 9;
    }
}

