#include "evarobot_minimu9/evarobot_minimu9.h"

#include <dynamic_reconfigure/server.h>
#include <evarobot_minimu9/ParamsConfig.h>

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
  //      std::cout << "  " << acceleration << "  " << magnetic_field << std::endl << std::flush;

        // Ensure that each iteration of the loop takes at least 20 ms.
        while(millis() - start < 20)
        {
            usleep(1000);
        }
    }
}

bool b_is_received_params = false;

void CallbackReconfigure(evarobot_minimu9::ParamsConfig &config, uint32_t level)
{
   b_is_received_params = true;        
}


int main(int argc, char *argv[])
{
	// Semaphore
	key_t key;
	sem_t *mutex;
	FILE * fd;
	
  std::stringstream ss;
	
	// ROS PARAMS
	double d_frequency;
	double d_min_freq, d_max_freq;
	// ROS PARAMS END
	
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
  
	ros::init(argc, argv, "evarobot_minimu9");
	ros::NodeHandle n;
	
	/*if(!n.getParam("evarobot_sonar/frequency", d_frequency))
	{
		ROS_INFO("Failed to get param 'frequency'");
		d_frequency = 10.0;
	}
	
	ros::Rate loop_rate(d_frequency);*/
		
	n.param<std::string>("evarobot_minimu9/i2cDevice", i2cDevice, "/dev/i2c-1");
	n.param("evarobot_odometry/minFreq", d_min_freq, 0.2);
	n.param("evarobot_odometry/maxFreq", d_max_freq, 10.0);
	
	if(!n.getParam("evarobot_minimu9/frequency", d_frequency))
	{
	  ROS_ERROR("Failed to get param 'frequency'");
	} 	
	
	realtime_tools::RealtimePublisher<sensor_msgs::Imu> * imu_pub = new realtime_tools::RealtimePublisher<sensor_msgs::Imu>(n, "imu", 10);
	
	// Dynamic Reconfigure
	dynamic_reconfigure::Server<evarobot_minimu9::ParamsConfig> srv;
	dynamic_reconfigure::Server<evarobot_minimu9::ParamsConfig>::CallbackType f;
	f = boost::bind(&CallbackReconfigure, _1, _2);
	srv.setCallback(f);
	///////////////
	
	// Diagnostics
	diagnostic_updater::Updater updater;
	updater.setHardwareID("None");
		
	diagnostic_updater::HeaderlessTopicDiagnostic pub_freq("imu", updater,
											diagnostic_updater::FrequencyStatusParam(&d_min_freq, &d_max_freq, 0.1, 10));
	
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
			
			if(b_is_received_params)
			{
				ROS_INFO("Updating IMU Params...");
			
				b_is_received_params = false;
			}	
			
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

			double roll, pitch, yaw;
			double qx, qy, qz, qw;
			vector euler_angles;
			
			euler_angles = (vector)(rotation.toRotationMatrix().eulerAngles(2, 1, 0));
			
			roll = euler_angles(2);
			pitch = -euler_angles(1);
			yaw = -euler_angles(0);
			
			qx = sin(roll*0.5) * cos(pitch*0.5) * cos(yaw*0.5) - cos(roll*0.5) * sin(pitch*0.5) * sin(yaw*0.5);
			qy = cos(roll*0.5) * sin(pitch*0.5) * cos(yaw*0.5) + sin(roll*0.5) * cos(pitch*0.5) * sin(yaw*0.5);
			qz = cos(roll*0.5) * cos(pitch*0.5) * sin(yaw*0.5) - sin(roll*0.5) * sin(pitch*0.5) * cos(yaw*0.5);
			qw = cos(roll*0.5) * cos(pitch*0.5) * cos(yaw*0.5) + sin(roll*0.5) * sin(pitch*0.5) * sin(yaw*0.5);
		
			
//			std::cout << "Roll: " << -roll * (180 / M_PI);
//			std::cout << "  Pitch: " << pitch * (180 / M_PI) << std::endl;


//			output_euler(rotation);
//			std::cout << std::endl;
			
			ss.str("");
			ss << n.resolveName(n.getNamespace(), true) << "/imu_link";
			imu_pub->msg_.header.frame_id = ss.str();
			imu_pub->msg_.header.stamp = ros::Time::now();
						
			imu_pub->msg_.orientation.x = qx; //rotation.x();
			imu_pub->msg_.orientation.y = qy; //rotation.y();
			imu_pub->msg_.orientation.z = qz; //rotation.z();
			imu_pub->msg_.orientation.w = qw; //rotation.w();

			if (imu_pub->trylock())
			{
				imu_pub->unlockAndPublish();
			}			
			pub_freq.tick();
			
			updater.update();
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

