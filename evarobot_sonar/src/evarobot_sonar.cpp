#include "evarobot_sonar/evarobot_sonar.h"

/*
 * 
 * IMSONAR
 * 
 * 
 * */

IMSONAR::IMSONAR(int fd, 
								 int id, 
								 int pin,
								 boost::shared_ptr<IMDynamicReconfig> _dynamic_params):i_fd(fd), i_id(id), i_pin_no(pin), 
													b_is_alive(false), dynamic_params(_dynamic_params)
{
	struct sonar_ioc_transfer set_params;
	stringstream ss;
	
	// Publisher
	ss << "sonar" << this->i_id;
	this->pub_sonar = this->n.advertise<sensor_msgs::Range>(ss.str().c_str(), 10);
	
	// Diagnostics
	this->updater.setHardwareID("IM-SMO20");
	this->updater.add(ss.str().c_str(), this, &IMSONAR::ProduceDiagnostics);
		
	this->min_freq = this->dynamic_params->d_min_freq; 
	this->max_freq = this->dynamic_params->d_max_freq; 
	this->pub_sonar_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(ss.str().c_str(), this->updater,
											diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10));
											
	
	// Frame id
	ss.str("");
	ss << this->n.resolveName(n.getNamespace(), true) << "/sonar" << this->i_id << "_link";
			 	
	this->sonar_msg.header.frame_id = ss.str();
	this->sonar_msg.radiation_type = 0;
	
	// Set params to driver
	set_params.i_gpio_pin = this->i_pin_no;
	set_params.i_sonar_id = this->i_id;
	ioctl(this->i_fd, IOCTL_SET_PARAM, &set_params);
}

IMSONAR::~IMSONAR()
{
	delete pub_sonar_freq;
}


bool IMSONAR::ReadRange()
{
	int ret_val = 0;
	char read_buf[100];
	stringstream ss;
	
	ss << this->i_id;
	
	int raw_dist = -1;

	//ioctl(this->i_fd, IOCTL_READ_RANGE, &this->data);
	
	double start, stop;
	
	start = ros::Time::now().toSec();
	
	write(this->i_fd, ss.str().c_str(), sizeof(ss.str().c_str()));

	while(raw_dist < 0)
	{
		stop = ros::Time::now().toSec();
		
		if(stop - start > 0.5)
		{
			ROS_INFO("Sonar[%d] is not ALIVE ", this->i_id);
			this->b_is_alive = false;
			return false;
		}
		
		//ROS_INFO("Timeout[%d]: %f ", this->i_id, stop-start);
		ret_val = read(this->i_fd, read_buf, sizeof(read_buf));
		usleep(50000);
		raw_dist = atoi(read_buf);
		
	}
	
	this->b_is_alive = true;
	this->sonar_msg.range = (float)(raw_dist * 0.0001);
	this->sonar_msg.header.stamp = ros::Time::now();
	this->sonar_msg.field_of_view = this->dynamic_params->d_fov;
	this->sonar_msg.min_range = this->dynamic_params->d_min_range;
	this->sonar_msg.max_range = this->dynamic_params->d_max_range;
	
	return true;
}

void IMSONAR::Publish()
{
	if(this->pub_sonar.getNumSubscribers() > 0 || this->dynamic_params->b_always_on)
	{
		this->pub_sonar.publish(this->sonar_msg);
		this->pub_sonar_freq->tick();
	}
}

void IMSONAR::ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	if(this->b_is_alive)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Sonar%d is alive!", this->i_id);
	}
	else
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Sonar%d is NOT alive!", this->i_id);
	}
}


/*
 * 
 * IMDynamicReconfig
 * 
 * 
 * */

void IMDynamicReconfig::CallbackReconfigure(evarobot_sonar::SonarParamsConfig &config, uint32_t level)
{
	ROS_INFO("Updating Sonar Params...");
		
	this->d_fov = config.field_of_view;
	this->d_min_range = config.minRange;
	this->d_max_range = config.maxRange;
	this->b_always_on = config.alwaysOn;
}

IMDynamicReconfig::IMDynamicReconfig():n_private("~")
{
		// Get Params
	n_private.param("alwaysOn", this->b_always_on, false);
	n_private.param("field_of_view", this->d_fov, 0.7);
	n_private.param("minRange", this->d_min_range, 0.0);
	n_private.param("maxRange", this->d_max_range, 5.0);
	n_private.param("maxFreq", this->d_max_freq, 10.0);
	n_private.param("minFreq", this->d_min_freq, 0.2);
		
	ROS_DEBUG("b_always_on: %d", this->b_always_on);
	ROS_DEBUG("d_fov: %f", this->d_fov);
	ROS_DEBUG("d_min_range: %f", this->d_min_range);
	ROS_DEBUG("d_max_range: %f", this->d_max_range);
	
	
	// Dynamic Reconfigure
	this->f = boost::bind(&IMDynamicReconfig::CallbackReconfigure, this, _1, _2);
	this->srv.setCallback(this->f);
}


	

int main(int argc, char **argv)
{
	int i_fd;
  stringstream ss;
  
  vector<IMSONAR *> sonar;
  
  // ROS PARAMS
	vector<int> T_i_sonar_pins;
	double d_frequency;
	string str_driver_path;
	// rosparams end
	
	unsigned int u_i_delay = 100*1000; // Note: Delay in ns
		
	ros::init(argc, argv, "/evarobot_sonar");
	ros::NodeHandle n;
	
	
	n.param<string>("evarobot_sonar/driverPath", str_driver_path, "/dev/evarobotSonar");
	n.param<double>("evarobot_sonar/frequency", d_frequency, 10.0);
	n.getParam("evarobot_sonar/sonarPins", T_i_sonar_pins);
		
	if(T_i_sonar_pins.size() > MAX_SONAR)
	{
		ROS_ERROR("Number of sonar devices mustn't be greater than %d", MAX_SONAR);
	}
	
	
	#ifdef DEBUG
		ROS_INFO("Number of sonars: %d", T_i_sonar_pins.size());
	#endif
	
	/******************************************************************/
	/* Init */
	/******************************************************************/
	
/*	clock_gettime(CLOCK_MONOTONIC, &ts);

	// Lock memory to ensure no swapping is done.
	if(mlockall(MCL_FUTURE|MCL_CURRENT))
	{
		ROS_ERROR("Failed to lock memory");
	}

	// Set our thread to real time priority
	struct sched_param sp;
	
	sp.sched_priority = 30;
	
	if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp))
	{
		ROS_ERROR("Failed to set stepper thread"
				"to real-time priority");
	}
	*/


	i_fd = open(str_driver_path.c_str(), O_RDWR);
	
	if(i_fd < 0)
	{
		printf("file %s either does not exist or has been locked by another process\n", str_driver_path.c_str());
		exit(-1);
	}
		
	/*****************************************************************/
			
	// Define frequency
	ros::Rate loop_rate(d_frequency);
	

	// create objects
	boost::shared_ptr<IMDynamicReconfig> dyn_params(new IMDynamicReconfig);
	
	for(uint i = 0; i < T_i_sonar_pins.size(); i++)
	{
		if(T_i_sonar_pins[i] < MAX_SONAR)
		{
			IMSONAR * dummy_sonar = new IMSONAR(i_fd, T_i_sonar_pins[i], SONAR[T_i_sonar_pins[i]], dyn_params);
			sonar.push_back(dummy_sonar);
		}
		else
		{
			ROS_ERROR("Undefined pin value.");
		}
	}
	
	while(ros::ok())
	{
		
		for(uint i = 0; i < T_i_sonar_pins.size(); i++)
		{
			if(sonar[i]->ReadRange())
			{
				sonar[i]->Publish();
			}	
			
			sonar[i]->updater.update();
			
			usleep(45000);

		}
		
		ros::spinOnce();
		loop_rate.sleep();	
	
	}
	
	close(i_fd);
	return 0;
}
