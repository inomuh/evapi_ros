#include "evarobot_sonar/evarobot_sonar.h"

/**
 * IMSONAR constructor with custom parameters.
 */
IMSONAR::IMSONAR(int fd, 
								 int id, 
								 int pin,
								 boost::shared_ptr<IMDynamicReconfig> _dynamic_params):i_fd(fd), i_id(id), i_pin_no(pin), 
													b_is_alive(false), dynamic_params(_dynamic_params)
{
	struct sonar_ioc_transfer set_params;
	stringstream ss;
	
	/**
	 * Publisher is created with topic name sonar<id> and message type sensor_msgs::Range
	 */
	ss << "sonar" << this->i_id;
	this->pub_sonar = this->n.advertise<sensor_msgs::Range>(ss.str().c_str(), 10);
	
	/**
	 * Set diagnostics to handle and publish error.
	 */
	this->updater.setHardwareID("IM-SMO20");
	this->updater.add(ss.str().c_str(), this, &IMSONAR::ProduceDiagnostics);
	this->min_freq = this->dynamic_params->d_min_freq; 
	this->max_freq = this->dynamic_params->d_max_freq; 
	this->pub_sonar_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(ss.str().c_str(), this->updater,
											diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10));
											
	
	/**
	 * Constants in ROS message are set.
	 */
	ss.str("");
	ss << this->n.resolveName(n.getNamespace(), true) << "/sonar" << this->i_id << "_link";
			 	
	this->sonar_msg.header.frame_id = ss.str();
	this->sonar_msg.radiation_type = 0;
	
	/**
	 * Set params to driver
	 */
	set_params.i_gpio_pin = this->i_pin_no;
	set_params.i_sonar_id = this->i_id;
	ioctl(this->i_fd, IOCTL_SET_PARAM, &set_params);
}

/**
 * Destructor
 */
IMSONAR::~IMSONAR()
{
	delete pub_sonar_freq;
}

/**
 * Checks sonar is alive or not.
 */
bool IMSONAR::Check()
{
	int ret_val = 0;
	char read_buf[100];
	stringstream ss;

	ss << (this->i_id + 100);

	int raw_dist = -1;

	double start, stop;


	write(this->i_fd, ss.str().c_str(), sizeof(ss.str().c_str()));
	usleep(40000);

	start = ros::Time::now().toSec();

	while(raw_dist < 0)
	{
		stop = ros::Time::now().toSec();

		if(stop - start > 0.02)
		{
			ROS_DEBUG("EvarobotSonar: Sonar[%d] is not ALIVE ", this->i_id);
			this->b_is_alive = false;
			return false;
		}

		ss.str("");
		ss << this->i_id;
		strcpy(read_buf, ss.str().c_str());

		ret_val = read(this->i_fd, read_buf, sizeof(read_buf));
		raw_dist = atoi(read_buf);
	}

	this->b_is_alive = true; 
	return true;
}

/**
 * Writes sonar id to sonar driver file.
 */
void IMSONAR::Trigger()
{
	stringstream ss;
	int raw_dist = -1;

	ss << this->i_id;
	write(this->i_fd, ss.str().c_str(), sizeof(ss.str().c_str()));
}

/**
 * Reads file and gets distance data.
 */
int IMSONAR::Echo()
{
	stringstream ss;
	char read_buf[100];
	int raw_dist = -1;
	int ret_val;

	/**
	 * Raw data is read
	 */
	ss << this->i_id;

	strcpy(read_buf, ss.str().c_str());
	
	ret_val = read(this->i_fd, read_buf, sizeof(read_buf));
	raw_dist = atoi(read_buf);

	/**
	 * ROS message content is filled.
	 */
	this->sonar_msg.range = (float)(raw_dist * 0.0001);
	this->sonar_msg.header.stamp = ros::Time::now();
	this->sonar_msg.field_of_view = this->dynamic_params->d_fov;
	this->sonar_msg.min_range = this->dynamic_params->d_min_range;
	this->sonar_msg.max_range = this->dynamic_params->d_max_range;

	return ret_val;
}

/**
 * Sleeps the execution for 0.3 second.
 */
void IMSONAR::Wait()
{
	usleep(300000);
}

/**
 * Publishes data from sonar<id> topic.
 */
void IMSONAR::Publish()
{
	if(this->pub_sonar.getNumSubscribers() > 0 || this->dynamic_params->b_always_on)
	{
		this->pub_sonar.publish(this->sonar_msg);
		this->pub_sonar_freq->tick();
	}
}

/**
 * If an error occurs publishes it. Also publishes aliveness of sonar sensor.
 */
void IMSONAR::ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	if(i_error_code<0)
	{
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, GetErrorDescription(i_error_code).c_str());
        i_error_code = 0;
	}
	else if(this->b_is_alive)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Sonar%d is alive!", this->i_id);
	}
	else
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Sonar%d is NOT alive!", this->i_id);
	}
}

/**
 * Changes values of variables at runtime.
 */
void IMDynamicReconfig::CallbackReconfigure(evarobot_sonar::SonarParamsConfig &config, uint32_t level)
{
	ROS_DEBUG("EvarobotSonar: Updating Sonar Params...");
		
	this->d_fov = config.field_of_view;
	this->d_min_range = config.minRange;
	this->d_max_range = config.maxRange;
	this->b_always_on = config.alwaysOn;
}

/**
 * IMDynamicReconfig constructor.
 */
IMDynamicReconfig::IMDynamicReconfig():n_private("~")
{
	/**
	 * Get default parameters.
	 */
	n_private.param("alwaysOn", this->b_always_on, false);
	n_private.param("field_of_view", this->d_fov, 0.7);
	n_private.param("minRange", this->d_min_range, 0.0);
	n_private.param("maxRange", this->d_max_range, 5.0);
	n_private.param("maxFreq", this->d_max_freq, 10.0);
	n_private.param("minFreq", this->d_min_freq, 0.2);
		
	ROS_DEBUG("EvarobotSonar: b_always_on: %d", this->b_always_on);
	ROS_DEBUG("EvarobotSonar: d_fov: %f", this->d_fov);
	ROS_DEBUG("EvarobotSonar: d_min_range: %f", this->d_min_range);
	ROS_DEBUG("EvarobotSonar: d_max_range: %f", this->d_max_range);
	
	/**
	 * Set dynamic reconfigure to change values of variables at runtime.
	 */
	this->f = boost::bind(&IMDynamicReconfig::CallbackReconfigure, this, _1, _2);
	this->srv.setCallback(this->f);
}

/**
 * Program start here.
 */
int main(int argc, char **argv)
{
	/**
	 * File descriptor of sonar driver file.
	 */
	int i_fd;
	stringstream ss;

	/**
	 * Aliveness vector of sonar sensors.
	 */
	vector<bool> b_sonar_alive;

	/**
	 * Vector of sonar sensors in type of IMSONAR.
	 */
	vector<IMSONAR *> sonar;
  
	/**
	 * GPIO pins of sonar sensors.
	 */
	vector<int> T_i_sonar_pins;

	/**
	 * How many sonar sensors are triggered at one time.
	 */
	int i_element_no;

	/**
	 * Frequency variable.
	 */
	double d_frequency;

	/**
	 * Path of sonar driver.
	 */
	string str_driver_path;
	
	/**
	 * Initializes ROS node with evarobot_sonar name.
	 */
	ros::init(argc, argv, "/evarobot_sonar");

	/**
	 * Creates ROS node handler.
	 */
	ros::NodeHandle n;
	
	/**
	 * Gets parameters from configuration file.
	 */
	n.param<string>("evarobot_sonar/driverPath", str_driver_path, "/dev/evarobotSonar");
	n.param<double>("evarobot_sonar/frequency", d_frequency, 10.0);
	n.param<int>("evarobot_sonar/element", i_element_no, 3);
	n.getParam("evarobot_sonar/sonarPins", T_i_sonar_pins);
		
	if(T_i_sonar_pins.size() > MAX_SONAR)
	{
		ROS_INFO(GetErrorDescription(-120).c_str());
        i_error_code = -120;
	}

	ROS_DEBUG("EvarobotSonar: Number of sonars: %d", T_i_sonar_pins.size());

	/**
	 * Open sonar driver file.
	 */
	i_fd = open(str_driver_path.c_str(), O_RDWR);
	if(i_fd < 0)
	{
		ROS_INFO(GetErrorDescription(-121).c_str());
        i_error_code = -121;
		exit(-1);
	}

	/**
	 * Define frequency
	 */
	ros::Rate loop_rate(d_frequency);
	
	/**
	 * Create number of sonar objects.
	 */
	boost::shared_ptr<IMDynamicReconfig> dyn_params(new IMDynamicReconfig);	
	for(uint i = 0; i < T_i_sonar_pins.size(); i++)
	{
		b_sonar_alive.push_back(true);
		if(T_i_sonar_pins[i] < MAX_SONAR)
		{
			IMSONAR * dummy_sonar = new IMSONAR(i_fd, T_i_sonar_pins[i], SONAR[T_i_sonar_pins[i]], dyn_params);
			sonar.push_back(dummy_sonar);
		}
		else
		{
			ROS_INFO(GetErrorDescription(-122).c_str());
			i_error_code = -122;
		}
	}
	double d_dummy = (double)T_i_sonar_pins.size() / (double)i_element_no;
	int i_group_max = (int)ceil(d_dummy);
	ROS_DEBUG("Group MAX -> %d", i_group_max);
	ROS_DEBUG("Pins -> %d", T_i_sonar_pins.size());
	ROS_DEBUG("element -> %d", i_element_no);

	int i_check_sonar = 0;

	while(ros::ok())
	{
		/**
		 * Checks one sonar at each loop.
		 */
		b_sonar_alive[i_check_sonar] = sonar[i_check_sonar]->Check();
		i_check_sonar++;
		if(i_check_sonar >= T_i_sonar_pins.size())
			i_check_sonar = 0;

		/**
		 * Reads sonar distances group by group.
		 */
		for(int i_group_no = 0; i_group_no < i_group_max; i_group_no++)
		{
			for(int i = (i_group_no * i_element_no); i < ((i_group_no+1) * i_element_no) && i < T_i_sonar_pins.size(); i++)
			{
				if(b_sonar_alive[i])
					sonar[i]->Trigger();
			}
			usleep(40000);

			for(int i = (i_group_no * i_element_no); i < ((i_group_no+1) * i_element_no) && i < T_i_sonar_pins.size(); i++)
			{
				if(b_sonar_alive[i])
				{
					sonar[i]->Echo();
					sonar[i]->Publish();
				}
				sonar[i]->updater.update();
			}
		}
		
		/**
		 * Loop is slept to hold frequency.
		 */
		ros::spinOnce();
	}
	
	/**
	 * Closes sonar driver file.
	 */
	close(i_fd);
	return 0;
}
