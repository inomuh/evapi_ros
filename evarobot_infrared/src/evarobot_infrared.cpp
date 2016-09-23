#include "evarobot_infrared/evarobot_infrared.h"

/**
 * IMInfrared constructor with custom parameters.
 */
IMInfrared::IMInfrared(int id,
					 boost::shared_ptr<IMADC> _adc,
					 boost::shared_ptr<IMDynamicReconfig> _dynamic_params):i_id(id), adc(_adc),
					 b_is_alive(false), dynamic_params(_dynamic_params)
{
	stringstream ss;
	
	/**
	 * Publisher is created with topic name ir<id> and message type sensor_msgs::Range
	 */
	ss << "ir" << this->i_id;
	this->pub_inf = this->n.advertise<sensor_msgs::Range>(ss.str().c_str(), 10);
	
	/**
	 * Set diagnostics to handle and publish error.
	 */
	this->updater.setHardwareID("SHARP");
	this->updater.add(ss.str().c_str(), this, &IMInfrared::ProduceDiagnostics);
	this->min_freq = this->dynamic_params->d_min_freq;
	this->max_freq = this->dynamic_params->d_max_freq;
	this->pub_inf_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(ss.str().c_str(), this->updater,
											diagnostic_updater::FrequencyStatusParam(&this->min_freq, &this->max_freq, 0.1, 10));
											
	
	/**
	 * Constants in ROS message are set.
	 */
	ss.str("");
	ss << this->n.resolveName(n.getNamespace(), true) << "/ir" << this->i_id << "_link";
	this->inf_msg.header.frame_id = ss.str();
	this->inf_msg.radiation_type = 1;
}

/**
 * Destructor
 */
IMInfrared::~IMInfrared()
{
	delete pub_inf_freq;
}

/**
 * Reads raw data from infrared device using adc. Converts raw data to range. Prepares ROS message with range value.
 */
bool IMInfrared::ReadRange()
{
	int i_raw_data = 0;
	bool b_ret = false;
	float f_range = 0.0;
	
	/**
	 * Raw data teken.
	 */
	try{		
		i_raw_data = this->adc->ReadChannel(this->i_id);
	}catch(int e){
		ROS_INFO(GetErrorDescription(e).c_str());
        i_error_code = e;
	}

	/**
	 * Range is calculated.
	 */
	f_range = 0.01 * ((1 / (this->dynamic_params->d_par_a * (double)i_raw_data + this->dynamic_params->d_par_b)) 
						- this->dynamic_params->d_par_k);
						
	if(f_range > this->dynamic_params->d_max_range || f_range < 0)
				f_range = this->dynamic_params->d_max_range;
	

	if(i_raw_data < 10)
	{
		this->b_is_alive = false;
		b_ret = false;
	}
	else
	{
		this->b_is_alive = true;
		b_ret = true;
	}
		
	/**
	 * ROS message content is filled.
	 */
	this->inf_msg.range = f_range;
	this->inf_msg.header.stamp = ros::Time::now();
	this->inf_msg.field_of_view = this->dynamic_params->d_fov;
	this->inf_msg.min_range = this->dynamic_params->d_min_range;
	this->inf_msg.max_range = this->dynamic_params->d_max_range;
	
	return b_ret;
}

/**
 * Publishes message which is prepared in ReadRange function.
 */
void IMInfrared::Publish()
{
	if(this->pub_inf.getNumSubscribers() > 0 || this->dynamic_params->b_always_on)
	{
		this->pub_inf.publish(this->inf_msg);
		this->pub_inf_freq->tick();
	}
}

/**
 * If an error occurs publishes it. Also publishes aliveness of infrared sensor.
 */
void IMInfrared::ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	if(i_error_code<0)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "%s", GetErrorDescription(i_error_code).c_str());
        i_error_code = 0;
	}
	else if(this->b_is_alive)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "INFRARED%d is alive!", this->i_id);
	}
	else
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "INFRARED%d is NOT alive!", this->i_id);
	}
}



/**
 * Changes values of variables at runtime.
 */
void IMDynamicReconfig::CallbackReconfigure(evarobot_infrared::ParamsConfig &config, uint32_t level)
{
	ROS_DEBUG("EvarobotInfrared: Updating Sonar Params...");
		
	this->d_fov = config.field_of_view;
	this->d_min_range = config.minRange;
	this->d_max_range = config.maxRange;
	this->b_always_on = config.alwaysOn;
	this->d_par_k = config.parK;
	this->d_par_a = config.parA;
	this->d_par_b = config.parB;
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
	n_private.param("parK", this->d_par_k, 0.42);
	n_private.param("parA", this->d_par_a, 0.0000473);
	n_private.param("parB", this->d_par_b, -0.0013461);
		
	ROS_DEBUG("EvarobotInfrared: b_always_on: %d", this->b_always_on);
	ROS_DEBUG("EvarobotInfrared: d_fov: %f", this->d_fov);
	ROS_DEBUG("EvarobotInfrared: d_min_range: %f", this->d_min_range);
	ROS_DEBUG("EvarobotInfrared: d_max_range: %f", this->d_max_range);
	
	/**
	 * Set dynamic reconfigure to change values of variables at runtime.
	 */
	this->f = boost::bind(&IMDynamicReconfig::CallbackReconfigure, this, _1, _2);
	this->srv.setCallback(this->f);
}

/**
 * Program starts here
 */
int main(int argc, char **argv)
{
	/**
	 * SPI mode between SPI_MODE_0 and SPI_MODE_3
	 */
	unsigned char u_c_spi_mode;

	/**
	 * Vector of infrared sensors in type of IMInfrared.
	 */
	vector<IMInfrared *> infrared;
	
	/**
	 * Infrared channels.
	 */
	vector<int> T_i_channels;

	/**
	 * Frequency variable.
	 */
	double d_frequency;
	
	/**
	 * Driver path of spi device.
	 */
	string str_driver_path;
	
	/**
	 * Defines spi mode.
	 */
	int i_spi_mode;

	/**
	 * Defines spi speed.
	 */
	int i_spi_speed;

	/**
	 * Defines spi bits.
	 */
	int i_spi_bits;

	/**
	 * Defines adc bits.
	 */
	int i_adc_bits;
	
	/**
	 * Initializes ROS node with evarobot_infrared name.
	 */
	ros::init(argc, argv, "/evarobot_infrared");
	
	/**
	 * Creates ROS node handler.
	 */
	ros::NodeHandle n;
	
	/**
	 * Gets parameters from configuration file.
	 */
	n.param<string>("evarobot_infrared/driverPath", str_driver_path, "/dev/spidev0.0");
	n.param("evarobot_infrared/spiMode", i_spi_mode, 0);
	n.param("evarobot_infrared/spiSpeed", i_spi_speed, 1000000);
	n.param("evarobot_infrared/spiBits", i_spi_bits, 8);
	n.param("evarobot_infrared/adcBits", i_adc_bits, 12);
	if(!n.getParam("evarobot_infrared/frequency", d_frequency))
	{
		ROS_INFO(GetErrorDescription(-100).c_str());
        i_error_code = -100;
	} 
	if(i_adc_bits > 16)
	{
		ROS_INFO(GetErrorDescription(-101).c_str());
        i_error_code = -101;
	}
	n.getParam("evarobot_infrared/channelNo", T_i_channels);
	if(T_i_channels.size() > MAX_IR)
	{
		ROS_INFO(GetErrorDescription(-102).c_str());
        i_error_code = -102;
	}
	ROS_DEBUG("EvarobotInfrared: Number of ir sensors: %d", T_i_channels.size());
	
	/**
	 * Define frequency
	 */
	ros::Rate loop_rate(d_frequency);

	/**
	 * Define spi mode
	 */
	switch(i_spi_mode)
	{
		case 0:
		{
			u_c_spi_mode = SPI_MODE_0;
			break;
		}
		
		case 1:
		{
			u_c_spi_mode = SPI_MODE_1;
			break;
		}
		
		case 2:
		{
			u_c_spi_mode = SPI_MODE_2;
			break;
		}
		
		case 3:
		{
			u_c_spi_mode = SPI_MODE_3;
			break;
		}
		
		default:
		{
			ROS_INFO(GetErrorDescription(-103).c_str());
			i_error_code = -103;
		}
	}
	
	/**
	 * Create spi and adc class objects.
	 */
	IMSPI * p_im_spi;
	boost::shared_ptr<IMADC> p_im_adc;
	boost::shared_ptr<IMDynamicReconfig> dyn_params(new IMDynamicReconfig);
	try{
		p_im_spi = new IMSPI(str_driver_path, u_c_spi_mode, i_spi_speed, i_spi_bits);
		p_im_adc = boost::shared_ptr<IMADC>(new IMADC(p_im_spi, i_adc_bits));
	}catch(int e){
		ROS_INFO(GetErrorDescription(e).c_str());
        i_error_code = e;
		return 0;
	}
	
	/**
	 * Create number of infrared objects.
	 */
	for(uint i = 0; i < T_i_channels.size(); i++)
	{
		if(T_i_channels[i] < MAX_IR)
		{
			IMInfrared * dummy_inf = new IMInfrared(T_i_channels[i], p_im_adc, dyn_params);
			infrared.push_back(dummy_inf);
		}
		else
		{
			ROS_INFO(GetErrorDescription(-104).c_str());
			i_error_code = -104;
		}
	}
		
	while(ros::ok())
	{
		/**
		 * Read ranges and publish to topic.
		 */
		for(uint i = 0; i < T_i_channels.size(); i++)
		{
			if(infrared[i]->ReadRange())
			{
				infrared[i]->Publish();
			}	
			infrared[i]->updater.update();
		}
		
		/**
		 * Loop is slept to hold frequency.
		 */
		ros::spinOnce();
		loop_rate.sleep();	
	}
	
	return 0;
}

