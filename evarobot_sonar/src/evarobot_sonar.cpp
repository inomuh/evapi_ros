//! Bu sınıfa ait dokümantasyon evapi_ros 85rpm altındaki dokümantasyon ile aynıdır.

#include "evarobot_sonar/evarobot_sonar.h"

 
int i_error_code = 0;

bool b_ekb_test_status = false;


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

		//ROS_INFO("Timeout[%d]: %f ", this->i_id, stop-start);
		ss.str("");
		ss << this->i_id;
		strcpy(read_buf, ss.str().c_str());

		ret_val = read(this->i_fd, read_buf, sizeof(read_buf));
		raw_dist = atoi(read_buf);

	}

	this->b_is_alive = true; 
	return true;

}

void IMSONAR::Trigger()
{
	stringstream ss;
	int raw_dist = -1;

	ss << this->i_id;
	write(this->i_fd, ss.str().c_str(), sizeof(ss.str().c_str()));
}


int IMSONAR::Echo()
{
	stringstream ss;
	char read_buf[100];
	int raw_dist = -1;
	int ret_val;

	ss << this->i_id;

	strcpy(read_buf, ss.str().c_str());
	
	ret_val = read(this->i_fd, read_buf, sizeof(read_buf));
	raw_dist = atoi(read_buf);

	this->sonar_msg.range = (float)(raw_dist * 0.0001);
	this->sonar_msg.header.stamp = ros::Time::now();
	this->sonar_msg.field_of_view = this->dynamic_params->d_fov;
	this->sonar_msg.min_range = this->dynamic_params->d_min_range;
	this->sonar_msg.max_range = this->dynamic_params->d_max_range;

	return ret_val;
}

void IMSONAR::Wait()
{
	usleep(300000);
}

void IMSONAR::Publish()
{
	this->pub_sonar.publish(this->sonar_msg);
	this->pub_sonar_freq->tick();
}

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


/*
 * 
 * IMDynamicReconfig
 * 
 * 
 * */

void IMDynamicReconfig::CallbackReconfigure(evarobot_sonar::SonarParamsConfig &config, uint32_t level)
{
	ROS_DEBUG("EvarobotSonar: Updating Sonar Params...");
		
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
		
	ROS_DEBUG("EvarobotSonar: b_always_on: %d", this->b_always_on);
	ROS_DEBUG("EvarobotSonar: d_fov: %f", this->d_fov);
	ROS_DEBUG("EvarobotSonar: d_min_range: %f", this->d_min_range);
	ROS_DEBUG("EvarobotSonar: d_max_range: %f", this->d_max_range);
	
	
	// Dynamic Reconfigure
	this->f = boost::bind(&IMDynamicReconfig::CallbackReconfigure, this, _1, _2);
	this->srv.setCallback(this->f);
}


bool CallbackEKBTest(im_msgs::EKBTest::Request& request, im_msgs::EKBTest::Response& response)
{
	ROS_DEBUG("EvarobotSonar: EKB Test");
	b_ekb_test_status = request.b_on_off;
	response.b_success = true;
	return true;
}


int main(int argc, char **argv)
{
	int i_fd;
	stringstream ss;
	vector<bool> b_sonar_alive;
	vector<IMSONAR *> sonar;
  
  // ROS PARAMS
	vector<int> T_i_sonar_pins;
	int i_element_no;
	double d_frequency;
	string str_driver_path;
	// rosparams end
	
	unsigned int u_i_delay = 100*1000; // Note: Delay in ns
		
	ros::init(argc, argv, "/evarobot_sonar");
	ros::NodeHandle n;
	
	
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

	i_fd = open(str_driver_path.c_str(), O_RDWR);
	
	if(i_fd < 0)
	{
		ROS_INFO(GetErrorDescription(-121).c_str());
        i_error_code = -121;
		exit(-1);
	}
		
	/*****************************************************************/
			
	// Define frequency
	ros::Rate loop_rate(d_frequency);
	

	// create objects
	boost::shared_ptr<IMDynamicReconfig> dyn_params(new IMDynamicReconfig);
	
	ros::ServiceServer service = n.advertiseService("evarobot_sonar/ekb_test", CallbackEKBTest);
	
	while(!b_ekb_test_status && ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	

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
			b_sonar_alive[i_check_sonar] = sonar[i_check_sonar]->Check();
			i_check_sonar++;
			if(i_check_sonar >= T_i_sonar_pins.size())
				i_check_sonar = 0;

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
		ros::spinOnce();
	}
	
	close(i_fd);
	return 0;
}
