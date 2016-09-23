#include "evarobot_bumper/evarobot_bumper.h"

/**
 * If an error occurs publishes it.
 * If there are any collision, publishes a warning.
 */
void ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    stringstream ss;
    bool b_any_coll = false;

    ss << "Collision: ";

    for(int i = 0; i < MAX_BUMPER; i++)
    {
        if(!b_collision[i])
        {
            ss << "Bumper[" << i << "] ";
            b_any_coll = true;
        }
    }

    if(i_error_code<0)
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "%s", GetErrorDescription(i_error_code).c_str());
        i_error_code = 0;
    }
    else if(b_any_coll)
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "%s", ss.str().c_str());
    }
    else
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No collision!");
    }
}

/**
 * Callback function to change b_always_on variable at runtime.
 */
void CallbackReconfigure(evarobot_bumper::ParamsConfig &config, uint32_t level)
{
	ROS_DEBUG("EvarobotBumper: Bumper Param Callback Function...");
	b_is_received_params = true;
    b_always_on = config.alwaysOn;
}

/**
 * Program starts from here.
 */
int main(int argc, char **argv)
{
	/**
	 * Semaphore variables.
	 */
    key_t key = 1005;
    sem_t *mutex;
    FILE * fd;

    /**
	 * create & initialize semaphore
	 */
    mutex = sem_open(SEM_NAME,O_CREAT,0644,1);
    if(mutex == SEM_FAILED)
    {
        ROS_INFO(GetErrorDescription(-44).c_str());
        i_error_code = -44;

        sem_unlink(SEM_NAME);

        return(-1);
    }

    /**
	 * Frequency, minimum frequency and maximum frequency variables.
	 */
    double d_frequency;
    double d_min_freq, d_max_freq;

    /**
     * i2c device path.
     */
    string str_i2c_path;

    /**
     * ROS message that holds bumper data.
     */
    im_msgs::Bumper bumpers;

    /**
	 * Initializes ROS node with evarobot_bumper name.
	 */
    ros::init(argc, argv, "/evarobot_bumper");

    /**
	 * Creates ROS node handler.
	 */
    ros::NodeHandle n;

    /**
	 * Gets parameters from configuration file.
	 */
    n.param<string>("evarobot_bumper/i2c_path", str_i2c_path, "/dev/i2c-1");
    n.param("evarobot_bumper/alwaysOn", b_always_on, false);
    n.param("evarobot_bumper/minFreq", d_min_freq, 0.2);
    n.param("evarobot_bumper/maxFreq", d_max_freq, 10.0);
    if(!n.getParam("evarobot_bumper/frequency", d_frequency))
    {
        ROS_INFO(GetErrorDescription(-27).c_str());
        i_error_code = -27;
    }

    /**
	 * Publisher topic is created with bumper topic name and im_msgs::Bumper message type.
	 */
    ros::Publisher pub_bumper = n.advertise<im_msgs::Bumper>("bumper", 10);

    /**
     * Dynamic reconfigure is set to provide changing variables at runtime.
     */
    dynamic_reconfigure::Server<evarobot_bumper::ParamsConfig> srv;
    dynamic_reconfigure::Server<evarobot_bumper::ParamsConfig>::CallbackType f;
    f = boost::bind(&CallbackReconfigure, _1, _2);
    srv.setCallback(f);

    /**
	 * Define frequency
	 */
    ros::Rate loop_rate(d_frequency);

    /**
     * Object is created from IMEIO class and bumper pins are set as input.
     */
    IMEIO * eio;
    try {
        eio = new IMEIO(0b00100000, string("/dev/i2c-1"),  mutex);
		eio->SetPinADirection(IMEIO::BUMPER0, IMEIO::INPUT);
        eio->SetPinADirection(IMEIO::BUMPER1, IMEIO::INPUT);
        eio->SetPinADirection(IMEIO::BUMPER2, IMEIO::INPUT);
    }
    catch(int i) {
    	ROS_INFO(GetErrorDescription(i).c_str());
        i_error_code = i;
    }

    /**
     * This vector stores bumper numbers.
     */
    vector<int> T_i_bumper_no;

    T_i_bumper_no.resize(MAX_BUMPER);

    T_i_bumper_no[0] = IMEIO::BUMPER0;
    T_i_bumper_no[1] = IMEIO::BUMPER1;
    T_i_bumper_no[2] = IMEIO::BUMPER2;

    /**
	 * Set diagnostics to handle and publish error.
	 */
    diagnostic_updater::Updater updater;
    updater.setHardwareID("IM-BMP10");
    updater.add("bumper", &ProduceDiagnostics);

    diagnostic_updater::HeaderlessTopicDiagnostic pub_freq("bumper", updater,
            diagnostic_updater::FrequencyStatusParam(&d_min_freq, &d_max_freq, 0.1, 10));

    while(ros::ok())
    {
    	/**
    	 * If new parameters are set, information message is written.
    	 */
    	if(b_is_received_params)
		{
			ROS_DEBUG("EvarobotBumper: Updating Bumper Params...");

			b_is_received_params = false;
		}

        /**
         * Reads bumper state from eio pins.
         */
        for(uint i = 0; i < T_i_bumper_no.size(); i++)
        {
            im_msgs::BumperState bumper_state;

            bool b_data;
			try{
					eio->GetPinAValue(T_i_bumper_no[i], b_data);
			} catch(int e) {
				ROS_INFO(GetErrorDescription(e).c_str());
				i_error_code = e;
			}

            b_collision[i] = b_data;
            bumper_state.bumper_state = b_data;
            bumpers.state.push_back(bumper_state);
        }

        bumpers.header.stamp = ros::Time::now();

        /**
		 * If there are any subscriber or b_always_on is set as true, message is published.
		 * Else, message is not published.
		 */
        if(pub_bumper.getNumSubscribers() > 0 || b_always_on)
        {
            pub_bumper.publish(bumpers);
            pub_freq.tick();
        }

        /**
         * Clears data for future usages.
         */
        bumpers.state.clear();

        /**
         * Loop is slept to hold frequency.
         */
        updater.update();
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
