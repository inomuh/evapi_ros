#include "evarobot_bumper/evarobot_bumper.h"

/**
 * If an error occurs publishes it.
 * If there are any collision, publishes a warning.
 */
 class LinuxFile
 {
 private:
    int m_Handle;
 
 public:
     LinuxFile(const char *pFile, int flags = O_RDWR)
     {
         m_Handle = open(pFile, flags);
     }
 
     ~LinuxFile()
     {
        if (m_Handle != -1)
             close(m_Handle);
     }
 
     size_t Write(const void *pBuffer, size_t size)
     {
         return write(m_Handle, pBuffer, size);
     }
 
     size_t Read(void *pBuffer, size_t size)
     {
         return read(m_Handle, pBuffer, size);
     }
 
     size_t Write(const char *pText)
     {
         return Write(pText, strlen(pText));
     }
 
     size_t Write(int number)
     {
         char szNum[32];
         snprintf(szNum, sizeof(szNum), "%d", number);
         return Write(szNum);
     }
 };
 
 class LinuxGPIOExporter
 {
 protected:
    int m_Number;
 
 public:
     LinuxGPIOExporter(int number)
         : m_Number(number)
     {
         LinuxFile("/sys/class/gpio/export", O_WRONLY).Write(number);
     }
 
     ~LinuxGPIOExporter()
     {
         LinuxFile("/sys/class/gpio/unexport", O_WRONLY).Write(m_Number);
     }
 };
 
 class LinuxGPIO : public LinuxGPIOExporter
 {
 public:
     LinuxGPIO(int number)
         : LinuxGPIOExporter(number)
     {
     }
 
    void SetValue(bool value)
     {
         char szFN[128];
         snprintf(szFN, sizeof(szFN), "/sys/class/gpio/gpio%d/value", m_Number);
         LinuxFile(szFN).Write(value ? "1" : "0");
     }
 
    void SetDirection(bool isOutput)
     {
        char szFN[128];
         snprintf(szFN, sizeof(szFN), 
            "/sys/class/gpio/gpio%d/direction", m_Number);
         LinuxFile(szFN).Write(isOutput ? "out" : "in");
     }
     void GetValue(char pBuffer2[])
     {
         char szFN[128];
         snprintf(szFN, sizeof(szFN), "/sys/class/gpio/gpio%d/value", m_Number);
         LinuxFile(szFN).Read(pBuffer2,2);
     }
 };
void ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    stringstream ss;
    bool b_any_coll = false;

    ss << "Collision: ";

    
    
        if(!b_collision)
        {
            ss << "Bumper[" << 0 << "] ";
            b_any_coll = true;
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
    LinuxGPIO gpio27(21);
    gpio27.SetDirection(false);
    bool on = true;
    char pBuffer2[2];
	/**
	 * Semaphore variables.
	 */
    

    /**
	 * Frequency, minimum frequency and maximum frequency variables.
	 */
    double d_frequency;
    double d_min_freq, d_max_freq;

    /**
     * i2c device path.
     */
    

    /**
     * ROS message that holds bumper data.
     */
    

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
    ros::Publisher pub_bumper = n.advertise<std_msgs::Bool>("bumper", 10);

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
        std_msgs::Bool bumper_state;
        
        bool b_data;
        /**
         * Reads bumper state from eio pins.
         */
         gpio27.GetValue(pBuffer2);
         
         if(pBuffer2[0]=='1')
         {  
            b_data=true;
         }
         else
         {
            b_data=false;
         }
        
            
			

            b_collision= b_data;
            bumper_state.data = b_data;
    
        


        /**
		 * If there are any subscriber or b_always_on is set as true, message is published.
		 * Else, message is not published.
		 */
        if(pub_bumper.getNumSubscribers() > 0 || b_always_on)
        {
            pub_bumper.publish(bumper_state);
            pub_freq.tick();
        }

        /**
         * Clears data for future usages.
         */

        /**
         * Loop is slept to hold frequency.
         */
        updater.update();
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
