/*
 * IMPWM.cpp
 *
 *  Created on: Mar 19, 2015
 *      Author: makcakoca
 */

#include "../include/IMPWM.h"

//Private constants
const int IMPWM::BCM2708_PERI_BASE;
const int IMPWM::PWM_BASE;/* PWM controller */
const int IMPWM::CLOCK_BASE; /* Clock controller */
const int IMPWM::GPIO_BASE; /* GPIO controller */

const int IMPWM::PWM_CTL ;

const int IMPWM::PWM_RNG1;
const int IMPWM::PWM_DAT1;

const int IMPWM::PWM_RNG2;
const int IMPWM::PWM_DAT2;

const int IMPWM::PWMCLK_CNTL;
const int IMPWM::PWMCLK_DIV;
const int IMPWM::BLOCK_SIZE;


//Public constants
const int IMPWM::PWMMODE;
const int IMPWM::MSMODE;
const int IMPWM::ERRFREQ;
const int IMPWM::ERRCOUNT;
const int IMPWM::ERRDUTY;
const int IMPWM::ERRMODE;

/***********************************************************************
 * rpiPWM::rpiPWM()
 * This is the Default constructor. First, it mmaps the registers in
 * Physical memory responsible for configuring GPIO, PWM and the PWM clock.
 * It then sets the frequency to 1KHz, PWM resolution to 256, duty
 * cycle to 50% & pwm mode to 'PWMMODE'
 * It then calls configPWMPin() to configure GPIO18 to ALT5 to allow it to
 * output PWM1 waveforms.
 * Finally  configPWM() is called to configure the PWM1 peripheral
 ***********************************************************************/
IMPWM::IMPWM()
{
  this->p_v_u_clk = MapRegisterAddres(CLOCK_BASE);// map PWM clock registers into memory
  this->p_v_u_pwm = MapRegisterAddres(PWM_BASE); //map PWM registers into memory
  this->p_v_u_gpio = MapRegisterAddres(GPIO_BASE);// map GPIO registers into memory
  this->d_frequency = 1000.0; // set frequency
  this->u_i_counts = 256; //set PWM resolution
  this->d_dutyCycle = 50.0; //set duty cycle
  this->i_mode = PWMMODE; // set pwm mode


  ConfigPWMPin(); //configure GPIO18 to ALT15 (PWM output)
  ConfigPWM();   // configure PWM1

}

/***********************************************************************
 * rpiPWM::rpiPWM(double Hz, unsigned int cnts, double duty, unsigned int m)
 * This is the overloaded constructor. First, it mmaps the registers in
 * Physical memory responsible for configuring GPIO, PWM and the PWM clock.
 * It then sets the frequency, PWM resolution, duty cycle & pwm mode as
 * per the parameters provided.
 *
 * It then calls configPWMPin() to configure GPIO18 to ALT5 to allow it to
 * output PWM1 waveforms.
 * Finally  configPWM() is called to configure the PWM1 peripheral
 * Parameters: - Hz (double) - Frequency
 *             - cnts (unsigned int) - PWM resolution (counts)
 *             - duty (double) - Duty Cycle as a percentage
 *             - m     (int) - PWM mode (can be either 1 for PWMMODE (rpiPWM::PWMMODE)
 *               or 2 for MSMODE (rpiPWM::MSMODE)
 ***********************************************************************/
IMPWM::IMPWM(double d_Hz, unsigned int u_i_counts, double d_duty,  int i_m)
{

  this->p_v_u_clk = MapRegisterAddres(CLOCK_BASE);
  this->p_v_u_gpio = MapRegisterAddres(GPIO_BASE);
  this->p_v_u_pwm = MapRegisterAddres(PWM_BASE);


   if( (u_i_counts < 0) || (u_i_counts > UINT_MAX) ) {
   printf("counts value must be between 0-%d\n",UINT_MAX);
   exit(1);
  }

  if ((d_Hz < 1e-5) || (d_Hz > 19200000.0f)){
    printf("frequency value must be between 0-19200000\n");
    exit(1);
  }

  if( (d_duty < 1e-5) || (d_duty> 99.99999) ) {
    printf("dutyCycle value must be between 0-99.99999\n");
    exit(1);
  }

  if( (i_m != PWMMODE) && (i_m != MSMODE) ) {
    printf("mode must be either PWMMODE(1) or MSMODE(2)\n");
    exit(1);
  }

  this->d_frequency = d_Hz;
  this->u_i_counts = u_i_counts;
  this->d_dutyCycle = d_duty;
  this->i_mode = i_m;
  ConfigPWMPin();
  ConfigPWM();
  SetDutyCycleCount(0, 0);
  SetDutyCycleCount(0, 1);
}

/***********************************************************************
 * rpiPWM::~rpiPWM()
 * Destructor - Puts all Peripheral registers in their original (reset state)
 * and then unmaps the portions of memory containing to register addresses
 * for the PWM clock, PWM and GPIO peripherals
 ***********************************************************************/
IMPWM::~IMPWM()
{
	printf("pwm kapatiliyor\n");

	//lets put the PWM peripheral registers in their original state
	*(p_v_u_pwm + PWM_CTL) = 0;
	*(p_v_u_pwm + PWM_RNG1) = 0x20;
	*(p_v_u_pwm + PWM_DAT1) = 0;

    // unmap the memory block containing PWM registers
    if(munmap((void*)p_v_u_pwm, BLOCK_SIZE) < 0){
		perror("munmap (pwm) failed");
		exit(1);
	}
	//lets put the PWM Clock peripheral registers in their original state
    //kill PWM clock
    *(p_v_u_clk + PWMCLK_CNTL) = 0x5A000000 | (1 << 5);
    usleep(10);

    // wait until busy flag is set
    while ( (*(p_v_u_clk + PWMCLK_CNTL)) & 0x00000080){}

    //reset divisor
    *(p_v_u_clk + PWMCLK_DIV) = 0x5A000000;
    usleep(10);

    // source=osc and enable clock
    *(p_v_u_clk + PWMCLK_CNTL) = 0x5A000011;

    // unmap the memory block containing PWM Clock registers
    if(munmap((void*)p_v_u_clk, BLOCK_SIZE) < 0){
		perror("munmap (clk) failed");
		exit(1);
	}

   //lets put the GPIO peripheral registers in their original state
   //first put it in input mode (default)
   //taken from #define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
   *(p_v_u_gpio+1) &= ~(7 << 24);
   //then munmap
    if(munmap((void*)p_v_u_gpio, BLOCK_SIZE) < 0){
		perror("munmap (gpio) failed");
		exit(1);
	}
}


/***********************************************************************
 * unsigned int rpiPWM::setFrequency(const double &hz)
 * This function sets the PWM frequency and then reinitializes the PWM1
 * peripheral to update the frequency. The function performs a check to
 * ensure that the PWM frequency is between 0 & 19.2MHz
 * Parameters: hz (double) - Frequency in Hz
 * Return Value: 0 if successful or rpiPWM::ERRFREQ if frequency
 *               parameter is invalid
 ***********************************************************************/
 unsigned int IMPWM::SetFrequency(const double & c_d_hz)
 {
	 unsigned int u_i_return_value = 0;
	if (c_d_hz < 1e-5 || c_d_hz > 19200000.0f)
	{ // make sure that Frequency is valid
		u_i_return_value = ERRFREQ; //if not return error code
	}
	else
	{
		this->d_frequency = c_d_hz;
		ConfigPWM();
	}

	return u_i_return_value; // return 0 for success.....
}


/***********************************************************************
 * unsigned int rpiPWM::setCounts(const int &cnts)
 * This function sets the PWM resolution and then reinitializes the PWM1
 * peripheral to update the PWM resolution (counts). The function performs a check to
 * ensure that the PWM resolution is between 0 &  UINT_MAX (its a 32-bit register)
 * Parameters: cnts (unsigned int) - counts
 * Return Value: 0 if successful or rpiPWM::ERRCOUNT if count value is invalid
 ***********************************************************************/
unsigned int IMPWM::SetCounts(const unsigned int & c_u_i_counts)
{
	unsigned int u_i_return_value = 0;

	if( (c_u_i_counts < 0) || (c_u_i_counts > UINT_MAX) )
	{
		u_i_return_value = ERRCOUNT;
	}
	else
	{
		this->u_i_counts = c_u_i_counts;
		ConfigPWM();
	}

	return u_i_return_value;
}

/***********************************************************************
 * unsigned int rpiPWM::setDutyCycle(const double &duty)
 * This function sets the PWM DutyCycle while the PWM peripheral is running.
 * The function performs a check to ensure that the PWM Duty Cycle is between
 * 0 & 99.99999 %
 * Parameters: duty (double) - Duty Cycle in %
 * Return Value: 0 if successful or rpiPWM::ERRDUTY if Duty cycle is invalid
 ****************************************************************************/
unsigned int IMPWM::SetDutyCycle(const double &c_d_duty, int i_pwm_no)
{
	unsigned int u_i_bit_count = 0;
	unsigned int u_i_return_value = 0;

	if( (c_d_duty < 1e-5) || (c_d_duty > 99.99999) )
	{
		u_i_return_value = ERRDUTY;
	}
	else
	{
		this->d_dutyCycle = c_d_duty;
		u_i_bit_count = (int) ((this->d_dutyCycle/100.0) * this->u_i_counts);
		if(i_pwm_no == 0)
			*(p_v_u_pwm + PWM_DAT1) = u_i_bit_count;
		else
			*(p_v_u_pwm + PWM_DAT2) = u_i_bit_count;
	}

	return u_i_return_value;
}

/***********************************************************************
 * unsigned int rpiPWM::setDutyCycleForce(const double &duty, unsigned int &m)
 * This function firsts stops the PWM1 peripheral, sets the PWM DutyCycle
 * and the PWM mode and then re-enables the PWM1 peripheral in the new mode
 * The function performs a check to ensure that the PWM Duty Cycle is between
 * 0 & 99.99999 % and that an appropriate mode is selected.
 *
 * Parameters: duty (double) - Duty Cycle in %
 *             m (int) - pwm mode (rpiPWM::PWMMODE or rpiPWM::MSMODE)
 * Return Value: 0 if successful or rpiPWM::ERRDUTY if Duty cycle is invalid
 *******************************************************************************/
unsigned int IMPWM::SetDutyCycleForce(const double &c_d_duty, const  int &c_i_m, int i_pwm_no)
{
	unsigned int u_i_return_value = 0;

	if( (c_i_m != PWMMODE) && (c_i_m != MSMODE) )
	{
		u_i_return_value = ERRMODE;
	}
	else if( (c_d_duty < 1e-5) || (c_d_duty > 99.99999) )
	{
		u_i_return_value = ERRDUTY;
	}
	else
	{
		this->i_mode = c_i_m;
		this->d_dutyCycle = c_d_duty;
		// disable PWM & start from a clean slate
		*(p_v_u_pwm + PWM_CTL) = 0;
		// needs some time until the PWM module gets disabled, without the delay the PWM module crashs
		usleep(10);
		if(i_pwm_no == 0)
		{
			// set the number of counts that constitute a period
			*(p_v_u_pwm + PWM_RNG1) = this->u_i_counts;
			//set  duty cycle
			*(p_v_u_pwm + PWM_DAT1) = (int) ((this->d_dutyCycle / 100.0) * this->u_i_counts);
			// start PWM1 in
			if(this->i_mode == PWMMODE) //PWM mode
				*(p_v_u_pwm + PWM_CTL) |= (1 << 0);
			else // M/S Mode
				*(p_v_u_pwm + PWM_CTL) |= ( (1 << 7) | (1 << 0) );

		}
		else
		{
			// set the number of counts that constitute a period
			*(p_v_u_pwm + PWM_RNG2) = this->u_i_counts;
            //set  duty cycle
			*(p_v_u_pwm + PWM_DAT2) = (int) ((this->d_dutyCycle/100.0) * this->u_i_counts);
			// start PWM1 in
			if(this->i_mode == PWMMODE) //PWM mode
				*(p_v_u_pwm + PWM_CTL) |= (1 << 8);
			else // M/S Mode
				*(p_v_u_pwm + PWM_CTL) |= ( (1 << 15) | (1 << 8) );
		}

	}
	return u_i_return_value;
}

/***********************************************************************
 * unsigned int rpiPWM::setDutyCycleCount(unsigned int &dutyCycleCnts )
 * This function sets the PWM DutyCycle as a function of PWM resolution,
 * while the PWM peripheral is running. The function performs a check to
 * ensure that the PWM Duty Cycle count value is between 0 and count
 * Parameters: dutyCycleCnts (unsigned int) - Duty Cycle in counts
 * Return Value:0 if successful or rpiPWM::ERRDUTY if Duty cycle is invalid
 ***********************************************************************/
unsigned int IMPWM::SetDutyCycleCount(const unsigned int &c_u_i_counts, int i_pwm_no)
{
	unsigned int u_i_return_value = 0;

    if( (c_u_i_counts < 0) || ( c_u_i_counts > this->u_i_counts ))
    {
    	u_i_return_value = ERRDUTY;
	}
	else
	{
		this->d_dutyCycle = ((c_u_i_counts * 1.0)/ this->u_i_counts) * 100.0;
		if(i_pwm_no == 0)
			*(p_v_u_pwm + PWM_DAT1) = c_u_i_counts;
		else
			*(p_v_u_pwm + PWM_DAT2) = c_u_i_counts;
	}

	return u_i_return_value;
}

/***********************************************************************
 * unsigned int rpiPWM::setMode(unsigned int &m)
 * This function sets the PWM mode. The function performs a check to
 * ensure that a valid PWM mode is requested.
 * Parameters: m (int) - pwm mode (rpiPWM::PWMMODE or rpiPWM::MSMODE)
 * Return Value: 0 if successful or rpiPWM::ERRMODE if MODE is invalid
 ********************************************************************************/
unsigned int IMPWM::SetMode(const  int &c_i_m)
{
	unsigned int u_i_return_value = 0;

	if( (c_i_m != PWMMODE) && (c_i_m != MSMODE) )
	{
		u_i_return_value = ERRMODE;
	}
	else
	{
		this->i_mode = c_i_m;
		SetDutyCycleForce(this->d_dutyCycle, this->i_mode, 0);
		SetDutyCycleForce(this->d_dutyCycle, this->i_mode, 1);
	 }

	return u_i_return_value;
}


/***********************************************************************
 *These are a bunch of 'getter' functions that enable the user
 * to access the PWM frequency, resolution, duty cycle and mode as well
 * as the PWM clock divisor value.
 *********************************************************************/
 double IMPWM::GetFrequency() const { return this->d_frequency;}

 int IMPWM::GetCounts() const { return this->u_i_counts;}

 int IMPWM::GetDivisor() const {return this->u_i_divisor;}

 double IMPWM::GetDutyCycle() const {return this->d_dutyCycle;}

 int IMPWM::GetMode() const{return this->i_mode;}

/***********************************************************************
 *	volatile unsigned *rpiPWM::mapRegAddr(unsigned long baseAddr)
 * This function maps a block (4KB) of physical memory into the memory of
 * the calling process. It enables a user space process to access registers
 * in physical memory directly without having to interact with in kernel side
 * code i.e. device drivers
 * Parameter - baseAddr (unsigned long) - this is the base address of a 4KB
 *             block of physical memory that will be mapped into the user
 *             space process memory.
 * Return Value - mapped pointer in process memory
 ***********************************************************************/
volatile unsigned *IMPWM::MapRegisterAddres(unsigned long u_l_base_address)
{
	int i_mem_fd = 0;
	void *p_v_register_address_map = MAP_FAILED;

	if (!i_mem_fd)
	{
		if ((i_mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0)
		{
			perror("can't open /dev/mem");
			exit (1);
		}
	}

   /* mmap IO */
	p_v_register_address_map = mmap(
      NULL,             //Any adddress in our space will do
      BLOCK_SIZE,       //Map length
      PROT_READ|PROT_WRITE|PROT_EXEC,// Enable reading & writting to mapped memory
      MAP_SHARED|MAP_LOCKED,       //Shared with other processes
	  i_mem_fd,           //File to map
	  u_l_base_address         //Offset to base address
	  );

	if (p_v_register_address_map == MAP_FAILED)
	{
		perror("mmap error");
		close(i_mem_fd);
		exit (1);
	}

	if(close(i_mem_fd) < 0)
	{ //No need to keep mem_fd open after mmap
	   //i.e. we can close /dev/mem
		perror("couldn't close /dev/mem file descriptor");
		exit(1);
	}

	return (volatile unsigned *)p_v_register_address_map;
}


/***********************************************************************
 * void rpiPWM::configPWMPin()
 * This function is responsible for putting GPIO18 in PWM mode by setting
 * its alternate function to ALT5. It firsts make the GPIO an input
 * (default state) and then switches to alternate function 5 (PWM mode)
 ***********************************************************************/
void IMPWM::ConfigPWMPin()
{

//#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
//#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
//#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

//#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
//#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

//#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH

//#define GPIO_PULL *(gpio+37) // Pull up/pull down
//#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock

	// PWM0

	// GPIO18 - ALT5 Config
	*(this->p_v_u_gpio + 1) &= ~(7 << 24);
	*(this->p_v_u_gpio + 1) |= (2 << 24);

	*(this->p_v_u_gpio + 1) &= ~(7 << 6);
//	printf("GPIO18\n");

	// GPIO13 - ALT0 Config
	*(this->p_v_u_gpio+1) &= ~(7 << 9);
	*(this->p_v_u_gpio+1) |= (4 << 9);

	*(this->p_v_u_gpio+1) &= ~(7 << 27);
//	printf("GPIO13\n");

}




/***********************************************************************
 * void  rpiPWM::configPWM()
 * This function configures the PWM1 peripheral.
 * - It stops the PWM clock
 * - Calculates an appropriate divisor value based on PWM Freq and resolution
 * - Writes this divisor value to the PWM clock
 * - Enables the PWM clock
 * - Disables the PWM peripheral
 * - Writes the PWM resolution and Duty Cycle to the appropriate registers
 * - Enables the PWM peripheral in the requested mode
 * *********************************************************************/
void  IMPWM::ConfigPWM()
{

  double d_period;
  double d_count_duration;

  // stop clock and waiting for busy flag doesn't work, so kill clock
  *(p_v_u_clk + PWMCLK_CNTL) = 0x5A000000 | (1 << 5);
  usleep(10);

  // wait until busy flag is set
  while ( (*(p_v_u_clk + PWMCLK_CNTL)) & 0x00000080){}

  //calculate divisor value for PWM1 clock...base frequency is 19.2MHz
  d_period = 1.0/this->d_frequency;
  d_count_duration = d_period/(this->u_i_counts*1.0f);
  this->u_i_divisor = (int)(19200000.0f / (1.0/d_count_duration));

  if( this->u_i_divisor < 0 || this->u_i_divisor > 4095 ) {
    printf("divisor value must be between 0-4095\n");
    exit(-1);
  }

  //set divisor
  *(p_v_u_clk + PWMCLK_DIV) = 0x5A000000 | (this->u_i_divisor << 12);

  // source=osc and enable clock
  *(p_v_u_clk + PWMCLK_CNTL) = 0x5A000011;

  // disable PWM & start from a clean slate
  *(p_v_u_pwm + PWM_CTL) = 0;

  // needs some time until the PWM module gets disabled, without the delay the PWM module crashs
  usleep(10);

  // set the number of counts that constitute a period with 0 for 20 milliseconds = 320 bits


    *(p_v_u_pwm + PWM_RNG1) = this->u_i_counts;
    *(p_v_u_pwm + PWM_RNG2) = this->u_i_counts;
    usleep(10);

    //set  duty cycle
    *(p_v_u_pwm + PWM_DAT1) = (int) ((this->d_dutyCycle/100.0) * this->u_i_counts);
    *(p_v_u_pwm + PWM_DAT2) = (int) ((this->d_dutyCycle/100.0) * this->u_i_counts);
    usleep(10);

    // start PWM1 in
    if(this->i_mode == PWMMODE) //PWM mode
    {
		*(p_v_u_pwm + PWM_CTL) |= (1 << 0) | (1 << 8);
//		*(p_v_u_pwm + PWM_CTL) |= (1 << 8);
    }
    else // M/S Mode
    {
		*(p_v_u_pwm + PWM_CTL) |= ( (1 << 7) | (1 << 0) ) | ( (1 << 15) | (1 << 8) );
	//	*(pwm + PWM_CTL) |= ( (1 << 15) | (1 << 8) );
	}

}




