#include "../include/IMPWM.h"

/*
 * This is the Default constructor. First, it mmaps the registers in
 * Physical memory responsible for configuring GPIO, PWM and the PWM clock.
 * It then sets the frequency to 1KHz, PWM resolution to 256, duty
 * cycle to 50% & pwm mode to 'PWMMODE'
 * It then calls configPWMPin() to configure GPIO18 to ALT5 to allow it to
 * output PWM1 waveforms.
 * Finally  configPWM() is called to configure the PWM1 peripheral
 */
IMPWM::IMPWM()
{
	/**
	 * map PWM clock registers into memory
	 */
    this->p_v_u_clk = MapRegisterAddres(CLOCK_BASE);
    /**
     * map PWM registers into memory
     */
    this->p_v_u_pwm = MapRegisterAddres(PWM_BASE);
    /**
     * map GPIO registers into memory
     */
    this->p_v_u_gpio = MapRegisterAddres(GPIO_BASE);
    /**
	 * set frequency
	 */
    this->d_frequency = 1000.0;
    /**
     * set PWM resolution
     */
    this->u_i_counts = 256;
    /**
     * set duty cycle
     */
    this->d_dutyCycle = 50.0;
    /**
     * set pwm mode
     */
    this->i_mode = PWMMODE;

    try {
    	/**
    	 * configure GPIO18 to ALT15 (PWM output)
    	 */
        ConfigPWMPin();
        /**
         * configure PWM1
         */
        ConfigPWM();
    } catch(int i) {
        throw i;
    }

}

/*
 * This is the overloaded constructor. First, it mmaps the registers in
 * Physical memory responsible for configuring GPIO, PWM and the PWM clock.
 * It then sets the frequency, PWM resolution, duty cycle & pwm mode as
 * per the parameters provided.
 * It then calls configPWMPin() to configure GPIO18 to ALT5 to allow it to
 * output PWM1 waveforms.
 * Finally  configPWM() is called to configure the PWM1 peripheral
 */
IMPWM::IMPWM(double d_Hz, unsigned int u_i_counts, double d_duty,  int i_m)
{
    try {
        this->p_v_u_clk = MapRegisterAddres(CLOCK_BASE);
        this->p_v_u_gpio = MapRegisterAddres(GPIO_BASE);
        this->p_v_u_pwm = MapRegisterAddres(PWM_BASE);
    } catch(int i) {
        throw i;
    }

    if( (u_i_counts < 0) || (u_i_counts > UINT_MAX) ) {
        throw -44;
    }

    if ((d_Hz < 1e-5) || (d_Hz > 19200000.0f)) {
        throw -45;
    }

    if( (d_duty < 1e-5) || (d_duty> 99.99999) ) {
        throw -27;
    }

    if( (i_m != PWMMODE) && (i_m != MSMODE) ) {
        throw -28;
    }

    this->d_frequency = d_Hz;
    this->u_i_counts = u_i_counts;
    this->d_dutyCycle = d_duty;
    this->i_mode = i_m;
    try {
        ConfigPWMPin();
        ConfigPWM();
        SetDutyCycleCount(0, 0);
        SetDutyCycleCount(0, 1);
    } catch(int i) {
        throw i;
    }
}

/*
 * Destructor - Puts all Peripheral registers in their original (reset state)
 * and then unmaps the portions of memory containing to register addresses
 * for the PWM clock, PWM and GPIO peripherals
 */
IMPWM::~IMPWM()
{
    /**
     * Puts the PWM peripheral registers in their original state
     */
    *(p_v_u_pwm + PWM_CTL) = 0;
    *(p_v_u_pwm + PWM_RNG1) = 0x20;
    *(p_v_u_pwm + PWM_DAT1) = 0;

    /**
     * unmap the memory block containing PWM registers
     */
    if(munmap((void*)p_v_u_pwm, BLOCK_SIZE) < 0) {
        throw -66;
    }

    /**
     * lets put the PWM Clock peripheral registers in their original state
     * kill PWM clock
     */
    *(p_v_u_clk + PWMCLK_CNTL) = 0x5A000000 | (1 << 5);
    usleep(10);

    /**
     * wait until busy flag is set
     */
    while ( (*(p_v_u_clk + PWMCLK_CNTL)) & 0x00000080) {}

    /**
     * reset divisor
     */
    *(p_v_u_clk + PWMCLK_DIV) = 0x5A000000;
    usleep(10);

    /**
     * source=osc and enable clock
     */
    *(p_v_u_clk + PWMCLK_CNTL) = 0x5A000011;

    /**
     * unmap the memory block containing PWM Clock registers
     */
    if(munmap((void*)p_v_u_clk, BLOCK_SIZE) < 0) {
        throw -67;
    }

    /**
     * lets put the GPIO peripheral registers in their original state
     * first put it in input mode (default)
     * taken from #define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
     */
    *(p_v_u_gpio+1) &= ~(7 << 24);
    /**
     * then munmap
     */
    if(munmap((void*)p_v_u_gpio, BLOCK_SIZE) < 0) {
        throw -68;
    }
}


/*
 * This function sets the PWM frequency and then reinitializes the PWM1
 * peripheral to update the frequency. The function performs a check to
 * ensure that the PWM frequency is between 0 & 19.2MHz
 */
void IMPWM::SetFrequency(const double & c_d_hz)
{
    if (c_d_hz < 1e-5 || c_d_hz > 19200000.0f)
    {   
        throw -30;
    }
    else
    {
        this->d_frequency = c_d_hz;
        try {
            ConfigPWM();
        } catch(int i) {
            throw i;
        }
    }
}


/*
 * This function sets the PWM resolution and then reinitializes the PWM1
 * peripheral to update the PWM resolution (counts). The function performs a check to
 * ensure that the PWM resolution is between 0 &  UINT_MAX (its a 32-bit register)
 */
void IMPWM::SetCounts(const unsigned int & c_u_i_counts)
{
    if( (c_u_i_counts < 0) || (c_u_i_counts > UINT_MAX) )
    {
        throw -31;
    }
    else
    {
        this->u_i_counts = c_u_i_counts;
        try {
            ConfigPWM();
        } catch(int i) {
            throw i;
        }
    }
}

/*
 * This function sets the PWM DutyCycle while the PWM peripheral is running.
 * The function performs a check to ensure that the PWM Duty Cycle is between
 * 0 & 99.99999 %
 */
void IMPWM::SetDutyCycle(const double &c_d_duty, int i_pwm_no)
{
    unsigned int u_i_bit_count = 0;

    if( (c_d_duty < 1e-5) || (c_d_duty > 99.99999) )
    {
        throw -32;
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
}

/*
 * This function firsts stops the PWM1 peripheral, sets the PWM DutyCycle
 * and the PWM mode and then re-enables the PWM1 peripheral in the new mode
 * The function performs a check to ensure that the PWM Duty Cycle is between
 * 0 & 99.99999 % and that an appropriate mode is selected.
 */
void IMPWM::SetDutyCycleForce(const double &c_d_duty, const  int &c_i_m, int i_pwm_no)
{
    if( (c_i_m != PWMMODE) && (c_i_m != MSMODE) )
    {
        throw -33;
    }
    else if( (c_d_duty < 1e-5) || (c_d_duty > 99.99999) )
    {
        throw -32;
    }
    else
    {
        this->i_mode = c_i_m;
        this->d_dutyCycle = c_d_duty;
        /**
         * disable PWM & start from a clean slate
         */
        *(p_v_u_pwm + PWM_CTL) = 0;
        /**
         * needs some time until the PWM module gets disabled, without the delay the PWM module crashs
         */
        usleep(10);
        if(i_pwm_no == 0)
        {
            /**
             * set the number of counts that constitute a period
             */
            *(p_v_u_pwm + PWM_RNG1) = this->u_i_counts;
            /**
             * set  duty cycle
             */
            *(p_v_u_pwm + PWM_DAT1) = (int) ((this->d_dutyCycle / 100.0) * this->u_i_counts);
            /**
             * start PWM1 in
             */
            if(this->i_mode == PWMMODE) //PWM mode
                *(p_v_u_pwm + PWM_CTL) |= (1 << 0);
            else // M/S Mode
                *(p_v_u_pwm + PWM_CTL) |= ( (1 << 7) | (1 << 0) );

        }
        else
        {
            /**
             * set the number of counts that constitute a period
             */
            *(p_v_u_pwm + PWM_RNG2) = this->u_i_counts;
            /**
             * set  duty cycle
             */
            *(p_v_u_pwm + PWM_DAT2) = (int) ((this->d_dutyCycle/100.0) * this->u_i_counts);
            /**
             * start PWM1 in
             */
            if(this->i_mode == PWMMODE) //PWM mode
                *(p_v_u_pwm + PWM_CTL) |= (1 << 8);
            else // M/S Mode
                *(p_v_u_pwm + PWM_CTL) |= ( (1 << 15) | (1 << 8) );
        }
    }
}

/*
 * This function sets the PWM DutyCycle as a function of PWM resolution,
 * while the PWM peripheral is running. The function performs a check to
 * ensure that the PWM Duty Cycle count value is between 0 and count
 */
void IMPWM::SetDutyCycleCount(const unsigned int &c_u_i_counts, int i_pwm_no)
{
    if( (c_u_i_counts < 0) || ( c_u_i_counts > this->u_i_counts ))
    {
        throw -32;
    }
    else
    {
        this->d_dutyCycle = ((c_u_i_counts * 1.0)/ this->u_i_counts) * 100.0;
        if(i_pwm_no == 0)
            *(p_v_u_pwm + PWM_DAT1) = c_u_i_counts;
        else
            *(p_v_u_pwm + PWM_DAT2) = c_u_i_counts;
    }
}

/*
 * This function sets the PWM mode. The function performs a check to
 * ensure that a valid PWM mode is requested.
 */
void IMPWM::SetMode(const  int &c_i_m)
{
    if( (c_i_m != PWMMODE) && (c_i_m != MSMODE) )
    {
        throw -32;
    }
    else
    {
        try {
            this->i_mode = c_i_m;
            SetDutyCycleForce(this->d_dutyCycle, this->i_mode, 0);
            SetDutyCycleForce(this->d_dutyCycle, this->i_mode, 1);
        } catch(int i) {
            throw i;
        }
    }
}


/*
 * These are a bunch of 'getter' functions that enable the user
 * to access the PWM frequency, resolution, duty cycle and mode as well
 * as the PWM clock divisor value.
 */
double IMPWM::GetFrequency() const {
    return this->d_frequency;
}

int IMPWM::GetCounts() const {
    return this->u_i_counts;
}

int IMPWM::GetDivisor() const {
    return this->u_i_divisor;
}

double IMPWM::GetDutyCycle() const {
    return this->d_dutyCycle;
}

int IMPWM::GetMode() const {
    return this->i_mode;
}

/*
 * This function maps a block (4KB) of physical memory into the memory of
 * the calling process. It enables a user space process to access registers
 * in physical memory directly without having to interact with in kernel side
 * code i.e. device drivers
 */
volatile unsigned *IMPWM::MapRegisterAddres(unsigned long u_l_base_address)
{
    int i_mem_fd = 0;
    void *p_v_register_address_map = MAP_FAILED;

    if (!i_mem_fd)
    {
        if ((i_mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0)
        {
            throw -26;
        }
    }

    /*
     * mmap IO
     */
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
        close(i_mem_fd);
        throw -34;
    }

    if(close(i_mem_fd) < 0)
    {   
        throw -35;
    }

    return (volatile unsigned *)p_v_register_address_map;
}


/*
 * This function is responsible for putting GPIO18 in PWM mode by setting
 * its alternate function to ALT5. It firsts make the GPIO an input
 * (default state) and then switches to alternate function 5 (PWM mode)
 */
void IMPWM::ConfigPWMPin()
{
    /**
     * GPIO18 - ALT5 Config
     */
    *(this->p_v_u_gpio + 1) &= ~(7 << 24);
    *(this->p_v_u_gpio + 1) |= (2 << 24);

    *(this->p_v_u_gpio + 1) &= ~(7 << 6);

    /**
     * GPIO13 - ALT0 Config
     */
    *(this->p_v_u_gpio+1) &= ~(7 << 9);
    *(this->p_v_u_gpio+1) |= (4 << 9);

    *(this->p_v_u_gpio+1) &= ~(7 << 27);
}




/*
 * This function configures the PWM1 peripheral.
 * - It stops the PWM clock
 * - Calculates an appropriate divisor value based on PWM Freq and resolution
 * - Writes this divisor value to the PWM clock
 * - Enables the PWM clock
 * - Disables the PWM peripheral
 * - Writes the PWM resolution and Duty Cycle to the appropriate registers
 * - Enables the PWM peripheral in the requested mode
 */
void IMPWM::ConfigPWM()
{
    double d_period;
    double d_count_duration;

    /**
     * stop clock and waiting for busy flag doesn't work, so kill clock
     */
    *(p_v_u_clk + PWMCLK_CNTL) = 0x5A000000 | (1 << 5);
    usleep(10);

    /**
     * wait until busy flag is set
     */
    while ( (*(p_v_u_clk + PWMCLK_CNTL)) & 0x00000080) {}

    /**
     * calculate divisor value for PWM1 clock...base frequency is 19.2MHz
     */
    d_period = 1.0/this->d_frequency;
    d_count_duration = d_period/(this->u_i_counts*1.0f);
    this->u_i_divisor = (int)(19200000.0f / (1.0/d_count_duration));

    if( this->u_i_divisor < 0 || this->u_i_divisor > 4095 ) {
        throw -36;
    }

    /**
     * set divisor
     */
    *(p_v_u_clk + PWMCLK_DIV) = 0x5A000000 | (this->u_i_divisor << 12);

    /**
     * source=osc and enable clock
     */
    *(p_v_u_clk + PWMCLK_CNTL) = 0x5A000011;

    /**
     * disable PWM & start from a clean slate
     */
    *(p_v_u_pwm + PWM_CTL) = 0;

    /**
     * needs some time until the PWM module gets disabled, without the delay the PWM module crashs
     */
    usleep(10);

    /**
     * set the number of counts that constitute a period with 0 for 20 milliseconds = 320 bits
     */
    *(p_v_u_pwm + PWM_RNG1) = this->u_i_counts;
    *(p_v_u_pwm + PWM_RNG2) = this->u_i_counts;
    usleep(10);

    /**
     * set  duty cycle
     */
    *(p_v_u_pwm + PWM_DAT1) = (int) ((this->d_dutyCycle/100.0) * this->u_i_counts);
    *(p_v_u_pwm + PWM_DAT2) = (int) ((this->d_dutyCycle/100.0) * this->u_i_counts);
    usleep(10);

    /**
     * start PWM1 in
     */
    if(this->i_mode == PWMMODE) //PWM mode
    {
        *(p_v_u_pwm + PWM_CTL) |= (1 << 0) | (1 << 8);
    }
    else // M/S Mode
    {
        *(p_v_u_pwm + PWM_CTL) |= ( (1 << 7) | (1 << 0) ) | ( (1 << 15) | (1 << 8) );
    }
}
