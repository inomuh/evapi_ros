#include "../include/IMGPIO.h"

using namespace std;

const string IMGPIO::RISING = "rising";
const string IMGPIO::FALLING = "falling";
const string IMGPIO::BOTH = "both";

const string IMGPIO::INPUT = "in";
const string IMGPIO::OUTPUT = "out";

const string IMGPIO::HIGH = "1";
const string IMGPIO::LOW = "0";

const string IMGPIO::GPIO0 = "23";
const string IMGPIO::GPIO1 = "24";
const string IMGPIO::GPIO2 = "25";
const string IMGPIO::GPIO3 = "7";
const string IMGPIO::GPIO4 = "1";
const string IMGPIO::GPIO5 = "16";
const string IMGPIO::GPIO6 = "20";
const string IMGPIO::GPIO7 = "21";
const string IMGPIO::GPIO8 = "26";
const string IMGPIO::GPIO9 = "6";
const string IMGPIO::GPIO10 = "5";
const string IMGPIO::GPIO11 = "0";
const string IMGPIO::GPIO12 = "12";
const string IMGPIO::GPIO13 = "19";

/**
 * IMGPIO constructor with default parameters.
 * Initializes required variables.
 * Exports GPIO.
 */
IMGPIO::IMGPIO():i_value_fd(-1),i_direction_fd(-1),i_export_fd(-1),
i_unexport_fd(-1),i_edge_fd(-1),str_pin_number("23")
{
	try{
		this->ExportGpio();
	}catch(int i){
		throw i;
	}
}

/**
 * IMGPIO constructor with pin number parameter. Initializes required variables. Exports GPIO.
 */
IMGPIO::IMGPIO(string _str_pin_number):i_value_fd(-1),
i_direction_fd(-1),i_export_fd(-1),i_unexport_fd(-1),i_edge_fd(-1),
str_pin_number(_str_pin_number)
{
	try{
		this->ExportGpio();
	}catch(int i){
		throw i;
	}
}

/**
 * Destructor which unexports GPIO.
 */
IMGPIO::~IMGPIO()
{
	try{
		this->UnexportGpio();
	}catch(int i){
		throw i;
	}
}

/**
 * Exports GPIO pin
 */
int IMGPIO::ExportGpio()
{
	int i_status_value = -1;
	string str_export = "/sys/class/gpio/export";

	/**
	 * File descriptor to export GPIO pin.
	 */
	this->i_export_fd = i_status_value = open(str_export.c_str(),
												O_WRONLY|O_SYNC);
	if (i_status_value < 0)
	{
        throw -46;
	}

	/**
	 * Writes pin number to export GPIO pin.
	 */
	i_status_value = write(this->i_export_fd, this->str_pin_number.c_str(),
								this->str_pin_number.length());
	if (i_status_value < 0)
	{
        throw -47;
	}

	/**
	 * Closes file.
	 */
	i_status_value = close(this->i_export_fd);

	if (i_status_value < 0)
	{
        throw -48;
	}

    return i_status_value;
}

/**
 * Unexports GPIO pin
 */
int IMGPIO::UnexportGpio()
{
	int i_status_value = -1;
	string str_unexport = "/sys/class/gpio/unexport";

	/**
	 * File descriptor to unexport GPIO pin.
	 */
	this->i_export_fd = i_status_value = open(str_unexport.c_str(),
												O_WRONLY|O_SYNC);

	if (i_status_value < 0)
	{
        throw -49;
	}

	/**
	 * Writes pin number to unexport GPIO pin.
	 */
	i_status_value = write(this->i_export_fd, this->str_pin_number.c_str(),
						this->str_pin_number.length());

	if (i_status_value < 0)
	{
        throw -50;
	}

	/**
	 * Closes file.
	 */
	i_status_value = close(this->i_export_fd);

	if (i_status_value < 0)
	{
        throw -51;
	}

	return i_status_value;
}

/**
  * Sets pin direction as IMGPIO::INPUT or IMGPIO::OUTPUT
  */
int IMGPIO::SetPinDirection(string str_direction)
{
	int i_status_value = -1;
	string str_setdir ="/sys/class/gpio/gpio" + this->str_pin_number + "/direction";

	/**
	 * File descriptor to to set GPIO pin direction.
	 */
	this->i_direction_fd = i_status_value = open(str_setdir.c_str(),
												O_WRONLY|O_SYNC);
	if (i_status_value < 0)
	{
		throw -52;
	}

	if (str_direction.compare(INPUT) != 0 && str_direction.compare(OUTPUT) != 0 )
	{
		throw -53;
	}

	/**
	 * Writes direction to set GPIO pin direction.
	 */
	i_status_value = write(this->i_direction_fd, str_direction.c_str(),
							str_direction.length());

	if (i_status_value < 0)
	{
        throw -54;
	}

	/**
	 * Closes file.
	 */
	i_status_value = close(this->i_direction_fd);

	if (i_status_value < 0)
	{
        throw -55;
	}

	    return i_status_value;
}

/**
 * Sets pin value
 */
int IMGPIO::SetPinValue(string str_value)
{

    int i_status_value = -1;
	string str_set_value = "/sys/class/gpio/gpio" + this->str_pin_number + "/value";

	/**
	 * File descriptor to to set GPIO pin value.
	 */
	this->i_value_fd = i_status_value = open(str_set_value.c_str(),
												O_WRONLY|O_SYNC);

	if (i_status_value < 0)
	{
        throw -56;
	}

	if (str_value.compare(HIGH) != 0 && str_value.compare(LOW) != 0 )
	{
		throw -57;
	}

	/**
	 * Writes value to set GPIO pin.
	 */
	i_status_value = write(this->i_value_fd, str_value.c_str(),
							str_value.length());

	if (i_status_value < 0)
	{
        throw -58;
	}

	/**
	 * Closes file.
	 */
	i_status_value = close(this->i_value_fd);

	if (i_status_value < 0)
	{
        throw -59;
	}
	return i_status_value;
}

/**
 * Gets pin value
 */
int IMGPIO::GetPinValue(string & str_value)
{
	string str_get_value = "/sys/class/gpio/gpio" + this->str_pin_number + "/value";
	char c_buff[2];
	int i_status_value = -1;

	/**
	 * File descriptor to to get GPIO pin value.
	 */
	this->i_value_fd = i_status_value = open(str_get_value.c_str(),
											O_RDONLY|O_SYNC);

	if (i_status_value < 0)
	{
        throw -56;
	}

	/**
	 * Reads value from GPIO pin.
	 */
	i_status_value = read(this->i_value_fd, &c_buff, 1);

	if (i_status_value < 0)
	{
        throw -60;
	}

	c_buff[1]='\0';

	str_value = string(c_buff);

	/**
	 * Controls value is valid or not.
	 */
	if (str_value.compare(HIGH) != 0 && str_value.compare(LOW) != 0 ) 
	{
		throw -61;
	}

	/**
	 * Closes file.
	 */
	i_status_value = close(this->i_value_fd);

	if (i_status_value < 0)
	{
        throw -59;
	}

	return i_status_value;
}

/**
 * Returns pin number which set in Constructor.
 */
string IMGPIO::GetPinNumber() const
{
	return this->str_pin_number;
}