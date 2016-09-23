#include "../include/ErrorCodes.h"

/**
 * Gets error code parameter and returns error description.
 *
 * If no description is defined for input parameter, function returns default error description.
 */
std::string GetErrorDescription(int i_error_code)
{
	std::string str_error_description = "An unhandled exception is occured";

	switch(i_error_code)
	{
		case -1: str_error_description = "[Error Code 1] IMSerial: A failure is occured while setting serial port settings."; break;

		case -2: str_error_description = "[Error Code 2] IMSerial: A failure is occured while reading data."; break;

		case -3: str_error_description = "[Error Code 3] IMSerial: No available data to read"; break;

		case -4: str_error_description = "[Error Code 4] IMSerial: A failure is occured while writing async data"; break;

		case -5: str_error_description = "[Error Code 5] IMSerial: A failure is occured while opening serialport"; break;

		case -6: str_error_description = "[Error Code 6] IMSerial: A failure is occured while closing serialport"; break;

		case -7: str_error_description = "[Error Code 7] IMSerial: A failure is occured while clearing input/output queues."; break;

		case -8: str_error_description = "[Error Code 8] IMSerial: A failure is occured at callback function pointer"; break;

		case -9: str_error_description = "[Error Code 9] IMSerial: A failure is occured while making the device asynchronous"; break;

		case -10: str_error_description = "[Error Code 10] IMSerial: An unhandled exception is occured"; break;

		case -11: str_error_description = "[Error Code 11] IMSPI: An unhandled exception is occured"; break;

		case -12: str_error_description = "[Error Code 12] IMSPI: A failure is occured while opening SPI"; break;

		case -13: str_error_description = "[Error Code 13] IMSPI: ioctl error, SPIMode (WR) cannot be assigned"; break;

		case -14: str_error_description = "[Error Code 14] IMSPI: ioctl error, SPIMode (RD) cannot be assigned"; break;

		case -15: str_error_description = "[Error Code 15] IMSPI: ioctl error, SPI bitsPerWord (WR) cannot be assigned"; break;

		case -16: str_error_description = "[Error Code 16] IMSPI: ioctl error, SPI bitsPerWord(RD) cannot be assigned"; break;

		case -17: str_error_description = "[Error Code 17] IMSPI: ioctl error, SPI speed (WR) cannot be assigned"; break;

		case -18: str_error_description = "[Error Code 18] IMSPI: ioctl error, SPI speed (RD) cannot be assigned"; break;

		case -19: str_error_description = "[Error Code 19] IMSPI: A failure is occured while closing SPI"; break;

		case -20: str_error_description = "[Error Code 20] IMSPI: ioctl error, a failure is occured while transfering SPI data"; break;

		case -21: str_error_description = "[Error Code 21] IMI2C: An unhandled exception is occured"; break;

		case -22: str_error_description = "[Error Code 22] IMI2C: A failure is occured while opening file"; break;

		case -23: str_error_description = "[Error Code 23] IMI2C: A failure is occured while closing file"; break;

		case -24: str_error_description = "[Error Code 24] IMI2C: A failure is occured while writing to I2C device"; break;

		case -25: str_error_description = "[Error Code 25] IMI2C: A failure is occured while reading from I2C device"; break;

		case -130: str_error_description = "[Error Code 130] IMI2C: Could not detect the I2C device."; break;

		case -128: str_error_description = "[Error Code 128] IMI2C: A failure is occured while multiple registers reading from I2C device"; break;

		case -29: str_error_description = "[Error Code 29] IMPWM: An unhandled exception is occured"; break;

		case -30: str_error_description = "[Error Code 30] IMPWM: A failure is occured while setting frequency"; break;

		case -31: str_error_description = "[Error Code 31] IMPWM: A failure is occured while setting counts"; break;

		case -32: str_error_description = "[Error Code 32] IMPWM: A failure is occured while setting duty cycle"; break;

		case -33: str_error_description = "[Error Code 33] IMPWM: A failure is occured while setting mode"; break;

		case -26: str_error_description = "[Error Code 26] IMPWM: A failure is occured while opening /dev/mem"; break;

		case -34: str_error_description = "[Error Code 34] IMPWM: A failure is occured while mapping register address"; break;

		case -35: str_error_description = "[Error Code 35] IMPWM: A failure is occured while closing /dev/mem file descriptor"; break;

		case -36: str_error_description = "[Error Code 36] IMPWM: Divisor value must be between 0-4095"; break;

		case -66: str_error_description = "[Error Code 66] IMPWM: A failure is occured while unmapping the memory block containing PWM registers"; break;

		case -67: str_error_description = "[Error Code 67] IMPWM: A failure is occured while unmapping the memory block containing PWM Clock registers"; break;

		case -68: str_error_description = "[Error Code 68] IMPWM: A failure is occured while unmapping the memory block containing PWM GPIO registers"; break;

		case -37: str_error_description = "[Error Code 37] IMUM6: An unhandled exception is occured"; break;

		case -38: str_error_description = "[Error Code 38] IMUM6: Undefined packet number"; break;

		case -39: str_error_description = "[Error Code 39] IMUM6: A failure occured while reading data"; break;

		case -40: str_error_description = "[Error Code 40] IMUM6: Undefined baudrate"; break;

		case -41: str_error_description = "[Error Code 41] IMUM6: Wrong count of pins"; break;

		case -42: str_error_description = "[Error Code 42] IMUM6: Negative batch"; break;

		case -43: str_error_description = "[Error Code 43] IMUM6: Command failed while processing data"; break;

		case -119: str_error_description = "[Error Code 119] IMUM6: Data is not received"; break;

		case -45: str_error_description = "[Error Code 45] IMGPIO: An unhandled exception is occured"; break;

		case -46: str_error_description = "[Error Code 46] IMGPIO: A failure occured while opening SYSFS GPIO export device"; break;

		case -47: str_error_description = "[Error Code 47] IMGPIO: A failure occured while writing to SYSFS GPIO export device"; break;

		case -48: str_error_description = "[Error Code 48] IMGPIO: A failure occured while closing SYSFS GPIO export device"; break;

		case -49: str_error_description = "[Error Code 49] IMGPIO: A failure occured while opening SYSFS GPIO unexport device"; break;

		case -50: str_error_description = "[Error Code 50] IMGPIO: A failure occured while writing to SYSFS GPIO unexport device"; break;

		case -51: str_error_description = "[Error Code 51] IMGPIO: A failure occured while closing SYSFS GPIO unexport device"; break;

		case -52: str_error_description = "[Error Code 52] IMGPIO: A failure occured while opening SYSFS GPIO direction device"; break;

		case -53: str_error_description = "[Error Code 53] IMGPIO: Invalid pin mode is assigned. It should be INPUT or OUTPUT"; break;

		case -54: str_error_description = "[Error Code 54] IMGPIO: A failure occured while writing to SYSFS GPIO direction device"; break;

		case -55: str_error_description = "[Error Code 55] IMGPIO: A failure occured while closing SYSFS GPIO direction device"; break;

		case -56: str_error_description = "[Error Code 56] IMGPIO: A failure occured while opening SYSFS GPIO value device"; break;

		case -57: str_error_description = "[Error Code 57] IMGPIO: Invalid parameter is assigned. Iy should be HIGH or LOW"; break;

		case -58: str_error_description = "[Error Code 58] IMGPIO: A failure occured while writing to SYSFS GPIO value device"; break;

		case -59: str_error_description = "[Error Code 59] IMGPIO: IMGPIO: A failure occured while closing SYSFS GPIO value device"; break;

		case -60: str_error_description = "[Error Code 60] IMGPIO: A failure occured while reading from SYSFS GPIO value device"; break;

		case -61: str_error_description = "[Error Code 61] IMGPIO: Invalid value has been read. It should be HIGH or LOW"; break;

		case -62: str_error_description = "[Error Code 62] IMEIO: An unhandled exception is occured"; break;

		case -63: str_error_description = "[Error Code 63] IMADC: An unhandled exception is occured"; break;

		case -64: str_error_description = "[Error Code 64] IMADC: Wrong Channel Selection"; break;

		case -44: str_error_description = "[Error Code 44] EvarobotBumper: Unable to create semaphore"; break;

		case -27: str_error_description = "[Error Code 27] EvarobotBumper: Failed to get param 'frequency'"; break;

		case -28: str_error_description = "[Error Code 28] EvarobotController: Failed to get param 'wheelSeparation'"; break;

		case -65: str_error_description = "[Error Code 65] EvarobotController: Failed to get param 'integralConstLeft'"; break;

		case -69: str_error_description = "[Error Code 69] EvarobotController: Failed to get param 'derivativeConstLeft'"; break;

		case -70: str_error_description = "[Error Code 70] EvarobotController: Failed to get param 'proportionalConstLeft'"; break;

		case -71: str_error_description = "[Error Code 71] EvarobotController: Failed to get param 'integralConstRight'"; break;

		case -72: str_error_description = "[Error Code 72] EvarobotController: Failed to get param 'derivativeConstRight'"; break;

		case -73: str_error_description = "[Error Code 73] EvarobotController: Failed to get param 'proportipnalConstRight'"; break;

		case -74: str_error_description = "[Error Code 74] EvarobotController: Failed to get param 'Frequency'"; break;

		case -75: str_error_description = "[Error Code 75] EvarobotController: Odometry Timeout Error (Past)"; break;

		case -76: str_error_description = "[Error Code 76] EvarobotController: Odometry Timeout Error (Future)"; break;

		case -77: str_error_description = "[Error Code 77] EvarobotDriver: Failed to call service evarobot_rgb/SetRGB"; break;

		case -78: str_error_description = "[Error Code 78] EvarobotDriver: HIGH CURRENT SENSE IN LEFT MOTOR"; break;

		case -79: str_error_description = "[Error Code 79] EvarobotDriver: HIGH CURRENT SENSE IN RIGHT MOTOR"; break;

		case -80: str_error_description = "[Error Code 80] EvarobotDriver: Controller Timeout Error (Past)"; break;

		case -81: str_error_description = "[Error Code 81] EvarobotDriver: Controller Timeout Error (Future)"; break;

		case -82: str_error_description = "[Error Code 82] EvarobotDriver: Failed to get param 'maxLinearVel'"; break;

		case -83: str_error_description = "[Error Code 83] EvarobotDriver: Failed to get param 'maxAngularVel"; break;

		case -84: str_error_description = "[Error Code 84] EvarobotDriver: Failed to get param 'wheelSeparation'"; break;

		case -85: str_error_description = "[Error Code 85] EvarobotDriver: Failed to get param 'wheelDiameter'"; break;

		case -86: str_error_description = "[Error Code 86] EvarobotDriver: Failed to get param 'pwmFrequency'"; break;

		case -87: str_error_description = "[Error Code 87] EvarobotDriver: Failed to get param 'pwmDuty'"; break;

		case -88: str_error_description = "[Error Code 88] EvarobotDriver: Failed to get param 'pwmCounts'"; break;

		case -89: str_error_description = "[Error Code 89] EvarobotDriver: Undefined PWM Mode"; break;

		case -90: str_error_description = "[Error Code 90] EvarobotDriver: Wrong spiMode. It should be 0-3"; break;

		case -91: str_error_description = "[Error Code 91] EvarobotDriver: Failed to get param 'pwmMode'"; break;

		case -92: str_error_description = "[Error Code 92] EvarobotRGB: Invalid Color while turning led on"; break;

		case -93: str_error_description = "[Error Code 93] EvarobotRGB: Division by zero"; break;

		case -94: str_error_description = "[Error Code 94] EvarobotRGB: Invalid mode"; break;

		case -95: str_error_description = "[Error Code 95] EvarobotRGB: Invalid frequency"; break;

		case -96: str_error_description = "[Error Code 96] EvarobotRGB: Invalid color"; break;

		case -97: str_error_description = "[Error Code 97] EvarobotRGB: Unable to create semaphore"; break;

		case -98: str_error_description = "[Error Code 98] EvarobotEIO: Unable to create semaphore"; break;

		case -99: str_error_description = "[Error Code 99] EvarobotEIO: Failed to get param 'frequency'"; break;

		case -100: str_error_description = "[Error Code 100] EvarobotInfrared: Failed to get param 'frequency'"; break;

		case -101: str_error_description = "[Error Code 101] EvarobotInfrared: adcBits mustn't be greater than 16"; break;

		case -102: str_error_description = "[Error Code 102] EvarobotInfrared: Number of infrared sensors mustn't be greater than MAX_IR"; break;

		case -103: str_error_description = "[Error Code 103] EvarobotInfrared: Wrong spiMode. It should be 0-3"; break;

		case -104: str_error_description = "[Error Code 104] EvarobotInfrared: Undefined channel value"; break;

		case -105: str_error_description = "[Error Code 105] EvarobotIMU: Unable to create semaphore"; break;

		case -106: str_error_description = "[Error Code 106] EvarobotIMU: Failed to get param 'frequency'"; break;

		case -107: str_error_description = "[Error Code 107] EvarobotIMU: Time went backwards"; break;

		case -108: str_error_description = "[Error Code 108] EvarobotOdometry: Failed to get param 'wheelSeparation'"; break;

		case -109: str_error_description = "[Error Code 109] EvarobotOdometry: Failed to get param 'height'"; break;

		case -110: str_error_description = "[Error Code 110] EvarobotOdometry: Failed to get param 'frequency'"; break;

		case -111: str_error_description = "[Error Code 111] EvarobotOdometry: Failed to get param 'gearRatio'"; break;

		case -112: str_error_description = "[Error Code 112] EvarobotOdometry: Failed to get param 'CPR'"; break;

		case -113: str_error_description = "[Error Code 113] EvarobotOdometry: Failed to get param 'wheelDiameter'"; break;

		case -114: str_error_description = "[Error Code 114] EvarobotOdometry: File either does not exist or has been locked by another process"; break;

		case -115: str_error_description = "[Error Code 115] EvarobotOdometry: Division by Zero"; break;

		case -116: str_error_description = "[Error Code 116] EvarobotOrientation: Failed to get param 'frequency'"; break;

		case -117: str_error_description = "[Error Code 117] EvarobotOrientation: Wrong Parity Mode. It should be 0-2"; break;

		case -118: str_error_description = "[Error Code 118] EvarobotOrientation: Checksum error in serial communication"; break;

		case -120: str_error_description = "[Error Code 120] EvarobotSonar: Number of sonar devices mustn't be greater than MAX_SONAR"; break;

		case -121: str_error_description = "[Error Code 121] EvarobotSonar: File either does not exist or has been locked by another process"; break;

		case -122: str_error_description = "[Error Code 122] EvarobotSonar: Undefined pin value"; break;

		case -123: str_error_description = "[Error Code 123] EvarobotTeleop: Failed to call service evarobot_rgb/SetRGB"; break;

		case -124: str_error_description = "[Error Code 124] EvarobotTeleop: Timeout Error"; break;

		case -125: str_error_description = "[Error Code 125] EvarobotBattery: Unable to create semaphore"; break;

		case -126: str_error_description = "[Error Code 126] EvarobotBattery: High Voltage Threshold is exceeded"; break;

		case -127: str_error_description = "[Error Code 127] EvarobotBattery: Low Voltage"; break;

		case -129: str_error_description = "[Error Code 129] EvarobotBattery: LOW Battery Level."; break;

		default: break;
	}

	return str_error_description;
}
