#ifdef English_dox
/**
 * \file   IMUM6.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Performs IMU operations like transformation, communication etc.
 * \details
 */
#endif

#ifndef INCLUDE_IMUM6_H_
#define INCLUDE_IMUM6_H_

#include "IMSerial.h"
#include <sstream>
#include <iostream>
#include <list>
#include <vector>

#ifdef English_dox
//! Performs IMU operations like transformation, communication etc.
#endif
class IMUM6
{
public:

	#ifdef English_dox
	//! Information about packet type.
	#endif
	struct PACKET_TYPE
	{
		int i_packet_has_data;
		int i_packet_is_batch;
		int i_batch_length;
		int i_command_failed;
		int i_size_of_data;
	};

	#ifdef English_dox
	//! Information about raw data.
	#endif
	struct UM6_DATA
	{
		PACKET_TYPE packet_type;
		char c_PT;
		char c_adress;
		char c_databytes[64];
		char c_checksum[2];
	}um6_data;

	#ifdef English_dox
	//! Constructor
	/**
	 * \param *p_imserial pointer of IMSerial class.
	 */
	#endif
	IMUM6(IMSerial * p_imserial);

	#ifdef English_dox
	//! Controls data is received or not.
	/**
	 * \param *p_c_start_bits Control start from this bit.
	 * \param i_length Control from start to start+length
	 * \return Data is received or not.
	 */
	#endif
	bool isReceivedData(char * p_c_start_bits, int i_length);

	#ifdef English_dox
	//! Returns command is completed or not.
	/**
	 * \return Command is completed or not.
	 */
	#endif
	bool isCommandCompleted() const;

	#ifdef English_dox
	//! Checks if the checksums are correct.
	/**
	 * \return checksums are correct or not.
	 */
	#endif
	bool CheckData();

	#ifdef English_dox
	//! Used to set sensor serial port baud rate.
	/**
	 * Used to set sensor serial port baud rate. The three configuration bits
	 * set the desired baud rate as shown below:
	 * 000 -> 9600 Baud
	 * 001 -> 14400 Baud
	 * 010 -> 19200 Baud
	 * 011 -> 38400 Baud
	 * 100 -> 57600 Baud
	 * 101 -> 115200 Baud
	 * 110 -> NA
	 * 111 -> NA
	 * Note that if the desired baud rate is changed, the new configuration
	 * must be written to FLASH in order for the new baud rate to persist
	 * when power is cycled. A write to FLASH must be triggered by
	 * sending a WRITE_TO_FLASH command with the currently selected
	 * baud rate. This ensures that the baud rate is never mistakenly
	 * changed permanently.
	 * \param l_baudrate Baudrate value
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetBaudRate(long l_baudrate);

	#ifdef English_dox
	//! Used to set sensor serial port frequency.
	/**
	 * When in broadcast mode, these bits specify how often a data
	 * packets are automatically transmitted over the serial port. The actual
	 * broadcast frequency is given by
	 * freq = (280/255)*broadcast_rate + 20
	 * \param i_frequency Frequency value
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetFrequency(int i_frequency);

	#ifdef English_dox
	//! Used to set the baud rate used by the connected GPS receiver (if used).
	/**
	 * Used to set the baud rate used by the connected GPS receiver (if
	 * used). The three configuration bits set the desired baud rate as
	 * shown below:
	 * 000 -> 9600 Baud
	 * 001 -> 14400 Baud
	 * 010 -> 19200 Baud
	 * 011 -> 38400 Baud
	 * 100 -> 57600 Baud
	 * 101 -> 115200 Baud
	 * 110 -> NA
	 * 111 -> NA
	 * Note that if the desired baud rate is changed, the new configuration
	 * must be written to FLASH in order for the new baud rate to persist
	 * when power is cycled. A write to FLASH must be triggered by
	 * sending a WRITE_TO_FLASH command with the currently selected
	 * baud rate. This ensures that the baud rate is never mistakenly
	 * changed permanently.
	 * \param l_baudrate Baudrate value
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetGPSBaudrate(long l_baudrate);

	#ifdef English_dox
	//! Detailed satellite status information enabled. This is available in firmware revisions UM2B and later.
	/**
	 * \param b_status if b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetDetailedSatelliteStatusTrasmission(bool b_status);

	#ifdef English_dox
	//! Satellite summary transmission enabled. This is available in firmware revisions UM2B and later.
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetSatelliteSummaryTrasmission(bool b_status);

	#ifdef English_dox
	//! GPS course and velocity transmission enabled. This is available in firmware revisions UM2B and later.
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetGPSCourseVelocityTrasmission(bool b_status);

	#ifdef English_dox
	//! GPS relative position transmission enabled. This is available in firmware revisions UM2B and later.
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetGPSRelativePositionTrasmission(bool b_status);

	#ifdef English_dox
	//! GPS position transmission enabled. This is available in firmware revisions UM2B and later.
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetGPSPositionTransmission(bool b_status);

	#ifdef English_dox
	//! Temperature measurement transmission enabled. This is only available in firmware revision UM2A and later.
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetTemperatureMeasurementTransmission(bool b_status);

	#ifdef English_dox
	//! Covariance matrix transmission enabled.
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetCovarianceMatrixTransmission(bool b_status);

	#ifdef English_dox
	//! Euler angle transmission enabled.
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetEulerAngleTransmission(bool b_status);

	#ifdef English_dox
	//! Quaternion transmission enabled (only outputs valid data if quaternion estimation is enabled in the MISC_CONFIG register)
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetQuaternionTransmission(bool b_status);

	#ifdef English_dox
	//! Processed magnetometer data transmission enabled.
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetProcessedMagnetometerTransmission(bool b_status);

	#ifdef English_dox
	//! Processed accelerometer data transmission enabled.
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetProcessedAccelerometerTransmission(bool b_status);

	#ifdef English_dox
	//! Processed gyro data transmission enabled.
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetProcessedGyroTransmission(bool b_status);

	#ifdef English_dox
	//! Raw magnetometer data transmission enabled.
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetRawMagnetometerTransmission(bool b_status);

	#ifdef English_dox
	//! Raw accelerometer data transmission enabled.
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetRawAccelerometerTransmission(bool b_status);

	#ifdef English_dox
	//! Raw gyro data transmission enabled.
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetRawGyroTransmission(bool b_status);

	#ifdef English_dox
	//! Sets broadcast mode or listen mode.
	/**
	 * \param b_mode true for broadcast mode, false for listen mode
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetMode(bool b_mode);


	// UM6_MISC_CONFIG Register (0x01) Functions

	#ifdef English_dox
	//! Specifies whether PPS timing is enabled. This will be implemented in future firmware revisions.
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetPPSTiming(bool b_status);

	#ifdef English_dox
	//! Specifies whether quaternion state estimation is enabled.
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetQuaterninonState(bool b_status);

	#ifdef English_dox
	//! Enables startup gyro calibration. If this bit is set, then gyros will be calibrated automatically on sensor startup.
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetGyroCalibration(bool b_status);

	#ifdef English_dox
	//! EKF accelerometer updates enabled (pitch and roll angle correction).
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetEKFAccelerometerUpdates(bool b_status);

	#ifdef English_dox
	//! EKF magnetometer updates enabled (yaw angle correction).
	/**
	 * \param b_status If b_status is true, then detailed satellite status information is enabled.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetEKFMagnetometerUpdates(bool b_status);

	#ifdef English_dox
	//! Gets raw data.
	/**
	 * \param &data Data is loaded to this variable.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	bool GetRawData(IMUM6::UM6_DATA & data);

	#ifdef English_dox
	//! Gets received data to a vector.
	/**
	 * \param &T_i_registers Data stored on this vector.
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int ProcessData(vector<int> & T_i_registers);

	#ifdef English_dox
	//! Returns data at specified register.
	/**
	 * \param i_register Register address
	 * \return Data at specified register.
	 */
	#endif
	int32_t GetDataRegister(int i_register) const;

	#ifdef English_dox
	//! Sets specified register.
	/**
	 * \param i_register Register address
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetCommand(int i_register);

	#ifdef English_dox
		//! Struct to define a register.
	#endif
	struct REGISTERS
	{
		#ifdef English_dox
			//! Configuration registers in Polulu MinIMU9
		#endif
		enum CONFIGURATION_REGISTERS
			{
				UM6_COMMUNICATION, // 0x00
				UM6_MISC_CONFIG, // 0x01
				UM6_MAG_REF_X, // 0x02
				UM6_MAG_REF_Y, // 0x03
				UM6_MAG_REF_Z, // 0x04
				UM6_ACCEL_REF_X, // 0x05
				UM6_ACCEL_REF_Y, // 0x06
				UM6_ACCEL_REF_Z, // 0x07
				UM6_EKF_MAG_VARIANCE, // 0x08
				UM6_EKF_ACCEL_VARIANCE, // 0x09
				UM6_EKF_PROCESS_VARIANCE, // 0x0A
				UM6_GYRO_BIAS_XY, // 0x0B
				UM6_GYRO_BIAS_Z, // 0x0C
				UM6_ACCEL_BIAS_XY, // 0x0D
				UM6_ACCEL_BIAS_Z, // 0x0E
				UM6_MAG_BIAS_XY, // 0x0F
				UM6_MAG_BIAS_Z, // 0x10
				UM6_ACCEL_CAL_00, // 0x11
				UM6_ACCEL_CAL_01, // 0x12
				UM6_ACCEL_CAL_02, // 0x13
				UM6_ACCEL_CAL_10, // 0x14
				UM6_ACCEL_CAL_11, // 0x15
				UM6_ACCEL_CAL_12, // 0x16
				UM6_ACCEL_CAL_20, // 0x17
				UM6_ACCEL_CAL_21, // 0x18
				UM6_ACCEL_CAL_22, // 0x19
				UM6_GYRO_CAL_00, // 0x1A
				UM6_GYRO_CAL_01, // 0x1B
				UM6_GYRO_CAL_02, // 0x1C
				UM6_GYRO_CAL_10, // 0x1D
				UM6_GYRO_CAL_11, // 0x1E
				UM6_GYRO_CAL_12, // 0x1F
				UM6_GYRO_CAL_20, // 0x20
				UM6_GYRO_CAL_21, // 0x21
				UM6_GYRO_CAL_22, // 0x22
				UM6_GYRO_CAL_23, // 0x23
				UM6_MAG_CAL_01, // 0x24
				UM6_MAG_CAL_02, // 0x25
				UM6_MAG_CAL_10, // 0x26
				UM6_MAG_CAL_11, // 0x27
				UM6_MAG_CAL_12, // 0x28
				UM6_MAG_CAL_20, // 0x29
				UM6_MAG_CAL_21, // 0x2A
				UM6_MAG_CAL_22, // 0x2B
				UM6_GYROX_BIAS_0, // 0x2C
				UM6_GYROX_BIAS_1, // 0x2D
				UM6_GYROX_BIAS_2, // 0x2E
				UM6_GYROX_BIAS_3, // 0x2F
				UM6_GYROY_BIAS_0, // 0x30
				UM6_GYROY_BIAS_1, // 0x31
				UM6_GYROY_BIAS_2, // 0x32
				UM6_GYROY_BIAS_3, // 0x33
				UM6_GYROZ_BIAS_0, // 0x34
				UM6_GYROZ_BIAS_1, // 0x35
				UM6_GYROZ_BIAS_2, // 0x36
				UM6_GYROZ_BIAS_3, // 0x37
				UM6_GPS_HOME_LAT, // 0x38
				UM6_GPS_HOME_LON, // 0x39
				UM6_GPS_HOME_ALTITUDE = 0x40, // 0x40
			};

		#ifdef English_dox
			//! Data registers in Polulu MinIMU9
		#endif
		enum DATA_REGISTERS
		{
			UM6_STATUS = 0x55, // 0x55
			UM6_GYRO_RAW_XY, // 0x56
			UM6_GYRO_RAW_Z, // 0x57
			UM6_ACCEL_RAW_XY, // 0x58
			UM6_ACCEL_RAW_Z, // 0x59
			UM6_MAG_RAW_XY, // 0x5A
			UM6_MAG_RAW_Z, // 0x5B
			UM6_GYRO_PROC_XY, // 0x5C
			UM6_GYRO_PROC_Z, // 0x5D
			UM6_ACCEL_PROC_XY, // 0x5E
			UM6_ACCEL_PROC_Z, // 0x5F
			UM6_MAG_PROC_XY, // 0x60
			UM6_MAG_PROC_Z, // 0x61
			UM6_EULER_PHI_THETA, // 0x62
			UM6_EULER_PSI, // 0x63
			UM6_QUAT_AB, // 0x64
			UM6_QUAT_CD, // 0x65
			UM6_ERROR_COV_00, // 0x66
			UM6_ERROR_COV_01, // 0x67
			UM6_ERROR_COV_02, // 0x68
			UM6_ERROR_COV_03, // 0x69
			UM6_ERROR_COV_10, // 0x6A
			UM6_ERROR_COV_11, // 0x6B
			UM6_ERROR_COV_12, // 0x6C
			UM6_ERROR_COV_13, // 0x6D
			UM6_ERROR_COV_20, // 0x6E
			UM6_ERROR_COV_21, // 0x6F
			UM6_ERROR_COV_22, // 0x70
			UM6_ERROR_COV_23, // 0x71
			UM6_ERROR_COV_30, // 0x72
			UM6_ERROR_COV_31, // 0x73
			UM6_ERROR_COV_32, // 0x74
			UM6_ERROR_COV_33, // 0x75
			UM6_TEMPERATURE, // 0x76
			UM6_GPS_LONGITUDE, // 0x77
			UM6_GPS_LATITUDE, // 0x78
			UM6_GPS_ALTITUDE, // 0x79
			UM6_GPS_POSITION_N, // 0x7A
			UM6_GPS_POSITION_E, // 0x7B
			UM6_GPS_POSITION_H, // 0x7C
			UM6_GPS_COURSE_SPEED, // 0x7D
			UM6_GPS_SAT_SUMMARY, // 0x7E
			UM6_GPS_SAT_1_2, // 0x7F
			UM6_GPS_SAT_3_4, // 0x80
			UM6_GPS_SAT_5_6, // 0x81
			UM6_GPS_SAT_7_8, // 0x82
			UM6_GPS_SAT_9_10, // 0x83
			UM6_GPS_SAT_11_12 // 0x84
		};

		#ifdef English_dox
			//! Command registers in Polulu MinIMU9
		#endif
		enum COMMAND_REGISTERS
		{
			UM6_GET_FW_VERSION = 0xAA,
			UM6_FLASH_COMMIT,
			UM6_ZERO_GYROS,
			UM6_RESET_EKF,
			UM6_GET_DATA,
			UM6_SET_ACCEL_REF,
			UM6_SET_MAG_REF,
			UM6_RESET_TO_FACTORY,
			UM6_SET_HOME_POSITION = 0xB3,
			UM6_BAD_CHECKSUM,
			UM6_UNKNOWN_ADDRESS,
			UM6_INVALID_BATCH_SIZE
		};

	};

	#ifdef English_dox
		//! Union to store data
	#endif
	union UNION32
	{
		float f;
		int32_t i;
		unsigned char c[4];
	};

private:

	#ifdef English_dox
	//! Parses data and writes it to T_c_stored_data
	/**
	 * \return Packet is complete or not.
	 */
	#endif
	bool ParseData();

	#ifdef English_dox
	//! Stores data from serial port to T_c_stored_data
	#endif
	void StoreData();

	#ifdef English_dox
	//! Sets values of register pins.
	/**
	 * \param &i_register Register number
	 * \param i_pin_no Pin number
	 * \param i_count_of_pins Number of pins
	 * \param c_value Value to store
	 */
	#endif
	void SetRegisterPinValue(
			int32_t & i_register,
			int i_pin_no,
			int i_count_of_pins,
			char c_value
			);

	#ifdef English_dox
	//! Sets register value.
	/**
	 * \param i_register Register number
	 * \param f_data Value to store
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetRegister32bitValue(int i_register, float f_data);

	#ifdef English_dox
	//! Sets register value.
	/**
	 * \param i_register Register number
	 * \param i_data Value to store
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetRegister32bitValue(int i_register, int32_t i_data);

	#ifdef English_dox
	//! Sets register value.
	/**
	 * \param i_register Register number
	 * \param i_data Value to store
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetRegister16bitValue(int i_register, int16_t i_data);

	#ifdef English_dox
	//! Sets register value.
	/**
	 * \param i_register Register number
	 * \param *p_i_data Value to store
	 * \retval <0 if there is an error
	 * \retval >=0 if everthing is OK
	 */
	#endif
	int SetRegister16bitValue(int i_register, int16_t * p_i_data);

	#ifdef English_dox
	//! Data registers. 0x55 - 0xA9
	#endif
	int32_t i_data_registers[85];
	#ifdef English_dox
	//! Communication registers.
	#endif
	int32_t i_communication_register;
	#ifdef English_dox
	//! Misc config registers.
	#endif
	int32_t i_misc_config_register;
	#ifdef English_dox
	//! Stores input data.
	#endif
	char c_input_buffer[500];
	#ifdef English_dox
	//! Start bits consists of three characters.
	#endif
	char c_start_bits[3];
	#ifdef English_dox
	//! Holds command completed or not.
	#endif
	bool b_command_complete;

	#ifdef English_dox
	//! Stores data which comes from sensor.
	#endif
	list<char> T_c_stored_data;
	#ifdef English_dox
	//! Stores logs.
	#endif
	list<char> T_c_serial_log;

	#ifdef English_dox
	//! Holds packet completed or not.
	#endif
	bool b_incomplete_packet;

	#ifdef English_dox
	//! Holds number of uncompleted packets.
	#endif
	int i_incomplete_no;
	#ifdef English_dox
	//! Number of start bits.
	#endif
	int i_start_bit_counter;

	#ifdef English_dox
	//! Class IMSerial is used for communicating with serial port.
	#endif
	IMSerial * p_imserial;
};



#endif /* INCLUDE_IMIMU_UM6_H_ */
