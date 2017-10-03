#ifdef English_dox
/**
 * \file   IMI2C.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief
 * \details
 *
 */
#endif
#ifdef Turkish_dox
/**
 * \file   IMI2C.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief
 * \details
 *
 */
#endif

#ifndef INCLUDE_IMUM6_H_
#define INCLUDE_IMUM6_H_

#include "IMSerial.h"
#include <sstream>
#include <iostream>
#include <list>
#include <vector>

class IMUM6
{
public:

	struct PACKET_TYPE
	{
		int i_packet_has_data;
		int i_packet_is_batch;
		int i_batch_length;
		int i_command_failed;
		int i_size_of_data;
	};

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
	 * \param
	 * \param
	 */
	#endif

	#ifdef Turkish_dox
	//! Constructor
	/**
	 * \param
	 * \param
	 */
	#endif
	IMUM6(IMSerial * p_imserial);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	bool isReceivedData(char * p_c_start_bits, int i_length);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	bool isCommandCompleted() const;

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	bool CheckData();

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	void GetFWVersion();

	// UM6_COMMUNICATION Register (0x00) Functions

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetBaudRate(long l_baudrate);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetFrequency(int i_frequency);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetGPSBaudrate(long l_baudrate);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetDetailedSatelliteStatusTrasmission(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetSatelliteSummaryTrasmission(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetGPSCourseVelocityTrasmission(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetGPSRelativePositionTrasmission(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetGPSPositionTransmission(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetTemperatureMeasurementTransmission(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetCovarianceMatrixTransmission(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetEulerAngleTransmission(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetQuaternionTransmission(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetProcessedMagnetometerTransmission(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetProcessedAccelerometerTransmission(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetProcessedGyroTransmission(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetRawMagnetometerTransmission(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetRawAccelerometerTransmission(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetRawGyroTransmission(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetMode(bool b_mode); // broadcast mode: true, listen mode: false


	// UM6_MISC_CONFIG Register (0x01) Functions

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetPPSTiming(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetQuaterninonState(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetGyroCalibration(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetEKFAccelerometerUpdates(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetEKFMagnetometerUpdates(bool b_status);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	bool GetRawData(IMUM6::UM6_DATA & data);

//	void PrintLogData();

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int ProcessData(vector<int> & T_i_registers);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int32_t GetDataRegister(int i_register) const;

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetCommand(int i_register);

	struct REGISTERS
	{
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

	union UNION32
	{
		float f;
		int32_t i;
		unsigned char c[4];

	};

private:

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	bool ParseData();

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	void StoreData();

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	void SetRegisterPinValue(
			int32_t & i_register,
			int i_pin_no,
			int i_count_of_pins,
			char c_value
			);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetRegister32bitValue(int i_register, float f_data);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetRegister32bitValue(int i_register, int32_t i_data);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetRegister16bitValue(int i_register, int16_t i_data);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int SetRegister16bitValue(int i_register, int16_t * p_i_data);

	#ifdef English_dox
	//! Brief
	/**
	 * Detailed
	 * \param
	 * \return
	 */
	#endif
	#ifdef Turkish_dox
	//! Kısa bilgi
	/**
	 * Detaylı bilgi
	 * \param i_channel_no
	 * \return
	 */
	#endif
	int CalculateChecksum();

	int32_t i_data_registers[85]; // 0x55 - 0xA9
	int32_t i_communication_register;
	int32_t i_misc_config_register;
	char c_input_buffer[500];
	char c_start_bits[3];
	bool b_command_complete;


	list<char> T_c_stored_data;
	list<char> T_c_serial_log;

	bool b_incomplete_packet;

	int i_incomplete_no;
	int i_start_bit_counter;
	int i_buffer_read_index;

	IMSerial * p_imserial;
};



#endif /* INCLUDE_IMIMU_UM6_H_ */
