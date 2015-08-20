/*
 * IMIMU_UM6.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: makcakoca
 */

#include "../include/IMUM6.h"

IMUM6::IMUM6(IMSerial * p_imserial)
{
	this->p_imserial = p_imserial;
	this->i_buffer_read_index = 0;

	this->c_start_bits[0] = 's';
	this->c_start_bits[1] = 'n';
	this->c_start_bits[2] = 'p';

	this->b_incomplete_packet = false;
	this->i_incomplete_no = 0;
	this->i_start_bit_counter = 0;
	this->b_command_complete = false;
	this->i_communication_register = 0x7F902D09;
	this->i_misc_config_register   = 0xF0000000;
}

bool IMUM6::ParseData()
{
	bool b_return = false;

	bool b_received_data = false;

	this->StoreData();

	if(!this->b_incomplete_packet)
	{
		b_received_data = this->isReceivedData(this->c_start_bits, sizeof(this->c_start_bits));
		if(!b_received_data)
		{
			return b_return;
		}

		this->i_incomplete_no = 0;
	}


	if(this->b_incomplete_packet || b_received_data)
	{
		int i_m = 0;

		for(list<char>::iterator it = this->T_c_stored_data.begin(); it != this->T_c_stored_data.end(); it++)
		{
			++this->i_incomplete_no;
			++i_m;
			this->b_incomplete_packet = true;

			if(this->i_incomplete_no == 1)
			{
				// Packet Type

				this->um6_data.c_PT = (*it);

				this->um6_data.packet_type.i_packet_has_data = (this->um6_data.c_PT >> 7) & 0x01;
				this->um6_data.packet_type.i_packet_is_batch = (this->um6_data.c_PT >> 6) & 0x01;
				this->um6_data.packet_type.i_batch_length = (this->um6_data.c_PT >> 2) & 0x0F;
				this->um6_data.packet_type.i_command_failed = this->um6_data.c_PT & 0x01;



				if(this->um6_data.packet_type.i_packet_has_data == 0x00)
				{
					// No Data
					this->um6_data.packet_type.i_size_of_data = 0;
				}
				else if(this->um6_data.packet_type.i_packet_has_data == 0x01
						&& this->um6_data.packet_type.i_packet_is_batch == 0x00)
				{
					// only 4 bytes data
					this->um6_data.packet_type.i_size_of_data = 4;

				}
				else if(this->um6_data.packet_type.i_packet_has_data == 0x01
						&& this->um6_data.packet_type.i_packet_is_batch == 0x01)
				{
					// (4 * batch_length) bytes data
					this->um6_data.packet_type.i_size_of_data = 4 * this->um6_data.packet_type.i_batch_length;

				}

			}
			else if(this->i_incomplete_no == 2)
			{
				// Address
				this->um6_data.c_adress = (*it);

			}
			else if(this->i_incomplete_no > 2
					&& this->i_incomplete_no <= 2 + this->um6_data.packet_type.i_size_of_data
					&& this->um6_data.packet_type.i_packet_has_data != 0x00)
			{
				// Data Bytes
				this->um6_data.c_databytes[this->i_incomplete_no - 3] = (*it);

			}
			else if(this->i_incomplete_no == 3 + this->um6_data.packet_type.i_size_of_data)
			{
				// Checksum 0
				this->um6_data.c_checksum[0] = *(it);
			}
			else if(this->i_incomplete_no == 4 + this->um6_data.packet_type.i_size_of_data)
			{
				// Checksum 1
				this->um6_data.c_checksum[1] = *(it);
				this->b_incomplete_packet = false;
				//this->i_incomplete_no = 0;
				break;
			}
			else
			{
				printf("IMIMU_UM6: Undefined Packet Number. Number: %d\n", this->i_incomplete_no);
				this->b_incomplete_packet = false;
				//this->i_incomplete_no = 0;
				break;
			}
		}


		//printf("parse_dataCleaning: ");

		for(int i = 0; i < i_m; i++)
		{
			//printf("%x_", this->T_c_stored_data.front());
			this->T_c_stored_data.pop_front();
		}
		//printf("\n");
	}

//	printf("StatusNumber: %d\n", this->i_incomplete_no);
	b_return = !this->b_incomplete_packet;

	return b_return;
}

bool IMUM6::GetRawData(IMUM6::UM6_DATA & data)
{

	bool b_status = this->ParseData();

	if(b_status)
	{
		data = this->um6_data;
	}

	return b_status;
}

bool IMUM6::isReceivedData(char * p_c_start_bits, int i_length)
{
	bool b_received_data = false;

	int i_k = 0;

	for(list<char>::iterator it = this->T_c_stored_data.begin(); it != this->T_c_stored_data.end(); it++)
	{
		++i_k;
		if((*it) == p_c_start_bits[this->i_start_bit_counter])
		{
			++this->i_start_bit_counter;

			if(this->i_start_bit_counter == i_length)
			{
				b_received_data = true;
				this->i_start_bit_counter = 0;
				break;
			}

		}
		else
		{
			this->i_start_bit_counter = 0;
		}

	}

//	if(b_received_data)
//		printf("received_dataCleaning: %d  ", i_k);

	for(int i = 0; i < i_k; i++)
	{
//		if(b_received_data)
//			printf("%x_", this->T_c_stored_data.front());
		this->T_c_stored_data.pop_front();

	}

//	if(b_received_data)
//		printf("\n");

	return b_received_data;
}

void IMUM6::StoreData()
{
	if(this->p_imserial->isReadyRead())
	{
		char c_data[500];
		int i_size = this->p_imserial->ReadData(c_data, sizeof(c_data));

		if(i_size < 0)
		{
			perror("IMIMU_UM6: A failure occured when reading data.");
			exit(1);
		}

		for(int i = 0; i < i_size; i++)
		{
			//printf("%x_", c_data[i]);
			this->T_c_stored_data.push_back(c_data[i]);
			//this->T_c_serial_log.push_back(c_data[i]);
		}
		//printf("\n");

	}


}

/*
 * Checks if the checksums are correct.
 */
bool IMUM6::CheckData()
{
	bool b_right_data = false;

	int i_received_checksum = 0;
	int i_computed_checksum = 0;


	// Sum of start bits
	for(uint i = 0; i < sizeof(this->c_start_bits); i++)
	{
		i_computed_checksum += this->c_start_bits[i];
	}

	// Add packet type byte
	i_computed_checksum += this->um6_data.c_PT;

	// Add address byte
	i_computed_checksum += this->um6_data.c_adress;

	// Add sum of data bytes
	for(int i = 0; i < this->um6_data.packet_type.i_size_of_data; i++)
	{
		i_computed_checksum += this->um6_data.c_databytes[i];
	}

	i_received_checksum = this->um6_data.c_checksum[0] << 8;
	i_received_checksum |= this->um6_data.c_checksum[1];

	if(i_received_checksum == i_computed_checksum)
	{
		b_right_data = true;
	}
	else
	{
		printf("%x, %x, %d \n", this->um6_data.c_PT,
				this->um6_data.c_adress,
				this->um6_data.packet_type.i_size_of_data);
	}

	return b_right_data;
}

/*
 * Prints
 */
/*void IMUM6::PrintLogData()
{
	printf("LOG DATA:\n");

	for(list<char>::iterator it = this->T_c_serial_log.begin(); it != this->T_c_serial_log.end(); it++)
	{
		printf("%x_", (*it));
	}
}*/



/*ParseData
 * When in broadcast mode, these bits specify how often a data
 * packets are automatically transmitted over the serial port. The actual
 * broadcast frequency is given by
 * 		freq = (280/255)*broadcast_rate + 20
 */
int IMUM6::SetFrequency(int i_frequency)
{
	int i_status_value = -1;

	char c_broadcast_rate;

	c_broadcast_rate = (i_frequency - 20.0) * (255.0 / 280.0);

	printf("Broadcast Rate: %x\n", c_broadcast_rate);

	this->SetRegisterPinValue(this->i_communication_register, 0, 8, c_broadcast_rate);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION, this->i_communication_register);

	return i_status_value;
}

/*
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
 *
 * Note that if the desired baud rate is changed, the new configuration
 * must be written to FLASH in order for the new baud rate to persist
 * when power is cycled. A write to FLASH must be triggered by
 * sending a WRITE_TO_FLASH command with the currently selected
 * baud rate. This ensures that the baud rate is never mistakenly
 * changed permanently.
 */
int IMUM6::SetBaudRate(long l_baudrate)
{
	int i_status_value = -1;

	char c_baudrate = 0x00;

	switch (l_baudrate)
	{
		case 9600:
		{
			c_baudrate = 0b000;
			break;
		}
		case 14400:
		{
			c_baudrate = 0b001;
			break;
		}
		case 19200:
		{
			c_baudrate = 0b010;
			break;
		}
		case 38400:
		{
			c_baudrate = 0b011;
			break;
		}
		case 57600:
		{
			c_baudrate = 0b100;
			break;
		}
		case 115200:
		{
			c_baudrate = 0b101;
			break;
		}
		default:
		{
			perror("IMUM6: Undefined Baudrate.");
			exit(1);
			break;
		}

	}

	this->SetRegisterPinValue(this->i_communication_register, 8, 3, c_baudrate);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION, this->i_communication_register);

	return i_status_value;
}

/*
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
 */
int IMUM6::SetGPSBaudrate(long l_baudrate)
{
	int i_status_value = -1;

	char c_baudrate = 0x00;

	switch (l_baudrate)
	{
		case 9600:
		{
			c_baudrate = 0b000;
			break;
		}
		case 14400:
		{
			c_baudrate = 0b001;
			break;
		}
		case 19200:
		{
			c_baudrate = 0b010;
			break;
		}
		case 38400:
		{
			c_baudrate = 0b011;
			break;
		}
		case 57600:
		{
			c_baudrate = 0b100;
			break;
		}
		case 115200:
		{
			c_baudrate = 0b101;
			break;
		}
		default:
		{
			perror("IMIMU_UM6: Undefined Baudrate.");
			exit(1);
			break;
		}

	}

	this->SetRegisterPinValue(this->i_communication_register, 11, 3, c_baudrate);


	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION, this->i_communication_register);

	return i_status_value;
}

/*
 * Detailed satellite status information enabled. This is available in
 * firmware revisions UM2B and later.
 *
 * if b_status is true, then detailed satellite status information is enabled.
 */
int IMUM6::SetDetailedSatelliteStatusTrasmission(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_communication_register, 15, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION, this->i_communication_register);

	return i_status_value;
}

/*
 * Satellite summary transmission enabled. This is available in
 * firmware revisions UM2B and later.
 *
 * If b_status is true, then detailed satellite status information is enabled.
 */
int IMUM6::SetSatelliteSummaryTrasmission(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_communication_register, 16, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION, this->i_communication_register);

	return i_status_value;
}

/*
 * GPS course and velocity transmission enabled. This is available in
 * firmware revisions UM2B and later.
 *
 * If b_status is true, then detailed satellite status information is enabled.
 *
 */
int IMUM6::SetGPSCourseVelocityTrasmission(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_communication_register, 17, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION,
			this->i_communication_register);

	return i_status_value;
}

/*
 * GPS relative position transmission enabled. This is available in
 * firmware revisions UM2B and later.
 *
 * If b_status is true, then detailed satellite status information is enabled.
 *
 */
int IMUM6::SetGPSRelativePositionTrasmission(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_communication_register, 18, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION,
			this->i_communication_register);
	return i_status_value;
}

/*
 * GPS position transmission enabled. This is available in firmware
 * revisions UM2B and later.
 *
 * If b_status is true, then detailed satellite status information is enabled.
 */
int IMUM6::SetGPSPositionTransmission(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_communication_register, 19, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION,
			this->i_communication_register);
	return i_status_value;
}

/*
 * Temperature measurement transmission enabled. This is only
 * available in firmware revision UM2A and later.
 *
 * If b_status is true, then detailed satellite status information is enabled. *
 */
int IMUM6::SetTemperatureMeasurementTransmission(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_communication_register, 20, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION,
			this->i_communication_register);
	return i_status_value;
}

/*
 * Covariance matrix transmission enabled
 *
 * If b_status is true, then detailed satellite status information is enabled.
 */
int IMUM6::SetCovarianceMatrixTransmission(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_communication_register, 21, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION,
			this->i_communication_register);

	return i_status_value;
}

/*
 * Euler angle transmission enabled
 * If b_status is true, then detailed satellite status information is enabled.
 */
int IMUM6::SetEulerAngleTransmission(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_communication_register, 22, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION,
			this->i_communication_register);

	return i_status_value;
}

/*
 * Quaternion transmission enabled (only outputs valid data if
 * quaternion estimation is enabled in the MISC_CONFIG register)
 *
 * If b_status is true, then detailed satellite status information is enabled.
 */
int IMUM6::SetQuaternionTransmission(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_communication_register, 23, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION,
			this->i_communication_register);

	return i_status_value;
}

/*
 * Processed magnetometer data transmission enabled
 *
 * If b_status is true, then detailed satellite status information is enabled.
 */
int IMUM6::SetProcessedMagnetometerTransmission(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_communication_register, 24, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION,
			this->i_communication_register);

	return i_status_value;
}

/*
 * Processed accelerometer data transmission enabled
 *
 * If b_status is true, then detailed satellite status information is enabled.
 */
int IMUM6::SetProcessedAccelerometerTransmission(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_communication_register, 25, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION,
			this->i_communication_register);

	return i_status_value;
}

/*
 * Processed gyro data transmission enabled
 *
 * If b_status is true, then detailed satellite status information is enabled.
 */
int IMUM6::SetProcessedGyroTransmission(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_communication_register, 26, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION,
			this->i_communication_register);

	return i_status_value;
}

/*
 * Raw magnetometer data transmission enabled
 *
 * If b_status is true, then detailed satellite status information is enabled.
 */
int IMUM6::SetRawMagnetometerTransmission(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_communication_register, 27, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION,
			this->i_communication_register);

	return i_status_value;
}

/*
 * Raw accelerometer data transmission enabled
 *
 * If b_status is true, then detailed satellite status information is enabled.
 */
int IMUM6::SetRawAccelerometerTransmission(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_communication_register, 28, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION,
			this->i_communication_register);

	return i_status_value;
}

/*
 * Raw gyro data transmission enabled
 *
 * If b_status is true, then detailed satellite status information is enabled.
 */
int IMUM6::SetRawGyroTransmission(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_communication_register, 29, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION,
			this->i_communication_register);

	return i_status_value;
}

/*
 * Broadcast mode enabled
 *
 * If b_status is true, then detailed satellite status information is enabled.
 */
int IMUM6::SetMode(bool b_mode)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_mode) { c_status = 0x01;}
	else {c_status = 0x00;}

	this->SetRegisterPinValue(this->i_communication_register, 30, 1, c_status);

/*	this->i_communication_register &= (0xBFFF);
	this->i_communication_register |= (c_status);*/

	printf("Communication Register (Setmode): %x\n", this->i_communication_register);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_COMMUNICATION,
			this->i_communication_register);

	return i_status_value;
}


/*
 * Specifies whether PPS timing is enabled. This will be implemented
 * in future firmware revisions.
 */
int IMUM6::SetPPSTiming(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_misc_config_register, 27, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_MISC_CONFIG,
			this->i_misc_config_register);

	return i_status_value;
}

/*
 * Specifies whether quaternion state estimation is enabled.
 */
int IMUM6::SetQuaterninonState(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_misc_config_register, 28, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_MISC_CONFIG,
			this->i_misc_config_register);

	return i_status_value;
}

/*
 * Enables startup gyro calibration. If this bit is set, then gyros will be
 * calibrated automatically on sensor startup.
 */
int IMUM6::SetGyroCalibration(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_misc_config_register, 29, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_MISC_CONFIG,
			this->i_misc_config_register);

	return i_status_value;
}

/*
 * EKF accelerometer updates enabled (pitch and roll angle correction)
 */
int IMUM6::SetEKFAccelerometerUpdates(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_misc_config_register, 30, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_MISC_CONFIG,
			this->i_misc_config_register);

	return i_status_value;
}

/*
 * EKF magnetometer updates enabled (yaw angle correction)
 */
int IMUM6::SetEKFMagnetometerUpdates(bool b_status)
{
	int i_status_value = -1;
	char c_status = 0x00;

	if(b_status) c_status = 0x01;

	this->SetRegisterPinValue(this->i_misc_config_register, 31, 1, c_status);

	i_status_value = this->SetRegister32bitValue(IMUM6::REGISTERS::UM6_MISC_CONFIG,
			this->i_misc_config_register);

	return i_status_value;
}


void IMUM6::SetRegisterPinValue(
				int32_t & i_register,
				int i_pin_no,
				int i_count_of_pins,
				char c_value
			)
{

	int32_t i_mask = 0x00000000;
	int32_t i_mask_dummy = 0x00000000;

	if(i_count_of_pins > 32)
	{
		perror("IMIMU_UM6: Wrong count of pins. \n");
		exit(1);
	}

	for(int i = 0; i < i_count_of_pins; i++)
	{
		i_mask_dummy |= (0b1 << i);
	}

	i_mask |= (i_mask_dummy << i_pin_no);
	i_mask = ~i_mask;

	printf("IMASK: %x\n", i_mask);

	i_register &= i_mask;
	i_register |= (c_value << i_pin_no);
}









/*
 * Example:
 * float var = 0.0f;
 * Notice the lowercase f to indicate the literal should be interpreted as a 32-bit floating point number.
 */
int IMUM6::SetRegister32bitValue(int i_register, float f_data)
{
	int i_status_value = -1;
	int i_computed_checksum = 0;

	char c_data[11];

	UNION32 union32;

	union32.f = f_data;

	c_data[0] = this->c_start_bits[0];
	c_data[1] = this->c_start_bits[1];
	c_data[2] = this->c_start_bits[2];
	c_data[3] = 0b10000000; //PT
	c_data[4] = i_register;  // Address
	// DATA: [3][2][1][0]
	c_data[5] = union32.c[3];
	c_data[6] = union32.c[2];
	c_data[7] = union32.c[1];
	c_data[8] = union32.c[0];



	// Sum of
	for(uint i = 0; i < sizeof(c_data)-2; i++)
	{
		i_computed_checksum += c_data[i];
	}

	c_data[9] = (i_computed_checksum >> 8) & 0xFF;
	c_data[10] = i_computed_checksum & 0xFF;


	i_status_value = this->p_imserial->WriteData(c_data, sizeof(c_data));
	usleep(1000);
	this->b_command_complete = false;

	return i_status_value;
}

int IMUM6::SetRegister32bitValue(int i_register, int32_t i_data)
{
	int i_status_value = -1;
	int i_computed_checksum = 0;

	char c_data[11];

	UNION32 union32;

	union32.i = i_data;

	c_data[0] = this->c_start_bits[0];
	c_data[1] = this->c_start_bits[1];
	c_data[2] = this->c_start_bits[2];
	c_data[3] = 0b10000000; //PT
	c_data[4] = i_register;  // Address
	// DATA: [3][2][1][0]
	c_data[5] = union32.c[3];
	c_data[6] = union32.c[2];
	c_data[7] = union32.c[1];
	c_data[8] = union32.c[0];



	// Sum of
	for(uint i = 0; i < sizeof(c_data)-2; i++)
	{
		i_computed_checksum += c_data[i];
	}

	c_data[9] = (i_computed_checksum >> 8) & 0xFF;
	c_data[10] = i_computed_checksum & 0xFF;


	i_status_value = this->p_imserial->WriteData(c_data, sizeof(c_data));
	usleep(1000);
	this->b_command_complete = false;

	return i_status_value;

}

int IMUM6::SetRegister16bitValue(int i_register, int16_t * p_i_data)
{
	int i_status_value = -1;

	UNION32 union32;

	// Two's complement
	union32.i = -(unsigned int)p_i_data[0];
	union32.i |= (-(unsigned int)p_i_data[1]) << 16;

	i_status_value = this->SetRegister32bitValue(i_register, union32.i);

	return i_status_value;
}


int IMUM6::SetRegister16bitValue(int i_register, int16_t i_data)
{
	int i_status_value = -1;

	UNION32 union32;


	// Two's complement
	union32.i = 0x0000;
	union32.i |= (-(unsigned int)i_data) << 16;

	i_status_value = this->SetRegister32bitValue(i_register, union32.i);


	return i_status_value;
}


int IMUM6::ProcessData(vector<int> & T_i_registers)
{
	int i_status_value = -1;
	T_i_registers.clear();

	if(this->um6_data.packet_type.i_packet_has_data == 1)
	{
		if(this->um6_data.packet_type.i_packet_is_batch == 0)
		{
			UNION32 union32;

			union32.c[3] = this->um6_data.c_databytes[0];
			union32.c[2] = this->um6_data.c_databytes[1];
			union32.c[1] = this->um6_data.c_databytes[2];
			union32.c[0] = this->um6_data.c_databytes[3];

			this->i_data_registers[this->um6_data.c_adress - 85] = union32.i;
			T_i_registers.push_back(this->um6_data.c_adress);
		}
		else if(this->um6_data.packet_type.i_packet_is_batch > 0)
		{
			for(int i = 0; i < this->um6_data.packet_type.i_batch_length; i++)
			{
				UNION32 union32;

				union32.c[3] = this->um6_data.c_databytes[0 + i * 4];
				union32.c[2] = this->um6_data.c_databytes[1 + i * 4];
				union32.c[1] = this->um6_data.c_databytes[2 + i * 4];
				union32.c[0] = this->um6_data.c_databytes[3 + i * 4];

				this->i_data_registers[this->um6_data.c_adress + i - 85] = union32.i;
				T_i_registers.push_back(this->um6_data.c_adress + i);
			}
		}
		else
		{
			perror("IMIMU_UM6: Negative Batch.\n");
			exit(1);
		}
	}
	else if(this->um6_data.packet_type.i_packet_has_data == 0)
	{
		if(this->um6_data.packet_type.i_command_failed == 1)
		{
			printf("IMIMU_UM6: Command Failed. Address: %x\n", this->um6_data.c_adress);
			this->b_command_complete = false;
		}
		else
		{
			printf("IMIMU_UM6: Command Completed. Address: %x\n", this->um6_data.c_adress);
			this->b_command_complete = true;
		}
	}
	else
	{
		perror("IMIMU_UM6: Negative Has Data.\n");
		exit(1);
	}


	return i_status_value;
}

int32_t IMUM6::GetDataRegister(int i_register) const
{
	return this->i_data_registers[i_register - 85];
}

int IMUM6::SetCommand(int i_register)
{
	int i_status_value = -1;
	int i_computed_checksum = 0;

	char c_data[7];

	c_data[0] = this->c_start_bits[0];
	c_data[1] = this->c_start_bits[1];
	c_data[2] = this->c_start_bits[2];
	c_data[3] = 0b00000000; //PT
	c_data[4] = i_register;  // Address


	// Sum of
	for(uint i = 0; i < sizeof(c_data)-2; i++)
	{
		i_computed_checksum += c_data[i];
	}

	c_data[5] = (i_computed_checksum >> 8) & 0xFF;
	c_data[6] = i_computed_checksum & 0xFF;


	i_status_value = this->p_imserial->WriteData(c_data, sizeof(c_data));
	usleep(1000);
	this->b_command_complete = false;

	return i_status_value;
}













int IMUM6::CalculateChecksum()
{
	int i_status_value = -1;

	return i_status_value;
}


bool IMUM6::isCommandCompleted() const
{
	return this->b_command_complete;
}










/*void IMUM6::CBReadData()
{
	char c_read_dummy[1];
	this->p_imserial->ReadData(c_read_dummy, sizeof(c_read_dummy));
	this->c_input_buffer[this->i_buffer_read_index] = c_read_dummy[0];

	if(this->i_buffer_read_index + sizeof(c_read_dummy) > sizeof(this->c_input_buffer))
	{
		perror("IMIMU_UM6: Buffer is FULL");
		exit(1);
	}

	this->i_buffer_read_index += sizeof(c_read_dummy);
}

void IMUM6::StaticCallback(void *p)
{
	((IMIMU_UM6 *)p)->CBReadData();
}
*/

