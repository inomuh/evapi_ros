/****************************************
* 1 0 0 0 single-ended CH0
* 1 0 0 1 single-ended CH1
* 1 0 1 0 single-ended CH2
* 1 0 1 1 single-ended CH3
* 1 1 0 0 single-ended CH4
* 1 1 0 1 single-ended CH5
* 1 1 1 0 single-ended CH6
* 1 1 1 1 single-ended CH7
* 0 0 0 0 differential CH0 = IN+ CH1 = IN-
* 0 0 0 1 differential CH0 = IN- CH1 = IN+
* 0 0 1 0 differential CH2 = IN+ CH3 = IN-
* 0 0 1 1 differential CH2 = IN- CH3 = IN+
* 0 1 0 0 differential CH4 = IN+ CH5 = IN-
* 0 1 0 1 differential CH4 = IN- CH5 = IN+
* 0 1 1 0 differential CH6 = IN+ CH7 = IN-
* 0 1 1 1 differential CH6 = IN- CH7 = IN+
********************************************/
#include "../include/IMADC.h"

/**
 * IMADC constructor. Initializes required variables.
 */
IMADC::IMADC(IMSPI * p_imspi, int i_adc_bits)
{
	this->p_imspi = p_imspi;

	this->i_mask = 0;

	for(int i = 8; i < i_adc_bits; i++)
	{
		this->i_mask |= (0b1 << i);
	}
}

/**
 * Reads single-ended Analog to Digital Converter Channel (MCP3208) and returns value.
 */
int IMADC::ReadChannel(int i_channel_no)
{
	/**
	 * Data on the channel in integer format
	 */
	int i_value = 0;
	/**
	 * Data on the channel
	 */
	unsigned char u_c_data[3];
	/**
	 * Channel number with mask
	 */
	unsigned char u_c_dummy;
	u_c_dummy = i_channel_no & 0x07;

	/**
	 * FIGURE 6-1: SPI Communication using 8-bit segments (Mode 0,0: SCLK idles low).
	 */
	u_c_data[0] = 0x06 | (u_c_dummy >> 2);
	u_c_data[1] = (u_c_dummy << 6);
	u_c_data[2] = 0x00;

	/**
	 * Data is taken from spi.
	 */
	try{
		this->p_imspi->SpiWriteRead(u_c_data, sizeof(u_c_data));
	}catch(int i){
		throw i;
	}

	/**
	 * Data converted to integer format.
	 */
    i_value = (u_c_data[1]<< 8) & this->i_mask; //0b111100000000; //0b1100000000;  //merge data[1] & data[2] to get result
    i_value |=  (u_c_data[2] & 0xff);

	return i_value;
}

/**
 * Reads single-ended Analog to Digital Converter Channel (MCP3208) and returns value.
 */
int IMADC::ReadMotorChannel(int i_channel_no)
{
	/**
	 * Data on the channel in integer format
	 */
	int i_value = 0;
	/**
	 * Data on the channel
	 */
	unsigned char u_c_data[3];

	/**
	 * FIGURE 6-1: SPI Communication using 8-bit segments (Mode 0,0: SCLK idles low).
	 */
	u_c_data[0] = 0x01;
	if(i_channel_no == 0)
	{
		u_c_data[1] = 0xA0;
	}
	else if(i_channel_no == 1)
	{
		u_c_data[1] = 0xE0;
	}
	else
	{
		throw -64;
	}
	u_c_data[2] = 0x00;
	
	/**
	 * Data is taken from spi.
	 */
	try{
		this->p_imspi->SpiWriteRead(u_c_data, sizeof(u_c_data));
	}catch(int i){
		throw i;
	}

	/**
	 * Data converted to integer format.
	 */
    i_value = (u_c_data[1]<< 8) & this->i_mask; //0b111100000000; //0b1100000000;  //merge data[1] & data[2] to get result
    i_value |=  (u_c_data[2] & 0xff);

	return i_value;
}

/**
 * Reads differential Analog to Digital Converter Channel (MCP3208).
 */
int IMADC::ReadChannel(int i_channel_plus, int i_channel_negative)
{
	/**
	 * Data on the channel in integer format
	 */
	int i_value = 0;
	/**
	 * Data on the channel
	 */
	unsigned char u_c_data[3];

	/**
	 * Controls for i_channel_plus and i_channel_negative
	 */
	if(i_channel_plus > 7 || i_channel_negative > 7)
	{
		throw -64;
	}

	if(i_channel_plus % 2 == 0 && i_channel_negative <= i_channel_plus)
	{
		throw -64;
	}

	if(i_channel_plus % 2 != 0 && i_channel_negative >= i_channel_plus)
	{
		throw -64;
	}

	/**
	 * FIGURE 6-1: SPI Communication using 8-bit segments (Mode 0,0: SCLK idles low).
	 */
	u_c_data[0] = 1;  //  first byte transmitted -> start bit
	u_c_data[1] = 0b00000000 |( ((i_channel_plus & 7) << 4)); // second byte transmitted -> (SGL/DIF = 1, D2=D1=D0=0)
	u_c_data[2] = 0; // third byte transmitted....don't care

	/**
	 * Data is taken from spi.
	 */
	try{
		this->p_imspi->SpiWriteRead(u_c_data, sizeof(u_c_data));
	}catch(int i){
		throw i;
	}

	/**
	 * Data converted to integer format.
	 */
    i_value = (u_c_data[1]<< 8) & this->i_mask; // 0b111100000000; //merge data[1] & data[2] to get result
    i_value |=  (u_c_data[2] & 0xff);

	return i_value;
}
