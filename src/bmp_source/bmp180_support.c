/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file bmp180_support.c
* @date 10/01/2020
* @version  2.2.5
*
*/

/*---------------------------------------------------------------------------*/
/* Includes*/
/*---------------------------------------------------------------------------*/
#include "bmp180.h"
#define BMP180_API
/*----------------------------------------------------------------------------
 * The following functions are used for reading and writing of
 * sensor data using I2C or SPI communication
 *  ----------------------------------------------------------------------------*/
#ifdef BMP180_API

/*  \Brief: The function is used as I2C bus read
 *  \Return : Status of the I2C read
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register, will data is going to be read
 *  \param reg_data : This data read from the sensor, which is hold in an array
 *  \param cnt : The no of byte of data to be read
 */
s8 BMP180_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*  \Brief: The function is used as SPI bus write
 *  \Return : Status of the SPI write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register, will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *      will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
s8 BMP180_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*
 * \Brief: I2C init routine
 */
s8 I2C_routine(void);

#endif

/********************End of I2C function declarations***********************/

/*  Brief : The delay routine
 *  \param : delay in ms
 */
void BMP180_delay_msek(u32 msek);

/* This function is an example for reading sensor data
 *  \param: None
 *  \return: communication result
 */
s32 bmp180_data_readout_template(void);

/*----------------------------------------------------------------------------
 * struct bmp180_t parameters can be accessed by using bmp180
 *  bmp180_t having the following parameters
 *  Bus write function pointer: BMP180_WR_FUNC_PTR
 *  Bus read function pointer: BMP180_RD_FUNC_PTR
 *  Delay function pointer: delay_msec
 *  I2C address: dev_addr
 *  Chip id of the sensor: chip_id
 *  Calibration parameters
 * ---------------------------------------------------------------------------*/
struct bmp180_t bmp180;

/*----------------------------------------------------------------------------*/

/* This function is an example for reading sensor data
 *  \param: None
 *  \return: communication result
 */
s32 bmp180_data_readout_template(void)
{
    /* result of communication results*/
    s32 com_rslt = E_BMP_COMM_RES;
    u16 v_uncomp_temp_u16 = BMP180_INIT_VALUE;
    u32 v_uncomp_press_u32 = BMP180_INIT_VALUE;

    /*********************** START INITIALIZATION ************************/
    /**************Call the I2C init routine ***************/
#ifdef BMP180_API
    I2C_routine();
#endif

    /*--------------------------------------------------------------------------
     *  This function used to assign the value/reference of
     *  the following parameters
     *  I2C address
     *  Bus Write
     *  Bus read
     *  Chip id
     *  Calibration values
     * -------------------------------------------------------------------------*/
    com_rslt = bmp180_init(&bmp180);

    /************************* END INITIALIZATION *************************/

    /*------------------------------------------------------------------*
     ************************* START CALIPRATION ********
     *---------------------------------------------------------------------*/

    /*  This function used to read the calibration values of following
     *  these values are used to calculate the true pressure and temperature
     *  Parameter       MSB     LSB     bit
     *      AC1         0xAA    0xAB    0 to 7
     *      AC2         0xAC    0xAD    0 to 7
     *      AC3         0xAE    0xAF    0 to 7
     *      AC4         0xB0    0xB1    0 to 7
     *      AC5         0xB2    0xB3    0 to 7
     *      AC6         0xB4    0xB5    0 to 7
     *      B1          0xB6    0xB7    0 to 7
     *      B2          0xB8    0xB9    0 to 7
     *      MB          0xBA    0xBB    0 to 7
     *      MC          0xBC    0xBD    0 to 7
     *      MD          0xBE    0xBF    0 to 7*/
    com_rslt += bmp180_get_calib_param();

    /*------------------------------------------------------------------*
     ************************* END CALIPRATION ********
     *---------------------------------------------------------------------*/

    /************************* START READ UNCOMPENSATED TEMPERATURE AND PRESSURE********/

    /*  This API is used to read the
     *  uncompensated temperature(ut) value*/
    v_uncomp_temp_u16 = bmp180_get_uncomp_temperature();

    /*  This API is used to read the
     *  uncompensated pressure(ut) value*/
    v_uncomp_press_u32 = bmp180_get_uncomp_pressure();

    /************************* END READ UNCOMPENSATED TEMPERATURE AND PRESSURE********/
    /************************* START READ TRUE TEMPERATURE AND PRESSURE********/

    /****************************************************************************
     *  This API is used to read the
     *  true temperature(t) value input
     *  parameter as uncompensated temperature(ut)
     *
     ***************************************************************************/
    com_rslt += bmp180_get_temperature(v_uncomp_temp_u16);

    /****************************************************************************
     *  This API is used to read the
     *  true pressure(p) value
     *  input parameter as uncompensated pressure(up)
     *
     ***************************************************************************/
    com_rslt += bmp180_get_pressure(v_uncomp_press_u32);

    /************************* END READ TRUE TEMPERATURE AND PRESSURE********/
    return com_rslt;
}

#ifdef BMP180_API

/*--------------------------------------------------------------------------*
 *  The following function is used to map the I2C bus read, write, delay and
 *  device address with global structure bmp180_t
 *-------------------------------------------------------------------------*/
s8 I2C_routine(void)
{
    /*--------------------------------------------------------------------------*
    *  By using bmp180 the following structure parameter can be accessed
    *   Bus write function pointer: BMP180_WR_FUNC_PTR
    *   Bus read function pointer: BMP180_RD_FUNC_PTR
    *   Delay function pointer: delay_msec
    *   I2C address: dev_addr
    *--------------------------------------------------------------------------*/
    bmp180.bus_write = BMP180_I2C_bus_write;
    bmp180.bus_read = BMP180_I2C_bus_read;
    bmp180.dev_addr = BMP180_I2C_ADDR;
    bmp180.delay_msec = BMP180_delay_msek;

    return BMP180_INIT_VALUE;
}

/************** I2C buffer length ******/

#define I2C_BUFFER_LEN 8
#define I2C0           5

/*-------------------------------------------------------------------*
 *  This is a sample code for read and write the data by using I2C
 *  Configure the below code to your I2C driver
 *  The device address is defined in the bmp180.c file
 *-----------------------------------------------------------------------*/

/*  \Brief: The function is used as I2C bus write
 *  \Return : Status of the I2C write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register, will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *      will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
s8 BMP180_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{

	uint8_t c=0;
	if ( (cnt<0 || cnt > I2C_BUFFER_LEN) )  return 0;
	if ((I2C1->SR1 & 0xf0) != 0)
	{
		I2C1->CR1 = I2C_CR1_SWRST;
	};
	if (I2C1->SR2&I2C_SR2_BUSY)
	{
		I2C1->CR1|=I2C_CR1_STOP;
		while (!(I2C1->SR2&I2C_SR2_BUSY));
	}
	I2C1->CR1|=I2C_CR1_START;
	I2C1->CR1|=I2C_CR1_ACK;
	while (!(I2C1->SR1 & I2C_SR1_SB));
	I2C1->DR=dev_addr<<1; // write address
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	if (I2C1->SR2);
	I2C1->DR=reg_addr;
	while (!(I2C1->SR1 & I2C_SR1_BTF));
	while (c < cnt)
	{
		while (!(I2C1->SR1&I2C_SR1_BTF));
		I2C1->DR=*(reg_data+c);
		c++;
		if (c==cnt)
		{
			//I2C1->CR1&=~I2C_CR1_ACK;
			I2C1->CR1|=I2C_CR1_STOP;
		}
	};
	return 1;


}

/*  \Brief: The function is used as I2C bus read
 *  \Return : Status of the I2C read
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register, will data is going to be read
 *  \param reg_data : This data read from the sensor, which is hold in an array
 *  \param cnt : The no of byte of data to be read
 */
s8 BMP180_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{

    uint8_t c=0;

    if (cnt <0 || cnt > I2C_BUFFER_LEN)  return 0;
    	if ((I2C1->SR1 & 0xf0) != 0)
    	{
    		I2C1->CR1 = I2C_CR1_SWRST;

    	};
    	if (I2C1->SR2&I2C_SR2_BUSY)
    	{
    		I2C1->CR1|=I2C_CR1_STOP;
    		while (!(I2C1->SR2&I2C_SR2_BUSY));
    	}
    	I2C1->CR1|=I2C_CR1_START;
    	I2C1->CR1|=I2C_CR1_ACK;
    	while (!(I2C1->SR1 & I2C_SR1_SB));
    	I2C1->DR=dev_addr<<1; // write address to read
    	while (!(I2C1->SR1 & I2C_SR1_ADDR));
    	if (I2C1->SR2);
    	I2C1->DR=reg_addr;
    	//I2C1->CR1|=I2C_CR1_STOP;
    	while (!(I2C1->SR1 & I2C_SR1_BTF)); //?
    	I2C1->CR1|=I2C_CR1_START;
    	while (!(I2C1->SR1 & I2C_SR1_SB));
    	I2C1->DR=(dev_addr<<1)|0x01; // address  read mode
    	while (!(I2C1->SR1 & I2C_SR1_ADDR));
    	if (I2C1->SR2);
    	while (c < cnt)
    	{
    		if (c==(cnt-1))I2C1->CR1&=~I2C_CR1_ACK;
    		while (!(I2C1->SR1&I2C_SR1_RXNE));

    		*(reg_data+c)=I2C1->DR;
    		c++;
    		if (c==cnt)
    		{
    			I2C1->CR1|=I2C_CR1_STOP;
    		}


    	};
    	return 1;
   }

/*  Brief : The delay routine
 *  \param : delay in ms
 */
void BMP180_delay_msek(u32 msek)
{
    /*Here you can write your own delay routine*/
}

#endif
