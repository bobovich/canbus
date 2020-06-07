/*
 * lcd_parralel.cpp
 *
 *  Created on: 26 мая 2020 г.
 *      Author: Иван
 */

#include "lcd_parralel.h"

void lcdTask(void *p)
{
uint16_t dataB=0;
	lcd_parallel *lcd=new lcd_parallel();
	dataB=lcd->lcd_r_cmd(0);
	while(1)
	{


	}
}


lcd_parallel::lcd_parallel()
{
	PORT_INIT;//
	LCD_RES_SET;
	WAIT_150NS;
	WAIT_150NS;
	WAIT_150NS;
	LCD_RES_RESET;
	WAIT_150NS;
	WAIT_150NS;
	WAIT_150NS;
}

uint32_t lcd_parallel::lcd_w_cmd(uint16_t address, uint16_t data)// no optimized code for test
{
	LCD_nCS_SET;
	//address
	WAIT_50NS;
	LCD_RS_RESET;
	LCD_PORT_DATA_WRITE;
	LCD_PORT->ODR&=0xFFFF0000;
	LCD_nWR_SET;
	WAIT_150NS;
	LCD_nWR_RESET;
	LCD_PORT->ODR|=(0x0000FFFF&address);
	LCD_nWR_SET;
	WAIT_150NS;
	LCD_nWR_RESET;
	//data
	LCD_RS_SET;
	WAIT_150NS;
	LCD_PORT->ODR|=data>>8;
	LCD_nWR_SET;
	WAIT_150NS;
	LCD_nWR_RESET;
	LCD_PORT->ODR|=(0x0000FFFF&data);
	LCD_nWR_SET;
	WAIT_150NS;
	LCD_nWR_RESET;
	WAIT_150NS;
	LCD_nCS_RESET;
	return 1;
}

uint16_t lcd_parallel::lcd_r_cmd(uint16_t address)
{
	uint16_t data;
	LCD_nCS_SET;
	//address
	WAIT_50NS;
	LCD_RS_RESET;
	LCD_PORT_DATA_WRITE;
	LCD_PORT->ODR&=0xFFFF0000;
	LCD_nWR_SET;
	WAIT_150NS;
	LCD_nWR_RESET;
	LCD_PORT->ODR&=(0xFFFF0000|address);
	LCD_nWR_SET;
	WAIT_150NS;
	LCD_nWR_RESET;
	//data
	LCD_RS_SET;
	LCD_PORT_DATA_READ;
	WAIT_150NS;
	LCD_nRD_SET;
	WAIT_150NS;
	LCD_nRD_RESET;
	WAIT_150NS
	data=(LCD_PORT->IDR&0x0000FFFFF)<<8;
	LCD_nRD_SET;
	WAIT_150NS;
	LCD_nRD_RESET;
	WAIT_150NS;
	data|=(LCD_PORT->IDR&0x0000FFFFF);

	LCD_nCS_RESET;
	return data;
}
