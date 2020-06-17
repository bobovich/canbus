/*
 * lcd_parralel.cpp
 *
 *  Created on: 26 мая 2020 г.
 *      Author: Иван
 */

#include "lcd_parralel.h"
#include "asciitable.h"
#include "string.h"
#include "uart_com.h"
#include <cstdlib>
void lcdTask(void *p)
{

char he[50];
char bufTmp[10];
	lcd_parallel *lcd=new lcd_parallel();
	pixelcolor col={255,0,0};
	pixelcolor bg={0,0,0};
	lcd->lcd_init();
	lcd->lcd_flush(bg);

	air_condition airData;
	QueueHandle_t dQueue =(QueueHandle_t)p;
	vTaskDelay(1000/ portTICK_PERIOD_MS);



	while(1)
	{
		vTaskDelay(500/ portTICK_PERIOD_MS);
		xQueueReceive( dQueue, &airData,5);
		strcpy(he, "CO2: ");
		strcat(he, itoa((int)airData.CO2, bufTmp, 10));
		lcd->lcd_write_str90(he,0,0,col, bg, 0);
	//	wtireString90( he , 0,0,0);
		strcpy(he, "TVOC: ");
		strcat(he, itoa((int)airData.TVOC, bufTmp, 10));
		lcd->lcd_write_str90(he,130,0,col, bg, 0);
		//wtireString90( he , 130,0,0);
		strcpy(he, "Temp: ");
		strcat(he, ftoa(airData.temp,2, bufTmp));
		lcd->lcd_write_str90(he,0,25,col, bg, 0);
		//wtireString90( he , 0,25,0);
		strcpy(he, "Humidity: ");
		strcat(he, ftoa(airData.humidity,2, bufTmp));
		lcd->lcd_write_str90(he,130,25,col, bg, 0);
		//wtireString90( he , 130,25,0);
		strcpy(he, "T2: ");
		strcat(he, ftoa(airData.temp2,2, bufTmp));
		lcd->lcd_write_str90(he,0,50,col, bg, 0);
		//wtireString90( he , 0,50,0);
		strcpy(he, "Press: ");
		strcat(he, ftoa(airData.pressure,2, bufTmp));
		lcd->lcd_write_str90(he,130,50,col, bg, 0);
		//wtireString90( he , 130,50,0);
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
	LCD_nCS_SET;
	WAIT_150NS;
	LCD_RS_RESET;
	LCD_PORT_DATA_WRITE;
	LCD_PORT->ODR&=0xFFFF0000;
	for (char i=0; i<4; i++)
	{
		LCD_nWR_SET;
		WAIT_150NS;
		LCD_nWR_RESET;
	};
	LCD_nCS_RESET;
	LCD_RS_RESET;
}

uint32_t lcd_parallel::lcd_flush(pixelcolor color)
{
	this->lcd_w_cmd(0x20, 0);
	this->lcd_w_cmd(0x21,0);
	LCD_nCS_SET;
		//address
	for (int i=0;i<0x13fef; i++)
	{
		//WAIT_50NS;
		LCD_RS_RESET;
		LCD_PORT_DATA_WRITE;
		LCD_PORT->ODR&=0xFF00;
		LCD_nWR_SET;
		//WAIT_150NS;
		LCD_nWR_RESET;
		LCD_PORT->ODR&=0xFF00;
		LCD_PORT->ODR|=(0x00FF&0x22);
		LCD_nWR_SET;
		//WAIT_150NS;
		LCD_nWR_RESET;
		//data
		LCD_RS_SET;
		//WAIT_150NS;

		LCD_PORT->ODR&=0xFF00;
		LCD_PORT->ODR|=(0x00FF&color.B);
		LCD_nWR_SET;
		//WAIT_150NS;
		LCD_nWR_RESET;

		LCD_PORT->ODR&=0xFF00;
		LCD_PORT->ODR|=(0x00FF&color.G);
		LCD_nWR_SET;
		//WAIT_150NS;
		LCD_nWR_RESET;
		//WAIT_150NS;

		LCD_PORT->ODR&=0xFF00;
		LCD_PORT->ODR|=(0x00FF&color.R);
		LCD_nWR_SET;
		//WAIT_150NS;
		LCD_nWR_RESET;
		//WAIT_150NS;
	};
		LCD_nCS_RESET;
		return 1;
}
void lcd_parallel::lcd_init()
{
	lcd_w_cmd(0x00, 0x0001);

	lcd_w_cmd(0x15, 0x0030);
	lcd_w_cmd(0x9a, 0x0010);
	lcd_w_cmd(0x11, 0x0020);
	lcd_w_cmd(0x12, 0x0002);//
	lcd_w_cmd(0x13, 0x0d39);
	lcd_w_cmd(0x10, 0x1428);
	WAIT_40MS;
	lcd_w_cmd(0x12, 0x0014);
	WAIT_40MS;
	lcd_w_cmd(0x10, 0x3420);
	lcd_w_cmd(0x13, 0x2d39);
	WAIT_40MS;
	lcd_w_cmd(0x30, 0x0401);
	lcd_w_cmd(0x31, 0x0405);
	lcd_w_cmd(0x32, 0x0002);
	lcd_w_cmd(0x33, 0x0404);
	lcd_w_cmd(0x34, 0x0407);
	lcd_w_cmd(0x35, 0x0304);
	lcd_w_cmd(0x36, 0x0604);
	lcd_w_cmd(0x37, 0x0404);
	lcd_w_cmd(0x38, 0x0604);
	lcd_w_cmd(0x39, 0x0604);
	lcd_w_cmd(0x01, 0x0100);
	lcd_w_cmd(0x02, 0x0300);
	lcd_w_cmd(0x03, 0xC030);
	lcd_w_cmd(0x08, 0x0808);
	lcd_w_cmd(0x0a, 0x0008);
	lcd_w_cmd(0x60, 0x2700);//
	lcd_w_cmd(0x61, 0x0001);
	lcd_w_cmd(0x90, 0x013e);
	lcd_w_cmd(0x92, 0x0100);
	lcd_w_cmd(0x93, 0x0100);
	lcd_w_cmd(0xa0, 0x3000);
	lcd_w_cmd(0xa3, 0x0010);
	lcd_w_cmd(0x07, 0x0001);
	lcd_w_cmd(0x07, 0x0021);
	lcd_w_cmd(0x07, 0x0023);
	lcd_w_cmd(0x07, 0x0033);
	lcd_w_cmd(0x07, 0x0133);

/*
	lcd_w_cmd(0x15, 0x0030);
	lcd_w_cmd(0x9a, 0x0010);
	lcd_w_cmd(0x11, 0x0020);
	lcd_w_cmd(0x10, 0x3428);
	lcd_w_cmd(0x12, 0x0002);//
	lcd_w_cmd(0x13, 0x1038);

	WAIT_40MS;
	lcd_w_cmd(0x12, 0x0012);
	WAIT_40MS;
	lcd_w_cmd(0x10, 0x3420);
	lcd_w_cmd(0x13, 0x3045);
	WAIT_40MS;
	lcd_w_cmd(0x30, 0x0000);
	lcd_w_cmd(0x31, 0x0402);
	lcd_w_cmd(0x32, 0x0307);
	lcd_w_cmd(0x33, 0x0304);
	lcd_w_cmd(0x34, 0x0004);
	lcd_w_cmd(0x35, 0x0401);
	lcd_w_cmd(0x36, 0x0707);
	lcd_w_cmd(0x37, 0x0305);
	lcd_w_cmd(0x38, 0x0610);
	lcd_w_cmd(0x39, 0x0610);
	lcd_w_cmd(0x01, 0x0100);
	lcd_w_cmd(0x02, 0x0300);
	lcd_w_cmd(0x03, 0x1030);//?
	lcd_w_cmd(0x08, 0x0808);
	lcd_w_cmd(0x0a, 0x0008);
	lcd_w_cmd(0x60, 0x2700);//
	lcd_w_cmd(0x61, 0x0001);
	lcd_w_cmd(0x90, 0x013e);
	lcd_w_cmd(0x92, 0x0100);
	lcd_w_cmd(0x93, 0x0100);
	lcd_w_cmd(0xa0, 0x3000);
	lcd_w_cmd(0xa3, 0x0010);
	lcd_w_cmd(0x07, 0x0001);
	lcd_w_cmd(0x07, 0x0021);
	lcd_w_cmd(0x07, 0x0023);
	lcd_w_cmd(0x07, 0x0033);
	lcd_w_cmd(0x07, 0x0133);*/

}
uint32_t lcd_parallel::lcd_w_cmd(uint16_t address, uint16_t data)// no optimized code for test
{
	LCD_nCS_SET;
	//address
	WAIT_50NS;
	LCD_RS_RESET;
	LCD_PORT_DATA_WRITE;
	LCD_PORT->ODR&=0xFF00;
	LCD_nWR_SET;
	WAIT_150NS;
	LCD_nWR_RESET;
	LCD_PORT->ODR&=0xFF00;
	LCD_PORT->ODR|=(0x00FF&address);
	LCD_nWR_SET;
	WAIT_150NS;
	LCD_nWR_RESET;
	//data
	LCD_RS_SET;
	WAIT_150NS;
	LCD_PORT->ODR&=0xFF00;
	LCD_PORT->ODR|=(0x00ff&(data>>8));
	LCD_nWR_SET;
	WAIT_150NS;
	LCD_nWR_RESET;
	LCD_PORT->ODR&=0xFF00;
	LCD_PORT->ODR|=(0x00FF&data);
	LCD_nWR_SET;
	WAIT_150NS;
	LCD_nWR_RESET;
	WAIT_150NS;
	LCD_nCS_RESET;
	return 1;
}

uint32_t lcd_parallel::lcd_draw_XY(uint16_t x, uint16_t y, pixelcolor color)// no optimized code for test
{
	this->lcd_w_cmd(0x20, x);
	this->lcd_w_cmd(0x21,y);
	LCD_nCS_SET;
	//address
	WAIT_50NS;
	LCD_RS_RESET;
	LCD_PORT_DATA_WRITE;
	LCD_PORT->ODR&=0xFF00;
	LCD_nWR_SET;
	WAIT_150NS;
	LCD_nWR_RESET;
	LCD_PORT->ODR&=0xFF00;
	LCD_PORT->ODR|=(0x00FF&0x22);
	LCD_nWR_SET;
	WAIT_150NS;
	LCD_nWR_RESET;
	//data
	LCD_RS_SET;
	WAIT_150NS;

	LCD_PORT->ODR&=0xFF00;
	LCD_PORT->ODR|=(0x00FF&color.B);
	LCD_nWR_SET;
	WAIT_150NS;
	LCD_nWR_RESET;

	LCD_PORT->ODR&=0xFF00;
	LCD_PORT->ODR|=(0x00FF&color.G);
	LCD_nWR_SET;
	WAIT_150NS;
	LCD_nWR_RESET;
	WAIT_150NS;

	LCD_PORT->ODR&=0xFF00;
	LCD_PORT->ODR|=(0x00FF&color.R);
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
	data=(uint16_t)(LCD_PORT->IDR&0x00FF)<<8;
	LCD_nRD_RESET;
	WAIT_150NS;

	LCD_nRD_SET;
	WAIT_150NS;
	data|=((uint16_t)(LCD_PORT->IDR&0x00FF));
	LCD_nRD_RESET;
	WAIT_150NS;


	LCD_nCS_RESET;
	return data;
}

#define FONT_H 22
uint32_t lcd_parallel::lcd_write_str(char *str, uint16_t x, uint16_t y, pixelcolor text_color, pixelcolor back_color,  void* font)
{
	uint32_t by,cx,cy;
	cx=x;
	cy=y;
	uint32_t i=0;
	while ( str[i] !=0 )
	{
		for (char cnt=1; cnt<31; cnt++)
		{
			by= Consolas10x22[(str[i]-32)*31+cnt];
			for (uint32_t bc=0; bc<8; bc++)
			{
				if ( ( ((by>>bc)&0x01) == 0 ) &&((cy-y)<=FONT_H))
				{
					this->lcd_draw_XY(cx,cy, back_color);
				}
				if ( ( ((by>>bc)&0x01) == 1 ) &&((cy-y)<=FONT_H))
				{
					this->lcd_draw_XY(cx,cy, text_color);
				};

			 cy++;
			};

			if ((cnt-(((uint32_t)(cnt/3))*3)) == 0 )
			{
				cy=y;
				cx++;
			};
		};
		i++;
	};
	return 1;
}

uint32_t lcd_parallel::lcd_write_str90(char *str, uint16_t x, uint16_t y, pixelcolor text_color, pixelcolor back_color,  void* font)
{
	uint32_t by,cx,cy;
	cx=x;
	cy=y;
	uint32_t i=0;
	while ( str[i] !=0 )
	{
		for (char cnt=1; cnt<31; cnt++)
		{
			by= Consolas10x22[(str[i]-32)*31+cnt];
			for (uint32_t bc=0; bc<8; bc++)
			{
				if ( ( ((by>>bc)&0x01) == 0 ) &&((cy-y)<=FONT_H))
				{
					this->lcd_draw_XY(cy,319-cx, back_color);
				}
				if ( ( ((by>>bc)&0x01) == 1 ) &&((cy-y)<=FONT_H))
				{
					this->lcd_draw_XY(cy,319-cx, text_color);
				};

			 cy++;
			};

			if ((cnt-(((uint32_t)(cnt/3))*3)) == 0 )
			{
				cy=y;
				cx++;
			};
		};
		i++;
	};
}



