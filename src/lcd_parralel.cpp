/*
 * lcd_parralel.cpp
 *
 *  Created on: 26 пїЅпїЅпїЅ 2020 пїЅ.
 *      Author: пїЅпїЅпїЅпїЅ
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
	pixelcolor col={63,0,0};
	pixelcolor bg={0,0,0};
	pixelcolor h1={0,63,0};
	lcd->lcd_init();
	lcd->flush(bg);
	putXY touch;
	air_condition airData;
	QueueHandle_t dQueue =(QueueHandle_t)p;
	lcd->touch_init();
	vTaskDelay(1000/ portTICK_PERIOD_MS);
	lcd->draw_bitmap90(150,142, rawData);
	//lcd->draw_bitmap90(0,0, rawData1);
	//vTaskDelay(10000/ portTICK_PERIOD_MS);
	while(1)
	{
		//vTaskDelay(500/ portTICK_PERIOD_MS);
		xQueueReceive( dQueue, &airData,5);
		//CO2

		strcpy(he, "CO2: ");
		strcat(he, itoa((int)airData.CO2, bufTmp, 10));
		strcat(he, "  ");
		lcd->write_str90(he,0,0,col, bg, Courier_New13x23);
	//	TVOC
		strcpy(he, "TVOC: ");
		strcat(he, itoa((int)airData.TVOC, bufTmp, 10));
		strcat(he, "  ");
		lcd->write_str90(he,130,0,col, bg, Courier_New13x23);
		//TEMP
		strcpy(he, "Темп: ");
		strcat(he, ftoa(airData.temp,2, bufTmp));
		strcat(he, "  ");
		lcd->write_str90(he,0,25,col, bg, Courier_New13x23);
		//HUMI
		strcpy(he, "Влажн.: ");
		strcat(he, ftoa(airData.humidity,2, bufTmp));
		strcat(he, "  ");
		lcd->write_str90(he,130,25,col, bg, Courier_New13x23);
		//T2
		strcpy(he, "Дат2: ");
		strcat(he, ftoa(airData.temp2,2, bufTmp));
		strcat(he, "  ");
		lcd->write_str90(he,0,50,col, bg, Courier_New13x23);
		//PRESSURE
		strcpy(he, "Давление: ");
		strcat(he, ftoa(airData.pressure,2, bufTmp));
		strcat(he, "  ");
		lcd->write_str90(he,130,50,col, bg, Courier_New13x23);


		lcd->write_str90("Детям Мороженное! \0",0,83,h1, bg, Courier_New13x23);
		lcd->write_str90("Любимой жене цветы! \0",0,105,h1, bg, Courier_New13x23);

		h1.B=(uint8_t)rand();
		h1.R=(uint8_t)rand();
		h1.G=(uint8_t)rand();
		lcd->touch_loop(&touch);
		strcpy(he, "X");
		strcat(he, itoa((int)touch.x, bufTmp, 10));
		strcat(he, "      ");
		lcd->write_str90(he,0,125,col, bg, Courier_New13x23);

		strcpy(he, "Y");
		strcat(he, itoa((int)touch.y, bufTmp, 10));
		strcat(he, "      ");
		lcd->write_str90(he,0,150,col, bg, Courier_New13x23);

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

uint32_t lcd_parallel::flush(pixelcolor color)
{
	this->w_cmd(0x20, 0);
	this->w_cmd(0x21,0);
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
	w_cmd(0x00, 0x0001);
	w_cmd(0x15, 0x0030);
	w_cmd(0x9a, 0x0010);
	w_cmd(0x11, 0x0020);
	w_cmd(0x12, 0x0002);//
	w_cmd(0x13, 0x0d39);
	w_cmd(0x10, 0x1428);
	WAIT_40MS;
	w_cmd(0x12, 0x0014);
	WAIT_40MS;
	w_cmd(0x10, 0x3420);
	w_cmd(0x13, 0x2d39);
	WAIT_40MS;
	w_cmd(0x30, 0x0401);
	w_cmd(0x31, 0x0405);
	w_cmd(0x32, 0x0002);
	w_cmd(0x33, 0x0404);
	w_cmd(0x34, 0x0407);
	w_cmd(0x35, 0x0304);
	w_cmd(0x36, 0x0604);
	w_cmd(0x37, 0x0404);
	w_cmd(0x38, 0x0604);
	w_cmd(0x39, 0x0604);
	w_cmd(0x01, 0x0100);
	w_cmd(0x02, 0x0300);
	w_cmd(0x03, 0xC030);
	w_cmd(0x08, 0x0808);
	w_cmd(0x0a, 0x0008);
	w_cmd(0x60, 0x2700);//
	w_cmd(0x61, 0x0001);
	w_cmd(0x90, 0x013c);//
	w_cmd(0x92, 0x0100);
	w_cmd(0x93, 0x0100);
	w_cmd(0xa0, 0x3000);
	w_cmd(0xa3, 0x0010);
	w_cmd(0x07, 0x0001);
	w_cmd(0x07, 0x0021);
	w_cmd(0x07, 0x0023);
	w_cmd(0x07, 0x0033);
	w_cmd(0x07, 0x0133);
}
uint32_t lcd_parallel::w_cmd(uint16_t address, uint16_t data)// no optimized code for test
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

uint32_t lcd_parallel::draw_XY(uint16_t x, uint16_t y, pixelcolor color)// no optimized code for test
{
	this->w_cmd(0x20, x);
	this->w_cmd(0x21,y);
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
	LCD_PORT->ODR|=(0x00FF&(color.B<<2));
	LCD_nWR_SET;
	WAIT_150NS;
	LCD_nWR_RESET;

	LCD_PORT->ODR&=0xFF00;
	LCD_PORT->ODR|=(0x00FF&(color.G<<2));
	LCD_nWR_SET;
	WAIT_150NS;
	LCD_nWR_RESET;
	WAIT_150NS;

	LCD_PORT->ODR&=0xFF00;
	LCD_PORT->ODR|=(0x00FF&(color.R<<2));
	LCD_nWR_SET;
	WAIT_150NS;
	LCD_nWR_RESET;
	WAIT_150NS;

	LCD_nCS_RESET;
	return 1;
}

uint16_t lcd_parallel::r_cmd(uint16_t address)
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
uint32_t lcd_parallel::write_str(char *str, uint16_t x, uint16_t y, pixelcolor text_color, pixelcolor back_color,  const  uint8_t* font)
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
					this->draw_XY(cx,cy, back_color);
				}
				if ( ( ((by>>bc)&0x01) == 1 ) &&((cy-y)<=FONT_H))
				{
					this->draw_XY(cx,cy, text_color);
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

/*uint32_t lcd_parallel::lcd_write_str90(char *str, uint16_t x, uint16_t y, pixelcolor text_color, pixelcolor back_color,  const uint8_t* font)
{
	uint32_t by,cx,cy;
	cx=x;
	cy=y;
	uint32_t i=0;
	while ( str[i] !=0 )
	{
		for (char cnt=1; cnt<31; cnt++)
		{
			by= Consolas10x22[(str[i]-32)*31+cnt+2];
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
}*/
uint32_t lcd_parallel::write_str90(char *str, uint16_t x, uint16_t y, pixelcolor text_color, pixelcolor back_color,  const  uint8_t* font)
{
	uint32_t by,cx,cy;
	u8 bpc;
	u8 lng_offset=0;
	bpc=(u8)*(font+1)/8;
	if ((*(font+1)%8)>0)bpc++;
	cx=x;
	cy=y;
	uint32_t i=0;
	while ( str[i] !=0 )
	{
		if (str[i]>= 0xc0) lng_offset=font[2]; else lng_offset=0;
		for (char cnt=1; cnt< (font[(str[i]-(32+lng_offset))*((font[0] * bpc)+1)+3] * bpc)+1; cnt++)
		{

			by= font[(str[i]-(32+lng_offset))*((font[0] * bpc)+1)+cnt+3];//?
			for (uint32_t bc=0; bc<8; bc++)
			{
				if ( ( ((by>>bc)&0x01) == 0 ) &&((cy-y)<=font[1]))
				{
					this->draw_XY(cy,dW-1-cx, back_color);
				}
				if ( ( ((by>>bc)&0x01) == 1 ) &&((cy-y)<=font[1]))
				{
					this->draw_XY(cy,dW-1-cx, text_color);
				};

			 cy++;
			};

			if ((cnt-(((uint32_t)(cnt/bpc))*bpc)) == 0 )
			{
				cy=y;
				cx++;
			};
		};
		i++;
	};
	return 1;
}


uint32_t lcd_parallel::draw_bitmap(uint16_t x, uint16_t y, const uint8_t* bmp)
{
	uint32_t w,h;
	w=bmp[0];
	h=bmp[1];
	uint8_t* pa;
	pixelcolor pix;
	for (uint32_t h0=0; h0<h; h0++)
	{
		for (uint32_t w0=0; w0<w; w0++)
		{
			pa=(uint8_t*)bmp+3*(w0+w*h0);
			pix.G=*pa/4;
			pix.B=*(pa+1)/4;
			pix.R=*(pa+2)/4;
			this->draw_XY(x+w0,y+h-h0, pix );
		}
	}
	return 1;
}

uint32_t lcd_parallel::draw_bitmap90(uint16_t x, uint16_t y, const uint8_t* bmp)
{
	uint32_t w,h;
	w=bmp[0];
	h=bmp[1];
	uint8_t* pa;
	pixelcolor pix;
	for (uint32_t h0=0; h0<h; h0++)
	{
		for (uint32_t w0=0; w0<w; w0++)
		{
			pa=(uint8_t*)bmp+3*(w0+w*h0);
			pix.G=*pa/4;
			pix.B=*(++pa)/4;
			pix.R=*(++pa)/4;
			this->draw_XY(y+h-h0,dW-x-w0, pix );
		}
	};
	return 1;
}

void lcd_parallel::touch_init(void)
{
	RCC->CFGR|=RCC_CFGR_ADCPRE_DIV4;
	RCC->APB2ENR|=RCC_APB2ENR_ADC1EN;
	ADC1->CR2|=ADC_CR2_ADON;
	WAIT_40MS;
	ADC1->CR2|=ADC_CR2_CAL;
	while(ADC1->CR2&ADC_CR2_CAL); /* critical to failure !!!!	*/
	return;
}

uint32_t lcd_parallel::touch_loop(putXY* touch)
{
	uint16_t x,y;
	LCD_nCS_RESET;
	// for x
	//set RS as float
	GPIOC->CRL&=~0x00000F00;
	GPIOC->CRL|=0x00000400;
	//set nWR to 1
	LCD_nWR_RESET;
	//set D7 to 0
	GPIOA->ODR&=~0x0080;
	//set D6 to float
	GPIOA->CRL&=~0x0F000000;
	GPIOA->CRL|=0x0400000;
	//read data from D6(adc6)
	ADC1->SQR1|= 1<<20;
	ADC1->SQR3=6;
	ADC1->CR2|=ADC_CR2_ADON;
	while (!(ADC1->SR|ADC_SR_EOC));
	ADC1->SR&=~ADC_SR_EOC;
	touch->x=ADC1->DR;

	//for y

	//set nWR as float
	GPIOC->CRL&=~	0x000000F0;
	GPIOC->CRL|=	0x00000040;
	//set RS to 1
	LCD_RS_SET;
	//set D6 to 0
	GPIOA->ODR&=~0x0040;
	//set D7 to float
	GPIOA->CRL&=~0xF0000000;
	GPIOA->CRL|=0x4000000;
	//read data from D7(adc7)
	ADC1->SQR1|= 1<<20;
	ADC1->SQR3=7;
	ADC1->CR2|=ADC_CR2_ADON;
	while (!(ADC1->SR|ADC_SR_EOC));
	ADC1->SR&=~ADC_SR_EOC;
	touch->y=ADC1->DR;
	/*GPIOB->CRL&=0x0000000F;
	GPIOB->CRL|=0x00000004;*/

	PORT_INIT;



	return 1;
}

