/*
 * eink_display.cpp
 *
 *  Created on: 12 ‡Ô. 2020 „.
 *      Author: »‚‡Ì
 */

#include "eink_display.h"
#include "Ap_29demo.h"
#include "asciitable.h"
#include "string.h"
#include "uart_com.h"
#include <cstdlib>
#define D_WIDTH 128
#define D_HEIGHT 296
#define FONT_H 22
uint8_t dispRam[4736];

unsigned int size;
unsigned char HRES,VRES_byte1,VRES_byte2;
void GPIO_Configuration(void);
void clearPix(uint32_t x, uint32_t y);
void clearDispRAM(void);
void setPix(uint32_t x, uint32_t y);
void wtireString( char*str ,  uint32_t x, uint32_t y,  uint32_t mode);
void wtireString90( char*str ,  uint32_t x, uint32_t y,  uint32_t mode);
char he[50];


void displayTask(void* pvParams)
{
	GPIO_Configuration();
	char bufTmp[10];
	air_condition airData;
	QueueHandle_t dQueue =(QueueHandle_t)pvParams;
	vTaskDelay(10000/ portTICK_PERIOD_MS);
	EPD_init();
	for(;;)
	{


		xQueueReceive( dQueue, &airData,5);
		clearDispRAM();

		strcpy(he, "CO2: ");
		strcat(he, itoa((int)airData.CO2, bufTmp, 10));
		wtireString90( he , 0,0,0);
		strcpy(he, "TVOC: ");
		strcat(he, itoa((int)airData.TVOC, bufTmp, 10));
		wtireString90( he , 0,30,0);
		strcpy(he, "Temp: ");
		strcat(he, ftoa(airData.temp,2, bufTmp));
		wtireString90( he , 0,60,0);
		strcpy(he, "Humidity: ");
		strcat(he, ftoa(airData.humidity,2, bufTmp));
		wtireString90( he , 0,90,0);
		//EPD_init(); 	///EPD init
		PIC_display(dispRam,NULL);//EPD_picture1
		EPD_refresh();//EPD_refresh
		//EPD_sleep();//EPD_sleep,Sleep instruction is necessary, please do not delete!!!
		vTaskDelay(20000/ portTICK_PERIOD_MS);


	};
}


void GPIO_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);


	 //CS-->PA8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11|GPIO_Pin_12;		//Port configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//SCK-->PC11  SDO--->PC10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;		//Port configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);


	 // D/C--->PA11	   RES-->PA12
	/*GPIO_InitStructure.GPIO_Pin = ;		//Port configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);*/

	// BUSY--->PA13
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//Pull up input
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);				//Initialize GPIO

	/* //LED
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;		//Port configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);*/
}

void setPix(uint32_t x, uint32_t y)
{
	dispRam[x/8+y*16]&=~(1<<(7-((x)%8)));
}

void clearPix(uint32_t x, uint32_t y)
{
	dispRam[(x/8+(y*16))]|=1<<(7-(x%8));

}

void clearDispRAM(void)
{

	for (int x=0; x< 4736; x++)
	{
		dispRam[x]=0xff;
	}
}

void wtireString( char*str ,  uint32_t x, uint32_t y,  uint32_t mode)
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
				if (mode==0) clearPix(cx,cy); else setPix(cx,cy);
			}
			if ( ( ((by>>bc)&0x01) == 1 ) &&((cy-y)<=FONT_H))
			{
				if (mode==0) setPix(cx,cy); else clearPix(cx,cy);
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

void wtireString90( char*str ,  uint32_t x, uint32_t y,  uint32_t mode)
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
				if (mode==0) clearPix(cy,295-cx); else setPix(cy,cx);
			}
			if ( ( ((by>>bc)&0x01) == 1 ) &&((cy-y)<=FONT_H))
			{
				if (mode==0) setPix(cy,295-cx); else clearPix(cy,cx);
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

void SPI_Delay(unsigned char xrate)
{
	unsigned char i;
	while(xrate)
	{
		for(i=0;i<EPD_W21_SPI_SPEED;i++);
		xrate--;
	}
}


void SPI_Write(unsigned char value)
{
    unsigned char i;


	SPI_Delay(1);
    for(i=0; i<8; i++)
    {
        EPD_W21_CLK_0;
		SPI_Delay(1);
        if(value & 0x80)
        	EPD_W21_MOSI_1;
        else
        	EPD_W21_MOSI_0;
        value = (value << 1);
		SPI_Delay(1);
		driver_delay_us(1);
        EPD_W21_CLK_1;
        SPI_Delay(1);
    }
}

void EPD_W21_WriteCMD(unsigned char command)
{
    SPI_Delay(1);
    EPD_W21_CS_0;
	EPD_W21_DC_0;		// command write
	SPI_Write(command);
	EPD_W21_CS_1;
}

void EPD_W21_WriteDATA(unsigned char command)
{
    SPI_Delay(1);
    EPD_W21_CS_0;
	EPD_W21_DC_1;		// command write
	SPI_Write(command);
	EPD_W21_CS_1;
}

void EPD_W21_Init(void)
{
	EPD_W21_RST_0;		// Module reset
	driver_delay_xms(1000);//At least 10ms delay
	EPD_W21_RST_1;
	driver_delay_xms(1000);//At least 10ms delay

}

void EPD_init(void)
{
	  HRES=0x80;						//128
	  VRES_byte1=0x01;			//296
	  VRES_byte2=0x28;

		EPD_W21_Init(); //Electronic paper IC reset

		EPD_W21_WriteCMD(0x06);         //boost soft start
		EPD_W21_WriteDATA (0x17);		//A
		EPD_W21_WriteDATA (0x17);		//B
		EPD_W21_WriteDATA (0x17);		//C

		EPD_W21_WriteCMD(0x04);
		lcd_chkstatus();//waiting for the electronic paper IC to release the idle signal

		EPD_W21_WriteCMD(0x00);			//panel setting
		EPD_W21_WriteDATA(0x0f);		//LUT from OTP£¨128x296
	  EPD_W21_WriteDATA(0x0d);    //waiting for the electronic paper IC to release the idle signal

		EPD_W21_WriteCMD(0x61);			//resolution setting
		EPD_W21_WriteDATA (HRES);
		EPD_W21_WriteDATA (VRES_byte1);
		EPD_W21_WriteDATA (VRES_byte2);

		EPD_W21_WriteCMD(0X50);			//VCOM AND DATA INTERVAL SETTING
		EPD_W21_WriteDATA(0x77);		//WBmode:VBDF 17|D7 VBDW 97 VBDB 57		WBRmode:VBDF F7 VBDW 77 VBDB 37  VBDR B7
}
void EPD_refresh(void)
{
		EPD_W21_WriteCMD(0x12);			//DISPLAY REFRESH
		driver_delay_xms(100);	        //!!!The delay here is necessary, 200uS at least!!!
		lcd_chkstatus();
}
void EPD_sleep(void)
{
		EPD_W21_WriteCMD(0X02);  	//power off
		lcd_chkstatus();//waiting for the electronic paper IC to release the idle signal
		EPD_W21_WriteCMD(0X07);  	//deep sleep
		EPD_W21_WriteDATA(0xA5);
}



void PIC_display(const unsigned char* picData_old,const unsigned char* picData_new)
{
    unsigned int i;
    //uint8_t  u=1;
		EPD_W21_WriteCMD(0x10);	       //Transfer old data
	  for(i=0;i<4736;i++)
	{
	  EPD_W21_WriteDATA(*picData_old);
	  picData_old++;
	}
		EPD_W21_WriteCMD(0x13);		     //Transfer new data
	  for(i=0;i<4736;i++)
	{
	  EPD_W21_WriteDATA(255);
	}

}

void PIC_display_Clean(void)
{
    unsigned int i;
		EPD_W21_WriteCMD(0x10);	       //Transfer old data
	  for(i=0;i<4736;i++)
	{
	  EPD_W21_WriteDATA(0xff);
	}

		EPD_W21_WriteCMD(0x13);		     //Transfer new data
	  for(i=0;i<4736;i++)
	{
	  EPD_W21_WriteDATA(0xff);

}
}


void lcd_chkstatus(void)
{
	unsigned char busy;
	do
	{
		EPD_W21_WriteCMD(0x71);
		busy = isEPD_W21_BUSY;
		busy =!(busy & 0x01);
	}
	while(busy);
	driver_delay_xms(200);
}

void driver_delay_us(unsigned int xus)  //1us
{
	for(;xus>1;xus--);
}

void driver_delay_xms(unsigned long xms) //1ms
{
    unsigned long i = 0 , j=0;

    for(j=0;j<xms;j++)
	{
        for(i=0; i<256; i++);
    }
}
void DELAY_S(int delaytime)     //  1s
{
	int i,j,k;
	for(i=0;i<delaytime;i++)
  {
		for(j=0;j<4000;j++)
		{
			for(k=0;k<222;k++);

		}
	}
}
void DELAY_M( int delaytime)     //  1M
{
	int i;
	for(i=0;i<delaytime;i++)
		DELAY_S(60);
}
