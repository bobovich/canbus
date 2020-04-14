/*
 * eink_display.cpp
 *
 *  Created on: 12 àïð. 2020 ã.
 *      Author: Èâàí
 */

#include "eink_display.h"
#include "Ap_29demo.h"

unsigned int size;
unsigned char HRES,VRES_byte1,VRES_byte2;
void GPIO_Configuration(void);
void displayTask(void* pvParams)
{
	GPIO_Configuration();
	vTaskDelay(3000/ portTICK_PERIOD_MS);
	 EPD_init(); //EPD init
	 PIC_display(gImage_black1,gImage_red1);//EPD_picture1
	 EPD_refresh();//EPD_refresh
	 EPD_sleep();//EPD_sleep,Sleep instruction is necessary, please do not delete!!!
	 DELAY_S(60);
	 EPD_init(); //EPD init
	 PIC_display(gImage_black2,gImage_red2);//EPD_picture2
	 EPD_refresh();//EPD_refresh
	 EPD_sleep();//EPD_sleep,Sleep instruction is necessary, please do not delete!!!
	 DELAY_S(60);
	 EPD_init(); //EPD init
	PIC_display(gImage_black3,gImage_red3);//EPD_picture2
	EPD_refresh();//EPD_refresh
	EPD_sleep();//EPD_sleep,Sleep instruction is necessary, please do not delete!!!
	DELAY_S(60);
			//EPD_Clean
	 EPD_init(); //EPD init
	 PIC_display_Clean();//EPD_Clean
	 EPD_refresh();//EPD_refresh
	 EPD_sleep();//EPD_sleep,Sleep instruction is necessary, please do not delete!!!
	 DELAY_S(60);



	for(;;)
	{



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
		EPD_W21_WriteDATA(0x0f);		//LUT from OTP£¬128x296
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
    uint8_t  u=1;
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
	 /* u++;
	  if (u==255) u=0;*/
	  //picData_new++;
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
