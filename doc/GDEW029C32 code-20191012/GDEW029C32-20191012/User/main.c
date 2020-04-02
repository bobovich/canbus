#include "stm32f10x.h"
#include "Display_EPD_W21_spi.h"
#include "Display_EPD_W21.h"
#include "Ap_29demo.h"	

ErrorStatus HSEStartUpStatus;


unsigned int size;
unsigned char HRES,VRES_byte1,VRES_byte2;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);

//EPD
void EPD_init(void);
void PIC_display(const unsigned char* picData_old,const unsigned char* picData_new);
void EPD_sleep(void);
void EPD_refresh(void);
void lcd_chkstatus(void);
void PIC_display_Clean(void);
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
void DELAY_S(unsigned int delaytime)     //  1s
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
void DELAY_M(unsigned int delaytime)     //  1M
{
	int i;
	for(i=0;i<delaytime;i++)
		DELAY_S(60);
}
/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
#define NVIC_VectTab_FLASH  misc.h
*******************************************************************************/

//Tips//
/*When the electronic paper is refreshed in full screen, the picture flicker is a normal phenomenon, and the main function is to clear the display afterimage in the previous picture.
  When the local refresh is performed, the screen does not flash.*/
/*When you need to transplant the driver, you only need to change the corresponding IO. The BUSY pin is the input mode and the others are the output mode. */
int	main(void)
{
//	unsigned int i;
	RCC_Configuration();
	//GPIO setting
	GPIO_Configuration();

	while(1)	   
	{
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

		//EPD_Clean
		EPD_init(); //EPD init
	  PIC_display_Clean();//EPD_Clean
		EPD_refresh();//EPD_refresh		
		EPD_sleep();//EPD_sleep,Sleep instruction is necessary, please do not delete!!!
		DELAY_S(60);	
		while(1);
	}
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
		EPD_W21_WriteCMD(0x10);	       //Transfer old data
	  for(i=0;i<4736;i++)	     
	{
	  EPD_W21_WriteDATA(*picData_old);
	  picData_old++;
	}
		EPD_W21_WriteCMD(0x13);		     //Transfer new data
	  for(i=0;i<4736;i++)	     
	{
	  EPD_W21_WriteDATA(*picData_new);
	  picData_new++;
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
/***********************************************************
						end file
***********************************************************/

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
 
  // Reset RCC clock configuration
  RCC_DeInit();
 
  // Enable external crystal
  RCC_HSEConfig(RCC_HSE_ON);
  
  // Waiting for the external crystal to stabilize
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  if(HSEStartUpStatus == SUCCESS)
  {
    // Set the phase-locked loop frequency PLLCLK = 8MHz * 9 = 72 MHz
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
  }
  else {
    // Enable internal crystal
    RCC_HSICmd(ENABLE);
    // Waiting for the internal crystal to stabilize
    while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

    // Set the phase-locked loop frequency PLLCLK = 8MHz/2 * 16 = 64 MHz 
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_16);
  }

    // Enable flash prefetch cache
  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

  //Set the code delay, FLASH_Latency_2 is two delay cycles
  FLASH_SetLatency(FLASH_Latency_2);
	
  //Set the system total clock
  RCC_HCLKConfig(RCC_SYSCLK_Div1); 

  //Set the high speed device total clock, RCC_HCLK_Div1 is the system clock divided by 1
  RCC_PCLK2Config(RCC_HCLK_Div1); 

  //Set the low speed device total clock, RCC_HCLK_Div2 is the system clock divided by 2
  RCC_PCLK1Config(RCC_HCLK_Div2);
  
  //Enable phase-locked loop multiplier
  RCC_PLLCmd(ENABLE);
  
  // Waiting for the frequency of the phase-locked loop to multiply after frequency stabilization
  while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
  
  // Select the phase-locked loop clock as the system clock
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  
  // Waiting for setup to complete
  while(RCC_GetSYSCLKSource() != 0x08);
    
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
            RCC_APB2Periph_USART1|RCC_APB2Periph_AFIO,
            ENABLE);

}

/*******************************************************************************
* Function name  : GPIO_Configuration
* Description         : Set the GPIO pin parameters used by the SPI serial port.
* Input        : None
* Output        : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE, ENABLE);
	  				     	
	
	 //CS-->PD8   SCK-->PD9  SDO--->PD10 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;		//Port configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 		
	GPIO_Init(GPIOD, &GPIO_InitStructure);	  	
	
	
	
	 // D/C--->PE15	   RES-->PE14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;		//Port configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 		
	GPIO_Init(GPIOE, &GPIO_InitStructure);	  				     		
	
	// BUSY--->PE13
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//Pull up input
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
 	GPIO_Init(GPIOE, &GPIO_InitStructure);				//Initialize GPIO
	
	 //LED
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;		//Port configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 		
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{ 
  //NVIC_InitTypeDef NVIC_InitStructure;
  ;
}


#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif






