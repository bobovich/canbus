

/* Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include <stddef.h>
#include <stdio.h>

#include "stm32f10x.h"
#include "CC1101.h"
#include "uart_com.h"
#include "i2c_sensor.h"
#include "string.h"
#include <cstdlib>
#define DEBUG
#define FREERTOS
//#define FREERTOS

void prvClockCoreInit (void);
void prvCommunicationInit(void);

// application defenition

void ATaskCanBus (void *pvParameters);
void run1Task(void *pvParameters);
void run2Task(void *pvParameters);
static  xTaskParam  RTask1 =
{
		.pTaskSerial= SPI1_BASE,
		.xTaskPortH=PORT_NORMAL,
		.pRxEvent=	(uint32_t*)(PERIPH_BB_BASE + ((GPIOC_BASE-PERIPH_BASE+0x08)  * 32) + (4 * 4)),
		.pTxcEvent=	(uint32_t*)(PERIPH_BB_BASE + ((GPIOC_BASE-PERIPH_BASE+0x08)  * 32) + (4 * 4))
};
static  xTaskParam  RTask2 =
{
		.pTaskSerial= SPI2_BASE,
		.xTaskPortH=PORT_NORMAL,
		.pRxEvent=	(uint32_t*)(PERIPH_BB_BASE + ((GPIOC_BASE-PERIPH_BASE+0x08)  * 32) + (6 * 4))
};
static pQueueComm pQComm;
QueueHandle_t sQueue;



int main(void)
{
	prvClockCoreInit();
	//prvCommunicationInit();
	GPIOC->CRL|= 0x4<<16;
	RTask1.xCommRX=xQueueCreate(QUEUE_SIZE, sizeof(pack));
	RTask1.xCommTX=xQueueCreate(QUEUE_SIZE, sizeof(pack));
	RTask2.xCommRX=xQueueCreate(QUEUE_SIZE, sizeof(pack));
	RTask2.xCommTX=xQueueCreate(QUEUE_SIZE, sizeof(pack));
	pQComm.a1RX=RTask1.xCommRX;
	pQComm.a1TX = RTask1.xCommTX;
	pQComm.a2RX = RTask2.xCommRX;
	pQComm.a2TX = RTask2.xCommTX;
	pQComm.qSensor= xQueueCreate(3, sizeof(air_condition));

//xTaskCreate(ATaskCanBus, "CAN Task",  100, NULL, tskIDLE_PRIORITY,  NULL);
	//xTaskCreate(ARadioTask, "RF Task1",  500,(void*) &RTask1 ,2,  NULL);
	//xTaskCreate(ARadioTaskS, "RF Task2",  500,(void*) &RTask2 ,2,  NULL);
	//delete RTask1;
	xTaskCreate(aIAQCore, "TaskSensor",  100, (void*)pQComm.qSensor, 2,  NULL);
	xTaskCreate(run1Task, "Run2 Task",  100, NULL,2,  NULL);
	xTaskCreate(aTaskUart, "Run2 Task",  300, &pQComm, 2,  NULL);
	vTaskStartScheduler();

  while (1)
  {
		   //error loop  hook? make harvesting errors and reboots
  }
}



//core  clock init function, this a critical function
void prvClockCoreInit (void)
{
	FLASH->ACR |= FLASH_ACR_LATENCY_2;// flash latency delay 2t
	FLASH->ACR |= FLASH_ACR_PRFTBE;// Perfech enable
	RCC->CFGR|=(0xC<<18);// set mul x14
	RCC->CFGR&=~0x00010000; // set pll clk hsi div 2
	RCC->CFGR&=0xfffffffc;
	RCC->CFGR|=0x2;// use pll1 from clock
	RCC->CR|=0x1000000;// enable pll
	RCC->CFGR|=(0<<13); //APB2 no div
	RCC->CFGR|=(4<<10);  // APB1 /2 28  MHz
	RCC->CFGR|=(0<<7); // AHB no div
	while(!(RCC->CR & RCC_CR_PLLRDY));// wait stabilize pll
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL));//56 Mhz

	/*RCC->APB1ENR=RCC_APB1ENR_CAN1EN;
		RCC->APB2ENR=RCC_APB2ENR_TIM1EN|RCC_APB2ENR_USART1EN;
		RCC->APB2ENR=RCC_APB2ENR_AFIOEN|RCC_APB2ENR_IOPAEN|RCC_APB2ENR_IOPBEN|RCC_APB2ENR_IOPCEN|
		RCC_APB2ENR_TIM1EN|RCC_APB2ENR_ADC1EN;// enable clk pereph here
		*/
}

void prvCommunicationInit(void)
{
	//RCC->APB1ENR|=RCC_APB1ENR_CAN1EN; //CAN clk EN
	//RCC->APB2ENR|=RCC_APB2ENR_USART1EN; //UART1 clk EN
	// uart init definition

	//USART1->BRR=((0x1e<<4)|4);//115200
	//USART1->CR1|=USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

	// CAN bus definition
	//CAN1->MCR=0;

    //SPI Definition
	/*RCC->APB2ENR|=RCC_APB2ENR_SPI1EN  ;
	SPI1->CR1=0;
	SPI1->CR1=(0x6<<3)|SPI_CR1_MSTR; // clock div 128(56M/128), SPI is a master mode
	SPI1->CR1|=SPI_CR1_SPE;*/

}

void run1Task(void *pvParameters)
{
	int x,y;
	for(;;)
	{
		x++;
		y=x+y;
		if (x>1000000) x=0;
		if (y>999999)y=0;
	};

}

void run2Task(void *pvParameters)
{
	int x,y;
	for(;;)
	{
		x++;
		y=x+y;
		if (x>1000000) x=0;
		if (y>999999)y=0;
	};
}

