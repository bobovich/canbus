/*
 * CC1101.cpp
 *
 *  Created on: 23 Feb 2020
 *      Author: Ivan Bobovich
 *
 *      This source licensed GPL3
 *
 *      Implementation of CC1101 SPI communication
 *
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "CC1101.h"
#include <stddef.h>
#include <stdio.h>
#include "stm32f10x.h"
#include "cc1101_conf.h"
#define NSS 4
#define MISO 6
//#define FREERTOS
/*
 * pack leng
 * Packet structure
 * |_PACK_LENGTH_|_ADDRESS_DST_|_ADDRESS_SRC_|_AXIS_1_|_AXIS_2_|_AXIS_3_|
 *
 * _AXIS_4_|_AXIS_5_|_AXIS_6_|_CRC_8_AXIS_ONLY|_RSSI_|//_HW_CRC_16 machine added and removed
 *
 *
 *
 *
 * */

void ARadioTask (void* pvParameters)
{
	xTaskParam * xPort=(xTaskParam *) pvParameters;
	cc11xx_class *cc1101= new cc11xx_class(xPort, 46, rfSettings);
#ifdef DEBUG
    RCC->APB2ENR|= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH|=0x2<<20;
    uint32_t* pt=(uint32_t*)(PERIPH_BB_BASE + ((GPIOC_BASE-PERIPH_BASE+0x0C)  * 32) + (13 * 4));
#endif
	//base task loop
    cc1101->sendByte(9,  87);
    cc1101->sendSTB(SCAL);
    cc1101->sendSTB(SRX);
	for (;;)
	{


#ifdef DEBUG
		if (*pt==0)
			*pt=1;
		else *pt=0;

#endif
		cc1101->chekStatus();
		cc1101->txEventHook();
		cc1101->rxEventHook();
		//vTaskDelay(50 / portTICK_PERIOD_MS);
	};
}

void ARadioTaskS (void* pvParameters)
{
	xTaskParam * xPort=(xTaskParam *) pvParameters;
	cc11xx_class *cc1101= new cc11xx_class(xPort, 46, rfSettings);
#ifdef DEBUG
   /* RCC->APB2ENR|= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH|=0x2<<20;
    uint32_t* pt=(uint32_t*)(PERIPH_BB_BASE + ((GPIOC_BASE-PERIPH_BASE+0x0C)  * 32) + (13 * 4));*/
#endif
    cc1101->sendByte(9,  88);
    cc1101->sendSTB(SCAL);
    cc1101->sendSTB(SRX);
   	for (;;)
	{
		cc1101->chekStatus();
		cc1101->txEventHook();
		cc1101->rxEventHook();
	//	vTaskDelay(50 / portTICK_PERIOD_MS);


	};
}
