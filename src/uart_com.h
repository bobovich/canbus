/*
 * uart_com.h
 *
 *  Created on: 18 ���. 2020 �.
 *      Author: ����
 */

#ifndef UART_COM_H_
#define UART_COM_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "CC1101.h"


void aTaskUart(void * pvParameters);
struct pQueueComm
{
	QueueHandle_t a1TX;
	QueueHandle_t a1RX;
	QueueHandle_t a2TX;
	QueueHandle_t a2RX;
};


#endif /* UART_COM_H_ */
