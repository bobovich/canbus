
#ifndef UART_COM_H_
#define UART_COM_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "CC1101.h"
#include "i2c_sensor.h"

void aTaskUart(void * pvParameters);
struct pQueueComm
{
	QueueHandle_t a1TX;
	QueueHandle_t a1RX;
	QueueHandle_t a2TX;
	QueueHandle_t a2RX;
	QueueHandle_t qSensor;
};


#endif /* UART_COM_H_ */
