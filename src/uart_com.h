
#ifndef UART_COM_H_
#define UART_COM_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "CC1101.h"
#include "i2c_sensor.h"

void aTaskUart(void * pvParameters);
void printUart(char * str);
char * ftoa(double f, uint8_t w, char * buf);
struct pQueueComm
{
	QueueHandle_t a1TX;
	QueueHandle_t a1RX;
	QueueHandle_t a2TX;
	QueueHandle_t a2RX;
	QueueHandle_t qSensor;
	QueueHandle_t qDisplay;
};


#endif /* UART_COM_H_ */
