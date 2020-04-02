/*
 * i2c_sensor.h
 *
 *  Created on: 1 апр. 2020 г.
 *      Author: Иван
 */

#ifndef I2C_SENSOR_H_
#define I2C_SENSOR_H_

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"

/*
 * definition about iAQ-Core sensor
 *
 */

void aIAQCore(void *parameter);

struct iaq_data {
	uint32_t co2;
	uint32_t tvoc;

};

class iaq_core
{

private:
	uint8_t iaq_pack[9];
	const uint8_t addr=0xb5;
	iaq_data sVal;
	uint8_t status;
public:
	iaq_core();
	void i2c_init(void);
	uint32_t readI2C(void);
	uint32_t hookRecievePack( QueueHandle_t qHandle);
	int8_t getTemp (void);
	uint8_t getHumiduty(void);
	uint8_t getStatus(void);
	uint32_t getCO2(void);
	uint32_t getTVOC(void);
};




struct air_condition
{
	uint32_t CO2;
	uint32_t TVOC;
	int32_t temp;
	uint32_t humiduty;
};

#endif /* I2C_SENSOR_H_ */
