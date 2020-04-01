/*
 * i2c_sensor.h
 *
 *  Created on: 1 ���. 2020 �.
 *      Author: ����
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


class iaq_core
{

private:
	uint8_t iaq_pack[9];
	const uint8_t addr=0x5a;
public:
	uint32_t hookRecievePack( uint8_t* pData);
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
