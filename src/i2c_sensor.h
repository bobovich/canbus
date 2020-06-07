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
#include "bmp180.h"

#define PART_ID				0x00
#define PART_ID_SIZE		2
#define UID					0x04
#define UID_SIZE			8
#define SYS_CTRL			0x10
#define SYS_CTRL_SIZE		1
#define SYS_STAT			0x11
#define SYS_STAT_SIZE		1
#define SENS_RUN			0x21
#define SENS_RUN_SIZE		1
#define SENS_START			0x22
#define SENS_START_SIZE		1
#define SENS_STOP			0x23
#define SENS_STOP_SIZE		1
#define SENS_STAT			0x24
#define SENS_STAT_SIZE		1
#define T_VAL				0x30
#define T_VAL_SIZE			3
#define H_VAL				0x33
#define H_VAL_SIZE			3
#define ENS210_ADDR			(0x43<<1)

#define ENS210_ADDRESS		0x43
#define IAQ_C_ADDRESS		0x5A
#define BMP180_ADDRESS		0x77

/*
 * definition about iAQ-Core sensor
 *
 */

void aIAQCore(void *parameter);

struct iaq_data {
	uint32_t co2;
	uint32_t tvoc;
};

class bmp180_class
{
private:
	int32_t 	X1;
	int32_t 	X2;
	int32_t 	X3;
	int32_t		B3;
	uint32_t	B4;
	int32_t		B5;
	int32_t		B6;
	uint32_t	B7;






public:
	int16_t 	AC1=408;
	int16_t 	AC2=-72;
	int16_t 	AC3=-14383;
	uint16_t	AC4=32741;
	uint16_t 	AC5=32757;
	uint16_t 	AC6=23153;
	int16_t 	B1=6190;
	int16_t		B2=4;
	int16_t 	MB=-32768;
	int16_t 	MC=-8711;
	int16_t 	MD=2868;
	int32_t 	UT;
	int32_t 	UP;
	int16_t		oss=0;
	int32_t		T;
	int32_t		p;
	void calc(void);

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
	uint32_t hookRecievePack(void);
	int8_t getTemp (void);
	uint8_t getHumiduty(void);
	uint8_t getStatus(void);
	uint32_t getCO2(void);
	uint32_t getTVOC(void);
};

class ens210_class
{
private:
	double temp;
	double hum;
	const uint8_t addr=ENS210_ADDR;
	volatile uint8_t buffer[10];
public:
	ens210_class(void);
	void i2c_init(void);
	uint32_t sens_init(void);
	uint32_t appHook(void);
	uint32_t readI2C(uint8_t saddr, uint8_t len=1);
	uint32_t writeI2C(uint8_t saddr, uint8_t len=1);
	double getTemp(void);
	double getHumidity(void);


};


struct air_condition
{
	uint32_t CO2;
	uint32_t TVOC;
	double temp;
	double humidity;
	double pressure;
	double temp2;
};

#endif /* I2C_SENSOR_H_ */
