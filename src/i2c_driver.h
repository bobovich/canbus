/*
 * i2c_driver.h
 *
 *  Created on: 21 апр. 2020 г.
 *      Author: Иван
 */

#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_

#include "stm32f10x.h"


#define WRITE_MODE	1
#define WRITE_TO_ADDR_MODE	12
#define READ_MODE	2
#define READ_FROM_ADDR_MODE	21

#define BUS_BUSY	200
#define BUS_ERROR	201
#define BUS_OK	0


#define I2C_BUFFER_SIZE	5


struct i2c_struct_com
{
	uint8_t mode;
	uint8_t address;
	uint8_t regAddr;
	uint8_t dataSize;
	uint8_t data[I2C_BUFFER_SIZE];
};


class i2c_driver_class
{
private:
	i2c_struct_com* i2cBuffer;
	uint8_t bState;
	uint8_t ev;
	I2C_TypeDef* i2c_port;
	void I2C_ISR(void);
	void I2C_ERR_ISR(void);

public:
	i2c_driver_class(uint32_t i2c_base_addr);
	uint32_t communicate (i2c_struct_com * dataBuffer);


};

extern "C"
{

/*void I2C_ISR(void)
{

};

void I2C_ERR_ISR(void)
{

}
*/
}
#endif /* I2C_DRIVER_H_ */
