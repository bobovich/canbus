/*
 * i2c_sensor.cpp
 *
 *  Created on: 1 апр. 2020 г.
 *      Author: Иван
 */


#include "i2c_sensor.h"


void aIAQCore(void *parameter)
{

	iaq_core* iaq = new iaq_core();
	QueueHandle_t comQueue= (QueueHandle_t) parameter;
	iaq->i2c_init();
	while (1)
	{
			vTaskDelay(3000/ portTICK_PERIOD_MS);
			iaq->hookRecievePack(comQueue);

	}



}

iaq_core::iaq_core()
{
	return;
}


void iaq_core::i2c_init(void)
{

	RCC->APB2ENR|=RCC_APB2ENR_IOPBEN;
	RCC->APB1ENR|=RCC_APB1ENR_I2C1EN;
	I2C1->CR1 = I2C_CR1_SWRST;
	I2C1->CR1=0;
	I2C1->CR2=28;
	I2C1->CCR=140;
	I2C1->TRISE=29;
	I2C1->CR1|=I2C_CR1_PE;

	GPIOB->CRL&=0x00FFFFFF;
	GPIOB->CRL|=0xDD000000;
	//GPIOB->ODR|=0x3<<6;
}


uint32_t iaq_core::readI2C(void)
{
	uint8_t c=0;
	if ((I2C1->SR1 & 0xf0) != 0)
		{
		I2C1->CR1 = I2C_CR1_SWRST;
		i2c_init();
		};
	if (I2C1->SR2&I2C_SR2_BUSY)
		{
		I2C1->CR1|=I2C_CR1_STOP;
		while (!(I2C1->SR2&I2C_SR2_BUSY));

		}


	I2C1->CR1|=I2C_CR1_START;
	I2C1->CR1|=I2C_CR1_ACK;
	while (!(I2C1->SR1 & I2C_SR1_SB));
	I2C1->DR=this->addr;
	while (!(I2C1->SR1 & I2C_SR1_ADDR));

	if (!(I2C1->SR2 & I2C_SR2_TRA) );
	while (c < 9)
	{
		while (!(I2C1->SR1&I2C_SR1_RXNE));
		this->iaq_pack[c]=I2C1->DR;
		c++;
		if (c==8)
		{
			I2C1->CR1&=~I2C_CR1_ACK;
			I2C1->CR1|=I2C_CR1_STOP;
		}
	};


	this->status=this->iaq_pack[2];
	return 1;
}

uint32_t iaq_core::hookRecievePack(QueueHandle_t qHandle)
{
	this->readI2C();
	sVal.co2=this->iaq_pack[0]*256+ this->iaq_pack[1];
	sVal.tvoc=this->iaq_pack[7]*256+ this->iaq_pack[8];
	if ( uxQueueSpacesAvailable( qHandle))
	{
		xQueueSend(qHandle, &sVal, 1);
		return 1;
	}
	else return 0;

}
