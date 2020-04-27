/*
 * i2c_driver.cpp
 *
 *  Created on: 21 апр. 2020 г.
 *      Author: Иван
 */


#include "i2c_driver.h"

i2c_driver_class::i2c_driver_class(uint32_t i2c_base_addr)
{
this->i2c_port=(I2C_TypeDef*)i2c_base_addr;

	// port defenition hw specific test
	RCC->APB2ENR|=RCC_APB2ENR_IOPBEN;
	RCC->APB1ENR|=RCC_APB1ENR_I2C1EN;
	GPIOB->CRL&=0x00FFFFFF;
	GPIOB->CRL|=0xDD000000;


i2c_port->CR1 = I2C_CR1_SWRST;
i2c_port->CR1=0;
i2c_port->CR2=28;
i2c_port->CCR=140;
i2c_port->TRISE=29;
i2c_port->CR2|=I2C_CR2_ITERREN |I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN;
i2c_port->CR1|=I2C_CR1_PE;

}

void i2c_driver_class::I2C_ISR()
{
	uint32_t sr;
	sr=i2c_port->SR1;
	if (sr & I2C_SR1_ADDR)
	{

	} else if(sr&I2C_SR1_BTF)
	{

	} else if(sr&I2C_SR1_ADD10)
	{

	} else if(sr&I2C_SR1_RXNE)
	{

	} else if(sr& I2C_SR1_TXE)
	{

	};



}
void i2c_driver_class::I2C_ERR_ISR()
{
	uint32_t sr;
	sr=i2c_port->SR1;
	if (sr&I2C_SR1_BERR)
	{

	}
	else if (sr&I2C_SR1_ARLO)
	{

	}
	else if (sr&I2C_SR1_AF)
	{

	}
	else if (sr&I2C_SR1_OVR)
	{

	};
}


