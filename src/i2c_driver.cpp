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

	switch (this->ev)
	{
	case EV_IDLE:
		if (!(i2c_port->SR2 & I2C_SR2_BUSY))
		{
		i2c_port->CR1|=I2C_CR1_START;
		this->ev=EV_BUSY_TO_BUS;
		}
		this->bState=BUS_BUSY;
		break;
	case EV_START_COMPLET:
		if (i2cBuffer->mode < 10)
		{
		i2c_port->DR=(this->i2cBuffer->address<<1);
		} else if ((10 < i2cBuffer->mode) && (i2cBuffer->mode < 20))
		{
		i2c_port->DR=(this->i2cBuffer->address<<1)|1;
		}else
		{
			return ;
		}
		break;
	case EV_SEND_ADDRES_COMPLET:
		i2c_port->DR=i2cBuffer->regAddr;
		this->ev=EV_SEND_REG_ADDR;
		break;
	case EV_DO_FOLLOW:




		break;
	};

	sr=i2c_port->SR1;

	if (sr & I2C_SR1_SB)
	{
		this->ev=EV_START_COMPLET;
	}
	else if (sr & I2C_SR1_ADDR)
	{
		this->ev= EV_SEND_ADDRES_COMPLET;
	} else if(sr & I2C_SR1_BTF)
	{
		if (this->ev==EV_SEND_REG_ADDR )
		{
			this->ev=EV_DO_FOLLOW;
		}
	} else if(sr & I2C_SR1_ADD10)
	{

	} else if(sr & I2C_SR1_RXNE)
	{

	} else if(sr & I2C_SR1_TXE)
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


