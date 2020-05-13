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
i2c_port->CR2|=I2C_CR2_ITERREN |I2C_CR2_ITEVTEN;
i2c_port->CR1|=I2C_CR1_PE;


}

uint8_t i2c_driver_class::Status()
{
	return this->bState;
}

void i2c_driver_class::I2C_ISR()//is not work need re-send address for read from address
{
	uint32_t sr;
	static uint32_t i=0;
if (this->ev == EV_IDLE)
{
	if (!(i2c_port->SR2 & I2C_SR2_BUSY))
	{
		if ((i2cBuffer->dataSize == 0) || (this->i2cBuffer == 0 ) )
		{
			I2C1->CR1|=I2C_CR1_STOP;
			this->ev=EV_IDLE;
			i=0;
			this->i2cBuffer=0;
			i2c_port->CR2&=0x00FF;
			this->bState=BUS_OK;
			return;
		}
		i2c_port->CR2|=I2C_CR2_ITERREN |I2C_CR2_ITEVTEN;
		this->ev=EV_BUSY_TO_BUS;
		this->bState=BUS_BUSY;
		i2c_port->CR1|=I2C_CR1_START;
	}
	return;
};

	sr=i2c_port->SR1;
	if(i2c_port->SR2);
	if (sr & I2C_SR1_SB)
	{

		if (this->ev == EV_DO_RESEND_ADDR_REC)
		{
			i2c_port->DR=(this->i2cBuffer->address<<1)|1;
			return;
		}

		else
		{
			this->ev=EV_START_COMPLET;
			if (i2cBuffer->mode == READ_MODE)
			{
				i2c_port->DR=(this->i2cBuffer->address<<1)|1;
			}else
			{
				i2c_port->DR=(this->i2cBuffer->address<<1);
			};
		};
		return;
	}
	else if (sr & I2C_SR1_ADDR)
	{
		if (this->ev == EV_DO_RESEND_ADDR_REC)
		{
			this->ev=EV_DO_FOLLOW;
			i2c_port->CR2|=I2C_CR2_ITBUFEN; //enable buffer irq
			if (i2cBuffer->dataSize > I2C_BUFFER_SIZE )
			{
				i2cBuffer->dataSize = I2C_BUFFER_SIZE; // buffer size control
			}
			I2C1->CR1|=I2C_CR1_ACK;
			this->ev=EV_DO_RECEIVE;
			return;
		}
		else
		{
			this->ev= EV_SEND_ADDRES_COMPLET;
			//if direct write
			if (i2cBuffer->mode == WRITE_MODE )
			{
				this->ev=EV_DO_FOLLOW; //if direct read or write, mode skip send register address
				i2c_port->CR2|=I2C_CR2_ITBUFEN; //enable buffer irq
				this->ev=EV_DO_TRANSMIT;
				if (i2cBuffer->dataSize > I2C_BUFFER_SIZE )
				{
					i2cBuffer->dataSize = I2C_BUFFER_SIZE; // buffer size control
				}
				return;
			}else
			//if direct recieve
			if ((i2cBuffer->mode == READ_MODE))
			{
				this->ev=EV_DO_RECEIVE;
				I2C1->CR1|=I2C_CR1_ACK;
				i2c_port->CR2|=I2C_CR2_ITBUFEN;
				if (i2cBuffer->dataSize > I2C_BUFFER_SIZE )
				{
					i2cBuffer->dataSize = I2C_BUFFER_SIZE;
				};
				return;
			};
			/*else
			if ((i2cBuffer->mode == WRITE_TO_ADDR_MODE)||(i2cBuffer->mode == READ_FROM_ADDR_MODE))
			{
				i2c_port->CR1|=I2C_CR1_START;
				this->ev=EV_SW_TO_RECEIVER;
				return;
			};*/
			i2c_port->DR=i2cBuffer->regAddr;
			this->ev=EV_SEND_REG_ADDR;
			return;
		};
	}else if (sr & I2C_SR1_ADD10)
	{

	};

	if(sr & I2C_SR1_BTF)
	{
		if (this->ev==EV_SEND_REG_ADDR )
		{

			if (i2cBuffer->mode == READ_FROM_ADDR_MODE )
			{
				i2c_port->CR1|=I2C_CR1_START;
				this->ev=EV_DO_RESEND_ADDR_REC;
				return;
			}else
			{
				this->ev=EV_DO_FOLLOW;
				i2c_port->CR2|=I2C_CR2_ITBUFEN; //enable buffer irq
				if (i2cBuffer->dataSize > I2C_BUFFER_SIZE )
				{
					i2cBuffer->dataSize = I2C_BUFFER_SIZE; // buffer size control
				}
				this->ev=EV_DO_TRANSMIT;
				return;
			}
		};
	}

	if(sr & I2C_SR1_RXNE)
	{
		//if (i2c_port->SR2);
		if (i < i2cBuffer->dataSize)
		{
			if (i == (uint32_t)(i2cBuffer->dataSize - 2))
			{
				i2c_port->CR1&=~I2C_CR1_ACK;
			};
			this->i2cBuffer->data[i]=i2c_port->DR;
			i++;
		};
		if (i==i2cBuffer->dataSize)
		{
			i2c_port->CR1|=I2C_CR1_STOP;
			this->ev=EV_IDLE;
			i2c_port->CR2&=0x00FF;
			i=0;
			this->bState=BUS_OK;

		};
		return;
	}
	if((sr & I2C_SR1_TXE)|(sr & I2C_SR1_BTF))
	{
		if (this->ev == EV_DO_TRANSMIT )
		{
			i2c_port->DR = this->i2cBuffer->data[i];
			i++;
			if (i==i2cBuffer->dataSize)
			{
				I2C1->CR1|=I2C_CR1_STOP;
				this->ev=EV_IDLE;
				i=0;
				i2c_port->CR2&=0x00FF;
				this->bState=BUS_OK;
				return;
			};
		};
	};






///void dofollow (void)
/*
	i2c_port->CR2|=I2C_CR2_ITBUFEN; //enable buffer irq
	if (i2cBuffer->dataSize > I2C_BUFFER_SIZE )
	{
		i2cBuffer->dataSize = I2C_BUFFER_SIZE; // buffer size control
	}
	I2C1->CR1|=I2C_CR1_ACK;
	if ((i2cBuffer->mode == WRITE_MODE)||(i2cBuffer->mode == WRITE_TO_ADDR_MODE))
	{
			this->ev=EV_DO_TRANSMIT;
	};
	if ((i2cBuffer->mode == READ_MODE)||(i2cBuffer->mode == READ_FROM_ADDR_MODE))
	{
		this->ev=EV_DO_RECEIVE;
	};

*/

};


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


