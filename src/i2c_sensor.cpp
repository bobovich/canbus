/*
 * i2c_sensor.cpp
 *
 *  Created on: 1 апр. 2020 г.
 *      Author: Иван
 */


#include "i2c_sensor.h"
#include "bmp180.h"
#include "i2c_driver.h"

//#define WORK
extern i2c_driver_class* iic;

void aIAQCore(void *parameter)
{

	//iaq_core* iaq = new iaq_core();
	//ens210_class* ens210 = new ens210_class();
	air_condition air;
	//int32_t temp, press;
	i2c_struct_com i2b;
	QueueHandle_t comQueue= (QueueHandle_t) parameter;
	//bmp180_t *bmp= new bmp180_t;
	vTaskDelay(100/ portTICK_PERIOD_MS);
	//iaq->i2c_init();
	//ens210->i2c_init();

	//ens210->sens_init();




#ifndef WORK


	NVIC_EnableIRQ(I2C1_EV_IRQn );
	NVIC_EnableIRQ(I2C1_ER_IRQn );

	/*
	 * begin init ens210
	 */
	i2b.address=ENS210_ADDRESS;
	iic->i2cBuffer=&i2b;
	i2b.mode=WRITE_TO_ADDR_MODE;
	i2b.regAddr=SYS_CTRL;
	i2b.data[0]= 0x80;
	i2b.dataSize=1;
	iic->I2C_ISR();
	while (iic->Status()!=BUS_OK);
	vTaskDelay(10/ portTICK_PERIOD_MS);

	i2b.regAddr=SYS_CTRL;
	i2b.data[0]= 0x00;
	i2b.dataSize=1;
	iic->I2C_ISR();
	while (iic->Status()!=BUS_OK);
	vTaskDelay(10/ portTICK_PERIOD_MS);

	i2b.regAddr=SENS_RUN;
	i2b.data[0]= 0x03;
	i2b.dataSize=1;
	iic->I2C_ISR();
	while (iic->Status()!=BUS_OK);
	vTaskDelay(10/ portTICK_PERIOD_MS);

	i2b.regAddr=SENS_START;
	i2b.data[0]= 0x03;
	i2b.dataSize=1;
	iic->I2C_ISR();
	while (iic->Status()!=BUS_OK);
	vTaskDelay(10/ portTICK_PERIOD_MS);

#else
	NVIC_DisableIRQ(I2C1_EV_IRQn );
#endif

	//bmp180_init(bmp);

	while (1)
	{

#ifdef WORK
			vTaskDelay(3000/ portTICK_PERIOD_MS);
			iaq->hookRecievePack();
			ens210->appHook();
			air.CO2=iaq->getCO2();
			air.TVOC=iaq->getTVOC();
			air.temp=ens210->getTemp();
			air.humidity=ens210->getHumidity();
#else
			vTaskDelay(3000/ portTICK_PERIOD_MS);
			//ENS210 READ
			i2b.address=ENS210_ADDRESS;
			i2b.dataSize=T_VAL_SIZE;
			//i2b.data[0]=0x03;
			i2b.regAddr=T_VAL;//0x30;
			i2b.mode=READ_FROM_ADDR_MODE;
			iic->i2cBuffer=&i2b;
			iic->I2C_ISR();
			while (iic->Status()!=BUS_OK);
			if (i2b.dataSize)
			{
				air.temp=( (double)(( i2b.data[1]<<8) | i2b.data[0] ) )/64-273.15;
			}
			vTaskDelay(10/ portTICK_PERIOD_MS);
			i2b.dataSize=H_VAL_SIZE;
			i2b.regAddr=H_VAL;//0x30;
			i2b.mode=READ_FROM_ADDR_MODE;
			iic->i2cBuffer=&i2b;
			iic->I2C_ISR();
			while (iic->Status()!=BUS_OK);
			if (i2b.dataSize)
			{
			air.humidity=((double)((i2b.data[1]<<8) | i2b.data[0]))/512;
			}
			vTaskDelay(30/ portTICK_PERIOD_MS);
			i2b.address=ENS210_ADDRESS;
			i2b.dataSize=SENS_START_SIZE;
			i2b.data[0]=0x03;
			i2b.regAddr=SENS_START;
			i2b.mode=WRITE_TO_ADDR_MODE;
			iic->i2cBuffer=&i2b;
			iic->I2C_ISR();
			while (iic->Status()!=BUS_OK);
			vTaskDelay(30/ portTICK_PERIOD_MS);
			//IAQ CORE READ
			i2b.address=IAQ_C_ADDRESS;
			i2b.dataSize=9;
			i2b.mode=READ_MODE;
			iic->i2cBuffer=&i2b;
			iic->I2C_ISR();
			while (iic->Status()!=BUS_OK);
			if (i2b.dataSize)
			{
			air.CO2=i2b.data[0]*256+ i2b.data[1];
			air.TVOC=i2b.data[7]*256+ i2b.data[8];
			}

#endif

			if (uxQueueSpacesAvailable(comQueue))
			{
				xQueueSend(comQueue, &air , 1);
			}
	}



}

iaq_core::iaq_core()
{
	i2c_init();
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

uint32_t iaq_core::hookRecievePack()
{
	this->readI2C();
	sVal.co2=this->iaq_pack[0]*256+ this->iaq_pack[1];
	sVal.tvoc=this->iaq_pack[7]*256+ this->iaq_pack[8];
	return 1;

}

uint32_t iaq_core::getCO2(void)
{
	return sVal.co2;
}

uint32_t iaq_core::getTVOC(void)
{
	return sVal.tvoc;
}

ens210_class::ens210_class(void)
{
	i2c_init();
	return;
}

void ens210_class::i2c_init(void)
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
}

uint32_t ens210_class::readI2C(uint8_t saddr, uint8_t len)
{
	uint8_t c=0;
	if (len<0 || len > 10)  return 0;
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
	I2C1->DR=this->addr; // write address to read
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	if (I2C1->SR2);
	I2C1->DR=saddr;
	//I2C1->CR1|=I2C_CR1_STOP;
	while (!(I2C1->SR1 & I2C_SR1_BTF)); //?
	I2C1->CR1|=I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB));
	I2C1->DR=this->addr|0x01; // address  read mode
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	if (I2C1->SR2);
	while (c < len)
	{
		if (c==(len-1))I2C1->CR1&=~I2C_CR1_ACK;
		while (!(I2C1->SR1&I2C_SR1_RXNE));

		this->buffer[c]=I2C1->DR;
		c++;
		if (c==len)
		{
			I2C1->CR1|=I2C_CR1_STOP;
		}


	};
	return 1;
}

uint32_t ens210_class::writeI2C(uint8_t saddr, uint8_t len)
{
	uint8_t c=0;
	if ( (len<0 || len > 10) )  return 0;
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
	I2C1->DR=this->addr; // write address
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	if (I2C1->SR2);
	I2C1->DR=saddr;
	while (!(I2C1->SR1 & I2C_SR1_BTF));
	while (c < len)
	{
		while (!(I2C1->SR1&I2C_SR1_BTF));
		I2C1->DR=this->buffer[c];
		c++;
		if (c==len)
		{
			//I2C1->CR1&=~I2C_CR1_ACK;
			I2C1->CR1|=I2C_CR1_STOP;
		}
	};
	return 1;
}

uint32_t ens210_class::sens_init(void)
{
	buffer[0]= 0x80;
	writeI2C(SYS_CTRL);
	vTaskDelay(10/ portTICK_PERIOD_MS);
	buffer[0]= 0x00;
	writeI2C(SYS_CTRL);
	buffer[0]= 0x03;
	writeI2C(SENS_RUN);
	writeI2C(SENS_START);
	return 1;
}

uint32_t ens210_class::appHook(void)
{
	//
	//
	readI2C(T_VAL, T_VAL_SIZE);
	temp= ( (double)(( buffer[1]<<8) | buffer[0] ) )/64-273.15;
	vTaskDelay(10/ portTICK_PERIOD_MS);
	readI2C(H_VAL, H_VAL_SIZE);
	hum=((double)((buffer[1]<<8) | buffer[0]))/512;
	//readI2C(SENS_STAT);
	buffer[0]= 0x03;
	vTaskDelay(10/ portTICK_PERIOD_MS);
	writeI2C(SENS_START);
	return 1;
}

double ens210_class::getTemp()
{
	return this->temp;
}

double ens210_class::getHumidity()
{
	return this->hum;
}
