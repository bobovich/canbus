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
uint8_t bittwist (uint8_t b);
void aIAQCore(void *parameter)
{

	//iaq_core* iaq = new iaq_core();
	//ens210_class* ens210 = new ens210_class();
	air_condition air;
	bmp180_class bmp180;
	i2c_struct_com i2b;
	QueueHandle_t comQueue= (QueueHandle_t) parameter;
	vTaskDelay(100/ portTICK_PERIOD_MS);



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

	//begin read bmp180
	//
	i2b.address=BMP180_ADDRESS;
	i2b.regAddr=0xAA;
	i2b.mode=READ_FROM_ADDR_MODE;
	i2b.dataSize=24;
	iic->I2C_ISR();
	while (iic->Status()!=BUS_OK);
	if (i2b.dataSize)
	{
		bmp180.AC1=	(uint32_t)(i2b.data[0]<<8) | i2b.data[1];
		bmp180.AC2=	(uint32_t)(i2b.data[2]<<8)| i2b.data[3];
		bmp180.AC3=	(uint32_t)(i2b.data[4]<<8)|i2b.data[5];
		bmp180.AC4= (uint32_t)	(i2b.data[6]<<8)|i2b.data[7];
		bmp180.AC5=(uint32_t)	(i2b.data[8]<<8)|i2b.data[9];
		bmp180.AC6=	(uint32_t)(i2b.data[10]<<8)|i2b.data[11];
		bmp180.B1=	(uint32_t)(i2b.data[12]<<8)|i2b.data[13];
		bmp180.B2=	(uint32_t)(i2b.data[14]<<8)|i2b.data[15];
		bmp180.MB=	(uint32_t)(i2b.data[16]<<8)|i2b.data[17];
		bmp180.MC=	(uint32_t)(i2b.data[18]<<8)|i2b.data[19];
		bmp180.MD=	(uint32_t)(i2b.data[20]<<8)|i2b.data[21];
	}

	//temp
	vTaskDelay(6/ portTICK_PERIOD_MS);
	i2b.regAddr=0xF4;
	i2b.mode=WRITE_TO_ADDR_MODE;
	i2b.dataSize=1;
	i2b.data[0]=0x2E;
	iic->I2C_ISR();
	while (iic->Status()!=BUS_OK);
	vTaskDelay(6/ portTICK_PERIOD_MS);

	i2b.regAddr=0xF6;
	i2b.mode=READ_FROM_ADDR_MODE;
	i2b.dataSize=3;////
	iic->I2C_ISR();
	while (iic->Status()!=BUS_OK);
	if (i2b.dataSize)
	{

		bmp180.UT=(uint32_t)i2b.data[0]<<8 | i2b.data[1];
	}
	//press
	vTaskDelay(6/ portTICK_PERIOD_MS);
	i2b.regAddr=0xF4;
	i2b.mode=WRITE_TO_ADDR_MODE;
	i2b.dataSize=1;
	i2b.data[0]=0x34 + (bmp180.oss<<6);
	iic->I2C_ISR();
	while (iic->Status()!=BUS_OK);
	vTaskDelay(6/ portTICK_PERIOD_MS);

	i2b.regAddr=0xF6;
	i2b.mode=READ_FROM_ADDR_MODE;
	i2b.dataSize=3;
	iic->I2C_ISR();
	while (iic->Status()!=BUS_OK);
	if (i2b.dataSize)
	{

		bmp180.UP=((uint32_t)i2b.data[0]<<16 | (uint32_t)i2b.data[1]<<8 | i2b.data[2])>>(8-bmp180.oss);
	}
	bmp180.calc();

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
			vTaskDelay(10/ portTICK_PERIOD_MS);
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
			};
			// bmp180
			//temp

			 vTaskDelay(10/ portTICK_PERIOD_MS);
				i2b.address=BMP180_ADDRESS;
				i2b.regAddr=0xF4;
				i2b.mode=WRITE_TO_ADDR_MODE;
				i2b.dataSize=1;
				i2b.data[0]=0x2E;
				iic->I2C_ISR();
				while (iic->Status()!=BUS_OK);
				vTaskDelay(10/ portTICK_PERIOD_MS);

				i2b.regAddr=0xF6;
				i2b.mode=READ_FROM_ADDR_MODE;
				i2b.dataSize=3;
				iic->I2C_ISR();
				while (iic->Status()!=BUS_OK);
				if (i2b.dataSize)
				{

					bmp180.UT=(i2b.data[0]<<8) | i2b.data[1];
				}
				//press
				vTaskDelay(10/ portTICK_PERIOD_MS);
				i2b.regAddr=0xF4;
				i2b.mode=WRITE_TO_ADDR_MODE;
				i2b.dataSize=1;
				i2b.data[0]=0x34 + (bmp180.oss<<6);
				iic->I2C_ISR();
				while (iic->Status()!=BUS_OK);
				vTaskDelay(6/ portTICK_PERIOD_MS);

				i2b.regAddr=0xF6;
				i2b.mode=READ_FROM_ADDR_MODE;
				i2b.dataSize=3;
				iic->I2C_ISR();
				while (iic->Status()!=BUS_OK);
				if (i2b.dataSize)
				{

					bmp180.UP=((i2b.data[0]<<16) | (i2b.data[1]<<8) | i2b.data[2])>>(8-bmp180.oss);
				}
				bmp180.calc();
				air.pressure=bmp180.p/133.3;
				air.temp2=bmp180.T/10;



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


void bmp180_class::calc()
{
	// CALCULATE TEMPERATURE
	X1=((UT-AC6)*AC5)/32768;
	X2=(MC*2048)/(X1+MD);
	B5=(int32_t)X1+X2;
	T=(B5+8)/(16);
	// CALCULATE PRESSURE
	B6=B5-4000;
	X1=(B2*(B6*B6/4096))/(2048);
	X2=AC2*B6/2048;
	X3=X1+X2;
	B3=(((AC1*4+X3)<<oss)+2)/4;
	X1=AC3*B6/8192;
	X2=(B1*(B6*B6/4096))/(65536);
	X3=((X1+X2)+2)/4;
	B4=AC4*(uint32_t)(X3+32768)/32768;
	B7=((uint32_t)UP-B3)*(50000>>oss);
	if (B7 < 0x80000000)
	{
		p=(B7*2)/B4;
	}else
	{
		p=(B7/B4)*2;
	}
	X1=(p/256)*(p/256);
	X1=(X1*3038)/65536;
	X2=(-7357*p)/65536;
	p=p+(X1+X2+3791)/16;


}

uint8_t bittwist (uint8_t b)
{
	uint8_t tm=0,tl=0, t0=0;
	for (uint32_t i=0;i<4; i++ )
	{
		tm= (b>>(7-i*2))&(1<<i);
		tl= (b<<(7-(i*2)))&(1<<(7-i));
		t0|=tl|tm;
	}
	return t0;
}

