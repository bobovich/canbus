/*
 * cc11x_class.cpp
 *
 *  Created on: 10 ���. 2020 �.
 *      Author: ����
 */

#include "CC1101.h"



cc11xx_class::cc11xx_class(xTaskParam * pPortParam, uint8_t set_len, uint8_t *rfSettings) // class constructor
{
	SP=(SPI_TypeDef*)pPortParam->pTaskSerial;
	xPortHW= pPortParam->xTaskPortH;
	pRX=pPortParam->xCommRX;
	pTX=pPortParam->xCommTX;
	pErr=pPortParam->xErr;
	this->rxEvent=pPortParam->pRxEvent;
	this->txcEvent=pPortParam->pTxcEvent;
	switch  (xPortHW)
	{
		case PORT_NORMAL:
		{
			if (SP == (SPI_TypeDef*)SPI1_BASE)
			{
				RCC->APB2ENR|=RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPAEN;
				GPIOA->CRL=(1<<16)|(0x9<<20)|(0x4<<24)|(0x9<<28);// set GPIO
				NSS_get=	(uint32_t*)(PERIPH_BB_BASE + ((GPIOA_BASE-PERIPH_BASE+0x08)  * 32) + (4 * 4));//bit bang address
				NSS_set=	(uint32_t*)(PERIPH_BB_BASE + ((GPIOA_BASE-PERIPH_BASE+0x10)  * 32) + (4 * 4));
				NSS_reset=	(uint32_t*)(PERIPH_BB_BASE + ((GPIOA_BASE-PERIPH_BASE+0x14)  * 32) + (4 * 4));
				MISO_lv=	(uint32_t*)(PERIPH_BB_BASE + ((GPIOA_BASE-PERIPH_BASE+0x08)  * 32) + (6 * 4));


			} else if (SP == (SPI_TypeDef*)SPI2_BASE)
			{
				RCC->APB2ENR|= RCC_APB2ENR_IOPBEN;
				RCC->APB1ENR|= RCC_APB1ENR_SPI2EN;
				GPIOB->CRH=(1<<16)|(0x9<<20)|(0x4<<24)|(0x9<<28);		// set GPIO
				NSS_get=	(uint32_t*)(PERIPH_BB_BASE + ((GPIOB_BASE-PERIPH_BASE+0x08)  * 32) + (12 * 4));//bit bang address
				NSS_set=	(uint32_t*)(PERIPH_BB_BASE + ((GPIOB_BASE-PERIPH_BASE+0x10)  * 32) + (12 * 4));
				NSS_reset=	(uint32_t*)(PERIPH_BB_BASE + ((GPIOB_BASE-PERIPH_BASE+0x14)  * 32) + (12 * 4));
				MISO_lv=	(uint32_t*)(PERIPH_BB_BASE + ((GPIOB_BASE-PERIPH_BASE+0x08)  * 32) + (14 * 4));
			} else
			{
				// Return task init error here!!!!!!!!!!!!!!!
			};

			break;
		}
		case PORT_REMAP :
		{
			if (SP == (SPI_TypeDef*)SPI1_BASE)
			{
				RCC->APB2ENR|=RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPAEN;
				GPIOA->CRL=(1<<16)|(0x9<<20)|(0x4<<24)|(0x9<<28);// set GPIO
				NSS_get=	0;//bit bang address
				NSS_set=	0;
				NSS_reset=	0;
				MISO_lv=	0;

			} else if (SP == (SPI_TypeDef*)SPI2_BASE)
			{
				RCC->APB2ENR|= RCC_APB2ENR_IOPBEN;
				RCC->APB1ENR|= RCC_APB1ENR_SPI2EN;
				GPIOB->CRH=(1<<16)|(0x9<<20)|(0x4<<24)|(0x9<<28);// set GPIO
				NSS_get=0;//bit bang address
				NSS_set=0;
				NSS_reset=0;
				MISO_lv=0;
			} else
			{
						// Return task init error here!!!!!!!!!!!!!!!
			};
			break;
		}
	};
	SP->CR1=0;
	SP->CR1=(0x6<<3);
	SP->CR1|=SPI_CR1_SSM|SPI_CR1_SSI;
	// clock div 128(56M/128), SPI is a master mode
	if ((SP->SR&SPI_SR_MODF));
	SP->CR1|=SPI_CR1_MSTR;
	SP->CR1|=SPI_CR1_SPE;
	selectChip();
	while (getMISO()==1); //check miso low state
	deselectChip();
	selectChip();
	while (getMISO()==1);
	deselectChip();
	sendCmd( SRES,0);// reset chip
	deselectChip();
	vTaskDelay(10 / portTICK_PERIOD_MS);
	selectChip();
	while (getMISO()==1); //check miso low state
	deselectChip();
	selectChip();
	while (getMISO()==1);
	deselectChip();
	this->sendBurstCmd(0,62, rfSettings);





}

btype_t cc11xx_class::selectChip()
{
	*NSS_reset=1;
	return *NSS_get;
}

btype_t cc11xx_class::deselectChip()
{
	*NSS_set=1;
	return *NSS_get;
}

btype_t cc11xx_class::getMISO()
{

	return *MISO_lv;
}

btype_t cc11xx_class::sendCmd (uint8_t address, uint8_t  cmd)
{
	deselectChip();
	selectChip();
	while (getMISO());
	while  (!(SP->SR & (SPI_SR_TXE)));
	SP->DR=cmd;
	while  (!(SP->SR & (SPI_SR_TXE)));
	uint8_t stsb;
	stsb=SP->DR;
	cStatus->rdy= stsb>>7;
	cStatus->state=(stsb>>4) & 0x7;
	if ((cmd & 0x3f))
			cStatus->fifo_rx_av = stsb & 0x0f;
		else cStatus->fifo_tx_av = stsb & 0x0f;

	return 1;
}

btype_t cc11xx_class::sendBurstCmd(uint8_t sAddress, uint8_t  cmdCount, uint8_t* cmds)
{
	uint8_t stsb;
	deselectChip();
	selectChip();
	while (getMISO());
	while  (!(SP->SR & SPI_SR_TXE));
	stsb=SP->DR;
	SP->DR=0x40|sAddress;
	for (int i=0; i<cmdCount; i++)
	{
		while  (!(SP->SR & (SPI_SR_TXE)));
		SP->DR=*(cmds+i);
		while  (!(SP->SR & (SPI_SR_TXE)));
	}

	return 1;
}

btype_t cc11xx_class::chekStatus()
{
	uint8_t stsb;
	deselectChip();
	selectChip();
	while (getMISO());
	while  (!(SP->SR & SPI_SR_TXE));
	stsb=SP->DR;
	SP->DR=SNOP;
	while  (!((SP->SR & SPI_SR_TXE)&&(SP->SR & SPI_SR_RXNE)));
	stsb=SP->DR;
	cStatus->rdy= stsb>>7;
	cStatus->state=(stsb>>4) & 0x7;
	cStatus->fifo_tx_av= stsb & 0x0f;
	SP->DR=0x80|SNOP;
	while  (!((SP->SR & SPI_SR_TXE)&&(SP->SR & SPI_SR_RXNE)));
	stsb=SP->DR;
	cStatus->rdy= stsb>>7;
	cStatus->state=(stsb>>4) & 0x7;
	cStatus->fifo_rx_av= stsb & 0x0f;
	return 1;
}

btype_t cc11xx_class::txPack(void)
{
	uint8_t stsb;
	deselectChip();
	selectChip();
	while (getMISO());
	while  (!(SP->SR & SPI_SR_TXE));
	stsb=SP->DR;
	SP->DR=WRITE|BURST|FIFO;
		for (uint32_t i=0; i<sizeof(pack); i++)
		{
			while  (!(SP->SR & (SPI_SR_TXE)));
			SP->DR=*((uint8_t*)(txp+i));
			while  (!(SP->SR & (SPI_SR_TXE)));
		}
		deselectChip();
		selectChip();
		while (getMISO());
		SP->DR=STX;
		while  (!(SP->SR & (SPI_SR_TXE)));

		return 1;
}
btype_t cc11xx_class::rxPack(void)
{
	uint8_t stsb;
	deselectChip();
	selectChip();
	while (getMISO());
	while  (!(SP->SR & SPI_SR_TXE));
	stsb=SP->DR;
	SP->DR=0xc0;//READ|BURST|FIFO;
	while  (!(SP->SR & (SPI_SR_TXE)));
	stsb=SP->DR;
	for (uint32_t i=0; i<sizeof(pack); i++)
		{
		SP->DR=READ|SNOP;
		while  (!((SP->SR & SPI_SR_TXE)&&(SP->SR & SPI_SR_RXNE)));
		*((uint8_t*)(txp+i))=SP->DR;
		//while  (!(SP->SR & (SPI_SR_TXE)));
		}
	deselectChip();
	selectChip();
	while (getMISO());
	SP->DR=STX;
	while  (!(SP->SR & (SPI_SR_TXE)));

			return 1;
}

pack* cc11xx_class::getRxPack(void)
{
	return this->rxp;
}

btype_t cc11xx_class::sendSTB(uint8_t stb)
{
	deselectChip();
	selectChip();
	while (getMISO());
	while  (!(SP->SR & SPI_SR_TXE));
	SP->DR=stb;
	while  (!(SP->SR & (SPI_SR_TXE)));
	return 1;
}

btype_t cc11xx_class::rxEventHook(void)
{
	if (*rxEvent == RX_EVENT)
	{
		this->rxPack();
		xQueueSend(this->pRX, (const void *) this->rxp, 1 / portTICK_PERIOD_MS );
		this->sendSTB(SRX);
		return RX_EVENT;
	}
	else
		return 0;

}

btype_t cc11xx_class::txEventHook(void)
{
	if ( (uxQueueSpacesAvailable( this->pTX) < QUEUE_SIZE) )
	{
		xQueueReceive(this->pTX, this->txp, 1 / portTICK_PERIOD_MS);
		this->txPack();
		this->sendSTB(SRX);
		return TX_EVENT;
	}
	else
	{
		this->sendSTB(SRX);
		return 0;
	}
}


