/*
 * cc11x_class.cpp
 *
 *  Created on: 10 ���. 2020 �.
 *      Author: ����
 */

#include "CC1101.h"


/*
 * this metod a class contructor
 * he setup port, target ic from port, and his queues
 */
cc11xx_class::cc11xx_class(xTaskParam * pPortParam, uint8_t set_len, uint8_t *rfSettings)
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
	sendSTB(SRES);// reset chip
	deselectChip();
	vTaskDelay(10 / portTICK_PERIOD_MS);
	selectChip();
	while (getMISO()==1); //check miso low state
	deselectChip();
	selectChip();
	while (getMISO()==1);
	deselectChip();
	this->sendBurst(0,set_len, rfSettings);
	this->sendByte(PATAB, 0x50);
}

/*
 * chip select set to zero nss pin. use only bit bang
 */
btype_t cc11xx_class::selectChip()
{
	*NSS_reset=1;
	return *NSS_get;
}

/*
 * chip deselect set to hight nss pin. use only bit bang
 */
btype_t cc11xx_class::deselectChip()
{
	*NSS_set=1;
	return *NSS_get;
}
/*
 * return miso pin level for response ready state ic, to communication
 */
btype_t cc11xx_class::getMISO()
{

	return *MISO_lv;
}

/*
 *  @Brief this private metod
 * send byte to chip to address
 */
btype_t cc11xx_class::sendByte (uint8_t address, uint8_t  cmd)
{
	uint8_t stsb;
	deselectChip();
	selectChip();
	while (getMISO());
	while  ((SP->SR & SPI_SR_BSY));
	if ((SP->SR & SPI_SR_OVR))
	{
		stsb=SP->DR;
		if ((SP->SR & SPI_SR_OVR)) return 0;
	};
	SP->DR=WRITE|address;
	while  (!((SP->SR & SPI_SR_TXE)&& (SP->SR &SPI_SR_RXNE)));
	stsb=SP->DR;
	cStatus->rdy= stsb>>7;
	cStatus->state=(stsb>>4) & 0x7;
	cStatus->fifo_tx_av = stsb & 0x0f;
	SP->DR=cmd;
	while  (!((SP->SR & SPI_SR_TXE)&& (SP->SR &SPI_SR_RXNE)));
	stsb=SP->DR;
	deselectChip();
	return 1;
}

/*
 *  @Brief read byte from chip from address
 */
uint8_t cc11xx_class::readByte (uint8_t address)
{
	//deselectChip();
	selectChip();
	uint8_t stsb;
	while (getMISO());
	while  ((SP->SR & (SPI_SR_BSY)));
	if ((SP->SR & SPI_SR_OVR))
	{
		stsb=SP->DR;
		if ((SP->SR & SPI_SR_OVR))
			return 0;
	}
	SP->DR=READ|address;
	while  (!((SP->SR & SPI_SR_TXE)&& (SP->SR &SPI_SR_RXNE)));
	stsb=SP->DR;
	cStatus->rdy= stsb>>7;
	cStatus->state=(stsb>>4) & 0x7;
	cStatus->fifo_rx_av = stsb & 0x0f;
	SP->DR=0;

	while  (!((SP->SR & SPI_SR_TXE)&& (SP->SR &SPI_SR_RXNE)));
	stsb=SP->DR;
	deselectChip();
	return stsb;
}

/*
 * @Brief send flow byte to address
 */
btype_t cc11xx_class::sendBurst(uint8_t sAddress, uint8_t  cmdCount, uint8_t* cmds)
{
	uint8_t stsb;
	deselectChip();
	selectChip();
	while (getMISO());
	while  ((SP->SR & SPI_SR_BSY));
	if ((SP->SR & SPI_SR_OVR))
	{
		stsb=SP->DR;
		if ((SP->SR & SPI_SR_OVR)) return 0;
	};
	stsb=SP->DR;
	SP->DR=BURST|sAddress;
	for (int i=0; i<cmdCount; i++)
	{
		while  (!((SP->SR & SPI_SR_TXE)&& (SP->SR &SPI_SR_RXNE)));
		SP->DR=*(cmds+i);
		stsb=SP->DR;
	}
	while  (!((SP->SR & SPI_SR_TXE)&& (SP->SR &SPI_SR_RXNE)));
	stsb=SP->DR;
	cStatus->rdy= stsb>>7;
	cStatus->state=(stsb>>4) & 0x7;
	cStatus->fifo_tx_av = stsb & 0x0f;
	deselectChip();
	return 1;
}

/*
 * @Brief read flow byte to address
 */
btype_t cc11xx_class::readBurst (uint8_t address, uint8_t  cmdCount, uint8_t* cmds)
{
	deselectChip();
	selectChip();
	uint8_t stsb;
	while (getMISO());
	while  ((SP->SR & (SPI_SR_BSY)));
	if ((SP->SR & SPI_SR_OVR))
	{
		stsb=SP->DR;
		if ((SP->SR & SPI_SR_OVR)) return 0;
	};
	SP->DR=READ|BURST|address;
	while  (!((SP->SR & SPI_SR_TXE)&& (SP->SR &SPI_SR_RXNE)));
	stsb=SP->DR;
	cStatus->rdy= stsb>>7;
	cStatus->state=(stsb>>4) & 0x7;
	cStatus->fifo_rx_av = stsb & 0x0f;
	for (int i=0; i<cmdCount; i++)
	{
	SP->DR=0;
	while  (!((SP->SR & SPI_SR_TXE)&& (SP->SR &SPI_SR_RXNE)));
	*(cmds+i)=SP->DR;
	}
	deselectChip();
	return 1;
}

/*
 * @Brief return status of chip
 */
btype_t cc11xx_class::chekStatus()
{
	uint8_t stsb;

	selectChip();
	while (getMISO());
	while  ((SP->SR & SPI_SR_BSY));
	if ((SP->SR & SPI_SR_OVR))
	{
		stsb=SP->DR;
		if ((SP->SR & SPI_SR_OVR)) return 0;
	};
	//stsb=SP->DR;
	SP->DR=WRITE|SNOP;
	while  (!((SP->SR & SPI_SR_TXE)&& (SP->SR &SPI_SR_RXNE)));
	stsb=SP->DR;
	cStatus->rdy= stsb>>7;
	cStatus->state=(stsb>>4) & 0x7;
	cStatus->fifo_tx_av= stsb & 0x0f;
	SP->DR=READ|SNOP;
	while  (!((SP->SR & SPI_SR_TXE)&& (SP->SR &SPI_SR_RXNE)));
	stsb=SP->DR;
	cStatus->rdy= stsb>>7;
	cStatus->state=(stsb>>4) & 0x7;
	cStatus->fifo_rx_av= stsb & 0x0f;
	deselectChip();
	switch (cStatus->state)
	{
	case RXFIFO_OVERFLOW: this->sendSTB(SFRX);this->sendSTB(SRX); break;
	case TXFIFO_UNDERFLOW:this->sendSTB(SFTX); this->sendSTB(SRX); break;
#ifdef FREERTOS
	case CALIBRATE_MODE: case SETTLING_MODE: vTaskDelay(1 / portTICK_PERIOD_MS); break;
#endif
	}
	return 1;
}

/*
 * @Brief send packet to fifo and radio
 */
btype_t cc11xx_class::txPack(void)
{
	uint8_t stsb;
	deselectChip();
	selectChip();
	while (getMISO());
	while  ((SP->SR & SPI_SR_BSY));
	if ((SP->SR & SPI_SR_OVR))
	{
		stsb=SP->DR;
		if ((SP->SR & SPI_SR_OVR)) return 0;
	};
	//stsb=SP->DR;
	SP->DR=WRITE|BURST|FIFO;
	while  ( ! ((SP->SR & SPI_SR_TXE) && (SP->SR & SPI_SR_RXNE)) );
	stsb=SP->DR;
		for (uint32_t i=0; i<PACK_TX_COUNT; i++)
		{
			SP->DR=*(((uint8_t*)txp+i));
			while  (!((SP->SR & SPI_SR_TXE)&& (SP->SR &SPI_SR_RXNE)));
			stsb=SP->DR;
		}
	deselectChip();
	this->sendSTB(STX);
	return 1;
}

/*
 * @Brief read data from fifo
 */
btype_t cc11xx_class::rxPack(void)
{
	uint8_t stsb;
	deselectChip();
	selectChip();
	while (getMISO());
	while  ((SP->SR & SPI_SR_BSY));
	if ((SP->SR & SPI_SR_OVR))
	{
		stsb=SP->DR;
		if ((SP->SR & SPI_SR_OVR)) return 0;
	};
	stsb=SP->DR;
	SP->DR=READ|BURST|FIFO;
	while  (!((SP->SR & SPI_SR_TXE)&& (SP->SR &SPI_SR_RXNE)));
	stsb=SP->DR;
	for (uint32_t i=0; i< PACK_RX_COUNT; i++)
		{
			SP->DR=0;
			while  (!((SP->SR & SPI_SR_TXE)&& (SP->SR &SPI_SR_RXNE)));
			* ( ((uint8_t*) rxp) +i  )=SP->DR;
		}
	deselectChip();
	return 1;
}


/*
 * @Brief return recieved packet
 */
pack* cc11xx_class::getRxPack(void)
{
	return this->rxp;
}


/*
 * @Brief send strobe
 */
btype_t cc11xx_class::sendSTB(uint8_t stb)
{
	uint8_t stsb;
	deselectChip();
	selectChip();
	while (getMISO());
	while  ((SP->SR & SPI_SR_BSY));
	if ((SP->SR & SPI_SR_OVR))
	{
		stsb=SP->DR;
		if ((SP->SR & SPI_SR_OVR)) return 0;
	};
	SP->DR=stb;
	while  (!((SP->SR & SPI_SR_TXE)&& (SP->SR &SPI_SR_RXNE)));
	stsb=SP->DR;
	deselectChip();
	return 1;
}


/*
 * @Brief crc8 calc
 */
uint8_t cc11xx_class::crc8(uint8_t * pcBlock, uint8_t len)
{

	uint8_t crc = 0xFF;
	uint32_t i;
    while (len--)
	{
	  crc ^= *pcBlock++;
      for (i = 0; i < 8; i++)
	  crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
	}
    return crc;
}

/*
 * @Brief rtos function to dicountinune  read receieved data
 */
btype_t cc11xx_class::rxEventHook(void)
{
	if ((*rxEvent == RX_EVENT) || ( this->cStatus->fifo_rx_av >= sizeof(pack)) )
	{
		this->rxPack();
		if (this->crc8((uint8_t*)this->rxp, sizeof(pack)-3) == this->rxp->crc8d )
		{
		xQueueSend(this->pRX, (const void *) this->rxp, 1 / portTICK_PERIOD_MS );
		return 1;
		} else
		{
			this->sendSTB(SIDLE);
			this->sendSTB(SFRX);
			this->sendSTB(SRX);
			return 0;
		}
	}
	else
		return 0;
}

/*
 * @Brief rtos function to dicountinune  send  data
 */
btype_t cc11xx_class::txEventHook(void)
{
	if ( (uxQueueSpacesAvailable( this->pTX) < QUEUE_SIZE) )
	{
		if (xQueueReceive(this->pTX, (void*)this->txp, 1 / portTICK_PERIOD_MS)==pdTRUE)
		this->txp->crc8d=this->crc8((uint8_t*)this->txp, sizeof(pack)-3);
		this->txPack();
		this->chekStatus();
		int i=0;
		while((cStatus->state != TX_MODE ) && (i<3))
		{
			this->sendSTB(STX);
			this->chekStatus();
			i++;
		};
		if (i>3)return 0;  else return 1;
	}
	else
	{
		return 0;
	}
}


