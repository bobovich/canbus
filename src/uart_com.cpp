

#include "stm32f10x.h"
#include "uart_com.h"


void aTaskUart(void * pvParameters)
{
	pQueueComm* pQComm = (pQueueComm*)pvParameters;
	//preset ports
	pack rx;
	pack tx;
	GPIOA->CRH&=~(0xff<<4);
	GPIOA->CRH|=0x49<<4;
	RCC->APB2ENR|=RCC_APB2ENR_USART1EN;
	USART1->BRR=((0x1e<<4)|4);//115200
	USART1->CR1|=USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
	tx.addrdst=137;
	tx.bLeng=10;
	tx.addrsrc=255;
	tx.crc8d=255;
	tx.data[0]=255;
	tx.data[1]=255;
	tx.data[2]=255;
	tx.data[3]=255;
	tx.data[4]=255;
	tx.data[5]=255;
	tx.rssi=255;
	while(1)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);
		//USART1->DR= 0x30;
		if(uxQueueSpacesAvailable(pQComm->a2TX))
		{
			xQueueSend(pQComm->a2TX,&tx,3);
		};
		if(!uxQueueSpacesAvailable(pQComm->a1RX))
		{
			xQueueReceive( pQComm->a1RX, &rx,0);
			for( uint32_t i=0; i< sizeof(pack); i++ )
			{
				while (!(USART1->SR & USART_SR_TXE));
				USART1->DR=*(((uint8_t*)&rx + i));
			}
			while (!(USART1->SR & USART_SR_TXE));
			USART1->DR=0xd;
		}
	}

}
