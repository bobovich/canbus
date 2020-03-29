

#include "stm32f10x.h"
#include "uart_com.h"
#include "string.h"

char bufTx[50];
char bufTmp[50];
void printUart(char * str);
char* rawtohex(void* data, uint32_t count,  char * str);
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
	tx.addrdst=87;
	tx.bLeng=10;
	tx.addrsrc=255;
	tx.crc8d=255;
	tx.data[0]=127;
	tx.data[1]=255;
	tx.data[2]=127;
	tx.data[3]=255;
	tx.data[4]=255;
	tx.data[5]=255;
	tx.rssi=255;
	bufTx[0]=0;
	while(1)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);
		//USART1->DR= 0x30;
		if(uxQueueSpacesAvailable(pQComm->a2TX))
		{
			tx.addrdst=87;
			xQueueSend(pQComm->a2TX,&tx,0);
		};
		if(uxQueueSpacesAvailable(pQComm->a1TX))
		{
			tx.addrdst=88;
			xQueueSend(pQComm->a1TX,&tx,0);
		};
		if(!uxQueueSpacesAvailable(pQComm->a1RX))
		{
			xQueueReceive( pQComm->a1RX, &rx,0);
			strcpy(bufTx, "APP1 data:\0");
			strcat(bufTx, rawtohex( (void*)&rx, sizeof( pack ), bufTmp ));
			strcat(bufTx, "\n");
			printUart(bufTx);
		};
		if(!uxQueueSpacesAvailable(pQComm->a2RX))
		{
			xQueueReceive( pQComm->a2RX, &rx,0);
			strcat(bufTx, "APP2 data:\0");
			strcat(bufTx, rawtohex( (void*)&rx, sizeof( pack ), bufTmp ));
			strcat(bufTx, "\n\0");
			printUart(bufTx);
		};

	}

}
void printUart(char * str)
{

	int i=0;
	while (str[i]!=0)
	{
		while (!(USART1->SR & USART_SR_TXE));
		USART1->DR=str[i];
		i++;
	}
	while (!(USART1->SR & USART_SR_TXE));
	str[0]=0;

}

char* rawtohex(void* data, uint32_t count,  char * str)
{
	uint32_t i=0, is=0;
	while (i<count)
	{
		char b, c=0;
		while (c<2)
		{
			if (c==0) b= ( *((uint8_t*)data+i) >> 4 ) & 0x0f; else b = *((uint8_t*)data+i) & 0x0f;
			switch (b)
			{
			case 0: case 1:	case 2:	case 3:	case 4:	case 5:	case 6:	case 7:	case 8:	case 9:
			{
				b+=0x30; break;
			};
			case 10: b='A'; break;
			case 11: b='B'; break;
			case 12: b='C'; break;
			case 13: b='D'; break;
			case 14: b='E'; break;
			case 15: b='F'; break;
			}
			str[is]=b;
			is++;
			c++;
		}
		str[is]=',';
		is++;
		i++;
	}
	str[is-1]=0;
	return str;
}
