

#include "stm32f10x.h"
#include "uart_com.h"
#include "string.h"
#include <cstdlib>
#include <cmath>
char bufTx[50];
char bufTmp[50];

char* rawtohex(void* data, uint32_t count,  char * str);
void aTaskUart(void * pvParameters)
{

	pQueueComm* pQComm = (pQueueComm*)pvParameters;
	//preset ports
	pack rx;
	pack tx;
	air_condition airData;

	RCC->APB2ENR|= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR|=RCC_APB2ENR_USART1EN;
	USART1->CR1=0;
	GPIOA->CRH&=~(0x00ff<<4);
	GPIOA->CRH|=0x0089<<4;
	GPIOA->BSRR=1<<10;
	USART1->BRR=((0x1e<<4)|4);//115200
	USART1->CR1=USART_CR1_RE | USART_CR1_TE;
	USART1->CR1|= USART_CR1_UE;
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
	//USART1->CR1|= USART_CR1_SBK;
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	if(uxQueueSpacesAvailable(pQComm->a1TX))
				{
					tx.addrdst=88;
					tx.rssi=rx.rssi_r;
					xQueueSend(pQComm->a1TX,&tx,1);
				};
	while(1)
	{

		/*strcpy(bufTx, "Hello\n");
		printUart(bufTx);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
*/

		if(xQueueReceive( pQComm->a1RX, &rx,0)==pdPASS)
		{
			//xQueueReceive( pQComm->a1RX, &rx,0);
			strcpy(bufTx, "APP1 data:\0");
			strcat(bufTx, rawtohex( (void*)&rx, sizeof( pack ), bufTmp ));
			strcat(bufTx, "\n");
			printUart(bufTx);
			if(uxQueueSpacesAvailable(pQComm->a1TX))
			{
				tx.addrdst=88;
				tx.rssi=rx.rssi_r;
				xQueueSend(pQComm->a1TX,&tx,0);
			};


		}else if(xQueueReceive( pQComm->a2RX, &rx,0)==pdPASS)
		{
			//xQueueReceive( pQComm->a2RX, &rx,0);
			strcat(bufTx, "APP2 data:\0");
			strcat(bufTx, rawtohex( (void*)&rx, sizeof( pack ), bufTmp ));
			strcat(bufTx, "\n\0");
			printUart(bufTx);
			if(uxQueueSpacesAvailable(pQComm->a2TX))
			{
				tx.addrdst=87;
				tx.rssi=rx.rssi_r;
				xQueueSend(pQComm->a2TX,&tx,0);
			};
		};
		if(xQueueReceive( pQComm->qSensor, &airData,0)==pdPASS)
		{
			strcat(bufTx, "CO2: ");
			strcat(bufTx, itoa((int)airData.CO2, bufTmp, 10));
			strcat(bufTx,"\n");
			strcat(bufTx, "TVOC: ");
			strcat(bufTx, itoa((int)airData.TVOC, bufTmp, 10));
			strcat(bufTx,"\n");
			printUart(bufTx);
			strcat(bufTx, "Temp: ");
			strcat(bufTx, ftoa(airData.temp,2, bufTmp));
			strcat(bufTx,"\n");
			strcat(bufTx, "Влажность: ");
			strcat(bufTx, ftoa(airData.humidity,2, bufTmp));
			strcat(bufTx,"\n");
			printUart(bufTx);
			xQueueOverwrite(pQComm->qDisplay, &airData  );
		}

	}

}
void printUart(char * str)
{

	int i=0;
	while (str[i]!=0)
	{
		if ((USART1->SR & USART_SR_TXE)){
		USART1->DR=str[i];
		i++;
		};
	}
	while (!(USART1->SR & USART_SR_TXE));
	//USART1->CR1|= USART_CR1_SBK;
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

char * ftoa(double f, uint8_t w, char * buf)
{
	int32_t n=0, fr=0, d=pow(10,w );

	char btm[10];
	buf[0]=0;
	if (f == 0)
		{
		strcpy(buf, "0.0");
		return buf;
		}
	if (f<0)
	{
		strcpy(buf, "-");
	}
	btm[0]=0;
	n=(int32_t)abs(f);
	fr=(int32_t)( (( f - (double)n ) + (5*pow(0.1,w) )) * d);
	if(fr >= d)
	{
		fr-=100;
		n++;
	}
	strcat(buf, itoa(n,btm,10));
	btm[0]=0;
	strcat(buf, ".");
	strcat(buf, itoa(fr,btm, 10));
	return buf;

}
