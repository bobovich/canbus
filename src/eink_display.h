/*
 * eink_display.h
 *
 *  Created on: 12 апр. 2020 г.
 *      Author: Иван
 */

#ifndef EINK_DISPLAY_H_
#define EINK_DISPLAY_H_


#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "CC1101.h"
#include "i2c_sensor.h"

#define EPD_W21_SPI_SPEED 0x02

#define EPD_W21_MOSI_0	GPIO_ResetBits(GPIOC, GPIO_Pin_10)
#define EPD_W21_MOSI_1	GPIO_SetBits(GPIOC, GPIO_Pin_10)

#define EPD_W21_CLK_0	GPIO_ResetBits(GPIOC, GPIO_Pin_11)
#define EPD_W21_CLK_1	GPIO_SetBits(GPIOC, GPIO_Pin_11)

#define EPD_W21_CS_0	GPIO_ResetBits(GPIOA, GPIO_Pin_8)
#define EPD_W21_CS_1	GPIO_SetBits(GPIOA, GPIO_Pin_8)

#define EPD_W21_DC_0	GPIO_ResetBits(GPIOA, GPIO_Pin_11)
#define EPD_W21_DC_1	GPIO_SetBits(GPIOA, GPIO_Pin_11)

#define EPD_W21_RST_0	GPIO_ResetBits(GPIOA, GPIO_Pin_12)
#define EPD_W21_RST_1	GPIO_SetBits(GPIOA, GPIO_Pin_12)

#define isEPD_W21_BUSY GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_13) // for solomen solutions

#define EPD_W21_WRITE_DATA 1
#define EPD_W21_WRITE_CMD  0

void displayTask(void* pvParams);
extern void driver_delay_us(unsigned int xus);
extern void driver_delay_xms(unsigned long xms);

void SPI_Write(unsigned char value);
void EPD_W21_WriteDATA(unsigned char command);
void EPD_W21_WriteCMD(unsigned char command);

 void driver_delay_us(unsigned int xus);
 void driver_delay_xms(unsigned long xms);
 void EPD_W21_Init(void);
void EPD_init(void);
void PIC_display(const unsigned char* picData_old,const unsigned char* picData_new);
void EPD_sleep(void);
void EPD_refresh(void);
void lcd_chkstatus(void);
void PIC_display_Clean(void);
void driver_delay_us(unsigned int xus) ;
void DELAY_S( int delaytime);






#endif /* EINK_DISPLAY_H_ */
