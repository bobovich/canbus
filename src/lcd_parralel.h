/*
 * lcd_parralel.h
 *
 *  Created on: 26 мая 2020 г.
 *      Author: Иван
 */

#ifndef LCD_PARRALEL_H_
#define LCD_PARRALEL_H_


#include "FreeRTOS.h"
#include "queue.h"
#include "stm32f10x.h"

#define LCD_PORT				GPIOA
#define LCD_PORT_DATA_READ		GPIOA->CRL=0x44444444
#define LCD_PORT_DATA_WRITE		GPIOA->CRL=0x11111111

#define LCD_RS_SET  	*((uint32_t*)(PERIPH_BB_BASE + ((GPIOC_BASE-PERIPH_BASE+0x10)  * 32) + (4 * 2)))=1 //PC.2
#define LCD_RS_RESET 	*((uint32_t*)(PERIPH_BB_BASE + ((GPIOC_BASE-PERIPH_BASE+0x14)  * 32) + (4 * 2)))=1 //PC.2

#define LCD_nCS_SET		*((uint32_t*)(PERIPH_BB_BASE + ((GPIOC_BASE-PERIPH_BASE+0x14)  * 32) + (4 * 3)))=1 //PC.3
#define LCD_nCS_RESET	*((uint32_t*)(PERIPH_BB_BASE + ((GPIOC_BASE-PERIPH_BASE+0x10)  * 32) + (4 * 3)))=1 //PC.3

#define LCD_nRD_SET		*((uint32_t*)(PERIPH_BB_BASE + ((GPIOC_BASE-PERIPH_BASE+0x14)  * 32) + (4 * 0)))=1 //PC.0
#define LCD_nRD_RESET	*((uint32_t*)(PERIPH_BB_BASE + ((GPIOC_BASE-PERIPH_BASE+0x10)  * 32) + (4 * 0)))=1

#define LCD_nWR_SET		*((uint32_t*)(PERIPH_BB_BASE + ((GPIOC_BASE-PERIPH_BASE+0x14)  * 32) + (4 * 1)))=1 //PC.1
#define LCD_nWR_RESET	*((uint32_t*)(PERIPH_BB_BASE + ((GPIOC_BASE-PERIPH_BASE+0x10)  * 32) + (4 * 1)))=1

#define LCD_RES_SET		*((uint32_t*)(PERIPH_BB_BASE + ((GPIOC_BASE-PERIPH_BASE+0x14)  * 32) + (4 * 4)))=1 //PC.4
#define LCD_RES_RESET	*((uint32_t*)(PERIPH_BB_BASE + ((GPIOC_BASE-PERIPH_BASE+0x10)  * 32) + (4 * 4)))=1

#define WAIT_150NS		asm("nop")//for (int i=0; i<2; i++)
#define WAIT_50NS		asm("nop")//for (int i=0; i<1; i++)
#define WAIT_40MS		for (int i=0; i<1800; i++)

#define PORT_INIT 		RCC->APB2ENR|= RCC_APB2ENR_IOPAEN; \
						RCC->APB2ENR|= RCC_APB2ENR_IOPCEN; \
						GPIOC->CRL=0x00011111; \
						GPIOC->ODR= 0x1f



void lcdTask(void *p);

struct pixelcolor
{
	uint8_t R;
	uint8_t G;
	uint8_t B;
};

class lcd_parallel
{
private:
	u8 dW;//dfl 320
	u8 dH;//dfl 240

public:
	lcd_parallel(void);
	void lcd_init(void);
	uint32_t lcd_w_cmd(uint16_t address, uint16_t data);
	uint16_t lcd_r_cmd(uint16_t address);
	uint32_t lcd_flush(pixelcolor color);
	uint32_t lcd_draw_XY(uint16_t x, uint16_t y, pixelcolor color);
	uint32_t lcd_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, pixelcolor color);
	uint32_t lcd_print_XY(uint32_t x, uint32_t y, pixelcolor color, void* font);
	uint32_t lcd_write_str(char *str, uint16_t x, uint16_t y, pixelcolor text_color, pixelcolor back_color,  void* font);
	uint32_t lcd_write_str90(char *str, uint16_t x, uint16_t y, pixelcolor text_color, pixelcolor back_color,  void* font);
};


#endif /* LCD_PARRALEL_H_ */
