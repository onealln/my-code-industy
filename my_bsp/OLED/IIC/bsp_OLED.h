#ifndef __BSP_OLED_H_
#define __BSP_OLED_H_

#include "main.h"

#define OLED_ADDERSS 0x78

extern uint8_t OlED_GRAM[8][128];


void OLED_Init(void);
void OLED_SetPos(uint8_t x, uint8_t y);
void OLED_ClearAll(void);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size,uint8_t mode);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *s, uint8_t size);
void OLED_ShowNum( uint8_t x,uint8_t y,uint16_t num,uint8_t length,uint8_t size);
void OLED_Refresh_Gram(void);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
void OLED_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);
void OLED_DrawAngleLine(uint32_t x,uint32_t y,float du,uint32_t len,uint8_t c);
void OLED_DrawAngleLine2(uint32_t x,uint32_t y,float du,uint32_t len,uint32_t w,uint8_t c);
void OLED_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void OLED_Circle(uint16_t x0,uint16_t y0,uint8_t r);
void OLED_ShowChineseFont(uint8_t x, uint8_t y, uint8_t size, uint8_t number);





#endif 

