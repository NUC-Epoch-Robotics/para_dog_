#ifndef __OLED_LEFT_H__
#define __OLED_LEFT_H__

#include "font.h"
#include "main.h"
#include "string.h"

typedef enum {
  OLED_COLOR_NORMAL_left = 0, // 正常模式 黑底白字
  OLED_COLOR_REVERSED_left    // 反色模式 白底黑字
} OLED_ColorMode_left;

void OLED_Init_left(void);
void OLED_DisPlay_On_left(void);
void OLED_DisPlay_Off_left(void);

void OLED_NewFrame_left(void);
void OLED_ShowFrame_left(void);
void OLED_SetPixel_left(uint8_t x, uint8_t y, OLED_ColorMode_left color);

void OLED_DrawLine_left(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED_ColorMode_left color);
void OLED_DrawRectangle_left(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode_left color);
void OLED_DrawFilledRectangle_left(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode_left color);
void OLED_DrawTriangle_left(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode_left color);
void OLED_DrawFilledTriangle_left(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode_left color);
void OLED_DrawCircle_left(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode_left color);
void OLED_DrawFilledCircle_left(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode_left color);
void OLED_DrawEllipse_left(uint8_t x, uint8_t y, uint8_t a, uint8_t b, OLED_ColorMode_left color);
void OLED_DrawImage_left(uint8_t x, uint8_t y, const Image *img, OLED_ColorMode_left color);

void OLED_PrintASCIIChar_left(uint8_t x, uint8_t y, char ch, const ASCIIFont *font, OLED_ColorMode_left color);
void OLED_PrintASCIIString_left(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode_left color);
void OLED_PrintString_left(uint8_t x, uint8_t y, char *str, const Font *font, OLED_ColorMode_left color);

#endif 

