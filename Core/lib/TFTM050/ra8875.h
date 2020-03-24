/**
  ******************************************************************************
  * @file FSMC_LCD/inc/stm3210e_lcd.h
  * @author   MCD Application Team
  * @version  V2.0.0
  * @date     04/27/2009
  * @brief    This file contains all the functions prototypes for the
  *           stm3210e_lcd firmware driver.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM3210E_LCD_H
#define __STM3210E_LCD_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* LCD color */
#define WHITE          0xFFFF
#define BLACK          0x0000
#define GRAY           0xF7DE
#define NAVY           0x001F
#define BLUE           0x051F
#define RED            0xF800
#define MAGENTA        0xF81F
#define GREEN          0x07E0
#define CYAN           0x7FFF
#define YELLOW         0xFFE0

#define Line0          0
#define Line1          24
#define Line2          48
#define Line3          72
#define Line4          96
#define Line5          120
#define Line6          144
#define Line7          168
#define Line8          192
#define Line9          216

#define Horizontal     0x00
#define Vertical       0x01

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void TFTM050_Init(void);




void LCD_DisplayOn(void);
void LCD_DisplayOff(void);
void LCD_CGROMFont(uint8_t font);
void LCD_ExtROMFont(uint8_t font);
void LCD_EnterTextMode(void);
void LCD_ExitTextMode(void);
void LCD_SetTextWriteCursorAbs(uint16_t x, uint16_t y);
void LCD_PrintString(uint8_t col, uint8_t line, const char* str);
void LCD_SetWriteCursor(uint16_t x, uint16_t y);
void LCD_SetReadCursor(uint16_t x, uint16_t y);
void LCD_SetPwm1(int pwm_duty_cycle);
void LCD_SetPwm2(int pwm_duty_cycle);


void LCD_Clear(__IO uint16_t color);
void LCD_SetForeColor(__IO uint16_t color);
void LCD_SetBackColor(__IO uint16_t color);
void LCD_WriteForeColor(uint8_t r, uint8_t g, uint8_t b);
void LCD_WriteBackColor(uint8_t r, uint8_t g, uint8_t b);
void LCD_SetCursor(uint8_t xpos, uint16_t ypos);
void LCD_PutString(uint16_t posx, uint16_t posy, const char* str);









/*----- High layer function -----*/

void LCD_ClearLine(uint8_t Line);
void LCD_DrawChar(uint16_t Xpos, uint16_t Ypos, const uint16_t *c);
void LCD_DisplayChar(uint8_t Line, uint16_t Column, uint8_t Ascii);
void LCD_DisplayStringLine(uint8_t Line, uint8_t *ptr);
void LCD_SetDisplayWindow(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width);
void LCD_WindowModeDisable(void);
void LCD_DrawLine(uint8_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction);
void LCD_DrawRect(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width);
void LCD_DrawCircle(uint8_t Xpos, uint16_t Ypos, uint16_t Radius);
void LCD_DrawMonoPict(const uint32_t *Pict);
void LCD_WriteBMP(uint32_t BmpAddress);

/*----- Medium layer function -----*/

/*----- Low layer function -----*/
void LCD_CtrlLinesConfig(void);
void LCD_FSMCConfig(void);
void LCD_Delay(uint32_t delay);

#endif /* __STM3210E_LCD_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
