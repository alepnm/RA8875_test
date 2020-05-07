
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RA8875_H
#define __RA8875_H

/* Includes ------------------------------------------------------------------*/
#include "GUI.h"
#include "main.h"
#include "ra_fsmc.h"
#include "ra_bte.h"


#define X_SIZE 480
#define Y_SIZE 272
#define DISPLAY_PIXELS (X_SIZE*Y_SIZE)


#define RA8875_RST_LOW()    LL_GPIO_ResetOutputPin(LCD_RST_GPIO_Port, LCD_RST_Pin)
#define RA8875_RST_HIGH()   LL_GPIO_SetOutputPin(LCD_RST_GPIO_Port, LCD_RST_Pin)
//#define RA8875_WAIT()       while(!LL_GPIO_IsInputPinSet(LCD_WAIT_GPIO_Port, LCD_WAIT_Pin))
#define RA8875_GPOX(state)  FSMC_WriteRegister(0xC7, state);



struct _lcd{

    uint8_t  IsInitilized;
    __IO uint16_t BackColor;
    __IO uint16_t FontColor;
    uint8_t  Backlight;

    struct{
        uint8_t Type;
        uint8_t Width;
        uint8_t Height;
    }Font;

    uint8_t Columns;
    uint8_t Lines;
};

extern struct _lcd Display;




struct _bte{

    uint8_t Layer;
    uint16_t XStart;
    uint16_t YStart;
    uint16_t XSize;
    uint16_t YSize;
    const unsigned short* pData;
};

extern const struct _bte Background_Enot;
extern const struct _bte Background_Krym;


/* LCD color */
#define COL_WHITE          0xFFFF
#define COL_BLACK          0x0000
#define COL_GRAY           0xF7DE
#define COL_NAVY           0x001F
#define COL_BLUE           0x051F
#define COL_RED            0xF800
#define COL_MAGENTA        0xF81F
#define COL_GREEN          0x07E0
#define COL_CYAN           0x7FFF
#define COL_YELLOW         0xFFE0

#define LCD_HORISONTAL     0x00
#define LCD_VERTICAL       0x01
#define LCD_ROTATE         LCD_HORIZONTAL


/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void RA8875_Init(void);
void RA8875_Reset(void);


uint16_t FSMC_ReadPixel(uint16_t xpos, uint16_t ypos);
void FSMC_WritePixel(uint16_t xpos, uint16_t ypos, uint16_t pixel);

void RA8875_SetPwm(uint8_t ch, int pwm_duty_cycle);

void RA8875_CGROMFont(void);
void RA8875_ExtROMFont(void);
void RA8875_SetTextMode(void);
void RA8875_SetGraphicMode(void);
void RA8875_SetTextWriteCursorAbs(uint16_t x, uint16_t y);
void RA8875_SetPixelWriteCursor(uint16_t x, uint16_t y);
void RA8875_SetPixelReadCursor(uint16_t x, uint16_t y);
void RA8875_SetCursor(uint8_t xpos, uint16_t ypos);

void RA8875_ClearActiveWindow(void);



/* Display API */
void LCD_Display_OnOff(uint8_t state);
void LCD_SetBackground(uint16_t* pData, uint32_t len, uint8_t layer);
void LCD_SetForeColor(uint16_t color); //ok
void LCD_SetBackColor(uint16_t color); //ok
uint16_t LCD_GetForeColor(void);
uint16_t LCD_GetBackColor(void);
void LCD_Clear(void);
void LCD_ClearColor(uint16_t color);
void LCD_SetBacklight(uint8_t bl);
void LCD_SelectLayer(uint8_t layer);
void LCD_ShowLayer(uint8_t layer);


/* TEXT API */
void TEXT_PutString(uint8_t col, uint8_t line, const char* str);


/* DRAWING API */
void GEO_DrawLine(uint16_t Xpos_start, uint16_t Ypos_start, uint16_t Xpos_end, uint16_t Ypos_end);
void GEO_DrawTriangle(uint16_t xa, uint16_t ya, uint16_t xb, uint16_t yb, uint16_t xc, uint16_t yc, uint8_t fill);
void GEO_DrawRect(uint16_t Xpos_start, uint16_t Ypos_start, uint16_t Xpos_end, uint16_t Ypos_end, uint8_t fill);
void GEO_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t radius, uint8_t fill);
void GEO_DrawSquareOfCircleCorner(uint16_t Xpos_start, uint16_t Ypos_start, uint16_t Xpos_end, uint16_t Ypos_end, uint16_t axish, uint16_t axisy, uint8_t fill);


void RA8875_SetBTEBlock(const struct _bte* block);



/* Touchscreen  */
struct _tp{
    uint8_t        IsEnabled;
    uint8_t        IsTouched;
    uint8_t        ConvStep;

    __IO uint16_t  XAdc;
    __IO uint16_t  YAdc;
    __IO int       XPos;
    __IO int       YPos;

    char           strXPos[10];
    char           strYPos[10];
};

extern struct _tp TS_Data;


uint8_t TS_Init(void);
uint8_t TS_ReadXY(void);;


#endif /* __RA8875_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
