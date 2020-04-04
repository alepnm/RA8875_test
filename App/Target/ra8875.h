
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RA8875_H
#define __RA8875_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define X_SIZE 480
#define Y_SIZE 272
#define DISPLAY_PIXELS (X_SIZE*Y_SIZE)


#define RA8875_RST_LOW()    LL_GPIO_ResetOutputPin(LCD_RST_GPIO_Port, LCD_RST_Pin)
#define RA8875_RST_HIGH()   LL_GPIO_SetOutputPin(LCD_RST_GPIO_Port, LCD_RST_Pin)
#define RA8875_WAIT()       while(!LL_GPIO_IsInputPinSet(LCD_WAIT_GPIO_Port, LCD_WAIT_Pin))
#define RA8875_GPOX(state)  FSMC_WriteReg(0xC7, state);


#define Bank1_SRAM1_ADDR  ((uint32_t)0x60000000)  // NE1
#define Bank1_SRAM2_ADDR  ((uint32_t)0x64000000)  // NE2
#define Bank1_SRAM3_ADDR  ((uint32_t)0x68000000)  // NE3
#define Bank1_SRAM4_ADDR  ((uint32_t)0x6C000000)  // NE4

typedef struct {
    __IO uint16_t LCD_REG;
    __IO uint16_t LCD_RAM;
}LCD_TypeDef;

/* LCD is connected to the FSMC_Bank1_NOR/SRAM1 and NE1 is used as ship select signal */
#define LCD_BASE    ((uint32_t)(Bank1_SRAM1_ADDR | 0x000FFFFE))
#define LCD         ((LCD_TypeDef *) LCD_BASE)



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

extern char str[];


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

#define LCD_HORISONTAL 0x00
#define LCD_VERTICAL   0x01
#define LCD_ROTATE     LCD_HORIZONTAL


/* Exported macro ------------------------------------------------------------*/

#define FSMC_WAIT_BUSY() while((FSMC_ReadStatus()&0x80) == 0x80)


/*  */
static inline uint16_t FSMC_ReadStatus(void){

    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
    //LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);

    //FSMC_WAIT_BUSY();
    uint16_t status = LCD->LCD_RAM;

    //LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);

    return status;
}



/*  */
static inline void FSMC_CmdWrite( uint8_t cmd){

    LCD->LCD_REG = cmd;
}

/*  */
static inline void FSMC_WriteRAM_Prepare(void){

    LCD->LCD_REG = 0x02;
    FSMC_WAIT_BUSY();
}

/*  */
static inline void FSMC_DataWrite(uint16_t data){

    LCD->LCD_RAM = data;
    FSMC_WAIT_BUSY();
}

/*  */
static inline uint16_t FSMC_DataRead(void){

    //FSMC_WAIT_BUSY();
    return LCD->LCD_RAM;
}

/*  */
static inline void FSMC_WriteReg(uint8_t reg, uint8_t val) {

    LCD->LCD_REG = reg;
    LCD->LCD_RAM = val;
}

/*  */
static inline uint16_t FSMC_ReadReg(uint8_t reg) {

    LCD->LCD_REG = reg;
    return LCD->LCD_RAM;
}

/*  */
static inline uint16_t FSMC_ReadRAM(void) {

    LCD->LCD_REG = 0x02;
    //FSMC_WAIT_BUSY();
    return LCD->LCD_RAM;
}



/* Exported functions ------------------------------------------------------- */
void RA8875_Init(void);
void RA8875_Display_OnOff(uint8_t state);
void RA8875_Reset(void);
void RA8875_ClearScreen(void);
void RA8875_ClearScreenColor(uint16_t color);
void RA8875_SetBacklight(void); //ok
void RA8875_SetPwm2(int pwm_duty_cycle);

void RA8875_CGROMFont(void);
void RA8875_ExtROMFont(void);
void RA8875_EnterTextMode(void);
void RA8875_ExitTextMode(void);
void RA8875_SetTextWriteCursorAbs(uint16_t x, uint16_t y);

void RA8875_SetPixelWriteCursor(uint16_t x, uint16_t y);
void RA8875_SetReadCursor(uint16_t x, uint16_t y);


void RA8875_SetForeColor(uint16_t color); //ok
void RA8875_SetBackColor(uint16_t color); //ok
void RA8875_SetCursor(uint8_t xpos, uint16_t ypos);
void RA8875_PutString(uint8_t col, uint8_t line, const char* str);


void RA8875_DrawLine(uint16_t Xpos_start, uint16_t Ypos_start, uint16_t Xpos_end, uint16_t Ypos_end);
void RA8875_DrawTriangle(uint16_t xa, uint16_t ya, uint16_t xb, uint16_t yb, uint16_t xc, uint16_t yc, uint8_t fill);
void RA8875_DrawRect(uint16_t Xpos_start, uint16_t Ypos_start, uint16_t Xpos_end, uint16_t Ypos_end, uint8_t fill);
void RA8875_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t radius, uint8_t fill);
void RA8875_DrawSquareOfCircleCorner(uint16_t Xpos_start, uint16_t Ypos_start, uint16_t Xpos_end, uint16_t Ypos_end, uint16_t axish, uint16_t axisy, uint8_t fill);


void RA8875_ClearActiveWindow(void);
void RA8875_SetDisplayWindow(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width);
void RA8875_WindowModeDisable(void);




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
