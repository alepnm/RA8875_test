/**
  ******************************************************************************
  * @file FSMC_LCD/src/stm3210e_lcd.c
  * @author   MCD Application Team
  * @version  V2.0.0
  * @date     04/27/2009
  * @brief    This file includes the LCD driver for AM-240320L8TNQW00H
  *           (LCD_ILI9320) and AM-240320LDTNQW00H (SPFD5408B) Liquid Crystal
  *           Display Module of STM3210E-EVAL board.
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

/** @addtogroup FSMC_LCD
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "stm3210e_lcd.h"
#include "main.h"
#include "fonts.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    __IO uint16_t LCD_REG;
    __IO uint16_t LCD_RAM;
} LCD_TypeDef;


#define Bank1_SRAM1_ADDR  ((uint32_t)0x60000000)  // NE1
#define Bank1_SRAM2_ADDR  ((uint32_t)0x64000000)  // NE2
#define Bank1_SRAM3_ADDR  ((uint32_t)0x68000000)  // NE3
#define Bank1_SRAM4_ADDR  ((uint32_t)0x6C000000)  // NE4


/* LCD is connected to the FSMC_Bank1_NOR/SRAM4 and NE1 is used as ship select signal */
//#define LCD_BASE    ((uint32_t)(Bank1_SRAM1_ADDR | 1<<18))       // RS=0|RS=1
#define LCD_BASE    ((uint32_t)(Bank1_SRAM1_ADDR | 0x00FFFFFE))  	//1111 1111 1111 1111 1111 1110
//#define LCD_BASE    ( (uint32_t)(Bank1_SRAM1_ADDR | 0x0007FFFE | 1<<18) )  	//0111 1111 1111 1111 1110
#define LCD         ((LCD_TypeDef *) LCD_BASE)

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables to set the written text color */
static  __IO uint16_t TextColor = 0x0000, BackColor = 0xFFFF;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the LCD.
  * @param  None
  * @retval : None
  */
void STM3210E_LCD_Init(void) {
    /* Configure the LCD Control pins --------------------------------------------*/
    LCD_CtrlLinesConfig();

    /* Configure the FSMC Parallel interface -------------------------------------*/
    LCD_FSMCConfig();


    LL_GPIO_SetOutputPin(LCD_RST_GPIO_Port, LCD_RST_Pin);



    LL_mDelay(50); /* delay 50 ms */


    /* PLL clock frequency */
    LCD_WriteReg(0x88, 0x0C);
    LCD_WriteReg(0x89, 0x02);
    LL_mDelay(10);


    /* color deep/MCU Interface */
    LCD_WriteReg(0x10, 0x0F);

    /* pixel clock period */
    LCD_WriteReg(0x04, 0x82);
    LL_mDelay(1);


    uint8_t qqq = LCD_ReadReg(0x88);


    /* Horisontal settings */
    LCD_WriteReg(0x14, 0x3B);
    LCD_WriteReg(0x15, 0x02);
    LCD_WriteReg(0x16, 0x03);
    LCD_WriteReg(0x17, 0x01);
    LCD_WriteReg(0x18, 0x03);

    /* Vertical settings */
    LCD_WriteReg(0x19, 0x0F);
    LCD_WriteReg(0x1A, 0x01);
    LCD_WriteReg(0x1B, 0x0F);
    LCD_WriteReg(0x1C, 0x00);
    LCD_WriteReg(0x1D, 0x0E);
    LCD_WriteReg(0x1E, 0x06);
    LCD_WriteReg(0x1F, 0x01);

    /* setting active window X */
    LCD_WriteReg(0x30, 0x00);
    LCD_WriteReg(0x31, 0x00);
    LCD_WriteReg(0x34, 0x1F);
    LCD_WriteReg(0x35, 0x03);

    /* setting active window Y */
    LCD_WriteReg(0x32, 0x00);
    LCD_WriteReg(0x33, 0x00);
    LCD_WriteReg(0x36, 0xDF);
    LCD_WriteReg(0x37, 0x01);


    /* PWM clock */
    LCD_WriteReg(0x8A, 0x81);
    LCD_WriteReg(0x8B, 0xFF);

    /* Display ON */
    LCD_WriteReg(0x01, 0x80);

    //LCD_PowerOn();

}

/**
  * @brief  Sets the Text color.
  * @param Color: specifies the Text color code RGB(5-6-5).
  *   and LCD_DrawPicture functions.
  * @retval : None
  */
void LCD_SetTextColor(__IO uint16_t Color) {
    TextColor = Color;
}

/**
  * @brief  Sets the Background color.
  * @param Color: specifies the Background color code RGB(5-6-5).
  *   LCD_DrawChar and LCD_DrawPicture functions.
  * @retval : None
  */
void LCD_SetBackColor(__IO uint16_t Color) {
    BackColor = Color;
}

/**
  * @brief  Clears the selected line.
  * @param Line: the Line to be cleared.
  *   This parameter can be one of the following values:
  * @arg Linex: where x can be 0..9
  * @retval : None
  */
void LCD_ClearLine(uint8_t Line) {
    LCD_DisplayStringLine(Line, (uint8_t*)"                    ");
}

/**
  * @brief  Clears the hole LCD.
  * @param Color: the color of the background.
  * @retval : None
  */
void LCD_Clear(uint16_t Color) {

    uint32_t index = 0;

    LCD_SetCursor(0x00, 0x013F);

    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */

    for(index = 0; index < 76800; index++) {
        LCD->LCD_RAM = Color;
    }
}

/**
  * @brief  Sets the cursor position.
  * @param Xpos: specifies the X position.
  * @param Ypos: specifies the Y position.
  * @retval : None
  */
void LCD_SetCursor(uint8_t Xpos, uint16_t Ypos) {
    LCD_WriteReg(R32, Xpos);
    LCD_WriteReg(R33, Ypos);
}

/**
  * @brief  Draws a character on LCD.
  * @param Xpos: the Line where to display the character shape.
  *   This parameter can be one of the following values:
  * @arg Linex: where x can be 0..9
  * @param Ypos: start column address.
  * @param c: pointer to the character data.
  * @retval : None
  */
void LCD_DrawChar(uint8_t Xpos, uint16_t Ypos, const uint16_t *c) {
    uint32_t index = 0, i = 0;
    uint8_t Xaddress = 0;

    Xaddress = Xpos;

    LCD_SetCursor(Xaddress, Ypos);

    for(index = 0; index < 24; index++) {
        LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
        for(i = 0; i < 16; i++) {
            if((c[index] & (1 << i)) == 0x00) {
                LCD_WriteRAM(BackColor);
            } else {
                LCD_WriteRAM(TextColor);
            }
        }
        Xaddress++;
        LCD_SetCursor(Xaddress, Ypos);
    }
}

/**
  * @brief  Displays one character (16dots width, 24dots height).
  * @param Line: the Line where to display the character shape .
  *   This parameter can be one of the following values:
  * @arg Linex: where x can be 0..9
  * @param Column: start column address.
  * @param Ascii: character ascii code, must be between 0x20 and 0x7E.
  * @retval : None
  */
void LCD_DisplayChar(uint8_t Line, uint16_t Column, uint8_t Ascii) {
    Ascii -= 32;
    LCD_DrawChar(Line, Column, &ASCII_Table[Ascii * 24]);
}

/**
  * @brief  Displays a maximum of 20 char on the LCD.
  * @param Line: the Line where to display the character shape .
  *   This parameter can be one of the following values:
  * @arg Linex: where x can be 0..9
  * @param *ptr: pointer to string to display on LCD.
  * @retval : None
  */
void LCD_DisplayStringLine(uint8_t Line, uint8_t *ptr) {
    uint32_t i = 0;
    uint16_t refcolumn = 319;

    /* Send the string character by character on lCD */
    while ((*ptr != 0) & (i < 20)) {
        /* Display one character on LCD */
        LCD_DisplayChar(Line, refcolumn, *ptr);
        /* Decrement the column position by 16 */
        refcolumn -= 16;
        /* Point on the next character */
        ptr++;
        /* Increment the character counter */
        i++;
    }
}

/**
  * @brief  Sets a display window
  * @param Xpos: specifies the X buttom left position.
  * @param Ypos: specifies the Y buttom left position.
  * @param Height: display window height.
  * @param Width: display window width.
  * @retval : None
  */
void LCD_SetDisplayWindow(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width) {
    /* Horizontal GRAM Start Address */
    if(Xpos >= Height) {
        LCD_WriteReg(R80, (Xpos - Height + 1));
    } else {
        LCD_WriteReg(R80, 0);
    }
    /* Horizontal GRAM End Address */
    LCD_WriteReg(R81, Xpos);
    /* Vertical GRAM Start Address */
    if(Ypos >= Width) {
        LCD_WriteReg(R82, (Ypos - Width + 1));
    } else {
        LCD_WriteReg(R82, 0);
    }
    /* Vertical GRAM End Address */
    LCD_WriteReg(R83, Ypos);

    LCD_SetCursor(Xpos, Ypos);
}

/**
  * @brief  Disables LCD Window mode.
  * @param  None
  * @retval : None
  */
void LCD_WindowModeDisable(void) {
    LCD_SetDisplayWindow(239, 0x13F, 240, 320);
    LCD_WriteReg(R3, 0x1018);
}

/**
  * @brief  Displays a line.
  * @param Xpos: specifies the X position.
  * @param Ypos: specifies the Y position.
  * @param Length: line length.
  * @param Direction: line direction.
  *   This parameter can be one of the following values: Vertical
  *   or Horizontal.
  * @retval : None
  */
void LCD_DrawLine(uint8_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction) {
    uint32_t i = 0;

    LCD_SetCursor(Xpos, Ypos);

    if(Direction == Horizontal) {
        LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
        for(i = 0; i < Length; i++) {
            LCD_WriteRAM(TextColor);
        }
    } else {
        for(i = 0; i < Length; i++) {
            LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
            LCD_WriteRAM(TextColor);
            Xpos++;
            LCD_SetCursor(Xpos, Ypos);
        }
    }
}

/**
  * @brief  Displays a rectangle.
  * @param Xpos: specifies the X position.
  * @param Ypos: specifies the Y position.
  * @param Height: display rectangle height.
  * @param Width: display rectangle width.
  * @retval : None
  */
void LCD_DrawRect(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width) {
    LCD_DrawLine(Xpos, Ypos, Width, Horizontal);
    LCD_DrawLine((Xpos + Height), Ypos, Width, Horizontal);

    LCD_DrawLine(Xpos, Ypos, Height, Vertical);
    LCD_DrawLine(Xpos, (Ypos - Width + 1), Height, Vertical);
}

/**
  * @brief  Displays a circle.
  * @param Xpos: specifies the X position.
  * @param Ypos: specifies the Y position.
  * @param Height: display rectangle height.
  * @param Width: display rectangle width.
  * @retval : None
  */
void LCD_DrawCircle(uint8_t Xpos, uint16_t Ypos, uint16_t Radius) {
    int32_t  D;/* Decision Variable */
    uint32_t  CurX;/* Current X Value */
    uint32_t  CurY;/* Current Y Value */

    D = 3 - (Radius << 1);
    CurX = 0;
    CurY = Radius;

    while (CurX <= CurY) {
        LCD_SetCursor(Xpos + CurX, Ypos + CurY);
        LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
        LCD_WriteRAM(TextColor);

        LCD_SetCursor(Xpos + CurX, Ypos - CurY);
        LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
        LCD_WriteRAM(TextColor);

        LCD_SetCursor(Xpos - CurX, Ypos + CurY);
        LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
        LCD_WriteRAM(TextColor);

        LCD_SetCursor(Xpos - CurX, Ypos - CurY);
        LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
        LCD_WriteRAM(TextColor);

        LCD_SetCursor(Xpos + CurY, Ypos + CurX);
        LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
        LCD_WriteRAM(TextColor);

        LCD_SetCursor(Xpos + CurY, Ypos - CurX);
        LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
        LCD_WriteRAM(TextColor);

        LCD_SetCursor(Xpos - CurY, Ypos + CurX);
        LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
        LCD_WriteRAM(TextColor);

        LCD_SetCursor(Xpos - CurY, Ypos - CurX);
        LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
        LCD_WriteRAM(TextColor);

        if (D < 0) {
            D += (CurX << 2) + 6;
        } else {
            D += ((CurX - CurY) << 2) + 10;
            CurY--;
        }
        CurX++;
    }
}

/**
  * @brief  Displays a monocolor picture.
  * @param Pict: pointer to the picture array.
  * @retval : None
  */
void LCD_DrawMonoPict(const uint32_t *Pict) {
    uint32_t index = 0, i = 0;

    LCD_SetCursor(0, 319);

    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    for(index = 0; index < 2400; index++) {
        for(i = 0; i < 32; i++) {
            if((Pict[index] & (1 << i)) == 0x00) {
                LCD_WriteRAM(BackColor);
            } else {
                LCD_WriteRAM(TextColor);
            }
        }
    }
}

/**
  * @brief  Displays a bitmap picture loaded in the internal Flash.
  * @param BmpAddress: Bmp picture address in the internal Flash.
  * @retval : None
  */
void LCD_WriteBMP(uint32_t BmpAddress) {
    uint32_t index = 0, size = 0;

    /* Read bitmap size */
    size = *(__IO uint16_t *) (BmpAddress + 2);
    size |= (*(__IO uint16_t *) (BmpAddress + 4)) << 16;

    /* Get bitmap data address offset */
    index = *(__IO uint16_t *) (BmpAddress + 10);
    index |= (*(__IO uint16_t *) (BmpAddress + 12)) << 16;

    size = (size - index)/2;

    BmpAddress += index;

    /* Set GRAM write direction and BGR = 1 */
    /* I/D=00 (Horizontal : decrement, Vertical : decrement) */
    /* AM=1 (address is updated in vertical writing direction) */
    LCD_WriteReg(R3, 0x1008);

    LCD_WriteRAM_Prepare();

    for(index = 0; index < size; index++) {
        LCD_WriteRAM(*(__IO uint16_t *)BmpAddress);
        BmpAddress += 2;
    }

    /* Set GRAM write direction and BGR = 1 */
    /* I/D = 01 (Horizontal : increment, Vertical : decrement) */
    /* AM = 1 (address is updated in vertical writing direction) */
    LCD_WriteReg(R3, 0x1018);
}

/**
  * @brief  Writes to the selected LCD register.
  * @param LCD_Reg: address of the selected register.
  * @arg LCD_RegValue: value to write to the selected register.
  * @retval : None
  */
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue) {
    LCD->LCD_REG = LCD_Reg;
    LCD->LCD_RAM = LCD_RegValue;
}

/**
  * @brief  Reads the selected LCD Register.
  * @param  None
  * @retval : LCD Register Value.
  */
uint16_t LCD_ReadReg(uint8_t LCD_Reg) {
    LCD->LCD_REG = LCD_Reg;
    return (LCD->LCD_RAM);
}

/**
  * @brief  Prepare to write to the LCD RAM.
  * @param  None
  * @retval : None
  */
void LCD_WriteRAM_Prepare(void) {
    LCD->LCD_REG = R34;
}

/**
  * @brief  Writes to the LCD RAM.
  * @param RGB_Code: the pixel color in RGB mode (5-6-5).
  * @retval : None
  */
void LCD_WriteRAM(uint16_t RGB_Code) {
    LCD->LCD_RAM = RGB_Code;
}

/**
  * @brief  Reads the LCD RAM.
  * @param  None
  * @retval : LCD RAM Value.
  */
uint16_t LCD_ReadRAM(void) {
    LCD->LCD_REG = R34; /* Select GRAM Reg */
    return LCD->LCD_RAM;
}

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval : None
  */
void LCD_PowerOn(void) {
    /* Power On sequence ---------------------------------------------------------*/
    LCD_WriteReg(R16, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_WriteReg(R17, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
    LCD_WriteReg(R18, 0x0000); /* VREG1OUT voltage */
    LCD_WriteReg(R19, 0x0000); /* VDV[4:0] for VCOM amplitude*/
    LL_mDelay(20);                 /* Dis-charge capacitor power voltage (200ms) */
    LCD_WriteReg(R16, 0x17B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_WriteReg(R17, 0x0137); /* DC1[2:0], DC0[2:0], VC[2:0] */
    LL_mDelay(5);                  /* Delay 50 ms */
    LCD_WriteReg(R18, 0x0139); /* VREG1OUT voltage */
    LL_mDelay(5);                  /* Delay 50 ms */
    LCD_WriteReg(R19, 0x1d00); /* VDV[4:0] for VCOM amplitude */
    LCD_WriteReg(R41, 0x0013); /* VCM[4:0] for VCOMH */
    LL_mDelay(5);                  /* Delay 50 ms */
    LCD_WriteReg(R7, 0x0173);  /* 262K color and display ON */
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval : None
  */
void LCD_DisplayOn(void) {
    /* Display On */
    LCD_WriteReg(R7, 0x0173); /* 262K color and display ON */
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval : None
  */
void LCD_DisplayOff(void) {
    /* Display Off */
    LCD_WriteReg(R7, 0x0);
}

/**
  * @brief  Configures LCD Control lines (FSMC Pins) in alternate function
  *   mode.
  * @param  None
  * @retval : None
  */
void LCD_CtrlLinesConfig(void)
{


}

/**
  * @brief  Configures the Parallel interface (FSMC) for LCD(Parallel mode)
  * @param  None
  * @retval : None
  */
void LCD_FSMCConfig(void)
{


}


/*  */
void LCD_Delay(uint32_t delay)
{
    while(delay--);
    return;
}



/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
