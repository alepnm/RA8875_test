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
#include "ra8875.h"
/* Private typedef -----------------------------------------------------------*/
extern GUI_CONST_STORAGE unsigned short _acenot[];
extern GUI_CONST_STORAGE unsigned short _ackrym[];


struct _lcd Display;
struct _tp TS_Data;

const struct _bte Background_Enot = {

    .Layer = 0,
    .XStart = 0,
    .YStart = 0,
    .XSize = X_SIZE,
    .YSize = Y_SIZE,
    .pData = _acenot
};

const struct _bte Background_Krym = {

    .Layer = 1,
    .XStart = 0,
    .YStart = 0,
    .XSize = X_SIZE,
    .YSize = Y_SIZE,
    .pData = _ackrym
};


/*  */
void RA8875_Init(void) {

    uint8_t RegisterTestStatus = 0, BusTestStatus = 0;

    Display.IsInitilized = 0;
    Display.FontColor = COL_WHITE;
    Display.BackColor = COL_BLACK;
    Display.Backlight = 80;
    Display.Font.Type = 0;
    Display.Font.Width = 8;
    Display.Font.Height = 16;

    Display.Columns = X_SIZE/Display.Font.Width;
    Display.Lines = Y_SIZE/Display.Font.Height;


    /* Reset */
    RA8875_Reset();

    /* Display ON */
    LCD_Display_OnOff(1);

    FSMC_WAIT_BUSY();

    /* register rw test */
    {
        uint8_t tmp1, tmp2;

        FSMC_WriteRegister(0x23, 0x55);

        FSMC_WAIT_BUSY();

        tmp1 = FSMC_ReadRegister(0x23);

        FSMC_WriteRegister(0x23, 0xAA);

        FSMC_WAIT_BUSY();

        tmp2 = FSMC_ReadRegister(0x23);

        if ((tmp1 == 0x55) && (tmp2 == 0xAA)) RegisterTestStatus = 1;

    } /* register rw test */

    HAL_Delay(50); /* delay 50 ms */


    /* PLL clock frequency */
    FSMC_WriteRegister(0x88, 0x05);   // PLL Control Register1
    HAL_Delay(1);
    FSMC_WriteRegister(0x89, 0x01);   // PLL Control Register2
    HAL_Delay(1);


    /* color deep/MCU Interface */
    FSMC_WriteRegister(0x10, 0x0F);   // System Configuration Register

    /* pixel clock period */
    FSMC_WriteRegister(0x04, 0x83);   // Pixel Clock Setting Register
    HAL_Delay(1);

    /* Serial Flash/ROM settings */
    FSMC_WriteRegister(0x05, 0x00);   // Serial Flash/ROM Configuration Register
    FSMC_WriteRegister(0x06, 0x02);   // Serial Flash/ROM CLK Setting Register

    /* Horisontal settings */
    FSMC_WriteRegister(0x14, (uint8_t)(X_SIZE/8-1));   // LCD Horizontal Display Width Register
    FSMC_WriteRegister(0x15, 0x02);   // Horizontal Non-Display Period Fine Tuning Option Register
    FSMC_WriteRegister(0x16, 0x03);   // LCD Horizontal Non-Display Period Register
    FSMC_WriteRegister(0x17, 0x01);   // HSYNC Start Position Register
    FSMC_WriteRegister(0x18, 0x03);   // HSYNC Pulse Width Register

    /* Vertical settings */
    FSMC_WriteRegister(0x19, (uint8_t)(Y_SIZE-1));        // LCD Vertical Display Height Register0
    FSMC_WriteRegister(0x1A, (uint8_t)((Y_SIZE-1)>>8));   // LCD Vertical Display Height Register1
    FSMC_WriteRegister(0x1B, 0x0F);   // LCD Vertical Non-Display Period Register0
    FSMC_WriteRegister(0x1C, 0x00);   // LCD Vertical Non-Display Period Register1

    FSMC_WriteRegister(0x1D, 0x0E);   // VSYNC Start Position Register0
    FSMC_WriteRegister(0x1E, 0x06);   // VSYNC Start Position Register1
    FSMC_WriteRegister(0x1F, 0x01);   // VSYNC Pulse Width Register

    FSMC_WriteRegister(0x20, 0x8C);   // Display Configuration Register

    /* setting active window X */
    FSMC_WriteRegister(0x30, 0x00);      // Horizontal Start Point0 of Active Window
    FSMC_WriteRegister(0x31, 0x00);      // Horizontal Start Point1 of Active Window
    FSMC_WriteRegister(0x34, (uint8_t)(X_SIZE&0x00FF)-1);    // Horizontal End Point0 of Active Window
    FSMC_WriteRegister(0x35, (uint8_t)((X_SIZE>>8)&0x03));   // Horizontal End Point1 of Active Window


    /* setting active window Y */
    FSMC_WriteRegister(0x32, 0x00);      // Vertical Start Point0 of Active Window
    FSMC_WriteRegister(0x33, 0x00);      // Vertical Start Point1 of Active Window
    FSMC_WriteRegister(0x36, (uint8_t)(Y_SIZE&0x00FF)-1);    // Vertical End Point0 of Active Window
    FSMC_WriteRegister(0x37, (uint8_t)((Y_SIZE>>8)&0x03));   // Vertical End Point1 of Active Window

    FSMC_WriteRegister(0x70, 0xA2);    // Touch Panel Control Register0
    FSMC_WriteRegister(0x71, 0x41);    // Touch Panel Control Register1
    //FSMC_WriteRegister(0xF0, 0x04);    // enable TP INT

    FSMC_WriteRegister(0x8A, 0x83);    // PWM1 Control Register
    FSMC_WriteRegister(0x8C, 0x83);    // PWM2 Control Register

    LCD_SetBacklight(100);

    /* data bus test. */
    {
        uint16_t pixel;
        uint32_t i;

        /* irasom i GRAM testinius duomenys */
        FSMC_WriteRegister(0x40, 0x00);

        RA8875_SetPixelWriteCursor(0, 0);

        LCD->LCD_REG = 0x02;

        for(i=0; i < DISPLAY_PIXELS; i++)
        {
            FSMC_WAIT_BUSY();
            LCD->LCD_RAM = i;
        }


        /* tikrinam irasytus testines duomenys */
        FSMC_WriteRegister(0x45, 0x00);

        RA8875_SetPixelWriteCursor(0, 0);

        LCD->LCD_REG = 0x02;

        FSMC_WAIT_BUSY();
        (void)LCD->LCD_RAM;// dummy read

        for(i=0; i<0x10000; i++)
        {
            FSMC_WAIT_BUSY();
            pixel = LCD->LCD_RAM;

            if(pixel != i) break;
        }


        if(i == 0x10000) BusTestStatus = 1;

        if(RegisterTestStatus != 1 || BusTestStatus != 1){

            /* set RA8875 GPIOX pin to 0 - disp panel off */
            RA8875_GPOX(0);// Extra General Purpose IO Register

            Display.IsInitilized = 0;

            // LCD inicializavimo klaida
            while(1);

        }else{

            /* set RA8875 GPIOX pin to 1 - disp panel on */
            RA8875_GPOX(1);// Extra General Purpose IO Register

            Display.IsInitilized = 1;
        }


        RA8875_ExtROMFont();
        //RA8875_CGROMFont();

        LCD_Clear();

        if(BusTestStatus) TEXT_PutString(0, 0, "BUS Test OK...");
        else TEXT_PutString(0, 0, "BUS Test FAIL...");

        if(RegisterTestStatus) TEXT_PutString(0, 1, "Register Test OK...");
        else TEXT_PutString(0, 1, "Register Test FAIL...");

        if( TS_Init() ) TEXT_PutString(0, 2, "Touchscreen initial OK...");
        else TEXT_PutString(0, 2, "Touchscreen initial FAIL...");

        HAL_Delay(1000);

    } /* data bus test. */
}



/* ok */
void RA8875_CGROMFont(void){

    Display.Font.Type = 0;
    Display.Font.Width = 8;
    Display.Font.Height = 16;

    Display.Columns = X_SIZE/Display.Font.Width;
    Display.Lines = Y_SIZE/Display.Font.Height;

    uint8_t tmp = FSMC_ReadRegister(0x21);

    CLEAR_BIT(tmp, 1<<7);
    CLEAR_BIT(tmp, 1<<5);

    FSMC_WriteRegister(0x21, tmp);
    FSMC_WriteRegister(0x22, 0x00);
    FSMC_WriteRegister(0x2E, 0x00);
    FSMC_WriteRegister(0x2F, 0x00);
}

/* ok */
void RA8875_ExtROMFont(void){

    Display.Font.Type = 1;
    Display.Font.Width = 8;
    Display.Font.Height = 12;

    Display.Columns = X_SIZE/Display.Font.Width;
    Display.Lines = Y_SIZE/Display.Font.Height;

    uint8_t tmp = FSMC_ReadRegister(0x21);

    CLEAR_BIT(tmp, 1<<7);
    SET_BIT(tmp, 1<<5);

    FSMC_WriteRegister(0x21, tmp);

    FSMC_WriteRegister(0x06, 0x03);
    FSMC_WriteRegister(0x05, 0x00);
    FSMC_WriteRegister(0x2E, 0x00);
    FSMC_WriteRegister(0x2F, 0x13);

    FSMC_WriteRegister(0x22, 0x00);
}


/* ok */
void RA8875_SetTextMode(void){

    uint8_t tmp = FSMC_ReadRegister(0x40);

    SET_BIT(tmp, 0x80);
    FSMC_WriteRegister(0x40, tmp);
}

/* ok */
void RA8875_SetGraphicMode(void){

    uint8_t tmp = FSMC_ReadRegister(0x40);

    CLEAR_BIT(tmp, 0x80);
    FSMC_WriteRegister(0x40, tmp);
}

/* texto rasymo koordinates nustatymas */
void RA8875_SetTextWriteCursorAbs(uint16_t x, uint16_t y)
{
    FSMC_WriteRegister(0x2A, x);
    FSMC_WriteRegister(0x2B, x>>8);
    FSMC_WriteRegister(0x2C, y);
    FSMC_WriteRegister(0x2D, y>>8);
}

/* pikselio rasymo i LCD RAM koordinates nustatymas  */
void RA8875_SetPixelWriteCursor(uint16_t x, uint16_t y)
{
    FSMC_WriteRegister(0x47, x>>8);
    FSMC_WriteRegister(0x46, x);
    FSMC_WriteRegister(0x49, y>>8);
    FSMC_WriteRegister(0x48, y);
}

/* pikselio skaitymo is LCD RAM koordinates nustatymas */
void RA8875_SetPixelReadCursor(uint16_t x, uint16_t y)
{
    FSMC_WriteRegister(0x4B, x>>8);
    FSMC_WriteRegister(0x4A, x);
    FSMC_WriteRegister(0x4D, y>>8);
    FSMC_WriteRegister(0x4C, y);
}


/*  */
uint16_t RA8875_ReadPixel(uint16_t xpos, uint16_t ypos){

    uint16_t pixel;

    RA8875_SetPixelReadCursor(xpos, ypos);

    FSMC_ReadDDRAM(&pixel, 1);

    return pixel;
}

/*  */
void RA8875_WritePixel(uint16_t xpos, uint16_t ypos, uint16_t pixel){

    RA8875_SetPixelWriteCursor(xpos, ypos);

    FSMC_WriteDDRAM(&pixel, 1);
}



/*   */
void RA8875_SetPwm(uint8_t ch, int pwm_duty_cycle)
{
    uint8_t reg = (ch) ? 0x8B : 0x8D;
    uint16_t value = (pwm_duty_cycle * 2.56);

    if(value > 255) value = 255;

    FSMC_WriteRegister(reg, value);
}










/* ok */
void RA8875_Reset(void)
{
    RA8875_RST_LOW();
    HAL_Delay(10);
    RA8875_RST_HIGH();

    HAL_Delay(50);
}




/* ok */
void RA8875_SetCursor(uint8_t xpos, uint16_t ypos) {

    FSMC_WriteRegister(0x32, xpos);
    FSMC_WriteRegister(0x33, ypos);
}



/*  */
void RA8875_ClearActiveWindow(void){

    FSMC_WriteRegister(0x8E, 0xC0);
    FSMC_WAIT_BUSY();
}


/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
