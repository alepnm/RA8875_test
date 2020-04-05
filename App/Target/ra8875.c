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
#include "main.h"

/* Private typedef -----------------------------------------------------------*/


struct _lcd Display;
struct _tp TS_Data;

char str[40];


/*  */
void RA8875_Init(void) {

    uint8_t RegisterTestStatus = 0, BusTestStatus = 0;

    Display.IsInitilized = 0;
    Display.FontColor = WHITE;
    Display.BackColor = BLACK;
    Display.Backlight = 80;
    Display.Font.Type = 0;
    Display.Font.Width = 8;
    Display.Font.Height = 16;

    Display.Columns = X_SIZE/Display.Font.Width;
    Display.Lines = Y_SIZE/Display.Font.Height;


    /* Reset */
    RA8875_Reset();

    /* Display ON */
    RA8875_Display_OnOff(1);

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
    FSMC_WriteRegister(0x06, 0x00);   // Serial Flash/ROM CLK Setting Register

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
    FSMC_WriteRegister(0x34, (uint8_t)(X_SIZE&0x00FF)-1);      // Horizontal End Point0 of Active Window
    FSMC_WriteRegister(0x35, (uint8_t)((X_SIZE>>8)&0x00FF));   // Horizontal End Point1 of Active Window


    /* setting active window Y */
    FSMC_WriteRegister(0x36, (uint8_t)(Y_SIZE&0x00FF)-1);      // Vertical End Point0 of Active Window
    FSMC_WriteRegister(0x37, (uint8_t)((Y_SIZE>>8)&0x00FF));   // Vertical End Point1 of Active Window


    FSMC_WriteRegister(0x70, 0xA2);    // Touch Panel Control Register0
    FSMC_WriteRegister(0x71, 0x41);    // Touch Panel Control Register1
    //FSMC_WriteRegister(0xF0, 0x04);    // enable TP INT

    FSMC_WriteRegister(0x8A, 0x83);    // PWM1 Control Register
    FSMC_WriteRegister(0x8C, 0x83);    // PWM2 Control Register

    RA8875_SetBacklight();

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

        RA8875_ClearScreen();

        if(BusTestStatus) RA8875_PutString(0, 0, "BUS Test OK...");
        else RA8875_PutString(0, 0, "BUS Test FAIL...");

        if(RegisterTestStatus) RA8875_PutString(0, 1, "Register Test OK...");
        else RA8875_PutString(0, 1, "Register Test FAIL...");

        if( TS_Init() ) RA8875_PutString(0, 2, "Touchscreen initial OK...");
        else RA8875_PutString(0, 2, "Touchscreen initial FAIL...");

        HAL_Delay(1000);

    } /* data bus test. */
}


/*  */
void FSMC_ReadDDRAM(uint16_t *pdata, int pixels){

    LCD->LCD_REG = 0x02;
    FSMC_WAIT_BUSY();
    (void)LCD->LCD_RAM;// dummy read

    while(pixels--){
        FSMC_WAIT_BUSY();
        *pdata = LCD->LCD_RAM;
        pdata++;
    }
}

/*  */
void FSMC_WriteDDRAM(uint16_t *pdata, int pixels){

    LCD->LCD_REG = 0x02;

    while(pixels--){
        FSMC_WAIT_BUSY();
        LCD->LCD_RAM = *pdata;
        pdata++;
    }
}




/* ok */
void RA8875_CGROMFont(void){

    Display.Font.Type = 0;
    Display.Font.Width = 8;
    Display.Font.Height = 16;

    Display.Columns = X_SIZE/Display.Font.Width;
    Display.Lines = Y_SIZE/Display.Font.Height;

    uint8_t temp = FSMC_ReadRegister(0x21);

    CLEAR_BIT(temp, 1<<7);
    CLEAR_BIT(temp, 1<<5);

    FSMC_WriteRegister(0x21, temp);
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

    uint8_t temp = FSMC_ReadRegister(0x21);

    CLEAR_BIT(temp, 1<<7);
    SET_BIT(temp, 1<<5);

    FSMC_WriteRegister(0x21, temp);

    FSMC_WriteRegister(0x06, 0x03);
    FSMC_WriteRegister(0x05, 0x00);
    FSMC_WriteRegister(0x2E, 0x00);
    FSMC_WriteRegister(0x2F, 0x13);

    FSMC_WriteRegister(0x22, 0x00);
}


/* ok */
void RA8875_EnterTextMode(void){

    uint8_t temp = FSMC_ReadRegister(0x40);

    SET_BIT(temp, 0x80);
    FSMC_WriteRegister(0x40, temp);
}

/* ok */
void RA8875_ExitTextMode(void){

    uint8_t temp = FSMC_ReadRegister(0x40);

    CLEAR_BIT(temp, 0x80);
    FSMC_WriteRegister(0x40, temp);
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



/* PWM */
/*  */
void RA8875_SetBacklight(void)
{
    uint32_t value = (Display.Backlight * 256) / 100;

    if(value > 0xFF) value = 0xFF;

    FSMC_WriteRegister(0x8B, value);
}

/*  */
void RA8875_SetPwm2(int pwm_duty_cycle)
{
    uint32_t value = (pwm_duty_cycle * 256) / 100;

    if(value > 0xFF) value = 0xFF;

    FSMC_WriteRegister(0x8D, value);
}










/*  */
void RA8875_Display_OnOff(uint8_t state){

    uint8_t tmp = FSMC_ReadRegister(0x01);

    (state) ? SET_BIT(tmp, 0x80) : CLEAR_BIT(tmp, 0x80);

    FSMC_WriteRegister(0x01, tmp);
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
void RA8875_ClearScreenColor(uint16_t color) {

    uint32_t index = 0;

    RA8875_SetCursor(0x00, 0x00);

    LCD->LCD_REG = 0x02;

    for(index = 0; index < DISPLAY_PIXELS; index++) {
        FSMC_WAIT_BUSY();
        LCD->LCD_RAM = color;
    }
}


/* ok */
void RA8875_SetForeColor(uint16_t color){

    Display.FontColor = color;

    FSMC_WriteRegister(0x63, (uint8_t)(color>>11));
    FSMC_WriteRegister(0x64, (uint8_t)((color>>5)&0x3F));
    FSMC_WriteRegister(0x65, (uint8_t)(color&0x1F));
}


/* ok */
void RA8875_SetBackColor(uint16_t color){

    Display.BackColor = color;

    FSMC_WriteRegister(0x60, (uint8_t)(color>>11));
    FSMC_WriteRegister(0x61, (uint8_t)((color>>5)&0x3F));
    FSMC_WriteRegister(0x62, (uint8_t)(color&0x1F));
}





/* ok */
void RA8875_SetCursor(uint8_t xpos, uint16_t ypos) {

    FSMC_WriteRegister(0x32, xpos);
    FSMC_WriteRegister(0x33, ypos);
}


/* ok */
void RA8875_PutString(uint8_t col, uint8_t line, const char* str){

    uint16_t posx = col*Display.Font.Width, posy = line*Display.Font.Height;

    RA8875_SetTextWriteCursorAbs(posx, posy);

    RA8875_EnterTextMode();

    LCD->LCD_REG = 0x02;

    while(*str){
        FSMC_WAIT_BUSY();
        LCD->LCD_RAM = *str++;
    }

    FSMC_WAIT_BUSY();

    RA8875_ExitTextMode();
}
















/**
  * @brief  Sets a display window
  * @param Xpos: specifies the X buttom left position.
  * @param Ypos: specifies the Y buttom left position.
  * @param Height: display window height.
  * @param Width: display window width.
  * @retval : None
  */
void RA8875_SetDisplayWindow(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width) {

    /* Horizontal GRAM Start Address */
    if(Xpos >= Height) {
        FSMC_WriteRegister(0x80, (Xpos - Height + 1));
    } else {
        FSMC_WriteRegister(0x80, 0);
    }

    /* Horizontal GRAM End Address */
    FSMC_WriteRegister(0x81, Xpos);

    /* Vertical GRAM Start Address */
    if(Ypos >= Width) {
        FSMC_WriteRegister(0x82, (Ypos - Width + 1));
    } else {
        FSMC_WriteRegister(0x82, 0);
    }

    /* Vertical GRAM End Address */
    FSMC_WriteRegister(0x83, Ypos);

    RA8875_SetCursor(Xpos, Ypos);
}








/* ok */
void RA8875_ClearScreen(void){

    FSMC_WriteRegister(0x8E, 0x80);
    while((FSMC_ReadRegister(0x8E)&0x80) == 0x80) FSMC_WAIT_BUSY();
}

/*  */
void RA8875_ClearActiveWindow(void){

    FSMC_WriteRegister(0x8E, 0xC0);
    while((FSMC_ReadRegister(0x8E)&0x80) == 0x80) FSMC_WAIT_BUSY();
}


/*  Draw API  */

/* ok */
void RA8875_DrawLine(uint16_t Xpos_start, uint16_t Ypos_start, uint16_t Xpos_end, uint16_t Ypos_end) {

    FSMC_WriteRegister(0x91, Xpos_start&0xFF);
    FSMC_WriteRegister(0x92, Xpos_start>>0x08);

    FSMC_WriteRegister(0x93, Ypos_start&0xFF);
    FSMC_WriteRegister(0x94, Ypos_start>>0x08);

    FSMC_WriteRegister(0x95, Xpos_end&0xFF);
    FSMC_WriteRegister(0x96, Xpos_end>>0x08);

    FSMC_WriteRegister(0x97, Ypos_end&0xFF);
    FSMC_WriteRegister(0x98, Ypos_end>>0x08);

    FSMC_WriteRegister(0x90, 0x80);

    while((FSMC_ReadRegister(0x90)&0x80) == 0x80) FSMC_WAIT_BUSY();
}


/* ok */
void RA8875_DrawRect(uint16_t Xpos_start, uint16_t Ypos_start, uint16_t Xpos_end, uint16_t Ypos_end, uint8_t fill) {

    if(fill) fill = 0x20;

    FSMC_WriteRegister(0x91, Xpos_start&0xFF);
    FSMC_WriteRegister(0x92, Xpos_start>>0x08);

    FSMC_WriteRegister(0x93, Ypos_start&0xFF);
    FSMC_WriteRegister(0x94, Ypos_start>>0x08);

    FSMC_WriteRegister(0x95, Xpos_end&0xFF);
    FSMC_WriteRegister(0x96, Xpos_end>>0x08);

    FSMC_WriteRegister(0x97, Ypos_end&0xFF);
    FSMC_WriteRegister(0x98, Ypos_end>>0x08);

    FSMC_WriteRegister(0x90, (0x90|fill));

    while((FSMC_ReadRegister(0x90)&0x80) == 0x80) FSMC_WAIT_BUSY();
}


/* ok */
void RA8875_DrawTriangle(uint16_t xa, uint16_t ya, uint16_t xb, uint16_t yb, uint16_t xc, uint16_t yc, uint8_t fill) {

    if(fill) fill = 0x20;

    FSMC_WriteRegister(0x91, xa&0xFF);
    FSMC_WriteRegister(0x92, xa>>0x08);

    FSMC_WriteRegister(0x93, ya&0xFF);
    FSMC_WriteRegister(0x94, ya>>0x08);

    FSMC_WriteRegister(0x95, xb&0xFF);
    FSMC_WriteRegister(0x96, xb>>0x08);

    FSMC_WriteRegister(0x97, yb&0xFF);
    FSMC_WriteRegister(0x98, yb>>0x08);

    FSMC_WriteRegister(0xA9, xc&0xFF);
    FSMC_WriteRegister(0xAA, xc>>0x08);

    FSMC_WriteRegister(0xAB, yc&0xFF);
    FSMC_WriteRegister(0xAC, yc>>0x08);

    FSMC_WriteRegister(0x90, (0x81|fill));

    while((FSMC_ReadRegister(0x90)&0x80) == 0x80) FSMC_WAIT_BUSY();
}


/* ok */
void RA8875_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t radius, uint8_t fill) {

    if(fill) fill = 0x20;

    /* horizontal position */
    FSMC_WriteRegister(0x99, Xpos&0xFF);
    FSMC_WriteRegister(0x9A, Xpos>>0x08);

    /* vertical position */
    FSMC_WriteRegister(0x9B, Ypos&0xFF);
    FSMC_WriteRegister(0x9C, Ypos>>0x08);

    /* circle radius */
    FSMC_WriteRegister(0x9D, radius);

    /* start draw */
    FSMC_WriteRegister(0x90, (0x40|fill));

    while((FSMC_ReadRegister(0x90)&0x40) == 0x40) FSMC_WAIT_BUSY();
}


/* ok */
void RA8875_DrawSquareOfCircleCorner(uint16_t Xpos_start, uint16_t Ypos_start, uint16_t Xpos_end, uint16_t Ypos_end, uint16_t axish, uint16_t axisy, uint8_t fill) {

    if(fill) fill = 0x40;

    FSMC_WriteRegister(0x91, Xpos_start&0xFF);
    FSMC_WriteRegister(0x92, Xpos_start>>0x08);

    FSMC_WriteRegister(0x93, Ypos_start&0xFF);
    FSMC_WriteRegister(0x94, Ypos_start>>0x08);

    FSMC_WriteRegister(0x95, Xpos_end&0xFF);
    FSMC_WriteRegister(0x96, Xpos_end>>0x08);

    FSMC_WriteRegister(0x97, Ypos_end&0xFF);
    FSMC_WriteRegister(0x98, Ypos_end>>0x08);

    FSMC_WriteRegister(0xA1, axish&0xFF);
    FSMC_WriteRegister(0xA2, axish>>0x08);

    FSMC_WriteRegister(0xA3, axisy&0xFF);
    FSMC_WriteRegister(0xA4, axisy>>0x08);

    FSMC_WriteRegister(0xA0, (0xA0|fill));

    while((FSMC_ReadRegister(0xA0)&0x80) == 0x80) FSMC_WAIT_BUSY();
}






/*  */
uint8_t TS_Init(void){

    TS_Data.IsTouched = 0;
    TS_Data.XPos = 0;
    TS_Data.YPos = 0;

    FSMC_WriteRegister(0x71, 0x41);

    if(FSMC_ReadRegister(0x71) == (0x41|0x80)){
        TS_Data.IsEnabled = 1;
        return 1;
    }
    else{
        TS_Data.IsEnabled = 0;
        return 0;
    }
}


/* skaitom x-y reiksmes */
uint8_t TS_ReadXY(void){

    TS_Data.IsTouched = ((FSMC_ReadRegister(0x74)&0x80) == 0x80) ? 0 : 1;

    if(TS_Data.IsTouched){

        FSMC_WriteRegister(0x71, (0x42|0x04));
        HAL_Delay(1);

        FSMC_WriteRegister(0x71, (0x43|0x04));
        HAL_Delay(1);

        FSMC_WriteRegister(0x71, 0x40);

        TS_Data.XAdc = (FSMC_ReadRegister(0x72)<<2); // TS_Data.XAdc = ( (FSMC_ReadRegister(0x72)<<2) | (FSMC_ReadRegister(0x74)&0x03) );
        TS_Data.YAdc = (FSMC_ReadRegister(0x73)<<2); // TS_Data.YAdc = ( (FSMC_ReadRegister(0x73)<<2) | ((FSMC_ReadRegister(0x74)&0x0C)>>2) );

        TS_Data.XPos = (TS_Data.XAdc-50) * X_SIZE / 920;
        TS_Data.YPos = (TS_Data.YAdc-80) * Y_SIZE / 820;

        FSMC_WriteRegister(0x71, 0x41);

    }else{

        TS_Data.XPos = -1;
        TS_Data.YPos = -1;
    }

    return TS_Data.IsTouched;
}



/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
