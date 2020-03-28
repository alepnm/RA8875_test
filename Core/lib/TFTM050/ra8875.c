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



#include "fonts.h"

/* Private typedef -----------------------------------------------------------*/


struct _lcd Display;

static void _set_gpio_od(void);
static void _set_gpio_pp(void);


/*  */
void TFTM050_Init(void) {

    uint8_t RegisterTestStatus = 0, BusTestStatus = 0, TpTestStatus = 0;

    Display.IsInitilized = 0;
    Display.FontColor = YELLOW;
    Display.BackColor = BLACK;
    Display.Backlight = 80;
    Display.Font.Type = 0;
    Display.Font.Width = 8;
    Display.Font.Height = 16;

    Display.Columns = X_SIZE/Display.Font.Width;
    Display.Lines = Y_SIZE/Display.Font.Height;


    /* Reset */
    LCD_Reset();

    /* Display ON */
    LCD_Display_OnOff(1);

    FSMC_WAIT_BUSY();

    /* register rw test */
    {
        uint8_t tmp1, tmp2;

        FSMC_WriteReg(0x23, 0x55);

        FSMC_WAIT_BUSY();

        tmp1 = FSMC_ReadReg(0x23);

        FSMC_WriteReg(0x23, 0xAA);

        FSMC_WAIT_BUSY();

        tmp2 = FSMC_ReadReg(0x23);

        if ((tmp1 == 0x55) && (tmp2 == 0xAA)) RegisterTestStatus = 1;

    } /* register rw test */

    LL_mDelay(50); /* delay 50 ms */


    /* PLL clock frequency */
    FSMC_WriteReg(0x88, 0x09);   // PLL Control Register1
    LL_mDelay(1);
    FSMC_WriteReg(0x89, 0x02);   // PLL Control Register2
    LL_mDelay(1);


    /* color deep/MCU Interface */
    FSMC_WriteReg(0x10, 0x0F);   // System Configuration Register

    /* pixel clock period */
    FSMC_WriteReg(0x04, 0x82);   // Pixel Clock Setting Register
    LL_mDelay(1);

    /* Serial Flash/ROM settings */
    FSMC_WriteReg(0x05, 0x00);   // Serial Flash/ROM Configuration Register
    FSMC_WriteReg(0x06, 0x00);   // Serial Flash/ROM CLK Setting Register

    /* Horisontal settings */
    FSMC_WriteReg(0x14, (uint8_t)(X_SIZE/8-1));   // LCD Horizontal Display Width Register
    FSMC_WriteReg(0x15, 0x02);   // Horizontal Non-Display Period Fine Tuning Option Register
    FSMC_WriteReg(0x16, 0x03);   // LCD Horizontal Non-Display Period Register
    FSMC_WriteReg(0x17, 0x01);   // HSYNC Start Position Register
    FSMC_WriteReg(0x18, 0x03);   // HSYNC Pulse Width Register

    /* Vertical settings */
    FSMC_WriteReg(0x19, (uint8_t)(Y_SIZE-1));        // LCD Vertical Display Height Register0
    FSMC_WriteReg(0x1A, (uint8_t)((Y_SIZE-1)>>8));   // LCD Vertical Display Height Register1
    FSMC_WriteReg(0x1B, 0x0F);   // LCD Vertical Non-Display Period Register0
    FSMC_WriteReg(0x1C, 0x00);   // LCD Vertical Non-Display Period Register1
    FSMC_WriteReg(0x1D, 0x0E);   // VSYNC Start Position Register0
    FSMC_WriteReg(0x1E, 0x06);   // VSYNC Start Position Register1
    FSMC_WriteReg(0x1F, 0x01);   // VSYNC Pulse Width Register

    FSMC_WriteReg(0x20, 0x8C);   // Display Configuration Register

    /* setting active window X */
    FSMC_WriteReg(0x34, (uint8_t)(X_SIZE&0x00FF)-1);      // Horizontal End Point0 of Active Window
    FSMC_WriteReg(0x35, (uint8_t)((X_SIZE>>8)&0x00FF));   // Horizontal End Point1 of Active Window


    /* setting active window Y */
    FSMC_WriteReg(0x36, (uint8_t)(Y_SIZE&0x00FF)-1);      // Vertical End Point0 of Active Window
    FSMC_WriteReg(0x37, (uint8_t)((Y_SIZE>>8)&0x00FF));   // Vertical End Point1 of Active Window


    FSMC_WriteReg(0x70, 0xA2);    // Touch Panel Control Register0
    FSMC_WriteReg(0x71, 0x41);    // Touch Panel Control Register1
    //FSMC_WriteReg(0xF0, 0x04);    // enable TP INT

    FSMC_WriteReg(0x8A, 0x83);    // PWM1 Control Register
    FSMC_WriteReg(0x8C, 0x83);    // PWM2 Control Register

    LCD_SetBacklight();

    /* data bus test. */
    {
        uint16_t pixel;
        uint32_t i;

        /* irasom i GRAM testinius duomenys */
        FSMC_WriteReg(0x40, 0x00);

        LCD_SetWriteCursor(0, 0);

        FSMC_CmdWrite(0x02);

        for(i=0; i < DISPLAY_PIXELS; i++)
        {
            FSMC_WAIT_BUSY();
            FSMC_DataWrite(i);
        }

        /* tikrinam irasytus testines duomenys */
        FSMC_WriteReg(0x45, 0x00);

        LCD_SetReadCursor(0, 0);

        FSMC_CmdWrite(0x02);

        _set_gpio_od();

        FSMC_WAIT_BUSY();

        pixel = FSMC_DataRead();/* dummy read cycle. */

        for(i=0; i<0x10000; i++)
        {
            FSMC_WAIT_BUSY();
            pixel = FSMC_DataRead();

            if(pixel != i) break;
        }

        _set_gpio_pp();


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

        //LCD_CGROMFont();
        LCD_ExtROMFont();

        LCD_Clear();
        Display.FontColor = GREEN;

        if(BusTestStatus) LCD_PutString(0, 0, "BUS Test OK...");
        else LCD_PutString(0, 0, "BUS Test FAIL...");

        if(RegisterTestStatus) LCD_PutString(0, 1, "Register Test OK...");
        else LCD_PutString(0, 1, "Register Test FAIL...");

        LL_mDelay(1000);

    } /* data bus test. */


    LCD_SetForeColor(BLUE);

}




/*  */
void LCD_CGROMFont(void){

    Display.Font.Type = 0;
    Display.Font.Width = 8;
    Display.Font.Height = 16;

    Display.Columns = X_SIZE/Display.Font.Width;
    Display.Lines = Y_SIZE/Display.Font.Height;

    uint8_t temp = FSMC_ReadReg(0x21);

    CLEAR_BIT(temp, 1<<7);
    CLEAR_BIT(temp, 1<<5);

    FSMC_WriteReg(0x21, temp);
    FSMC_WriteReg(0x22, 0x00);
    FSMC_WriteReg(0x2E, 0x00);
    FSMC_WriteReg(0x2F, 0x00);
}

/*  */
void LCD_ExtROMFont(void){

    Display.Font.Type = 1;
    Display.Font.Width = 8;
    Display.Font.Height = 12;

    Display.Columns = X_SIZE/Display.Font.Width;
    Display.Lines = Y_SIZE/Display.Font.Height;

    uint8_t temp = FSMC_ReadReg(0x21);

    CLEAR_BIT(temp, 1<<7);
    SET_BIT(temp, 1<<5);

    FSMC_WriteReg(0x21, temp);

    FSMC_WriteReg(0x06, 0x03);
    FSMC_WriteReg(0x05, 0x00);
    FSMC_WriteReg(0x2E, 0x00);
    FSMC_WriteReg(0x2F, 0x13);

    FSMC_WriteReg(0x22, 0x00);
}


/*  */
void LCD_EnterTextMode(void){

    uint8_t temp = FSMC_ReadReg(0x40);

    SET_BIT(temp, 1<<7);
    FSMC_WriteReg(0x40, temp);
}

/*  */
void LCD_ExitTextMode(void){

    uint8_t temp = FSMC_ReadReg(0x40);

    CLEAR_BIT(temp, 1<<7);
    FSMC_WriteReg(0x40, temp);
}



/* texto rasymo koordinates nustatymas */
void LCD_SetTextWriteCursorAbs(uint16_t x, uint16_t y)
{
    FSMC_WriteReg(0x2A, x);
    FSMC_WriteReg(0x2B, x>>8);
    FSMC_WriteReg(0x2C, y);
    FSMC_WriteReg(0x2D, y>>8);
}





/* pikselio rasymo i LCD RAM koordinates nustatymas  */
void LCD_SetWriteCursor(uint16_t x, uint16_t y)
{
    FSMC_WriteReg(0x47, x>>8);
    FSMC_WriteReg(0x46, x);
    FSMC_WriteReg(0x49, y>>8);
    FSMC_WriteReg(0x48, y);
}

/* pikselio skaitymo is LCD RAM koordinates nustatymas */
void LCD_SetReadCursor(uint16_t x, uint16_t y)
{
    FSMC_WriteReg(0x4B, x>>8);
    FSMC_WriteReg(0x4A, x);
    FSMC_WriteReg(0x4D, y>>8);
    FSMC_WriteReg(0x4C, y);
}






/* PWM */
/*  */
void LCD_SetBacklight(void)
{
    uint32_t value = (Display.Backlight * 256) / 100;

    if(value > 0xFF) value = 0xFF;

    FSMC_WriteReg(0x8B, value);
}

/*  */
void RA8875_SetPwm2(int pwm_duty_cycle)
{
    uint32_t value = (pwm_duty_cycle * 256) / 100;

    if(value > 0xFF) value = 0xFF;

    FSMC_WriteReg(0x8D, value);
}


/*  */
void LCD_SetPixel(const char* pixel, int x, int y)
{
    LCD_SetWriteCursor(x, y);

    FSMC_WriteReg(0x02, *(uint16_t *)pixel);
}

/*  */
void LCD_GetPixel(char* pixel, int x, int y)
{
    LCD_SetReadCursor(x, y);

    FSMC_WriteRAM_Prepare();

    _set_gpio_od();

    FSMC_WAIT_BUSY();//RA8875_WAIT();
    *(uint16_t*)pixel = FSMC_DataRead();/* dummy read */

    FSMC_WAIT_BUSY();//RA8875_WAIT();
    *(uint16_t*)pixel = FSMC_DataRead();

    _set_gpio_pp();
}

/* NE1 (PD7) - open drive */
static void _set_gpio_od(void)
{
    LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_7, LL_GPIO_PULL_UP);
}

/* NE1 (PD7) - push-pull */
static void _set_gpio_pp(void)
{
    LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_7, LL_GPIO_PULL_NO);
}




/*  */
void LCD_Display_OnOff(uint8_t state){

    uint8_t tmp = FSMC_ReadReg(0x01);

    (state) ? SET_BIT(tmp, 0x80) : CLEAR_BIT(tmp, 0x80);

    FSMC_WriteReg(0x01, tmp);
}

/*  */
void LCD_Reset(void)
{
    RA8875_RST_LOW();
    LL_mDelay(10);
    RA8875_RST_HIGH();

    LL_mDelay(50);
}



/**
  * @brief  Clears the hole LCD.
  * @param Color: the color of the background color code RGB(5-6-5).
  * @retval : None
  */
void LCD_Clear(void) {

    uint32_t index = 0;

    LCD_SetCursor(0x00, 0x00);

    FSMC_WriteRAM_Prepare(); /* Prepare to write GRAM */

    for(index = 0; index < DISPLAY_PIXELS; index++) {
        FSMC_WAIT_BUSY();
        FSMC_DataWrite(Display.BackColor);
    }
}


/*  */
void LCD_SetForeColor(uint16_t color){

    Display.FontColor = color;

    FSMC_WriteReg(0x63, (uint8_t)(color>>11));
    FSMC_WriteReg(0x64, (uint8_t)((color>>5)&0x3F));
    FSMC_WriteReg(0x65, (uint8_t)(color&0x1F));
}


/*  */
void LCD_SetBackColor(uint16_t color){

    Display.BackColor = color;

    FSMC_WriteReg(0x60, (uint8_t)(color>>11));
    FSMC_WriteReg(0x61, (uint8_t)((color>>5)&0x3F));
    FSMC_WriteReg(0x62, (uint8_t)(color&0x1F));
}





/**
  * @brief  Sets the cursor position.
  * @param xpos: specifies the X position.
  * @param ypos: specifies the Y position.
  * @retval : None
  */
void LCD_SetCursor(uint8_t xpos, uint16_t ypos) {

    FSMC_WriteReg(0x32, xpos);
    FSMC_WriteReg(0x33, ypos);
}



void LCD_PutString(uint8_t col, uint8_t line, const char* str){

    uint16_t posx = col*Display.Font.Width, posy = line*Display.Font.Height;

    LCD_SetTextWriteCursorAbs(posx, posy);

    LCD_EnterTextMode();

    FSMC_WriteRAM_Prepare();

    while(*str){
        FSMC_WAIT_BUSY();
        FSMC_DataWrite(*str++);
    }

    FSMC_WAIT_BUSY();

    LCD_ExitTextMode();
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
  * @brief  Draws a character on LCD.
  * @param Xpos: the Line where to display the character shape.
  *   This parameter can be one of the following values:
  * @arg Linex: where x can be 0..9
  * @param Ypos: start column address.
  * @param c: pointer to the character data.
  * @retval : None
  */
void LCD_DrawChar(uint16_t Xpos, uint16_t Ypos, const uint16_t *c) {

    uint32_t index = 0, i = 0;

    LCD_SetWriteCursor(Xpos, Ypos);

    for(index = 0; index < 24; index++) {

        FSMC_WriteRAM_Prepare(); /* Prepare to write GRAM */

        for(i = 0; i < 16; i++) {
            if((c[index] & (1 << i)) == 0x00) {
                FSMC_DataWrite(BLACK);
            } else {
                FSMC_DataWrite(WHITE);
            }
        }

        LCD_SetWriteCursor(Xpos, ++Ypos);
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
    LCD_DrawChar(Column, Line, &ASCII_Table[Ascii * 24]);
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
    uint16_t refcolumn = 0;

    /* Send the string character by character on lCD */
    while ((*ptr != 0) & (i < 20)) {
        /* Display one character on LCD */
        LCD_DisplayChar(Line*12, refcolumn, *ptr);
        /* Decrement the column position by 16 */
        refcolumn += 18;
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
        FSMC_WriteReg(0x80, (Xpos - Height + 1));
    } else {
        FSMC_WriteReg(0x80, 0);
    }

    /* Horizontal GRAM End Address */
    FSMC_WriteReg(0x81, Xpos);

    /* Vertical GRAM Start Address */
    if(Ypos >= Width) {
        FSMC_WriteReg(0x82, (Ypos - Width + 1));
    } else {
        FSMC_WriteReg(0x82, 0);
    }

    /* Vertical GRAM End Address */
    FSMC_WriteReg(0x83, Ypos);

    LCD_SetCursor(Xpos, Ypos);
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

    LCD_SetWriteCursor(Xpos, Ypos);

    if(Direction == LCD_HORISONTAL) {

        FSMC_WriteRAM_Prepare(); /* Prepare to write GRAM */

        for(i = 0; i < Length; i++) {
            FSMC_DataWrite(Display.FontColor);
        }
    } else {

        for(i = 0; i < Length; i++) {

            FSMC_WriteRAM_Prepare(); /* Prepare to write GRAM */

            FSMC_DataWrite(Display.FontColor);

            Ypos++;

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
    LCD_DrawLine(Xpos, Ypos, Width, LCD_HORISONTAL);
    LCD_DrawLine((Xpos + Height), Ypos, Width, LCD_HORISONTAL);

    LCD_DrawLine(Xpos, Ypos, Height, LCD_VERTICAL);
    LCD_DrawLine(Xpos, (Ypos - Width + 1), Height, LCD_VERTICAL);
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
        FSMC_WriteRAM_Prepare(); /* Prepare to write GRAM */
        FSMC_DataWrite(WHITE);

        LCD_SetCursor(Xpos + CurX, Ypos - CurY);
        FSMC_WriteRAM_Prepare(); /* Prepare to write GRAM */
        FSMC_DataWrite(WHITE);

        LCD_SetCursor(Xpos - CurX, Ypos + CurY);
        FSMC_WriteRAM_Prepare(); /* Prepare to write GRAM */
        FSMC_DataWrite(WHITE);

        LCD_SetCursor(Xpos - CurX, Ypos - CurY);
        FSMC_WriteRAM_Prepare(); /* Prepare to write GRAM */
        FSMC_DataWrite(WHITE);

        LCD_SetCursor(Xpos + CurY, Ypos + CurX);
        FSMC_WriteRAM_Prepare(); /* Prepare to write GRAM */
        FSMC_DataWrite(WHITE);

        LCD_SetCursor(Xpos + CurY, Ypos - CurX);
        FSMC_WriteRAM_Prepare(); /* Prepare to write GRAM */
        FSMC_DataWrite(WHITE);

        LCD_SetCursor(Xpos - CurY, Ypos + CurX);
        FSMC_WriteRAM_Prepare(); /* Prepare to write GRAM */
        FSMC_DataWrite(WHITE);

        LCD_SetCursor(Xpos - CurY, Ypos - CurX);
        FSMC_WriteRAM_Prepare(); /* Prepare to write GRAM */
        FSMC_DataWrite(WHITE);

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

    LCD_SetWriteCursor(0, 319);

    FSMC_WriteRAM_Prepare(); /* Prepare to write GRAM */

    for(index = 0; index < 2400; index++) {
        for(i = 0; i < 32; i++) {
            if((Pict[index] & (1 << i)) == 0x00) {
                FSMC_DataWrite(BLACK);
            } else {
                FSMC_DataWrite(WHITE);
            }
        }
    }
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
