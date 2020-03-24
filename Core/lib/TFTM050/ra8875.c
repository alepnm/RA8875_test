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

typedef struct {
    __IO uint16_t LCD_REG;
    __IO uint16_t LCD_RAM;
}LCD_TypeDef;


#define Bank1_SRAM1_ADDR  ((uint32_t)0x60000000)  // NE1
#define Bank1_SRAM2_ADDR  ((uint32_t)0x64000000)  // NE2
#define Bank1_SRAM3_ADDR  ((uint32_t)0x68000000)  // NE3
#define Bank1_SRAM4_ADDR  ((uint32_t)0x6C000000)  // NE4


/* LCD is connected to the FSMC_Bank1_NOR/SRAM1 and NE1 is used as ship select signal */
#define LCD_BASE    ((uint32_t)(Bank1_SRAM1_ADDR | 0x000FFFFE))
#define LCD         ((LCD_TypeDef *) LCD_BASE)

#define RA8875_CmdWrite(cmd)       LCD->LCD_REG = cmd
#define RA8875_WriteRAM_Prepare()  LCD->LCD_REG = 0x02
#define RA8875_DataWrite(data)     LCD->LCD_RAM = data
#define RA8875_DataRead()          LCD->LCD_RAM

#define LCD_WAIT()              while(!LL_GPIO_IsInputPinSet(LCD_WAIT_GPIO_Port, LCD_WAIT_Pin))

#define TEST_FAIL   0
#define TEST_PASS   1




static __IO uint16_t X_Size = 480, Y_Size = 272;
static __IO uint16_t ForeColor = 0x0000, BackColor = 0xFFFF;

static uint8_t RegisterTestStatus = 0, BusTestStatus = 0;


static void _set_gpio_od(void);
static void _set_gpio_pp(void);
static void _lcd_reset(void);

static void     RA8875_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue);
static uint16_t RA8875_ReadReg(uint8_t LCD_Reg);
static uint16_t RA8875_ReadRAM(void);
static uint8_t  RA8875_ReadStatus(void);


/*  */
void TFTM050_Init(void) {

    /* Reset */
    _lcd_reset();

    /* register rw test */
    {
        uint8_t tmp1, tmp2;

        RA8875_WriteReg(0x23, 0x55);

        LCD_WAIT();

        tmp1 = RA8875_ReadReg(0x23);

        RA8875_WriteReg(0x23, 0xAA);

        LCD_WAIT();

        tmp2 = RA8875_ReadReg(0x23);

        if ((tmp1 == 0x55) && (tmp2 == 0xAA)) RegisterTestStatus = TEST_PASS;

    } /* register rw test */



    LL_mDelay(50); /* delay 50 ms */

    /* PLL clock frequency */
    RA8875_WriteReg(0x88, 0x0C);   // PLL Control Register1
    LL_mDelay(1);
    RA8875_WriteReg(0x89, 0x02);   // PLL Control Register2
    LL_mDelay(1);


    /* color deep/MCU Interface */
    RA8875_WriteReg(0x10, 0x0F);   // System Configuration Register

    /* pixel clock period */
    RA8875_WriteReg(0x04, 0x82);   // Pixel Clock Setting Register
    LL_mDelay(1);

    /* Horisontal settings */
    RA8875_WriteReg(0x14, 0x3B);   // LCD Horizontal Display Width Register
    RA8875_WriteReg(0x15, 0x02);   // Horizontal Non-Display Period Fine Tuning Option Register
    RA8875_WriteReg(0x16, 0x03);   // LCD Horizontal Non-Display Period Register
    RA8875_WriteReg(0x17, 0x01);   // HSYNC Start Position Register
    RA8875_WriteReg(0x18, 0x03);   // HSYNC Pulse Width Register

    /* Vertical settings */
    RA8875_WriteReg(0x19, 0x0F);   // LCD Vertical Display Height Register
    RA8875_WriteReg(0x1A, 0x01);   // LCD Vertical Display Height Register0
    RA8875_WriteReg(0x1B, 0x0F);   // LCD Vertical Non-Display Period Register
    RA8875_WriteReg(0x1C, 0x00);   // LCD Vertical Non-Display Period Register
    RA8875_WriteReg(0x1D, 0x0E);   // VSYNC Start Position Register
    RA8875_WriteReg(0x1E, 0x06);   // VSYNC Start Position Register
    RA8875_WriteReg(0x1F, 0x01);   // VSYNC Pulse Width Register


    /* setting active window X */
    RA8875_WriteReg(0x34, (uint8_t)(X_Size&0x00FF)-1);      // Horizontal End Point0 of Active Window
    RA8875_WriteReg(0x35, (uint8_t)((X_Size>>8)&0x00FF));   // Horizontal End Point1 of Active Window


    /* setting active window Y */
    RA8875_WriteReg(0x36, (uint8_t)(Y_Size&0x00FF)-1);      // Vertical End Point0 of Active Window
    RA8875_WriteReg(0x37, (uint8_t)((Y_Size>>8)&0x00FF));   // Vertical End Point1 of Active Window


    RA8875_WriteReg(0x70, 0xD7);    // enable Touch Panel
    RA8875_WriteReg(0x71, 0x01);    // set Auto Mode
    RA8875_WriteReg(0xF0, 0x04);    // enable TP INT


    LCD_SetPwm1(80);

    /* set lift right */
    RA8875_WriteReg(0x20, 0x80);   // Display Configuration Register: 2 layers

    /* Display ON */
    RA8875_WriteReg(0x01, 0x80);   // Power and Display Control Register



    /* data bus test. */
    {
        uint16_t pixel;
        uint32_t i;

        /* irasom i GRAM testinius duomenys */
        RA8875_WriteReg(0x40, 0x00);

        LCD_SetWriteCursor(0, 0);

        RA8875_CmdWrite(0x02);

        for(i=0; i<X_Size*Y_Size; i++)
        {
            RA8875_DataWrite(i);
            LCD_WAIT();
        }

        /* tikrinam irasytus testines duomenys */
        RA8875_WriteReg(0x45, 0x00);

        LCD_SetReadCursor(0, 0);

        RA8875_CmdWrite(0x02);

        _set_gpio_od();

        pixel = RA8875_DataRead();/* dummy read cycle. */

        for(i=0; i<0x10000; i++)
        {
            pixel = RA8875_DataRead();

            if(pixel != i) break;
        }

        _set_gpio_pp();


        if(i == 0x10000) BusTestStatus = TEST_PASS;

        if(RegisterTestStatus != TEST_PASS || BusTestStatus != TEST_PASS){
            // LCD test fault
            LCD_Clear(RED);

            /* set RA8875 GPIOX pin to 0 - disp panel off */
            RA8875_WriteReg(0xC7, 0x00);   // Extra General Purpose IO Register

        }else{

            LCD_Clear(GREEN);

            /* set RA8875 GPIOX pin to 1 - disp panel on */
            RA8875_WriteReg(0xC7, 0x01);   // Extra General Purpose IO Register
        }

        LL_mDelay(3000);

    } /* data bus test. */



    LCD_Clear(BLACK);

    LCD_WriteForeColor(0xFF, 0xFF, 0xFF);
    LCD_WriteBackColor(0x00, 0x00, 0x00);

    //LCD_CGROMFont(0);
    LCD_ExtROMFont(0);

    //LCD_PutString(100, 0, "qwertyuiopasdfghjkl");
    //LCD_PutString(100, 16, "123456789");

    LCD_PrintString(0, 1, "QWERTYUIOPASDFGHJKLZXCVBNM");
    LCD_PrintString(0, 2, "qwertyuiopasdfghjklzxcvbnm");
    LCD_PrintString(1, 3, "0123456789");


    LL_mDelay(2000);








}



/*  */
uint8_t TP_Check(void){

    return ((RA8875_ReadStatus()&0x20) == 0x20) ? 1 : 0;
}

void TP_Clear_IRQ(void){

    RA8875_WriteReg(0xF1, 0x00);
}







/* Display ON */
void LCD_DisplayOn(void) {

    RA8875_WriteReg(0x07, 0x0173);
}


/* Display OFF */
void LCD_DisplayOff(void) {

    RA8875_WriteReg(0x07, 0x0);
}











/*  */
void LCD_CGROMFont(uint8_t font){

    uint8_t temp = RA8875_ReadReg(0x21);

    CLEAR_BIT(temp, 1<<7);
    CLEAR_BIT(temp, 1<<5);

    RA8875_WriteReg(0x21, temp|0x00);
    RA8875_WriteReg(0x22, 0x00);
    RA8875_WriteReg(0x2F, 0x00);
}

/*  */
void LCD_ExtROMFont(uint8_t font){

    uint8_t temp = RA8875_ReadReg(0x21);

    CLEAR_BIT(temp, 1<<7);
    SET_BIT(temp, 1<<5);

    RA8875_WriteReg(0x21, temp);

    RA8875_WriteReg(0x06, 0x03);
    RA8875_WriteReg(0x05, 0x00);
    RA8875_WriteReg(0x2E, 0x00);
    RA8875_WriteReg(0x2F, 0x13);

    RA8875_WriteReg(0x22, 0x00);
}


/*  */
void LCD_EnterTextMode(void){

    uint8_t temp = RA8875_ReadReg(0x40);
    SET_BIT(temp, 1<<7);
    RA8875_WriteReg(0x40, temp);
}

/*  */
void LCD_ExitTextMode(void){

    uint8_t temp = RA8875_ReadReg(0x40);
    CLEAR_BIT(temp, 1<<7);
    RA8875_WriteReg(0x40, temp);
}



/* texto rasymo koordinates nustatymas */
void LCD_SetTextWriteCursorAbs(uint16_t x, uint16_t y)
{
    RA8875_WriteReg(0x2A, x);
    RA8875_WriteReg(0x2B, x>>8);
    RA8875_WriteReg(0x2C, y);
    RA8875_WriteReg(0x2D, y>>8);
}

/* texto rasymo koordinates nustatymas tekstiniam formate */
void LCD_PrintString(uint8_t col, uint8_t line, const char* str)
{
    uint16_t posx = col*8, posy = line*12; //<-- for external ROM font
    //uint16_t posx = col*8, posy = line*16; // <-- for internal CGROM font

    LCD_PutString(posx, posy, str);
}






/* pikselio rasymo i LCD RAM koordinates nustatymas  */
void LCD_SetWriteCursor(uint16_t x, uint16_t y)
{
    RA8875_WriteReg(0x47, x>>8);
    RA8875_WriteReg(0x46, x);
    RA8875_WriteReg(0x49, y>>8);
    RA8875_WriteReg(0x48, y);
}

/* pikselio skaitymo is LCD RAM koordinates nustatymas */
void LCD_SetReadCursor(uint16_t x, uint16_t y)
{
    RA8875_WriteReg(0x4B, x>>8);
    RA8875_WriteReg(0x4A, x);
    RA8875_WriteReg(0x4D, y>>8);
    RA8875_WriteReg(0x4C, y);
}






/* PWM */
void LCD_SetPwm1(int pwm_duty_cycle)
{
    uint32_t value;

    value  = (1 << 7); /* enable PWM. */
    value |= (0 << 6); /* ouput LOW when PWM STOP. */
    value |= (0 << 4); /* selet PWM1 function. */
    value |= 5;        /* 8: PWM clk = SYS_CLK/32. */

    RA8875_WriteReg(0x8A, value);

    value = (pwm_duty_cycle * 256) / 100;

    if(value > 0xFF) value = 0xFF;

    RA8875_WriteReg(0x8B, value);
}


void LCD_SetPwm2(int pwm_duty_cycle)
{
    uint32_t value;

    value  = (1 << 7); /* enable PWM. */
    value |= (0 << 6); /* ouput LOW when PWM STOP. */
    value |= (0 << 4); /* selet PWM1 function. */
    value |= 5;        /* 8: PWM clk = SYS_CLK/32. */

    RA8875_WriteReg(0x8C, value);

    value = (pwm_duty_cycle * 256) / 100;

    if(value > 0xFF) value = 0xFF;

    RA8875_WriteReg(0x8D, value);
}

/*  */
void LCD_SetPixel(const char* pixel, int x, int y)
{
    LCD_SetWriteCursor(x, y);

    RA8875_WriteReg(0x02, *(uint16_t *)pixel);
}

/*  */
void LCD_GetPixel(char* pixel, int x, int y)
{
    LCD_SetReadCursor(x, y);

    RA8875_CmdWrite(0x02);

    _set_gpio_od();

    *(uint16_t*)pixel = RA8875_DataRead();/* dummy read */
    *(uint16_t*)pixel = RA8875_DataRead();

    _set_gpio_pp();
}

/* NE1 (PD7) - open drive */
static void _set_gpio_od(void)
{
    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_7, LL_GPIO_PULL_UP);
}

/* NE1 (PD7) - push-pull */
static void _set_gpio_pp(void)
{
    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_7, LL_GPIO_PULL_NO);
}

/*  */
static void _lcd_reset(void)
{
    LL_GPIO_ResetOutputPin(LCD_RST_GPIO_Port, LCD_RST_Pin);
    LL_mDelay(10);
    LL_GPIO_SetOutputPin(LCD_RST_GPIO_Port, LCD_RST_Pin);

    LL_mDelay(50);
}



/**
  * @brief  Clears the hole LCD.
  * @param Color: the color of the background color code RGB(5-6-5).
  * @retval : None
  */
void LCD_Clear(__IO uint16_t color) {

    uint32_t index = 0;

    LCD_SetCursor(0x00, 0x00);

    RA8875_WriteRAM_Prepare(); /* Prepare to write GRAM */

    for(index = 0; index < (X_Size*Y_Size); index++) {
        RA8875_DataWrite(color);
    }
}


/**
  * @brief  Sets the Text color.
  * @param Color: specifies the Text color code RGB(5-6-5).
  *   and LCD_DrawPicture functions.
  * @retval : None
  */
void LCD_SetForeColor(__IO uint16_t color) {
    ForeColor = color;
}

/**
  * @brief  Sets the Background color.
  * @param Color: specifies the Background color code RGB(5-6-5).
  *   LCD_DrawChar and LCD_DrawPicture functions.
  * @retval : None
  */
void LCD_SetBackColor(__IO uint16_t color) {
    BackColor = color;
}


/*  */
void LCD_WriteForeColor(uint8_t r, uint8_t g, uint8_t b){

    RA8875_WriteReg(0x63, r);
    RA8875_WriteReg(0x64, g);
    RA8875_WriteReg(0x65, b);
}


/*  */
void LCD_WriteBackColor(uint8_t r, uint8_t g, uint8_t b){

    RA8875_WriteReg(0x60, r);
    RA8875_WriteReg(0x61, g);
    RA8875_WriteReg(0x62, b);
}





/**
  * @brief  Sets the cursor position.
  * @param xpos: specifies the X position.
  * @param ypos: specifies the Y position.
  * @retval : None
  */
void LCD_SetCursor(uint8_t xpos, uint16_t ypos) {
    RA8875_WriteReg(0x32, xpos);
    RA8875_WriteReg(0x33, ypos);
}


/**
  * @brief  Writes to the selected LCD register.
  * @param LCD_Reg: address of the selected register.
  * @arg LCD_RegValue: value to write to the selected register.
  * @retval : None
  */
static void RA8875_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue) {
    RA8875_CmdWrite(LCD_Reg);
    RA8875_DataWrite(LCD_RegValue);
}

/**
  * @brief  Reads the selected LCD Register.
  * @param  None
  * @retval : LCD Register Value.
  */
static uint16_t RA8875_ReadReg(uint8_t LCD_Reg) {
    RA8875_CmdWrite(LCD_Reg);
    return RA8875_DataRead();
}

/**
  * @brief  Reads the LCD RAM.
  * @param  None
  * @retval : LCD RAM Value.
  */
static uint16_t RA8875_ReadRAM(void) {
    RA8875_WriteRAM_Prepare(); /* Select GRAM Reg */
    return RA8875_DataRead();
}



/*  */
static uint8_t RA8875_ReadStatus(void){

    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);

    uint8_t status = RA8875_DataRead();

    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);

    return status;
}



void LCD_PutString(uint16_t posx, uint16_t posy, const char* str){

    LCD_SetTextWriteCursorAbs(posx, posy);

    LCD_EnterTextMode();

    RA8875_WriteRAM_Prepare();

    while(*str){
        RA8875_DataWrite(*str++);
        while((RA8875_ReadStatus()&0x80) == 0x80);
    }

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
    //uint8_t Xaddress = 0;

    //Xaddress = Xpos;

    LCD_SetWriteCursor(Xpos, Ypos);

    for(index = 0; index < 24; index++) {

        RA8875_WriteRAM_Prepare(); /* Prepare to write GRAM */

        for(i = 0; i < 16; i++) {
            if((c[index] & (1 << i)) == 0x00) {
                RA8875_DataWrite(BackColor);
            } else {
                RA8875_DataWrite(ForeColor);
            }
        }

        //Xaddress++;

        //LCD_SetWriteCursor(Xaddress, Ypos);

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
        RA8875_WriteReg(0x80, (Xpos - Height + 1));
    } else {
        RA8875_WriteReg(0x80, 0);
    }
    /* Horizontal GRAM End Address */
    RA8875_WriteReg(0x81, Xpos);
    /* Vertical GRAM Start Address */
    if(Ypos >= Width) {
        RA8875_WriteReg(0x82, (Ypos - Width + 1));
    } else {
        RA8875_WriteReg(0x82, 0);
    }
    /* Vertical GRAM End Address */
    RA8875_WriteReg(0x83, Ypos);

    LCD_SetCursor(Xpos, Ypos);
}

/**
  * @brief  Disables LCD Window mode.
  * @param  None
  * @retval : None
  */
void LCD_WindowModeDisable(void) {
    LCD_SetDisplayWindow(239, 0x13F, 240, 320);
    RA8875_WriteReg(0x03, 0x1018);
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

    if(Direction == Horizontal) {

        RA8875_WriteRAM_Prepare(); /* Prepare to write GRAM */

        for(i = 0; i < Length; i++) {
            RA8875_DataWrite(ForeColor);
        }
    } else {

        for(i = 0; i < Length; i++) {

            RA8875_WriteRAM_Prepare(); /* Prepare to write GRAM */

            RA8875_DataWrite(ForeColor);

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
        RA8875_WriteRAM_Prepare(); /* Prepare to write GRAM */
        RA8875_DataWrite(ForeColor);

        LCD_SetCursor(Xpos + CurX, Ypos - CurY);
        RA8875_WriteRAM_Prepare(); /* Prepare to write GRAM */
        RA8875_DataWrite(ForeColor);

        LCD_SetCursor(Xpos - CurX, Ypos + CurY);
        RA8875_WriteRAM_Prepare(); /* Prepare to write GRAM */
        RA8875_DataWrite(ForeColor);

        LCD_SetCursor(Xpos - CurX, Ypos - CurY);
        RA8875_WriteRAM_Prepare(); /* Prepare to write GRAM */
        RA8875_DataWrite(ForeColor);

        LCD_SetCursor(Xpos + CurY, Ypos + CurX);
        RA8875_WriteRAM_Prepare(); /* Prepare to write GRAM */
        RA8875_DataWrite(ForeColor);

        LCD_SetCursor(Xpos + CurY, Ypos - CurX);
        RA8875_WriteRAM_Prepare(); /* Prepare to write GRAM */
        RA8875_DataWrite(ForeColor);

        LCD_SetCursor(Xpos - CurY, Ypos + CurX);
        RA8875_WriteRAM_Prepare(); /* Prepare to write GRAM */
        RA8875_DataWrite(ForeColor);

        LCD_SetCursor(Xpos - CurY, Ypos - CurX);
        RA8875_WriteRAM_Prepare(); /* Prepare to write GRAM */
        RA8875_DataWrite(ForeColor);

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

    RA8875_WriteRAM_Prepare(); /* Prepare to write GRAM */

    for(index = 0; index < 2400; index++) {
        for(i = 0; i < 32; i++) {
            if((Pict[index] & (1 << i)) == 0x00) {
                RA8875_DataWrite(BackColor);
            } else {
                RA8875_DataWrite(ForeColor);
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
    RA8875_WriteReg(0x03, 0x1008);

    RA8875_WriteRAM_Prepare();

    for(index = 0; index < size; index++) {
        RA8875_DataWrite(*(__IO uint16_t *)BmpAddress);
        BmpAddress += 2;
    }

    /* Set GRAM write direction and BGR = 1 */
    /* I/D = 01 (Horizontal : increment, Vertical : decrement) */
    /* AM = 1 (address is updated in vertical writing direction) */
    RA8875_WriteReg(0x03, 0x1018);
}


/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval : None
  */
void LCD_PowerOn(void) {
    /* Power On sequence ---------------------------------------------------------*/
    RA8875_WriteReg(0x16, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    RA8875_WriteReg(0x17, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
    RA8875_WriteReg(0x18, 0x0000); /* VREG1OUT voltage */
    RA8875_WriteReg(0x19, 0x0000); /* VDV[4:0] for VCOM amplitude*/
    LL_mDelay(20);                 /* Dis-charge capacitor power voltage (200ms) */
    RA8875_WriteReg(0x16, 0x17B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    RA8875_WriteReg(0x17, 0x0137); /* DC1[2:0], DC0[2:0], VC[2:0] */
    LL_mDelay(5);                  /* Delay 50 ms */
    RA8875_WriteReg(0x18, 0x0139); /* VREG1OUT voltage */
    LL_mDelay(5);                  /* Delay 50 ms */
    RA8875_WriteReg(0x19, 0x1d00); /* VDV[4:0] for VCOM amplitude */
    RA8875_WriteReg(0x41, 0x0013); /* VCM[4:0] for VCOMH */
    LL_mDelay(5);                  /* Delay 50 ms */
    RA8875_WriteReg(0x07, 0x0173);  /* 262K color and display ON */
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
