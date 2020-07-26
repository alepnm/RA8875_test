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
#include "ra8875_registers.h"

#include "cmsis_os.h"
#include "timers.h"



/* Private typedef -----------------------------------------------------------*/


extern EventGroupHandle_t xEventGroupHandle;
extern uint16_t BackLightTimeoutCounter;

struct _lcd Display;

const struct _bte Background_Enot = {

    .Layer = 0,
    .XStart = 0,
    .YStart = 0,
    .XSize = X_SIZE,
    .YSize = Y_SIZE,
    .pData = NULL
};

const struct _bte Background_Krym = {

    .Layer = 1,
    .XStart = 0,
    .YStart = 0,
    .XSize = X_SIZE,
    .YSize = Y_SIZE,
    .pData = NULL
};



static void RA8875_HW_Test(void);


/*  */
void RA8875_Init(void) {

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
    LCD_DisplayON();
    FSMC_WAIT_BUSY();

    RA8875_ClearScreen();

    /* PLL clock frequency */
    FSMC_WriteRegister(RA8875_REG_PLLC1, RA8875_PLLC1_PLL_PREDIV2|0x15);   // PLL Control Register1
    LL_mDelay(1);
    FSMC_WriteRegister(RA8875_REG_PLLC2, RA8875_PLLC2_PLL_PLLDIV4);   // PLL Control Register2
    LL_mDelay(1);


    /* color deep/MCU Interface */
    FSMC_WriteRegister(RA8875_REG_SYSR, RA8875_SYSR_COLOR_DEPTH_16K|RA8875_SYSR_MCU_INTERFACE_16BIT);   // System Configuration Register

    /* pixel clock period */
    FSMC_WriteRegister(RA8875_REG_PCSR, 0x83);   // Pixel Clock Setting Register
    LL_mDelay(1);

    /* Serial Flash/ROM settings */
    FSMC_WriteRegister(RA8875_REG_SFCLR, 0x03);   // Serial Flash/ROM CLK Setting Register
    FSMC_WriteRegister(RA8875_REG_SROC, 0x00);   // Serial Flash/ROM Configuration Register

    /* Horisontal settings */
    FSMC_WriteRegister(RA8875_REG_HDWR, (uint8_t)(X_SIZE/8-1));   // LCD Horizontal Display Width Register
    FSMC_WriteRegister(RA8875_REG_HNDFTR, 0x02);   // Horizontal Non-Display Period Fine Tuning Option Register
    FSMC_WriteRegister(RA8875_REG_HNDR, 0x03);   // LCD Horizontal Non-Display Period Register
    FSMC_WriteRegister(RA8875_REG_HSTR, 0x01);   // HSYNC Start Position Register
    FSMC_WriteRegister(RA8875_REG_HPWR, RA8875_xPWR_SYNC_POLARITY_LOW|0x03);   // HSYNC Pulse Width Register

    /* Vertical settings */
    FSMC_WriteRegister(RA8875_REG_VDHR0, (uint8_t)(Y_SIZE-1));        // LCD Vertical Display Height Register0
    FSMC_WriteRegister(RA8875_REG_VDHR1, (uint8_t)((Y_SIZE-1)>>8));   // LCD Vertical Display Height Register1
    FSMC_WriteRegister(RA8875_REG_VNDR0, 0x0F);   // LCD Vertical Non-Display Period Register0
    FSMC_WriteRegister(RA8875_REG_VNDR1, 0x00);   // LCD Vertical Non-Display Period Register1

    FSMC_WriteRegister(RA8875_REG_VSTR0, 0x0E);   // VSYNC Start Position Register0
    FSMC_WriteRegister(RA8875_REG_VSTR1, 0x06);   // VSYNC Start Position Register1
    FSMC_WriteRegister(RA8875_REG_VPWR, RA8875_xPWR_SYNC_POLARITY_LOW|0x03);   // VSYNC Pulse Width Register

    FSMC_WriteRegister(RA8875_REG_DPCR, RA8875_DPCR_TWO_LAYER|
                                        RA8875_DPCR_HSCAN_DIR_LEFT|
                                        RA8875_DPCR_VSCAN_DIR_LEFT);   // Display Configuration Register

    RA8875_ExtROMFont();      // init external ROM
    RA8875_CGROMFont(0x00);   // init CGROM

    FSMC_WriteRegister(RA8875_REG_FLDR, 0x00);   // Font Line Distance Setting Register

    RA8875_SetActiveWindow(0, 0, X_SIZE-1, Y_SIZE-1);
    RA8875_ClearActiveWindow();

    RA8875_SetScrollWindow(0, 0, X_SIZE-1, Y_SIZE-1);


    FSMC_WriteRegister(RA8875_REG_MWCR0, RA8875_MWCR0_GRAPHIC_MODE|RA8875_MWCR0_CURSOR_NOT_VISIBLE|RA8875_MWCR0_CURSOR_BLINK_DISABLE|
                                         RA8875_MWCR0_MEM_WRITE_DIR_LR_TD|RA8875_MWCR0_MEM_WRITE_AUTOINC_ENABLE|RA8875_MWCR0_MEM_READ_AUTOINC_ENABLE);   // Memory Write Control Register 0

    FSMC_WriteRegister(RA8875_REG_MWCR1, RA8875_MWCR1_GRAPHIC_CURSOR_DISABLE|RA8875_MWCR1_GRAPHIC_CURSOR_SET1|RA8875_MWCR1_WR_DEST_SEL_LAYER1_2|
                                         RA8875_MWCR1_SELECT_LAYER1);   // Memory Write Control Register 1

    FSMC_WriteRegister(RA8875_REG_BTCR, 0x00);   //Blink Time Control Register

    FSMC_WriteRegister(RA8875_REG_MRCD, RA8875_MRCD_MEM_READ_DIR_LR_TD);   // Memory Read Cursor Direction

    FSMC_WriteRegister(RA8875_REG_BECR0, RA8875_BECR0_BTE_SOURCE_BLOCK_MODE|RA8875_BECR0_BTE_DEST_BLOCK_MODE);

    FSMC_WriteRegister(RA8875_REG_TPCR0, RA8875_TPCR0_TOUCH_ENABLE|RA8875_TPCR0_WAIT_4096_CLOCS|RA8875_TPCR0_WAKE_DISABLE|RA8875_TPCR0_SYSTEM_CLK_DIV1);
    FSMC_WriteRegister(RA8875_REG_TPCR1, RA8875_TPCR1_MANUAL_MODE|RA8875_TPCR1_VREF_INTERNAL|RA8875_TPCR1_DEBOUNCE_DISABLE|RA8875_TPCR1_MODE_WAIT_EVENT);

    FSMC_WriteRegister(RA8875_REG_P1CR, RA8875_PxCR_PWM_ENABLE|RA8875_PxCR_PWM_CLOCK_DIV8);    // PWM1 Control Register
    FSMC_WriteRegister(RA8875_REG_P1DCR, 0x80);

    FSMC_WriteRegister(RA8875_REG_P2CR, RA8875_PxCR_PWM_ENABLE|RA8875_PxCR_PWM_CLOCK_DIV8);    // PWM2 Control Register
    FSMC_WriteRegister(RA8875_REG_P2DCR, 0x80);


    FSMC_WriteRegister(RA8875_REG_KSCR1, RA8875_KSCR1_KEYSCAN_DISABLE|RA8875_KSCR1_LONGKEY_DISABLE|RA8875_KSCR1_KEYSCAN_SAMPLING_4|RA8875_KSCR1_KEYSCAN_FREQ_DIV16);
    FSMC_WriteRegister(RA8875_REG_KSCR2, RA8875_KSCR2_KEYSCAN_WAKE_DISABLE|RA8875_KSCR2_LONGKEY_TIMING_DIV2);

    FLASH_ExtFlashInit();

    RA8875_ExtROMFont();
    //RA8875_CGROMFont(RA8875_FNCR0_FONT_IEC8859_4);

    RA8875_ConfigIRQ();

    RA8875_HW_Test();

    if( TP_Init() ){
        TEXT_PutStringColored(0, 32, "Touchscreen initial OK...", COL_GREEN, COL_BLACK);
    }else{
        TEXT_PutStringColored(0, 32, "Touchscreen initial FAIL...", COL_RED, COL_BLACK);
    }

    Display.BackLightMax = BACKLIGHT_MAX_DEF;
    Display.BackLightMin = BACKLIGHT_MIN_DEF;
    BackLightTimeoutCounter = 10000;
    Display.Backlight = 100;

    LCD_SetBacklight(Display.Backlight);

    LL_mDelay(1000);

    RA8875_ClearScreen();
}


/* pasirenkam ROM chipa */
void RA8875_SelectRomChip(uint8_t chip){

    if(chip == RA8875_ROM_FONT_CHIP){
        FSMC_WriteRegister(RA8875_REG_SFCLR, RA8875_SFCLR_SFCL_FREQ_SYSCLK_DIV4);
        FSMC_WriteRegister(RA8875_REG_SROC, RA8875_SROC_EXTROM_SELECT_CS0|RA8875_SROC_ROM_ACCESS_MODE_FONT);
    }else{
        FSMC_WriteRegister(RA8875_REG_SFCLR, RA8875_SFCLR_SFCL_FREQ_SYSCLK_DIV1);
        FSMC_WriteRegister(RA8875_REG_SROC, RA8875_SROC_EXTROM_SELECT_CS1|RA8875_SROC_ROM_ACCESS_MODE_DMA|RA8875_SROC_ROM_DATA_LATCH_MODE_DUAL1);
    }
}



/* ok */
void RA8875_CGROMFont(uint8_t coding){

    FSMC_WriteRegister(RA8875_REG_FNCR0, RA8875_FNCR0_FONT_CGROM|RA8875_FNCR0_FONT_INTERNAL_CGROM|coding);
    FSMC_WriteRegister(RA8875_REG_FNCR1, RA8875_FNCR1_ALIGMENT_DISABLE|RA8875_FNCR1_BG_TRANSPARENCY_OFF|
                                         RA8875_FNCR1_FONT_ROTATION_DEG0|RA8875_FNCR1_FONT_HENLARGEMENT_X1|
                                         RA8875_FNCR1_FONT_VENLARGEMENT_X1);
    FSMC_WriteRegister(RA8875_REG_FTSR, RA8875_FWTSR_FONT_SIZE_16X16|0x00);
}

/* ok */
void RA8875_ExtROMFont(void){

    FSMC_WriteRegister(RA8875_REG_FNCR0, RA8875_FNCR0_FONT_EXTERNAL_CGROM);
    FSMC_WriteRegister(RA8875_REG_FNCR1, RA8875_FNCR1_ALIGMENT_DISABLE|RA8875_FNCR1_BG_TRANSPARENCY_OFF|
                                         RA8875_FNCR1_FONT_ROTATION_DEG0|RA8875_FNCR1_FONT_HENLARGEMENT_X1|
                                         RA8875_FNCR1_FONT_VENLARGEMENT_X1);

    FSMC_WriteRegister(RA8875_REG_FTSR, RA8875_FWTSR_FONT_SIZE_16X16|0x00);
    FSMC_WriteRegister(RA8875_REG_FROM, RA8875_SFROM_ROM_SELECT_TYPE1|RA8875_SFROM_FONT_CODING_ASCII|RA8875_SFROM_FONT_TYPE_BOLD);
}


/* ok */
void RA8875_SetTextMode(void){

    uint8_t tmp = FSMC_ReadRegister(RA8875_REG_MWCR0);

    SET_BIT(tmp, 0x80);
    FSMC_WriteRegister(RA8875_REG_MWCR0, tmp);
}

/* ok */
void RA8875_SetGraphicMode(void){

    uint8_t tmp = FSMC_ReadRegister(RA8875_REG_MWCR0);

    CLEAR_BIT(tmp, 0x80);
    FSMC_WriteRegister(RA8875_REG_MWCR0, tmp);
}

/* texto rasymo koordinates nustatymas */
void RA8875_SetTextWriteCursorAbs(uint16_t x, uint16_t y)
{
    FSMC_WriteRegister(RA8875_REG_FCURXL, x);
    FSMC_WriteRegister(RA8875_REG_FCURXH, x>>8);
    FSMC_WriteRegister(RA8875_REG_FCURYL, y);
    FSMC_WriteRegister(RA8875_REG_FCURYH, y>>8);
}

/* pikselio rasymo i LCD RAM koordinates nustatymas  */
void RA8875_SetPixelWriteCursor(uint16_t x, uint16_t y)
{
    FSMC_WriteRegister(RA8875_REG_CURH0, x);
    FSMC_WriteRegister(RA8875_REG_CURH1, x>>8);
    FSMC_WriteRegister(RA8875_REG_CURV0, y);
    FSMC_WriteRegister(RA8875_REG_CURV1, y>>8);
}

/* pikselio skaitymo is LCD RAM koordinates nustatymas */
void RA8875_SetPixelReadCursor(uint16_t x, uint16_t y)
{
    FSMC_WriteRegister(RA8875_REG_RCURH0, x);
    FSMC_WriteRegister(RA8875_REG_RCURH1, x>>8);
    FSMC_WriteRegister(RA8875_REG_RCURV0, y);
    FSMC_WriteRegister(RA8875_REG_RCURV1, y>>8);
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


/* Active window settings */
void RA8875_SetActiveWindow(uint16_t startx, uint16_t starty, uint16_t endx, uint16_t endy){

    FSMC_WriteRegister(RA8875_REG_HSAW0, (uint8_t)startx);
    FSMC_WriteRegister(RA8875_REG_HSAW1, (uint8_t)(startx>>8));
    FSMC_WriteRegister(RA8875_REG_VSAW0, (uint8_t)starty);
    FSMC_WriteRegister(RA8875_REG_VSAW1, (uint8_t)(starty>>8));

    FSMC_WriteRegister(RA8875_REG_HEAW0, (uint8_t)endx);
    FSMC_WriteRegister(RA8875_REG_HEAW1, (uint8_t)(endx>>8));
    FSMC_WriteRegister(RA8875_REG_VEAW0, (uint8_t)endy);
    FSMC_WriteRegister(RA8875_REG_VEAW1, (uint8_t)(endy>>8));
}


/* Scroll window settingas */
void RA8875_SetScrollWindow(uint16_t startx, uint16_t starty, uint16_t endx, uint16_t endy){

    FSMC_WriteRegister(RA8875_REG_HSSW0, (uint8_t)startx);
    FSMC_WriteRegister(RA8875_REG_HSSW1, (uint8_t)(startx>>8));
    FSMC_WriteRegister(RA8875_REG_VSSW0, (uint8_t)starty);
    FSMC_WriteRegister(RA8875_REG_VSSW1, (uint8_t)(starty>>8));

    FSMC_WriteRegister(RA8875_REG_HESW0, (uint8_t)endx);
    FSMC_WriteRegister(RA8875_REG_HESW1, (uint8_t)(endx>>8));
    FSMC_WriteRegister(RA8875_REG_VESW0, (uint8_t)endy);
    FSMC_WriteRegister(RA8875_REG_VESW1, (uint8_t)(endy>>8));
}

/* hardwarinis aktyvaus lango isvalymas */
void RA8875_ClearActiveWindow(void){

    FSMC_WriteRegister(RA8875_REG_MCLR, 0xC0);
    while( (FSMC_ReadRegister(RA8875_REG_MCLR)&0x80) == 0x80 );
}

/* hardwarinis displejaus isvalymas */
void RA8875_ClearScreen(void){

    FSMC_WriteRegister(RA8875_REG_MCLR, 0x80);
    while( (FSMC_ReadRegister(RA8875_REG_MCLR)&0x80) == 0x80 );
}


/*   */
void RA8875_SetPwm(uint8_t ch, int val)
{
    uint8_t reg = (ch == 0 || ch == 1) ? 0x8B : 0x8D;

    FSMC_WriteRegister(reg, val);
}


/* ok */
void RA8875_Reset(void)
{
    RA8875_RST_LOW();
    HAL_Delay(10);
    RA8875_RST_HIGH();

    HAL_Delay(50);
}


/*  */
void RA8875_ConfigIRQ(void){

    /*
    bit0 - Font write / BTE MCU R/W interrupt
    bit1 - BTE Process Complete Interrupt
    bit2 - Touch Panel Interrupt (jungiam Touch modulyje)
    bit3 - DMA Interrupt
    bit4 - KEYSCAN Interrupt
    */

    FSMC_WriteRegister(RA8875_REG_INTC2, 0xFF);   // isvalom IRQ flagus

    uint8_t reg = FSMC_ReadRegister(RA8875_REG_INTC1);
    //FSMC_WriteRegister(RA8875_REG_INTC1, reg|RA8875_INTC1_TP_IRQ_ENABLE);   // ijungiam/isjungiam Touch IRQ
}


/*  */
static void RA8875_HW_Test(void){

    uint8_t tmp1 = 0, tmp2 = 0;
    uint8_t RegisterTestStatus = 0, BusTestStatus = 0;
    uint16_t pixel = 0;
    uint32_t i = 0;

    __disable_irq();

    /* register rw test */
    FSMC_WriteRegister(RA8875_REG_CGSR, 0x55);
    FSMC_WAIT_BUSY();

    tmp1 = FSMC_ReadRegister(RA8875_REG_CGSR);

    FSMC_WriteRegister(RA8875_REG_CGSR, 0xAA);
    FSMC_WAIT_BUSY();

    tmp2 = FSMC_ReadRegister(RA8875_REG_CGSR);

    if ((tmp1 == 0x55) && (tmp2 == 0xAA)) RegisterTestStatus = 1;

    LL_mDelay(50); /* delay 50 ms */

    /* data bus test. */
    /* irasom i GRAM testinius duomenys */
    FSMC_WriteRegister(RA8875_REG_MWCR0, RA8875_MWCR0_MEM_WRITE_DIR_LR_TD);

    RA8875_SetPixelWriteCursor(0, 0);

    LCD->LCD_REG = 0x02;
    FSMC_WAIT_BUSY();

    for(i=0; i < DISPLAY_PIXELS; i++)
    {
        LCD->LCD_RAM = i;
        FSMC_WAIT_BUSY();
    }


    /* tikrinam irasytus testines duomenys */
    FSMC_WriteRegister(RA8875_REG_MRCD, RA8875_MRCD_MEM_READ_DIR_LR_TD);

    RA8875_SetPixelReadCursor(0, 0);

    LCD->LCD_REG = 0x02;
    FSMC_WAIT_BUSY();

    (void)LCD->LCD_RAM;// dummy read
    FSMC_WAIT_BUSY();

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

    __enable_irq();

    RA8875_ClearScreen();

    if(BusTestStatus){
        TEXT_PutStringColored(0, 0, "BUS Test OK...", COL_GREEN, COL_BLACK);
    }else{
        TEXT_PutStringColored(0, 0, "BUS Test FAIL...", COL_RED, COL_BLACK);
    }

    if(RegisterTestStatus){
        TEXT_PutStringColored(0, 16, "Register Test OK...", COL_GREEN, COL_BLACK);
    }else{
        TEXT_PutStringColored(0, 16, "Register Test FAIL...", COL_RED, COL_BLACK);
    }
}




/* IRQ handler */
void RA8875_IRQ_Handler(void){

//    BaseType_t xHigherPriorityTaskWoken, xResult;

    uint8_t reg = FSMC_ReadRegister(RA8875_REG_INTC2);

    /* check for Font write / BTE MCU R/W interrupt */
    if((reg&0x01) == 0x01){

        if( (FSMC_ReadRegister(RA8875_REG_BECR0)&0x80) == 0x80){
        /* BTE MCU R/W interrupt */

        }else{
        /* Font write interrupt */

        }

        reg |= 0x01;   // clear font write / BTE MCU R/W interrupt
    }

    /* check for BTE process complete interrupt */
    if((reg&RA8875_INTC2_BTE_COMPLETE_IF) == RA8875_INTC2_BTE_COMPLETE_IF){

        reg |= RA8875_INTC2_BTE_COMPLETE_IF;   // clear BTE process complete interrupt
    }

    /* check for touch interrupt */
    if((reg&RA8875_INTC2_TP_IF) == RA8875_INTC2_TP_IF){

        reg |= RA8875_INTC2_TP_IF;   // numetam touch irq flaga

        //xResult = xEventGroupSetBitsFromISR( xEventGroupHandle, TOUCH_EVENT_FLAG, &xHigherPriorityTaskWoken );
        //if( xResult != pdFAIL ) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

    /* check for DMA interrupt */
    if((reg&RA8875_INTC2_DMA_IF) == RA8875_INTC2_DMA_IF){

        reg |= RA8875_INTC2_DMA_IF;   // clear the DMA interrupt
    }

    /* check for keyscan interrupt */
    if((reg&RA8875_INTC2_KEYSCAN_IF) == RA8875_INTC2_KEYSCAN_IF){

        reg |= RA8875_INTC2_KEYSCAN_IF;   // clear the keyscan interrupt
    }

    FSMC_WriteRegister(RA8875_REG_INTC2, reg);
}



/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
