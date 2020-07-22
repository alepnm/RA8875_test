#include "ra8875.h"


/* LCD ijungimas/isjungimas */
void LCD_Display_OnOff(uint8_t state){

    uint8_t tmp = FSMC_ReadRegister(RA8875_REG_PWRR);

    FSMC_WriteRegister(RA8875_REG_PWRR, (tmp | state<<7));
}


/* foninio paveikslelio irasymas i LCD */
void LCD_SetBackground(uint16_t* pData, uint32_t len, uint8_t layer){

    LCD_SelectLayer(layer);

    FSMC_WriteDDRAM(pData, len);
}


/* LCD sviesumo nustatymas 0-100% */
void LCD_SetBacklight(uint8_t bl){

    if(bl > 100) bl = 100;

    RA8875_SetPwm(0, bl*2.55);
}


/* ok */
void LCD_SelectLayer(uint8_t layer){

    uint8_t tmp = FSMC_ReadRegister(RA8875_REG_MWCR1);
    layer = (layer) ? 1 : 0;
    FSMC_WriteRegister(RA8875_REG_MWCR1, ((tmp&0xFE) | layer));
}


/* ok */
void LCD_ShowLayer(uint8_t layer){

    uint8_t tmp = FSMC_ReadRegister(RA8875_REG_LTPR0);
    layer = (layer) ? 1 : 0;
    FSMC_WriteRegister(RA8875_REG_LTPR0, ((tmp&0xFE) | layer));
}


/* ok */
void LCD_Clear(void){

    FSMC_WriteRegister(RA8875_REG_MCLR, 0x80);
    FSMC_WAIT_BUSY();
}



/* ok */
void LCD_ClearColor(uint16_t color) {

    uint32_t index = 0;

    RA8875_SetCursor(0x00, 0x00);

    LCD->LCD_REG = 0x02;

    for(index = 0; index < DISPLAY_PIXELS; index++) {
        LCD->LCD_RAM = color;
        FSMC_WAIT_BUSY();
    }

//  LCD_SetBackColor(color);
//
//  /* Destination */
//  FSMC_WriteRegister(RA8875_REG_HDBE0, 0x00);
//  FSMC_WriteRegister(RA8875_REG_HDBE1, 0x00);
//  FSMC_WriteRegister(RA8875_REG_VDBE0, 0x00);
//  FSMC_WriteRegister(RA8875_REG_VDBE1, 0x00);
//
//  /* BTE size */
//  FSMC_WriteRegister(RA8875_REG_BEWR0, (uint8_t)(X_SIZE));
//  FSMC_WriteRegister(RA8875_REG_BEWR1, (uint8_t)(X_SIZE>>8)&0x03);
//  FSMC_WriteRegister(RA8875_REG_BEHR0, (uint8_t)(Y_SIZE));
//  FSMC_WriteRegister(RA8875_REG_BEHR1, (uint8_t)(Y_SIZE>>8)&0x03);
//
//  FSMC_WriteRegister(RA8875_REG_BECR1, 0xAC);
//  FSMC_WriteRegister(RA8875_REG_BECR0, 0xE0); // start BTE
//
//  LCD->LCD_REG = 0x02;
//  FSMC_WaitMem();
//
//  FSMC_WaitBTE();
}


/* ok */
void LCD_SetForeColor(uint16_t color){

    FSMC_WriteRegister(RA8875_REG_FGCR0, (uint8_t)(color>>11));
    FSMC_WriteRegister(RA8875_REG_FGCR1, (uint8_t)((color>>5)&0x3F));
    FSMC_WriteRegister(RA8875_REG_FGCR2, (uint8_t)(color&0x1F));
}


/*  */
uint16_t LCD_GetForeColor(void){

    return ( ((FSMC_ReadRegister(0x63)&0x1F)<<11) | ((FSMC_ReadRegister(0x64)&0x3F)<<5) | (FSMC_ReadRegister(0x65)&0x1F) );
}



/* ok */
void LCD_SetBackColor(uint16_t color){

    FSMC_WriteRegister(RA8875_REG_BGCR0, (uint8_t)(color>>11));
    FSMC_WriteRegister(RA8875_REG_BGCR1, (uint8_t)((color>>5)&0x3F));
    FSMC_WriteRegister(RA8875_REG_BGCR2, (uint8_t)(color&0x1F));
}


/*  */
uint16_t LCD_GetBackColor(void){

    return ( ((FSMC_ReadRegister(RA8875_REG_BGCR0)&0x1F)<<11) | ((FSMC_ReadRegister(RA8875_REG_BGCR1)&0x3F)<<5) | (FSMC_ReadRegister(RA8875_REG_BGCR2)&0x1F) );
}

