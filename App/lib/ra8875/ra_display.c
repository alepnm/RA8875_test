#include "ra8875.h"

/* LCD ijungimas */
void LCD_DisplayON(void){

    uint8_t tmp = FSMC_ReadRegister(RA8875_REG_PWRR);

    SET_BIT(tmp, 0x80);
    FSMC_WriteRegister(RA8875_REG_PWRR, (tmp));
}

/* LCD isjungimas */
void LCD_DisplayOFF(void){

    uint8_t tmp = FSMC_ReadRegister(RA8875_REG_PWRR);

    CLEAR_BIT(tmp, 0x80);
    FSMC_WriteRegister(RA8875_REG_PWRR, (tmp));
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

