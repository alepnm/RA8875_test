#include "ra8875.h"


/* ok */
void TEXT_PutString(uint8_t col, uint8_t line, const char* str){

    uint16_t posx = col*Display.Font.Width, posy = line*Display.Font.Height;

    TEXT_PutStringAbs(posx, posy, str);
}


void TEXT_PutStringAbs(uint16_t posx, uint16_t posy, const char* str){

    RA8875_SetTextWriteCursorAbs(posx, posy);

    RA8875_SetTextMode();

    LCD->LCD_REG = 0x02;

    while(*str){

        LCD->LCD_RAM = *str;
        FSMC_WAIT_BUSY();
        str++;
    }

    RA8875_SetGraphicMode();
}
