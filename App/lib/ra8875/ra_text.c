#include "ra8875.h"



void TEXT_PutStringColored(uint16_t posx, uint16_t posy, const char* str, uint16_t color, uint16_t bgcolor){

    //RA8875_SetTextWriteCursorAbs(posx, posy);

    //RA8875_SetTextMode();

    LCD_SetForeColor(color);
    LCD_SetBackColor(bgcolor);

//    LCD->LCD_REG = 0x02;
//
//    while(*str){
//
//        LCD->LCD_RAM = *str;
//        FSMC_WAIT_BUSY();
//        str++;
//    }



    TEXT_PutString(posx, posy, str);

    LCD_SetForeColor(Display.FontColor);
    LCD_SetBackColor(Display.BackColor);

    //RA8875_SetGraphicMode();
}


void TEXT_PutString(uint16_t posx, uint16_t posy, const char* str){

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
