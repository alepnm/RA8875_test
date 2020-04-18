#include "ra8875.h"


/* ok */
void TEXT_PutString(uint8_t col, uint8_t line, const char* str){

    uint16_t posx = col*Display.Font.Width, posy = line*Display.Font.Height;

    RA8875_SetTextWriteCursorAbs(posx, posy);

    RA8875_EnterTextMode();

    LCD->LCD_REG = 0x02;

    while(*str){

        LCD->LCD_RAM = *str++;
        FSMC_WAIT_BUSY();
    }

    RA8875_ExitTextMode();
}
