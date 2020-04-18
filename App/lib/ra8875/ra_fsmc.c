#include "main.h"
#include "ra_fsmc.h"








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
        LCD->LCD_RAM = *pdata;
        FSMC_WAIT_BUSY();
        pdata++;
    }
}
