#include "cmsis_os.h"
#include "ra8875.h"

#define FLASH_DIRECT_ACCESS_MODE_DISABLE   0x00
#define FLASH_DIRECT_ACCESS_MODE_ENABLE    0x01



static void FLASH_DMA_Transfer(uint32_t addr, uint32_t len);



/*  */
void FLASH_ExtFlashInit(void){

    FSMC_WriteRegister(RA8875_REG_SACS_MODE, FLASH_DIRECT_ACCESS_MODE_DISABLE);


}


/*  */
uint8_t FLASH_ReadByte(uint32_t addr){

    FSMC_WriteRegister(0x05, 0b10000011);
    FSMC_WriteRegister(0x06, 0b00000011);

    FSMC_WriteRegister(0xE0, 0b00000001);   //direct access mode enabled

    FSMC_WriteRegister(RA8875_REG_SACS_ADDR, (addr>>16)&0xFF);
    FSMC_WriteRegister(RA8875_REG_SACS_ADDR, (addr>>8)&0xFF);
    FSMC_WriteRegister(RA8875_REG_SACS_ADDR, addr&0xFF);

    FSMC_WaitROM();

    uint8_t result = FSMC_ReadRegister(RA8875_REG_SACS_DATA);

    FSMC_WriteRegister(0xE0, 0b00000000);   //direct access mode disabled

    return result;
}


/*  */
void FLASH_ReadContinuous(uint32_t start, uint32_t* dest, uint32_t len){

}


/*  */
void FLASH_ReadBlock(uint32_t start, uint32_t len, uint16_t startx, uint16_t starty, uint16_t endx, uint16_t endy){



}


/*  */
void FLASH_GetScreenByAddress(uint32_t addr){

    RA8875_SetPixelWriteCursor(0, 0);

    FLASH_DMA_Transfer(addr, DISPLAY_PIXELS);
}


/*  */
static void FLASH_DMA_Transfer(uint32_t addr, uint32_t len){

    /* perjungiam ROM parametrus darbui su SPI Flash */
    RA8875_SelectRomChip(RA8875_ROM_FLASH_CHIP);

    FSMC_WriteRegister(RA8875_REG_SSAR0, (uint8_t)addr);
    FSMC_WriteRegister(RA8875_REG_SSAR1, (uint8_t)(addr>>8));
    FSMC_WriteRegister(RA8875_REG_SSAR2, (uint8_t)(addr>>16));

    FSMC_WriteRegister(RA8875_REG_DTNR0, (uint8_t)len);
    FSMC_WriteRegister(RA8875_REG_DTNR1, (uint8_t)(len>>8));
    FSMC_WriteRegister(RA8875_REG_DTNR2, (uint8_t)(len>>16));

    FSMC_WriteRegister(RA8875_REG_DMACR, 0x01);
    while((FSMC_ReadRegister(RA8875_REG_DMACR)&0x01) == 0x01);

    /* perjungiam ROM parametrus darbui su Font chip */
    RA8875_SelectRomChip(RA8875_ROM_FONT_CHIP);
}


