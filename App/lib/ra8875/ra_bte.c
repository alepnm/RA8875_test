#include "ra_bte.h"
#include "ra8875.h"


/*  */
static void BTE_SetBlockSize(uint16_t xsize, uint16_t ysize)
{
    /* BTE size */
    FSMC_WriteRegister(0x5C, (uint8_t)(xsize));
    FSMC_WriteRegister(0x5D, (uint8_t)((xsize>>8)&0x03));
    FSMC_WriteRegister(0x5E, (uint8_t)(ysize));
    FSMC_WriteRegister(0x5F, (uint8_t)((ysize>>8)&0x03));
}



/*  */
static void BTE_SetSourceBlockPos(uint16_t xpos, uint16_t ypos, uint8_t layer)
{
    layer = (layer) ? 0x80 : 0x00;

    FSMC_WriteRegister(0x54, (uint8_t)xpos);
    FSMC_WriteRegister(0x55, (uint8_t)((xpos>>8)&0x03));
    FSMC_WriteRegister(0x56, (uint8_t)ypos);
    FSMC_WriteRegister(0x57, (uint8_t)((ypos>>8)&0x03) | layer);
}



/*  */
static void BTE_SetDestinationBlockPos(uint16_t xpos, uint16_t ypos, uint8_t layer)
{
    layer = (layer) ? 0x80 : 0x00;

    FSMC_WriteRegister(0x58, (uint8_t)xpos);
    FSMC_WriteRegister(0x59, (uint8_t)((xpos>>8)&0x03));
    FSMC_WriteRegister(0x5A, (uint8_t)ypos);
    FSMC_WriteRegister(0x5B, (uint8_t)((ypos>>8)&0x03) | layer);
}



/* ok
GRAM bloko kopijavimas is vienos vietos i kita

*/
void BTE_RasterOperation(uint16_t xsize, uint16_t ysize,
                         uint16_t source_xpos, uint16_t source_ypos, uint8_t source_layer,
                         uint16_t dest_xpos, uint16_t dest_ypos, uint8_t dest_layer,
                         uint8_t raster_operation,
                         uint8_t bool_func)
{

    BTE_SetBlockSize(xsize, ysize);
    BTE_SetSourceBlockPos(source_xpos, source_ypos, source_layer);
    BTE_SetDestinationBlockPos(dest_xpos, dest_ypos, dest_layer);

    FSMC_WriteRegister(0x51, ((bool_func<<4) | raster_operation));
    FSMC_WriteRegister(0x50, 0x80); // start BTE

    FSMC_WaitBTE();
}


/* ok */
void BTE_SolidFill(uint16_t xsize, uint16_t ysize,
                   uint16_t dest_xpos, uint16_t dest_ypos, uint8_t dest_layer,
                   uint16_t color)
{
    LCD_SetForeColor(color);

    BTE_SetBlockSize(xsize, ysize);
    BTE_SetDestinationBlockPos(dest_xpos, dest_ypos, dest_layer);

    FSMC_WriteRegister(0x51, BTE_ROP_FILL_SOLID);
    FSMC_WriteRegister(0x50, 0x80); // start BTE

    FSMC_WaitBTE();

    LCD_SetForeColor(Display.FontColor);
}


/*  */
void BTE_MoveBlockPositiveDir(uint16_t xsize, uint16_t ysize,
                              uint16_t source_xpos, uint16_t source_ypos, uint8_t source_layer,
                              uint16_t dest_xpos, uint16_t dest_ypos, uint8_t dest_layer)
{
    BTE_RasterOperation(xsize, ysize, source_xpos, source_ypos, source_layer,
                        dest_xpos, dest_ypos, dest_layer, BTE_ROP_MOVE_POSITIVE, BTE_BOOL_SOURCE);
}















