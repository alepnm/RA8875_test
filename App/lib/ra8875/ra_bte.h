#ifndef RA_BTE_H_INCLUDED
#define RA_BTE_H_INCLUDED

#include "main.h"


/* BTE bool operations REG[51h] Bits[7:4] */
#define BTE_BOOL_BLACKNESS                        0x00
#define BTE_BOOL_DEST                             0x0A
#define BTE_BOOL_SOURCE                           0x0C
#define BTE_BOOL_WHITENESS                        0x0F


/* BTE Operation Functions REG[51h] Bits[3:0] */
#define BTE_ROP_WRITE                             0x00
#define BTE_ROP_READ                              0x01
#define BTE_ROP_MOVE_POSITIVE                     0x02
#define BTE_ROP_MOVE_NEGATIVE                     0x03
#define BTE_ROP_WRITE_TRANSPARENCY                0x04
#define BTE_ROP_MOVE_TRANSPARENCY                 0x05
#define BTE_ROP_FILL                              0x06
#define BTE_ROP_FILL_TRANSPARENCY                 0x07
#define BTE_ROP_COLOR_EXPANSION                   0x08
#define BTE_ROP_COLOR_EXPANSION_TRANSPARENCY      0x09
#define BTE_ROP_MOVE_COLOR_EXPANSION              0x0A
#define BTE_ROP_MOVE_COLOR_EXPANSION_TRANSPARENCY 0x0B
#define BTE_ROP_FILL_SOLID                        0x0C



/* BTE API */
void BTE_RasterOperation(uint16_t xsize, uint16_t ysize,
                              uint16_t source_xpos, uint16_t source_ypos, uint8_t source_layer,
                              uint16_t dest_xpos, uint16_t dest_ypos, uint8_t dest_layer,
                              uint8_t raster_operation,
                              uint8_t bool_func);
void BTE_SolidFill(uint16_t xsize, uint16_t ysize,
                   uint16_t dest_xpos, uint16_t dest_ypos, uint8_t dest_layer,
                   uint16_t color);





/* Helpers */

/*  */
void BTE_MoveBlockPositiveDir(uint16_t xsize, uint16_t ysize,
                              uint16_t source_xpos, uint16_t source_ypos, uint8_t source_layer,
                              uint16_t dest_xpos, uint16_t dest_ypos, uint8_t dest_layer);




#endif /* RA_BTE_H_INCLUDED */
