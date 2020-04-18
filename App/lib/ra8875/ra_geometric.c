#include "ra8875.h"



/* ok */
void GEO_DrawLine(uint16_t Xpos_start, uint16_t Ypos_start, uint16_t Xpos_end, uint16_t Ypos_end) {

    FSMC_WriteRegister(0x91, Xpos_start&0xFF);
    FSMC_WriteRegister(0x92, Xpos_start>>0x08);

    FSMC_WriteRegister(0x93, Ypos_start&0xFF);
    FSMC_WriteRegister(0x94, Ypos_start>>0x08);

    FSMC_WriteRegister(0x95, Xpos_end&0xFF);
    FSMC_WriteRegister(0x96, Xpos_end>>0x08);

    FSMC_WriteRegister(0x97, Ypos_end&0xFF);
    FSMC_WriteRegister(0x98, Ypos_end>>0x08);

    FSMC_WriteRegister(0x90, 0x80);

    while((FSMC_ReadRegister(0x90)&0x80) == 0x80) FSMC_WAIT_BUSY();
}


/* ok */
void GEO_DrawRect(uint16_t Xpos_start, uint16_t Ypos_start, uint16_t Xpos_end, uint16_t Ypos_end, uint8_t fill) {

    if(fill) fill = 0x20;

    FSMC_WriteRegister(0x91, Xpos_start&0xFF);
    FSMC_WriteRegister(0x92, Xpos_start>>0x08);

    FSMC_WriteRegister(0x93, Ypos_start&0xFF);
    FSMC_WriteRegister(0x94, Ypos_start>>0x08);

    FSMC_WriteRegister(0x95, Xpos_end&0xFF);
    FSMC_WriteRegister(0x96, Xpos_end>>0x08);

    FSMC_WriteRegister(0x97, Ypos_end&0xFF);
    FSMC_WriteRegister(0x98, Ypos_end>>0x08);

    FSMC_WriteRegister(0x90, (0x90|fill));

    while((FSMC_ReadRegister(0x90)&0x80) == 0x80) FSMC_WAIT_BUSY();
}


/* ok */
void GEO_DrawTriangle(uint16_t xa, uint16_t ya, uint16_t xb, uint16_t yb, uint16_t xc, uint16_t yc, uint8_t fill) {

    if(fill) fill = 0x20;

    FSMC_WriteRegister(0x91, xa&0xFF);
    FSMC_WriteRegister(0x92, xa>>0x08);

    FSMC_WriteRegister(0x93, ya&0xFF);
    FSMC_WriteRegister(0x94, ya>>0x08);

    FSMC_WriteRegister(0x95, xb&0xFF);
    FSMC_WriteRegister(0x96, xb>>0x08);

    FSMC_WriteRegister(0x97, yb&0xFF);
    FSMC_WriteRegister(0x98, yb>>0x08);

    FSMC_WriteRegister(0xA9, xc&0xFF);
    FSMC_WriteRegister(0xAA, xc>>0x08);

    FSMC_WriteRegister(0xAB, yc&0xFF);
    FSMC_WriteRegister(0xAC, yc>>0x08);

    FSMC_WriteRegister(0x90, (0x81|fill));

    while((FSMC_ReadRegister(0x90)&0x80) == 0x80) FSMC_WAIT_BUSY();
}


/* ok */
void GEO_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t radius, uint8_t fill) {

    if(fill) fill = 0x20;

    /* horizontal position */
    FSMC_WriteRegister(0x99, Xpos&0xFF);
    FSMC_WriteRegister(0x9A, Xpos>>0x08);

    /* vertical position */
    FSMC_WriteRegister(0x9B, Ypos&0xFF);
    FSMC_WriteRegister(0x9C, Ypos>>0x08);

    /* circle radius */
    FSMC_WriteRegister(0x9D, radius);

    /* start draw */
    FSMC_WriteRegister(0x90, (0x40|fill));

    while((FSMC_ReadRegister(0x90)&0x40) == 0x40) FSMC_WAIT_BUSY();
}


/* ok */
void GEO_DrawSquareOfCircleCorner(uint16_t Xpos_start, uint16_t Ypos_start, uint16_t Xpos_end, uint16_t Ypos_end, uint16_t axish, uint16_t axisy, uint8_t fill) {

    if(fill) fill = 0x40;

    FSMC_WriteRegister(0x91, Xpos_start&0xFF);
    FSMC_WriteRegister(0x92, Xpos_start>>0x08);

    FSMC_WriteRegister(0x93, Ypos_start&0xFF);
    FSMC_WriteRegister(0x94, Ypos_start>>0x08);

    FSMC_WriteRegister(0x95, Xpos_end&0xFF);
    FSMC_WriteRegister(0x96, Xpos_end>>0x08);

    FSMC_WriteRegister(0x97, Ypos_end&0xFF);
    FSMC_WriteRegister(0x98, Ypos_end>>0x08);

    FSMC_WriteRegister(0xA1, axish&0xFF);
    FSMC_WriteRegister(0xA2, axish>>0x08);

    FSMC_WriteRegister(0xA3, axisy&0xFF);
    FSMC_WriteRegister(0xA4, axisy>>0x08);

    FSMC_WriteRegister(0xA0, (0xA0|fill));

    while((FSMC_ReadRegister(0xA0)&0x80) == 0x80) FSMC_WAIT_BUSY();
}
