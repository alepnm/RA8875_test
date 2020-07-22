#include "ra8875.h"



/* ok */
void GEO_DrawLine(uint16_t Xpos_start, uint16_t Ypos_start, uint16_t Xpos_end, uint16_t Ypos_end) {

    FSMC_WriteRegister(RA8875_REG_DLHSR0, Xpos_start&0xFF);
    FSMC_WriteRegister(RA8875_REG_DLHSR1, Xpos_start>>0x08);

    FSMC_WriteRegister(RA8875_REG_DLVSR0, Ypos_start&0xFF);
    FSMC_WriteRegister(RA8875_REG_DLVSR1, Ypos_start>>0x08);

    FSMC_WriteRegister(RA8875_REG_DLHER0, Xpos_end&0xFF);
    FSMC_WriteRegister(RA8875_REG_DLHER1, Xpos_end>>0x08);

    FSMC_WriteRegister(RA8875_REG_DLVER0, Ypos_end&0xFF);
    FSMC_WriteRegister(RA8875_REG_DLVER1, Ypos_end>>0x08);

    FSMC_WriteRegister(RA8875_REG_DCR, 0x80);

    while((FSMC_ReadRegister(RA8875_REG_DCR)&0x80) == 0x80) FSMC_WAIT_BUSY();
}


/* ok */
void GEO_DrawRect(uint16_t Xpos_start, uint16_t Ypos_start, uint16_t Xpos_end, uint16_t Ypos_end, uint8_t fill) {

    if(fill) fill = 0x20;

    FSMC_WriteRegister(RA8875_REG_DLHSR0, Xpos_start&0xFF);
    FSMC_WriteRegister(RA8875_REG_DLHSR1, Xpos_start>>0x08);

    FSMC_WriteRegister(RA8875_REG_DLVSR0, Ypos_start&0xFF);
    FSMC_WriteRegister(RA8875_REG_DLVSR1, Ypos_start>>0x08);

    FSMC_WriteRegister(RA8875_REG_DLHER0, Xpos_end&0xFF);
    FSMC_WriteRegister(RA8875_REG_DLHER1, Xpos_end>>0x08);

    FSMC_WriteRegister(RA8875_REG_DLVER0, Ypos_end&0xFF);
    FSMC_WriteRegister(RA8875_REG_DLVER1, Ypos_end>>0x08);

    FSMC_WriteRegister(RA8875_REG_DCR, (0x90|fill));

    while((FSMC_ReadRegister(RA8875_REG_DCR)&0x80) == 0x80) FSMC_WAIT_BUSY();
}


/* ok */
void GEO_DrawTriangle(uint16_t xa, uint16_t ya, uint16_t xb, uint16_t yb, uint16_t xc, uint16_t yc, uint8_t fill) {

    if(fill) fill = 0x20;

    FSMC_WriteRegister(RA8875_REG_DLHSR0, xa&0xFF);
    FSMC_WriteRegister(RA8875_REG_DLHSR1, xa>>0x08);

    FSMC_WriteRegister(RA8875_REG_DLVSR0, ya&0xFF);
    FSMC_WriteRegister(RA8875_REG_DLVSR1, ya>>0x08);

    FSMC_WriteRegister(RA8875_REG_DLHER0, xb&0xFF);
    FSMC_WriteRegister(RA8875_REG_DLHER1, xb>>0x08);

    FSMC_WriteRegister(RA8875_REG_DLVER0, yb&0xFF);
    FSMC_WriteRegister(RA8875_REG_DLVER1, yb>>0x08);

    FSMC_WriteRegister(RA8875_REG_DTPH0, xc&0xFF);
    FSMC_WriteRegister(RA8875_REG_DTPH1, xc>>0x08);

    FSMC_WriteRegister(RA8875_REG_DTPV0, yc&0xFF);
    FSMC_WriteRegister(RA8875_REG_DTPV1, yc>>0x08);

    FSMC_WriteRegister(RA8875_REG_DCR, (0x81|fill));

    while((FSMC_ReadRegister(RA8875_REG_DCR)&0x80) == 0x80) FSMC_WAIT_BUSY();
}


/* ok */
void GEO_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t radius, uint8_t fill) {

    if(fill) fill = 0x20;

    /* horizontal position */
    FSMC_WriteRegister(RA8875_REG_DCHR0, Xpos&0xFF);
    FSMC_WriteRegister(RA8875_REG_DCHR1, Xpos>>0x08);

    /* vertical position */
    FSMC_WriteRegister(RA8875_REG_DCVR0, Ypos&0xFF);
    FSMC_WriteRegister(RA8875_REG_DCVR1, Ypos>>0x08);

    /* circle radius */
    FSMC_WriteRegister(RA8875_REG_DCRR, radius);

    /* start draw */
    FSMC_WriteRegister(RA8875_REG_DCR, (0x40|fill));

    while((FSMC_ReadRegister(RA8875_REG_DCR)&0x40) == 0x40) FSMC_WAIT_BUSY();
}


/* ok */
void GEO_DrawSquareOfCircleCorner(uint16_t Xpos_start, uint16_t Ypos_start, uint16_t Xpos_end, uint16_t Ypos_end, uint16_t axish, uint16_t axisy, uint8_t fill) {

    if(fill) fill = 0x40;

    FSMC_WriteRegister(RA8875_REG_DLHSR0, Xpos_start&0xFF);
    FSMC_WriteRegister(RA8875_REG_DLHSR1, Xpos_start>>0x08);

    FSMC_WriteRegister(RA8875_REG_DLVSR0, Ypos_start&0xFF);
    FSMC_WriteRegister(RA8875_REG_DLVSR1, Ypos_start>>0x08);

    FSMC_WriteRegister(RA8875_REG_DLHER0, Xpos_end&0xFF);
    FSMC_WriteRegister(RA8875_REG_DLHER1, Xpos_end>>0x08);

    FSMC_WriteRegister(RA8875_REG_DLVER0, Ypos_end&0xFF);
    FSMC_WriteRegister(RA8875_REG_DLVER1, Ypos_end>>0x08);

    FSMC_WriteRegister(RA8875_REG_ELLA0, axish&0xFF);
    FSMC_WriteRegister(RA8875_REG_ELLA1, axish>>0x08);

    FSMC_WriteRegister(RA8875_REG_ELLB0, axisy&0xFF);
    FSMC_WriteRegister(RA8875_REG_ELLB1, axisy>>0x08);

    FSMC_WriteRegister(RA8875_REG_DCR1, (0xA0|fill));

    while((FSMC_ReadRegister(RA8875_REG_DCR1)&0x80) == 0x80) FSMC_WAIT_BUSY();
}
