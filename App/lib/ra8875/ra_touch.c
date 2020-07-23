#include "cmsis_os.h"
#include "ra8875.h"


#define TS_WITH_STATUS    0
#define TS_WITH_IF_FLAG   1
#define TS_TOUCH_DETECT   TS_WITH_IF_FLAG


/*  */
uint8_t TS_Init(void){

    TS_Data.IsTouched = 0;
    TS_Data.XPos = 0;
    TS_Data.YPos = 0;

    if((FSMC_ReadRegister(RA8875_REG_TPCR0)&RA8875_TPCR0_TOUCH_ENABLE) == RA8875_TPCR0_TOUCH_ENABLE){
        TS_Data.IsEnabled = 1;
        return 1;
    }
    else{
        TS_Data.IsEnabled = 0;
        return 0;
    }
}


/* skaitom XY reiksmes (Manual Mode)*/
void TS_ReadXY_Manual(void){

    FSMC_WriteRegister(RA8875_REG_TPCR1, 0x42);
    osDelay(4);

    FSMC_WriteRegister(RA8875_REG_TPCR1, 0x43);
    osDelay(4);

    FSMC_WriteRegister(RA8875_REG_TPCR1, 0x40);

    TS_ReadXY();
}


/* konvertuojame XY (Auto Mode) */
void TS_ReadXY(void){

    static uint8_t n = 0;
    static uint16_t sumx = 0, sumy = 0, lastx = 0, lasty = 0;

    TS_Data.XAdc = ((FSMC_ReadRegister(RA8875_REG_TPXH)<<2)|(FSMC_ReadRegister(RA8875_REG_TPXYL)&0x03));
    TS_Data.YAdc = ((FSMC_ReadRegister(RA8875_REG_TPYH)<<2)|((FSMC_ReadRegister(RA8875_REG_TPXYL)>>2)&0x03));

    sumx = sumx + (lastx + TS_Data.XAdc)/2;
    sumy = sumy + (lasty + TS_Data.YAdc)/2;

    lastx = TS_Data.XAdc;
    lasty = TS_Data.YAdc;

    if(n++ == 4){

        n = 0;

        TS_Data.XPos = (float)(sumx>>2)/2.3 - 40;
        TS_Data.YPos = (float)(sumy>>2)/3.6 - 40;

        if(TS_Data.XPos < 0) TS_Data.XPos = 0;
        if(TS_Data.YPos < 0) TS_Data.YPos = 0;

        if(TS_Data.XPos > X_SIZE) TS_Data.XPos = X_SIZE;
        if(TS_Data.YPos > Y_SIZE) TS_Data.YPos = Y_SIZE;

        sprintf(TS_Data.strXPos, "%3d", TS_Data.XPos);
        sprintf(TS_Data.strYPos, "%3d", TS_Data.YPos);

        sumx = 0;
        sumy = 0;
    }

    FSMC_WriteRegister(RA8875_REG_TPCR1, 0x41);
}


/*  */
uint8_t TP_CheckForTouch(void){

#if (TS_TOUCH_DETECT == TS_WITH_IF_FLAG)

    uint8_t reg = FSMC_ReadRegister(RA8875_REG_INTC2);

    if((reg&RA8875_INTC2_TP_IF) == RA8875_INTC2_TP_IF){
        FSMC_WriteRegister(RA8875_REG_INTC2, (reg&RA8875_INTC2_TP_IF));
        return 1;
    }

    return 0;

#else

    if((FSMC_ReadStatus()&RA8875_BIT_TOUCH_EVENT_DETECTED) == RA8875_BIT_TOUCH_EVENT_DETECTED) return 1;
    else return 0;

#endif
}


/*  */
void TS_IRQ_Handler(void){




}
