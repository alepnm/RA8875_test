#include "touch.h"
#include "ra8875.h"



/*  */
void TS_Init(void){

    TS_Data.IsEnabled = 0;
    TS_Data.IsTouched = 0;
    TS_Data.XPos = 0;
    TS_Data.YPos = 0;
}


/* skaitom tik busena - vykdom is irq */
void TS_ReadState(void){

    if(!TS_Data.IsEnabled) return;

    TS_Data.IsTouched = ((FSMC_ReadReg(0x74)&0x80) == 0x80) ? 0 : 1;
}


/* skaitom x-y reiksmes */
void TS_ReadXY(void){

    static int32_t tmpx, tmpy = 0;
    static uint8_t i = 0;

    if(TS_Data.IsTouched){

        FSMC_WriteReg(0x71, 0x42);
        LL_mDelay(1);

        FSMC_WriteReg(0x71, 0x43);
        LL_mDelay(1);

        tmpx += ( (FSMC_ReadReg(0x72)<<8) | (FSMC_ReadReg(0x74)&0x03) );
        tmpy += ( (FSMC_ReadReg(0x73)<<8) | (FSMC_ReadReg(0x74)&0x0C) );

        if(++i == 8){

            TS_Data.XAdc = tmpx>>3;
            TS_Data.YAdc = tmpy>>3;

            tmpx = (TS_Data.XAdc - 4000) * X_SIZE/60000;
            tmpy = (TS_Data.YAdc - 4000) * Y_SIZE/54000;

            tmpx = (tmpx < 0) ? 0 : tmpx;
            tmpx = (tmpx > X_SIZE) ? X_SIZE : tmpx;

            tmpy = (tmpy < 0) ? 0 : tmpy;
            tmpy = (tmpy > Y_SIZE) ? Y_SIZE : tmpy;

            TS_Data.XPos = (uint16_t)tmpx;
            TS_Data.YPos = (uint16_t)tmpy;

            tmpx = tmpy = 0;

            i = 0;
        }

        FSMC_WriteReg(0x71, 0x41);
    }else{

        TS_Data.XPos = 0;
        TS_Data.YPos = 0;
    }
}
