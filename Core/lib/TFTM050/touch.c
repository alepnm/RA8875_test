#include "touch.h"
#include "ra8875.h"



/*  */
void TS_Init(void){

    TS_Data.IsEnabled = 0;
    TS_Data.IsTouched = 0;
    TS_Data.XPos = 0;
    TS_Data.YPos = 0;

    FSMC_WriteReg(0x71, 0x41);
}


/* skaitom tik busena - vykdom is irq */
void TS_ReadState(void){

    if(!TS_Data.IsEnabled) return;

    TS_Data.IsTouched = ((FSMC_ReadReg(0x74)&0x80) == 0x80) ? 0 : 1;

}


/* skaitom x-y reiksmes */
void TS_ReadXY(void){

    int32_t tmpx = 0, tmpy = 0;

    if(TS_Data.IsTouched){

        FSMC_WriteReg(0x71, 0x42);
        LL_mDelay(1);

        FSMC_WriteReg(0x71, 0x43);
        LL_mDelay(1);

        TS_Data.XAdc = (FSMC_ReadReg(0x72)<<2); // TS_Data.XAdc = ( (FSMC_ReadReg(0x72)<<2) | (FSMC_ReadReg(0x74)&0x03) );
        TS_Data.YAdc = (FSMC_ReadReg(0x73)<<2); // TS_Data.YAdc = ( (FSMC_ReadReg(0x73)<<2) | ((FSMC_ReadReg(0x74)&0x0C)>>2) );

        tmpx = (TS_Data.XAdc-60) * X_SIZE / 920;
        tmpy = (TS_Data.YAdc-80) * Y_SIZE / 820;

        tmpx = (tmpx < 0) ? 0 : tmpx;
        tmpx = (tmpx > X_SIZE) ? X_SIZE : tmpx;

        tmpy = (tmpy < 0) ? 0 : tmpy;
        tmpy = (tmpy > Y_SIZE) ? Y_SIZE : tmpy;

        TS_Data.XPos = (uint16_t)tmpx;
        TS_Data.YPos = (uint16_t)tmpy;

        FSMC_WriteReg(0x71, 0x41);
    }else{

        TS_Data.XPos = -1;
        TS_Data.YPos = -1;
    }
}
