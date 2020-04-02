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


/* skaitom x-y reiksmes */
void TS_ReadXY(void){

    TS_Data.IsTouched = ((FSMC_ReadReg(0x74)&0x80) == 0x80) ? 0 : 1;

    if(TS_Data.IsTouched){

        FSMC_WriteReg(0x71, (0x42|0x04));
        osDelay(1);

        FSMC_WriteReg(0x71, (0x43|0x04));
        osDelay(1);

        FSMC_WriteReg(0x71, 0x40);

        TS_Data.XAdc = (FSMC_ReadReg(0x72)<<2); // TS_Data.XAdc = ( (FSMC_ReadReg(0x72)<<2) | (FSMC_ReadReg(0x74)&0x03) );
        TS_Data.YAdc = (FSMC_ReadReg(0x73)<<2); // TS_Data.YAdc = ( (FSMC_ReadReg(0x73)<<2) | ((FSMC_ReadReg(0x74)&0x0C)>>2) );

        TS_Data.XPos = (TS_Data.XAdc-50) * X_SIZE / 920;
        TS_Data.YPos = (TS_Data.YAdc-80) * Y_SIZE / 820;

        FSMC_WriteReg(0x71, 0x41);

    }else{

        TS_Data.XPos = -1;
        TS_Data.YPos = -1;
    }
}
