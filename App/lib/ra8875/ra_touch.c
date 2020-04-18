#include "ra8875.h"


/*  */
uint8_t TS_Init(void){

    TS_Data.IsTouched = 0;
    TS_Data.XPos = 0;
    TS_Data.YPos = 0;

    FSMC_WriteRegister(0x71, 0x41);

    if(FSMC_ReadRegister(0x71) == (0x41|0x80)){
        TS_Data.IsEnabled = 1;
        return 1;
    }
    else{
        TS_Data.IsEnabled = 0;
        return 0;
    }
}


/* skaitom x-y reiksmes */
uint8_t TS_ReadXY(void){

    //TS_Data.IsTouched = ((FSMC_ReadRegister(0x74)&0x80) == 0x80) ? 0 : 1;
    TS_Data.IsTouched = ((FSMC_ReadStatus()&0x20) == 0x20) ? 1 : 0;

    if(TS_Data.IsTouched){

        FSMC_WriteRegister(0x71, (0x42|0x04));
        HAL_Delay(1);

        FSMC_WriteRegister(0x71, (0x43|0x04));
        HAL_Delay(1);

        FSMC_WriteRegister(0x71, 0x40);

        TS_Data.XAdc = (FSMC_ReadRegister(0x72)<<2); // TS_Data.XAdc = ( (FSMC_ReadRegister(0x72)<<2) | (FSMC_ReadRegister(0x74)&0x03) );
        TS_Data.YAdc = (FSMC_ReadRegister(0x73)<<2); // TS_Data.YAdc = ( (FSMC_ReadRegister(0x73)<<2) | ((FSMC_ReadRegister(0x74)&0x0C)>>2) );

        TS_Data.XPos = (TS_Data.XAdc-50) * X_SIZE / 920;
        TS_Data.YPos = (TS_Data.YAdc-80) * Y_SIZE / 820;

        FSMC_WriteRegister(0x71, 0x41);

    }else{

        TS_Data.XPos = -1;
        TS_Data.YPos = -1;
    }

    sprintf(TS_Data.strXPos, "%3d", TS_Data.XPos);
    sprintf(TS_Data.strYPos, "%3d", TS_Data.YPos);

    return TS_Data.IsTouched;
}
