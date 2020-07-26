#include "cmsis_os.h"
#include "ra8875.h"


#define TS_XADC_MIN       60
#define TS_XADC_MAX       980
#define TS_YADC_MIN       100
#define TS_YADC_MAX       900


struct _tp TP_Data;


/*  */
uint8_t TP_Init(void){

    TP_Data.State.Pressed = 0;
    TP_Data.State.Layer = 0;
    TP_Data.State.x = 0;
    TP_Data.State.y = 0;

    if((FSMC_ReadRegister(RA8875_REG_TPCR0)&RA8875_TPCR0_TOUCH_ENABLE) == RA8875_TPCR0_TOUCH_ENABLE){
        TP_Data.IsEnabled = 1;
        return 1;
    }
    else{
        TP_Data.IsEnabled = 0;
        return 0;
    }
}


/* skaitom XY reiksmes (Manual Mode)*/
void TP_ReadXY_Manual(void){

    FSMC_WriteRegister(RA8875_REG_TPCR1, RA8875_TPCR1_MANUAL_MODE|RA8875_TPCR1_LATCH_XDATA);
    osDelay(3);

    FSMC_WriteRegister(RA8875_REG_TPCR1, RA8875_TPCR1_MANUAL_MODE|RA8875_TPCR1_LATCH_YDATA);
    osDelay(3);

    FSMC_WriteRegister(RA8875_REG_TPCR1, RA8875_TPCR1_MANUAL_MODE|RA8875_TPCR1_MODE_IDLE);

    TP_ReadXY();
}


/* konvertuojame XY (Auto Mode) */
void TP_ReadXY(void){

    static GUI_PID_STATE last_state;
    uint16_t xDiff = 0, yDiff = 0;

    TP_Data.XAdc = ((FSMC_ReadRegister(RA8875_REG_TPXH)<<2)|(FSMC_ReadRegister(RA8875_REG_TPXYL)&0x03));
    TP_Data.YAdc = ((FSMC_ReadRegister(RA8875_REG_TPYH)<<2)|((FSMC_ReadRegister(RA8875_REG_TPXYL)>>2)&0x03));

    FSMC_WriteRegister(RA8875_REG_TPCR1, RA8875_TPCR1_MANUAL_MODE|RA8875_TPCR1_MODE_WAIT_EVENT);

    xDiff = (last_state.x > TP_Data.XAdc) ? (last_state.x - TP_Data.XAdc) : (TP_Data.XAdc - last_state.x);
    yDiff = (last_state.y > TP_Data.YAdc) ? (last_state.y - TP_Data.YAdc) : (TP_Data.YAdc - last_state.y);

    last_state.x = TP_Data.XAdc;
    last_state.y = TP_Data.YAdc;

    if(xDiff > 3 || yDiff > 3){

        TP_Data.State.x = X_SIZE*(float)(TP_Data.XAdc-TS_XADC_MIN)/(TS_XADC_MAX-TS_XADC_MIN);
        TP_Data.State.y = Y_SIZE*(float)(TP_Data.YAdc-TS_YADC_MIN)/(TS_YADC_MAX-TS_YADC_MIN);

        if(TP_Data.State.x < 0) TP_Data.State.x = 0;
        if(TP_Data.State.y < 0) TP_Data.State.y = 0;

        if(TP_Data.State.x > X_SIZE) TP_Data.State.x = X_SIZE;
        if(TP_Data.State.y > Y_SIZE) TP_Data.State.y = Y_SIZE;

        sprintf(TP_Data.strXPos, "%3d", TP_Data.State.x);
        sprintf(TP_Data.strYPos, "%3d", TP_Data.State.y);
    }
}





/* Touchpad skaitymas su rezultato vidurkinimu */
void TP_ReadXY_Avg(void){

    static uint8_t n = 0;
    static uint16_t sumx = 0, sumy = 0, lastx = 0, lasty = 0;

    TP_Data.XAdc = ((FSMC_ReadRegister(RA8875_REG_TPXH)<<2)|(FSMC_ReadRegister(RA8875_REG_TPXYL)&0x03));
    TP_Data.YAdc = ((FSMC_ReadRegister(RA8875_REG_TPYH)<<2)|((FSMC_ReadRegister(RA8875_REG_TPXYL)>>2)&0x03));

    FSMC_WriteRegister(RA8875_REG_TPCR1, 0x41);

    sumx = sumx + (lastx + TP_Data.XAdc)/2;
    sumy = sumy + (lasty + TP_Data.YAdc)/2;

    lastx = TP_Data.XAdc;
    lasty = TP_Data.YAdc;

    if(n++ == 3){

        n = 0;

        TP_Data.State.x = X_SIZE*(float)((sumx>>2)-TS_XADC_MIN)/(TS_XADC_MAX-TS_XADC_MIN);
        TP_Data.State.y = Y_SIZE*(float)((sumy>>2)-TS_YADC_MIN)/(TS_YADC_MAX-TS_YADC_MIN);

        if(TP_Data.State.x < 0) TP_Data.State.x = 0;
        if(TP_Data.State.y < 0) TP_Data.State.y = 0;

        if(TP_Data.State.x > X_SIZE) TP_Data.State.x = X_SIZE;
        if(TP_Data.State.y > Y_SIZE) TP_Data.State.y = Y_SIZE;

        sprintf(TP_Data.strXPos, "%3d", TP_Data.State.x);
        sprintf(TP_Data.strYPos, "%3d", TP_Data.State.y);

        sumx = 0;
        sumy = 0;
    }
}


/*  */
uint8_t TP_CheckStatus(void){

    return (FSMC_ReadStatus()&RA8875_STSR_TOUCH_EVENT_DETECTED);
}


/*  */
void TP_IRQ_Handler(void){




}
