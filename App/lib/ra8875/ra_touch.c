#include "cmsis_os.h"
#include "ra8875.h"


#define TS_WITH_STATUS    0
#define TS_WITH_IF_FLAG   1
#define TS_TOUCH_DETECT   TS_WITH_IF_FLAG


/* Touch Panel Control Register0 0x70 */
#define TP_CR0_SYSTEM_CLK         0x00
#define TP_CR0_SYSTEM_CLK_DIV2    0x01
#define TP_CR0_SYSTEM_CLK_DIV4    0x02
#define TP_CR0_SYSTEM_CLK_DIV8    0x03
#define TP_CR0_SYSTEM_CLK_DIV16   0x04
#define TP_CR0_SYSTEM_CLK_DIV32   0x05
#define TP_CR0_SYSTEM_CLK_DIV64   0x06
#define TP_CR0_SYSTEM_CLK_DIV128  0x07
#define TP_CR0_WAKEUP_ENABLE      0x08
#define TP_CR0_WAIT_512_CLOCS     (0x00<<5)
#define TP_CR0_WAIT_1024_CLOCS    (0x01<<5)
#define TP_CR0_WAIT_2048_CLOCS    (0x02<<5)
#define TP_CR0_WAIT_4096_CLOCS    (0x03<<5)
#define TP_CR0_WAIT_8192_CLOCS    (0x04<<5)
#define TP_CR0_WAIT_16384_CLOCS   (0x05<<5)
#define TP_CR0_WAIT_32768_CLOCS   (0x06<<5)
#define TP_CR0_WAIT_65536_CLOCS   (0x07<<5)
#define TP_CR0_TOUCH_ENABLE       0x80


/* Touch Panel Control Register1 0x71 */
#define TP_CR1_MANUAL_MODE        0x40
#define TP_CR1_VREF_EXTERNAL      0x20
#define TP_CR1_DEBOUNCE_ENABLE    0x04
#define TP_CR1_IDLE               0x00
#define TP_CR1_WAIT_EVENT         0x01
#define TP_CR1_LATCH_XDATA        0x02
#define TP_CR1_LATCH_YDATA        0x03


/* Interrubt Control Register Touch bits */
#define TP_ICR1_IRQ_ENABLE        0x04   //  IRQ enable/disable
#define TP_ICR2_IF_FLAG           0x04   //  IRQ flag



/*  */
uint8_t TS_Init(void){

    TS_Data.IsTouched = 0;
    TS_Data.XPos = 0;
    TS_Data.YPos = 0;

    // Touch Panel Control Register0
    FSMC_WriteRegister(RA8875_REG_TPCR0, TP_CR0_TOUCH_ENABLE|TP_CR0_WAIT_4096_CLOCS|TP_CR0_SYSTEM_CLK_DIV16);

    // Touch Panel Control Register1
    FSMC_WriteRegister(RA8875_REG_TPCR1, TP_CR1_MANUAL_MODE|TP_CR1_WAIT_EVENT|TP_CR1_DEBOUNCE_ENABLE);

    uint8_t reg = FSMC_ReadRegister(RA8875_REG_INTC1);
    //FSMC_WriteRegister(0xF0, reg|TP_ICR1_IRQ_ENABLE);   // ijungiam/isjungiam Touch IRQ

    if((FSMC_ReadRegister(RA8875_REG_TPCR1)&TP_CR0_TOUCH_ENABLE) == TP_CR0_TOUCH_ENABLE){
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

    if((reg&TP_ICR2_IF_FLAG) == TP_ICR2_IF_FLAG){
        FSMC_WriteRegister(RA8875_REG_INTC2, (reg&TP_ICR2_IF_FLAG));
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
