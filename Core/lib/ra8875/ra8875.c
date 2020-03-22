#include "main.h"
#include "ra8875.h"
#include "ra8875_registers.h"







uint8_t RA8875_Init(void) {




    RA8875_SetBackLight(40);

    return 1;
}



void RA8875_SetBackLight(uint8_t val) {

    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_OC_SetCompareCH1(TIM1, val);
}



