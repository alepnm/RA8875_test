#ifndef ADC_H_INCLUDED
#define ADC_H_INCLUDED


#include "cmsis_os.h"
#include "main.h"

typedef struct{
    uint16_t   adcval;
    uint16_t   adc_conv_mv;
    float adc_conv_temp;
    char str_temp[10];
}adcdat_t;

extern adcdat_t xAdcData_BankA[];
extern adcdat_t xAdcData_BankB[];

extern uint16_t VRefValue;
extern uint16_t CpuTemperature;


/*  */
void ADC_TaskInit(UBaseType_t Priority);
float ConvertADCvalueToTemp(uint16_t adcval);

#endif /* ADC_H_INCLUDED */
