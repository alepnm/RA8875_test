
#include <math.h>
#include "adc.h"


#define ADC_CONVERTED_DATA_BUFFER_SIZE  ((uint32_t) 12)

#define VDDA_APPLI                      VREFINT_CAL_VREF

#define INTERNAL_TEMPSENSOR_AVGSLOPE    ((int32_t) 4300)
#define INTERNAL_TEMPSENSOR_V25         ((int32_t) 1430)
#define INTERNAL_TEMPSENSOR_V25_TEMP    ((int32_t)   25)
#define INTERNAL_TEMPSENSOR_V25_VREF    *(VREFINT_CAL_ADDR)


TaskHandle_t AdcHandle;

/* Variables for ADC conversion data */
adcdat_t xAdcData_BankA[ADC_CONVERTED_DATA_BUFFER_SIZE];
adcdat_t xAdcData_BankB[ADC_CONVERTED_DATA_BUFFER_SIZE];

volatile uint8_t ubDmaTransferComplete = 0;

uint16_t VRefValue = 0;
uint16_t CpuTemperature = 0;


static uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];


void xADC_Task(void* arg);
static void ADC_StoreAdcData(adcdat_t* bank);



/*  */
void ADC_TaskInit(UBaseType_t Priority)
{
    xTaskCreate((void*)xADC_Task, "AdcTask", configMINIMAL_STACK_SIZE, (void*)0, Priority, &AdcHandle);
}


/*  */
void xADC_Task(void* arg)
{
    /* DMA init */
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_0, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                           (uint32_t)&aADCxConvertedData, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, ADC_CONVERTED_DATA_BUFFER_SIZE);

    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);
    LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_0);

    LL_DMA_EnableStream(DMA2,LL_DMA_STREAM_0);

    LL_ADC_Enable(ADC1);

    vTaskDelay(10);

    while(1)
    {
        // perjungiam multipleksoriu i kanalus A

        LL_ADC_REG_StartConversionSWStart(ADC1);
        ubDmaTransferComplete = 0;

        while(!ubDmaTransferComplete) vTaskDelay(10);

        ADC_StoreAdcData(xAdcData_BankA);

        VRefValue =      __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, xAdcData_BankA[10].adcval, LL_ADC_RESOLUTION_10B);

        /* vienu metu konvertuojamas arba TEMPSENSOR arba VBAT <-- pasiaiskinti */
        CpuTemperature = __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(INTERNAL_TEMPSENSOR_AVGSLOPE, INTERNAL_TEMPSENSOR_V25, INTERNAL_TEMPSENSOR_V25_TEMP,
                         VDDA_APPLI, xAdcData_BankA[11].adcval, LL_ADC_RESOLUTION_10B);
        //CpuTemperature = __LL_ADC_CALC_TEMPERATURE(VRefValue,  xAdcData_BankA[10].adcval, LL_ADC_RESOLUTION_10B);

        vTaskDelay(10);


        /* cia perjungiam analoginius iejimus ir matuojam - 6 kanalai */
        // perjungiam multipleksoriu i kanalus B

        LL_ADC_REG_StartConversionSWStart(ADC1);
        ubDmaTransferComplete = 0;

        while(!ubDmaTransferComplete) vTaskDelay(10);

        ADC_StoreAdcData(xAdcData_BankB);

        vTaskDelay(100);
    }

    vTaskDelete(NULL);
}



/*    */
static void ADC_StoreAdcData(adcdat_t* bank)
{
    uint8_t i = 0;

    while(i < ADC_CONVERTED_DATA_BUFFER_SIZE ){

        (bank+i)->adcval = aADCxConvertedData[i];
        (bank+i)->adc_converted_mv = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, aADCxConvertedData[i], LL_ADC_RESOLUTION_10B);
        i++;
    }
}




/*

T = {[(0x4B8 - ADCDAT) * ADC_REF*1000]/4096} /1.25

0x4B8 is the ADCDAT at 0C
Voltage TC is around 1.25mV/C


*/
float ConvertADCvalueToTemp(uint16_t adcval)
{
    float T2, T1=298.15;

    //hU1-maitinimo itampa, hR-varza po maitinimo
    //sB- jutiklio beta koefic, sR- varza prie 25 laipsniu
    float U, hR=8200, sR=10000, hU1=3300, sB=3280;

    if(!adcval) adcval = 1;

    U = (adcval * hU1)/1024;

    T2 = (T1*sB)/(sB-T1 * (M_LN10 * log10((sR*(U-hU1))/((-U)*hR))));

    T2 -= 273.15;

    return T2;
}
