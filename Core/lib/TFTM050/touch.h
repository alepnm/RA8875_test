#ifndef TP_H_INCLUDED
#define TP_H_INCLUDED

#include "stm32f4xx.h"

struct _tp{
    uint8_t        IsEnabled;
    uint8_t        IsTouched;
    uint8_t        ConvStep;

    __IO uint16_t  XAdc;
    __IO uint16_t  YAdc;
    __IO int       XPos;
    __IO int       YPos;

    char           strXPos[10];
    char           strYPos[10];
}TS_Data;



void TS_Init(void);
void TS_ReadXY(void);



#endif /* TP_H_INCLUDED */
