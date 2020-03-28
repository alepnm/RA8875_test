#ifndef TP_H_INCLUDED
#define TP_H_INCLUDED

#include "stm32f4xx.h"

struct _tp{
    uint8_t        IsEnabled;
    uint8_t        IsTouched;

    __IO uint16_t  XAdc;
    __IO uint16_t  YAdc;
    __IO uint16_t  XPos;
    __IO uint16_t  YPos;

    char           strXPos[10];
    char           strYPos[10];
}TS_Data;



void TS_Init(void);
void TS_ReadState(void);
void TS_ReadXY(void);



#endif /* TP_H_INCLUDED */
