#ifndef TP_H_INCLUDED
#define TP_H_INCLUDED

#include "stm32f4xx.h"

struct _tp{
    uint8_t        IsTouched;
    __IO uint16_t  XPos;
    __IO uint16_t  YPos;

    char           strXPos[10];
    char           strYPos[10];
}TP_Data;


#endif /* TP_H_INCLUDED */
