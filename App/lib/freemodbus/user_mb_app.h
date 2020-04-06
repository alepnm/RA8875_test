/******************************************************************************
* Project Name		: Stepper Motor Control
* File Name			: user_mb_app.h
* Version 			: 1.0
* Device Used		: STM32F051C8T6
* Software Used		: EmBitz 1.11
* Compiler    		: ARM GCC Compiller (EmBitz - bare -metal)
* Related Hardware	:
*
* Owner             : Ventmatika Inc.
*******************************************************************************/

#ifndef	USER_APP
#define USER_APP
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbconfig.h"

/* -----------------------Slave Defines -------------------------------------*/
#define     DISCRETE_INPUT_START        0
#define     DISCRETE_INPUT_NDISCRETES   32
#define     COIL_START                  0
#define     COIL_NCOILS                 32
#define     REG_INPUT_START             0
#define     REG_INPUT_NREGS             32
#define     REG_HOLDING_START           0
#define     REG_HOLDING_NREGS           32


/* COILS */
#define     CO_OUTPIN1                  ( 0u )
#define     CO_OUTPIN2                  ( 1u )
#define     CO_OUTPIN3                  ( 2u )
#define     CO_OUTPIN4                  ( 3u )
#define     CO_OUTPIN5                  ( 4u )
#define     CO_OUTPIN6                  ( 5u )
#define     CO_REG6                     ( 6u )
#define     CO_REG7                     ( 7u )
#define     CO_REG8                     ( 8u )
#define     CO_REG9                     ( 9u )
#define     CO_REG10                    ( 10u )
#define     CO_REG11                    ( 11u )
#define     CO_REG12                    ( 12u )

/* DESCREETS */
#define     DI_CHANNEL1                 ( 0u )
#define     DI_CHANNEL2                 ( 1u )
#define     DI_CHANNEL3                 ( 2u )
#define     DI_CHANNEL4                 ( 3u )
#define     DI_CHANNEL5                 ( 4u )
#define     DI_CHANNEL6                 ( 5u )
#define     DI_CHANNEL7                 ( 6u )
#define     DI_CHANNEL8                 ( 7u )
#define     DI_REG8                     ( 0u )
#define     DI_REG9                     ( 0u )
#define     DI_REG10                    ( 0u )
#define     DI_REG11                    ( 0u )
//...

/* INPUTS */
#define     IR_ADC_CH1                  ( 0u )
#define     IR_ADC_CH2                  ( 1u )
#define     IR_ADC_CH3                  ( 2u )
#define     IR_ADC_CH4                  ( 3u )
#define     IR_ADC_CH5                  ( 4u )
#define     IR_ADC_CH6                  ( 5u )
#define     IR_ADC_CH7                  ( 6u )
#define     IR_ADC_CH8                  ( 7u )
#define     IR_ADC_CH9                  ( 8u )
#define     IR_ADC_CH10                 ( 9u )
#define     IR_REG0                     ( 10u )
#define     IR_REG1                     ( 11u )
#define     IR_DS_QUANT                 ( 12u )
#define     IR_DS1_ID                   ( 13u )
#define     IR_DS1                      ( 14u )
#define     IR_DS2_ID                   ( 15u )
#define     IR_DS2                      ( 16u )
#define     IR_DS3_ID                   ( 17u )
#define     IR_DS3                      ( 18u )
#define     IR_DS4_ID                   ( 19u )
#define     IR_DS4                      ( 20u )
#define     IR_DS5_ID                   ( 21u )
#define     IR_DS5                      ( 22u )

//...

/* HOLDINGS */
#define     HR_USER_LANGUAGE            3   // 0-RU, 1-EN, 2-LT
#define     HR_MBADDR                   4   // 1-247
#define     HR_MBPARITY                 5
#define     HR_MBBAUDRATE               6
#define     HR_MBSTOPBITS               7
#define     HR_WTIME                    8
#define     HR_REG9                     9
#define     HR_010_OUT1                 10
#define     HR_010_OUT2                 11
#define     HR_010_OUT3                 12

//...


/* daugiafunkcinis registras:
 */
#define     HR_MAGIC_REG                ( 15u )     // 0x31h


/* default values */
#define     MB_ADDR_DEF                 ( 3u )
#define     MB_BAUDRATE_DEF             BR19200
#define     MB_PARITY_DEF               PARITY_NONE
#define     MB_STOPBITS_DEF             STOPBITS_1
#define     MB_DATABITS_DEF             8


/* extern data */
extern UCHAR        ucDiscInputBuf[];
extern UCHAR        ucCoilBuf[];
extern USHORT       usRegInputBuf[];
extern USHORT       usRegHoldingBuf[];
extern uint8_t      ucSlaveIdBuf[];

extern void MagicRegParser(uint16_t* magic_reg);
extern void MbProcessCoils(void);
extern void MbProcessDescreetes(void);
extern void MbProcessInputRegs(void);
extern void MbProcessHoldingRegs(void);


/**/
inline static uint8_t xMbGetCoil( uint16_t usBitOffset ) {
    return xMBUtilGetBits( ucCoilBuf, usBitOffset, 1 );
}


/**/
inline static void xMbSetCoil( uint16_t usBitOffset, uint8_t ucValue ) {
    xMBUtilSetBits( ucCoilBuf, usBitOffset, 1, ucValue );
}


/**/
inline static uint8_t xMbGetDInput( uint16_t usBitOffset ) {
    return xMBUtilGetBits( ucDiscInputBuf, usBitOffset, 1 );
}


/**/
inline static void xMbSetDInput( uint16_t usBitOffset, uint8_t ucValue ) {
    xMBUtilSetBits( ucDiscInputBuf, usBitOffset, 1, ucValue );
}


/**/
inline static uint8_t xMbGetNCoils( uint16_t usBitOffset, uint8_t ucNBits ) {
    return xMBUtilGetBits( ucCoilBuf, usBitOffset, ucNBits );
}


/**/
inline static void xMbSetNCoils( uint16_t usBitOffset, uint8_t ucNBits, uint8_t ucValue ) {
    xMBUtilSetBits( ucCoilBuf, usBitOffset, ucNBits, ucValue );
}

/**/
inline static uint8_t xMbGetNDInputs( uint16_t usBitOffset, uint8_t ucNBits ) {
    return xMBUtilGetBits( ucDiscInputBuf, usBitOffset, ucNBits );
}


/**/
inline static void xMbSetNDInputs( uint16_t usBitOffset, uint8_t ucNBits, uint8_t ucValue ) {
    xMBUtilSetBits( ucDiscInputBuf, usBitOffset, ucNBits, ucValue );
}




eMBErrorCode
eMBReportSlaveIdCB( UCHAR * pucRegBuffer, UCHAR ucNBytes );


#endif
