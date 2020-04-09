
#ifndef ONEWIRE_H
#define ONEWIRE_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "main.h"


#if (DS18B20_CONFIG_USE_FREERTOS==1)
    #define	mDelay(x)			vTaskDelay(x)
#else
    #define	mDelay(x)			LL_mDelay(x)
#endif


    typedef struct
    {
        GPIO_TypeDef*  GPIOx;                 /*!< GPIOx port to be used for I/O functions */
        uint32_t       GPIO_Pin;              /*!< GPIO Pin to be used for I/O functions */
        uint8_t        LastDiscrepancy;       /*!< Search private */
        uint8_t        LastFamilyDiscrepancy; /*!< Search private */
        uint8_t        LastDeviceFlag;        /*!< Search private */
        uint8_t        ROM_NO[8];             /*!< 8-bytes address of last search device */
    } OneWire_t;


    /* OneWire commands */
#define ONEWIRE_CMD_RSCRATCHPAD			0xBE
#define ONEWIRE_CMD_WSCRATCHPAD			0x4E
#define ONEWIRE_CMD_CPYSCRATCHPAD		0x48
#define ONEWIRE_CMD_RECEEPROM			0xB8
#define ONEWIRE_CMD_RPWRSUPPLY			0xB4
#define ONEWIRE_CMD_SEARCHROM			0xF0
#define ONEWIRE_CMD_READROM				0x33
#define ONEWIRE_CMD_MATCHROM			0x55
#define ONEWIRE_CMD_SKIPROM				0xCC

//#######################################################################################################
    void    OW_Init(OneWire_t* OneWireStruct, GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin);
    uint8_t OW_Reset(OneWire_t* OneWireStruct);

    uint8_t OW_ReadByte(OneWire_t* OneWireStruct);
    void    OW_WriteByte(OneWire_t* OneWireStruct, uint8_t byte);
    void    OW_WriteBit(OneWire_t* OneWireStruct, uint8_t bit);
    uint8_t OW_ReadBit(OneWire_t* OneWireStruct);

    uint8_t OW_Search(OneWire_t* OneWireStruct, uint8_t command);
    uint8_t OW_First(OneWire_t* OneWireStruct);
    uint8_t OW_Next(OneWire_t* OneWireStruct);
    int     OW_Verify(OneWire_t* OneWireStruct);
    void    OW_TargetSetup(OneWire_t* OneWireStruct, uint8_t family_code);
    void    OW_FamilySkipSetup(OneWire_t* OneWireStruct);
    uint8_t OW_GetROM(OneWire_t* OneWireStruct, uint8_t index);
    void    OW_GetFullROM(OneWire_t* OneWireStruct, uint8_t *firstIndex);
    void    OW_Select(OneWire_t* OneWireStruct, uint8_t* addr);
    void    OW_SelectWithPointer(OneWire_t* OneWireStruct, uint8_t* ROM);
    uint8_t OW_CRC8(uint8_t* addr, uint8_t len);
//#######################################################################################################

    /* C++ detection */
#ifdef __cplusplus
}
#endif

#endif

