#ifndef __RA_FSMC_H
#define __RA_FSMC_H

#include "main.h"
#include "cmsis_os.h"


#define Bank1_SRAM1_ADDR  ((uint32_t)0x60000000)  // NE1
#define Bank1_SRAM2_ADDR  ((uint32_t)0x64000000)  // NE2
#define Bank1_SRAM3_ADDR  ((uint32_t)0x68000000)  // NE3
#define Bank1_SRAM4_ADDR  ((uint32_t)0x6C000000)  // NE4

typedef struct {
    __IO uint16_t LCD_REG;
    __IO uint16_t LCD_RAM;
}LCD_TypeDef;

/* LCD is connected to the FSMC_Bank1_NOR/SRAM1 and NE1 is used as ship select signal */
#define LCD_BASE    ((uint32_t)(Bank1_SRAM1_ADDR | 0x000FFFFE))
#define LCD         ( (LCD_TypeDef *) LCD_BASE)


#define FSMC_WAIT_BUSY() FSMC_Wait()//FSMC_WaitMem()


extern SemaphoreHandle_t xFsmcMutexHandle;



/*  */
static inline void FSMC_Wait(void){

    __disable_irq();

    while(!LL_GPIO_IsInputPinSet(LCD_WAIT_GPIO_Port, LCD_WAIT_Pin));

    __enable_irq();
}

/*  */
static inline void FSMC_WaitMem(void){

    __disable_irq();

    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
    while((LCD->LCD_RAM&0x80) == 0x80);
    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);

    __enable_irq();
}


/*  */
static inline void FSMC_WaitBTE(void){

    __disable_irq();

    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
    while((LCD->LCD_RAM&0x40) == 0x40);
    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);

    __enable_irq();
}


/*  */
static inline void FSMC_WaitROM(void){

    __disable_irq();

    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
    while((LCD->LCD_RAM&0x01) == 0x01);
    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);

    __enable_irq();
}


/*  */
static inline uint16_t FSMC_ReadStatus(void){

    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
    uint16_t status = LCD->LCD_RAM;
    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);

    return status;
}


/*  */
static inline uint16_t FSMC_ReadRegister(uint16_t reg){

    LCD->LCD_REG = reg;
    return LCD->LCD_RAM;
}

/*  */
static inline void FSMC_WriteRegister(uint16_t reg, uint16_t val){

    LCD->LCD_REG = reg;
    LCD->LCD_RAM = val;
}


/*  */
static inline uint16_t FSMC_GetA0(void){

    FSMC_WAIT_BUSY();
    return LCD->LCD_RAM;
}

/*  */
static inline void FSMC_SetA0(uint16_t data){

    LCD->LCD_RAM = data;
    FSMC_WAIT_BUSY();
}


/*  */
static inline uint16_t FSMC_GetA1(void){

    return LCD->LCD_REG;
}

/*  */
static inline void FSMC_SetA1(uint16_t data){

    LCD->LCD_REG = data;
}




void FSMC_ReadDDRAM(uint16_t *pdata, int pixels);
void FSMC_WriteDDRAM(uint16_t *pdata, int pixels);


#endif /* __RA_FSMC_H */
