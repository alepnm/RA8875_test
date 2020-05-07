
#include "ds18b20.h"

//###################################################################################
Ds18b20Sensor_t	ds18b20[DS18B20_CONFIG_MAX_SENSORS];

OneWire_t   ds;
uint8_t	    OneWireDevices = 0;
uint8_t 	TempSensorCount = 0;
uint8_t		Ds18b20StartConvert = 0;
uint16_t	Ds18b20Timeout = 0;

#if (DS18B20_CONFIG_USE_FREERTOS == 1)

TaskHandle_t Ds18b20Handle;
void         xDS18B20_Task(void* arg);
#endif

//###########################################################################################
#if (DS18B20_CONFIG_USE_FREERTOS == 1)
void	DS18B20_TaskInit(UBaseType_t Priority)
{
    xTaskCreate((void*)xDS18B20_Task, "OwTask", configMINIMAL_STACK_SIZE+128, (void*)0, Priority, &Ds18b20Handle);
}
#else

#warning sutvarkyti analogiskai kaip ir taskas
bool	DS18B20_Init(void)
{
    uint8_t	Ds18b20TryToFind = 3;

    do
    {
        do
        {
            OW_Init(&ds, DS18B20_CONFIG_GPIO, DS18B20_CONFIG_PIN);
            TempSensorCount = 0;

            mDelay(1000);

            OneWireDevices = OW_First(&ds);

            while (OneWireDevices)
            {
                OW_GetFullROM(&ds, ds18b20[TempSensorCount].Address);

                OneWireDevices = OW_Next(&ds);

                TempSensorCount++;
            }

            mDelay(100);

        }
        while(TempSensorCount < DS18B20_CONFIG_MAX_SENSORS-1);


        if(TempSensorCount > 0) break;

        Ds18b20TryToFind--;

    }
    while(Ds18b20TryToFind > 0);

    if(Ds18b20TryToFind == 0) return false;

    for (uint8_t i = 0; i < TempSensorCount; i++)
    {
        mDelay(50);
        DS18B20_SetResolution(&ds, ds18b20[i].Address, DS18B20_Resolution_12bits);
        mDelay(50);
        DS18B20_DisableAlarmTemperature(&ds,  ds18b20[i].Address);
    }

    return true;
}
#endif


//###########################################################################################
bool	DS18B20_ManualConvert(void)
{
#if (DS18B20_CONFIG_USE_FREERTOS == 1)

    Ds18b20StartConvert = 1;

    while(Ds18b20StartConvert == 1) mDelay(10);

    if(Ds18b20Timeout == 0) return false;
    else return true;
#else
    Ds18b20Timeout = DS18B20_CONFIG_CONVERT_TIMEOUT_MS/10;

    DS18B20_StartAll(&ds);

    mDelay(100);

    while(!DS18B20_AllDone(&ds))
    {
        mDelay(10);
        Ds18b20Timeout -= 1;

        if(Ds18b20Timeout == 0) break;
    }

    if(Ds18b20Timeout)
    {
        for (uint8_t i = 0; i < TempSensorCount; i++)
        {
            mDelay(100);
            ds18b20[i].DataIsValid = DS18B20_Read(&ds, ds18b20[i].Address, &ds18b20[i].Temperature);
        }
    }
    else
    {
        for (uint8_t i = 0; i < TempSensorCount; i++)
            ds18b20[i].DataIsValid = false;
    }

    if(Ds18b20Timeout == 0) return false;
    else return true;
#endif
}

//###########################################################################################
#if (DS18B20_CONFIG_USE_FREERTOS == 1)
void xDS18B20_Task(void* arg)
{
    uint8_t	try_to_read = DS18B20_TRYES_TO_READ;

    OW_Init(&ds, DS18B20_CONFIG_GPIO, DS18B20_CONFIG_PIN);

    TempSensorCount = 0;

    mDelay(10);

    OneWireDevices = OW_First(&ds);

    while (OneWireDevices)
    {
        mDelay(10);

        OW_GetFullROM(&ds, ds18b20[TempSensorCount].Address);

        ds18b20[TempSensorCount].DevID = ds18b20[TempSensorCount].Address[7];

        ds18b20[TempSensorCount].Alive = true;

        OneWireDevices = OW_Next(&ds);

        TempSensorCount++;
    }

    if(TempSensorCount == 0) vTaskDelete(NULL);

    for (uint8_t i = 0; i < TempSensorCount; i++)
    {
        mDelay(10);
        DS18B20_SetResolution(&ds, ds18b20[i].Address, DS18B20_Resolution_10bits);
        mDelay(10);
        DS18B20_DisableAlarmTemperature(&ds,  ds18b20[i].Address);
    }


    for(;;)
    {
        DS18B20_StartAll(&ds);

        Ds18b20Timeout = DS18B20_CONFIG_CONVERT_TIMEOUT_MS;

        while (!DS18B20_AllDone(&ds) && Ds18b20Timeout)
        {
            mDelay(100);
            Ds18b20Timeout--;
        }

        if(Ds18b20Timeout)
        {
            for (uint8_t i = 0; i < TempSensorCount; i++)
            {
                try_to_read = DS18B20_TRYES_TO_READ;

                mDelay(20);

                do
                {
                    ds18b20[i].DataIsValid = DS18B20_Read(&ds, ds18b20[i].Address, &ds18b20[i].Temperature);
                    mDelay(10);
                }
                while(ds18b20[i].DataIsValid == false && --try_to_read);

                ds18b20[i].Alive = (try_to_read) ? true : false;

                sprintf(ds18b20[i].TemperatureStr, "%2.2f°C", ds18b20[i].Temperature);
            }
        }
        else
        {
            for (uint8_t i = 0; i < TempSensorCount; i++)
                ds18b20[i].DataIsValid = false;
        }

        mDelay(DS18B20_CONFIG_UPDATE_INTERVAL_MS);
    }
}
#endif



//###########################################################################################
uint8_t DS18B20_Start(OneWire_t* ds, uint8_t *ROM)
{
    /* Check if device is DS18B20 */
    if (!DS18B20_Is(ROM)) return 0;

    /* Reset line */
    OW_Reset(ds);
    /* Select ROM number */
    OW_SelectWithPointer(ds, ROM);
    /* Start temperature conversion */
    OW_WriteByte(ds, DS18B20_CMD_CONVERTTEMP);

    return 1;
}

void DS18B20_StartAll(OneWire_t* ds)
{
    /* Reset pulse */
    OW_Reset(ds);
    /* Skip rom */
    OW_WriteByte(ds, ONEWIRE_CMD_SKIPROM);
    /* Start conversion on all connected devices */
    OW_WriteByte(ds, DS18B20_CMD_CONVERTTEMP);
}

bool DS18B20_Read(OneWire_t* ds, uint8_t *ROM, float *destination)
{
    uint16_t temperature;
    uint8_t resolution, crc;
    int8_t digit, minus = 0;
    float decimal;
    uint8_t data[9];

    /* Check if device is DS18B20 */
    if (!DS18B20_Is(ROM)) return false;

    /* Check if line is released, if it is, then conversion is complete */
    if (!OW_ReadBit(ds)) return false; // Conversion is not finished yet

    /* Reset line */
    OW_Reset(ds);
    /* Select ROM number */
    OW_SelectWithPointer(ds, ROM);
    /* Read scratchpad command by onewire protocol */
    OW_WriteByte(ds, ONEWIRE_CMD_RSCRATCHPAD);

    /* Get data */
    for (uint8_t i = 0; i < 9; i++)
    {
        /* Read byte by byte */
        data[i] = OW_ReadByte(ds);
    }

    /* Calculate CRC */
    crc = OW_CRC8(data, 8);

    /* Check if CRC is ok */
    if (crc != data[8])
        /* CRC invalid */
        return 0;

    /* First two bytes of scratchpad are temperature values */
    temperature = data[0] | (data[1] << 8);

    /* Reset line */
    OW_Reset(ds);

    /* Check if temperature is negative */
    if (temperature & 0x8000)
    {
        /* Two's complement, temperature is negative */
        temperature = ~temperature + 1;
        minus = 1;
    }

    /* Get sensor resolution */
    resolution = ((data[4] & 0x60) >> 5) + 9;

    /* Store temperature integer digits and decimal digits */
    digit = temperature >> 4;
    digit |= ((temperature >> 8) & 0x7) << 4;

    /* Store decimal digits */
    switch (resolution)
    {
    case 9:
        decimal = (temperature >> 3) & 0x01;
        decimal *= (float)DS18B20_DECIMAL_STEPS_9BIT;
        break;
    case 10:
        decimal = (temperature >> 2) & 0x03;
        decimal *= (float)DS18B20_DECIMAL_STEPS_10BIT;
        break;
    case 11:
        decimal = (temperature >> 1) & 0x07;
        decimal *= (float)DS18B20_DECIMAL_STEPS_11BIT;
        break;
    case 12:
        decimal = temperature & 0x0F;
        decimal *= (float)DS18B20_DECIMAL_STEPS_12BIT;
        break;
    default:
        decimal = 0xFF;
        digit = 0;
    }

    /* Check for negative part */
    decimal = digit + decimal;

    if (minus) decimal = 0 - decimal;

    /* Set to pointer */
    *destination = decimal;

    /* Return 1, temperature valid */
    return true;
}

uint8_t DS18B20_GetResolution(OneWire_t* ds, uint8_t *ROM)
{
    uint8_t conf;

    if (!DS18B20_Is(ROM)) return 0;

    /* Reset line */
    OW_Reset(ds);
    /* Select ROM number */
    OW_SelectWithPointer(ds, ROM);
    /* Read scratchpad command by onewire protocol */
    OW_WriteByte(ds, ONEWIRE_CMD_RSCRATCHPAD);

    /* Ignore first 4 bytes */
    OW_ReadByte(ds);
    OW_ReadByte(ds);
    OW_ReadByte(ds);
    OW_ReadByte(ds);

    /* 5th byte of scratchpad is configuration register */
    conf = OW_ReadByte(ds);

    /* Return 9 - 12 value according to number of bits */
    return ((conf & 0x60) >> 5) + 9;
}

uint8_t DS18B20_SetResolution(OneWire_t* ds, uint8_t *ROM, DS18B20_Resolution_t resolution)
{
    uint8_t th, tl, conf;

    if (!DS18B20_Is(ROM)) return 0;

    /* Reset line */
    OW_Reset(ds);
    /* Select ROM number */
    OW_SelectWithPointer(ds, ROM);
    /* Read scratchpad command by onewire protocol */
    OW_WriteByte(ds, ONEWIRE_CMD_RSCRATCHPAD);

    /* Ignore first 2 bytes */
    OW_ReadByte(ds);
    OW_ReadByte(ds);

    th = OW_ReadByte(ds);
    tl = OW_ReadByte(ds);
    conf = OW_ReadByte(ds);

    if (resolution == DS18B20_Resolution_9bits)
    {
        conf &= ~(1 << DS18B20_RESOLUTION_R1);
        conf &= ~(1 << DS18B20_RESOLUTION_R0);
    }
    else if (resolution == DS18B20_Resolution_10bits)
    {
        conf &= ~(1 << DS18B20_RESOLUTION_R1);
        conf |= 1 << DS18B20_RESOLUTION_R0;
    }
    else if (resolution == DS18B20_Resolution_11bits)
    {
        conf |= 1 << DS18B20_RESOLUTION_R1;
        conf &= ~(1 << DS18B20_RESOLUTION_R0);
    }
    else if (resolution == DS18B20_Resolution_12bits)
    {
        conf |= 1 << DS18B20_RESOLUTION_R1;
        conf |= 1 << DS18B20_RESOLUTION_R0;
    }

    /* Reset line */
    OW_Reset(ds);
    /* Select ROM number */
    OW_SelectWithPointer(ds, ROM);
    /* Write scratchpad command by onewire protocol, only th, tl and conf register can be written */
    OW_WriteByte(ds, ONEWIRE_CMD_WSCRATCHPAD);

    /* Write bytes */
    OW_WriteByte(ds, th);
    OW_WriteByte(ds, tl);
    OW_WriteByte(ds, conf);

    /* Reset line */
    OW_Reset(ds);
    /* Select ROM number */
    OW_SelectWithPointer(ds, ROM);
    /* Copy scratchpad to EEPROM of DS18B20 */
    OW_WriteByte(ds, ONEWIRE_CMD_CPYSCRATCHPAD);

    return 1;
}

uint8_t DS18B20_Is(uint8_t *ROM)
{
    /* Checks if first byte is equal to DS18B20's family code */
    if (*ROM == DS18B20_FAMILY_CODE) return 1;

    return 0;
}

uint8_t DS18B20_SetAlarmLowTemperature(OneWire_t* ds, uint8_t *ROM, int8_t temp)
{
    uint8_t tl, th, conf;

    if (!DS18B20_Is(ROM)) return 0;

    if (temp > 125) temp = 125;
    if (temp < -55) temp = -55;

    /* Reset line */
    OW_Reset(ds);
    /* Select ROM number */
    OW_SelectWithPointer(ds, ROM);
    /* Read scratchpad command by onewire protocol */
    OW_WriteByte(ds, ONEWIRE_CMD_RSCRATCHPAD);

    /* Ignore first 2 bytes */
    OW_ReadByte(ds);
    OW_ReadByte(ds);

    th = OW_ReadByte(ds);
    tl = OW_ReadByte(ds);
    conf = OW_ReadByte(ds);

    tl = (uint8_t)temp;

    /* Reset line */
    OW_Reset(ds);
    /* Select ROM number */
    OW_SelectWithPointer(ds, ROM);
    /* Write scratchpad command by onewire protocol, only th, tl and conf register can be written */
    OW_WriteByte(ds, ONEWIRE_CMD_WSCRATCHPAD);

    /* Write bytes */
    OW_WriteByte(ds, th);
    OW_WriteByte(ds, tl);
    OW_WriteByte(ds, conf);

    /* Reset line */
    OW_Reset(ds);
    /* Select ROM number */
    OW_SelectWithPointer(ds, ROM);
    /* Copy scratchpad to EEPROM of DS18B20 */
    OW_WriteByte(ds, ONEWIRE_CMD_CPYSCRATCHPAD);

    return 1;
}

uint8_t DS18B20_SetAlarmHighTemperature(OneWire_t* ds, uint8_t *ROM, int8_t temp)
{
    uint8_t tl, th, conf;

    if (!DS18B20_Is(ROM)) return 0;

    if (temp > 125) temp = 125;
    if (temp < -55) temp = -55;

    /* Reset line */
    OW_Reset(ds);
    /* Select ROM number */
    OW_SelectWithPointer(ds, ROM);
    /* Read scratchpad command by onewire protocol */
    OW_WriteByte(ds, ONEWIRE_CMD_RSCRATCHPAD);

    /* Ignore first 2 bytes */
    OW_ReadByte(ds);
    OW_ReadByte(ds);

    th = OW_ReadByte(ds);
    tl = OW_ReadByte(ds);
    conf = OW_ReadByte(ds);

    th = (uint8_t)temp;

    /* Reset line */
    OW_Reset(ds);
    /* Select ROM number */
    OW_SelectWithPointer(ds, ROM);
    /* Write scratchpad command by onewire protocol, only th, tl and conf register can be written */
    OW_WriteByte(ds, ONEWIRE_CMD_WSCRATCHPAD);

    /* Write bytes */
    OW_WriteByte(ds, th);
    OW_WriteByte(ds, tl);
    OW_WriteByte(ds, conf);

    /* Reset line */
    OW_Reset(ds);
    /* Select ROM number */
    OW_SelectWithPointer(ds, ROM);
    /* Copy scratchpad to EEPROM of DS18B20 */
    OW_WriteByte(ds, ONEWIRE_CMD_CPYSCRATCHPAD);

    return 1;
}

uint8_t DS18B20_DisableAlarmTemperature(OneWire_t* ds, uint8_t *ROM)
{
    uint8_t tl, th, conf;

    if (!DS18B20_Is(ROM)) return 0;

    /* Reset line */
    OW_Reset(ds);
    /* Select ROM number */
    OW_SelectWithPointer(ds, ROM);
    /* Read scratchpad command by onewire protocol */
    OW_WriteByte(ds, ONEWIRE_CMD_RSCRATCHPAD);

    /* Ignore first 2 bytes */
    OW_ReadByte(ds);
    OW_ReadByte(ds);

    th = OW_ReadByte(ds);
    tl = OW_ReadByte(ds);
    conf = OW_ReadByte(ds);

    th = 125;
    tl = (uint8_t)-55;

    /* Reset line */
    OW_Reset(ds);
    /* Select ROM number */
    OW_SelectWithPointer(ds, ROM);
    /* Write scratchpad command by onewire protocol, only th, tl and conf register can be written */
    OW_WriteByte(ds, ONEWIRE_CMD_WSCRATCHPAD);

    /* Write bytes */
    OW_WriteByte(ds, th);
    OW_WriteByte(ds, tl);
    OW_WriteByte(ds, conf);

    /* Reset line */
    OW_Reset(ds);
    /* Select ROM number */
    OW_SelectWithPointer(ds, ROM);
    /* Copy scratchpad to EEPROM of DS18B20 */
    OW_WriteByte(ds, ONEWIRE_CMD_CPYSCRATCHPAD);

    return 1;
}

uint8_t DS18B20_AlarmSearch(OneWire_t* ds)
{
    /* Start alarm search */
    return OW_Search(ds, DS18B20_CMD_ALARMSEARCH);
}

uint8_t DS18B20_AllDone(OneWire_t* ds)
{
    /* If read bit is low, then device is not finished yet with calculation temperature */
    return OW_ReadBit(ds);
}


