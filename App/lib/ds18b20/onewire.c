/**
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#include "onewire.h"
#include "ds18b20Config.h"


/* Pin settings */
static inline void OW_PinLow(OneWire_t *gp);
static inline void OW_PinHigh(OneWire_t *gp);
static inline void OW_PinInput(OneWire_t *gp);
static inline void OW_PinOutput(OneWire_t *gp);



static inline __attribute__((always_inline)) void OW_PinLow(OneWire_t* ow)
{
    LL_GPIO_ResetOutputPin(ow->GPIOx, ow->GPIO_Pin);
}


static inline __attribute__((always_inline)) void OW_PinHigh(OneWire_t* ow)
{
    LL_GPIO_SetOutputPin(ow->GPIOx, ow->GPIO_Pin);
}


static inline __attribute__((always_inline)) void OW_PinInput(OneWire_t* ow)
{
    LL_GPIO_SetPinMode(ow->GPIOx, ow->GPIO_Pin, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(ow->GPIOx, ow->GPIO_Pin, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinSpeed(ow->GPIOx, ow->GPIO_Pin, LL_GPIO_SPEED_FREQ_HIGH);
}


static inline __attribute__((always_inline)) void OW_PinOutput(OneWire_t* ow)
{
    LL_GPIO_SetPinMode(ow->GPIOx, ow->GPIO_Pin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(ow->GPIOx, ow->GPIO_Pin, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(ow->GPIOx, ow->GPIO_Pin, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinSpeed(ow->GPIOx, ow->GPIO_Pin, LL_GPIO_SPEED_FREQ_HIGH);
}


void OW_Init(OneWire_t* ow, GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin)
{
    ow->GPIOx = GPIOx;
    ow->GPIO_Pin = GPIO_Pin;

    OW_PinOutput(ow);

    OW_PinHigh(ow);
}

inline uint8_t OW_Reset(OneWire_t* ow)
{
    OW_PinLow(ow);
    Delay_us(480);
    OW_PinHigh(ow);

    Delay_us(120);

    uint8_t i = LL_GPIO_IsInputPinSet(ow->GPIOx, ow->GPIO_Pin);

    Delay_us(360);

    return i;
}

inline __attribute__((always_inline)) void OW_WriteBit(OneWire_t* ow, uint8_t bit)
{
    taskENTER_CRITICAL();//__disable_irq();

    OW_PinLow(ow);
    Delay_us(bit ? 1 : 80);

    OW_PinHigh(ow);
    Delay_us(bit ? 80 : 2);

    taskEXIT_CRITICAL();//__enable_irq();
}

inline __attribute__((always_inline)) uint8_t OW_ReadBit(OneWire_t* ow)
{
    taskENTER_CRITICAL();//__disable_irq();

    OW_PinLow(ow);
    Delay_us(1);
    OW_PinHigh(ow);

    Delay_us(15);

    uint8_t bit = (LL_GPIO_IsInputPinSet(ow->GPIOx, ow->GPIO_Pin));

    Delay_us(40);

    taskEXIT_CRITICAL();//__enable_irq();

    return bit;
}

void OW_WriteByte(OneWire_t* ow, uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++) {
        OW_WriteBit(ow, (byte >> i)&0x01);
    }
}


uint8_t OW_ReadByte(OneWire_t* ow)
{
    uint8_t byte = 0;

    for (uint8_t i = 0; i <= 7; i++){
        byte += OW_ReadBit(ow) << i;
    }

    return byte;
}

uint8_t OW_First(OneWire_t* ow)
{
    /* Reset search values */
    ow->LastDiscrepancy = 0;
    ow->LastDeviceFlag = 0;
    ow->LastFamilyDiscrepancy = 0;

    /* Start with searching */
    return OW_Search(ow, ONEWIRE_CMD_SEARCHROM);
}

uint8_t OW_Next(OneWire_t* ow)
{
    /* Leave the search state alone */
    return OW_Search(ow, ONEWIRE_CMD_SEARCHROM);
}

uint8_t OW_Search(OneWire_t* ow, uint8_t command)
{
    uint8_t id_bit_number;
    uint8_t last_zero, rom_byte_number, search_result;
    uint8_t id_bit, cmp_id_bit;
    uint8_t rom_byte_mask, search_direction;

    /* Initialize for search */
    id_bit_number = 1;
    last_zero = 0;
    rom_byte_number = 0;
    rom_byte_mask = 1;
    search_result = 0;

    // if the last call was not the last one
    if (!ow->LastDeviceFlag)
    {
        // 1-Wire reset
        if (OW_Reset(ow))
        {
            /* Reset the search */
            ow->LastDiscrepancy = 0;
            ow->LastDeviceFlag = 0;
            ow->LastFamilyDiscrepancy = 0;
            return 0;
        }

        // issue the search command
        OW_WriteByte(ow, command);

        do
        {
            // read a bit and its complement
            id_bit = OW_ReadBit(ow);
            cmp_id_bit = OW_ReadBit(ow);

            // check for no devices on 1-wire
            if ((id_bit == 1) && (cmp_id_bit == 1))
            {
                break;
            }
            else
            {
                // all devices coupled have 0 or 1
                if (id_bit != cmp_id_bit)
                {
                    search_direction = id_bit;  // bit write value for search
                }
                else
                {
                    // if this discrepancy if before the Last Discrepancy
                    // on a previous next then pick the same as last time
                    if (id_bit_number < ow->LastDiscrepancy)
                    {
                        search_direction = ((ow->ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
                    }
                    else
                    {
                        // if equal to last pick 1, if not then pick 0
                        search_direction = (id_bit_number == ow->LastDiscrepancy);
                    }

                    // if 0 was picked then record its position in LastZero
                    if (search_direction == 0)
                    {
                        last_zero = id_bit_number;

                        // check for Last discrepancy in family
                        if (last_zero < 9)
                        {
                            ow->LastFamilyDiscrepancy = last_zero;
                        }
                    }
                }

                // set or clear the bit in the ROM byte rom_byte_number
                // with mask rom_byte_mask
                if (search_direction == 1)
                {
                    ow->ROM_NO[rom_byte_number] |= rom_byte_mask;
                }
                else
                {
                    ow->ROM_NO[rom_byte_number] &= ~rom_byte_mask;
                }

                // serial number search direction write bit
                OW_WriteBit(ow, search_direction);

                // increment the byte counter id_bit_number
                // and shift the mask rom_byte_mask
                id_bit_number++;
                rom_byte_mask <<= 1;

                // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
                if (rom_byte_mask == 0)
                {
                    //docrc8(ROM_NO[rom_byte_number]);  // accumulate the CRC
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        }
        while (rom_byte_number < 8);    // loop until through all ROM bytes 0-7

        // if the search was successful then
        if (!(id_bit_number < 65))
        {
            // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
            ow->LastDiscrepancy = last_zero;

            // check for last device
            if (ow->LastDiscrepancy == 0)
            {
                ow->LastDeviceFlag = 1;
            }

            search_result = 1;
        }
    }

    // if no device found then reset counters so next 'search' will be like a first
    if (!search_result || !ow->ROM_NO[0])
    {
        ow->LastDiscrepancy = 0;
        ow->LastDeviceFlag = 0;
        ow->LastFamilyDiscrepancy = 0;
        search_result = 0;
    }

    return search_result;
}

int OW_Verify(OneWire_t* ow)
{
    unsigned char rom_backup[8];
    int rslt, ld_backup, ldf_backup, lfd_backup;

    // keep a backup copy of the current state
    for (uint8_t i = 0; i < 8; i++)
        rom_backup[i] = ow->ROM_NO[i];

    ld_backup = ow->LastDiscrepancy;
    ldf_backup = ow->LastDeviceFlag;
    lfd_backup = ow->LastFamilyDiscrepancy;

    // set search to find the same device
    ow->LastDiscrepancy = 64;
    ow->LastDeviceFlag = 0;

    if (OW_Search(ow, ONEWIRE_CMD_SEARCHROM))
    {
        // check if same device found
        rslt = 1;

        for (uint8_t i = 0; i < 8; i++)
        {
            if (rom_backup[i] != ow->ROM_NO[i])
            {
                rslt = 1;
                break;
            }
        }
    }
    else
    {
        rslt = 0;
    }

    // restore the search state
    for (uint8_t i = 0; i < 8; i++)
    {
        ow->ROM_NO[i] = rom_backup[i];
    }

    ow->LastDiscrepancy = ld_backup;
    ow->LastDeviceFlag = ldf_backup;
    ow->LastFamilyDiscrepancy = lfd_backup;

    // return the result of the verify
    return rslt;
}

void OW_TargetSetup(OneWire_t* ow, uint8_t family_code)
{
    // set the search state to find SearchFamily type devices
    ow->ROM_NO[0] = family_code;

    for (uint8_t i = 1; i < 8; i++)
    {
        ow->ROM_NO[i] = 0;
    }

    ow->LastDiscrepancy = 64;
    ow->LastFamilyDiscrepancy = 0;
    ow->LastDeviceFlag = 0;
}

void OW_FamilySkipSetup(OneWire_t* ow)
{
    // set the Last discrepancy to last family discrepancy
    ow->LastDiscrepancy = ow->LastFamilyDiscrepancy;
    ow->LastFamilyDiscrepancy = 0;

    // check for end of list
    if (ow->LastDiscrepancy == 0)
    {
        ow->LastDeviceFlag = 1;
    }
}

uint8_t OW_GetROM(OneWire_t* ow, uint8_t index)
{
    return ow->ROM_NO[index];
}

void OW_Select(OneWire_t* ow, uint8_t* addr)
{
    OW_WriteByte(ow, ONEWIRE_CMD_MATCHROM);

    for (uint8_t i = 0; i < 8; i++)
    {
        OW_WriteByte(ow, *(addr + i));
    }
}

void OW_SelectWithPointer(OneWire_t* ow, uint8_t *ROM)
{
    OW_WriteByte(ow, ONEWIRE_CMD_MATCHROM);

    for (uint8_t i = 0; i < 8; i++)
    {
        OW_WriteByte(ow, *(ROM + i));
    }
}

void OW_GetFullROM(OneWire_t* ow, uint8_t *firstIndex)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        *(firstIndex + i) = ow->ROM_NO[i];
    }
}

uint8_t OW_CRC8(uint8_t *addr, uint8_t len)
{
    uint8_t crc = 0, inbyte, mix;

    while (len--)
    {
        inbyte = *addr++;
        for (uint8_t i = 8; i; i--)
        {
            mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix)
            {
                crc ^= 0x8C;
            }
            inbyte >>= 1;
        }
    }

    /* Return calculated CRC */
    return crc;
}

