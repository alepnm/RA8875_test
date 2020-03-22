/**
  ******************************************************************************
  * @file FSMC_LCD/inc/stm3210e_lcd.h
  * @author   MCD Application Team
  * @version  V2.0.0
  * @date     04/27/2009
  * @brief    This file contains all the functions prototypes for the
  *           stm3210e_lcd firmware driver.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM3210E_LCD_H
#define __STM3210E_LCD_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* LCD Registers */
#define R0             0x00         // Status Register (STSR)
#define R1             0x01         // Power and Display Control Register (PWRR)
#define R2             0x02         // Memory Read/Write Command (MRWC)
#define R3             0x03         // Pixel Clock Setting Register (PCSR)
#define R4             0x04         // Serial Flash/ROM Configuration Register (SROC)
#define R5             0x05         // Serial Flash/ROM CLK Setting Register (SFCLR)
#define R6             0x06         //
#define R7             0x07
#define R8             0x08
#define R9             0x09
#define R10            0x0A
#define R12            0x0C
#define R13            0x0D
#define R14            0x0E
#define R15            0x0F
#define R16            0x10         // System Configuration Register (SYSR)
#define R17            0x11
#define R18            0x12         // GPI
#define R19            0x13         // GPO
#define R20            0x14         // LCD Horizontal Display Width Register (HDWR)
#define R21            0x15         // Horizontal Non-Display Period Fine Tuning Option Register (HNDFTR)
#define R22            0x16         // LCD Horizontal Non-Display Period Register (HNDR)
#define R23            0x17         // HSYNC Start Position Register (HSTR)
#define R24            0x18         // HSYNC Pulse Width Register (HPWR)
#define R25            0x19         // LCD Vertical Display Height Register (VDHR0)
#define R26            0x1A         // LCD Vertical Display Height Register0 (VDHR1)
#define R27            0x1B         // LCD Vertical Non-Display Period Register (VNDR0)
#define R28            0x1C         // LCD Vertical Non-Display Period Register (VNDR1)
#define R29            0x1D         // VSYNC Start Position Register (VSTR0)
#define R30            0x1E         // VSYNC Start Position Register (VSTR1)
#define R31            0x1F         // VSYNC Pulse Width Register (VPWR)
#define R32            0x20         // Display Configuration Register (DPCR)
#define R33            0x21         // Font Control Register 0 (FNCR0)
#define R34            0x22         // Font Control Register1 (FNCR1)
#define R35            0x23         // CGRAM Select Register (CGSR)
#define R36            0x24         // Horizontal Scroll Offset Register 0 (HOFS0)
#define R37            0x25         // Horizontal Scroll Offset Register 1 (HOFS1)
#define R38            0x26         // Vertical Scroll Offset Register 0 (VOFS0)
#define R39            0x27         // Vertical Scroll Offset Register 1 (VOFS1)
#define R41            0x29         // Font Line Distance Setting Register (FLDR)
#define R42            0x2A         // Font Write Cursor Horizontal Position Register 0 (F_CURXL)
#define R43            0x2B         // Font Write Cursor Horizontal Position Register 1 (F_CURXH)
#define R44            0x2C         // Font Write Cursor Vertical Position Register 0 (F_CURYL)
#define R45            0x2D         // Font Write Cursor Vertical Position Register 1 (F_CURYH)
#define R46            0x2E         // Font Write Type Setting Register
#define R47            0x2F         // Serial Font ROM Setting

// Active Window & Scroll Window Setting Registers
#define R48            0x30         // Horizontal Start Point 0 of Active Window (HSAW0)
#define R49            0x31         // Horizontal Start Point 1 of Active Window (HSAW1)
#define R50            0x32         // Vertical Start Point 0 of Active Window (VSAW0)
#define R51            0x33         // Vertical Start Point 1 of Active Window (VSAW1)
#define R52            0x34         // Horizontal End Point 0 of Active Window (HEAW0)
#define R53            0x35         // Horizontal End Point 1 of Active Window (HEAW1)
#define R54            0x36         // Vertical End Point of Active Window 0 (VEAW0)
#define R55            0x37         // Vertical End Point of Active Window 1 (VEAW1)
#define R56            0x38         // Horizontal Start Point 0 of Scroll Window (HSSW0)
#define R57            0x39         // Horizontal Start Point 1 of Scroll Window (HSSW1)
#define R58            0x3A         // Vertical Start Point 0 of Scroll Window (VSSW0)
#define R59            0x3B         // Vertical Start Point 1 of Scroll Window (VSSW1)
#define R60            0x3C         // Horizontal End Point 0 of Scroll Window (HESW0)
#define R61            0x3D         // Horizontal End Point 1 of Scroll Window (HESW1)
#define R62            0x3E         // Vertical End Point 0 of Scroll Window (VESW0)
#define R63            0x3F         // Vertical End Point 1 of Scroll Window (VESW1)

// Cursor Setting Registers
#define R64            0x40         // Memory Write Control Register 0 (MWCR0)
#define R65            0x41         // Memory Write Control Register1 (MWCR1)
#define R66            0x42         //
#define R67            0x43
#define R68            0x44         // Blink Time Control Register (BTCR)
#define R69            0x45         // Memory Read Cursor Direction (MRCD)
#define R70            0x46         // Memory Write Cursor Horizontal Position Register 0 (CURH0)
#define R71            0x47         // Memory Write Cursor Horizontal Position Register 1 (CURH1)
#define R72            0x48         // Memory Write Cursor Vertical Position Register 0 (CURV0)
#define R73            0x49         // Memory Write Cursor Vertical Position Register 1 (CURV1)
#define R74            0x4A         // Memory Read Cursor Horizontal Position Register 0 (RCURH0)
#define R75            0x4B         // Memory Read Cursor Horizontal Position Register 1 (RCURH01)
#define R76            0x4C         // Memory Read Cursor Vertical Position Register 0 (RCURV0)
#define R77            0x4D         // Memory Read Cursor Vertical Position Register 1 (RCURV1)
#define R78            0x4E         // Font Write Cursor and Memory Write Cursor Horizontal Size Register (CURHS)
#define R79            0x4F         // Font Write Cursor Vertical Size Register (CURVS)

// Block Transfer Engine(BTE) Control Registers
#define R80            0x50         // BTE Function Control Register 0 (BECR0)
#define R81            0x51         // BTE Function Control Register1 (BECR1)
#define R82            0x52         // Layer Transparency Register0 (LTPR0)
#define R83            0x53         // Layer Transparency Register1 (LTPR1)
#define R84            0x54         // Horizontal Source Point 0 of BTE (HSBE0)
#define R85            0x55         // Horizontal Source Point 1 of BTE (HSBE1)
#define R86            0x56         // Vertical Source Point 0 of BTE (VSBE0)
#define R87            0x57         // Vertical Source Point 1 of BTE (VSBE1)
#define R88            0x58         // Horizontal Destination Point 0 of BTE (HDBE0)
#define R89            0x59         // Horizontal Destination Point 1 of BTE (HDBE1)
#define R90            0x5A         // Vertical Destination Point 0 of BTE (VDBE0)
#define R91            0x5B         // Vertical Destination Point 1 of BTE (VDBE1)
#define R92            0x5C         // BTE Width Register 0 (BEWR0)
#define R93            0x5D         // BTE Width Register 1 (BEWR1)
#define R94            0x5E         // BTE Height Register 0 (BEHR0)
#define R95            0x5F         // BTE Height Register 1 (BEHR1)
#define R96            0x60         // Background Color Register 0 (BGCR0)
#define R97            0x61         // Background Color Register 1 (BGCR1)
#define R98            0x62         // Background Color Register 2 (BGCR2)
#define R99            0x63         // Foreground Color Register 0 (FGCR0)
#define R100           0x64         // Foreground Color Register 1 (FGCR1)
#define R101           0x65         // Foreground Color Register 2 (FGCR2)
#define R102           0x66         // Pattern Set No for BTE (PTNO)
#define R103           0x67         // Background Color Register for Transparent 0 (BGTR0)
#define R104           0x68         // Background Color Register for Transparent 1 (BGTR1)
#define R105           0x69         // Background Color Register for Transparent 2 (BGTR2)

// Touch Panel Control Registers
#define R112           0x70         // Touch Panel Control Register 0 (TPCR0)
#define R113           0x71         // Touch Panel Control Register 1 (TPCR1)
#define R114           0x72         // Touch Panel X High Byte Data Register (TPXH)
#define R115           0x73         // Touch Panel Y High Byte Data Register (TPYH)
#define R116           0x74         // Touch Panel X/Y Low Byte Data Register (TPXYL)

// Graphic Cursor Setting Registers
#define R128           0x80         // Graphic Cursor Horizontal Position Register 0 (GCHP0)
#define R129           0x81         // Graphic Cursor Horizontal Position Register 1 (GCHP1)
#define R130           0x82         // Graphic Cursor Vertical Position Register 0 (GCVP0)
#define R131           0x83         // Graphic Cursor Vertical Position Register 1 (GCVP1)
#define R132           0x84         // Graphic Cursor Color 0 (GCC0)
#define R133           0x85         // Graphic Cursor Color 1 (GCC1)
#define R134           0x86         //
#define R135           0x87

// PLL Setting Registers
#define R136           0x88         // PLL Control Register 1 (PLLC1)
#define R137           0x89         // PLL Control Register 2 (PLLC2)

// PWM Control Registers
#define R138           0x8A         // PWM1 Control Register (P1CR)
#define R139           0x8B         // PWM1 Duty Cycle Register (P1DCR)
#define R140           0x8C         // PWM2 Control Register (P2CR)
#define R141           0x8D         // PWM2 Control Register (P2DCR)
#define R142           0x8E         // Memory Clear Control Register (MCLR)


// Drawing Control Registers
#define R144           0x90         // Draw Line/Circle/Square Control Register (DCR)
#define R145           0x91         // Draw Line/Square Horizontal Start Address Register0 (DLHSR0)
#define R146           0x92         // Draw Line/Square Horizontal Start Address Register1 (DLHSR1)
#define R147           0x93         // Draw Line/Square Vertical Start Address Register0 (DLVSR0)
#define R148           0x94         // Draw Line/Square Vertical Start Address Register1 (DLVSR1)
#define R149           0x95         // Draw Line/Square Horizontal End Address Register0 (DLHER0)
#define R150           0x96         // Draw Line/Square Horizontal End Address Register1 (DLHER1)
#define R151           0x97         // Draw Line/Square Vertical End Address Register0 (DLVER0)
#define R152           0x98         // Draw Line/Square Vertical End Address Register1 (DLVER1)
#define R153           0x99         // Draw Circle Center Horizontal Address Register0 (DCHR0)
#define R154           0x9A         // Draw Circle Center Horizontal Address Register1 (DCHR1)
#define R155           0x9B         // Draw Circle Center Vertical Address Register0 (DCVR0)
#define R156           0x9C         // Draw Circle Center Vertical Address Register1 (DCVR1)
#define R157           0x9D         // Draw Circle Radius Register (DCRR)
#define R160           0xA0         // Draw Ellipse/Ellipse Curve/Circle Square Control Register
#define R161           0xA1         // Draw Ellipse/Circle Square Long axis Setting Register (ELL_A0)
#define R162           0xA2         // Draw Ellipse/Circle Square Long axis Setting Register (ELL_A1)
#define R163           0xA3         // Draw Ellipse/Circle Square Short axis Setting Register (ELL_B0)
#define R164           0xA4         // Draw Ellipse/Circle Square Short axis Setting Register (ELL_B1)
#define R165           0xA5         // Draw Ellipse/Circle Square Center Horizontal Address Register0 (DEHR0)
#define R166           0xA6         // Draw Ellipse/Circle Square Center Horizontal Address Register1 (DEHR1)
#define R167           0xA7         // Draw Ellipse/Circle Square Center Vertical Address Register0 (DEVR0)
#define R168           0xA8         // Draw Ellipse/Circle Square Center Vertical Address Register1 (DEVR1)
#define R169           0xA9         // Draw Triangle Point 2 Horizontal Address Register0 (DTPH0)
#define R170           0xAA         // Draw Triangle Point 2 Horizontal Address Register1 (DTPH1)
#define R171           0xAB         // Draw Triangle Point 2 Vertical Address Register0 (DTPV0)
#define R172           0xAC         // Draw Triangle Point 2 Vertical Address Register1 (DTPV1)
#define R173           0xAD         //


// DMA Registers
#define R176           0xB0         // Source Starting Address REG0 (SSAR0)
#define R177           0xB1         // Source Starting Address REG 1 (SSAR1)
#define R178           0xB2         // Source Starting Address REG 2 (SSAR2)
#define R179           0xB4         // Block Width REG 0(BWR0) / DMA Transfer Number REG 0 (DTNR0)
#define R180           0xB5         // Block Width REG 1 (BWR1)
#define R181           0xB6         // Block Height REG 0(BHR0) /DMA Transfer Number REG 1 (DTNR1)
#define R182           0xB7         // Block Height REG 1 (BHR1)
#define R183           0xB8         // Source Picture Width REG 0(SPWR0) / DMA Transfer Number REG 2(DTNR2)
#define R184           0xB9         // Source Picture Width REG 1 (SPWR1)
#define R191           0xBF         // DMA Configuration REG (DMACR)
#define R192           0xC0         // Key-Scan Control Register 1 (KSCR1)
#define R193           0xC1         // Key-Scan Controller Register 2 (KSCR2)
#define R194           0xC2         // Key-Scan Data Register (KSDR0)
#define R195           0xC3         // Key-Scan Data Register (KSDR1)
#define R196           0xC4         // Key-Scan Data Register (KSDR2)
#define R199           0xC7         // Extra General Purpose IO Register (GPIOX)


// Floating Window Control Registers
#define R208           0xD0         // Floating Windows Start Address XA 0 (FWSAXA0)
#define R209           0xD1         // Floating Windows Start Address XA 1 (FWSAXA1)
#define R210           0xD2         // Floating Windows Start Address YA 0 (FWSAYA0)
#define R211           0xD3         // Floating Windows Start Address YA 1 (FWSAYA1)
#define R212           0xD4         // Floating Windows Width 0 (FWW0)
#define R213           0xD5         // Floating Windows Width 1 (FWW1)
#define R214           0xD6         // Floating Windows Height 0 (FWH0)
#define R215           0xD7         // Floating Windows Height 1 (FWH1)
#define R216           0xD8         // Floating Windows Display X Address 0 (FWDXA0)
#define R217           0xD9         // Floating Windows Display X Address 1 (FWDXA1)
#define R218           0xDA         // Floating Windows Display Y Address 0 (FWDYA0)
#define R219           0xDB         // Floating Windows Display Y Address 1 (FWDYA1)


// Serial Flash Control Registers
#define R224           0xE0         // Serial Flash/ROM Direct Access Mode
#define R225           0xE1         // Serial Flash/ROM Direct Access Mode Address
#define R226           0xE2         // Serial Flash/ROM Direct Access Data Read



// Interrupt Control Registers
#define R240           0xF0         // Interrupt Control Register1 (INTC1)
#define R241           0xF1         // Interrupt Control Register2 (INTC2)


/* LCD color */
#define WHITE          0xFFFF
#define BLACK          0x0000
#define GRAY           0xF7DE
#define NAVY           0x001F
#define BLUE           0x051F
#define RED            0xF800
#define MAGENTA        0xF81F
#define GREEN          0x07E0
#define CYAN           0x7FFF
#define YELLOW         0xFFE0

#define Line0          0
#define Line1          24
#define Line2          48
#define Line3          72
#define Line4          96
#define Line5          120
#define Line6          144
#define Line7          168
#define Line8          192
#define Line9          216

#define Horizontal     0x00
#define Vertical       0x01

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/*----- High layer function -----*/
void STM3210E_LCD_Init(void);
void LCD_SetTextColor(__IO uint16_t Color);
void LCD_SetBackColor(__IO uint16_t Color);
void LCD_ClearLine(uint8_t Line);
void LCD_Clear(uint16_t Color);
void LCD_SetCursor(uint8_t Xpos, uint16_t Ypos);
void LCD_DrawChar(uint8_t Xpos, uint16_t Ypos, const uint16_t *c);
void LCD_DisplayChar(uint8_t Line, uint16_t Column, uint8_t Ascii);
void LCD_DisplayStringLine(uint8_t Line, uint8_t *ptr);
void LCD_SetDisplayWindow(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width);
void LCD_WindowModeDisable(void);
void LCD_DrawLine(uint8_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction);
void LCD_DrawRect(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width);
void LCD_DrawCircle(uint8_t Xpos, uint16_t Ypos, uint16_t Radius);
void LCD_DrawMonoPict(const uint32_t *Pict);
void LCD_WriteBMP(uint32_t BmpAddress);

/*----- Medium layer function -----*/
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue);
uint16_t LCD_ReadReg(uint8_t LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(uint16_t RGB_Code);
uint16_t LCD_ReadRAM(void);
void LCD_PowerOn(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);

/*----- Low layer function -----*/
void LCD_CtrlLinesConfig(void);
void LCD_FSMCConfig(void);
void LCD_Delay(uint32_t delay);

#endif /* __STM3210E_LCD_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
