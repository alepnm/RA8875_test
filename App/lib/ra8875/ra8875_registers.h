#ifndef RA8875_REGISTERS_H_INCLUDED
#define RA8875_REGISTERS_H_INCLUDED


/* Status Register */
#define RA8875_REG_STSR

/* System & Configuration Registers */
#define RA8875_REG_PWRR    0x01   // Power and Display Control Register
#define RA8875_REG_MRWC    0x02   // Memory Read/Write Command
#define RA8875_REG_PCSR    0x04   // Pixel Clock Setting Register
#define RA8875_REG_SROC    0x05   // Serial Flash/ROM Configuration Register
#define RA8875_REG_SFCLR   0x06   // Serial Flash/ROM CLK Setting Register
#define RA8875_REG_SYSR    0x10   // System Configuration Register
#define RA8875_REG_GPI     0x12   //
#define RA8875_REG_GPO     0x13
#define RA8875_REG_HDWR    0x14   // LCD Horizontal Display Width Register
#define RA8875_REG_HNDFTR  0x15   // Horizontal Non-Display Period Fine Tuning Option Register
#define RA8875_REG_HNDR    0x16   // LCD Horizontal Non-Display Period Register
#define RA8875_REG_HSTR    0x17   // HSYNC Start Position Register
#define RA8875_REG_HPWR    0x18   // HSYNC Pulse Width Register
#define RA8875_REG_VDHR0   0x19   // LCD Vertical Display Height Register
#define RA8875_REG_VDHR1   0x1A   // LCD Vertical Display Height Register0
#define RA8875_REG_VNDR0   0x1B   // LCD Vertical Non-Display Period Register
#define RA8875_REG_VNDR1   0x1C   // LCD Vertical Non-Display Period Register
#define RA8875_REG_VSTR0   0x1D   // VSYNC Start Position Register
#define RA8875_REG_VSTR1   0x1E   // VSYNC Start Position Register
#define RA8875_REG_VPWR    0x1F   // VSYNC Pulse Width Register

/* LCD Display Control Registers */
#define RA8875_REG_DPCR    0x20   // Display Configuration Register
#define RA8875_REG_FNCR0   0x21   // Font Control Register 0
#define RA8875_REG_FNCR1   0x22   // Font Control Register 1
#define RA8875_REG_CGSR    0x23   // CGRAM Select Register
#define RA8875_REG_HOFS0   0x24   // Horizontal Scroll Offset Register 0
#define RA8875_REG_HOFS1   0x25   // Horizontal Scroll Offset Register 1
#define RA8875_REG_VOFS0   0x26   // Vertical Scroll Offset Register 0
#define RA8875_REG_VOFS1   0x27   // Vertical Scroll Offset Register 1
#define RA8875_REG_FLDR    0x29   // Font Line Distance Setting Register
#define RA8875_REG_FCURXL  0x2A   // Font Write Cursor Horizontal Position Register 0
#define RA8875_REG_FCURXH  0x2B   // Font Write Cursor Horizontal Position Register 1
#define RA8875_REG_FCURYL  0x2C   // Font Write Cursor Vertical Position Register 0
#define RA8875_REG_FCURYH  0x2D   // Font Write Cursor Vertical Position Register 1
#define RA8875_REG_FTSR    0x2E   // Font Write Type Setting Register
#define RA8875_REG_FROM    0x2F   // Serial Font ROM Setting

/* Active Window & Scroll Window Setting Registers */
#define RA8875_REG_HSAW0   0x30   // Horizontal Start Point 0 of Active Window
#define RA8875_REG_HSAW1   0x31   // Horizontal Start Point 1 of Active Window
#define RA8875_REG_VSAW0   0x32   // Vertical Start Point 0 of Active Window
#define RA8875_REG_VSAW1   0x33   // Vertical Start Point 1 of Active Window
#define RA8875_REG_HEAW0   0x34   // Horizontal End Point 0 of Active Window
#define RA8875_REG_HEAW1   0x35   // Horizontal End Point 1 of Active Window
#define RA8875_REG_VEAW0   0x36   // Vertical End Point of Active Window 0
#define RA8875_REG_VEAW1   0x37   // Vertical End Point of Active Window 1
#define RA8875_REG_HSSW0   0x38   // Horizontal Start Point 0 of Scroll Window
#define RA8875_REG_HSSW1   0x39   // Horizontal Start Point 1 of Scroll Window
#define RA8875_REG_VSSW0   0x3A   // Vertical Start Point 0 of Scroll Window
#define RA8875_REG_VSSW1   0x3B   // Vertical Start Point 1 of Scroll Window
#define RA8875_REG_HESW0   0x3C   // Horizontal End Point 0 of Scroll Window
#define RA8875_REG_HESW1   0x3D   // Horizontal End Point 1 of Scroll Window
#define RA8875_REG_VESW0   0x3E   // Vertical End Point 0 of Scroll Window
#define RA8875_REG_VESW1   0x3F   // Vertical End Point 1 of Scroll Window

/* Cursor Setting Registers */
#define RA8875_REG_MWCR0   0x40   // Memory Write Control Register 0
#define RA8875_REG_MWCR1   0x41   // Memory Write Control Register 1
#define RA8875_REG_BTCR    0x44   // Blink Time Control Register
#define RA8875_REG_MRCD    0x45   // Memory Read Cursor Direction
#define RA8875_REG_CURH0   0x46   // Memory Write Cursor Horizontal Position Register 0
#define RA8875_REG_CURH1   0x47   // Memory Write Cursor Horizontal Position Register 1
#define RA8875_REG_CURV0   0x48   // Memory Write Cursor Vertical Position Register 0
#define RA8875_REG_CURV1   0x49   // Memory Write Cursor Vertical Position Register 1
#define RA8875_REG_RCURH0  0x4A   // Memory Read Cursor Horizontal Position Register 0
#define RA8875_REG_RCURH1  0x4B   // Memory Read Cursor Horizontal Position Register 1
#define RA8875_REG_RCURV0  0x4C   // Memory Read Cursor Vertical Position Register 0
#define RA8875_REG_RCURV1  0x4D   // Memory Read Cursor Vertical Position Register 1
#define RA8875_REG_CURHS   0x4E   // Font Write Cursor and Memory Write Cursor Horizontal Size Register
#define RA8875_REG_CURVS   0x4F   // Font Write Cursor Vertical Size Register

/* Block Transfer Engine(BTE) Control Registers */
#define RA8875_REG_BECR0   0x50   // BTE Function Control Register 0
#define RA8875_REG_BECR1   0x51   // BTE Function Control Register 1
#define RA8875_REG_LTPR0   0x52   // Layer Transparency Register0
#define RA8875_REG_LTPR1   0x53   // Layer Transparency Register1
#define RA8875_REG_HSBE0   0x54   // Horizontal Source Point 0 of BTE
#define RA8875_REG_HSBE1   0x55   // Horizontal Source Point 1 of BTE
#define RA8875_REG_VSBE0   0x56   // Vertical Source Point 0 of BTE
#define RA8875_REG_VSBE1   0x57   // Vertical Source Point 1 of BTE
#define RA8875_REG_HDBE0   0x58   // Horizontal Destination Point 0 of BTE
#define RA8875_REG_HDBE1   0x59   // Horizontal Destination Point 1 of BTE
#define RA8875_REG_VDBE0   0x5A   // Vertical Destination Point 0 of BTE
#define RA8875_REG_VDBE1   0x5B   // Vertical Destination Point 1 of BTE
#define RA8875_REG_BEWR0   0x5C   // BTE Width Register 0
#define RA8875_REG_BEWR1   0x5D   // BTE Width Register 1
#define RA8875_REG_BEHR0   0x5E   // BTE Height Register 0
#define RA8875_REG_BEHR1   0x5F   // BTE Height Register 1
#define RA8875_REG_BGCR0   0x60   // Background Color Register 0
#define RA8875_REG_BGCR1   0x61   // Background Color Register 1
#define RA8875_REG_BGCR2   0x62   // Background Color Register 2
#define RA8875_REG_FGCR0   0x63   // Foreground Color Register 0
#define RA8875_REG_FGCR1   0x64   // Foreground Color Register 1
#define RA8875_REG_FGCR2   0x65   // Foreground Color Register 2
#define RA8875_REG_PTNO    0x66   // Pattern Set No for BTE
#define RA8875_REG_BGTR0   0x67   // Background Color Register for Transparent 0
#define RA8875_REG_BGTR1   0x68   // Background Color Register for Transparent 1
#define RA8875_REG_BGTR2   0x69   // Background Color Register for Transparent 2

/* Touch Panel Control Registers */
#define RA8875_REG_TPCR0   0x70   // Touch Panel Control Register 0
#define RA8875_REG_TPCR1   0x71   // Touch Panel Control Register 1
#define RA8875_REG_TPXH    0x72   // Touch Panel X High Byte Data Register
#define RA8875_REG_TPYH    0x73   // Touch Panel Y High Byte Data Register
#define RA8875_REG_TPXYL   0x74   // Touch Panel X/Y Low Byte Data Register

/* Graphic Cursor Setting Registers */
#define RA8875_REG_GCHP0   0x80   // Graphic Cursor Horizontal Position Register 0
#define RA8875_REG_GCHP1   0x81   // Graphic Cursor Horizontal Position Register 1
#define RA8875_REG_GCVP0   0x82   // Graphic Cursor Vertical Position Register 0
#define RA8875_REG_GCVP1   0x83   // Graphic Cursor Vertical Position Register 1
#define RA8875_REG_GCC0    0x84   // Graphic Cursor Color 0
#define RA8875_REG_GCC1    0x85   // Graphic Cursor Color 1

/* PLL Setting Registers */
#define RA8875_REG_PLLC1   0x88   // PLL Control Register 1
#define RA8875_REG_PLLC2   0x89   // PLL Control Register 2

/* PWM Control Registers */
#define RA8875_REG_P1CR    0x8A   // PWM1 Control Register
#define RA8875_REG_P1DCR   0x8B   // PWM1 Duty Cycle Register
#define RA8875_REG_P2CR    0x8C   // PWM2 Control Register
#define RA8875_REG_P2DCR   0x8D   // PWM2 Control Register
#define RA8875_REG_MCLR    0x8E   // Memory Clear Control Register

/* Drawing Control Registers */
#define RA8875_REG_DCR     0x90   // Draw Line/Circle/Square Control Register
#define RA8875_REG_DLHSR0  0x91   // Draw Line/Square Horizontal Start Address Register0
#define RA8875_REG_DLHSR1  0x92   // Draw Line/Square Horizontal Start Address Register1
#define RA8875_REG_DLVSR0  0x93   // Draw Line/Square Vertical Start Address Register0
#define RA8875_REG_DLVSR1  0x94   // Draw Line/Square Vertical Start Address Register1
#define RA8875_REG_DLHER0  0x95   // Draw Line/Square Horizontal End Address Register0
#define RA8875_REG_DLHER1  0x96   // Draw Line/Square Horizontal End Address Register1
#define RA8875_REG_DLVER0  0x97   // Draw Line/Square Vertical End Address Register0
#define RA8875_REG_DLVER1  0x98   // Draw Line/Square Vertical End Address Register1
#define RA8875_REG_DCHR0   0x99   // Draw Circle Center Horizontal Address Register0
#define RA8875_REG_DCHR1   0x9A   // Draw Circle Center Horizontal Address Register1
#define RA8875_REG_DCVR0   0x9B   // Draw Circle Center Vertical Address Register0
#define RA8875_REG_DCVR1   0x9C   // Draw Circle Center Vertical Address Register1
#define RA8875_REG_DCRR    0x9D   // Draw Circle Radius Register
#define RA8875_REG_DCR1    0xA0   // Draw Ellipse/Ellipse Curve/Circle Square Control Register
#define RA8875_REG_ELLA0   0xA1   // Draw Ellipse/Circle Square Long axis Setting Register
#define RA8875_REG_ELLA1   0xA2   // Draw Ellipse/Circle Square Long axis Setting Register
#define RA8875_REG_ELLB0   0xA3   // Draw Ellipse/Circle Square Short axis Setting Register
#define RA8875_REG_ELLB1   0xA4   // Draw Ellipse/Circle Square Short axis Setting Register
#define RA8875_REG_DEHR0   0xA5   // Draw Ellipse/Circle Square Center Horizontal Address Register0
#define RA8875_REG_DEHR1   0xA6   // Draw Ellipse/Circle Square Center Horizontal Address Register1
#define RA8875_REG_DEVR0   0xA7   // Draw Ellipse/Circle Square Center Vertical Address Register0
#define RA8875_REG_DEVR1   0xA8   // Draw Ellipse/Circle Square Center Vertical Address Register1
#define RA8875_REG_DTPH0   0xA9   // Draw Triangle Point 2 Horizontal Address Register0
#define RA8875_REG_DTPH1   0xAA   // Draw Triangle Point 2 Horizontal Address Register1
#define RA8875_REG_DTPV0   0xAB   // Draw Triangle Point 2 Vertical Address Register0
#define RA8875_REG_DTPV1   0xAC   // Draw Triangle Point 2 Vertical Address Register1

/* DMA Registers */
#define RA8875_REG_SSAR0   0xB0   // Source Starting Address REG 0
#define RA8875_REG_SSAR1   0xB1   // Source Starting Address REG 1
#define RA8875_REG_SSAR2   0xB2   // Source Starting Address REG 2
#define RA8875_REG_BWR0    0xB4   // Block Width REG 0
#define RA8875_REG_DTNR0   0xB4   // DMA Transfer Number REG 0
#define RA8875_REG_BWR1    0xB5   // Block Width REG 1
#define RA8875_REG_BHR0    0xB6   // Block Height REG 0
#define RA8875_REG_DTNR1   0xB6   // DMA Transfer Number REG 1
#define RA8875_REG_BHR1    0xB7   // Block Height REG 1
#define RA8875_REG_SPWR0   0xB8   // Source Picture Width REG 0
#define RA8875_REG_DTNR2   0xB8   // DMA Transfer Number REG 2
#define RA8875_REG_SPWR1   0xB9   // Source Picture Width REG 1
#define RA8875_REG_DMACR   0xBF   // DMA Configuration REG

/* Key & IO Control Registers */
#define RA8875_REG_KSCR1   0xC0   // Key-Scan Control Register 1
#define RA8875_REG_KSCR2   0xC1   // Key-Scan Control Register 2
#define RA8875_REG_KSDR0   0xC2   // Key-Scan Data Register
#define RA8875_REG_KSDR1   0xC3   // Key-Scan Data Register
#define RA8875_REG_KSDR2   0xC4   // Key-Scan Data Register
#define RA8875_REG_GPIOX   0xC7   // Extra General Purpose IO Register

/* Floating Window Control Registers */
#define RA8875_REG_FWSAXA0 0xD0   // Floating Windows Start Address XA 0
#define RA8875_REG_FWSAXA1 0xD1   // Floating Windows Start Address XA 1
#define RA8875_REG_FWSAYA0 0xD2   // Floating Windows Start Address YA 0
#define RA8875_REG_FWSAYA1 0xD3   // Floating Windows Start Address YA 1
#define RA8875_REG_FWW0    0xD4   // Floating Windows Width 0
#define RA8875_REG_FWW1    0xD5   // Floating Windows Width 1
#define RA8875_REG_FWH0    0xD6   // Floating Windows Height 0
#define RA8875_REG_FWH1    0xD7   // Floating Windows Height 1
#define RA8875_REG_FWDXA0  0xD8   // Floating Windows Display X Address 0
#define RA8875_REG_FWDXA1  0xD9   // Floating Windows Display X Address 1
#define RA8875_REG_FWDYA0  0xDA   // Floating Windows Display Y Address 0
#define RA8875_REG_FWDYA1  0xDB   // Floating Windows Display Y Address 1

/* Serial Flash Control Registers */
#define RA8875_REG_SACS_MODE 0xE0   // Serial Flash/ROM Direct Access Mode
#define RA8875_REG_SACS_ADDR 0xE1   // Serial Flash/ROM Direct Access Mode Address
#define RA8875_REG_SACS_DATA 0xE2   // Serial Flash/ROM Direct Access Data Read

/* Interrupt Control Registers */
#define RA8875_REG_INTC1   0xF0   // Interrupt Control Register1
#define RA8875_REG_INTC2   0xF1   // Interrupt Control Register2




/*  Registru bitai  */

/* STATUS Register (STSR) */
#define RA8875_STSR_MEMORY_BUSY                   0x80
#define RA8875_STSR_BTE_BUSY                      0x40
#define RA8875_STSR_TOUCH_EVENT_DETECTED          0x20
#define RA8875_STSR_SLEEP_MODE_STATUS             0x10
#define RA8875_STSR_SERIAL_FLASH_ROM_BUSY         0x01

/* Power and Display Control Register (PWRR) REG[01h] */
#define RA8875_PWRR_LCD_DISPLAY_ON                0x80   // Display ON/OFF
#define RA8875_PWRR_SLEEP_MODE                    0x02   // Sleep Mode/Normal Mode
#define RA8875_PWRR_SOFTWARE_RESET                0x01   // SoftReset

/* Pixel Clock Setting Register (PCSR) REG[04h] */
#define RA8875_PCSR_DATA_FETCH_ON_RISING          0x00
#define RA8875_PCSR_DATA_FETCH_ON_FALLING         0x80   //Falling/Rising
#define RA8875_PCSR_PCLK_PERIOD_SYSCLK_DIV1       0x00
#define RA8875_PCSR_PCLK_PERIOD_SYSCLK_DIV2       0x01
#define RA8875_PCSR_PCLK_PERIOD_SYSCLK_DIV4       0x02
#define RA8875_PCSR_PCLK_PERIOD_SYSCLK_DIV8       0x03

/* Serial Flash/ROM Configuration Register (SROC) REG[05h] */
#define RA8875_SROC_EXTROM_SELECT_CS0             0x00
#define RA8875_SROC_EXTROM_SELECT_CS1             0x80
#define RA8875_SROC_ADRESS_MODE_24BIT             0x00
#define RA8875_SROC_WAVEFORM_MODE0                0x00
#define RA8875_SROC_WAVEFORM_MODE3                0x20
#define RA8875_SROC_ROM_READ_CYCLE_NO_DUMMY       0x00
#define RA8875_SROC_ROM_READ_CYCLE_ONE_BYTE       0x08
#define RA8875_SROC_ROM_READ_CYCLE_TWO_BYTE       0x10
#define RA8875_SROC_ROM_ACCESS_MODE_FONT          0x00
#define RA8875_SROC_ROM_ACCESS_MODE_DMA           0x04
#define RA8875_SROC_ROM_DATA_LATCH_MODE_SINGLE    0x00
#define RA8875_SROC_ROM_DATA_LATCH_MODE_DUAL0     0x02
#define RA8875_SROC_ROM_DATA_LATCH_MODE_DUAL1     0x03

/* Serial Flash/ROM CLK Setting Register (SFCLR) REG[06h] */
#define RA8875_SFCLR_SFCL_FREQ_SYSCLK_DIV1        0x00
#define RA8875_SFCLR_SFCL_FREQ_SYSCLK_DIV2        0x02
#define RA8875_SFCLR_SFCL_FREQ_SYSCLK_DIV4        0x03

/* System Configuration Register (SYSR) REG[10h] */
#define RA8875_SYSR_COLOR_DEPTH_256               0x00
#define RA8875_SYSR_COLOR_DEPTH_16K               (2<<2)
#define RA8875_SYSR_MCU_INTERFACE_8BIT            0x00
#define RA8875_SYSR_MCU_INTERFACE_16BIT           0x02


/* xSYNC Pulse Width Register (xPWR) REG[18h] ir REG[1Fh] */
#define RA8875_xPWR_SYNC_POLARITY_LOW             0x00
#define RA8875_xPWR_SYNC_POLARITY_HIGH            0x80


/* Display Configuration Register (DPCR) REG[20h] */
#define RA8875_DPCR_ONE_LAYER                     0x00
#define RA8875_DPCR_TWO_LAYER                     0x80
#define RA8875_DPCR_HSCAN_DIR_RIGHT               0x00
#define RA8875_DPCR_HSCAN_DIR_LEFT                0x08
#define RA8875_DPCR_VSCAN_DIR_RIGHT               0x00
#define RA8875_DPCR_VSCAN_DIR_LEFT                0x04


/* Font Control Register 0 (FNCR0) REG[21h] */
#define RA8875_FNCR0_FONT_CGROM                   0x00
#define RA8875_FNCR0_FONT_CGRAM                   0x80
#define RA8875_FNCR0_FONT_INTERNAL_CGROM          0x00
#define RA8875_FNCR0_FONT_EXTERNAL_CGROM          0x20
#define RA8875_FNCR0_FONT_IEC8859_1               0x00
#define RA8875_FNCR0_FONT_IEC8859_2               0x01
#define RA8875_FNCR0_FONT_IEC8859_3               0x02
#define RA8875_FNCR0_FONT_IEC8859_4               0x03

/* Font Control Register 1 (FNCR1) REG[22h] */
#define RA8875_FNCR1_ALIGMENT_DISABLE             0x00
#define RA8875_FNCR1_ALIGMENT_ENABLE              0x80
#define RA8875_FNCR1_BG_TRANSPARENCY_OFF          0x00
#define RA8875_FNCR1_BG_TRANSPARENCY_ON           0x40
#define RA8875_FNCR1_FONT_ROTATION_DEG0           0x00
#define RA8875_FNCR1_FONT_ROTATION_DEG90          0x10
#define RA8875_FNCR1_FONT_HENLARGEMENT_X1         0x00
#define RA8875_FNCR1_FONT_HENLARGEMENT_X2         (1<<2)
#define RA8875_FNCR1_FONT_HENLARGEMENT_X3         (2<<2)
#define RA8875_FNCR1_FONT_HENLARGEMENT_X4         (3<<2)
#define RA8875_FNCR1_FONT_VENLARGEMENT_X1         0x00
#define RA8875_FNCR1_FONT_VENLARGEMENT_X2         0x01
#define RA8875_FNCR1_FONT_VENLARGEMENT_X3         0x02
#define RA8875_FNCR1_FONT_VENLARGEMENT_X4         0x03

/* Font Write Type Setting Register REG[2Eh] */
#define RA8875_FWTSR_FONT_SIZE_16X16              0x00
#define RA8875_FWTSR_FONT_SIZE_24X24              0x40
#define RA8875_FWTSR_FONT_SIZE_32X32              0x80
#define RA8875_FWTSR_FONT_TO_FONT_WIDTH_MSK       0x3F

/* Serial Font ROM Setting REG[2Fh] */
#define RA8875_SFROM_ROM_SELECT_TYPE1             0x00
#define RA8875_SFROM_ROM_SELECT_TYPE2             (1<<5)
#define RA8875_SFROM_ROM_SELECT_TYPE3             (2<<5)
#define RA8875_SFROM_ROM_SELECT_TYPE4             (3<<5)
#define RA8875_SFROM_ROM_SELECT_TYPE5             (4<<5)
#define RA8875_SFROM_FONT_CODING_GB2312           0x00
#define RA8875_SFROM_FONT_CODING_GB12345          (1<<2)
#define RA8875_SFROM_FONT_CODING_BIG5             (2<<2)
#define RA8875_SFROM_FONT_CODING_UNICODE          (3<<2)
#define RA8875_SFROM_FONT_CODING_ASCII            (4<<2)
#define RA8875_SFROM_FONT_CODING_UNI_JAP          (5<<2)
#define RA8875_SFROM_FONT_CODING_JIS0208          (6<<2)
#define RA8875_SFROM_FONT_CODING_OTHER            (7<<2)
#define RA8875_SFROM_FONT_TYPE_NORMAL             0x00
#define RA8875_SFROM_FONT_TYPE_ARIAL              0x01
#define RA8875_SFROM_FONT_TYPE_ROMAN              0x02
#define RA8875_SFROM_FONT_TYPE_BOLD               0x03


/* Memory Write Control Register 0 (MWCR0) REG[40h] */
#define RA8875_MWCR0_GRAPHIC_MODE                 0x00
#define RA8875_MWCR0_TEXT_MODE                    0x80
#define RA8875_MWCR0_CURSOR_NOT_VISIBLE           0x00
#define RA8875_MWCR0_CURSOR_VISIBLE               0x40
#define RA8875_MWCR0_CURSOR_BLINK_DISABLE         0x00
#define RA8875_MWCR0_CURSOR_BLINK_ENABLE          0x20
#define RA8875_MWCR0_MEM_WRITE_DIR_LR_TD          0x00
#define RA8875_MWCR0_MEM_WRITE_DIR_RL_TD          (1<<2)
#define RA8875_MWCR0_MEM_WRITE_DIR_TD_LR          (2<<2)
#define RA8875_MWCR0_MEM_WRITE_DIR_DT_LR          (3<<2)
#define RA8875_MWCR0_MEM_WRITE_AUTOINC_ENABLE     0x00
#define RA8875_MWCR0_MEM_WRITE_AUTOINC_DISABLE    0x02
#define RA8875_MWCR0_MEM_READ_AUTOINC_ENABLE      0x00
#define RA8875_MWCR0_MEM_READ_AUTOINC_DISABLE     0x01

/* Memory Write Control Register 1 (MWCR1) REG[41h] */
#define RA8875_MWCR1_GRAPHIC_CURSOR_DISABLE       0x00
#define RA8875_MWCR1_GRAPHIC_CURSOR_ENABLE        0x80
#define RA8875_MWCR1_GRAPHIC_CURSOR_SET1          0x00
#define RA8875_MWCR1_GRAPHIC_CURSOR_SET2          (1<<4)
#define RA8875_MWCR1_GRAPHIC_CURSOR_SET3          (2<<4)
#define RA8875_MWCR1_GRAPHIC_CURSOR_SET4          (3<<4)
#define RA8875_MWCR1_GRAPHIC_CURSOR_SET5          (4<<4)
#define RA8875_MWCR1_GRAPHIC_CURSOR_SET6          (5<<4)
#define RA8875_MWCR1_GRAPHIC_CURSOR_SET7          (6<<4)
#define RA8875_MWCR1_GRAPHIC_CURSOR_SET8          (7<<4)
#define RA8875_MWCR1_WR_DEST_SEL_LAYER1_2         0x00
#define RA8875_MWCR1_WR_DEST_SEL_CGRAM            (1<<2)
#define RA8875_MWCR1_WR_DEST_SEL_GRAPH_CURSOR     (2<<2)
#define RA8875_MWCR1_WR_DEST_SEL_PATTERN          (3<<2)
#define RA8875_MWCR1_SELECT_LAYER1                0x00
#define RA8875_MWCR1_SELECT_LAYER2                0x01


/* Memory Read Cursor Direction (MRCD) REG[45h] */
#define RA8875_MRCD_MEM_READ_DIR_LR_TD            0x00
#define RA8875_MRCD_MEM_READ_DIR_RL_TD            (1<<2)
#define RA8875_MRCD_MEM_READ_DIR_TD_LR            (2<<2)
#define RA8875_MRCD_MEM_READ_DIR_DT_LR            (3<<2)


/* BTE Function Control Register 0 (BECR0) REG[50h] */
#define RA8875_BECR0_BTE_ENABLE                   0x80
#define RA8875_BECR0_BTE_IDLE                     0x00
#define RA8875_BECR0_BTE_BUSY                     0x80
#define RA8875_BECR0_BTE_SOURCE_BLOCK_MODE        0x00
#define RA8875_BECR0_BTE_SOURCE_LINEAR_MODE       0x40
#define RA8875_BECR0_BTE_DEST_BLOCK_MODE          0x00
#define RA8875_BECR0_BTE_DEST_LINEAR_MODE         0x20


/* Touch Panel Control Register0 (TPCR0) REG[70h] */
#define RA8875_TPCR0_TOUCH_DISABLE               0x00
#define RA8875_TPCR0_TOUCH_ENABLE                0x80
#define RA8875_TPCR0_WAIT_512_CLOCS              0x00
#define RA8875_TPCR0_WAIT_1024_CLOCS             (1<<4)
#define RA8875_TPCR0_WAIT_2048_CLOCS             (2<<4)
#define RA8875_TPCR0_WAIT_4096_CLOCS             (3<<4)
#define RA8875_TPCR0_WAIT_8192_CLOCS             (4<<4)
#define RA8875_TPCR0_WAIT_16384_CLOCS            (5<<4)
#define RA8875_TPCR0_WAIT_32768_CLOCS            (6<<4)
#define RA8875_TPCR0_WAIT_65536_CLOCS            (7<<4)
#define RA8875_TPCR0_WAKE_DISABLE                0x00
#define RA8875_TPCR0_WAKE_ENABLE                 0x08
#define RA8875_TPCR0_SYSTEM_CLK_DIV1             0x00
#define RA8875_TPCR0_SYSTEM_CLK_DIV2             0x01
#define RA8875_TPCR0_SYSTEM_CLK_DIV4             0x02
#define RA8875_TPCR0_SYSTEM_CLK_DIV8             0x03
#define RA8875_TPCR0_SYSTEM_CLK_DIV16            0x04
#define RA8875_TPCR0_SYSTEM_CLK_DIV32            0x05
#define RA8875_TPCR0_SYSTEM_CLK_DIV64            0x06
#define RA8875_TPCR0_SYSTEM_CLK_DIV128           0x07


/* Touch Panel Control Register1 (TPCR1) REG[71h] */
#define RA8875_TPCR0_AUTO_MODE                   0x00
#define RA8875_TPCR0_MANUAL_MODE                 0x40
#define RA8875_TPCR0_VREF_INTERNAL               0x00
#define RA8875_TPCR0_VREF_EXTERNAL               0x20
#define RA8875_TPCR0_DEBOUNCE_DISABLE            0x00
#define RA8875_TPCR0_DEBOUNCE_ENABLE             0x04
#define RA8875_TPCR0_MODE_IDLE                   0x00
#define RA8875_TPCR0_MODE_WAIT_EVENT             0x01
#define RA8875_TPCR0_LATCH_XDATA                 0x02
#define RA8875_TPCR0_LATCH_YDATA                 0x03


/* Interrupt Control Register Touch bits */
//#define TP_ICR1_IRQ_ENABLE                       0x04   //  IRQ enable/disable
//#define TP_ICR2_IF_FLAG                          0x04   //  IRQ flag


/* PLL Control Register1 (PLLC1) REG[88h] */
#define RA8875_PLLC1_PLL_PREDIV1                 0x00
#define RA8875_PLLC1_PLL_PREDIV2                 0x80

/* PLL Control Register2 (PLLC2) REG[89h] */
#define RA8875_PLLC2_PLL_PLLDIV1                 0x00
#define RA8875_PLLC2_PLL_PLLDIV2                 0x01
#define RA8875_PLLC2_PLL_PLLDIV4                 0x02
#define RA8875_PLLC2_PLL_PLLDIV8                 0x03
#define RA8875_PLLC2_PLL_PLLDIV16                0x04
#define RA8875_PLLC2_PLL_PLLDIV32                0x05
#define RA8875_PLLC2_PLL_PLLDIV64                0x06
#define RA8875_PLLC2_PLL_PLLDIV1128              0x07

/* PWMx Control Register (PxCR) REG[8Ah] ir REG[8Ch] */
#define RA8875_PxCR_PWM_DISABLE                  0x00
#define RA8875_PxCR_PWM_ENABLE                   0x80
#define RA8875_PxCR_PWM_DISABLE_LEVEL_LOW        0x00
#define RA8875_PxCR_PWM_DISABLE_LEVEL_HIGH       0x40
#define RA8875_PxCR_PWM_FUNC_PWM                 0x00
#define RA8875_PxCR_PWM_FUNC_FIXED_FREQ_SIGNAL   0x10
#define RA8875_PxCR_PWM_CLOCK_DIV1               0x00
#define RA8875_PxCR_PWM_CLOCK_DIV2               0x01
#define RA8875_PxCR_PWM_CLOCK_DIV4               0x02
#define RA8875_PxCR_PWM_CLOCK_DIV8               0x03
#define RA8875_PxCR_PWM_CLOCK_DIV16              0x04
#define RA8875_PxCR_PWM_CLOCK_DIV32              0x05
#define RA8875_PxCR_PWM_CLOCK_DIV64              0x06
#define RA8875_PxCR_PWM_CLOCK_DIV128             0x07



/* Key-Scan Control Register 1 (KSCR1) REG [C0h] */
#define RA8875_KSCR1_KEYSCAN_DISABLE             0x00
#define RA8875_KSCR1_KEYSCAN_ENABLE              0x80
#define RA8875_KSCR1_LONGKEY_DISABLE             0x00
#define RA8875_KSCR1_LONGKEY_ENABLE              0x40
#define RA8875_KSCR1_KEYSCAN_SAMPLING_4          0x00
#define RA8875_KSCR1_KEYSCAN_SAMPLING_8          (1<<4)
#define RA8875_KSCR1_KEYSCAN_SAMPLING_16         (2<<4)
#define RA8875_KSCR1_KEYSCAN_SAMPLING_32         (3<<4)
#define RA8875_KSCR1_KEYSCAN_FREQ_DIV1           0x00
#define RA8875_KSCR1_KEYSCAN_FREQ_DIV2           0x01
#define RA8875_KSCR1_KEYSCAN_FREQ_DIV4           0x02
#define RA8875_KSCR1_KEYSCAN_FREQ_DIV8           0x03
#define RA8875_KSCR1_KEYSCAN_FREQ_DIV16          0x04
#define RA8875_KSCR1_KEYSCAN_FREQ_DIV32          0x05
#define RA8875_KSCR1_KEYSCAN_FREQ_DIV64          0x06
#define RA8875_KSCR1_KEYSCAN_FREQ_DIV128         0x07

/* Key-Scan Control Register 2 (KSCR2) REG [C1h] */
#define RA8875_KSCR2_KEYSCAN_WAKE_DISABLE        0x00
#define RA8875_KSCR2_KEYSCAN_WAKE_ENABLE         0x80
#define RA8875_KSCR2_LONGKEY_TIMING_DIV1         0x00
#define RA8875_KSCR2_LONGKEY_TIMING_DIV2         (1<<2)
#define RA8875_KSCR2_LONGKEY_TIMING_DIV4         (2<<2)
#define RA8875_KSCR2_LONGKEY_TIMING_DIV8         (3<<2)

/* Interrupt Control Register1 (INTC1) REG[F0h] */
#define RA8875_INTC1_KEYSCAN_IRQ_DISABLE         0x00
#define RA8875_INTC1_KEYSCAN_IRQ_ENABLE          0x10
#define RA8875_INTC1_DMA_IRQ_DISABLE             0x00
#define RA8875_INTC1_DMA_IRQ_ENABLE              0x08
#define RA8875_INTC1_TP_IRQ_DISABLE              0x00
#define RA8875_INTC1_TP_IRQ_ENABLE               0x04
#define RA8875_INTC1_BTE_COMPLETE_IRQ_DISABLE    0x00
#define RA8875_INTC1_BTE_COMPLETE_IRQ_ENABLE     0x02
#define RA8875_INTC1_BTE_MCU_IRQ_DISABLE         0x00
#define RA8875_INTC1_BTE_MCU_IRQ_ENABLE          0x01
#define RA8875_INTC1_FONT_WRITE_IRQ_DISABLE      0x00
#define RA8875_INTC1_FONT_WRITE_IRQ_ENABLE       0x01

/* Interrupt Control Register1 (INTC2) REG[F1h] */
#define RA8875_INTC2_KEYSCAN_IF                  0x10
#define RA8875_INTC2_DMA_IF                      0x08
#define RA8875_INTC2_TP_IF                       0x04
#define RA8875_INTC2_BTE_COMPLETE_IF             0x02
#define RA8875_INTC2_BTE_MCU_IF                  0x01
#define RA8875_INTC2_FONT_WRITE_IF               0x01


#endif /* RA8875_REGISTERS_H_INCLUDED */
