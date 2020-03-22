#ifndef RA8875_H_INCLUDED
#define RA8875_H_INCLUDED


// RA8875 does not support multiple commands without
// triggering CS line between them
#define SEQUENTIAL_RS_SUPPORT 0
#define LCD_GEOMETRIC_CLEAR 1

/* BTE functions parameters */
typedef struct _ra8875_bte_conf {
	uint16_t x;     /* Left */
	uint16_t y;     /* Top */
	uint16_t w;     /* Right */
	uint16_t h;     /* Bottom */
	_Bool trans;    /* Use transparency */
	uint16_t color; /* Transparency color/Fill color */
	uint8_t rop;    /* Raster operation code (0-15) */
} ra8875_bte_conf_t;

// Registers & bits
#define RA8875_PWRR                 0x01
#define RA8875_PWRR_DISPON          0x80
#define RA8875_PWRR_DISPOFF         0x00
#define RA8875_PWRR_SLEEP           0x02
#define RA8875_PWRR_NORMAL          0x00
#define RA8875_PWRR_SOFTRESET       0x01

/* REG[02h] Memory Read/Write Command (MRWC) */
#define RA8875_MRWC                 0x02

/* REG[05h] Serial Flash/ROM Configuration Register */
#define RA8875_SROC                 0x05

#define RA8875_GPIOX                0xC7

#define RA8875_PLLC1                0x88
#define RA8875_PLLC1_PLLDIV2        0x80
#define RA8875_PLLC1_PLLDIV1        0x00

#define RA8875_PLLC2                0x89
#define RA8875_PLLC2_DIV1           0x00
#define RA8875_PLLC2_DIV2           0x01
#define RA8875_PLLC2_DIV4           0x02
#define RA8875_PLLC2_DIV8           0x03
#define RA8875_PLLC2_DIV16          0x04
#define RA8875_PLLC2_DIV32      0x05
#define RA8875_PLLC2_DIV64      0x06
#define RA8875_PLLC2_DIV128     0x07

#define RA8875_SYSR             0x10
#define RA8875_SYSR_8BPP        0x00
#define RA8875_SYSR_16BPP       0x0C
#define RA8875_SYSR_MCU8        0x00
#define RA8875_SYSR_MCU16       0x03

#define RA8875_PCSR             0x04
#define RA8875_PCSR_PDATR       0x00
#define RA8875_PCSR_PDATL       0x80
#define RA8875_PCSR_CLK         0x00
#define RA8875_PCSR_2CLK        0x01
#define RA8875_PCSR_4CLK        0x02
#define RA8875_PCSR_8CLK        0x03

#define RA8875_HDWR             0x14

#define RA8875_HNDFTR           0x15
#define RA8875_HNDFTR_DE_HIGH   0x00
#define RA8875_HNDFTR_DE_LOW    0x80

#define RA8875_HNDR             0x16
#define RA8875_HSTR             0x17
#define RA8875_HPWR             0x18
#define RA8875_HPWR_LOW         0x00
#define RA8875_HPWR_HIGH        0x80

#define RA8875_VDHR0            0x19
#define RA8875_VDHR1            0x1A
#define RA8875_VNDR0            0x1B
#define RA8875_VNDR1            0x1C
#define RA8875_VSTR0            0x1D
#define RA8875_VSTR1            0x1E
#define RA8875_VPWR             0x1F
#define RA8875_VPWR_LOW         0x00
#define RA8875_VPWR_HIGH        0x80

#define RA8875_HSAW0            0x30
#define RA8875_HSAW1            0x31
#define RA8875_VSAW0            0x32
#define RA8875_VSAW1            0x33

#define RA8875_HEAW0            0x34
#define RA8875_HEAW1            0x35
#define RA8875_VEAW0            0x36
#define RA8875_VEAW1            0x37

#define RA8875_MCLR             0x8E
#define RA8875_MCLR_START       0x80
#define RA8875_MCLR_STOP        0x00
#define RA8875_MCLR_READSTATUS  0x80
#define RA8875_MCLR_FULL        0x00
#define RA8875_MCLR_ACTIVE      0x40

#define RA8875_DCR                    0x90
#define RA8875_DCR_LINESQUTRI_START   0x80
#define RA8875_DCR_LINESQUTRI_STOP    0x00
#define RA8875_DCR_LINESQUTRI_STATUS  0x80
#define RA8875_DCR_CIRCLE_START       0x40
#define RA8875_DCR_CIRCLE_STATUS      0x40
#define RA8875_DCR_CIRCLE_STOP        0x00
#define RA8875_DCR_FILL               0x20
#define RA8875_DCR_NOFILL             0x00
#define RA8875_DCR_DRAWLINE           0x00
#define RA8875_DCR_DRAWTRIANGLE       0x01
#define RA8875_DCR_DRAWSQUARE         0x10


#define RA8875_ELLIPSE                0xA0
#define RA8875_ELLIPSE_STATUS         0x80

/* REG[40h] Memory Write Control Register 0 (MWCR0) */
#define RA8875_MWCR0            0x40
// Do not auto-increase cursor position (read)
#define RA8875_MWCR0_NOAUTOINCR	0x01
// Do not auto-increase cursor position (write)
#define RA8875_MWCR0_NOAUTOINCW	0x02
// Graphics mode
#define RA8875_MWCR0_GFXMODE    0x00
// Text mode
#define RA8875_MWCR0_TXTMODE    0x80
// Memory write direction
// Left-Right then Top-Down
#define RA8875_MWCR0_LRTB		0x00
// Right-Left then Top-Down
#define RA8875_MWCR0_RLTB		0x04
// Top-Down then Left-Right
#define RA8875_MWCR0_TBLR		0x08
// Down-Top then Left-Right
#define RA8875_MWCR0_BTLR		0x0C

/* REG[41h] Memory Write Control Register 1 (MWCR1) */
#define RA8875_MWCR1			0x41
#define RA8875_MWCR1_LAYER1		0x00
#define RA8875_MWCR1_LAYER2		0x01
#define RA8875_MWCR1_DST_LAYER	0x00
#define RA8875_MWCR1_DST_CGRAM	0x04
#define RA8875_MWCR1_DST_CURS	0x08
#define RA8875_MWCR1_DST_PATT	0x0C
// When writing to CGRAM this must be 0
#define RA8875_MWCR1_GCURSOR	0x80

#define RA8875_CURH0            0x46
#define RA8875_CURH1            0x47
#define RA8875_CURV0            0x48
#define RA8875_CURV1            0x49

#define RA8875_F_CURXL			0x2A
#define RA8875_F_CURXH			0x2B
#define RA8875_F_CURYL			0x2C
#define RA8875_F_CURYH			0x2D

#define RA8875_P1CR             0x8A
#define RA8875_P1CR_ENABLE      0x80
#define RA8875_P1CR_DEFAULT_HIGH      0x40
#define RA8875_P1CR_DISABLE     0x00
#define RA8875_P1CR_CLKOUT      0x10
#define RA8875_P1CR_PWMOUT      0x00

#define RA8875_P1DCR            0x8B

#define RA8875_P2CR             0x8C
#define RA8875_P2CR_ENABLE      0x80
#define RA8875_P2CR_DEFAULT_HIGH      0x40
#define RA8875_P2CR_DISABLE     0x00
#define RA8875_P2CR_CLKOUT      0x10
#define RA8875_P2CR_PWMOUT      0x00

#define RA8875_P2DCR            0x8D

#define RA8875_PWM_CLK_DIV1     0x00
#define RA8875_PWM_CLK_DIV2     0x01
#define RA8875_PWM_CLK_DIV4     0x02
#define RA8875_PWM_CLK_DIV8     0x03
#define RA8875_PWM_CLK_DIV16    0x04
#define RA8875_PWM_CLK_DIV32    0x05
#define RA8875_PWM_CLK_DIV64    0x06
#define RA8875_PWM_CLK_DIV128   0x07
#define RA8875_PWM_CLK_DIV256   0x08
#define RA8875_PWM_CLK_DIV512   0x09
#define RA8875_PWM_CLK_DIV1024  0x0A
#define RA8875_PWM_CLK_DIV2048  0x0B
#define RA8875_PWM_CLK_DIV4096  0x0C
#define RA8875_PWM_CLK_DIV8192  0x0D
#define RA8875_PWM_CLK_DIV16384 0x0E
#define RA8875_PWM_CLK_DIV32768 0x0F

#define RA8875_TPCR0                  0x70
#define RA8875_TPCR0_ENABLE           0x80
#define RA8875_TPCR0_DISABLE          0x00
#define RA8875_TPCR0_WAIT_512CLK      0x00
#define RA8875_TPCR0_WAIT_1024CLK     0x10
#define RA8875_TPCR0_WAIT_2048CLK     0x20
#define RA8875_TPCR0_WAIT_4096CLK     0x30
#define RA8875_TPCR0_WAIT_8192CLK     0x40
#define RA8875_TPCR0_WAIT_16384CLK    0x50
#define RA8875_TPCR0_WAIT_32768CLK    0x60
#define RA8875_TPCR0_WAIT_65536CLK    0x70
#define RA8875_TPCR0_WAKEENABLE       0x08
#define RA8875_TPCR0_WAKEDISABLE      0x00
#define RA8875_TPCR0_ADCCLK_DIV1      0x00
#define RA8875_TPCR0_ADCCLK_DIV2      0x01
#define RA8875_TPCR0_ADCCLK_DIV4      0x02
#define RA8875_TPCR0_ADCCLK_DIV8      0x03
#define RA8875_TPCR0_ADCCLK_DIV16     0x04
#define RA8875_TPCR0_ADCCLK_DIV32     0x05
#define RA8875_TPCR0_ADCCLK_DIV64     0x06
#define RA8875_TPCR0_ADCCLK_DIV128    0x07

#define RA8875_TPCR1            0x71
#define RA8875_TPCR1_AUTO       0x00
#define RA8875_TPCR1_MANUAL     0x40
#define RA8875_TPCR1_VREFINT    0x00
#define RA8875_TPCR1_VREFEXT    0x20
#define RA8875_TPCR1_DEBOUNCE   0x04
#define RA8875_TPCR1_NODEBOUNCE 0x00
#define RA8875_TPCR1_IDLE       0x00
#define RA8875_TPCR1_WAIT       0x01
#define RA8875_TPCR1_LATCHX     0x02
#define RA8875_TPCR1_LATCHY     0x03

#define RA8875_TPXH             0x72
#define RA8875_TPYH             0x73
#define RA8875_TPXYL            0x74

#define RA8875_INTC1            0xF0
#define RA8875_INTC1_KEY        0x10
#define RA8875_INTC1_DMA        0x08
#define RA8875_INTC1_TP         0x04
#define RA8875_INTC1_BTE        0x02

#define RA8875_INTC2            0xF1
#define RA8875_INTC2_KEY        0x10
#define RA8875_INTC2_DMA        0x08
#define RA8875_INTC2_TP         0x04
#define RA8875_INTC2_BTE        0x02



uint8_t     RA8875_Init(void);
void        RA8875_SetBackLight(uint8_t val);



#endif /* RA8875_H_INCLUDED */
