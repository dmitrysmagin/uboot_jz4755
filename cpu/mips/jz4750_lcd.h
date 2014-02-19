/*
 * linux/drivers/video/jz4750_lcd.h -- Ingenic Jz4750 On-Chip LCD frame buffer device
 *
 * Copyright (C) 2005-2008, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __JZ4750_LCD_H__
#define __JZ4750_LCD_H__

//#include <asm/io.h>


#define NR_PALETTE	256
#define PALETTE_SIZE	(NR_PALETTE*2)

/* use new descriptor(8 words) */
struct jz4750_lcd_dma_desc {
	unsigned int next_desc; 	/* LCDDAx */
	unsigned int databuf;   	/* LCDSAx */
	unsigned int frame_id;  	/* LCDFIDx */ 
	unsigned int cmd; 		/* LCDCMDx */
	unsigned int offsize;       	/* Stride Offsize(in word) */
	unsigned int page_width; 	/* Stride Pagewidth(in word) */
	unsigned int cmd_num; 		/* Command Number(for SLCD) */
	unsigned int desc_size; 	/* Foreground Size */
};

struct jz4750lcd_panel_t {
	unsigned int cfg;	/* panel mode and pin usage etc. */
	unsigned int slcd_cfg;	/* Smart lcd configurations */
	unsigned int ctrl;	/* lcd controll register */
	unsigned int w;		/* Panel Width(in pixel) */
	unsigned int h;		/* Panel Height(in line) */
	unsigned int fclk;	/* frame clk */
	unsigned int hsw;	/* hsync width, in pclk */
	unsigned int vsw;	/* vsync width, in line count */
	unsigned int elw;	/* end of line, in pclk */
	unsigned int blw;	/* begin of line, in pclk */
	unsigned int efw;	/* end of frame, in line count */
	unsigned int bfw;	/* begin of frame, in line count */
};


struct jz4750lcd_fg_t {
	int bpp;	/* foreground bpp */
	int x;		/* foreground start position x */
	int y;		/* foreground start position y */
	int w;		/* foreground width */
	int h;		/* foreground height */
};

struct jz4750lcd_osd_t {
	unsigned int osd_cfg;	/* OSDEN, ALHPAEN, F0EN, F1EN, etc */
	unsigned int osd_ctrl;	/* IPUEN, OSDBPP, etc */
	unsigned int rgb_ctrl;	/* RGB Dummy, RGB sequence, RGB to YUV */
	unsigned int bgcolor;	/* background color(RGB888) */
	unsigned int colorkey0;	/* foreground0's Colorkey enable, Colorkey value */
	unsigned int colorkey1; /* foreground1's Colorkey enable, Colorkey value */
	unsigned int alpha;	/* ALPHAEN, alpha value */
	unsigned int ipu_restart; /* IPU Restart enable, ipu restart interval time */

#define FG_NOCHANGE 		0x0000
#define FG0_CHANGE_SIZE 	0x0001
#define FG0_CHANGE_POSITION 	0x0002
#define FG1_CHANGE_SIZE 	0x0010
#define FG1_CHANGE_POSITION 	0x0020
#define FG_CHANGE_ALL 		( FG0_CHANGE_SIZE | FG0_CHANGE_POSITION | \
		FG1_CHANGE_SIZE | FG1_CHANGE_POSITION )
	int fg_change;
	struct jz4750lcd_fg_t fg0;	/* foreground 0 */
	struct jz4750lcd_fg_t fg1;	/* foreground 1 */
};

struct jz4750lcd_info {
	struct jz4750lcd_panel_t panel;
	struct jz4750lcd_osd_t osd;
};


/* Jz LCDFB supported I/O controls. */
#define FBIOSETBACKLIGHT	0x4688 /* set back light level */
#define FBIODISPON		0x4689 /* display on */
#define FBIODISPOFF		0x468a /* display off */
#define FBIORESET		0x468b /* lcd reset */
#define FBIOPRINT_REG		0x468c /* print lcd registers(debug) */
#define FBIOROTATE		0x46a0 /* rotated fb */
#define FBIOGETBUFADDRS		0x46a1 /* get buffers addresses */
#define FBIO_GET_MODE		0x46a2 /* get lcd info */
#define FBIO_SET_MODE		0x46a3 /* set osd mode */
#define FBIO_DEEP_SET_MODE	0x46a4 /* set panel and osd mode */
#define FBIO_MODE_SWITCH	0x46a5 /* switch mode between LCD and TVE */
#define FBIO_GET_TVE_MODE	0x46a6 /* get tve info */
#define FBIO_SET_TVE_MODE	0x46a7 /* set tve mode */

/*
 * LCD panel specific definition
 */
/* AUO */

/* TRULY_TFTG320240DTSW */

// Wolfgang 2008.02.23


#if  defined(CONFIG_JZ4750_SLCD_HX8352B)

static inline void CmdWrite(unsigned int cmd)
{  
	while (REG_SLCD_STATE & SLCD_STATE_BUSY); /* wait slcd ready */
	udelay(30);
	REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | cmd;
}

static inline void DataWrite(unsigned int data)
{  
	while (REG_SLCD_STATE & SLCD_STATE_BUSY); /* wait slcd ready */
	REG_SLCD_DATA = SLCD_DATA_RS_DATA | data;
}

static inline void Mcupanel_RegSet(unsigned int cmd, unsigned int data)
{
	cmd = (cmd & 0xff);
	data = (data & 0xff);

	while (REG_SLCD_STATE & SLCD_STATE_BUSY)
	{
		serial_puts("busy busy cmd \n");
	}
	
	REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | cmd;

	while (REG_SLCD_STATE & SLCD_STATE_BUSY) 
	{
		serial_puts("busy busy data \n");
	}
	REG_SLCD_DATA = SLCD_DATA_RS_DATA | data;
}


static inline void delay(long delay_time)
{
	long cnt; 

	//  delay_time *= (384/8);
	delay_time *= (43/8);

	for (cnt=0;cnt<delay_time;cnt++)
	{
		asm("nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				"nop\n"
				);
	}
}

static inline void mdelay(int n)
{
	delay(n*1000);
}

/*---- LCD Initial ----*/
static void SlcdInit(void)
{
       serial_puts("slcd init start \n");	
	Mcupanel_RegSet(0xE5,0x18);
       serial_puts("slcd init start 1 \n");	
	Mcupanel_RegSet(0xE7,0x18);
       serial_puts("slcd init start 2 \n");	
	Mcupanel_RegSet(0xE8,0x64);
	Mcupanel_RegSet(0xEC,0x08);
	Mcupanel_RegSet(0xED,0x47);
	Mcupanel_RegSet(0xEE,0x20);
	Mcupanel_RegSet(0xEF,0x50);

	Mcupanel_RegSet(0x23,0x83);
	Mcupanel_RegSet(0x24,0x79);
	Mcupanel_RegSet(0x25,0x4F);
	Mcupanel_RegSet(0x29,0x00);
	Mcupanel_RegSet(0x2B,0x03);

	Mcupanel_RegSet(0xE2,0x1A);
	Mcupanel_RegSet(0x1B,0x1E);
	Mcupanel_RegSet(0x36,0x00);
	Mcupanel_RegSet(0x01,0x00);
	Mcupanel_RegSet(0x1A,0x00);	

	Mcupanel_RegSet(0x1C,0x03);
	Mcupanel_RegSet(0x19,0x01);
	Mcupanel_RegSet(0x18,0x0C);

	mdelay(5);
	Mcupanel_RegSet(0x1F,0x90);
	mdelay(10);
	Mcupanel_RegSet(0x1F,0xD4);
	mdelay(5);

	Mcupanel_RegSet(0x40,0x00);
	Mcupanel_RegSet(0x41,0x29);
	Mcupanel_RegSet(0x42,0x26);
	Mcupanel_RegSet(0x43,0x3E);
	Mcupanel_RegSet(0x44,0x3D);
	Mcupanel_RegSet(0x45,0x3F);
	Mcupanel_RegSet(0x46,0x1B);
	Mcupanel_RegSet(0x47,0x68);
	Mcupanel_RegSet(0x48,0x04);
	Mcupanel_RegSet(0x49,0x05);
	Mcupanel_RegSet(0x4A,0x06);
	Mcupanel_RegSet(0x4B,0x0C);
	Mcupanel_RegSet(0x4C,0x17);
	Mcupanel_RegSet(0x50,0x00);
	Mcupanel_RegSet(0x51,0x02);
	Mcupanel_RegSet(0x52,0x01);
	Mcupanel_RegSet(0x53,0x19);
	Mcupanel_RegSet(0x54,0x16);
	Mcupanel_RegSet(0x55,0x3F);
	Mcupanel_RegSet(0x56,0x17);
	Mcupanel_RegSet(0x57,0x64);

	Mcupanel_RegSet(0x58,0x08);
	Mcupanel_RegSet(0x59,0x13);
	Mcupanel_RegSet(0x5A,0x19);
	Mcupanel_RegSet(0x5B,0x1A);
	Mcupanel_RegSet(0x5C,0x1B);
	Mcupanel_RegSet(0x5D,0xFF);

	Mcupanel_RegSet(0x28,0x20);
	mdelay(10);
	Mcupanel_RegSet(0x28,0x38);
	mdelay(10);
	Mcupanel_RegSet(0x28,0x3C);
	Mcupanel_RegSet(0x17,0x05);
	Mcupanel_RegSet(0x16,0x68);
	Mcupanel_RegSet(0x02,0x00); // SC[15:8]
	Mcupanel_RegSet(0x03,0x00); // SC[7:0]
	Mcupanel_RegSet(0x04,0x01); // EC[15:8]
	Mcupanel_RegSet(0x05,0x8F); // EC[7:0]
	Mcupanel_RegSet(0x06,0x00); // SP[15:8]
	Mcupanel_RegSet(0x07,0x00); // SP[7:0]
	Mcupanel_RegSet(0x08,0x00); // EP[15:8]
	Mcupanel_RegSet(0x09,0xEF); // EP[7:0]

	serial_puts("lcd init dodo finish here \n");


	CmdWrite(0x22);
}

#define PIN_RESET_N 	(32*3+21)	//reset D21
#define PIN_CS_N 	(32*3+18)	//CS D18

#define __lcd_slcd_pin_init()						\
	do {	__gpio_as_output(PIN_CS_N); /*CS: LCD_SPL */	\
		__gpio_clear_pin(PIN_CS_N);				\
		__gpio_as_output(PIN_RESET_N); /* RESET#: LCD_SPL */	\
		__gpio_set_pin(PIN_RESET_N);				\
		mdelay(100);						\
		__gpio_clear_pin(PIN_RESET_N);				\
		mdelay(100);						\
		__gpio_set_pin(PIN_RESET_N);				\
		/* Configure SLCD module */				\
		REG_LCD_CTRL &= ~(LCD_CTRL_ENA|LCD_CTRL_DIS); /* disable lcdc */ \
		REG_LCD_CFG = LCD_CFG_LCDPIN_SLCD | 0x0D; /* LCM */	\
		REG_SLCD_CTRL &= ~SLCD_CTRL_DMA_EN; /* disable slcd dma */ \
		REG_SLCD_CFG = SLCD_CFG_DWIDTH_16 | SLCD_CFG_CWIDTH_16BIT | SLCD_CFG_CS_ACTIVE_LOW | SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING | SLCD_CFG_TYPE_PARALLEL; \
		REG_LCD_REV = 0x04;	/* lcd clock??? */			\
	}while (0)

#define __lcd_slcd_special_on()						\
	do {								\
		printf("__lcd_slcd_special_on fuck now \n");		\
		__lcd_slcd_pin_init();					\
		SlcdInit();						\
		REG_SLCD_CTRL |= SLCD_CTRL_DMA_EN; /* slcdc dma enable */ \
	} while (0)

#endif	/* #if CONFIG_JZ4750_LCD_TRULY_TFT_GG1P0319LTSW_W */

#ifndef __lcd_special_pin_init
#define __lcd_special_pin_init()
#endif
#ifndef __lcd_special_on
#define __lcd_special_on()
#endif
#ifndef __lcd_special_off
#define __lcd_special_off()
#endif


/*
 * Platform specific definition
 */
#if defined(CONFIG_CETUS) || defined(CONFIG_320E) /* board apus */

#define LCD_PWM_FULL 101
/* 100 level: 0,1,...,100 */
#define __lcd_set_backlight_level(n)	\
	do {					\
		__gpio_as_output(GPIO_LCD_PWM);	\
		__gpio_set_pin(GPIO_LCD_PWM);	\
	} while (0)

#define __lcd_close_backlight()		\
	do {					\
		__gpio_as_output(GPIO_LCD_PWM);	\
		__gpio_clear_pin(GPIO_LCD_PWM);	\
	} while (0)

#define __lcd_display_pin_init() \
	do { \
		__lcd_special_pin_init();	   \
	} while (0)
#define __lcd_display_on() \
	do { \
		__lcd_special_on();			\
		__lcd_slcd_special_on();			\
		__lcd_set_backlight_level(80);		\
	} while (0)

#define __lcd_display_off() \
	do { \
		__lcd_close_backlight();	   \
		__lcd_special_off();	 \
	} while (0)
#endif /* APUS */


/*****************************************************************************
 * LCD display pin dummy macros
 *****************************************************************************/

#ifndef __lcd_display_pin_init
#define __lcd_display_pin_init()
#endif
#ifndef __lcd_slcd_special_on
#define __lcd_slcd_special_on()
#endif
#ifndef __lcd_display_on
#define __lcd_display_on()
#endif
#ifndef __lcd_display_off
#define __lcd_display_off()
#endif
#ifndef __lcd_set_backlight_level
#define __lcd_set_backlight_level(n)
#endif

#endif /* __JZ4750_LCD_H__ */
