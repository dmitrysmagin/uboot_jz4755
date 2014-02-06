/*
 * JzRISC lcd controller
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/************************************************************************/
/* ** HEADER FILES							*/
/************************************************************************/

/* 
 * Fallowing macro may be used:
 *  CONFIG_LCD                        : LCD support 
 *  LCD_BPP                           : Bits per pixel, 0 = 1, 1 = 2, 2 = 4, 3 = 8
 *  CFG_WHITE_ON_BLACK
 *  CONFIG_LCD_LOGO                   : show logo
 *  CFG_LCD_LOGOONLY_NOINFO           : not display info on lcd screen, only logo
 * -----------------------------------------------------------------------
 * bugs:
 * if BMP_LOGO_HEIGHT > (lcd screen height - 2*VIDEO_FONT_HEIGHT),
 * must not print info onto screen,
 * it means should define CFG_LCD_LOGOONLY_NOINFO.
 */



#include <config.h>
#include <common.h>
#include <devices.h>
#include <lcd.h>

#include <asm/io.h>               /* virt_to_phys() */

#if defined(CONFIG_JZ4750) || defined(CONFIG_JZ4750D) || defined(CONFIG_JZ4750L)
#if defined(CONFIG_LCD) && !defined(CONFIG_SLCD)
//#if defined(CONFIG_4750_LCD)// && !defined(CONFIG_4750_SLCD)
#if defined(CONFIG_JZ4750)
#include <asm/jz4750.h>
#endif
#if defined(CONFIG_JZ4750D)
#include <asm/jz4750d.h>
#endif
#if defined(CONFIG_JZ4750L)
#include <asm/jz4750l.h>
#endif
#include "jz4750_lcd.h"
#include "battery_null.h"
#include "battery_full.h"



#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define dprintf(x...)	printf(x)
#else
#define dprintf(x...)
#endif

#define PRINTF_DEBUG dprintf("%s %d\n",__FILE__,__LINE__)

static struct jz4750_lcd_dma_desc *dma_desc_base = NULL;
static struct jz4750_lcd_dma_desc *dma0_desc_palette = NULL, *dma0_desc0 = NULL, *dma0_desc1 = NULL, *dma1_desc0 = NULL, *dma1_desc1 = NULL;
#define DMA_DESC_NUM 		6
static struct jz4750_lcd_dma_desc *dma0_desc_cmd0 = NULL, *dma0_desc_cmd = NULL;
static unsigned char *lcd_palette;
static unsigned char *lcd_frame0;
static unsigned char *lcd_frame1;
static struct lcd_cfb_info *jz4750fb_info;


struct jz4750lcd_info jzfb = {
#if defined(CONFIG_JZ4750_LCD_SAMSUNG_LTP400WQF02)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */ 
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_GENERIC_TFT | /* General TFT panel */
		LCD_CFG_MODE_TFT_18BIT | 	/* output 18bpp */
		LCD_CFG_HSP | 	/* Hsync polarity: active low */
		LCD_CFG_VSP,	/* Vsync polarity: leading edge is falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
		480, 272, 60, 41, 10, 2, 2, 2, 2,
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
//		 LCD_OSDC_ALPHAEN | /* enable alpha */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x000000, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {32, 0, 0, 480, 272}, /* bpp, x, y, w, h */
		 .fg1 = {32, 0, 0, 480, 272}, /* bpp, x, y, w, h */
	 },
#elif defined(CONFIG_JZ4750_LCD_AUO_A043FL01V2)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */ 
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_GENERIC_TFT | /* General TFT panel */
		LCD_CFG_MODE_TFT_18BIT | 	/* output 18bpp */
		LCD_CFG_HSP | 	/* Hsync polarity: active low */
		LCD_CFG_VSP,	/* Vsync polarity: leading edge is falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
		480, 272, 60, 41, 10, 8, 4, 4, 2,
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
//		 LCD_OSDC_ALPHAEN | /* enable alpha */
//		 LCD_OSDC_F1EN | /* enable Foreground1 */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x0000ff, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {32, 0, 0, 320, 240}, /* bpp, x, y, w, h */
		 .fg1 = {32, 0, 0, 320, 240}, /* bpp, x, y, w, h */
	 },
#elif defined(CONFIG_JZ4750_LCD_TRULY_TFT_GG1P0319LTSW_W)
	.panel = {
//		 .cfg = LCD_CFG_LCDPIN_SLCD | LCD_CFG_RECOVER | /* Underrun recover*/ 
		 .cfg = LCD_CFG_LCDPIN_SLCD | /* Underrun recover*/ 
		 LCD_CFG_NEWDES | /* 8words descriptor */
		 LCD_CFG_MODE_SLCD, /* TFT Smart LCD panel */
		 .slcd_cfg = SLCD_CFG_DWIDTH_16BIT | SLCD_CFG_CWIDTH_16BIT | SLCD_CFG_CS_ACTIVE_LOW | SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING | SLCD_CFG_TYPE_PARALLEL,
		 .ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
		 240, 320, 60, 0, 0, 0, 0, 0, 0,
	 },
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
//		 LCD_OSDC_ALPHAEN | /* enable alpha */
//		 LCD_OSDC_F1EN | /* enable Foreground0 */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x000000, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {32, 0, 0, 240, 320}, /* bpp, x, y, w, h */
		 .fg1 = {32, 0, 0, 240, 320}, /* bpp, x, y, w, h */
	 },

#elif defined(CONFIG_JZ4750_LCD_FOXCONN_PT035TN01)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */ 
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_GENERIC_TFT | /* General TFT panel */
//		LCD_CFG_MODE_TFT_18BIT | 	/* output 18bpp */
		LCD_CFG_MODE_TFT_24BIT | 	/* output 24bpp */
		LCD_CFG_HSP | 	/* Hsync polarity: active low */
		LCD_CFG_VSP |	/* Vsync polarity: leading edge is falling edge */
		LCD_CFG_PCP,	/* Pix-CLK polarity: data translations at falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
		320, 240, 80, 1, 1, 10, 50, 10, 13
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
//		 LCD_OSDC_ALPHAEN | /* enable alpha */
//		 LCD_OSDC_F1EN |	/* enable Foreground1 */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x000000, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {32, 0, 0, 320, 240}, /* bpp, x, y, w, h */
		 .fg1 = {32, 0, 0, 320, 240}, /* bpp, x, y, w, h */
	 },
#elif defined(CONFIG_JZ4750_LCD_INNOLUX_PT035TN01_SERIAL)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */ 
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_SERIAL_TFT | /* Serial TFT panel */
		LCD_CFG_MODE_TFT_18BIT | 	/* output 18bpp */
		LCD_CFG_HSP | 	/* Hsync polarity: active low */
		LCD_CFG_VSP |	/* Vsync polarity: leading edge is falling edge */
		LCD_CFG_PCP,	/* Pix-CLK polarity: data translations at falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
		320, 240, 60, 1, 1, 10, 50, 10, 13
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
//		 LCD_OSDC_ALPHAEN | /* enable alpha */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x000000, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {32, 0, 0, 320, 240}, /* bpp, x, y, w, h */
		 .fg1 = {32, 0, 0, 320, 240}, /* bpp, x, y, w, h */
	 },
#elif defined(CONFIG_JZ4750_SLCD_KGM701A3_TFT_SPFD5420A)
	.panel = {
//		 .cfg = LCD_CFG_LCDPIN_SLCD | LCD_CFG_RECOVER | /* Underrun recover*/ 
		 .cfg = LCD_CFG_LCDPIN_SLCD | /* Underrun recover*/ 
//		 LCD_CFG_DITHER | /* dither */
		 LCD_CFG_NEWDES | /* 8words descriptor */
		 LCD_CFG_MODE_SLCD, /* TFT Smart LCD panel */
		 .slcd_cfg = SLCD_CFG_DWIDTH_18BIT | SLCD_CFG_CWIDTH_18BIT | SLCD_CFG_CS_ACTIVE_LOW | SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING | SLCD_CFG_TYPE_PARALLEL,
		 .ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
		 400, 240, 60, 0, 0, 0, 0, 0, 0,
	 },
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
//		 LCD_OSDC_ALPHAEN | /* enable alpha */
//		 LCD_OSDC_ALPHAMD | /* alpha blending mode */
//		 LCD_OSDC_F1EN | /* enable Foreground1 */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x000000, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {32, 0, 0, 400, 240}, /* bpp, x, y, w, h */
		 .fg1 = {32, 0, 0, 400, 240}, /* bpp, x, y, w, h */
	 },
#elif defined(CONFIG_JZ4750_LCD_L430)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */ 
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_GENERIC_TFT | /* Serial TFT panel */
		LCD_CFG_MODE_TFT_24BIT | 	/* output 24bpp */
		LCD_CFG_HSP | 	/* Hsync polarity: active low */
		LCD_CFG_VSP ,	/* Vsync polarity: leading edge is falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
                480, 272,  40, 1, 1, 40, 215, 0, 45,
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
//		 LCD_OSDC_ALPHAEN | /* enable alpha */
		 LCD_OSDC_F0EN,	/* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x000000, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {16, 0, 0, 480, 272}, /* bpp, x, y, w, h */
		 .fg1 = {32, 0, 0, 480, 272}, /* bpp, x, y, w, h */
	 },

#else
#error "Select LCD panel first!!!"
#endif
};
struct jz4750lcd_info *jz4750_lcd_info = &jzfb; /* default output to lcd panel */

/************************************************************************/

vidinfo_t panel_info = {
#if defined(CONFIG_JZ4750_LCD_SAMSUNG_LTP400WQF02)
	480, 272, LCD_BPP,
#elif defined(CONFIG_JZ4750_LCD_AUO_A043FL01V2)
	480, 272, LCD_BPP,
#elif defined(CONFIG_JZ4750_LCD_TRULY_TFT_GG1P0319LTSW_W)
	240, 320, LCD_BPP,
#elif defined(CONFIG_JZ4750_LCD_FOXCONN_PT035TN01)
	320, 240, LCD_BPP,
#elif defined(CONFIG_JZ4750_LCD_INNOLUX_PT035TN01_SERIAL)
	320, 240, LCD_BPP,
#elif defined(CONFIG_JZ4750_SLCD_KGM701A3_TFT_SPFD5420A)
	400, 240, LCD_BPP,
#elif defined(CONFIG_JZ4750_LCD_L430)
	480, 272, LCD_BPP,
#else
#error "Select LCD panel first!!!"
#endif
//#if defined(CONFIG_JZLCD_SHARP_LQ035Q7)
//	240, 320, LCD_BPP,

};


/*----------------------------------------------------------------------*/

int lcd_line_length;

int lcd_color_fg;
int lcd_color_bg;

/*
 * Frame buffer memory information
 */
void *lcd_base;			/* Start of framebuffer memory	*/
void *lcd_console_address;	/* Start of console buffer	*/

short console_col;
short console_row;

/************************************************************************/

void lcd_ctrl_init (void *lcdbase);

void lcd_enable (void);
void lcd_disable (void);
#if defined(CONFIG_JZ4750D)
static int lcd_open(void);
#endif

/************************************************************************/

static int  jz_lcd_init_mem(void *lcdbase, vidinfo_t *vid);
//static void jz_lcd_desc_init(vidinfo_t *vid);
//static int  jz_lcd_hw_init( vidinfo_t *vid );
extern int flush_cache_all(void);

#if LCD_BPP == LCD_COLOR8
void lcd_setcolreg (ushort regno, ushort red, ushort green, ushort blue);
#endif
#if LCD_BPP == LCD_MONOCHROME
void lcd_initcolregs (void);
#endif

/************************************************************************/
static void jz4750fb_change_clock( struct jz4750lcd_info * lcd_info )
{
  unsigned int val = 0;
  unsigned int pclk;
  /* Timing setting */
  __cpm_stop_lcd();

  val = lcd_info->panel.fclk; /* frame clk */

  if ( (lcd_info->panel.cfg & LCD_CFG_MODE_MASK) != LCD_CFG_MODE_SERIAL_TFT) {
    pclk = val * (lcd_info->panel.w + lcd_info->panel.hsw + lcd_info->panel.elw + lcd_info->panel.blw) * (lcd_info->panel.h + lcd_info->panel.vsw + lcd_info->panel.efw + lcd_info->panel.bfw); /* Pixclk */
  }
  else {
    /* serial mode: Hsync period = 3*Width_Pixel */
    pclk = val * (lcd_info->panel.w*3 + lcd_info->panel.hsw + lcd_info->panel.elw + lcd_info->panel.blw) * (lcd_info->panel.h + lcd_info->panel.vsw + lcd_info->panel.efw + lcd_info->panel.bfw); /* Pixclk */
  }

  /********* In TVE mode PCLK = 27MHz ***********/
  if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { 		/* LCDC output to TVE */
  }
  else { 		/* LCDC output to  LCD panel */
    //pclk = 12000000;
    pclk = 22000000;
    val = __cpm_get_pllout2() / pclk; /* pclk */
    dprintf("pllout2 = %d plck is %d\n", __cpm_get_pllout2(),pclk);
    val--;
    dprintf("ratio: val = %d\n", val);
    if ( val > 0x7ff ) {
      dprintf("pixel clock divid is too large, set it to 0x7ff\n");
      val = 0x7ff;
    }
    __cpm_set_pixdiv(val);
    dprintf("REG_CPM_LPCDR = 0x%08x\n", REG_CPM_LPCDR);
#if defined(CONFIG_SOC_JZ4750) /* Jz4750D don't use LCLK */
    val = pclk * 3 ;	/* LCDClock > 2.5*Pixclock */
    val =__cpm_get_pllout2() / val;
    if ( val > 0x1f ) {
      dprintf("lcd clock divide is too large, set it to 0x1f\n");
      val = 0x1f;
    }
    __cpm_set_ldiv( val );
#endif
    REG_CPM_CPCCR |= CPM_CPCCR_CE ; /* update divide */

  }

  dprintf("REG_CPM_LPCDR=0x%08x\n", REG_CPM_LPCDR);
  dprintf("REG_CPM_CPCCR=0x%08x\n", REG_CPM_CPCCR);

  __cpm_start_lcd();
  udelay(1000);
  /* 
   * set lcd device clock and lcd pixel clock.
   * what about TVE mode???
   *
   */
}
static void jz4750fb_set_osd_mode( struct jz4750lcd_info * lcd_info )
{
	dprintf("%s, %d\n", __FILE__, __LINE__ );
	lcd_info->osd.osd_ctrl &= ~(LCD_OSDCTRL_OSDBPP_MASK);
	if ( lcd_info->osd.fg1.bpp == 15 )
		lcd_info->osd.osd_ctrl |= LCD_OSDCTRL_OSDBPP_15_16|LCD_OSDCTRL_RGB555;
	else if ( lcd_info->osd.fg1.bpp == 16 )
		lcd_info->osd.osd_ctrl |= LCD_OSDCTRL_OSDBPP_15_16|LCD_OSDCTRL_RGB565;
	else {
		lcd_info->osd.fg1.bpp = 32;
		lcd_info->osd.osd_ctrl |= LCD_OSDCTRL_OSDBPP_18_24;
	}

	REG_LCD_OSDC 	= lcd_info->osd.osd_cfg; /* F0, F1, alpha, */

	REG_LCD_OSDCTRL = lcd_info->osd.osd_ctrl; /* IPUEN, bpp */
	REG_LCD_RGBC  	= lcd_info->osd.rgb_ctrl;
	REG_LCD_BGC  	= lcd_info->osd.bgcolor;
	REG_LCD_KEY0 	= lcd_info->osd.colorkey0;
	REG_LCD_KEY1 	= lcd_info->osd.colorkey1;
	REG_LCD_ALPHA 	= lcd_info->osd.alpha;
	REG_LCD_IPUR 	= lcd_info->osd.ipu_restart;
}
static void jz4750fb_foreground_resize( struct jz4750lcd_info * lcd_info )
{
	int fg0_line_size, fg0_frm_size, fg1_line_size, fg1_frm_size;
	/* 
	 * NOTE: 
	 * Foreground change sequence: 
	 * 	1. Change Position Registers -> LCD_OSDCTL.Change;
	 * 	2. LCD_OSDCTRL.Change -> descripter->Size
	 * Foreground, only one of the following can be change at one time:
	 * 	1. F0 size;
	 *	2. F0 position
	 * 	3. F1 size
	 *	4. F1 position
	 */
	
	/* 
	 * The rules of f0, f1's position:
	 * 	f0.x + f0.w <= panel.w;
	 * 	f0.y + f0.h <= panel.h;
	 *
	 * When output is LCD panel, fg.y and fg.h can be odd number or even number.
	 * When output is TVE, as the TVE has odd frame and even frame,
	 * to simplified operation, fg.y and fg.h should be even number always.
	 *
	 */
        dprintf("the fg0x is %d fg0y is %d fg0w is %d fgoh is %d\n",lcd_info->osd.fg0.x,lcd_info->osd.fg0.y,lcd_info->osd.fg0.w,lcd_info->osd.fg0.h);
        dprintf("the fg1x is %d fg1y is %d fg1w is %d fg1h is %d\n",lcd_info->osd.fg1.x,lcd_info->osd.fg1.y,lcd_info->osd.fg1.w,lcd_info->osd.fg1.h);
	/* Foreground 0  */
	if ( lcd_info->osd.fg0.x >= lcd_info->panel.w )
		lcd_info->osd.fg0.x = lcd_info->panel.w;
	if ( lcd_info->osd.fg0.y >= lcd_info->panel.h )
		lcd_info->osd.fg0.y = lcd_info->panel.h;
	if ( lcd_info->osd.fg0.x + lcd_info->osd.fg0.w > lcd_info->panel.w )
		lcd_info->osd.fg0.w = lcd_info->panel.w - lcd_info->osd.fg0.x;
	if ( lcd_info->osd.fg0.y + lcd_info->osd.fg0.h > lcd_info->panel.h ) 
		lcd_info->osd.fg0.h = lcd_info->panel.h - lcd_info->osd.fg0.y;
	/* Foreground 1 */
	/* Case TVE ??? TVE 720x573 or 720x480*/
	if ( lcd_info->osd.fg1.x >= lcd_info->panel.w ) 
		lcd_info->osd.fg1.x = lcd_info->panel.w;
	if ( lcd_info->osd.fg1.y >= lcd_info->panel.h ) 
		lcd_info->osd.fg1.y = lcd_info->panel.h;
	if ( lcd_info->osd.fg1.x + lcd_info->osd.fg1.w > lcd_info->panel.w ) 
		lcd_info->osd.fg1.w = lcd_info->panel.w - lcd_info->osd.fg1.x;
	if ( lcd_info->osd.fg1.y + lcd_info->osd.fg1.h > lcd_info->panel.h ) 
		lcd_info->osd.fg1.h = lcd_info->panel.h - lcd_info->osd.fg1.y;

//	fg0_line_size = lcd_info->osd.fg0.w*((lcd_info->osd.fg0.bpp+7)/8);
	fg0_line_size = (lcd_info->osd.fg0.w*(lcd_info->osd.fg0.bpp)/8);
	fg0_line_size = ((fg0_line_size+3)>>2)<<2; /* word aligned */
	fg0_frm_size = fg0_line_size * lcd_info->osd.fg0.h;
	
	dprintf("fg0_frm_size = 0x%x\n",fg0_frm_size);

	fg1_line_size = lcd_info->osd.fg1.w*((lcd_info->osd.fg1.bpp+7)/8);
	fg1_line_size = ((fg1_line_size+3)>>2)<<2; /* word aligned */
	fg1_frm_size = fg1_line_size * lcd_info->osd.fg1.h;

	if ( lcd_info->osd.fg_change ) {
		if ( lcd_info->osd.fg_change & FG0_CHANGE_POSITION ) { /* F0 change position */
			REG_LCD_XYP0 = lcd_info->osd.fg0.y << 16 | lcd_info->osd.fg0.x;
		}
		if ( lcd_info->osd.fg_change & FG1_CHANGE_POSITION ) { /* F1 change position */
			REG_LCD_XYP1 = lcd_info->osd.fg1.y << 16 | lcd_info->osd.fg1.x;
		}

		/* set change */
		if ( !(lcd_info->osd.osd_ctrl & LCD_OSDCTRL_IPU) && 
		     (lcd_info->osd.fg_change != FG_CHANGE_ALL) )
			REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;

		/* wait change ready???  maddrone open*/ 
		while ( REG_LCD_OSDS & LCD_OSDS_READY )	/* fix in the future, Wolfgang, 06-20-2008 */
		dprintf("wait LCD_OSDS_READY\n");
		
		if ( lcd_info->osd.fg_change & FG0_CHANGE_SIZE ) { /* change FG0 size */
			if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* output to TV */
				dma0_desc0->cmd = dma0_desc1->cmd = (fg0_frm_size/4)/2;
				dma0_desc0->offsize = dma0_desc1->offsize 
					= fg0_line_size/4;
				dma0_desc0->page_width = dma0_desc1->page_width 
					= fg0_line_size/4;
				#ifdef TVOUT_2x
				if(tvout_640_480)
				dma0_desc1->databuf = virt_to_phys((void *)(lcd_frame0 + fg0_line_size));  //maddrone
				else
				dma0_desc1->databuf = virt_to_phys((void *)(lcd_frame01 + fg0_line_size));  //maddrone
				#else
				dma0_desc1->databuf = virt_to_phys((void *)(lcd_frame0 + fg0_line_size));
				#endif
				REG_LCD_DA0 = virt_to_phys(dma0_desc0); //tft
			}
			else {
				dma0_desc0->cmd = dma0_desc1->cmd = fg0_frm_size/4;
				dma0_desc0->offsize = dma0_desc1->offsize =0;
				dma0_desc0->page_width = dma0_desc1->page_width = 0;
			}

			dma0_desc0->desc_size = dma0_desc1->desc_size 
				= lcd_info->osd.fg0.h << 16 | lcd_info->osd.fg0.w;
			REG_LCD_SIZE0 = (lcd_info->osd.fg0.h<<16)|lcd_info->osd.fg0.w;

		}

		if ( lcd_info->osd.fg_change & FG1_CHANGE_SIZE ) { /* change FG1 size*/
			if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* output to TV */
				dma1_desc0->cmd = dma1_desc1->cmd = (fg1_frm_size/4)/2;
				dma1_desc0->offsize = dma1_desc1->offsize = fg1_line_size/4;
				dma1_desc0->page_width = dma1_desc1->page_width = fg1_line_size/4;
				dma1_desc1->databuf = virt_to_phys((void *)(lcd_frame1 + fg1_line_size));  //maddrone
				REG_LCD_DA1 = virt_to_phys(dma0_desc1); //tft

			}
			else {
				dma1_desc0->cmd = dma1_desc1->cmd = fg1_frm_size/4;
				dma1_desc0->offsize = dma1_desc1->offsize = 0;
				dma1_desc0->page_width = dma1_desc1->page_width = 0;
			}
			
			dma1_desc0->desc_size = dma1_desc1->desc_size 
				= lcd_info->osd.fg1.h << 16 | lcd_info->osd.fg1.w;
			REG_LCD_SIZE1 = lcd_info->osd.fg1.h << 16|lcd_info->osd.fg1.w;
		}

		//dma_cache_wback((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz4750_lcd_dma_desc));
		lcd_info->osd.fg_change = FG_NOCHANGE; /* clear change flag */
	}
}
/* 
 * jz4750fb_set_mode(), set osd configure, resize foreground
 *
 */
static void jz4750fb_set_mode( struct jz4750lcd_info * lcd_info )
{
	struct lcd_cfb_info *cfb = jz4750fb_info;

	jz4750fb_set_osd_mode(lcd_info);
	jz4750fb_foreground_resize(lcd_info);
	//jz4750fb_set_var(&cfb->fb.var, -1, &cfb->fb);
}
#undef  GPIO_LCD_VCC_EN_N
#undef  GPIO_LCD_PWM   	
#undef  SPEN		
#undef  SPCK		
#undef  SPDA		
#undef  LCD_RET 	

#define GPIO_LCD_VCC_EN_N	(32 * 4 + 1) /* SDATI */
#define GPIO_LCD_PWM   		(32*4+22) /* GPE22 PWM2 */ 
#define SPEN		(32*3+22)       /*LCD_CS*/
#define SPCK		(32*4+13)       /*LCD_SCL*/
#define SPDA		(32*4+12)       /*LCD_SDA*/
#define LCD_RET 	(32*4+2)       /*LCD_DISP_N use for lcd reset*/

#define __lcd_special_pin_init() \
	do { \
		__gpio_as_output(LCD_RET);\
		udelay(50);\
		__gpio_clear_pin(LCD_RET);\
		udelay(150000);\
		__gpio_set_pin(LCD_RET);\
	} while (0)
#define __lcd_display_pin_init() \
do { \
	__gpio_as_output(GPIO_LCD_VCC_EN_N);	 \
	__gpio_as_output(GPIO_LCD_PWM);	 \
	__lcd_special_pin_init();	   \
} while (0)
/* initial dma descriptors */
static void jz4750fb_descriptor_init( struct jz4750lcd_info * lcd_info )
{
	unsigned int pal_size;

	switch ( lcd_info->osd.fg0.bpp ) {
	case 1:
		pal_size = 4;
		break;
	case 2:
		pal_size = 8;
		break;
	case 4:
		pal_size = 32;
		break;
	case 8:
	default:
		pal_size = 512;
	}

	pal_size /= 4;

	dma0_desc_palette 	= dma_desc_base + 0;
	dma0_desc0 		= dma_desc_base + 1;
	dma0_desc1 		= dma_desc_base + 2;
	dma0_desc_cmd0 		= dma_desc_base + 3; /* use only once */
	dma0_desc_cmd 		= dma_desc_base + 4;
	dma1_desc0 		= dma_desc_base + 5;
	dma1_desc1 		= dma_desc_base + 6;

	/*
	 * Normal TFT panel's DMA Chan0: 
	 *	TO LCD Panel: 	
	 * 		no palette:	dma0_desc0 <<==>> dma0_desc0 
	 * 		palette :	dma0_desc_palette <<==>> dma0_desc0
	 *	TO TV Encoder:
	 * 		no palette:	dma0_desc0 <<==>> dma0_desc1
	 * 		palette:	dma0_desc_palette --> dma0_desc0 
	 * 				--> dma0_desc1 --> dma0_desc_palette --> ...
	 * 				
	 * SMART LCD TFT panel(dma0_desc_cmd)'s DMA Chan0: 
	 *	TO LCD Panel:
	 * 		no palette:	dma0_desc_cmd <<==>> dma0_desc0 
	 * 		palette :	dma0_desc_palette --> dma0_desc_cmd
	 * 				--> dma0_desc0 --> dma0_desc_palette --> ...
	 *	TO TV Encoder:
	 * 		no palette:	dma0_desc_cmd --> dma0_desc0 
	 * 				--> dma0_desc1 --> dma0_desc_cmd --> ...
	 * 		palette:	dma0_desc_palette --> dma0_desc_cmd 
	 * 				--> dma0_desc0 --> dma0_desc1 
	 * 				--> dma0_desc_palette --> ...
	 * DMA Chan1:
	 *	TO LCD Panel:
	 * 		dma1_desc0 <<==>> dma1_desc0 
	 *	TO TV Encoder:
	 * 		dma1_desc0 <<==>> dma1_desc1
	 */

#if defined(CONFIG_FB_JZ4750_SLCD)
	/* First CMD descriptors, use only once, cmd_num isn't 0 */
	dma0_desc_cmd0->next_desc 	= (unsigned int)virt_to_phys(dma0_desc0);
	dma0_desc_cmd0->databuf 	= (unsigned int)virt_to_phys((void *)lcd_cmdbuf);
	dma0_desc_cmd0->frame_id 	= (unsigned int)0x0da0cad0; /* dma0's cmd0 */
	dma0_desc_cmd0->cmd 		= LCD_CMD_CMD | 3; /* command */
	//dma0_desc_cmd0->cmd 		= LCD_CMD_CMD | 0; /* command */
	dma0_desc_cmd0->offsize 	= 0;
	dma0_desc_cmd0->page_width 	= 0; 
	dma0_desc_cmd0->cmd_num 	= 3;
	//dma0_desc_cmd0->cmd_num 	= 0;


	/* Dummy Command Descriptor, cmd_num is 0 */
	dma0_desc_cmd->next_desc 	= (unsigned int)virt_to_phys(dma0_desc0);
	dma0_desc_cmd->databuf 		= 0; 
	dma0_desc_cmd->frame_id 	= (unsigned int)0x0da000cd; /* dma0's cmd0 */
	dma0_desc_cmd->cmd 		= LCD_CMD_CMD | 0; /* dummy command */
	dma0_desc_cmd->cmd_num 		= 0;
	dma0_desc_cmd->offsize 		= 0; 
	dma0_desc_cmd->page_width 	= 0; 

	/* Palette Descriptor */
	dma0_desc_palette->next_desc 	= (unsigned int)virt_to_phys(dma0_desc_cmd0);
#else
	/* Palette Descriptor */
	dma0_desc_palette->next_desc 	= (unsigned int)virt_to_phys(dma0_desc0);
#endif
	dma0_desc_palette->databuf 	= (unsigned int)virt_to_phys((void *)lcd_palette);
	dma0_desc_palette->frame_id 	= (unsigned int)0xaaaaaaaa;
	dma0_desc_palette->cmd 		= LCD_CMD_PAL | pal_size; /* Palette Descriptor */

	/* DMA0 Descriptor0 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) /* TVE mode */
		dma0_desc0->next_desc 	= (unsigned int)virt_to_phys(dma0_desc1);
	else{			/* Normal TFT LCD */
#if defined(CONFIG_FB_JZ4750_SLCD)
			dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd);
			//dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
#else
			dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
#endif
	}

	//maddrone change here
	if(lcd_info->panel.cfg & LCD_CFG_TVEN)
        {
          ;
        }
	else
	{
		dma0_desc0->databuf = virt_to_phys((void *)lcd_frame0);
		dma0_desc0->frame_id = (unsigned int)0x0000da00; /* DMA0'0 */
		//maddrone
		unsigned int frame_size0;
		frame_size0 = (480 * 272 * 16) >> 3;
		frame_size0 /= 4;
		dma0_desc0->cmd = frame_size0;
		dma0_desc0->desc_size = (272 << 16) | 480;
		dma0_desc0->offsize = 0;
		dma0_desc0->cmd_num = 0;
         	}	

	/* DMA0 Descriptor1 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* TVE mode */
		
		dprintf("TV Enable Mode...\n");
		if (lcd_info->osd.fg0.bpp <= 8) /* load palette only once at setup */
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc_palette);
		else
#if defined(CONFIG_FB_JZ4750_SLCD)  /* for smatlcd */
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd);
#else
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
#endif
		dma0_desc1->frame_id = (unsigned int)0x0000da01; /* DMA0'1 */
	}

	if (lcd_info->osd.fg0.bpp <= 8) /* load palette only once at setup */
		REG_LCD_DA0 = virt_to_phys(dma0_desc_palette);
	else {
#if defined(CONFIG_FB_JZ4750_SLCD)  /* for smartlcd */
		REG_LCD_DA0 = virt_to_phys(dma0_desc_cmd0); //smart lcd
		//dprintf("SET REG_LCD_DA0 ======== \n");
		//REG_LCD_DA0 = virt_to_phys(dma0_desc_cmd); //smart lcd
#else
		REG_LCD_DA0 = virt_to_phys(dma0_desc0); //tft
#endif
	}

	/* DMA1 Descriptor0 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) /* TVE mode */
		dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc1);
	else			/* Normal TFT LCD */
		dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc0);

	dma1_desc0->databuf = virt_to_phys((void *)lcd_frame1);
	dma1_desc0->frame_id = (unsigned int)0x0000da10; /* DMA1'0 */

	/* DMA1 Descriptor1 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* TVE mode */
		dma1_desc1->next_desc = (unsigned int)virt_to_phys(dma1_desc0);
		dma1_desc1->frame_id = (unsigned int)0x0000da11; /* DMA1'1 */
	}

	REG_LCD_DA1 = virt_to_phys(dma1_desc0);	/* set Dma-chan1's Descripter Addrress */
	//dma_cache_wback_inv((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz4750_lcd_dma_desc));

#if 0
	/* Palette Descriptor */
	if ( lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD ) 
//		dma0_desc_palette->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd);
		dma0_desc_palette->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd1);
	else
		dma0_desc_palette->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
	dma0_desc_palette->databuf = (unsigned int)virt_to_phys((void *)lcd_palette);
	dma0_desc_palette->frame_id = (unsigned int)0xaaaaaaaa;
	dma0_desc_palette->cmd 	= LCD_CMD_PAL | pal_size; /* Palette Descriptor */

	/* Dummy Command Descriptor, cmd_num is 0 */
	dma0_desc_cmd->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
	dma0_desc_cmd->databuf 	= (unsigned int)virt_to_phys((void *)lcd_cmdbuf);
	dma0_desc_cmd->frame_id = (unsigned int)0x0da0cad0; /* dma0's cmd0 */
	dma0_desc_cmd->cmd 	= LCD_CMD_CMD | 3; /* dummy command */
	dma0_desc_cmd->offsize 	= 0; /* dummy command */
	dma0_desc_cmd->page_width = 0; /* dummy command */
	dma0_desc_cmd->cmd_num 	= 3;

//---------------------------------
	dma0_desc_cmd1->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
	dma0_desc_cmd1->databuf 	= 0; 
	dma0_desc_cmd1->frame_id = (unsigned int)0x0da0cad1; /* dma0's cmd0 */
	dma0_desc_cmd1->cmd 	= LCD_CMD_CMD | 0; /* dummy command */
	dma0_desc_cmd1->cmd_num 	= 0;
	dma0_desc_cmd1->offsize 	= 0; /* dummy command */
	dma0_desc_cmd1->page_width = 0; /* dummy command */
//-----------------------------------
	/* DMA0 Descriptor0 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) /* TVE mode */
		dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc1);
	else{			/* Normal TFT LCD */
		if (lcd_info->osd.fg0.bpp <= 8) /* load palette only once at setup?? */
//			dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc_palette); //tft
			dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd); // smart lcd
		else if ( lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD ) 
			dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd1);
//			dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd);
		else 
			dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
	}

	dma0_desc0->databuf = virt_to_phys((void *)lcd_frame0);
	dma0_desc0->frame_id = (unsigned int)0x0000da00; /* DMA0'0 */

	/* DMA0 Descriptor1 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* TVE mode */
		if (lcd_info->osd.fg0.bpp <= 8) /* load palette only once at setup?? */
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc_palette);
		
		else if ( lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD ) 
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd);
		else 
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
		dma0_desc1->frame_id = (unsigned int)0x0000da01; /* DMA0'1 */
	}

	/* DMA1 Descriptor0 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) /* TVE mode */
		dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc1);
	else			/* Normal TFT LCD */
		dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc0);

	dma1_desc0->databuf = virt_to_phys((void *)lcd_frame1);
	dma1_desc0->frame_id = (unsigned int)0x0000da10; /* DMA1'0 */

	/* DMA1 Descriptor1 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* TVE mode */
		dma1_desc1->next_desc = (unsigned int)virt_to_phys(dma1_desc0);
		dma1_desc1->frame_id = (unsigned int)0x0000da11; /* DMA1'1 */
	}

	if (lcd_info->osd.fg0.bpp <= 8) /* load palette only once at setup?? */
		REG_LCD_DA0 = virt_to_phys(dma0_desc_palette);
	else
//		REG_LCD_DA0 = virt_to_phys(dma0_desc_cmd); //smart lcd
		REG_LCD_DA0 = virt_to_phys(dma0_desc0); //tft
	REG_LCD_DA1 = virt_to_phys(dma1_desc0);	/* set Dma-chan1's Descripter Addrress */
	dma_cache_wback_inv((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz4750_lcd_dma_desc));
#endif
}
static void jz4750fb_set_panel_mode( struct jz4750lcd_info * lcd_info )
{
	struct jz4750lcd_panel_t *panel = &lcd_info->panel;
#ifdef CONFIG_JZ4750D_VGA_DISPLAY
	REG_TVE_CTRL |= TVE_CTRL_DAPD;
	REG_TVE_CTRL &= ~( TVE_CTRL_DAPD1 | TVE_CTRL_DAPD2 | TVE_CTRL_DAPD3);
#endif
	/* set bpp */
	lcd_info->panel.ctrl &= ~LCD_CTRL_BPP_MASK;
	if ( lcd_info->osd.fg0.bpp == 1 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_1;
	else if ( lcd_info->osd.fg0.bpp == 2 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_2;
	else if ( lcd_info->osd.fg0.bpp == 4 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_4;
	else if ( lcd_info->osd.fg0.bpp == 8 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_8;
	else if ( lcd_info->osd.fg0.bpp == 15 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB555;
	else if ( lcd_info->osd.fg0.bpp == 16 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB565;
	else if ( lcd_info->osd.fg0.bpp > 16 && lcd_info->osd.fg0.bpp < 32+1 ) {
		lcd_info->osd.fg0.bpp = 32;
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_18_24;
	}
	else {
		dprintf("The BPP %d is not supported\n", lcd_info->osd.fg0.bpp);
		lcd_info->osd.fg0.bpp = 32;
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_18_24;
	}

	lcd_info->panel.cfg |= LCD_CFG_NEWDES; /* use 8words descriptor always */

	REG_LCD_CTRL = lcd_info->panel.ctrl; /* LCDC Controll Register */
	REG_LCD_CFG = lcd_info->panel.cfg; /* LCDC Configure Register */
	REG_SLCD_CFG = lcd_info->panel.slcd_cfg; /* Smart LCD Configure Register */
	
	if ( lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD ) /* enable Smart LCD DMA */
		REG_SLCD_CTRL = SLCD_CTRL_DMA_EN;

	switch ( lcd_info->panel.cfg & LCD_CFG_MODE_MASK ) {
	case LCD_CFG_MODE_GENERIC_TFT:
	case LCD_CFG_MODE_INTER_CCIR656:
	case LCD_CFG_MODE_NONINTER_CCIR656:
	case LCD_CFG_MODE_SLCD:
	default:		/* only support TFT16 TFT32, not support STN and Special TFT by now(10-06-2008)*/
		REG_LCD_VAT = (((panel->blw + panel->w + panel->elw + panel->hsw)) << 16) | (panel->vsw + panel->bfw + panel->h + panel->efw);
		REG_LCD_DAH = ((panel->hsw + panel->blw) << 16) | (panel->hsw + panel->blw + panel->w);
		REG_LCD_DAV = ((panel->vsw + panel->bfw) << 16) | (panel->vsw + panel->bfw + panel->h);
		REG_LCD_HSYNC = (0 << 16) | panel->hsw;
		REG_LCD_VSYNC = (0 << 16) | panel->vsw;
		break;
	}
}
/**
 * @brief jz4750fb_deep_set_mode 
 *
 * @param lcd_info
 */
static void jz4750fb_deep_set_mode( struct jz4750lcd_info * lcd_info )
{
	/* configurate sequence:
	 * 1. disable lcdc.
	 * 2. init frame descriptor.
	 * 3. set panel mode
	 * 4. set osd mode
	 * 5. start lcd clock in CPM
	 * 6. enable lcdc.
	 */

        dprintf("In jz4750fb_deep_set_mode  \n");
        dprintf("\n");

        int cnt = 336000 * 16;

#define NORMAL_DISABLE 0
#define  QUICH_DISABLE 1
#if QUICH_DISABLE
        __lcd_clr_ena();
        while(!__lcd_quick_disable_done() && cnt > 1) {
          cnt--;
        }
        if (cnt == 1)
          dprintf("LCD quick disable timeout! REG_LCD_STATE=0x%08x\n",REG_LCD_STATE);

#elif  NORMAL_DISABLE
        __lcd_set_dis(); /* regular disable */
        while(!__lcd_disable_done() && cnt > 1) {
          cnt--;
        }
        if (cnt == 1)
          dprintf("LCD disable timeout! REG_LCD_STATE=0x%08x\n",REG_LCD_STATE);

#endif
        //udelay(5000);
        //dprintf("cnt is %d\n",cnt);
	//__slcd_disable_dma();   //maddrone add
	lcd_info->osd.fg_change = FG_CHANGE_ALL; /* change FG0, FG1 size, postion??? */
      jz4750fb_descriptor_init(lcd_info);
      jz4750fb_set_panel_mode(lcd_info);
      jz4750fb_set_mode(lcd_info);
      jz4750fb_change_clock(lcd_info);
	//__slcd_enable_dma();   //maddrone add
	//REG_SLCD_CTRL |= SLCD_CTRL_DMA_EN; //maddrone add
        __lcd_clr_dis();
	__lcd_set_ena();	/* enable lcdc */

}

unsigned short temp_lcd_frame0[BATTERY_NULL_W*BATTERY_NULL_H];
void display_battery_null(void)
{
  unsigned short *p;
  unsigned short *data = battery_null;
  int k = 0,j =0;
  p = (unsigned short *)temp_lcd_frame0;
  //memset(lcd_frame0,0x00,480*272*2);
  flush_cache_all();
  for(k = 0; k < BATTERY_NULL_H; k++)
  {
    for(j = 0; j < BATTERY_NULL_W; j++)
    {
      *p++ = *data++;
    }
  }
  flush_cache_all();
}
static void display_h_color_bar(int w, int h, int bpp) 
{
  dprintf("========== Test H Color BAR Over ============\n");
  dprintf("====%d====== Test H Color BAR Over ============\n",__LINE__);

  int i,j;
  unsigned short *p;
  int k = 0;

  p = (unsigned short *)lcd_frame0;
  dprintf("====%d====== Test H Color BAR Over ============\n",__LINE__);
  if(1)
  {
    for(i=0; i<90; i++)
      for(j=0; j<480; j++)
        p[k++] = 0xF800;
    for(i=0; i<90; i++)
      for(j=0; j<480; j++)
        p[k++] = 0x07E0;

    for(i=0; i<92; i++)
      for(j=0; j<480; j++)
        p[k++] = 0x001F;
  }
  //dma_cache_wback((unsigned int)(lcd_frame0), 400 * 240);
  dprintf("====%d====== Test H Color BAR Over ============\n",__LINE__);
  flush_cache_all();
}
#define __lcd_special_on() \
	do { \
		; \
              } while (0)
#if 0
#define __lcd_set_backlight_level(n)	\
do {					\
	__gpio_as_output(GPIO_LCD_PWM);	\
	__gpio_set_pin(GPIO_LCD_PWM);	\
} while (0)
#else
#define PWM_CHN 2    /* pwm channel */
#define LCD_PWM_FULL 101
/* 100 level: 0,1,...,100 */
#define __lcd_set_backlight_level(n)			\
do {							\
	u32 v = __cpm_get_extalclk() / 50000;		\
	__gpio_as_pwm(2);				\
	__tcu_disable_pwm_output(PWM_CHN);		\
	__tcu_stop_counter(PWM_CHN);			\
	__tcu_init_pwm_output_high(PWM_CHN);		\
	__tcu_set_pwm_output_shutdown_abrupt(PWM_CHN);	\
	__tcu_select_clk_div1(PWM_CHN);			\
	__tcu_mask_full_match_irq(PWM_CHN);		\
	__tcu_mask_half_match_irq(PWM_CHN);		\
	__tcu_set_count(PWM_CHN, 0);			\
	__tcu_set_full_data(PWM_CHN, v + 1);		\
	__tcu_set_half_data(PWM_CHN, v * n / 100);	\
	__tcu_enable_pwm_output(PWM_CHN);		\
	__tcu_select_extalclk(PWM_CHN);			\
	__tcu_start_counter(PWM_CHN);			\
} while (0)


#endif
#define __lcd_display_on() \
do { \
        __lcd_special_on();	\
	__gpio_set_pin(GPIO_LCD_VCC_EN_N);	\
        udelay(3000);\
} while (0)
void turn_lcd_backlight(int i)
{
  __lcd_set_backlight_level(i);
}
void mdelay(int i)
{
  while(i--)
    udelay(1000);
}
void print_lcdc_registers(void)	/* debug */
{
  return;
	/* LCD Controller Resgisters */
	dprintf("REG_LCD_CFG:\t0x%08x\n", REG_LCD_CFG);
	dprintf("REG_LCD_CTRL:\t0x%08x\n", REG_LCD_CTRL);
	dprintf("REG_LCD_STATE:\t0x%08x\n", REG_LCD_STATE);
	dprintf("REG_LCD_OSDC:\t0x%08x\n", REG_LCD_OSDC);
	dprintf("REG_LCD_OSDCTRL:\t0x%08x\n", REG_LCD_OSDCTRL);
	dprintf("REG_LCD_OSDS:\t0x%08x\n", REG_LCD_OSDS);
	dprintf("REG_LCD_BGC:\t0x%08x\n", REG_LCD_BGC);
	dprintf("REG_LCD_KEK0:\t0x%08x\n", REG_LCD_KEY0);
	dprintf("REG_LCD_KEY1:\t0x%08x\n", REG_LCD_KEY1);
	dprintf("REG_LCD_ALPHA:\t0x%08x\n", REG_LCD_ALPHA);
	dprintf("REG_LCD_IPUR:\t0x%08x\n", REG_LCD_IPUR);
	dprintf("REG_LCD_VAT:\t0x%08x\n", REG_LCD_VAT);
	dprintf("REG_LCD_DAH:\t0x%08x\n", REG_LCD_DAH);
	dprintf("REG_LCD_DAV:\t0x%08x\n", REG_LCD_DAV);
	dprintf("REG_LCD_XYP0:\t0x%08x\n", REG_LCD_XYP0);
	dprintf("REG_LCD_XYP1:\t0x%08x\n", REG_LCD_XYP1);
	dprintf("REG_LCD_SIZE0:\t0x%08x\n", REG_LCD_SIZE0);
	dprintf("REG_LCD_SIZE1:\t0x%08x\n", REG_LCD_SIZE1);
	dprintf("REG_LCD_RGBC\t0x%08x\n", REG_LCD_RGBC);
	dprintf("REG_LCD_VSYNC:\t0x%08x\n", REG_LCD_VSYNC);
	dprintf("REG_LCD_HSYNC:\t0x%08x\n", REG_LCD_HSYNC);
	dprintf("REG_LCD_PS:\t0x%08x\n", REG_LCD_PS);
	dprintf("REG_LCD_CLS:\t0x%08x\n", REG_LCD_CLS);
	dprintf("REG_LCD_SPL:\t0x%08x\n", REG_LCD_SPL);
	dprintf("REG_LCD_REV:\t0x%08x\n", REG_LCD_REV);
	dprintf("REG_LCD_IID:\t0x%08x\n", REG_LCD_IID);
	dprintf("REG_LCD_DA0:\t0x%08x\n", REG_LCD_DA0);
	dprintf("REG_LCD_SA0:\t0x%08x\n", REG_LCD_SA0);
	dprintf("REG_LCD_FID0:\t0x%08x\n", REG_LCD_FID0);
	dprintf("REG_LCD_CMD0:\t0x%08x\n", REG_LCD_CMD0);
	dprintf("REG_LCD_OFFS0:\t0x%08x\n", REG_LCD_OFFS0);
	dprintf("REG_LCD_PW0:\t0x%08x\n", REG_LCD_PW0);
	dprintf("REG_LCD_CNUM0:\t0x%08x\n", REG_LCD_CNUM0);
	dprintf("REG_LCD_DESSIZE0:\t0x%08x\n", REG_LCD_DESSIZE0);
	dprintf("REG_LCD_DA1:\t0x%08x\n", REG_LCD_DA1);
	dprintf("REG_LCD_SA1:\t0x%08x\n", REG_LCD_SA1);
	dprintf("REG_LCD_FID1:\t0x%08x\n", REG_LCD_FID1);
	dprintf("REG_LCD_CMD1:\t0x%08x\n", REG_LCD_CMD1);
	dprintf("REG_LCD_OFFS1:\t0x%08x\n", REG_LCD_OFFS1);
	dprintf("REG_LCD_PW1:\t0x%08x\n", REG_LCD_PW1);
	dprintf("REG_LCD_CNUM1:\t0x%08x\n", REG_LCD_CNUM1);
	dprintf("REG_LCD_DESSIZE1:\t0x%08x\n", REG_LCD_DESSIZE1);
	dprintf("==================================\n");
	dprintf("REG_LCD_VSYNC:\t%d:%d\n", REG_LCD_VSYNC>>16, REG_LCD_VSYNC&0xfff);
	dprintf("REG_LCD_HSYNC:\t%d:%d\n", REG_LCD_HSYNC>>16, REG_LCD_HSYNC&0xfff);
	dprintf("REG_LCD_VAT:\t%d:%d\n", REG_LCD_VAT>>16, REG_LCD_VAT&0xfff);
	dprintf("REG_LCD_DAH:\t%d:%d\n", REG_LCD_DAH>>16, REG_LCD_DAH&0xfff);
	dprintf("REG_LCD_DAV:\t%d:%d\n", REG_LCD_DAV>>16, REG_LCD_DAV&0xfff);
	dprintf("==================================\n");

	/* Smart LCD Controller Resgisters */
	dprintf("REG_SLCD_CFG:\t0x%08x\n", REG_SLCD_CFG);
	dprintf("REG_SLCD_CTRL:\t0x%08x\n", REG_SLCD_CTRL);
	dprintf("REG_SLCD_STATE:\t0x%08x\n", REG_SLCD_STATE);
	dprintf("==================================\n");

	
        dprintf("==================================\n");
	if ( dma_desc_base != NULL ) {
		unsigned int * pii = (unsigned int *)dma_desc_base;
		int i, j;
		for (j=0;j< DMA_DESC_NUM ; j++) {
			dprintf("dma_desc%d(0x%08x):\n", j, (unsigned int)pii);
			for (i =0; i<8; i++ ) {
				dprintf("\t\t0x%08x\n", *pii++);
			}
		}
	}
}
void lcd_rinit(void *lcdbase)
{
/* gpio init __gpio_as_lcd */
#if 0
  if (jz4750_lcd_info->panel.cfg & LCD_CFG_MODE_TFT_16BIT)
    __gpio_as_lcd_16bit();
  else if (jz4750_lcd_info->panel.cfg & LCD_CFG_MODE_TFT_24BIT)
  {
    PRINTF_DEBUG;
    __gpio_as_lcd_24bit();
  }
  else
  {
    dprintf("attention  __gpio_as_lcd_8bit\n\n\n\n");
    __gpio_as_lcd_8bit();
  }
  switch (jz4750_lcd_info->panel.cfg & LCD_CFG_MODE_MASK) {
    case LCD_CFG_MODE_SPECIAL_TFT_1:
    case LCD_CFG_MODE_SPECIAL_TFT_2:
    case LCD_CFG_MODE_SPECIAL_TFT_3:
      {
        dprintf("attention __gpio_as_lcd_special \n\n\n");
        __gpio_as_lcd_special();
        break;
      }
    default:
      ;
  }
  if ( jz4750_lcd_info->osd.fg0.bpp > 16 && 
      jz4750_lcd_info->osd.fg0.bpp < 32 ) {
    jz4750_lcd_info->osd.fg0.bpp = 32;
  }
  switch ( jz4750_lcd_info->osd.fg1.bpp ) {
    case 15:
    case 16:
      break;
    case 17 ... 32:
      jz4750_lcd_info->osd.fg1.bpp = 32;
      break;
    default:
      dprintf("jz4750fb fg1 not support bpp(%d), force to 32bpp\n", 
          jz4750_lcd_info->osd.fg1.bpp);
      jz4750_lcd_info->osd.fg1.bpp = 32;
  }
#endif
  __lcd_clr_dis();
  __lcd_clr_ena();
  int cnt = 336000 * 16;

  while(!__lcd_quick_disable_done() && cnt > 1) {
    cnt--;
  }
  if (cnt == 1)
    dprintf("%s: %dLCD quick disable timeout! REG_LCD_STATE=0x%08x\n",__FILE__,__LINE__,REG_LCD_STATE);

#if 0
  /* init clk */
  jz4750fb_change_clock(jz4750_lcd_info);
  __lcd_display_pin_init();
  __gpio_as_output(GPIO_LCD_PWM);	
  __gpio_clear_pin(GPIO_LCD_PWM);	

  //alloc_fb_info
  jz_lcd_init_mem(lcdbase, &panel_info);
#endif
 // jz4750fb_deep_set_mode( jz4750_lcd_info );
  dprintf("In jz4750fb_deep_set_mode  \n");
  dprintf("\n");

  cnt = 336000 * 16;

  __lcd_clr_ena();
  while(!__lcd_quick_disable_done() && cnt > 1) {
    cnt--;
  }
  if (cnt == 1)
    dprintf("LCD quick disable timeout! REG_LCD_STATE=0x%08x\n",REG_LCD_STATE);
  //udelay(5000);
  //dprintf("cnt is %d\n",cnt);
  //__slcd_disable_dma();   //maddrone add
  jz4750_lcd_info->osd.fg_change = FG_CHANGE_ALL; /* change FG0, FG1 size, postion??? */
  jz4750fb_descriptor_init(jz4750_lcd_info);
  //  jz4750fb_set_panel_mode(jz4750_lcd_info);
  jz4750fb_set_mode(jz4750_lcd_info);
  //  jz4750fb_change_clock(jz4750_lcd_info);
  //__slcd_enable_dma();   //maddrone add
  //REG_SLCD_CTRL |= SLCD_CTRL_DMA_EN; //maddrone add
  __lcd_clr_dis();
  __lcd_set_ena();	/* enable lcdc */

  //__lcd_set_ena();	/* enalbe LCD Controller */
  //if(mv < LOW_BATTERY_DATA )
#if 0
  while((mv < LOW_BATTERY_DATA ) &(usb_status == 1))
  {
    usb_status = usb_detect();
    mv =get_battery_mv();
    //dprintf("lcd: mv is %d usb_status is %d\n",mv,usb_status);
    mdelay(500);
  }
#endif
  
    
}
#define MAX_DRAW_STEP 3
void draw_batter_charg(int charge_count)
{
  display_battery_null();
  unsigned short *p;
  unsigned short *data = l009_bootpic_battery_full;
  int k = 0,j =0;
  p = (unsigned short *)temp_lcd_frame0;
  //memset(lcd_frame0,0x00,480*272*2);
  //flush_cache_all();
  int draw_length = charge_count;
  switch (draw_length)
  {
    case 0:
      draw_length = 0;
      break;
    case 1:
      draw_length = BATTERY_NULL_W*5/17;
      break;
    case 2:
      draw_length = BATTERY_NULL_W*5/15;
      break;
    case 3:
      draw_length = BATTERY_NULL_W*5/14;
      break;
 
  }
  for(k = 0; k < BATTERY_NULL_H; k++)
  {
    for(j = 0; j < draw_length; j++)
    {
      *p++ = *data++;
    }
    for(j = 0; j < (BATTERY_NULL_W - draw_length); j++)
    {
      *p++;
      *data++;
    }
  }
  p = (unsigned short *)lcd_frame0;
  data = (unsigned short *)temp_lcd_frame0;
  for(k = 0; k < BATTERY_NULL_H; k++)
  {
    for(j = 0; j < BATTERY_NULL_W; j++)
    {
      *p++ = *data++;
    }
  }
  flush_cache_all();

}

void lcd_ctrl_init (void *lcdbase)
{
  PRINTF_DEBUG;
  /* gpio init __gpio_as_lcd */
  if (jz4750_lcd_info->panel.cfg & LCD_CFG_MODE_TFT_16BIT)
    __gpio_as_lcd_16bit();
  else if (jz4750_lcd_info->panel.cfg & LCD_CFG_MODE_TFT_24BIT)
  {
    PRINTF_DEBUG;
    __gpio_as_lcd_24bit();
  }
  else
  {
    dprintf("attention  __gpio_as_lcd_8bit\n\n\n\n");
    __gpio_as_lcd_8bit();
  }
  switch (jz4750_lcd_info->panel.cfg & LCD_CFG_MODE_MASK) {
    case LCD_CFG_MODE_SPECIAL_TFT_1:
    case LCD_CFG_MODE_SPECIAL_TFT_2:
    case LCD_CFG_MODE_SPECIAL_TFT_3:
      {
        dprintf("attention __gpio_as_lcd_special \n\n\n");
        __gpio_as_lcd_special();
        break;
      }
    default:
      ;
  }
  if ( jz4750_lcd_info->osd.fg0.bpp > 16 && 
      jz4750_lcd_info->osd.fg0.bpp < 32 ) {
    jz4750_lcd_info->osd.fg0.bpp = 32;
  }
  switch ( jz4750_lcd_info->osd.fg1.bpp ) {
    case 15:
    case 16:
      break;
    case 17 ... 32:
      jz4750_lcd_info->osd.fg1.bpp = 32;
      break;
    default:
      dprintf("jz4750fb fg1 not support bpp(%d), force to 32bpp\n", 
          jz4750_lcd_info->osd.fg1.bpp);
      jz4750_lcd_info->osd.fg1.bpp = 32;
  }
  __lcd_clr_dis();
//  __lcd_clr_ena();
  __lcd_clr_ena();
  int cnt = 336000*16;

#if 0
  int cnt = 336000*16;
  while(!__lcd_quick_disable_done() && cnt > 1) {
    cnt--;
  }
  if (cnt == 1)
  {
    dprintf("LCD quick disable timeout! REG_LCD_STATE=0x%08x\n",REG_LCD_STATE);
//retry
retry_lcd:
    PRINTF_DEBUG;
    __lcd_set_ena();
    mdelay(10);
    cnt = 336000*16;
    __lcd_clr_ena();
    while(!__lcd_quick_disable_done() && cnt > 1) {
      cnt--;
    }
    if (cnt == 1)
    {
      dprintf("LCD quick disable timeout! REG_LCD_STATE=0x%08x\n",REG_LCD_STATE);
      goto retry_lcd;
    }

  }
#endif
  PRINTF_DEBUG;

  /* init clk */
  //jz4750fb_change_clock(jz4750_lcd_info);
  __lcd_display_pin_init();
  __gpio_as_output(GPIO_LCD_PWM);	
  __gpio_clear_pin(GPIO_LCD_PWM);	

  //alloc_fb_info
  jz_lcd_init_mem(lcdbase, &panel_info);

  jz4750fb_deep_set_mode( jz4750_lcd_info );
 
  __lcd_set_ena();	/* enalbe LCD Controller */
#if 0 
  extern int get_battery_mv(void);

  int mv =get_battery_mv();
  dprintf("lcd: mv is %d\n",mv);
#endif
#if 1
extern void mdelay(int i);
//#undef LOW_BATTERY_DATA
//#define LOW_BATTERY_DATA 4000
  mdelay(10);
  extern int usb_detect();
  int usb_status = usb_detect();
  PRINTF_DEBUG;
  dprintf("usb_status is %d\n",usb_status);
  //if(mv < LOW_BATTERY_DATA )
  {
    display_battery_null();
    mdelay(50);
    __lcd_display_on();
    //turn_lcd_backlight(100);
    //if(usb_status == 0)
    //  mdelay(3000);
  }
  PRINTF_DEBUG;
#endif
#if 0
  while((mv < LOW_BATTERY_DATA ) &(usb_status == 1))
  {
    usb_status = usb_detect();
    mv =get_battery_mv();
    //dprintf("lcd: mv is %d usb_status is %d\n",mv,usb_status);
    mdelay(500);
  }
#endif
  
  int i  = 1;
#if 0
  while(i--)
  {
      mdelay(1000);

  }
#endif
  PRINTF_DEBUG;

  lcd_rinit(lcd_base);
  PRINTF_DEBUG;
  //handler the battery low
  extern int get_battery_mv(void);

  int mv =get_battery_mv();
  dprintf("lcd: mv is %d\n",mv);
  if(mv < LOW_BATTERY_DATA)
  {
    //display_battery_null();
    draw_batter_charg(0);
    mdelay(500);
    __lcd_display_on();
    turn_lcd_backlight(10);
    mdelay(3000);
  }
  int charge_count = 0;
  while(mv < LOW_BATTERY_DATA  && usb_status == 1)
  {
    mv =get_battery_mv();
    usb_status = usb_detect();
    dprintf("%s %d %d %d %d \n",__FILE__,__LINE__,mv,usb_status,charge_count);
    mdelay(500);
    draw_batter_charg(charge_count);
    charge_count++;
    if(charge_count > MAX_DRAW_STEP)
      charge_count = 0;
    mdelay(500);
  }
  if(mv < LOW_BATTERY_DATA)
  {
     dprintf("do hibernate\n");
    extern int power_off_ub(void);
    power_off_ub();
  }
#if 0
    i  = 1;
  while(i--)
  {
    mdelay(1000);

  }
#endif
   PRINTF_DEBUG;

  __gpio_as_output(GPIO_LCD_PWM);	
  __gpio_clear_pin(GPIO_LCD_PWM);	
//  test_lcd_init(lcd_base);
  if(0)//(mv < LOW_BATTERY_DATA)
  {
    //do hibernate
    dprintf("do hibernate\n");
    extern int power_off_ub(void);
    power_off_ub();
  }
  //__gpio_clear_pin(GPIO_LCD_VCC_EN_N);	
  //display_battery_null();
}

/*----------------------------------------------------------------------*/
#if LCD_BPP == LCD_COLOR8
void
lcd_setcolreg (ushort regno, ushort red, ushort green, ushort blue)
{
}
#endif
/*----------------------------------------------------------------------*/

#if LCD_BPP == LCD_MONOCHROME
static
void lcd_initcolregs (void)
{
}
#endif

/*----------------------------------------------------------------------*/

/*
 * Before enabled lcd controller, lcd registers should be configured correctly.
 */

void lcd_enable (void)
{
	__lcd_clr_dis();
	__lcd_set_ena();
}

/*----------------------------------------------------------------------*/


void lcd_disable (void)
{
	__lcd_set_dis();
	/* __lcd_clr_ena(); */  /* quikly disable */
}

/************************************************************************/


static int jz_lcd_init_mem(void *lcdbase, vidinfo_t *vid)
{
	u_long palette_mem_size;
	struct jz_fb_info *fbi = &vid->jz_fb;
	int fb_size = vid->vl_row * (vid->vl_col * NBITS (vid->vl_bpix)) / 8;

        fbi->screen = (u_long)lcdbase;
        lcd_frame0 = (u_long)lcdbase;
	fbi->palette_size = 256;
	palette_mem_size = fbi->palette_size * sizeof(u16);

	debug("palette_mem_size = 0x%08lx\n", (u_long) palette_mem_size);
	/* locate palette and descs at end of page following fb */
        fbi->palette = (u_long)lcdbase + fb_size + PAGE_SIZE - palette_mem_size;
        lcd_palette =  fbi->palette;
        dma_desc_base = (struct jz4750_lcd_dma_desc *)((void*)lcd_palette + ((palette_mem_size+3)/4)*4);
//        jz4750fb_descriptor_init(jz4750_lcd_info);
        int j = 0,i = 0;
        if ( dma_desc_base != NULL ) {
          unsigned int * pii = (unsigned int *)dma_desc_base;
          for (j=0;j< DMA_DESC_NUM ; j++) {
            dprintf("dma_desc%d(0x%08x):\n", j, (unsigned int)pii);
            for (i =0; i<8; i++ ) {
              *pii++ = 0;
              //dprintf("\t\t0x%08x\n", *pii++);
            }
          }
        }
        return 0;
}
#if 0
static void jz_lcd_desc_init(vidinfo_t *vid)
{
	struct jz_fb_info * fbi;
	fbi = &vid->jz_fb;
	fbi->dmadesc_fblow = (struct jz_fb_dma_descriptor *)((unsigned int)fbi->palette - 3*32);
	fbi->dmadesc_fbhigh = (struct jz_fb_dma_descriptor *)((unsigned int)fbi->palette - 2*32);
	fbi->dmadesc_palette = (struct jz_fb_dma_descriptor *)((unsigned int)fbi->palette - 1*32);


//	#define BYTES_PER_PANEL	 (vid->vl_col * vid->vl_row * NBITS(vid->vl_bpix) / 8)
#define BYTES_PER_PANEL	 (((vid->vl_col * NBITS(vid->vl_bpix) / 8 + 3) >> 2 << 2) * vid->vl_row)

	/* populate descriptors */
	fbi->dmadesc_fblow->fdadr = virt_to_phys(fbi->dmadesc_fblow);
	fbi->dmadesc_fblow->fsadr = virt_to_phys((void *)(fbi->screen + BYTES_PER_PANEL));
	fbi->dmadesc_fblow->fidr  = 1;
	fbi->dmadesc_fblow->ldcmd = BYTES_PER_PANEL / 4 ;
	fbi->dmadesc_fblow->offsize = 0;
	fbi->dmadesc_fblow->page_width = 0;
	fbi->dmadesc_fblow->desc_size = jzfb.osd.fg1.h << 16 | jzfb.osd.fg1.w;
	REG_LCD_SIZE1 = (jzfb.osd.fg1.h<<16)|jzfb.osd.fg1.w;

	fbi->fdadr1 = virt_to_phys(fbi->dmadesc_fblow); /* only used in dual-panel mode */

	fbi->dmadesc_fbhigh->fsadr = virt_to_phys((void *)fbi->screen); 
	fbi->dmadesc_fbhigh->fidr = 0;
	fbi->dmadesc_fbhigh->ldcmd =  BYTES_PER_PANEL / 4; /* length in word */
	fbi->dmadesc_fbhigh->offsize = 0;
	fbi->dmadesc_fbhigh->page_width = 0;
	fbi->dmadesc_fbhigh->desc_size = jzfb.osd.fg0.h << 16 | jzfb.osd.fg0.w;
	REG_LCD_SIZE0 = jzfb.osd.fg0.h << 16|jzfb.osd.fg0.w;

	fbi->dmadesc_palette->fsadr = virt_to_phys((void *)fbi->palette);
	fbi->dmadesc_palette->fidr  = 0;
	fbi->dmadesc_palette->ldcmd = (fbi->palette_size * 2)/4 | (1<<28);

	if( NBITS(vid->vl_bpix) < 12)
	{
		/* assume any mode with <12 bpp is palette driven */
		fbi->dmadesc_palette->fdadr = virt_to_phys(fbi->dmadesc_fbhigh);
		fbi->dmadesc_fbhigh->fdadr = virt_to_phys(fbi->dmadesc_palette);
		/* flips back and forth between pal and fbhigh */
		fbi->fdadr0 = virt_to_phys(fbi->dmadesc_palette);
	}
	else
	{
		/* palette shouldn't be loaded in true-color mode */
		fbi->dmadesc_fbhigh->fdadr = virt_to_phys((void *)fbi->dmadesc_fbhigh);
		fbi->dmadesc_fblow->fdadr = virt_to_phys((void *)fbi->dmadesc_fblow);
		fbi->fdadr0 = virt_to_phys(fbi->dmadesc_fbhigh); /* no pal just fbhigh */
		fbi->fdadr1 = virt_to_phys(fbi->dmadesc_fblow); /* just fblow */
	}
//	print_lcdc_desc(fbi);
	flush_cache_all();
//	print_lcdc_desc(fbi);
}
#endif

#if defined(CONFIG_JZ4750D)
static int lcd_open(void)
{
 	unsigned int enable = 1, i;

	if(!(REG_LCD_CTRL & LCD_CTRL_ENA))
		REG_LCD_CTRL |= LCD_CTRL_ENA;

	for (i = 0; i < 10; i++) {
		if (!(REG_LCD_CTRL & LCD_CTRL_ENA)) {
			enable = 0;
			break;
		}
	}

	if (enable == 0) {
//		dprintf("Put CPU into hibernate mode.\n");
		jz_sync();
		REG_RTC_RCR = RTC_RCR_AIE | RTC_RCR_AE | RTC_RCR_RTCE;
		REG_RTC_HWRSR = 0;
		REG_RTC_HSPR = 0x12345678;
		REG_RTC_RSAR = REG_RTC_RSR + 2;
		REG_RTC_HWCR |= RTC_HWCR_EALM;
		REG_RTC_HRCR |= 0x0fe0;
		REG_RTC_HWFCR |= (0x00FF << 4);
	   	REG_RTC_HCR |= RTC_HCR_PD;
	}
	else {
		if (jzfb.panel.cfg & LCD_CFG_RECOVER) {
			REG_LCD_CTRL &= ~LCD_CTRL_ENA;
			REG_LCD_CFG |= LCD_CFG_RECOVER;
			REG_LCD_CTRL |= LCD_CTRL_ENA;
		}
	}
	return 0;
}
#endif
#if 0
static int  jz_lcd_hw_init(vidinfo_t *vid)
{
	struct jz_fb_info *fbi = &vid->jz_fb;
	unsigned int val = 0;
	unsigned int pclk;


	/* Setting Control register */
	val = jzfb.panel.ctrl;

	switch (vid->vl_bpix) {
	case 0:
		val |= LCD_CTRL_BPP_1;
		break;
	case 1:
		val |= LCD_CTRL_BPP_2;
		break;
	case 2:
		val |= LCD_CTRL_BPP_4;
		break;
	case 3:
		val |= LCD_CTRL_BPP_8;
		break;
	case 4:
		val |= LCD_CTRL_BPP_16;
		break;
	case 5:
		val |= LCD_CTRL_BPP_18_24;	/* target is 4bytes/pixel */
		break;
	default:
		dprintf("The BPP %d is not supported\n", 1 << panel_info.vl_bpix);
		val |= LCD_CTRL_BPP_18_24;
		break;
	}


//	val |= LCD_CTRL_BST_16;		/* Burst Length is 16WORD=64Byte */
//	val |= LCD_CTRL_OFUP;		/* OutFIFO underrun protect */

	jzfb.panel.ctrl = val;
	REG_LCD_CTRL = val;

	switch ( jzfb.panel.cfg & LCD_CFG_MODE_MASK ) {
	case LCD_CFG_MODE_GENERIC_TFT:
	case LCD_CFG_MODE_INTER_CCIR656:
	case LCD_CFG_MODE_NONINTER_CCIR656:
	case LCD_CFG_MODE_SLCD:
	default:		/* only support TFT16 TFT32, not support STN and Special TFT by now(10-06-2008)*/
		REG_LCD_VAT = (((jzfb.panel.blw + jzfb.panel.w + jzfb.panel.elw + jzfb.panel.hsw)) << 16) | (jzfb.panel.vsw + jzfb.panel.bfw + jzfb.panel.h + jzfb.panel.efw);
		REG_LCD_DAH = ((jzfb.panel.hsw + jzfb.panel.blw) << 16) | (jzfb.panel.hsw + jzfb.panel.blw + jzfb.panel.w);
		REG_LCD_DAV = ((jzfb.panel.vsw + jzfb.panel.bfw) << 16) | (jzfb.panel.vsw + jzfb.panel.bfw + jzfb.panel.h);
		REG_LCD_HSYNC = (0 << 16) | jzfb.panel.hsw;
		REG_LCD_VSYNC = (0 << 16) | jzfb.panel.vsw;
		break;
	}
	/* Configure the LCD panel */
	REG_LCD_CFG = jzfb.panel.cfg;
	REG_LCD_OSDCTRL = jzfb.osd.osd_ctrl; /* IPUEN, bpp */
	REG_LCD_RGBC  	= jzfb.osd.rgb_ctrl;
	REG_LCD_BGC  	= jzfb.osd.bgcolor;
	REG_LCD_KEY0 	= jzfb.osd.colorkey0;
	REG_LCD_KEY1 	= jzfb.osd.colorkey1;
	REG_LCD_ALPHA 	= jzfb.osd.alpha;
	REG_LCD_IPUR 	= jzfb.osd.ipu_restart;

	/* Timing setting */
	__cpm_stop_lcd();

	val = jzfb.panel.fclk; /* frame clk */
	if ( (jzfb.panel.cfg & LCD_CFG_MODE_MASK) != LCD_CFG_MODE_SERIAL_TFT) {
		pclk = val * (jzfb.panel.w + jzfb.panel.hsw + jzfb.panel.elw + jzfb.panel.blw) * (jzfb.panel.h + jzfb.panel.vsw + jzfb.panel.efw + jzfb.panel.bfw); /* Pixclk */
	}
	else {
		/* serial mode: Hsync period = 3*Width_Pixel */
		pclk = val * (jzfb.panel.w*3 + jzfb.panel.hsw + jzfb.panel.elw + jzfb.panel.blw) * (jzfb.panel.h + jzfb.panel.vsw + jzfb.panel.efw + jzfb.panel.bfw); /* Pixclk */
	}

	val = __cpm_get_pllout2() / pclk; /* pclk */
	val--;
	if ( val > 0x7ff ) {
		dprintf("pixel clock divid is too large, set it to 0x7ff\n");
		val = 0x7ff;
	}
	
	__cpm_set_pixdiv(val);
#if defined(CONFIG_JZ4750)	
	val = pclk * 3 ;	/* LCDClock > 2.5*Pixclock */
	val =__cpm_get_pllout2() / val;
	if ( val > 0x1f ) {
		dprintf("lcd clock divide is too large, set it to 0x1f\n");
		val = 0x1f;
	}
	__cpm_set_ldiv( val );
#endif
	REG_CPM_CPCCR |= CPM_CPCCR_CE ; /* update divide */

	__cpm_start_lcd();
	udelay(1000);

	REG_LCD_DA0 = fbi->fdadr0; /* foreground 0 descripter*/
	REG_LCD_DA1 = fbi->fdadr1; /* foreground 1 descripter*/
	return 0;
}
#endif

#endif /* CONFIG_LCD */
#endif //CONFIG_JZ4750
