/*
 * (C) Copyright 2006
 * Ingenic Semiconductor, <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <command.h>
#include <asm/mipsregs.h>
#include <asm/jz4750d.h>

#define ADD_BOOTPIC 
#ifdef ADD_BOOTPIC 
#include "bootpic1.h"
#if 0
//#include "bootpic2.h"
#include "bootpic3.h"
//#include "bootpic4.h"
#include "bootpic5.h"
//#include "bootpic6.h"
#include "bootpic7.h"
//#include "bootpic8.h"
#include "bootpic9.h"
//#include "bootpic10.h"
#include "bootpic11.h"
//#include "bootpic12.h"
#include "bootpic13.h"
//#include "bootpic14.h"
#include "bootpic15.h"
#endif
#endif


#if defined(CFG_CHIP_COUNT)
extern int chip_count( void );

static int hs = 6;
inline int get_cpu_speed(void)
{
	unsigned int speed, cfg;

	/* set gpio as input??? */
	cfg = (REG_GPIO_PXPIN(2) >> 11) & 0x7; /* read GPC11,GPC12,GPC13 */
	switch (cfg) {
	case 0:
		speed = 336000000;
		break;
	case 1:
		speed = 392000000;
		break;
	case 2:
		speed = 400000000;
		break;
	case 3:
		speed = 180000000;
		break;
	case 4:
		speed = 410000000;
		break;
	default:
		speed = 420000000; /* default speed */
		break;
	}

	return speed;
}

/* PLL output clock = EXTAL * NF / (NR * NO)
 *
 * NF = FD + 2, NR = RD + 2
 * NO = 1 (if OD = 0), NO = 2 (if OD = 1 or 2), NO = 4 (if OD = 3)
 */
static void pll_init(void)
{
	register unsigned int cfcr, plcr1;
	int n2FR[33] = {
		0, 0, 1, 2, 3, 0, 4, 0, 5, 0, 0, 0, 6, 0, 0, 0,
		7, 0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0,
		9
	};
	int div[5] = {1, hs, hs, hs, hs}; /* divisors of I:S:P:L:M */
	int nf, pllout2;

	cfcr = 	CPM_CPCCR_PCS |
		(n2FR[div[0]] << CPM_CPCCR_CDIV_BIT) | 
		(n2FR[div[1]] << CPM_CPCCR_HDIV_BIT) | 
		(n2FR[div[2]] << CPM_CPCCR_PDIV_BIT) |
		(n2FR[div[3]] << CPM_CPCCR_MDIV_BIT) |
		(n2FR[div[4]] << CPM_CPCCR_H1DIV_BIT);

	if (CFG_EXTAL > 16000000)
		cfcr |= CPM_CPCCR_ECS;
	else
		cfcr &= ~CPM_CPCCR_ECS;

	pllout2 = (cfcr & CPM_CPCCR_PCS) ? get_cpu_speed() : (get_cpu_speed() / 2);

	nf = get_cpu_speed()  / 1000000;

//	nf = get_cpu_speed() * 2 / CFG_EXTAL;
	plcr1 = ((nf - 2) << CPM_CPPCR_PLLM_BIT) | /* FD */
		(22 << CPM_CPPCR_PLLN_BIT) |	/* RD=0, NR=2 */
		(0 << CPM_CPPCR_PLLOD_BIT) |    /* OD=0, NO=1 */
		(0x20 << CPM_CPPCR_PLLST_BIT) | /* PLL stable time */
		CPM_CPPCR_PLLEN;                /* enable PLL */          

	/* init PLL */
	REG_CPM_CPCCR = cfcr;
	REG_CPM_CPPCR = plcr1;
}

#endif
static void gpio_init(void)
{
	/*
	 * Initialize SDRAM pins
	 */
#if CONFIG_NR_DRAM_BANKS == 2   /*Use Two Banks SDRAM*/
	__gpio_as_sdram_x2_32bit();
#else
	__gpio_as_sdram_32bit();
#endif

	/*
	 * Initialize lcd pins
	 */
	//__gpio_as_lcd_18bit();

	/*
	 * Initialize UART1 pins
	 */
	__gpio_as_uart1();
	__gpio_as_i2c();
	__cpm_start_i2c();
}
int power_off_ub(void)
{
	//printf("Put CPU into hibernate mode.\n");

	/* Mask all interrupts */
	REG_INTC_IMSR = 0xffffffff;

	/* 
	 * RTC Wakeup or 1Hz interrupt can be enabled or disabled 
	 * through  RTC driver's ioctl (linux/driver/char/rtc_jz.c).
	 */

	/* Set minimum wakeup_n pin low-level assertion time for wakeup: 100ms */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HWFCR = (100 << RTC_HWFCR_BIT);

	/* Set reset pin low-level assertion time after wakeup: must  > 60ms */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HRCR = (60 << RTC_HRCR_BIT); /* 60 ms */
        while (!(REG_RTC_RCR & RTC_RCR_WRDY));
        printf("%s %d REG_RTC_HRCR is 0x%x\n",__FUNCTION__,__LINE__,REG_RTC_HRCR);
        while (!(REG_RTC_RCR & RTC_RCR_WRDY));
        printf("%s %d REG_RTC_HRCR is 0x%x\n",__FUNCTION__,__LINE__,REG_RTC_HRCR);

	/* Scratch pad register to be reserved */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HSPR = 0x12345678;

	/* clear wakeup status register */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HWRSR = 0x0;
#undef RTC_ALARM_WAKEUP
#ifdef RTC_ALARM_WAKEUP
#define ENABLE_RTC 1
        while ( !__rtc_write_ready() ) ; /* set wakeup alarm enable */
        if ( ENABLE_RTC != (REG_RTC_HWCR & 0x1) ) {
          while ( !__rtc_write_ready() ) ;
          REG_RTC_HWCR = (REG_RTC_HWCR & ~0x1) | ENABLE_RTC;
        }
        while ( !__rtc_write_ready() ) ; /* set alarm function */
        if ( ENABLE_RTC != ((REG_RTC_RCR>>2) & 0x1) ) {
          while ( !__rtc_write_ready() ) ;
          REG_RTC_RCR = (REG_RTC_RCR & ~(1<<2)) | (ENABLE_RTC<<2);
        }

        while ( !__rtc_write_ready() ) ;
        if ( !(REG_RTC_RCR & RTC_RCR_AIE) ) { /* Enable alarm irq */
		__rtc_enable_alarm_irq();
	}
        while ( !__rtc_write_ready() ) ;
        REG_RTC_RSR = 0;

        while ( !__rtc_write_ready() ) ;
        REG_RTC_RSAR = 10;

        while ( !__rtc_write_ready() ) ; /* set alarm function */
        if ( !((REG_RTC_RCR>>2) & 0x1) ) {
          while ( !__rtc_write_ready() ) ;
          __rtc_enable_alarm();
        }

        while ( !__rtc_write_ready() ) ;
        if ( !(REG_RTC_RCR & RTC_RCR_AIE) ) { /* Enable alarm irq */
          __rtc_enable_alarm_irq();
        }
#if 0
        int i = 10000;
        while(i--)
        {
          while ( !__rtc_write_ready() ) ;
          printk("REG_RTC_RSR is %d REG_RTC_RSAR is %d i is %d\n",REG_RTC_RSR,REG_RTC_RSAR,i);
          mdelay(500);
        }
#endif
#endif

	/* Put CPU to power down mode */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HCR = RTC_HCR_PD;

	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	while(1);

	/* We can't get here */
	return 0;
}
void sadc_init_clock(int div)
{
	if (div < 2) div = 2;
	if (div > 23) div = 23;

#if defined(CONFIG_SOC_JZ4750) || defined(CONFIG_SOC_JZ4750D)
	REG_SADC_ADCLK &= ~SADC_ADCLK_CLKDIV_MASK;
	REG_SADC_ADCLK |= (div - 1) << SADC_ADCLK_CLKDIV_BIT;
	REG_SADC_ADCLK &= ~SADC_ADCLK_CLKDIV_BIT;
	REG_SADC_ADCLK |= 39 << SADC_ADCLK_CLKDIV_10_BIT;  /* if div ==3,here is 39 */
#endif
}
void start_pbat_adc(void)
{
	//REG_SADC_CFG |= SADC_CFG_PBAT_HIGH;/* PBAT >= 2.5V */
        REG_SADC_CFG |= SADC_CFG_PBAT_LOW; /* PBAT < 2.5V */

	REG_SADC_ENA |= SADC_ENA_PBATEN;
}
unsigned int jz4740_read_battery(void)
{
	unsigned int ret;
	unsigned short pbat;

//	if(!(REG_SADC_STATE & SADC_STATE_PBATRDY) == 1)
	start_pbat_adc();
		//printf("l009 test 111\n");
		//udelay(1000000);
		//printf("l009 test 222\n");
	
	while(!(REG_SADC_STATE & SADC_STATE_PBATRDY))
	//printf("+++++++++++wait here for data ready++++++++++++++++\n");		

	pbat = REG_SADC_BATDAT;
	ret = pbat & 0x0fff;

        REG_SADC_STATE = SADC_STATE_PBATRDY; // 
        REG_SADC_CTRL &= ~SADC_CTRL_PBATRDYM; 
	
	return ret;
}
#define GPIO_USB_DETE		(32 * 4 + 6)
int usb_detect()
{
	__gpio_as_input(GPIO_USB_DETE);
	__gpio_disable_pull(GPIO_USB_DETE);

	if(__gpio_get_pin(GPIO_USB_DETE))
		return 1;

	return 0;
}


//----------------------------------------------------------------------
// board early init routine

#ifdef ADD_BOOTPIC 
void ready_bootpic_mem(void)
{
  short *p = 0x83000000;
  int i;
  //printf("%s %d\n",__FILE__,__LINE__);
  for( i = 0; i < 480*272; i++)
  {
    *p = l009_bootpic1[i];
    p++;
  }
  const unsigned short carlos_sexi[8] = {0x23,0x83,0xe1,0x38,0x24,0xf0,0x23,0x45};
  for(  i = 0; i < 8; i += 1)
  {
    *p++ = carlos_sexi[i];
  }

  //printf("%s %d\n",__FILE__,__LINE__);
  flush_cache_all();
  return;

 
#if 0
  for( i = 0; i < 400*240; i++)
  {
    *p = l009_bootpic2[i];
    p++;
  }
#endif
#if 0
  for( i = 0; i < 400*240; i++)
  {
    *p = l009_bootpic3[i];
    p++;
  }
  printf("%s %d\n",__FILE__,__LINE__);
 
#if 0
  for( i = 0; i < 400*240; i++)
  {
    *p = l009_bootpic4[i];
    p++;
  }
#endif
  for( i = 0; i < 400*240; i++)
  {
    *p = l009_bootpic5[i];
    p++;
  }
printf("%s %d\n",__FILE__,__LINE__);
 
#if 0
  for( i = 0; i < 400*240; i++)
  {
    *p = l009_bootpic6[i];
    p++;
  }
#endif
  for( i = 0; i < 400*240; i++)
  {
    *p = l009_bootpic7[i];
    p++;
  }
printf("%s %d\n",__FILE__,__LINE__);
 
#if 0
  for( i = 0; i < 400*240; i++)
  {
    *p = l009_bootpic8[i];
    p++;
  }
#endif
  for( i = 0; i < 400*240; i++)
  {
    *p = l009_bootpic9[i];
    p++;
  }
printf("%s %d\n",__FILE__,__LINE__);
 
#if 0
  for( i = 0; i < 400*240; i++)
  {
    *p = l009_bootpic10[i];
    p++;
  }
#endif
  for( i = 0; i < 400*240; i++)
  {
    *p = l009_bootpic11[i];
    p++;
  }
printf("%s %d\n",__FILE__,__LINE__);
 
#if 0
  for( i = 0; i < 400*240; i++)
  {
    *p = l009_bootpic12[i];
    p++;
  }
#endif
  for( i = 0; i < 400*240; i++)
  {
    *p = l009_bootpic13[i];
    p++;
  }
printf("%s %d\n",__FILE__,__LINE__);
 
#if 0
  for( i = 0; i < 400*240; i++)
  {
    *p = l009_bootpic14[i];
    p++;
  }
#endif
  for( i = 0; i < 400*240; i++)
  {
    *p = l009_bootpic15[i];
    p++;
  }
  printf("%s %d\n",__FILE__,__LINE__);
 
  flush_cache_all();
#endif
}
#endif
unsigned int mv = 0;
int usb_detect_flag = 0;
int read_count = 0;
#define COUNT_TIMES 5
unsigned int get_battery_val(void)
{
  unsigned int bat_cap;
  bat_cap = jz4740_read_battery();
  read_count = 0;
  int j = 0;
  while(bat_cap == 0)
  {
    j = 1000;
    while(j--);
    read_count++;
    bat_cap = jz4740_read_battery();
  }
  bat_cap = (bat_cap*10000/4096)+80;
  return bat_cap;
}
unsigned int mv_array_1[COUNT_TIMES+1] ={0};
unsigned int mv_array_2[COUNT_TIMES+1] ={0};
unsigned int get_first_time_battery(void)
{
  int i = 0;
  mv = 0;
  for(i = 0; i <COUNT_TIMES; i++)
  {
    mv_array_1[i] = get_battery_val();
    mv += mv_array_1[i];
  }
  unsigned int date_diff1 = 0;
  mv_array_1[COUNT_TIMES] = mv/COUNT_TIMES;
  date_diff1 = 0;
  for(i = 0; i <COUNT_TIMES; i++)
  {
    if(mv_array_1[i] >= mv_array_1[COUNT_TIMES])
      date_diff1 += mv_array_1[i] - mv_array_1[COUNT_TIMES];
    else
      date_diff1 += mv_array_1[COUNT_TIMES] - mv_array_1[i];
  }
  return date_diff1;

}
unsigned int get_second_time_battery(void)
{
  unsigned int date_diff2 = 0;
  int i;
  mv = 0;
  for(i = 0; i <COUNT_TIMES; i++)
  {
    mv_array_2[i] = get_battery_val();
    mv += mv_array_2[i];
  }
  mv_array_2[COUNT_TIMES] = mv/COUNT_TIMES;
  date_diff2 = 0;
  for(i = 0; i <COUNT_TIMES; i++)
  {
    if(mv_array_2[i] >= mv_array_2[COUNT_TIMES])
      date_diff2 += mv_array_2[i] - mv_array_2[COUNT_TIMES];
    else
      date_diff2 += mv_array_2[COUNT_TIMES] - mv_array_2[i];
  }
  return date_diff2;
}
unsigned int date_diff1,date_diff2;
int get_battery_mv(void)
{
  mv = 0;
  date_diff1 = get_first_time_battery();
  date_diff2 = get_second_time_battery();

  if(date_diff1 >= date_diff2)
    mv = mv_array_2[COUNT_TIMES];
  else
    mv = mv_array_1[COUNT_TIMES];
  return mv;
}
void board_early_init(void)
{
  gpio_init();
  sadc_init_clock(3);
//  mv = get_battery_val();
  int i;
  mv = 0;
  date_diff1 = get_first_time_battery();
  date_diff2 = get_second_time_battery();
  
  if(date_diff1 >= date_diff2)
    mv = mv_array_2[COUNT_TIMES];
  else
    mv = mv_array_1[COUNT_TIMES];
}
//	pll_init();
//----------------------------------------------------------------------
// U-Boot common routines
static void led_flush_test(int level)
{
  return;
  #define LCD_BACKLIGHT_PIN 32*4+22
  __gpio_as_output(LCD_BACKLIGHT_PIN);
  __gpio_enable_pull(LCD_BACKLIGHT_PIN);
  int i = 0;
#define FLUSH_TIMES 5
  //for(i = 0; i < FLUSH_TIMES; i++)
  {
    //__gpio_clear_pin(LCD_BACKLIGHT_PIN);
    //delay_1S();
    if(1 == level)
      __gpio_set_pin(LCD_BACKLIGHT_PIN);
    if(0 == level)
      __gpio_clear_pin(LCD_BACKLIGHT_PIN);
    //delay_1S();
  }
}
void printf_test_rtc(void)
{
	printf(" %s \n",__FUNCTION__);

	/* 
	 * RTC Wakeup or 1Hz interrupt can be enabled or disabled 
	 * through  RTC driver's ioctl (linux/driver/char/rtc_jz.c).
	 */
        while (!(REG_RTC_RCR & RTC_RCR_WRDY));
        printf("REG_RTC_RCR is 0x%x\n",REG_RTC_RCR);
#define WAKEUP_PIN_PLUSE 100
	/* Set minimum wakeup_n pin low-level assertion time for wakeup: 100ms */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
        printf("REG_RTC_HWFCR is 0x%x\n",REG_RTC_HWFCR);
        //check the date
        while (!(REG_RTC_RCR & RTC_RCR_WRDY));
#define RESET_PIN_ASSERTION_TIME 127

	/* Set reset pin low-level assertion time after wakeup: must  > 60ms */
        printf("REG_RTC_HRCR is 0x%x\n",REG_RTC_HRCR);

	/* Scratch pad register to be reserved */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
        printf("REG_RTC_HSPR is 0x%x\n",REG_RTC_HSPR);

        while (!(REG_RTC_RCR & RTC_RCR_WRDY));
        printf("REG_RTC_HWCR is 0x%x\n",REG_RTC_HWCR);

        /* clear wakeup status register */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
        printf("REG_RTC_HWRSR is 0x%x\n",REG_RTC_HWRSR);
#if 0


        //set alarm hibernate reset
        while (!(REG_RTC_RCR & RTC_RCR_WRDY));
        REG_RTC_RCR |= 0x0d;
        while (!(REG_RTC_RCR & RTC_RCR_WRDY));
        REG_RTC_RSR = 0;
        while (!(REG_RTC_RCR & RTC_RCR_WRDY));
        REG_RTC_RSAR |= 100000;
        while (!(REG_RTC_RCR & RTC_RCR_WRDY));
        REG_RTC_HWCR = 0x01;
#endif
        
        while ( !__rtc_write_ready() ) ;
        printf("REG_RTC_RSR is 0x%x\n",REG_RTC_RSR);

        while ( !__rtc_write_ready() ) ;
        printf("REG_RTC_RSAR is 0x%x\n",REG_RTC_RSAR);
	return 0;

}
int checkboard (void)
{
	DECLARE_GLOBAL_DATA_PTR;
	printf("%s Board: Ingenic CETUS (CPU Speed %d MHz)\n",__TIME__,
	       gd->cpu_clk/1000000);

#ifdef ADD_BOOTPIC 
        ready_bootpic_mem();
#endif
        return 0;

        printf(" %s %s mv is %d read_count is %d\n",__TIME__,__FUNCTION__,mv,read_count);
        //while (!(REG_RTC_RCR & RTC_RCR_WRDY));
        //printf("REG_RTC_HWFCR is 0x%x\n",REG_RTC_HWFCR);
        //printf_test_rtc(); 
        //led_flush_test(1);  
        if(mv < 3350 )
          power_off_ub();
       display_battery_null();
        extern void turn_lcd_backlight(int i);
        turn_lcd_backlight(100);
        while(1);
        return 0; /* success */
}

