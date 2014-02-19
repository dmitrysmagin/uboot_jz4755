/*
 * Driver for NAND support, Rick Bronson
 * borrowed heavily from:
 * (c) 1999 Machine Vision Holdings, Inc.
 * (c) 1999, 2000 David Woodhouse <dwmw2@infradead.org>
 *
 * Added 16-bit nand support
 * (C) 2004 Texas Instruments
 */

#include <common.h>

#include <asm/io.h>
#include <command.h>

#include <asm/jzsoc.h>

#define u32 unsigned int
#define u16 unsigned short
#define u8 unsigned char
static int rca;
static int highcap = 0;

/*
 * GPIO definition
 */
#define MMC_IRQ_MASK()				\
	do {						\
		REG_MSC_IMASK = 0xffff;			\
		REG_MSC_IREG = 0xffff;			\
	} while (0)

/* Stop the MMC clock and wait while it happens */
static inline int jz_mmc_stop_clock(void)
{
	int timeout = 1000;

	REG_MSC_STRPCL = MSC_STRPCL_CLOCK_CONTROL_STOP;

	while (timeout && (REG_MSC_STAT & MSC_STAT_CLK_EN)) {
		timeout--;
		if (timeout == 0) {
			return -1;
		}
		udelay(1);
	}
	return 0;
}

/* Start the MMC clock and operation */
static inline int jz_mmc_start_clock(void)
{
	REG_MSC_STRPCL = MSC_STRPCL_CLOCK_CONTROL_START | MSC_STRPCL_START_OP;
	return 0;
}

static u8 * mmc_cmd(u16 cmd, unsigned int arg, unsigned int cmdat, u16 rtype)
{
	static u8 resp[20];
	u32 timeout = 0x3fffff;
	int words, i;

	jz_mmc_stop_clock();
	REG_MSC_CMD   = cmd;
	REG_MSC_ARG   = arg;
	REG_MSC_CMDAT = cmdat;

	REG_MSC_IMASK = ~MSC_IMASK_END_CMD_RES;
	jz_mmc_start_clock();

	while (timeout-- && !(REG_MSC_STAT & MSC_STAT_END_CMD_RES))
		;
	REG_MSC_IREG = MSC_IREG_END_CMD_RES;

	switch (rtype) {
		case MSC_CMDAT_RESPONSE_R1:
		case MSC_CMDAT_RESPONSE_R3:
			words = 3;
			break;

		case MSC_CMDAT_RESPONSE_R2:
			words = 8;
			break;

		default:
			return 0;
	}

	for (i = words-1; i >= 0; i--) {
		u16 res_fifo = REG_MSC_RES;
		int offset = i << 1;

		resp[offset] = ((u8 *)&res_fifo)[0];
		resp[offset+1] = ((u8 *)&res_fifo)[1];
	}

	return resp;
}
static int mmc_block_writem(u32 src, u32 num, u8 *dst)
{
	u8 *resp;
	u32 stat, timeout, cnt, nob, sorm;
	u32 *wbuf = (u32 *)dst;

	resp = mmc_cmd(16, 0x200, 0x401, MSC_CMDAT_RESPONSE_R1);
	REG_MSC_BLKLEN = 0x200;
	REG_MSC_NOB = num / 512;
	nob  = num / 512;

	if (nob == 1) {
		if (highcap)
			resp = mmc_cmd(24, src, 0x419, MSC_CMDAT_RESPONSE_R1);
		else
			resp = mmc_cmd(24, src * 512, 0x419, MSC_CMDAT_RESPONSE_R1);

		sorm = 0;
	} else {
		if (highcap)
			resp = mmc_cmd(25, src, 0x419, MSC_CMDAT_RESPONSE_R1); // for sdhc card
		else
			resp = mmc_cmd(25, src * 512, 0x419, MSC_CMDAT_RESPONSE_R1);

		sorm = 1;
	}

	for (nob; nob >= 1; nob--) {

		timeout = 0x3FFFFFF;

		while (timeout) {
			timeout--;
			stat = REG_MSC_STAT;
			if (stat & (MSC_STAT_CRC_WRITE_ERROR | MSC_STAT_CRC_WRITE_ERROR_NOSTS)) {
				serial_puts("\n MSC_STAT_CRC_WRITE_ERROR\n\n");
				return -1;
			}
			else if (!(stat & MSC_STAT_DATA_FIFO_FULL)) {
				/* Ready to write data */
				break;
			}

			udelay(1);
		}

		if (!timeout)
			return -1;

		/* Write data to TXFIFO */
		cnt = 128;
		while (cnt) {
			while (REG_MSC_STAT & MSC_STAT_DATA_FIFO_FULL)
				;
			REG_MSC_TXFIFO = *wbuf++;
			cnt--;
		}
	}

	if (sorm)
		resp = mmc_cmd(12, 0, 0x41, MSC_CMDAT_RESPONSE_R1);

	while (!(REG_MSC_STAT & MSC_STAT_DATA_TRAN_DONE))
		;

	while (!(REG_MSC_STAT & MSC_STAT_PRG_DONE))
		;
	jz_mmc_stop_clock();
	return 0;
}
static int mmc_block_readm(u32 src, u32 num, u8 *dst)
{
	u8 *resp;
	u32 stat, timeout, data, cnt, nob, sorm;

	resp = mmc_cmd(16, 0x200, 0x401, MSC_CMDAT_RESPONSE_R1);
	REG_MSC_BLKLEN = 0x200;
	REG_MSC_NOB = num / 512;
	nob  = num / 512;

	if (nob == 1) {
		if (highcap)
			resp = mmc_cmd(17, src, 0x409, MSC_CMDAT_RESPONSE_R1);
		else
			resp = mmc_cmd(17, src * 512, 0x409, MSC_CMDAT_RESPONSE_R1);

		sorm = 0;
	} else {
		if (highcap)
			resp = mmc_cmd(18, src, 0x409, MSC_CMDAT_RESPONSE_R1);
		else
			resp = mmc_cmd(18, src * 512, 0x409, MSC_CMDAT_RESPONSE_R1);

		sorm = 1;
	}


	for (nob; nob >= 1; nob--) {

		timeout = 0x3ffffff;

		while (timeout) {
			timeout--;
			stat = REG_MSC_STAT;
			if (stat & MSC_STAT_TIME_OUT_READ) {
				printf ("\n MSC_STAT_TIME_OUT_READ\n\n");
				return -1;
			}
			else if (stat & MSC_STAT_CRC_READ_ERROR) {
				printf ("\n MSC_STAT_CRC_READ_ERROR\n\n");
				return -1;
			}
			else if (!(stat & MSC_STAT_DATA_FIFO_EMPTY)) {
				/* Ready to read data */
				break;
			}
			udelay(1);
		}
		if (!timeout) {
			printf ("\n mmc/sd read timeout\n");
			return -1;
		}

		/* Read data from RXFIFO. It could be FULL or PARTIAL FULL */
		cnt =128;
		while (cnt) {
			while (cnt && (REG_MSC_STAT & MSC_STAT_DATA_FIFO_EMPTY))
				;
			cnt --;

			data = REG_MSC_RXFIFO;
			{
				*dst++ = (u8)(data >> 0);
				*dst++ = (u8)(data >> 8);
				*dst++ = (u8)(data >> 16);
				*dst++ = (u8)(data >> 24);
			}
		}
	}

	//for test
	printf ("kaka yoyo\n");
	//while(1);
	printf ("kaka yoyoEnd\n");

	if (sorm)
		resp = mmc_cmd(12, 0, 0x41, MSC_CMDAT_RESPONSE_R1);

	jz_mmc_stop_clock();

	return 0;
}

static void sd_init(void)
{
	int retries;
	u8 *resp;
	unsigned int cardaddr;

	resp = mmc_cmd(41, 0x40ff8000, 0x3, MSC_CMDAT_RESPONSE_R3);
	retries = 500;
	while (retries-- && resp && !(resp[4] & 0x80)) {
		resp = mmc_cmd(55, 0, 0x1, MSC_CMDAT_RESPONSE_R1);
		resp = mmc_cmd(41, 0x40ff8000, 0x3, MSC_CMDAT_RESPONSE_R3);
		udelay(1000);
		udelay(1000);
	}

	if (resp[4] & 0x80) 
		serial_puts("SD init ok\n");
	else 
		serial_puts("SD init fail\n");

	/* try to get card id */
	resp = mmc_cmd(2, 0, 0x2, MSC_CMDAT_RESPONSE_R2);
	resp = mmc_cmd(3, 0, 0x6, MSC_CMDAT_RESPONSE_R1);
	cardaddr = (resp[4] << 8) | resp[3]; 
	rca = cardaddr << 16;
	resp = mmc_cmd(9, rca, 0x2, MSC_CMDAT_RESPONSE_R2);
	highcap = (resp[14] & 0xc0) >> 6;
	REG_MSC_CLKRT = 2;    //2 is ok ,dont't grater
	resp = mmc_cmd(7, rca, 0x41, MSC_CMDAT_RESPONSE_R1);
	resp = mmc_cmd(55, rca, 0x1, MSC_CMDAT_RESPONSE_R1);
	resp = mmc_cmd(6, 0x2, 0x401, MSC_CMDAT_RESPONSE_R1);
}





#define INREG32(x)           ( (unsigned int)(*(volatile unsigned int* const)(x)) )
#define OUTREG32(x, y)       *(volatile unsigned int * const)(x) = (y)
#define SETREG32(x, y)      OUTREG32(x, INREG32(x)|(y))
#define OUTREG16(x, y)       *(volatile unsigned short * const)(x) = (y)
#define INREG16(x)           ( (unsigned short)(*(volatile unsigned short * const)(x)) )


#define OSC_CLOCK       24000000	// The main OSC clock(PLL-IN, Unit: Hz)

enum card_speed_mode_t {
	SLOW_SPEED_MODEL = 0,
	FAST_SPEED_MODEL,
	HIGH_SPEED_MODEL
};

enum card_type_t {
	SD_CARD = 0,
	MMC_CARD
};

/* Standard MMC/SD clock speeds */
#define MMC_CLOCK_SLOW    400000      /* 400 kHz for initial setup */
#define MMC_CLOCK_FAST  20000000      /* 20 MHz for maximum for normal operation */
#define MMC_CLOCK_HIGH  48000000      /* 48 MHz for MMC Card */
#define SD_CLOCK_FAST   24000000      /* 24 MHz for SD Cards */
#define SD_CLOCK_HIGH   48000000      /* 48 MHz for SD Cards */



static inline unsigned int jz_mmc_calc_clkrt(unsigned int pclk_rate, unsigned int set_rate)
{
	unsigned int msc_clkrt;
	unsigned int clk_src = pclk_rate;
	//unsigned int clk_src = is_sd ? 48000000 : 20000000;

	msc_clkrt = 0;
	while (set_rate < clk_src) 
	{
		msc_clkrt ++;
		clk_src >>= 1;
	}
	if(msc_clkrt > 7) 
	{
		msc_clkrt = 7;
		printf("\n ERROR --The value of MSC_CLKRT lager than 7 please check \n");
	}

	return msc_clkrt;
}



unsigned int CalcutatePLLClock(unsigned int cppcr, int  bCheckPCSBit)
{
	unsigned int  m, n, od, pll, pcs;

	if ((cppcr & CPM_CPPCR_PLLEN) && !(cppcr & CPM_CPPCR_PLLBP))
	{
		m = ((cppcr & CPM_CPPCR_PLLM_MASK) >> CPM_CPPCR_PLLM_BIT) + 2;
		n = ((cppcr & CPM_CPPCR_PLLN_MASK) >> CPM_CPPCR_PLLN_BIT) + 2;
		od = ((cppcr & CPM_CPPCR_PLLOD_MASK) >> CPM_CPPCR_PLLOD_BIT) + 1;
		od = (od == 3) ? (od - 1) : od;
		pll = OSC_CLOCK * m / (n * od);
	}
	else
		pll = OSC_CLOCK;

	// Get PCS Bit for MSC, I2S, LCD and USB if need.
	if (bCheckPCSBit)
	{
		pcs = INREG32(CPM_CPCCR) & CPM_CPCCR_PCS;
		if (pcs == 0)
			pll >>= 1;
	}

	return (pll);
}


unsigned int GetCurrentPLLClock(int  bCheckPCSBit)
{
	return CalcutatePLLClock(INREG32(CPM_CPPCR), bCheckPCSBit);
}


static unsigned int jz_set_msc_pclk_from_pll(unsigned int card_type, unsigned int speed_mode)
{
	unsigned int clock,div = 0;

	clock = GetCurrentPLLClock(1);

	if (card_type == SD_CARD)
	{
		if(speed_mode == HIGH_SPEED_MODEL)
		{
			div = (clock + SD_CLOCK_HIGH -1)/ SD_CLOCK_HIGH;			
		}
		else 
		{
			div = (clock + SD_CLOCK_FAST - 1)/ SD_CLOCK_FAST;
		}
	}

	else if(card_type == MMC_CARD) 
	{
		if(speed_mode == HIGH_SPEED_MODEL)
		{
			div = (clock + MMC_CLOCK_HIGH -1)/ MMC_CLOCK_HIGH;			
		}
		else 
		{
			div = (clock + MMC_CLOCK_FAST -1)/ MMC_CLOCK_FAST;
		}
	}
	else
	{
		printf("\n your card_type value error !!! \n");
	}

	OUTREG32(CPM_MSCCDR(1),(1<<31)|(div -1));
	printf("CpmSelectMscHSClk:clock = %d,div = %d,CPM_MSCCDR[0x%x]\n",clock,(div-1),INREG32(CPM_MSCCDR(1)));

	if(div > 0)	clock = clock/div;
	return clock;
}



void jz_mmc_set_card_clock(unsigned int msc_ch, unsigned int card_type, unsigned int speed_mode, unsigned int clk_set)
{
	/* get MSC_CLKRT value */
	unsigned int msc_clkrt = 0;
	unsigned int pclk;

	jz_mmc_stop_clock();  //stop sd/mmc   clock 

	pclk = jz_set_msc_pclk_from_pll(card_type, speed_mode);	/* select clock source from CPM */
	SETREG32(CPM_CPCCR, CPM_CPCCR_CE);  //upgrate  devision immediately
	msc_clkrt = jz_mmc_calc_clkrt(pclk, clk_set);
	OUTREG16(MSC_CLKRT, msc_clkrt);

	printf("set clock to %u Hz msc_clkrt=%d PLL_out=%d MSC_CLKRT[0x%x]\n", clk_set, msc_clkrt, pclk,INREG16(MSC_CLKRT));
}


/* init mmc/sd card we assume that the card is in the slot */
static int  mmc_init(void)
{

	int retries;
	u8 *resp;


#define INREG32(x)           ( (unsigned int)(*(volatile unsigned int* const)(x)) )
#define OUTREG32(x, y)       *(volatile unsigned int * const)(x) = (y)
#define CLRREG32(x, y)      OUTREG32(x, INREG32(x)&~(y))

	//	CLRREG32(CPM_CLKGR, CPM_CLKGR_MSC0);
	CLRREG32(CPM_CLKGR, CPM_CLKGR_MSC1);

	__gpio_as_msc();  //old, save it,and now support msc_1_1bit

	//MMC_POWER_PIN
#define	GPIO_GROUP_E	0x80
#define MMC1_POWER_PIN			(0xff)
#define MMC_POWER_PIN			MMC1_POWER_PIN
	__gpio_as_output(MMC_POWER_PIN);
	__gpio_disable_pull(MMC_POWER_PIN);
	__gpio_set_pin(MMC_POWER_PIN);

	//MMC_CD_PIN
#define MMC1_CD_PIN				(GPIO_GROUP_E + 8)
#define MMC_CD_PIN				MMC1_CD_PIN
	__gpio_as_input(MMC_CD_PIN);
	__gpio_disable_pull(MMC_CD_PIN);

	//MMC_SD_WP
#define UNUSED_GPIO_PIN		0xFFFFFFFF
#define MMC1_SD_WP				(UNUSED_GPIO_PIN)
#define MMC_SD_WP				MMC1_SD_WP
	__gpio_as_input(MMC_SD_WP);
	__gpio_disable_pull(MMC_SD_WP);


	//MMC_POWER_PIN  /* turn on power of card */
#define MSC1_POWER_ON()				\
	do {						\
		__gpio_clear_pin(MMC_POWER_PIN);		\
	} while (0)

	MSC1_POWER_ON();		/* turn on power of card */



	//MMC_RESET
#define OUTREG16(x, y)       *(volatile unsigned short * const)(x) = (y)
#define MMC_RESET()					\
	do { 								\
		OUTREG16(MSC_STRPCL,MSC_STRPCL_RESET);			\
		while (INREG32(MSC_STAT) & MSC_STAT_IS_RESETTING);		\
	} while (0)

	MMC_RESET();		/* reset mmc/sd controller */

	OUTREG32(A_MSC_LPM, 1); //set it  low power mode

	OUTREG32(MSC_RESTO, 0xffff);

	MMC_IRQ_MASK();		/* mask all IRQs */

	jz_mmc_stop_clock();	/* stop MMC/SD clock */

	__msc_reset();  //msc 1
	MMC_IRQ_MASK();	//msc 1


	/*卡初始化阶段频率为 200K */
	jz_mmc_set_card_clock(1, SD_CARD, FAST_SPEED_MODEL, 200000);

	resp = mmc_cmd(12, 0, 0x41, MSC_CMDAT_RESPONSE_R1);
	/* reset */

	resp = mmc_cmd(0, 0, 0x80, 0);
	resp = mmc_cmd(8, 0x1aa, 0x1, MSC_CMDAT_RESPONSE_R1);
	resp = mmc_cmd(55, 0, 0x1, MSC_CMDAT_RESPONSE_R1);
	if(!(resp[0] & 0x20) && (resp[5] != 0x37)) { 
		resp = mmc_cmd(1, 0x40ff8000, 0x3, MSC_CMDAT_RESPONSE_R3);
		retries = 500;
		while (retries-- && resp && !(resp[4] & 0x80)) {
			resp = mmc_cmd(1, 0x40300000, 0x3, MSC_CMDAT_RESPONSE_R3);
			udelay(1000);
			udelay(1000);
		}

		if ((resp[4] & 0x80 ) == 0x80) 
			serial_puts("MMC init ok\n");
		else 
			serial_puts("MMC init fail\n");

		if((resp[4] & 0x60) == 0x40)
			highcap = 1;
		else
			highcap = 0;

		/* try to get card id */
		resp = mmc_cmd(2, 0, 0x2, MSC_CMDAT_RESPONSE_R2);
		resp = mmc_cmd(3, 0x10, 0x1, MSC_CMDAT_RESPONSE_R1);

		REG_MSC_CLKRT = 2;	/* 16/1 MHz */
		resp = mmc_cmd(7, 0x10, 0x1, MSC_CMDAT_RESPONSE_R1);
		resp = mmc_cmd(6, 0x3b70101, 0x441, MSC_CMDAT_RESPONSE_R1);
	}
	else
		sd_init();
	return 0;
}


/*
 * Load kernel image from MMC/SD into RAM
 */
int msc_read(ulong start_byte, u8 *dst, size_t len)
{
	int start_sect;

	start_sect = start_byte / 512;

	mmc_init();

	mmc_block_readm(start_sect, len, dst);
	return 0;
}

int msc_write(ulong start_byte, u8 *dst, size_t len)
{
	int start_sect;

	start_sect = start_byte / 512;

	mmc_block_writem(start_sect, len, dst);
	return 0;
}

static inline int str2long(char *p, ulong *num)
{
	char *endptr;

	*num = simple_strtoul(p, &endptr, 16);
	return (*p != '\0' && *endptr == '\0') ? 1 : 0;
}

int do_msc(cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	int ret=0;
	ulong addr, off, size;
	char *cmd;
	int quiet = 1;
	const char *quiet_str = getenv("quiet");

	/* at least two arguments please */
	if (argc < 2)
		goto usage;

	if (quiet_str)
		quiet = simple_strtoul(quiet_str, NULL, 0) != 0;

	cmd = argv[1];

	if (strncmp(cmd, "read", 4) == 0) {
		int read;

		if (argc < 4)
			goto usage;

		addr = (ulong)simple_strtoul(argv[2], NULL, 16);

		printf ("~~~~~~~~~~~~~ addr is %x ~~~~~~~~~~~~ \n", addr);
		off = (ulong)simple_strtoul(argv[3], NULL, 16);
		size = (ulong)simple_strtoul(argv[4], NULL, 16);

		read = strncmp(cmd, "read", 4) == 0; /* 1 = read, 0 = write */
		ret = msc_read(off, (u_char *)addr, size);

		return ret == 0 ? 0 : 1;
	}

usage:
	printf("Usage:\n%s\n", cmdtp->usage);
	return 1;
}

U_BOOT_CMD(msc, 5, 1, do_msc,
		"msc	- MMC/SD dingux\n",
		"msc read	- addr off|partition size\n");
