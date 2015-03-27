/*
 * (C) Copyright 2006
 * Texas Instruments, <www.ti.com>
 * Jian Zhang <jzhang@ti.com>
 * Richard Woodruff <r-woodruff2@ti.com>
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
#include <part.h>
#include <fat.h>
#include <asm/arch/cpu.h>
#include <asm/arch/bits.h>
#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/sys_info.h>
#include <asm/arch/clocks.h>
#include <asm/arch/mem.h>

/* params for omap37xx */
#define CORE_DPLL_PARAM_M2	0x09
#define CORE_DPLL_PARAM_M	0x360
#define CORE_DPLL_PARAM_N	0xC

/* BeagleBoard revisions */
#define REVISION_AXBX		0x7
#define REVISION_CX		0x6
#define REVISION_C4		0x5
#define REVISION_XM		0x0

/* Used to index into DPLL parameter tables */
struct dpll_param {
	unsigned int m;
	unsigned int n;
	unsigned int fsel;
	unsigned int m2;
};

typedef struct dpll_param dpll_param;

/* Following functions are exported from lowlevel_init.S */
extern dpll_param *get_mpu_dpll_param(void);
extern dpll_param *get_iva_dpll_param(void);
extern dpll_param *get_core_dpll_param(void);
extern dpll_param *get_per_dpll_param(void);

#define __raw_readl(a)		(*(volatile unsigned int *)(a))
#define __raw_writel(v, a)	(*(volatile unsigned int *)(a) = (v))
#define __raw_readw(a)		(*(volatile unsigned short *)(a))
#define __raw_writew(v, a)	(*(volatile unsigned short *)(a) = (v))

static char *rev_s[CPU_3XX_MAX_REV] = {
	"1.0",
	"2.0",
	"2.1",
	"3.0",
	"3.1",
	"UNKNOWN",
	"UNKNOWN",
	"3.1.2"};

/*******************************************************
 * Routine: delay
 * Description: spinning delay to use before udelay works
 ******************************************************/
static inline void delay(unsigned long loops)
{
	__asm__ volatile ("1:\n" "subs %0, %1, #1\n"
			  "bne 1b":"=r" (loops):"0"(loops));
}

void udelay (unsigned long usecs) {
	delay(usecs);
}

/*****************************************
 * Routine: board_init
 * Description: Early hardware init.
 *****************************************/
int board_init(void)
{
	/*
	 use code like this to early enable USB charging!
	 so that the board boots with no battery but USB power

	 if i2c does not work at this position,
	 try to move this code to gta04_revision()

	 I2C_SET_BUS(0);
	 data = 0x01;
	 i2c_write(0x4B, 0x29, 1, &data, 1);
	 data = 0x0c;
	 i2c_write(0x4B, 0x2b, 1, &data, 1);
	 i2c_read(0x4B, 0x2a, 1, &data, 1);

*/

#if 0   /* no - does not work as expected because the i2c driver is not part of X-Loader */

	/*
	 For charging using USB, the software must enable the USB automatic
	 charge by forcing the BOOT_BCI[1] BCIAUTOUSB bit to 1. When the USB
	 charger is plugged in, the software detects the type of device
	 (USB charger or carkit charger). The software must set the POWER_CTRL[5]
	 OTG_EN bit to 1 at least 50 ms before forcing the BCIMFSTS4[2] USBFASTMCHG
	 bit to 1.
	 */

	u8 data;
	int i;

	I2C_SET_BUS(0);

	 /* enable access to BCIIREF1 */
	data = 0xE7;
	i2c_write(0x4A, 0x85, 1, &data, 1);	// write BCIMFKEY

	/* set charging current = 852mA */
	data = 0xFF;
	i2c_write(0x4A, 0x9B, 1, &data, 1);	// write BCIIREF1

	/* forcing the field BCIAUTOUSB (BOOT_BCI[1]) to 1 */
	i2c_read(0x4B, 0x3D, 1, &data, 1);	// read BOOT_BCI
	data |= 0x33;	// CONFIG_DONE | BCIAUTOWEN | BCIAUTOUSB | BCIAUTOAC
	i2c_write(0x4B, 0x3D, 1, &data, 1);	// write BOOT_BCI

	/* Enabling interfacing with usb through OCP */
	i2c_read(0x48, 0xFE, 1, &data, 1);	// read BOOT_BCI
	data |= 0x01;	// PHY_DPLL_CLK
	i2c_write(0x48, 0xFE, 1, &data, 1);	// write BOOT_BCI

	do {
		udelay(10);
		i2c_read(0x48, 0xFE, 1, &data, 1);	// read PHY_CLK_CTRL_STS
	} while (!(data & 0x01));	/* loop until PHY DPLL is locked */

	/* OTG_EN (POWER_CTRL[5]) to 1 */
	data = 0x20;
	i2c_write(0x48, 0xAD, 1, &data, 1);	// write POWER_CTRL_SET

	for(i=0; i<50; i++)
		udelay(1000);

	/* forcing USBFASTMCHG(BCIMFSTS4[2]) to 1 */
	i2c_read(0x4A, 0x84, 1, &data, 1);	// read BCIMFSTS4
	data |= 0x04;	// USBFASTMCHG
	i2c_write(0x4A, 0x84, 1, &data, 1);	// write BCIMFSTS4

	/* if we can make this work,
	 * we should try to enable the power LED (TCA6507)
	 * so that the user gets an early feedback that X-Loader
	 * has started, early after pressing the power button
	 * but can X-Loader write to other I2C or only bus 0?
	 */

#endif
	return 0;
}

/*************************************************************
 *  get_device_type(): tell if GP/HS/EMU/TST
 *************************************************************/
u32 get_device_type(void)
{
	int mode;
	mode = __raw_readl(CONTROL_STATUS) & (DEVICE_MASK);
	return mode >>= 8;
}

/************************************************
 * get_sysboot_value(void) - return SYS_BOOT[4:0]
 ************************************************/
u32 get_sysboot_value(void)
{
	int mode;
	mode = __raw_readl(CONTROL_STATUS) & (SYSBOOT_MASK);
	return mode;
}

/*************************************************************
 * Routine: get_mem_type(void) - returns the kind of memory connected
 * to GPMC that we are trying to boot form. Uses SYS BOOT settings.
 *************************************************************/
u32 get_mem_type(void)
{
	u32   mem_type = get_sysboot_value();
#if 0	// force some NAND type for debugging
	return GPMC_ONENAND;
#endif
	switch (mem_type) {
	case 0:
	case 2:
	case 4:
	case 16:	// BB ONE-NAND or BB XM with SAMSUNG MCP
	case 22:
		return GPMC_ONENAND;

	case 1:
	case 12:
	case 15:	// BB NAND or early BB-XM with NAND
	case 21:
	case 27:
		return GPMC_NAND;

	case 3:
	case 6:
		return MMC_ONENAND;

	case 8:
	case 11:
	case 14:
	case 20:
	case 26:
		return GPMC_MDOC;

	case 18:
#ifndef CONFIG_GTA04
		{
		extern int beagle_revision(void);
		if (beagle_revision() == REVISION_XM)
			return GPMC_NONE;	// standard BB XM has no NAND
		}
#endif
	case 17:
	case 24:
		return MMC_NAND;

	case 7:
	case 10:
	case 13:
	case 19:
	case 25:
	default:
		return GPMC_NOR;
	}
}

/******************************************
 * get_cpu_type(void) - extract cpu info
 ******************************************/
u32 get_cpu_type(void)
{
	return __raw_readl(CONTROL_OMAP_STATUS);
}

/******************************************
 * get_cpu_id(void) - extract cpu id
 * returns 0 for ES1.0, cpuid otherwise
 ******************************************/
u32 get_cpu_id(void)
{
	u32 cpuid = 0;

	/*
	 * On ES1.0 the IDCODE register is not exposed on L4
	 * so using CPU ID to differentiate between ES1.0 and > ES1.0.
	 */
	__asm__ __volatile__("mrc p15, 0, %0, c0, c0, 0":"=r"(cpuid));
	if ((cpuid & 0xf) == 0x0) {
		return 0;
	} else {
		/* Decode the IDs on > ES1.0 */
		cpuid = __raw_readl(CONTROL_IDCODE);
	}

	return cpuid;
}

/******************************************
 * get_cpu_family(void) - extract cpu info
 ******************************************/
u32 get_cpu_family(void)
{
	u16 hawkeye;
	u32 cpu_family;
	u32 cpuid = get_cpu_id();

	if (cpuid == 0)
		return CPU_OMAP34XX;

	hawkeye = (cpuid >> HAWKEYE_SHIFT) & 0xffff;
	switch (hawkeye) {
		case HAWKEYE_OMAP34XX:
			cpu_family = CPU_OMAP34XX;
			break;
		case HAWKEYE_AM35XX:
			cpu_family = CPU_AM35XX;
			break;
		case HAWKEYE_OMAP36XX:
			cpu_family = CPU_OMAP36XX;
			break;
		default:
			cpu_family = CPU_OMAP34XX;
	}

	return cpu_family;
}

/******************************************
 * get_cpu_rev(void) - extract version info
 ******************************************/
u32 get_cpu_rev(void)
{
	u32 cpuid = 0;
	/* On ES1.0 the IDCODE register is not exposed on L4
	 * so using CPU ID to differentiate
	 * between ES2.0 and ES1.0.
	 */
	__asm__ __volatile__("mrc p15, 0, %0, c0, c0, 0":"=r" (cpuid));
	if ((cpuid  & 0xf) == 0x0)
		return CPU_3430_ES1;
	else
		return CPU_3430_ES2;

}

/******************************************
 * Print CPU information
 ******************************************/
int print_cpuinfo (void)
{
	char *cpu_family_s, *cpu_s, *sec_s;

	switch (get_cpu_family()) {
		case CPU_OMAP34XX:
			cpu_family_s = "OMAP";
			switch (get_cpu_type()) {
				case OMAP3503:
					cpu_s = "3503";
					break;
				case OMAP3515:
					cpu_s = "3515";
					break;
				case OMAP3525:
					cpu_s = "3525";
					break;
				case OMAP3530:
					cpu_s = "3530";
					break;
				default:
					cpu_s = "35XX";
					break;
			}
			break;
		case CPU_AM35XX:
			cpu_family_s = "AM";
			switch (get_cpu_type()) {
				case AM3505:
					cpu_s = "3505";
					break;
				case AM3517:
					cpu_s = "3517";
					break;
				default:
					cpu_s = "35XX";
					break;
			}
			break;
		case CPU_OMAP36XX:
			cpu_family_s = "OMAP";
			switch (get_cpu_type()) {
				case OMAP3730:
					cpu_s = "3630/3730";
					break;
				default:
					cpu_s = "36XX/37XX";
					break;
			}
			break;
		default:
			cpu_family_s = "OMAP";
			cpu_s = "35XX";
	}

	switch (get_device_type()) {
		case TST_DEVICE:
			sec_s = "TST";
			break;
		case EMU_DEVICE:
			sec_s = "EMU";
			break;
		case HS_DEVICE:
			sec_s = "HS";
			break;
		case GP_DEVICE:
			sec_s = "GP";
			break;
		default:
			sec_s = "?";
	}

	printf("%s%s-%s ES%s\n",
		   cpu_family_s, cpu_s, sec_s, rev_s[get_cpu_rev()]);

	return 0;
}

/******************************************
 * cpu_is_3410(void) - returns true for 3410
 ******************************************/
u32 cpu_is_3410(void)
{
	int status;
	if (get_cpu_rev() < CPU_3430_ES2) {
		return 0;
	} else {
		/* read scalability status and return 1 for 3410*/
		status = __raw_readl(CONTROL_SCALABLE_OMAP_STATUS);
		/* Check whether MPU frequency is set to 266 MHz which
		 * is nominal for 3410. If yes return true else false
		 */
		if (((status >> 8) & 0x3) == 0x2)
			return 1;
		else
			return 0;
	}
}

#ifdef CONFIG_GTA04

int gta04_revision(void)
{
	 int rev;
	 static char revision[8] = {	/* revision table defined by pull-down R305, R306, R307 */
		9,
		6,
		7,
		3,
		8,
		4,
		5,
		2
	 };
	 omap_request_gpio(171);
	 omap_request_gpio(172);
	 omap_request_gpio(173);
	 omap_set_gpio_direction(171, 1);
	 omap_set_gpio_direction(172, 1);
	 omap_set_gpio_direction(173, 1);

	 rev = omap_get_gpio_datain(173) << 2 |
		omap_get_gpio_datain(172) << 1 |
		omap_get_gpio_datain(171) << 0;
	 omap_free_gpio(171);
	 omap_free_gpio(172);
	 omap_free_gpio(173);

	 return revision[rev];	/* 000 means GTA04A2 */
}

#else

/******************************************
 * beagle_identify
 * Description: Detect if we are running on a Beagle revision Ax/Bx,
 *		C1/2/3, C4 or D. This can be done by reading
 *		the level of GPIO173, GPIO172 and GPIO171. This should
 *		result in
 *		GPIO173, GPIO172, GPIO171: 1 1 1 => Ax/Bx
 *		GPIO173, GPIO172, GPIO171: 1 1 0 => C1/2/3
 *		GPIO173, GPIO172, GPIO171: 1 0 1 => C4
 *		GPIO173, GPIO172, GPIO171: 0 0 0 => XM
 ******************************************/
int beagle_revision(void)
{
	int rev;

	omap_request_gpio(171);
	omap_request_gpio(172);
	omap_request_gpio(173);
	omap_set_gpio_direction(171, 1);
	omap_set_gpio_direction(172, 1);
	omap_set_gpio_direction(173, 1);

	rev = omap_get_gpio_datain(173) << 2 |
		omap_get_gpio_datain(172) << 1 |
		omap_get_gpio_datain(171);
	omap_free_gpio(171);
	omap_free_gpio(172);
	omap_free_gpio(173);

	return rev;
}

#endif

/*****************************************************************
 * sr32 - clear & set a value in a bit range for a 32 bit address
 *****************************************************************/
void sr32(u32 addr, u32 start_bit, u32 num_bits, u32 value)
{
	u32 tmp, msk = 0;
	msk = 1 << num_bits;
	--msk;
	tmp = __raw_readl(addr) & ~(msk << start_bit);
	tmp |= value << start_bit;
	__raw_writel(tmp, addr);
}

/*********************************************************************
 * wait_on_value() - common routine to allow waiting for changes in
 *   volatile regs.
 *********************************************************************/
u32 wait_on_value(u32 read_bit_mask, u32 match_value, u32 read_addr, u32 bound)
{
	u32 i = 0, val;
	do {
		++i;
		val = __raw_readl(read_addr) & read_bit_mask;
		if (val == match_value)
			return 1;
		if (i == bound)
			return 0;
	} while (1);
}

#ifdef CFG_3430SDRAM_DDR

#define DDR_ONLY	1
#define NUMONYX_MCP	2
#define MICRON_MCP256	3
#define MICRON_MCP512	4
#define SAMSUNG_MCP	5

int identify_xm_ddr(void)
{
#ifdef CFG_ONENAND
	if (get_mem_type() == GPMC_ONENAND)
		return SAMSUNG_MCP;	// BeagleBoard XM or GTA04b7/Neo900 with Samsung NAND
#endif
#ifdef CONFIG_NAND
	if (get_mem_type() == GPMC_NAND)
	{
	int	mfr, id;
	extern int nand_readid(int *mfr, int *id);

	__raw_writel(M_NAND_GPMC_CONFIG1, GPMC_CONFIG1 + GPMC_CONFIG_CS0);
	__raw_writel(M_NAND_GPMC_CONFIG2, GPMC_CONFIG2 + GPMC_CONFIG_CS0);
	__raw_writel(M_NAND_GPMC_CONFIG3, GPMC_CONFIG3 + GPMC_CONFIG_CS0);
	__raw_writel(M_NAND_GPMC_CONFIG4, GPMC_CONFIG4 + GPMC_CONFIG_CS0);
	__raw_writel(M_NAND_GPMC_CONFIG5, GPMC_CONFIG5 + GPMC_CONFIG_CS0);
	__raw_writel(M_NAND_GPMC_CONFIG6, GPMC_CONFIG6 + GPMC_CONFIG_CS0);

	/* Enable the GPMC Mapping */
	__raw_writel((((OMAP34XX_GPMC_CS0_SIZE & 0xF)<<8) |
			     ((NAND_BASE_ADR>>24) & 0x3F) |
			     (1<<6)),  (GPMC_CONFIG7 + GPMC_CONFIG_CS0));
	delay(2000);

	nand_readid(&mfr, &id);
	if (mfr == 0)
		return DDR_ONLY;
	if ((mfr == 0x20) && (id == 0xba))
		return NUMONYX_MCP;
	if ((mfr == 0x2c) && (id == 0xba))
		return MICRON_MCP256;
	if ((mfr == 0x2c) && (id == 0xbc))
		return MICRON_MCP512;
	if ((mfr == 0x2c) && (id == 0xb3))
		return MICRON_MCP512;
	}
#endif
	return DDR_ONLY;
}

int identify_real_ddr(void)
{ // overwrite some special cases
	int ram;
	ram=identify_xm_ddr();
#ifndef CONFIG_GTA04
	switch(beagle_revision()) {
		case REVISION_C4:
			if(ram == NUMONYX_MCP)
				ram=MICRON_MCP512;	// C4 has fast 512 MB
			break;
		case REVISION_XM:
			if(ram == DDR_ONLY)
				ram=MICRON_MCP512;	// XM has fast 512 MB
			break;
		default:
			ram=MICRON_MCP256;	// older beagle boards
			break;
	}
#endif
	return ram;
}

/*********************************************************************
 * Various helper functions to calculate Samsung MCP DDR timings.
 * Code borrowed from linux kernel (sdram-nokia.c)
 *********************************************************************/
static const u32 osc_sys_clk[] = {
	12000, 13000, 19200, 26000, 38400, 16800
};

static u32 get_sys_clkin_rate(void)
{
  u32 rv;

  rv = osc_sys_clk[__raw_readl(PRM_CLKSEL) & 7];

  if (((__raw_readl(PRM_CLKSRC_CTRL) >> 6) & 3) == 2)
    rv /= 2;

  return rv;
}

static u32 get_dpll3_rate(void)
{
	u32 dpll3, dpll3_mult, dpll3_div, dpll3_out_div;

	dpll3 = __raw_readl(CM_CLKSEL1_PLL);
	dpll3_mult = (dpll3 >> 16) & 0x7ff;
	dpll3_div = ((dpll3 >> 8) & 0x7f) + 1;
	dpll3_out_div = dpll3 >> 27;

	return (((get_sys_clkin_rate() * dpll3_mult) / dpll3_div) /
		dpll3_out_div);
}

static u32 get_l3_rate(void)
{
	return get_dpll3_rate() / (__raw_readl(CM_CLKSEL_CORE) & 3);
}

static u32 get_fclk_period(void)
{
	return 1000000000 / get_l3_rate();
}

static u32 ps_to_ticks(unsigned int time_ps)
{
	u32 tick_ps;

	/* Calculate in picosecs to yield more exact results */
	tick_ps = get_fclk_period();

	return (time_ps + tick_ps - 1) / tick_ps;
}

static int set_timing_regval(u32 *regval, int st_bit, int end_bit, int ticks)
{
	int mask, nr_bits;

	nr_bits = end_bit - st_bit + 1;

	if (ticks >= 1 << nr_bits)
		return -1;

	mask = (1 << nr_bits) - 1;
	*regval &= ~(mask << st_bit);
	*regval |= ticks << st_bit;

	return 0;
}

static int set_timing_regval_ps(u32 *regval, int st_bit, int end_bit, int time)
{
	int ticks;

	if (time == 0)
		ticks = 0;
	else
		ticks = ps_to_ticks(time);

	return set_timing_regval(regval, st_bit, end_bit, ticks);
}

/*********************************************************************
 * config_samsung_mcp_ddr() - Init DDR on BeagleBoard XM or
 * GTA04b7/Neo900 with Samsung MCP
 *********************************************************************/
static int config_samsung_mcp_ddr(void)
{
	u32 rfr, auto_rfr, actim_ctrlb = 0, actim_ctrla = 0;

	__raw_writel(SMART_IDLE, SDRC_SYSCONFIG);

	__raw_writel(0x3690019, SDRC_MCFG_0);
	__raw_writel(0x3690019, SDRC_MCFG_1);
	__raw_writel(4, SDRC_CS_CFG);

	/* FIXME - use defines or a structure instead of hardcoded values */
	if (set_timing_regval_ps(&actim_ctrla, 0, 4, 30725) < 0 ||
	    set_timing_regval_ps(&actim_ctrla, 6, 8, 15362) < 0 ||
	    set_timing_regval_ps(&actim_ctrla, 9, 11, 10241) < 0 ||
	    set_timing_regval_ps(&actim_ctrla, 12, 14, 20483) < 0 ||
	    set_timing_regval_ps(&actim_ctrla, 15, 17, 15362) < 0 ||
	    set_timing_regval_ps(&actim_ctrla, 18, 21, 40967) < 0 ||
	    set_timing_regval_ps(&actim_ctrla, 22, 26, 56330) < 0 ||
	    set_timing_regval_ps(&actim_ctrla, 27, 31, 138266) < 0 ||
	    set_timing_regval_ps(&actim_ctrlb, 0, 7, 204839) < 0 ||
	    set_timing_regval(&actim_ctrlb, 8, 10, 2) < 0 ||
	    set_timing_regval(&actim_ctrlb, 12, 14, 4) < 0 ||
	    set_timing_regval(&actim_ctrlb, 16, 17, 2) < 0 )
		return -1;

	rfr = 7720 * ps_to_ticks(1000000) / 1000;

	if (rfr > 65535 + 50)
		rfr = 65535;
	else
		rfr -= 50;

	rfr <<= 8;

	__raw_writel(actim_ctrla, SDRC_ACTIM_CTRLA_0);
	__raw_writel(actim_ctrlb, SDRC_ACTIM_CTRLB_0);
	__raw_writel(actim_ctrla, SDRC_ACTIM_CTRLA_1);
	__raw_writel(actim_ctrlb, SDRC_ACTIM_CTRLB_1);
	__raw_writel(rfr, SDRC_RFR_CTRL_0);
	__raw_writel(rfr, SDRC_RFR_CTRL_1);
	__raw_writel(853, SDRC_POWER);
	__raw_writel(0x7F00001Au, SDRC_DLLA_CTRL);

	delay(10);

	__raw_writel(__raw_readl(SDRC_DLLA_CTRL) & 0xFFFFFFEF, SDRC_DLLA_CTRL);

	delay(10);

	while (!(__raw_readl(SDRC_DLLA_STATUS) & 4));

	__raw_writel(CMD_CKE_LOW, SDRC_MANUAL_0);
	__raw_writel(CMD_CKE_LOW, SDRC_MANUAL_1);

	delay(200);

	__raw_writel(CMD_PRECHARGE, SDRC_MANUAL_0);
	__raw_writel(CMD_PRECHARGE, SDRC_MANUAL_1);

	__raw_writel(CMD_AUTOREFRESH, SDRC_MANUAL_0);
	__raw_writel(CMD_AUTOREFRESH, SDRC_MANUAL_0);
	__raw_writel(CMD_NOP, SDRC_MANUAL_0);
	__raw_writel(CMD_NOP, SDRC_MANUAL_0);

	__raw_writel(CMD_AUTOREFRESH, SDRC_MANUAL_1);
	__raw_writel(CMD_AUTOREFRESH, SDRC_MANUAL_1);
	__raw_writel(CMD_NOP, SDRC_MANUAL_1);
	__raw_writel(CMD_NOP, SDRC_MANUAL_1);

	__raw_writel(50, SDRC_MR_0);
	__raw_writel(50, SDRC_MR_1);
	__raw_writel(32, SDRC_EMR2_0);
	__raw_writel(32, SDRC_EMR2_1);
	__raw_writel(__raw_readl(SDRC_POWER) | 8, SDRC_POWER);

	auto_rfr = (__raw_readl(SDRC_RFR_CTRL_0) & 0xFFFFFFFC) | 1;
	__raw_writel(auto_rfr, SDRC_RFR_CTRL_0);
	__raw_writel(auto_rfr, SDRC_RFR_CTRL_1);

	return 0;
}

/*********************************************************************
 * config_3430sdram_ddr() - Init DDR on 3430SDP dev board.
 *********************************************************************/
void config_3430sdram_ddr(void)
{
	/* reset sdrc controller */
	__raw_writel(SOFTRESET, SDRC_SYSCONFIG);
	wait_on_value(BIT0, BIT0, SDRC_STATUS, 12000000);
	__raw_writel(0, SDRC_SYSCONFIG);

	/* setup sdrc to ball mux */
	__raw_writel(SDP_SDRC_SHARING, SDRC_SHARING);

	switch (identify_real_ddr()) {
		case SAMSUNG_MCP:
			if (config_samsung_mcp_ddr())
				printf("DDR initialization failed\n");
			return;
		case NUMONYX_MCP:
			__raw_writel(0x4, SDRC_CS_CFG); /* 512MB/bank @ 200 MHz */
			__raw_writel(SDP_SDRC_MDCFG_0_DDR_NUMONYX_XM, SDRC_MCFG_0);
			__raw_writel(SDP_SDRC_MDCFG_0_DDR_NUMONYX_XM, SDRC_MCFG_1);
			__raw_writel(NUMONYX_V_ACTIMA_165, SDRC_ACTIM_CTRLA_0);
			__raw_writel(NUMONYX_V_ACTIMB_165, SDRC_ACTIM_CTRLB_0);
			__raw_writel(NUMONYX_V_ACTIMA_165, SDRC_ACTIM_CTRLA_1);
			__raw_writel(NUMONYX_V_ACTIMB_165, SDRC_ACTIM_CTRLB_1);
			__raw_writel(SDP_3430_SDRC_RFR_CTRL_165MHz, SDRC_RFR_CTRL_0);
			__raw_writel(SDP_3430_SDRC_RFR_CTRL_165MHz, SDRC_RFR_CTRL_1);
			break;
		case MICRON_MCP512:
			// bigger and faster Micron
			__raw_writel(0x2, SDRC_CS_CFG); /* 256MB/bank @ 200 MHz */
			__raw_writel(SDP_SDRC_MDCFG_0_DDR_MICRON_XM, SDRC_MCFG_0);
			__raw_writel(SDP_SDRC_MDCFG_0_DDR_MICRON_XM, SDRC_MCFG_1);
			__raw_writel(MICRON_V_ACTIMA_200, SDRC_ACTIM_CTRLA_0);
			__raw_writel(MICRON_V_ACTIMB_200, SDRC_ACTIM_CTRLB_0);
			__raw_writel(MICRON_V_ACTIMA_200, SDRC_ACTIM_CTRLA_1);
			__raw_writel(MICRON_V_ACTIMB_200, SDRC_ACTIM_CTRLB_1);
			__raw_writel(SDP_3430_SDRC_RFR_CTRL_200MHz, SDRC_RFR_CTRL_0);
			__raw_writel(SDP_3430_SDRC_RFR_CTRL_200MHz, SDRC_RFR_CTRL_1);
			break;
		case MICRON_MCP256:
		default:
			// small and slower Micron
			__raw_writel(0x1, SDRC_CS_CFG); /* 128MB/bank @ 165 MHz */
			__raw_writel(SDP_SDRC_MDCFG_0_DDR, SDRC_MCFG_0);
			__raw_writel(SDP_SDRC_MDCFG_0_DDR, SDRC_MCFG_1);
			__raw_writel(MICRON_V_ACTIMA_165, SDRC_ACTIM_CTRLA_0);
			__raw_writel(MICRON_V_ACTIMB_165, SDRC_ACTIM_CTRLB_0);
			__raw_writel(MICRON_V_ACTIMA_165, SDRC_ACTIM_CTRLA_1);
			__raw_writel(MICRON_V_ACTIMB_165, SDRC_ACTIM_CTRLB_1);
			__raw_writel(SDP_3430_SDRC_RFR_CTRL_165MHz, SDRC_RFR_CTRL_0);
			__raw_writel(SDP_3430_SDRC_RFR_CTRL_165MHz, SDRC_RFR_CTRL_1);
			break;
	}

	__raw_writel(SDP_SDRC_POWER_POP, SDRC_POWER);

	/* init sequence for mDDR/mSDR using manual commands (DDR is different) */
	__raw_writel(CMD_NOP, SDRC_MANUAL_0);
	__raw_writel(CMD_NOP, SDRC_MANUAL_1);

	delay(5000);

	__raw_writel(CMD_PRECHARGE, SDRC_MANUAL_0);
	__raw_writel(CMD_PRECHARGE, SDRC_MANUAL_1);

	__raw_writel(CMD_AUTOREFRESH, SDRC_MANUAL_0);
	__raw_writel(CMD_AUTOREFRESH, SDRC_MANUAL_1);

	__raw_writel(CMD_AUTOREFRESH, SDRC_MANUAL_0);
	__raw_writel(CMD_AUTOREFRESH, SDRC_MANUAL_1);

	/* set mr0 */
	__raw_writel(SDP_SDRC_MR_0_DDR, SDRC_MR_0);
	__raw_writel(SDP_SDRC_MR_0_DDR, SDRC_MR_1);

	/* set up dll */
	__raw_writel(SDP_SDRC_DLLAB_CTRL, SDRC_DLLA_CTRL);
	delay(0x2000);	/* give time to lock */

}
#endif /* CFG_3430SDRAM_DDR */

/*************************************************************
 * get_sys_clk_speed - determine reference oscillator speed
 *  based on known 32kHz clock and gptimer.
 *************************************************************/
u32 get_osc_clk_speed(void)
{
	u32 start, cstart, cend, cdiff, cdiv, val;

	val = __raw_readl(PRM_CLKSRC_CTRL);

	if (val & SYSCLKDIV_2)
		cdiv = 2;
	else
		cdiv = 1;

	/* enable timer2 */
	val = __raw_readl(CM_CLKSEL_WKUP) | BIT0;
	__raw_writel(val, CM_CLKSEL_WKUP);	/* select sys_clk for GPT1 */

	/* Enable I and F Clocks for GPT1 */
	val = __raw_readl(CM_ICLKEN_WKUP) | BIT0 | BIT2;
	__raw_writel(val, CM_ICLKEN_WKUP);
	val = __raw_readl(CM_FCLKEN_WKUP) | BIT0;
	__raw_writel(val, CM_FCLKEN_WKUP);

	__raw_writel(0, OMAP34XX_GPT1 + TLDR);		/* start counting at 0 */
	__raw_writel(GPT_EN, OMAP34XX_GPT1 + TCLR);	/* enable clock */
	/* enable 32kHz source */
	/* enabled out of reset */
	/* determine sys_clk via gauging */

	start = 20 + __raw_readl(S32K_CR);	/* start time in 20 cycles */
	while (__raw_readl(S32K_CR) < start) ;	/* dead loop till start time */
	cstart = __raw_readl(OMAP34XX_GPT1 + TCRR);	/* get start sys_clk count */
	while (__raw_readl(S32K_CR) < (start + 20)) ;	/* wait for 40 cycles */
	cend = __raw_readl(OMAP34XX_GPT1 + TCRR);	/* get end sys_clk count */
	cdiff = cend - cstart;	/* get elapsed ticks */
	cdiff *= cdiv;

	/* based on number of ticks assign speed */
	if (cdiff > 19000)
		return S38_4M;
	else if (cdiff > 15200)
		return S26M;
	else if (cdiff > 13000)
		return S24M;
	else if (cdiff > 9000)
		return S19_2M;
	else if (cdiff > 7600)
		return S13M;
	else
		return S12M;
}

/******************************************************************************
 * get_sys_clkin_sel() - returns the sys_clkin_sel field value based on
 *   -- input oscillator clock frequency.
 *
 *****************************************************************************/
void get_sys_clkin_sel(u32 osc_clk, u32 *sys_clkin_sel)
{
	if (osc_clk == S38_4M)
		*sys_clkin_sel = 4;
	else if (osc_clk == S26M)
		*sys_clkin_sel = 3;
	else if (osc_clk == S19_2M)
		*sys_clkin_sel = 2;
	else if (osc_clk == S13M)
		*sys_clkin_sel = 1;
	else if (osc_clk == S12M)
		*sys_clkin_sel = 0;
}

/******************************************************************************
 * prcm_init() - inits clocks for PRCM as defined in clocks.h
 *   -- called from SRAM, or Flash (using temp SRAM stack).
 *****************************************************************************/
void prcm_init(void)
{
	u32 osc_clk = 0, sys_clkin_sel;
	dpll_param *dpll_param_p;
	u32 clk_index, sil_index;

	/* Gauge the input clock speed and find out the sys_clkin_sel
	 * value corresponding to the input clock.
	 */
	osc_clk = get_osc_clk_speed();
	get_sys_clkin_sel(osc_clk, &sys_clkin_sel);

	sr32(PRM_CLKSEL, 0, 3, sys_clkin_sel);	/* set input crystal speed */

	/* If the input clock is greater than 19.2M always divide/2 */
	if (sys_clkin_sel > 2) {
		sr32(PRM_CLKSRC_CTRL, 6, 2, 2);	/* input clock divider */
		clk_index = sys_clkin_sel / 2;
	} else {
		sr32(PRM_CLKSRC_CTRL, 6, 2, 1);	/* input clock divider */
		clk_index = sys_clkin_sel;
	}

	sr32(PRM_CLKSRC_CTRL, 0, 2, 0);/* Bypass mode: T2 inputs a square clock */

	/* The DPLL tables are defined according to sysclk value and
	 * silicon revision. The clk_index value will be used to get
	 * the values for that input sysclk from the DPLL param table
	 * and sil_index will get the values for that SysClk for the
	 * appropriate silicon rev.
	 */
	sil_index = get_cpu_rev() - 1;

	/* Unlock MPU DPLL (slows things down, and needed later) */
	sr32(CM_CLKEN_PLL_MPU, 0, 3, PLL_LOW_POWER_BYPASS);
	wait_on_value(BIT0, 0, CM_IDLEST_PLL_MPU, LDELAY);

	/* Getting the base address of Core DPLL param table */
	dpll_param_p = (dpll_param *) get_core_dpll_param();
	/* Moving it to the right sysclk and ES rev base */
	dpll_param_p = dpll_param_p + 3 * clk_index + sil_index;
	/* CORE DPLL */
	/* sr32(CM_CLKSEL2_EMU) set override to work when asleep */
	sr32(CM_CLKEN_PLL, 0, 3, PLL_FAST_RELOCK_BYPASS);
	wait_on_value(BIT0, 0, CM_IDLEST_CKGEN, LDELAY);

	 /* For 3430 ES1.0 Errata 1.50, default value directly doesnt
	work. write another value and then default value. */
	sr32(CM_CLKSEL1_EMU, 16, 5, CORE_M3X2 + 1);     /* m3x2 */
	sr32(CM_CLKSEL1_EMU, 16, 5, CORE_M3X2);	/* m3x2 */
	sr32(CM_CLKSEL1_PLL, 27, 2, dpll_param_p->m2);	/* Set M2 */
	sr32(CM_CLKSEL1_PLL, 16, 11, dpll_param_p->m);	/* Set M */
	sr32(CM_CLKSEL1_PLL, 8, 7, dpll_param_p->n);	/* Set N */
	sr32(CM_CLKSEL1_PLL, 6, 1, 0);	/* 96M Src */
	sr32(CM_CLKSEL_CORE, 8, 4, CORE_SSI_DIV);	/* ssi */
	sr32(CM_CLKSEL_CORE, 4, 2, CORE_FUSB_DIV);	/* fsusb */
	sr32(CM_CLKSEL_CORE, 2, 2, CORE_L4_DIV);	/* l4 */
	sr32(CM_CLKSEL_CORE, 0, 2, CORE_L3_DIV);	/* l3 */
	sr32(CM_CLKSEL_GFX, 0, 3, GFX_DIV);	/* gfx */
	sr32(CM_CLKSEL_WKUP, 1, 2, WKUP_RSM);	/* reset mgr */
	sr32(CM_CLKEN_PLL, 4, 4, dpll_param_p->fsel);	/* FREQSEL */
	sr32(CM_CLKEN_PLL, 0, 3, PLL_LOCK);	/* lock mode */
	wait_on_value(BIT0, 1, CM_IDLEST_CKGEN, LDELAY);

	/* Getting the base address to PER  DPLL param table */
	dpll_param_p = (dpll_param *) get_per_dpll_param();
	/* Moving it to the right sysclk base */
	dpll_param_p = dpll_param_p + clk_index;
	/* PER DPLL */
	sr32(CM_CLKEN_PLL, 16, 3, PLL_STOP);
	wait_on_value(BIT1, 0, CM_IDLEST_CKGEN, LDELAY);
	sr32(CM_CLKSEL1_EMU, 24, 5, PER_M6X2);	/* set M6 */
	sr32(CM_CLKSEL_CAM, 0, 5, PER_M5X2);	/* set M5 */
	sr32(CM_CLKSEL_DSS, 0, 5, PER_M4X2);	/* set M4 */
	sr32(CM_CLKSEL_DSS, 8, 5, PER_M3X2);	/* set M3 */

	if (get_cpu_family() == CPU_OMAP36XX) {
	        sr32(CM_CLKSEL3_PLL, 0, 5, CORE_DPLL_PARAM_M2);   /* set M2 */
	        sr32(CM_CLKSEL2_PLL, 8, 11, CORE_DPLL_PARAM_M);   /* set m */
	        sr32(CM_CLKSEL2_PLL, 0, 7, CORE_DPLL_PARAM_N);    /* set n */
	} else {
		sr32(CM_CLKSEL3_PLL, 0, 5, dpll_param_p->m2);	/* set M2 */
		sr32(CM_CLKSEL2_PLL, 8, 11, dpll_param_p->m);	/* set m */
		sr32(CM_CLKSEL2_PLL, 0, 7, dpll_param_p->n);	/* set n */
	}

	sr32(CM_CLKEN_PLL, 20, 4, dpll_param_p->fsel);	/* FREQSEL */
	sr32(CM_CLKEN_PLL, 16, 3, PLL_LOCK);	/* lock mode */
	wait_on_value(BIT1, 2, CM_IDLEST_CKGEN, LDELAY);

	/* Getting the base address to MPU DPLL param table */
	dpll_param_p = (dpll_param *) get_mpu_dpll_param();

	/* Moving it to the right sysclk and ES rev base */
	dpll_param_p = dpll_param_p + 3 * clk_index + sil_index;

	/* MPU DPLL (unlocked already) */
	sr32(CM_CLKSEL2_PLL_MPU, 0, 5, dpll_param_p->m2);	/* Set M2 */
	sr32(CM_CLKSEL1_PLL_MPU, 8, 11, dpll_param_p->m);	/* Set M */
	sr32(CM_CLKSEL1_PLL_MPU, 0, 7, dpll_param_p->n);	/* Set N */
	sr32(CM_CLKEN_PLL_MPU, 4, 4, dpll_param_p->fsel);	/* FREQSEL */
	sr32(CM_CLKEN_PLL_MPU, 0, 3, PLL_LOCK);	/* lock mode */
	wait_on_value(BIT0, 1, CM_IDLEST_PLL_MPU, LDELAY);

	/* Getting the base address to IVA DPLL param table */
	dpll_param_p = (dpll_param *) get_iva_dpll_param();
	/* Moving it to the right sysclk and ES rev base */
	dpll_param_p = dpll_param_p + 3 * clk_index + sil_index;
	/* IVA DPLL (set to 12*20=240MHz) */
	sr32(CM_CLKEN_PLL_IVA2, 0, 3, PLL_STOP);
	wait_on_value(BIT0, 0, CM_IDLEST_PLL_IVA2, LDELAY);
	sr32(CM_CLKSEL2_PLL_IVA2, 0, 5, dpll_param_p->m2);	/* set M2 */
	sr32(CM_CLKSEL1_PLL_IVA2, 8, 11, dpll_param_p->m);	/* set M */
	sr32(CM_CLKSEL1_PLL_IVA2, 0, 7, dpll_param_p->n);	/* set N */
	sr32(CM_CLKEN_PLL_IVA2, 4, 4, dpll_param_p->fsel);	/* FREQSEL */
	sr32(CM_CLKEN_PLL_IVA2, 0, 3, PLL_LOCK);	/* lock mode */
	wait_on_value(BIT0, 1, CM_IDLEST_PLL_IVA2, LDELAY);

	/* Set up GPTimers to sys_clk source only */
	sr32(CM_CLKSEL_PER, 0, 8, 0xff);
	sr32(CM_CLKSEL_WKUP, 0, 1, 1);

	delay(5000);
}

/*****************************************
 * Routine: secure_unlock
 * Description: Setup security registers for access
 * (GP Device only)
 *****************************************/
void secure_unlock(void)
{
	/* Permission values for registers -Full fledged permissions to all */
#define UNLOCK_1 0xFFFFFFFF
#define UNLOCK_2 0x00000000
#define UNLOCK_3 0x0000FFFF
	/* Protection Module Register Target APE (PM_RT) */
	__raw_writel(UNLOCK_1, RT_REQ_INFO_PERMISSION_1);
	__raw_writel(UNLOCK_1, RT_READ_PERMISSION_0);
	__raw_writel(UNLOCK_1, RT_WRITE_PERMISSION_0);
	__raw_writel(UNLOCK_2, RT_ADDR_MATCH_1);

	__raw_writel(UNLOCK_3, GPMC_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, GPMC_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, GPMC_WRITE_PERMISSION_0);

	__raw_writel(UNLOCK_3, OCM_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, OCM_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, OCM_WRITE_PERMISSION_0);
	__raw_writel(UNLOCK_2, OCM_ADDR_MATCH_2);

	/* IVA Changes */
	__raw_writel(UNLOCK_3, IVA2_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, IVA2_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, IVA2_WRITE_PERMISSION_0);

	__raw_writel(UNLOCK_1, SMS_RG_ATT0);	/* SDRC region 0 public */
}

/**********************************************************
 * Routine: try_unlock_sram()
 * Description: If chip is GP type, unlock the SRAM for
 *  general use.
 ***********************************************************/
void try_unlock_memory(void)
{
	int mode;

	/* if GP device unlock device SRAM for general use */
	/* secure code breaks for Secure/Emulation device - HS/E/T */
	mode = get_device_type();
	if (mode == GP_DEVICE)
		secure_unlock();
	return;
}

/**********************************************************
 * Routine: s_init
 * Description: Does early system init of muxing and clocks.
 * - Called at time when only stack is available.
 **********************************************************/

void s_init(void)
{
	watchdog_init();
#ifdef CONFIG_3430_AS_3410
	/* setup the scalability control register for
	 * 3430 to work in 3410 mode
	 */
	__raw_writel(0x5ABF, CONTROL_SCALABLE_OMAP_OCP);
#endif
	try_unlock_memory();
	set_muxconf_regs();
	delay(100);
	per_clocks_enable();
	prcm_init();
	config_3430sdram_ddr();
}

/*******************************************************
 * Routine: misc_init_r
 * Description: Init ethernet (done here so udelay works)
 ********************************************************/
int misc_init_r(void)
{
	int rev;

	print_cpuinfo();
	printf("Board detected: ");
#ifdef CONFIG_GTA04
	rev = gta04_revision();
	printf("GTA04A%d\n", rev);
#else
	rev = beagle_revision();
	switch (rev) {
	case REVISION_AXBX:
		printf("Beagle Rev Ax/Bx\n");
		break;
	case REVISION_CX:
		printf("Beagle Rev C1/C2/C3\n");
		break;
	case REVISION_C4:
		if (identify_xm_ddr() == NUMONYX_MCP)
			printf("Beagle Rev C4 from Special Computing\n");
		else
			printf("Beagle Rev C4\n");
		break;
	case REVISION_XM:
		printf("Beagle xM Rev A\n");
		break;
	// detect B/C if possible
	default:
		printf("Beagle Rev unknown (0x%02x)\n", rev);
	}
#endif
	printf("SYSBOOT[5:0]: 0x%02x/%d\n",
			get_sysboot_value(), get_sysboot_value());
	printf("GMPC Memory: %d\n", get_mem_type());
	printf("SDRC Memory: ");
	switch (identify_real_ddr()) {
		case SAMSUNG_MCP: printf("SAMSUNG MCP 512MB/bank\n"); break;
		case NUMONYX_MCP: printf("Numonyx MCP 512MB/bank\n"); break;
		case MICRON_MCP512: printf("Micron MCP 256MB/bank\n"); break;
		case MICRON_MCP256: printf("Micron MCP 128MB/bank\n"); break;
		case DDR_ONLY: printf("DDR 128MB/bank\n"); break;
		default: printf("unknown assuming 128MB/bank\n"); break;
	}
#ifdef CONFIG_NAND
	if (get_mem_type() == GPMC_NAND) {
		int	mfr, id;
		extern int nand_readid(int *mfr, int *id);
		identify_xm_ddr();	// may initialize something
		nand_readid(&mfr, &id);
		printf("NAND: mfr=0x%02x id=0x%02x\n", mfr, id);
	}
#endif
	return 0;
}

/******************************************************
 * Routine: wait_for_command_complete
 * Description: Wait for posting to finish on watchdog
 ******************************************************/
void wait_for_command_complete(unsigned int wd_base)
{
	int pending = 1;
	do {
		pending = __raw_readl(wd_base + WWPS);
	} while (pending);
}

/****************************************
 * Routine: watchdog_init
 * Description: Shut down watch dogs
 *****************************************/
void watchdog_init(void)
{
	/* There are 3 watch dogs WD1=Secure, WD2=MPU, WD3=IVA. WD1 is
	 * either taken care of by ROM (HS/EMU) or not accessible (GP).
	 * We need to take care of WD2-MPU or take a PRCM reset.  WD3
	 * should not be running and does not generate a PRCM reset.
	 */
	sr32(CM_FCLKEN_WKUP, 5, 1, 1);
	sr32(CM_ICLKEN_WKUP, 5, 1, 1);
	wait_on_value(BIT5, 0x20, CM_IDLEST_WKUP, 5);	/* some issue here */

	__raw_writel(WD_UNLOCK1, WD2_BASE + WSPR);
	wait_for_command_complete(WD2_BASE);
	__raw_writel(WD_UNLOCK2, WD2_BASE + WSPR);
}

/**********************************************
 * Routine: dram_init
 * Description: sets uboots idea of sdram size
 **********************************************/
int dram_init(void)
{
	return 0;
}

/*****************************************************************
 * Routine: peripheral_enable
 * Description: Enable the clks & power for perifs (GPT2, UART1,...)
 ******************************************************************/
void per_clocks_enable(void)
{
	/* Enable GP2 timer. */
	sr32(CM_CLKSEL_PER, 0, 1, 0x1);	/* GPT2 = sys clk */
	sr32(CM_ICLKEN_PER, 3, 1, 0x1);	/* ICKen GPT2 */
	sr32(CM_FCLKEN_PER, 3, 1, 0x1);	/* FCKen GPT2 */

#ifdef CFG_NS16550
	/* UART1 clocks */
	sr32(CM_FCLKEN1_CORE, 13, 1, 0x1);
	sr32(CM_ICLKEN1_CORE, 13, 1, 0x1);

	/* UART 3 Clocks */
	sr32(CM_FCLKEN_PER, 11, 1, 0x1);
	sr32(CM_ICLKEN_PER, 11, 1, 0x1);

#endif

#ifdef CONFIG_DRIVER_OMAP34XX_I2C
	/* Turn on all 3 I2C clocks */
	sr32(CM_FCLKEN1_CORE, 15, 3, 0x7);
	sr32(CM_ICLKEN1_CORE, 15, 3, 0x7);	/* I2C1,2,3 = on */
#endif

	/* Enable the ICLK for 32K Sync Timer as its used in udelay */
	sr32(CM_ICLKEN_WKUP, 2, 1, 0x1);

	sr32(CM_FCLKEN_IVA2, 0, 32, FCK_IVA2_ON);
	sr32(CM_FCLKEN1_CORE, 0, 32, FCK_CORE1_ON);
	sr32(CM_ICLKEN1_CORE, 0, 32, ICK_CORE1_ON);
	sr32(CM_ICLKEN2_CORE, 0, 32, ICK_CORE2_ON);
	sr32(CM_FCLKEN_WKUP, 0, 32, FCK_WKUP_ON);
	sr32(CM_ICLKEN_WKUP, 0, 32, ICK_WKUP_ON);
	sr32(CM_FCLKEN_DSS, 0, 32, FCK_DSS_ON);
	sr32(CM_ICLKEN_DSS, 0, 32, ICK_DSS_ON);
	sr32(CM_FCLKEN_CAM, 0, 32, FCK_CAM_ON);
	sr32(CM_ICLKEN_CAM, 0, 32, ICK_CAM_ON);
	sr32(CM_FCLKEN_PER, 0, 32, FCK_PER_ON);
	sr32(CM_ICLKEN_PER, 0, 32, ICK_PER_ON);

	/* Enable GPIO 5 & GPIO 6 clocks */
	sr32(CM_FCLKEN_PER, 17, 2, 0x3);
	sr32(CM_ICLKEN_PER, 17, 2, 0x3);

	delay(1000);
}

/* Set MUX for UART, GPMC, SDRC, GPIO */

#define 	MUX_VAL(OFFSET,VALUE)\
		__raw_writew((VALUE), OMAP34XX_CTRL_BASE + (OFFSET));

#define		CP(x)	(CONTROL_PADCONF_##x)
/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 * The commented string gives the final mux configuration for that pin
 */
#define MUX_DEFAULT()\
	MUX_VAL(CP(SDRC_D0),        (IEN  | PTD | DIS | M0)) /*SDRC_D0*/\
	MUX_VAL(CP(SDRC_D1),        (IEN  | PTD | DIS | M0)) /*SDRC_D1*/\
	MUX_VAL(CP(SDRC_D2),        (IEN  | PTD | DIS | M0)) /*SDRC_D2*/\
	MUX_VAL(CP(SDRC_D3),        (IEN  | PTD | DIS | M0)) /*SDRC_D3*/\
	MUX_VAL(CP(SDRC_D4),        (IEN  | PTD | DIS | M0)) /*SDRC_D4*/\
	MUX_VAL(CP(SDRC_D5),        (IEN  | PTD | DIS | M0)) /*SDRC_D5*/\
	MUX_VAL(CP(SDRC_D6),        (IEN  | PTD | DIS | M0)) /*SDRC_D6*/\
	MUX_VAL(CP(SDRC_D7),        (IEN  | PTD | DIS | M0)) /*SDRC_D7*/\
	MUX_VAL(CP(SDRC_D8),        (IEN  | PTD | DIS | M0)) /*SDRC_D8*/\
	MUX_VAL(CP(SDRC_D9),        (IEN  | PTD | DIS | M0)) /*SDRC_D9*/\
	MUX_VAL(CP(SDRC_D10),       (IEN  | PTD | DIS | M0)) /*SDRC_D10*/\
	MUX_VAL(CP(SDRC_D11),       (IEN  | PTD | DIS | M0)) /*SDRC_D11*/\
	MUX_VAL(CP(SDRC_D12),       (IEN  | PTD | DIS | M0)) /*SDRC_D12*/\
	MUX_VAL(CP(SDRC_D13),       (IEN  | PTD | DIS | M0)) /*SDRC_D13*/\
	MUX_VAL(CP(SDRC_D14),       (IEN  | PTD | DIS | M0)) /*SDRC_D14*/\
	MUX_VAL(CP(SDRC_D15),       (IEN  | PTD | DIS | M0)) /*SDRC_D15*/\
	MUX_VAL(CP(SDRC_D16),       (IEN  | PTD | DIS | M0)) /*SDRC_D16*/\
	MUX_VAL(CP(SDRC_D17),       (IEN  | PTD | DIS | M0)) /*SDRC_D17*/\
	MUX_VAL(CP(SDRC_D18),       (IEN  | PTD | DIS | M0)) /*SDRC_D18*/\
	MUX_VAL(CP(SDRC_D19),       (IEN  | PTD | DIS | M0)) /*SDRC_D19*/\
	MUX_VAL(CP(SDRC_D20),       (IEN  | PTD | DIS | M0)) /*SDRC_D20*/\
	MUX_VAL(CP(SDRC_D21),       (IEN  | PTD | DIS | M0)) /*SDRC_D21*/\
	MUX_VAL(CP(SDRC_D22),       (IEN  | PTD | DIS | M0)) /*SDRC_D22*/\
	MUX_VAL(CP(SDRC_D23),       (IEN  | PTD | DIS | M0)) /*SDRC_D23*/\
	MUX_VAL(CP(SDRC_D24),       (IEN  | PTD | DIS | M0)) /*SDRC_D24*/\
	MUX_VAL(CP(SDRC_D25),       (IEN  | PTD | DIS | M0)) /*SDRC_D25*/\
	MUX_VAL(CP(SDRC_D26),       (IEN  | PTD | DIS | M0)) /*SDRC_D26*/\
	MUX_VAL(CP(SDRC_D27),       (IEN  | PTD | DIS | M0)) /*SDRC_D27*/\
	MUX_VAL(CP(SDRC_D28),       (IEN  | PTD | DIS | M0)) /*SDRC_D28*/\
	MUX_VAL(CP(SDRC_D29),       (IEN  | PTD | DIS | M0)) /*SDRC_D29*/\
	MUX_VAL(CP(SDRC_D30),       (IEN  | PTD | DIS | M0)) /*SDRC_D30*/\
	MUX_VAL(CP(SDRC_D31),       (IEN  | PTD | DIS | M0)) /*SDRC_D31*/\
	MUX_VAL(CP(SDRC_CLK),       (IEN  | PTD | DIS | M0)) /*SDRC_CLK*/\
	MUX_VAL(CP(SDRC_DQS0),      (IEN  | PTD | DIS | M0)) /*SDRC_DQS0*/\
	MUX_VAL(CP(SDRC_DQS1),      (IEN  | PTD | DIS | M0)) /*SDRC_DQS1*/\
	MUX_VAL(CP(SDRC_DQS2),      (IEN  | PTD | DIS | M0)) /*SDRC_DQS2*/\
	MUX_VAL(CP(SDRC_DQS3),      (IEN  | PTD | DIS | M0)) /*SDRC_DQS3*/\
	MUX_VAL(CP(GPMC_A1),        (IDIS | PTD | DIS | M0)) /*GPMC_A1*/\
	MUX_VAL(CP(GPMC_A2),        (IDIS | PTD | DIS | M0)) /*GPMC_A2*/\
	MUX_VAL(CP(GPMC_A3),        (IDIS | PTD | DIS | M0)) /*GPMC_A3*/\
	MUX_VAL(CP(GPMC_A4),        (IDIS | PTD | DIS | M0)) /*GPMC_A4*/\
	MUX_VAL(CP(GPMC_A5),        (IDIS | PTD | DIS | M0)) /*GPMC_A5*/\
	MUX_VAL(CP(GPMC_A6),        (IDIS | PTD | DIS | M0)) /*GPMC_A6*/\
	MUX_VAL(CP(GPMC_A7),        (IDIS | PTD | DIS | M0)) /*GPMC_A7*/\
	MUX_VAL(CP(GPMC_A8),        (IDIS | PTD | DIS | M0)) /*GPMC_A8*/\
	MUX_VAL(CP(GPMC_A9),        (IDIS | PTD | DIS | M0)) /*GPMC_A9*/\
	MUX_VAL(CP(GPMC_A10),       (IDIS | PTD | DIS | M0)) /*GPMC_A10*/\
	MUX_VAL(CP(GPMC_D0),        (IEN  | PTD | DIS | M0)) /*GPMC_D0*/\
	MUX_VAL(CP(GPMC_D1),        (IEN  | PTD | DIS | M0)) /*GPMC_D1*/\
	MUX_VAL(CP(GPMC_D2),        (IEN  | PTD | DIS | M0)) /*GPMC_D2*/\
	MUX_VAL(CP(GPMC_D3),        (IEN  | PTD | DIS | M0)) /*GPMC_D3*/\
	MUX_VAL(CP(GPMC_D4),        (IEN  | PTD | DIS | M0)) /*GPMC_D4*/\
	MUX_VAL(CP(GPMC_D5),        (IEN  | PTD | DIS | M0)) /*GPMC_D5*/\
	MUX_VAL(CP(GPMC_D6),        (IEN  | PTD | DIS | M0)) /*GPMC_D6*/\
	MUX_VAL(CP(GPMC_D7),        (IEN  | PTD | DIS | M0)) /*GPMC_D7*/\
	MUX_VAL(CP(GPMC_D8),        (IEN  | PTD | DIS | M0)) /*GPMC_D8*/\
	MUX_VAL(CP(GPMC_D9),        (IEN  | PTD | DIS | M0)) /*GPMC_D9*/\
	MUX_VAL(CP(GPMC_D10),       (IEN  | PTD | DIS | M0)) /*GPMC_D10*/\
	MUX_VAL(CP(GPMC_D11),       (IEN  | PTD | DIS | M0)) /*GPMC_D11*/\
	MUX_VAL(CP(GPMC_D12),       (IEN  | PTD | DIS | M0)) /*GPMC_D12*/\
	MUX_VAL(CP(GPMC_D13),       (IEN  | PTD | DIS | M0)) /*GPMC_D13*/\
	MUX_VAL(CP(GPMC_D14),       (IEN  | PTD | DIS | M0)) /*GPMC_D14*/\
	MUX_VAL(CP(GPMC_D15),       (IEN  | PTD | DIS | M0)) /*GPMC_D15*/\
	MUX_VAL(CP(GPMC_nCS0),      (IDIS | PTU | EN  | M0)) /*GPMC_nCS0*/\
	MUX_VAL(CP(GPMC_nCS1),      (IDIS | PTU | EN  | M0)) /*GPMC_nCS1*/\
	MUX_VAL(CP(GPMC_nCS2),      (IDIS | PTU | EN  | M0)) /*GPMC_nCS2*/\
	MUX_VAL(CP(GPMC_nCS3),      (IDIS | PTU | EN  | M0)) /*GPMC_nCS3*/\
	MUX_VAL(CP(GPMC_nCS4),      (IDIS | PTU | EN  | M0)) /*GPMC_nCS4*/\
	MUX_VAL(CP(GPMC_nCS5),      (IDIS | PTD | DIS | M0)) /*GPMC_nCS5*/\
	MUX_VAL(CP(GPMC_nCS6),      (IEN  | PTD | DIS | M1)) /*GPMC_nCS6*/\
	MUX_VAL(CP(GPMC_nCS7),      (IEN  | PTU | EN  | M1)) /*GPMC_nCS7*/\
	MUX_VAL(CP(GPMC_CLK),       (IDIS | PTD | DIS | M0)) /*GPMC_CLK*/\
	MUX_VAL(CP(GPMC_nADV_ALE),  (IDIS | PTD | DIS | M0)) /*GPMC_nADV_ALE*/\
	MUX_VAL(CP(GPMC_nOE),       (IDIS | PTD | DIS | M0)) /*GPMC_nOE*/\
	MUX_VAL(CP(GPMC_nWE),       (IDIS | PTD | DIS | M0)) /*GPMC_nWE*/\
	MUX_VAL(CP(GPMC_nBE0_CLE),  (IDIS | PTD | DIS | M0)) /*GPMC_nBE0_CLE*/\
	MUX_VAL(CP(GPMC_nBE1),      (IEN  | PTD | DIS | M0)) /*GPIO_61*/\
	MUX_VAL(CP(GPMC_nWP),       (IEN  | PTD | DIS | M0)) /*GPMC_nWP*/\
	MUX_VAL(CP(GPMC_WAIT0),     (IEN  | PTU | EN  | M0)) /*GPMC_WAIT0*/\
	MUX_VAL(CP(GPMC_WAIT1),     (IEN  | PTU | EN  | M0)) /*GPMC_WAIT1*/\
	MUX_VAL(CP(GPMC_WAIT2),     (IEN  | PTU | EN  | M0)) /*GPIO_64*/\
	MUX_VAL(CP(GPMC_WAIT3),     (IEN  | PTU | EN  | M0)) /*GPIO_65*/\
	MUX_VAL(CP(DSS_DATA18),     (IEN  | PTD | DIS | M4)) /*GPIO_88*/\
	MUX_VAL(CP(DSS_DATA19),     (IEN  | PTD | DIS | M4)) /*GPIO_89*/\
	MUX_VAL(CP(DSS_DATA20),     (IEN  | PTD | DIS | M4)) /*GPIO_90*/\
	MUX_VAL(CP(DSS_DATA21),     (IEN  | PTD | DIS | M4)) /*GPIO_91*/\
	MUX_VAL(CP(CAM_WEN),        (IEN  | PTD | DIS | M4)) /*GPIO_167*/\
	MUX_VAL(CP(MMC1_CLK),       (IDIS | PTU | EN  | M0)) /*MMC1_CLK*/\
	MUX_VAL(CP(MMC1_CMD),       (IEN  | PTU | EN  | M0)) /*MMC1_CMD*/\
	MUX_VAL(CP(MMC1_DAT0),      (IEN  | PTU | EN  | M0)) /*MMC1_DAT0*/\
	MUX_VAL(CP(MMC1_DAT1),      (IEN  | PTU | EN  | M0)) /*MMC1_DAT1*/\
	MUX_VAL(CP(MMC1_DAT2),      (IEN  | PTU | EN  | M0)) /*MMC1_DAT2*/\
	MUX_VAL(CP(MMC1_DAT3),      (IEN  | PTU | EN  | M0)) /*MMC1_DAT3*/\
	MUX_VAL(CP(MMC1_DAT4),      (IEN  | PTU | EN  | M0)) /*MMC1_DAT4*/\
	MUX_VAL(CP(MMC1_DAT5),      (IEN  | PTU | EN  | M0)) /*MMC1_DAT5*/\
	MUX_VAL(CP(MMC1_DAT6),      (IEN  | PTU | EN  | M0)) /*MMC1_DAT6*/\
	MUX_VAL(CP(MMC1_DAT7),      (IEN  | PTU | EN  | M0)) /*MMC1_DAT7*/\
	MUX_VAL(CP(UART1_TX),       (IDIS | PTD | DIS | M0)) /*UART1_TX*/\
	MUX_VAL(CP(UART1_RTS),      (IDIS | PTD | DIS | M4)) /*GPIO_149*/\
	MUX_VAL(CP(UART1_CTS),      (IDIS | PTD | DIS | M4)) /*GPIO_150*/\
	MUX_VAL(CP(UART1_RX),       (IEN  | PTD | DIS | M0)) /*UART1_RX*/\
	MUX_VAL(CP(UART3_CTS_RCTX), (IEN  | PTD | EN  | M0)) /*UART3_CTS_RCTX */\
	MUX_VAL(CP(UART3_RTS_SD),   (IDIS | PTD | DIS | M0)) /*UART3_RTS_SD */\
	MUX_VAL(CP(UART3_RX_IRRX),  (IEN  | PTD | DIS | M0)) /*UART3_RX_IRRX*/\
	MUX_VAL(CP(UART3_TX_IRTX),  (IDIS | PTD | DIS | M0)) /*UART3_TX_IRTX*/\
	MUX_VAL(CP(I2C1_SCL),       (IEN  | PTU | EN  | M0)) /*I2C1_SCL*/\
	MUX_VAL(CP(I2C1_SDA),       (IEN  | PTU | EN  | M0)) /*I2C1_SDA*/\
	MUX_VAL(CP(I2C2_SCL),       (IEN  | PTU | EN  | M0)) /*I2C2_SCL*/\
	MUX_VAL(CP(I2C2_SDA),       (IEN  | PTU | EN  | M0)) /*I2C2_SDA*/\
	MUX_VAL(CP(I2C3_SCL),       (IEN  | PTU | EN  | M0)) /*I2C3_SCL*/\
	MUX_VAL(CP(I2C3_SDA),       (IEN  | PTU | EN  | M0)) /*I2C3_SDA*/\
	MUX_VAL(CP(I2C4_SCL),       (IEN  | PTU | EN  | M0)) /*I2C4_SCL*/\
	MUX_VAL(CP(I2C4_SDA),       (IEN  | PTU | EN  | M0)) /*I2C4_SDA*/\
	MUX_VAL(CP(McSPI1_CLK),     (IEN  | PTU | EN  | M4)) /*GPIO_171*/\
	MUX_VAL(CP(McSPI1_SIMO),    (IEN  | PTU | EN  | M4)) /*GPIO_172*/\
	MUX_VAL(CP(McSPI1_SOMI),    (IEN  | PTU | EN  | M4)) /*GPIO_173*/\
	MUX_VAL(CP(McBSP1_DX),      (IEN  | PTD | DIS | M4)) /*GPIO_158*/\
	MUX_VAL(CP(SYS_32K),        (IEN  | PTD | DIS | M0)) /*SYS_32K*/\
	MUX_VAL(CP(SYS_BOOT0),      (IEN  | PTD | DIS | M4)) /*GPIO_2 */\
	MUX_VAL(CP(SYS_BOOT1),      (IEN  | PTD | DIS | M4)) /*GPIO_3 */\
	MUX_VAL(CP(SYS_BOOT2),      (IEN  | PTD | DIS | M4)) /*GPIO_4 */\
	MUX_VAL(CP(SYS_BOOT3),      (IEN  | PTD | DIS | M4)) /*GPIO_5 */\
	MUX_VAL(CP(SYS_BOOT4),      (IEN  | PTD | DIS | M4)) /*GPIO_6 */\
	MUX_VAL(CP(SYS_BOOT5),      (IEN  | PTD | DIS | M4)) /*GPIO_7 */\
	MUX_VAL(CP(SYS_BOOT6),      (IEN  | PTD | DIS | M4)) /*GPIO_8 */\
	MUX_VAL(CP(SYS_CLKOUT2),    (IEN  | PTU | EN  | M4)) /*GPIO_186*/\
	MUX_VAL(CP(JTAG_nTRST),     (IEN  | PTD | DIS | M0)) /*JTAG_nTRST*/\
	MUX_VAL(CP(JTAG_TCK),       (IEN  | PTD | DIS | M0)) /*JTAG_TCK*/\
	MUX_VAL(CP(JTAG_TMS),       (IEN  | PTD | DIS | M0)) /*JTAG_TMS*/\
	MUX_VAL(CP(JTAG_TDI),       (IEN  | PTD | DIS | M0)) /*JTAG_TDI*/\
	MUX_VAL(CP(JTAG_EMU0),      (IEN  | PTD | DIS | M0)) /*JTAG_EMU0*/\
	MUX_VAL(CP(JTAG_EMU1),      (IEN  | PTD | DIS | M0)) /*JTAG_EMU1*/\
	MUX_VAL(CP(ETK_CLK),        (IEN  | PTD | DIS | M4)) /*GPIO_12*/\
	MUX_VAL(CP(ETK_CTL),        (IEN  | PTD | DIS | M4)) /*GPIO_13*/\
	MUX_VAL(CP(ETK_D0),         (IEN  | PTD | DIS | M4)) /*GPIO_14*/\
	MUX_VAL(CP(ETK_D1),         (IEN  | PTD | DIS | M4)) /*GPIO_15*/\
	MUX_VAL(CP(ETK_D2),         (IEN  | PTD | DIS | M4)) /*GPIO_16*/\
	MUX_VAL(CP(ETK_D11),        (IEN  | PTD | DIS | M4)) /*GPIO_25*/\
	MUX_VAL(CP(ETK_D12),        (IEN  | PTD | DIS | M4)) /*GPIO_26*/\
	MUX_VAL(CP(ETK_D13),        (IEN  | PTD | DIS | M4)) /*GPIO_27*/\
	MUX_VAL(CP(ETK_D14),        (IEN  | PTD | DIS | M4)) /*GPIO_28*/\
	MUX_VAL(CP(ETK_D15),        (IEN  | PTD | DIS | M4)) /*GPIO_29 */\
	MUX_VAL(CP(sdrc_cke0),      (IDIS | PTU | EN  | M0)) /*sdrc_cke0 */\
	MUX_VAL(CP(sdrc_cke1),      (IDIS | PTD | DIS | M7)) /*sdrc_cke1 not used*/

/**********************************************************
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers
 *              specific to the hardware. Many pins need
 *              to be moved from protect to primary mode.
 *********************************************************/
void set_muxconf_regs(void)
{
	MUX_DEFAULT();
#ifdef CONFIG_GTA04
	// enable backlight as first boot indication
	MUX_VAL(CP(GPMC_nCS6),      (IEN  | PTU | EN  | M4)) /*GPMC_nCS6=gpio_57=gpt11_pwm*/
	MUX_VAL(CP(UART3_CTS_RCTX), (IEN  | PTD | EN  | M4)) /*UART3_CTS_RCTX */
	MUX_VAL(CP(UART3_RTS_SD),   (IDIS | PTD | DIS | M4)) /*UART3_RTS_SD */
	MUX_VAL(CP(ETK_CTL),        (IEN  | PTU | DIS | M4)) /*GPIO_13*/
#endif
}

/**********************************************************
 * Routine: nand+_init
 * Description: Set up nand for nand and jffs2 commands
 *********************************************************/

int nand_init(void)
{
#ifdef CONFIG_NAND
	/* global settings */
	__raw_writel(0x10, GPMC_SYSCONFIG);	/* smart idle */
	__raw_writel(0x0, GPMC_IRQENABLE);	/* isr's sources masked */
	__raw_writel(0, GPMC_TIMEOUT_CONTROL);/* timeout disable */

	/* Set the GPMC Vals, NAND is mapped at CS0, oneNAND at CS0.
	 *  We configure only GPMC CS0 with required values. Configiring other devices
	 *  at other CS is done in u-boot. So we don't have to bother doing it here.
	 */
	__raw_writel(0 , GPMC_CONFIG7 + GPMC_CONFIG_CS0);
	delay(1000);

#ifdef CFG_NAND_K9F1G08R0A
	if ((get_mem_type() == GPMC_NAND) || (get_mem_type() == MMC_NAND)) {
		__raw_writel(M_NAND_GPMC_CONFIG1, GPMC_CONFIG1 + GPMC_CONFIG_CS0);
		__raw_writel(M_NAND_GPMC_CONFIG2, GPMC_CONFIG2 + GPMC_CONFIG_CS0);
		__raw_writel(M_NAND_GPMC_CONFIG3, GPMC_CONFIG3 + GPMC_CONFIG_CS0);
		__raw_writel(M_NAND_GPMC_CONFIG4, GPMC_CONFIG4 + GPMC_CONFIG_CS0);
		__raw_writel(M_NAND_GPMC_CONFIG5, GPMC_CONFIG5 + GPMC_CONFIG_CS0);
		__raw_writel(M_NAND_GPMC_CONFIG6, GPMC_CONFIG6 + GPMC_CONFIG_CS0);

		/* Enable the GPMC Mapping */
		__raw_writel((((OMAP34XX_GPMC_CS0_SIZE & 0xF)<<8) |
			     ((NAND_BASE_ADR>>24) & 0x3F) |
			     (1<<6)),  (GPMC_CONFIG7 + GPMC_CONFIG_CS0));
		delay(2000);

		if (nand_chip()) {
#ifdef CFG_PRINTF
			printf("Unsupported Chip!\n");
#endif
//			return 1;
		}
	}
#endif

#ifdef CFG_ONENAND
	if ((get_mem_type() == GPMC_ONENAND) || (get_mem_type() == MMC_ONENAND)) {
		__raw_writel(ONENAND_GPMC_CONFIG1, GPMC_CONFIG1 + GPMC_CONFIG_CS0);
		__raw_writel(ONENAND_GPMC_CONFIG2, GPMC_CONFIG2 + GPMC_CONFIG_CS0);
		__raw_writel(ONENAND_GPMC_CONFIG3, GPMC_CONFIG3 + GPMC_CONFIG_CS0);
		__raw_writel(ONENAND_GPMC_CONFIG4, GPMC_CONFIG4 + GPMC_CONFIG_CS0);
		__raw_writel(ONENAND_GPMC_CONFIG5, GPMC_CONFIG5 + GPMC_CONFIG_CS0);
		__raw_writel(ONENAND_GPMC_CONFIG6, GPMC_CONFIG6 + GPMC_CONFIG_CS0);

		/* Enable the GPMC Mapping */
		__raw_writel((((OMAP34XX_GPMC_CS0_SIZE & 0xF)<<8) |
			     ((ONENAND_BASE>>24) & 0x3F) |
			     (1<<6)),  (GPMC_CONFIG7 + GPMC_CONFIG_CS0));
		delay(2000);

		if (onenand_chip()) {
#ifdef CFG_PRINTF
			printf("OneNAND Unsupported !\n");
#endif
//			return 1;
		}
	}
#endif
#endif
	return 0;
}

#ifdef CONFIG_GTA04
#define DEBUG_LED1			149	/* gpio */
// #define DEBUG_LED2			57	/* gpio */
#else
#define DEBUG_LED1			149	/* gpio */
#define DEBUG_LED2			150	/* gpio */
#endif

void blinkLEDs(void)
{
	void *p;

	/* Alternately turn the LEDs on and off */
	p = (unsigned long *)OMAP34XX_GPIO5_BASE;
	while (1) {
		/* turn LED1 on and LED2 off */
		*(unsigned long *)(p + 0x94) = 1 << (DEBUG_LED1 % 32);
#ifdef DEBUG_LED2
		*(unsigned long *)(p + 0x90) = 1 << (DEBUG_LED2 % 32);
#endif
		/* delay for a while */
		delay(1000);

		/* turn LED1 off and LED2 on */
		*(unsigned long *)(p + 0x90) = 1 << (DEBUG_LED1 % 32);
#ifdef DEBUG_LED2
		*(unsigned long *)(p + 0x94) = 1 << (DEBUG_LED2 % 32);
#endif
		/* delay for a while */
		delay(1000);
	}
}

/* optionally do something like blinking LED */
void board_hang(void)
{
	while (1)
		blinkLEDs();
}

/******************************************************************************
 * Dummy function to handle errors for EABI incompatibility
 *****************************************************************************/
void raise(void)
{
}

/******************************************************************************
 * Dummy function to handle errors for EABI incompatibility
 *****************************************************************************/
void abort(void)
{
}
