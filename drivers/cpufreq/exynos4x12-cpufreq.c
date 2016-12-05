/*
 * Copyright (c) 2010-2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS4X12 - CPU frequency scaling support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/


/*
 * BASETIS:
 *
 * 	1) Tablas de frecuencias adaptadas a las indicadas en el datasheet
 * 	2) Tabla de voltajes cambiada a valores indicados en el datasheet
 * 	3) Añadidos mensajes de debug para comprobar valores de registros
 * 	4) Reconfiguración del MPLL para una salida de 800MHz. -> Baja el consumo !!
 *
 * 	Comentarios:
 * 		1) Los warnings al compilar se refieren a estructuras no usadas. Obviarlos.
 * 		2) Al modificar el MPLL en 'caliente' el minicom se 'cuelga'.
 * 		3) Se cree que la AUART1 utilizada por el minicom depende del MPLL (ACLK100).
 *
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/cpufreq.h>

#include <mach/regs-clock.h>
#include <mach/cpufreq.h>

#if defined(CONFIG_ODROID_X) || (CONFIG_MACH_CLICKARM4412)
// BASETIS: cambio índice de las tablas
//#define CPUFREQ_LEVEL_END	(L16 + 1)
#define CPUFREQ_LEVEL_END	(L12 + 1)
#elif defined(CONFIG_ODROID_X2) || defined(CONFIG_ODROID_U2)
#define CPUFREQ_LEVEL_END	(L18 + 1)
#endif

static int max_support_idx;
static int min_support_idx = (CPUFREQ_LEVEL_END - 1);

static struct clk *cpu_clk;
static struct clk *moutcore;
static struct clk *mout_mpll;
static struct clk *mout_apll;

struct cpufreq_clkdiv {
	unsigned int	index;
	unsigned int	clkdiv;
	unsigned int	clkdiv1;
};

static unsigned int exynos4x12_volt_table[CPUFREQ_LEVEL_END];

#if defined(CONFIG_ODROID_X) || (CONFIG_MACH_CLICKARM4412)
static struct cpufreq_frequency_table exynos4x12_freq_table[] = {
// BASETIS: cambio en los valores. Valor máximo soportado es 1400 MHz.
/*
	{L0, 1800 * 1000},
	{L1, 1704 * 1000},
	{L2, 1600 * 1000},	
	{L3, 1500 * 1000},
	{L4, 1400 * 1000},
	{L5, 1300 * 1000},
	{L6, 1200 * 1000},
	{L7, 1100 * 1000},
	{L8, 1000 * 1000},
	{L9, 900 * 1000},
	{L10,800 * 1000},
	{L11,700 * 1000},
	{L12,600 * 1000},
	{L13,500 * 1000},
	{L14,400 * 1000},
	{L15,300 * 1000},
	{L16,200 * 1000},
	{0, CPUFREQ_TABLE_END},
*/
	{L0, 1400 * 1000},
	{L1, 1300 * 1000},
	{L2, 1200 * 1000},
	{L3, 1100 * 1000},
	{L4, 1000 * 1000},
	{L5, 900 * 1000},
	{L6,800 * 1000},
	{L7,700 * 1000},
	{L8,600 * 1000},
	{L9,500 * 1000},
	{L10,400 * 1000},
	{L11,300 * 1000},
	{L12,200 * 1000},
	{0, CPUFREQ_TABLE_END},
};
#elif defined(CONFIG_ODROID_X2) || defined(CONFIG_ODROID_U2)
static struct cpufreq_frequency_table exynos4x12_freq_table[] = {
	{L0, 2000*1000}, 
	{L1, 1920*1000}, 
	{L2, 1800*1000}, 
	{L3, 1704*1000}, 
	{L4, 1600*1000},
	{L5, 1500*1000}, 
	{L6, 1400*1000}, 
	{L7, 1300*1000}, 
	{L8, 1200*1000}, 
	{L9, 1100*1000},
	{L10, 1000*1000}, 
	{L11, 900*1000}, 
	{L12, 800*1000}, 
	{L13, 700*1000}, 
	{L14, 600*1000},
	{L15, 500*1000}, 
	{L16, 400*1000}, 
	{L17, 300*1000}, 
	{L18, 200*1000},
	{0, CPUFREQ_TABLE_END},
};
#endif
static struct cpufreq_clkdiv exynos4x12_clkdiv_table[CPUFREQ_LEVEL_END];

static unsigned int clkdiv_cpu0_4212[CPUFREQ_LEVEL_END][8] = {
	/*
	 * Clock divider value for following
	 * { DIVCORE, DIVCOREM0, DIVCOREM1, DIVPERIPH,
	 *		DIVATB, DIVPCLK_DBG, DIVAPLL, DIVCORE2 }
	 */
	/* ARM L0: 1500 MHz */
	{ 0, 3, 7, 0, 6, 1, 2, 0 },

	/* ARM L1: 1400 MHz */
	{ 0, 3, 7, 0, 6, 1, 2, 0 },

	/* ARM L2: 1300 MHz */
	{ 0, 3, 7, 0, 5, 1, 2, 0 },

	/* ARM L3: 1200 MHz */
	{ 0, 3, 7, 0, 5, 1, 2, 0 },

	/* ARM L4: 1100 MHz */
	{ 0, 3, 6, 0, 4, 1, 2, 0 },

	/* ARM L5: 1000 MHz */
	{ 0, 2, 5, 0, 4, 1, 1, 0 },

	/* ARM L6: 900 MHz */
	{ 0, 2, 5, 0, 3, 1, 1, 0 },

	/* ARM L7: 800 MHz */
	{ 0, 2, 5, 0, 3, 1, 1, 0 },

	/* ARM L8: 700 MHz */
	{ 0, 2, 4, 0, 3, 1, 1, 0 },

	/* ARM L9: 600 MHz */
	{ 0, 2, 4, 0, 3, 1, 1, 0 },

	/* ARM L10: 500 MHz */
	{ 0, 2, 4, 0, 3, 1, 1, 0 },

	/* ARM L11: 400 MHz */
	{ 0, 2, 4, 0, 3, 1, 1, 0 },

	/* ARM L12: 300 MHz */
	{ 0, 2, 4, 0, 2, 1, 1, 0 },

	/* ARM L13: 200 MHz */
	{ 0, 1, 3, 0, 1, 1, 1, 0 },
};

static unsigned int clkdiv_cpu0_4412[CPUFREQ_LEVEL_END][8] = {
	/*
	 * Clock divider value for following
	 * { DIVCORE, DIVCOREM0, DIVCOREM1, DIVPERIPH,
	 *		DIVATB, DIVPCLK_DBG, DIVAPLL, DIVCORE2 }
	 */
	#if defined(CONFIG_ODROID_X2) || defined(CONFIG_ODROID_U2)
	/* 2000 Mhz Support */
	{ 0, 3, 7, 0, 6, 1, 2, 0},

	/* 1920 Mhz Support */
	{ 0, 3, 7, 0, 6, 1, 2, 0},
	#endif

// BASETIS: cambio en los valores. Valor máximo soportado es 1400 MHz.
// BASETIS: los valores de los divisores son opcionales. (Ver pág. 451 datasheet).

	/* 1800 Mhz Support */
	//{ 0, 3, 7, 0, 6, 1, 2, 0},

	/* 1704 Mhz Support */
	//{ 0, 3, 7, 0, 6, 1, 2, 0},

	/* ARM L0: 1600Mhz */
	//{ 0, 3, 7, 0, 6, 1, 2, 0 },
	/* ARM L0: 1500 MHz */
	//{ 0, 3, 7, 0, 6, 1, 2, 0 },

	/* ARM L0: 1400 MHz */
	{ 0, 3, 7, 0, 6, 1, 2, 0 },

	/* ARM L1: 1300 MHz */
	{ 0, 3, 7, 0, 5, 1, 2, 0 },

	/* ARM L2: 1200 MHz */
	{ 0, 3, 7, 0, 5, 1, 2, 0 },

	/* ARM L3: 1100 MHz */
	{ 0, 3, 6, 0, 4, 1, 2, 0 },

	/* ARM L4: 1000 MHz */
	{ 0, 2, 5, 0, 4, 1, 1, 0 },

	/* ARM L5: 900 MHz */
	{ 0, 2, 5, 0, 3, 1, 1, 0 },

	/* ARM L6: 800 MHz */
	{ 0, 2, 5, 0, 3, 1, 1, 0 },

	/* ARM L7: 700 MHz */
	{ 0, 2, 4, 0, 3, 1, 1, 0 },

	/* ARM L8: 600 MHz */
	{ 0, 2, 4, 0, 3, 1, 1, 0 },

	/* ARM L9: 500 MHz */
	{ 0, 2, 4, 0, 3, 1, 1, 0 },

	/* ARM L10: 400 MHz */
	{ 0, 2, 4, 0, 3, 1, 1, 0 },

	/* ARM L11: 300 MHz */
	{ 0, 2, 4, 0, 2, 1, 1, 0 },

	/* ARM L12: 200 MHz */
	{ 0, 1, 3, 0, 1, 1, 1, 0 },
};

static unsigned int clkdiv_cpu1_4212[CPUFREQ_LEVEL_END][2] = {
	/* Clock divider value for following
	 * { DIVCOPY, DIVHPM }
	 */
	/* ARM L0: 1500 MHz */
	{ 6, 0 },

	/* ARM L1: 1400 MHz */
	{ 6, 0 },

	/* ARM L2: 1300 MHz */
	{ 5, 0 },

	/* ARM L3: 1200 MHz */
	{ 5, 0 },

	/* ARM L4: 1100 MHz */
	{ 4, 0 },

	/* ARM L5: 1000 MHz */
	{ 4, 0 },

	/* ARM L6: 900 MHz */
	{ 3, 0 },

	/* ARM L7: 800 MHz */
	{ 3, 0 },

	/* ARM L8: 700 MHz */
	{ 3, 0 },

	/* ARM L9: 600 MHz */
	{ 3, 0 },

	/* ARM L10: 500 MHz */
	{ 3, 0 },

	/* ARM L11: 400 MHz */
	{ 3, 0 },

	/* ARM L12: 300 MHz */
	{ 3, 0 },

	/* ARM L13: 200 MHz */
	{ 3, 0 },
};

static unsigned int clkdiv_cpu1_4412[CPUFREQ_LEVEL_END][3] = {
	/* Clock divider value for following
	 * { DIVCOPY, DIVHPM, DIVCORES }
	 */
#if defined(CONFIG_ODROID_X2) || defined(CONFIG_ODROID_Q2) || defined(CONFIG_ODROID_U2)
	/* 2000Mhz */
	{ 7, 0, 7 },

	/* 1920Mhz */
	{ 7, 0, 7 },
#endif

// BASETIS: cambio en los valores. Valor máximo soportado es 1400 MHz.
// BASETIS: los valores de los divisores son opcionales. (Ver pág. 451 datasheet).

	/* 1800Mhz */
	//{ 7, 0, 7 },

	/* 1704Mhz */
	//{ 7, 0, 7 },

	/* 1600MHz */
	//{ 6, 0, 7 },

	/* ARM L0: 1500 MHz */
	//{ 6, 0, 7 },

	/* ARM L0: 1400 MHz */
	{ 6, 0, 6 },

	/* ARM L1: 1300 MHz */
	{ 5, 0, 6 },

	/* ARM L2: 1200 MHz */
	{ 5, 0, 5 },

	/* ARM L3: 1100 MHz */
	{ 4, 0, 5 },

	/* ARM L4: 1000 MHz */
	{ 4, 0, 4 },

	/* ARM L5: 900 MHz */
	{ 3, 0, 4 },

	/* ARM L6: 800 MHz */
	{ 3, 0, 3 },

	/* ARM L7: 700 MHz */
	{ 3, 0, 3 },

	/* ARM L8: 600 MHz */
	{ 3, 0, 2 },

	/* ARM L9: 500 MHz */
	{ 3, 0, 2 },

	/* ARM L10: 400 MHz */
	{ 3, 0, 1 },

	/* ARM L11: 300 MHz */
	{ 3, 0, 1 },

	/* ARM L12: 200 MHz */
	{ 3, 0, 0 },
};

static unsigned int exynos4x12_apll_pms_table[CPUFREQ_LEVEL_END] = {
#if defined(CONFIG_ODROID_X2) || defined(CONFIG_ODROID_U2)
	/* 2000MHz */
	((250 << 16) | (3 << 8) | (0x0)),

	/* 1920Mhz */
	((240 << 16) | (3 << 8) | (0x0)),
#endif

// BASETIS: cambio en los valores. Valor máximo soportado es 1400 MHz.
// BASETIS: los valores de configuración del APLL son fijos. (Ver pág. 447 datasheet).
// BASETIS: orden: MDIV, PDIV, SDIV

	/* 1800MHz */
	//((300 << 16) | (4 << 8) | (0x0)),

	/* 1704MHz */
	//((213 << 16) | (3 << 8) | (0x0)),

	/* 1600MHz */
	//((200 << 16) | (3 << 8) | (0x0)),

	/* APLL FOUT L0: 1500 MHz */
	//((250 << 16) | (4 << 8) | (0x0)),

	/* APLL FOUT L0: 1400 MHz */
	((175 << 16) | (3 << 8) | (0x0)),

	/* APLL FOUT L1: 1300 MHz */
	((325 << 16) | (6 << 8) | (0x0)),

	/* APLL FOUT L2: 1200 MHz */
	((200 << 16) | (4 << 8) | (0x0)),

	/* APLL FOUT L3: 1100 MHz */
	((275 << 16) | (6 << 8) | (0x0)),

	/* APLL FOUT L4: 1000 MHz */
	((125 << 16) | (3 << 8) | (0x0)),

	/* APLL FOUT L5: 900 MHz */
	((150 << 16) | (4 << 8) | (0x0)),

	/* APLL FOUT L6: 800 MHz */
	((100 << 16) | (3 << 8) | (0x0)),

	/* APLL FOUT L7: 700 MHz */
	((175 << 16) | (3 << 8) | (0x1)),

	/* APLL FOUT L8: 600 MHz */
	((200 << 16) | (4 << 8) | (0x1)),

	/* APLL FOUT L9: 500 MHz */
	((125 << 16) | (3 << 8) | (0x1)),

	/* APLL FOUT L10 400 MHz */
	((100 << 16) | (3 << 8) | (0x1)),

	/* APLL FOUT L11: 300 MHz */
	((200 << 16) | (4 << 8) | (0x2)),

	/* APLL FOUT L12: 200 MHz */
	((100 << 16) | (3 << 8) | (0x2)),
};

#if defined(CONFIG_ODROID_X2) || defined(CONFIG_ODROID_U2)
static const unsigned int asv_voltage_4x12[CPUFREQ_LEVEL_END] = {
	1425000,	// 2000Mhz (L0)
	1400000,	// 1920Mhz (L1)
	1400000,	// 1800Mhz (L2)
	1350000,	// 1704Mhz (L3)
	1350000,	// 1600Mhz (L4)
	1300000, 	// 1500Mhz (L5)
    1225000, 	// 1400Mhz (L6)
    1175000,	// 1300Mhz (L7)
    1125000,	// 1200Mhz (L8)
    1075000,	// 1100Mhz (L9)
    1037500,	// 1000Mhz (L10)
    1012500,	//  900Mhz (L11)
    1000000,	//  800Mhz (L12)
     987500,    //  700Mhz (L13)
     975000,    //  600Mhz (L14)
     925000,    //  500Mhz (L15)
     925000,    //  400Mhz (L16)
     925000,    //  300Mhz (L17)
     900000,	//  200Mhz (L18)
};
#else
static const unsigned int asv_voltage_4x12[CPUFREQ_LEVEL_END] = {

// BASETIS: cambio en los valores. No se corresponden con el datasheet. (Ver pág. 2837).
/*
	 1450000,
     1400000,
     1400000,
     1350000,
     1325000,
     1275000,
     1225000,
     1175000,
     1137500,
     1112500,
     1100000,
      987500,
      975000,
      925000,
      925000,
      925000,
      925000,
*/
	 1287500,	// 1400MHz (L0)
	 1287500,	// 1300MHz (L1)
	 1187500,	// 1200MHz (L2)
	 1187500,	// 1100MHz (L3)
	 1087500,	// 1000MHz (L4)
	 1087500,	//  900MHz (L5)
	  987500,	//  800MHz (L6)
	  987500,	//  700MHz (L7)
	  987500,	//  600MHz (L8)
	  937500,	//  500MHz (L9)
	  937500,	//  400MHz (L10)
	  937500,	//	300MHz (L11)
	  900000,	//  200MHz (L12)
};
#endif
static void exynos4x12_set_clkdiv(unsigned int div_index)
{
	unsigned int tmp;
	unsigned int stat_cpu1;

	/* Change Divider - CPU0 */

	tmp = exynos4x12_clkdiv_table[div_index].clkdiv;

	__raw_writel(tmp, EXYNOS4_CLKDIV_CPU);

	while (__raw_readl(EXYNOS4_CLKDIV_STATCPU) & 0x11111111)
		cpu_relax();

	/* Change Divider - CPU1 */
	tmp = exynos4x12_clkdiv_table[div_index].clkdiv1;

	__raw_writel(tmp, EXYNOS4_CLKDIV_CPU1);
	if (soc_is_exynos4212())
		stat_cpu1 = 0x11;
	else
		stat_cpu1 = 0x111;

	while (__raw_readl(EXYNOS4_CLKDIV_STATCPU1) & stat_cpu1)
		cpu_relax();
}

static void exynos4x12_set_apll(unsigned int index)
{
	unsigned int tmp, pdiv;
	unsigned int apll_con0;

	// BASETIS: código de debug
	printk(KERN_INFO "BASETIS: exynos4x12_set_apll\n");
	printk(KERN_INFO "BASETIS: exynos4x12_set_apll, index = %u\n", index);

	/* 1. MUX_CORE_SEL = MPLL, ARMCLK uses MPLL for lock time */
	clk_set_parent(moutcore, mout_mpll);

	do {
		cpu_relax();
		tmp = (__raw_readl(EXYNOS4_CLKMUX_STATCPU)
			>> EXYNOS4_CLKSRC_CPU_MUXCORE_SHIFT);
		tmp &= 0x7;
	} while (tmp != 0x2);

	/* 2. Set APLL Lock time */
	pdiv = ((exynos4x12_apll_pms_table[index] >> 8) & 0x3f);

	__raw_writel((pdiv * 250), EXYNOS4_APLL_LOCK);

	/* 3. Change PLL PMS values */
	tmp = __raw_readl(EXYNOS4_APLL_CON0);
	tmp &= ~((0x3ff << 16) | (0x3f << 8) | (0x7 << 0));
	tmp |= exynos4x12_apll_pms_table[index];
	__raw_writel(tmp, EXYNOS4_APLL_CON0);

	/* 4. wait_lock_time */
	do {
		cpu_relax();
		tmp = __raw_readl(EXYNOS4_APLL_CON0);
	} while (!(tmp & (0x1 << EXYNOS4_APLLCON0_LOCKED_SHIFT)));

	/* 5. MUX_CORE_SEL = APLL */
	clk_set_parent(moutcore, mout_apll);

	do {
		cpu_relax();
		tmp = __raw_readl(EXYNOS4_CLKMUX_STATCPU);
		tmp &= EXYNOS4_CLKMUX_STATCPU_MUXCORE_MASK;
	} while (tmp != (0x1 << EXYNOS4_CLKSRC_CPU_MUXCORE_SHIFT));

	// BASETIS: código de debug
	apll_con0 = __raw_readl(EXYNOS4_APLL_CON0);
	printk(KERN_INFO "BASETIS: apll_con0 : %u\n", apll_con0);

}

bool exynos4x12_pms_change(unsigned int old_index, unsigned int new_index)
{
	unsigned int old_pm = exynos4x12_apll_pms_table[old_index] >> 8;
	unsigned int new_pm = exynos4x12_apll_pms_table[new_index] >> 8;

	return (old_pm == new_pm) ? 0 : 1;
}

static void exynos4x12_set_frequency(unsigned int old_index,
				  unsigned int new_index)
{
	unsigned int tmp;

	// BASETIS: código de debug
	printk(KERN_INFO "BASETIS: exynos4x12_set_frequency\n");
	printk(KERN_INFO "BASETIS: exynos4x12_set_frequency, old_index = %u\n", old_index);
	printk(KERN_INFO "BASETIS: exynos4x12_set_frequency, new_index = %u\n", new_index);

	if (old_index > new_index) {
		if (!exynos4x12_pms_change(old_index, new_index)) {
			/* 1. Change the system clock divider values */
			exynos4x12_set_clkdiv(new_index);
			/* 2. Change just s value in apll m,p,s value */
			tmp = __raw_readl(EXYNOS4_APLL_CON0);
			tmp &= ~(0x7 << 0);
			tmp |= (exynos4x12_apll_pms_table[new_index] & 0x7);
			__raw_writel(tmp, EXYNOS4_APLL_CON0);

		} else {
			/* Clock Configuration Procedure */
			/* 1. Change the system clock divider values */
			exynos4x12_set_clkdiv(new_index);
			/* 2. Change the apll m,p,s value */
			exynos4x12_set_apll(new_index);
		}
	} else if (old_index < new_index) {
		if (!exynos4x12_pms_change(old_index, new_index)) {
			/* 1. Change just s value in apll m,p,s value */
			tmp = __raw_readl(EXYNOS4_APLL_CON0);
			tmp &= ~(0x7 << 0);
			tmp |= (exynos4x12_apll_pms_table[new_index] & 0x7);
			__raw_writel(tmp, EXYNOS4_APLL_CON0);
			/* 2. Change the system clock divider values */
			exynos4x12_set_clkdiv(new_index);
		} else {
			/* Clock Configuration Procedure */
			/* 1. Change the apll m,p,s value */
			exynos4x12_set_apll(new_index);
			/* 2. Change the system clock divider values */
			exynos4x12_set_clkdiv(new_index);
		}
	}
}

static void __init set_volt_table(void)
{
	unsigned int i;

	max_support_idx = L0;

	for (i = 0 ; i < CPUFREQ_LEVEL_END ; i++)
		exynos4x12_volt_table[i] = asv_voltage_4x12[i];
}

int exynos4x12_cpufreq_init(struct exynos_dvfs_info *info)
{
	int i;
	unsigned int tmp;
	unsigned long rate;
	//BASETIS:
	unsigned int apll_con0;
	unsigned int mpll_con0;

	set_volt_table();

	cpu_clk = clk_get(NULL, "armclk");
	if (IS_ERR(cpu_clk))
		return PTR_ERR(cpu_clk);

	moutcore = clk_get(NULL, "moutcore");
	if (IS_ERR(moutcore))
		goto err_moutcore;

	mout_mpll = clk_get(NULL, "mout_mpll");
	if (IS_ERR(mout_mpll))
		goto err_mout_mpll;

	rate = clk_get_rate(mout_mpll) / 1000;

	mout_apll = clk_get(NULL, "mout_apll");
	if (IS_ERR(mout_apll))
		goto err_mout_apll;

	for (i = L0; i <  CPUFREQ_LEVEL_END; i++) {

		exynos4x12_clkdiv_table[i].index = i;

		tmp = __raw_readl(EXYNOS4_CLKDIV_CPU);

		tmp &= ~(EXYNOS4_CLKDIV_CPU0_CORE_MASK |
			EXYNOS4_CLKDIV_CPU0_COREM0_MASK |
			EXYNOS4_CLKDIV_CPU0_COREM1_MASK |
			EXYNOS4_CLKDIV_CPU0_PERIPH_MASK |
			EXYNOS4_CLKDIV_CPU0_ATB_MASK |
			EXYNOS4_CLKDIV_CPU0_PCLKDBG_MASK |
			EXYNOS4_CLKDIV_CPU0_APLL_MASK);

		if (soc_is_exynos4212()) {

			// BASETIS: código de debug
			printk(KERN_INFO "BASETIS: exynos4212 \n");

			tmp |= ((clkdiv_cpu0_4212[i][0] << EXYNOS4_CLKDIV_CPU0_CORE_SHIFT) |
				(clkdiv_cpu0_4212[i][1] << EXYNOS4_CLKDIV_CPU0_COREM0_SHIFT) |
				(clkdiv_cpu0_4212[i][2] << EXYNOS4_CLKDIV_CPU0_COREM1_SHIFT) |
				(clkdiv_cpu0_4212[i][3] << EXYNOS4_CLKDIV_CPU0_PERIPH_SHIFT) |
				(clkdiv_cpu0_4212[i][4] << EXYNOS4_CLKDIV_CPU0_ATB_SHIFT) |
				(clkdiv_cpu0_4212[i][5] << EXYNOS4_CLKDIV_CPU0_PCLKDBG_SHIFT) |
				(clkdiv_cpu0_4212[i][6] << EXYNOS4_CLKDIV_CPU0_APLL_SHIFT));
		} else {

			// BASETIS: código de debugBASETIS:
			printk(KERN_INFO "BASETIS: exynos4412 \n");

			tmp &= ~EXYNOS4_CLKDIV_CPU0_CORE2_MASK;

			tmp |= ((clkdiv_cpu0_4412[i][0] << EXYNOS4_CLKDIV_CPU0_CORE_SHIFT) |
				(clkdiv_cpu0_4412[i][1] << EXYNOS4_CLKDIV_CPU0_COREM0_SHIFT) |
				(clkdiv_cpu0_4412[i][2] << EXYNOS4_CLKDIV_CPU0_COREM1_SHIFT) |
				(clkdiv_cpu0_4412[i][3] << EXYNOS4_CLKDIV_CPU0_PERIPH_SHIFT) |
				(clkdiv_cpu0_4412[i][4] << EXYNOS4_CLKDIV_CPU0_ATB_SHIFT) |
				(clkdiv_cpu0_4412[i][5] << EXYNOS4_CLKDIV_CPU0_PCLKDBG_SHIFT) |
				(clkdiv_cpu0_4412[i][6] << EXYNOS4_CLKDIV_CPU0_APLL_SHIFT) |
				(clkdiv_cpu0_4412[i][7] << EXYNOS4_CLKDIV_CPU0_CORE2_SHIFT));
		}

		exynos4x12_clkdiv_table[i].clkdiv = tmp;

		tmp = __raw_readl(EXYNOS4_CLKDIV_CPU1);

		if (soc_is_exynos4212()) {
			tmp &= ~(EXYNOS4_CLKDIV_CPU1_COPY_MASK |
				EXYNOS4_CLKDIV_CPU1_HPM_MASK);
			tmp |= ((clkdiv_cpu1_4212[i][0] << EXYNOS4_CLKDIV_CPU1_COPY_SHIFT) |
				(clkdiv_cpu1_4212[i][1] << EXYNOS4_CLKDIV_CPU1_HPM_SHIFT));
		} else {
			tmp &= ~(EXYNOS4_CLKDIV_CPU1_COPY_MASK |
				EXYNOS4_CLKDIV_CPU1_HPM_MASK |
				EXYNOS4_CLKDIV_CPU1_CORES_MASK);
			tmp |= ((clkdiv_cpu1_4412[i][0] << EXYNOS4_CLKDIV_CPU1_COPY_SHIFT) |
				(clkdiv_cpu1_4412[i][1] << EXYNOS4_CLKDIV_CPU1_HPM_SHIFT) |
				(clkdiv_cpu1_4412[i][2] << EXYNOS4_CLKDIV_CPU1_CORES_SHIFT));
		}
		exynos4x12_clkdiv_table[i].clkdiv1 = tmp;
	}

	info->mpll_freq_khz = rate;
	info->pm_lock_idx = L5;
	info->pll_safe_idx = L7;
	info->max_support_idx = max_support_idx;
	info->min_support_idx = min_support_idx;
	info->cpu_clk = cpu_clk;
	info->volt_table = exynos4x12_volt_table;
	info->freq_table = exynos4x12_freq_table;
	info->set_freq = exynos4x12_set_frequency;
	info->need_apll_change = exynos4x12_pms_change;

	// BASETIS: código de debug
	apll_con0 = __raw_readl(EXYNOS4_APLL_CON0);
	printk(KERN_INFO "BASETIS: apll_con0 = %x\n", apll_con0);
	mpll_con0 = __raw_readl(EXYNOS4_MPLL_CON0);
	printk(KERN_INFO "BASETIS: mpll_con0 = %x\n", mpll_con0);

	// BASETIS: código para reconfigurar el MPLL.
	__raw_writel(0x0 << 31, EXYNOS4_MPLL_CON0);		// Paramos MPLL
	mpll_con0 &= 0xFFF4FFFF;
	__raw_writel(mpll_con0, EXYNOS4_MPLL_CON0);		// Corrección a 800MHz
	__raw_writel(0x1 << 31, EXYNOS4_MPLL_CON0);		// Arrancamos MPLL
	do
	{
		mpll_con0 = __raw_readl(EXYNOS4_MPLL_CON0);
	} while (!(mpll_con0 & 0x1 << 29));	// Esperamos al lock del MPLL


	printk(KERN_INFO "BASETIS: mpll_freq_khz = %lu\n", rate);
	printk(KERN_INFO "BASETIS: max_support_idx = %u\n", max_support_idx);
	printk(KERN_INFO "BASETIS: min_support_idx = %u\n", min_support_idx);
	//printk(KERN_INFO "BASETIS: cpu_clk : %u\n", cpu_clk);

	return 0;

err_mout_apll:
	clk_put(mout_mpll);
err_mout_mpll:
	clk_put(moutcore);
err_moutcore:
	clk_put(cpu_clk);

	pr_debug("%s: failed initialization\n", __func__);
	return -EINVAL;
}
EXPORT_SYMBOL(exynos4x12_cpufreq_init);
