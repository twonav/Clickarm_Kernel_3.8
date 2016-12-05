/*
 * Exynos Generic power domain support.
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Implementation of Exynos specific power domain control which is used in
 * conjunction with runtime-pm. Support for both device-tree and non-device-tree
 * based power domain support is included.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/


/*
 * BASETIS:
 *
 * 	1) Añadido comentario sobre nombre de registro incorrecto.
 * 	2) Añadido código para desactivar 'power domains' de CPU 1, 2 y 3, y Cachés.
 *
 * 	Comentarios:
 * 		1) No se observan cambios en el consumo al añadir los 'power domains' descritos.
 * 		2) Parece código innecesario ya que el driver del DVFS ya desactiva CPU 2 y 3.
 * 		3) Ya que no afecta, se deja esta parte de código comentada.
 *
 */


#include <linux/io.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/pm_domain.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/sched.h>

#include <mach/regs-pmu.h>
#include <plat/devs.h>

extern struct platform_device mali_gpu_device;
/*
 * Exynos specific wrapper around the generic power domain
 */
struct exynos_pm_domain {
	void __iomem *base;
	char const *name;
	bool is_off;
	struct generic_pm_domain pd;
};

static int exynos_pd_power(struct generic_pm_domain *domain, bool power_on)
{
	struct exynos_pm_domain *pd;
	void __iomem *base;
	u32 timeout, pwr;
	char *op;

	pd = container_of(domain, struct exynos_pm_domain, pd);
	base = pd->base;

	/* FIXME: the HDMI device relies on LCD0 and TV power domains, but is
	 * only registered under TV (one device can only be in 1 domain).
	 * Hack here to avoid powering off LCD0 to avoid a loss of TV output.
	 */
	if (base == S5P_PMU_LCD0_CONF && !power_on)
		return 0;

	/* HACK: don't reset the vp */
	if (base == S5P_PMU_TV_CONF && !power_on)
		return 0;

	pwr = power_on ? S5P_INT_LOCAL_PWR_EN : 0;
	__raw_writel(pwr, base);

	/* Wait max 1ms */
	timeout = 10;

	while ((__raw_readl(base + 0x4) & S5P_INT_LOCAL_PWR_EN)	!= pwr) {
		if (!timeout) {
			op = (power_on) ? "enable" : "disable";
			pr_err("Power domain %s %s failed\n", domain->name, op);
			return -ETIMEDOUT;
		}
		timeout--;
		cpu_relax();
		usleep_range(80, 100);
	}
	return 0;
}

static int exynos_pd_power_on(struct generic_pm_domain *domain)
{
	return exynos_pd_power(domain, true);
}

static int exynos_pd_power_off(struct generic_pm_domain *domain)
{
	return exynos_pd_power(domain, false);
}

#define EXYNOS_GPD(PD, BASE, NAME)			\
static struct exynos_pm_domain PD = {			\
	.base = (void __iomem *)BASE,			\
	.name = NAME,					\
	.pd = {						\
		.name = NAME, \
		.power_off = exynos_pd_power_off,	\
		.power_on = exynos_pd_power_on,	\
	},						\
}

#ifdef CONFIG_OF
static void exynos_add_device_to_domain(struct exynos_pm_domain *pd,
					 struct device *dev)
{
	int ret;

	dev_dbg(dev, "adding to power domain %s\n", pd->pd.name);

	while (1) {
		ret = pm_genpd_add_device(&pd->pd, dev);
		if (ret != -EAGAIN)
			break;
		cond_resched();
	}

	pm_genpd_dev_need_restore(dev, true);
}

static void exynos_remove_device_from_domain(struct device *dev)
{
	struct generic_pm_domain *genpd = dev_to_genpd(dev);
	int ret;

	dev_dbg(dev, "removing from power domain %s\n", genpd->name);

	while (1) {
		ret = pm_genpd_remove_device(genpd, dev);
		if (ret != -EAGAIN)
			break;
		cond_resched();
	}
}

static void exynos_read_domain_from_dt(struct device *dev)
{
	struct platform_device *pd_pdev;
	struct exynos_pm_domain *pd;
	struct device_node *node;

	node = of_parse_phandle(dev->of_node, "samsung,power-domain", 0);
	if (!node)
		return;
	pd_pdev = of_find_device_by_node(node);
	if (!pd_pdev)
		return;
	pd = platform_get_drvdata(pd_pdev);
	exynos_add_device_to_domain(pd, dev);
}

static int exynos_pm_notifier_call(struct notifier_block *nb,
				    unsigned long event, void *data)
{
	struct device *dev = data;

	switch (event) {
	case BUS_NOTIFY_BIND_DRIVER:
		if (dev->of_node)
			exynos_read_domain_from_dt(dev);

		break;

	case BUS_NOTIFY_UNBOUND_DRIVER:
		exynos_remove_device_from_domain(dev);

		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block platform_nb = {
	.notifier_call = exynos_pm_notifier_call,
};

static __init int exynos_pm_dt_parse_domains(void)
{
	struct platform_device *pdev;
	struct device_node *np;

	for_each_compatible_node(np, NULL, "samsung,exynos4210-pd") {
		struct exynos_pm_domain *pd;
		int on;

		pdev = of_find_device_by_node(np);

		pd = kzalloc(sizeof(*pd), GFP_KERNEL);
		if (!pd) {
			pr_err("%s: failed to allocate memory for domain\n",
					__func__);
			return -ENOMEM;
		}

		pd->pd.name = kstrdup(np->name, GFP_KERNEL);
		pd->name = pd->pd.name;
		pd->base = of_iomap(np, 0);
		pd->pd.power_off = exynos_pd_power_off;
		pd->pd.power_on = exynos_pd_power_on;
		pd->pd.of_node = np;

		platform_set_drvdata(pdev, pd);

		on = __raw_readl(pd->base + 0x4) & S5P_INT_LOCAL_PWR_EN;

		pm_genpd_init(&pd->pd, NULL, !on);
	}

	bus_register_notifier(&platform_bus_type, &platform_nb);

	return 0;
}
#else
static __init int exynos_pm_dt_parse_domains(void)
{
	return 0;
}
#endif /* CONFIG_OF */

static __init __maybe_unused void exynos_pm_add_dev_to_genpd(struct platform_device *pdev,
						struct exynos_pm_domain *pd)
{
	if (pdev->dev.bus) {
		if (!pm_genpd_add_device(&pd->pd, &pdev->dev))
			pm_genpd_dev_need_restore(&pdev->dev, true);
		else
			pr_info("%s: error in adding %s device to %s power"
				"domain\n", __func__, dev_name(&pdev->dev),
				pd->name);
	}
}

EXYNOS_GPD(exynos4_pd_mfc, S5P_PMU_MFC_CONF, "pd-mfc");
EXYNOS_GPD(exynos4_pd_g3d, S5P_PMU_G3D_CONF, "pd-g3d");
EXYNOS_GPD(exynos4_pd_lcd0, S5P_PMU_LCD0_CONF, "pd-lcd0");
// BASETIS: el nombre "lcd1" es incorrecto. Realmente apunta al registro ISP_CONGIGURATION.
EXYNOS_GPD(exynos4_pd_lcd1, S5P_PMU_LCD1_CONF, "pd-lcd1");
EXYNOS_GPD(exynos4_pd_tv, S5P_PMU_TV_CONF, "pd-tv");
EXYNOS_GPD(exynos4_pd_cam, S5P_PMU_CAM_CONF, "pd-cam");
EXYNOS_GPD(exynos4_pd_gps, S5P_PMU_GPS_CONF, "pd-gps");
// BASETIS: añadimos Power Domains que faltarían (ver sección 8-6 del datasheet)
//EXYNOS_GPD(exynos4_pd_cpu1, S5P_ARM_CORE1_CONFIGURATION, "pd-cpu1");
//EXYNOS_GPD(exynos4_pd_cpu2, S5P_ARM_CORE2_CONFIGURATION, "pd-cpu2");
//EXYNOS_GPD(exynos4_pd_cpu3, S5P_ARM_CORE3_CONFIGURATION, "pd-cpu3");
//EXYNOS_GPD(exynos4_pd_l2_1, S5P_ARM_L2_1_CONFIGURATION, "pd-l2_1");

static struct exynos_pm_domain *exynos4_pm_domains[] = {
	&exynos4_pd_mfc,
	&exynos4_pd_g3d,
	&exynos4_pd_lcd0,
	&exynos4_pd_lcd1,
	&exynos4_pd_tv,
	&exynos4_pd_cam,
	&exynos4_pd_gps,
	// BASETIS: añadimos Power Domains que faltarían
	//&exynos4_pd_cpu1,
	//&exynos4_pd_cpu3,
	//&exynos4_pd_cpu2,
	//&exynos4_pd_l2_1,
};

static __init int exynos4_pm_init_power_domain(void)
{
	int idx;

	if (of_have_populated_dt())
		return exynos_pm_dt_parse_domains();

	for (idx = 0; idx < ARRAY_SIZE(exynos4_pm_domains); idx++) {
		struct exynos_pm_domain *pd = exynos4_pm_domains[idx];
		int on = __raw_readl(pd->base + 0x4) & S5P_INT_LOCAL_PWR_EN;

		pm_genpd_init(&pd->pd, NULL, !on);
	}

#ifdef CONFIG_S5P_DEV_FIMD0
	exynos_pm_add_dev_to_genpd(&s5p_device_fimd0, &exynos4_pd_lcd0);
#endif
#ifdef CONFIG_S5P_DEV_TV
	exynos_pm_add_dev_to_genpd(&s5p_device_hdmi, &exynos4_pd_tv);
	exynos_pm_add_dev_to_genpd(&s5p_device_mixer, &exynos4_pd_tv);
#endif
#ifdef CONFIG_S5P_DEV_MFC
	exynos_pm_add_dev_to_genpd(&s5p_device_mfc, &exynos4_pd_mfc);
#endif
#ifdef CONFIG_S5P_DEV_FIMC0
	exynos_pm_add_dev_to_genpd(&s5p_device_fimc0, &exynos4_pd_cam);
#endif
#ifdef CONFIG_S5P_DEV_FIMC1
	exynos_pm_add_dev_to_genpd(&s5p_device_fimc1, &exynos4_pd_cam);
#endif
#ifdef CONFIG_S5P_DEV_FIMC2
	exynos_pm_add_dev_to_genpd(&s5p_device_fimc2, &exynos4_pd_cam);
#endif
#ifdef CONFIG_S5P_DEV_FIMC3
	exynos_pm_add_dev_to_genpd(&s5p_device_fimc3, &exynos4_pd_cam);
#endif
#ifdef CONFIG_S5P_DEV_CSIS0
	exynos_pm_add_dev_to_genpd(&s5p_device_mipi_csis0, &exynos4_pd_cam);
#endif
#ifdef CONFIG_S5P_DEV_CSIS1
	exynos_pm_add_dev_to_genpd(&s5p_device_mipi_csis1, &exynos4_pd_cam);
#endif
#ifdef CONFIG_S5P_DEV_G2D
	exynos_pm_add_dev_to_genpd(&s5p_device_g2d, &exynos4_pd_lcd0);
#endif
#ifdef CONFIG_S5P_DEV_JPEG
	exynos_pm_add_dev_to_genpd(&s5p_device_jpeg, &exynos4_pd_cam);
#endif
#ifdef CONFIG_MALI400
	exynos_pm_add_dev_to_genpd(&mali_gpu_device, &exynos4_pd_g3d);
#endif	        
	return 0;
}
arch_initcall(exynos4_pm_init_power_domain);

int __init exynos_pm_late_initcall(void)
{
	pm_genpd_poweroff_unused();
	return 0;
}
