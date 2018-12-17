// SPDX-License-Identifier: GPL-2.0+
/*
 * Mellanox watchdog driver
 *
 * Copyright (C) 2018 Mellanox Technologies
 * Copyright (C) 2018 Michael Shych <mshych@mellanox.com>
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/platform_data/mlxreg.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/watchdog.h>

#define MLXREG_WDT_CLOCK_SCALE		1000
#define MLXREG_WDT_MAX_TIMEOUT_TYPE1	32
#define MLXREG_WDT_MAX_TIMEOUT_TYPE2	255
#define MLXREG_WDT_MIN_TIMEOUT	1
#define MLXREG_WDT_HW_TIMEOUT_CONVERT(hw_timeout) ((1 << (hw_timeout)) \
						   / MLXREG_WDT_CLOCK_SCALE)

/**
 * enum mlxreg_wdt_type - type of HW watchdog
 *
 * TYPE1 can be differentiated by different register/mask
 *	 for WD action set and ping.
 */
enum mlxreg_wdt_type {
	MLX_WDT_TYPE1,
	MLX_WDT_TYPE2,
};

/**
 * struct mlxreg_wdt - wd private data:
 *
 * @wdd:	watchdog device;
 * @device:	basic device;
 * @pdata:	data received from platform driver;
 * @regmap:	register map of parent device;
 * @timeout:	defined timeout in sec.;
 * @hw_timeout:	real timeout set in hw;
 *		It will be roundup base of 2 in WD type 1,
 *		in WD type 2 it will be same number of sec as timeout;
 * @action_idx:	index for direct access to action register;
 * @timeout_idx:index for direct access to TO register;
 * @ping_idx:	index for direct access to ping register;
 * @reset_idx:	index for direct access to reset cause register;
 * @wd_type:	watchdog HW type;
 * @hw_timeout:	actual HW timeout;
 * @io_lock:	spinlock for io access;
 */
struct mlxreg_wdt {
	struct watchdog_device wdd;
	struct mlxreg_core_platform_data *pdata;
	void *regmap;
	int action_idx;
	int timeout_idx;
	int ping_idx;
	int reset_idx;
	enum mlxreg_wdt_type wdt_type;
	u8 hw_timeout;
	spinlock_t io_lock;	/* the lock for io operations */
};

static int mlxreg_wdt_roundup_to_base_2(struct mlxreg_wdt *wdt, int timeout)
{
	timeout *= MLXREG_WDT_CLOCK_SCALE;

	wdt->hw_timeout = order_base_2(timeout);
	dev_info(wdt->wdd.parent,
		 "watchdog %s timeout %d was rounded up to %lu (msec)\n",
		 wdt->wdd.info->identity, timeout, roundup_pow_of_two(timeout));

	return 0;
}

static enum mlxreg_wdt_type
mlxreg_wdt_check_watchdog_type(struct mlxreg_wdt *wdt,
			       struct mlxreg_core_platform_data *pdata)
{
	if ((pdata->data[wdt->action_idx].reg ==
	     pdata->data[wdt->ping_idx].reg) &&
	    (pdata->data[wdt->action_idx].mask ==
	     pdata->data[wdt->ping_idx].mask))
		return MLX_WDT_TYPE2;
	else
		return MLX_WDT_TYPE1;
}

static int mlxreg_wdt_check_card_reset(struct mlxreg_wdt *wdt)
{
	struct mlxreg_core_data *reg_data;
	u32 regval;
	int rc;

	if (wdt->reset_idx == -EINVAL)
		return -EINVAL;

	if (!(wdt->wdd.info->options & WDIOF_CARDRESET))
		return 0;

	spin_lock(&wdt->io_lock);
	reg_data = &wdt->pdata->data[wdt->reset_idx];
	rc = regmap_read(wdt->regmap, reg_data->reg, &regval);
	spin_unlock(&wdt->io_lock);
	if (rc)
		goto read_error;

	if (regval & ~reg_data->mask) {
		wdt->wdd.bootstatus = WDIOF_CARDRESET;
		dev_info(wdt->wdd.parent,
			 "watchdog previously reset the CPU\n");
	}

read_error:
	return rc;
}

static int mlxreg_wdt_start(struct watchdog_device *wdd)
{
	struct mlxreg_wdt *wdt = watchdog_get_drvdata(wdd);
	struct mlxreg_core_data *reg_data = &wdt->pdata->data[wdt->action_idx];
	u32 regval;
	int rc;

	spin_lock(&wdt->io_lock);
	rc = regmap_read(wdt->regmap, reg_data->reg, &regval);
	if (rc) {
		spin_unlock(&wdt->io_lock);
		goto read_error;
	}

	regval = (regval & reg_data->mask) | BIT(reg_data->bit);
	rc = regmap_write(wdt->regmap, reg_data->reg, regval);
	spin_unlock(&wdt->io_lock);
	if (!rc) {
		set_bit(WDOG_HW_RUNNING, &wdt->wdd.status);
		dev_info(wdt->wdd.parent, "watchdog %s started\n",
			 wdd->info->identity);
	}

read_error:
	return rc;
}

static int mlxreg_wdt_stop(struct watchdog_device *wdd)
{
	struct mlxreg_wdt *wdt = watchdog_get_drvdata(wdd);
	struct mlxreg_core_data *reg_data = &wdt->pdata->data[wdt->action_idx];
	u32 regval;
	int rc;

	spin_lock(&wdt->io_lock);
	rc = regmap_read(wdt->regmap, reg_data->reg, &regval);
	if (rc) {
		spin_unlock(&wdt->io_lock);
		goto read_error;
	}

	regval = (regval & reg_data->mask) & ~BIT(reg_data->bit);
	rc = regmap_write(wdt->regmap, reg_data->reg, regval);
	spin_unlock(&wdt->io_lock);
	if (!rc)
		dev_info(wdt->wdd.parent, "watchdog %s stopped\n",
			 wdd->info->identity);

read_error:
	return rc;
}

static int mlxreg_wdt_ping(struct watchdog_device *wdd)
{
	struct mlxreg_wdt *wdt = watchdog_get_drvdata(wdd);
	struct mlxreg_core_data *reg_data = &wdt->pdata->data[wdt->ping_idx];
	u32 regval;
	int rc;

	spin_lock(&wdt->io_lock);
	rc = regmap_read(wdt->regmap, reg_data->reg, &regval);
	if (rc)
		goto read_error;

	regval = (regval & reg_data->mask) | BIT(reg_data->bit);
	rc = regmap_write(wdt->regmap, reg_data->reg, regval);

read_error:
	spin_unlock(&wdt->io_lock);

	return rc;
}

static int mlxreg_wdt_set_timeout(struct watchdog_device *wdd,
				  unsigned int timeout)
{
	struct mlxreg_wdt *wdt = watchdog_get_drvdata(wdd);
	struct mlxreg_core_data *reg_data = &wdt->pdata->data[wdt->timeout_idx];
	u32 regval;
	int rc;

	spin_lock(&wdt->io_lock);

	if (wdt->wdt_type == MLX_WDT_TYPE1) {
		rc = regmap_read(wdt->regmap, reg_data->reg, &regval);
		if (rc)
			goto read_error;
		regval = (regval & reg_data->mask) | wdt->hw_timeout;
	} else {
		wdt->hw_timeout = timeout;
		regval = timeout;
	}

	rc = regmap_write(wdt->regmap, reg_data->reg, regval);

read_error:
	spin_unlock(&wdt->io_lock);

	return rc;
}

static unsigned int mlxreg_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct mlxreg_wdt *wdt = watchdog_get_drvdata(wdd);
	struct mlxreg_core_data *reg_data = &wdt->pdata->data[wdt->timeout_idx];
	u32 regval;
	int rc;

	if (wdt->wdt_type == MLX_WDT_TYPE1)
		return 0;

	spin_lock(&wdt->io_lock);
	rc = regmap_read(wdt->regmap, reg_data->reg, &regval);
	if (rc)
		rc = 0;
	else
		rc = regval;

	spin_unlock(&wdt->io_lock);

	return rc;
}

static const struct watchdog_ops mlxreg_wdt_ops_type1 = {
	.start		= mlxreg_wdt_start,
	.stop		= mlxreg_wdt_stop,
	.ping		= mlxreg_wdt_ping,
	.set_timeout	= mlxreg_wdt_set_timeout,
	.owner		= THIS_MODULE,
};

static const struct watchdog_ops mlxreg_wdt_ops_type2 = {
	.start		= mlxreg_wdt_start,
	.stop		= mlxreg_wdt_stop,
	.ping		= mlxreg_wdt_ping,
	.set_timeout	= mlxreg_wdt_set_timeout,
	.get_timeleft	= mlxreg_wdt_get_timeleft,
	.owner		= THIS_MODULE,
};

static const struct watchdog_info mlxreg_wdt_main_info = {
	.options	= WDIOF_KEEPALIVEPING
			| WDIOF_MAGICCLOSE
			| WDIOF_SETTIMEOUT
			| WDIOF_CARDRESET,
	.identity	= "mlx-wdt-main",
};

static const struct watchdog_info mlxreg_wdt_aux_info = {
	.options	= WDIOF_KEEPALIVEPING
			| WDIOF_MAGICCLOSE
			| WDIOF_SETTIMEOUT
			| WDIOF_ALARMONLY,
	.identity	= "mlx-wdt-aux",
};

static int mlxreg_wdt_config(struct mlxreg_wdt *wdt,
			     struct mlxreg_core_platform_data *pdata)
{
	struct mlxreg_core_data *data = pdata->data;
	int i, timeout;

	wdt->reset_idx = -EINVAL;
	for (i = 0; i < pdata->counter; i++, data++) {
		if (strnstr(data->label, "action", sizeof(data->label)))
			wdt->action_idx = i;
		else if (strnstr(data->label, "timeout", sizeof(data->label)))
			wdt->timeout_idx = i;
		else if (strnstr(data->label, "ping", sizeof(data->label)))
			wdt->ping_idx = i;
		else if (strnstr(data->label, "reset", sizeof(data->label)))
			wdt->reset_idx = i;
	}

	wdt->pdata = pdata;
	if (strnstr(pdata->identity, mlxreg_wdt_main_info.identity,
		    sizeof(mlxreg_wdt_main_info.identity)))
		wdt->wdd.info = &mlxreg_wdt_main_info;
	else
		wdt->wdd.info = &mlxreg_wdt_aux_info;

	timeout = pdata->data[wdt->timeout_idx].health_cntr;
	wdt->wdt_type = mlxreg_wdt_check_watchdog_type(wdt, pdata);
	if (wdt->wdt_type == MLX_WDT_TYPE2) {
		wdt->hw_timeout = timeout;
		wdt->wdd.ops = &mlxreg_wdt_ops_type2;
		wdt->wdd.timeout = wdt->hw_timeout;
		wdt->wdd.max_timeout = MLXREG_WDT_MAX_TIMEOUT_TYPE2;
	} else {
		mlxreg_wdt_roundup_to_base_2(wdt, timeout);
		wdt->wdd.ops = &mlxreg_wdt_ops_type1;
		/* Rowndown to actual closest number of sec. */
		wdt->wdd.timeout =
			MLXREG_WDT_HW_TIMEOUT_CONVERT(wdt->hw_timeout);
		wdt->wdd.max_timeout = MLXREG_WDT_MAX_TIMEOUT_TYPE1;
	}
	wdt->wdd.min_timeout = MLXREG_WDT_MIN_TIMEOUT;

	return -EINVAL;
}

static int mlxreg_wdt_probe(struct platform_device *pdev)
{
	struct mlxreg_core_platform_data *pdata;
	struct mlxreg_wdt *wdt;
	int rc;

	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		dev_err(&pdev->dev, "Failed to get platform data.\n");
		return -EINVAL;
	}
	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	spin_lock_init(&wdt->io_lock);

	wdt->wdd.parent = &pdev->dev;
	wdt->regmap = pdata->regmap;
	mlxreg_wdt_config(wdt, pdata);

	if ((pdata->features & MLXREG_CORE_WD_FEATURE_NOSTOP_AFTER_START))
		watchdog_set_nowayout(&wdt->wdd, WATCHDOG_NOWAYOUT);
	watchdog_stop_on_reboot(&wdt->wdd);
	watchdog_init_timeout(&wdt->wdd, 0, &pdev->dev);
	watchdog_set_drvdata(&wdt->wdd, wdt);

	mlxreg_wdt_check_card_reset(wdt);
	rc = devm_watchdog_register_device(&pdev->dev, &wdt->wdd);
	if (rc) {
		dev_err(&pdev->dev,
			"Cannot register watchdog device (err=%d)\n", rc);
		return rc;
	}

	mlxreg_wdt_set_timeout(&wdt->wdd, wdt->wdd.timeout);
	if ((pdata->features & MLXREG_CORE_WD_FEATURE_START_AT_BOOT))
		mlxreg_wdt_start(&wdt->wdd);

	return rc;
}

static int mlxreg_wdt_remove(struct platform_device *pdev)
{
	struct mlxreg_wdt *wdt = dev_get_platdata(&pdev->dev);

	mlxreg_wdt_stop(&wdt->wdd);
	watchdog_unregister_device(&wdt->wdd);

	return 0;
}

static struct platform_driver mlxreg_wdt_driver = {
	.probe	= mlxreg_wdt_probe,
	.remove	= mlxreg_wdt_remove,
	.driver	= {
		.name = "mlx-wdt",
	},
};

module_platform_driver(mlxreg_wdt_driver);

MODULE_AUTHOR("Michael Shych <michaelsh@mellanox.com>");
MODULE_DESCRIPTION("Mellanox watchdog driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mlx-wdt");
