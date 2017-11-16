/*
 * Copyright (c) 2016 Mellanox Technologies. All rights reserved.
 * Copyright (c) 2016 Vadim Pasternak <vadimp@mellanox.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the names of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/device.h>
#include <linux/dmi.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/platform_data/i2c-mux-reg.h>
#include <linux/platform_data/mlxreg.h>
#include <linux/regmap.h>

#define MLX_PLAT_DEVICE_NAME		"mlxplat"

/* LPC bus IO offsets */
#define MLXPLAT_CPLD_LPC_I2C_BASE_ADRR		0x2000
#define MLXPLAT_CPLD_LPC_REG_BASE_ADRR		0x2500
#define MLXPLAT_CPLD_LPC_REG_CPLD1_VER_OFF	0x00
#define MLXPLAT_CPLD_LPC_REG_CPLD2_VER_OFF	0x01
#define MLXPLAT_CPLD_LPC_REG_RESET_CAUSE_OFF	0x1d
#define MLXPLAT_CPLD_LPC_REG_LED1_OFF		0x20
#define MLXPLAT_CPLD_LPC_REG_LED2_OFF		0x21
#define MLXPLAT_CPLD_LPC_REG_LED3_OFF		0x22
#define MLXPLAT_CPLD_LPC_REG_LED4_OFF		0x23
#define MLXPLAT_CPLD_LPC_REG_LED5_OFF		0x24
#define MLXPLAT_CPLD_LPC_REG_GP1_OFF		0x30
#define MLXPLAT_CPLD_LPC_REG_WP1_OFF		0x31
#define MLXPLAT_CPLD_LPC_REG_GP2_OFF		0x32
#define MLXPLAT_CPLD_LPC_REG_WP2_OFF		0x33
#define MLXPLAT_CPLD_LPC_REG_AGGR_OFF		0x3a
#define MLXPLAT_CPLD_LPC_REG_AGGR_MASK_OFF	0x3b
#define MLXPLAT_CPLD_LPC_REG_AGGR_LOW_OFF	0x40
#define MLXPLAT_CPLD_LPC_REG_AGGR_LOW_MASK_OFF	0x41
#define MLXPLAT_CPLD_LPC_REG_PSU_OFF		0x58
#define MLXPLAT_CPLD_LPC_REG_PSU_EVENT_OFF	0x59
#define MLXPLAT_CPLD_LPC_REG_PSU_MASK_OFF	0x5a
#define MLXPLAT_CPLD_LPC_REG_PWR_OFF		0x64
#define MLXPLAT_CPLD_LPC_REG_PWR_EVENT_OFF	0x65
#define MLXPLAT_CPLD_LPC_REG_PWR_MASK_OFF	0x66
#define MLXPLAT_CPLD_LPC_REG_FAN_OFF		0x88
#define MLXPLAT_CPLD_LPC_REG_FAN_EVENT_OFF	0x89
#define MLXPLAT_CPLD_LPC_REG_FAN_MASK_OFF	0x8a
#define MLXPLAT_CPLD_LPC_IO_RANGE		0x100
#define MLXPLAT_CPLD_LPC_I2C_CH1_OFF		0xdb
#define MLXPLAT_CPLD_LPC_I2C_CH2_OFF		0xda
#define MLXPLAT_CPLD_LPC_PIO_OFFSET		0x10000UL
#define MLXPLAT_CPLD_LPC_REG1	((MLXPLAT_CPLD_LPC_REG_BASE_ADRR + \
				  MLXPLAT_CPLD_LPC_I2C_CH1_OFF) | \
				  MLXPLAT_CPLD_LPC_PIO_OFFSET)
#define MLXPLAT_CPLD_LPC_REG2	((MLXPLAT_CPLD_LPC_REG_BASE_ADRR + \
				  MLXPLAT_CPLD_LPC_I2C_CH2_OFF) | \
				  MLXPLAT_CPLD_LPC_PIO_OFFSET)

/* Masks for aggregation, psu, pwr and fan event in CPLD related registers. */
#define MLXPLAT_CPLD_AGGR_PSU_MASK_DEF	0x08
#define MLXPLAT_CPLD_AGGR_PWR_MASK_DEF	0x08
#define MLXPLAT_CPLD_AGGR_FAN_MASK_DEF	0x40
#define MLXPLAT_CPLD_AGGR_MASK_DEF	(MLXPLAT_CPLD_AGGR_PSU_MASK_DEF | \
					 MLXPLAT_CPLD_AGGR_FAN_MASK_DEF)
#define MLXPLAT_CPLD_AGGR_MASK_NG_DEF	0x04
#define MLXPLAT_CPLD_LOW_AGGR_MASK_LOW	0xc0
#define MLXPLAT_CPLD_AGGR_MASK_MSN21XX	0x04
#define MLXPLAT_CPLD_PSU_MASK		GENMASK(1, 0)
#define MLXPLAT_CPLD_PWR_MASK		GENMASK(1, 0)
#define MLXPLAT_CPLD_FAN_MASK		GENMASK(3, 0)
#define MLXPLAT_CPLD_FAN_NG_MASK	GENMASK(5, 0)
#define MLXPLAT_CPLD_LED_LO_NIBBLE_MASK	GENMASK(7, 4)
#define MLXPLAT_CPLD_LED_HI_NIBBLE_MASK	GENMASK(3, 0)

/* Start channel numbers */
#define MLXPLAT_CPLD_CH1			2
#define MLXPLAT_CPLD_CH2			10

/* Number of LPC attached MUX platform devices */
#define MLXPLAT_CPLD_LPC_MUX_DEVS		2

/* PSU adapter numbers */
#define MLXPLAT_CPLD_PSU_DEFAULT_NR		10
#define MLXPLAT_CPLD_PSU_MSNXXXX_NR		4

/* mlxplat_priv - platform private data
 * @pdev_i2c - i2c controller platform device
 * @pdev_mux - array of mux platform devices
 * @pdev_hotplug - hotplug platform devices
 * @pdev_led - led platform devices
 * @pdev_io_regs - register access platform devices
 */
struct mlxplat_priv {
	struct platform_device *pdev_i2c;
	struct platform_device *pdev_mux[MLXPLAT_CPLD_LPC_MUX_DEVS];
	struct platform_device *pdev_hotplug;
	struct platform_device *pdev_led;
	struct platform_device *pdev_io_regs;
};

/* Regions for LPC I2C controller and LPC base register space */
static const struct resource mlxplat_lpc_resources[] = {
	[0] = DEFINE_RES_NAMED(MLXPLAT_CPLD_LPC_I2C_BASE_ADRR,
			       MLXPLAT_CPLD_LPC_IO_RANGE,
			       "mlxplat_cpld_lpc_i2c_ctrl", IORESOURCE_IO),
	[1] = DEFINE_RES_NAMED(MLXPLAT_CPLD_LPC_REG_BASE_ADRR,
			       MLXPLAT_CPLD_LPC_IO_RANGE,
			       "mlxplat_cpld_lpc_regs",
			       IORESOURCE_IO),
};

/* Platform default channels */
static const int mlxplat_default_channels[][8] = {
	{
		MLXPLAT_CPLD_CH1, MLXPLAT_CPLD_CH1 + 1, MLXPLAT_CPLD_CH1 + 2,
		MLXPLAT_CPLD_CH1 + 3, MLXPLAT_CPLD_CH1 + 4, MLXPLAT_CPLD_CH1 +
		5, MLXPLAT_CPLD_CH1 + 6, MLXPLAT_CPLD_CH1 + 7
	},
	{
		MLXPLAT_CPLD_CH2, MLXPLAT_CPLD_CH2 + 1, MLXPLAT_CPLD_CH2 + 2,
		MLXPLAT_CPLD_CH2 + 3, MLXPLAT_CPLD_CH2 + 4, MLXPLAT_CPLD_CH2 +
		5, MLXPLAT_CPLD_CH2 + 6, MLXPLAT_CPLD_CH2 + 7
	},
};

/* Platform channels for MSN21xx system family */
static const int mlxplat_msn21xx_channels[] = { 1, 2, 3, 4, 5, 6, 7, 8 };

/* Platform mux data */
static struct i2c_mux_reg_platform_data mlxplat_mux_data[] = {
	{
		.parent = 1,
		.base_nr = MLXPLAT_CPLD_CH1,
		.write_only = 1,
		.reg = (void __iomem *)MLXPLAT_CPLD_LPC_REG1,
		.reg_size = 1,
		.idle_in_use = 1,
	},
	{
		.parent = 1,
		.base_nr = MLXPLAT_CPLD_CH2,
		.write_only = 1,
		.reg = (void __iomem *)MLXPLAT_CPLD_LPC_REG2,
		.reg_size = 1,
		.idle_in_use = 1,
	},

};

/* Platform hotplug devices */
static struct i2c_board_info mlxplat_mlxcpld_psu[] = {
	{
		I2C_BOARD_INFO("24c02", 0x51),
	},
	{
		I2C_BOARD_INFO("24c02", 0x50),
	},
};

static struct i2c_board_info mlxplat_mlxcpld_pwr[] = {
	{
		I2C_BOARD_INFO("dps460", 0x59),
	},
	{
		I2C_BOARD_INFO("dps460", 0x58),
	},
};

static struct i2c_board_info mlxplat_mlxcpld_fan[] = {
	{
		I2C_BOARD_INFO("24c32", 0x50),
	},
	{
		I2C_BOARD_INFO("24c32", 0x50),
	},
	{
		I2C_BOARD_INFO("24c32", 0x50),
	},
	{
		I2C_BOARD_INFO("24c32", 0x50),
	},
};

/* Platform hotplug default data */
static struct mlxreg_core_data mlxplat_mlxcpld_default_psu_items_data[] = {
	{
		.label = "psu1",
		.reg = MLXPLAT_CPLD_LPC_REG_PSU_OFF,
		.mask = BIT(0),
		.hpdev.brdinfo = &mlxplat_mlxcpld_psu[0],
		.hpdev.nr = MLXPLAT_CPLD_PSU_DEFAULT_NR,
	},
	{
		.label = "psu2",
		.reg = MLXPLAT_CPLD_LPC_REG_PSU_OFF,
		.mask = BIT(1),
		.hpdev.brdinfo = &mlxplat_mlxcpld_psu[1],
		.hpdev.nr = MLXPLAT_CPLD_PSU_DEFAULT_NR,
	},
};

static struct mlxreg_core_data mlxplat_mlxcpld_default_pwr_items_data[] = {
	{
		.label = "pwr1",
		.reg = MLXPLAT_CPLD_LPC_REG_PWR_OFF,
		.mask = BIT(0),
		.hpdev.brdinfo = &mlxplat_mlxcpld_pwr[0],
		.hpdev.nr = MLXPLAT_CPLD_PSU_DEFAULT_NR,
	},
	{
		.label = "pwr2",
		.reg = MLXPLAT_CPLD_LPC_REG_PWR_OFF,
		.mask = BIT(1),
		.hpdev.brdinfo = &mlxplat_mlxcpld_pwr[1],
		.hpdev.nr = MLXPLAT_CPLD_PSU_DEFAULT_NR,
	},
};

static struct mlxreg_core_data mlxplat_mlxcpld_default_fan_items_data[] = {
	{
		.label = "fan1",
		.reg = MLXPLAT_CPLD_LPC_REG_FAN_OFF,
		.mask = BIT(0),
		.hpdev.brdinfo = &mlxplat_mlxcpld_fan[0],
		.hpdev.nr = 11,
	},
	{
		.label = "fan2",
		.reg = MLXPLAT_CPLD_LPC_REG_FAN_OFF,
		.mask = BIT(1),
		.hpdev.brdinfo = &mlxplat_mlxcpld_fan[1],
		.hpdev.nr = 12,
	},
	{
		.label = "fan3",
		.reg = MLXPLAT_CPLD_LPC_REG_FAN_OFF,
		.mask = BIT(2),
		.hpdev.brdinfo = &mlxplat_mlxcpld_fan[2],
		.hpdev.nr = 13,
	},
	{
		.label = "fan4",
		.reg = MLXPLAT_CPLD_LPC_REG_FAN_OFF,
		.mask = BIT(3),
		.hpdev.brdinfo = &mlxplat_mlxcpld_fan[3],
		.hpdev.nr = 14,
	},
};

static struct mlxreg_core_item mlxplat_mlxcpld_default_items[] = {
	{
		.data = mlxplat_mlxcpld_default_psu_items_data,
		.aggr_mask = MLXPLAT_CPLD_AGGR_PSU_MASK_DEF,
		.reg = MLXPLAT_CPLD_LPC_REG_PSU_OFF,
		.mask = MLXPLAT_CPLD_PSU_MASK,
		.count = ARRAY_SIZE(mlxplat_mlxcpld_psu),
		.inversed = 1,
		.health = false,
	},
	{
		.data = mlxplat_mlxcpld_default_pwr_items_data,
		.aggr_mask = MLXPLAT_CPLD_AGGR_PWR_MASK_DEF,
		.reg = MLXPLAT_CPLD_LPC_REG_PWR_OFF,
		.mask = MLXPLAT_CPLD_PWR_MASK,
		.count = ARRAY_SIZE(mlxplat_mlxcpld_pwr),
		.inversed = 0,
		.health = false,
	},
	{
		.data = mlxplat_mlxcpld_default_fan_items_data,
		.aggr_mask = MLXPLAT_CPLD_AGGR_FAN_MASK_DEF,
		.reg = MLXPLAT_CPLD_LPC_REG_FAN_OFF,
		.mask = MLXPLAT_CPLD_FAN_MASK,
		.count = ARRAY_SIZE(mlxplat_mlxcpld_fan),
		.inversed = 1,
		.health = false,
	},
};

static
struct mlxreg_core_hotplug_platform_data mlxplat_mlxcpld_default_data = {
	.items = mlxplat_mlxcpld_default_items,
	.counter = ARRAY_SIZE(mlxplat_mlxcpld_default_items),
	.cell = MLXPLAT_CPLD_LPC_REG_AGGR_OFF,
	.mask = MLXPLAT_CPLD_AGGR_MASK_DEF,
};

/* Platform hotplug MSN21xx system family data */
static struct i2c_board_info mlxplat_mlxcpld_msn21xx_pwr[] = {
	{
		I2C_BOARD_INFO("holder", 0x59),
	},
	{
		I2C_BOARD_INFO("holder", 0x58),
	},
};

static struct mlxreg_core_data mlxplat_mlxcpld_msn21xx_pwr_items_data[] = {
	{
		.label = "pwr1",
		.reg = MLXPLAT_CPLD_LPC_REG_PWR_OFF,
		.mask = BIT(0),
		.hpdev.brdinfo = &mlxplat_mlxcpld_msn21xx_pwr[0],
		.hpdev.nr = MLXPLAT_CPLD_PSU_DEFAULT_NR,
	},
	{
		.label = "pwr2",
		.reg = MLXPLAT_CPLD_LPC_REG_PWR_OFF,
		.mask = BIT(1),
		.hpdev.brdinfo = &mlxplat_mlxcpld_msn21xx_pwr[1],
		.hpdev.nr = MLXPLAT_CPLD_PSU_DEFAULT_NR,
	},
};

static struct mlxreg_core_item mlxplat_mlxcpld_msn21xx_items[] = {
	{
		.data = mlxplat_mlxcpld_msn21xx_pwr_items_data,
		.aggr_mask = MLXPLAT_CPLD_AGGR_PWR_MASK_DEF,
		.reg = MLXPLAT_CPLD_LPC_REG_PWR_OFF,
		.mask = MLXPLAT_CPLD_PWR_MASK,
		.count = ARRAY_SIZE(mlxplat_mlxcpld_msn21xx_pwr_items_data),
		.inversed = 0,
		.health = false,
	},
};

static
struct mlxreg_core_hotplug_platform_data mlxplat_mlxcpld_msn21xx_data = {
	.items = mlxplat_mlxcpld_msn21xx_items,
	.counter = ARRAY_SIZE(mlxplat_mlxcpld_msn21xx_items),
	.cell = MLXPLAT_CPLD_LPC_REG_AGGR_OFF,
	.mask = MLXPLAT_CPLD_AGGR_MASK_DEF,
	.cell_low = MLXPLAT_CPLD_LPC_REG_AGGR_LOW_OFF,
	.mask_low = MLXPLAT_CPLD_LOW_AGGR_MASK_LOW,
};

/* Platform hotplug MSN201x system family data */
static struct mlxreg_core_data mlxplat_mlxcpld_msn201x_pwr_items_data[] = {
	{
		.label = "pwr1",
		.reg = MLXPLAT_CPLD_LPC_REG_PWR_OFF,
		.mask = BIT(0),
		.hpdev.brdinfo = &mlxplat_mlxcpld_msn21xx_pwr[0],
		.hpdev.nr = MLXPLAT_CPLD_PSU_MSNXXXX_NR,
	},
	{
		.label = "pwr2",
		.reg = MLXPLAT_CPLD_LPC_REG_PWR_OFF,
		.mask = BIT(1),
		.hpdev.brdinfo = &mlxplat_mlxcpld_msn21xx_pwr[1],
		.hpdev.nr = MLXPLAT_CPLD_PSU_MSNXXXX_NR,
	},
};

static struct mlxreg_core_item mlxplat_mlxcpld_msn201x_items[] = {
	{
		.data = mlxplat_mlxcpld_msn201x_pwr_items_data,
		.aggr_mask = MLXPLAT_CPLD_AGGR_PWR_MASK_DEF,
		.reg = MLXPLAT_CPLD_LPC_REG_PWR_OFF,
		.mask = MLXPLAT_CPLD_PWR_MASK,
		.count = ARRAY_SIZE(mlxplat_mlxcpld_msn201x_pwr_items_data),
		.inversed = 0,
		.health = false,
	},
};

static
struct mlxreg_core_hotplug_platform_data mlxplat_mlxcpld_msn201x_data = {
	.items = mlxplat_mlxcpld_msn21xx_items,
	.counter = ARRAY_SIZE(mlxplat_mlxcpld_msn201x_items),
	.cell = MLXPLAT_CPLD_LPC_REG_AGGR_OFF,
	.mask = MLXPLAT_CPLD_AGGR_MASK_DEF,
	.cell_low = MLXPLAT_CPLD_LPC_REG_AGGR_LOW_OFF,
	.mask_low = MLXPLAT_CPLD_LOW_AGGR_MASK_LOW,
};

/* Platform hotplug next generation system family data */
static struct i2c_board_info mlxplat_mlxcpld_ng_fan = {
	I2C_BOARD_INFO("holder", 0x50),
};

static struct mlxreg_core_data mlxplat_mlxcpld_default_ng_psu_items_data[] = {
	{
		.label = "psu1",
		.reg = MLXPLAT_CPLD_LPC_REG_PSU_OFF,
		.mask = BIT(0),
		.hpdev.brdinfo = &mlxplat_mlxcpld_psu[0],
		.hpdev.nr = MLXPLAT_CPLD_PSU_MSNXXXX_NR,
	},
	{
		.label = "psu2",
		.reg = MLXPLAT_CPLD_LPC_REG_PSU_OFF,
		.mask = BIT(1),
		.hpdev.brdinfo = &mlxplat_mlxcpld_psu[1],
		.hpdev.nr = MLXPLAT_CPLD_PSU_MSNXXXX_NR,
	},
};

static struct mlxreg_core_data mlxplat_mlxcpld_default_ng_pwr_items_data[] = {
	{
		.label = "pwr1",
		.reg = MLXPLAT_CPLD_LPC_REG_PWR_OFF,
		.mask = BIT(0),
		.hpdev.brdinfo = &mlxplat_mlxcpld_pwr[0],
		.hpdev.nr = MLXPLAT_CPLD_PSU_MSNXXXX_NR,
	},
	{
		.label = "pwr2",
		.reg = MLXPLAT_CPLD_LPC_REG_PWR_OFF,
		.mask = BIT(1),
		.hpdev.brdinfo = &mlxplat_mlxcpld_pwr[1],
		.hpdev.nr = MLXPLAT_CPLD_PSU_MSNXXXX_NR,
	},
};

static struct mlxreg_core_data mlxplat_mlxcpld_default_ng_fan_items_data[] = {
	{
		.label = "fan1",
		.reg = MLXPLAT_CPLD_LPC_REG_FAN_OFF,
		.mask = BIT(0),
		.hpdev.brdinfo = &mlxplat_mlxcpld_ng_fan,
		.hpdev.nr = 11,
	},
	{
		.label = "fan2",
		.reg = MLXPLAT_CPLD_LPC_REG_FAN_OFF,
		.mask = BIT(1),
		.hpdev.brdinfo = &mlxplat_mlxcpld_ng_fan,
		.hpdev.nr = 12,
	},
	{
		.label = "fan3",
		.reg = MLXPLAT_CPLD_LPC_REG_FAN_OFF,
		.mask = BIT(2),
		.hpdev.brdinfo = &mlxplat_mlxcpld_ng_fan,
		.hpdev.nr = 13,
	},
	{
		.label = "fan4",
		.reg = MLXPLAT_CPLD_LPC_REG_FAN_OFF,
		.mask = BIT(3),
		.hpdev.brdinfo = &mlxplat_mlxcpld_ng_fan,
		.hpdev.nr = 14,
	},
	{
		.label = "fan5",
		.reg = MLXPLAT_CPLD_LPC_REG_FAN_OFF,
		.mask = BIT(3),
		.hpdev.brdinfo = &mlxplat_mlxcpld_ng_fan,
		.hpdev.nr = 15,
	},
	{
		.label = "fan6",
		.reg = MLXPLAT_CPLD_LPC_REG_FAN_OFF,
		.mask = BIT(3),
		.hpdev.brdinfo = &mlxplat_mlxcpld_ng_fan,
		.hpdev.nr = 16,
	},
};

static struct mlxreg_core_item mlxplat_mlxcpld_default_ng_items[] = {
	{
		.data = mlxplat_mlxcpld_default_ng_psu_items_data,
		.aggr_mask = MLXPLAT_CPLD_AGGR_MASK_NG_DEF,
		.reg = MLXPLAT_CPLD_LPC_REG_PSU_OFF,
		.mask = MLXPLAT_CPLD_PSU_MASK,
		.count = ARRAY_SIZE(mlxplat_mlxcpld_psu),
		.inversed = 1,
		.health = false,
	},
	{
		.data = mlxplat_mlxcpld_default_ng_pwr_items_data,
		.aggr_mask = MLXPLAT_CPLD_AGGR_MASK_NG_DEF,
		.reg = MLXPLAT_CPLD_LPC_REG_PWR_OFF,
		.mask = MLXPLAT_CPLD_PWR_MASK,
		.count = ARRAY_SIZE(mlxplat_mlxcpld_pwr),
		.inversed = 0,
		.health = false,
	},
	{
		.data = mlxplat_mlxcpld_default_ng_fan_items_data,
		.aggr_mask = MLXPLAT_CPLD_AGGR_MASK_NG_DEF,
		.reg = MLXPLAT_CPLD_LPC_REG_FAN_OFF,
		.mask = MLXPLAT_CPLD_FAN_MASK,
		.count = ARRAY_SIZE(mlxplat_mlxcpld_default_ng_fan_items_data),
		.inversed = 1,
		.health = false,
	},
};

static
struct mlxreg_core_hotplug_platform_data mlxplat_mlxcpld_default_ng_data = {
	.items = mlxplat_mlxcpld_default_ng_items,
	.counter = ARRAY_SIZE(mlxplat_mlxcpld_default_ng_items),
	.cell = MLXPLAT_CPLD_LPC_REG_AGGR_OFF,
	.mask = MLXPLAT_CPLD_AGGR_MASK_NG_DEF,
	.cell_low = MLXPLAT_CPLD_LPC_REG_AGGR_LOW_OFF,
	.mask_low = MLXPLAT_CPLD_LOW_AGGR_MASK_LOW,
};

static struct mlxreg_core_data mlxplat_mlxcpld_msn274x_fan_items_data[] = {
	{
		.label = "fan1",
		.reg = MLXPLAT_CPLD_LPC_REG_FAN_OFF,
		.mask = BIT(0),
		.hpdev.brdinfo = &mlxplat_mlxcpld_ng_fan,
		.hpdev.nr = 11,
	},
	{
		.label = "fan2",
		.reg = MLXPLAT_CPLD_LPC_REG_FAN_OFF,
		.mask = BIT(1),
		.hpdev.brdinfo = &mlxplat_mlxcpld_ng_fan,
		.hpdev.nr = 12,
	},
	{
		.label = "fan3",
		.reg = MLXPLAT_CPLD_LPC_REG_FAN_OFF,
		.mask = BIT(2),
		.hpdev.brdinfo = &mlxplat_mlxcpld_ng_fan,
		.hpdev.nr = 13,
	},
	{
		.label = "fan4",
		.reg = MLXPLAT_CPLD_LPC_REG_FAN_OFF,
		.mask = BIT(3),
		.hpdev.brdinfo = &mlxplat_mlxcpld_ng_fan,
		.hpdev.nr = 14,
	},
};

static struct mlxreg_core_item mlxplat_mlxcpld_msn274x_items[] = {
	{
		.data = mlxplat_mlxcpld_default_ng_psu_items_data,
		.aggr_mask = MLXPLAT_CPLD_AGGR_MASK_NG_DEF,
		.reg = MLXPLAT_CPLD_LPC_REG_PSU_OFF,
		.mask = MLXPLAT_CPLD_PSU_MASK,
		.count = ARRAY_SIZE(mlxplat_mlxcpld_psu),
		.inversed = 1,
		.health = false,
	},
	{
		.data = mlxplat_mlxcpld_default_ng_pwr_items_data,
		.aggr_mask = MLXPLAT_CPLD_AGGR_MASK_NG_DEF,
		.reg = MLXPLAT_CPLD_LPC_REG_PWR_OFF,
		.mask = MLXPLAT_CPLD_PWR_MASK,
		.count = ARRAY_SIZE(mlxplat_mlxcpld_pwr),
		.inversed = 0,
		.health = false,
	},
	{
		.data = mlxplat_mlxcpld_msn274x_fan_items_data,
		.aggr_mask = MLXPLAT_CPLD_AGGR_MASK_NG_DEF,
		.reg = MLXPLAT_CPLD_LPC_REG_FAN_OFF,
		.mask = MLXPLAT_CPLD_FAN_MASK,
		.count = ARRAY_SIZE(mlxplat_mlxcpld_msn274x_fan_items_data),
		.inversed = 1,
		.health = false,
	},
};

static
struct mlxreg_core_hotplug_platform_data mlxplat_mlxcpld_msn274x_data = {
	.items = mlxplat_mlxcpld_msn274x_items,
	.counter = ARRAY_SIZE(mlxplat_mlxcpld_default_ng_items),
	.cell = MLXPLAT_CPLD_LPC_REG_AGGR_OFF,
	.mask = MLXPLAT_CPLD_AGGR_MASK_NG_DEF,
	.cell_low = MLXPLAT_CPLD_LPC_REG_AGGR_LOW_OFF,
	.mask_low = MLXPLAT_CPLD_LOW_AGGR_MASK_LOW,
};

/* Platform led default data */
static struct mlxreg_core_data mlxplat_mlxcpld_default_led_data[] = {
	{
		.label = "status:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED1_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
	{
		.label = "status:red",
		.reg = MLXPLAT_CPLD_LPC_REG_LED1_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK
	},
	{
		.label = "psu:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED1_OFF,
		.mask = MLXPLAT_CPLD_LED_HI_NIBBLE_MASK,
	},
	{
		.label = "psu:red",
		.reg = MLXPLAT_CPLD_LPC_REG_LED1_OFF,
		.mask = MLXPLAT_CPLD_LED_HI_NIBBLE_MASK,
	},
	{
		.label = "fan1:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED2_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
	{
		.label = "fan1:red",
		.reg = MLXPLAT_CPLD_LPC_REG_LED2_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
	{
		.label = "fan2:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED2_OFF,
		.mask = MLXPLAT_CPLD_LED_HI_NIBBLE_MASK,
	},
	{
		.label = "fan2:red",
		.reg = MLXPLAT_CPLD_LPC_REG_LED2_OFF,
		.mask = MLXPLAT_CPLD_LED_HI_NIBBLE_MASK,
	},
	{
		.label = "fan3:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED3_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
	{
		.label = "fan3:red",
		.reg = MLXPLAT_CPLD_LPC_REG_LED3_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
	{
		.label = "fan4:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED3_OFF,
		.mask = MLXPLAT_CPLD_LED_HI_NIBBLE_MASK,
	},
	{
		.label = "fan4:red",
		.reg = MLXPLAT_CPLD_LPC_REG_LED3_OFF,
		.mask = MLXPLAT_CPLD_LED_HI_NIBBLE_MASK,
	},
};

static struct mlxreg_core_platform_data mlxplat_default_led_data = {
		.data = mlxplat_mlxcpld_default_led_data,
		.counter = ARRAY_SIZE(mlxplat_mlxcpld_default_led_data),
};

/* Platform led MSN21xx system family data */
static struct mlxreg_core_data mlxplat_mlxcpld_msn21xx_led_data[] = {
	{
		.label = "status:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED1_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
	{
		.label = "status:red",
		.reg = MLXPLAT_CPLD_LPC_REG_LED1_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK
	},
	{
		.label = "fan:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED2_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
	{
		.label = "fan:red",
		.reg = MLXPLAT_CPLD_LPC_REG_LED2_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
	{
		.label = "psu1:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED4_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
	{
		.label = "psu1:red",
		.reg = MLXPLAT_CPLD_LPC_REG_LED4_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
	{
		.label = "psu2:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED4_OFF,
		.mask = MLXPLAT_CPLD_LED_HI_NIBBLE_MASK,
	},
	{
		.label = "psu2:red",
		.reg = MLXPLAT_CPLD_LPC_REG_LED4_OFF,
		.mask = MLXPLAT_CPLD_LED_HI_NIBBLE_MASK,
	},
	{
		.label = "uid:blue",
		.reg = MLXPLAT_CPLD_LPC_REG_LED5_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
};

static struct mlxreg_core_platform_data mlxplat_msn21xx_led_data = {
		.data = mlxplat_mlxcpld_msn21xx_led_data,
		.counter = ARRAY_SIZE(mlxplat_mlxcpld_msn21xx_led_data),
};

/* Platform led for default data for 200GbE systems */
static struct mlxreg_core_data mlxplat_mlxcpld_default_ng_led_data[] = {
	{
		.label = "status:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED1_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
	{
		.label = "status:orange",
		.reg = MLXPLAT_CPLD_LPC_REG_LED1_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK
	},
	{
		.label = "psu:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED1_OFF,
		.mask = MLXPLAT_CPLD_LED_HI_NIBBLE_MASK,
	},
	{
		.label = "psu:orange",
		.reg = MLXPLAT_CPLD_LPC_REG_LED1_OFF,
		.mask = MLXPLAT_CPLD_LED_HI_NIBBLE_MASK,
	},
	{
		.label = "fan1:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED2_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
	{
		.label = "fan1:orange",
		.reg = MLXPLAT_CPLD_LPC_REG_LED2_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
	{
		.label = "fan2:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED2_OFF,
		.mask = MLXPLAT_CPLD_LED_HI_NIBBLE_MASK,
	},
	{
		.label = "fan2:orange",
		.reg = MLXPLAT_CPLD_LPC_REG_LED2_OFF,
		.mask = MLXPLAT_CPLD_LED_HI_NIBBLE_MASK,
	},
	{
		.label = "fan3:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED3_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
	{
		.label = "fan3:orange",
		.reg = MLXPLAT_CPLD_LPC_REG_LED3_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
	{
		.label = "fan4:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED3_OFF,
		.mask = MLXPLAT_CPLD_LED_HI_NIBBLE_MASK,
	},
	{
		.label = "fan4:orange",
		.reg = MLXPLAT_CPLD_LPC_REG_LED3_OFF,
		.mask = MLXPLAT_CPLD_LED_HI_NIBBLE_MASK,
	},
	{
		.label = "fan5:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED4_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
	{
		.label = "fan5:orange",
		.reg = MLXPLAT_CPLD_LPC_REG_LED4_OFF,
		.mask = MLXPLAT_CPLD_LED_LO_NIBBLE_MASK,
	},
	{
		.label = "fan6:green",
		.reg = MLXPLAT_CPLD_LPC_REG_LED4_OFF,
		.mask = MLXPLAT_CPLD_LED_HI_NIBBLE_MASK,
	},
	{
		.label = "fan6:orange",
		.reg = MLXPLAT_CPLD_LPC_REG_LED4_OFF,
		.mask = MLXPLAT_CPLD_LED_HI_NIBBLE_MASK,
	},
};

static struct mlxreg_core_platform_data mlxplat_default_ng_led_data = {
		.data = mlxplat_mlxcpld_default_ng_led_data,
		.counter = ARRAY_SIZE(mlxplat_mlxcpld_default_ng_led_data),
};

static bool mlxplat_mlxcpld_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MLXPLAT_CPLD_LPC_REG_LED1_OFF:
	case MLXPLAT_CPLD_LPC_REG_LED2_OFF:
	case MLXPLAT_CPLD_LPC_REG_LED3_OFF:
	case MLXPLAT_CPLD_LPC_REG_LED4_OFF:
	case MLXPLAT_CPLD_LPC_REG_LED5_OFF:
	case MLXPLAT_CPLD_LPC_REG_GP1_OFF:
	case MLXPLAT_CPLD_LPC_REG_WP1_OFF:
	case MLXPLAT_CPLD_LPC_REG_GP2_OFF:
	case MLXPLAT_CPLD_LPC_REG_WP2_OFF:
	case MLXPLAT_CPLD_LPC_REG_AGGR_MASK_OFF:
	case MLXPLAT_CPLD_LPC_REG_AGGR_LOW_MASK_OFF:
	case MLXPLAT_CPLD_LPC_REG_PSU_EVENT_OFF:
	case MLXPLAT_CPLD_LPC_REG_PSU_MASK_OFF:
	case MLXPLAT_CPLD_LPC_REG_PWR_EVENT_OFF:
	case MLXPLAT_CPLD_LPC_REG_PWR_MASK_OFF:
	case MLXPLAT_CPLD_LPC_REG_FAN_EVENT_OFF:
	case MLXPLAT_CPLD_LPC_REG_FAN_MASK_OFF:
		return true;
	}
	return false;
}

static bool mlxplat_mlxcpld_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MLXPLAT_CPLD_LPC_REG_CPLD1_VER_OFF:
	case MLXPLAT_CPLD_LPC_REG_CPLD2_VER_OFF:
	case MLXPLAT_CPLD_LPC_REG_RESET_CAUSE_OFF:
	case MLXPLAT_CPLD_LPC_REG_LED1_OFF:
	case MLXPLAT_CPLD_LPC_REG_LED2_OFF:
	case MLXPLAT_CPLD_LPC_REG_LED3_OFF:
	case MLXPLAT_CPLD_LPC_REG_LED4_OFF:
	case MLXPLAT_CPLD_LPC_REG_LED5_OFF:
	case MLXPLAT_CPLD_LPC_REG_GP1_OFF:
	case MLXPLAT_CPLD_LPC_REG_WP1_OFF:
	case MLXPLAT_CPLD_LPC_REG_GP2_OFF:
	case MLXPLAT_CPLD_LPC_REG_WP2_OFF:
	case MLXPLAT_CPLD_LPC_REG_AGGR_OFF:
	case MLXPLAT_CPLD_LPC_REG_AGGR_MASK_OFF:
	case MLXPLAT_CPLD_LPC_REG_AGGR_LOW_OFF:
	case MLXPLAT_CPLD_LPC_REG_AGGR_LOW_MASK_OFF:
	case MLXPLAT_CPLD_LPC_REG_PSU_OFF:
	case MLXPLAT_CPLD_LPC_REG_PSU_EVENT_OFF:
	case MLXPLAT_CPLD_LPC_REG_PSU_MASK_OFF:
	case MLXPLAT_CPLD_LPC_REG_PWR_OFF:
	case MLXPLAT_CPLD_LPC_REG_PWR_EVENT_OFF:
	case MLXPLAT_CPLD_LPC_REG_PWR_MASK_OFF:
	case MLXPLAT_CPLD_LPC_REG_FAN_OFF:
	case MLXPLAT_CPLD_LPC_REG_FAN_EVENT_OFF:
	case MLXPLAT_CPLD_LPC_REG_FAN_MASK_OFF:
		return true;
	}
	return false;
}

static bool mlxplat_mlxcpld_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MLXPLAT_CPLD_LPC_REG_CPLD1_VER_OFF:
	case MLXPLAT_CPLD_LPC_REG_CPLD2_VER_OFF:
	case MLXPLAT_CPLD_LPC_REG_RESET_CAUSE_OFF:
	case MLXPLAT_CPLD_LPC_REG_LED1_OFF:
	case MLXPLAT_CPLD_LPC_REG_LED2_OFF:
	case MLXPLAT_CPLD_LPC_REG_LED3_OFF:
	case MLXPLAT_CPLD_LPC_REG_LED4_OFF:
	case MLXPLAT_CPLD_LPC_REG_LED5_OFF:
	case MLXPLAT_CPLD_LPC_REG_GP1_OFF:
	case MLXPLAT_CPLD_LPC_REG_GP2_OFF:
	case MLXPLAT_CPLD_LPC_REG_AGGR_OFF:
	case MLXPLAT_CPLD_LPC_REG_AGGR_MASK_OFF:
	case MLXPLAT_CPLD_LPC_REG_AGGR_LOW_OFF:
	case MLXPLAT_CPLD_LPC_REG_AGGR_LOW_MASK_OFF:
	case MLXPLAT_CPLD_LPC_REG_PSU_OFF:
	case MLXPLAT_CPLD_LPC_REG_PSU_EVENT_OFF:
	case MLXPLAT_CPLD_LPC_REG_PSU_MASK_OFF:
	case MLXPLAT_CPLD_LPC_REG_PWR_OFF:
	case MLXPLAT_CPLD_LPC_REG_PWR_EVENT_OFF:
	case MLXPLAT_CPLD_LPC_REG_PWR_MASK_OFF:
	case MLXPLAT_CPLD_LPC_REG_FAN_OFF:
	case MLXPLAT_CPLD_LPC_REG_FAN_EVENT_OFF:
	case MLXPLAT_CPLD_LPC_REG_FAN_MASK_OFF:
		return true;
	}
	return false;
}

static const struct reg_default mlxplat_mlxcpld_regmap_default[] = {
	{ MLXPLAT_CPLD_LPC_REG_WP1_OFF, 0x00 },
	{ MLXPLAT_CPLD_LPC_REG_WP2_OFF, 0x00 },
};

static int
mlxplat_mlxcpld_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	*val = ioread8(context + reg);
	return 0;
}

static int
mlxplat_mlxcpld_reg_write(void *context, unsigned int reg, unsigned int val)
{
	iowrite8(val, context + reg);
	return 0;
}

const struct regmap_config mlxplat_mlxcpld_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 255,
	.cache_type = REGCACHE_FLAT,
	.writeable_reg = mlxplat_mlxcpld_writeable_reg,
	.readable_reg = mlxplat_mlxcpld_readable_reg,
	.volatile_reg = mlxplat_mlxcpld_volatile_reg,
	.reg_defaults = mlxplat_mlxcpld_regmap_default,
	.num_reg_defaults = ARRAY_SIZE(mlxplat_mlxcpld_regmap_default),
	.reg_read = mlxplat_mlxcpld_reg_read,
	.reg_write = mlxplat_mlxcpld_reg_write,
};

static struct resource mlxplat_mlxcpld_resources[] = {
	[0] = DEFINE_RES_IRQ_NAMED(17, "mlxreg-hotplug"),
};

static struct mlxreg_core_data mlxplat_mlxcpld_default_regs_io_data[] = {
	{ "cpld1_version", MLXPLAT_CPLD_LPC_REG_CPLD1_VER_OFF, 0x00,
	  GENMASK(7, 0), 0444 },
	{ "cpld2_version", MLXPLAT_CPLD_LPC_REG_CPLD2_VER_OFF, 0x00,
	  GENMASK(7, 0), 0444 },
	{ "cause_long_pb", MLXPLAT_CPLD_LPC_REG_RESET_CAUSE_OFF,
	  GENMASK(7, 0) & ~BIT(0), 0x00, 0444 },
	{ "cause_short_pb", MLXPLAT_CPLD_LPC_REG_RESET_CAUSE_OFF,
	  GENMASK(7, 0) & ~BIT(1), 0x00, 0444 },
	{ "cause_pwr_aux", MLXPLAT_CPLD_LPC_REG_RESET_CAUSE_OFF,
	  GENMASK(7, 0) & ~BIT(2), 0x00, 0444 },
	{ "cause_pwr_fail", MLXPLAT_CPLD_LPC_REG_RESET_CAUSE_OFF,
	  GENMASK(7, 0) & ~BIT(3), 0x00, 0444 },
	{ "psu1_on", MLXPLAT_CPLD_LPC_REG_GP1_OFF, GENMASK(7, 0) & ~BIT(0),
	  0x00, 0200 },
	{ "psu2_on", MLXPLAT_CPLD_LPC_REG_GP1_OFF,  GENMASK(7, 0) & ~BIT(1),
	  0x00, 0200 },
	{ "pwr_cycle", MLXPLAT_CPLD_LPC_REG_GP1_OFF, GENMASK(7, 0) & ~BIT(2),
	  0x00, 0200 },
	{ "select_iio", MLXPLAT_CPLD_LPC_REG_GP2_OFF, GENMASK(7, 0) & ~BIT(6),
	  0x00, 0644 },
};

static struct mlxreg_core_platform_data mlxplat_default_regs_io_data = {
		.data = mlxplat_mlxcpld_default_regs_io_data,
		.counter = ARRAY_SIZE(mlxplat_mlxcpld_default_regs_io_data),
};

static struct platform_device *mlxplat_dev;
static struct mlxreg_core_hotplug_platform_data *mlxplat_hotplug;
static struct mlxreg_core_platform_data *mlxplat_led;
static struct mlxreg_core_platform_data *mlxplat_regs_io;

static int __init mlxplat_dmi_default_matched(const struct dmi_system_id *dmi)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mlxplat_mux_data); i++) {
		mlxplat_mux_data[i].values = mlxplat_default_channels[i];
		mlxplat_mux_data[i].n_values =
				ARRAY_SIZE(mlxplat_default_channels[i]);
	}
	mlxplat_hotplug = &mlxplat_mlxcpld_default_data;
	mlxplat_led = &mlxplat_default_led_data;
	mlxplat_regs_io = &mlxplat_default_regs_io_data;

	return 1;
};

static int __init mlxplat_dmi_msn21xx_matched(const struct dmi_system_id *dmi)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mlxplat_mux_data); i++) {
		mlxplat_mux_data[i].values = mlxplat_msn21xx_channels;
		mlxplat_mux_data[i].n_values =
				ARRAY_SIZE(mlxplat_msn21xx_channels);
	}
	mlxplat_hotplug = &mlxplat_mlxcpld_msn21xx_data;
	mlxplat_led = &mlxplat_msn21xx_led_data;
	mlxplat_regs_io = &mlxplat_default_regs_io_data;

	return 1;
};

static int __init mlxplat_dmi_msn274x_matched(const struct dmi_system_id *dmi)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mlxplat_mux_data); i++) {
		mlxplat_mux_data[i].values = mlxplat_msn21xx_channels;
		mlxplat_mux_data[i].n_values =
				ARRAY_SIZE(mlxplat_msn21xx_channels);
	}
	mlxplat_hotplug = &mlxplat_mlxcpld_msn274x_data;
	mlxplat_led = &mlxplat_default_led_data;
	mlxplat_regs_io = &mlxplat_default_regs_io_data;

	return 1;
};

static int __init mlxplat_dmi_qmb7xx_matched(const struct dmi_system_id *dmi)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mlxplat_mux_data); i++) {
		mlxplat_mux_data[i].values = mlxplat_msn21xx_channels;
		mlxplat_mux_data[i].n_values =
				ARRAY_SIZE(mlxplat_msn21xx_channels);
	}
	mlxplat_hotplug = &mlxplat_mlxcpld_default_ng_data;
	mlxplat_led = &mlxplat_default_ng_led_data;
	mlxplat_regs_io = &mlxplat_default_regs_io_data;

	return 1;
};

static int __init mlxplat_dmi_msn201x_matched(const struct dmi_system_id *dmi)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mlxplat_mux_data); i++) {
		mlxplat_mux_data[i].values = mlxplat_msn21xx_channels;
		mlxplat_mux_data[i].n_values =
				ARRAY_SIZE(mlxplat_msn21xx_channels);
	}
	mlxplat_hotplug = &mlxplat_mlxcpld_msn201x_data;
	mlxplat_led = &mlxplat_msn21xx_led_data;
	mlxplat_regs_io = &mlxplat_default_regs_io_data;

	return 1;
};

static struct dmi_system_id mlxplat_dmi_table[] __initdata = {
	{
		.callback = mlxplat_dmi_msn274x_matched,
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "Mellanox Technologies"),
			DMI_MATCH(DMI_PRODUCT_NAME, "MSN274"),
		},
	},
	{
		.callback = mlxplat_dmi_default_matched,
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "Mellanox Technologies"),
			DMI_MATCH(DMI_PRODUCT_NAME, "MSN24"),
		},
	},
	{
		.callback = mlxplat_dmi_default_matched,
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "Mellanox Technologies"),
			DMI_MATCH(DMI_PRODUCT_NAME, "MSN27"),
		},
	},
	{
		.callback = mlxplat_dmi_default_matched,
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "Mellanox Technologies"),
			DMI_MATCH(DMI_PRODUCT_NAME, "MSB"),
		},
	},
	{
		.callback = mlxplat_dmi_default_matched,
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "Mellanox Technologies"),
			DMI_MATCH(DMI_PRODUCT_NAME, "MSX"),
		},
	},
	{
		.callback = mlxplat_dmi_msn21xx_matched,
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "Mellanox Technologies"),
			DMI_MATCH(DMI_PRODUCT_NAME, "MSN21"),
		},
	},
	{
		.callback = mlxplat_dmi_msn201x_matched,
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "Mellanox Technologies"),
			DMI_MATCH(DMI_PRODUCT_NAME, "MSN201"),
		},
	},
	{
		.callback = mlxplat_dmi_qmb7xx_matched,
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "Mellanox Technologies"),
			DMI_MATCH(DMI_PRODUCT_NAME, "QMB7"),
		},
	},
	{
		.callback = mlxplat_dmi_qmb7xx_matched,
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "Mellanox Technologies"),
			DMI_MATCH(DMI_PRODUCT_NAME, "SN37"),
		},
	},
	{
		.callback = mlxplat_dmi_qmb7xx_matched,
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "Mellanox Technologies"),
			DMI_MATCH(DMI_PRODUCT_NAME, "SN34"),
		},
	},
	{ }
};

static int __init mlxplat_init(void)
{
	struct mlxplat_priv *priv;
	void __iomem *base;
	int i, j, err = 0;

	if (!dmi_check_system(mlxplat_dmi_table))
		return -ENODEV;

	mlxplat_dev = platform_device_register_simple(MLX_PLAT_DEVICE_NAME, -1,
					mlxplat_lpc_resources,
					ARRAY_SIZE(mlxplat_lpc_resources));

	if (IS_ERR(mlxplat_dev))
		return PTR_ERR(mlxplat_dev);

	priv = devm_kzalloc(&mlxplat_dev->dev, sizeof(struct mlxplat_priv),
			    GFP_KERNEL);
	if (!priv) {
		err = -ENOMEM;
		goto fail_alloc;
	}
	platform_set_drvdata(mlxplat_dev, priv);

	priv->pdev_i2c = platform_device_register_simple("i2c_mlxcpld", -1,
							 NULL, 0);
	if (IS_ERR(priv->pdev_i2c)) {
		err = PTR_ERR(priv->pdev_i2c);
		goto fail_alloc;
	}

	for (i = 0; i < ARRAY_SIZE(mlxplat_mux_data); i++) {
		priv->pdev_mux[i] = platform_device_register_resndata(
						&mlxplat_dev->dev,
						"i2c-mux-reg", i, NULL,
						0, &mlxplat_mux_data[i],
						sizeof(mlxplat_mux_data[i]));
		if (IS_ERR(priv->pdev_mux[i])) {
			err = PTR_ERR(priv->pdev_mux[i]);
			goto fail_platform_mux_register;
		}
	}

	base = devm_ioport_map(&mlxplat_dev->dev,
			       mlxplat_lpc_resources[1].start, 1);
	if (IS_ERR(base))
		goto fail_platform_mux_register;

	mlxplat_hotplug->regmap = devm_regmap_init(&mlxplat_dev->dev, NULL,
					base, &mlxplat_mlxcpld_regmap_config);
	if (IS_ERR(mlxplat_hotplug->regmap))
		goto fail_platform_mux_register;

	/* Set default registers. */
	for (j = 0; j <  mlxplat_mlxcpld_regmap_config.num_reg_defaults; j++) {
		err = regmap_write(mlxplat_hotplug->regmap,
				   mlxplat_mlxcpld_regmap_default[j].reg,
				   mlxplat_mlxcpld_regmap_default[j].def);
		if (err)
			goto fail_platform_mux_register;
	}

	priv->pdev_hotplug = platform_device_register_resndata(
				&mlxplat_dev->dev, "mlxreg-hotplug",
				PLATFORM_DEVID_NONE,
				mlxplat_mlxcpld_resources,
				ARRAY_SIZE(mlxplat_mlxcpld_resources),
				mlxplat_hotplug, sizeof(*mlxplat_hotplug));
	if (IS_ERR(priv->pdev_hotplug)) {
		err = PTR_ERR(priv->pdev_hotplug);
		goto fail_platform_mux_register;
	}

	mlxplat_led->regmap = mlxplat_hotplug->regmap;
	priv->pdev_led = platform_device_register_resndata(
				&mlxplat_dev->dev, "leds-mlxreg",
				PLATFORM_DEVID_NONE, NULL, 0,
				mlxplat_led, sizeof(*mlxplat_led));
	if (IS_ERR(priv->pdev_led)) {
		err = PTR_ERR(priv->pdev_led);
		goto fail_platform_hotplug_register;
	}

	mlxplat_regs_io->regmap = mlxplat_hotplug->regmap;
	priv->pdev_io_regs = platform_device_register_resndata(
				&mlxplat_dev->dev, "mlxreg-io",
				PLATFORM_DEVID_NONE, NULL, 0,
				mlxplat_regs_io, sizeof(*mlxplat_regs_io));
	if (IS_ERR(priv->pdev_io_regs)) {
		err = PTR_ERR(priv->pdev_io_regs);
		goto fail_platform_led_register;
	}

	/* Sync registers with hardware. */
	regcache_mark_dirty(mlxplat_hotplug->regmap);
	err = regcache_sync(mlxplat_hotplug->regmap);
	if (err)
		goto fail_platform_led_register;

	return 0;

fail_platform_led_register:
	platform_device_unregister(priv->pdev_led);
fail_platform_hotplug_register:
	platform_device_unregister(priv->pdev_hotplug);
fail_platform_mux_register:
	while (--i >= 0)
		platform_device_unregister(priv->pdev_mux[i]);
	platform_device_unregister(priv->pdev_i2c);
fail_alloc:
	platform_device_unregister(mlxplat_dev);

	return err;
}
module_init(mlxplat_init);

static void __exit mlxplat_exit(void)
{
	struct mlxplat_priv *priv = platform_get_drvdata(mlxplat_dev);
	int i;

	platform_device_unregister(priv->pdev_io_regs);
	platform_device_unregister(priv->pdev_led);
	platform_device_unregister(priv->pdev_hotplug);

	for (i = ARRAY_SIZE(mlxplat_mux_data) - 1; i >= 0 ; i--)
		platform_device_unregister(priv->pdev_mux[i]);

	platform_device_unregister(priv->pdev_i2c);
	platform_device_unregister(mlxplat_dev);
}
module_exit(mlxplat_exit);

MODULE_AUTHOR("Vadim Pasternak (vadimp@mellanox.com)");
MODULE_DESCRIPTION("Mellanox platform driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("dmi:*:*Mellanox*:MSN24*:");
MODULE_ALIAS("dmi:*:*Mellanox*:MSN27*:");
MODULE_ALIAS("dmi:*:*Mellanox*:MSB*:");
MODULE_ALIAS("dmi:*:*Mellanox*:MSX*:");
MODULE_ALIAS("dmi:*:*Mellanox*:MSN21*:");
MODULE_ALIAS("dmi:*:*Mellanox*MSN274*:");
MODULE_ALIAS("dmi:*:*Mellanox*MSN201*:");
MODULE_ALIAS("dmi:*:*Mellanox*QMB7*:");
MODULE_ALIAS("dmi:*:*Mellanox*SN37*:");
MODULE_ALIAS("dmi:*:*Mellanox*QM34*:");
