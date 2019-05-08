/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2018 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <baseboard/variants.h>
#include <baseboard/gpio.h>
#include <gpio.h>
#include <soc/cnl_memcfg_init.h>
#include <string.h>

static const struct cnl_mb_cfg baseboard_memcfg = {
	/* Baseboard uses 121, 81 and 100 rcomp resistors */
	.rcomp_resistor = { 121, 81, 100 },

	/* Baseboard Rcomp target values */
	.rcomp_targets = { 100, 40, 20, 20, 26 },

	/* Set CaVref config to 2 */
	.vref_ca_config = 2,

	/* Enable Early Command Training */
	.ect = 1,
};

void __weak variant_memory_params(struct cnl_mb_cfg *bcfg)
{
	memcpy(bcfg, &baseboard_memcfg, sizeof(baseboard_memcfg));
	/*
	 * GPP_F2 is the MEM_CH_SEL gpio, which is set to 1 for single
	 * channel skus and 0 for dual channel skus.
	 */
	if (gpio_get(GPP_F2) == 1) {
		/*
		 * Single channel config: for Hatch, Channel 0 is
		 * always populated.
		 */
		bcfg->channel_empty[0] = 0;
		bcfg->channel_empty[1] = 1;
	} else {
		/* Dual channel config: both channels populated. */
		bcfg->channel_empty[0] = 0;
		bcfg->channel_empty[1] = 0;
	}
}

int __weak variant_memory_sku(void)
{
	const gpio_t spd_gpios[] = {
		GPIO_MEM_CONFIG_0,
		GPIO_MEM_CONFIG_1,
		GPIO_MEM_CONFIG_2,
		GPIO_MEM_CONFIG_3,
	};

	return gpio_base2_value(spd_gpios, ARRAY_SIZE(spd_gpios));
}
