/*
 * This file is part of the coreboot project.
 *
 * Copyright 2018 MediaTek Inc.
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

#include <soc/emi.h>

struct sdram_params params = {
	.impedance = {
		[ODT_OFF] = {0x8, 0x7, 0x0, 0xF},
		[ODT_ON] = {0x9, 0x9, 0x0, 0xE}
	},
	.wr_level = {
		[CHANNEL_A] = { {0x1F, 0x1C}, {0x1C, 0x1B} },
		[CHANNEL_B] = { {0x27, 0x28}, {0x23, 0x28} }
	},
	.cbt_cs = {
		[CHANNEL_A] = {0x3, 0x3},
		[CHANNEL_B] = {0x4, 0x6}
	},
	.cbt_mr12 = {
		[CHANNEL_A] = {0x5C, 0x5A},
		[CHANNEL_B] = {0x5C, 0x5A}
	},
	.emi_cona_val = 0xF053F154,
	.emi_conh_val = 0x44440003,
	.emi_conf_val = 0x00421000,
	.chn_emi_cona_val = {0x0444F051, 0x0444F051},
	.cbt_mode_extern = CBT_NORMAL_MODE,
	.delay_cell_unit = 868,
};
