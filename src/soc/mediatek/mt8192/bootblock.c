/* SPDX-License-Identifier: GPL-2.0-only */

#include <bootblock_common.h>
#include <soc/mmu_operations.h>
#include <soc/pll.h>
#include <soc/pmif.h>
#include <soc/wdt.h>

void bootblock_soc_init(void)
{
	mtk_mmu_init();
	mtk_wdt_init();
	mt_pll_init();
	mtk_pmif_init();
}
