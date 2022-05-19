/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <device/device.h>
#include <device/pci.h>
#include <device/pci_ids.h>
#include <intelblocks/lpc_lib.h>
#include <soc/pm.h>

#include "lpc_def.h"

/* SoC overrides */

/* Common weak definition, needs to be implemented in each soc LPC driver. */
__weak void lpc_soc_init(struct device *dev)
{
	/* no-op */
}

/* Fill up LPC IO resource structure inside SoC directory */
__weak void pch_lpc_soc_fill_io_resources(struct device *dev)
{
	/* no-op */
}

void pch_lpc_add_new_resource(struct device *dev, uint8_t offset,
	uintptr_t base, size_t size, unsigned long flags)
{
	struct resource *res;
	res = new_resource(dev, offset);
	res->base = base;
	res->size = size;
	res->flags = flags;
}

static void pch_lpc_add_io_resources(struct device *dev)
{
	uint32_t gen_io_dec;
	uint16_t base, size;

	/* Add the default claimed legacy IO range for the LPC device. */
	pch_lpc_add_new_resource(dev, 0, 0, 0x1000, IORESOURCE_IO |
			IORESOURCE_ASSIGNED | IORESOURCE_FIXED);

	/* LPC Generic IO Decode ranges */
	for (size_t i = 0; i < LPC_NUM_GENERIC_IO_RANGES; i++) {
		gen_io_dec = pci_read_config32(dev, LPC_GENERIC_IO_RANGE(i));
		if (gen_io_dec & LPC_LGIR_EN) {
			base = gen_io_dec & LPC_LGIR_ADDR_MASK;
			size = (0x3 | ((gen_io_dec >> 16) & 0xfc)) + 1;
			pch_lpc_add_new_resource(dev, LPC_GENERIC_IO_RANGE(i), base, size,
						 IORESOURCE_IO | IORESOURCE_ASSIGNED |
						 IORESOURCE_FIXED);
		}
	}

	/* SoC IO resource overrides */
	pch_lpc_soc_fill_io_resources(dev);
}

static void pch_lpc_add_mmio_resources(struct device *dev)
{
	/* LPC Memory Decode */
	uint32_t lgmr = pci_read_config32(dev, LPC_GENERIC_MEM_RANGE);
	if (lgmr & LPC_LGMR_EN) {
		lgmr &= LPC_LGMR_ADDR_MASK;
		pch_lpc_add_new_resource(dev, LPC_GENERIC_MEM_RANGE, lgmr, LPC_LGMR_WINDOW_SIZE,
					 IORESOURCE_MEM | IORESOURCE_ASSIGNED |
					 IORESOURCE_FIXED | IORESOURCE_RESERVE);
	}
}

static void pch_lpc_read_resources(struct device *dev)
{
	/* Get the PCI resources of this device. */
	pci_dev_read_resources(dev);

	/* Add IO resources to LPC. */
	pch_lpc_add_io_resources(dev);

	/* Add non-standard MMIO resources. */
	pch_lpc_add_mmio_resources(dev);
}

static void pch_lpc_set_child_resources(struct device *dev);

static void pch_lpc_loop_resources(struct device *dev)
{
	struct resource *res;

	for (res = dev->resource_list; res; res = res->next) {
		if (res->flags & IORESOURCE_IO)
			lpc_open_pmio_window(res->base, res->size);
	}
	pch_lpc_set_child_resources(dev);
}

/*
 * Loop through all the child devices' resources, and open up windows to the
 * LPC bus, as appropriate.
 */
static void pch_lpc_set_child_resources(struct device *dev)
{
	struct bus *link;
	struct device *child;

	for (link = dev->link_list; link; link = link->next) {
		for (child = link->children; child; child = child->sibling)
			pch_lpc_loop_resources(child);
	}
}

static void pch_lpc_set_resources(struct device *dev)
{
	pci_dev_set_resources(dev);

	/* Now open up windows to devices which have declared resources. */
	pch_lpc_set_child_resources(dev);
}

#if CONFIG(HAVE_ACPI_TABLES)
static const char *lpc_acpi_name(const struct device *dev)
{
	return "LPCB";
}
#endif

static struct device_operations device_ops = {
	.read_resources		= pch_lpc_read_resources,
	.set_resources		= pch_lpc_set_resources,
	.enable_resources	= pci_dev_enable_resources,
#if CONFIG(HAVE_ACPI_TABLES)
	.write_acpi_tables	= southbridge_write_acpi_tables,
	.acpi_name		= lpc_acpi_name,
#endif
	.init			= lpc_soc_init,
	.scan_bus		= scan_static_bus,
	.ops_pci		= &pci_dev_ops_pci,
};

static const unsigned short pci_device_ids[] = {
	PCI_DID_INTEL_MTL_ESPI_0,
	PCI_DID_INTEL_MTL_ESPI_1,
	PCI_DID_INTEL_MTL_ESPI_2,
	PCI_DID_INTEL_MTL_ESPI_3,
	PCI_DID_INTEL_MTL_ESPI_4,
	PCI_DID_INTEL_MTL_ESPI_5,
	PCI_DID_INTEL_MTL_ESPI_6,
	PCI_DID_INTEL_MTL_ESPI_7,
	PCI_DID_INTEL_RPP_P_ESPI_0,
	PCI_DID_INTEL_RPP_P_ADP_P_ESPI_1,
	PCI_DID_INTEL_RPP_P_ADP_P_ESPI_2,
	PCI_DID_INTEL_RPP_P_ESPI_3,
	PCI_DID_INTEL_RPP_P_ESPI_4,
	PCI_DID_INTEL_RPP_P_ESPI_5,
	PCI_DID_INTEL_RPP_P_ADP_M_ESPI_6,
	PCI_DID_INTEL_RPP_P_ESPI_7,
	PCI_DID_INTEL_RPP_P_ESPI_8,
	PCI_DID_INTEL_RPP_P_ESPI_9,
	PCI_DID_INTEL_RPP_P_ESPI_10,
	PCI_DID_INTEL_RPP_P_ESPI_11,
	PCI_DID_INTEL_RPP_P_ESPI_12,
	PCI_DID_INTEL_RPP_P_ESPI_13,
	PCI_DID_INTEL_RPP_P_ESPI_14,
	PCI_DID_INTEL_RPP_P_ESPI_15,
	PCI_DID_INTEL_RPP_P_ESPI_16,
	PCI_DID_INTEL_RPP_P_ESPI_17,
	PCI_DID_INTEL_RPP_P_ESPI_18,
	PCI_DID_INTEL_RPP_P_ESPI_19,
	PCI_DID_INTEL_RPP_P_ESPI_20,
	PCI_DID_INTEL_RPP_P_ESPI_21,
	PCI_DID_INTEL_RPP_P_ESPI_22,
	PCI_DID_INTEL_RPP_P_ESPI_23,
	PCI_DID_INTEL_RPP_P_ESPI_24,
	PCI_DID_INTEL_RPP_P_ESPI_25,
	PCI_DID_INTEL_RPP_P_ESPI_26,
	PCI_DID_INTEL_RPP_P_ESPI_27,
	PCI_DID_INTEL_RPP_P_ESPI_28,
	PCI_DID_INTEL_RPP_P_ESPI_29,
	PCI_DID_INTEL_RPP_P_ESPI_30,
	PCI_DID_INTEL_RPP_P_ESPI_31,
	PCI_DID_INTEL_SPT_LP_SAMPLE,
	PCI_DID_INTEL_SPT_LP_U_BASE,
	PCI_DID_INTEL_SPT_LP_U_PREMIUM,
	PCI_DID_INTEL_SPT_LP_Y_PREMIUM,
	PCI_DID_INTEL_SPT_H_H110,
	PCI_DID_INTEL_SPT_H_H170,
	PCI_DID_INTEL_SPT_H_Z170,
	PCI_DID_INTEL_SPT_H_Q170,
	PCI_DID_INTEL_SPT_H_Q150,
	PCI_DID_INTEL_SPT_H_B150,
	PCI_DID_INTEL_SPT_H_C236,
	PCI_DID_INTEL_SPT_H_C232,
	PCI_DID_INTEL_SPT_H_QM170,
	PCI_DID_INTEL_SPT_H_HM170,
	PCI_DID_INTEL_SPT_H_CM236,
	PCI_DID_INTEL_SPT_H_HM175,
	PCI_DID_INTEL_SPT_H_QM175,
	PCI_DID_INTEL_SPT_H_CM238,
	PCI_DID_INTEL_LWB_C621,
	PCI_DID_INTEL_LWB_C622,
	PCI_DID_INTEL_LWB_C624,
	PCI_DID_INTEL_LWB_C625,
	PCI_DID_INTEL_LWB_C626,
	PCI_DID_INTEL_LWB_C627,
	PCI_DID_INTEL_LWB_C628,
	PCI_DID_INTEL_LWB_C629,
	PCI_DID_INTEL_LWB_C621A,
	PCI_DID_INTEL_LWB_C627A,
	PCI_DID_INTEL_LWB_C629A,
	PCI_DID_INTEL_LWB_C624_SUPER,
	PCI_DID_INTEL_LWB_C627_SUPER_1,
	PCI_DID_INTEL_LWB_C621_SUPER,
	PCI_DID_INTEL_LWB_C627_SUPER_2,
	PCI_DID_INTEL_LWB_C628_SUPER,
	PCI_DID_INTEL_LWB_C621A_SUPER,
	PCI_DID_INTEL_LWB_C627A_SUPER,
	PCI_DID_INTEL_LWB_C629A_SUPER,
	PCI_DID_INTEL_EMB_SUPER,
	PCI_DID_INTEL_UPT_H_Q270,
	PCI_DID_INTEL_UPT_H_H270,
	PCI_DID_INTEL_UPT_H_Z270,
	PCI_DID_INTEL_UPT_H_Q250,
	PCI_DID_INTEL_UPT_H_B250,
	PCI_DID_INTEL_UPT_H_Z370,
	PCI_DID_INTEL_UPT_H_H310C,
	PCI_DID_INTEL_UPT_H_B365,
	PCI_DID_INTEL_SPT_LP_Y_PREMIUM_HDCP22,
	PCI_DID_INTEL_SPT_LP_U_PREMIUM_HDCP22,
	PCI_DID_INTEL_SPT_LP_U_BASE_HDCP22,
	PCI_DID_INTEL_UPT_LP_SUPER_SKU,
	PCI_DID_INTEL_UPT_LP_U_BASE,
	PCI_DID_INTEL_UPT_LP_U_PREMIUM,
	PCI_DID_INTEL_UPT_LP_Y_PREMIUM,
	PCI_DID_INTEL_APL_LPC,
	PCI_DID_INTEL_GLK_LPC,
	PCI_DID_INTEL_GLK_ESPI,
	PCI_DID_INTEL_CNL_BASE_U_LPC,
	PCI_DID_INTEL_CNL_U_PREMIUM_LPC,
	PCI_DID_INTEL_CNL_Y_PREMIUM_LPC,
	PCI_DID_INTEL_CNP_H_LPC_H310,
	PCI_DID_INTEL_CNP_H_LPC_H370,
	PCI_DID_INTEL_CNP_H_LPC_Z390,
	PCI_DID_INTEL_CNP_H_LPC_Q370,
	PCI_DID_INTEL_CNP_H_LPC_B360,
	PCI_DID_INTEL_CNP_H_LPC_C246,
	PCI_DID_INTEL_CNP_H_LPC_C242,
	PCI_DID_INTEL_CNP_H_LPC_QM370,
	PCI_DID_INTEL_CNP_H_LPC_HM370,
	PCI_DID_INTEL_CNP_H_LPC_CM246,
	PCI_DID_INTEL_ICL_BASE_U_ESPI,
	PCI_DID_INTEL_ICL_BASE_Y_ESPI,
	PCI_DID_INTEL_ICL_U_PREMIUM_ESPI,
	PCI_DID_INTEL_ICL_U_SUPER_U_ESPI,
	PCI_DID_INTEL_ICL_U_SUPER_U_ESPI_REV0,
	PCI_DID_INTEL_ICL_SUPER_Y_ESPI,
	PCI_DID_INTEL_ICL_Y_PREMIUM_ESPI,
	PCI_DID_INTEL_CMP_SUPER_U_LPC,
	PCI_DID_INTEL_CMP_PREMIUM_Y_LPC,
	PCI_DID_INTEL_CMP_PREMIUM_U_LPC,
	PCI_DID_INTEL_CMP_BASE_U_LPC,
	PCI_DID_INTEL_CMP_SUPER_Y_LPC,
	PCI_DID_INTEL_CMP_H_LPC_HM470,
	PCI_DID_INTEL_CMP_H_LPC_WM490,
	PCI_DID_INTEL_CMP_H_LPC_QM480,
	PCI_DID_INTEL_CMP_H_LPC_W480,
	PCI_DID_INTEL_CMP_H_LPC_H470,
	PCI_DID_INTEL_CMP_H_LPC_Z490,
	PCI_DID_INTEL_CMP_H_LPC_Q470,
	PCI_DID_INTEL_TGP_ESPI_0,
	PCI_DID_INTEL_TGP_SUPER_U_ESPI,
	PCI_DID_INTEL_TGP_PREMIUM_U_ESPI,
	PCI_DID_INTEL_TGP_BASE_U_ESPI,
	PCI_DID_INTEL_TGP_ESPI_1,
	PCI_DID_INTEL_TGP_ESPI_2,
	PCI_DID_INTEL_TGP_SUPER_Y_ESPI,
	PCI_DID_INTEL_TGP_PREMIUM_Y_ESPI,
	PCI_DID_INTEL_TGP_ESPI_3,
	PCI_DID_INTEL_TGP_ESPI_4,
	PCI_DID_INTEL_TGP_ESPI_5,
	PCI_DID_INTEL_TGP_ESPI_6,
	PCI_DID_INTEL_TGP_ESPI_7,
	PCI_DID_INTEL_TGP_ESPI_8,
	PCI_DID_INTEL_TGP_ESPI_9,
	PCI_DID_INTEL_TGP_ESPI_10,
	PCI_DID_INTEL_TGP_ESPI_11,
	PCI_DID_INTEL_TGP_ESPI_12,
	PCI_DID_INTEL_TGP_ESPI_13,
	PCI_DID_INTEL_TGP_ESPI_14,
	PCI_DID_INTEL_TGP_ESPI_15,
	PCI_DID_INTEL_TGP_ESPI_16,
	PCI_DID_INTEL_TGP_ESPI_17,
	PCI_DID_INTEL_TGP_ESPI_18,
	PCI_DID_INTEL_TGP_ESPI_19,
	PCI_DID_INTEL_TGP_ESPI_20,
	PCI_DID_INTEL_TGP_ESPI_21,
	PCI_DID_INTEL_TGP_ESPI_22,
	PCI_DID_INTEL_TGP_ESPI_23,
	PCI_DID_INTEL_TGP_ESPI_24,
	PCI_DID_INTEL_TGP_ESPI_25,
	PCI_DID_INTEL_TGP_ESPI_26,
	PCI_DID_INTEL_TGP_H_ESPI_B560,
	PCI_DID_INTEL_TGP_H_ESPI_H510,
	PCI_DID_INTEL_TGP_H_ESPI_H570,
	PCI_DID_INTEL_TGP_H_ESPI_Q570,
	PCI_DID_INTEL_TGP_H_ESPI_W580,
	PCI_DID_INTEL_TGP_H_ESPI_Z590,
	PCI_DID_INTEL_TGP_H_ESPI_HM570,
	PCI_DID_INTEL_TGP_H_ESPI_QM580,
	PCI_DID_INTEL_TGP_H_ESPI_WM590,
	PCI_DID_INTEL_MCC_ESPI_0,
	PCI_DID_INTEL_MCC_ESPI_1,
	PCI_DID_INTEL_MCC_BASE_ESPI,
	PCI_DID_INTEL_MCC_PREMIUM_ESPI,
	PCI_DID_INTEL_MCC_SUPER_ESPI,
	PCI_DID_INTEL_MCC_ESPI_2,
	PCI_DID_INTEL_MCC_ESPI_3,
	PCI_DID_INTEL_MCC_ESPI_4,
	PCI_DID_INTEL_JSP_SUPER_ESPI,
	PCI_DID_INTEL_ADP_P_ESPI_0,
	PCI_DID_INTEL_ADP_P_ESPI_1,
	PCI_DID_INTEL_ADP_P_ESPI_2,
	PCI_DID_INTEL_ADP_P_ESPI_3,
	PCI_DID_INTEL_ADP_P_ESPI_4,
	PCI_DID_INTEL_ADP_P_ESPI_5,
	PCI_DID_INTEL_ADP_P_ESPI_6,
	PCI_DID_INTEL_ADP_P_ESPI_7,
	PCI_DID_INTEL_ADP_P_ESPI_8,
	PCI_DID_INTEL_ADP_P_ESPI_9,
	PCI_DID_INTEL_ADP_P_ESPI_10,
	PCI_DID_INTEL_ADP_P_ESPI_11,
	PCI_DID_INTEL_ADP_P_ESPI_12,
	PCI_DID_INTEL_ADP_P_ESPI_13,
	PCI_DID_INTEL_ADP_P_ESPI_14,
	PCI_DID_INTEL_ADP_P_ESPI_15,
	PCI_DID_INTEL_ADP_P_ESPI_16,
	PCI_DID_INTEL_ADP_P_ESPI_17,
	PCI_DID_INTEL_ADP_P_ESPI_18,
	PCI_DID_INTEL_ADP_P_ESPI_19,
	PCI_DID_INTEL_ADP_P_ESPI_20,
	PCI_DID_INTEL_ADP_P_ESPI_21,
	PCI_DID_INTEL_ADP_P_ESPI_22,
	PCI_DID_INTEL_ADP_P_ESPI_23,
	PCI_DID_INTEL_ADP_P_ESPI_24,
	PCI_DID_INTEL_ADP_P_ESPI_25,
	PCI_DID_INTEL_ADP_P_ESPI_26,
	PCI_DID_INTEL_ADP_P_ESPI_27,
	PCI_DID_INTEL_ADP_P_ESPI_28,
	PCI_DID_INTEL_ADP_P_ESPI_29,
	PCI_DID_INTEL_ADP_P_ESPI_30,
	PCI_DID_INTEL_ADP_P_ESPI_31,
	PCI_DID_INTEL_ADP_S_ESPI_0,
	PCI_DID_INTEL_ADP_S_ESPI_1,
	PCI_DID_INTEL_ADP_S_ESPI_2,
	PCI_DID_INTEL_ADP_S_ESPI_3,
	PCI_DID_INTEL_ADP_S_ESPI_4,
	PCI_DID_INTEL_ADP_S_ESPI_5,
	PCI_DID_INTEL_ADP_S_ESPI_6,
	PCI_DID_INTEL_ADP_S_ESPI_7,
	PCI_DID_INTEL_ADP_S_ESPI_8,
	PCI_DID_INTEL_ADP_S_ESPI_9,
	PCI_DID_INTEL_ADP_S_ESPI_10,
	PCI_DID_INTEL_ADP_S_ESPI_11,
	PCI_DID_INTEL_ADP_S_ESPI_12,
	PCI_DID_INTEL_ADP_S_ESPI_13,
	PCI_DID_INTEL_ADP_S_ESPI_14,
	PCI_DID_INTEL_ADP_S_ESPI_15,
	PCI_DID_INTEL_ADP_S_ESPI_16,
	PCI_DID_INTEL_ADP_S_ESPI_17,
	PCI_DID_INTEL_ADP_S_ESPI_18,
	PCI_DID_INTEL_ADP_S_ESPI_19,
	PCI_DID_INTEL_ADP_S_ESPI_20,
	PCI_DID_INTEL_ADP_S_ESPI_21,
	PCI_DID_INTEL_ADP_S_ESPI_22,
	PCI_DID_INTEL_ADP_S_ESPI_23,
	PCI_DID_INTEL_ADP_S_ESPI_24,
	PCI_DID_INTEL_ADP_S_ESPI_25,
	PCI_DID_INTEL_ADP_S_ESPI_26,
	PCI_DID_INTEL_ADP_S_ESPI_27,
	PCI_DID_INTEL_ADP_S_ESPI_28,
	PCI_DID_INTEL_ADP_S_ESPI_29,
	PCI_DID_INTEL_ADP_S_ESPI_30,
	PCI_DID_INTEL_ADP_S_ESPI_31,
	PCI_DID_INTEL_ADP_M_N_ESPI_0,
	PCI_DID_INTEL_ADP_M_N_ESPI_1,
	PCI_DID_INTEL_ADP_M_N_ESPI_2,
	PCI_DID_INTEL_ADP_M_N_ESPI_3,
	PCI_DID_INTEL_ADP_M_N_ESPI_4,
	PCI_DID_INTEL_ADP_M_N_ESPI_5,
	PCI_DID_INTEL_ADP_M_N_ESPI_7,
	PCI_DID_INTEL_ADP_M_N_ESPI_8,
	PCI_DID_INTEL_ADP_M_N_ESPI_9,
	PCI_DID_INTEL_ADP_M_N_ESPI_10,
	PCI_DID_INTEL_ADP_M_N_ESPI_11,
	PCI_DID_INTEL_ADP_M_N_ESPI_12,
	PCI_DID_INTEL_ADP_M_N_ESPI_13,
	PCI_DID_INTEL_ADP_M_N_ESPI_14,
	PCI_DID_INTEL_ADP_M_N_ESPI_15,
	PCI_DID_INTEL_ADP_M_N_ESPI_16,
	PCI_DID_INTEL_ADP_M_N_ESPI_17,
	PCI_DID_INTEL_ADP_M_N_ESPI_18,
	PCI_DID_INTEL_ADP_M_N_ESPI_19,
	PCI_DID_INTEL_ADP_M_N_ESPI_20,
	PCI_DID_INTEL_ADP_M_N_ESPI_21,
	PCI_DID_INTEL_ADP_M_N_ESPI_22,
	PCI_DID_INTEL_ADP_M_N_ESPI_23,
	PCI_DID_INTEL_ADP_M_N_ESPI_24,
	PCI_DID_INTEL_ADP_M_N_ESPI_25,
	PCI_DID_INTEL_ADP_M_N_ESPI_26,
	PCI_DID_INTEL_ADP_M_N_ESPI_27,
	PCI_DID_INTEL_ADP_M_N_ESPI_28,
	PCI_DID_INTEL_ADP_M_N_ESPI_29,
	PCI_DID_INTEL_ADP_M_N_ESPI_30,
	PCI_DID_INTEL_ADP_M_N_ESPI_31,
	PCI_DID_INTEL_SPR_ESPI_1,
	0
};

static const struct pci_driver pch_lpc __pci_driver = {
	.ops = &device_ops,
	.vendor = PCI_VID_INTEL,
	.devices = pci_device_ids,
};
