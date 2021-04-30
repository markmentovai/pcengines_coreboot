/* SPDX-License-Identifier: GPL-2.0-only */

#include <console/console.h>
#include <cpu/x86/msr.h>
#include <cpu/amd/msr.h>
#include <cpu/x86/mtrr.h>
#include <cpu/amd/mtrr.h>
#include <device/device.h>
#include <cpu/x86/pae.h>
#include <cpu/x86/lapic.h>
#include <cpu/cpu.h>
#include <cpu/x86/cache.h>
#include <acpi/acpi.h>
#include <northbridge/amd/agesa/agesa_helper.h>
#include <smp/node.h>

static void model_14_init(struct device *dev)
{
	msr_t msr;
	u32 siblings;
	printk(BIOS_DEBUG, "Model 14 Init.\n");


	if (acpi_is_wakeup_s3()) {
		restore_mtrr();
	} else {
		/*
		 * All cores are initialized sequentially, so the solution for APs will be
		 * created before they start.
		 */
		x86_setup_mtrrs_with_detect();
		/*
		 * Enable ROM caching on BSP we just lost when creating MTRR solution, for
		 * faster execution
		 */
		if (boot_cpu()) {
			mtrr_use_temp_range(OPTIMAL_CACHE_ROM_BASE, OPTIMAL_CACHE_ROM_SIZE,
					    MTRR_TYPE_WRPROT);
		}
	}

	x86_mtrr_check();
	enable_cache();

	/* zero the machine check error status registers */
	mca_clear_status();

	/* Enable the local CPU APICs */
	setup_lapic();

	if (CONFIG(LOGICAL_CPUS)) {
		siblings = cpuid_ecx(0x80000008) & 0xff;

		if (siblings > 0) {
			msr = rdmsr_amd(CPU_ID_FEATURES_MSR);
			msr.lo |= 1 << 28;
			wrmsr_amd(CPU_ID_FEATURES_MSR, msr);

			msr = rdmsr_amd(CPU_ID_EXT_FEATURES_MSR);
			msr.hi |= 1 << (33 - 32);
			wrmsr_amd(CPU_ID_EXT_FEATURES_MSR, msr);
		}
		printk(BIOS_DEBUG, "siblings = %02d, ", siblings);
	}

	/* DisableCf8ExtCfg */
	msr = rdmsr(NB_CFG_MSR);
	msr.hi &= ~(1 << (46 - 32));
	wrmsr(NB_CFG_MSR, msr);

	/* Write protect SMM space with SMMLOCK. */
	msr = rdmsr(HWCR_MSR);
	msr.lo |= (1 << 0);
	wrmsr(HWCR_MSR, msr);

	display_mtrrs();
}

static struct device_operations cpu_dev_ops = {
	.init = model_14_init,
};

static const struct cpu_device_id cpu_table[] = {
	{ X86_VENDOR_AMD, 0x500f00 },   /* ON-A0 */
	{ X86_VENDOR_AMD, 0x500f01 },   /* ON-A1 */
	{ X86_VENDOR_AMD, 0x500f10 },   /* ON-B0 */
	{ X86_VENDOR_AMD, 0x500f20 },   /* ON-C0 */
	{ 0, 0 },
};

static const struct cpu_driver model_14 __cpu_driver = {
	.ops      = &cpu_dev_ops,
	.id_table = cpu_table,
};
