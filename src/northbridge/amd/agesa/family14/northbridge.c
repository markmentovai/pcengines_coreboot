/* SPDX-License-Identifier: GPL-2.0-only */

#include <console/console.h>
#include <device/pci_ops.h>
#include <acpi/acpi.h>
#include <acpi/acpigen.h>
#include <stdint.h>
#include <device/device.h>
#include <device/pci.h>
#include <device/pci_ids.h>
#include <string.h>
#include <lib.h>
#include <cpu/cpu.h>
#include <cpu/amd/msr.h>
#include <cpu/amd/mtrr.h>
#include <northbridge/amd/nb_common.h>
#include <northbridge/amd/agesa/state_machine.h>
#include <northbridge/amd/agesa/agesa_helper.h>
#include <sb_cimx.h>

#define FX_DEVS 1

static struct device *__f0_dev[FX_DEVS];
static struct device *__f1_dev[FX_DEVS];
static struct device *__f2_dev[FX_DEVS];
static struct device *__f4_dev[FX_DEVS];
static unsigned int fx_devs = 0;

static struct device *get_node_pci(u32 nodeid, u32 fn)
{
	return pcidev_on_root(DEV_CDB + nodeid, fn);
}

static void get_fx_devs(void)
{
	int i;
	for (i = 0; i < FX_DEVS; i++) {
		__f0_dev[i] = get_node_pci(i, 0);
		__f1_dev[i] = get_node_pci(i, 1);
		__f2_dev[i] = get_node_pci(i, 2);
		__f4_dev[i] = get_node_pci(i, 4);
		if (__f0_dev[i] != NULL && __f1_dev[i] != NULL)
			fx_devs = i + 1;
	}
	if (__f1_dev[0] == NULL || __f0_dev[0] == NULL || fx_devs == 0) {
		die("Cannot find 0:0x18.[0|1]\n");
	}
}

static void f1_write_config32(unsigned int reg, u32 value)
{
	int i;
	if (fx_devs == 0)
		get_fx_devs();
	for (i = 0; i < fx_devs; i++) {
		struct device *dev;
		dev = __f1_dev[i];
		if (dev && dev->enabled) {
			pci_write_config32(dev, reg, value);
		}
	}
}

static int get_dram_base_limit(u32 nodeid, resource_t *basek, resource_t *limitk)
{
	u32 temp;

	if (fx_devs == 0)
		get_fx_devs();


	temp = pci_read_config32(__f1_dev[nodeid], 0x40 + (nodeid << 3)); //[39:24] at [31:16]
	if (!(temp & 1))
		return 0; // this memory range is not enabled
	/*
	 * BKDG: {DramBase[35:24], 00_0000h} <= address[35:0] so shift left by 8 bits
	 * for physical address and the convert to KiB by shifting 10 bits left
	 */
	*basek = ((temp & 0x0fff0000)) >> (10 - 8);
	/*
	 * BKDG address[35:0] <= {DramLimit[35:24], FF_FFFFh} converted as above but
	 * ORed with 0xffff to get real limit before shifting.
	 */
	temp = pci_read_config32(__f1_dev[nodeid], 0x44 + (nodeid << 3)); //[39:24] at [31:16]
	*limitk = ((temp & 0x0fff0000) | 0xffff) >> (10 - 8);
	*limitk += 1; // round up last byte

	return 1;
}

static u32 amdfam14_nodeid(struct device *dev)
{
	return (dev->path.pci.devfn >> 3) - DEV_CDB;
}

static void northbridge_init(struct device *dev)
{
	printk(BIOS_DEBUG, "Northbridge init\n");
}

static void set_vga_enable_reg(u32 nodeid, u32 linkn)
{
	u32 val;

	val = 1 | (nodeid << 4) | (linkn << 12);
	/* it will routing (1)mmio  0xa0000:0xbffff (2) io 0x3b0:0x3bb,
	   0x3c0:0x3df */
	f1_write_config32(0xf4, val);

}

#if CONFIG_HW_MEM_HOLE_SIZEK != 0

struct hw_mem_hole_info {
	unsigned int hole_startk;
	int node_id;
};

static struct hw_mem_hole_info get_hw_mem_hole_info(void)
{
	struct hw_mem_hole_info mem_hole;

	mem_hole.hole_startk = CONFIG_HW_MEM_HOLE_SIZEK;
	mem_hole.node_id = -1;

	resource_t basek, limitk;
	u32 hole;

	if (get_dram_base_limit(0, &basek, &limitk)) {
		hole = pci_read_config32(__f1_dev[0], 0xf0);
		if (hole & 1) {	// we find the hole
			mem_hole.hole_startk = (hole & (0xff << 24)) >> 10;
			mem_hole.node_id = 0;	// record the node No with hole
		}
	}
	return mem_hole;
}
#endif

static void add_fixed_resources(struct device *dev, int index)
{
	/* Reserve everything between A segment and 1MB:
	 *
	 * 0xa0000 - 0xbffff: legacy VGA
	 * 0xc0000 - 0xfffff: option ROMs and SeaBIOS (if used)
	 */
	mmio_resource(dev, index++, 0xa0000 >> 10, (0xc0000 - 0xa0000) >> 10);
	reserved_ram_resource(dev, index++, 0xc0000 >> 10, (0x100000 - 0xc0000) >> 10);

	if (fx_devs == 0)
		get_fx_devs();

	/*
	 * Check if CC6 is enabled (bits [11:0] C6Base). If CC6 is not enabled, the base
	 * must be zero according to BKDG.
	 */
	if (pci_read_config32(__f4_dev[0], 0x12C) & 0xfff) {
		unsigned long c6basek;
		c6basek = pci_read_config32(__f4_dev[0], 0x12C) & 0xfff; // [35:24] at [11:0]
		/*
		 * Shift left by 24 bits for physical address and the convert to KiB by
		 * shifting 10 bits left. The C6Base is shifted 14 bits left thus no overflow.
		 */
		c6basek = c6basek << (24 - 10);
		mmio_resource(dev, index++, c6basek, 16*1024);
	}
}

static void nb_read_resources(struct device *dev)
{
	printk(BIOS_DEBUG, "\nFam14h - %s\n", __func__);

	/*
	 * This MMCONF resource must be reserved in the PCI domain.
	 * It is not honored by the coreboot resource allocator if it is in
	 * the CPU_CLUSTER.
	 */
	mmconf_resource(dev, MMIO_CONF_BASE);

	add_fixed_resources(dev, 0);
}

#if CONFIG(CONSOLE_VGA_MULTI)
extern struct device *vga_pri;	// the primary vga device, defined in device.c
#endif

static void create_vga_resource(struct device *dev, unsigned int nodeid)
{
	struct bus *link;

	printk(BIOS_DEBUG, "\nFam14h - %s\n", __func__);

	/* find out which link the VGA card is connected,
	 * we only deal with the 'first' vga card */
	for (link = dev->link_list; link; link = link->next) {
		if (link->bridge_ctrl & PCI_BRIDGE_CTL_VGA) {
#if CONFIG(CONSOLE_VGA_MULTI)
			printk(BIOS_DEBUG,
				"VGA: vga_pri bus num = %d bus range [%d,%d]\n",
				 vga_pri->bus->secondary, link->secondary,
				 link->subordinate);
			/* We need to make sure the vga_pri is under the link */
			if ((vga_pri->bus->secondary >= link->secondary) &&
			    (vga_pri->bus->secondary <= link->subordinate))
#endif
				break;
		}
	}

	/* no VGA card installed */
	if (link == NULL)
		return;

	printk(BIOS_DEBUG, "VGA: %s (aka node %d) link %d has VGA device\n",
		dev_path(dev), nodeid, link->link_num);
	set_vga_enable_reg(nodeid, link->link_num);
}

static void nb_set_resources(struct device *dev)
{
	unsigned int nodeid;

	printk(BIOS_DEBUG, "\nFam14h - %s\n", __func__);

	/* Find the nodeid */
	nodeid = amdfam14_nodeid(dev);

	create_vga_resource(dev, nodeid);

	pci_dev_set_resources(dev);
}

/* Domain/Root Complex related code */

static void domain_read_resources(struct device *dev)
{
	printk(BIOS_DEBUG, "\nFam14h - %s\n", __func__);
	printk(BIOS_DEBUG, "  amsr - incoming dev = %p\n", dev);

	unsigned long mmio_basek;
	resource_t basek, limitk, sizek;
	int idx;
#if CONFIG_HW_MEM_HOLE_SIZEK != 0
	struct hw_mem_hole_info mem_hole;
	u32 reset_memhole = 1;
#endif

	pci_domain_read_resources(dev);

	/* TOP_MEM MSR is our boundary between DRAM and MMIO under 4G */
	mmio_basek = amd_topmem() >> 10;

	if (fx_devs == 0)
		get_fx_devs();

#if CONFIG_HW_MEM_HOLE_SIZEK != 0
/* if the hw mem hole is already set in raminit stage, here we will compare
 * mmio_basek and hole_basek. if mmio_basek is bigger that hole_basek and will
 * use hole_basek as mmio_basek and we don't need to reset hole.
 * otherwise We reset the hole to the mmio_basek
 */

	mem_hole = get_hw_mem_hole_info();

	// Use hole_basek as mmio_basek, and we don't need to reset hole anymore
	if ((mem_hole.node_id != -1) && (mmio_basek > mem_hole.hole_startk)) {
		mmio_basek = mem_hole.hole_startk;
		reset_memhole = 0;
	}
#endif

	idx = 0x10;

	if (get_dram_base_limit(0, &basek, &limitk)) {
		sizek = limitk - basek;

		printk(BIOS_DEBUG, "adsr: basek = %llx, limitk = %llx, sizek = %llx.\n",
				   basek, limitk, sizek);

		/* see if we need a hole from 0xa0000 to 0xfffff */
		if ((basek < (0xa0000 >> 10) && (sizek > (0x100000 >> 10)))) {
			ram_resource(dev, idx, basek, (0xa0000 >> 10) - basek);
			idx += 0x10;
			basek = 0x100000 >> 10;
			sizek = limitk - basek;
		}

		printk(BIOS_DEBUG, "adsr: mmio_basek=%08lx, basek=%08llx, limitk=%08llx\n",
				   mmio_basek, basek, limitk);

		/* split the region to accommodate pci memory space */
		if ((basek < 4 * 1024 * 1024) && (limitk > mmio_basek)) {
			if (basek <= mmio_basek) {
				unsigned int pre_sizek;
				pre_sizek = mmio_basek - basek;
				if (pre_sizek > 0) {
					ram_resource(dev, idx, basek,
						     pre_sizek);
					idx += 0x10;
					sizek -= pre_sizek;
				}
				basek = mmio_basek;
			}
			if ((basek + sizek) <= 4 * 1024 * 1024) {
				sizek = 0;
			} else {
				uint64_t topmem2 = amd_topmem2();
				basek = 4 * 1024 * 1024;
				sizek = topmem2 / 1024 - basek;
			}
		}

		ram_resource(dev, idx, basek, sizek);
		idx += 0x10;
		printk(BIOS_DEBUG, "%d: mmio_basek=%08lx, basek=%08llx, limitk=%08llx\n", 0,
				   mmio_basek, basek, limitk);
	}
	printk(BIOS_DEBUG, "  adsr - mmio_basek = %lx.\n", mmio_basek);

	add_uma_resource_below_tolm(dev, 7);
}

static const char *domain_acpi_name(const struct device *dev)
{
	if (dev->path.type == DEVICE_PATH_DOMAIN)
		return "PCI0";

	return NULL;
}

/* Bus related code */

static void cpu_bus_scan(struct device *dev)
{
	struct bus *cpu_bus = dev->link_list;
	struct device *cpu;
	int apic_id, cores_found;

	/* There is only one node for fam14, but there may be multiple cores. */
	cpu = pcidev_on_root(0x18, 0);
	if (!cpu)
		printk(BIOS_ERR, "ERROR: %02x:%02x.0 not found", 0, 0x18);

	cores_found = (pci_read_config32(pcidev_on_root(0x18, 0x3),
					0xe8) >> 12) & 3;
	printk(BIOS_DEBUG, "  AP siblings=%d\n", cores_found);

	for (apic_id = 0; apic_id <= cores_found; apic_id++) {
		cpu = add_cpu_device(cpu_bus, apic_id, 1);
		if (cpu)
			amd_cpu_topology(cpu, 0, apic_id);
	}
}

static void cpu_bus_init(struct device *dev)
{
	initialize_cpus(dev->link_list);
}

/* North Bridge Structures */

static void northbridge_fill_ssdt_generator(const struct device *device)
{
	msr_t msr;
	char pscope[] = "\\_SB.PCI0";

	acpigen_write_scope(pscope);
	msr = rdmsr(TOP_MEM);
	acpigen_write_name_dword("TOM1", msr.lo);
	msr = rdmsr(TOP_MEM2);
	/*
	 * Since XP only implements parts of ACPI 2.0, we can't use a qword
	 * here.
	 * See http://www.acpi.info/presentations/S01USMOBS169_OS%2520new.ppt
	 * slide 22ff.
	 * Shift value right by 20 bit to make it fit into 32bit,
	 * giving us 1MB granularity and a limit of almost 4Exabyte of memory.
	 */
	acpigen_write_name_dword("TOM2", (msr.hi << 12) | msr.lo >> 20);
	acpigen_pop_len();
}

static unsigned long acpi_fill_hest(acpi_hest_t *hest)
{
	void *addr, *current;

	/* Skip the HEST header. */
	current = (void *)(hest + 1);

	addr = agesawrapper_getlateinitptr(PICK_WHEA_MCE);
	if (addr != NULL)
		current += acpi_create_hest_error_source(hest, current, 0,
				addr + 2, *(UINT16 *)addr - 2);

	addr = agesawrapper_getlateinitptr(PICK_WHEA_CMC);
	if (addr != NULL)
		current += acpi_create_hest_error_source(hest, current, 1,
				addr + 2, *(UINT16 *)addr - 2);

	return (unsigned long)current;
}

static void patch_ssdt_processor_scope(acpi_header_t *ssdt)
{
	unsigned int len = ssdt->length - sizeof(acpi_header_t);
	unsigned int i;

	for (i = sizeof(acpi_header_t); i < len; i++) {
		/* Search for _PR_ scope and replace it with _SB_ */
		if (*(uint32_t *)((unsigned long)ssdt + i) == 0x5f52505f)
			*(uint32_t *)((unsigned long)ssdt + i) = 0x5f42535f;
	}
	/* Recalculate checksum */
	ssdt->checksum = 0;
	ssdt->checksum = acpi_checksum((void *)ssdt, ssdt->length);
}

static unsigned long agesa_write_acpi_tables(const struct device *device,
					     unsigned long current,
					     acpi_rsdp_t *rsdp)
{
	acpi_srat_t *srat;
	acpi_slit_t *slit;
	acpi_header_t *ssdt;
	acpi_header_t *alib;
	acpi_hest_t *hest;

	/* HEST */
	current = ALIGN(current, 8);
	hest = (acpi_hest_t *)current;
	acpi_write_hest((void *)current, acpi_fill_hest);
	acpi_add_table(rsdp, (void *)current);
	current += ((acpi_header_t *)current)->length;

	/* SRAT */
	current = ALIGN(current, 8);
	printk(BIOS_DEBUG, "ACPI:  * SRAT at %lx\n", current);
	srat = (acpi_srat_t *) agesawrapper_getlateinitptr (PICK_SRAT);
	if (srat != NULL) {
		memcpy((void *)current, srat, srat->header.length);
		srat = (acpi_srat_t *) current;
		current += srat->header.length;
		acpi_add_table(rsdp, srat);
	}
	else {
		printk(BIOS_DEBUG, "  AGESA SRAT table NULL. Skipping.\n");
	}

	/* SLIT */
	current = ALIGN(current, 8);
	printk(BIOS_DEBUG, "ACPI:  * SLIT at %lx\n", current);
	slit = (acpi_slit_t *) agesawrapper_getlateinitptr (PICK_SLIT);
	if (slit != NULL) {
		memcpy((void *)current, slit, slit->header.length);
		slit = (acpi_slit_t *) current;
		current += slit->header.length;
		acpi_add_table(rsdp, slit);
	}
	else {
		printk(BIOS_DEBUG, "  AGESA SLIT table NULL. Skipping.\n");
	}

	/* SSDT */
	current = ALIGN(current, 16);
	printk(BIOS_DEBUG, "ACPI:  * AGESA ALIB SSDT at %lx\n", current);
	alib = (acpi_header_t *)agesawrapper_getlateinitptr (PICK_ALIB);
	if (alib != NULL) {
		memcpy((void *)current, alib, alib->length);
		alib = (acpi_header_t *) current;
		current += alib->length;
		acpi_add_table(rsdp, (void *)alib);
	} else {
		printk(BIOS_DEBUG, "	AGESA ALIB SSDT table NULL. Skipping.\n");
	}

	/* The DSDT needs additional work for the AGESA SSDT Pstate table */
	/* Keep the comment for a while. */
	current = ALIGN(current, 16);
	printk(BIOS_DEBUG, "ACPI:  * AGESA SSDT Pstate at %lx\n", current);
	ssdt = (acpi_header_t *)agesawrapper_getlateinitptr (PICK_PSTATE);
	if (ssdt != NULL) {
		hexdump(ssdt, ssdt->length);
		patch_ssdt_processor_scope(ssdt);
		hexdump(ssdt, ssdt->length);
		memcpy((void *)current, ssdt, ssdt->length);
		ssdt = (acpi_header_t *) current;
		current += ssdt->length;
		acpi_add_table(rsdp,ssdt);
	} else {
		printk(BIOS_DEBUG, "  AGESA SSDT Pstate table NULL. Skipping.\n");
	}

	return current;
}

static struct device_operations northbridge_operations = {
	.read_resources    = nb_read_resources,
	.set_resources     = nb_set_resources,
	.enable_resources  = pci_dev_enable_resources,
	.init              = northbridge_init,
	.ops_pci           = &pci_dev_ops_pci,
	.acpi_fill_ssdt    = northbridge_fill_ssdt_generator,
	.write_acpi_tables = agesa_write_acpi_tables,
};

static const struct pci_driver northbridge_driver __pci_driver = {
	.ops = &northbridge_operations,
	.vendor = PCI_VENDOR_ID_AMD,
	.device = 0x1510,
};

struct chip_operations northbridge_amd_agesa_family14_ops = {
	CHIP_NAME("AMD Family 14h Northbridge")
	.enable_dev = 0,
};

/* Root Complex Structures */

static struct device_operations pci_domain_ops = {
	.read_resources = domain_read_resources,
	.set_resources	= pci_domain_set_resources,
	.scan_bus       = pci_domain_scan_bus,
	.acpi_name      = domain_acpi_name,
};

static struct device_operations cpu_bus_ops = {
	.read_resources = noop_read_resources,
	.set_resources = noop_set_resources,
	.init = cpu_bus_init,
	.scan_bus = cpu_bus_scan,
};

static void root_complex_enable_dev(struct device *dev)
{
	/* Set the operations if it is a special bus type */
	if (dev->path.type == DEVICE_PATH_DOMAIN) {
		dev->ops = &pci_domain_ops;
	} else if (dev->path.type == DEVICE_PATH_CPU_CLUSTER) {
		dev->ops = &cpu_bus_ops;
	}
}

struct chip_operations northbridge_amd_agesa_family14_root_complex_ops = {
	CHIP_NAME("AMD Family 14h Root Complex")
	.enable_dev = root_complex_enable_dev,
};

/********************************************************************
* Change the vendor / device IDs to match the generic VBIOS header.
********************************************************************/
u32 map_oprom_vendev(u32 vendev)
{
	u32 new_vendev = vendev;

	switch (vendev) {
	case 0x10029809:
	case 0x10029808:
	case 0x10029807:
	case 0x10029806:
	case 0x10029805:
	case 0x10029804:
	case 0x10029803:
		new_vendev = 0x10029802;
		break;
	}

	return new_vendev;
}
