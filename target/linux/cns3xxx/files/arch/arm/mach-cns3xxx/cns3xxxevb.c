/*
 * Cavium Networks CNS3411 Evaluation Board version 1.x
 *
 *  Copyright (c) 2008 Cavium Networks
 *  Copyright (C) 2008 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, Version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include "cns3xxx.h"
#include "pm.h"
#include "core.h"
#include "devices.h"

/*
 * NOR
 */
static struct mtd_partition cns3xxxevb_nor_partitions[] = {
	{
		.name		= "UBoot",
		.offset		= 0,
		.size		= 0x00040000,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "Manuinfo",
		.size		= 0x00020000,
		.offset =	MTDPART_OFS_APPEND,
	}, {
		.name		= "Calibration",
		.size		= 0x00020000,
		.offset =	MTDPART_OFS_APPEND,
	}, {
		.name		= "Kernel",
		.size		= 0x00900000,
		.offset =	MTDPART_OFS_APPEND,
	}, {
		.name		= "File System",
		.size		= 0x01460000,
		.offset =	MTDPART_OFS_APPEND,
	}, {
		.name		= "Configure",
		.size		= 0x00200000,
		.offset =	MTDPART_OFS_APPEND,
	}, {
		.name =		"UBootEnv",	/* bottom 8KiB for env vars */
		.size =		MTDPART_SIZ_FULL,
		.offset =	MTDPART_OFS_APPEND,
	}
};

static struct physmap_flash_data cns3xxxevb_nor_pdata = {
	.width = 2,
	.parts = cns3xxxevb_nor_partitions,
	.nr_parts = ARRAY_SIZE(cns3xxxevb_nor_partitions),
};

static struct resource cns3xxxevb_nor_res = {
	.start = CNS3XXX_FLASH_BASE,
	.end = CNS3XXX_FLASH_BASE + SZ_128M - 1,
	.flags = IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
};

static struct platform_device cns3xxxevb_nor_pdev = {
	.name = "physmap-flash",
	.id = 0,
	.resource = &cns3xxxevb_nor_res,
	.num_resources = 1,
	.dev = {
		.platform_data = &cns3xxxevb_nor_pdata,
	},
};

/*
 * SPI
 */
static struct mtd_partition cns3xxxevb_spi_partitions[] = {
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= 0x4000,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "BootpImage",
		.offset		= 0x4000,
		.size		= MTDPART_SIZ_FULL,
	}
};

static struct flash_platform_data cns3xxxevb_spi_pdata = {
	.parts = cns3xxxevb_spi_partitions,
	.nr_parts = ARRAY_SIZE(cns3xxxevb_spi_partitions),
};

static struct spi_board_info __initdata cns3xxxevb_spi_devices[] = {
	{
		.modalias = "m25p80",
		.platform_data = &cns3xxxevb_spi_pdata,
		.max_speed_hz = 25000000,
		.bus_num = 1,
		.chip_select = 0,
	},
};

/*
 * Initialization
 */
static void __init cns3xxxevb_init(void)
{
	cns3xxx_usb_init();
	cns3xxx_ahci_init();
	cns3xxx_sdhci_init();

	cns3xxx_i2c_init();
	cns3xxx_i2s_init();
	cns3xxx_spi_init();
	cns3xxx_watchdog_init();
	cns3xxx_uart_init(1);

	platform_device_register(&cns3xxxevb_nor_pdev);
	spi_register_board_info(cns3xxxevb_spi_devices, ARRAY_SIZE(cns3xxxevb_spi_devices));

	pm_power_off = cns3xxx_power_off;
}

MACHINE_START(CNS3XXX, "Cavium Networks ARM11 MPCore")
	.map_io		= cns3xxx_map_io,
	.init_irq	= cns3xxx_init_irq,
	.init_time	= cns3xxx_timer_init,
	.init_machine	= cns3xxxevb_init,
	.init_late      = cns3xxx_pcie_init_late,
	.restart	= cns3xxx_restart,
MACHINE_END
