/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Arch specific extensions to struct device
 */
#ifndef ASMARM_DEVICE_H
#define ASMARM_DEVICE_H

struct dev_archdata {
#ifdef CONFIG_DMABOUNCE
	struct dmabounce_device_info *dmabounce;
#endif
	unsigned int dma_coherent:1;
	unsigned int dma_ops_setup:1;
};

struct omap_device;

struct pdev_archdata {
#ifdef CONFIG_ARCH_OMAP
	struct omap_device *od;
#endif
};

#endif
