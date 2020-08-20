// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2012 Samsung Electronics Co., Ltd.
// Author: Inki Dae <inki.dae@samsung.com>
// Author: Andrzej Hajda <a.hajda@samsung.com>

#include <linux/dma-iommu.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/platform_device.h>

#include <drm/drm_print.h>
#include <drm/exynos_drm.h>

#include "exynos_drm_drv.h"


#define EXYNOS_DEV_ADDR_START	0x20000000
#define EXYNOS_DEV_ADDR_SIZE	0x40000000

/*
 * drm_iommu_attach_device- attach device to iommu mapping
 *
 * @drm_dev: DRM device
 * @subdrv_dev: device to be attach
 *
 * This function should be called by sub drivers to attach it to iommu
 * mapping.
 */
static int drm_iommu_attach_device(struct drm_device *drm_dev,
				struct device *subdrv_dev)
{
	struct exynos_drm_private *priv = drm_dev->dev_private;
	int ret = 0;

	if (get_dma_ops(priv->dma_dev) != get_dma_ops(subdrv_dev)) {
		DRM_DEV_ERROR(subdrv_dev, "Device %s lacks support for IOMMU\n",
			  dev_name(subdrv_dev));
		return -EINVAL;
	}

	dma_set_max_seg_size(subdrv_dev, DMA_BIT_MASK(32));
	ret = iommu_attach_device(priv->mapping, subdrv_dev);
	return ret;
}

/*
 * drm_iommu_detach_device -detach device address space mapping from device
 *
 * @drm_dev: DRM device
 * @subdrv_dev: device to be detached
 *
 * This function should be called by sub drivers to detach it from iommu
 * mapping
 */
static void drm_iommu_detach_device(struct drm_device *drm_dev,
				struct device *subdrv_dev)
{
	struct exynos_drm_private *priv = drm_dev->dev_private;

	iommu_detach_device(priv->mapping, subdrv_dev);
}

int exynos_drm_register_dma(struct drm_device *drm, struct device *dev)
{
	struct exynos_drm_private *priv = drm->dev_private;

	if (!priv->dma_dev) {
		priv->dma_dev = dev;
		DRM_INFO("Exynos DRM: using %s device for DMA mapping operations\n",
			 dev_name(dev));
	}

	if (!IS_ENABLED(CONFIG_EXYNOS_IOMMU))
		return 0;

	if (!priv->mapping) {
		void *mapping = iommu_get_domain_for_dev(priv->dma_dev);

		if (IS_ERR(mapping))
			return PTR_ERR(mapping);
		priv->mapping = mapping;
	}

	return drm_iommu_attach_device(drm, dev);
}

void exynos_drm_unregister_dma(struct drm_device *drm, struct device *dev)
{
	if (IS_ENABLED(CONFIG_EXYNOS_IOMMU))
		drm_iommu_detach_device(drm, dev);
}

void exynos_drm_cleanup_dma(struct drm_device *drm)
{
	struct exynos_drm_private *priv = drm->dev_private;

	if (!IS_ENABLED(CONFIG_EXYNOS_IOMMU))
		return;

	priv->dma_dev = NULL;
}
