// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung MIPI DSIM glue for Exynos SoCs.
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd
 *
 * Contacts: Tomasz Figa <t.figa@samsung.com>
 */

#include <linux/component.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>

#include <drm/bridge/samsung-dsim.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"

struct exynos_dsi {
	struct drm_encoder encoder;
	struct gpio_desc *te_gpio;
};

static void exynos_dsi_enable_irq(struct samsung_dsim *dsim)
{
	struct exynos_dsi *dsi = dsim->priv;

	if (dsi->te_gpio)
		enable_irq(gpiod_to_irq(dsi->te_gpio));
}

static void exynos_dsi_disable_irq(struct samsung_dsim *dsim)
{
	struct exynos_dsi *dsi = dsim->priv;

	if (dsi->te_gpio)
		disable_irq(gpiod_to_irq(dsi->te_gpio));
}

static const struct samsung_dsim_irq_ops samsung_dsim_exynos_host_irq = {
	.enable = exynos_dsi_enable_irq,
	.disable = exynos_dsi_disable_irq,
};

static irqreturn_t exynos_dsi_te_irq_handler(int irq, void *dev_id)
{
	struct samsung_dsim *dsim = (struct samsung_dsim *)dev_id;
	struct exynos_dsi *dsi = dsim->priv;
	struct drm_encoder *encoder = &dsi->encoder;

	if (dsim->state & DSIM_STATE_VIDOUT_AVAILABLE)
		exynos_drm_crtc_te_handler(encoder->crtc);

	return IRQ_HANDLED;
}

static int exynos_dsi_register_te_irq(struct samsung_dsim *dsim, struct device *panel)
{
	struct exynos_dsi *dsi = dsim->priv;
	int te_gpio_irq;
	int ret;

	dsi->te_gpio = devm_gpiod_get_optional(panel, "te", GPIOD_IN);
	if (!dsi->te_gpio) {
		return 0;
	} else if (IS_ERR(dsi->te_gpio)) {
		dev_err(dsim->dev, "gpio request failed with %ld\n",
			PTR_ERR(dsi->te_gpio));
		return PTR_ERR(dsi->te_gpio);
	}

	te_gpio_irq = gpiod_to_irq(dsi->te_gpio);

	ret = request_threaded_irq(te_gpio_irq, exynos_dsi_te_irq_handler, NULL,
				   IRQF_TRIGGER_RISING | IRQF_NO_AUTOEN, "TE",
				   dsim);
	if (ret) {
		dev_err(dsim->dev, "request interrupt failed with %d\n", ret);
		gpiod_put(dsi->te_gpio);
		return ret;
	}

	return 0;
}

static void exynos_dsi_unregister_te_irq(struct samsung_dsim *dsim)
{
	struct exynos_dsi *dsi = dsim->priv;

	if (dsi->te_gpio) {
		free_irq(gpiod_to_irq(dsi->te_gpio), dsi);
		gpiod_put(dsi->te_gpio);
	}
}

static int exynos_dsi_host_attach(struct samsung_dsim *dsim,
				  struct mipi_dsi_device *device)
{
	struct exynos_dsi *dsi = dsim->priv;
	struct drm_encoder *encoder = &dsi->encoder;
	struct drm_device *drm = encoder->dev;
	int ret;

	drm_bridge_attach(encoder, &dsim->bridge, NULL, 0);

	/*
	 * This is a temporary solution and should be made by more generic way.
	 *
	 * If attached panel device is for command mode one, dsi should register
	 * TE interrupt handler.
	 */
	if (!(device->mode_flags & MIPI_DSI_MODE_VIDEO)) {
		ret = exynos_dsi_register_te_irq(dsim, &device->dev);
		if (ret)
			return ret;
	}

	mutex_lock(&drm->mode_config.mutex);

	dsim->lanes = device->lanes;
	dsim->format = device->format;
	dsim->mode_flags = device->mode_flags;
	exynos_drm_crtc_get_by_type(drm, EXYNOS_DISPLAY_TYPE_LCD)->i80_mode =
			!(dsim->mode_flags & MIPI_DSI_MODE_VIDEO);

	mutex_unlock(&drm->mode_config.mutex);

	if (drm->mode_config.poll_enabled)
		drm_kms_helper_hotplug_event(drm);

	return 0;
}

static int exynos_dsi_host_detach(struct samsung_dsim *dsim,
				  struct mipi_dsi_device *device)
{
	struct exynos_dsi *dsi = dsim->priv;
	struct drm_device *drm = dsi->encoder.dev;

	if (drm->mode_config.poll_enabled)
		drm_kms_helper_hotplug_event(drm);

	exynos_dsi_unregister_te_irq(dsim);

	return 0;
}

static int exynos_dsi_register_host(struct samsung_dsim *dsim);
static void exynos_dsi_unregister_host(struct samsung_dsim *dsim);

static const struct samsung_dsim_host_ops samsung_dsim_exynos_host_ops = {
	.register_host = exynos_dsi_register_host,
	.unregister_host = exynos_dsi_unregister_host,
	.attach = exynos_dsi_host_attach,
	.detach = exynos_dsi_host_detach,
};

static int exynos_dsi_bind(struct device *dev, struct device *master, void *data)
{
	struct samsung_dsim *dsim = dev_get_drvdata(dev);
	struct exynos_dsi *dsi = dsim->priv;
	struct drm_encoder *encoder = &dsi->encoder;
	struct drm_device *drm_dev = data;
	int ret;

	drm_simple_encoder_init(drm_dev, encoder, DRM_MODE_ENCODER_TMDS);

	ret = exynos_drm_set_possible_crtcs(encoder, EXYNOS_DISPLAY_TYPE_LCD);
	if (ret < 0)
		return ret;

	return mipi_dsi_host_register(&dsim->dsi_host);
}

static void exynos_dsi_unbind(struct device *dev, struct device *master, void *data)
{
	struct samsung_dsim *dsim = dev_get_drvdata(dev);

	dsim->bridge.funcs->atomic_disable(&dsim->bridge, NULL);

	mipi_dsi_host_unregister(&dsim->dsi_host);
}

static const struct component_ops exynos_dsi_component_ops = {
	.bind	= exynos_dsi_bind,
	.unbind	= exynos_dsi_unbind,
};

static int exynos_dsi_register_host(struct samsung_dsim *dsim)
{
	struct exynos_dsi *exynos_dsi;

	exynos_dsi = devm_kzalloc(dsim->dev, sizeof(*exynos_dsi), GFP_KERNEL);
	if (!exynos_dsi)
		return -ENOMEM;

	dsim->priv = exynos_dsi;

	return component_add(dsim->dev, &exynos_dsi_component_ops);
}

static void exynos_dsi_unregister_host(struct samsung_dsim *dsim)
{
	component_del(dsim->dev, &exynos_dsi_component_ops);
}

static const struct samsung_dsim_plat_data exynos3250_dsi_pdata = {
	.hw_type = SAMSUNG_DSIM_TYPE_EXYNOS3250,
	.host_ops = &samsung_dsim_exynos_host_ops,
	.irq_ops = &samsung_dsim_exynos_host_irq,
};

static const struct samsung_dsim_plat_data exynos4210_dsi_pdata = {
	.hw_type = SAMSUNG_DSIM_TYPE_EXYNOS4210,
	.host_ops = &samsung_dsim_exynos_host_ops,
	.irq_ops = &samsung_dsim_exynos_host_irq,
};

static const struct samsung_dsim_plat_data exynos5410_dsi_pdata = {
	.hw_type = SAMSUNG_DSIM_TYPE_EXYNOS5410,
	.host_ops = &samsung_dsim_exynos_host_ops,
	.irq_ops = &samsung_dsim_exynos_host_irq,
};

static const struct samsung_dsim_plat_data exynos5422_dsi_pdata = {
	.hw_type = SAMSUNG_DSIM_TYPE_EXYNOS5422,
	.host_ops = &samsung_dsim_exynos_host_ops,
	.irq_ops = &samsung_dsim_exynos_host_irq,
};

static const struct samsung_dsim_plat_data exynos5433_dsi_pdata = {
	.hw_type = SAMSUNG_DSIM_TYPE_EXYNOS5433,
	.host_ops = &samsung_dsim_exynos_host_ops,
	.irq_ops = &samsung_dsim_exynos_host_irq,
};

static const struct of_device_id exynos_dsi_of_match[] = {
	{
		.compatible = "samsung,exynos3250-mipi-dsi",
		.data = &exynos3250_dsi_pdata,
	},
	{
		.compatible = "samsung,exynos4210-mipi-dsi",
		.data = &exynos4210_dsi_pdata,
	},
	{
		.compatible = "samsung,exynos5410-mipi-dsi",
		.data = &exynos5410_dsi_pdata,
	},
	{
		.compatible = "samsung,exynos5422-mipi-dsi",
		.data = &exynos5422_dsi_pdata,
	},
	{
		.compatible = "samsung,exynos5433-mipi-dsi",
		.data = &exynos5433_dsi_pdata,
	},
	{ /* sentinel. */ }
};
MODULE_DEVICE_TABLE(of, exynos_dsi_of_match);

struct platform_driver dsi_driver = {
	.probe = samsung_dsim_probe,
	.remove = samsung_dsim_remove,
	.driver = {
		   .name = "exynos-dsi",
		   .owner = THIS_MODULE,
		   .pm = &samsung_dsim_pm_ops,
		   .of_match_table = exynos_dsi_of_match,
	},
};

MODULE_AUTHOR("Tomasz Figa <t.figa@samsung.com>");
MODULE_AUTHOR("Andrzej Hajda <a.hajda@samsung.com>");
MODULE_DESCRIPTION("Samsung SoC MIPI DSI Master");
MODULE_LICENSE("GPL v2");
