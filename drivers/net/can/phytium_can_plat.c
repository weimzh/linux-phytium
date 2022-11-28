// SPDX-License-Identifier: GPL-2.0+
/* Platform CAN bus driver for Phytium CAN controller
 *
 * Copyright (C) 2018-2021, Phytium Technology Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/kfifo.h>
#include <linux/acpi.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/led.h>

#include "phytium_can.h"

#define DRV_NAME "phytium_can_plat"

static const struct can_bittiming_const phytium_bittiming_const = {
	.name = "phytium_can",
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 512,
	.brp_inc = 2,
};

static const struct can_bittiming_const phytium_ext_bittiming_const = {
	.name = "phytium_can_ext",
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 8192,
	.brp_inc = 2,
};

static int phytium_can_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct net_device *ndev;
	struct phytium_can_priv *priv;
	u32 tx_max;
	struct fwnode_handle *fwnode = dev_fwnode(&pdev->dev);
	int ret;

	ret = fwnode_property_read_u32(fwnode, "tx-fifo-depth", &tx_max);
	if (ret < 0) {
		dev_err(&pdev->dev, "tx-fifo-depth get error.\n");
		goto err;
	}

	ndev = alloc_candev(sizeof(struct phytium_can_priv), tx_max);
	if (!ndev) {
		ret = -ENOMEM;
		goto err;
	}

	priv = netdev_priv(ndev);
	priv->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->reg_base))
		return PTR_ERR(priv->reg_base);

	priv->can.ctrlmode_supported = CAN_CTRLMODE_BERR_REPORTING;
	priv->tx_head = 0;
	priv->tx_tail = 0;
	priv->tx_max = tx_max;
	priv->ndev = ndev;
	priv->is_kfifo_full_err = false;

	if (fwnode_property_present(fwnode, "extend_brp"))
		priv->can.bittiming_const = &phytium_ext_bittiming_const;
	else
		priv->can.bittiming_const = &phytium_bittiming_const;

	ndev->irq = platform_get_irq(pdev, 0);
	ndev->flags |= IFF_ECHO;	/* We support local echo */
	priv->irq_flags = IRQF_SHARED;

	spin_lock_init(&priv->lock);

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);

	/* Getting the CAN can_clk info */
	if (pdev->dev.of_node) {
		priv->can_clk = devm_clk_get(&pdev->dev, "phytium_can_clk");
		if (IS_ERR(priv->can_clk)) {
			dev_err(&pdev->dev, "Device clock not found.\n");
			ret = PTR_ERR(priv->can_clk);
			goto free;
		}

		ret = clk_prepare_enable(priv->can_clk);
		if (ret)
			goto free;

		priv->can.clock.freq = clk_get_rate(priv->can_clk);
	} else if (has_acpi_companion(&pdev->dev)) {
		ret = fwnode_property_read_u32(fwnode, "clock-frequency",
					       &priv->can.clock.freq);
		if (ret < 0) {
			dev_err(&pdev->dev, "clock frequency get error.\n");
			goto free;
		}
	}

	register_phytium_can(priv);

	return 0;

free:
	free_candev(ndev);
err:
	return ret;
}

static int phytium_can_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct phytium_can_priv *priv = netdev_priv(ndev);

	kfifo_free(&priv->rx_kfifo);
	unregister_candev(ndev);
	free_candev(ndev);

	return 0;
}

#ifdef CONFIG_ACPI
static const struct acpi_device_id phytium_can_acpi_match[] = {
	{ "PHYT000A", 0 },
	{}
};
MODULE_DEVICE_TABLE(acpi, phytium_can_acpi_match);
#endif

/* Match table for OF platform binding */
static const struct of_device_id phytium_can_of_ids[] = {
	{ .compatible = "phytium,can", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, phytium_can_of_ids);

static struct platform_driver phytium_can_driver = {
	.probe = phytium_can_probe,
	.remove	= phytium_can_remove,
	.driver	= {
		.name = DRV_NAME,
		.of_match_table	= phytium_can_of_ids,
		.acpi_match_table = ACPI_PTR(phytium_can_acpi_match),
	},
};

module_platform_driver(phytium_can_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cheng Quan <chengquan@phytium.com.cn>");
MODULE_DESCRIPTION("Platform CAN bus driver for Phytium CAN Controller");
