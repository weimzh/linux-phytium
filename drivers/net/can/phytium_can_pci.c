// SPDX-License-Identifier: GPL-2.0+
/* PCI CAN bus driver for Phytium CAN controller
 *
 * Copyright (C) 2021, Phytium Technology Co., Ltd.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/device.h>
#include <linux/kfifo.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/led.h>

#include "phytium_can.h"

#define DRV_NAME	"phytium_can_pci"

#define TX_MAX		64
#define CLK_FREQ	480000000

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

static int phytium_can_pci_probe(struct pci_dev *pdev, const struct pci_device_id *pid)
{
	struct net_device *ndev;
	struct phytium_can_priv *priv;
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	ret = pcim_iomap_regions(pdev, 0x1, pci_name(pdev));
	if (ret)
		return ret;

	ndev = alloc_candev(sizeof(struct phytium_can_priv), TX_MAX);
	if (!ndev)
		return -ENOMEM;

	priv = netdev_priv(ndev);
	priv->dev = &pdev->dev;

	priv->reg_base = pcim_iomap_table(pdev)[0];

	priv->tx_head = 0;
	priv->tx_tail = 0;
	priv->tx_max = TX_MAX;
	priv->ndev = ndev;
	priv->is_kfifo_full_err = false;
	priv->can.bittiming_const = &phytium_ext_bittiming_const;

	ndev->irq = pdev->irq;
	ndev->flags |= IFF_ECHO;	/* We support local echo */
	priv->irq_flags = IRQF_SHARED;

	spin_lock_init(&priv->lock);

	pci_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);

	priv->can.clock.freq = CLK_FREQ;

	register_phytium_can(priv);

	return ret;
}

static void phytium_can_pci_remove(struct pci_dev *pdev)
{
	struct net_device *ndev = pci_get_drvdata(pdev);
	struct phytium_can_priv *priv = netdev_priv(ndev);

	kfifo_free(&priv->rx_kfifo);
	unregister_candev(ndev);
	free_candev(ndev);
}

static const struct pci_device_id phytium_pci_id_table[] = {
	{ PCI_VDEVICE(PHYTIUM, 0xdc2d) },
	{},
};
MODULE_DEVICE_TABLE(pci, phytium_pci_id_table);

static struct pci_driver phytium_can_pci_driver = {
	.name		= DRV_NAME,
	.id_table	= phytium_pci_id_table,
	.probe		= phytium_can_pci_probe,
	.remove		= phytium_can_pci_remove,
};

module_pci_driver(phytium_can_pci_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Phytium can controller driver");
MODULE_AUTHOR("Cheng Quan <chengquan@phytium.com.cn>");
