// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021, Phytium Corporation.
 */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "gpio-phytium-core.h"

static struct irq_chip phytium_gpio_pci_irqchip = {
	.name		= "phytium_gpio_pci",
	.irq_ack	= phytium_gpio_irq_ack,
	.irq_mask	= phytium_gpio_irq_mask,
	.irq_unmask	= phytium_gpio_irq_unmask,
	.irq_set_type	= phytium_gpio_irq_set_type,
	.irq_enable	= phytium_gpio_irq_enable,
	.irq_disable	= phytium_gpio_irq_disable,
};

static int phytium_gpio_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct device *dev = &pdev->dev;
	struct phytium_gpio *gpio;
	int err;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	pci_set_drvdata(pdev, gpio);

	err = pcim_enable_device(pdev);
	if (err) {
		dev_err(dev, "Failed to enable PCI device: err %d\n", err);
		goto err0;
	}

	err = pcim_iomap_regions(pdev, 1 << 0, pci_name(pdev));
	if (err) {
		dev_err(dev, "Failed to iomap PCI device: err %d\n", err);
		goto err0;
	}

	gpio->regs = pcim_iomap_table(pdev)[0];
	if (!gpio->regs) {
		dev_err(dev, "Cannot map PCI resource\n");
		err = -ENOMEM;
		goto err0;
	}

	err = pci_enable_msi(pdev);
	if (err < 0)
		goto err0;

	gpio->irq = pdev->irq;
	if (gpio->irq < 0)
		dev_warn(dev, "no irq is found.\n");

	/* There is only one group of Pins at the moment. */
	gpio->ngpio[0] = NGPIO_MAX;

	/* irq_chip support */
	raw_spin_lock_init(&gpio->lock);

	gpio->gc.base = -1;
	gpio->gc.get_direction = phytium_gpio_get_direction;
	gpio->gc.direction_input = phytium_gpio_direction_input;
	gpio->gc.direction_output = phytium_gpio_direction_output;
	gpio->gc.get = phytium_gpio_get;
	gpio->gc.set = phytium_gpio_set;
	gpio->gc.ngpio = gpio->ngpio[0] + gpio->ngpio[1];
	gpio->gc.label = dev_name(dev);
	gpio->gc.parent = dev;
	gpio->gc.owner = THIS_MODULE;

	err = gpiochip_add_data(&gpio->gc, gpio);
	if (err) {
		dev_err(dev, "failed to register gpiochip\n");
		goto err1;
	}

	err = gpiochip_irqchip_add(&gpio->gc, &phytium_gpio_pci_irqchip,
				   0, handle_bad_irq, IRQ_TYPE_NONE);
	if (err) {
		dev_info(dev, "could not add irqchip\n");
		goto err1;
	}
	gpiochip_set_chained_irqchip(&gpio->gc, &phytium_gpio_pci_irqchip,
				     gpio->irq,
				     phytium_gpio_irq_handler);

	dev_info(dev, "Phytium PCI GPIO controller @%pa registered\n",
		&gpio->regs);

	return 0;

err1:
	gpiochip_remove(&gpio->gc);
err0:
	pci_set_drvdata(pdev, NULL);
	return err;
}

static void phytium_gpio_pci_remove(struct pci_dev *pdev)
{
	struct phytium_gpio *gpio = pci_get_drvdata(pdev);

	gpiochip_remove(&gpio->gc);

	pci_set_drvdata(pdev, NULL);
}

static const struct pci_device_id phytium_gpio_pci_ids[] = {
	{ PCI_DEVICE(0x1DB7, 0xDC31) },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, phytium_gpio_pci_ids);

#ifdef CONFIG_PM_SLEEP
static int phytium_gpio_pci_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct phytium_gpio *gpio = pci_get_drvdata(pdev);
	unsigned long flags;

	raw_spin_lock_irqsave(&gpio->lock, flags);

	gpio->ctx.swporta_dr = readl(gpio->regs + GPIO_SWPORTA_DR);
	gpio->ctx.swporta_ddr = readl(gpio->regs + GPIO_SWPORTA_DDR);
	gpio->ctx.ext_porta = readl(gpio->regs + GPIO_EXT_PORTA);
	gpio->ctx.swportb_dr = readl(gpio->regs + GPIO_SWPORTB_DR);
	gpio->ctx.swportb_ddr = readl(gpio->regs + GPIO_SWPORTB_DDR);
	gpio->ctx.ext_portb = readl(gpio->regs + GPIO_EXT_PORTB);

	gpio->ctx.inten = readl(gpio->regs + GPIO_INTEN);
	gpio->ctx.intmask = readl(gpio->regs + GPIO_INTMASK);
	gpio->ctx.inttype_level = readl(gpio->regs + GPIO_INTTYPE_LEVEL);
	gpio->ctx.int_polarity = readl(gpio->regs + GPIO_INT_POLARITY);
	gpio->ctx.debounce = readl(gpio->regs + GPIO_DEBOUNCE);

	raw_spin_unlock_irqrestore(&gpio->lock, flags);

	return 0;
}

static int phytium_gpio_pci_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct phytium_gpio *gpio = pci_get_drvdata(pdev);
	unsigned long flags;

	raw_spin_lock_irqsave(&gpio->lock, flags);

	writel(gpio->ctx.swporta_dr, gpio->regs + GPIO_SWPORTA_DR);
	writel(gpio->ctx.swporta_ddr, gpio->regs + GPIO_SWPORTA_DDR);
	writel(gpio->ctx.ext_porta, gpio->regs + GPIO_EXT_PORTA);
	writel(gpio->ctx.swportb_dr, gpio->regs + GPIO_SWPORTB_DR);
	writel(gpio->ctx.swportb_ddr, gpio->regs + GPIO_SWPORTB_DDR);
	writel(gpio->ctx.ext_portb, gpio->regs + GPIO_EXT_PORTB);

	writel(gpio->ctx.inten, gpio->regs + GPIO_INTEN);
	writel(gpio->ctx.intmask, gpio->regs + GPIO_INTMASK);
	writel(gpio->ctx.inttype_level, gpio->regs + GPIO_INTTYPE_LEVEL);
	writel(gpio->ctx.int_polarity, gpio->regs + GPIO_INT_POLARITY);
	writel(gpio->ctx.debounce, gpio->regs + GPIO_DEBOUNCE);

	writel(0xffffffff, gpio->regs + GPIO_PORTA_EOI);

	raw_spin_unlock_irqrestore(&gpio->lock, flags);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(phytium_gpio_pci_pm_ops,
			 phytium_gpio_pci_suspend,
			 phytium_gpio_pci_resume);

static struct pci_driver phytium_gpio_pci_driver = {
	.name 		= "gpio-phytium-pci",
	.id_table	= phytium_gpio_pci_ids,
	.probe		= phytium_gpio_pci_probe,
	.remove		= phytium_gpio_pci_remove,
	.driver		= {
		.pm	= &phytium_gpio_pci_pm_ops,
	},
};

module_pci_driver(phytium_gpio_pci_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Cheng Quan <chengquan@phytium.com.cn>");
MODULE_DESCRIPTION("Phytium GPIO PCI Driver");
