// SPDX-License-Identifier: GPL-2.0
/*
 * Support functions for Phytium GPIO
 *
 * Copyright (c) 2019, Phytium Corporation.
 * Written by Chen Baozi <chenbaozi@phytium.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/acpi.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include "gpio-phytium-core.h"

static struct irq_chip phytium_gpio_irqchip = {
	.name		= "phytium_gpio",
	.irq_ack	= phytium_gpio_irq_ack,
	.irq_mask	= phytium_gpio_irq_mask,
	.irq_unmask	= phytium_gpio_irq_unmask,
	.irq_set_type	= phytium_gpio_irq_set_type,
	.irq_enable	= phytium_gpio_irq_enable,
	.irq_disable	= phytium_gpio_irq_disable,
};

static const struct of_device_id phytium_gpio_of_match[] = {
	{ .compatible = "phytium,gpio", },
	{ }
};
MODULE_DEVICE_TABLE(of, phytium_gpio_of_match);

static const struct acpi_device_id phytium_gpio_acpi_match[] = {
	{ "PHYT0001", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, phytium_gpio_acpi_match);

static int phytium_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct phytium_gpio *gpio;
	struct fwnode_handle *fwnode;
	int err;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gpio->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(gpio->regs))
		return PTR_ERR(gpio->regs);

	gpio->irq = -ENXIO;
	gpio->irq = platform_get_irq(pdev, 0);
	if (gpio->irq < 0)
		dev_warn(dev, "no irq is found.\n");

	if (!device_get_child_node_count(dev))
		return -ENODEV;

	device_for_each_child_node(dev, fwnode) {
		int idx;

		if (fwnode_property_read_u32(fwnode, "reg", &idx) ||
		    idx >= MAX_NPORTS) {
			dev_err(dev, "missing/invalid port index\n");
			fwnode_handle_put(fwnode);
			return -EINVAL;
		}

		if (fwnode_property_read_u32(fwnode, "nr-gpios",
					     &gpio->ngpio[idx])) {
			dev_info(dev,
				 "failed to get number of gpios for Port%c\n",
				 idx ? 'B' : 'A');
			gpio->ngpio[idx] = NGPIO_DEFAULT;
		}
	}

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

	err = gpiochip_irqchip_add(&gpio->gc, &phytium_gpio_irqchip,
				   0, handle_bad_irq, IRQ_TYPE_NONE);
	if (err) {
		dev_info(dev, "could not add irqchip\n");
		goto err0;
	}
	gpiochip_set_chained_irqchip(&gpio->gc, &phytium_gpio_irqchip,
				     gpio->irq,
				     phytium_gpio_irq_handler);

	platform_set_drvdata(pdev, gpio);
	dev_info(dev, "Phytium GPIO controller @%pa registered\n",
		&res->start);

	return 0;

err1:
	gpiochip_remove(&gpio->gc);
err0:
	return err;
}

static int phytium_gpio_remove(struct platform_device *pdev)
{
	struct phytium_gpio *gpio = platform_get_drvdata(pdev);

	gpiochip_remove(&gpio->gc);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int phytium_gpio_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct phytium_gpio *gpio = platform_get_drvdata(pdev);
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

static int phytium_gpio_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct phytium_gpio *gpio = platform_get_drvdata(pdev);
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

static SIMPLE_DEV_PM_OPS(phytium_gpio_pm_ops, phytium_gpio_suspend,
			 phytium_gpio_resume);

static struct platform_driver phytium_gpio_driver = {
	.driver		= {
		.name	= "gpio-phytium-platform",
		.pm	= &phytium_gpio_pm_ops,
		.of_match_table = of_match_ptr(phytium_gpio_of_match),
		.acpi_match_table = ACPI_PTR(phytium_gpio_acpi_match),
	},
	.probe		= phytium_gpio_probe,
	.remove		= phytium_gpio_remove,
};

module_platform_driver(phytium_gpio_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Chen Baozi <chenbaozi@phytium.com.cn>");
MODULE_DESCRIPTION("Phytium GPIO driver");
