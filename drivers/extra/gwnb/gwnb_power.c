// SPDX-License-Identifier: GPL-2.0
/*
 * Power supply driver for Great Wall ft notebooks
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (C) 2012 Intel, Inc.
 * Copyright (C) 2013 Intel, Inc.
 * Copyright (c) 2019 Great Wall
 * Author: Mike Lockwood <lockwood@android.com>
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/acpi.h>
#include <linux/delay.h>

struct gw_nb_battery_data
{
	void __iomem *reg_base;
	int irq;
	spinlock_t lock;

	struct power_supply *battery;
	struct power_supply *ac;
};

#define POWER_STATUS 0xB0
#define AC_STATUS 1U << 0
#define BAT_STATUS 1U << 1

#define BATTERY_STATUS 0x3C

#define BATTERY_PERCENT 0x21
#define BATTERY_VOLTAGE_L 0x2A
#define BATTERY_VOLTAGE_H 0x2B

#define BATTERY_TEMP_L 0x28
#define BATTERY_TEMP_H 0x29

#define BATTERY_CUR_NOW_L 0x2C
#define BATTERY_CUR_NOW_H 0x2D

#define BATTERY_CUR_AVG_L 0x2E
#define BATTERY_CUR_AVG_H 0x2F

#define BATTERY_FULL_CAP_L 0x24
#define BATTERY_FULL_CAP_H 0x25

#define BATTERY_CUR_CAP_L 0x26
#define BATTERY_CUR_CAP_H 0x27

#define BATTERY_DES_FULL_CAP_L 0x38
#define BATTERY_DES_FULL_CAP_H 0x39

#define BATTERY_DES_VOLTAGE_L 0x3A
#define BATTERY_DES_VOLTAGE_H 0x3B

#define BATTERY_SN_L 0x3E
#define BATTERY_SN_H 0x3F

#define GPIO_0_INT 42
#define GPIO_1_INT 43

#define LPC_BASE_ADDR 0x20000000

#define I8042_COMMAND_REG 0x66UL
#define I8042_DATA_REG 0x62UL

void __iomem *lpc_base;
void __iomem *gpio_iobase;

// Wait till EC I/P buffer is free
static int
EcIbFree(void)
{
	static int Status;
	do
	{
		Status = readb(lpc_base + I8042_COMMAND_REG);
	} while (Status & 2);
	return 0;
}

// Wait till EC O/P buffer is full
static int
EcObFull(void)
{
	static int Status;
	do
	{
		Status = readb(lpc_base + I8042_COMMAND_REG);
	} while (!(Status & 1));
	return 0;
}

// Send EC command
static int
EcWriteCmd(
	int Cmd)
{
	EcIbFree();
	writeb(Cmd, lpc_base + I8042_COMMAND_REG);
	return 0;
}

// Write Data from EC data port
static int
EcWriteData(
	int Data)
{
	EcIbFree();
	writeb(Data, lpc_base + I8042_DATA_REG);
	return 0;
}

// Read Data from EC data Port
static int
EcReadData(
	int *PData)
{
	//暂时修改
	*PData = readb(lpc_base + I8042_DATA_REG);
	EcObFull();
	*PData = readb(lpc_base + I8042_DATA_REG);
	return 0;
}

// Read Data from EC Memory from location pointed by Index
static int
EcReadMem(
	int Index,
	int *Data)
{
	static int Cmd;
	Cmd = 0x80;
	EcWriteCmd(Cmd);
	EcWriteData(Index);
	EcReadData(Data);
	return 0;
}

// Write Data to EC memory at location pointed by Index
//  static int
//  EcWriteMem (
//     int  Index,
//     int  Data
//    )
//  {
//    static int  Cmd;
//    Cmd = 0x81;
//    EcWriteCmd (Cmd);
//    EcWriteData (Index);
//    EcWriteData (Data);
//    return 0;
//  }

static int gw_ec_read(int offset)
{
	int tmp;
	EcReadMem(offset, &tmp);
	// printk(KERN_ERR "gw_ec_read offset:0x%X value:0x%x\n",offset,tmp);
	return tmp;
}

// static int gw_ec_write(int offset,int value)
// {
// 	printk(KERN_ERR "gw_ec_write\n");

// 	EcWriteMem(offset, value);
// 	return 0;
// }

static int goldfish_ac_get_property(struct power_supply *psy,
									enum power_supply_property psp,
									union power_supply_propval *val)
{
	int ret = 0;

	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		if (gw_ec_read(POWER_STATUS) & AC_STATUS)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int goldfish_battery_get_property(struct power_supply *psy,
										 enum power_supply_property psp,
										 union power_supply_propval *val)
{
	int ret = 0;
	int temp;
	int16_t temp16;

	switch (psp)
	{
	case POWER_SUPPLY_PROP_STATUS:

		if (gw_ec_read(POWER_STATUS) & AC_STATUS)
		{
			temp = gw_ec_read(BATTERY_STATUS);
			if (temp & BIT(5))
			{
				val->intval = POWER_SUPPLY_STATUS_FULL;
			}
			else if (!(temp & BIT(6)))
			{
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			}else{
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			}
		}
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;

		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			if (gw_ec_read(POWER_STATUS) & BAT_STATUS)
				val->intval = 1;
			else
				val->intval = 0;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = gw_ec_read(BATTERY_PERCENT);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			temp = gw_ec_read(BATTERY_VOLTAGE_L);
			temp += gw_ec_read(BATTERY_VOLTAGE_H) << 8;
			val->intval = temp * 1000;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN: /* THE design voltage... */
			temp = gw_ec_read(BATTERY_DES_VOLTAGE_L);
			temp += gw_ec_read(BATTERY_DES_VOLTAGE_H) << 8;
			val->intval = temp * 1000;
			break;
		case POWER_SUPPLY_PROP_TEMP:
			temp = gw_ec_read(BATTERY_TEMP_L);
			temp += gw_ec_read(BATTERY_TEMP_H) << 8;
			val->intval = temp - 2722;
			break;
		case POWER_SUPPLY_PROP_CHARGE_COUNTER:
			val->intval = 1;
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
		// case POWER_SUPPLY_PROP_POWER_NOW:
			temp16 = gw_ec_read(BATTERY_CUR_NOW_L);
			temp16 += gw_ec_read(BATTERY_CUR_NOW_H) << 8;
			if (temp16 < 0)
				temp16 = ~(temp16 - 1);
			val->intval = temp16 * 1000;
			break;
		case POWER_SUPPLY_PROP_CURRENT_AVG:
			temp16 = gw_ec_read(BATTERY_CUR_NOW_L);
			temp16 += gw_ec_read(BATTERY_CUR_NOW_H) << 8;
			if (temp16 < 0)
				temp16 = ~(temp16 - 1);
			val->intval = temp16 * 1000;
			break;
		case POWER_SUPPLY_PROP_CHARGE_NOW:
			temp = gw_ec_read(BATTERY_CUR_CAP_L);
			temp += gw_ec_read(BATTERY_CUR_CAP_H) << 8;
			val->intval = temp * 1000;
			break;

		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			temp = gw_ec_read(BATTERY_DES_FULL_CAP_L);
			temp += gw_ec_read(BATTERY_DES_FULL_CAP_H) << 8;
			val->intval = temp * 1000;
			break;
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL:
			temp = gw_ec_read(BATTERY_FULL_CAP_L);
			temp += gw_ec_read(BATTERY_FULL_CAP_H) << 8;
			val->intval = temp * 1000;
			break;
		case POWER_SUPPLY_PROP_CYCLE_COUNT:
			val->intval = 1;
			break;
		default:
			ret = -EINVAL;
			break;
		}

	return ret;
}

static enum power_supply_property goldfish_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	// POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
};

static enum power_supply_property goldfish_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static irqreturn_t goldfish_battery_interrupt(int irq, void *dev_id)
{
	unsigned long irq_flags;
	struct gw_nb_battery_data *data = dev_id;
	unsigned int q_event;
	printk("[%s] :%d, irq :%d\n", __func__, __LINE__, irq);

	spin_lock_irqsave(&data->lock, irq_flags);
	// //read and confirm if this is a interrupt from gpio 7.
	if (readl(gpio_iobase + 0x28) & BIT(7))
	{

		EcWriteCmd(0x84);
		EcReadData(&q_event);
		printk("[%s] :%d, recieve an SCI, Q num:0x%X\n", __func__, __LINE__, q_event);

		switch (q_event)
		{
		case 0xB3:
			power_supply_changed(data->battery);
			break;
		case 0xB4:
			power_supply_changed(data->battery);
			power_supply_changed(data->ac);
			break;
		default:
			break;
		}
		// clear sci.
		writel(BIT(7), gpio_iobase + 0x38);
	}

	spin_unlock_irqrestore(&data->lock, irq_flags);
	return IRQ_HANDLED;
}

static const struct power_supply_desc battery_desc = {
	.properties = goldfish_battery_props,
	.num_properties = ARRAY_SIZE(goldfish_battery_props),
	.get_property = goldfish_battery_get_property,
	.name = "gwnb-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
};

static const struct power_supply_desc ac_desc = {
	.properties = goldfish_ac_props,
	.num_properties = ARRAY_SIZE(goldfish_ac_props),
	.get_property = goldfish_ac_get_property,
	.name = "gwnb-ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
};

static int goldfish_battery_probe(struct platform_device *pdev)
{
	int ret;
	// struct resource *r;
	struct gw_nb_battery_data *data;
	struct power_supply_config psy_cfg = {};
	u32 temp;

	printk(KERN_ERR "GW NB Battery probe\n");

	data = devm_kzalloc(&pdev->dev, sizeof(struct gw_nb_battery_data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	spin_lock_init(&data->lock);

	lpc_base = ioremap(LPC_BASE_ADDR, 0x100);

	printk(KERN_ERR "GW NB ioremap lpc_base addr:0x%p\n", lpc_base);

	data->irq = acpi_register_gsi(NULL, GPIO_0_INT, ACPI_LEVEL_SENSITIVE,
								  ACPI_ACTIVE_HIGH);
	printk(KERN_ERR "GW NB acpi_register_gsi:%d\n", data->irq);

	ret = request_irq(data->irq, goldfish_battery_interrupt, 0, "gw-nb-power", data);
	if (ret)
		printk(KERN_ERR "GW NB Requset irq fail status:%d\n", ret);

	psy_cfg.drv_data = data;

	data->ac = power_supply_register(&pdev->dev, &ac_desc, &psy_cfg);
	if (IS_ERR(data->ac))
		return PTR_ERR(data->ac);

	data->battery =
		power_supply_register(&pdev->dev, &battery_desc, &psy_cfg);
	if (IS_ERR(data->battery))
	{
		power_supply_unregister(data->ac);
		return PTR_ERR(data->battery);
	}

	platform_set_drvdata(pdev, data);

	lpc_base = ioremap(LPC_BASE_ADDR, 0x100);
	gpio_iobase = ioremap(0x28004000, 0x100);

	// Charles set SCI gpio (GPIOA 07)
	// This should be done by BIOS not driver.
	temp = readl(gpio_iobase + 0x00);
	temp &= (~BIT(7));
	writel(temp, gpio_iobase + 0x00);

	temp = readl(gpio_iobase + 0x1C);
	temp &= (~BIT(7));
	writel(temp, gpio_iobase + 0x1C);

	temp = readl(gpio_iobase + 0x20);
	temp |= BIT(7);
	writel(temp, gpio_iobase + 0x20);

	temp = readl(gpio_iobase + 0x24);
	temp &= (~BIT(7));
	writel(temp, gpio_iobase + 0x24);

	temp = readl(gpio_iobase + 0x18);
	temp |= BIT(7);
	writel(temp, gpio_iobase + 0x18);

	EcWriteCmd(0x86);

	return 0;
}

static int goldfish_battery_remove(struct platform_device *pdev)
{
	struct gw_nb_battery_data *data = platform_get_drvdata(pdev);

	power_supply_unregister(data->battery);
	power_supply_unregister(data->ac);
	return 0;
}
int gw_nb_device_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	printk(KERN_ERR "gw_nb_device_suspend\n");
	return 0;
}
EXPORT_SYMBOL_GPL(gw_nb_device_suspend);

int gw_nb_device_resume(struct platform_device *pdev)
{
	printk(KERN_ERR "gw_nb_device_resume\n");
	EcWriteCmd(0x86);
	return 0;
}
EXPORT_SYMBOL_GPL(gw_nb_device_resume);

#ifdef CONFIG_ACPI
static const struct acpi_device_id goldfish_battery_acpi_match[] = {
	{"PHYT000C", 0},
	{"FTEC0002", 0},
	{},
};
MODULE_DEVICE_TABLE(acpi, goldfish_battery_acpi_match);
#endif

static struct platform_driver
	goldfish_battery_device = {
		.probe = goldfish_battery_probe,
		.remove = goldfish_battery_remove,
		.driver = {
			.name = "gw-nb-battery",
			.acpi_match_table = ACPI_PTR(
				goldfish_battery_acpi_match),
		},
		.suspend = gw_nb_device_suspend,
		.resume = gw_nb_device_resume,
};
module_platform_driver(goldfish_battery_device);

MODULE_AUTHOR("zhangshuzhen@greatwall.com.cn");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Power supply driver for Great Wall ft notebooks");
