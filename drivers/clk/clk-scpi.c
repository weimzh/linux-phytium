/*
 * System Control and Power Interface (SCPI) Protocol based clock driver
 *
 * Copyright (C) 2015 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/acpi.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/scpi_protocol.h>

struct scpi_clk {
	u32 id;
	struct clk_hw hw;
	struct scpi_dvfs_info *info;
	struct scpi_ops *scpi_ops;
};

#define to_scpi_clk(clk) container_of(clk, struct scpi_clk, hw)

static struct platform_device *cpufreq_dev;

static unsigned long scpi_clk_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	struct scpi_clk *clk = to_scpi_clk(hw);

	return clk->scpi_ops->clk_get_val(clk->id);
}

static long scpi_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent_rate)
{
	/*
	 * We can't figure out what rate it will be, so just return the
	 * rate back to the caller. scpi_clk_recalc_rate() will be called
	 * after the rate is set and we'll know what rate the clock is
	 * running at then.
	 */
	return rate;
}

static int scpi_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			     unsigned long parent_rate)
{
	struct scpi_clk *clk = to_scpi_clk(hw);

	return clk->scpi_ops->clk_set_val(clk->id, rate);
}

static const struct clk_ops scpi_clk_ops = {
	.recalc_rate = scpi_clk_recalc_rate,
	.round_rate = scpi_clk_round_rate,
	.set_rate = scpi_clk_set_rate,
};

/* find closest match to given frequency in OPP table */
static long __scpi_dvfs_round_rate(struct scpi_clk *clk, unsigned long rate)
{
	int idx;
	unsigned long fmin = 0, fmax = ~0, ftmp;
	const struct scpi_opp *opp = clk->info->opps;

	for (idx = 0; idx < clk->info->count; idx++, opp++) {
		ftmp = opp->freq;
		if (ftmp >= rate) {
			if (ftmp <= fmax)
				fmax = ftmp;
			break;
		} else if (ftmp >= fmin) {
			fmin = ftmp;
		}
	}
	return fmax != ~0 ? fmax : fmin;
}

static unsigned long scpi_dvfs_recalc_rate(struct clk_hw *hw,
					   unsigned long parent_rate)
{
	struct scpi_clk *clk = to_scpi_clk(hw);
	int idx = clk->scpi_ops->dvfs_get_idx(clk->id);
	const struct scpi_opp *opp;

	if (idx < 0)
		return 0;

	opp = clk->info->opps + idx;
	return opp->freq;
}

static long scpi_dvfs_round_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long *parent_rate)
{
	struct scpi_clk *clk = to_scpi_clk(hw);

	return __scpi_dvfs_round_rate(clk, rate);
}

static int __scpi_find_dvfs_index(struct scpi_clk *clk, unsigned long rate)
{
	int idx, max_opp = clk->info->count;
	const struct scpi_opp *opp = clk->info->opps;

	for (idx = 0; idx < max_opp; idx++, opp++)
		if (opp->freq == rate)
			return idx;
	return -EINVAL;
}

static int scpi_dvfs_set_rate(struct clk_hw *hw, unsigned long rate,
			      unsigned long parent_rate)
{
	struct scpi_clk *clk = to_scpi_clk(hw);
	int ret = __scpi_find_dvfs_index(clk, rate);

	if (ret < 0)
		return ret;
	return clk->scpi_ops->dvfs_set_idx(clk->id, (u8)ret);
}

static const struct clk_ops scpi_dvfs_ops = {
	.recalc_rate = scpi_dvfs_recalc_rate,
	.round_rate = scpi_dvfs_round_rate,
	.set_rate = scpi_dvfs_set_rate,
};

static const struct of_device_id scpi_clk_match[] = {
	{ .compatible = "arm,scpi-dvfs-clocks", .data = &scpi_dvfs_ops, },
	{ .compatible = "arm,scpi-variable-clocks", .data = &scpi_clk_ops, },
	{}
};

static int
scpi_clk_ops_init(struct device *dev, const struct of_device_id *match,
		  struct scpi_clk *sclk, const char *name)
{
	struct clk_init_data init;
	unsigned long min = 0, max = 0;
	int ret;

	init.name = name;
	init.flags = 0;
	init.num_parents = 0;
	init.ops = match->data;
	sclk->hw.init = &init;
	sclk->scpi_ops = get_scpi_ops();

	if (init.ops == &scpi_dvfs_ops) {
		sclk->info = sclk->scpi_ops->dvfs_get_info(sclk->id);
		if (IS_ERR(sclk->info))
			return PTR_ERR(sclk->info);
	} else if (init.ops == &scpi_clk_ops) {
		if (sclk->scpi_ops->clk_get_range(sclk->id, &min, &max) || !max)
			return -EINVAL;
	} else {
		return -EINVAL;
	}

	ret = devm_clk_hw_register(dev, &sclk->hw);
	if (!ret && max)
		clk_hw_set_rate_range(&sclk->hw, min, max);
	return ret;
}

struct scpi_clk_data {
	struct scpi_clk **clk;
	unsigned int clk_num;
};

static struct clk_hw *
scpi_of_clk_src_get(struct of_phandle_args *clkspec, void *data)
{
	struct scpi_clk *sclk;
	struct scpi_clk_data *clk_data = data;
	unsigned int idx = clkspec->args[0], count;

	for (count = 0; count < clk_data->clk_num; count++) {
		sclk = clk_data->clk[count];
		if (idx == sclk->id)
			return &sclk->hw;
	}

	return ERR_PTR(-EINVAL);
}

static int scpi_clk_add(struct device *dev, struct device_node *np,
			const struct of_device_id *match)
{
	int idx, count, err;
	struct scpi_clk_data *clk_data;

	count = of_property_count_strings(np, "clock-output-names");
	if (count < 0) {
		dev_err(dev, "%s: invalid clock output count\n", np->name);
		return -EINVAL;
	}

	clk_data = devm_kmalloc(dev, sizeof(*clk_data), GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	clk_data->clk_num = count;
	clk_data->clk = devm_kcalloc(dev, count, sizeof(*clk_data->clk),
				     GFP_KERNEL);
	if (!clk_data->clk)
		return -ENOMEM;

	for (idx = 0; idx < count; idx++) {
		struct scpi_clk *sclk;
		const char *name;
		u32 val;

		sclk = devm_kzalloc(dev, sizeof(*sclk), GFP_KERNEL);
		if (!sclk)
			return -ENOMEM;

		if (of_property_read_string_index(np, "clock-output-names",
						  idx, &name)) {
			dev_err(dev, "invalid clock name @ %s\n", np->name);
			return -EINVAL;
		}

		if (of_property_read_u32_index(np, "clock-indices",
					       idx, &val)) {
			dev_err(dev, "invalid clock index @ %s\n", np->name);
			return -EINVAL;
		}

		sclk->id = val;

		err = scpi_clk_ops_init(dev, match, sclk, name);
		if (err) {
			dev_err(dev, "failed to register clock '%s'\n", name);
			return err;
		}

		dev_dbg(dev, "Registered clock '%s'\n", name);
		clk_data->clk[idx] = sclk;
	}

	return of_clk_add_hw_provider(np, scpi_of_clk_src_get, clk_data);
}

static int scpi_clocks_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *child, *np = dev->of_node;

	if (cpufreq_dev) {
		platform_device_unregister(cpufreq_dev);
		cpufreq_dev = NULL;
	}

	if (np) {
		for_each_available_child_of_node(np, child)
			of_clk_del_provider(np);
	}

	return 0;
}

#ifdef CONFIG_ACPI
static int scpi_clk_add_acpi(struct device *dev, struct fwnode_handle *np)
{
	int idx, count, err, ret = 0;
	const char **names;
	u32 *clk;

	count = fwnode_property_read_string_array(np, "clock-output-names", NULL, 0);
	if (count < 0) {
		dev_err(dev, "scpi-dvfs-clock: invalid clock output count\n");
		ret = -EINVAL;
		goto out;
	}

	names = kcalloc(count, sizeof(*names), GFP_KERNEL);
	if (!names) {
		ret = -ENOMEM;
		goto out;
	}
	err = fwnode_property_read_string_array(np, "clock-output-names",
						names, count);
	if (err < 0) {
		dev_err(dev, "error read clock-output-names\n");
		ret = -EINVAL;
		goto free;
	}

	clk = kcalloc(count, sizeof(*clk), GFP_KERNEL);
	if (!clk) {
		ret = -ENOMEM;
		goto free;
	}
	err = fwnode_property_read_u32_array(np, "clock-indices",
					     clk, count);
	if (err < 0) {
		dev_err(dev, "error read clock-indices\n");
		ret = -EINVAL;
		goto free2;
	}

	for (idx = 0; idx < count; idx++) {
		struct scpi_clk *sclk;
		const char *name;

		sclk = devm_kzalloc(dev, sizeof(*sclk), GFP_KERNEL);
		if (!sclk) {
			ret = -ENOMEM;
			goto free2;
		}

		name = names[idx];
		sclk->id = clk[idx];

		/* Only support scpi-dvfs-clocks right now. */
		err = scpi_clk_ops_init(dev, &scpi_clk_match[0], sclk, name);
		if (!err)
			ret = clk_hw_register_clkdev(&sclk->hw, name, NULL);
		if (ret || err) {
			dev_err(dev, "failed to register clock '%s'\n", name);
			goto free2;
		}

		dev_dbg(dev, "Registered clock '%s'\n", name);
	}

free2:
	kfree(clk);
free:
	kfree(names);
out:
	return ret;
}

static int scpi_clocks_probe_acpi(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct fwnode_handle *child, *np = dev->fwnode;
	const char *str;

	if (!get_scpi_ops())
		return -EPROBE_DEFER;

	fwnode_for_each_available_child_node(np, child) {
		ret = fwnode_property_read_string(child, "compatible", &str);
		if (strcmp(str, "arm,scpi-dvfs-clocks"))
			continue;

		ret = scpi_clk_add_acpi(dev, child);
		if (ret) {
			scpi_clocks_remove(pdev);
			fwnode_handle_put(child);
			return ret;
		}

		/* Add the virtual cpufreq device if it's DVFS clock provider */
		cpufreq_dev = platform_device_register_simple("scpi-cpufreq",
							      -1, NULL, 0);
		if (IS_ERR(cpufreq_dev))
			pr_warn("unable to register cpufreq device");
	}

	return 0;
}
#else
static int scpi_clocks_probe_acpi(struct platform_device *pdev)
{
	return 0;
}
#endif

static int scpi_clocks_probe_dt(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct device_node *child, *np = dev->of_node;
	const struct of_device_id *match;

	if (!get_scpi_ops())
		return -ENXIO;

	for_each_available_child_of_node(np, child) {
		match = of_match_node(scpi_clk_match, child);
		if (!match)
			continue;
		ret = scpi_clk_add(dev, child, match);
		if (ret) {
			scpi_clocks_remove(pdev);
			of_node_put(child);
			return ret;
		}

		if (match->data != &scpi_dvfs_ops)
			continue;
		/* Add the virtual cpufreq device if it's DVFS clock provider */
		cpufreq_dev = platform_device_register_simple("scpi-cpufreq",
							      -1, NULL, 0);
		if (IS_ERR(cpufreq_dev))
			pr_warn("unable to register cpufreq device");
	}
	return 0;
}

static int scpi_clocks_probe(struct platform_device *pdev)
{
	int ret = 0;

	if (pdev->dev.of_node)
		ret = scpi_clocks_probe_dt(pdev);
	else if (has_acpi_companion(&pdev->dev))
		ret = scpi_clocks_probe_acpi(pdev);

	return ret;
}

#ifdef CONFIG_ACPI
static const struct acpi_device_id scpi_clocks_acpi_match[] = {
	{ "PHYT8001", 0 },
	{ "FTCK0001", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, scpi_clocks_acpi_match);
#endif

static const struct of_device_id scpi_clocks_ids[] = {
	{ .compatible = "arm,scpi-clocks", },
	{}
};
MODULE_DEVICE_TABLE(of, scpi_clocks_ids);

static struct platform_driver scpi_clocks_driver = {
	.driver	= {
		.name = "scpi_clocks",
		.of_match_table = scpi_clocks_ids,
		.acpi_match_table = ACPI_PTR(scpi_clocks_acpi_match),
	},
	.probe = scpi_clocks_probe,
	.remove = scpi_clocks_remove,
};
module_platform_driver(scpi_clocks_driver);

MODULE_AUTHOR("Sudeep Holla <sudeep.holla@arm.com>");
MODULE_DESCRIPTION("ARM SCPI clock driver");
MODULE_LICENSE("GPL v2");
