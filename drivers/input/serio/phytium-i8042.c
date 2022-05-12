	/*
	 * Copyright (c) 2020, Phytium Corporation.
	 *
	 * This program is free software; you can redistribute it and/or modify
	 * it under the terms of the GNU General Public License version 2 as
	 * published by the Free Software Foundation.
	 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/err.h>
#include <linux/rcupdate.h>
#include <linux/platform_device.h>
#include <linux/i8042.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/acpi.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <asm/io.h>

#define I8042_KBD_PHYS_DESC "isa0060/serio0"
#define I8042_AUX_PHYS_DESC "isa0060/serio1"
#define I8042_MUX_PHYS_DESC "isa0060/serio%d"

#define I8042_KBD_IRQ	1
#define I8042_AUX_IRQ	12

#define I8042_COMMAND_REG	0x64
#define I8042_STATUS_REG	0x64
#define I8042_DATA_REG		0x60
#define LPC_STATUS_REG      0xFFF4
#define LPC_INTERRUPT_REG	0xFFF0

#define I8042_CTL_TIMEOUT	10000
#define I8042_RET_CTL_TEST	0x55
#define I8042_BUFFER_SIZE	16
#define I8042_NUM_MUX_PORTS	4

enum i8042_controller_reset_mode {
	I8042_RESET_NEVER,
	I8042_RESET_ALWAYS,
	I8042_RESET_ON_S2RAM,
};

static enum i8042_controller_reset_mode i8042_reset = I8042_RESET_ON_S2RAM;

#ifdef DEBUG
static unsigned long i8042_start_time;

#define dbg_init() do { i8042_start_time = jiffies; } while (0)

#define dbg(format, arg...)							\
	do {									\
		if (i8042_debug)						\
			printk(KERN_DEBUG KBUILD_MODNAME ": [%d] " format,	\
			       (int) (jiffies - i8042_start_time), ##arg);	\
	} while (0)

#define filter_dbg(filter, data, format, args...)		\
	do {							\
		if (!i8042_debug)				\
			break;					\
								\
		if (!filter || i8042_unmask_kbd_data)		\
			dbg("%02x " format, data, ##args);	\
		else						\
			dbg("** " format, ##args);		\
	} while (0)

#else

#define dbg_init() do { } while (0)
#define dbg(format, arg...)							\
	do {									\
		if (0)								\
			printk(KERN_DEBUG pr_fmt(format), ##arg);		\
	} while (0)

#define filter_dbg(filter, data, format, args...) do { } while (0)

#endif

static void __iomem *phytium_i8042_iobase;
static void __iomem *phytium_lpc_base;

static inline int i8042_read_lpc_status(void)
{
	return readw(phytium_lpc_base + LPC_STATUS_REG);
}

static inline void i8042_write_lpc_interrupt_clear(int val)
{
	return writew(val, phytium_lpc_base + LPC_INTERRUPT_REG);
}

static inline int i8042_read_data(void)
{
	return readb(phytium_i8042_iobase + I8042_DATA_REG);
}

static inline int i8042_read_status(void)
{
	return readb(phytium_i8042_iobase + I8042_STATUS_REG);
}

static inline void i8042_write_data(int val)
{
	writeb(val, phytium_i8042_iobase + I8042_DATA_REG);
}

static inline void i8042_write_command(int val)
{
	writeb(val, phytium_i8042_iobase + I8042_COMMAND_REG);
}

MODULE_AUTHOR("Phytium");
MODULE_DESCRIPTION("Phytium LPC interface keyboard and ps2 touch pad");
MODULE_LICENSE("GPL");

static bool i8042_nokbd;
module_param_named(nokbd, i8042_nokbd, bool, 0);
MODULE_PARM_DESC(nokbd, "Do not probe or use KBD port.");

static bool i8042_noaux;
module_param_named(noaux, i8042_noaux, bool, 0);
MODULE_PARM_DESC(noaux, "Do not probe or use AUX (mouse) port.");

static bool i8042_nomux;
module_param_named(nomux, i8042_nomux, bool, 0);
MODULE_PARM_DESC(nomux,
		 "Do not check whether an active multiplexing controller is present.");

static bool i8042_unlock;
module_param_named(unlock, i8042_unlock, bool, 0);
MODULE_PARM_DESC(unlock, "Ignore keyboard lock.");

static struct kobject *debug_kobj;
static int debug_value;

static ssize_t status_show(struct device *kobj, struct device_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "%d\n", READ_ONCE(debug_value));
}

static ssize_t status_store(struct device *kobj, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	if (kstrtoint(buf, 0, &debug_value))
		return -EINVAL;
	return count;
}

static DEVICE_ATTR(status, S_IWUSR | S_IRUGO, status_show, status_store);

static struct attribute *debug_attrs[] = {
	&dev_attr_status.attr,
	NULL
};

static const struct attribute_group sysfs_debug_attr_group = {
	.attrs = debug_attrs,
};

static int i8042_set_reset(const char *val, const struct kernel_param *kp)
{
	enum i8042_controller_reset_mode *arg = kp->arg;
	int error;
	bool reset;

	if (val) {
		error = kstrtobool(val, &reset);
		if (error)
			return error;
	} else {
		reset = true;
	}

	*arg = reset ? I8042_RESET_ALWAYS : I8042_RESET_NEVER;
	return 0;
}

static const struct kernel_param_ops param_ops_reset_param = {
	.flags = KERNEL_PARAM_OPS_FL_NOARG,
	.set = i8042_set_reset,
};

#define param_check_reset_param(name, p)	\
	__param_check(name, p, enum i8042_controller_reset_mode)
module_param_named(reset, i8042_reset, reset_param, 0);
MODULE_PARM_DESC(reset, "Reset controller on resume, cleanup or both");

static bool i8042_direct;
module_param_named(direct, i8042_direct, bool, 0);
MODULE_PARM_DESC(direct, "Put keyboard port into non-translated mode.");

static bool i8042_dumbkbd;
module_param_named(dumbkbd, i8042_dumbkbd, bool, 0);
MODULE_PARM_DESC(dumbkbd,
		 "Pretend that controller can only read data from keyboard");

static bool i8042_noloop;
module_param_named(noloop, i8042_noloop, bool, 0);
MODULE_PARM_DESC(noloop,
		 "Disable the AUX Loopback command while probing for the AUX port");

static bool i8042_notimeout;
module_param_named(notimeout, i8042_notimeout, bool, 0);
MODULE_PARM_DESC(notimeout, "Ignore timeouts signalled by i8042");

static bool i8042_kbdreset;
module_param_named(kbdreset, i8042_kbdreset, bool, 0);
MODULE_PARM_DESC(kbdreset, "Reset device connected to KBD port");

#ifdef CONFIG_PNP
static bool i8042_nopnp;
module_param_named(nopnp, i8042_nopnp, bool, 0);
MODULE_PARM_DESC(nopnp, "Do not use PNP to detect controller settings");
#endif

#define DEBUG
#ifdef DEBUG
static bool i8042_debug;
module_param_named(debug, i8042_debug, bool, 0600);
MODULE_PARM_DESC(debug, "Turn i8042 debugging mode on and off");

static bool i8042_unmask_kbd_data;
module_param_named(unmask_kbd_data, i8042_unmask_kbd_data, bool, 0600);
MODULE_PARM_DESC(unmask_kbd_data,
		 "Unconditional enable (may reveal sensitive data) of normally sanitize-filtered kbd data traffic debug log [pre-condition: i8042.debug=1 enabled]");
#endif

static char i8042_kbd_firmware_id[128];
static char i8042_aux_firmware_id[128];

/*
 * i8042_lock protects serialization between i8042_command and
 * the interrupt handler.
 */
static DEFINE_SPINLOCK(i8042_lock);

/*
 * Writers to AUX and KBD ports as well as users issuing i8042_command
 * directly should acquire i8042_mutex (by means of calling
 * i8042_lock_chip() and i8042_unlock_ship() helpers) to ensure that
 * they do not disturb each other (unfortunately in many i8042
 * implementations write to one of the ports will immediately abort
 * command that is being processed by another port).
 */
static DEFINE_MUTEX(phytium_i8042_mutex);

struct i8042_port {
	struct serio *serio;
	int irq;
	bool exists;
	bool driver_bound;
	signed char mux;
};

#define I8042_KBD_PORT_NO	0
#define I8042_AUX_PORT_NO	1
#define I8042_MUX_PORT_NO	2
#define I8042_NUM_PORTS		(I8042_NUM_MUX_PORTS + 2)
#define I8042_LPC_OFFSET	0x7FF0000

static struct i8042_port i8042_ports[I8042_NUM_PORTS];

static unsigned char i8042_initial_ctr;
static unsigned char i8042_ctr;
static bool i8042_mux_present;
static bool i8042_kbd_irq_registered;
static bool i8042_aux_irq_registered;
static unsigned char i8042_suppress_kbd_ack;
static struct platform_device *i8042_platform_device;
static struct notifier_block i8042_kbd_bind_notifier_block;

static irqreturn_t i8042_interrupt(int irq, void *dev_id);
static bool(*i8042_platform_filter) (unsigned char data, unsigned char str,
				     struct serio * serio);

void phytium_i8042_lock_chip(void)
{
	mutex_lock(&phytium_i8042_mutex);
}

EXPORT_SYMBOL(phytium_i8042_lock_chip);

void phytium_i8042_unlock_chip(void)
{
	mutex_unlock(&phytium_i8042_mutex);
}

EXPORT_SYMBOL(phytium_i8042_unlock_chip);

int phytium_i8042_install_filter(bool(*filter)
				 (unsigned char data, unsigned char str,
				  struct serio * serio))
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&i8042_lock, flags);

	if (i8042_platform_filter) {
		ret = -EBUSY;
		goto out;
	}

	i8042_platform_filter = filter;

out:
	spin_unlock_irqrestore(&i8042_lock, flags);
	return ret;
}

EXPORT_SYMBOL(phytium_i8042_install_filter);

int phytium_i8042_remove_filter(bool(*filter)
				(unsigned char data, unsigned char str,
				 struct serio * port))
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&i8042_lock, flags);

	if (i8042_platform_filter != filter) {
		ret = -EINVAL;
		goto out;
	}

	i8042_platform_filter = NULL;

out:
	spin_unlock_irqrestore(&i8042_lock, flags);
	return ret;
}

EXPORT_SYMBOL(phytium_i8042_remove_filter);

/*
 * The i8042_wait_read() and i8042_wait_write functions wait for the i8042 to
 * be ready for reading values from it / writing values to it.
 * Called always with i8042_lock held.
 */

static int i8042_wait_read(void)
{
	int i = 0;

	while ((~i8042_read_status() & I8042_STR_OBF)
	       && (i < I8042_CTL_TIMEOUT)) {
		udelay(50);
		i++;
	}
	return -(i == I8042_CTL_TIMEOUT);
}

static int i8042_wait_write(void)
{
	int i = 0;

	while ((i8042_read_status() & I8042_STR_IBF) && (i < I8042_CTL_TIMEOUT)) {
		udelay(50);
		i++;
	}
	return -(i == I8042_CTL_TIMEOUT);
}

/*
 * i8042_flush() flushes all data that may be in the keyboard and mouse buffers
 * of the i8042 down the toilet.
 */

static int i8042_flush(void)
{
	unsigned long flags;
	unsigned char data, str;
	int count = 0;
	int retval = 0;

	spin_lock_irqsave(&i8042_lock, flags);

	while ((str = i8042_read_status()) & I8042_STR_OBF) {
		if (count++ < I8042_BUFFER_SIZE) {
			udelay(50);
			data = i8042_read_data();
			dbg("%02x <- i8042 (flush, %s)\n",
			    data, str & I8042_STR_AUXDATA ? "aux" : "kbd");
		} else {
			retval = -EIO;
			break;
		}
	}

	spin_unlock_irqrestore(&i8042_lock, flags);

	return retval;
}

/*
 * i8042_command() executes a command on the i8042. It also sends the input
 * parameter(s) of the commands to it, and receives the output value(s). The
 * parameters are to be stored in the param array, and the output is placed
 * into the same array. The number of the parameters and output values is
 * encoded in bits 8-11 of the command number.
 */

static int __i8042_command(unsigned char *param, int command)
{
	int i, error;

	if (i8042_noloop && command == I8042_CMD_AUX_LOOP)
		return -1;

	error = i8042_wait_write();
	if (error)
		return error;

	dbg("%02x -> i8042 (command)\n", command & 0xff);
	i8042_write_command(command & 0xff);

	for (i = 0; i < ((command >> 12) & 0xf); i++) {
		error = i8042_wait_write();
		if (error) {
			dbg("     -- i8042 (wait write timeout)\n");
			return error;
		}
		dbg("%02x -> i8042 (parameter)\n", param[i]);
		i8042_write_data(param[i]);
	}

	for (i = 0; i < ((command >> 8) & 0xf); i++) {
		error = i8042_wait_read();
		if (error) {
			dbg("     -- i8042 (wait read timeout)\n");
			return error;
		}

		if (command == I8042_CMD_AUX_LOOP &&
		    !(i8042_read_status() & I8042_STR_AUXDATA)) {
			dbg("     -- i8042 (auxerr)\n");
			return -1;
		}

		param[i] = i8042_read_data();
		dbg("%02x <- i8042 (return)\n", param[i]);
	}

	return 0;
}

int phytium_i8042_command(unsigned char *param, int command)
{
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&i8042_lock, flags);
	retval = __i8042_command(param, command);
	spin_unlock_irqrestore(&i8042_lock, flags);

	return retval;
}

EXPORT_SYMBOL(phytium_i8042_command);

/*
 * i8042_kbd_write() sends a byte out through the keyboard interface.
 */

static int i8042_kbd_write(struct serio *port, unsigned char c)
{
	unsigned long flags;
	int retval = 0;

	spin_lock_irqsave(&i8042_lock, flags);

	if (!(retval = i8042_wait_write())) {
		dbg("%02x -> i8042 (kbd-data)\n", c);
		i8042_write_data(c);
	}

	spin_unlock_irqrestore(&i8042_lock, flags);

	return retval;
}

/*
 * i8042_aux_write() sends a byte out through the aux interface.
 */

static int i8042_aux_write(struct serio *serio, unsigned char c)
{
	struct i8042_port *port = serio->port_data;

	return phytium_i8042_command(&c, port->mux == -1 ?
				     I8042_CMD_AUX_SEND :
				     I8042_CMD_MUX_SEND + port->mux);
}

/*
 * i8042_port_close attempts to clear AUX or KBD port state by disabling
 * and then re-enabling it.
 */

static void i8042_port_close(struct serio *serio)
{
	int irq_bit;
	int disable_bit;
	const char *port_name;

	if (serio == i8042_ports[I8042_AUX_PORT_NO].serio) {
		irq_bit = I8042_CTR_AUXINT;
		disable_bit = I8042_CTR_AUXDIS;
		port_name = "AUX";
	} else {
		irq_bit = I8042_CTR_KBDINT;
		disable_bit = I8042_CTR_KBDDIS;
		port_name = "KBD";
	}

	i8042_ctr &= ~irq_bit;
	if (phytium_i8042_command(&i8042_ctr, I8042_CMD_CTL_WCTR))
		pr_warn("Can't write CTR while closing %s port\n", port_name);

	udelay(50);

	i8042_ctr &= ~disable_bit;
	i8042_ctr |= irq_bit;
	if (phytium_i8042_command(&i8042_ctr, I8042_CMD_CTL_WCTR))
		pr_err("Can't reactivate %s port\n", port_name);

	/*
	 * See if there is any data appeared while we were messing with
	 * port state.
	 */
	i8042_interrupt(0, NULL);
}

/*
 * i8042_start() is called by serio core when port is about to finish
 * registering. It will mark port as existing so i8042_interrupt can
 * start sending data through it.
 */
static int i8042_start(struct serio *serio)
{
	struct i8042_port *port = serio->port_data;

	spin_lock_irq(&i8042_lock);
	port->exists = true;
	spin_unlock_irq(&i8042_lock);

	return 0;
}

/*
 * i8042_stop() marks serio port as non-existing so i8042_interrupt
 * will not try to send data to the port that is about to go away.
 * The function is called by serio core as part of unregister procedure.
 */
static void i8042_stop(struct serio *serio)
{
	struct i8042_port *port = serio->port_data;

	spin_lock_irq(&i8042_lock);
	port->exists = false;
	port->serio = NULL;
	spin_unlock_irq(&i8042_lock);

	/*
	 * We need to make sure that interrupt handler finishes using
	 * our serio port before we return from this function.
	 * We synchronize with both AUX and KBD IRQs because there is
	 * a (very unlikely) chance that AUX IRQ is raised for KBD port
	 * and vice versa.
	 */
	synchronize_irq(I8042_AUX_IRQ);
	synchronize_irq(I8042_KBD_IRQ);
}

/*
 * i8042_filter() filters out unwanted bytes from the input data stream.
 * It is called from i8042_interrupt and thus is running with interrupts
 * off and i8042_lock held.
 */
static bool i8042_filter(unsigned char data, unsigned char str,
			 struct serio *serio)
{
	if (unlikely(i8042_suppress_kbd_ack)) {
		if ((~str & I8042_STR_AUXDATA) &&
		    (data == 0xfa || data == 0xfe)) {
			i8042_suppress_kbd_ack--;
			dbg("Extra keyboard ACK - filtered out\n");
			return true;
		}
	}

	if (i8042_platform_filter && i8042_platform_filter(data, str, serio)) {
		dbg("Filtered out by platform filter\n");
		return true;
	}

	return false;
}

/*
 * i8042_interrupt() is the most important function in this driver -
 * it handles the interrupts from the i8042, and sends incoming bytes
 * to the upper layers.
 */

static irqreturn_t i8042_interrupt(int irq, void *dev_id)
{
	struct i8042_port *port;
	struct serio *serio;
	unsigned long flags;
	unsigned char str, data;
	unsigned int dfl;
	unsigned int port_no;
	bool filtered;
	int lpc_data;
	static int count;
	int ret = 1;

	count++;

	spin_lock_irqsave(&i8042_lock, flags);

	lpc_data = i8042_read_lpc_status();
	if (lpc_data & (1 << 11)) {	//check bit3
		debug_value = 1;
	} else {
		debug_value = 0;
	}

	i8042_write_lpc_interrupt_clear(0x0);
	str = i8042_read_status();

	if (unlikely(~str & I8042_STR_OBF)) {
		spin_unlock_irqrestore(&i8042_lock, flags);
		if (irq)
			dbg("Interrupt %d, without any data\n", irq);
		ret = 0;
		goto out;
	}

	data = i8042_read_data();

	if (i8042_mux_present && (str & I8042_STR_AUXDATA)) {
		static unsigned long last_transmit;
		static unsigned char last_str;

		dfl = 0;
		if (str & I8042_STR_MUXERR) {
			dbg("MUX error, status is %02x, data is %02x\n",
			    str, data);
/*
 * When MUXERR condition is signalled the data register can only contain
 * 0xfd, 0xfe or 0xff if implementation follows the spec. Unfortunately
 * it is not always the case. Some KBCs also report 0xfc when there is
 * nothing connected to the port while others sometimes get confused which
 * port the data came from and signal error leaving the data intact. They
 * _do not_ revert to legacy mode (actually I've never seen KBC reverting
 * to legacy mode yet, when we see one we'll add proper handling).
 * Anyway, we process 0xfc, 0xfd, 0xfe and 0xff as timeouts, and for the
 * rest assume that the data came from the same serio last byte
 * was transmitted (if transmission happened not too long ago).
 */

			switch (data) {
			default:
				if (time_before
				    (jiffies, last_transmit + HZ / 10)) {
					str = last_str;
					break;
				}
				/* fall through - report timeout */
			case 0xfc:
			case 0xfd:
			case 0xfe:
				dfl = SERIO_TIMEOUT;
				data = 0xfe;
				break;
			case 0xff:
				dfl = SERIO_PARITY;
				data = 0xfe;
				break;
			}
		}

		port_no = I8042_MUX_PORT_NO + ((str >> 6) & 3);
		last_str = str;
		last_transmit = jiffies;
	} else {

		dfl = ((str & I8042_STR_PARITY) ? SERIO_PARITY : 0) |
		    ((str & I8042_STR_TIMEOUT
		      && !i8042_notimeout) ? SERIO_TIMEOUT : 0);

		port_no = (str & I8042_STR_AUXDATA) ?
		    I8042_AUX_PORT_NO : I8042_KBD_PORT_NO;
	}

	port = &i8042_ports[port_no];
	serio = port->exists ? port->serio : NULL;

	if (irq && serio)
		pm_wakeup_event(&serio->dev, 0);

	filter_dbg(port->driver_bound, data,
		   "<- i8042 (interrupt, %d, %d%s%s)\n", port_no, irq,
		   dfl & SERIO_PARITY ? ", bad parity" : "",
		   dfl & SERIO_TIMEOUT ? ", timeout" : "");

	filtered = i8042_filter(data, str, serio);

	spin_unlock_irqrestore(&i8042_lock, flags);

	if (likely(serio && !filtered))
		serio_interrupt(serio, data, dfl);

out:
	return IRQ_RETVAL(ret);
}

/*
 * i8042_enable_kbd_port enables keyboard port on chip
 */

static int i8042_enable_kbd_port(void)
{
	i8042_ctr &= ~I8042_CTR_KBDDIS;
	i8042_ctr |= I8042_CTR_KBDINT;

	if (phytium_i8042_command(&i8042_ctr, I8042_CMD_CTL_WCTR)) {
		i8042_ctr &= ~I8042_CTR_KBDINT;
		i8042_ctr |= I8042_CTR_KBDDIS;
		pr_err("Failed to enable KBD port\n");
		return -EIO;
	}

	return 0;
}

/*
 * i8042_enable_aux_port enables AUX (mouse) port on chip
 */

static int i8042_enable_aux_port(void)
{
	i8042_ctr &= ~I8042_CTR_AUXDIS;
	i8042_ctr |= I8042_CTR_AUXINT;

	if (phytium_i8042_command(&i8042_ctr, I8042_CMD_CTL_WCTR)) {
		i8042_ctr &= ~I8042_CTR_AUXINT;
		i8042_ctr |= I8042_CTR_AUXDIS;
		pr_err("Failed to enable AUX port\n");
		return -EIO;
	}

	return 0;
}

/*
 * i8042_enable_mux_ports enables 4 individual AUX ports after
 * the controller has been switched into Multiplexed mode
 */

static int i8042_enable_mux_ports(void)
{
	unsigned char param;
	int i;

	for (i = 0; i < I8042_NUM_MUX_PORTS; i++) {
		phytium_i8042_command(&param, I8042_CMD_MUX_PFX + i);
		phytium_i8042_command(&param, I8042_CMD_AUX_ENABLE);
	}

	return i8042_enable_aux_port();
}

/*
 * i8042_set_mux_mode checks whether the controller has an
 * active multiplexor and puts the chip into Multiplexed (true)
 * or Legacy (false) mode.
 */

static int i8042_set_mux_mode(bool multiplex, unsigned char *mux_version)
{

	unsigned char param, val;
/*
 * Get rid of bytes in the queue.
 */

	i8042_flush();

/*
 * Internal loopback test - send three bytes, they should come back from the
 * mouse interface, the last should be version.
 */

	param = val = 0xf0;
	if (phytium_i8042_command(&param, I8042_CMD_AUX_LOOP) || param != val)
		return -1;
	param = val = multiplex ? 0x56 : 0xf6;
	if (phytium_i8042_command(&param, I8042_CMD_AUX_LOOP) || param != val)
		return -1;
	param = val = multiplex ? 0xa4 : 0xa5;
	if (phytium_i8042_command(&param, I8042_CMD_AUX_LOOP) || param == val)
		return -1;

/*
 * Workaround for interference with USB Legacy emulation
 * that causes a v10.12 MUX to be found.
 */
	if (param == 0xac)
		return -1;

	if (mux_version)
		*mux_version = param;

	return 0;
}

/*
 * i8042_check_mux() checks whether the controller supports the PS/2 Active
 * Multiplexing specification by Synaptics, Phoenix, Insyde and
 * LCS/Telegraphics.
 */

static int __init i8042_check_mux(void)
{
	unsigned char mux_version;

	if (i8042_set_mux_mode(true, &mux_version))
		return -1;

	pr_info("Detected active multiplexing controller, rev %d.%d\n",
		(mux_version >> 4) & 0xf, mux_version & 0xf);

/*
 * Disable all muxed ports by disabling AUX.
 */
	i8042_ctr |= I8042_CTR_AUXDIS;
	i8042_ctr &= ~I8042_CTR_AUXINT;

	if (phytium_i8042_command(&i8042_ctr, I8042_CMD_CTL_WCTR)) {
		pr_err("Failed to disable AUX port, can't use MUX\n");
		return -EIO;
	}

	i8042_mux_present = true;

	return 0;
}

static int i8042_controller_check(void)
{
	if (i8042_flush()) {
		pr_info("No controller found\n");
		return -ENODEV;
	}

	return 0;
}

static int i8042_controller_selftest(void)
{
	unsigned char param;
	int i = 0;

	/*
	 * We try this 5 times; on some really fragile systems this does not
	 * take the first time...
	 */
	do {

		if (phytium_i8042_command(&param, I8042_CMD_CTL_TEST)) {
			pr_err("i8042 controller selftest timeout\n");
			return -ENODEV;
		}

		if (param == I8042_RET_CTL_TEST)
			return 0;

		dbg("i8042 controller selftest: %#x != %#x\n",
		    param, I8042_RET_CTL_TEST);
		msleep(50);
	} while (i++ < 5);

	pr_err("i8042 controller selftest failed\n");
	return -EIO;
}

/*
 * i8042_controller init initializes the i8042 controller, and,
 * most importantly, sets it into non-xlated mode if that's
 * desired.
 */

static int i8042_controller_init(void)
{
	unsigned long flags;
	int n = 0;
	unsigned char ctr[2];

/*
 * Save the CTR for restore on unload / reboot.
 */

	do {
		if (n >= 10) {
			pr_err("Unable to get stable CTR read\n");
			return -EIO;
		}

		if (n != 0)
			udelay(50);

		if (phytium_i8042_command(&ctr[n++ % 2], I8042_CMD_CTL_RCTR)) {
			pr_err("Can't read CTR while initializing i8042\n");
			return -EIO;
		}

	} while (n < 2 || ctr[0] != ctr[1]);

	i8042_initial_ctr = i8042_ctr = ctr[0];

/*
 * Disable the keyboard interface and interrupt.
 */

	i8042_ctr |= I8042_CTR_KBDDIS;
	i8042_ctr &= ~I8042_CTR_KBDINT;

/*
 * Handle keylock.
 */

	spin_lock_irqsave(&i8042_lock, flags);
	if (~i8042_read_status() & I8042_STR_KEYLOCK) {
		if (i8042_unlock)
			i8042_ctr |= I8042_CTR_IGNKEYLOCK;
		else
			pr_warn("Warning: Keylock active\n");
	}
	spin_unlock_irqrestore(&i8042_lock, flags);

/*
 * If the chip is configured into nontranslated mode by the BIOS, don't
 * bother enabling translating and be happy.
 */

	if (~i8042_ctr & I8042_CTR_XLATE)
		i8042_direct = true;

/*
 * Set nontranslated mode for the kbd interface if requested by an option.
 * After this the kbd interface becomes a simple serial in/out, like the aux
 * interface is. We don't do this by default, since it can confuse notebook
 * BIOSes.
 */

	if (i8042_direct)
		i8042_ctr &= ~I8042_CTR_XLATE;

/*
 * Write CTR back.
 */

	if (phytium_i8042_command(&i8042_ctr, I8042_CMD_CTL_WCTR)) {
		pr_err("Can't write CTR while initializing i8042\n");
		return -EIO;
	}

/*
 * Flush whatever accumulated while we were disabling keyboard port.
 */

	i8042_flush();

	return 0;
}

/*
 * Reset the controller and reset CRT to the original value set by BIOS.
 */

static void i8042_controller_reset(bool s2r_wants_reset)
{
	i8042_flush();

/*
 * Disable both KBD and AUX interfaces so they don't get in the way
 */

	i8042_ctr |= I8042_CTR_KBDDIS | I8042_CTR_AUXDIS;
	i8042_ctr &= ~(I8042_CTR_KBDINT | I8042_CTR_AUXINT);

	if (phytium_i8042_command(&i8042_ctr, I8042_CMD_CTL_WCTR))
		pr_warn("Can't write CTR while resetting\n");

/*
 * Disable MUX mode if present.
 */

	if (i8042_mux_present)
		i8042_set_mux_mode(false, NULL);

/*
 * Reset the controller if requested.
 */

	if (i8042_reset == I8042_RESET_ALWAYS ||
	    (i8042_reset == I8042_RESET_ON_S2RAM && s2r_wants_reset)) {
		i8042_controller_selftest();
	}

/*
 * Restore the original control register setting.
 */

	if (phytium_i8042_command(&i8042_initial_ctr, I8042_CMD_CTL_WCTR))
		pr_warn("Can't restore CTR\n");
}

/*
 * i8042_panic_blink() will turn the keyboard LEDs on or off and is called
 * when kernel panics. Flashing LEDs is useful for users running X who may
 * not see the console and will help distinguishing panics from "real"
 * lockups.
 *
 * Note that DELAY has a limit of 10ms so we will not get stuck here
 * waiting for KBC to free up even if KBD interrupt is off
 */

#define DELAY do { mdelay(1); if (++delay > 10) return delay; } while(0)

static long i8042_panic_blink(int state)
{
	long delay = 0;
	char led;

	led = (state) ? 0x01 | 0x04 : 0;
	while (i8042_read_status() & I8042_STR_IBF)
		DELAY;
	dbg("%02x -> i8042 (panic blink)\n", 0xed);
	i8042_suppress_kbd_ack = 2;
	i8042_write_data(0xed);	/* set leds */
	DELAY;
	while (i8042_read_status() & I8042_STR_IBF)
		DELAY;
	DELAY;
	dbg("%02x -> i8042 (panic blink)\n", led);
	i8042_write_data(led);
	DELAY;
	return delay;
}

#undef DELAY

#ifdef CONFIG_PM

/*
 * Here we try to reset everything back to a state we had
 * before suspending.
 */

static int i8042_controller_resume(bool s2r_wants_reset)
{
	int error;

	error = i8042_controller_check();
	if (error)
		return error;

	if (i8042_reset == I8042_RESET_ALWAYS ||
	    (i8042_reset == I8042_RESET_ON_S2RAM && s2r_wants_reset)) {
		error = i8042_controller_selftest();
		if (error)
			return error;
	}

/*
 * Restore original CTR value and disable all ports
 */

	i8042_ctr = i8042_initial_ctr;
	if (i8042_direct)
		i8042_ctr &= ~I8042_CTR_XLATE;
	i8042_ctr |= I8042_CTR_AUXDIS | I8042_CTR_KBDDIS;
	i8042_ctr &= ~(I8042_CTR_AUXINT | I8042_CTR_KBDINT);
	if (phytium_i8042_command(&i8042_ctr, I8042_CMD_CTL_WCTR)) {
		pr_warn("Can't write CTR to resume, retrying...\n");
		msleep(50);
		if (phytium_i8042_command(&i8042_ctr, I8042_CMD_CTL_WCTR)) {
			pr_err("CTR write retry failed\n");
			return -EIO;
		}
	}

	if (i8042_mux_present) {
		if (i8042_set_mux_mode(true, NULL) || i8042_enable_mux_ports())
			pr_warn
			    ("failed to resume active multiplexor, mouse won't work\n");
	} else if (i8042_ports[I8042_AUX_PORT_NO].serio)
		i8042_enable_aux_port();

	if (i8042_ports[I8042_KBD_PORT_NO].serio)
		i8042_enable_kbd_port();

	i8042_interrupt(0, NULL);

	return 0;
}

/*
 * Here we try to restore the original BIOS settings to avoid
 * upsetting it.
 */

static int i8042_pm_suspend(struct device *dev)
{
	int i;

	if (pm_suspend_via_firmware())
		i8042_controller_reset(true);

	/* Set up serio interrupts for system wakeup. */
	for (i = 0; i < I8042_NUM_PORTS; i++) {
		struct serio *serio = i8042_ports[i].serio;

		if (serio && device_may_wakeup(&serio->dev))
			enable_irq_wake(i8042_ports[i].irq);
	}

	return 0;
}

static int i8042_pm_resume_noirq(struct device *dev)
{
	if (!pm_resume_via_firmware())
		i8042_interrupt(0, NULL);

	return 0;
}

static int i8042_pm_resume(struct device *dev)
{
	bool want_reset;
	int i;

	for (i = 0; i < I8042_NUM_PORTS; i++) {
		struct serio *serio = i8042_ports[i].serio;

		if (serio && device_may_wakeup(&serio->dev))
			disable_irq_wake(i8042_ports[i].irq);
	}

	/*
	 * If platform firmware was not going to be involved in suspend, we did
	 * not restore the controller state to whatever it had been at boot
	 * time, so we do not need to do anything.
	 */
	if (!pm_suspend_via_firmware())
		return 0;

	/*
	 * We only need to reset the controller if we are resuming after handing
	 * off control to the platform firmware, otherwise we can simply restore
	 * the mode.
	 */
	want_reset = pm_resume_via_firmware();

	return i8042_controller_resume(want_reset);
}

static int i8042_pm_thaw(struct device *dev)
{
	i8042_interrupt(0, NULL);

	return 0;
}

static int i8042_pm_reset(struct device *dev)
{
	i8042_controller_reset(false);

	return 0;
}

static int i8042_pm_restore(struct device *dev)
{
	return i8042_controller_resume(false);
}

static const struct dev_pm_ops i8042_pm_ops = {
	.suspend = i8042_pm_suspend,
	.resume_noirq = i8042_pm_resume_noirq,
	.resume = i8042_pm_resume,
	.thaw = i8042_pm_thaw,
	.poweroff = i8042_pm_reset,
	.restore = i8042_pm_restore,
};

#endif /* CONFIG_PM */

/*
 * We need to reset the 8042 back to original mode on system shutdown,
 * because otherwise BIOSes will be confused.
 */

static void i8042_shutdown(struct platform_device *dev)
{
	i8042_controller_reset(false);
}

static int __init i8042_create_kbd_port(void)
{
	struct serio *serio;
	struct i8042_port *port = &i8042_ports[I8042_KBD_PORT_NO];

	serio = kzalloc(sizeof(struct serio), GFP_KERNEL);
	if (!serio)
		return -ENOMEM;

	serio->id.type = i8042_direct ? SERIO_8042 : SERIO_8042_XL;
	serio->write = i8042_dumbkbd ? NULL : i8042_kbd_write;
	serio->start = i8042_start;
	serio->stop = i8042_stop;
	serio->close = i8042_port_close;
	serio->ps2_cmd_mutex = &phytium_i8042_mutex;
	serio->port_data = port;
	serio->dev.parent = &i8042_platform_device->dev;
	strlcpy(serio->name, "i8042 KBD port", sizeof(serio->name));
	strlcpy(serio->phys, I8042_KBD_PHYS_DESC, sizeof(serio->phys));
	strlcpy(serio->firmware_id, i8042_kbd_firmware_id,
		sizeof(serio->firmware_id));

	port->serio = serio;
	port->irq = I8042_KBD_IRQ;

	return 0;
}

static int __init i8042_create_aux_port(int idx)
{
	struct serio *serio;
	int port_no = idx < 0 ? I8042_AUX_PORT_NO : I8042_MUX_PORT_NO + idx;
	struct i8042_port *port = &i8042_ports[port_no];

	serio = kzalloc(sizeof(struct serio), GFP_KERNEL);
	if (!serio)
		return -ENOMEM;

	serio->id.type = SERIO_8042;
	serio->write = i8042_aux_write;
	serio->start = i8042_start;
	serio->stop = i8042_stop;
	serio->ps2_cmd_mutex = &phytium_i8042_mutex;
	serio->port_data = port;
	serio->dev.parent = &i8042_platform_device->dev;
	if (idx < 0) {
		strlcpy(serio->name, "i8042 AUX port", sizeof(serio->name));
		strlcpy(serio->phys, I8042_AUX_PHYS_DESC, sizeof(serio->phys));
		strlcpy(serio->firmware_id, i8042_aux_firmware_id,
			sizeof(serio->firmware_id));
		serio->close = i8042_port_close;
	} else {
		snprintf(serio->name, sizeof(serio->name), "i8042 AUX%d port",
			 idx);
		snprintf(serio->phys, sizeof(serio->phys), I8042_MUX_PHYS_DESC,
			 idx + 1);
		strlcpy(serio->firmware_id, i8042_aux_firmware_id,
			sizeof(serio->firmware_id));
	}

	port->serio = serio;
	port->mux = idx;
	port->irq = I8042_AUX_IRQ;

	return 0;
}

static void __init i8042_free_kbd_port(void)
{
	if (i8042_ports[I8042_KBD_PORT_NO].serio) {
		kfree(i8042_ports[I8042_KBD_PORT_NO].serio);
		i8042_ports[I8042_KBD_PORT_NO].serio = NULL;
	}
}

static void __init i8042_free_aux_ports(void)
{
	int i;

	for (i = I8042_AUX_PORT_NO; i < I8042_NUM_PORTS; i++) {
		if (i8042_ports[i].serio) {
			kfree(i8042_ports[i].serio);
			i8042_ports[i].serio = NULL;
		}
	}
}

static void __init i8042_register_ports(void)
{
	int i;

	for (i = 0; i < I8042_NUM_PORTS; i++) {
		struct serio *serio = i8042_ports[i].serio;

		if (!serio)
			continue;

		printk(KERN_INFO "serio: %s at %#lx,%#lx irq %d\n",
		       serio->name,
		       (unsigned long)I8042_DATA_REG,
		       (unsigned long)I8042_COMMAND_REG, i8042_ports[i].irq);
		serio_register_port(serio);
		device_set_wakeup_capable(&serio->dev, true);

		/*
		 * On platforms using suspend-to-idle, allow the keyboard to
		 * wake up the system from sleep by enabling keyboard wakeups
		 * by default.  This is consistent with keyboard wakeup
		 * behavior on many platforms using suspend-to-RAM (ACPI S3)
		 * by default.
		 */
		if (pm_suspend_via_s2idle() && i == I8042_KBD_PORT_NO)
			device_set_wakeup_enable(&serio->dev, true);
	}
}

static void i8042_unregister_ports(void)
{
	int i;

	for (i = 0; i < I8042_NUM_PORTS; i++) {
		if (i8042_ports[i].serio) {
			serio_unregister_port(i8042_ports[i].serio);
			i8042_ports[i].serio = NULL;
		}
	}
}

static int __init i8042_setup_aux(void)
{
	int (*aux_enable) (void);
	int error;
	int i;

	if (i8042_nomux || i8042_check_mux()) {
		error = i8042_create_aux_port(-1);
		if (error)
			goto err_free_ports;
		aux_enable = i8042_enable_aux_port;
	} else {
		for (i = 0; i < I8042_NUM_MUX_PORTS; i++) {
			error = i8042_create_aux_port(i);
			if (error)
				goto err_free_ports;
		}
		aux_enable = i8042_enable_mux_ports;
	}

	if (aux_enable())
		goto err_free_ports;
	i8042_aux_irq_registered = true;
	return 0;

err_free_ports:
	i8042_free_aux_ports();
	return error;
}

static int __init i8042_setup_kbd(void)
{
	int error;
	int irq;

	error = i8042_create_kbd_port();
	if (error)
		return error;

	irq = platform_get_irq(i8042_platform_device, 0);
	if (irq < 0) {
		dev_warn(&i8042_platform_device->dev,
			 "unable to get ws0 interrupt.\n");
	} else {
		/*
		 * In case there is a pending ws0 interrupt, just ping
		 * the watchdog before registering the interrupt routine
		 */
		if (devm_request_irq
		    (&i8042_platform_device->dev, irq, i8042_interrupt,
		     IRQF_SHARED, "i8042,bitland_kbd", i8042_platform_device)) {
			dev_warn(&i8042_platform_device->dev,
				 "unable to request IRQ %d.\n", irq);
			goto err_free_port;
		}
	}

	error = i8042_enable_kbd_port();
	if (error)
		goto err_free_port;
	i8042_kbd_irq_registered = true;

	return 0;

err_free_port:
	i8042_free_kbd_port();
	return error;
}

static int i8042_kbd_bind_notifier(struct notifier_block *nb,
				   unsigned long action, void *data)
{
	struct device *dev = data;
	struct serio *serio = to_serio_port(dev);
	struct i8042_port *port = serio->port_data;

	if (serio != i8042_ports[I8042_KBD_PORT_NO].serio)
		return 0;

	switch (action) {
	case BUS_NOTIFY_BOUND_DRIVER:
		port->driver_bound = true;
		break;

	case BUS_NOTIFY_UNBIND_DRIVER:
		port->driver_bound = false;
		break;
	}

	return 0;
}

static int i8042_probe(struct platform_device *dev)
{
	struct resource *res = NULL;
	int error = -1;

	i8042_platform_device = dev;

	if (dev->dev.of_node) {
		res =
		    platform_get_resource(i8042_platform_device, IORESOURCE_MEM,
					  0);
		if (!res) {
			printk("get resource failed\n");
			return -EFAULT;
		}

		phytium_i8042_iobase =
		    devm_ioremap_resource(&i8042_platform_device->dev, res);
		if (!phytium_i8042_iobase) {
			printk("ioremap failed\n");
			return -EFAULT;
		}

		phytium_lpc_base =
		    ioremap(res->start + I8042_LPC_OFFSET, 0xFFFD);
		if (phytium_lpc_base == NULL) {
			printk("ioremap lpc failed\n");
			goto err_iomap;
		}
	} else {
		phytium_i8042_iobase = ioremap(0x20000000, 0x100);
		if (!phytium_i8042_iobase) {
			printk("ioremap failed\n");
			return EFAULT;
		}

		phytium_lpc_base = ioremap(0x27FF0000, 0xFFFD);
		if (!phytium_lpc_base) {
			printk("ioremap failed\n");
			return EFAULT;
		}
	}

	error = i8042_controller_check();
	if (error)
		return error;

	error = i8042_controller_init();
	if (error)
		return error;

	if (!i8042_noaux) {
		error = i8042_setup_aux();
		if (error && error != -ENODEV && error != -EBUSY)
			goto out_fail;
	}
	if (!i8042_nokbd) {
		error = i8042_setup_kbd();
		if (error)
			goto out_fail;
	}
/*
 * Ok, everything is ready, let's register all serio ports
 */
	i8042_register_ports();
	return 0;

out_fail:
	if (phytium_lpc_base) {
		iounmap(phytium_lpc_base);
		phytium_lpc_base = NULL;
	}
	i8042_free_aux_ports();	/* in case KBD failed but AUX not */
	i8042_unregister_ports();
	i8042_controller_reset(false);
	i8042_platform_device = NULL;

err_iomap:
	if (phytium_i8042_iobase) {
		iounmap(phytium_i8042_iobase);
		phytium_i8042_iobase = NULL;
	}

	return error;
}

static int i8042_remove(struct platform_device *dev)
{
	i8042_unregister_ports();
	i8042_controller_reset(false);
	i8042_platform_device = NULL;

	return 0;
}

static const struct of_device_id i8042_of_match[] = {
	{.compatible = "phytium,i8042"},
	{}
};

MODULE_DEVICE_TABLE(of, i8042_of_match);

static const struct acpi_device_id i8042_acpi_match[] = {
	{"LPC0001", 0},
	{}
};

MODULE_DEVICE_TABLE(acpi, i8042_acpi_match);

static struct platform_driver i8042_driver = {
	.driver = {
		   .name = "i8042",
		   .of_match_table = of_match_ptr(i8042_of_match),
		   .acpi_match_table = ACPI_PTR(i8042_acpi_match),
#ifdef CONFIG_PM
		   .pm = &i8042_pm_ops,
#endif
		   },
	.remove = i8042_remove,
	.shutdown = i8042_shutdown,
	.probe = i8042_probe,
};

static struct notifier_block i8042_kbd_bind_notifier_block = {
	.notifier_call = i8042_kbd_bind_notifier,
};

static int __init i8042_init(void)
{
	int ret;
	dbg_init();
	debug_value = 0;

	if ((debug_kobj = kobject_create_and_add("sysfs_debug", NULL)) != NULL) {
		if (sysfs_create_group(debug_kobj, &sysfs_debug_attr_group))
			printk("create group failed");
	}

	ret = platform_driver_register(&i8042_driver);
	if (ret) {
		pr_info("platform_driver_register fail\n");
		return 0;
	}

	bus_register_notifier(&serio_bus, &i8042_kbd_bind_notifier_block);
	panic_blink = i8042_panic_blink;

	return 0;
}

static void __exit i8042_exit(void)
{
	platform_device_unregister(i8042_platform_device);
	platform_driver_unregister(&i8042_driver);

	bus_unregister_notifier(&serio_bus, &i8042_kbd_bind_notifier_block);
	panic_blink = NULL;
}

module_init(i8042_init);
module_exit(i8042_exit);
