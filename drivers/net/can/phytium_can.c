// SPDX-License-Identifier: GPL-2.0+
/* Phytium CAN core controller driver
 *
 * Copyright (C) 2018-2021, Phytium Technology Co., Ltd.
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/kfifo.h>

#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/led.h>

#include "phytium_can.h"

static void phytium_write_reg(const struct phytium_can_priv *priv,
			      enum phytium_can_reg reg, u32 val)
{
	writel(val, priv->reg_base + reg);
}

static u32 phytium_read_reg(const struct phytium_can_priv *priv,
			    enum phytium_can_reg reg)
{
	return readl(priv->reg_base + reg);
}

static void phytium_set_reg_bits(const struct phytium_can_priv *priv,
				 enum phytium_can_reg reg, u32 bs)
{
	u32 val = readl(priv->reg_base + reg);

	val |= bs;
	writel(val, priv->reg_base + reg);
}

static void phytium_clr_reg_bits(const struct phytium_can_priv *priv,
				 enum phytium_can_reg reg, u32 bs)
{
	u32 val = readl(priv->reg_base + reg);

	val &= ~bs;
	writel(val, priv->reg_base + reg);
}

static int phytium_set_bittiming(struct net_device *ndev)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);
	struct can_bittiming *bt = &priv->can.bittiming;
	u32 btr;
	u32 is_config_mode;

	/* Check whether Phytium CAN is in configuration mode.
	 * It cannot set bit timing if Phytium CAN is not in configuration mode.
	 */
	is_config_mode = (priv->read_reg(priv, FTCAN_CTRL) &
			  FTCAN_CTRL_XFER_MASK);
	if (is_config_mode) {
		netdev_alert(ndev,
			     "BUG! Cannot set bittiming - CAN is not in config mode\n");
		return -EPERM;
	}

	/* Setting Baud Rate prescalar value in BRPR Register */
	btr = (bt->brp - 1) << 16;

	/* Setting Time Segment 1 in BTR Register */
	btr |= (bt->prop_seg - 1) << 2;

	btr |= (bt->phase_seg1 - 1) << 5;

	/* Setting Time Segment 2 in BTR Register */
	btr |= (bt->phase_seg2 - 1) << 8;

	/* Setting Synchronous jump width in BTR Register */
	btr |= (bt->sjw - 1);

	priv->write_reg(priv, FTCAN_DAT_RATE_CTRL, btr);
	priv->write_reg(priv, FTCAN_ARB_RATE_CTRL, btr);

	netdev_dbg(ndev, "DAT=0x%08x, ARB=0x%08x\n",
		   priv->read_reg(priv, FTCAN_DAT_RATE_CTRL),
		   priv->read_reg(priv, FTCAN_ARB_RATE_CTRL));

	return 0;
}

static int phytium_can_start(struct net_device *ndev)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);
	int err;

	err = phytium_set_bittiming(ndev);
	if (err < 0)
		return err;

	/* Identifier mask enable */
	priv->set_reg_bits(priv, FTCAN_CTRL, FTCAN_CTRL_AIME_MASK);
	priv->write_reg(priv, FTCAN_ACC_ID0_MASK, FTCAN_ACC_IDN_MASK);
	priv->write_reg(priv, FTCAN_ACC_ID1_MASK, FTCAN_ACC_IDN_MASK);
	priv->write_reg(priv, FTCAN_ACC_ID2_MASK, FTCAN_ACC_IDN_MASK);
	priv->write_reg(priv, FTCAN_ACC_ID3_MASK, FTCAN_ACC_IDN_MASK);

	/* Enable interrupts */
	priv->write_reg(priv, FTCAN_INTR, FTCAN_INTR_EN);

	/*Enable Transfer*/
	priv->set_reg_bits(priv, FTCAN_CTRL, FTCAN_CTRL_XFER_MASK);

	netdev_dbg(ndev, "status:#x%08x\n",
		   priv->read_reg(priv, FTCAN_XFER_STS));

	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	return 0;
}

static int phytium_do_set_mode(struct net_device *ndev, enum can_mode mode)
{
	int ret;

	switch (mode) {
	case CAN_MODE_START:
		ret = phytium_can_start(ndev);
		if (ret < 0) {
			netdev_err(ndev, "xcan_chip_start failed!\n");
			return ret;
		}
		netif_wake_queue(ndev);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int phytium_can_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct can_frame *cf = (struct can_frame *)skb->data;
	u32 id, dlc, frame_head[2] = {0, 0},  data[8] = {0, 0};
	u32 tx_fifo_cnt;
	unsigned long flags;

	if (can_dropped_invalid_skb(ndev, skb))
		return NETDEV_TX_OK;

	/* Check if the TX buffer is full */
	tx_fifo_cnt = (priv->read_reg(priv, FTCAN_FIFO_CNT) >> FTCAN_FIFO_CNT_TFN_SHIFT);
	if (tx_fifo_cnt == priv->tx_max) {
		netif_stop_queue(ndev);
		netdev_err(ndev, "BUG!, TX FIFO full when queue awake!\n");
		return NETDEV_TX_BUSY;
	}

	if (priv->tx_head == priv->tx_tail) {
		priv->tx_head = 0;
		priv->tx_tail = 0;
	}

	/* Watch carefully on the bit sequence */
	if (cf->can_id & CAN_EFF_FLAG) {
		/* Extended CAN ID format */
		id = ((cf->can_id & CAN_EFF_MASK) << FTCAN_IDR_ID2_SHIFT) &
			FTCAN_IDR_ID2_MASK;
		id |= (((cf->can_id & CAN_EFF_MASK) >>
			(CAN_EFF_ID_BITS - CAN_SFF_ID_BITS)) <<
			FTCAN_IDR_ID1_SHIFT) & FTCAN_IDR_ID1_MASK;

		/* The substibute remote TX request bit should be "1"
		 * for extended frames as in the Xilinx CAN datasheet
		 */
		id |= FTCAN_IDR_IDE_MASK | FTCAN_IDR_SRR_MASK;

		if (cf->can_id & CAN_RTR_FLAG)
			/* Extended frames remote TX request */
			id |= FTCAN_IDR_RTR_MASK;

		dlc = cf->can_dlc << FTCAN_IDR_EDLC_SHIFT;

		frame_head[0] = cpu_to_be32p(&id);//id;
		frame_head[1] = cpu_to_be32p(&dlc);//dlc;

		/* Write the Frame to Phytium CAN TX FIFO */
		priv->write_reg(priv, FTCAN_TX_FIFO, frame_head[0]);
		priv->write_reg(priv, FTCAN_TX_FIFO, frame_head[1]);
	} else {
		/* Standard CAN ID format */
		id = ((cf->can_id & CAN_SFF_MASK) << FTCAN_IDR_ID1_SHIFT) &
		     FTCAN_IDR_ID1_MASK;

		if (cf->can_id & CAN_RTR_FLAG)
			/* Standard frames remote TX request */
			id |= FTCAN_IDR_SRR_MASK;

		dlc = ((cf->can_dlc << FTCAN_IDR_SDLC_SHIFT) | FTCAN_IDR_PAD_MASK);
		id |= dlc;

		frame_head[0] =  cpu_to_be32p(&id);

		/* Write the Frame to Xilinx CAN TX FIFO */
		priv->write_reg(priv, FTCAN_TX_FIFO, frame_head[0]);
	}

	if (!(cf->can_id & CAN_RTR_FLAG)) {
		if (cf->can_dlc > 0) {
			data[0] = (*(__be32 *)(cf->data + 0));
			priv->write_reg(priv, FTCAN_TX_FIFO, data[0]);
		}
		if (cf->can_dlc > 4) {
			data[1] = (*(__be32 *)(cf->data + 4));
			priv->write_reg(priv, FTCAN_TX_FIFO, data[1]);
		}
		stats->tx_bytes += cf->can_dlc;
	}

	can_put_echo_skb(skb, ndev, priv->tx_head % priv->tx_max);
	priv->tx_head++;

	/* triggers tranmission */
	spin_lock_irqsave(&priv->lock, flags);
	priv->clr_reg_bits(priv, FTCAN_CTRL, FTCAN_CTRL_XFER_MASK);
	priv->set_reg_bits(priv, FTCAN_CTRL, FTCAN_CTRL_TXREQ_MASK);
	priv->set_reg_bits(priv, FTCAN_CTRL, FTCAN_CTRL_TXREQ_MASK | FTCAN_CTRL_XFER_MASK);
	netif_stop_queue(ndev);
	spin_unlock_irqrestore(&priv->lock, flags);

	return NETDEV_TX_OK;
}

static void phytium_can_rx(struct net_device *ndev)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);
	int data_cnt, i;
	u32 buf[64];

	data_cnt = priv->read_reg(priv, FTCAN_FIFO_CNT);
	data_cnt &= 0x7F;

	for (i = 0; i < data_cnt; ++i)
		buf[i] = priv->read_reg(priv, FTCAN_RX_FIFO);

	if (priv->is_kfifo_full_err)
		return;

	if ((KFIFO_LEN - kfifo_len(&priv->rx_kfifo)) < (data_cnt * 4)) {
		netdev_err(ndev, "RX kfifo is full,restart CAN controller!\n");
		priv->is_kfifo_full_err = true;
		return;
	}

	kfifo_in(&priv->rx_kfifo, buf, data_cnt * 4);

	cancel_delayed_work(&priv->can_frame_work);
	schedule_delayed_work(&priv->can_frame_work, 0);
}

static void phytium_err_interrupt(struct net_device *ndev, u32 isr)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	u32  txerr = 0, rxerr = 0;

	skb = alloc_can_err_skb(ndev, &cf);

	rxerr = priv->read_reg(priv, FTCAN_ERR_CNT) & FTCAN_ERR_CNT_RFN_MASK;
	txerr = ((priv->read_reg(priv, FTCAN_ERR_CNT) &
		FTCAN_ERR_CNT_TFN_MASK) >> FTCAN_ERR_CNT_TFN_SHIFT);

	if (isr & FTCAN_INTR_BOIS_MASK) {
		priv->can.state = CAN_STATE_BUS_OFF;
		priv->can.can_stats.bus_off++;
		/* Leave device in Config Mode in bus-off state */
		can_bus_off(ndev);
		if (skb)
			cf->can_id |= CAN_ERR_BUSOFF;
	} else if ((isr & FTCAN_INTR_PEIS_MASK) == FTCAN_INTR_PEIS_MASK) {
		priv->can.state = CAN_STATE_ERROR_PASSIVE;
		priv->can.can_stats.error_passive++;
		if (skb) {
			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] = (rxerr > 127) ?
					CAN_ERR_CRTL_RX_PASSIVE :
					CAN_ERR_CRTL_TX_PASSIVE;
			cf->data[6] = txerr;
			cf->data[7] = rxerr;
		}
	} else if (isr & FTCAN_INTR_PWIS_MASK) {
		priv->can.state = CAN_STATE_ERROR_WARNING;
		priv->can.can_stats.error_warning++;
		if (skb) {
			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] |= (txerr > rxerr) ?
					CAN_ERR_CRTL_TX_WARNING :
					CAN_ERR_CRTL_RX_WARNING;
			cf->data[6] = txerr;
			cf->data[7] = rxerr;
		}
	}

	/* Check for RX FIFO Overflow interrupt */
	if (isr & FTCAN_INTR_RFIS_MASK) {
		stats->rx_over_errors++;
		stats->rx_errors++;

		if (skb) {
			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
		}
	}

	if (skb) {
		stats->rx_packets++;
		stats->rx_bytes += cf->can_dlc;
		netif_rx(skb);
	}

	if ((isr & FTCAN_INTR_RFIS_MASK) &&
	    (FTCAN_XFER_OVERLOAD_FRAM ==
		(priv->read_reg(priv, FTCAN_XFER_STS) &
		FTCAN_XFER_FRAS_MASK)))
		phytium_can_rx(ndev);
}

static int
phytium_get_frame_from_kfifo(struct net_device *ndev, u8 *buf, int len)
{
	u32 id, dlc, net_dlc, is_standard_frame_flag;
	u32 rdout_dlc, data[2] = {0, 0};
	struct can_frame *cf;
	struct phytium_can_priv *priv;
	struct net_device_stats *stats;
	struct sk_buff *skb;

	memcpy(&id, buf, 4);

	id = be32_to_cpup(&id);

	if (id & FTCAN_IDR_IDE_MASK) {
		/* Received an Extended format frame */
		memcpy(&dlc, buf + 4, 4);
		dlc = (dlc >> 2) & 0xf;

		net_dlc = get_can_dlc(dlc);
		if (net_dlc > 4 && len >= 16)
			rdout_dlc = 16;
		else if (net_dlc > 0 && len >= 12)
			rdout_dlc = 12;
		else if (net_dlc == 0 && len >= 8)
			rdout_dlc = 8;
		else
			return 0;

		is_standard_frame_flag = 0;
	} else {
		/* Received a standard format frame */
		dlc = (id & FTCAN_IDR_DLC_MASK) >> FTCAN_IDR_SDLC_SHIFT;
		net_dlc = get_can_dlc(dlc);
		if (net_dlc > 4 && len >= 12)
			rdout_dlc = 12;
		else if (net_dlc > 0 && len >= 8)
			rdout_dlc = 8;
		else if (net_dlc == 0 && len >= 4)
			rdout_dlc = 4;
		else
			return 0;

		is_standard_frame_flag = 1;
	}

	if (unlikely(!ndev))
		return -1;

	priv = netdev_priv(ndev);
	stats = &ndev->stats;
	skb = alloc_can_skb(ndev, &cf);
	if (unlikely(!skb)) {
		stats->rx_dropped++;
		return rdout_dlc;
	}
	/* Change Phytium CAN ID format to socketCAN ID format */
	if (id & FTCAN_IDR_IDE_MASK) {
		cf->can_id = (id & FTCAN_IDR_ID1_MASK) >> 3;
		cf->can_id |= (id & FTCAN_IDR_ID2_MASK) >>
		FTCAN_IDR_ID2_SHIFT;
		cf->can_id |= CAN_EFF_FLAG;
		if (id & FTCAN_IDR_RTR_MASK)
			cf->can_id |= CAN_RTR_FLAG;
	} else {
		cf->can_id = (id & FTCAN_IDR_ID1_MASK) >>
		FTCAN_IDR_ID1_SHIFT;
		if (id & FTCAN_IDR_SRR_MASK)
			cf->can_id |= CAN_RTR_FLAG;
	}

	cf->can_dlc = net_dlc;

	if (!(cf->can_id & CAN_RTR_FLAG)) {
		if (cf->can_dlc > 0 && is_standard_frame_flag) {
			memcpy(data, buf + 4, 4);
			*(__be32 *)(cf->data) = (data[0]);
		} else if (cf->can_dlc > 0 && !is_standard_frame_flag) {
			memcpy(data, buf + 8, 4);
			*(__be32 *)(cf->data) = (data[0]);
		}

		if (cf->can_dlc > 4 && is_standard_frame_flag) {
			memcpy(data + 1, buf + 8, 4);
			*(__be32 *)(cf->data + 4) = (data[1]);
		} else if (cf->can_dlc > 0 && !is_standard_frame_flag) {
			memcpy(data + 1, buf + 12, 4);
			*(__be32 *)(cf->data + 4) = (data[1]);
		}
	}
	stats->rx_bytes += cf->can_dlc;
	stats->rx_packets++;

	netif_receive_skb(skb);
	return rdout_dlc;
}

static void phytium_poll_kfifo(struct work_struct *work)
{
	u32 len, no_rd_len;
	int rdout_len;
	u8 *buffer;
	struct phytium_can_priv *priv = container_of(work,
						     struct phytium_can_priv,
						     can_frame_work.work);
	struct net_device *ndev = priv->ndev;

	len = kfifo_len(&priv->rx_kfifo);
	if (!len)
		return;

	buffer = kzalloc(len + 4 * 4, GFP_KERNEL);
	if (!buffer)
		return;

	if (priv->can_frame[0])	{
		memcpy(buffer, priv->can_frame + 1, priv->can_frame[0]);
		if (!kfifo_out(&priv->rx_kfifo, buffer + priv->can_frame[0], len))
			dev_err(priv->dev, "Kfifo_out error.\n");
		len += priv->can_frame[0];
	} else {
		if (!kfifo_out(&priv->rx_kfifo, buffer, len))
			dev_err(priv->dev, "Kfifo_out error.\n");
	}

	no_rd_len = len;
	do {
		if (no_rd_len >= CAN_FRAM_MIN_IN_FIFO) {
			rdout_len = phytium_get_frame_from_kfifo(ndev, buffer + (len - no_rd_len),
								 no_rd_len);
			if (rdout_len == -1) {
				priv->can_frame[0] = 0;
				break;
			} else if (!rdout_len) {
				priv->can_frame[0] = no_rd_len;
				memcpy(priv->can_frame + 1,
				       buffer + (len - no_rd_len), no_rd_len);
				break;
			}

			no_rd_len -= rdout_len;
			if (!no_rd_len) {
				/* clear unfinished data length stored in can_frame[0] */
				priv->can_frame[0] = 0;
				break;
			}
		} else {
			priv->can_frame[0] = no_rd_len;
			memcpy(priv->can_frame + 1,
			       buffer + (len - no_rd_len), no_rd_len);
			break;
		}
	} while (1);

	kfree(buffer);
}

static void phytium_tx_interrupt(struct net_device *ndev, u32 isr)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;

	while ((priv->tx_head - priv->tx_tail > 0) &&
	       (isr & FTCAN_INTR_TEIS_MASK)) {
		priv->set_reg_bits(priv, FTCAN_INTR,
				   FTCAN_INTR_TEIC_MASK | FTCAN_INTR_REIC_MASK);
		can_get_echo_skb(ndev, priv->tx_tail % priv->tx_max);
		priv->tx_tail++;
		stats->tx_packets++;
		isr = (priv->read_reg(priv, FTCAN_INTR) &
		       FTCAN_INTR_STATUS_MASK);
	}

	priv->clr_reg_bits(priv, FTCAN_CTRL, FTCAN_CTRL_XFER_MASK);
	priv->clr_reg_bits(priv, FTCAN_CTRL, FTCAN_CTRL_TXREQ_MASK);
	priv->set_reg_bits(priv, FTCAN_CTRL, FTCAN_CTRL_XFER_MASK);
	can_led_event(ndev, CAN_LED_EVENT_TX);
	netif_wake_queue(ndev);
}

static irqreturn_t phytium_can_irq(int irq, void *dev_id)
{
	struct net_device *ndev = (struct net_device *)dev_id;
	struct phytium_can_priv *priv = netdev_priv(ndev);
	u32 isr;

	/* Get the interrupt status from Phytium CAN */
	isr = (priv->read_reg(priv, FTCAN_INTR) & FTCAN_INTR_STATUS_MASK);
	if (!isr)
		return IRQ_NONE;

	/* Check for the type of error interrupt and Processing it */
	if (isr & (FTCAN_INTR_EIS_MASK | FTCAN_INTR_RFIS_MASK |
	    FTCAN_INTR_BOIS_MASK | FTCAN_INTR_PEIS_MASK)) {
		if (isr & FTCAN_INTR_RFIS_MASK) {
			priv->clr_reg_bits(priv, FTCAN_INTR,
					   FTCAN_INTR_EN);
			priv->set_reg_bits(priv, FTCAN_INTR,
					   FTCAN_INTR_REIC_MASK |
					   FTCAN_INTR_TEIC_MASK);
		}

		phytium_err_interrupt(ndev, isr);

		priv->set_reg_bits(priv, FTCAN_INTR,
				   (FTCAN_INTR_EIC_MASK | FTCAN_INTR_RFIC_MASK |
				   FTCAN_INTR_BOIC_MASK | FTCAN_INTR_PEIC_MASK));
		priv->set_reg_bits(priv, FTCAN_INTR, FTCAN_INTR_EN);
		return IRQ_HANDLED;
	}

	if ((isr & FTCAN_INTR_TEIS_MASK)) {
		isr &= (~FTCAN_INTR_REIS_MASK);
		phytium_tx_interrupt(ndev, isr);
	}

	if (isr & (FTCAN_INTR_REIS_MASK)) {
		priv->clr_reg_bits(priv, FTCAN_INTR,
				   FTCAN_INTR_REIE_MASK);
		phytium_can_rx(ndev);
		priv->set_reg_bits(priv, FTCAN_INTR, FTCAN_INTR_REIC_MASK);
		priv->set_reg_bits(priv, FTCAN_INTR,
				   FTCAN_INTR_REIE_MASK);
	}

	return IRQ_HANDLED;
}

static void phytium_can_stop(struct net_device *ndev)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);
	u32 ier, data_cnt, i;

	/* Disable interrupts and leave the can in configuration mode */
	ier = (FTCAN_INTR_DIS & FTCAN_INTR_EN_MASK);
	priv->clr_reg_bits(priv, FTCAN_INTR, ier);

	priv = netdev_priv(ndev);

	data_cnt = priv->read_reg(priv, FTCAN_FIFO_CNT);
	data_cnt &= 0x7F;
	for (i = 0; i < data_cnt; ++i)
		priv->read_reg(priv, FTCAN_RX_FIFO);

	memset(priv->can_frame, 0, sizeof(priv->can_frame));
	priv->is_kfifo_full_err = false;
	kfifo_reset(&priv->rx_kfifo);
	/* Disable Transfer */
	priv->clr_reg_bits(priv, FTCAN_CTRL, FTCAN_CTRL_XFER_MASK);
	priv->can.state = CAN_STATE_STOPPED;
}

static int phytium_can_open(struct net_device *ndev)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);
	int ret;

	ret = request_irq(ndev->irq, phytium_can_irq, priv->irq_flags,
			  ndev->name, ndev);
	if (ret < 0) {
		netdev_err(ndev, "irq allocation for CAN failed\n");
		goto err;
	}

	ret = open_candev(ndev);
	if (ret)
		goto err_irq;

	ret = phytium_can_start(ndev);
	if (ret < 0) {
		netdev_err(ndev, "failed to start!\n");
		goto err_candev;
	}

	can_led_event(ndev, CAN_LED_EVENT_OPEN);

	netif_start_queue(ndev);

	return 0;

err_candev:
	close_candev(ndev);
err_irq:
	free_irq(ndev->irq, ndev);
err:
	return ret;
}

static int phytium_can_close(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	phytium_can_stop(ndev);
	free_irq(ndev->irq, ndev);
	close_candev(ndev);
	can_led_event(ndev, CAN_LED_EVENT_STOP);

	return 0;
}

static int phytium_get_berr_counter(const struct net_device *ndev, struct can_berr_counter *bec)
{
	struct phytium_can_priv *priv = netdev_priv(ndev);

	bec->rxerr = priv->read_reg(priv, FTCAN_ERR_CNT) & FTCAN_ERR_CNT_RFN_MASK;
	bec->txerr = ((priv->read_reg(priv, FTCAN_ERR_CNT) &
		      FTCAN_ERR_CNT_TFN_MASK) >> FTCAN_ERR_CNT_TFN_SHIFT);

	return 0;
}

static const struct net_device_ops phytium_can_netdev_ops = {
	.ndo_open	= phytium_can_open,
	.ndo_stop	= phytium_can_close,
	.ndo_start_xmit	= phytium_can_start_xmit,
	.ndo_change_mtu	= can_change_mtu,
};

void register_phytium_can(struct phytium_can_priv *priv)
{
	int ret;

	priv->can.do_set_mode = phytium_do_set_mode;
	priv->can.do_get_berr_counter = phytium_get_berr_counter;

	priv->ndev->netdev_ops = &phytium_can_netdev_ops;

	priv->write_reg = phytium_write_reg;
	priv->read_reg = phytium_read_reg;
	priv->set_reg_bits = phytium_set_reg_bits;
	priv->clr_reg_bits = phytium_clr_reg_bits;

	if (kfifo_alloc(&priv->rx_kfifo, KFIFO_LEN, GFP_KERNEL)) {
		dev_err(priv->dev, "failed to allocate kfifo\n");
		goto err;
	}

	INIT_DELAYED_WORK(&priv->can_frame_work, phytium_poll_kfifo);

	ret = register_candev(priv->ndev);
	if (ret) {
		dev_err(priv->dev, "fail to register failed (err=%d)\n", ret);
		goto err;
	}

	devm_can_led_init(priv->ndev);
	netdev_dbg(priv->ndev, "reg_base=0x%p irq=%d clock=%d, tx fifo depth:%d\n",
		   priv->reg_base, priv->ndev->irq, priv->can.clock.freq, priv->tx_max);
	return;
err:
	free_candev(priv->ndev);
}
EXPORT_SYMBOL_GPL(register_phytium_can);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cheng Quan <chengquan@phytium.com.cn>");
MODULE_DESCRIPTION("Core driver for Phytium CAN controller");
