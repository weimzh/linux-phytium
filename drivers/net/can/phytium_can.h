/* SPDX-License-Identifier: GPL-2.0 */
/* CAN bus driver for Phytium CAN controller.
 *
 * Copyright (C) 2021, Phytium Technology Co.,Ltd.
 */

#ifndef PHYTIUM_CAN_H
#define PHYTIUM_CAN_H

enum phytium_can_reg {
	FTCAN_CTRL		= 0x00,  /* Global control register */
	FTCAN_INTR		= 0x04,  /* Interrupt register */
	FTCAN_ARB_RATE_CTRL	= 0x08,  /* Arbitration rate control register */
	FTCAN_DAT_RATE_CTRL	= 0x0C,  /* Data rate control register */
	FTCAN_ACC_ID0		= 0x10,  /* Acceptance identifier0 register */
	FTCAN_ACC_ID1		= 0x14,  /* Acceptance identifier1 register */
	FTCAN_ACC_ID2		= 0x18,  /* Acceptance identifier2 register */
	FTCAN_ACC_ID3		= 0x1C,  /* Acceptance identifier3 register */
	FTCAN_ACC_ID0_MASK	= 0x20,  /* Acceptance identifier0 mask register */
	FTCAN_ACC_ID1_MASK	= 0x24,  /* Acceptance identifier1 mask register */
	FTCAN_ACC_ID2_MASK	= 0x28,  /* Acceptance identifier2 mask register */
	FTCAN_ACC_ID3_MASK	= 0x2C,  /* Acceptance identifier3 mask register */
	FTCAN_XFER_STS		= 0x30,  /* Transfer status register */
	FTCAN_ERR_CNT		= 0x34,  /* Error counter register */
	FTCAN_FIFO_CNT		= 0x38,  /* FIFO counter register */
	FTCAN_DMA_CTRL		= 0x3C,  /* DMA request control register */
	FTCAN_TX_FIFO		= 0x100, /* TX FIFO shadow register */
	FTCAN_RX_FIFO		= 0x200, /* RX FIFO shadow register */
};

/* FTCAN_CTRL mask */
#define FTCAN_CTRL_XFER_MASK   (0x1 << 0)  /* Transfer enable */
#define FTCAN_CTRL_TXREQ_MASK  (0x1 << 1)  /* Transmit request */
#define FTCAN_CTRL_AIME_MASK   (0x1 << 2)  /* Acceptance identifier mask enable */

/* FTCAN_INTR mask */
#define FTCAN_INTR_STATUS_MASK (0xFF << 0) /* the interrupt status */
#define FTCAN_INTR_BOIS_MASK   (0x1 << 0)  /* Bus off interrupt status */
#define FTCAN_INTR_PWIS_MASK   (0x1 << 1)  /* Passive warning interrupt status */
#define FTCAN_INTR_PEIS_MASK   (0x1 << 2)  /* Passive error interrupt status */
#define FTCAN_INTR_RFIS_MASK   (0x1 << 3)  /* RX FIFO full interrupt status */
#define FTCAN_INTR_TFIS_MASK   (0x1 << 4)  /* TX FIFO empty interrupt status */
#define FTCAN_INTR_REIS_MASK   (0x1 << 5)  /* RX frame end interrupt status */
#define FTCAN_INTR_TEIS_MASK   (0x1 << 6)  /* TX frame end interrupt status */
#define FTCAN_INTR_EIS_MASK    (0x1 << 7)  /* Error interrupt status */

#define FTCAN_INTR_EN_MASK     (0xFF << 8) /* the interrupt enable */
#define FTCAN_INTR_BOIE_MASK   (0x1 << 8)  /* Bus off interrupt enable */
#define FTCAN_INTR_PWIE_MASK   (0x1 << 9)  /* Passive warning interrupt enable */
#define FTCAN_INTR_PEIE_MASK   (0x1 << 10) /* Passive error interrupt enable */
#define FTCAN_INTR_RFIE_MASK   (0x1 << 11) /* RX FIFO full interrupt enable */
#define FTCAN_INTR_TFIE_MASK   (0x1 << 12) /* TX FIFO empty interrupt enable */
#define FTCAN_INTR_REIE_MASK   (0x1 << 13) /* RX frame end interrupt enable */
#define FTCAN_INTR_TEIE_MASK   (0x1 << 14) /* TX frame end interrupt enable */
#define FTCAN_INTR_EIE_MASK    (0x1 << 15) /* Error interrupt enable */

#define FTCAN_INTR_BOIC_MASK   (0x1 << 16) /* Bus off interrupt clear */
#define FTCAN_INTR_PWIC_MASK   (0x1 << 17) /* Passive warning interrupt clear */
#define FTCAN_INTR_PEIC_MASK   (0x1 << 18) /* Passive error interrupt clear */
#define FTCAN_INTR_RFIC_MASK   (0x1 << 19) /* RX FIFO full interrupt clear */
#define FTCAN_INTR_TFIC_MASK   (0x1 << 20) /* TX FIFO empty interrupt clear */
#define FTCAN_INTR_REIC_MASK   (0x1 << 21) /* RX frame end interrupt clear */
#define FTCAN_INTR_TEIC_MASK   (0x1 << 22) /* TX frame end interrupt clear */
#define FTCAN_INTR_EIC_MASK    (0x1 << 23) /* Error interrupt clear */

#define FTCAN_XFER_XFERS_MASK	(0x1 << 10) /* Transfer status, 1:idle,0:busy */
#define FTCAN_XFER_FRAS_MASK	(0x7)	    /* frame status */
#define FTCAN_XFER_OVERLOAD_FRAM 0x3

/* FTCAN_ACC_ID(0-3)_MASK mask */
#define FTCAN_ACC_IDN_MASK      0x1FFFFFFF

/* FTCAN_ERR_CNT_OFFSET mask */
#define FTCAN_ERR_CNT_RFN_MASK (0xFF << 0)  /* Receive error counter */
#define FTCAN_ERR_CNT_TFN_MASK (0xFF << 16) /* Transmit error counter */

/* FTCAN_FIFO_CNT_OFFSET mask */
#define FTCAN_FIFO_CNT_RFN_MASK (0xFF << 0) /* Receive FIFO valid data number */
#define FTCAN_FIFO_CNT_TFN_MASK (0xFF << 16)/* Transmit FIFO valid data number */

#define FTCAN_ERR_CNT_TFN_SHIFT	  16  /* Tx Error Count shift */
#define FTCAN_FIFO_CNT_TFN_SHIFT  16  /* Tx FIFO Count shift */
#define FTCAN_IDR_ID1_SHIFT       21  /* Standard Messg Identifier */
#define FTCAN_IDR_ID2_SHIFT       1   /* Extended Message Identifier */
#define FTCAN_IDR_SDLC_SHIFT      14
#define FTCAN_IDR_EDLC_SHIFT      26

#define FTCAN_IDR_ID2_MASK	0x0007FFFE /* Extended message ident */
#define FTCAN_IDR_ID1_MASK	0xFFE00000 /* Standard msg identifier */
#define FTCAN_IDR_IDE_MASK	0x00080000 /* Identifier extension */
#define FTCAN_IDR_SRR_MASK	0x00100000 /* Substitute remote TXreq */
#define FTCAN_IDR_RTR_MASK	0x00000001 /* Extended frames remote TX request */
#define FTCAN_IDR_DLC_MASK	0x0003C000 /* Standard msg dlc */
#define FTCAN_IDR_PAD_MASK	0x00003FFF /* Standard msg padding 1 */

#define FTCAN_INTR_EN		(FTCAN_INTR_TEIE_MASK | \
				FTCAN_INTR_REIE_MASK | \
				FTCAN_INTR_RFIE_MASK)

#define FTCAN_INTR_DIS		0x00000000
#define FTCAN_NAPI_WEIGHT	64

#define KFIFO_LEN		4096
#define CAN_FRAM_MIN_IN_FIFO	4

/* struct phytium_can_priv - This definition define CAN driver instance
 * @can:		CAN private data structure.
 * @rx_kfifo:		Received frame FIFO
 * @can_frame_work:	Poll data from kfifo
 * @can_clk:		Pointer to struct clk
 * @tx_head:		Tx CAN packets ready to send on the queue
 * @tx_tail:		Tx CAN packets successfully sended on the queue
 * @tx_max:		Maximum number packets the driver can send
 * @read_reg:		For reading data from CAN registers
 * @write_reg:		For writing data to CAN registers
 * @set_reg_bits:	For writing data to CAN registers bit
 * @clr_reg_bits:	For writing 0 to CAN registers bit
 * @dev:		Device data structure
 * @ndev:		Network device data structure
 * @reg_base:		Ioremapped address to registers
 * @irq_flags:		For request_irq()
 * @lock:		The spin lock flag
 * @isr:		The interrupt status
 * @can_frame:		Store unfinished data frame
 * @is_kfifo_full_err:	Full flag for kfifo
 */
struct phytium_can_priv {
	struct can_priv can;
	struct kfifo rx_kfifo;
	struct delayed_work can_frame_work;
	struct clk *can_clk;

	unsigned int tx_head;
	unsigned int tx_tail;
	unsigned int tx_max;

	u32 (*read_reg)(const struct phytium_can_priv *priv, enum phytium_can_reg reg);
	void (*write_reg)(const struct phytium_can_priv *priv, enum phytium_can_reg reg, u32 val);
	void (*set_reg_bits)(const struct phytium_can_priv *priv, enum phytium_can_reg reg, u32 bs);
	void (*clr_reg_bits)(const struct phytium_can_priv *priv, enum phytium_can_reg reg, u32 bs);

	struct device *dev;
	struct net_device *ndev;

	void __iomem *reg_base;
	unsigned long irq_flags;
	spinlock_t lock;	/* lock for tx */

	u32 isr;
	u32 can_frame[4];
	u32 is_kfifo_full_err;
};

void register_phytium_can(struct phytium_can_priv *priv);

#endif /* PHYTIUM_CAN_H */
