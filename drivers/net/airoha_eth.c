// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Based on Linux airoha_eth.c majorly rewritten
 * and simplified for U-Boot usage for single TX/RX ring.
 *
 * Copyright (c) 2024 AIROHA Inc
 * Author: Lorenzo Bianconi <lorenzo@kernel.org>
 *         Christian Marangi <ansuelsmth@gmail.org>
 */

#include <dm.h>
#include <dm/devres.h>
#include <dm/lists.h>
#include <mapmem.h>
#include <net.h>
#include <regmap.h>
#include <reset.h>
#include <syscon.h>
#include <asm-generic/gpio.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/time.h>
#include <asm/arch/scu-regmap.h>

#define AIROHA_MAX_NUM_GDM_PORTS	4
#define AIROHA_MAX_NUM_QDMA		1
#define AIROHA_MAX_NUM_RSTS		3
#define AIROHA_MAX_NUM_XSI_RSTS		4

#define AIROHA_MAX_PACKET_SIZE		2048
#define AIROHA_NUM_TX_RING		1
#define AIROHA_NUM_RX_RING		1
#define AIROHA_NUM_TX_IRQ		1
#define HW_DSCP_NUM			32
#define IRQ_QUEUE_LEN			1
#define TX_DSCP_NUM			16
#define RX_DSCP_NUM			PKTBUFSRX
#define PSE_QUEUE_RSV_PAGES		64

/* SCU */
#define SCU_SHARE_FEMEM_SEL		0x958

/* SWITCH */
#define SWITCH_CFC			0x04
#define   SWITCH_CPU_PMAP		GENMASK(7, 0)
#define SWITCH_AGC			0x0c
#define   SWITCH_LOCAL_EN		BIT(7)
#define SWITCH_MFC			0x10
#define   SWITCH_BC_FFP			GENMASK(31, 24)
#define   SWITCH_UNM_FFP		GENMASK(23, 16)
#define   SWITCH_UNU_FFP		GENMASK(15, 8)
#define SWITCH_PCR(_n)			(0x2004 + ((_n) * 0x100))
#define   SWITCH_PORT_VLAN_MASK		GENMASK(1, 0)
#define   SWITCH_PORT_FALLBACK_MODE	1
#define   SWITCH_PORT_MATRIX		GENMASK(23, 16)
#define SWITCH_PVC(_n)			(0x2010 + ((_n) * 0x100))
#define   SWITCH_STAG_VPID		GENMASK(31, 16)
#define   SWITCH_VLAN_ATTR		GENMASK(7, 6)
#define   SWITCH_VLAN_ATTR_USER		0
#define   SWITCH_PORT_SPEC_TAG		BIT(5)
#define SWITCH_PMCR(_n)			0x3000 + ((_n) * 0x100)
#define   SWITCH_IPG_CFG		GENMASK(19, 18)
#define     SWITCH_IPG_CFG_NORMAL	FIELD_PREP(SWITCH_IPG_CFG, 0x0)
#define     SWITCH_IPG_CFG_SHORT	FIELD_PREP(SWITCH_IPG_CFG, 0x1)
#define     SWITCH_IPG_CFG_SHRINK	FIELD_PREP(SWITCH_IPG_CFG, 0x2)
#define   SWITCH_MAC_MODE		BIT(16)
#define   SWITCH_FORCE_MODE		BIT(15)
#define   SWITCH_MAC_TX_EN		BIT(14)
#define   SWITCH_MAC_RX_EN		BIT(13)
#define   SWITCH_BKOFF_EN		BIT(9)
#define   SWITCH_BKPR_EN		BIT(8)
#define   SWITCH_FORCE_RX_FC		BIT(5)
#define   SWITCH_FORCE_TX_FC		BIT(4)
#define   SWITCH_FORCE_SPD		GENMASK(3, 2)
#define     SWITCH_FORCE_SPD_10		FIELD_PREP(SWITCH_FORCE_SPD, 0x0)
#define     SWITCH_FORCE_SPD_100	FIELD_PREP(SWITCH_FORCE_SPD, 0x1)
#define     SWITCH_FORCE_SPD_1000	FIELD_PREP(SWITCH_FORCE_SPD, 0x2)
#define   SWITCH_FORCE_DPX		BIT(1)
#define   SWITCH_FORCE_LNK		BIT(0)
#define SWITCH_SMACCR0			0x30e4
#define   SMACCR0_MAC2			GENMASK(31, 24)
#define   SMACCR0_MAC3			GENMASK(23, 16)
#define   SMACCR0_MAC4			GENMASK(15, 8)
#define   SMACCR0_MAC5			GENMASK(7, 0)
#define SWITCH_SMACCR1			0x30e8
#define   SMACCR1_MAC0			GENMASK(15, 8)
#define   SMACCR1_MAC1			GENMASK(7, 0)
#define SWITCH_PHY_POLL			0x7018
#define   SWITCH_PHY_AP_EN		GENMASK(30, 24)
#define   SWITCH_EEE_POLL_EN		GENMASK(22, 16)
#define   SWITCH_PHY_PRE_EN		BIT(15)
#define   SWITCH_PHY_END_ADDR		GENMASK(12, 8)
#define   SWITCH_PHY_ST_ADDR		GENMASK(4, 0)
#define SWITCH_CPORT_SPTAG_CFG		0x7c10
#define   SWITCH_SW2FE_STAG_EN		BIT(1)
#define   SWITCH_FE2SW_STAG_EN		BIT(0)

/* FE */
#define PSE_BASE			0x0100
#define CSR_IFC_BASE			0x0200
#define CDM1_BASE			0x0400
#define GDM1_BASE			0x0500
#define PPE1_BASE			0x0c00

#define CDM2_BASE			0x1400
#define GDM2_BASE			0x1500

#define GDM3_BASE			0x1100
#define GDM4_BASE			0x2500

#define GDM_BASE(_n)			\
	((_n) == 4 ? GDM4_BASE :	\
	 (_n) == 3 ? GDM3_BASE :	\
	 (_n) == 2 ? GDM2_BASE : GDM1_BASE)

#define REG_FE_DMA_GLO_CFG		0x0000
#define FE_DMA_GLO_L2_SPACE_MASK	GENMASK(7, 4)
#define FE_DMA_GLO_PG_SZ_MASK		BIT(3)

#define REG_FE_RST_GLO_CFG		0x0004
#define FE_RST_GDM4_MBI_ARB_MASK	BIT(3)
#define FE_RST_GDM3_MBI_ARB_MASK	BIT(2)
#define FE_RST_CORE_MASK		BIT(0)

#define REG_FE_CDM1_OQ_MAP0		0x0050
#define REG_FE_CDM1_OQ_MAP1		0x0054
#define REG_FE_CDM1_OQ_MAP2		0x0058
#define REG_FE_CDM1_OQ_MAP3		0x005c

#define REG_FE_PCE_CFG			0x0070
#define PCE_DPI_EN_MASK			BIT(2)
#define PCE_KA_EN_MASK			BIT(1)
#define PCE_MC_EN_MASK			BIT(0)

#define REG_FE_PSE_QUEUE_CFG_WR		0x0080
#define PSE_CFG_PORT_ID_MASK		GENMASK(27, 24)
#define PSE_CFG_QUEUE_ID_MASK		GENMASK(20, 16)
#define PSE_CFG_WR_EN_MASK		BIT(8)
#define PSE_CFG_OQRSV_SEL_MASK		BIT(0)

#define REG_FE_PSE_QUEUE_CFG_VAL	0x0084
#define PSE_CFG_OQ_RSV_MASK		GENMASK(13, 0)

#define PSE_FQ_CFG			0x008c
#define PSE_FQ_LIMIT_MASK		GENMASK(14, 0)

#define REG_FE_PSE_BUF_SET		0x0090
#define PSE_SHARE_USED_LTHD_MASK	GENMASK(31, 16)
#define PSE_ALLRSV_MASK			GENMASK(14, 0)

#define REG_PSE_SHARE_USED_THD		0x0094
#define PSE_SHARE_USED_MTHD_MASK	GENMASK(31, 16)
#define PSE_SHARE_USED_HTHD_MASK	GENMASK(15, 0)

#define REG_GDM_MISC_CFG		0x0148
#define GDM2_RDM_ACK_WAIT_PREF_MASK	BIT(9)
#define GDM2_CHN_VLD_MODE_MASK		BIT(5)

#define REG_FE_CSR_IFC_CFG		CSR_IFC_BASE
#define FE_IFC_EN_MASK			BIT(0)

#define REG_PSE_IQ_REV1			(PSE_BASE + 0x08)
#define PSE_IQ_RES1_P2_MASK		GENMASK(23, 16)

#define REG_PSE_IQ_REV2			(PSE_BASE + 0x0c)
#define PSE_IQ_RES2_P5_MASK		GENMASK(15, 8)
#define PSE_IQ_RES2_P4_MASK		GENMASK(7, 0)

#define REG_CDM1_FWD_CFG		(CDM1_BASE + 0x08)
#define REG_CDM1_VLAN_CTRL		CDM1_BASE
#define CDM1_VLAN_MASK			GENMASK(31, 16)
#define CDM1_VIP_QSEL_MASK		GENMASK(24, 20)

#define REG_CDM2_FWD_CFG		(CDM2_BASE + 0x08)
#define CDM2_OAM_QSEL_MASK		GENMASK(31, 27)
#define CDM2_VIP_QSEL_MASK		GENMASK(24, 20)

#define REG_GDM_FWD_CFG(_n)		GDM_BASE(_n)
#define GDM_PAD_EN			BIT(28)
#define GDM_DROP_CRC_ERR		BIT(23)
#define GDM_IP4_CKSUM			BIT(22)
#define GDM_TCP_CKSUM			BIT(21)
#define GDM_UDP_CKSUM			BIT(20)
#define GDM_UCFQ_MASK			GENMASK(15, 12)
#define GDM_BCFQ_MASK			GENMASK(11, 8)
#define GDM_MCFQ_MASK			GENMASK(7, 4)
#define GDM_OCFQ_MASK			GENMASK(3, 0)

#define REG_GDM_INGRESS_CFG(_n)		(GDM_BASE(_n) + 0x10)
#define GDM_STAG_EN_MASK		BIT(0)

#define REG_GDM_LEN_CFG(_n)		(GDM_BASE(_n) + 0x14)
#define GDM_SHORT_LEN_MASK		GENMASK(13, 0)
#define GDM_LONG_LEN_MASK		GENMASK(29, 16)
#define REG_FE_CPORT_CFG		(GDM1_BASE + 0x40)
#define FE_CPORT_PAD			BIT(26)
#define FE_CPORT_PORT_XFC_MASK		BIT(25)
#define FE_CPORT_QUEUE_XFC_MASK		BIT(24)

#define REG_GDM2_CHN_RLS		(GDM2_BASE + 0x20)
#define MBI_RX_AGE_SEL_MASK		GENMASK(26, 25)
#define MBI_TX_AGE_SEL_MASK		GENMASK(18, 17)
#define REG_GDM3_FWD_CFG		GDM3_BASE
#define GDM3_PAD_EN_MASK		BIT(28)
#define REG_GDM4_FWD_CFG		GDM4_BASE
#define GDM4_PAD_EN_MASK		BIT(28)
#define REG_GDM4_SRC_PORT_SET		(GDM4_BASE + 0x23c)
#define GDM4_SPORT_OFF2_MASK		GENMASK(19, 16)
#define GDM4_SPORT_OFF1_MASK		GENMASK(15, 12)
#define GDM4_SPORT_OFF0_MASK		GENMASK(11, 8)

/* QDMA */
#define REG_QDMA_GLOBAL_CFG			0x0004
#define GLOBAL_CFG_RX_2B_OFFSET_MASK		BIT(31)
#define GLOBAL_CFG_DMA_PREFERENCE_MASK		GENMASK(30, 29)
#define GLOBAL_CFG_CPU_TXR_RR_MASK		BIT(28)
#define GLOBAL_CFG_DSCP_BYTE_SWAP_MASK		BIT(27)
#define GLOBAL_CFG_PAYLOAD_BYTE_SWAP_MASK	BIT(26)
#define GLOBAL_CFG_MULTICAST_MODIFY_FP_MASK	BIT(25)
#define GLOBAL_CFG_OAM_MODIFY_MASK		BIT(24)
#define GLOBAL_CFG_RESET_MASK			BIT(23)
#define GLOBAL_CFG_RESET_DONE_MASK		BIT(22)
#define GLOBAL_CFG_MULTICAST_EN_MASK		BIT(21)
#define GLOBAL_CFG_IRQ1_EN_MASK			BIT(20)
#define GLOBAL_CFG_IRQ0_EN_MASK			BIT(19)
#define GLOBAL_CFG_LOOPCNT_EN_MASK		BIT(18)
#define GLOBAL_CFG_RD_BYPASS_WR_MASK		BIT(17)
#define GLOBAL_CFG_QDMA_LOOPBACK_MASK		BIT(16)
#define GLOBAL_CFG_LPBK_RXQ_SEL_MASK		GENMASK(13, 8)
#define GLOBAL_CFG_CHECK_DONE_MASK		BIT(7)
#define GLOBAL_CFG_TX_WB_DONE_MASK		BIT(6)
#define GLOBAL_CFG_MAX_ISSUE_NUM_MASK		GENMASK(5, 4)
#define GLOBAL_CFG_RX_DMA_BUSY_MASK		BIT(3)
#define GLOBAL_CFG_RX_DMA_EN_MASK		BIT(2)
#define GLOBAL_CFG_TX_DMA_BUSY_MASK		BIT(1)
#define GLOBAL_CFG_TX_DMA_EN_MASK		BIT(0)

#define REG_FWD_DSCP_BASE			0x0010
#define REG_FWD_BUF_BASE			0x0014

#define REG_HW_FWD_DSCP_CFG			0x0018
#define HW_FWD_DSCP_PAYLOAD_SIZE_MASK		GENMASK(29, 28)
#define HW_FWD_DSCP_SCATTER_LEN_MASK		GENMASK(17, 16)
#define HW_FWD_DSCP_MIN_SCATTER_LEN_MASK	GENMASK(15, 0)

#define REG_INT_STATUS(_n)		\
	(((_n) == 4) ? 0x0730 :		\
	 ((_n) == 3) ? 0x0724 :		\
	 ((_n) == 2) ? 0x0720 :		\
	 ((_n) == 1) ? 0x0024 : 0x0020)

#define REG_TX_IRQ_BASE(_n)		((_n) ? 0x0048 : 0x0050)

#define REG_TX_IRQ_CFG(_n)		((_n) ? 0x004c : 0x0054)
#define TX_IRQ_THR_MASK			GENMASK(27, 16)
#define TX_IRQ_DEPTH_MASK		GENMASK(11, 0)

#define REG_IRQ_CLEAR_LEN(_n)		((_n) ? 0x0064 : 0x0058)
#define IRQ_CLEAR_LEN_MASK		GENMASK(7, 0)

#define REG_TX_RING_BASE(_n)	\
	(((_n) < 8) ? 0x0100 + ((_n) << 5) : 0x0b00 + (((_n) - 8) << 5))

#define REG_TX_CPU_IDX(_n)	\
	(((_n) < 8) ? 0x0108 + ((_n) << 5) : 0x0b08 + (((_n) - 8) << 5))

#define TX_RING_CPU_IDX_MASK		GENMASK(15, 0)

#define REG_TX_DMA_IDX(_n)	\
	(((_n) < 8) ? 0x010c + ((_n) << 5) : 0x0b0c + (((_n) - 8) << 5))

#define TX_RING_DMA_IDX_MASK		GENMASK(15, 0)

#define IRQ_RING_IDX_MASK		GENMASK(20, 16)
#define IRQ_DESC_IDX_MASK		GENMASK(15, 0)

#define REG_RX_RING_BASE(_n)	\
	(((_n) < 16) ? 0x0200 + ((_n) << 5) : 0x0e00 + (((_n) - 16) << 5))

#define REG_RX_RING_SIZE(_n)	\
	(((_n) < 16) ? 0x0204 + ((_n) << 5) : 0x0e04 + (((_n) - 16) << 5))

#define RX_RING_THR_MASK		GENMASK(31, 16)
#define RX_RING_SIZE_MASK		GENMASK(15, 0)

#define REG_RX_CPU_IDX(_n)	\
	(((_n) < 16) ? 0x0208 + ((_n) << 5) : 0x0e08 + (((_n) - 16) << 5))

#define RX_RING_CPU_IDX_MASK		GENMASK(15, 0)

#define REG_RX_DMA_IDX(_n)	\
	(((_n) < 16) ? 0x020c + ((_n) << 5) : 0x0e0c + (((_n) - 16) << 5))

#define REG_RX_DELAY_INT_IDX(_n)	\
	(((_n) < 16) ? 0x0210 + ((_n) << 5) : 0x0e10 + (((_n) - 16) << 5))

#define RX_DELAY_INT_MASK		GENMASK(15, 0)

#define RX_RING_DMA_IDX_MASK		GENMASK(15, 0)

#define REG_LMGR_INIT_CFG		0x1000
#define LMGR_INIT_START			BIT(31)
#define LMGR_SRAM_MODE_MASK		BIT(30)
#define HW_FWD_PKTSIZE_OVERHEAD_MASK	GENMASK(27, 20)
#define HW_FWD_DESC_NUM_MASK		GENMASK(16, 0)

/* CTRL */
#define QDMA_DESC_DONE_MASK		BIT(31)
#define QDMA_DESC_DROP_MASK		BIT(30) /* tx: drop - rx: overflow */
#define QDMA_DESC_MORE_MASK		BIT(29) /* more SG elements */
#define QDMA_DESC_DEI_MASK		BIT(25)
#define QDMA_DESC_NO_DROP_MASK		BIT(24)
#define QDMA_DESC_LEN_MASK		GENMASK(15, 0)
/* DATA */
#define QDMA_DESC_NEXT_ID_MASK		GENMASK(15, 0)
/* TX MSG0 */
#define QDMA_ETH_TXMSG_MIC_IDX_MASK	BIT(30)
#define QDMA_ETH_TXMSG_SP_TAG_MASK	GENMASK(29, 14)
#define QDMA_ETH_TXMSG_ICO_MASK		BIT(13)
#define QDMA_ETH_TXMSG_UCO_MASK		BIT(12)
#define QDMA_ETH_TXMSG_TCO_MASK		BIT(11)
#define QDMA_ETH_TXMSG_TSO_MASK		BIT(10)
#define QDMA_ETH_TXMSG_FAST_MASK	BIT(9)
#define QDMA_ETH_TXMSG_OAM_MASK		BIT(8)
#define QDMA_ETH_TXMSG_CHAN_MASK	GENMASK(7, 3)
#define QDMA_ETH_TXMSG_QUEUE_MASK	GENMASK(2, 0)
/* TX MSG1 */
#define QDMA_ETH_TXMSG_NO_DROP		BIT(31)
#define QDMA_ETH_TXMSG_METER_MASK	GENMASK(30, 24)	/* 0x7f no meters */
#define QDMA_ETH_TXMSG_FPORT_MASK	GENMASK(23, 20)
#define QDMA_ETH_TXMSG_NBOQ_MASK	GENMASK(19, 15)
#define QDMA_ETH_TXMSG_HWF_MASK		BIT(14)
#define QDMA_ETH_TXMSG_HOP_MASK		BIT(13)
#define QDMA_ETH_TXMSG_PTP_MASK		BIT(12)
#define QDMA_ETH_TXMSG_ACNT_G1_MASK	GENMASK(10, 6)	/* 0x1f do not count */
#define QDMA_ETH_TXMSG_ACNT_G0_MASK	GENMASK(5, 0)	/* 0x3f do not count */

/* RX MSG1 */
#define QDMA_ETH_RXMSG_DEI_MASK		BIT(31)
#define QDMA_ETH_RXMSG_IP6_MASK		BIT(30)
#define QDMA_ETH_RXMSG_IP4_MASK		BIT(29)
#define QDMA_ETH_RXMSG_IP4F_MASK	BIT(28)
#define QDMA_ETH_RXMSG_L4_VALID_MASK	BIT(27)
#define QDMA_ETH_RXMSG_L4F_MASK		BIT(26)
#define QDMA_ETH_RXMSG_SPORT_MASK	GENMASK(25, 21)
#define QDMA_ETH_RXMSG_CRSN_MASK	GENMASK(20, 16)
#define QDMA_ETH_RXMSG_PPE_ENTRY_MASK	GENMASK(15, 0)

struct airoha_qdma_desc {
	__le32 rsv;
	__le32 ctrl;
	__le32 addr;
	__le32 data;
	__le32 msg0;
	__le32 msg1;
	__le32 msg2;
	__le32 msg3;
};

struct airoha_qdma_fwd_desc {
	__le32 addr;
	__le32 ctrl0;
	__le32 ctrl1;
	__le32 ctrl2;
	__le32 msg0;
	__le32 msg1;
	__le32 rsv0;
	__le32 rsv1;
};

struct airoha_queue {
	struct airoha_qdma_desc *desc;
	u16 head;

	int ndesc;
};

struct airoha_tx_irq_queue {
	struct airoha_qdma *qdma;

	int size;
	u32 *q;
};

struct airoha_qdma {
	struct airoha_eth *eth;
	void __iomem *regs;

	struct airoha_tx_irq_queue q_tx_irq[AIROHA_NUM_TX_IRQ];

	struct airoha_queue q_tx[AIROHA_NUM_TX_RING];
	struct airoha_queue q_rx[AIROHA_NUM_RX_RING];

	/* descriptor and packet buffers for qdma hw forward */
	struct {
		void *desc;
		void *q;
	} hfwd;
};

struct airoha_gdm_port {
	struct airoha_qdma *qdma;
	int id;
};

struct airoha_eth {
	void __iomem *fe_regs;
	void __iomem *switch_regs;

	struct reset_ctl_bulk rsts;
	struct reset_ctl_bulk xsi_rsts;
	struct reset_ctl switch_rst;
	bool has_switch_rst;

	struct airoha_qdma qdma[AIROHA_MAX_NUM_QDMA];
	struct airoha_gdm_port *ports[AIROHA_MAX_NUM_GDM_PORTS];
};

struct airoha_eth_soc_data {
	int num_xsi_rsts;
	const char * const *xsi_rsts_names;
	const char *switch_compatible;
};

enum {
	FE_PSE_PORT_CDM1,
	FE_PSE_PORT_GDM1,
	FE_PSE_PORT_GDM2,
	FE_PSE_PORT_GDM3,
	FE_PSE_PORT_PPE1,
	FE_PSE_PORT_CDM2,
	FE_PSE_PORT_CDM3,
	FE_PSE_PORT_CDM4,
	FE_PSE_PORT_PPE2,
	FE_PSE_PORT_GDM4,
	FE_PSE_PORT_CDM5,
};

static const char * const en7523_xsi_rsts_names[] = {
	"hsi0-mac",
	"hsi1-mac",
	"hsi-mac",
};

static const char * const en7581_xsi_rsts_names[] = {
	"hsi0-mac",
	"hsi1-mac",
	"hsi-mac",
	"xfp-mac",
};

static u32 airoha_rr(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

static void airoha_wr(void __iomem *base, u32 offset, u32 val)
{
	writel(val, base + offset);
}

static u32 airoha_rmw(void __iomem *base, u32 offset, u32 mask, u32 val)
{
	val |= (airoha_rr(base, offset) & ~mask);
	airoha_wr(base, offset, val);

	return val;
}

#define airoha_fe_rr(eth, offset)				\
	airoha_rr((eth)->fe_regs, (offset))
#define airoha_fe_wr(eth, offset, val)				\
	airoha_wr((eth)->fe_regs, (offset), (val))
#define airoha_fe_rmw(eth, offset, mask, val)			\
	airoha_rmw((eth)->fe_regs, (offset), (mask), (val))
#define airoha_fe_set(eth, offset, val)				\
	airoha_rmw((eth)->fe_regs, (offset), 0, (val))
#define airoha_fe_clear(eth, offset, val)			\
	airoha_rmw((eth)->fe_regs, (offset), (val), 0)

#define airoha_qdma_rr(qdma, offset)				\
	airoha_rr((qdma)->regs, (offset))
#define airoha_qdma_wr(qdma, offset, val)			\
	airoha_wr((qdma)->regs, (offset), (val))
#define airoha_qdma_rmw(qdma, offset, mask, val)		\
	airoha_rmw((qdma)->regs, (offset), (mask), (val))
#define airoha_qdma_set(qdma, offset, val)			\
	airoha_rmw((qdma)->regs, (offset), 0, (val))
#define airoha_qdma_clear(qdma, offset, val)			\
	airoha_rmw((qdma)->regs, (offset), (val), 0)

#define airoha_switch_wr(eth, offset, val)			\
	airoha_wr((eth)->switch_regs, (offset), (val))
#define airoha_switch_rr(eth, offset)				\
	airoha_rr((eth)->switch_regs, (offset))
#define airoha_switch_rmw(eth, offset, mask, val)		\
	airoha_rmw((eth)->switch_regs, (offset), (mask), (val))

static void airoha_set_gdm_port_fwd_cfg(struct airoha_eth *eth, u32 addr,
					u32 val)
{
	airoha_fe_rmw(eth, addr, GDM_OCFQ_MASK,
		      FIELD_PREP(GDM_OCFQ_MASK, val));
	airoha_fe_rmw(eth, addr, GDM_MCFQ_MASK,
		      FIELD_PREP(GDM_MCFQ_MASK, val));
	airoha_fe_rmw(eth, addr, GDM_BCFQ_MASK,
		      FIELD_PREP(GDM_BCFQ_MASK, val));
	airoha_fe_rmw(eth, addr, GDM_UCFQ_MASK,
		      FIELD_PREP(GDM_UCFQ_MASK, val));
}

static inline dma_addr_t dma_map_unaligned(void *vaddr, size_t len,
					   enum dma_data_direction dir)
{
	uintptr_t start, end;

	start = ALIGN_DOWN((uintptr_t)vaddr, ARCH_DMA_MINALIGN);
	end = ALIGN((uintptr_t)(vaddr + len), ARCH_DMA_MINALIGN);

	return dma_map_single((void *)start, end - start, dir);
}

static inline void dma_unmap_unaligned(dma_addr_t addr, size_t len,
				       enum dma_data_direction dir)
{
	uintptr_t start, end;

	start = ALIGN_DOWN((uintptr_t)addr, ARCH_DMA_MINALIGN);
	end = ALIGN((uintptr_t)(addr + len), ARCH_DMA_MINALIGN);
	dma_unmap_single(start, end - start, dir);
}

static void airoha_fe_maccr_init(struct airoha_eth *eth)
{
	int p;

	for (p = 1; p <= AIROHA_MAX_NUM_GDM_PORTS; p++) {
		/*
		 * Disable any kind of CRC drop or offload.
		 * Enable padding of short TX packets to 60 bytes.
		 */
		airoha_fe_wr(eth, REG_GDM_FWD_CFG(p), GDM_PAD_EN);
	}

	airoha_fe_rmw(eth, REG_CDM1_VLAN_CTRL, CDM1_VLAN_MASK,
		      FIELD_PREP(CDM1_VLAN_MASK, 0x8100));
	airoha_fe_set(eth, REG_FE_CPORT_CFG, FE_CPORT_PAD);
}

static u32 airoha_fe_get_pse_queue_rsv_pages(struct airoha_eth *eth,
					     u32 port, u32 queue)
{
	u32 val;

	airoha_fe_rmw(eth, REG_FE_PSE_QUEUE_CFG_WR,
		      PSE_CFG_PORT_ID_MASK | PSE_CFG_QUEUE_ID_MASK,
		      FIELD_PREP(PSE_CFG_PORT_ID_MASK, port) |
		      FIELD_PREP(PSE_CFG_QUEUE_ID_MASK, queue));
	val = airoha_fe_rr(eth, REG_FE_PSE_QUEUE_CFG_VAL);

	return FIELD_GET(PSE_CFG_OQ_RSV_MASK, val);
}

static void airoha_fe_set_pse_queue_rsv_pages(struct airoha_eth *eth,
					      u32 port, u32 queue, u32 val)
{
	airoha_fe_rmw(eth, REG_FE_PSE_QUEUE_CFG_VAL, PSE_CFG_OQ_RSV_MASK,
		      FIELD_PREP(PSE_CFG_OQ_RSV_MASK, val));
	airoha_fe_rmw(eth, REG_FE_PSE_QUEUE_CFG_WR,
		      PSE_CFG_PORT_ID_MASK | PSE_CFG_QUEUE_ID_MASK |
		      PSE_CFG_WR_EN_MASK | PSE_CFG_OQRSV_SEL_MASK,
		      FIELD_PREP(PSE_CFG_PORT_ID_MASK, port) |
		      FIELD_PREP(PSE_CFG_QUEUE_ID_MASK, queue) |
		      PSE_CFG_WR_EN_MASK | PSE_CFG_OQRSV_SEL_MASK);
}

static u32 airoha_fe_get_pse_all_rsv(struct airoha_eth *eth)
{
	u32 val = airoha_fe_rr(eth, REG_FE_PSE_BUF_SET);

	return FIELD_GET(PSE_ALLRSV_MASK, val);
}

static void airoha_fe_set_pse_oq_rsv(struct airoha_eth *eth,
				     u32 port, u32 queue, u32 val)
{
	u32 orig_val = airoha_fe_get_pse_queue_rsv_pages(eth, port, queue);
	u32 tmp, all_rsv, fq_limit;

	airoha_fe_set_pse_queue_rsv_pages(eth, port, queue, val);

	all_rsv = airoha_fe_get_pse_all_rsv(eth);
	all_rsv += (val - orig_val);
	airoha_fe_rmw(eth, REG_FE_PSE_BUF_SET, PSE_ALLRSV_MASK,
		      FIELD_PREP(PSE_ALLRSV_MASK, all_rsv));

	tmp = airoha_fe_rr(eth, PSE_FQ_CFG);
	fq_limit = FIELD_GET(PSE_FQ_LIMIT_MASK, tmp);
	tmp = fq_limit - all_rsv - 0x20;
	airoha_fe_rmw(eth, REG_PSE_SHARE_USED_THD,
		      PSE_SHARE_USED_HTHD_MASK,
		      FIELD_PREP(PSE_SHARE_USED_HTHD_MASK, tmp));

	tmp = fq_limit - all_rsv - 0x100;
	airoha_fe_rmw(eth, REG_PSE_SHARE_USED_THD,
		      PSE_SHARE_USED_MTHD_MASK,
		      FIELD_PREP(PSE_SHARE_USED_MTHD_MASK, tmp));
	tmp = (3 * tmp) >> 2;
	airoha_fe_rmw(eth, REG_FE_PSE_BUF_SET,
		      PSE_SHARE_USED_LTHD_MASK,
		      FIELD_PREP(PSE_SHARE_USED_LTHD_MASK, tmp));
}

static void airoha_fe_pse_ports_init(struct airoha_eth *eth)
{
	static const u8 pse_port_num_queues[] = {
		[FE_PSE_PORT_CDM1] = 6,
		[FE_PSE_PORT_GDM1] = 6,
		[FE_PSE_PORT_GDM2] = 32,
		[FE_PSE_PORT_GDM3] = 6,
		[FE_PSE_PORT_PPE1] = 4,
		[FE_PSE_PORT_CDM2] = 6,
		[FE_PSE_PORT_CDM3] = 8,
		[FE_PSE_PORT_CDM4] = 10,
		[FE_PSE_PORT_PPE2] = 4,
		[FE_PSE_PORT_GDM4] = 2,
		[FE_PSE_PORT_CDM5] = 2,
	};
	u32 all_rsv;
	int q;

	all_rsv = airoha_fe_get_pse_all_rsv(eth);
	all_rsv += PSE_QUEUE_RSV_PAGES * pse_port_num_queues[FE_PSE_PORT_PPE2];
	airoha_fe_set(eth, REG_FE_PSE_BUF_SET, all_rsv);

	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_CDM1]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_CDM1, q,
					 PSE_QUEUE_RSV_PAGES);
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_GDM1]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_GDM1, q,
					 PSE_QUEUE_RSV_PAGES);
	for (q = 6; q < pse_port_num_queues[FE_PSE_PORT_GDM2]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_GDM2, q, 0);
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_GDM3]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_GDM3, q,
					 PSE_QUEUE_RSV_PAGES);
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_PPE1]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_PPE1, q,
					 PSE_QUEUE_RSV_PAGES);
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_CDM2]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_CDM2, q,
					 PSE_QUEUE_RSV_PAGES);
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_CDM3] - 1; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_CDM3, q, 0);
	for (q = 4; q < pse_port_num_queues[FE_PSE_PORT_CDM4]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_CDM4, q,
					 PSE_QUEUE_RSV_PAGES);
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_PPE2] / 2; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_PPE2, q,
					 PSE_QUEUE_RSV_PAGES);
	for (q = pse_port_num_queues[FE_PSE_PORT_PPE2] / 2;
	     q < pse_port_num_queues[FE_PSE_PORT_PPE2]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_PPE2, q, 0);
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_GDM4]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_GDM4, q,
					 PSE_QUEUE_RSV_PAGES);
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_CDM5]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_CDM5, q,
					 PSE_QUEUE_RSV_PAGES);
}

static int airoha_fe_init(struct airoha_eth *eth)
{
	airoha_fe_maccr_init(eth);

	airoha_fe_rmw(eth, REG_PSE_IQ_REV1, PSE_IQ_RES1_P2_MASK,
		      FIELD_PREP(PSE_IQ_RES1_P2_MASK, 0x10));
	airoha_fe_rmw(eth, REG_PSE_IQ_REV2,
		      PSE_IQ_RES2_P5_MASK | PSE_IQ_RES2_P4_MASK,
		      FIELD_PREP(PSE_IQ_RES2_P5_MASK, 0x40) |
		      FIELD_PREP(PSE_IQ_RES2_P4_MASK, 0x34));

	airoha_fe_wr(eth, REG_FE_PCE_CFG,
		     PCE_DPI_EN_MASK | PCE_KA_EN_MASK | PCE_MC_EN_MASK);
	airoha_fe_rmw(eth, REG_CDM1_FWD_CFG, CDM1_VIP_QSEL_MASK,
		      FIELD_PREP(CDM1_VIP_QSEL_MASK, 0x4));
	airoha_fe_rmw(eth, REG_CDM2_FWD_CFG, CDM2_VIP_QSEL_MASK,
		      FIELD_PREP(CDM2_VIP_QSEL_MASK, 0x4));
	airoha_set_gdm_port_fwd_cfg(eth, REG_GDM_FWD_CFG(1),
				    FE_PSE_PORT_PPE1);
	airoha_fe_clear(eth, REG_GDM_INGRESS_CFG(1), GDM_STAG_EN_MASK);
	airoha_fe_rmw(eth, REG_GDM_LEN_CFG(1),
		      GDM_SHORT_LEN_MASK | GDM_LONG_LEN_MASK,
		      FIELD_PREP(GDM_SHORT_LEN_MASK, 60) |
		      FIELD_PREP(GDM_LONG_LEN_MASK, AIROHA_MAX_PACKET_SIZE));

	/*
	 * U-Boot only needs a minimal data path for recovery traffic, but the
	 * stock Linux driver still programs a handful of FE registers that are
	 * required to make EN7581 ports come out of reset reliably.
	 */
	airoha_fe_rmw(eth, REG_GDM4_SRC_PORT_SET,
		      GDM4_SPORT_OFF2_MASK |
		      GDM4_SPORT_OFF1_MASK |
		      GDM4_SPORT_OFF0_MASK,
		      FIELD_PREP(GDM4_SPORT_OFF2_MASK, 8) |
		      FIELD_PREP(GDM4_SPORT_OFF1_MASK, 8) |
		      FIELD_PREP(GDM4_SPORT_OFF0_MASK, 8));

	airoha_fe_rmw(eth, REG_FE_DMA_GLO_CFG,
		      FE_DMA_GLO_L2_SPACE_MASK | FE_DMA_GLO_PG_SZ_MASK,
		      FIELD_PREP(FE_DMA_GLO_L2_SPACE_MASK, 2) |
		      FE_DMA_GLO_PG_SZ_MASK);
	airoha_fe_wr(eth, REG_FE_RST_GLO_CFG,
		     FE_RST_CORE_MASK | FE_RST_GDM3_MBI_ARB_MASK |
		     FE_RST_GDM4_MBI_ARB_MASK);
	udelay(2000);

	/* Route RxRing1/RxRing15 to the alternate OQ used by the FE path. */
	airoha_fe_wr(eth, REG_FE_CDM1_OQ_MAP0, BIT(4));
	airoha_fe_wr(eth, REG_FE_CDM1_OQ_MAP1, BIT(28));
	airoha_fe_wr(eth, REG_FE_CDM1_OQ_MAP2, BIT(4));
	airoha_fe_wr(eth, REG_FE_CDM1_OQ_MAP3, BIT(28));

	airoha_fe_pse_ports_init(eth);

	airoha_fe_set(eth, REG_GDM_MISC_CFG,
		      GDM2_RDM_ACK_WAIT_PREF_MASK |
		      GDM2_CHN_VLD_MODE_MASK);
	airoha_fe_rmw(eth, REG_CDM2_FWD_CFG, CDM2_OAM_QSEL_MASK,
		      FIELD_PREP(CDM2_OAM_QSEL_MASK, 15));
	airoha_fe_set(eth, REG_GDM3_FWD_CFG, GDM3_PAD_EN_MASK);
	airoha_fe_set(eth, REG_GDM4_FWD_CFG, GDM4_PAD_EN_MASK);

	airoha_fe_clear(eth, REG_FE_CPORT_CFG, FE_CPORT_QUEUE_XFC_MASK);
	airoha_fe_set(eth, REG_FE_CPORT_CFG, FE_CPORT_PORT_XFC_MASK);

	airoha_fe_rmw(eth, REG_GDM2_CHN_RLS,
		      MBI_RX_AGE_SEL_MASK | MBI_TX_AGE_SEL_MASK,
		      FIELD_PREP(MBI_RX_AGE_SEL_MASK, 3) |
		      FIELD_PREP(MBI_TX_AGE_SEL_MASK, 3));

	/* IFC interferes with the simple recovery datapath, keep it disabled. */
	airoha_fe_clear(eth, REG_FE_CSR_IFC_CFG, FE_IFC_EN_MASK);

	return 0;
}

static void airoha_reset_ext_phys(ofnode mdio_node)
{
	ofnode child;

	ofnode_for_each_subnode(child, mdio_node) {
		struct gpio_desc reset = { 0 };
		u32 assert_us, deassert_us;
		int ret;

		ret = gpio_request_by_name_nodev(child, "reset-gpios", 0, &reset,
						 GPIOD_IS_OUT |
						 GPIOD_IS_OUT_ACTIVE);
		if (ret)
			continue;

		assert_us = ofnode_read_u32_default(child, "reset-assert-us",
						     10000);
		deassert_us = ofnode_read_u32_default(child, "reset-deassert-us",
						       10000);

		udelay(assert_us);
		dm_gpio_set_value(&reset, 0);
		udelay(deassert_us);
		gpio_free_list_nodev(&reset, 1);
	}
}

static void airoha_qdma_reset_rx_desc(struct airoha_queue *q, int index)
{
	struct airoha_qdma_desc *desc;
	uchar *rx_packet;
	u32 val;

	desc = &q->desc[index];
	rx_packet = net_rx_packets[index];
	index = (index + 1) % q->ndesc;

	dma_map_single(rx_packet, PKTSIZE_ALIGN, DMA_TO_DEVICE);

	WRITE_ONCE(desc->msg0, cpu_to_le32(0));
	WRITE_ONCE(desc->msg1, cpu_to_le32(0));
	WRITE_ONCE(desc->msg2, cpu_to_le32(0));
	WRITE_ONCE(desc->msg3, cpu_to_le32(0));
	WRITE_ONCE(desc->addr, cpu_to_le32(virt_to_phys(rx_packet)));
	WRITE_ONCE(desc->data, cpu_to_le32(index));
	val = FIELD_PREP(QDMA_DESC_LEN_MASK, PKTSIZE_ALIGN);
	WRITE_ONCE(desc->ctrl, cpu_to_le32(val));

	dma_map_unaligned(desc, sizeof(*desc), DMA_TO_DEVICE);
}

static void airoha_qdma_init_rx_desc(struct airoha_queue *q)
{
	int i;

	for (i = 0; i < q->ndesc; i++)
		airoha_qdma_reset_rx_desc(q, i);
}

static int airoha_qdma_init_rx_queue(struct airoha_queue *q,
				     struct airoha_qdma *qdma, int ndesc)
{
	int qid = q - &qdma->q_rx[0];
	unsigned long dma_addr;

	q->ndesc = ndesc;
	q->head = 0;

	q->desc = dma_alloc_coherent(q->ndesc * sizeof(*q->desc), &dma_addr);
	if (!q->desc)
		return -ENOMEM;

	memset(q->desc, 0, q->ndesc * sizeof(*q->desc));
	dma_map_single(q->desc, q->ndesc * sizeof(*q->desc), DMA_TO_DEVICE);

	airoha_qdma_wr(qdma, REG_RX_RING_BASE(qid), dma_addr);
	airoha_qdma_rmw(qdma, REG_RX_RING_SIZE(qid),
			RX_RING_SIZE_MASK,
			FIELD_PREP(RX_RING_SIZE_MASK, ndesc));

	airoha_qdma_rmw(qdma, REG_RX_RING_SIZE(qid), RX_RING_THR_MASK,
			FIELD_PREP(RX_RING_THR_MASK, 0));
	airoha_qdma_rmw(qdma, REG_RX_CPU_IDX(qid), RX_RING_CPU_IDX_MASK,
			FIELD_PREP(RX_RING_CPU_IDX_MASK, q->ndesc - 1));
	airoha_qdma_rmw(qdma, REG_RX_DMA_IDX(qid), RX_RING_DMA_IDX_MASK,
			FIELD_PREP(RX_RING_DMA_IDX_MASK, q->head));

	return 0;
}

static int airoha_qdma_init_rx(struct airoha_qdma *qdma)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(qdma->q_rx); i++) {
		int err;

		err = airoha_qdma_init_rx_queue(&qdma->q_rx[i], qdma,
						RX_DSCP_NUM);
		if (err)
			return err;
	}

	return 0;
}

static int airoha_qdma_init_tx_queue(struct airoha_queue *q,
				     struct airoha_qdma *qdma, int size)
{
	int qid = q - &qdma->q_tx[0];
	unsigned long dma_addr;

	q->ndesc = size;
	q->head = 0;

	q->desc = dma_alloc_coherent(q->ndesc * sizeof(*q->desc), &dma_addr);
	if (!q->desc)
		return -ENOMEM;

	memset(q->desc, 0, q->ndesc * sizeof(*q->desc));
	dma_map_single(q->desc, q->ndesc * sizeof(*q->desc), DMA_TO_DEVICE);

	airoha_qdma_wr(qdma, REG_TX_RING_BASE(qid), dma_addr);
	airoha_qdma_rmw(qdma, REG_TX_CPU_IDX(qid), TX_RING_CPU_IDX_MASK,
			FIELD_PREP(TX_RING_CPU_IDX_MASK, q->head));
	airoha_qdma_rmw(qdma, REG_TX_DMA_IDX(qid), TX_RING_DMA_IDX_MASK,
			FIELD_PREP(TX_RING_DMA_IDX_MASK, q->head));

	return 0;
}

static int airoha_qdma_tx_irq_init(struct airoha_tx_irq_queue *irq_q,
				   struct airoha_qdma *qdma, int size)
{
	int id = irq_q - &qdma->q_tx_irq[0];
	unsigned long dma_addr;

	irq_q->q = dma_alloc_coherent(size * sizeof(u32), &dma_addr);
	if (!irq_q->q)
		return -ENOMEM;

	memset(irq_q->q, 0xffffffff, size * sizeof(u32));
	irq_q->size = size;
	irq_q->qdma = qdma;

	dma_map_single(irq_q->q, size * sizeof(u32), DMA_TO_DEVICE);

	airoha_qdma_wr(qdma, REG_TX_IRQ_BASE(id), dma_addr);
	airoha_qdma_rmw(qdma, REG_TX_IRQ_CFG(id), TX_IRQ_DEPTH_MASK,
			FIELD_PREP(TX_IRQ_DEPTH_MASK, size));

	return 0;
}

static int airoha_qdma_init_tx(struct airoha_qdma *qdma)
{
	int i, err;

	for (i = 0; i < ARRAY_SIZE(qdma->q_tx_irq); i++) {
		err = airoha_qdma_tx_irq_init(&qdma->q_tx_irq[i], qdma,
					      IRQ_QUEUE_LEN);
		if (err)
			return err;
	}

	for (i = 0; i < ARRAY_SIZE(qdma->q_tx); i++) {
		err = airoha_qdma_init_tx_queue(&qdma->q_tx[i], qdma,
						TX_DSCP_NUM);
		if (err)
			return err;
	}

	return 0;
}

static int airoha_qdma_init_hfwd_queues(struct airoha_qdma *qdma)
{
	unsigned long dma_addr;
	u32 status;
	int size;

	size = HW_DSCP_NUM * sizeof(struct airoha_qdma_fwd_desc);
	qdma->hfwd.desc = dma_alloc_coherent(size, &dma_addr);
	if (!qdma->hfwd.desc)
		return -ENOMEM;

	memset(qdma->hfwd.desc, 0, size);
	dma_map_single(qdma->hfwd.desc, size, DMA_TO_DEVICE);

	airoha_qdma_wr(qdma, REG_FWD_DSCP_BASE, dma_addr);

	size = AIROHA_MAX_PACKET_SIZE * HW_DSCP_NUM;
	qdma->hfwd.q = dma_alloc_coherent(size, &dma_addr);
	if (!qdma->hfwd.q)
		return -ENOMEM;

	memset(qdma->hfwd.q, 0, size);
	dma_map_single(qdma->hfwd.q, size, DMA_TO_DEVICE);

	airoha_qdma_wr(qdma, REG_FWD_BUF_BASE, dma_addr);

	airoha_qdma_rmw(qdma, REG_HW_FWD_DSCP_CFG,
			HW_FWD_DSCP_PAYLOAD_SIZE_MASK |
			HW_FWD_DSCP_MIN_SCATTER_LEN_MASK,
			FIELD_PREP(HW_FWD_DSCP_PAYLOAD_SIZE_MASK, 0) |
			FIELD_PREP(HW_FWD_DSCP_MIN_SCATTER_LEN_MASK, 1));
	airoha_qdma_rmw(qdma, REG_LMGR_INIT_CFG,
			LMGR_INIT_START | LMGR_SRAM_MODE_MASK |
			HW_FWD_DESC_NUM_MASK,
			FIELD_PREP(HW_FWD_DESC_NUM_MASK, HW_DSCP_NUM) |
			LMGR_INIT_START);

	udelay(1000);
	return read_poll_timeout(airoha_qdma_rr, status,
				 !(status & LMGR_INIT_START), USEC_PER_MSEC,
				 30 * USEC_PER_MSEC, qdma,
				 REG_LMGR_INIT_CFG);
}

static int airoha_qdma_hw_init(struct airoha_qdma *qdma)
{
	int i;

	/* clear pending irqs */
	for (i = 0; i < 2; i++)
		airoha_qdma_wr(qdma, REG_INT_STATUS(i), 0xffffffff);

	airoha_qdma_wr(qdma, REG_QDMA_GLOBAL_CFG,
		       GLOBAL_CFG_CPU_TXR_RR_MASK |
		       GLOBAL_CFG_PAYLOAD_BYTE_SWAP_MASK |
		       GLOBAL_CFG_IRQ0_EN_MASK |
		       GLOBAL_CFG_TX_WB_DONE_MASK |
		       FIELD_PREP(GLOBAL_CFG_MAX_ISSUE_NUM_MASK, 3));

	/* disable qdma rx delay interrupt */
	for (i = 0; i < ARRAY_SIZE(qdma->q_rx); i++) {
		if (!qdma->q_rx[i].ndesc)
			continue;

		airoha_qdma_clear(qdma, REG_RX_DELAY_INT_IDX(i),
				  RX_DELAY_INT_MASK);
	}

	return 0;
}

static int airoha_qdma_init(struct udevice *dev,
			    struct airoha_eth *eth,
			    struct airoha_qdma *qdma)
{
	int err;

	qdma->eth = eth;
	qdma->regs = dev_remap_addr_name(dev, "qdma0");
	if (IS_ERR(qdma->regs))
		return PTR_ERR(qdma->regs);

	err = airoha_qdma_init_rx(qdma);
	if (err)
		return err;

	err = airoha_qdma_init_tx(qdma);
	if (err)
		return err;

	err = airoha_qdma_init_hfwd_queues(qdma);
	if (err)
		return err;

	return airoha_qdma_hw_init(qdma);
}

static int airoha_hw_init(struct udevice *dev,
			  struct airoha_eth *eth)
{
	int ret, i;

	/* disable xsi */
	ret = reset_assert_bulk(&eth->xsi_rsts);
	if (ret)
		return ret;

	if (eth->has_switch_rst) {
		ret = reset_assert(&eth->switch_rst);
		if (ret)
			return ret;
	}

	ret = reset_assert_bulk(&eth->rsts);
	if (ret)
		return ret;

	mdelay(20);

	ret = reset_deassert_bulk(&eth->rsts);
	if (ret)
		return ret;

	if (eth->has_switch_rst) {
		ret = reset_deassert(&eth->switch_rst);
		if (ret)
			return ret;
	}

	mdelay(20);

	ret = airoha_fe_init(eth);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(eth->qdma); i++) {
		ret = airoha_qdma_init(dev, eth, &eth->qdma[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static int airoha_switch_init(struct udevice *dev, struct airoha_eth *eth)
{
	struct airoha_eth_soc_data *data = (void *)dev_get_driver_data(dev);
	ofnode mdio_node, switch_node;
	u32 phy_poll_start, phy_poll_end;
	u32 cpu_port_mask = BIT(6);
	u32 user_port_mask = GENMASK(5, 0);
	u32 switch_pmcr;
	fdt_addr_t addr;
	int port;

	switch_node = ofnode_by_compatible(ofnode_null(),
					   data->switch_compatible);
	if (!ofnode_valid(switch_node))
		return -EINVAL;

	addr = ofnode_get_addr(switch_node);
	if (addr == FDT_ADDR_T_NONE)
		return -ENOMEM;

	/* Switch doesn't have a DEV, gets address and setup Flood and CPU port */
	eth->switch_regs = map_sysmem(addr, 0);

	/*
	 * Forward unknown/broadcast traffic to the CPU port only. Without
	 * this, recovery packets may get trapped inside the switch fabric.
	 */
	airoha_switch_wr(eth, SWITCH_MFC,
			 FIELD_PREP(SWITCH_BC_FFP, cpu_port_mask) |
			 FIELD_PREP(SWITCH_UNM_FFP, cpu_port_mask) |
			 FIELD_PREP(SWITCH_UNU_FFP, cpu_port_mask));

	/* Treat port 6 as the dedicated CPU port. */
	airoha_switch_rmw(eth, SWITCH_CFC, SWITCH_CPU_PMAP,
			  FIELD_PREP(SWITCH_CPU_PMAP, cpu_port_mask));
	airoha_switch_rmw(eth, SWITCH_AGC, 0, SWITCH_LOCAL_EN);
	airoha_switch_wr(eth, SWITCH_CPORT_SPTAG_CFG,
			 SWITCH_SW2FE_STAG_EN | SWITCH_FE2SW_STAG_EN);

	switch_pmcr = SWITCH_IPG_CFG_SHORT | SWITCH_MAC_MODE |
		      SWITCH_FORCE_MODE | SWITCH_MAC_TX_EN |
		      SWITCH_MAC_RX_EN | SWITCH_BKOFF_EN | SWITCH_BKPR_EN |
		      SWITCH_FORCE_RX_FC | SWITCH_FORCE_TX_FC |
		      SWITCH_FORCE_SPD_1000 | SWITCH_FORCE_DPX |
		      SWITCH_FORCE_LNK;

	/* Set CPU 6 PMCR */
	airoha_switch_wr(eth, SWITCH_PMCR(6), switch_pmcr);

	/*
	 * XR1710G recovery traffic uses the internal switch user ports that map
	 * to lan2/lan3. The stock reset state leaves their MACs partially
	 * disabled; force them up so broadcast ARP can actually egress.
	 */
	airoha_switch_wr(eth, SWITCH_PMCR(1), switch_pmcr);
	airoha_switch_wr(eth, SWITCH_PMCR(2), switch_pmcr);

	/*
	 * Match Linux switch setup: user ports only talk to the CPU port,
	 * while the CPU port can reach all user ports. The switch-specific
	 * tag stays in the DMA metadata rather than the frame payload.
	 */
	for (port = 0; port <= 6; port++) {
		u32 matrix = port == 6 ? user_port_mask : cpu_port_mask;
		u32 pvc = FIELD_PREP(SWITCH_STAG_VPID, 0x8100) |
			  FIELD_PREP(SWITCH_VLAN_ATTR, SWITCH_VLAN_ATTR_USER);

		if (port == 6)
			pvc |= SWITCH_PORT_SPEC_TAG;

		airoha_switch_rmw(eth, SWITCH_PCR(port),
				  SWITCH_PORT_MATRIX | SWITCH_PORT_VLAN_MASK,
				  FIELD_PREP(SWITCH_PORT_MATRIX, matrix) |
				  FIELD_PREP(SWITCH_PORT_VLAN_MASK,
					     SWITCH_PORT_FALLBACK_MODE));
		airoha_switch_rmw(eth, SWITCH_PVC(port),
				  SWITCH_STAG_VPID | SWITCH_VLAN_ATTR |
				  SWITCH_PORT_SPEC_TAG, pvc);
	}

	phy_poll_start = ofnode_read_u32_default(switch_node,
						 "airoha,phy-poll-start", 0x8);
	phy_poll_end = ofnode_read_u32_default(switch_node,
					       "airoha,phy-poll-end", 0xc);
	if (phy_poll_end < phy_poll_start)
		phy_poll_end = phy_poll_start;

	/*
	 * Some boards hang external ports off non-default MDIO addresses.
	 * Allow the DTS to override the switch PHY auto-poll window so link
	 * state is updated for the actual recovery-facing ports.
	 */
	airoha_switch_wr(eth, SWITCH_PHY_POLL,
			 FIELD_PREP(SWITCH_PHY_AP_EN, 0x7f) |
			 FIELD_PREP(SWITCH_EEE_POLL_EN, 0x7f) |
			 SWITCH_PHY_PRE_EN |
			 FIELD_PREP(SWITCH_PHY_END_ADDR, phy_poll_end) |
			 FIELD_PREP(SWITCH_PHY_ST_ADDR, phy_poll_start));

	mdio_node = ofnode_find_subnode(switch_node, "mdio");
	if (ofnode_valid(mdio_node))
		airoha_reset_ext_phys(mdio_node);

	return 0;
}

static int airoha_eth_probe(struct udevice *dev)
{
	struct airoha_eth_soc_data *data = (void *)dev_get_driver_data(dev);
	struct airoha_eth *eth = dev_get_priv(dev);
	ofnode switch_node;
	struct regmap *scu_regmap;
	int i, ret;

	scu_regmap = airoha_get_scu_regmap();
	if (IS_ERR(scu_regmap))
		return PTR_ERR(scu_regmap);

	/* It seems by default the FEMEM_SEL is set to Memory (0x1)
	 * preventing any access to any QDMA and FrameEngine register
	 * reporting all 0xdeadbeef (poor cow :( )
	 */
	regmap_write(scu_regmap, SCU_SHARE_FEMEM_SEL, 0x0);

	eth->fe_regs = dev_remap_addr_name(dev, "fe");
	if (!eth->fe_regs)
		return -ENOMEM;

	eth->rsts.resets = devm_kcalloc(dev, AIROHA_MAX_NUM_RSTS,
					sizeof(struct reset_ctl), GFP_KERNEL);
	if (!eth->rsts.resets)
		return -ENOMEM;
	eth->rsts.count = AIROHA_MAX_NUM_RSTS;

	eth->xsi_rsts.resets = devm_kcalloc(dev, data->num_xsi_rsts,
					    sizeof(struct reset_ctl), GFP_KERNEL);
	if (!eth->xsi_rsts.resets)
		return -ENOMEM;
	eth->xsi_rsts.count = data->num_xsi_rsts;

	ret = reset_get_by_name(dev, "fe", &eth->rsts.resets[0]);
	if (ret)
		return ret;

	ret = reset_get_by_name(dev, "pdma", &eth->rsts.resets[1]);
	if (ret)
		return ret;

	ret = reset_get_by_name(dev, "qdma", &eth->rsts.resets[2]);
	if (ret)
		return ret;

	for (i = 0; i < data->num_xsi_rsts; i++) {
		ret = reset_get_by_name(dev, data->xsi_rsts_names[i],
					&eth->xsi_rsts.resets[i]);
		if (ret)
			return ret;
	}

	switch_node = ofnode_by_compatible(ofnode_null(),
					   data->switch_compatible);
	if (ofnode_valid(switch_node)) {
		ret = reset_get_by_index_nodev(switch_node, 0, &eth->switch_rst);
		if (!ret)
			eth->has_switch_rst = true;
		else if (ret != -ENOENT && ret != -ENODEV &&
			 ret != -ENOSYS && ret != -ENOTSUPP)
			return ret;
	}

	ret = airoha_hw_init(dev, eth);
	if (ret)
		return ret;

	return airoha_switch_init(dev, eth);
}

static int airoha_eth_init(struct udevice *dev)
{
	struct airoha_eth *eth = dev_get_priv(dev);
	struct airoha_qdma *qdma = &eth->qdma[0];
	struct airoha_queue *q;
	int qid;

	qid = 0;
	q = &qdma->q_rx[qid];

	airoha_qdma_init_rx_desc(q);

	airoha_qdma_set(qdma, REG_QDMA_GLOBAL_CFG,
			GLOBAL_CFG_TX_DMA_EN_MASK |
			GLOBAL_CFG_RX_DMA_EN_MASK);

	return 0;
}

static void airoha_eth_stop(struct udevice *dev)
{
	struct airoha_eth *eth = dev_get_priv(dev);
	struct airoha_qdma *qdma = &eth->qdma[0];

	airoha_qdma_clear(qdma, REG_QDMA_GLOBAL_CFG,
			  GLOBAL_CFG_TX_DMA_EN_MASK |
			  GLOBAL_CFG_RX_DMA_EN_MASK);
}

static int airoha_eth_send(struct udevice *dev, void *packet, int length)
{
	struct airoha_eth *eth = dev_get_priv(dev);
	struct airoha_qdma *qdma = &eth->qdma[0];
	struct airoha_qdma_desc *desc;
	struct airoha_queue *q;
	dma_addr_t dma_addr;
	u32 msg0, msg1;
	int qid, index;
	u8 fport;
	u32 val;
	int i;

	/*
	 * There is no need to pad short TX packets to 60 bytes since the
	 * GDM_PAD_EN bit set in the corresponding REG_GDM_FWD_CFG(n) register.
	 */

	dma_addr = dma_map_single(packet, length, DMA_TO_DEVICE);

	qid = 0;
	q = &qdma->q_tx[qid];
	desc = &q->desc[q->head];
	index = (q->head + 1) % q->ndesc;

	fport = 1;

	msg0 = 0;
	msg1 = FIELD_PREP(QDMA_ETH_TXMSG_FPORT_MASK, fport) |
	       FIELD_PREP(QDMA_ETH_TXMSG_METER_MASK, 0x7f);

	val = FIELD_PREP(QDMA_DESC_LEN_MASK, length);
	WRITE_ONCE(desc->ctrl, cpu_to_le32(val));
	WRITE_ONCE(desc->addr, cpu_to_le32(dma_addr));
	val = FIELD_PREP(QDMA_DESC_NEXT_ID_MASK, index);
	WRITE_ONCE(desc->data, cpu_to_le32(val));
	WRITE_ONCE(desc->msg0, cpu_to_le32(msg0));
	WRITE_ONCE(desc->msg1, cpu_to_le32(msg1));
	WRITE_ONCE(desc->msg2, cpu_to_le32(0xffff));

	dma_map_unaligned(desc, sizeof(*desc), DMA_TO_DEVICE);

	airoha_qdma_rmw(qdma, REG_TX_CPU_IDX(qid), TX_RING_CPU_IDX_MASK,
			FIELD_PREP(TX_RING_CPU_IDX_MASK, index));

	for (i = 0; i < 100; i++) {
		dma_unmap_unaligned(virt_to_phys(desc), sizeof(*desc),
				    DMA_FROM_DEVICE);
		if (desc->ctrl & QDMA_DESC_DONE_MASK)
			break;

		udelay(1);
	}

	/* Return error if for some reason the descriptor never ACK */
	if (!(desc->ctrl & QDMA_DESC_DONE_MASK))
		return -EAGAIN;

	q->head = index;
	airoha_qdma_rmw(qdma, REG_IRQ_CLEAR_LEN(0),
			IRQ_CLEAR_LEN_MASK, 1);

	return 0;
}

static int airoha_eth_recv(struct udevice *dev, int flags, uchar **packetp)
{
	struct airoha_eth *eth = dev_get_priv(dev);
	struct airoha_qdma *qdma = &eth->qdma[0];
	struct airoha_qdma_desc *desc;
	struct airoha_queue *q;
	u16 length;
	int qid;

	qid = 0;
	q = &qdma->q_rx[qid];
	desc = &q->desc[q->head];

	dma_unmap_unaligned(virt_to_phys(desc), sizeof(*desc),
			    DMA_FROM_DEVICE);

	if (!(desc->ctrl & QDMA_DESC_DONE_MASK))
		return -EAGAIN;

	length = FIELD_GET(QDMA_DESC_LEN_MASK, desc->ctrl);
	dma_unmap_single(desc->addr, length,
			 DMA_FROM_DEVICE);

	*packetp = phys_to_virt(desc->addr);

	return length;
}

static int arht_eth_free_pkt(struct udevice *dev, uchar *packet, int length)
{
	struct airoha_eth *eth = dev_get_priv(dev);
	struct airoha_qdma *qdma = &eth->qdma[0];
	struct airoha_queue *q;
	int qid;

	if (!packet)
		return 0;

	qid = 0;
	q = &qdma->q_rx[qid];

	/*
	 * Due to cpu cache issue the airoha_qdma_reset_rx_desc() function
	 * will always touch 2 descriptors placed on the same cacheline:
	 *   - if current descriptor is even, then current and next
	 *     descriptors will be touched
	 *   - if current descriptor is odd, then current and previous
	 *     descriptors will be touched
	 *
	 * Thus, to prevent possible destroying of rx queue, we should:
	 *   - do nothing in the even descriptor case,
	 *   - utilize 2 descriptors (current and previous one) in the
	 *     odd descriptor case.
	 *
	 * WARNING: Observations shows that PKTBUFSRX must be even and
	 *          larger than 7 for reliable driver operations.
	 */
	if (q->head & 0x01) {
		airoha_qdma_reset_rx_desc(q, q->head - 1);
		airoha_qdma_reset_rx_desc(q, q->head);

		airoha_qdma_rmw(qdma, REG_RX_CPU_IDX(qid), RX_RING_CPU_IDX_MASK,
				FIELD_PREP(RX_RING_CPU_IDX_MASK, q->head));
	}

	q->head = (q->head + 1) % q->ndesc;

	return 0;
}

static int arht_eth_write_hwaddr(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct airoha_eth *eth = dev_get_priv(dev);
	unsigned char *mac = pdata->enetaddr;
	u32 macaddr_lsb, macaddr_msb;

	macaddr_lsb = FIELD_PREP(SMACCR0_MAC2, mac[2]) |
		      FIELD_PREP(SMACCR0_MAC3, mac[3]) |
		      FIELD_PREP(SMACCR0_MAC4, mac[4]) |
		      FIELD_PREP(SMACCR0_MAC5, mac[5]);
	macaddr_msb = FIELD_PREP(SMACCR1_MAC1, mac[1]) |
		      FIELD_PREP(SMACCR1_MAC0, mac[0]);

	/* Set MAC for Switch */
	airoha_switch_wr(eth, SWITCH_SMACCR0, macaddr_lsb);
	airoha_switch_wr(eth, SWITCH_SMACCR1, macaddr_msb);

	return 0;
}

static int airoha_eth_bind(struct udevice *dev)
{
	struct airoha_eth_soc_data *data = (void *)dev_get_driver_data(dev);
	ofnode switch_node, mdio_node;
	struct udevice *mdio_dev;
	int ret = 0;

	if (!CONFIG_IS_ENABLED(MDIO_MT7531_MMIO))
		return 0;

	switch_node = ofnode_by_compatible(ofnode_null(),
					   data->switch_compatible);
	if (!ofnode_valid(switch_node)) {
		debug("Warning: missing switch node\n");
		return 0;
	}

	mdio_node = ofnode_find_subnode(switch_node, "mdio");
	if (!ofnode_valid(mdio_node)) {
		debug("Warning: missing mdio node\n");
		return 0;
	}

	ret = device_bind_driver_to_node(dev, "mt7531-mdio-mmio", "mdio",
					 mdio_node, &mdio_dev);
	if (ret)
		debug("Warning: failed to bind mdio controller\n");

	return 0;
}

static const struct airoha_eth_soc_data en7523_data = {
	.xsi_rsts_names = en7523_xsi_rsts_names,
	.num_xsi_rsts = ARRAY_SIZE(en7523_xsi_rsts_names),
	.switch_compatible = "airoha,en7523-switch",
};

static const struct airoha_eth_soc_data en7581_data = {
	.xsi_rsts_names = en7581_xsi_rsts_names,
	.num_xsi_rsts = ARRAY_SIZE(en7581_xsi_rsts_names),
	.switch_compatible = "airoha,en7581-switch",
};

static const struct udevice_id airoha_eth_ids[] = {
	{ .compatible = "airoha,en7523-eth",
	  .data = (ulong)&en7523_data,
	},
	{ .compatible = "airoha,en7581-eth",
	  .data = (ulong)&en7581_data,
	},
	{ }
};

static const struct eth_ops airoha_eth_ops = {
	.start = airoha_eth_init,
	.stop = airoha_eth_stop,
	.send = airoha_eth_send,
	.recv = airoha_eth_recv,
	.free_pkt = arht_eth_free_pkt,
	.write_hwaddr = arht_eth_write_hwaddr,
};

U_BOOT_DRIVER(airoha_eth) = {
	.name = "airoha-eth",
	.id = UCLASS_ETH,
	.of_match = airoha_eth_ids,
	.probe = airoha_eth_probe,
	.bind = airoha_eth_bind,
	.ops = &airoha_eth_ops,
	.priv_auto = sizeof(struct airoha_eth),
	.plat_auto = sizeof(struct eth_pdata),
};
