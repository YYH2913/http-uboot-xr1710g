// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include "nss-switch.h"

#define CMN_BLK_ADDR			0x0009B780
#define FREQUENCY_MASK			0xfffffdf0
#define INTERNAL_48MHZ_CLOCK		0x7

u32 nb_vsi_config[CONFIG_ETH_MAX_MAC] = {
				0x03, 0x05, 0x09, 0x11, 0x21, 0x41};

/*
 * .id of the PHY TYPE
 * .clk_rate {10, 100, 1000, 10000, 2500, 5000}
 * .mac_mode {10, 100, 1000, 10000, 2500, 5000}
 * .modes {10, 100, 1000, 10000, 2500, 5000}
 * index are based on Mac speed
 */

static struct ipq_eth_port_config ipq9574_port_config[] = {
	{
		QCA8075_PHY_TYPE,
		{
			CLK_2_5_MHZ,
			CLK_25_MHZ,
			CLK_125_MHZ
		},
		{
			GMAC,
			GMAC,
			GMAC
		},
		{
			PORT_WRAPPER_PSGMII,
			PORT_WRAPPER_PSGMII,
			PORT_WRAPPER_PSGMII
		},
	}, {
		QCA8x8x_PHY_TYPE,
		{
			CLK_1_25_MHZ,
			CLK_12_5_MHZ,
			CLK_125_MHZ,
			-1,
			CLK_78_125_MHZ,
		},
		{
			XGMAC,
			XGMAC,
			XGMAC,
			-1,
			XGMAC
		},
		{
			PORT_WRAPPER_UQXGMII,
			PORT_WRAPPER_UQXGMII,
			PORT_WRAPPER_UQXGMII,
			-1,
			PORT_WRAPPER_UQXGMII,
		},
	}, {
		QCA8x8x_SWITCH_TYPE,
		{
			CLK_312_5_MHZ,
			CLK_312_5_MHZ,
			CLK_312_5_MHZ,
			-1,
			CLK_312_5_MHZ
		},
		{
			XGMAC,
			XGMAC,
			XGMAC,
			-1,
			XGMAC
		},
		{
			PORT_WRAPPER_SGMII_PLUS,
			PORT_WRAPPER_SGMII_PLUS,
			PORT_WRAPPER_SGMII_PLUS,
			-1,
			PORT_WRAPPER_SGMII_PLUS
		},
	}, {
		QCA8081_PHY_TYPE,
		{
			CLK_2_5_MHZ,
			CLK_25_MHZ,
			CLK_125_MHZ,
			-1,
			CLK_312_5_MHZ
		},
		{
			GMAC,
			GMAC,
			GMAC,
			-1,
			XGMAC
		},
		{
			PORT_WRAPPER_SGMII0_RGMII4,
			PORT_WRAPPER_SGMII0_RGMII4,
			PORT_WRAPPER_SGMII0_RGMII4,
			-1,
			PORT_WRAPPER_SGMII_PLUS
		},
	}, {
		SFP10G_PHY_TYPE,
		{
			CLK_312_5_MHZ,
			CLK_312_5_MHZ,
			CLK_312_5_MHZ,
			CLK_312_5_MHZ,
			CLK_312_5_MHZ,
			CLK_312_5_MHZ
		},
		{
			XGMAC,
			XGMAC,
			XGMAC,
			XGMAC,
			XGMAC,
			XGMAC
		},
		{
			PORT_WRAPPER_10GBASE_R,
			PORT_WRAPPER_10GBASE_R,
			PORT_WRAPPER_10GBASE_R,
			PORT_WRAPPER_10GBASE_R,
			PORT_WRAPPER_10GBASE_R,
			PORT_WRAPPER_10GBASE_R
		},
	}, {
		AQ_PHY_TYPE,
		{
			CLK_1_25_MHZ,
			CLK_12_5_MHZ,
			CLK_125_MHZ,
			CLK_312_5_MHZ,
			CLK_78_125_MHZ,
			CLK_156_25_MHZ,
		},
		{
			XGMAC,
			XGMAC,
			XGMAC,
			XGMAC,
			XGMAC,
			XGMAC
		},
		{
			PORT_WRAPPER_USXGMII,
			PORT_WRAPPER_USXGMII,
			PORT_WRAPPER_USXGMII,
			PORT_WRAPPER_USXGMII,
			PORT_WRAPPER_USXGMII,
			PORT_WRAPPER_USXGMII
		},
	}, {
		RTL8261BE_PHY_TYPE,
		{
			CLK_1_25_MHZ,
			CLK_12_5_MHZ,
			CLK_125_MHZ,
			CLK_312_5_MHZ,
			CLK_78_125_MHZ,
			CLK_156_25_MHZ,
		},
		{
			XGMAC,
			XGMAC,
			XGMAC,
			XGMAC,
			XGMAC,
			XGMAC
		},
		{
			PORT_WRAPPER_USXGMII,
			PORT_WRAPPER_USXGMII,
			PORT_WRAPPER_USXGMII,
			PORT_WRAPPER_USXGMII,
			PORT_WRAPPER_USXGMII,
			PORT_WRAPPER_USXGMII
		},
	}, {
		QCA81xx_PHY_TYPE,
		{
			-1,
			CLK_12_5_MHZ,
			CLK_125_MHZ,
			CLK_312_5_MHZ,
			CLK_78_125_MHZ,
			CLK_156_25_MHZ,
		},
		{
			-1,
			XGMAC,
			XGMAC,
			XGMAC,
			XGMAC,
			XGMAC
		},
		{
			-1,
			PORT_WRAPPER_USXGMII,
			PORT_WRAPPER_USXGMII,
			PORT_WRAPPER_USXGMII,
			PORT_WRAPPER_USXGMII,
			PORT_WRAPPER_USXGMII
		},
	}, {
		QCE1204_PHY_TYPE,
		{
			CLK_1_25_MHZ,
			CLK_12_5_MHZ,
			CLK_125_MHZ,
			CLK_312_5_MHZ,
			CLK_78_125_MHZ,
		},
		{
			XGMAC,
			XGMAC,
			XGMAC,
			XGMAC,
			XGMAC
		},
		{
			PORT_WRAPPER_UQXGMII,
			PORT_WRAPPER_UQXGMII,
			PORT_WRAPPER_UQXGMII,
			PORT_WRAPPER_UQXGMII,
			PORT_WRAPPER_UQXGMII,
		},
	}, {
		UNUSED_PHY_TYPE,
	},
};

struct ipq_eth_port_config *port_config = ipq9574_port_config;

static struct ipq_tdm_config ipq9574_tdm_config[] = {
	{
		{0x26, 0x34, 0x25, 0x30, 0x21, 0x36, 0x20, 0x35, 0x26,
		0x31, 0x22, 0x36, 0x27, 0x30, 0x25, 0x32, 0x26, 0x36,
		0x20, 0x37, 0x24, 0x30, 0x23, 0x36, 0x26, 0x34, 0x25,
		0x33, 0x20, 0x36, 0x21, 0x35, 0x26, 0x30, 0x27, 0x31,
		0x20, 0x36, 0x25, 0x37, 0x26, 0x30, 0x22, 0x35, 0x20,
		0x36, 0x23, 0x32, 0x26, 0x30, 0x25, 0x33, 0x20, 0x36,
		0x27, 0x35, 0x26, 0x30, 0x24, 0x37, 0x20, 0x36, 0x25,
		0x34, 0x26, 0x30, 0x21, 0x35, 0x20, 0x36, 0x22, 0x31,
		0x26, 0x30, 0x25, 0x32, 0x20, 0x36, 0x27, 0x35, 0x26,
		0x30, 0x23, 0x37, 0x20, 0x36, 0x25, 0x33, 0x26, 0x30,
		0x24, 0x35, 0x20, 0x36, 0x21, 0x34, 0x26, 0x30, 0x25,
		0x31, 0x20, 0x36, 0x22, 0x35, 0x26, 0x30, 0x23, 0x32,
		0x20, 0x36, 0x25, 0x33, 0x26, 0x30, 0x24, 0x35, 0x20,
		0x36},
	},
	{
		{0x26, 0x36, 0x20, 0x30, 0x25, 0x35, 0x21, 0x31, 0x22,
		0x32, 0x20, 0x30, 0x26, 0x36, 0x23, 0x33, 0x25, 0x35,
		0x21, 0x31, 0x20, 0x30, 0x24, 0x34, 0x26, 0x36, 0x25,
		0x35, 0x20, 0x30, 0x21, 0x31, 0x27, 0x37, 0x22, 0x32,
		0x26, 0x36, 0x20, 0x30, 0x25, 0x35, 0x21, 0x31, 0x23,
		0x33, 0x20, 0x30, 0x26, 0x36, 0x24, 0x34, 0x25, 0x35,
		0x21, 0x31, 0x20, 0x30, 0x27, 0x37, 0x26, 0x36, 0x22,
		0x32, 0x25, 0x35, 0x20, 0x30, 0x21, 0x31, 0x23, 0x33,
		0x26, 0x36, 0x25, 0x35, 0x20, 0x30, 0x21, 0x31, 0x24,
		0x34, 0x26, 0x36, 0x20, 0x30, 0x25, 0x35, 0x27, 0x37,
		0x21, 0x31, 0x22, 0x32, 0x20, 0x30, 0x26, 0x36, 0x25,
		0x35, 0x23, 0x33, 0x21, 0x31, 0x20, 0x30, 0x24, 0x34,
		0x27, 0x37, 0x25, 0x35, 0x26, 0x36, 0x20, 0x30, 0x21,
		0x31},
	}
};

struct ipq_tdm_config *tdm_config = ipq9574_tdm_config;

static struct ipq_eth_sku ipq9574_uniphy[CONFIG_ETH_MAX_UNIPHY] = {
	{
		.reg	= 0xA4024,
		.bit	= 23,
	},
	{
		.reg	= 0xA4024,
		.bit	= 24,
	},
	{
		.reg	= 0xA4024,
		.bit	= 25,
	},
};

struct ipq_eth_sku *ipq_uniphy = ipq9574_uniphy;

struct edma_config ipq_edma_config = {
	.sw_version		= EDMA_SW_VER_1_ID,
	.txdesc_ring_start	= 0,
	.txdesc_rings		= 1,
	.txdesc_ring_end	= 1,
	.txcmpl_ring_start	= 0,
	.txcmpl_rings		= 1,
	.txcmpl_ring_end	= 1,
	.rxfill_ring_start	= 0,
	.rxfill_rings		= 1,
	.rxfill_ring_end	= 1,
	.rxdesc_ring_start	= 0,
	.rxdesc_rings		= 1,
	.rxdesc_ring_end	= 1,
	.tx_map			= 6,
	.rx_map			= 3,
	.max_txcmpl_rings	= 32,
	.max_txdesc_rings	= 32,
	.max_rxdesc_rings	= 24,
	.max_rxfill_rings	= 8,
	.iports			= 8,
	.ports			= 6,
	.start_ports		= 1,
	.vsi			= 0x7f,
	.ipo_action		= 5,
	.tdm_ctrl_val		= 0x80000076,
};

void ipq_config_cmn_clock(void)
{
	unsigned int reg_val;
	/*
	 * Init CMN clock for ethernet
	 */
	reg_val = readl(CMN_BLK_ADDR + 4);
	reg_val = (reg_val & FREQUENCY_MASK) | INTERNAL_48MHZ_CLOCK;
	writel(reg_val, CMN_BLK_ADDR + 0x4);
	reg_val = readl(CMN_BLK_ADDR);
	reg_val = reg_val | 0x40;
	writel(reg_val, CMN_BLK_ADDR);
	mdelay(1);
	reg_val = reg_val & (~0x40);
	writel(reg_val, CMN_BLK_ADDR);
	mdelay(1);
	writel(0xbf, CMN_BLK_ADDR);
	mdelay(1);
	writel(0xff, CMN_BLK_ADDR);
	mdelay(1);
}
