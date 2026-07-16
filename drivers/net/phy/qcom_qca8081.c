/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <command.h>
#include <asm/io.h>
#include <phy.h>
#include <miiphy.h>

#include <linux/compat.h>
#include <linux/ethtool.h>

#define QTI_8081_PHY_V1				0x004DD100
#define QTI_8081_PHY_V1_1			0x004DD101

#define QTI_8081_PHY_FEATURES			(PHY_GBIT_FEATURES | \
						 SUPPORTED_2500baseX_Full)

#define QTI_8081_PHY_SPEC_STATUS                 17
#define QTI_8081_STATUS_LINK_PASS                0x0400
#define QTI_8081_STATUS_FULL_DUPLEX              0x2000
#define QTI_8081_STATUS_SPEED_MASK               0x380
#define QTI_8081_STATUS_SPEED_2500MBS            0x200
#define QTI_8081_STATUS_SPEED_1000MBS            0x100
#define QTI_8081_STATUS_SPEED_100MBS             0x80
#define QTI_8081_STATUS_SPEED_10MBS              0x0000

#define QTI_8081_PHY_MMD3_NUM                    3

#define QTI_8081_PHY_MMD3_ADDR_CLD_CTRL7         0x8007
#define QTI_8081_PHY_8023AZ_AFE_CTRL_MASK        0x01f0
#define QTI_8081_PHY_8023AZ_AFE_EN               0x0090

#define QTI_8081_PHY_MMD3_AZ_TRAINING_CTRL       0x8008
#define QTI_8081_PHY_MMD3_AZ_TRAINING_VAL        0x1c32

#define QTI_8081_PHY_SERDES_ADDR_OFFSET		1
#define QTI_8081_PHY_SERDES_MMD1_FIFO_CTRL	0x9072
#define QTI_8081_PHY_FIFO_RSTN			BIT(11)

static int qti_8081_config(struct phy_device *phydev)
{
	int adv_changed, ret;

	/* Enable vga when init napa to fix 8023az issue */
	ret = phy_modify_mmd(phydev, QTI_8081_PHY_MMD3_NUM,
			     QTI_8081_PHY_MMD3_ADDR_CLD_CTRL7,
			     QTI_8081_PHY_8023AZ_AFE_CTRL_MASK,
			     QTI_8081_PHY_8023AZ_AFE_EN);
	if (ret)
		return ret;

	/* Special configuration for AZ under 1G speed mode */
	ret = phy_write_mmd(phydev, QTI_8081_PHY_MMD3_NUM,
			    QTI_8081_PHY_MMD3_AZ_TRAINING_CTRL,
			    QTI_8081_PHY_MMD3_AZ_TRAINING_VAL);
	if (ret)
		return ret;

	adv_changed = phy_modify_mmd_changed(phydev, MDIO_MMD_AN,
					     MDIO_AN_10GBT_CTRL,
					     MDIO_AN_10GBT_CTRL_ADV2_5G,
					     (phydev->advertising &
					      SUPPORTED_2500baseX_Full) ?
					     MDIO_AN_10GBT_CTRL_ADV2_5G : 0);
	if (adv_changed < 0)
		return adv_changed;

	ret = genphy_config_aneg(phydev);
	if (ret < 0 || !adv_changed || phydev->autoneg != AUTONEG_ENABLE)
		return ret;

	return genphy_restart_aneg(phydev);
}

static int qti_8081_update_fifo(struct phy_device *phydev)
{
	struct mii_dev *bus = phydev->bus;
	int addr = phydev->addr + QTI_8081_PHY_SERDES_ADDR_OFFSET;
	int old, val;

	if (!bus || !bus->read || !bus->write || addr < 0 || addr >= PHY_MAX_ADDR)
		return -EINVAL;

	old = bus->read(bus, addr, MDIO_MMD_PMAPMD,
			QTI_8081_PHY_SERDES_MMD1_FIFO_CTRL);
	if (old < 0)
		return old;

	if (phydev->link)
		val = old | QTI_8081_PHY_FIFO_RSTN;
	else
		val = old & ~QTI_8081_PHY_FIFO_RSTN;
	if (val == old)
		return 0;

	return bus->write(bus, addr, MDIO_MMD_PMAPMD,
			  QTI_8081_PHY_SERDES_MMD1_FIFO_CTRL, val);
}

static int qti_8081_startup(struct phy_device *phydev)
{
	int phy_data, ret;

	phy_data = phy_read(phydev, MDIO_DEVAD_NONE, QTI_8081_PHY_SPEC_STATUS);
	if (phy_data < 0)
		return phy_data;
	if (phy_data & QTI_8081_STATUS_LINK_PASS)
		phydev->link = 1;
	else
		phydev->link = 0;

	ret = qti_8081_update_fifo(phydev);
	if (ret)
		return ret;

	if (phy_data & QTI_8081_STATUS_FULL_DUPLEX)
		phydev->duplex = DUPLEX_FULL;
	else
		phydev->duplex = DUPLEX_HALF;

	switch (phy_data & QTI_8081_STATUS_SPEED_MASK) {
	case QTI_8081_STATUS_SPEED_2500MBS:
		phydev->speed = SPEED_2500;
		break;
	case QTI_8081_STATUS_SPEED_1000MBS:
		phydev->speed = SPEED_1000;
		break;
	case QTI_8081_STATUS_SPEED_100MBS:
		phydev->speed = SPEED_100;
		break;
	case QTI_8081_STATUS_SPEED_10MBS:
		phydev->speed = SPEED_10;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

U_BOOT_PHY_DRIVER(qti_8081_driver) = {
	.name = "QTI 8081 PHY Driver",
	.uid = QTI_8081_PHY_V1,
	.mask = 0xfffffff0,
	.features = QTI_8081_PHY_FEATURES,
	.config = &qti_8081_config,
	.startup = &qti_8081_startup,
	.shutdown = &genphy_shutdown,
};
