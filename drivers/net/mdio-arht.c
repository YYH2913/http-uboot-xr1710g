// SPDX-License-Identifier: GPL-2.0+

#include <asm/io.h>
#include <dm.h>
#include <linux/bitfield.h>
#include <linux/iopoll.h>
#include <miiphy.h>

#define ARHT_MDIO_PHY_CTL			0xf01c
#define ARHT_MDIO_FE_MISC			0xfffc
#define   ARHT_MDIO_BUSY			BIT(31)
#define   ARHT_MDIO_DEVAD			GENMASK(29, 25)
#define   ARHT_MDIO_REG				GENMASK(29, 25)
#define   ARHT_MDIO_PHY_ADDR			GENMASK(24, 20)
#define   ARHT_MDIO_DATA			GENMASK(15, 0)

#define ARHT_MDIO_CMD_C45_ADDR			0x80000000
#define ARHT_MDIO_CMD_C45_WRITE			0x80040000
#define ARHT_MDIO_CMD_C22_WRITE			0x80050000
#define ARHT_MDIO_CMD_C45_READ			0x80080000
#define ARHT_MDIO_CMD_C22_READ			0x80090000

#define ARHT_MDIO_TIMEOUT_US			10000

struct arht_mdio_priv {
	phys_addr_t fe_regs;
};

static int arht_mdio_wait_ready(struct arht_mdio_priv *priv, u32 *val)
{
	u32 tmp;
	int ret;

	ret = readl_poll_timeout(priv->fe_regs + ARHT_MDIO_PHY_CTL, tmp,
				 !(tmp & ARHT_MDIO_BUSY),
				 ARHT_MDIO_TIMEOUT_US);
	if (!ret && val)
		*val = tmp;

	return ret;
}

static int arht_mdio_read(struct udevice *dev, int addr, int devad, int reg)
{
	struct arht_mdio_priv *priv = dev_get_priv(dev);
	u32 cmd, val;
	int ret;

	if (devad != MDIO_DEVAD_NONE) {
		ret = arht_mdio_wait_ready(priv, NULL);
		if (ret)
			return ret;

		cmd = ARHT_MDIO_CMD_C45_ADDR |
		      FIELD_PREP(ARHT_MDIO_PHY_ADDR, addr) |
		      FIELD_PREP(ARHT_MDIO_DEVAD, devad) |
		      FIELD_PREP(ARHT_MDIO_DATA, reg);
		writel(cmd, priv->fe_regs + ARHT_MDIO_PHY_CTL);

		ret = arht_mdio_wait_ready(priv, NULL);
		if (ret)
			return ret;

		cmd = ARHT_MDIO_CMD_C45_READ |
		      FIELD_PREP(ARHT_MDIO_PHY_ADDR, addr) |
		      FIELD_PREP(ARHT_MDIO_DEVAD, devad);
		writel(cmd, priv->fe_regs + ARHT_MDIO_PHY_CTL);
	} else {
		/*
		 * Stock FE MDIO does a benign read from FE+0xfffc before every
		 * Clause 22 command. Mirror that sequence for compatibility.
		 */
		readl(priv->fe_regs + ARHT_MDIO_FE_MISC);

		ret = arht_mdio_wait_ready(priv, NULL);
		if (ret)
			return ret;

		cmd = ARHT_MDIO_CMD_C22_READ |
		      FIELD_PREP(ARHT_MDIO_PHY_ADDR, addr) |
		      FIELD_PREP(ARHT_MDIO_REG, reg);
		writel(cmd, priv->fe_regs + ARHT_MDIO_PHY_CTL);
	}

	ret = arht_mdio_wait_ready(priv, &val);
	if (ret)
		return ret;

	return FIELD_GET(ARHT_MDIO_DATA, val);
}

static int arht_mdio_write(struct udevice *dev, int addr, int devad, int reg,
			   u16 value)
{
	struct arht_mdio_priv *priv = dev_get_priv(dev);
	u32 cmd;
	int ret;

	if (devad != MDIO_DEVAD_NONE) {
		ret = arht_mdio_wait_ready(priv, NULL);
		if (ret)
			return ret;

		cmd = ARHT_MDIO_CMD_C45_ADDR |
		      FIELD_PREP(ARHT_MDIO_PHY_ADDR, addr) |
		      FIELD_PREP(ARHT_MDIO_DEVAD, devad) |
		      FIELD_PREP(ARHT_MDIO_DATA, reg);
		writel(cmd, priv->fe_regs + ARHT_MDIO_PHY_CTL);

		ret = arht_mdio_wait_ready(priv, NULL);
		if (ret)
			return ret;

		cmd = ARHT_MDIO_CMD_C45_WRITE |
		      FIELD_PREP(ARHT_MDIO_PHY_ADDR, addr) |
		      FIELD_PREP(ARHT_MDIO_DEVAD, devad) |
		      FIELD_PREP(ARHT_MDIO_DATA, value);
		writel(cmd, priv->fe_regs + ARHT_MDIO_PHY_CTL);
	} else {
		readl(priv->fe_regs + ARHT_MDIO_FE_MISC);

		ret = arht_mdio_wait_ready(priv, NULL);
		if (ret)
			return ret;

		cmd = ARHT_MDIO_CMD_C22_WRITE |
		      FIELD_PREP(ARHT_MDIO_PHY_ADDR, addr) |
		      FIELD_PREP(ARHT_MDIO_REG, reg) |
		      FIELD_PREP(ARHT_MDIO_DATA, value);
		writel(cmd, priv->fe_regs + ARHT_MDIO_PHY_CTL);
	}

	return arht_mdio_wait_ready(priv, NULL);
}

static const struct mdio_ops arht_mdio_ops = {
	.read = arht_mdio_read,
	.write = arht_mdio_write,
};

static int arht_mdio_probe(struct udevice *dev)
{
	static const char *const eth_compat[] = {
		"airoha,en7581-eth",
		"airoha,en7523-eth",
	};
	struct arht_mdio_priv *priv = dev_get_priv(dev);
	ofnode parent, eth_node;
	fdt_addr_t fe_addr;
	int i;

	parent = ofnode_get_parent(dev_ofnode(dev));
	fe_addr = FDT_ADDR_T_NONE;

	if (ofnode_valid(parent) &&
	    (ofnode_device_is_compatible(parent, eth_compat[0]) ||
	     ofnode_device_is_compatible(parent, eth_compat[1])))
		fe_addr = ofnode_get_addr_index(parent, 0);

	for (i = 0; fe_addr == FDT_ADDR_T_NONE && i < ARRAY_SIZE(eth_compat); i++) {
		eth_node = ofnode_by_compatible(ofnode_null(), eth_compat[i]);
		if (ofnode_valid(eth_node))
			fe_addr = ofnode_get_addr_index(eth_node, 0);
	}

	if (fe_addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	priv->fe_regs = fe_addr;
	return 0;
}

static const struct udevice_id arht_mdio_ids[] = {
	{ .compatible = "airoha,arht-mdio" },
	{ }
};

U_BOOT_DRIVER(arht_mdio) = {
	.name		= "airoha-arht-mdio",
	.id		= UCLASS_MDIO,
	.of_match	= arht_mdio_ids,
	.probe		= arht_mdio_probe,
	.ops		= &arht_mdio_ops,
	.priv_auto	= sizeof(struct arht_mdio_priv),
};
