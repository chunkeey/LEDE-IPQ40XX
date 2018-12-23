// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2009 Felix Fietkau <nbd@nbd.name>
 * Copyright (C) 2011-2012 Gabor Juhos <juhosg@openwrt.org>
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 * Copyright (c) 2016 John Crispin <john@phrozen.org>
 */

#include <linux/module.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <net/dsa.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/if_bridge.h>
#include <linux/mdio.h>
#include <linux/etherdevice.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/mdio.h>
#include <linux/mfd/syscon.h>
#include <linux/of_mdio.h>
#include <linux/workqueue.h>

#include "qca8k.h"

#define MIB_DESC(_s, _o, _n)	\
	{			\
		.size = (_s),	\
		.offset = (_o),	\
		.name = (_n),	\
	}

static const struct qca8k_mib_desc ar8327_mib[] = {
	MIB_DESC(1, 0x00, "RxBroad"),
	MIB_DESC(1, 0x04, "RxPause"),
	MIB_DESC(1, 0x08, "RxMulti"),
	MIB_DESC(1, 0x0c, "RxFcsErr"),
	MIB_DESC(1, 0x10, "RxAlignErr"),
	MIB_DESC(1, 0x14, "RxRunt"),
	MIB_DESC(1, 0x18, "RxFragment"),
	MIB_DESC(1, 0x1c, "Rx64Byte"),
	MIB_DESC(1, 0x20, "Rx128Byte"),
	MIB_DESC(1, 0x24, "Rx256Byte"),
	MIB_DESC(1, 0x28, "Rx512Byte"),
	MIB_DESC(1, 0x2c, "Rx1024Byte"),
	MIB_DESC(1, 0x30, "Rx1518Byte"),
	MIB_DESC(1, 0x34, "RxMaxByte"),
	MIB_DESC(1, 0x38, "RxTooLong"),
	MIB_DESC(2, 0x3c, "RxGoodByte"),
	MIB_DESC(2, 0x44, "RxBadByte"),
	MIB_DESC(1, 0x4c, "RxOverFlow"),
	MIB_DESC(1, 0x50, "Filtered"),
	MIB_DESC(1, 0x54, "TxBroad"),
	MIB_DESC(1, 0x58, "TxPause"),
	MIB_DESC(1, 0x5c, "TxMulti"),
	MIB_DESC(1, 0x60, "TxUnderRun"),
	MIB_DESC(1, 0x64, "Tx64Byte"),
	MIB_DESC(1, 0x68, "Tx128Byte"),
	MIB_DESC(1, 0x6c, "Tx256Byte"),
	MIB_DESC(1, 0x70, "Tx512Byte"),
	MIB_DESC(1, 0x74, "Tx1024Byte"),
	MIB_DESC(1, 0x78, "Tx1518Byte"),
	MIB_DESC(1, 0x7c, "TxMaxByte"),
	MIB_DESC(1, 0x80, "TxOverSize"),
	MIB_DESC(2, 0x84, "TxByte"),
	MIB_DESC(1, 0x8c, "TxCollision"),
	MIB_DESC(1, 0x90, "TxAbortCol"),
	MIB_DESC(1, 0x94, "TxMultiCol"),
	MIB_DESC(1, 0x98, "TxSingleCol"),
	MIB_DESC(1, 0x9c, "TxExcDefer"),
	MIB_DESC(1, 0xa0, "TxDefer"),
	MIB_DESC(1, 0xa4, "TxLateCol"),
};


static u32
qca8k_read(struct qca8k_priv *priv, u32 reg)
{
	unsigned int val;

	regmap_read(priv->base, reg, &val);
	return val;
}

static void
qca8k_write(struct qca8k_priv *priv, u32 reg, u32 val)
{
	regmap_write(priv->base, reg, val);
}

static u32
qca8k_rmw(struct qca8k_priv *priv, u32 reg, u32 mask, u32 val)
{
	u32 ret;

	ret = qca8k_read(priv, reg);
	ret &= ~mask;
	ret |= val;
	qca8k_write(priv, reg, ret);

	return ret;
}

static void
qca8k_reg_set(struct qca8k_priv *priv, u32 reg, u32 val)
{
	qca8k_rmw(priv, reg, 0, val);
}

static void
qca8k_reg_clear(struct qca8k_priv *priv, u32 reg, u32 val)
{
	qca8k_rmw(priv, reg, val, 0);
}

static int
qca8k_regmap_read(void *ctx, uint32_t reg, uint32_t *val)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ctx;

	*val = qca8k_read(priv, reg);

	return 0;
}

static int
qca8k_regmap_write(void *ctx, uint32_t reg, uint32_t val)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ctx;

	qca8k_write(priv, reg, val);

	return 0;
}

static const struct regmap_range qca8k_readable_ranges[] = {
	regmap_reg_range(0x0000, 0x00e4), /* Global control */
	regmap_reg_range(0x0100, 0x0168), /* EEE control */
	regmap_reg_range(0x0200, 0x0270), /* Parser control */
	regmap_reg_range(0x0400, 0x0454), /* ACL */
	regmap_reg_range(0x0600, 0x0718), /* Lookup */
	regmap_reg_range(0x0800, 0x0b70), /* QM */
	regmap_reg_range(0x0c00, 0x0c80), /* PKT */
	regmap_reg_range(0x0e00, 0x0e98), /* L3 */
	regmap_reg_range(0x1000, 0x10ac), /* MIB - Port0 */
	regmap_reg_range(0x1100, 0x11ac), /* MIB - Port1 */
	regmap_reg_range(0x1200, 0x12ac), /* MIB - Port2 */
	regmap_reg_range(0x1300, 0x13ac), /* MIB - Port3 */
	regmap_reg_range(0x1400, 0x14ac), /* MIB - Port4 */
	regmap_reg_range(0x1500, 0x15ac), /* MIB - Port5 */
	regmap_reg_range(0x1600, 0x16ac), /* MIB - Port6 */

};

static const struct regmap_access_table qca8k_readable_table = {
	.yes_ranges = qca8k_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(qca8k_readable_ranges),
};

static struct regmap_config qca8k_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x16ac, /* end MIB - Port6 range */
	.reg_read = qca8k_regmap_read,
	.reg_write = qca8k_regmap_write,
	.rd_table = &qca8k_readable_table,
};

static int
qca8k_busy_wait(struct qca8k_priv *priv, u32 reg, u32 mask)
{
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(20);

	/* loop until the busy flag has cleared */
	do {
		u32 val = qca8k_read(priv, reg);
		int busy = val & mask;

		if (!busy)
			break;
		cond_resched();
	} while (!time_after_eq(jiffies, timeout));

	return time_after_eq(jiffies, timeout);
}

static void
qca8k_fdb_read(struct qca8k_priv *priv, struct qca8k_fdb *fdb)
{
	u32 reg[4];
	int i;

	/* load the ARL table into an array */
	for (i = 0; i < 4; i++)
		reg[i] = qca8k_read(priv, QCA8K_REG_ATU_DATA0 + (i * 4));

	/* vid - 83:72 */
	fdb->vid = (reg[2] >> QCA8K_ATU_VID_S) & QCA8K_ATU_VID_M;
	/* aging - 67:64 */
	fdb->aging = reg[2] & QCA8K_ATU_STATUS_M;
	/* portmask - 54:48 */
	fdb->port_mask = (reg[1] >> QCA8K_ATU_PORT_S) & QCA8K_ATU_PORT_M;
	/* mac - 47:0 */
	fdb->mac[0] = (reg[1] >> QCA8K_ATU_ADDR0_S) & 0xff;
	fdb->mac[1] = reg[1] & 0xff;
	fdb->mac[2] = (reg[0] >> QCA8K_ATU_ADDR2_S) & 0xff;
	fdb->mac[3] = (reg[0] >> QCA8K_ATU_ADDR3_S) & 0xff;
	fdb->mac[4] = (reg[0] >> QCA8K_ATU_ADDR4_S) & 0xff;
	fdb->mac[5] = reg[0] & 0xff;
}

static void
qca8k_fdb_write(struct qca8k_priv *priv, u16 vid, u8 port_mask, const u8 *mac,
		u8 aging)
{
	u32 reg[3] = { 0 };
	int i;

	/* vid - 83:72 */
	reg[2] = (vid & QCA8K_ATU_VID_M) << QCA8K_ATU_VID_S;
	/* aging - 67:64 */
	reg[2] |= aging & QCA8K_ATU_STATUS_M;
	/* portmask - 54:48 */
	reg[1] = (port_mask & QCA8K_ATU_PORT_M) << QCA8K_ATU_PORT_S;
	/* mac - 47:0 */
	reg[1] |= mac[0] << QCA8K_ATU_ADDR0_S;
	reg[1] |= mac[1];
	reg[0] |= mac[2] << QCA8K_ATU_ADDR2_S;
	reg[0] |= mac[3] << QCA8K_ATU_ADDR3_S;
	reg[0] |= mac[4] << QCA8K_ATU_ADDR4_S;
	reg[0] |= mac[5];

	/* load the array into the ARL table */
	for (i = 0; i < 3; i++)
		qca8k_write(priv, QCA8K_REG_ATU_DATA0 + (i * 4), reg[i]);
}

static int
qca8k_fdb_access(struct qca8k_priv *priv, enum qca8k_fdb_cmd cmd, int port)
{
	u32 reg;

	/* Set the command and FDB index */
	reg = QCA8K_ATU_FUNC_BUSY;
	reg |= cmd;
	if (port >= 0) {
		reg |= QCA8K_ATU_FUNC_PORT_EN;
		reg |= (port & QCA8K_ATU_FUNC_PORT_M) << QCA8K_ATU_FUNC_PORT_S;
	}

	/* Write the function register triggering the table access */
	qca8k_write(priv, QCA8K_REG_ATU_FUNC, reg);

	/* wait for completion */
	if (qca8k_busy_wait(priv, QCA8K_REG_ATU_FUNC, QCA8K_ATU_FUNC_BUSY))
		return -1;

	/* Check for table full violation when adding an entry */
	if (cmd == QCA8K_FDB_LOAD) {
		reg = qca8k_read(priv, QCA8K_REG_ATU_FUNC);
		if (reg & QCA8K_ATU_FUNC_FULL)
			return -1;
	}

	return 0;
}

static int
qca8k_fdb_next(struct qca8k_priv *priv, struct qca8k_fdb *fdb, int port)
{
	int ret;

	qca8k_fdb_write(priv, fdb->vid, fdb->port_mask, fdb->mac, fdb->aging);
	ret = qca8k_fdb_access(priv, QCA8K_FDB_NEXT, port);
	if (ret >= 0)
		qca8k_fdb_read(priv, fdb);

	return ret;
}

static int
qca8k_fdb_add(struct qca8k_priv *priv, const u8 *mac, u16 port_mask,
	      u16 vid, u8 aging)
{
	int ret;

	mutex_lock(&priv->reg_mutex);
	qca8k_fdb_write(priv, vid, port_mask, mac, aging);
	ret = qca8k_fdb_access(priv, QCA8K_FDB_LOAD, -1);
	mutex_unlock(&priv->reg_mutex);

	return ret;
}

static int
qca8k_fdb_del(struct qca8k_priv *priv, const u8 *mac, u16 port_mask, u16 vid)
{
	int ret;

	mutex_lock(&priv->reg_mutex);
	qca8k_fdb_write(priv, vid, port_mask, mac, 0);
	ret = qca8k_fdb_access(priv, QCA8K_FDB_PURGE, -1);
	mutex_unlock(&priv->reg_mutex);

	return ret;
}

static void
qca8k_fdb_flush(struct qca8k_priv *priv)
{
	mutex_lock(&priv->reg_mutex);
	qca8k_fdb_access(priv, QCA8K_FDB_FLUSH, -1);
	mutex_unlock(&priv->reg_mutex);
}

static void
qca8k_mib_init(struct qca8k_priv *priv)
{
	mutex_lock(&priv->reg_mutex);
	qca8k_reg_set(priv, QCA8K_REG_MIB, QCA8K_MIB_FLUSH | QCA8K_MIB_BUSY);
	qca8k_busy_wait(priv, QCA8K_REG_MIB, QCA8K_MIB_BUSY);
	qca8k_reg_set(priv, QCA8K_REG_MIB, QCA8K_MIB_CPU_KEEP);
	qca8k_write(priv, QCA8K_REG_MODULE_EN, QCA8K_MODULE_EN_MIB);
	mutex_unlock(&priv->reg_mutex);
}

static int
qca8k_set_pad_ctrl(struct qca8k_priv *priv, int port, int mode)
{
	u32 reg;

	switch (port) {
	case 0:
		reg = QCA8K_REG_PORT0_PAD_CTRL;
		break;
	case 6:
		reg = QCA8K_REG_PORT6_PAD_CTRL;
		break;
	default:
		pr_err("Can't set PAD_CTRL on port %d\n", port);
		return -EINVAL;
	}

	/* Configure a port to be directly connected to an external
	 * PHY or MAC.
	 */
	switch (mode) {
	case PHY_INTERFACE_MODE_RGMII:
		qca8k_write(priv, reg,
			    QCA8K_PORT_PAD_RGMII_EN |
			    QCA8K_PORT_PAD_RGMII_TX_DELAY(3) |
			    QCA8K_PORT_PAD_RGMII_RX_DELAY(3));

		/* According to the datasheet, RGMII delay is enabled through
		 * PORT5_PAD_CTRL for all ports, rather than individual port
		 * registers
		 */
		qca8k_write(priv, QCA8K_REG_PORT5_PAD_CTRL,
			    QCA8K_PORT_PAD_RGMII_RX_DELAY_EN);
		break;
	case PHY_INTERFACE_MODE_SGMII:
		qca8k_write(priv, reg, QCA8K_PORT_PAD_SGMII_EN);
		break;
	case PHY_INTERFACE_MODE_INTERNAL:
		break;
	default:
		pr_err("xMII mode %d not supported\n", mode);
		return -EINVAL;
	}

	return 0;
}

static void
qca8k_port_set_status(struct qca8k_priv *priv, int port, int enable)
{
	u32 mask = QCA8K_PORT_STATUS_TXMAC | QCA8K_PORT_STATUS_RXMAC;

	/* Port 0 and 6 have no internal PHY */
	if (port > 0 && port < 6)
		mask |= QCA8K_PORT_STATUS_LINK_AUTO;

	if (enable)
		qca8k_reg_set(priv, QCA8K_REG_PORT_STATUS(port), mask);
	else
		qca8k_reg_clear(priv, QCA8K_REG_PORT_STATUS(port), mask);
}

static int
qca8k_setup(struct dsa_switch *ds)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	int ret, i, phy_mode = -1;
	u32 mask;

	/* Make sure that port 0 is the cpu port */
	if (!dsa_is_cpu_port(ds, 0)) {
		pr_err("port 0 is not the CPU port\n");
		return -EINVAL;
	}

	mutex_init(&priv->reg_mutex);

	/* Start by setting up the register mapping */
	priv->regmap = devm_regmap_init(ds->dev, NULL, priv,
					&qca8k_regmap_config);
	if (IS_ERR(priv->regmap))
		pr_warn("regmap initialization failed");

	/* Initialize CPU port pad mode (xMII type, delays...) */
	phy_mode = of_get_phy_mode(ds->ports[QCA8K_CPU_PORT].dn);
	if (phy_mode < 0) {
		pr_err("Can't find phy-mode for master device\n");
		return phy_mode;
	}
	ret = qca8k_set_pad_ctrl(priv, QCA8K_CPU_PORT, phy_mode);
	if (ret < 0)
		return ret;

	/* Enable CPU Port, force it to maximum bandwidth and full-duplex */
	mask = QCA8K_PORT_STATUS_SPEED_1000 | QCA8K_PORT_STATUS_TXFLOW | QCA8K_PORT_TXHALF_FLOW |
	       QCA8K_PORT_STATUS_RXFLOW | QCA8K_PORT_STATUS_DUPLEX;
	qca8k_write(priv, QCA8K_REG_PORT_STATUS(QCA8K_CPU_PORT), mask);
	qca8k_reg_set(priv, QCA8K_REG_GLOBAL_FW_CTRL0,
		      QCA8K_GLOBAL_FW_CTRL0_CPU_PORT_EN);
	qca8k_port_set_status(priv, QCA8K_CPU_PORT, 1);
	priv->port_sts[QCA8K_CPU_PORT].enabled = 1;

	/* Enable MIB counters */
	qca8k_mib_init(priv);

	/* Disable buggy AZ */
	qca8k_write(priv, QCA8K_REG_EEE_CTRL, 0);

	/* enable jumbo frames */
	qca8k_rmw(priv, QCA8K_REG_MAX_FRAME_SIZE,
		  QCA8K_MAX_FRAME_SIZE_MTU, 9018 + 8 + 2);

	qca8k_write(priv, QCA8K_REG_PORT_FLOWCTRL_THRESH(0),
		(QCA8K_PORT0_FC_THRESH_ON_DFLT << 16) |
		QCA8K_PORT0_FC_THRESH_OFF_DFLT);

	/* Enable QCA header mode on the cpu port */
	qca8k_write(priv, QCA8K_REG_PORT_HDR_CTRL(QCA8K_CPU_PORT), 0);
	/*
		    QCA8K_PORT_HDR_CTRL_ALL << QCA8K_PORT_HDR_CTRL_TX_S |
		    QCA8K_PORT_HDR_CTRL_ALL << QCA8K_PORT_HDR_CTRL_RX_S);*/

	/* Disable forwarding by default on all ports */
	for (i = 0; i < QCA8K_NUM_PORTS; i++)
		qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(i),
			  QCA8K_PORT_LOOKUP_MEMBER, 0);

	/* Disable MAC by default on all user ports */
	for (i = 1; i < QCA8K_NUM_PORTS; i++)
		if (dsa_is_user_port(ds, i))
			qca8k_port_set_status(priv, i, 0);

	/* Forward all unknown frames to CPU port for Linux processing */
	qca8k_write(priv, QCA8K_REG_GLOBAL_FW_CTRL1,
		    BIT(0) << QCA8K_GLOBAL_FW_CTRL1_IGMP_DP_S |
		    GENMASK(5, 0) << QCA8K_GLOBAL_FW_CTRL1_BC_DP_S |
		    GENMASK(5, 0) << QCA8K_GLOBAL_FW_CTRL1_MC_DP_S |
		    GENMASK(5, 0) << QCA8K_GLOBAL_FW_CTRL1_UC_DP_S);

	/* Setup connection between CPU port & user ports */
	for (i = 0; i < DSA_MAX_PORTS; i++) {
		/* CPU port gets connected to all user ports of the switch */
		if (dsa_is_cpu_port(ds, i)) {
			qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(QCA8K_CPU_PORT),
				  QCA8K_PORT_LOOKUP_MEMBER, dsa_user_ports(ds));
		}

		/* Invividual user ports get connected to CPU port only */
		if (dsa_is_user_port(ds, i)) {
			int shift = 16 * (i % 2);

			qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(i),
				  QCA8K_PORT_LOOKUP_MEMBER,
				  BIT(QCA8K_CPU_PORT));

			/* Enable ARP Auto-learning by default */
			qca8k_reg_set(priv, QCA8K_PORT_LOOKUP_CTRL(i),
				      QCA8K_PORT_LOOKUP_LEARN);

			/* For port based vlans to work we need to set the
			 * default egress vid
			 */
			qca8k_rmw(priv, QCA8K_EGRESS_VLAN(i),
				  0xffff << shift, 1 << shift);
			qca8k_write(priv, QCA8K_REG_PORT_VLAN_CTRL0(i),
				    QCA8K_PORT_VLAN_CVID(1) |
				    QCA8K_PORT_VLAN_SVID(1));
		}
	}

	/* Flush the FDB table */
	qca8k_fdb_flush(priv);

	return 0;
}

static void
qca8k_adjust_link(struct dsa_switch *ds, int port, struct phy_device *phy)
{
	struct qca8k_priv *priv = ds->priv;
	u32 reg;

	/* Force fixed-link setting for CPU port, skip others. */
	if (!phy_is_pseudo_fixed_link(phy))
		return;

	/* Set port speed */
	switch (phy->speed) {
	case 10:
		reg = QCA8K_PORT_STATUS_SPEED_10;
		break;
	case 100:
		reg = QCA8K_PORT_STATUS_SPEED_100;
		break;
	case 1000:
		reg = QCA8K_PORT_STATUS_SPEED_1000;
		break;
	default:
		dev_dbg(priv->dev, "port%d link speed %dMbps not supported.\n",
			port, phy->speed);
		return;
	}

	/* Set duplex mode */
	if (phy->duplex == DUPLEX_FULL)
		reg |= QCA8K_PORT_STATUS_DUPLEX;

	/* Force flow control */
	if (dsa_is_cpu_port(ds, port))
		reg |= QCA8K_PORT_STATUS_RXFLOW | QCA8K_PORT_STATUS_TXFLOW |
		QCA8K_PORT_TXHALF_FLOW;

	/* Force link down before changing MAC options */
	qca8k_port_set_status(priv, port, 0);
	qca8k_write(priv, QCA8K_REG_PORT_STATUS(port), reg);
	qca8k_port_set_status(priv, port, 1);
}

static void
qca8k_get_strings(struct dsa_switch *ds, int port, u32 stringset, uint8_t *data)
{
	int i;

	if (stringset != ETH_SS_STATS)
		return;

	for (i = 0; i < ARRAY_SIZE(ar8327_mib); i++)
		strncpy(data + i * ETH_GSTRING_LEN, ar8327_mib[i].name,
			ETH_GSTRING_LEN);
}

static void
qca8k_get_ethtool_stats(struct dsa_switch *ds, int port,
			uint64_t *data)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	const struct qca8k_mib_desc *mib;
	u32 reg, i;
	u64 hi;

	for (i = 0; i < ARRAY_SIZE(ar8327_mib); i++) {
		mib = &ar8327_mib[i];
		reg = QCA8K_PORT_MIB_COUNTER(port) + mib->offset;

		data[i] = qca8k_read(priv, reg);
		if (mib->size == 2) {
			hi = qca8k_read(priv, reg + 4);
			data[i] |= hi << 32;
		}
	}
}

static int
qca8k_get_sset_count(struct dsa_switch *ds, int port, int sset)
{
	if (sset != ETH_SS_STATS)
		return 0;

	return ARRAY_SIZE(ar8327_mib);
}

static int
qca8k_set_mac_eee(struct dsa_switch *ds, int port, struct ethtool_eee *eee)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	u32 lpi_en = QCA8K_REG_EEE_CTRL_LPI_EN(port);
	u32 reg;

	mutex_lock(&priv->reg_mutex);
	reg = qca8k_read(priv, QCA8K_REG_EEE_CTRL);
	if (eee->eee_enabled)
		reg |= lpi_en;
	else
		reg &= ~lpi_en;
	qca8k_write(priv, QCA8K_REG_EEE_CTRL, reg);
	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static int
qca8k_get_mac_eee(struct dsa_switch *ds, int port, struct ethtool_eee *e)
{
	/* Nothing to do on the port's MAC */
	return 0;
}

static void
qca8k_port_stp_state_set(struct dsa_switch *ds, int port, u8 state)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	u32 stp_state;

	switch (state) {
	case BR_STATE_DISABLED:
		stp_state = QCA8K_PORT_LOOKUP_STATE_DISABLED;
		break;
	case BR_STATE_BLOCKING:
		stp_state = QCA8K_PORT_LOOKUP_STATE_BLOCKING;
		break;
	case BR_STATE_LISTENING:
		stp_state = QCA8K_PORT_LOOKUP_STATE_LISTENING;
		break;
	case BR_STATE_LEARNING:
		stp_state = QCA8K_PORT_LOOKUP_STATE_LEARNING;
		break;
	case BR_STATE_FORWARDING:
	default:
		stp_state = QCA8K_PORT_LOOKUP_STATE_FORWARD;
		break;
	}

	qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(port),
		  QCA8K_PORT_LOOKUP_STATE_MASK, stp_state);
}

static int
qca8k_port_bridge_join(struct dsa_switch *ds, int port, struct net_device *br)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	int port_mask = BIT(QCA8K_CPU_PORT);
	int i;

	for (i = 1; i < QCA8K_NUM_PORTS; i++) {
		if (dsa_to_port(ds, i)->bridge_dev != br)
			continue;
		/* Add this port to the portvlan mask of the other ports
		 * in the bridge
		 */
		qca8k_reg_set(priv,
			      QCA8K_PORT_LOOKUP_CTRL(i),
			      BIT(port));
		if (i != port)
			port_mask |= BIT(i);
	}
	/* Add all other ports to this ports portvlan mask */
	qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(port),
		  QCA8K_PORT_LOOKUP_MEMBER, port_mask);

	return 0;
}

static void
qca8k_port_bridge_leave(struct dsa_switch *ds, int port, struct net_device *br)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	int i;

	for (i = 1; i < QCA8K_NUM_PORTS; i++) {
		if (dsa_to_port(ds, i)->bridge_dev != br)
			continue;
		/* Remove this port to the portvlan mask of the other ports
		 * in the bridge
		 */
		qca8k_reg_clear(priv,
				QCA8K_PORT_LOOKUP_CTRL(i),
				BIT(port));
	}

	/* Set the cpu port to be the only one in the portvlan mask of
	 * this port
	 */
	qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(port),
		  QCA8K_PORT_LOOKUP_MEMBER, BIT(QCA8K_CPU_PORT));
}

static int
qca8k_port_enable(struct dsa_switch *ds, int port,
		  struct phy_device *phy)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;

	qca8k_port_set_status(priv, port, 1);
	priv->port_sts[port].enabled = 1;

	return 0;
}

static void
qca8k_port_disable(struct dsa_switch *ds, int port,
		   struct phy_device *phy)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;

	qca8k_port_set_status(priv, port, 0);
	priv->port_sts[port].enabled = 0;
}

static int
qca8k_port_fdb_insert(struct qca8k_priv *priv, const u8 *addr,
		      u16 port_mask, u16 vid)
{
	/* Set the vid to the port vlan id if no vid is set */
	if (!vid)
		vid = 1;

	return qca8k_fdb_add(priv, addr, port_mask, vid,
			     QCA8K_ATU_STATUS_STATIC);
}

static int
qca8k_port_fdb_add(struct dsa_switch *ds, int port,
		   const unsigned char *addr, u16 vid)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	u16 port_mask = BIT(port);

	return qca8k_port_fdb_insert(priv, addr, port_mask, vid);
}

static int
qca8k_port_fdb_del(struct dsa_switch *ds, int port,
		   const unsigned char *addr, u16 vid)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	u16 port_mask = BIT(port);

	if (!vid)
		vid = 1;

	return qca8k_fdb_del(priv, addr, port_mask, vid);
}

static int
qca8k_port_fdb_dump(struct dsa_switch *ds, int port,
		    dsa_fdb_dump_cb_t *cb, void *data)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	struct qca8k_fdb _fdb = { 0 };
	int cnt = QCA8K_NUM_FDB_RECORDS;
	bool is_static;
	int ret = 0;

	mutex_lock(&priv->reg_mutex);
	while (cnt-- && !qca8k_fdb_next(priv, &_fdb, port)) {
		if (!_fdb.aging)
			break;
		is_static = (_fdb.aging == QCA8K_ATU_STATUS_STATIC);
		ret = cb(_fdb.mac, _fdb.vid, is_static, data);
		if (ret)
			break;
	}
	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static enum dsa_tag_protocol
qca8k_get_tag_protocol(struct dsa_switch *ds, int port)
{
	return DSA_TAG_PROTO_QCA;
}

static const struct dsa_switch_ops qca8k_switch_ops = {
	.get_tag_protocol	= qca8k_get_tag_protocol,
	.setup			= qca8k_setup,
	.adjust_link            = qca8k_adjust_link,
	.port_enable		= qca8k_port_enable,
	.port_disable		= qca8k_port_disable,
	.get_strings		= qca8k_get_strings,
	.get_ethtool_stats	= qca8k_get_ethtool_stats,
	.get_sset_count		= qca8k_get_sset_count,
	.get_mac_eee		= qca8k_get_mac_eee,
	.set_mac_eee		= qca8k_set_mac_eee,
	.port_stp_state_set	= qca8k_port_stp_state_set,
	.port_bridge_join	= qca8k_port_bridge_join,
	.port_bridge_leave	= qca8k_port_bridge_leave,
	.port_fdb_add		= qca8k_port_fdb_add,
	.port_fdb_del		= qca8k_port_fdb_del,
	.port_fdb_dump		= qca8k_port_fdb_dump,
};

#define AR40XX_NUM_PORTS	6

enum ar40xx_port_wrapper_cfg {
	PORT_WRAPPER_PSGMII = 0,
	PORT_WRAPPER_RGMII = 3,
};

#define AR40XX_PSGMII_MODE_CONTROL			0x1b4
#define   AR40XX_PSGMII_ATHR_CSCO_MODE_25M		BIT(0)

#define AR40XX_PSGMIIPHY_TX_CONTROL			0x288

#define AR40XX_REG_RGMII_CTRL				0x0004
#define AR40XX_REG_PORT_LOOKUP(_i)			(0x660 + (_i) * 0xc)
#define   AR40XX_PORT_LOOKUP_LOOPBACK			BIT(21)

#define AR40XX_PHY_SPEC_STATUS				0x11
#define   AR40XX_PHY_SPEC_STATUS_LINK			BIT(10)
#define   AR40XX_PHY_SPEC_STATUS_DUPLEX			BIT(13)
#define   AR40XX_PHY_SPEC_STATUS_SPEED			GENMASK(16, 14)

#define AR40XX_PSGMII_ID				5
#define AR40XX_PSGMII_CALB_NUM				100
#define AR40XX_MALIBU_PSGMII_MODE_CTRL			0x6d
#define AR40XX_MALIBU_PHY_PSGMII_MODE_CTRL_ADJUST_VAL	0x220c
#define AR40XX_MALIBU_PHY_MMD7_DAC_CTRL			0x801a
#define AR40XX_MALIBU_DAC_CTRL_MASK			0x380
#define AR40XX_MALIBU_DAC_CTRL_VALUE			0x280
#define AR40XX_MALIBU_PHY_RLP_CTRL			0x805a
#define AR40XX_PSGMII_TX_DRIVER_1_CTRL			0xb
#define AR40XX_MALIBU_PHY_PSGMII_REDUCE_SERDES_TX_AMP	0x8a
#define AR40XX_MALIBU_PHY_LAST_ADDR			4

static u32
psgmii_read(struct qca8k_priv *priv, int reg)
{
	u32 val;

	regmap_read(priv->psgmii, reg, &val);
	return val;
}

static void
psgmii_write(struct qca8k_priv *priv, int reg, u32 val)
{
	regmap_write(priv->psgmii, reg, val);
}

static void
qca8k_phy_mmd_write(struct qca8k_priv *priv, u32 phy_id,
		     u16 mmd_num, u16 reg_id, u16 reg_val)
{
	struct mii_bus *bus = priv->bus;

	mutex_lock(&bus->mdio_lock);
	__mdiobus_write(bus, phy_id, MII_MMD_CTRL, mmd_num);
	__mdiobus_write(bus, phy_id, MII_MMD_DATA, reg_id);
	__mdiobus_write(bus, phy_id, MII_MMD_CTRL, MII_MMD_CTRL_NOINCR | mmd_num);
	__mdiobus_write(bus, phy_id, MII_MMD_DATA, reg_val);
	mutex_unlock(&bus->mdio_lock);
}

static u16
qca8k_phy_mmd_read(struct qca8k_priv *priv, u32 phy_id,
		    u16 mmd_num, u16 reg_id)
{
	struct mii_bus *bus = priv->bus;
	u16 value;

	mutex_lock(&bus->mdio_lock);
	__mdiobus_write(bus, phy_id, MII_MMD_CTRL, mmd_num);
	__mdiobus_write(bus, phy_id, MII_MMD_DATA, reg_id);
	__mdiobus_write(bus, phy_id, MII_MMD_CTRL, MII_MMD_CTRL_NOINCR | mmd_num);
	value = __mdiobus_read(bus, phy_id, MII_MMD_DATA);
	mutex_unlock(&bus->mdio_lock);

	return value;
}

static void
ess_reset(struct qca8k_priv *priv)
{
	reset_control_assert(priv->ess_rst);

	mdelay(10);

	reset_control_deassert(priv->ess_rst);

	/* Waiting for all inner tables to be flushed and reinitialized.
	 * This takes between 5 and 10ms.
	 */
	mdelay(10);
}

static void
ar40xx_malibu_psgmii_ess_reset(struct qca8k_priv *priv)
{
	struct mii_bus *bus = priv->bus;
	u32 n;

	/* Reset phy psgmii */
	/* fix phy psgmii RX 20bit */
	mdiobus_write(bus, AR40XX_PSGMII_ID, 0x0, 0x005b);
	/* reset phy psgmii */
	mdiobus_write(bus, AR40XX_PSGMII_ID, 0x0, 0x001b);
	/* release reset phy psgmii */
	mdiobus_write(bus, AR40XX_PSGMII_ID, 0x0, 0x005b);

	for (n = 0; n < AR40XX_PSGMII_CALB_NUM; n++) {
		u16 status;

		status = qca8k_phy_mmd_read(priv, AR40XX_PSGMII_ID,
					     MDIO_MMD_PMAPMD, 0x28);
		if (status & BIT(0))
			break;

		/* Polling interval to check PSGMII PLL in malibu is ready
		 * the worst time is 8.67ms
		 * for 25MHz reference clock
		 * [512+(128+2048)*49]*80ns+100us
		 */
		mdelay(2);
	}

	/* check malibu psgmii calibration done end... */

	/* freeze phy psgmii RX CDR */
	mdiobus_write(bus, AR40XX_PSGMII_ID, 0x1a, 0x2230);

	ess_reset(priv);

	/* wait for the psgmii calibration to complete */
	for (n = 0; n < AR40XX_PSGMII_CALB_NUM; n++) {
		u32 status;

		status = psgmii_read(priv, 0xa0);
		if (status & BIT(0))
			break;

		/* Polling interval to check PSGMII PLL in ESS is ready */
		mdelay(2);
	}

	/* release phy psgmii RX CDR */
	mdiobus_write(bus, AR40XX_PSGMII_ID, 0x1a, 0x3230);
	/* release phy psgmii RX 20bit */
	mdiobus_write(bus, AR40XX_PSGMII_ID, 0x0, 0x005f);
}

static void
ar40xx_psgmii_single_phy_testing(struct qca8k_priv *priv, int phy)
{
	struct mii_bus *bus = priv->bus;
	u32 tx_ok, tx_error;
	u32 rx_ok, rx_error;
	u32 tx_ok_high16;
	u32 rx_ok_high16;
	u32 tx_all_ok, rx_all_ok;
	int j;

	mdiobus_write(bus, phy, MII_BMCR, BMCR_RESET | BMCR_ANENABLE);
	mdiobus_write(bus, phy, MII_BMCR, BMCR_LOOPBACK | BMCR_FULLDPLX |
					  BMCR_SPEED1000);

	for (j = 0; j < AR40XX_PSGMII_CALB_NUM; j++) {
		u16 status;

		status = mdiobus_read(bus, phy, AR40XX_PHY_SPEC_STATUS);
		if (status & AR40XX_PHY_SPEC_STATUS_LINK)
			break;

		/* the polling interval to check if the PHY link up or not
		  * maxwait_timer: 750 ms +/-10 ms
		  * minwait_timer : 1 us +/- 0.1us
		  * time resides in minwait_timer ~ maxwait_timer
		  * see IEEE 802.3 section 40.4.5.2
		  */
		mdelay(8);
	}

	/* enable check */
	qca8k_phy_mmd_write(priv, phy, 7, 0x8029, 0x0000);
	qca8k_phy_mmd_write(priv, phy, 7, 0x8029, 0x0003);

	/* start traffic */
	qca8k_phy_mmd_write(priv, phy, 7, 0x8020, 0xa000);

	/* wait precisely for all traffic end
	 * 4096(pkt num) * 1524(size) * 8ns (125MHz) = 49.9ms
	 */
	mdelay(50);

	/* check counter */
	tx_ok = qca8k_phy_mmd_read(priv, phy, 7, 0x802e);
	tx_ok_high16 = qca8k_phy_mmd_read(priv, phy, 7, 0x802d);
	tx_error = qca8k_phy_mmd_read(priv, phy, 7, 0x802f);
	rx_ok = qca8k_phy_mmd_read(priv, phy, 7, 0x802b);
	rx_ok_high16 = qca8k_phy_mmd_read(priv, phy, 7, 0x802a);
	rx_error = qca8k_phy_mmd_read(priv, phy, 7, 0x802c);
	tx_all_ok = tx_ok + (tx_ok_high16 << 16);
	rx_all_ok = rx_ok + (rx_ok_high16 << 16);

	if (tx_all_ok == 0x1000 && tx_error == 0) {
		/* success */
		priv->phy_t_status &= (~BIT(phy));
	} else {
		pr_info("PHY %d single test PSGMII issue happen!\n", phy);
		priv->phy_t_status |= BIT(phy);
	}

	mdiobus_write(bus, phy, MII_BMCR, BMCR_ANENABLE | BMCR_PDOWN |
					  BMCR_SPEED1000);
}

static void
ar40xx_psgmii_all_phy_testing(struct qca8k_priv *priv)
{
	struct mii_bus *bus = priv->bus;
	int phy, j;

	mdiobus_write(bus, 0x1f, MII_BMCR, BMCR_RESET | BMCR_ANENABLE);
	mdiobus_write(bus, 0x1f, MII_BMCR, BMCR_LOOPBACK | BMCR_FULLDPLX |
					   BMCR_SPEED1000);

	for (j = 0; j < AR40XX_PSGMII_CALB_NUM; j++) {
		for (phy = 0; phy < AR40XX_NUM_PORTS - 1; phy++) {
			u16 status;

			status = mdiobus_read(bus, phy, AR40XX_PHY_SPEC_STATUS);
			if (!(status & AR40XX_PHY_SPEC_STATUS_LINK))
				break;
		}

		if (phy >= (AR40XX_NUM_PORTS - 1))
			break;
		/* The polling interva to check if the PHY link up or not */
		mdelay(8);
	}
	/* enable package accounting */
	qca8k_phy_mmd_write(priv, 0x1f, 7, 0x8029, 0x0000);
	qca8k_phy_mmd_write(priv, 0x1f, 7, 0x8029, 0x0003);

	/* start traffic generator */
	qca8k_phy_mmd_write(priv, 0x1f, 7, 0x8020, 0xa000);

	/* wait for the traffic to die down.
	 * 4096 Packets * 1524 Bytes/Packet * 8 ns/Byte (125MHz) = 49.9ms
	 */
	mdelay(50);

	for (phy = 0; phy < AR40XX_NUM_PORTS - 1; phy++) {
		u32 tx_ok, tx_error;
		u32 rx_ok, rx_error;
		u32 tx_ok_high16;
		u32 rx_ok_high16;
		u32 tx_all_ok, rx_all_ok;

		/* check counter */
		tx_ok = qca8k_phy_mmd_read(priv, phy, 7, 0x802e);
		tx_ok_high16 = qca8k_phy_mmd_read(priv, phy, 7, 0x802d);
		tx_error = qca8k_phy_mmd_read(priv, phy, 7, 0x802f);
		rx_ok = qca8k_phy_mmd_read(priv, phy, 7, 0x802b);
		rx_ok_high16 = qca8k_phy_mmd_read(priv, phy, 7, 0x802a);
		rx_error = qca8k_phy_mmd_read(priv, phy, 7, 0x802c);
		tx_all_ok = tx_ok + (tx_ok_high16 << 16);
		rx_all_ok = rx_ok + (rx_ok_high16 << 16);

		if (tx_all_ok == 4096 && tx_error == 0) {
			/* success */
			priv->phy_t_status &= ~BIT(phy + 8);
		} else {
			pr_info("PHY%d test see issue!\n", phy);
			priv->phy_t_status |= BIT(phy + 8);
		}
	}

	pr_debug("PHY all test 0x%x \r\n", priv->phy_t_status);
}

static void
ar40xx_psgmii_self_test(struct qca8k_priv *priv)
{
	struct mii_bus *bus = priv->bus;
	u32 i, phy;

	ar40xx_malibu_psgmii_ess_reset(priv);

	/* switch to access MII reg for copper */
	mdiobus_write(bus, 4, 0x1f, 0x8500);

	for (phy = 0; phy < AR40XX_NUM_PORTS - 1; phy++) {
		/*enable phy mdio broadcast write*/
		qca8k_phy_mmd_write(priv, phy, 7, 0x8028, 0x801f);
	}

	/* force no link by power down */
	mdiobus_write(bus, 0x1f, MII_BMCR, BMCR_ANENABLE | BMCR_PDOWN |
					   BMCR_SPEED1000);

	/* Setup packet generator for loopback calibration */
	qca8k_phy_mmd_write(priv, 0x1f, 7, 0x8021, 0x1000); /* 4096 Packets */
	qca8k_phy_mmd_write(priv, 0x1f, 7, 0x8062, 0x05e0); /* 1524 Bytes */

	/* fix mdi status */
	mdiobus_write(bus, 0x1f, 0x10, 0x6800);
	for (i = 0; i < AR40XX_PSGMII_CALB_NUM; i++) {
		priv->phy_t_status = 0;

		for (phy = 0; phy < AR40XX_NUM_PORTS - 1; phy++) {
			qca8k_rmw(priv, AR40XX_REG_PORT_LOOKUP(phy + 1),
				AR40XX_PORT_LOOKUP_LOOPBACK,
				AR40XX_PORT_LOOKUP_LOOPBACK);
		}

		for (phy = 0; phy < AR40XX_NUM_PORTS - 1; phy++)
			ar40xx_psgmii_single_phy_testing(priv, phy);

		ar40xx_psgmii_all_phy_testing(priv);

		if (priv->phy_t_status)
			ar40xx_malibu_psgmii_ess_reset(priv);
		else
			break;
	}

	if (i >= AR40XX_PSGMII_CALB_NUM)
		pr_info("PSGMII cannot recover\n");
	else
		pr_debug("PSGMII recovered after %d times reset\n", i);

	/* configuration recover */
	/* packet number */
	qca8k_phy_mmd_write(priv, 0x1f, 7, 0x8021, 0x0);
	/* disable check */
	qca8k_phy_mmd_write(priv, 0x1f, 7, 0x8029, 0x0);
	/* disable traffic */
	qca8k_phy_mmd_write(priv, 0x1f, 7, 0x8020, 0x0);
}

static void
ar40xx_psgmii_self_test_clean(struct qca8k_priv *priv)
{
	struct mii_bus *bus = priv->bus;
	int phy;

	/* disable phy internal loopback */
	mdiobus_write(bus, 0x1f, 0x10, 0x6860);
	mdiobus_write(bus, 0x1f, MII_BMCR, BMCR_ANENABLE | BMCR_RESET |
					   BMCR_SPEED1000);

	for (phy = 0; phy < AR40XX_NUM_PORTS - 1; phy++) {
		/* disable mac loop back */
		qca8k_rmw(priv, AR40XX_REG_PORT_LOOKUP(phy + 1),
				AR40XX_PORT_LOOKUP_LOOPBACK, 0);

		/* disable phy mdio broadcast write */
		qca8k_phy_mmd_write(priv, phy, 7, 0x8028, 0x001f);
	}
}

static void
ar40xx_malibu_init(struct qca8k_priv *priv)
{
	int i;
	u16 val;

	/* war to enable AZ transmitting ability */
	qca8k_phy_mmd_write(priv, AR40XX_PSGMII_ID, 1,
		      AR40XX_MALIBU_PSGMII_MODE_CTRL,
		      AR40XX_MALIBU_PHY_PSGMII_MODE_CTRL_ADJUST_VAL);

	for (i = 0; i < AR40XX_NUM_PORTS - 1; i++) {

		/* change malibu control_dac */
		val = qca8k_phy_mmd_read(priv, i, 7, AR40XX_MALIBU_PHY_MMD7_DAC_CTRL);
		val &= ~AR40XX_MALIBU_DAC_CTRL_MASK;
		val |= AR40XX_MALIBU_DAC_CTRL_VALUE;
		qca8k_phy_mmd_write(priv, i, 7, AR40XX_MALIBU_PHY_MMD7_DAC_CTRL, val);

		if (i == AR40XX_MALIBU_PHY_LAST_ADDR) {
			/* avoid PHY to get into hibernation */
			val = qca8k_phy_mmd_read(priv, i, 3,
						  AR40XX_MALIBU_PHY_RLP_CTRL);
			val &= (~(1<<1));
			qca8k_phy_mmd_write(priv, i, 3,
					     AR40XX_MALIBU_PHY_RLP_CTRL, val);
		}
	}

	/* adjust psgmii serdes tx amp */
	mdiobus_write(priv->bus, AR40XX_PSGMII_ID,
		      AR40XX_PSGMII_TX_DRIVER_1_CTRL,
		      AR40XX_MALIBU_PHY_PSGMII_REDUCE_SERDES_TX_AMP);
}

static void
ar40xx_mac_mode_init(struct qca8k_priv *priv)
{
	switch (priv->mac_mode) {
	case PORT_WRAPPER_PSGMII:
		ar40xx_malibu_init(priv);
		ar40xx_psgmii_self_test(priv);
		ar40xx_psgmii_self_test_clean(priv);

		psgmii_write(priv, AR40XX_PSGMII_MODE_CONTROL, 0x2200);
		psgmii_write(priv, AR40XX_PSGMIIPHY_TX_CONTROL, 0x8380);
		break;
	case PORT_WRAPPER_RGMII:
		qca8k_write(priv, AR40XX_REG_RGMII_CTRL, BIT(10));
		break;
	}
}

#ifdef QM_ERROR_WAR
/* Start of qm error WAR */

#define AR40XX_PORT_LINK_UP 1
#define AR40XX_PORT_LINK_DOWN 0
#define AR40XX_QM_NOT_EMPTY  1
#define AR40XX_QM_EMPTY  0

static
int ar40xx_force_1g_full(struct qca8k_priv *priv, u32 port_id)
{
	u32 reg;

	if (port_id < 0 || port_id > 6)
		return -1;

	reg = QCA8K_REG_PORT_STATUS(port_id);
	return qca8k_rmw(priv, reg, QCA8K_PORT_STATUS_SPEED,
			(QCA8K_PORT_STATUS_SPEED_1000 | QCA8K_PORT_STATUS_DUPLEX));
}

static
int ar40xx_get_qm_status(struct qca8k_priv *priv,
			 u32 port_id, u32 *qm_buffer_err)
{
	u32 reg;
	u32 qm_val;

	if (port_id < 1 || port_id > 5) {
		*qm_buffer_err = 0;
		return -1;
	}

	if (port_id < 4) {
		reg = AR40XX_REG_QM_PORT0_3_QNUM;
		qca8k_write(priv, AR40XX_REG_QM_DEBUG_ADDR, reg);
		qm_val = qca8k_read(priv, AR40XX_REG_QM_DEBUG_VALUE);
		/* every 8 bits for each port */
		*qm_buffer_err = (qm_val >> (port_id * 8)) & 0xFF;
	} else {
		reg = AR40XX_REG_QM_PORT4_6_QNUM;
		qca8k_write(priv, AR40XX_REG_QM_DEBUG_ADDR, reg);
		qm_val = qca8k_read(priv, AR40XX_REG_QM_DEBUG_VALUE);
		/* every 8 bits for each port */
		*qm_buffer_err = (qm_val >> ((port_id-4) * 8)) & 0xFF;
	}

	return 0;
}

static void
ar40xx_sw_mac_polling_task(struct qca8k_priv *priv)
{
	static int task_count;
	u32 i;
	u32 reg, value;
	u32 link, speed, duplex;
	u32 qm_buffer_err;
	u16 port_phy_status[AR40XX_NUM_PORTS];
	static u32 qm_err_cnt[AR40XX_NUM_PORTS] = {0, 0, 0, 0, 0, 0};
	static u32 link_cnt[AR40XX_NUM_PORTS] = {0, 0, 0, 0, 0, 0};
	struct mii_bus *bus = NULL;

	if (!priv || !priv->bus)
		return;

	bus = priv->bus;

	++task_count;

	for (i = 1; i < AR40XX_NUM_PORTS; ++i) {
		port_phy_status[i] =
			mdiobus_read(bus, i-1, AR40XX_PHY_SPEC_STATUS);
		speed = link = duplex = port_phy_status[i];
		speed &= AR40XX_PHY_SPEC_STATUS_SPEED;
		speed >>= 14;
		link &= AR40XX_PHY_SPEC_STATUS_LINK;
		link >>= 10;
		duplex &= AR40XX_PHY_SPEC_STATUS_DUPLEX;
		duplex >>= 13;

		if (link != priv->ar40xx_port_old_link[i]) {
			++link_cnt[i];
			/* Up --> Down */
			if ((priv->ar40xx_port_old_link[i] ==
					AR40XX_PORT_LINK_UP) &&
			    (link == AR40XX_PORT_LINK_DOWN)) {
				/* LINK_EN disable(MAC force mode)*/
				reg = QCA8K_REG_PORT_STATUS(i);
				qca8k_rmw(priv, reg,
					QCA8K_PORT_STATUS_LINK_AUTO, 0);

				/* Check queue buffer */
				qm_err_cnt[i] = 0;
				ar40xx_get_qm_status(priv, i, &qm_buffer_err);
				if (qm_buffer_err) {
					priv->ar40xx_port_qm_buf[i] =
						AR40XX_QM_NOT_EMPTY;
				} else {
					u16 phy_val = 0;

					priv->ar40xx_port_qm_buf[i] =
						AR40XX_QM_EMPTY;
					ar40xx_force_1g_full(priv, i);
					/* Ref:QCA8337 Datasheet,Clearing
					 * MENU_CTRL_EN prevents phy to
					 * stuck in 100BT mode when
					 * bringing up the link
					 */
					ar40xx_phy_dbg_read(priv, i-1,
							    AR40XX_PHY_DEBUG_0,
							    &phy_val);
					phy_val &= (~AR40XX_PHY_MANU_CTRL_EN);
					ar40xx_phy_dbg_write(priv, i-1,
							     AR40XX_PHY_DEBUG_0,
							     phy_val);
				}
				priv->ar40xx_port_old_link[i] = link;
			} else if ((priv->ar40xx_port_old_link[i] ==
						AR40XX_PORT_LINK_DOWN) &&
					(link == AR40XX_PORT_LINK_UP)) {
				/* Down --> Up */
				if (priv->port_link_up[i] < 1) {
					++priv->port_link_up[i];
				} else {
					/* Change port status */
					reg = QCA8K_REG_PORT_STATUS(i);
					value = qca8k_read(priv, reg);
					priv->port_link_up[i] = 0;

					value &= ~(QCA8K_PORT_STATUS_DUPLEX |
						   QCA8K_PORT_STATUS_SPEED);
					value |= speed | (duplex ? BIT(6) : 0);
					/**/qca8k_write(priv, reg, value);
					/* clock switch need such time
					 * to avoid glitch
					 */
					usleep_range(100, 200);

					value |= QCA8K_PORT_STATUS_LINK_AUTO;
					qca8k_write(priv, reg, value);
					/* HW need such time to make sure link
					 * stable before enable MAC
					 */
					usleep_range(100, 200);

					if (speed == QCA8K_PORT_STATUS_SPEED_100) {
						u16 phy_val = 0;
						/* Enable @100M, if down to 10M
						 * clock will change smoothly
						 */
						ar40xx_phy_dbg_read(priv, i-1,
								    0,
								    &phy_val);
						phy_val |=
							AR40XX_PHY_MANU_CTRL_EN;
						ar40xx_phy_dbg_write(priv, i-1,
								     0,
								     phy_val);
					}
					priv->ar40xx_port_old_link[i] = link;
				}
			}
		}

		if (priv->ar40xx_port_qm_buf[i] == AR40XX_QM_NOT_EMPTY) {
			/* Check QM */
			ar40xx_get_qm_status(priv, i, &qm_buffer_err);
			if (qm_buffer_err) {
				++qm_err_cnt[i];
			} else {
				priv->ar40xx_port_qm_buf[i] =
						AR40XX_QM_EMPTY;
				qm_err_cnt[i] = 0;
				ar40xx_force_1g_full(priv, i);
			}
		}
	}
}

#define AR40XX_QM_WORK_DELAY    100

static void
ar40xx_qm_err_check_work_task(struct work_struct *work)
{
	struct qca8k_priv *priv = container_of(work, struct qca8k_priv,
					qm_dwork.work);

	mutex_lock(&priv->qm_lock);

	ar40xx_sw_mac_polling_task(priv);

	mutex_unlock(&priv->qm_lock);

	schedule_delayed_work(&priv->qm_dwork,
			      msecs_to_jiffies(AR40XX_QM_WORK_DELAY));
}

static int
ar40xx_qm_err_check_work_start(struct qca8k_priv *priv)
{
	mutex_init(&priv->qm_lock);

	INIT_DELAYED_WORK(&priv->qm_dwork, ar40xx_qm_err_check_work_task);

	schedule_delayed_work(&priv->qm_dwork,
			      msecs_to_jiffies(AR40XX_QM_WORK_DELAY));

	return 0;
}
#else
static int
ar40xx_qm_err_check_work_start(struct qca8k_priv *priv)
{
	return 0;
}
#endif


static void
qca8k_dsa_init_work(struct work_struct *work)
{
	struct qca8k_priv *priv = container_of(work, struct qca8k_priv, dsa_init.work);
	struct device *parent = priv->pdev->dev.parent;
	int ret;

	ret = dsa_register_switch(priv->ds);

	switch (ret) {
	case 0:
		return;

	case -EPROBE_DEFER:
		dev_dbg(&priv->pdev->dev, "dsa_register_switch defered.\n");
		schedule_delayed_work(&priv->dsa_init, msecs_to_jiffies(200));
		return;

	default:
		dev_err(&priv->pdev->dev, "dsa_register_switch failed with (%d).\n", ret);
		/* unbind anything failed */
		if (parent)
			device_lock(parent);

		device_release_driver(&priv->pdev->dev);
		if (parent)
			device_unlock(parent);
		return;
	}
}

static int __init
qca8k_mmio_probe(struct platform_device *pdev)
{
	struct qca8k_priv *priv;
	struct device_node *np = pdev->dev.of_node, *mii_np;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);

	priv->pdev = pdev;
	mutex_init(&priv->reg_mutex);

	priv->ess_clk = of_clk_get_by_name(np, "ess_clk");
	if (IS_ERR(priv->ess_clk)) {
		dev_err(&pdev->dev, "Failed to get ess_clk\n");
		return PTR_ERR(priv->ess_clk);
	}

	priv->ess_rst = devm_reset_control_get(&pdev->dev, "ess_rst");
	if (IS_ERR(priv->ess_rst)) {
		dev_err(&pdev->dev, "Failed to get ess_rst control!\n");
		return PTR_ERR(priv->ess_rst);
	}

	ret = of_property_read_u32(np, "mac-mode", &priv->mac_mode);
	if (ret < 0)
		return -EINVAL;

	priv->base = syscon_node_to_regmap(np);
	if (IS_ERR_OR_NULL(priv->base))
		return -EINVAL;

	priv->psgmii = syscon_regmap_lookup_by_phandle(np, "psgmii-phy");
	if (IS_ERR_OR_NULL(priv->psgmii))
		return -EINVAL;

	mii_np = of_parse_phandle(np, "mii", 0);
	if (!mii_np)
		return -EINVAL;

	priv->bus = of_mdio_find_bus(mii_np);
	of_node_put(mii_np);
	if (!priv->bus)
		return -EPROBE_DEFER;

	priv->ds = dsa_switch_alloc(&pdev->dev, DSA_MAX_PORTS);
	if (!priv->ds)
		return -ENOMEM;

	priv->ds->priv = priv;
	priv->ds->ops = &qca8k_switch_ops;

	clk_prepare_enable(priv->ess_clk);

	platform_set_drvdata(pdev, priv);

	ar40xx_qm_err_check_work_start(priv);

	ess_reset(priv);

	ar40xx_mac_mode_init(priv);

	/* Ok. What's going on with the delayed dsa_switch_register?!
	 *
	 * On Bootup, this switch driver loads before the ethernet
	 * driver. This causes a problem in dsa_register_switch when
	 * it parses the tree and encounters the not-yet-ready
	 * 	"ethernet = <&gmac>;" property.
	 *
	 * Which will err with -EPROBE_DEFER. Normally this should be
	 * OK and the driver will just get loaded at a later time.
	 * However, the EthernetSubSystem (ESS for short) really doesn't
	 * like being resetted more than once in this fashion and will
	 * "lock it up for good"... like "real good".
	 *
	 * So far, only a reboot can "unwedge" it, which is not what
	 * we want.
	 *
	 * So this workaround (running dsa_register_switch in a
	 * workqueue task) is employed to fix this unknown issue within
	 * the SoC for now.
	 */

	INIT_DELAYED_WORK(&priv->dsa_init, qca8k_dsa_init_work);
	schedule_delayed_work(&priv->dsa_init, msecs_to_jiffies(1000));

	return 0;
}

static const struct of_device_id qca8k_of_match[] = {
	{ .compatible = "qca,qca8337-mmio" },
	{ /* sentinel */ },
};

static struct platform_driver qca8kmmio_driver = {
	.driver = {
		.name = "qca8k",
		.of_match_table = qca8k_of_match,
	},
};

module_platform_driver_probe(qca8kmmio_driver, qca8k_mmio_probe);

MODULE_AUTHOR("Mathieu Olivari, John Crispin <john@phrozen.org>");
MODULE_DESCRIPTION("Driver for QCA8K ethernet switch family");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:qca8k");
