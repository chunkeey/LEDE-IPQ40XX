/*
 * Copyright (c) 2017 John Crispin <john@phrozen.org>
 * Copyright (c) 2019 Christian Lamparter <chunkeey@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <net/dsa.h>
#include <net/switchdev.h>
#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/if_bridge.h>
#include <linux/reset.h>
#include <linux/etherdevice.h>

#include "qca8k.h"

#define AR40XX_NUM_PORTS	6

struct qca8k_mmio_priv {
	struct qca8k_priv qca8k;

	struct regmap *psgmii;

	struct reset_control *ess_rst;

	struct clk *ess_clk;

	u32 phy_t_status;
	u32 mac_mode;

	/* qm WAR */
	struct mutex qm_lock;
	struct delayed_work qm_dwork;
	u32 port_link_up[AR40XX_NUM_PORTS];
	u32 ar40xx_port_old_link[AR40XX_NUM_PORTS];
	u32 ar40xx_port_qm_buf[AR40XX_NUM_PORTS];
};

static u32
qca8k_mmio_read(struct qca8k_priv *priv, u32 reg)
{
	u32 val;

	regmap_read(priv->base, reg, &val);

	return val;
}

static void
qca8k_mmio_write(struct qca8k_priv *priv, u32 reg, u32 val)
{
	regmap_write(priv->base, reg, val);
}

static u32
qca8k_mmio_rmw(struct qca8k_priv *priv, u32 reg, u32 mask, u32 val)
{
	u32 _val = qca8k_mmio_read(priv, reg);

	_val &= ~mask;
	_val |= val;

	qca8k_mmio_write(priv, reg, _val);
	return _val;
}

static struct qca8k_reg_ops qca8k_mmio_reg_ops = {
	.read = qca8k_mmio_read,
	.write = qca8k_mmio_write,
	.rmw = qca8k_mmio_rmw,
};

static void
qca8k_mmio_phy_mmd_write(struct qca8k_mmio_priv *priv, u32 phy_addr,
			 u16 devad, u16 regnum, u16 val)
{
	struct mii_bus *bus = priv->qca8k.bus;

	mutex_lock(&bus->mdio_lock);
	__mdiobus_write(bus, phy_addr,
			MII_MMD_CTRL, devad);
	__mdiobus_write(bus, phy_addr,
			MII_MMD_DATA, regnum);
	__mdiobus_write(bus, phy_addr,
			MII_MMD_CTRL,
			MII_MMD_CTRL_NOINCR | devad);
	__mdiobus_write(bus, phy_addr,
			MII_MMD_DATA, val);
	mutex_unlock(&bus->mdio_lock);
}

static u16
qca8k_mmio_phy_mmd_read(struct qca8k_mmio_priv *priv, u32 phy_addr,
			u16 devad, u16 regnum)
{
	u16 value;
	struct mii_bus *bus = priv->qca8k.bus;

	mutex_lock(&bus->mdio_lock);
	__mdiobus_write(bus, phy_addr,
			MII_MMD_CTRL, devad);
	__mdiobus_write(bus, phy_addr,
			MII_MMD_DATA, regnum);
	__mdiobus_write(bus, phy_addr,
			MII_MMD_CTRL,
			MII_MMD_CTRL_NOINCR | devad);
	value = __mdiobus_read(bus, phy_addr, MII_MMD_DATA);
	mutex_unlock(&bus->mdio_lock);
	return value;
}

#define MII_ATH_DBG_ADDR		0x1d
#define MII_ATH_DBG_DATA		0x1e

static void
qca8k_mmio_phy_dbg_write(struct qca8k_mmio_priv *priv, int phy_addr,
		     u16 dbg_addr, u16 dbg_data)
{
	struct mii_bus *bus = priv->qca8k.bus;

	mutex_lock(&bus->mdio_lock);
	__mdiobus_write(bus, phy_addr, MII_ATH_DBG_ADDR, dbg_addr);
	__mdiobus_write(bus, phy_addr, MII_ATH_DBG_DATA, dbg_data);
	mutex_unlock(&bus->mdio_lock);
}

static void
qca8k_mmio_phy_dbg_read(struct qca8k_mmio_priv *priv, int phy_addr,
		    u16 dbg_addr, u16 *dbg_data)
{
	struct mii_bus *bus = priv->qca8k.bus;

	mutex_lock(&bus->mdio_lock);
	__mdiobus_write(bus, phy_addr, MII_ATH_DBG_ADDR, dbg_addr);
	*dbg_data = __mdiobus_read(bus, phy_addr, MII_ATH_DBG_DATA);
	mutex_unlock(&bus->mdio_lock);
}

static u32
qca8k_mmio_psgmii_read(struct qca8k_mmio_priv *priv, u32 reg)
{
	u32 val;

	regmap_read(priv->psgmii, reg, &val);

	return val;
}

static void
qca8k_mmio_psgmii_write(struct qca8k_mmio_priv *priv, u32 reg, u32 val)
{
	regmap_write(priv->psgmii, reg, val);
}

static void
qca8k_mmio_ess_reset(struct qca8k_mmio_priv *priv)
{
	reset_control_assert(priv->ess_rst);
	mdelay(10);
	reset_control_deassert(priv->ess_rst);
	/* Wait 5~10ms for all inner tables are set. */
	mdelay(10);

	dev_info(priv->qca8k.dev, "ESS reset.\n");
}

#define AR40XX_REG_PORT_LOOKUP(_i)		(0x660 + (_i) * 0xc)
#define   AR40XX_PORT_LOOKUP_MEMBER		BITS(0, 7)
#define   AR40XX_PORT_LOOKUP_IN_MODE		BITS(8, 2)
#define   AR40XX_PORT_LOOKUP_IN_MODE_S		8
#define   AR40XX_PORT_LOOKUP_STATE		BITS(16, 3)
#define   AR40XX_PORT_LOOKUP_STATE_S		16
#define   AR40XX_PORT_LOOKUP_LEARN		BIT(20)
#define   AR40XX_PORT_LOOKUP_LOOPBACK		BIT(21)
#define   AR40XX_PORT_LOOKUP_ING_MIRROR_EN	BIT(25)

static int
qca8k_wait_bit(struct qca8k_mmio_priv *priv, u32 reg, u32 mask, u32 val)
{
	int timeout = 20;
	u32 t;

	while (1) {
		t = qca8k_mmio_read(&priv->qca8k, reg);
		if ((t & mask) == val)
			return 0;

		if (timeout-- <= 0)
			break;

		usleep_range(10, 20);
	}

	dev_err(priv->qca8k.dev, "timeout for reg %04x: %08x & %08x != %08x\n",
		reg, t, mask, val);
	return -ETIMEDOUT;
}

#define AR40XX_REG_ATU_FUNC			0x60c
#define   AR40XX_ATU_FUNC_OP			BITS(0, 4)
#define   AR40XX_ATU_FUNC_OP_NOOP		0x0
#define   AR40XX_ATU_FUNC_OP_FLUSH		0x1
#define   AR40XX_ATU_FUNC_OP_LOAD		0x2
#define   AR40XX_ATU_FUNC_OP_PURGE		0x3
#define   AR40XX_ATU_FUNC_OP_FLUSH_LOCKED	0x4
#define   AR40XX_ATU_FUNC_OP_FLUSH_UNICAST	0x5
#define   AR40XX_ATU_FUNC_OP_GET_NEXT		0x6
#define   AR40XX_ATU_FUNC_OP_SEARCH_MAC		0x7
#define   AR40XX_ATU_FUNC_OP_CHANGE_TRUNK	0x8
#define   AR40XX_ATU_FUNC_BUSY			BIT(31)

static int
qca8k_atu_flush(struct qca8k_mmio_priv *priv)
{
	int ret;

	ret = qca8k_wait_bit(priv, AR40XX_REG_ATU_FUNC,
			      AR40XX_ATU_FUNC_BUSY, 0);
	if (!ret)
		qca8k_mmio_write(&priv->qca8k, AR40XX_REG_ATU_FUNC,
			     AR40XX_ATU_FUNC_OP_FLUSH |
			     AR40XX_ATU_FUNC_BUSY);

	return ret;
}

/* Start of psgmii self test */
#define AR40XX_PSGMII_ID	5
#define AR40XX_PSGMII_CALB_NUM	100

static void
psgmii_ess_reset(struct qca8k_mmio_priv *priv)
{
	u32 n;
	struct mii_bus *bus = priv->qca8k.bus;
	/* reset phy psgmii */
	/* fix phy psgmii RX 20bit */
	mdiobus_write(bus, 5, 0x0, 0x005b);
	/* reset phy psgmii */
	mdiobus_write(bus, 5, 0x0, 0x001b);
	/* release reset phy psgmii */
	mdiobus_write(bus, 5, 0x0, 0x005b);

	for (n = 0; n < AR40XX_PSGMII_CALB_NUM; n++) {
		u16 status;

		status = qca8k_mmio_phy_mmd_read(priv, 5, 1, 0x28);
		if (status & BIT(0))
			break;
		/* Polling interval to check PSGMII PLL in malibu is ready
		  * the worst time is 8.67ms
		  * for 25MHz reference clock
		  * [512+(128+2048)*49]*80ns+100us
		  */
		mdelay(2);
	}

	/*check malibu psgmii calibration done end..*/

	/*freeze phy psgmii RX CDR*/
	mdiobus_write(bus, 5, 0x1a, 0x2230);

	qca8k_mmio_ess_reset(priv);

	/*check psgmii calibration done start*/
	for (n = 0; n < AR40XX_PSGMII_CALB_NUM; n++) {
		u32 status;

		status = qca8k_mmio_psgmii_read(priv, 0xa0);
		if (status & BIT(0))
			break;
		/* Polling interval to check PSGMII PLL in ESS is ready */
		mdelay(2);
	}

	/* check dakota psgmii calibration done end..*/

	/* relesae phy psgmii RX CDR */
	mdiobus_write(bus, 5, 0x1a, 0x3230);
	/* release phy psgmii RX 20bit */
	mdiobus_write(bus, 5, 0x0, 0x005f);
}

#define BITS(_s, _n)	(((1UL << (_n)) - 1) << _s)

#define AR40XX_PHY_SPEC_STATUS 0x11
#define   AR40XX_PHY_SPEC_STATUS_LINK		BIT(10)
#define   AR40XX_PHY_SPEC_STATUS_DUPLEX		BIT(13)
#define   AR40XX_PHY_SPEC_STATUS_SPEED		BITS(14, 2)

static void
psgmii_single_phy_testing(struct qca8k_mmio_priv *priv, int phy)
{
	int j;
	u32 tx_ok, tx_error;
	u32 rx_ok, rx_error;
	u32 tx_ok_high16;
	u32 rx_ok_high16;
	u32 tx_all_ok, rx_all_ok;
	struct mii_bus *bus = priv->qca8k.bus;

	mdiobus_write(bus, phy, 0, 0x9000);
	mdiobus_write(bus, phy, 0, 0x4140);

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
	qca8k_mmio_phy_mmd_write(priv, phy, 7, 0x8029, 0x0000);
	qca8k_mmio_phy_mmd_write(priv, phy, 7, 0x8029, 0x0003);

	/* start traffic */
	qca8k_mmio_phy_mmd_write(priv, phy, 7, 0x8020, 0xa000);
	/* wait for all traffic end
	  * 4096(pkt num)*1524(size)*8ns(125MHz)=49.9ms
	  */
	mdelay(50);

	/* check counter */
	tx_ok = qca8k_mmio_phy_mmd_read(priv, phy, 7, 0x802e);
	tx_ok_high16 = qca8k_mmio_phy_mmd_read(priv, phy, 7, 0x802d);
	tx_error = qca8k_mmio_phy_mmd_read(priv, phy, 7, 0x802f);
	rx_ok = qca8k_mmio_phy_mmd_read(priv, phy, 7, 0x802b);
	rx_ok_high16 = qca8k_mmio_phy_mmd_read(priv, phy, 7, 0x802a);
	rx_error = qca8k_mmio_phy_mmd_read(priv, phy, 7, 0x802c);
	tx_all_ok = tx_ok + (tx_ok_high16 << 16);
	rx_all_ok = rx_ok + (rx_ok_high16 << 16);
	if (tx_all_ok == 0x1000 && tx_error == 0) {
		/* success */
		priv->phy_t_status &= (~BIT(phy));
	} else {
		dev_err(priv->qca8k.dev, "PHY %d single test PSGMII issue happen!\n", phy);
		priv->phy_t_status |= BIT(phy);
	}

	mdiobus_write(bus, phy, 0x0, 0x1840);
}

static void
psgmii_all_phy_testing(struct qca8k_mmio_priv *priv)
{
	int phy, j;
	struct mii_bus *bus = priv->qca8k.bus;

	mdiobus_write(bus, 0x1f, 0x0, 0x9000);
	mdiobus_write(bus, 0x1f, 0x0, 0x4140);

	for (j = 0; j < AR40XX_PSGMII_CALB_NUM; j++) {
		for (phy = 0; phy < AR40XX_NUM_PORTS - 1; phy++) {
			u16 status;

			status = mdiobus_read(bus, phy, 0x11);
			if (!(status & BIT(10)))
				break;
		}

		if (phy >= (AR40XX_NUM_PORTS - 1))
			break;
		/* The polling interva to check if the PHY link up or not */
		mdelay(8);
	}
	/* enable check */
	qca8k_mmio_phy_mmd_write(priv, 0x1f, 7, 0x8029, 0x0000);
	qca8k_mmio_phy_mmd_write(priv, 0x1f, 7, 0x8029, 0x0003);

	/* start traffic */
	qca8k_mmio_phy_mmd_write(priv, 0x1f, 7, 0x8020, 0xa000);
	/* wait for all traffic end
	  * 4096(pkt num)*1524(size)*8ns(125MHz)=49.9ms
	  */
	mdelay(50);

	for (phy = 0; phy < AR40XX_NUM_PORTS - 1; phy++) {
		u32 tx_ok, tx_error;
		u32 rx_ok, rx_error;
		u32 tx_ok_high16;
		u32 rx_ok_high16;
		u32 tx_all_ok, rx_all_ok;

		/* check counter */
		tx_ok = qca8k_mmio_phy_mmd_read(priv, phy, 7, 0x802e);
		tx_ok_high16 = qca8k_mmio_phy_mmd_read(priv, phy, 7, 0x802d);
		tx_error = qca8k_mmio_phy_mmd_read(priv, phy, 7, 0x802f);
		rx_ok = qca8k_mmio_phy_mmd_read(priv, phy, 7, 0x802b);
		rx_ok_high16 = qca8k_mmio_phy_mmd_read(priv, phy, 7, 0x802a);
		rx_error = qca8k_mmio_phy_mmd_read(priv, phy, 7, 0x802c);
		tx_all_ok = tx_ok + (tx_ok_high16<<16);
		rx_all_ok = rx_ok + (rx_ok_high16<<16);
		if (tx_all_ok == 0x1000 && tx_error == 0) {
			/* success */
			priv->phy_t_status &= ~BIT(phy + 8);
		} else {
			dev_info(priv->qca8k.dev, "PHY%d test see issue!\n", phy);
			priv->phy_t_status |= BIT(phy + 8);
		}
	}

	dev_dbg(priv->qca8k.dev, "PHY all test 0x%x \r\n",
		priv->phy_t_status);
}

void
psgmii_self_test(struct qca8k_mmio_priv *priv)
{
	u32 i, phy;
	struct mii_bus *bus = priv->qca8k.bus;

	psgmii_ess_reset(priv);

	/* switch to access MII reg for copper */
	mdiobus_write(bus, 4, 0x1f, 0x8500);
	for (phy = 0; phy < AR40XX_NUM_PORTS - 1; phy++) {
		/* enable phy mdio broadcast write */
		qca8k_mmio_phy_mmd_write(priv, phy, 7, 0x8028, 0x801f);
	}
	/* force no link by power down */
	mdiobus_write(bus, 0x1f, 0x0, 0x1840);
	/* packet number */
	qca8k_mmio_phy_mmd_write(priv, 0x1f, 7, 0x8021, 0x1000);
	qca8k_mmio_phy_mmd_write(priv, 0x1f, 7, 0x8062, 0x05e0);

	/* fix mdi status */
	mdiobus_write(bus, 0x1f, 0x10, 0x6800);
	for (i = 0; i < AR40XX_PSGMII_CALB_NUM; i++) {
		priv->phy_t_status = 0;

		for (phy = 0; phy < AR40XX_NUM_PORTS - 1; phy++) {
			qca8k_mmio_rmw(&priv->qca8k, AR40XX_REG_PORT_LOOKUP(phy + 1),
				AR40XX_PORT_LOOKUP_LOOPBACK,
				AR40XX_PORT_LOOKUP_LOOPBACK);
		}

		for (phy = 0; phy < AR40XX_NUM_PORTS - 1; phy++)
			psgmii_single_phy_testing(priv, phy);

		psgmii_all_phy_testing(priv);

		if (priv->phy_t_status)
			psgmii_ess_reset(priv);
		else
			break;
	}

	if (i >= AR40XX_PSGMII_CALB_NUM)
		dev_err(priv->qca8k.dev, "PSGMII cannot recover.\n");
	else
		dev_dbg(priv->qca8k.dev, "PSGMII recovered after %d times reset.\n", i);

	/* configuration recover */
	/* packet number */
	qca8k_mmio_phy_mmd_write(priv, 0x1f, 7, 0x8021, 0x0);
	/* disable check */
	qca8k_mmio_phy_mmd_write(priv, 0x1f, 7, 0x8029, 0x0);
	/* disable traffic */
	qca8k_mmio_phy_mmd_write(priv, 0x1f, 7, 0x8020, 0x0);
}

#define AR40XX_PHY_DEBUG_0   0
#define AR40XX_PHY_MANU_CTRL_EN  BIT(12)

#define AR40XX_PHY_DEBUG_2   2

void
psgmii_self_test_clean(struct qca8k_mmio_priv *priv)
{
	int phy;
	struct mii_bus *bus = priv->qca8k.bus;

	/* disable phy internal loopback */
	mdiobus_write(bus, 0x1f, 0x10, 0x6860);
	mdiobus_write(bus, 0x1f, 0x0, 0x9040);

	for (phy = 0; phy < AR40XX_NUM_PORTS - 1; phy++) {
		/* disable mac loop back */
		qca8k_mmio_rmw(&priv->qca8k, AR40XX_REG_PORT_LOOKUP(phy + 1),
				AR40XX_PORT_LOOKUP_LOOPBACK, 0);
		/* disable phy mdio broadcast write */
		qca8k_mmio_phy_mmd_write(priv, phy, 7, 0x8028, 0x001f);
	}

	/* clear fdb entry */
	qca8k_atu_flush(priv);
	
	for (phy = 0; phy < AR40XX_NUM_PORTS - 1; phy++) {
		u16 val;
		qca8k_mmio_phy_dbg_read(priv, phy, AR40XX_PHY_DEBUG_0, &val);
		val &= ~AR40XX_PHY_MANU_CTRL_EN;
		qca8k_mmio_phy_dbg_write(priv, phy, AR40XX_PHY_DEBUG_0, val);

		mdiobus_write(bus, phy,
			      MII_ADVERTISE, ADVERTISE_ALL |
			      ADVERTISE_PAUSE_CAP |
			      ADVERTISE_PAUSE_ASYM);
		mdiobus_write(bus, phy, MII_CTRL1000, ADVERTISE_1000FULL);
		mdiobus_write(bus, phy, MII_BMCR, BMCR_RESET | BMCR_ANENABLE);
	}
}

/* End of psgmii self test */

#define AR40XX_PSGMIIPHY_TX_CONTROL	 0x288
#define AR40XX_PSGMII_MODE_CONTROL	0x1b4

#define AR40XX_RGMII_CTRL         0x0004

enum ar40xx_port_wrapper_cfg {
	PORT_WRAPPER_PSGMII = 0,
	PORT_WRAPPER_RGMII = 3,
};

#define AR40XX_PSGMII_ID		5
#define AR40XX_PSGMII_CALB_NUM		100
#define AR40XX_MALIBU_PSGMII_MODE_CTRL	0x6d
#define AR40XX_MALIBU_PHY_PSGMII_MODE_CTRL_ADJUST_VAL	0x220c
#define AR40XX_MALIBU_PHY_MMD7_DAC_CTRL	0x801a
#define AR40XX_MALIBU_DAC_CTRL_MASK	0x380
#define AR40XX_MALIBU_DAC_CTRL_VALUE	0x280
#define AR40XX_MALIBU_PHY_RLP_CTRL	0x805a
#define AR40XX_PSGMII_TX_DRIVER_1_CTRL	0xb
#define AR40XX_MALIBU_PHY_PSGMII_REDUCE_SERDES_TX_AMP	0x8a
#define AR40XX_MALIBU_PHY_LAST_ADDR	4

static void
malibu_init(struct qca8k_mmio_priv *priv)
{
	int i;
	u16 val;

	/* war to enable AZ transmitting ability */
	qca8k_mmio_phy_mmd_write(priv, AR40XX_PSGMII_ID, 1,
			     AR40XX_MALIBU_PSGMII_MODE_CTRL,
			     AR40XX_MALIBU_PHY_PSGMII_MODE_CTRL_ADJUST_VAL);
	for (i = 0; i < AR40XX_NUM_PORTS - 1; i++) {
		/* change malibu control_dac */
		val = qca8k_mmio_phy_mmd_read(priv, i, 7,
					  AR40XX_MALIBU_PHY_MMD7_DAC_CTRL);
		val &= ~AR40XX_MALIBU_DAC_CTRL_MASK;
		val |= AR40XX_MALIBU_DAC_CTRL_VALUE;
		qca8k_mmio_phy_mmd_write(priv, i, 7,
				     AR40XX_MALIBU_PHY_MMD7_DAC_CTRL, val);
		if (i == AR40XX_MALIBU_PHY_LAST_ADDR) {
			/* to avoid goes into hibernation */
			val = qca8k_mmio_phy_mmd_read(priv, i, 3,
						  AR40XX_MALIBU_PHY_RLP_CTRL);
			val &= (~(1<<1));
			qca8k_mmio_phy_mmd_write(priv, i, 3,
					     AR40XX_MALIBU_PHY_RLP_CTRL, val);
		}
	}

	/* adjust psgmii serdes tx amp */
	mdiobus_write(priv->qca8k.bus, AR40XX_PSGMII_ID,
		      AR40XX_PSGMII_TX_DRIVER_1_CTRL,
		      AR40XX_MALIBU_PHY_PSGMII_REDUCE_SERDES_TX_AMP);
}

static void
psgmii_hw_init(struct qca8k_mmio_priv *priv)
{
	qca8k_mmio_ess_reset(priv);

	switch (priv->mac_mode) {
	case PORT_WRAPPER_PSGMII:
		dev_info(priv->qca8k.dev, "Set to PSGMII\n");
		malibu_init(priv);
		psgmii_self_test(priv);
		psgmii_self_test_clean(priv);
		qca8k_mmio_psgmii_write(priv, AR40XX_PSGMII_MODE_CONTROL, 0x2200);
		qca8k_mmio_psgmii_write(priv, AR40XX_PSGMIIPHY_TX_CONTROL, 0x8380);
		break;
	case PORT_WRAPPER_RGMII:
		dev_info(priv->qca8k.dev, "Set to RGMII\n");
		qca8k_mmio_write(&priv->qca8k, AR40XX_RGMII_CTRL, 0x400);
		break;
	default:
		dev_err(priv->qca8k.dev, "inknown mac-mode\n");
	}
}

static
int qca8k_force_1g_full(struct qca8k_mmio_priv *priv, u32 port_id)
{
	u32 reg;

	if (port_id < 0 || port_id > AR40XX_NUM_PORTS)
		return -1;

	reg = QCA8K_REG_PORT_STATUS(port_id);
	return qca8k_mmio_rmw(&priv->qca8k, reg, QCA8K_PORT_STATUS_SPEED,
			(QCA8K_PORT_STATUS_SPEED_1000 | QCA8K_PORT_STATUS_DUPLEX));
}

#define AR40XX_REG_QM_DEBUG_ADDR		0x820
#define AR40XX_REG_QM_DEBUG_VALUE		0x824
#define   AR40XX_REG_QM_PORT0_3_QNUM		0x1d
#define   AR40XX_REG_QM_PORT4_6_QNUM		0x1e

static
int qca8k_get_qm_status(struct qca8k_mmio_priv *priv,
			 u32 port_id, u32 *qm_buffer_err)
{
	u32 reg;
	u32 qm_val;

	if (port_id < 1 || port_id >= AR40XX_NUM_PORTS) {
		*qm_buffer_err = 0;
		return -1;
	}

	if (port_id < 4) {
		reg = AR40XX_REG_QM_PORT0_3_QNUM;
		qca8k_mmio_write(&priv->qca8k, AR40XX_REG_QM_DEBUG_ADDR, reg);
		qm_val = qca8k_mmio_read(&priv->qca8k, AR40XX_REG_QM_DEBUG_VALUE);
		/* every 8 bits for each port */
		*qm_buffer_err = (qm_val >> (port_id * 8)) & 0xFF;
	} else {
		reg = AR40XX_REG_QM_PORT4_6_QNUM;
		qca8k_mmio_write(&priv->qca8k, AR40XX_REG_QM_DEBUG_ADDR, reg);
		qm_val = qca8k_mmio_read(&priv->qca8k, AR40XX_REG_QM_DEBUG_VALUE);
		/* every 8 bits for each port */
		*qm_buffer_err = (qm_val >> ((port_id-4) * 8)) & 0xFF;
	}

	return 0;
}

#define AR40XX_PORT_LINK_UP 1
#define AR40XX_PORT_LINK_DOWN 0
#define AR40XX_QM_NOT_EMPTY  1
#define AR40XX_QM_EMPTY  0

static void
qca8k_sw_mac_polling_task(struct qca8k_mmio_priv *priv)
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

	bus = priv->qca8k.bus;

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
				qca8k_mmio_rmw(&priv->qca8k, reg,
						QCA8K_PORT_STATUS_LINK_AUTO, 0);

				/* Check queue buffer */
				qm_err_cnt[i] = 0;
				qca8k_get_qm_status(priv, i, &qm_buffer_err);
				if (qm_buffer_err) {
					priv->ar40xx_port_qm_buf[i] =
						AR40XX_QM_NOT_EMPTY;
				} else {
					u16 phy_val = 0;

					priv->ar40xx_port_qm_buf[i] =
						AR40XX_QM_EMPTY;
					qca8k_force_1g_full(priv, i);
					/* Ref:QCA8337 Datasheet,Clearing
					 * MENU_CTRL_EN prevents phy to
					 * stuck in 100BT mode when
					 * bringing up the link
					 */
					qca8k_mmio_phy_dbg_read(priv, i-1,
						AR40XX_PHY_DEBUG_0, &phy_val);
					phy_val &= (~AR40XX_PHY_MANU_CTRL_EN);
					qca8k_mmio_phy_dbg_write(priv, i-1,
						AR40XX_PHY_DEBUG_0, phy_val);
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
					value = qca8k_mmio_read(&priv->qca8k,
								reg);
					priv->port_link_up[i] = 0;

					value &= ~(QCA8K_PORT_STATUS_DUPLEX |
						   QCA8K_PORT_STATUS_SPEED);
					value |= speed | (duplex ?
						QCA8K_PORT_STATUS_DUPLEX : 0);
					qca8k_mmio_write(&priv->qca8k, reg,
							 value);
					/* clock switch need such time
					 * to avoid glitch
					 */
					usleep_range(100, 200);

					value |= QCA8K_PORT_STATUS_LINK_AUTO;
					qca8k_mmio_write(&priv->qca8k, reg,
							 value);
					/* HW need such time to make sure link
					 * stable before enable MAC
					 */
					usleep_range(100, 200);

					if (speed == QCA8K_PORT_SPEED_100M) {
						u16 phy_val = 0;
						/* Enable @100M, if down to 10M
						 * clock will change smoothly
						 */
						qca8k_mmio_phy_dbg_read(priv,
							i-1, 0, &phy_val);
						phy_val |=
							AR40XX_PHY_MANU_CTRL_EN;
						qca8k_mmio_phy_dbg_write(priv,
							i-1, 0, phy_val);
					}
					priv->ar40xx_port_old_link[i] = link;
				}
			}
		}

		if (priv->ar40xx_port_qm_buf[i] == AR40XX_QM_NOT_EMPTY) {
			/* Check QM */
			qca8k_get_qm_status(priv, i, &qm_buffer_err);
			if (qm_buffer_err) {
				++qm_err_cnt[i];
			} else {
				priv->ar40xx_port_qm_buf[i] =
						AR40XX_QM_EMPTY;
				qm_err_cnt[i] = 0;
				qca8k_force_1g_full(priv, i);
			}
		}
	}
}

#define AR40XX_QM_WORK_DELAY    100

static void
qca8k_qm_err_check_work_task(struct work_struct *work)
{
	struct qca8k_mmio_priv *priv = container_of(work, typeof(*priv),
					qm_dwork.work);

	if (!work || !priv) {
		pr_err("qm_err_worker without proper data\n");
		return;
	}

	mutex_lock(&priv->qm_lock);

	qca8k_sw_mac_polling_task(priv);

	mutex_unlock(&priv->qm_lock);

	schedule_delayed_work(&priv->qm_dwork,
			      msecs_to_jiffies(AR40XX_QM_WORK_DELAY));
}

static void
qca8k_qm_err_check_work_start(struct qca8k_mmio_priv *priv)
{
	mutex_init(&priv->qm_lock);

	INIT_DELAYED_WORK(&priv->qm_dwork, qca8k_qm_err_check_work_task);

	schedule_delayed_work(&priv->qm_dwork,
			      msecs_to_jiffies(AR40XX_QM_WORK_DELAY));

	dev_info(priv->qca8k.dev, "started QM WorkAround daemon.\n");
	return 0;
}

static int
qca8k_sw_probe_mmio(struct platform_device *pdev)
{
	struct qca8k_mmio_priv *priv;
	struct device_node *mdio;
	int err;

	/* allocate the private data struct so that we can probe the switches
	 * ID register
	 */
	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	
	mdio = of_parse_phandle(pdev->dev.of_node, "dsa,mii-bus", 0);
	if (!mdio)
		return -EINVAL;

	priv->qca8k.bus = of_mdio_find_bus(mdio);
	if (!priv->qca8k.bus)
		return -EPROBE_DEFER;

	priv->qca8k.base = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "switch");
	if (IS_ERR(priv->qca8k.base)) {
		err = PTR_ERR(priv->qca8k.base);
		dev_err(&pdev->dev, "failed to request switch regmap: %d.\n", err);
		return err;
	}
	
	priv->psgmii = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "qca,psgmii");
	if (IS_ERR(priv->psgmii)) {
		err = PTR_ERR(priv->psgmii);
		dev_err(&pdev->dev, "failed to request qca,psgmii regmap: %d.\n", err);
		return err;
	}
	
	priv->ess_rst = devm_reset_control_get(&pdev->dev, "ess_rst");
	if (IS_ERR(priv->ess_rst)) {
		err = PTR_ERR(priv->ess_rst);
		dev_err(&pdev->dev, "failed to get ess_rst control: %d.\n", err);
		return err;
	}
	
	of_property_read_u32(pdev->dev.of_node, "qca,mac-mode", &priv->mac_mode);
	if (priv->mac_mode < 0) {
		err = priv->mac_mode;
		dev_err(&pdev->dev, "failed to read 'qca,mac-mode' property: %d.\n", err);
		return err;
	}

	priv->ess_clk = of_clk_get_by_name(pdev->dev.of_node, "ess_clk");
	if (IS_ERR(priv->ess_clk)) {
		err = PTR_ERR(priv->ess_clk);
		dev_err(&pdev->dev, "failed to get ess_clk clock: %d.\n", err);
		return err;
	}

	err = clk_prepare_enable(priv->ess_clk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable ess_clk: %d.\n", err);
		goto out_clk;
	}

	spin_lock_init(&priv->qca8k.reg_lock);
	priv->qca8k.dev = &pdev->dev;
	priv->qca8k.reg = &qca8k_mmio_reg_ops;
	priv->qca8k.tag_protocol = DSA_TAG_PROTO_QCA_MMIO;

	psgmii_hw_init(priv);

	err = qca8k_sw_probe(&priv->qca8k);

out_clk:
	if (err) {
		clk_disable_unprepare(priv->ess_clk);
		clk_put(priv->ess_clk);
		priv->ess_clk = NULL;
	} else {
		qca8k_qm_err_check_work_start(priv);	
	}
	
	return err;
}

static int
qca8k_sw_remove_mmio(struct platform_device *pdev)
{
	struct qca8k_priv *qca8k = dev_get_drvdata(&pdev->dev);
	struct qca8k_mmio_priv *priv = container_of(qca8k, typeof(*priv), qca8k);
	int err;
	
	cancel_delayed_work_sync(&priv->qm_dwork);
	
	err = qca8k_sw_remove(qca8k);
	clk_disable_unprepare(priv->ess_clk);
	clk_put(priv->ess_clk);
	reset_control_put(priv->ess_rst);
	return err;
}

static const struct of_device_id qca8k_of_match[] = {
	{ .compatible = "qca,qca8337-mmio" },
	{ /* sentinel */ },
};

static struct platform_driver qca8k_mmio_driver = {
	.probe  = qca8k_sw_probe_mmio,
	.remove = qca8k_sw_remove_mmio,
	.driver = {
		.name = "qca8k-mmio",
		.of_match_table = qca8k_of_match,
		.pm = &qca8k_pm_ops,
	},
};

module_platform_driver(qca8k_mmio_driver);

MODULE_AUTHOR("John Crispin <john@phrozen.org>");
MODULE_DESCRIPTION("Driver for QCA8K ethernet switch family");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:qca8k-mmio");
