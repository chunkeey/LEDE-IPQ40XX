/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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

#include <linux/etherdevice.h>

#include "dsa_priv.h"

/* Both the IPQESS (essedma) + ESS-Switch Cores are part of the
 * IPQ40XX SoC. Because of their "proximity" the ethernet rx and
 * tx descriptor have dedicated fields set aside for the port id.
 */
static struct sk_buff *qca_tag_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct dsa_port *dp = dsa_slave_to_port(dev);

	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;

	skb->dev_scratch = BIT(dp->index);

	return skb;
}

static struct sk_buff *qca_tag_rcv(struct sk_buff *skb, struct net_device *dev,
				   struct packet_type *pt)
{
	int port;
	__le16 *rrd1;

#define EDMA_PORT_ID_SHIFT	12
#define EDMA_PORT_ID_MASK	0x7

	/* port_id is part of the hardware's rx descriptor (2nd word)
	 * to access it, we have to be a little naughty and access the
	 * data that comes "in front of the start of the frame".
	 */
	rrd1 = (__le16 *)(skb->data - 2 - 6 - 6 - 14);

	port = (le16_to_cpu(*rrd1) >> EDMA_PORT_ID_SHIFT) & EDMA_PORT_ID_MASK;

	skb->dev = dsa_master_find_slave(dev, 0, port);
	if (!skb->dev)
		return NULL;

	return skb;
}

const struct dsa_device_ops qca_netdev_ops = {
	.xmit	= qca_tag_xmit,
	.rcv	= qca_tag_rcv,
};
