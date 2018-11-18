/*
 * Copyright (c) 2014 - 2017, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all copies.
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/skbuff.h>
#include <linux/vmalloc.h>

#include <net/checksum.h>
#include <net/ip6_checksum.h>

#include "ipqess.h"

#define IPQESS_RRD_SIZE		16
#define IPQESS_NEXT_IDX(X, Y)  (((X) + 1) & ((Y) - 1))
#define IPQESS_TX_DMA_BUF_LEN	0x3fff

static void ipqess_w32(struct ipqess *ess, u32 reg, u32 val)
{
	__raw_writel(val, ess->hw_addr + reg);
}

static u32 ipqess_r32(struct ipqess *ess, u16 reg)
{
	return __raw_readl(ess->hw_addr + reg);
}

static void ipqess_m32(struct ipqess *ess, u32 mask, u32 val, u16 reg)
{
	u32 _val = ipqess_r32(ess, reg);
	_val &= ~mask;
	_val |= val;
	ipqess_w32(ess, reg, _val);
}

static int ipqess_tx_ring_alloc(struct ipqess *ess)
{
	int i;

	for (i = 0; i < IPQESS_NETDEV_QUEUES; i++) {
		u32 idx;

		ess->tx_ring[i].ess = ess;
		ess->tx_ring[i].idx = i * 4;
		ess->tx_ring[i].count = IPQESS_TX_RING_SIZE;
		ess->tx_ring[i].nq = netdev_get_tx_queue(ess->netdev, i);

		ess->tx_ring[i].buf = devm_kzalloc(&ess->pdev->dev,
			sizeof(struct ipqess_buf) * IPQESS_TX_RING_SIZE,
			GFP_KERNEL);
		if (!ess->tx_ring[i].buf) {
			netdev_err(ess->netdev, "buffer alloc of tx ring failed");
			return -ENOMEM;
		}

		ess->tx_ring[i].hw_desc = dmam_alloc_coherent(&ess->pdev->dev,
			sizeof(struct ipqess_tx_desc) * IPQESS_TX_RING_SIZE,
			&ess->tx_ring[i].dma, GFP_KERNEL | __GFP_ZERO);
		if (!ess->tx_ring[i].hw_desc) {
			netdev_err(ess->netdev, "descriptor allocation for tx ring failed");
			return -ENOMEM;
		}

		ipqess_w32(ess, IPQESS_REG_TPD_BASE_ADDR_Q(ess->tx_ring[i].idx),
			 (u32)ess->tx_ring[i].dma);

		idx = ipqess_r32(ess, IPQESS_REG_TPD_IDX_Q(ess->tx_ring[i].idx));
		idx >>= IPQESS_TPD_CONS_IDX_SHIFT; /* need u32 here */
		idx &= 0xffff;
		ess->tx_ring[i].head = ess->tx_ring[i].tail = idx;

		ipqess_m32(ess, IPQESS_TPD_PROD_IDX_MASK << IPQESS_TPD_PROD_IDX_SHIFT,
			 idx, IPQESS_REG_TPD_IDX_Q(ess->tx_ring[i].idx));
		ipqess_w32(ess, IPQESS_REG_TX_SW_CONS_IDX_Q(ess->tx_ring[i].idx), idx);
		ipqess_w32(ess, IPQESS_REG_TPD_RING_SIZE, IPQESS_TX_RING_SIZE);
	}

	return 0;
}

static int ipqess_tx_unmap_and_free(struct device *dev, struct ipqess_buf *buf)
{
	int len = 0;

	if (buf->flags & IPQESS_DESC_SINGLE)
		dma_unmap_single(dev, buf->dma,	buf->length, DMA_TO_DEVICE);
	else if (buf->flags & IPQESS_DESC_PAGE)
		dma_unmap_page(dev, buf->dma, buf->length, DMA_TO_DEVICE);

	if (buf->flags & IPQESS_DESC_LAST) {
		len = buf->skb->len;
		dev_kfree_skb_any(buf->skb);
	}

	buf->flags = 0;

	return len;
}

static void ipqess_tx_ring_free(struct ipqess *ess)
{
	int i;

	for (i = 0; i < IPQESS_NETDEV_QUEUES; i++) {
		int j;

		if (ess->tx_ring[i].hw_desc)
			continue;

		for (j = 0; j < IPQESS_TX_RING_SIZE; j++) {
			struct ipqess_buf *buf = &ess->tx_ring[i].buf[j];

			ipqess_tx_unmap_and_free(&ess->pdev->dev, buf);
		}

		ess->tx_ring[i].buf = NULL;
	}
}

static int ipqess_rx_buf_prepare(struct ipqess_buf *buf,
	struct ipqess_rx_ring *rx_ring)
{
	/* Clean the HW DESC header, otherwise we might end up
	 * with a spurious desc because of random garbage */
	memset(buf->skb->data, 0, sizeof(struct ipqess_rx_desc));

	buf->dma = dma_map_single(&rx_ring->ess->pdev->dev, buf->skb->data,
				  IPQESS_RX_HEAD_BUFF_SIZE, DMA_FROM_DEVICE);
	if (dma_mapping_error(&rx_ring->ess->pdev->dev, buf->dma)) {
		dev_err_once(&rx_ring->ess->pdev->dev,
			"IPQESS DMA mapping failed for linear address %x",
			buf->dma);
		dev_kfree_skb_any(buf->skb);
		buf->skb = NULL;
		return -EFAULT;
	}

	buf->length = IPQESS_RX_HEAD_BUFF_SIZE;
	rx_ring->hw_desc[rx_ring->head] = (void *)buf->dma;
	rx_ring->head = (rx_ring->head + 1) % IPQESS_RX_RING_SIZE;

	ipqess_m32(rx_ring->ess, IPQESS_RFD_PROD_IDX_BITS,
		 (rx_ring->head + IPQESS_RX_RING_SIZE - 1) % IPQESS_RX_RING_SIZE,
		 IPQESS_REG_RFD_IDX_Q(rx_ring->idx));

	return 0;
}

static int ipqess_rx_buf_alloc_napi(struct ipqess_rx_ring *rx_ring)
{
	struct ipqess_buf *buf = &rx_ring->buf[rx_ring->head];

	buf->skb = napi_alloc_skb(&rx_ring->napi_rx,
		IPQESS_RX_HEAD_BUFF_SIZE);
	if (!buf->skb)
		return -ENOMEM;
	return ipqess_rx_buf_prepare(buf, rx_ring);
}

static int ipqess_rx_buf_alloc(struct ipqess_rx_ring *rx_ring)
{
	struct ipqess_buf *buf = &rx_ring->buf[rx_ring->head];

	buf->skb = netdev_alloc_skb_ip_align(rx_ring->ess->netdev,
		IPQESS_RX_HEAD_BUFF_SIZE);
	if (!buf->skb)
		return -ENOMEM;
	return ipqess_rx_buf_prepare(buf, rx_ring);
}

static void ipqess_refill_work(struct work_struct *work)
{
	struct ipqess_rx_ring *rx_ring = container_of(work,
		struct ipqess_rx_ring, refill_work);
	int refill = 0;

	/* don't let this loop by accident. */
	while (atomic_dec_and_test(&rx_ring->refill_count)) {
		if (ipqess_rx_buf_alloc(rx_ring)) {
			refill++;
			dev_dbg(&rx_ring->ess->pdev->dev,
				"Not all buffers were reallocated");
		}
	}

	if (refill || atomic_read(&rx_ring->refill_count)) {
		atomic_add(refill, &rx_ring->refill_count);
		schedule_work(&rx_ring->refill_work);
	}
}

static int ipqess_rx_ring_alloc(struct ipqess *ess)
{
	int i;

	for (i = 0; i < IPQESS_NETDEV_QUEUES; i++) {
		int j;

		ess->rx_ring[i].ess = ess;
		ess->rx_ring[i].idx = i;

		ess->rx_ring[i].buf = devm_kzalloc(&ess->pdev->dev,
			sizeof(struct ipqess_buf) * IPQESS_RX_RING_SIZE,
			GFP_KERNEL);
		if (!ess->rx_ring[i].buf)
			return -ENOMEM;

		ess->rx_ring[i].hw_desc = dmam_alloc_coherent(&ess->pdev->dev,
			sizeof(struct ipqess_rx_desc) * IPQESS_RX_RING_SIZE,
			&ess->rx_ring[i].dma, GFP_KERNEL);
		if (!ess->rx_ring[i].hw_desc)
			return -ENOMEM;

		for (j = 0; j < IPQESS_RX_RING_SIZE; j++)
			if (ipqess_rx_buf_alloc(&ess->rx_ring[i]) < 0)
				return -ENOMEM;

		INIT_WORK(&ess->rx_ring[i].refill_work, ipqess_refill_work);

		ipqess_w32(ess, IPQESS_REG_RFD_BASE_ADDR_Q(i),
			 (u32)(ess->rx_ring[i].dma));
	}

	ipqess_w32(ess, IPQESS_REG_RX_DESC0,
		 (IPQESS_RX_HEAD_BUFF_SIZE << IPQESS_RX_BUF_SIZE_SHIFT) |
		 (IPQESS_RX_RING_SIZE << IPQESS_RFD_RING_SIZE_SHIFT));

	return 0;
}

static void ipqess_rx_ring_free(struct ipqess *ess)
{
	int i;

	for (i = 0, i = 0; i < IPQESS_NETDEV_QUEUES; i++) {
		int j;

		for (j = 0; j < IPQESS_RX_RING_SIZE; j++) {
			dma_unmap_single(&ess->pdev->dev,
					 ess->rx_ring[i].buf[j].dma,
					 ess->rx_ring[i].buf[j].length,
					 DMA_FROM_DEVICE);
			dev_kfree_skb_any(ess->rx_ring[i].buf[j].skb);
		}

		atomic_set(&ess->rx_ring[i].refill_count, 0);
		cancel_work_sync(&ess->rx_ring[i].refill_work);
	}
}

static struct net_device_stats *ipqess_get_stats(struct net_device *netdev)
{
	struct ipqess *ess = netdev_priv(netdev);
	uint32_t *p;
	int i;
	u32 stat;

	spin_lock(&ess->stats_lock);
	p = (uint32_t *)&(ess->ipqessstats);

	for (i = 0; i < IPQESS_MAX_TX_QUEUE; i++) {
		stat = ipqess_r32(ess, IPQESS_REG_TX_STAT_PKT_Q(i));
		*p += stat;
		p++;
	}

	for (i = 0; i < IPQESS_MAX_TX_QUEUE; i++) {
		stat = ipqess_r32(ess, IPQESS_REG_TX_STAT_BYTE_Q(i));
		*p += stat;
		p++;
	}

	for (i = 0; i < IPQESS_MAX_RX_QUEUE; i++) {
		stat = ipqess_r32(ess, IPQESS_REG_RX_STAT_PKT_Q(i));
		*p += stat;
		p++;
	}

	for (i = 0; i < IPQESS_MAX_RX_QUEUE; i++) {
		stat = ipqess_r32(ess, IPQESS_REG_RX_STAT_BYTE_Q(i));
		*p += stat;
		p++;
	}

	spin_unlock(&ess->stats_lock);
	return &ess->stats;
}

static int ipqess_phy_connect(struct net_device *netdev)
{
	struct ipqess *ess = netdev_priv(netdev);
	struct device_node *np = NULL;

	if (!of_phy_register_fixed_link(ess->of_node))
		np = of_node_get(ess->of_node);
	if (!np)
		return -ENODEV;

	netdev->phydev = of_phy_find_device(np);
	of_node_put(np);
	if (!netdev->phydev) {
		of_phy_deregister_fixed_link(ess->of_node);
		return -EINVAL;
	}
	return 0;
}

static int ipqess_rx_poll(struct ipqess_rx_ring *rx_ring, int budget)
{
	u32 length = 0, num_desc, tail, rx_ring_tail;
	int done = 0;

	rx_ring_tail = rx_ring->tail;

	tail = ipqess_r32(rx_ring->ess, IPQESS_REG_RFD_IDX_Q(rx_ring->idx));
	tail >>= IPQESS_RFD_CONS_IDX_SHIFT;
	tail &= IPQESS_RFD_CONS_IDX_MASK;

	while (done < budget) {
		struct sk_buff *skb;
		struct ipqess_rx_desc *rd;

		if (rx_ring_tail == tail)
			break;

		dma_unmap_single(&rx_ring->ess->pdev->dev,
				 rx_ring->buf[rx_ring_tail].dma,
				 rx_ring->buf[rx_ring_tail].length,
				 DMA_FROM_DEVICE);

		skb = xchg(&rx_ring->buf[rx_ring_tail].skb, NULL);
		rd = (struct ipqess_rx_desc *)skb->data;
		rx_ring_tail = IPQESS_NEXT_IDX(rx_ring_tail, IPQESS_RX_RING_SIZE);

		/* Check if RRD is valid */
		if (!(rd->rrd7 & IPQESS_RRD_DESC_VALID)) {
			num_desc = 1;
			dev_kfree_skb_any(skb);
			goto skip;
		}

		num_desc = rd->rrd1 & IPQESS_RRD_NUM_RFD_MASK;
		length = rd->rrd6 & IPQESS_RRD_PKT_SIZE_MASK;

		skb_reserve(skb, IPQESS_RRD_SIZE);
		if (num_desc > 1) {
			/* can we use build_skb here ? */
			struct sk_buff *skb_prev = NULL;
			int size_remaining;
			int i;

			skb->data_len = 0;
			skb->tail += (IPQESS_RX_HEAD_BUFF_SIZE - IPQESS_RRD_SIZE);
			skb->len = skb->truesize = length;
			size_remaining = length - (IPQESS_RX_HEAD_BUFF_SIZE - IPQESS_RRD_SIZE);

			for (i = 1; i < num_desc; i++) {
				/* TODO: use build_skb ? */
				struct sk_buff *skb_temp = rx_ring->buf[rx_ring_tail].skb;

				dma_unmap_single(&rx_ring->ess->pdev->dev,
						 rx_ring->buf[rx_ring_tail].dma,
						 rx_ring->buf[rx_ring_tail].length,
						 DMA_FROM_DEVICE);

				skb_put(skb_temp, min(size_remaining, IPQESS_RX_HEAD_BUFF_SIZE));
				if (skb_prev)
					skb_prev->next = rx_ring->buf[rx_ring_tail].skb;
				else
					skb_shinfo(skb)->frag_list = rx_ring->buf[rx_ring_tail].skb;
				skb_prev = rx_ring->buf[rx_ring_tail].skb;
				rx_ring->buf[rx_ring_tail].skb->next = NULL;

				skb->data_len += rx_ring->buf[rx_ring_tail].skb->len;
				size_remaining -= rx_ring->buf[rx_ring_tail].skb->len;

				rx_ring_tail = IPQESS_NEXT_IDX(rx_ring_tail, IPQESS_RX_RING_SIZE);
			}

		} else {
			skb_put(skb, length);
		}

		skb->dev = rx_ring->ess->netdev;
		skb->protocol = eth_type_trans(skb, rx_ring->ess->netdev);
		skb_record_rx_queue(skb, rx_ring->idx);

		if (rd->rrd6 & IPQESS_RRD_CSUM_FAIL_MASK)
			skb_checksum_none_assert(skb);
		else
			skb->ip_summed = CHECKSUM_UNNECESSARY;

		if (rd->rrd7 & IPQESS_RRD_CVLAN) {
			__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), rd->rrd4);
		} else if (rd->rrd1 & IPQESS_RRD_SVLAN) {
			__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021AD), rd->rrd4);
		}
		napi_gro_receive(&rx_ring->napi_rx, skb);

		/* TODO: do we need to have these here ? */
		rx_ring->ess->stats.rx_packets++;
		rx_ring->ess->stats.rx_bytes += length;

		done++;
skip:

		while (num_desc) {
			if (ipqess_rx_buf_alloc_napi(rx_ring)) {
				schedule_work(&rx_ring->refill_work);
				atomic_add(num_desc, &rx_ring->refill_count);
				break;
			}
			num_desc--;
		}

	}

	ipqess_w32(rx_ring->ess, IPQESS_REG_RX_SW_CONS_IDX_Q(rx_ring->idx),
		   rx_ring_tail);
	rx_ring->tail = rx_ring_tail;


	return done;
}

static int ipqess_tx_complete(struct ipqess_tx_ring *tx_ring, int budget)
{
	u32 tail;
	int done = 0;
	int total = 0, ret;

	tail = ipqess_r32(tx_ring->ess, IPQESS_REG_TPD_IDX_Q(tx_ring->idx));
	tail >>= IPQESS_TPD_CONS_IDX_SHIFT;
	tail &= IPQESS_TPD_CONS_IDX_MASK;

	while ((tx_ring->tail != tail) && (done < budget)) {
		//pr_info("freeing txq:%d tail:%d tailbuf:%p\n", tx_ring->idx, tx_ring->tail, &tx_ring->buf[tx_ring->tail]);
		ret = ipqess_tx_unmap_and_free(&tx_ring->ess->pdev->dev,
				       &tx_ring->buf[tx_ring->tail]);
		tx_ring->tail = IPQESS_NEXT_IDX(tx_ring->tail, tx_ring->count);
		if (ret) {
			total += ret;
			done++;
		}
	}

	ipqess_w32(tx_ring->ess,
		 IPQESS_REG_TX_SW_CONS_IDX_Q(tx_ring->idx),
		 tx_ring->tail);

	if (netif_tx_queue_stopped(tx_ring->nq)) {
		printk("S %d\n", tx_ring->idx);
		netif_tx_wake_queue(tx_ring->nq);
	}

	netdev_tx_completed_queue(tx_ring->nq, done, total);

	return done;
}

static int ipqess_tx_napi(struct napi_struct *napi, int budget)
{
	struct ipqess_tx_ring *tx_ring = container_of(napi, struct ipqess_tx_ring,
						    napi_tx);
	u32 reg_data;
	u32 shadow_tx_status;
	int work_done = 0;
	struct queue *queue = &tx_ring->ess->queue[tx_ring->idx / 4];

	reg_data = ipqess_r32(tx_ring->ess, IPQESS_REG_TX_ISR);
	queue->tx_status |= reg_data & BIT(tx_ring->idx);
	shadow_tx_status = queue->tx_status;

	work_done = ipqess_tx_complete(tx_ring, budget);

	ipqess_w32(tx_ring->ess, IPQESS_REG_TX_ISR, shadow_tx_status);

	if (likely(work_done < budget)) {
		napi_complete(napi);
		ipqess_w32(tx_ring->ess, IPQESS_REG_TX_INT_MASK_Q(tx_ring->idx), 0x1);
	}

	return work_done;
}

static int ipqess_rx_napi(struct napi_struct *napi, int budget)
{
	struct ipqess_rx_ring *rx_ring = container_of(napi, struct ipqess_rx_ring,
						    napi_rx);
	struct ipqess *ess = rx_ring->ess;
	int remain_budget = budget;
	int rx_done;
	u32 rx_mask = BIT(rx_ring->idx << IPQESS_RX_PER_CPU_MASK_SHIFT);
	u32 status;

poll_again:
	ipqess_w32(ess, IPQESS_REG_RX_ISR, rx_mask);
	rx_done = ipqess_rx_poll(rx_ring, remain_budget);

	if (rx_done == remain_budget)
		return budget;

	status = ipqess_r32(ess, IPQESS_REG_RX_ISR);
	if (status & rx_mask) {
		remain_budget -= rx_done;
		goto poll_again;
	}

	napi_complete(napi);
	ipqess_w32(ess, IPQESS_REG_RX_INT_MASK_Q(rx_ring->idx), 0x1);

	return rx_done + budget - remain_budget;
}

static irqreturn_t ipqess_interrupt_tx(int irq, void *priv)
{
	struct ipqess_tx_ring *tx_ring = (struct ipqess_tx_ring *) priv;

	if (likely(napi_schedule_prep(&tx_ring->napi_tx))) {
		__napi_schedule(&tx_ring->napi_tx);
		ipqess_w32(tx_ring->ess,
			 IPQESS_REG_TX_INT_MASK_Q(tx_ring->idx),
			 0x0);
	}

	return IRQ_HANDLED;
}

static irqreturn_t ipqess_interrupt_rx(int irq, void *priv)
{
	struct ipqess_rx_ring *rx_ring = (struct ipqess_rx_ring *) priv;

	if (likely(napi_schedule_prep(&rx_ring->napi_rx))) {
		__napi_schedule(&rx_ring->napi_rx);
		ipqess_w32(rx_ring->ess,
			 IPQESS_REG_RX_INT_MASK_Q(rx_ring->idx),
			 0x0);
	}

	return IRQ_HANDLED;
}

static void ipqess_irq_enable(struct ipqess *ess)
{
	int i;

	ipqess_w32(ess, IPQESS_REG_RX_ISR, 0xff);
	ipqess_w32(ess, IPQESS_REG_TX_ISR, 0xffff);
	for (i = 0; i < IPQESS_NETDEV_QUEUES; i++) {
		ipqess_w32(ess, IPQESS_REG_RX_INT_MASK_Q(i), 1);
		ipqess_w32(ess, IPQESS_REG_TX_INT_MASK_Q(i), 1);
	}
}

static void ipqess_irq_disable(struct ipqess *ess)
{
	int i;

	for (i = 0; i < IPQESS_NETDEV_QUEUES; i++) {
		ipqess_w32(ess, IPQESS_REG_RX_INT_MASK_Q(i), 0);
		ipqess_w32(ess, IPQESS_REG_TX_INT_MASK_Q(i), 0);
	}
}

static int __init ipqess_init(struct net_device *netdev)
{
	struct ipqess *ess = netdev_priv(netdev);
	const char *mac_addr;

	mac_addr = of_get_mac_address(ess->of_node);
	if (mac_addr)
		ether_addr_copy(netdev->dev_addr, mac_addr);
	if (!is_valid_ether_addr(netdev->dev_addr)) {
		random_ether_addr(netdev->dev_addr);
		dev_info(&ess->pdev->dev, "generated random MAC address %pM\n",
			netdev->dev_addr);
		netdev->addr_assign_type = NET_ADDR_RANDOM;
	}

	return ipqess_phy_connect(netdev);
}

static void ipqess_uninit(struct net_device *netdev)
{
	struct ipqess *ess = netdev_priv(netdev);

	phy_disconnect(netdev->phydev);
	of_phy_deregister_fixed_link(ess->of_node);
}

static int ipqess_open(struct net_device *netdev)
{
	struct ipqess *ess = netdev_priv(netdev);
	int i;

	for (i = 0; i < IPQESS_NETDEV_QUEUES; i++) {
		napi_enable(&ess->tx_ring[i].napi_tx);
		napi_enable(&ess->rx_ring[i].napi_rx);
	}
	ipqess_irq_enable(ess);
	phy_start(ess->netdev->phydev);
	netif_tx_start_all_queues(netdev);
	netif_carrier_on(netdev);

	return 0;
}

static int ipqess_stop(struct net_device *netdev)
{
	struct ipqess *ess = netdev_priv(netdev);
	int i;

	netif_tx_stop_all_queues(netdev);
	phy_stop(netdev->phydev);
	netif_carrier_off(netdev);
	ipqess_irq_disable(ess);
	for (i = 0; i < IPQESS_NETDEV_QUEUES; i++) {
		napi_disable(&ess->tx_ring[i].napi_tx);
		napi_disable(&ess->rx_ring[i].napi_rx);
	}

	return 0;
}

static int ipqess_do_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	switch (cmd) {
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		return phy_mii_ioctl(netdev->phydev, ifr, cmd);
	default:
		break;
	}

	return -EOPNOTSUPP;
}


static inline u16 ipqess_tx_desc_available(struct ipqess_tx_ring *tx_ring)
{
	u16 count = 0;

	if (tx_ring->tail <= tx_ring->head)
		count = IPQESS_TX_RING_SIZE;

	count += tx_ring->tail - tx_ring->head - 1;

	return count;
}

static inline int ipqess_cal_txd_req(struct sk_buff *skb)
{
	int i, nfrags;
	struct skb_frag_struct *frag;

	nfrags = 1;
	if (skb_is_gso(skb)) {
		for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
			frag = &skb_shinfo(skb)->frags[i];
			nfrags += DIV_ROUND_UP(frag->size, IPQESS_TX_DMA_BUF_LEN);
		}
	} else {
		nfrags += skb_shinfo(skb)->nr_frags;
	}

	return nfrags; // DIV_ROUND_UP(nfrags, 2);
}

static struct ipqess_buf *ipqess_get_tx_buffer(struct ipqess_tx_ring *tx_ring,
					       struct ipqess_tx_desc *desc)
{
	return &tx_ring->buf[desc - (struct ipqess_tx_desc *)tx_ring->hw_desc];
}

static struct ipqess_tx_desc *ipqess_tx_desc_next(struct ipqess_tx_ring *tx_ring)
{
	struct ipqess_tx_desc *desc;

	desc = (&((struct ipqess_tx_desc *)(tx_ring->hw_desc))[tx_ring->head]);
	tx_ring->head = IPQESS_NEXT_IDX(tx_ring->head, tx_ring->count);

	return desc;
}

static void ipqess_rollback_tx(struct ipqess *eth,
			    struct ipqess_tx_desc *first_desc, int queue_id)
{
	struct ipqess_tx_ring *tx_ring = &eth->tx_ring[queue_id / 4];
	struct ipqess_buf *buf;
	struct ipqess_tx_desc *desc = NULL;
	u16 start_index, index;

	start_index = first_desc - (struct ipqess_tx_desc *)(tx_ring->hw_desc);

	index = start_index;
	while (index != tx_ring->head) {
		desc = (&((struct ipqess_tx_desc *)(tx_ring->hw_desc))[index]);
		buf = &tx_ring->buf[index];
		ipqess_tx_unmap_and_free(&eth->pdev->dev, buf);
		memset(desc, 0, sizeof(struct ipqess_tx_desc));
		if (++index == tx_ring->count)
			index = 0;
	}
	tx_ring->head = start_index;
}

static int ipqess_tx_map_and_fill(struct ipqess_tx_ring *tx_ring, struct sk_buff *skb)
{
	struct ipqess_buf *buf = NULL;
	struct platform_device *pdev = tx_ring->ess->pdev;
	struct ipqess_tx_desc *desc = NULL, *first_desc = NULL;
	u32 word1 = 0, word3 = 0, lso_word1 = 0, svlan_tag = 0;
	u16 len, lso_len = 0;
	int i = 0;

	if (skb_is_gso(skb)) {
		if (skb_shinfo(skb)->gso_type & SKB_GSO_TCPV4) {
			lso_word1 |= IPQESS_TPD_IPV4_EN;
			ip_hdr(skb)->check = 0;
			tcp_hdr(skb)->check = ~csum_tcpudp_magic(ip_hdr(skb)->saddr,
				ip_hdr(skb)->daddr, 0, IPPROTO_TCP, 0);
		} else if (skb_shinfo(skb)->gso_type & SKB_GSO_TCPV6) {
			lso_word1 |= IPQESS_TPD_LSO_V2_EN;
			ipv6_hdr(skb)->payload_len = 0;
			tcp_hdr(skb)->check = ~csum_ipv6_magic(&ipv6_hdr(skb)->saddr,
				&ipv6_hdr(skb)->daddr, 0, IPPROTO_TCP, 0);
		}

		lso_word1 |= IPQESS_TPD_LSO_EN |
			     ((skb_shinfo(skb)->gso_size & IPQESS_TPD_MSS_MASK) << IPQESS_TPD_MSS_SHIFT) |
			     (skb_transport_offset(skb) << IPQESS_TPD_HDR_SHIFT);
	} else if (likely(skb->ip_summed == CHECKSUM_PARTIAL)) {
			u8 css, cso;
			cso = skb_checksum_start_offset(skb);
			css = cso + skb->csum_offset;

			word1 |= (IPQESS_TPD_CUSTOM_CSUM_EN);
			word1 |= (cso >> 1) << IPQESS_TPD_HDR_SHIFT;
			word1 |= ((css >> 1) << IPQESS_TPD_CUSTOM_CSUM_SHIFT);
	}

	if (skb_vlan_tag_present(skb)) {
		switch (skb->vlan_proto) {
		case htons(ETH_P_8021Q):
			word3 |= BIT(IPQESS_TX_INS_CVLAN);
			word3 |= skb_vlan_tag_get(skb) << IPQESS_TX_CVLAN_TAG_SHIFT;
			break;
		case htons(ETH_P_8021AD):
			word1 |= BIT(IPQESS_TX_INS_SVLAN);
			svlan_tag = skb_vlan_tag_get(skb) << IPQESS_TX_SVLAN_TAG_SHIFT;
			break;
		default:
			dev_err(&pdev->dev, "no ctag or stag present\n");
			goto vlan_tag_error;
		}
	}

        if (skb->protocol == htons(ETH_P_PPP_SES))
                word1 |= IPQESS_TPD_PPPOE_EN;

	word3 |= 0x3e << IPQESS_TPD_PORT_BITMAP_SHIFT;
	len = skb_headlen(skb);

	first_desc = desc = ipqess_tx_desc_next(tx_ring);
	if (lso_word1 & IPQESS_TPD_LSO_V2_EN) {
		desc->addr = cpu_to_le16(skb->len);
		desc->word1 = word1 | lso_word1;
		desc->svlan_tag = svlan_tag;
		desc->word3 = word3;
		desc = ipqess_tx_desc_next(tx_ring);
	}

	buf = ipqess_get_tx_buffer(tx_ring, desc);
	if (lso_word1)
		buf->length = lso_len;
	else
		buf->length = len;
	buf->dma = dma_map_single(&pdev->dev,
				skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(&pdev->dev, buf->dma))
		goto dma_error;

	desc->addr = cpu_to_le32(buf->dma);
	desc->len  = cpu_to_le16(len);

	buf->flags |= IPQESS_DESC_SINGLE;
	desc->word1 = word1 | lso_word1;
	desc->svlan_tag = svlan_tag;
	desc->word3 = word3;

	while (i < skb_shinfo(skb)->nr_frags) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		len = skb_frag_size(frag);
		desc = ipqess_tx_desc_next(tx_ring);
		buf = ipqess_get_tx_buffer(tx_ring, desc);
		buf->length = len;
		buf->flags |= IPQESS_DESC_PAGE;
		buf->dma = skb_frag_dma_map(&pdev->dev, frag, 0, len, DMA_TO_DEVICE);
		if (dma_mapping_error(NULL, buf->dma))
			goto dma_error;

		desc->addr = cpu_to_le32(buf->dma);
		desc->len  = cpu_to_le16(len);
		desc->svlan_tag = svlan_tag;
		desc->word1 = word1 | lso_word1;
		desc->word3 = word3;
		i++;
	}
	desc->word1 |= 1 << IPQESS_TPD_EOP_SHIFT;
	buf->skb = skb;
	buf->flags |= IPQESS_DESC_LAST;

	return 0;

dma_error:
	ipqess_rollback_tx(tx_ring->ess, first_desc, tx_ring->idx);
	dev_err(&pdev->dev, "TX DMA map failed\n");
vlan_tag_error:
	return -ENOMEM;
}

static netdev_tx_t ipqess_xmit(struct sk_buff *skb,
			     struct net_device *netdev)
{
	struct ipqess *ess = netdev_priv(netdev);
	struct ipqess_tx_ring *tx_ring;
	int tx_num;
	int ret;

	tx_ring = &ess->tx_ring[skb_get_queue_mapping(skb)];
	tx_num = ipqess_cal_txd_req(skb);
	if (ipqess_tx_desc_available(tx_ring) <= tx_num) {
		printk("s %d %x\n", tx_ring->idx, ipqess_r32(tx_ring->ess, IPQESS_REG_TX_INT_MASK_Q(tx_ring->idx)));
		netif_tx_stop_queue(tx_ring->nq);
		ipqess_w32(tx_ring->ess, IPQESS_REG_TX_INT_MASK_Q(tx_ring->idx), 0x1);
		return NETDEV_TX_BUSY;
	}

	ret = ipqess_tx_map_and_fill(tx_ring, skb);
	if (ret) {
		dev_kfree_skb_any(skb);
		ess->stats.tx_errors++;
		goto err_out;
	}

	ess->stats.tx_packets++;
	ess->stats.tx_bytes += skb->len;
	netdev_tx_sent_queue(tx_ring->nq, skb->len);

	if (!skb->xmit_more || netif_xmit_stopped(tx_ring->nq))
		ipqess_m32(ess,
			 IPQESS_TPD_PROD_IDX_BITS,
			 tx_ring->head,
			 IPQESS_REG_TPD_IDX_Q(tx_ring->idx));

err_out:
	return NETDEV_TX_OK;
}

static int ipqess_set_mac_address(struct net_device *netdev, void *p)
{
	int ret = eth_mac_addr(netdev, p);
	struct ipqess *ess = netdev_priv(netdev);
	const char *macaddr = netdev->dev_addr;

	if (ret)
		return ret;

//	spin_lock_bh(&mac->hw->page_lock);
	ipqess_w32(ess, IPQESS_REG_MAC_CTRL1,
		 (macaddr[0] << 8) | macaddr[1]);
	ipqess_w32(ess, IPQESS_REG_MAC_CTRL0,
		 (macaddr[2] << 24) | (macaddr[3] << 16) |
		 (macaddr[4] << 8) | macaddr[5]);
//	spin_unlock_bh(&mac->hw->page_lock);

	return 0;
}

static const struct net_device_ops ipqess_axi_netdev_ops = {
	.ndo_init		= ipqess_init,
	.ndo_uninit		= ipqess_uninit,
	.ndo_open		= ipqess_open,
	.ndo_stop		= ipqess_stop,
	.ndo_do_ioctl		= ipqess_do_ioctl,
	.ndo_start_xmit		= ipqess_xmit,
	.ndo_get_stats		= ipqess_get_stats,
	.ndo_set_mac_address	= ipqess_set_mac_address,
};

static void ipqess_reset(struct ipqess *ess)
{
	int i;

	/* disable all IRQs */
	for (i = 0; i < IPQESS_NETDEV_QUEUES; i++) {
		ipqess_w32(ess, IPQESS_REG_RX_INT_MASK_Q(i), 0x0);
		ipqess_w32(ess, IPQESS_REG_TX_INT_MASK_Q(i), 0x0);
	}

	ipqess_w32(ess, IPQESS_REG_MISC_IMR, 0);
	ipqess_w32(ess, IPQESS_REG_WOL_IMR, 0);

	/* clear the IRQ status registers */
	ipqess_w32(ess, IPQESS_REG_RX_ISR, 0xff);
	ipqess_w32(ess, IPQESS_REG_TX_ISR, 0xffff);
	ipqess_w32(ess, IPQESS_REG_MISC_ISR, 0x1fff);
	ipqess_w32(ess, IPQESS_REG_WOL_ISR, 0x1);
	ipqess_w32(ess, IPQESS_REG_WOL_CTRL, 0);

	/* disable RX and TX queues */
	ipqess_m32(ess, IPQESS_RXQ_CTRL_EN, 0, IPQESS_REG_RXQ_CTRL);
	ipqess_m32(ess, IPQESS_TXQ_CTRL_TXQ_EN, 0, IPQESS_REG_TXQ_CTRL);
}

static int ipqess_hw_init(struct ipqess *ess)
{
	int i, err;

	ipqess_reset(ess);

	ipqess_m32(ess, BIT(IPQESS_INTR_SW_IDX_W_TYP_SHIFT),
		 IPQESS_INTR_SW_IDX_W_TYPE << IPQESS_INTR_SW_IDX_W_TYP_SHIFT,
		 IPQESS_REG_INTR_CTRL);

	ipqess_w32(ess, IPQESS_REG_RX_ISR, 0xff);
	ipqess_w32(ess, IPQESS_REG_TX_ISR, 0xffff);

	/* enable IRQ delay slot */
	ipqess_w32(ess, IPQESS_REG_IRQ_MODRT_TIMER_INIT,
		 (IPQESS_TX_IMT << IPQESS_IRQ_MODRT_TX_TIMER_SHIFT) |
		 (IPQESS_RX_IMT << IPQESS_IRQ_MODRT_RX_TIMER_SHIFT));

	/* Configure the TX Queue bursting */
	ipqess_w32(ess, IPQESS_REG_TXQ_CTRL,
		 (IPQESS_TPD_BURST << IPQESS_TXQ_NUM_TPD_BURST_SHIFT) |
		 (IPQESS_TXF_BURST << IPQESS_TXQ_TXF_BURST_NUM_SHIFT) |
		 IPQESS_TXQ_CTRL_TPD_BURST_EN);

	/* Set RSS type */
	ipqess_w32(ess, IPQESS_REG_RSS_TYPE,
		 IPQESS_RSS_TYPE_IPV4TCP | IPQESS_RSS_TYPE_IPV6_TCP |
		 IPQESS_RSS_TYPE_IPV4_UDP | IPQESS_RSS_TYPE_IPV6UDP |
		 IPQESS_RSS_TYPE_IPV4 | IPQESS_RSS_TYPE_IPV6);

	/* Set RFD ring burst and threshold */
	ipqess_w32(ess, IPQESS_REG_RX_DESC1,
		(IPQESS_RFD_BURST << IPQESS_RXQ_RFD_BURST_NUM_SHIFT) |
		(IPQESS_RFD_THR << IPQESS_RXQ_RFD_PF_THRESH_SHIFT) |
		(IPQESS_RFD_LTHR << IPQESS_RXQ_RFD_LOW_THRESH_SHIFT));

	/* Set Rx FIFO
	 * - threshold to start to DMA data to host
	 */
	ipqess_w32(ess, IPQESS_REG_RXQ_CTRL,
		 IPQESS_FIFO_THRESH_128_BYTE | IPQESS_RXQ_CTRL_RMV_VLAN);

	err = ipqess_rx_ring_alloc(ess);
	if (err)
		return err;

	err = ipqess_tx_ring_alloc(ess);
	if (err)
		return err;

	/* Load all of ring base addresses above into the dma engine */
	ipqess_m32(ess, 0, BIT(IPQESS_LOAD_PTR_SHIFT),
		 IPQESS_REG_TX_SRAM_PART);

	/* Disable TX FIFO low watermark and high watermark */
	ipqess_w32(ess, IPQESS_REG_TXF_WATER_MARK, 0);

	/* Configure RSS indirection table.
	 * 128 hash will be configured in the following
	 * pattern: hash{0,1,2,3} = {Q0,Q2,Q4,Q6} respectively
	 * and so on
	 */
	for (i = 0; i < IPQESS_NUM_IDT; i++)
		ipqess_w32(ess, IPQESS_REG_RSS_IDT(i), IPQESS_RSS_IDT_VALUE);

	/* Configure load balance mapping table.
	 * 4 table entry will be configured according to the
	 * following pattern: load_balance{0,1,2,3} = {Q0,Q1,Q3,Q4}
	 * respectively.
	 */
	ipqess_w32(ess, IPQESS_REG_LB_RING, IPQESS_LB_REG_VALUE);

	/* Configure Virtual queue for Tx rings */
	ipqess_w32(ess, IPQESS_REG_VQ_CTRL0, IPQESS_VQ_REG_VALUE);
	ipqess_w32(ess, IPQESS_REG_VQ_CTRL1, IPQESS_VQ_REG_VALUE);

	/* Configure Max AXI Burst write size to 128 bytes*/
	ipqess_w32(ess, IPQESS_REG_AXIW_CTRL_MAXWRSIZE,
		 IPQESS_AXIW_MAXWRSIZE_VALUE);

	/* Enable All 16 tx and 8 rx irq mask */
	ipqess_m32(ess, 0, IPQESS_TXQ_CTRL_TXQ_EN, IPQESS_REG_TXQ_CTRL);
	ipqess_m32(ess, 0, IPQESS_RXQ_CTRL_EN, IPQESS_REG_RXQ_CTRL);

	return 0;
}

static void ipqess_cleanup(struct ipqess *ess)
{
	ipqess_reset(ess);
	unregister_netdev(ess->netdev);

	ipqess_tx_ring_free(ess);
	ipqess_rx_ring_free(ess);

	free_netdev(ess->netdev);
}

static int ipqess_axi_probe(struct platform_device *pdev)
{
	struct ipqess *ess;
	struct net_device *netdev;
	struct resource *res;
	int i, err = 0;

	netdev = alloc_etherdev_mqs(sizeof(struct ipqess),
				    IPQESS_NETDEV_QUEUES,
				    IPQESS_NETDEV_QUEUES);
	if (!netdev)
		return -ENODEV;

	ess = netdev_priv(netdev);
	memset(ess, 0, sizeof(struct ipqess));
	ess->netdev = netdev;
	ess->pdev = pdev;
	ess->of_node = pdev->dev.of_node;
	spin_lock_init(&ess->stats_lock);
	SET_NETDEV_DEV(netdev, &pdev->dev);
	platform_set_drvdata(pdev, netdev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ess->hw_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ess->hw_addr)) {
		err = PTR_ERR(ess->hw_addr);
		goto err_out;
	}

	for (i = 0; i < IPQESS_MAX_TX_QUEUE; i++)
		ess->tx_irq[i] = platform_get_irq(pdev, i);
	for (i = 0; i < IPQESS_MAX_RX_QUEUE; i++)
		ess->rx_irq[i] = platform_get_irq(pdev, i + IPQESS_MAX_TX_QUEUE);

	netdev->netdev_ops = &ipqess_axi_netdev_ops;
	netdev->features = NETIF_F_HW_CSUM | NETIF_F_RXCSUM |
			   NETIF_F_HW_VLAN_CTAG_RX |
			   NETIF_F_HW_VLAN_CTAG_TX |
			   NETIF_F_TSO | NETIF_F_TSO6 |
			   NETIF_F_GRO | NETIF_F_SG;
	netdev->hw_features = NETIF_F_HW_CSUM | NETIF_F_RXCSUM |
			      NETIF_F_HW_VLAN_CTAG_RX |
			      NETIF_F_HW_VLAN_CTAG_TX | NETIF_F_SG |
			      NETIF_F_TSO | NETIF_F_TSO6 |
			      NETIF_F_GRO;
	netdev->vlan_features = NETIF_F_HW_CSUM | NETIF_F_SG |
				NETIF_F_TSO | NETIF_F_TSO6 |
				NETIF_F_GRO;
	netdev->wanted_features = NETIF_F_HW_CSUM | NETIF_F_SG |
				  NETIF_F_TSO | NETIF_F_TSO6 |
				  NETIF_F_GRO;
	netdev->watchdog_timeo = 5 * HZ;
	netdev->base_addr = (u32) ess->hw_addr;
	netdev->max_mtu = 9000;
	netdev->gso_max_segs = IPQESS_TX_RING_SIZE / 2;

	ipqess_set_ethtool_ops(netdev);

	netif_carrier_off(netdev);
	err = register_netdev(netdev);
	if (err)
		goto err_out;

	err = ipqess_hw_init(ess);
	if (err)
		goto err_out;

	for (i = 0; i < IPQESS_NETDEV_QUEUES; i++) {
		ess->queue[i].ess = ess;
		ess->queue[i].idx = i;

		netif_napi_add(netdev,
			       &ess->tx_ring[i].napi_tx,
			       ipqess_tx_napi, 64);
		netif_napi_add(netdev,
			       &ess->rx_ring[i].napi_rx,
			       ipqess_rx_napi, 64);

		err = devm_request_irq(&ess->netdev->dev,
			ess->tx_irq[i << IPQESS_TX_CPU_START_SHIFT],
			ipqess_interrupt_tx, 0, "ipqess TX", &ess->tx_ring[i]);
		if (err)
			goto err_out;

		err = devm_request_irq(&ess->netdev->dev,
			ess->rx_irq[i << IPQESS_RX_CPU_START_SHIFT],
			ipqess_interrupt_rx, 0, "ipqess RX", &ess->rx_ring[i]);
		if (err)
			goto err_out;

		/*
		 * irq_set_affinity_hint(ess->tx_irq[i << IPQESS_TX_CPU_START_SHIFT],
		 *			 get_cpu_mask(i));
		 * irq_set_affinity_hint(ess->rx_irq[i << IPQESS_RX_CPU_START_SHIFT],
		 *			 get_cpu_mask(i));
		 */
	}


	return 0;

err_out:
	ipqess_cleanup(ess);
	return err;
}

static int ipqess_axi_remove(struct platform_device *pdev)
{
	const struct net_device *netdev = platform_get_drvdata(pdev);
	struct ipqess *ess = netdev_priv(netdev);

	ipqess_cleanup(ess);

	return 0;
}

static const struct of_device_id ipqess_of_mtable[] = {
	{.compatible = "qcom,ess-edma" },
	{}
};
MODULE_DEVICE_TABLE(of, ipqess_of_mtable);

static struct platform_driver ipqess_axi_driver = {
	.driver = {
		.name    = "ess-edma",
		.of_match_table = ipqess_of_mtable,
	},
	.probe    = ipqess_axi_probe,
	.remove   = ipqess_axi_remove,
};

module_platform_driver(ipqess_axi_driver);

MODULE_AUTHOR("Qualcomm Atheros Inc");
MODULE_AUTHOR("John Crispin <john@phrozen.org>");
MODULE_LICENSE("GPL");
