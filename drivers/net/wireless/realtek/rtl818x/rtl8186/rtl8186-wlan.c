
/* Linux device driver for RTL8180 / RTL8185 / RTL8187SE
 *
 * Copyright 2007 Michael Wu <flamingice@sourmilk.net>
 * Copyright 2007,2014 Andrea Merello <andrea.merello@gmail.com>
 *
 * Based on the r8180 driver, which is:
 * Copyright 2004-2005 Andrea Merello <andrea.merello@gmail.com>, et al.
 *
 * Thanks to Realtek for their support!
 *
 ************************************************************************
 *
 * The driver was extended to the RTL8187SE in 2014 by
 * Andrea Merello <andrea.merello@gmail.com>
 *
 * based also on:
 *  - portions of rtl8187se Linux staging driver, Copyright Realtek corp.
 *    (available in drivers/staging/rtl8187se directory of Linux 3.14)
 *  - other GPL, unpublished (until now), Linux driver code,
 *    Copyright Larry Finger <Larry.Finger@lwfinger.net>
 *
 * A huge thanks goes to Sara V. Nari who forgives me when I'm
 * sitting in front of my laptop at evening, week-end, night...
 *
 * A special thanks goes to Antonio Cuni, who helped me with
 * some python userspace stuff I used to debug RTL8187SE code, and who
 * bought a laptop with an unsupported Wi-Fi card some years ago...
 *
 * Thanks to Larry Finger for writing some code for rtl8187se and for
 * his suggestions.
 *
 * Thanks to Dan Carpenter for reviewing my initial patch and for his
 * suggestions.
 *
 * Thanks to Bernhard Schiffner for his help in testing and for his
 * suggestions.
 *
 ************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/module.h>
#include <net/mac80211.h>
#include <linux/platform_device.h>
#include <linux/ieee80211.h>

#include "rtl8186-wlan.h"
#include "rtl8225.h"

MODULE_AUTHOR("Yasha Cherikovsky <XXXXX@XXXXX");
MODULE_DESCRIPTION("RTL8186 WLAN driver");
MODULE_LICENSE("GPL");


static const struct ieee80211_rate rtl818x_rates[] = {
	{ .bitrate = 10, .hw_value = 0, },
	{ .bitrate = 20, .hw_value = 1, },
	{ .bitrate = 55, .hw_value = 2, },
	{ .bitrate = 110, .hw_value = 3, },
	{ .bitrate = 60, .hw_value = 4, },
	{ .bitrate = 90, .hw_value = 5, },
	{ .bitrate = 120, .hw_value = 6, },
	{ .bitrate = 180, .hw_value = 7, },
	{ .bitrate = 240, .hw_value = 8, },
	{ .bitrate = 360, .hw_value = 9, },
	{ .bitrate = 480, .hw_value = 10, },
	{ .bitrate = 540, .hw_value = 11, },
};

static const struct ieee80211_channel rtl818x_channels[] = {
	{ .center_freq = 2412 },
	{ .center_freq = 2417 },
	{ .center_freq = 2422 },
	{ .center_freq = 2427 },
	{ .center_freq = 2432 },
	{ .center_freq = 2437 },
	{ .center_freq = 2442 },
	{ .center_freq = 2447 },
	{ .center_freq = 2452 },
	{ .center_freq = 2457 },
	{ .center_freq = 2462 },
	{ .center_freq = 2467 },
	{ .center_freq = 2472 },
	{ .center_freq = 2484 },
};

/* Queues for rtl8180/rtl8185 cards
 *
 * name | reg  |  prio
 *  BC  |  7   |   3
 *  HI  |  6   |   0
 *  NO  |  5   |   1
 *  LO  |  4   |   2
 *
 * The complete map for DMA kick reg using all queue is:
 * static const int rtl8186_wlan_queues_map[RTL8180_NR_TX_QUEUES] = {6, 5, 4, 7};
 *
 * .. but .. Because the mac80211 needs at least 4 queues for QoS or
 * otherwise QoS can't be done, we use just one.
 * Beacon queue could be used, but this is not finished yet.
 * Actual map is:
 *
 * name | reg  |  prio
 *  BC  |  7   |   1  <- currently not used yet.
 *  HI  |  6   |   x  <- not used
 *  NO  |  5   |   x  <- not used
 *  LO  |  4   |   0  <- used
 */


#define RTL8186_WLAN_DRIVER_TXL_RING	2	/* Low */
#define RTL8186_WLAN_DRIVER_TXN_RING	1	/* Normal */
#define RTL8186_WLAN_DRIVER_TXH_RING	0	/* High */
#define RTL8186_WLAN_DRIVER_TXB_RING	3	/* Beacon */


static const int rtl8186_wlan_queues_map[RTL8186_NR_TX_QUEUES] = {
	[RTL8186_WLAN_DRIVER_TXL_RING] = 0,
	[RTL8186_WLAN_DRIVER_TXN_RING] = 1,
	[RTL8186_WLAN_DRIVER_TXH_RING] = 2,
	[RTL8186_WLAN_DRIVER_TXB_RING] = 3,
};

static void printhex_cont_nomsg(u8 *buf, unsigned int len) {
	unsigned int i;

	for (i = 0; i < len; i++) {
		printk(KERN_CONT "%02x", buf[i]);
	}
	printk(KERN_CONT "\n");
}


static void printhex(char *msg, u8 *buf, unsigned int len) {
	printk(KERN_ERR "%s: ", msg);
	printhex_cont_nomsg(buf, len);
}

static void rtl8186_wlan_dump_csr(struct rtl8186_wlan_priv *priv, const char *state_msg);

void rtl8186_wlan_write_phy_new(struct ieee80211_hw *dev, u8 addr, u32 data)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	u8 databyte = (u8)(data & 0xff);
	u8 readback;
	int errors = 0;

	do {
		rtl818x_iowrite8(priv, &priv->map->PHYDATAW, databyte);
		rtl818x_iowrite8(priv, &priv->map->PHYADDR, addr | 0x80);

		mdelay(2);

		readback = rtl8186_wlan_read_phy(dev, addr);
		if (readback != databyte) {
			printk(KERN_ERR "warning: PHY write to %02x failed. got %02x instead of %02x. Retrying...\n", addr, readback, databyte);
			errors++;
		}
	} while (readback != databyte && errors < 10);

	if (errors == 10) {
		printk(KERN_ERR "ERROR: PHY write to %02x COMPLETELY failed. got %02x instead of %02x\n", addr, readback, databyte);
	}
}

void rtl8186_wlan_write_phy(struct ieee80211_hw *dev, u8 addr, u32 data)
{
        struct rtl8186_wlan_priv *priv = dev->priv;
        int i = 10;
        u32 buf;

        buf = (data << 8) | addr;

        rtl818x_iowrite32(priv, (__be32 __iomem *)&priv->map->PHY[0], buf | 0x80);
        while (i--) {
                rtl818x_iowrite32(priv, (__be32 __iomem *)&priv->map->PHY[0], buf);
                if (rtl818x_ioread8(priv, &priv->map->PHY[2]) == (data & 0xFF))
                        return;
        }
}


u8 rtl8186_wlan_read_phy(struct ieee80211_hw *dev, u8 addr)
{
	struct rtl8186_wlan_priv *priv = dev->priv;

	rtl818x_iowrite8(priv, &priv->map->PHYADDR, addr);
	mdelay(2);
	return rtl818x_ioread8(priv, &priv->map->PHYDATAR);
	// rtl818x_iowrite8(priv, &priv->map->PHYDATAW, (u8) (data & 0xFF));
	// while (i--) {
	// 	rtl818x_iowrite8(priv, &priv->map->PHYADDR, addr);
	// 	mdelay(10);
	// 	// rtl818x_iowrite32(priv, (__be32 __iomem *)&priv->map->PHY[0], buf);
	// 	if (rtl818x_ioread8(priv, &priv->map->PHYDATAR) == (data & 0xFF)) {
	// 		printk(KERN_ERR "v1 write_phy: i = %d\n", i);
	// 		return;
	// 	}
	// }

	// printk(KERN_ERR "v1 write_phy: i = %d\n", i);
}

static struct device* rtl8186_wlan_get_dev(struct rtl8186_wlan_priv *priv) {
	if (priv->is_mmio) {
		return &(priv->plat_dev->dev);
	} else {
		return &(priv->pdev->dev);
	}
}

static void rtl8186_wlan_fifo_selt_test(struct rtl8186_wlan_priv *priv) {
	u8 config5 = rtl818x_ioread8(priv, &priv->map->CONFIG5);
	u8 tx_ok = (config5 & BIT(7)) != 0;
	u8 rx_ok = (config5 & BIT(6)) != 0;
	if (!tx_ok || !rx_ok) {
		printk(KERN_ERR "WARNING: fifo selt test fail: TX: %s, RX: %s\n", tx_ok ? "OK" : "FAIL",
					rx_ok ? "OK" : "FAIL");
	}
}

static void rtl8186_wlan_handle_rx(struct ieee80211_hw *dev)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	struct rtl818x_rx_cmd_desc *cmd_desc;
	unsigned int count = 32;
	u8 agc;
	s8 signal = 1;
	dma_addr_t mapping;
	struct ieee80211_mgmt *mgmt_test;

	while (count--) {
		void *entry = priv->rx_ring + priv->rx_idx * priv->rx_ring_sz;
		struct sk_buff *skb = priv->rx_buf[priv->rx_idx];
		u32 flags, flags2;
		u64 tsft;

		struct rtl8186_wlan_rx_desc *desc = entry;

		flags = be32_to_cpu(desc->flags);
		/* same as above */
		rmb();
		flags2 = be32_to_cpu(desc->flags2);
		tsft = be64_to_cpu(desc->tsft);

		if (flags & RTL818X_RX_DESC_FLAG_OWN)
			return;

		if (unlikely(flags & (RTL818X_RX_DESC_FLAG_DMA_FAIL |
				      RTL818X_RX_DESC_FLAG_FOF |
				      RTL818X_RX_DESC_FLAG_RX_ERR)))
			goto done;
		else {
			struct ieee80211_rx_status rx_status = {0};
			struct sk_buff *new_skb = dev_alloc_skb(MAX_RX_SIZE);

			if (unlikely(!new_skb))
				goto done;

			mapping = dma_map_single(rtl8186_wlan_get_dev(priv),
					       skb_tail_pointer(new_skb),
					       MAX_RX_SIZE, PCI_DMA_FROMDEVICE);

			if (dma_mapping_error(rtl8186_wlan_get_dev(priv), mapping)) {
				kfree_skb(new_skb);
				dev_err(rtl8186_wlan_get_dev(priv), "RX DMA map error\n");

				goto done;
			}

			dma_unmap_single(rtl8186_wlan_get_dev(priv),
					 *((dma_addr_t *)skb->cb),
					 MAX_RX_SIZE, PCI_DMA_FROMDEVICE);
			skb_put(skb, flags & 0xFFF);

			rx_status.antenna = (flags2 >> 15) & 1;
			rx_status.rate_idx = (flags >> 20) & 0xF;
			agc = (flags2 >> 17) & 0x7F;

			if (rx_status.rate_idx > 3)
				signal = -clamp_t(u8, agc, 25, 90) - 9;
			else
				signal = -clamp_t(u8, agc, 30, 95);
			rx_status.signal = signal;
			rx_status.freq = dev->conf.chandef.chan->center_freq;
			rx_status.band = dev->conf.chandef.chan->band;
			rx_status.mactime = tsft;
			rx_status.flag |= RX_FLAG_MACTIME_START;
			if (flags & RTL818X_RX_DESC_FLAG_SPLCP)
				rx_status.flag |= RX_ENC_FLAG_SHORTPRE;
			if (flags & RTL818X_RX_DESC_FLAG_CRC32_ERR)
				rx_status.flag |= RX_FLAG_FAILED_FCS_CRC;

			memcpy(IEEE80211_SKB_RXCB(skb), &rx_status, sizeof(rx_status));

			mgmt_test = (struct ieee80211_mgmt *) skb->data;
			if (memcmp(mgmt_test->da, priv->mac_addr, 6) == 0) {
				printhex("received", skb->data, skb->len);
			}
			ieee80211_rx_irqsafe(dev, skb);

			skb = new_skb;
			priv->rx_buf[priv->rx_idx] = skb;
			*((dma_addr_t *) skb->cb) = mapping;
		}

	done:
		cmd_desc = entry;
		cmd_desc->rx_buf = cpu_to_be32(*((dma_addr_t *)skb->cb));
		cmd_desc->flags = cpu_to_be32(RTL818X_RX_DESC_FLAG_OWN |
					   MAX_RX_SIZE);
		if (priv->rx_idx == 31)
			cmd_desc->flags |=
				cpu_to_be32(RTL818X_RX_DESC_FLAG_EOR);
		priv->rx_idx = (priv->rx_idx + 1) % 32;

		//printk(KERN_ERR "handled one RX <---\n");
	}
}

static void rtl8186_wlan_handle_tx(struct ieee80211_hw *dev, unsigned int prio)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	struct rtl8186_wlan_tx_ring *ring = &priv->tx_ring[prio];
	int freed_index;

	// printk(KERN_ERR "rtl8186_wlan_handle_tx   prio=%d\n", prio);

	while (skb_queue_len(&ring->queue)) {
		struct rtl8186_wlan_tx_desc *entry = &ring->desc[ring->idx];
		struct sk_buff *skb;
		struct ieee80211_tx_info *info;
		u32 flags = *((u32*)entry); // TODO horrible hack

		if (flags & RTL818X_TX_DESC_FLAG_OWN)
			return;

		freed_index = ring->idx;

		ring->idx = (ring->idx + 1) % ring->entries;
		skb = __skb_dequeue(&ring->queue);
		dma_unmap_single(rtl8186_wlan_get_dev(priv), be32_to_cpu(entry->tx_buf),
				 skb->len, PCI_DMA_TODEVICE);

		info = IEEE80211_SKB_CB(skb);
		ieee80211_tx_info_clear_status(info);

		if (!(info->flags & IEEE80211_TX_CTL_NO_ACK) &&
		    (flags & RTL818X_TX_DESC_FLAG_TX_OK))
			info->flags |= IEEE80211_TX_STAT_ACK;

		info->status.rates[0].count = (flags & 0xFF) + 1;

		ieee80211_tx_status_irqsafe(dev, skb);
		if (ring->entries - skb_queue_len(&ring->queue) == 2)
			ieee80211_wake_queue(dev, prio);
		
		printk(KERN_ERR "TX: freed entry at index = %d\n", freed_index);

		//printk(KERN_ERR "handled one TX --->\n");
	}
}

static irqreturn_t rtl8186_wlan_interrupt(int irq, void *dev_id)
{
	struct ieee80211_hw *dev = dev_id;
	struct rtl8186_wlan_priv *priv = dev->priv;
	u16 reg;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	reg = rtl818x_ioread16(priv, &priv->map->INT_STATUS);
	if (unlikely(reg == 0xFFFF)) {
		spin_unlock_irqrestore(&priv->lock, flags);
		return IRQ_HANDLED;
	}

	rtl818x_iowrite16(priv, &priv->map->INT_STATUS, reg);

	rtl8186_wlan_fifo_selt_test(priv);

	if (reg & RTL818X_INT_BEACON) {
		printk(KERN_ERR "Great! NIC prompted beacon interrupt\n");
	}

	if (reg & (RTL818X_INT_TXB_OK | RTL818X_INT_TXB_ERR)) {
		if (reg & RTL818X_INT_TXB_OK) {
			printk(KERN_ERR "TXB_OK\n");
		}
		if (reg & RTL818X_INT_TXB_ERR) {
			printk(KERN_ERR "ATTENTION: TXB_ERR\n");
		}

		rtl8186_wlan_handle_tx(dev, RTL8186_WLAN_DRIVER_TXB_RING);
	}

	if (reg & (RTL818X_INT_TXL_OK | RTL818X_INT_TXL_ERR)) {
		if (reg & RTL818X_INT_TXL_OK) {
			printk(KERN_ERR "TXL_OK\n");
		}
		if (reg & RTL818X_INT_TXL_ERR) {
			printk(KERN_ERR "ATTENTION: TXL_ERR\n");
		}
		rtl8186_wlan_handle_tx(dev, RTL8186_WLAN_DRIVER_TXL_RING);
	}

	if (reg & (RTL818X_INT_TXH_OK | RTL818X_INT_TXH_ERR)) {
		if (reg & RTL818X_INT_TXH_OK) {
			printk(KERN_ERR "TXH_OK\n");
		}
		if (reg & RTL818X_INT_TXH_ERR) {
			printk(KERN_ERR "ATTENTION: TXH_ERR bla\n");
		}
		rtl8186_wlan_handle_tx(dev, RTL8186_WLAN_DRIVER_TXH_RING);
	}

	if (reg & (RTL818X_INT_TXN_OK | RTL818X_INT_TXN_ERR)) {
		if (reg & RTL818X_INT_TXN_OK) {
			printk(KERN_ERR "TXN_OK\n");
		}
		if (reg & RTL818X_INT_TXN_ERR) {
			printk(KERN_ERR "ATTENTION: TXN_ERR\n");
		}
		rtl8186_wlan_handle_tx(dev, RTL8186_WLAN_DRIVER_TXN_RING);
	}

	if (reg & (RTL818X_INT_RX_OK | RTL818X_INT_RX_ERR)) {
		if (reg & RTL818X_INT_RX_ERR) {
			printk(KERN_ERR "ATTENTION: RX_ERR\n");
		}

		rtl8186_wlan_handle_rx(dev);
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return IRQ_HANDLED;
}

static void rtl8186_wlan_tx(struct ieee80211_hw *dev,
		       struct ieee80211_tx_control *control,
		       struct sk_buff *skb)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	struct rtl8186_wlan_priv *priv = dev->priv;
	struct rtl8186_wlan_tx_ring *ring;
	struct rtl8186_wlan_tx_desc *entry;
	unsigned long flags;
	unsigned int idx, prio, hw_prio;
	dma_addr_t mapping;
	// u32 tx_flags;
	u8 rc_flags;
	u16 plcp_len = 0;
	__be16 rts_duration = 0;
	u8 tppoll_reg;
	union rtl8186_wlan_tx_desc_flags my_flags = {0};
	// u32 flags_v2 = 0;
	/* do arithmetic and then convert to be16 */

	/* TODO: THIS TEST WORKED! when setting dst=BROADCAST, the NIC doesn't cry! */
	//memset(hdr->addr1, 0xff, 6);
	//memcpy(hdr->addr1, priv->mac_addr, 6);

	BUILD_BUG_ON(offsetof(struct rtl8186_wlan_tx_desc, __rsvd2) != 28);

	// printk(KERN_ERR "rtl8186_wlan_tx_desc size: %d\n", sizeof(struct rtl8186_wlan_tx_desc));
	// BUILD_BUG_ON(sizeof(struct rtl8186_wlan_tx_desc) != (32));

	printhex("sendbuf", skb->data, skb->len);
	//dump_stack();

	prio = skb_get_queue_mapping(skb);
	ring = &priv->tx_ring[prio];

	mapping = dma_map_single(rtl8186_wlan_get_dev(priv), skb->data,
				 skb->len, PCI_DMA_TODEVICE);

	// printk(KERN_ERR "TX DMA mapping: %08x\n", mapping);

	if (dma_mapping_error(rtl8186_wlan_get_dev(priv), mapping)) {
		kfree_skb(skb);
		dev_err(rtl8186_wlan_get_dev(priv), "TX DMA mapping error\n");
		return;
	}

	idx = (ring->idx + skb_queue_len(&ring->queue)) % ring->entries;
	entry = &ring->desc[idx];

	entry->flags.flags_dword = 0; // Set only this bit and zero all the other
	wmb();
	if (entry->flags.own != 0) {
		panic("HAHAHAHA YOU FAILED\n");
	}

	my_flags.own = 0;

	// entry->flags.own = 1;
	// entry->flags.flags_without_own = 0; // Zero remaining 31 bits
	my_flags.dma_ok = 1;
	my_flags.no_encryption = 1;
	my_flags.fs = 1;
	my_flags.ls = 1;
	my_flags.txrate = ieee80211_get_tx_rate(dev, info)->hw_value;
	my_flags.tpktsize = skb->len;

	// tx_flags = RTL818X_TX_DESC_FLAG_OWN | RTL818X_TX_DESC_FLAG_FS |
	// 	   RTL818X_TX_DESC_FLAG_LS |
	// 	   (ieee80211_get_tx_rate(dev, info)->hw_value << 24) |
	// 	   skb->len;

	// tx_flags |= RTL818X_TX_DESC_FLAG_DMA |
			// RTL818X_TX_DESC_FLAG_NO_ENC;

	rc_flags = info->control.rates[0].flags;

	/* HW will perform RTS-CTS when only RTS flags is set.
	 * HW will perform CTS-to-self when both RTS and CTS flags are set.
	 * RTS rate and RTS duration will be used also for CTS-to-self.
	 */
	if (rc_flags & IEEE80211_TX_RC_USE_RTS_CTS) {
		// tx_flags |= RTL818X_TX_DESC_FLAG_RTS;
		my_flags.rts_en = 1;
		my_flags.rts_rate = ieee80211_get_rts_cts_rate(dev, info)->hw_value;
		// tx_flags |= ieee80211_get_rts_cts_rate(dev, info)->hw_value << 19;
		rts_duration = ieee80211_rts_duration(dev, priv->vif,
						skb->len, info);
	} else if (rc_flags & IEEE80211_TX_RC_USE_CTS_PROTECT) {
		// tx_flags |= RTL818X_TX_DESC_FLAG_RTS | RTL818X_TX_DESC_FLAG_CTS;
		my_flags.rts_en = 1;
		my_flags.cts_en = 1;
		my_flags.rts_rate = ieee80211_get_rts_cts_rate(dev, info)->hw_value;
		// tx_flags |= ieee80211_get_rts_cts_rate(dev, info)->hw_value << 19;
		rts_duration = ieee80211_ctstoself_duration(dev, priv->vif,
						skb->len, info);
	}

	spin_lock_irqsave(&priv->lock, flags);

	if (info->flags & IEEE80211_TX_CTL_ASSIGN_SEQ) {
		if (info->flags & IEEE80211_TX_CTL_FIRST_FRAGMENT)
			priv->seqno += 0x10;
		hdr->seq_ctrl &= cpu_to_be16(IEEE80211_SCTL_FRAG);
		hdr->seq_ctrl |= cpu_to_be16(priv->seqno);
	}

	entry->frame_len = skb->len;

	entry->rts_dur = rts_duration;
	entry->plcp_length = plcp_len;
	entry->tx_buf = mapping;

	entry->retry_limit = info->control.rates[0].count - 1;
	printk("tx ring entry, retry limit: %d\n", entry->retry_limit);

	/* We must be sure that tx_flags is written last because the HW
	 * looks at it to check if the rest of data is valid or not
	 */

	my_flags.own = 1;
	wmb();
	entry->flags.flags_dword = my_flags.flags_dword;

	//printk(KERN_ERR "sending tx: idx=%02d, frame_len=%04d, rts_duration=%d, plcp_len=%d, tx_buf=%08x, retry_limit=%d, flags=%08x\n",
	//	   idx, entry->frame_len, entry->rts_duration, entry->plcp_len, entry->tx_buf, entry->retry_limit, entry->flags);
	/* We must be sure this has been written before followings HW
	 * register write, because this write will made the HW attempts
	 * to DMA the just-written data
	 */
	wmb();

	if (entry->flags.own != 1) {
		panic("HAHAHAHA YOU FAILED   v2\n");
	}

	__skb_queue_tail(&ring->queue, skb);
	// if (ring->entries - skb_queue_len(&ring->queue) < 2)
		// ieee80211_stop_queue(dev, prio);

	spin_unlock_irqrestore(&priv->lock, flags);

	hw_prio = rtl8186_wlan_queues_map[prio];
	// printk(KERN_ERR "TPPOLL offset: %08x\n", (uint32_t)((void*)&priv->map->TX_DMA_POLLING - (void*)&priv->map->MAC));
	tppoll_reg = rtl818x_ioread8(priv, &priv->map->TX_DMA_POLLING);
	// printk(KERN_ERR "tppoll_reg before: %08x\n", tppoll_reg);
	tppoll_reg |= (1 << hw_prio) << 4; /* ring to poll  */
	// printk(KERN_ERR "tppoll_reg after: %08x\n", tppoll_reg);
	rtl818x_iowrite8(priv, &priv->map->TX_DMA_POLLING, tppoll_reg);
}

void rtl8186_wlan_set_anaparam2(struct rtl8186_wlan_priv *priv, u32 anaparam2)
{
	u8 reg;

	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD,
			 RTL818X_EEPROM_CMD_CONFIG);

	reg = rtl818x_ioread8(priv, &priv->map->CONFIG3);
	rtl818x_iowrite8(priv, &priv->map->CONFIG3,
		 reg | RTL818X_CONFIG3_ANAPARAM_WRITE);

	rtl818x_iowrite32(priv, &priv->map->ANAPARAM2, anaparam2);

	rtl818x_iowrite8(priv, &priv->map->CONFIG3,
		 reg & ~RTL818X_CONFIG3_ANAPARAM_WRITE);

	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD,
			 RTL818X_EEPROM_CMD_NORMAL);
}

void rtl8186_wlan_set_anaparam(struct rtl8186_wlan_priv *priv, u32 anaparam)
{
	u8 reg;

	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
	reg = rtl818x_ioread8(priv, &priv->map->CONFIG3);
	rtl818x_iowrite8(priv, &priv->map->CONFIG3,
		 reg | RTL818X_CONFIG3_ANAPARAM_WRITE);
	rtl818x_iowrite32(priv, &priv->map->ANAPARAM, anaparam);
	rtl818x_iowrite8(priv, &priv->map->CONFIG3,
		 reg & ~RTL818X_CONFIG3_ANAPARAM_WRITE);
	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_NORMAL);
}

static void rtl8186_wlan_int_enable(struct ieee80211_hw *dev)
{
	struct rtl8186_wlan_priv *priv = dev->priv;

	rtl818x_iowrite16(priv, &priv->map->INT_MASK, 0xFFFF);
}

static void rtl8186_wlan_int_disable(struct ieee80211_hw *dev)
{
	struct rtl8186_wlan_priv *priv = dev->priv;

	rtl818x_iowrite16(priv, &priv->map->INT_MASK, 0);
}

static void rtl8186_wlan_conf_basic_rates(struct ieee80211_hw *dev,
			    u32 basic_mask)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	u32 resp_mask;
	u8 resp_max, resp_min;
	basic_mask = 0x1f0;

	priv->basic_mask = basic_mask;
	resp_mask = basic_mask;
	/* IEEE80211 says the response rate should be equal to the highest basic
	 * rate that is not faster than received frame. But it says also that if
	 * the basic rate set does not contains any rate for the current
	 * modulation class then mandatory rate set must be used for that
	 * modulation class. Eventually add OFDM mandatory rates..
	 */
	// if ((resp_mask & 0xf) == resp_mask)
		// resp_mask |= 0x150; /* 6, 12, 24Mbps */

	resp_max = fls(resp_mask) - 1;
	resp_min = ffs(resp_mask) - 1;
	printk(KERN_ERR "resp_min=0x%x,  resp_max=0x%x\n", resp_min, resp_max);

	/* HACK PATCHES */
	// basic_mask = 0x01ff;
	// resp_max = 0x4;
	// resp_min = 0x4;

	/* in 8185 this is a BITMAP */
	rtl818x_iowrite16(priv, &priv->map->BRSR, basic_mask);
	rtl818x_iowrite8(priv, &priv->map->RESP_RATE, (resp_max << 4) | resp_min);
}

static void rtl8186_wlan_config_cardbus(struct ieee80211_hw *dev)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	u16 reg16;
	u8 reg8;

	reg8 = rtl818x_ioread8(priv, &priv->map->CONFIG3);
	reg8 |= 1 << 1;
	rtl818x_iowrite8(priv, &priv->map->CONFIG3, reg8);

	reg16 = rtl818x_ioread16(priv, &priv->map->FEMR);
	reg16 |= (1 << 15) | (1 << 14) | (1 << 4);
	rtl818x_iowrite16(priv, &priv->map->FEMR, reg16);
}

static int rtl8186_wlan_init_hw(struct ieee80211_hw *dev)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	u16 reg;

	printk(KERN_ERR "rtl8186_wlan_init_hw()\n");

	rtl818x_iowrite8(priv, &priv->map->CMD, 0);
	rtl818x_ioread8(priv, &priv->map->CMD);
	msleep(10);

	/* reset */
	rtl8186_wlan_int_disable(dev);
	rtl818x_ioread8(priv, &priv->map->CMD);

	reg = rtl818x_ioread8(priv, &priv->map->CMD);
	reg &= (1 << 1);
	reg |= RTL818X_CMD_RESET;
	rtl818x_iowrite8(priv, &priv->map->CMD, RTL818X_CMD_RESET);
	rtl818x_ioread8(priv, &priv->map->CMD);
	msleep(200);

	/* check success of reset */
	if (rtl818x_ioread8(priv, &priv->map->CMD) & RTL818X_CMD_RESET) {
		wiphy_err(dev->wiphy, "reset timeout!\n");
		return -ETIMEDOUT;
	}

	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_LOAD);
	rtl818x_ioread8(priv, &priv->map->CMD);
	msleep(200);

	// TODO verify this
	if (rtl818x_ioread8(priv, &priv->map->CONFIG3) & (1 << 3)) {
		rtl8186_wlan_config_cardbus(dev);
	}

	rtl818x_iowrite8(priv, &priv->map->MSR, RTL818X_MSR_MASTER); // THIS IS VERY IMPORTANT FOR BEACONS

	rtl818x_iowrite32(priv, &priv->map->RDSAR, priv->rx_ring_dma);
	/* mac80211 queue have higher prio for lower index. The last queue
	 * (that mac80211 is not aware of) is reserved for beacons (and have
	 * the highest priority on the NIC)
	 */
	rtl818x_iowrite32(priv, &priv->map->TLPDA, priv->tx_ring[RTL8186_WLAN_DRIVER_TXL_RING].dma);
	rtl818x_iowrite32(priv, &priv->map->TNPDA, priv->tx_ring[RTL8186_WLAN_DRIVER_TXN_RING].dma);
	rtl818x_iowrite32(priv, &priv->map->THPDA, priv->tx_ring[RTL8186_WLAN_DRIVER_TXH_RING].dma);
	rtl818x_iowrite32(priv, &priv->map->TBDA, priv->tx_ring[RTL8186_WLAN_DRIVER_TXB_RING].dma);

	printk(KERN_ERR "tx ring 3 dma addr: %08x\n", priv->tx_ring[3].dma);

	/* TODO: necessary? specs indicate not */
	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
	reg = rtl818x_ioread8(priv, &priv->map->CONFIG2);
	rtl818x_iowrite8(priv, &priv->map->CONFIG2, reg & ~(1 << 3));
	reg = rtl818x_ioread8(priv, &priv->map->CONFIG2);
	rtl818x_iowrite8(priv, &priv->map->CONFIG2, reg | (1 << 4));
	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_NORMAL);

	/* TODO: set CONFIG5 for calibrating AGC on rtl8180 + philips radio? */

	/* TODO: turn off hw wep on rtl8180 */

	rtl818x_iowrite32(priv, &priv->map->INT_TIMEOUT, 0);

	rtl818x_iowrite16(priv, (__be16 __iomem *)&priv->map->WPA_CONF, 0);
	rtl818x_iowrite8(priv, &priv->map->RATE_FALLBACK, 0);
	rtl818x_iowrite8(priv, &priv->map->CW_CONF, RTL818X_CW_CONF_PERPACKET_RETRY);

	/* TODO: set ClkRun enable? necessary? */
	reg = rtl818x_ioread8(priv, &priv->map->GP_ENABLE);
	rtl818x_iowrite8(priv, &priv->map->GP_ENABLE, reg & ~(1 << 6));
	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
	reg = rtl818x_ioread8(priv, &priv->map->CONFIG3);
	rtl818x_iowrite8(priv, &priv->map->CONFIG3, reg | (1 << 2));
	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_NORMAL);
	/* fix eccessive IFS after CTS-to-self */
	if (priv->map_pio) {
		u8 reg;

		reg = rtl818x_ioread8(priv, &priv->map->PGSELECT);
		rtl818x_iowrite8(priv, &priv->map->PGSELECT, reg | 1);
		rtl818x_iowrite8(priv, REG_ADDR1(0xff), 0x35);
		rtl818x_iowrite8(priv, &priv->map->PGSELECT, reg);
	} else {
		rtl818x_iowrite8(priv, REG_ADDR1(0x1ff), 0x35);
	}

	/* make sure all rings are not stopped */
	rtl818x_iowrite8(priv, &priv->map->TX_DMA_POLLING, 0);

	priv->rf->init(dev);

	/* default basic rates are 1,2 Mbps for rtl8180. 1,2,6,9,12,18,24 Mbps
	 * otherwise. bitmask 0x3 and 0x01f3 respectively.
	 * NOTE: currenty rtl8225 RF code changes basic rates, so we need to do
	 * this after rf init.
	 * TODO: try to find out whether RF code really needs to do this..
	 */
	rtl8186_wlan_conf_basic_rates(dev, 0x1ff);

	return 0;
}

static int rtl8186_wlan_init_rx_ring(struct ieee80211_hw *dev)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	struct rtl818x_rx_cmd_desc *entry;
	int i;

	priv->rx_ring_sz = sizeof(struct rtl8186_wlan_rx_desc);

	priv->rx_ring = dma_zalloc_coherent(rtl8186_wlan_get_dev(priv), priv->rx_ring_sz * 32,
					      &priv->rx_ring_dma, GFP_ATOMIC);
	if (!priv->rx_ring || (unsigned long)priv->rx_ring & 0xFF) {
		wiphy_err(dev->wiphy, "Cannot allocate RX ring\n");
		return -ENOMEM;
	}

	priv->rx_idx = 0;

	for (i = 0; i < 32; i++) {
		struct sk_buff *skb = dev_alloc_skb(MAX_RX_SIZE);
		dma_addr_t *mapping;
		entry = priv->rx_ring + priv->rx_ring_sz*i;
		if (!skb) {
			dma_free_coherent(rtl8186_wlan_get_dev(priv), priv->rx_ring_sz * 32,
					priv->rx_ring, priv->rx_ring_dma);
			wiphy_err(dev->wiphy, "Cannot allocate RX skb\n");
			return -ENOMEM;
		}
		priv->rx_buf[i] = skb;
		mapping = (dma_addr_t *)skb->cb;
		*mapping = dma_map_single(rtl8186_wlan_get_dev(priv), skb_tail_pointer(skb),
					  MAX_RX_SIZE, PCI_DMA_FROMDEVICE);

		if (dma_mapping_error(rtl8186_wlan_get_dev(priv), *mapping)) {
			kfree_skb(skb);
			dma_free_coherent(rtl8186_wlan_get_dev(priv), priv->rx_ring_sz * 32,
					priv->rx_ring, priv->rx_ring_dma);
			wiphy_err(dev->wiphy, "Cannot map DMA for RX skb\n");
			return -ENOMEM;
		}

		entry->rx_buf = cpu_to_be32(*mapping);
		entry->flags = cpu_to_be32(RTL818X_RX_DESC_FLAG_OWN |
					   MAX_RX_SIZE);
	}
	entry->flags |= cpu_to_be32(RTL818X_RX_DESC_FLAG_EOR);
	return 0;
}

static void rtl8186_wlan_free_rx_ring(struct ieee80211_hw *dev)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	int i;

	for (i = 0; i < 32; i++) {
		struct sk_buff *skb = priv->rx_buf[i];
		if (!skb)
			continue;

		dma_unmap_single(rtl8186_wlan_get_dev(priv),
				 *((dma_addr_t *)skb->cb),
				 MAX_RX_SIZE, PCI_DMA_FROMDEVICE);
		kfree_skb(skb);
	}

	dma_free_coherent(rtl8186_wlan_get_dev(priv), priv->rx_ring_sz * 32,
			    priv->rx_ring, priv->rx_ring_dma);
	priv->rx_ring = NULL;
}

static int rtl8186_wlan_init_tx_ring(struct ieee80211_hw *dev,
				unsigned int prio, unsigned int entries)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	struct rtl8186_wlan_tx_desc *ring;
	dma_addr_t dma;
	int i;
	size_t aligned_size = ALIGN(sizeof(*ring), 256); // TODO we can alloc much less!

	// printk("single tx desc size: %d. aligned size: %d\n", sizeof(*ring), aligned_size);

	ring = dma_zalloc_coherent(rtl8186_wlan_get_dev(priv), aligned_size * entries,
				     &dma, GFP_ATOMIC);
	if (!ring || (unsigned long)ring & 0xFF) {
		wiphy_err(dev->wiphy, "Cannot allocate TX ring (prio = %d)\n",
			  prio);
		return -ENOMEM;
	}

	if (((uint32_t)ring) % 256 != 0) {
		panic("unaligned tx ring descriptor");
	}

	priv->tx_ring[prio].desc = ring;
	priv->tx_ring[prio].dma = dma;
	priv->tx_ring[prio].idx = 0;
	priv->tx_ring[prio].entries = entries;
	skb_queue_head_init(&priv->tx_ring[prio].queue);

	for (i = 0; i < entries; i++)
		ring[i].next_tx_desc =
			cpu_to_be32((u32)dma + ((i + 1) % entries) * aligned_size);

	return 0;
}

static void rtl8186_wlan_free_tx_ring(struct ieee80211_hw *dev, unsigned int prio)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	struct rtl8186_wlan_tx_ring *ring = &priv->tx_ring[prio];

	while (skb_queue_len(&ring->queue)) {
		struct rtl8186_wlan_tx_desc *entry = &ring->desc[ring->idx];
		struct sk_buff *skb = __skb_dequeue(&ring->queue);

		dma_unmap_single(rtl8186_wlan_get_dev(priv), be32_to_cpu(entry->tx_buf),
				 skb->len, PCI_DMA_TODEVICE);
		kfree_skb(skb);
		ring->idx = (ring->idx + 1) % ring->entries;
	}

	dma_free_coherent(rtl8186_wlan_get_dev(priv), sizeof(*ring->desc)*ring->entries,
			    ring->desc, ring->dma);
	ring->desc = NULL;
}

static int rtl8186_wlan_start(struct ieee80211_hw *dev)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	int ret, i;
	u32 reg;

	printk(KERN_ERR "rtl8186_wlan_start\n");

	ret = rtl8186_wlan_init_rx_ring(dev);
	if (ret)
		return ret;

	for (i = 0; i < RTL8186_NR_TX_QUEUES; i++)
		if ((ret = rtl8186_wlan_init_tx_ring(dev, i, 16)))
			goto err_free_rings;

	ret = rtl8186_wlan_init_hw(dev);
	if (ret)
		goto err_free_rings;

	ret = request_irq(priv->irq, rtl8186_wlan_interrupt, 0, "rtl8186-wlan", dev);  // TODO driver name

	if (ret) {
		wiphy_err(dev->wiphy, "failed to register IRQ handler\n");
		goto err_free_rings;
	}

	rtl8186_wlan_int_enable(dev);

	rtl818x_iowrite32(priv, &priv->map->MAR[0], ~0);
	rtl818x_iowrite32(priv, &priv->map->MAR[1], ~0);

	// we said:      0xf014070e
	// 
	// we say(new):  0x1054e20e
	// stock says:   0x1054e20e

	//               0x1054e20e

	reg = RTL818X_RX_CONF_RX_AUTORESETPHY |
	      RTL818X_RX_CONF_PM |
	      RTL818X_RX_CONF_MGMT |
	      RTL818X_RX_CONF_DATA |
		  (7 << 13 /* RX FIFO Threshold */) |
	      (2 << 8 /* MAX RX DMA */) |
	      RTL818X_RX_CONF_BROADCAST |
		  RTL818X_RX_CONF_MULTICAST |
	      RTL818X_RX_CONF_NICMAC;

	reg |= RTL818X_RX_CONF_CSDM1 | RTL818X_RX_CONF_CSDM2;

	priv->rx_conf = reg;
	printk(KERN_ERR "setting rx_conf = %08x", reg);
	rtl818x_iowrite32(priv, &priv->map->RX_CONF, reg);

	reg = rtl818x_ioread8(priv, &priv->map->CW_CONF);

	/* CW is not on per-packet basis.
		* in rtl8185 the CW_VALUE reg is used.
		* in rtl8187se the AC param regs are used.
		*/
	reg &= ~RTL818X_CW_CONF_PERPACKET_CW;
	/* retry limit IS on per-packet basis.
		* the short and long retry limit in TX_CONF
		* reg are ignored
		*/
	reg |= RTL818X_CW_CONF_PERPACKET_RETRY;
	rtl818x_iowrite8(priv, &priv->map->CW_CONF, reg);

	reg = rtl818x_ioread8(priv, &priv->map->TX_AGC_CTL);
	/* TX antenna and TX gain are not on per-packet basis.
		* TX Antenna is selected by ANTSEL reg (RX in BB regs).
		* TX gain is selected with CCK_TX_AGC and OFDM_TX_AGC regs
		*/
	reg &= ~RTL818X_TX_AGC_CTL_PERPACKET_GAIN;
	reg &= ~RTL818X_TX_AGC_CTL_PERPACKET_ANTSEL;
	reg |=  RTL818X_TX_AGC_CTL_FEEDBACK_ANT;
	rtl818x_iowrite8(priv, &priv->map->TX_AGC_CTL, reg);

	/* disable early TX */
	// rtl818x_iowrite8(priv, (u8 __iomem *)priv->map + 0xec, 0x3f);

	reg = 0;
	reg |= (7 << 21) |  /* MAX TX DMA */
	       RTL818X_TX_CONF_NO_ICV;

	// reg |= RTL818X_TX_CONF_R8185_ABC;
	// reg |= RTL818X_TX_CONF_DISREQQSIZE;
	reg &= ~RTL818X_TX_CONF_PROBE_DTS;
	reg &= ~RTL818X_TX_CONF_DISCW;

	/* different meaning, same value on both rtl8185 and rtl8180 */
	reg &= ~RTL818X_TX_CONF_SAT_HWPLCP;

	reg |= 0xffff; // Short+long retry limit

	printk(KERN_ERR "tx conf: %08x\n", reg);

	rtl818x_iowrite32(priv, &priv->map->TX_CONF, reg);

	reg = rtl818x_ioread8(priv, &priv->map->CMD);
	reg |= RTL818X_CMD_RX_ENABLE;
	reg |= RTL818X_CMD_TX_ENABLE;
	rtl818x_iowrite8(priv, &priv->map->CMD, reg);

	rtl818x_iowrite8(priv, &priv->map->EIFS, 0x5b);

	return 0;

 err_free_rings:
	rtl8186_wlan_free_rx_ring(dev);
	for (i = 0; i < (dev->queues + 1); i++)
		if (priv->tx_ring[i].desc)
			rtl8186_wlan_free_tx_ring(dev, i);

	return ret;
}

static void rtl8186_wlan_stop(struct ieee80211_hw *dev)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	u8 reg;
	int i;

	rtl8186_wlan_int_disable(dev);

	reg = rtl818x_ioread8(priv, &priv->map->CMD);
	reg &= ~RTL818X_CMD_TX_ENABLE;
	reg &= ~RTL818X_CMD_RX_ENABLE;
	rtl818x_iowrite8(priv, &priv->map->CMD, reg);

	priv->rf->stop(dev);

	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
	reg = rtl818x_ioread8(priv, &priv->map->CONFIG4);
	rtl818x_iowrite8(priv, &priv->map->CONFIG4, reg | RTL818X_CONFIG4_VCOOFF);
	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_NORMAL);

	free_irq(priv->irq, dev);

	rtl8186_wlan_free_rx_ring(dev);
	for (i = 0; i < (dev->queues + 1); i++)
		rtl8186_wlan_free_tx_ring(dev, i);
}

static u64 rtl8186_wlan_get_tsf(struct ieee80211_hw *dev,
			   struct ieee80211_vif *vif)
{
	struct rtl8186_wlan_priv *priv = dev->priv;

	return rtl818x_ioread32(priv, &priv->map->TSFT[0]) |
	       (u64)(rtl818x_ioread32(priv, &priv->map->TSFT[1])) << 32;
}

static void rtl8186_wlan_beacon_work(struct work_struct *work)
{
	struct rtl8186_wlan_vif *vif_priv =
		container_of(work, struct rtl8186_wlan_vif, beacon_work.work);
	struct ieee80211_vif *vif =
		container_of((void *)vif_priv, struct ieee80211_vif, drv_priv);
	struct ieee80211_hw *dev = vif_priv->dev;
	struct ieee80211_mgmt *mgmt;
	struct sk_buff *skb;

	/* don't overflow the tx ring */
	if (ieee80211_queue_stopped(dev, 0)) // TODO this is total garbage
		goto resched;

	/* grab a fresh beacon */
	skb = ieee80211_beacon_get(dev, vif);
	if (!skb)
		goto resched;

	/*
	 * update beacon timestamp w/ TSF value
	 * TODO: make hardware update beacon timestamp
	 */
	mgmt = (struct ieee80211_mgmt *)skb->data;
	mgmt->u.beacon.timestamp = cpu_to_be64(rtl8186_wlan_get_tsf(dev, vif));

	/* TODO: use actual beacon queue */
	skb_set_queue_mapping(skb, RTL8186_WLAN_DRIVER_TXB_RING);

	printk(KERN_ERR "sending beacon to tx\n");
	rtl8186_wlan_tx(dev, NULL, skb);

resched:
	/*
	 * schedule next beacon
	 * TODO: use hardware support for beacon timing
	 */
	schedule_delayed_work(&vif_priv->beacon_work,
			usecs_to_jiffies(1024 * vif->bss_conf.beacon_int));
}

static int rtl8186_wlan_add_interface(struct ieee80211_hw *dev,
				 struct ieee80211_vif *vif)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	struct rtl8186_wlan_vif *vif_priv;
	u8 tmp_mac[8];

	/*
	 * We only support one active interface at a time.
	 */
	if (priv->vif)
		return -EBUSY;

	switch (vif->type) {
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_ADHOC:
	case NL80211_IFTYPE_AP:
		break;
	default:
		return -EOPNOTSUPP;
	}

	priv->vif = vif;

	/* Initialize driver private area */
	vif_priv = (struct rtl8186_wlan_vif *)&vif->drv_priv;
	vif_priv->dev = dev;
	INIT_DELAYED_WORK(&vif_priv->beacon_work, rtl8186_wlan_beacon_work);
	vif_priv->enable_beacon = false;

	memset(tmp_mac, 0, sizeof(tmp_mac));
	memcpy(tmp_mac, vif->addr, 6);
	printhex("setting mac addr", tmp_mac, 8);

	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
	rtl818x_iowrite32(priv, (__le32 __iomem *)&priv->map->MAC[0],
			  le32_to_cpu(*(__le32 *)tmp_mac));
	rtl818x_iowrite16(priv, (__le16 __iomem *)&priv->map->MAC[4],
			  le16_to_cpu(*(__le16 *)(tmp_mac + 4)));
	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_NORMAL);

//	msleep(500);
//	memset(tmp_mac, 0, sizeof(tmp_mac));
//	tmp_mac[0] = rtl818x_ioread8(priv, &priv->map->MAC[0]);
//	tmp_mac[1] = rtl818x_ioread8(priv, &priv->map->MAC[1]);
//	tmp_mac[2] = rtl818x_ioread8(priv, &priv->map->MAC[2]);
//	tmp_mac[3] = rtl818x_ioread8(priv, &priv->map->MAC[3]);
//	tmp_mac[4] = rtl818x_ioread8(priv, &priv->map->MAC[4]);
//	tmp_mac[5] = rtl818x_ioread8(priv, &priv->map->MAC[5]);
//	tmp_mac[6] = rtl818x_ioread8(priv, &priv->map->reserved_0[0]);
//	tmp_mac[7] = rtl818x_ioread8(priv, &priv->map->reserved_0[1]);
//
//	printhex("read mac back   ", tmp_mac, 8);

	return 0;
}

static void rtl8186_wlan_remove_interface(struct ieee80211_hw *dev,
				     struct ieee80211_vif *vif)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	// rtl8186_wlan_dump_csr(priv, "remove_interface");
	priv->vif = NULL;
}

static int rtl8186_wlan_config(struct ieee80211_hw *dev, u32 changed)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	struct ieee80211_conf *conf = &dev->conf;

	if (changed & IEEE80211_CONF_CHANGE_CHANNEL) {
		// printk(KERN_ERR "Changed channel!!!!!!\n");
		priv->rf->set_chan(dev, conf);
	}

	return 0;
}

static int rtl8186_wlan_conf_tx(struct ieee80211_hw *dev,
			    struct ieee80211_vif *vif, u16 queue,
			    const struct ieee80211_tx_queue_params *params)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	u8 cw_min, cw_max;

	cw_min = fls(params->cw_min);
	cw_max = fls(params->cw_max);

	rtl818x_iowrite8(priv, &priv->map->CW_VAL,
			 (cw_max << 4) | cw_min);
	return 0;
}

static void rtl8186_wlan_conf_erp(struct ieee80211_hw *dev,
			    struct ieee80211_bss_conf *info)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	u8 sifs, difs;
	int eifs;
	u8 hw_eifs;

	/* I _hope_ this means 10uS for the HW.
	 * In reference code it is 0x22 for
	 * both rtl8187L and rtl8187SE
	 */
	sifs = 0x22;

	if (info->use_short_slot)
		priv->slot_time = 9;
	else
		priv->slot_time = 20;

	/* 10 is SIFS time in uS */
	difs = 10 + 2 * priv->slot_time;
	eifs = 10 + difs + priv->ack_time;

	/* HW should use 4uS units for EIFS (I'm sure for rtl8185)*/
	hw_eifs = DIV_ROUND_UP(eifs, 4);


	rtl818x_iowrite8(priv, &priv->map->SLOT, priv->slot_time);
	rtl818x_iowrite8(priv, &priv->map->SIFS, sifs);
	rtl818x_iowrite8(priv, &priv->map->DIFS, difs);

	/* from reference code. set ack timeout reg = eifs reg */
	rtl818x_iowrite8(priv, &priv->map->CARRIER_SENSE_COUNTER, hw_eifs);

	/* rtl8187/rtl8185 HW bug. After EIFS is elapsed,
		* the HW still wait for DIFS.
		* HW uses 4uS units for EIFS.
		*/
	hw_eifs = DIV_ROUND_UP(eifs - difs, 4);
	hw_eifs = 0x5b;

	rtl818x_iowrite8(priv, &priv->map->EIFS, hw_eifs);
}

static void rtl8186_wlan_bss_info_changed(struct ieee80211_hw *dev,
				     struct ieee80211_vif *vif,
				     struct ieee80211_bss_conf *info,
				     u32 changed)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	struct rtl8186_wlan_vif *vif_priv;
	u8 reg;
	u8 bssid_tmp[8];
	u8 ssid_tmp[sizeof(info->ssid)];

	vif_priv = (struct rtl8186_wlan_vif *)&vif->drv_priv;

	printk("rtl8186_wlan_bss_info_changed:  changed=%08x\n", changed);

	if (changed & BSS_CHANGED_SSID) {
		memcpy(ssid_tmp, info->ssid, info->ssid_len);
		ssid_tmp[info->ssid_len] = 0;
		printk(KERN_ERR "changed ssid. new is %s\n", ssid_tmp);
	}

	if (vif->type == NL80211_IFTYPE_ADHOC) {
		printk(KERN_ERR "adhoc mode\n");
		reg = RTL818X_MSR_ADHOC;
	} else if (vif->type  == NL80211_IFTYPE_STATION) {
		printk(KERN_ERR "station mode\n");
		reg = RTL818X_MSR_INFRA;
	} else if (vif->type == NL80211_IFTYPE_AP) {
		printk(KERN_ERR "ap mode\n");
		reg = RTL818X_MSR_MASTER;
	} else {
		reg = RTL818X_MSR_NO_LINK;
	}
	rtl818x_iowrite8(priv, &priv->map->MSR, reg);
	msleep(50);

	if (changed & BSS_CHANGED_BSSID) {
		memset(bssid_tmp, 0, sizeof(bssid_tmp));
		memcpy(bssid_tmp, info->bssid, 6);
		// memcpy(bssid_tmp, vif->addr, 6);

		printhex("setting bssid  ", bssid_tmp, 6);
		rtl818x_iowrite16(priv, (__le16 __iomem *)&priv->map->BSSID[0],
				  le16_to_cpu(*(__le16 *)bssid_tmp));
		rtl818x_iowrite32(priv, (__le32 __iomem *)&priv->map->BSSID[2],
				  le32_to_cpu(*(__le32 *)(bssid_tmp + 2)));
		msleep(50);
		// memset(bssid_tmp, 0, sizeof(bssid_tmp));
		// bssid_tmp[0] = rtl818x_ioread8(priv, &priv->map->BSSID[0]);
		// bssid_tmp[1] = rtl818x_ioread8(priv, &priv->map->BSSID[1]);
		// bssid_tmp[2] = rtl818x_ioread8(priv, &priv->map->BSSID[2]);
		// bssid_tmp[3] = rtl818x_ioread8(priv, &priv->map->BSSID[3]);
		// bssid_tmp[4] = rtl818x_ioread8(priv, &priv->map->BSSID[4]);
		// bssid_tmp[5] = rtl818x_ioread8(priv, &priv->map->BSSID[5]);
		// printhex("read back bssid", bssid_tmp, 8);
	}

	if (changed & BSS_CHANGED_BASIC_RATES) {
		printk(KERN_ERR "basic rates change: new is %08x", info->basic_rates);
		rtl8186_wlan_conf_basic_rates(dev, info->basic_rates);
	}

	if (changed & (BSS_CHANGED_ERP_SLOT | BSS_CHANGED_ERP_PREAMBLE)) {

		/* when preamble changes, acktime duration changes, and erp must
		 * be recalculated. ACK time is calculated at lowest rate.
		 * Since mac80211 include SIFS time we remove it (-10)
		 */
		priv->ack_time =
			be16_to_cpu(ieee80211_generic_frame_duration(dev,
					priv->vif,
					NL80211_BAND_2GHZ, 10,
					&priv->rates[0])) - 10;

		rtl8186_wlan_conf_erp(dev, info);
	}

	if (changed & BSS_CHANGED_BEACON_ENABLED) {
		vif_priv->enable_beacon = info->enable_beacon;
		printk(KERN_ERR "requested beacon int: %d\n", vif->bss_conf.beacon_int);
		rtl818x_iowrite16(priv, &priv->map->BEACON_INTERVAL, vif->bss_conf.beacon_int);
		rtl818x_iowrite16(priv, &priv->map->BEACON_INTERVAL_TIME, vif->bss_conf.beacon_int);
	}

	if (changed & (BSS_CHANGED_BEACON_ENABLED | BSS_CHANGED_BEACON)) {
		// cancel_delayed_work_sync(&vif_priv->beacon_work);
		// if (vif_priv->enable_beacon)
			// schedule_work(&vif_priv->beacon_work.work);
	}
}

static u64 rtl8186_wlan_prepare_multicast(struct ieee80211_hw *dev,
				     struct netdev_hw_addr_list *mc_list)
{
	return netdev_hw_addr_list_count(mc_list);
}

static void rtl8186_wlan_configure_filter(struct ieee80211_hw *dev,
				     unsigned int changed_flags,
				     unsigned int *total_flags,
				     u64 multicast)
{
	struct rtl8186_wlan_priv *priv = dev->priv;

	if (changed_flags & FIF_FCSFAIL)
		priv->rx_conf ^= RTL818X_RX_CONF_FCS;
	if (changed_flags & FIF_CONTROL)
		priv->rx_conf ^= RTL818X_RX_CONF_CTRL;
	if (changed_flags & FIF_OTHER_BSS)
		priv->rx_conf ^= RTL818X_RX_CONF_MONITOR;
	if (*total_flags & FIF_ALLMULTI || multicast > 0)
		priv->rx_conf |= RTL818X_RX_CONF_MULTICAST;
	else
		priv->rx_conf &= ~RTL818X_RX_CONF_MULTICAST;

	*total_flags = 0;

	if (priv->rx_conf & RTL818X_RX_CONF_FCS)
		*total_flags |= FIF_FCSFAIL;
	if (priv->rx_conf & RTL818X_RX_CONF_CTRL)
		*total_flags |= FIF_CONTROL;
	if (priv->rx_conf & RTL818X_RX_CONF_MONITOR)
		*total_flags |= FIF_OTHER_BSS;
	if (priv->rx_conf & RTL818X_RX_CONF_MULTICAST)
		*total_flags |= FIF_ALLMULTI;

	rtl818x_iowrite32(priv, &priv->map->RX_CONF, priv->rx_conf);
}

static const struct ieee80211_ops rtl8186_wlan_ops = {
	.tx			= rtl8186_wlan_tx,
	.start			= rtl8186_wlan_start,
	.stop			= rtl8186_wlan_stop,
	.add_interface		= rtl8186_wlan_add_interface,
	.remove_interface	= rtl8186_wlan_remove_interface,
	.config			= rtl8186_wlan_config,
	.bss_info_changed	= rtl8186_wlan_bss_info_changed,
	.conf_tx		= rtl8186_wlan_conf_tx,
	.prepare_multicast	= rtl8186_wlan_prepare_multicast,
	.configure_filter	= rtl8186_wlan_configure_filter,
	.get_tsf		= rtl8186_wlan_get_tsf,
};

static void inline dump_register_u8(const char *name, u8 __iomem *addr) {
	u8 val = rtl818x_ioread8(NULL, addr);
	printk(KERN_ERR "rtl8186: map->%s \t\t= 0x%02x\n", name, val);
}


static void inline dump_register_u16(const char *name, u16 __iomem *addr) {
	u16 val = rtl818x_ioread16(NULL, addr);
	printk(KERN_ERR "rtl8186: map->%s \t\t= 0x%04x\n", name, val);
}


static void inline dump_register_u32(const char *name, u32 __iomem *addr) {
	u32 val = rtl818x_ioread32(NULL, addr);
	printk(KERN_ERR "rtl8186: map->%s \t\t= 0x%08x\n", name, val);
}

static void inline dump_register_blob(const char *name, void *addr, unsigned int len) {
	printk(KERN_ERR "rtl8186: map->%s \t\t= ", name);
	printhex_cont_nomsg((u8 *)addr, len);
}


__attribute__((unused))
static void rtl8186_wlan_dump_csr(struct rtl8186_wlan_priv *priv, const char *state_msg) {
	struct rtl818x_csr *map = priv->map;
	printk(KERN_ERR "---------- rtl8186 register dump ----------\n");
	printk(KERN_ERR "---- STATE: %s\n", state_msg);
	dump_register_blob("MAC", &map->MAC, 8);
	dump_register_blob("MAR", &map->MAR, 8);
	dump_register_blob("TSFTR", &map->TSFT, 8);
	dump_register_u32("TLPDA", &map->TLPDA);
	dump_register_u32("TNPDA", &map->TNPDA);
	dump_register_u32("THPDA", &map->THPDA);
	dump_register_u16("BRSR", &map->BRSR);
	dump_register_blob("BSSID", &map->BSSID, 8);
	dump_register_u8("RR", &map->RESP_RATE);
	dump_register_u8("EIFS", &map->EIFS);
	dump_register_u8("CR", &map->CMD);
	dump_register_u16("IMR", &map->INT_MASK);
	dump_register_u16("ISR", &map->INT_STATUS);
	dump_register_u32("TCR", &map->TX_CONF);
	dump_register_u32("RCR", &map->RX_CONF);
	dump_register_u32("TINT", &map->INT_TIMEOUT);
	dump_register_u32("TBDA", &map->TBDA);
	dump_register_u8("CR 2 (EEPROM)", &map->EEPROM_CMD);
	dump_register_u8("CONFIG0", &map->CONFIG0);
	dump_register_u8("CONFIG1", &map->CONFIG1);
	dump_register_u8("CONFIG2", &map->CONFIG2);
	dump_register_u32("ANAPARM", &map->ANAPARAM);
	dump_register_u8("MSR", &map->MSR);
	dump_register_u8("CONFIG3", &map->CONFIG3);
	dump_register_u8("CONFIG4", &map->CONFIG4);
	dump_register_u8("TESTR", &map->TESTR);
	dump_register_u16("BCNITV", &map->BEACON_INTERVAL);
	dump_register_u16("ATIMWND", &map->ATIM_WND);
	dump_register_u16("BINTRITV", &map->BEACON_INTERVAL_TIME);
	dump_register_u16("ATIMTR_INTERVAL", &map->ATIMTR_INTERVAL);
	dump_register_u8("PHYADDR", &map->PHYADDR);
	dump_register_u8("PHYDATAR", &map->PHYDATAR);
	dump_register_u8("TX_ANTENNA", &map->TX_ANTENNA);
	dump_register_u16("WPA_CONFIG", (u16*)&map->WPA_CONF);
	dump_register_u16("AESMASK", (u16*)&map->reserved_15);
	dump_register_u8("CONFIG5", &map->CONFIG5);

	printk(KERN_ERR "-------------------------------------------\n");
}

static int rtl8186_wlan_probe(struct platform_device *pdev)
{
	struct ieee80211_hw *dev;
	struct rtl8186_wlan_priv *priv;
	struct resource *res;
	int irq;
	int err;
	const char *chip_name;
	u32 reg;

	dev = ieee80211_alloc_hw(sizeof(*priv), &rtl8186_wlan_ops);
	if (!dev) {
		printk(KERN_ERR "%s (rtl8186-wlan): ieee80211 alloc failed\n",
                      dev_name(&pdev->dev));
		err = -ENOMEM;
		goto err_free_reg;
	}

	priv = dev->priv;
	priv->plat_dev = pdev;
	priv->is_mmio = true;

	dev->max_rates = 1;
	SET_IEEE80211_DEV(dev, &pdev->dev);
	platform_set_drvdata(pdev, dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		err = -EINVAL;
		goto err_free_dev;
	}

	if (priv->is_mmio) {
		irq = platform_get_irq(pdev, 0);
		if (irq < 0) {
			err = irq;
			goto err_free_dev;
		}
	} else {
		irq = priv->pdev->irq;
	}

	priv->irq = irq;

	priv->map_pio = false;
	priv->map = (struct rtl818x_csr *) (res->start);

	// rtl8186_wlan_dump_csr(priv, "uninitialized");

	BUILD_BUG_ON(sizeof(priv->channels) != sizeof(rtl818x_channels));
	BUILD_BUG_ON(sizeof(priv->rates) != sizeof(rtl818x_rates));

	memcpy(priv->channels, rtl818x_channels, sizeof(rtl818x_channels));
	memcpy(priv->rates, rtl818x_rates, sizeof(rtl818x_rates));

	priv->band.band = NL80211_BAND_2GHZ;
	priv->band.channels = priv->channels;
	priv->band.n_channels = ARRAY_SIZE(rtl818x_channels);
	priv->band.bitrates = priv->rates;
	priv->band.n_bitrates = ARRAY_SIZE(rtl818x_rates);
	printk(KERN_ERR "num of bitrates: %d\n", priv->band.n_bitrates);
	dev->wiphy->bands[NL80211_BAND_2GHZ] = &priv->band;

	ieee80211_hw_set(dev, HOST_BROADCAST_PS_BUFFERING);
	ieee80211_hw_set(dev, RX_INCLUDES_FCS);

	dev->vif_data_size = sizeof(struct rtl8186_wlan_vif);
	dev->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION) |
					BIT(NL80211_IFTYPE_ADHOC) | BIT(NL80211_IFTYPE_AP);
	dev->max_signal = 65;

	reg = rtl818x_ioread32(priv, &priv->map->TX_CONF);
	reg &= RTL818X_TX_CONF_HWVER_MASK;
	switch (reg) {

	case RTL818X_TX_CONF_R8185_ABC:
		chip_name = "RTL8185";
		priv->chip_family = RTL818X_CHIP_FAMILY_RTL8185;
		break;

	default:
		printk(KERN_ERR "%s (rtl8186-wlan): Unknown chip! (0x%x)\n",
		       dev_name(rtl8186_wlan_get_dev(priv)), reg >> 25);
		err = -ENODEV;
		goto err_iounmap;
	}

	/* we declare to MAC80211 all the queues except for beacon queue
	 * that will be eventually handled by DRV.
	 * TX rings are arranged in such a way that lower is the IDX,
	 * higher is the priority, in order to achieve direct mapping
	 * with mac80211, however the beacon queue is an exception and it
	 * is mapped on the highst tx ring IDX.
	 */

	dev->queues = RTL8186_NR_TX_QUEUES - 1;

	priv->band.n_bitrates = ARRAY_SIZE(rtl818x_rates);

	ieee80211_hw_set(dev, SIGNAL_DBM);

	priv->rf_type = 9;
	priv->rf = rtl8186_wlan_detect_rf(dev);

	// HW_WLAN_ADDR=001f1f040403
	// eth_random_addr(priv->mac_addr);
	memcpy(priv->mac_addr, "\x00\x1f\x1f\x04\x04\x03", 6);
	// memcpy(priv->mac_addr, "\x02\x02\x02\x02\x00\x00", 6);
	if (!is_valid_ether_addr(priv->mac_addr)) {
		printk(KERN_ERR "invalid mac addr\n");
		err = -EINVAL;
		goto err_iounmap;
	}
	SET_IEEE80211_PERM_ADDR(dev, priv->mac_addr);

	spin_lock_init(&priv->lock);

	err = ieee80211_register_hw(dev);
	if (err) {
		printk(KERN_ERR "%s (rtl8186-wlan): Cannot register device\n",
		       dev_name(rtl8186_wlan_get_dev(priv)));
		goto err_iounmap;
	}

	wiphy_info(dev->wiphy, "hwaddr %pm, %s + %s\n",
		   priv->mac_addr, chip_name, priv->rf->name);

	return 0;

 err_iounmap:
//	pci_iounmap(pdev, priv->map);

 err_free_dev:
	ieee80211_free_hw(dev);

 err_free_reg:
//	pci_release_regions(pdev);

// err_disable_dev:
//	pci_disable_device(pdev);
	return err;
}

static int rtl8186_wlan_remove(struct platform_device *pdev)
{
	struct ieee80211_hw *dev = platform_get_drvdata(pdev);
	struct rtl8186_wlan_priv *priv;

	if (!dev)
		return -EINVAL;

	ieee80211_unregister_hw(dev);

	priv = dev->priv;

	//pci_iounmap(pdev, priv->map);
	//pci_release_regions(pdev);
	//pci_disable_device(pdev);
	ieee80211_free_hw(dev);
	return 0;
}


static const struct of_device_id of_rtl8186_wlan_match[] = {
	{ .compatible	= "realtek,rtl8186-wlan" },
	{ },
};
MODULE_DEVICE_TABLE(of, of_rtl8186_wlan_match);


static struct platform_driver rtl8186_wlan_driver = {
	.driver = {
		.name = "rtl8186-wlan",
		.of_match_table = of_rtl8186_wlan_match,
	},
	.probe = rtl8186_wlan_probe,
	.remove = rtl8186_wlan_remove,
};

module_platform_driver(rtl8186_wlan_driver);