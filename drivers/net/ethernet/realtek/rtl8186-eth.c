#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mii.h>

#define RTL8186_DEBUG_MDIO 0
#define RTL8186_FAST_BRIDGE_HACK 0
#define RTL8186_PROFILE_FORWARDING 0
#define RTL8186_PROFILE_RX 0

#define DRV_NAME "rtl8186-eth"
#define DRV_VERSION "1.0"

static const char version[] =
	DRV_NAME ": Realtek RTL8186 10/100 Ethernet driver v" DRV_VERSION "\n";

#define PFX DRV_NAME ": "
#define RTL8186_RX_RING_SIZE 64 /* TODO must be kept in sync with RingSize */
#define RTL8186_TX_RING_SIZE 64
#define DESC_ALIGN 0x100
#define UNCACHE_MASK 0xa0000000

#define RTL8186_RXRING_BYTES                                                   \
	((sizeof(struct dma_desc) * (RTL8186_RX_RING_SIZE + 1)) + DESC_ALIGN)
#define RTL8186_TXRING_BYTES                                                   \
	((sizeof(struct dma_desc) * (RTL8186_TX_RING_SIZE + 1)) + DESC_ALIGN)

#define NEXT_TX(N) (((N) + 1) & (RTL8186_TX_RING_SIZE - 1))
#define NEXT_RX(N) (((N) + 1) & (RTL8186_RX_RING_SIZE - 1))
#define TX_HQBUFFS_AVAIL(CP)                                                   \
	(((CP)->tx_hqtail <= (CP)->tx_hqhead) ?                                \
		 (CP)->tx_hqtail + (RTL8186_TX_RING_SIZE - 1) -                \
			 (CP)->tx_hqhead :                                     \
		 (CP)->tx_hqtail - (CP)->tx_hqhead - 1)

#define PKT_BUF_SZ 1536 /* Size of each temporary Rx buffer.*/
#define RX_OFFSET 2

#define RTL8186_MIN_MTU 64
#define RTL8186_MAX_MTU 4096

/* The following settings are log_2(bytes)-4:  0 == 16 bytes .. 6==1024, 7==end of packet. */

/* Time in jiffies before concluding the transmitter is hung. */
#define TX_TIMEOUT (6 * HZ)

#define RTL_W32(reg, value) (*(volatile u32 *)(cp->regs + reg)) = (u32)value
#define RTL_W16(reg, value) (*(volatile u16 *)(cp->regs + reg)) = (u16)value
#define RTL_W8(reg, value) (*(volatile u8 *)(cp->regs + reg)) = (u8)value
#define RTL_R32(reg) (*(volatile u32 *)(cp->regs + reg))
#define RTL_R16(reg) (*(volatile u16 *)(cp->regs + reg))
#define RTL_R8(reg) (*(volatile u8 *)(cp->regs + reg))

enum PHY_REGS {
	TXFCE = BIT(7),
	RXFCE = BIT(6),
	SP100 = BIT(3),
	LINK = BIT(2),
	TXPF = BIT(1),
	RXPF = BIT(0),
	FORCE_TX = BIT(5),
};

enum {
	/* NIC register offsets */
	IDR0 = 0, /* Ethernet ID */
	IDR1 = 0x1, /* Ethernet ID */
	IDR2 = 0x2, /* Ethernet ID */
	IDR3 = 0x3, /* Ethernet ID */
	IDR4 = 0x4, /* Ethernet ID */
	IDR5 = 0x5, /* Ethernet ID */
	MAR0 = 0x8, /* Multicast register */
	MAR1 = 0x9,
	MAR2 = 0xa,
	MAR3 = 0xb,
	MAR4 = 0xc,
	MAR5 = 0xd,
	MAR6 = 0xe,
	MAR7 = 0xf,
	TXOKCNT = 0x10,
	RXOKCNT = 0x12,
	TXERR = 0x14,
	RXERRR = 0x16,
	MISSPKT = 0x18,
	FAE = 0x1A,
	TX1COL = 0x1c,
	TXMCOL = 0x1e,
	RXOKPHY = 0x20,
	RXOKBRD = 0x22,
	RXOKMUL = 0x24,
	TXABT = 0x26,
	TXUNDRN = 0x28,
	TRSR = 0x34,
	CMD = 0x3B,
	IMR = 0x3C,
	ISR = 0x3E,
	TCR = 0x40,
	RCR = 0x44,
	MSR = 0x58,
	MIIAR = 0x5C,

	TxFDP1 = 0x1300,
	TxCDO1 = 0x1304,
	TXCPO1 = 0x1308,
	TXPSA1 = 0x130A,
	TXCPA1 = 0x130C,
	LastTxCDO1 = 0x1310,
	TXPAGECNT1 = 0x1312,
	Tx1ScratchDes = 0x1350,
	TxFDP2 = 0x1380,
	TxCDO2 = 0x1384,
	TXCPO2 = 0x1388,
	TXPSA2 = 0x138A,
	TXCPA2 = 0x138C,
	LASTTXCDO2 = 0x1390,
	TXPAGECNT2 = 0x1392,
	Tx2ScratchDes = 0x13A0,
	RxFDP = 0x13F0,
	RxCDO = 0x13F4,
	RxRingSize = 0x13F6,
	RxCPO = 0x13F8,
	RxPSA = 0x13FA,
	RXPLEN = 0x1403,
	//LASTRXCDO		=0x1402,
	RXPFDP = 0x1404,
	RXPAGECNT = 0x1408,
	RXSCRATCHDES = 0x1410,
	EthrntRxCPU_Des_Num = 0x1430,
	EthrntRxCPU_Des_Wrap = 0x1431,
	Rx_Pse_Des_Thres = 0x1432,

	IO_CMD = 0x1434,
	MII_RE = BIT(3),
	MII_TE = BIT(2),
	TX_FNL = BIT(1),
	TX_FNH = BIT(0),

};

enum RTL8186_STATUS_REGS {
	/* Tx and Rx status descriptors */
	DescOwn = (1 << 31), /* Descriptor is owned by NIC */
	RingEnd = (1 << 30), /* End of descriptor ring */
	FirstFrag = (1 << 29), /* First segment of a packet */
	LastFrag = (1 << 28), /* Final segment of a packet */
	TxError = (1 << 23), /* Tx error summary */
	RxError = (1 << 20), /* Rx error summary */
	IPCS = (1 << 18), /* Calculate IP checksum */
	UDPCS = (1 << 17), /* Calculate UDP/IP checksum */
	TCPCS = (1 << 16), /* Calculate TCP/IP checksum */
	TxVlanTag = (1 << 17), /* Add VLAN tag */
	RxVlanTagged = (1 << 16), /* Rx VLAN tag available */
	IPFail = (1 << 15), /* IP checksum failed */
	UDPFail = (1 << 14), /* UDP/IP checksum failed */
	TCPFail = (1 << 13), /* TCP/IP checksum failed */
	NormalTxPoll = (1 << 6), /* One or more normal Tx packets to send */
	PID1 = (1 << 17), /* 2 protocol id bits:  0==non-IP, */
	PID0 = (1 << 16), /* 1==UDP/IP, 2==TCP/IP, 3==IP */
	RxProtoTCP = 1,
	RxProtoUDP = 2,
	RxProtoIP = 3,
	TxFIFOUnder = (1 << 25), /* Tx FIFO underrun */
	TxOWC = (1 << 22), /* Tx Out-of-window collision */
	TxLinkFail = (1 << 21), /* Link failed during Tx of packet */
	TxMaxCol = (1 << 20), /* Tx aborted due to excessive collisions */
	TxColCntShift = 16, /* Shift, to get 4-bit Tx collision cnt */
	TxColCntMask = 0x01 | 0x02 | 0x04 | 0x08, /* 4-bit collision count */
	RxErrFrame = (1 << 27), /* Rx frame alignment error */
	RxMcast = (1 << 26), /* Rx multicast packet rcv'd */
	RxErrCRC = (1 << 18), /* Rx CRC error */
	RxErrRunt = (1 << 19), /* Rx error, packet < 64 bytes */
	RWT = (1 << 21), /* Rx  */
	E8023 = (1 << 22), /* Receive Ethernet 802.3 packet  */
	TxCRC = (1 << 23),
	PPPOE = (1 << 23),
	RxVlanOn = (1 << 2), /* Rx VLAN de-tagging enable */
	RxChkSum = (1 << 1), /* Rx checksum offload enable */

};

#define RXRINGSIZE_16 0x0
#define RXRINGSIZE_32 0x1
#define RXRINGSIZE_64 0x2

enum RTL8186_THRESHOLD_REGS {
	THVAL = 2,
	RINGSIZE = RXRINGSIZE_64,

	LOOPBACK = (0x3 << 8),
	AcceptErr = 0x20, /* Accept packets with CRC errors */
	AcceptRunt = 0x10, /* Accept runt (<64 bytes) packets */
	AcceptBroadcast = 0x08, /* Accept broadcast packets */
	AcceptMulticast = 0x04, /* Accept multicast packets */
	AcceptMyPhys = 0x02, /* Accept pkts with our MAC as dest */
	AcceptAllPhys = 0x01, /* Accept all pkts w/ physical dest */
	AcceptAll = AcceptBroadcast | AcceptMulticast | AcceptMyPhys |
		    AcceptAllPhys | AcceptErr | AcceptRunt,
	AcceptNoBroad = AcceptMulticast | AcceptMyPhys | AcceptAllPhys |
			AcceptErr | AcceptRunt,
	AcceptNoMulti = AcceptMyPhys | AcceptAllPhys | AcceptErr | AcceptRunt,
	NoErrAccept = AcceptBroadcast | AcceptMulticast | AcceptMyPhys |
		      AcceptAllPhys,

};

enum RTL8186_ISR_REGS {
	SW_INT = BIT(10),
	TX_EMPTY = BIT(9),
	LINK_CHG = BIT(8),
	TX_ERR = BIT(7),
	TX_OK = BIT(6),
	RX_EMPTY = BIT(5),
	RX_FIFOOVR = BIT(4),
	RX_ERR = BIT(2),
	RX_OK = BIT(0),
};

enum RTL8186_IOCMD_REG {
	RX_MIT = 1,
	RX_TIMER = 1,
	RX_FIFO = 2,
	TX_FIFO = 1,
	TX_MIT = 1,
	TX_POLL = 1 << 0,
	CMD_CONFIG = 0x3c | RX_MIT << 8 | RX_FIFO << 11 | RX_TIMER << 13 |
		     TX_MIT << 16 | TX_FIFO << 19,
};

#define RX_INTERRUPTS (RX_OK | RX_ERR | RX_EMPTY | RX_FIFOOVR)

// We can't have RX_EMPTY interrupt with NAPI. napi introduces latencies
// that sometimes make the RX ring full of not-handled-yet packets.
static const u32 rtl8186_intr_mask = LINK_CHG | RX_OK | RX_ERR | // | RX_FIFOOVR
				     TX_OK | TX_ERR; // | TX_EMPTY | SW_INT;

typedef struct dma_desc {
	u32 opts1; // status
	u32 addr; // buffer address
	u32 opts2; // vlan stuff
	u32 opts3; // partial checksum
} DMA_DESC;

// TODO just hold skb array
struct ring_info {
	struct sk_buff *skb;
};

struct re_private {
	unsigned tx_hqhead;
	unsigned tx_hqtail;
	unsigned tx_lqhead;
	unsigned tx_lqtail;
	unsigned rx_tail;
	void *regs;
	struct net_device *dev;
	spinlock_t lock;

	struct napi_struct rx_napi;
	struct napi_struct tx_napi;

	DMA_DESC *rx_ring;
	DMA_DESC *tx_hqring;
	DMA_DESC *tx_lqring;
	struct ring_info tx_skb[RTL8186_TX_RING_SIZE];
	struct ring_info rx_skb[RTL8186_RX_RING_SIZE];
	unsigned rx_buf_sz;
	dma_addr_t ring_dma;
	u32 msg_enable;
	struct net_device_stats net_stats;
	struct sk_buff *frag_skb;
	unsigned dropping_frag : 1;
	char *rxdesc_buf;
	char *txdesc_buf;
	struct mii_if_info mii_if;
	unsigned int phy_type;
	u16 intr_mask;
};

static int rtl8186_tx(struct re_private *cp, int budget);
static void rtl8186_clean_rings(struct re_private *cp);
static void rtl8186_tx_timeout(struct net_device *dev, unsigned int txqueue);
static void rtl8186_change_mac_address(struct net_device *dev, u8 *mac);
static int rtl8186_set_mac_address(struct net_device *dev, void *p);
static int rtl8186_start_xmit(struct sk_buff *skb, struct net_device *dev);
static int rtl8186_start_xmit_internal(struct sk_buff *skb,
				       struct re_private *cp,
				       struct net_device *dev);

static int mdio_read(struct net_device *dev, int phy_id, int location);
static void mdio_write(struct net_device *dev, int phy_id, int location,
		       int value);

struct net_device *rtl8186_hack_devs[2];

static inline u32 rtl8186_time(void)
{
	return ~(readl((void *)(unsigned long)0xBD010078)); // T2CNT
}

static inline u32 clockticks_to_nsec(u32 ticks)
{
	return ticks * 45;
}

static inline u32 clockticks_to_usec(u32 ticks)
{
	return (ticks * 45) / 1000;
}

static void stopwatch_init(struct rtl8186_stopwatch *sw)
{
	sw->is_started = 0;
	sw->start_time = 0;
	sw->total = 0;
}

static void stopwatch_start(struct rtl8186_stopwatch *sw)
{
	if (sw->is_started) {
		pr_warn("Tried to start already started stopwatch\n");
		dump_stack();
		return;
	}
	sw->is_started = 1;
	sw->start_time = rtl8186_time();
}

static void stopwatch_stop(struct rtl8186_stopwatch *sw)
{
	if (!sw->is_started) {
		pr_warn("Tried to stop already stopped stopwatch\n");
		dump_stack();
		return;
	}
	sw->total += rtl8186_time() - sw->start_time;
	sw->is_started = 0;
}

static u32 stopwatch_elapsed(struct rtl8186_stopwatch *sw)
{
	if (sw->is_started) {
		pr_warn("Stopwatch is running!\n");
		dump_stack();
		return 0;
	}
	return sw->total;
}

static inline struct re_private *rtl8186_priv(struct net_device *dev)
{
	return netdev_priv(dev);
}

static void rtl8186_set_intr_mask(struct re_private *cp, u16 intr_mask)
{
	cp->intr_mask = intr_mask;
	RTL_W16(IMR, intr_mask);
}

static inline void rtl8186_set_rxbufsize(struct re_private *cp)
{
	unsigned int mtu = cp->dev->mtu;

	if (mtu > ETH_DATA_LEN)
		/* MTU + ethernet header + FCS + optional VLAN tag */
		cp->rx_buf_sz = mtu + ETH_HLEN + 8;
	else
		cp->rx_buf_sz = PKT_BUF_SZ;
}

static inline void rtl8186_rx_skb(struct re_private *cp, struct sk_buff *skb,
				  DMA_DESC *desc)
{
#if RTL8186_FAST_BRIDGE_HACK
	// TODO achieve zero kalloc fails here for perfect performance
	struct net_device *dev = cp->dev;
	struct sk_buff *send_skb;
	void *backup_data = skb->data;
	void *backup_head = skb->head;
	void *backup_tail = skb->tail;
	unsigned long backup_len = skb->len;

	cp->net_stats.rx_packets++;
	cp->net_stats.rx_bytes += skb->len;

	skb->protocol = eth_type_trans(skb, cp->dev);

	send_skb = skb;

	if (dev == rtl8186_hack_devs[0]) {
		send_skb->dev = rtl8186_hack_devs[1];
		// rtl8186_start_xmit(skb, rtl8186_hack_devs[1]);
	} else {
		send_skb->dev = rtl8186_hack_devs[0];
		// rtl8186_start_xmit(skb, rtl8186_hack_devs[0]);
	}

	// skb_reset_network_header(skb);
	// skb_push(send_skb, ETH_HLEN);

	send_skb->data = backup_data;
	send_skb->head = backup_head;
	send_skb->tail = backup_tail;
	send_skb->len = backup_len;

	if (rtl8186_start_xmit_internal(send_skb, rtl8186_priv(send_skb->dev),
					send_skb->dev) == NETDEV_TX_BUSY) {
		dev_kfree_skb_irq(send_skb);
	}

	// dev_queue_xmit(send_skb);
#else
	cp->net_stats.rx_packets++;
	cp->net_stats.rx_bytes += skb->len;
	skb->protocol = eth_type_trans(skb, cp->dev);
	// netif_rx(skb);
	netif_receive_skb(skb);
#endif
}

static void rtl8186_rx_err_acct(struct re_private *cp, unsigned rx_tail,
				u32 status, u32 len)
{
	cp->net_stats.rx_errors++;
	if (status & RxErrFrame)
		cp->net_stats.rx_frame_errors++;
	if (status & RxErrCRC)
		cp->net_stats.rx_crc_errors++;
	if (status & RxErrRunt)
		cp->net_stats.rx_length_errors++;
	if ((status & (FirstFrag | LastFrag)) != (FirstFrag | LastFrag))
		cp->dev->stats.rx_length_errors++;
}

// static void rtl8186_rx_frag (struct re_private *cp, unsigned rx_tail,
// 			struct sk_buff *skb, u32 status, u32 len)
// {
// 	struct sk_buff *copy_skb, *frag_skb = cp->frag_skb;
// 	unsigned orig_len = frag_skb ? frag_skb->len : 0;
// 	unsigned target_len = orig_len + len;
// 	unsigned first_frag = status & FirstFrag;
// 	unsigned last_frag = status & LastFrag;

// 	printk(KERN_ERR "rtl8186_rx_frag\n");
// 	if (netif_msg_rx_status (cp))
// 		printk (KERN_DEBUG "%s: rx %s%sfrag, slot %d status 0x%x len %d\n",
// 			cp->dev->name,
// 			cp->dropping_frag ? "dropping " : "",
// 			first_frag ? "first " :
// 			last_frag ? "last " : "",
// 			rx_tail, status, len);

// 	if (!frag_skb && !first_frag)
// 		cp->dropping_frag = 1;
// 	if (cp->dropping_frag)
// 		goto drop_frag;

// 	copy_skb = netdev_alloc_skb(cp->dev, target_len + RX_OFFSET);
// 	if (!copy_skb) {
// 		printk(KERN_WARNING "%s: rx slot %d alloc failed\n",
// 		       cp->dev->name, rx_tail);

// 		cp->dropping_frag = 1;
// drop_frag:
// 		if (frag_skb) {
// 			dev_kfree_skb_irq(frag_skb);
// 			cp->frag_skb = NULL;
// 		}
// 		if (last_frag) {
// 			cp->net_stats.rx_dropped++;
// 			cp->dropping_frag = 0;
// 		}
// 		return;
// 	}

// 	copy_skb->dev = cp->dev;
// 	skb_reserve(copy_skb, RX_OFFSET);
// 	skb_put(copy_skb, target_len);
// 	if (frag_skb) {
// 		memcpy(copy_skb->data, frag_skb->data, orig_len);
// 		dev_kfree_skb_irq(frag_skb);
// 	}
// 	memcpy(copy_skb->data + orig_len, skb->data, len);

// 	copy_skb->ip_summed = CHECKSUM_NONE;

// 	if (last_frag) {
// 		if (status & (RxError)) {
// 			rtl8186_rx_err_acct(cp, rx_tail, status, len);
// 			dev_kfree_skb_irq(copy_skb);
// 		} else
// 			rtl8186_rx_skb(cp, copy_skb, &cp->rx_ring[rx_tail]);
// 		cp->frag_skb = NULL;
// 	} else {
// 		cp->frag_skb = copy_skb;
// 	}
// }

static inline void rtl8186_rx_checksum(struct sk_buff *skb, u32 status)
{
	unsigned int protocol = (status >> 16) & 0x3;

	skb_checksum_none_assert(skb);

	if ((protocol == RxProtoTCP && !(status & TCPFail)) ||
	    (protocol == RxProtoUDP && !(status & UDPFail)) ||
	    (protocol == RxProtoIP && !(status & IPFail))) {
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	}
}

static struct sk_buff *rtl8186_alloc_rx_skb(struct re_private *cp,
					    unsigned int len)
{
	struct sk_buff *skb = napi_alloc_skb(&cp->rx_napi, len + RX_OFFSET * 2);
	if (!skb)
		return NULL;

	/* Make sure skb->data is aligned */
	if (((u32)skb->data) & 0x3)
		skb_reserve(skb, RX_OFFSET);

	return skb;
}

static int rtl8186_rx_poll(struct napi_struct *napi, int budget)
{
	struct re_private *cp = container_of(napi, struct re_private, rx_napi);
	unsigned int rx_tail;
	unsigned long flags;
	int rx = 0;
	struct rtl8186_stopwatch rx_sw;

	spin_lock_irqsave(&cp->lock, flags);

	rx_tail = cp->rx_tail;

	while (rx < budget) {
		u32 status, len;
		dma_addr_t mapping;
		struct sk_buff *skb, *new_skb;
		DMA_DESC *desc;

		desc = &cp->rx_ring[rx_tail];
		status = desc->opts1;
		if (status & DescOwn)
			break;

		skb = cp->rx_skb[rx_tail].skb;
		stopwatch_init(&skb->alloc_sw);
		stopwatch_init(&skb->driver_sw);
		stopwatch_init(&rx_sw);

		stopwatch_start(&skb->driver_sw);
		len = (status & 0x0fff) -
		      4; // ethernet CRC-4 bytes- are padding after the payload
		/* Try allocating the new SKB fisrt */
		/* If we got here, it means driver has some data for us */
		/* On errors, just recycle the same SKB */
		new_skb = skb;

		if ((status & (FirstFrag | LastFrag)) !=
		    (FirstFrag | LastFrag)) {
			rtl8186_rx_err_acct(cp, rx_tail, status, len);
			printk_ratelimited(
				KERN_WARNING
				"%s: Got fragmented packet from device. Dropping!\n",
				cp->dev->name);
			goto setup_new_skb;
		}
		if (status & (RxError)) {
			rtl8186_rx_err_acct(cp, rx_tail, status, len);
			printk_ratelimited(KERN_WARNING "%s: RX error!\n",
					   cp->dev->name);
			goto setup_new_skb;
		}

		// If we got here, it means we have OK packet that we can handle.
		// now try to alloc the next skb. if it fails, we must drop the OK packet and recycle the SKB...

		stopwatch_start(&skb->alloc_sw);
		new_skb = rtl8186_alloc_rx_skb(cp, cp->rx_buf_sz);
		stopwatch_stop(&skb->alloc_sw);

		if (unlikely(!new_skb)) {
			printk_ratelimited(KERN_WARNING
					   "%s: napi_alloc_skb failed!\n",
					   cp->dev->name);
			cp->net_stats.rx_dropped++;
			new_skb = skb;
			goto setup_new_skb;
		}

		// if (netif_msg_rx_status(cp))
		// 	printk(KERN_DEBUG "%s: rx slot %d status 0x%x len %d\n",
		// 	       cp->dev->name, rx_tail, status, len);

		/* Handle checksum offloading for incoming packets. */
		rtl8186_rx_checksum(skb, status);

		/* The NIC never sets BIT(31) in opts2 (whether or not packets has two bytes before data) */
		skb_reserve(skb, RX_OFFSET);
		skb_put(skb, len);

		/* Publish the arrived packet to network stack */
		stopwatch_stop(&skb->driver_sw);

		stopwatch_start(&rx_sw);
		rtl8186_rx_skb(cp, skb, desc);
		stopwatch_stop(&rx_sw);

#if RTL8186_PROFILE_RX
		{
			u32 rx_total = stopwatch_elapsed(&rx_sw);
			pr_info("RX processing took %d usec\n",
				clockticks_to_usec(rx_total));
		}
#endif

	setup_new_skb:
		mapping = (u32)new_skb->data | UNCACHE_MASK;
		cp->rx_skb[rx_tail].skb = new_skb;

		desc->addr = mapping;
		desc->opts2 = 0;
		if (rx_tail == (RTL8186_RX_RING_SIZE - 1))
			desc->opts1 = (DescOwn | RingEnd | cp->rx_buf_sz);
		else
			desc->opts1 = (DescOwn | cp->rx_buf_sz);

		rx_tail = NEXT_RX(rx_tail);
		++rx;
	}

	// if (!rx_work)
	// printk(KERN_WARNING "%s: rx work limit reached\n", cp->dev->name);

	cp->rx_tail = rx_tail;

	if (rx < budget && napi_complete_done(napi, rx)) {
		rtl8186_set_intr_mask(cp, cp->intr_mask | RX_OK);
	}

	spin_unlock_irqrestore(&cp->lock, flags);

	//pr_info("%s: rtl8186_rx_poll  budget=%d, done=%d\n", cp->dev->name, budget, rx);

	return rx;
}

static int rtl8186_tx_poll(struct napi_struct *napi, int budget)
{
	struct re_private *cp = container_of(napi, struct re_private, tx_napi);
	unsigned long flags;
	int completed = 0;

	spin_lock_irqsave(&cp->lock, flags);

	completed = rtl8186_tx(cp, budget);

	if (completed < budget && napi_complete_done(napi, completed)) {
		rtl8186_set_intr_mask(cp, cp->intr_mask | TX_OK);
	}

	spin_unlock_irqrestore(&cp->lock, flags);

	return completed;
}

static irqreturn_t rtl8186_interrupt(int irq, void *dev_instance)
{
	struct net_device *dev = dev_instance;
	struct re_private *cp;
	u16 status;

	if (unlikely(!dev))
		return IRQ_NONE;

	cp = rtl8186_priv(dev);

	spin_lock(&cp->lock);

	status = RTL_R16(ISR);
	RTL_W16(ISR, status);

	if (!status || (status == 0xFFFF))
		goto out_unlock;

	status &= cp->intr_mask;

	// if (netif_msg_intr(cp))
	// 	printk(KERN_DEBUG "%s: intr, status %04x cmd %02x \n",
	// 	        dev->name, status, RTL_R8(CMD));

	if (status & LINK_CHG) {
		pr_info("%s: Link change\n", dev->name);
		mii_check_media(&cp->mii_if, netif_msg_link(cp), false);
	}

	if (status & TX_OK) {
		rtl8186_set_intr_mask(cp, cp->intr_mask & (~TX_OK));
		napi_schedule(&cp->tx_napi);
	}

	if (status & RX_OK) {
		rtl8186_set_intr_mask(cp, cp->intr_mask & (~RX_OK));
		napi_schedule(&cp->rx_napi);
	}

	if (unlikely(status & TX_ERR)) { // TODO TX_EMPTY sometimes?????
		printk_ratelimited("%s: tx error. status=0x%04x\n", dev->name,
				   status);
	}

	if (unlikely(status & RX_ERR)) {
		printk_ratelimited("%s: rx error. status=0x%04x\n", dev->name,
				   status);
	}

out_unlock:
	spin_unlock(&cp->lock);

	return IRQ_HANDLED;
}

static int rtl8186_tx(struct re_private *cp, int budget)
{
	unsigned int bytes_compl = 0, pkts_compl = 0;
	unsigned int tx_head = cp->tx_hqhead;
	unsigned int tx_tail = cp->tx_hqtail;
	struct net_device *dev = cp->dev;

	while (tx_tail != tx_head && pkts_compl < budget) {
		struct sk_buff *skb;
		u32 status;

		status = (cp->tx_hqring[tx_tail].opts1);
		if (status & DescOwn)
			break;
		if (status & TxError) {
			cp->net_stats.tx_errors++;
			if (status & TxFIFOUnder)
				cp->net_stats.tx_fifo_errors++;
		}
		skb = cp->tx_skb[tx_tail].skb;
		cp->tx_skb[tx_tail].skb = NULL;
		// cp->tx_hqring[tx_tail].addr = 0x0;
		// cp->tx_hqring[tx_tail].opts1 = 0x0;

		cp->net_stats.tx_packets++;
		cp->net_stats.tx_bytes += skb->len;

		bytes_compl += skb->len;
		pkts_compl++;
		// if (netif_msg_tx_done(cp))
		// 	printk(KERN_DEBUG "%s: tx done, slot %d\n", cp->dev->name, tx_tail);

		napi_consume_skb(skb, budget);

		tx_tail = NEXT_TX(tx_tail);
	}

	cp->tx_hqtail = tx_tail;

	//pr_info("netdev_completed_queue: pkts=%d, bytes=%d\n", pkts_compl, bytes_compl);
#if !RTL8186_FAST_BRIDGE_HACK
	netdev_completed_queue(dev, pkts_compl, bytes_compl); /* for BQL */
#endif

	if (netif_queue_stopped(dev) && (TX_HQBUFFS_AVAIL(cp) > 0)) {
		netif_wake_queue(dev);
	}

	return pkts_compl;
}

static int rtl8186_start_xmit_internal(struct sk_buff *skb,
				       struct re_private *cp,
				       struct net_device *dev)
{
	unsigned int entry;
	int available;
	DMA_DESC *txd;
	u32 eor;
	u32 total, alloc_total;

	available = TX_HQBUFFS_AVAIL(cp);

	/* This is a hard error, log it. */
	if (available <= 0) {
		netif_stop_queue(dev);
		printk_ratelimited("%s: TX ring is full!\n", dev->name);
		return NETDEV_TX_BUSY;
	}

	entry = cp->tx_hqhead;
	eor = (entry == (RTL8186_TX_RING_SIZE - 1)) ? RingEnd : 0;

	// BUG_ON(skb_shinfo(skb)->nr_frags != 0);  // we don't support SG. only full packets
	txd = &cp->tx_hqring[entry];

	txd->addr = ((u32)skb->data) | UNCACHE_MASK; // TODO dma remap etc...;
	txd->opts2 = 0;
	txd->opts1 = DescOwn | eor | skb->len | FirstFrag | LastFrag | TxCRC;

	// if (netif_msg_tx_queued(cp))
	// 	pr_info("%s: tx queued, slot %d, skblen %d\n",
	// 	       dev->name, entry, skb->len);

	if (available == 1)
		netif_stop_queue(dev);

	if (!netdev_xmit_more() || netif_queue_stopped(dev)) {
		/* The NIC can start transmitting */
		RTL_W32(IO_CMD, CMD_CONFIG | TX_POLL);
	}

	/* This work can be done after NIC starts transmission */

#if !RTL8186_FAST_BRIDGE_HACK
	netdev_sent_queue(dev, skb->len); /* for BQL */
#endif

	if (skb->driver_sw.is_started) {
		stopwatch_stop(&skb->driver_sw);
		total = stopwatch_elapsed(&skb->driver_sw);
		alloc_total = stopwatch_elapsed(&skb->alloc_sw);

#if RTL8186_PROFILE_FORWARDING
		pr_info("SKB processing took %d usec. alloc took %d usec\n",
			clockticks_to_usec(total),
			clockticks_to_usec(alloc_total));
#endif
	}

	cp->tx_skb[entry].skb = skb;
	cp->tx_hqhead = NEXT_TX(entry);

	return NETDEV_TX_OK;
}

static int rtl8186_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct re_private *cp = rtl8186_priv(dev);
	unsigned long flags;
	int ret;

	if (skb->driver_sw.total > 0) {
		stopwatch_start(&skb->driver_sw);
	}

	spin_lock_irqsave(&cp->lock, flags);
	ret = rtl8186_start_xmit_internal(skb, cp, dev);
	spin_unlock_irqrestore(&cp->lock, flags);
	return ret;
}

static struct net_device_stats *rtl8186_get_stats(struct net_device *dev)
{
	struct re_private *cp = rtl8186_priv(dev);
	return &cp->net_stats;
}

/*
 * Disable TX/RX through IO_CMD register
 */
static void rtl8186_stop_hw(struct re_private *cp)
{
	rtl8186_set_intr_mask(cp, 0);
	RTL_W16(ISR, 0xffff);
	RTL_W32(IO_CMD, 0); /* timer rx int 1 packets */
	synchronize_irq(cp->dev->irq);
	udelay(10);
	cp->rx_tail = 0;
	cp->tx_hqhead = cp->tx_hqtail = 0;
	netdev_reset_queue(cp->dev);
}

static void rtl8186_reset_hw(struct re_private *cp)
{
	unsigned work = 1000;

	RTL_W8(CMD, 0x1); /* Reset */
	while (work--) {
		if (!(RTL_R8(CMD) & 0x1))
			return;
		set_current_state(TASK_UNINTERRUPTIBLE); /* TODO: bug */
		schedule_timeout(10);
	}

	printk(KERN_ERR "%s: hardware reset timeout\n", cp->dev->name);
}

static inline void rtl8186_start_hw(struct re_private *cp)
{
	// set TX/RX setting/enable TX/RX
	RTL_W32(IO_CMD, CMD_CONFIG);
	netdev_reset_queue(cp->dev);
}

static void rtl8186_init_hw(struct re_private *cp)
{
	struct net_device *dev = cp->dev;
	u8 status;

	rtl8186_reset_hw(cp);
	RTL_W8(CMD,
	       0x2); /* RX checksum offload enable, VLAN de-tagging disable */

	RTL_W16(ISR, 0xffff);
	rtl8186_set_intr_mask(cp, 0);
	RTL_W32(RxFDP, (u32)(cp->rx_ring));
	RTL_W16(RxCDO, 0);
	RTL_W32(TxFDP1, (u32)(cp->tx_hqring));

	RTL_W16(TxCDO1, 0);
	RTL_W32(TxFDP2, (u32)(cp->tx_lqring));
	RTL_W16(TxCDO2, 0);
	RTL_W32(RCR, AcceptAll);
	RTL_W32(TCR, 0xC00);
	RTL_W8(RxRingSize, RINGSIZE);
	status = RTL_R8(MSR);

	status = status & ~(TXFCE | RXFCE);
	RTL_W8(MSR, status);

	cp->phy_type = mdio_read(dev, 0, 3);
	if (cp->phy_type == 0xffff) {
		printk_once(KERN_INFO "%s: No PHY connected\n", dev->name);
	} else {
		if (cp->phy_type == 0x8201) {
			printk_once(KERN_INFO "%s: PHY: RTL8201\n", dev->name);
			RTL_W32(MIIAR, BIT(31) | BIT(26) |
					       0x3300); // Reset/re-auto-nego
			mdelay(200);
		} else if (cp->phy_type == 0xc852) {
			/* TODO: future phy_id should be 0x001cc852, with phy_id_mask of 0x001fffff */
			printk_once(KERN_INFO "%s: PHY: RTL8305SC\n",
				    dev->name);
			/* PHY 0-5 returned c852 on BR6204Wg */
			/* PHY 0 is labeled "4" on hardware (BR6204wg) */
			/* The last accessed PHY will be the one that will give LNKCHG interrupts. Weird. */
			// writing 0x3b00 to PHY 4 has shut down WAN port
			// writing 0x3b00 to PHY 5 had no effect (aligned with docs...)
			/* accessing PHYs 2,3,4 (register 3) raises LNKCHG interrupt */

			// Don't reconfigure the switch for now. Bootloader/HW does that for us
			// mdio_write(dev, 0, 0, 0x3300);
			// mdio_write(dev, 1, 0, 0x3300);
			/* TODO currently, "ifconfig eth1 up" will reset eth0 4 ports */
			// mdio_write(dev, 2, 0, 0x3300);
			// mdio_write(dev, 3, 0, 0x3300);
			// mdio_write(dev, 4, 0, 0x3300);
			// mdio_write(dev, 5, 0, 0x3b00);

			// Should be 8305
			// if (dev->base_addr == 0xbd200000)
			// RTL_W32(MIIAR, BIT(31) |BIT(30) | 0x3300); // LAN MII is open
			// else
			// RTL_W32(MIIAR, BIT(31)| BIT(28) | 0x1300); // WAN is port 4 (5th port)
		}
	}

	rtl8186_change_mac_address(dev, dev->dev_addr);

	rtl8186_start_hw(cp);
}

/* TODO refactor to a function that renews single entry */
/* TODO maybe always use same buffers for TX/RX? Copy them when they arrive, and call build_skb */
static int rtl8186_refill_rx(struct re_private *cp)
{
	int i;

	for (i = 0; i < RTL8186_RX_RING_SIZE; i++) {
		struct sk_buff *skb;

		skb = rtl8186_alloc_rx_skb(cp, cp->rx_buf_sz);
		if (!skb)
			goto err_out;

		skb->dev = cp->dev;
		cp->rx_skb[i].skb = skb;

		if (unlikely(((u32)skb->data) & 0x3))
			printk(KERN_DEBUG "skb->data unaligment %8x\n",
			       (u32)skb->data);

		// set to noncache area
		cp->rx_ring[i].addr = ((u32)skb->data) | UNCACHE_MASK;

		if (i == (RTL8186_RX_RING_SIZE - 1))
			cp->rx_ring[i].opts1 =
				(DescOwn | RingEnd | cp->rx_buf_sz);
		else
			cp->rx_ring[i].opts1 = (DescOwn | cp->rx_buf_sz);

		cp->rx_ring[i].opts2 = 0;
	}

	return 0;

err_out:
	rtl8186_clean_rings(cp);
	return -ENOMEM;
}

static void rtl8186_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct re_private *cp = rtl8186_priv(dev);
	unsigned tx_head = cp->tx_hqhead;
	unsigned tx_tail = cp->tx_hqtail;
	unsigned long flags;

	spin_lock_irqsave(&cp->lock, flags);
	while (tx_tail != tx_head) {
		struct sk_buff *skb;
		u32 status;

		status = (cp->tx_hqring[tx_tail].opts1);
		if (status & DescOwn)
			break;
		skb = cp->tx_skb[tx_tail].skb;
		if (!skb)
			BUG();
		cp->net_stats.tx_packets++;
		cp->net_stats.tx_bytes += skb->len;
		napi_consume_skb(skb, 0);
		cp->tx_skb[tx_tail].skb = NULL;
		tx_tail = NEXT_TX(tx_tail);
	}

	cp->tx_hqtail = tx_tail;

	spin_unlock_irqrestore(&cp->lock, flags);

	if (netif_queue_stopped(cp->dev))
		netif_wake_queue(cp->dev);
}
static int rtl8186_init_rings(struct re_private *cp)
{
	memset(cp->tx_hqring, 0, sizeof(DMA_DESC) * RTL8186_TX_RING_SIZE);
	memset(cp->rx_ring, 0, sizeof(DMA_DESC) * RTL8186_RX_RING_SIZE);
	cp->rx_tail = 0;
	cp->tx_hqhead = cp->tx_hqtail = 0;

	return rtl8186_refill_rx(cp);
}

static int rtl8186_alloc_rings(struct re_private *cp)
{
	void *buf;

	buf = kzalloc(RTL8186_RXRING_BYTES, GFP_KERNEL);
	if (!buf)
		goto err_nomem;

	cp->rxdesc_buf = buf;

	buf = (void *)((u32)(buf + DESC_ALIGN - 1) & ~(DESC_ALIGN - 1));
	cp->rx_ring = (DMA_DESC *)((u32)(buf) | UNCACHE_MASK);

	buf = kzalloc(RTL8186_TXRING_BYTES, GFP_KERNEL);
	if (!buf)
		goto err_nomem;
	cp->txdesc_buf = buf;

	buf = (void *)((u32)(buf + DESC_ALIGN - 1) & ~(DESC_ALIGN - 1));
	cp->tx_hqring = (DMA_DESC *)((u32)(buf) | UNCACHE_MASK);

	return rtl8186_init_rings(cp);

err_nomem:
	if (cp->rxdesc_buf) {
		kfree(cp->rxdesc_buf);
		cp->rxdesc_buf = NULL;
		cp->rx_ring = NULL;
	}
	if (cp->txdesc_buf) {
		kfree(cp->txdesc_buf);
		cp->txdesc_buf = NULL;
		cp->tx_hqring = NULL;
	}
	return -ENOMEM;
}

static void rtl8186_clean_rings(struct re_private *cp)
{
	unsigned i;

	for (i = 0; i < RTL8186_RX_RING_SIZE; i++) {
		if (cp->rx_skb[i].skb) {
			napi_consume_skb(cp->rx_skb[i].skb, 0);
		}
	}

	for (i = 0; i < RTL8186_TX_RING_SIZE; i++) {
		if (cp->tx_skb[i].skb) {
			napi_consume_skb(cp->tx_skb[i].skb, 0);
			cp->net_stats.tx_dropped++;
		}
	}

	memset(&cp->rx_skb, 0, sizeof(struct ring_info) * RTL8186_RX_RING_SIZE);
	memset(&cp->tx_skb, 0, sizeof(struct ring_info) * RTL8186_TX_RING_SIZE);
}

static void rtl8186_free_rings(struct re_private *cp)
{
	rtl8186_clean_rings(cp);
	kfree(cp->rxdesc_buf);
	kfree(cp->txdesc_buf);

	cp->rx_ring = NULL;
	cp->tx_hqring = NULL;
}

static int rtl8186_open(struct net_device *dev)
{
	struct re_private *cp = rtl8186_priv(dev);
	int rc;

	if (netif_msg_ifup(cp))
		printk(KERN_DEBUG "%s: enabling interface\n", dev->name);

	rtl8186_set_rxbufsize(cp); /* set new rx buf size */
	rc = rtl8186_alloc_rings(cp);
	if (rc)
		return rc;

	napi_enable(&cp->rx_napi);
	napi_enable(&cp->tx_napi);

	// remember to enable IRQ before enabling TX/RX
	rtl8186_init_hw(cp);

	rc = request_irq(dev->irq, rtl8186_interrupt, 0, dev->name, dev);
	if (rc)
		goto err_out_hw;

	if (cp->phy_type != 0xFFFF) {
		netif_carrier_off(dev);
		mii_check_media(&cp->mii_if, netif_msg_link(cp), true);
	} else {
		// HACK: eth0 has no PHY. So report it is up...
		netif_carrier_on(dev);
	}

	/* Enable interrupts */
	rtl8186_set_intr_mask(cp, rtl8186_intr_mask);

	netif_start_queue(dev);

	return 0;

err_out_hw:
	napi_disable(&cp->rx_napi);
	napi_disable(&cp->tx_napi);
	rtl8186_stop_hw(cp);
	rtl8186_free_rings(cp);
	return rc;
}

static int rtl8186_close(struct net_device *dev)
{
	struct re_private *cp = rtl8186_priv(dev);

	napi_disable(&cp->rx_napi);
	napi_disable(&cp->tx_napi);
	// TODO lock spinlock here

	if (netif_msg_ifdown(cp))
		printk(KERN_DEBUG "%s: disabling interface\n", dev->name);
	netif_stop_queue(dev);
	rtl8186_stop_hw(cp);
	free_irq(dev->irq, dev);
	rtl8186_free_rings(cp);
	return 0;
}

static void rtl8186_change_mac_address(struct net_device *dev, u8 *mac)
{
	struct re_private *cp = rtl8186_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&cp->lock,
			  flags); // TODO is this lock should really be here??

	memmove(dev->dev_addr, mac, 6);

	RTL_W32(IDR0, dev->dev_addr + 0);
	RTL_W32(IDR4, dev->dev_addr + 4); // TODO fix this

	spin_unlock_irqrestore(&cp->lock, flags);
}

static int rtl8186_set_mac_address(struct net_device *dev, void *p)
{
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	rtl8186_change_mac_address(dev, addr->sa_data);

	return 0;
}

// static int rtl8186_change_mtu(struct net_device *dev, int new_mtu)
// {
// 	struct re_private *cp = rtl8186_priv(dev);
// 	int rc;

// 	/* check for invalid MTU, according to hardware limits */
// 	if (new_mtu < RTL8186_MIN_MTU || new_mtu > RTL8186_MAX_MTU)
// 		return -EINVAL;

// 	/* if network interface not up, no need for complexity */
// 	if (!netif_running(dev)) {
// 		dev->mtu = new_mtu;
// 		rtl8186_set_rxbufsize(cp);	/* set new rx buf size */
// 		return 0;
// 	}

// 	spin_lock_irq(&cp->lock);

// 	rtl8186_stop_hw(cp);			/* stop h/w and free rings */
// 	rtl8186_clean_rings(cp);

// 	dev->mtu = new_mtu;
// 	rtl8186_set_rxbufsize(cp);		/* set new rx buf size */

// 	rc = rtl8186_init_rings(cp);		/* realloc and restart h/w */
// 	rtl8186_start_hw(cp);
// 	spin_unlock_irq(&cp->lock);

// 	return rc;
// }

/* TODO more fine grained interrupt disable (only on NIC...) */
static int mdio_read(struct net_device *dev, int phy_id, int location)
{
	struct re_private *cp = rtl8186_priv(dev);
	uint32_t mii_data = (location << 16) | (phy_id << 26);
	unsigned long flags;
	int ret;

	local_irq_save(flags);

	RTL_W32(MIIAR, mii_data);
	mdelay(1);
	ret = (RTL_R32(MIIAR) & 0x0000ffff);

#if RTL8186_DEBUG_MDIO
	printk(KERN_ERR "mdio_read: %d:%d. result=%04x\n", phy_id, location,
	       ret);
#endif

	local_irq_restore(flags);
	return ret;
}

static void mdio_write(struct net_device *dev, int phy_id, int location,
		       int value)
{
	struct re_private *cp = rtl8186_priv(dev);
	uint32_t mii_data =
		BIT(31) | (location << 16) | (phy_id << 26) | (value & 0xffff);
	unsigned long flags;

	local_irq_save(flags);

#if RTL8186_DEBUG_MDIO
	printk(KERN_ERR "mdio write: %d:%d write %04x\n", phy_id, location,
	       value);
#endif
	RTL_W32(MIIAR, mii_data);
	mdelay(1);

	local_irq_restore(flags);
}

static const struct net_device_ops rtl8186_netdev_ops = {
	.ndo_open = rtl8186_open,
	.ndo_stop = rtl8186_close,
	.ndo_set_mac_address = rtl8186_set_mac_address,
	.ndo_get_stats = rtl8186_get_stats,
	.ndo_start_xmit = rtl8186_start_xmit,
	.ndo_tx_timeout = rtl8186_tx_timeout,
	// .ndo_change_mtu = rtl8186_change_mtu,
};

static int rtl8186_eth_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct net_device *dev;
	struct re_private *cp;
	void *regs;
	struct resource *res;

	pr_info_once("%s", version);

	dev = alloc_etherdev(sizeof(struct re_private));
	if (!dev)
		return -ENOMEM;

	SET_NETDEV_DEV(dev, &pdev->dev);

	dev->netdev_ops = &rtl8186_netdev_ops;

	cp = rtl8186_priv(dev);
	cp->dev = dev;
	cp->msg_enable = NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK;
	cp->mii_if.dev = dev;
	cp->mii_if.mdio_read = mdio_read;
	cp->mii_if.mdio_write = mdio_write;
	cp->mii_if.phy_id = 4; /* Currently only enable for port 4 (WAN) */
	cp->mii_if.phy_id_mask = 0x1f;
	cp->mii_if.reg_num_mask = 0x1f;
	spin_lock_init(&cp->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		rc = -EINVAL;
		goto err_out;
	}

	regs = (void *)(res->start);
	dev->base_addr = (unsigned long)regs;
	cp->regs = regs;

	rtl8186_stop_hw(cp);

	eth_random_addr(dev->dev_addr);

	dev->watchdog_timeo = TX_TIMEOUT;
	// dev->min_mtu = RTL8186_MIN_MTU;
	// dev->max_mtu = RTL8186_MAX_MTU;

	dev->features |= NETIF_F_RXCSUM; // NETIF_F_SG

	dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0) {
		rc = -EINVAL;
		goto err_out;
	}

	netif_napi_add(dev, &cp->rx_napi, rtl8186_rx_poll, 64);
	netif_tx_napi_add(dev, &cp->tx_napi, rtl8186_tx_poll, 64);

	rc = register_netdev(dev);
	if (rc)
		goto err_out;

	platform_set_drvdata(pdev, dev);

	pr_info("%s: %s at 0x%lx (irq = %d)\n", pdev->name, dev->name,
		dev->base_addr, dev->irq);

	if (dev->irq == 4) {
		rtl8186_hack_devs[0] = dev;
	} else {
		rtl8186_hack_devs[1] = dev;
	}

	return 0;

err_out:
	platform_set_drvdata(pdev, NULL);
	free_netdev(dev);
	return rc;
}

static int rtl8186_eth_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	if (dev) {
		unregister_netdev(dev);
		free_netdev(dev);
		platform_set_drvdata(pdev, NULL);
	}
	return 0;
}

static const struct of_device_id of_rtl8186_eth_match[] = {
	{ .compatible = "realtek,rtl8186-eth" },
	{},
};
MODULE_DEVICE_TABLE(of, of_rtl8186_eth_match);

static struct platform_driver rtl8186_eth_driver = {
	.driver = {
		.name = "rtl8186-eth",
		.of_match_table = of_rtl8186_eth_match,
	},
	.probe = rtl8186_eth_probe,
	.remove = rtl8186_eth_remove,
};

module_platform_driver(rtl8186_eth_driver);

MODULE_AUTHOR("Yasha Cherikovsky <xxxx@xxxx.com>");
MODULE_DESCRIPTION("Realtek RTL8186 10/100 Ethernet driver");
MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL");
