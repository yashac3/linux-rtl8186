/* SPDX-License-Identifier: GPL-2.0 */
#ifndef RTL8180_H
#define RTL8180_H

#include "rtl818x.h"

#define MAX_RX_SIZE IEEE80211_MAX_RTS_THRESHOLD

#define RF_PARAM_ANALOGPHY	(1 << 0)
#define RF_PARAM_ANTBDEFAULT	(1 << 1)
#define RF_PARAM_CARRIERSENSE1	(1 << 2)
#define RF_PARAM_CARRIERSENSE2	(1 << 3)

#define BB_ANTATTEN_CHAN14	0x0C
#define BB_ANTENNA_B 		0x40

#define BB_HOST_BANG 		(1 << 30)
#define BB_HOST_BANG_EN 	(1 << 2)
#define BB_HOST_BANG_CLK 	(1 << 1)
#define BB_HOST_BANG_DATA	1

#define ANAPARAM_TXDACOFF_SHIFT	27
#define ANAPARAM_PWR0_SHIFT	28
#define ANAPARAM_PWR0_MASK 	(0x07 << ANAPARAM_PWR0_SHIFT)
#define ANAPARAM_PWR1_SHIFT	20
#define ANAPARAM_PWR1_MASK	(0x7F << ANAPARAM_PWR1_SHIFT)

/* rtl8180/rtl8185 have 3 queue + beacon queue.
 * mac80211 can use just one, + beacon = 2 tot.
 */
#define RTL8186_NR_TX_QUEUES 4


#define TPPOLL_BQ	BIT(7)


union rtl8186_wlan_tx_desc_flags {
	struct {
		u32 own : 1;
		u32 dma_ok : 1;
		u32 fs : 1;
		u32 ls : 1;
		u32 txrate : 4;
		u32 rts_en : 1;
		u32 rts_rate : 4;
		u32 cts_en : 1;
		u32 more_frag : 1;
		u32 splcp : 1;
		u32 no_encryption : 1;
		u32 __rsvd4 : 3;
		u32 tpktsize : 12;

	} __packed;

	__be32 flags_dword;
} __packed;


struct rtl8186_wlan_tx_desc {
	union rtl8186_wlan_tx_desc_flags flags;

	struct {
		__be32 lengext : 1;
		__be32 plcp_length : 15;
		__be32 rts_dur : 16;
	} __packed;

	__be32 tx_buf;

	struct {
		__be32 __rsvd5 : 20;
		__be32 frame_len : 12;
	} __packed;

	__be32 next_tx_desc;

	struct {
		__be32 rate_fall_back_limit : 4;
		__be32 __rsvd0 : 3;
		__be32 antenna : 1;
		__be32 agc : 8;
		__be32 retry_limit : 8;
		__be32 cw_max : 4;
		__be32 cw_min : 4;
	} __packed;

	__be32 __rsvd1;
	__be32 __rsvd2;
} __packed __aligned(256);

struct rtl818x_rx_cmd_desc {
	__be32 flags;
	u32 reserved;
	__be32 rx_buf;
} __packed;

struct rtl8186_wlan_rx_desc {
	__be32 flags;
	__be32 flags2;
	__be64 tsft;

} __packed;

struct rtl8187se_rx_desc {
	__be32 flags;
	__be64 tsft;
	__be32 flags2;
	__be32 flags3;
	u32 reserved[3];
} __packed;

struct rtl8186_wlan_tx_ring {
	struct rtl8186_wlan_tx_desc *desc;
	dma_addr_t dma;
	unsigned int idx;
	unsigned int entries;
	struct sk_buff_head queue;
};

struct rtl8186_wlan_vif {
	struct ieee80211_hw *dev;

	/* beaconing */
	struct delayed_work beacon_work;
	bool enable_beacon;
};

struct rtl8186_wlan_priv {
	/* common between rtl818x drivers */
	struct rtl818x_csr __iomem *map;
	const struct rtl818x_rf_ops *rf;
	struct ieee80211_vif *vif;

	/* rtl8180 driver specific */
	bool map_pio;
	spinlock_t lock;
	void *rx_ring;
	u8 rx_ring_sz;
	dma_addr_t rx_ring_dma;
	unsigned int rx_idx;
	struct sk_buff *rx_buf[32];
	struct rtl8186_wlan_tx_ring tx_ring[RTL8186_NR_TX_QUEUES];
	struct ieee80211_channel channels[14];
	struct ieee80211_rate rates[12];
	struct ieee80211_supported_band band;
	struct ieee80211_tx_queue_params queue_param[4];
	bool is_mmio;
	int irq;
	struct platform_device *plat_dev;
	struct pci_dev *pdev;
	u32 rx_conf;
	u8 slot_time;
	u16 ack_time;

	enum {
		RTL818X_CHIP_FAMILY_RTL8180,
		RTL818X_CHIP_FAMILY_RTL8185,
		RTL818X_CHIP_FAMILY_RTL8187SE,
	} chip_family;
	u32 anaparam;
	u16 rfparam;
	u8 csthreshold;
	u8 mac_addr[ETH_ALEN];
	u8 rf_type;
	u8 xtal_out;
	u8 xtal_in;
	u8 xtal_cal;
	u8 thermal_meter_val;
	u8 thermal_meter_en;
	u8 antenna_diversity_en;
	u8 antenna_diversity_default;
	/* sequence # */
	u16 seqno;
	u32 basic_mask;
};

void rtl8186_wlan_write_phy(struct ieee80211_hw *dev, u8 addr, u32 data);
u8 rtl8186_wlan_read_phy(struct ieee80211_hw *dev, u8 addr);
void rtl8186_wlan_set_anaparam(struct rtl8186_wlan_priv *priv, u32 anaparam);
void rtl8186_wlan_set_anaparam2(struct rtl8186_wlan_priv *priv, u32 anaparam2);

static inline u8 rtl818x_ioread8(struct rtl8186_wlan_priv *priv, void __iomem *addr)
{
	return ioread8(addr);
}

static inline u16 rtl818x_ioread16(struct rtl8186_wlan_priv *priv, void __iomem *addr)
{
	return ioread16(addr);
}

static inline u32 rtl818x_ioread32(struct rtl8186_wlan_priv *priv, void __iomem *addr)
{
	return ioread32(addr);
}

static inline void rtl818x_iowrite8(struct rtl8186_wlan_priv *priv,
				    void __iomem *addr, u8 val)
{
	iowrite8(val, addr);
}

static inline void rtl818x_iowrite16(struct rtl8186_wlan_priv *priv,
				     void __iomem *addr, u16 val)
{
	iowrite16(val, addr);
}

static inline void rtl818x_iowrite32(struct rtl8186_wlan_priv *priv,
				     void __iomem *addr, u32 val)
{
	iowrite32(val, addr);
}

#endif /* RTL8180_H */
