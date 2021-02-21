/* SPDX-License-Identifier: GPL-2.0 */
#ifndef RTL8180_RTL8225_H
#define RTL8180_RTL8225_H

#define RTL8225_ANAPARAM_ON	0xa0000b59
#define RTL8225_ANAPARAM2_ON	0x860dec11
#define RTL8225_ANAPARAM_OFF	0xa00beb59
#define RTL8225_ANAPARAM2_OFF	0x840dec11

const struct rtl818x_rf_ops * rtl8186_wlan_detect_rf(struct ieee80211_hw *);

static inline void rtl8225_write_phy_ofdm(struct ieee80211_hw *dev,
					  u8 addr, u8 data)
{
	rtl8186_wlan_write_phy(dev, addr, data);
}

static inline void rtl8225_write_phy_cck(struct ieee80211_hw *dev,
					 u8 addr, u8 data)
{
	rtl8186_wlan_write_phy(dev, addr, data | 0x10000);
}

u16 rtl8225_read(struct ieee80211_hw *dev, u8 addr);
void rtl8225_write(struct ieee80211_hw *dev, u8 addr, u16 data);

void rtl8225_rf_init(struct ieee80211_hw *dev);
void rtl8225z2_rf_init(struct ieee80211_hw *dev);

void rtl8225z2_rf_close(struct ieee80211_hw *dev);
void rtl8225z2_rf_set_chan(struct ieee80211_hw *dev, struct ieee80211_conf *conf);

void rtl8225_rf_set_channel(struct ieee80211_hw *dev, struct ieee80211_conf *conf);

void rtl8225_rf_set_tx_power(struct ieee80211_hw *dev, int channel);
void rtl8225z2_rf_set_tx_power(struct ieee80211_hw *dev, int channel);

extern const u8 rtl8225_agc[128];
extern const u32 rtl8225_chan[];

#endif /* RTL8180_RTL8225_H */
