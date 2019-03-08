#include <linux/delay.h>
#include <net/mac80211.h>

#include "rtl8186-wlan.h"
#include "rtl8225.h"
#include "r8180_hw.h"

void rtl8225z2_rf_set_mode(struct ieee80211_hw *dev);
// u8 ZEBRA2_CCK_OFDM_GAIN_SETTING[]={
//         0x00,0x01,0x02,0x03,0x04,0x05,
//         0x06,0x07,0x08,0x09,0x0a,0x0b,
//         0x0c,0x0d,0x0e,0x0f,0x10,0x11,
//         0x12,0x13,0x14,0x15,0x16,0x17,
//         0x18,0x19,0x1a,0x1b,0x1c,0x1d,
//         0x1e,0x1f,0x20,0x21,0x22,0x23,
// };

// static const u8 rtl8225z2_tx_power_cck_ch14[] = {
// 	0x36, 0x35, 0x2e, 0x1b, 0x00, 0x00, 0x00, 0x00
// };

// static const u8 rtl8225z2_tx_power_cck_B[] = {
// 	0x30, 0x2f, 0x29, 0x21, 0x19, 0x10, 0x08, 0x04
// };

// static const u8 rtl8225z2_tx_power_cck_A[] = {
// 	0x33, 0x32, 0x2b, 0x23, 0x1a, 0x11, 0x08, 0x04
// };

// static const u8 rtl8225z2_tx_power_cck[] = {
// 	0x36, 0x35, 0x2e, 0x25, 0x1c, 0x12, 0x09, 0x04
// };

static const u8 correct_tx_power_cck[] = {
	0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b
};

static const u8 correct_tx_power_ofdm[] = {
	0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x16, 0x16, 0x16, 0x16, 0x16
};

// void rtl8225z2_rf_set_tx_power(struct ieee80211_hw *dev, int channel)
// {
// 	struct rtl8186_wlan_priv *priv = dev->priv;
// 	u8 cck_power, ofdm_power;
// 	int i;

// 	// cck_power = priv->channels[channel - 1].hw_value & 0xFF;
// 	// ofdm_power = priv->channels[channel - 1].hw_value >> 8;
// 	cck_power = correct_tx_power_cck[channel - 1];
// 	ofdm_power = correct_tx_power_ofdm[channel - 1];

// 	for (i = 0; i < ARRAY_SIZE(correct_tx_power_cck); i++) {
// 		rtl8225_write_phy_cck(dev, 0x44 + i, correct_tx_power_cck[i]);
// 	}

// 	// cck_power = min(cck_power, (u8)35);
// 	// if (cck_power == 13 || cck_power == 14)
// 		// cck_power = 12;
// 	// if (cck_power >= 15)
// 		// cck_power -= 2;
	
// 	rtl818x_iowrite8(priv, &priv->map->TX_GAIN_CCK, cck_power);
// 	rtl818x_ioread8(priv, &priv->map->TX_GAIN_CCK);
// 	msleep(1);

// 	// ofdm_power = min(ofdm_power, (u8)35);
// 	rtl818x_iowrite8(priv, &priv->map->TX_GAIN_OFDM, ofdm_power);

// 	printk(KERN_ERR "cck_power=%d, ofdm_power=%d\n", cck_power, ofdm_power);
// 	rtl8225_write_phy_ofdm(dev, 2, 0x62);
// 	rtl8225_write_phy_ofdm(dev, 5, 0x00);
// 	rtl8225_write_phy_ofdm(dev, 6, 0x40);
// 	rtl8225_write_phy_ofdm(dev, 7, 0x00);
// 	rtl8225_write_phy_ofdm(dev, 8, 0x40);

// 	msleep(1);
// }

// static const u16 rtl8225z2_rxgain[] = {
// 	0x0000, 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0008, 0x0009,
// 	0x000a, 0x000b, 0x0102, 0x0103, 0x0104, 0x0105, 0x0140, 0x0141,
// 	0x0142, 0x0143, 0x0144, 0x0145, 0x0180, 0x0181, 0x0182, 0x0183,
// 	0x0184, 0x0185, 0x0188, 0x0189, 0x018a, 0x018b, 0x0243, 0x0244,
// 	0x0245, 0x0280, 0x0281, 0x0282, 0x0283, 0x0284, 0x0285, 0x0288,
// 	0x0289, 0x028a, 0x028b, 0x028c, 0x0342, 0x0343, 0x0344, 0x0345,
// 	0x0380, 0x0381, 0x0382, 0x0383, 0x0384, 0x0385, 0x0388, 0x0389,
// 	0x038a, 0x038b, 0x038c, 0x038d, 0x0390, 0x0391, 0x0392, 0x0393,
// 	0x0394, 0x0395, 0x0398, 0x0399, 0x039a, 0x039b, 0x039c, 0x039d,
// 	0x03a0, 0x03a1, 0x03a2, 0x03a3, 0x03a4, 0x03a5, 0x03a8, 0x03a9,
// 	0x03aa, 0x03ab, 0x03ac, 0x03ad, 0x03b0, 0x03b1, 0x03b2, 0x03b3,
// 	0x03b4, 0x03b5, 0x03b8, 0x03b9, 0x03ba, 0x03bb, 0x03bb
// };

// void rtl8225z2_rf_init(struct ieee80211_hw *dev)
// {
// 	struct rtl8186_wlan_priv *priv = dev->priv;
// 	int i;

// 	printk(KERN_ERR "rtl8225z2_rf_init()\n");
// 	rtl8186_wlan_set_anaparam(priv, RTL8225_ANAPARAM_ON);

// 	/* host_pci_init */
// 	rtl818x_iowrite16(priv, &priv->map->RFPinsOutput, 0x0480);
// 	rtl818x_iowrite16(priv, &priv->map->RFPinsEnable, 0x1FFF);
// 	rtl818x_iowrite16(priv, &priv->map->RFPinsSelect, 0x0488);
// 	rtl818x_iowrite8(priv, &priv->map->GP_ENABLE, 0);
// 	rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
// 	msleep(200);	/* FIXME: ehh?? */
// 	rtl818x_iowrite8(priv, &priv->map->GP_ENABLE, 0xFF & ~(1 << 6));

// 	rtl818x_iowrite32(priv, &priv->map->RF_TIMING, 0x00088008);

// 	/* TODO: check if we need really to change BRSR to do RF config */
// 	rtl818x_ioread16(priv, &priv->map->BRSR);
// 	rtl818x_iowrite16(priv, &priv->map->BRSR, 0xFFFF);
// 	rtl818x_iowrite32(priv, &priv->map->RF_PARA, 0x00100044);
// 	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
// 	rtl818x_iowrite8(priv, &priv->map->CONFIG3, 0x44);
// 	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_NORMAL);

// 	rtl818x_iowrite16(priv, &priv->map->RFPinsEnable, 0x1FFF);

// 	rtl8225_write(dev, 0x0, 0x0B7); msleep(1);
// 	rtl8225_write(dev, 0x1, 0xEE0); msleep(1);
// 	rtl8225_write(dev, 0x2, 0x44D); msleep(1);
// 	rtl8225_write(dev, 0x3, 0x441); msleep(1);
// 	rtl8225_write(dev, 0x4, 0x8C3); msleep(1);
// 	rtl8225_write(dev, 0x5, 0xC72); msleep(1);
// 	rtl8225_write(dev, 0x6, 0x0E6); msleep(1);
// 	rtl8225_write(dev, 0x7, 0x82A); msleep(1);
// 	rtl8225_write(dev, 0x8, 0x03F); msleep(1);
// 	rtl8225_write(dev, 0x9, 0x335); msleep(1);
// 	rtl8225_write(dev, 0xa, 0x9D4); msleep(1);
// 	rtl8225_write(dev, 0xb, 0x7BB); msleep(1);
// 	rtl8225_write(dev, 0xc, 0x850); msleep(1);
// 	rtl8225_write(dev, 0xd, 0xCDF); msleep(1);
// 	rtl8225_write(dev, 0xe, 0x02B); msleep(1);
// 	rtl8225_write(dev, 0xf, 0x114); msleep(100);

// 	if (!(rtl8225_read(dev, 6) & (1 << 7))) {
// 		rtl8225_write(dev, 0x02, 0x0C4D);
// 		msleep(200);
// 		rtl8225_write(dev, 0x02, 0x044D);
// 		msleep(100);
// 		/* TODO: readd calibration failure message when the calibration
// 		   check works */
// 	}

// 	rtl8225_write(dev, 0x0, 0x1B7);
// 	rtl8225_write(dev, 0x3, 0x002);
// 	rtl8225_write(dev, 0x5, 0x004);

// 	for (i = 0; i < ARRAY_SIZE(rtl8225z2_rxgain); i++) {
// 		rtl8225_write(dev, 0x1, i + 1);
// 		rtl8225_write(dev, 0x2, rtl8225z2_rxgain[i]);
// 	}

// 	rtl8225_write(dev, 0x0, 0x0B7); msleep(100);
// 	rtl8225_write(dev, 0x2, 0xC4D);

// 	msleep(200);
// 	rtl8225_write(dev, 0x2, 0x44D);
// 	msleep(100);

// 	rtl8225_write(dev, 0x00, 0x2BF);
// 	rtl8225_write(dev, 0xFF, 0xFFFF);

// 	rtl818x_iowrite16(priv, &priv->map->RFPinsEnable, 0x1FFF);

// 	for (i = 0; i < ARRAY_SIZE(rtl8225_agc); i++) {
// 		rtl8225_write_phy_ofdm(dev, 0xB, rtl8225_agc[i]);
// 		msleep(1);
// 		rtl8225_write_phy_ofdm(dev, 0xA, 0x80 + i);
// 		msleep(1);
// 	}

// 	msleep(1);

// 	rtl8225_write_phy_ofdm(dev, 0x00, 0x01); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x01, 0x02); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x02, 0x62); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x03, 0x00); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x04, 0x00); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x05, 0x00); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x06, 0x40); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x07, 0x00); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x08, 0x40); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x09, 0xfe); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x0a, 0x09); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x18, 0xef); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x0b, 0x80); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x0c, 0x01); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x0d, 0x43);
// 	rtl8225_write_phy_ofdm(dev, 0x0e, 0xd3); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x0f, 0x38); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x10, 0x84); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x11, 0x06); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x12, 0x20); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x13, 0x20); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x14, 0x00); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x15, 0x40); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x16, 0x00); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x17, 0x40); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x18, 0xef); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x19, 0x19); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x1a, 0x20); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x1b, 0x11); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x1c, 0x04); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x1d, 0xc5); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x1e, 0xb3); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x1f, 0x75); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x20, 0x1f); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x21, 0x27); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x22, 0x16); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x23, 0x80); msleep(1); /* FIXME: not needed? */
// 	rtl8225_write_phy_ofdm(dev, 0x24, 0x46); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x25, 0x20); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x26, 0x90); msleep(1);
// 	rtl8225_write_phy_ofdm(dev, 0x27, 0x88); msleep(1);

// 	rtl8225_write_phy_cck(dev, 0x00, 0x98); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x03, 0x20); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x04, 0x7e); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x05, 0x12); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x06, 0xfc); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x07, 0x78); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x08, 0x2e); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x10, 0x93); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x11, 0x88); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x12, 0x47); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x13, 0xd0);
// 	rtl8225_write_phy_cck(dev, 0x19, 0x00);
// 	rtl8225_write_phy_cck(dev, 0x1a, 0xa0);
// 	rtl8225_write_phy_cck(dev, 0x1b, 0x08);
// 	rtl8225_write_phy_cck(dev, 0x40, 0x86);
// 	rtl8225_write_phy_cck(dev, 0x41, 0x8a); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x42, 0x15); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x43, 0x18); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x44, 0x36); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x45, 0x35); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x46, 0x2e); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x47, 0x25); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x48, 0x1c); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x49, 0x12); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x4a, 0x09); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x4b, 0x04); msleep(1);
// 	rtl8225_write_phy_cck(dev, 0x4c, 0x05); msleep(1);

// 	rtl818x_iowrite8(priv, (u8 __iomem *)((void __iomem *)priv->map + 0x5B), 0x0D); msleep(1);

// 	rtl8225z2_rf_set_tx_power(dev, 1);

// 	/* RX antenna default to A */
// 	rtl8225_write_phy_cck(dev, 0x10, 0x9b); msleep(1);	/* B: 0xDB */
// 	rtl8225_write_phy_ofdm(dev, 0x26, 0x90); msleep(1);	/* B: 0x10 */

// 	rtl818x_iowrite8(priv, &priv->map->TX_ANTENNA, 0x03);	/* B: 0x00 */
// 	msleep(1);
// 	rtl818x_iowrite32(priv, (__be32 __iomem *)((void __iomem *)priv->map + 0x94), 0x15c00002);
// 	rtl818x_iowrite16(priv, &priv->map->RFPinsEnable, 0x1FFF);
// }





#define RTL8225Z2_ANAPARAM_OFF	0x55480658
#define RTL8225Z2_ANAPARAM2_OFF	0x72003f70



u8 rtl8225_agc_v2[]={
	0x9e,0x9e,0x9e,0x9e,0x9e,0x9e,0x9e,0x9e,0x9d,0x9c,0x9b,0x9a,0x99,0x98,0x97,0x96,
	0x95,0x94,0x93,0x92,0x91,0x90,0x8f,0x8e,0x8d,0x8c,0x8b,0x8a,0x89,0x88,0x87,0x86,
	0x85,0x84,0x83,0x82,0x81,0x80,0x3f,0x3e,0x3d,0x3c,0x3b,0x3a,0x39,0x38,0x37,0x36,
	0x35,0x34,0x33,0x32,0x31,0x30,0x2f,0x2e,0x2d,0x2c,0x2b,0x2a,0x29,0x28,0x27,0x26,
	0x25,0x24,0x23,0x22,0x21,0x20,0x1f,0x1e,0x1d,0x1c,0x1b,0x1a,0x19,0x18,0x17,0x16,
	0x15,0x14,0x13,0x12,0x11,0x10,0x0f,0x0e,0x0d,0x0c,0x0b,0x0a,0x09,0x08,0x07,0x06,
	0x05,0x04,0x03,0x02,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
	0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
};


u32 rtl8225_chan_v2[] = {
	0,	
	0x085c, 
	0x08dc, 
	0x095c, 
	0x09dc, 
	0x0a5c, 
	0x0adc, 
	0x0b5c, 
	0x0bdc, 
	0x0c5c, 
	0x0cdc, 
	0x0d5c, 
	0x0ddc, 
	0x0e5c, 
	0x0f72, 
};


int rtl8186_get_channel(struct ieee80211_conf *conf)
{
	return ieee80211_frequency_to_channel(conf->chandef.chan->center_freq);
}

#define IEEE_G 1
#define IEEE_B 2
#define IEEE_A 3


int rtl8186_wlan_get_mode(struct ieee80211_hw *dev) {
	struct rtl8186_wlan_priv *priv = dev->priv;
	u32 resp_max = fls(priv->basic_mask) - 1;
	if (resp_max > 3) {
		return IEEE_G;
	} else {
		return IEEE_B;
	}
}


#define RTL8255_ANAPARAM_ON 0xa0000b59
#define RTL8255_ANAPARAM2_ON 0x840cf311


void force_pci_posting(void *ignore) {
	mb();
}

// #define write_phy_cck rtl8225_write_phy_cck
// #define write_phy_ofdm rtl8225_write_phy_ofdm



// #define write_rtl8225  rtl8225_write
// #define read_rtl8225  rtl8225_read


static inline u8 read_nic_byte(struct rtl8186_wlan_priv *priv, u32 offset)
{
	return ioread8(((void*)priv->map) + offset);
}

static inline u16 read_nic_word(struct rtl8186_wlan_priv *priv, u32 offset)
{
	return ioread16(((void*)priv->map) + offset);
}

static inline void write_nic_byte(struct rtl8186_wlan_priv *priv, u32 offset, u8 val)
{
	iowrite8(val, ((void*)priv->map) + offset);
}

static inline void write_nic_word(struct rtl8186_wlan_priv *priv, u32 offset, u16 val)
{
	iowrite16(val, ((void*)priv->map) + offset);
}

static inline void write_nic_dword(struct rtl8186_wlan_priv *priv, u32 offset, u32 val)
{
	iowrite32(val, ((void*)priv->map) + offset);
}


void rtl8180_set_mode(struct ieee80211_hw *dev,int mode)
{
	u8 ecmd;
	struct rtl8186_wlan_priv *priv = dev->priv;
	ecmd=read_nic_byte(priv, EPROM_CMD);
	ecmd=ecmd &~ EPROM_CMD_OPERATING_MODE_MASK;
	ecmd=ecmd | (mode<<EPROM_CMD_OPERATING_MODE_SHIFT);
	ecmd=ecmd &~ (1<<EPROM_CS_SHIFT);
	ecmd=ecmd &~ (1<<EPROM_CK_SHIFT);
	write_nic_byte(priv, EPROM_CMD, ecmd);
}



void rtl8185_rf_pins_enable(struct ieee80211_hw *dev)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	write_nic_word(priv, RFPinsEnable, 0x1fff);
}


void rtl8225_host_pci_init(struct ieee80211_hw *dev) 
{
	struct rtl8186_wlan_priv *priv = dev->priv;

	write_nic_word(priv, RFPinsOutput, 0x480);
	
	rtl8185_rf_pins_enable(dev);
	
	write_nic_word(priv, RFPinsSelect, 0x88 | SW_CONTROL_GPIO); 
	
	write_nic_byte(priv, GP_ENABLE, 0);
	
	force_pci_posting(dev);
	mdelay(200);
	
	write_nic_word(priv, GP_ENABLE, 0xff & (~(1<<6))); 	
}


void rtl8185_set_anaparam2(struct ieee80211_hw *dev, u32 a)
{
	u8 conf3;
	struct rtl8186_wlan_priv *priv = dev->priv;

	rtl8180_set_mode(dev, EPROM_CMD_CONFIG);

	conf3 = read_nic_byte(priv, CONFIG3);
	write_nic_byte(priv, CONFIG3, conf3 | (1<<CONFIG3_ANAPARAM_W_SHIFT));
	write_nic_dword(priv, ANAPARAM2, a);

	conf3 = read_nic_byte(priv, CONFIG3);
	write_nic_byte(priv, CONFIG3, conf3 &~(1<<CONFIG3_ANAPARAM_W_SHIFT));
	rtl8180_set_mode(dev, EPROM_CMD_NORMAL);

}


void rtl8180_set_anaparam(struct ieee80211_hw *dev, u32 a)
{
	u8 conf3;
	struct rtl8186_wlan_priv *priv = dev->priv;

	rtl8180_set_mode(dev, EPROM_CMD_CONFIG);

	conf3 = read_nic_byte(priv, CONFIG3);
	write_nic_byte(priv, CONFIG3, conf3 | (1<<CONFIG3_ANAPARAM_W_SHIFT));
	write_nic_dword(priv, ANAPARAM, a);

	conf3 = read_nic_byte(priv, CONFIG3);
	write_nic_byte(priv, CONFIG3, conf3 &~(1<<CONFIG3_ANAPARAM_W_SHIFT));
	rtl8180_set_mode(dev, EPROM_CMD_NORMAL);
}



void rtl8185_tx_antenna(struct ieee80211_hw *dev, u8 ant)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	write_nic_byte(priv, TX_ANTENNA, ant); 
	force_pci_posting(dev);
	mdelay(1);
}	



void rtl8185_write_phy(struct ieee80211_hw *dev, u8 adr, u32 data)
{
	u8 phyr;
	u32 phyw;
	int i;
	struct rtl8186_wlan_priv *priv = dev->priv;
	
	adr |= 0x80;
	 
	phyw= ((data<<8) | adr);
#if 1	
	
	write_nic_dword(priv, PHY_ADR, phyw);
	
	for(i=0;i<10;i++){
		write_nic_dword(priv, PHY_ADR, 0xffffff7f & phyw);
		phyr = read_nic_byte(priv, PHY_READ);
		if(phyr == (data&0xff)) break;
			
	}
#else
	write_nic_byte(dev, 0x7f, ((phyw & 0xff000000) >> 24));
	write_nic_byte(dev, 0x7e, ((phyw & 0x00ff0000) >> 16));
	write_nic_byte(dev, 0x7d, ((phyw & 0x0000ff00) >> 8));
	write_nic_byte(dev, 0x7c, ((phyw & 0x000000ff) ));
#endif
}


inline void write_phy_ofdm (struct ieee80211_hw *dev, u8 adr, u32 data)
{
	data = data & 0xff;
	rtl8185_write_phy(dev, adr, data);
}


void write_phy_cck (struct ieee80211_hw *dev, u8 adr, u32 data)
{
	data = data & 0xff;
	rtl8185_write_phy(dev, adr, data | 0x10000);
}




u32 read_rtl8225(struct ieee80211_hw *_dev, u8 adr)
{
	u32 data2Write = ((u32)(adr & 0x1f)) << 27;
	u32 dataRead;
	u32 mask;
	u16 oval,oval2,oval3,tmp;
	int i;
	short bit, rw;
	struct rtl8186_wlan_priv *dev = _dev->priv;
	
	u8 wLength = 6;
	u8 rLength = 12;
	u8 low2high = 0;

	oval = read_nic_word(dev, RFPinsOutput);
	oval2 = read_nic_word(dev, RFPinsEnable);
	oval3 = read_nic_word(dev, RFPinsSelect);

	write_nic_word(dev, RFPinsEnable, (oval2|0xf));
	write_nic_word(dev, RFPinsSelect, (oval3|0xf));

	dataRead = 0;

	oval &= ~0xf; 

	write_nic_word(dev, RFPinsOutput, oval | BB_HOST_BANG_EN ); udelay(4);

	write_nic_word(dev, RFPinsOutput, oval ); udelay(5);
	
	rw = 0;
	
	mask = (low2high) ? 0x01 : (((u32)0x01)<<(32-1));
	for(i = 0; i < wLength/2; i++)
	{
		bit = ((data2Write&mask) != 0) ? 1 : 0;
		write_nic_word(dev, RFPinsOutput, bit|oval | rw); udelay(1);
		
		write_nic_word(dev, RFPinsOutput, bit|oval | BB_HOST_BANG_CLK | rw); udelay(2);
		write_nic_word(dev, RFPinsOutput, bit|oval | BB_HOST_BANG_CLK | rw); udelay(2);

		mask = (low2high) ? (mask<<1): (mask>>1);

		if(i == 2)
		{
			rw = BB_HOST_BANG_RW;
			write_nic_word(dev, RFPinsOutput, bit|oval | BB_HOST_BANG_CLK | rw); udelay(2);
			write_nic_word(dev, RFPinsOutput, bit|oval | rw); udelay(2);
			break;
		}
		
		bit = ((data2Write&mask) != 0) ? 1: 0;
		
		write_nic_word(dev, RFPinsOutput, oval|bit|rw| BB_HOST_BANG_CLK); udelay(2);
		write_nic_word(dev, RFPinsOutput, oval|bit|rw| BB_HOST_BANG_CLK); udelay(2);

		write_nic_word(dev, RFPinsOutput, oval| bit |rw); udelay(1);

		mask = (low2high) ? (mask<<1) : (mask>>1);
	}

	write_nic_word(dev, RFPinsOutput, rw|oval); udelay(2);
	mask = (low2high) ? 0x01 : (((u32)0x01) << (12-1));
	
	write_nic_word(dev, RFPinsEnable, (oval2 & (~0x01)));

	for(i = 0; i < rLength; i++)
	{
		write_nic_word(dev, RFPinsOutput, rw|oval); udelay(1);
		
		write_nic_word(dev, RFPinsOutput, rw|oval|BB_HOST_BANG_CLK); udelay(2);
		write_nic_word(dev, RFPinsOutput, rw|oval|BB_HOST_BANG_CLK); udelay(2);
		write_nic_word(dev, RFPinsOutput, rw|oval|BB_HOST_BANG_CLK); udelay(2);
		tmp = read_nic_word(dev, RFPinsInput);
		
		dataRead |= (tmp & BB_HOST_BANG_CLK ? mask : 0);

		write_nic_word(dev, RFPinsOutput, (rw|oval)); udelay(2);

		mask = (low2high) ? (mask<<1) : (mask>>1);
	}
	
	write_nic_word(dev, RFPinsOutput, BB_HOST_BANG_EN|BB_HOST_BANG_RW|oval); udelay(2);

	write_nic_word(dev, RFPinsEnable, oval2);   
	write_nic_word(dev, RFPinsSelect, oval3);   
	write_nic_word(dev, RFPinsOutput, 0x3a0);

	return dataRead;
	
}



void write_rtl8225(struct ieee80211_hw *_dev, u8 adr, u16 data)
{
	int i;
	u16 out,select;
	u8 bit;
	u32 bangdata = (data << 4) | (adr & 0xf);
	struct rtl8186_wlan_priv *dev = _dev->priv;
	
	out = read_nic_word(dev, RFPinsOutput) & 0xfff3;
		
	write_nic_word(dev,RFPinsEnable,
		(read_nic_word(dev,RFPinsEnable) | 0x7));
	
	select = read_nic_word(dev, RFPinsSelect);
	
	write_nic_word(dev, RFPinsSelect, select | 0x7 | SW_CONTROL_GPIO);
	
	force_pci_posting(dev);
	udelay(10);
	
	write_nic_word(dev, RFPinsOutput, out | BB_HOST_BANG_EN );
	
	force_pci_posting(dev);
	udelay(2);
	
	write_nic_word(dev, RFPinsOutput, out);
	
	force_pci_posting(dev);
	udelay(10);
	
	
	for(i=15; i>=0;i--){
	
		bit = (bangdata & (1<<i)) >> i;
		
		write_nic_word(dev, RFPinsOutput, bit | out);
		
		write_nic_word(dev, RFPinsOutput, bit | out | BB_HOST_BANG_CLK);
		write_nic_word(dev, RFPinsOutput, bit | out | BB_HOST_BANG_CLK);

		i--;
		bit = (bangdata & (1<<i)) >> i;
		
		write_nic_word(dev, RFPinsOutput, bit | out | BB_HOST_BANG_CLK);
		write_nic_word(dev, RFPinsOutput, bit | out | BB_HOST_BANG_CLK);

		write_nic_word(dev, RFPinsOutput, bit | out);

	}
	
	write_nic_word(dev, RFPinsOutput, out | BB_HOST_BANG_EN);
	
	force_pci_posting(dev);
	udelay(10);

	write_nic_word(dev, RFPinsOutput, out | BB_HOST_BANG_EN);

	write_nic_word(dev, RFPinsSelect, select | SW_CONTROL_GPIO);	

	rtl8185_rf_pins_enable(_dev);
}

// void rtl8225_rf_set_chan(struct ieee80211_hw *dev, short ch)
// {
// 	struct r8180_priv *priv = ieee80211_priv(dev);
// 	short gset = (priv->ieee80211->state == IEEE80211_LINKED &&
// 		ieee80211_is_54g(priv->ieee80211->current_network)) ||
// 		priv->ieee80211->iw_mode == IW_MODE_MONITOR;
	
// 	rtl8225_SetTXPowerLevel(dev, ch);
	
// 	write_rtl8225(dev, 0x7, rtl8225_chan[ch]);
	
// 	force_pci_posting(dev);
// 	mdelay(10);
	
// 	if(gset){
// 		write_nic_byte(dev,SIFS,0x22);
// 		write_nic_byte(dev,DIFS,0x14); 
// 	}else{
// 		write_nic_byte(dev,SIFS,0x44);
// 		write_nic_byte(dev,DIFS,50 - 14); 
// 	}
// 	if(priv->ieee80211->state == IEEE80211_LINKED &&
// 		ieee80211_is_shortslot(priv->ieee80211->current_network))
// 		write_nic_byte(dev,SLOT,0x9); 
		
// 	else
// 		write_nic_byte(dev,SLOT,0x14); 
		
	
// 	if(gset){
// 		write_nic_byte(dev,EIFS,81);
// 		write_nic_byte(dev,CW_VAL,0x73); 
// 	}else{
// 		write_nic_byte(dev,EIFS,81); 
// 		write_nic_byte(dev,CW_VAL,0xa5); 
// 	}


// }

/******************************************************************************
 * Copyright(c) 2008 - 2010 Realtek Corporation. All rights reserved.
 *
 * Based on the r8180 driver, which is:
 * Copyright 2004-2005 Andrea Merello <andreamrl@tiscali.it>, et al.
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 * Contact Information:
 * Jerry chuang <wlanfae@realtek.com>
******************************************************************************/



u8 rtl8225z2_threshold[]={
        0x8d, 0x8d, 0x8d, 0x8d, 0x9d, 0xad, 0xbd,
};

u8 rtl8225z2_gain_bg[]={
	0x23, 0x15, 0xa5, 
        0x23, 0x15, 0xb5, 
        0x23, 0x15, 0xc5, 
        0x33, 0x15, 0xc5, 
        0x43, 0x15, 0xc5, 
        0x53, 0x15, 0xc5, 
        0x63, 0x15, 0xc5, 
};

u8 rtl8225z2_gain_a[]={
	0x13,0x27,0x5a,
	0x23,0x23,0x58,
	0x33,0x1f,0x56,
	0x43,0x1b,0x54,
	0x53,0x17,0x51,
	0x63,0x24,0x4f,
	0x73,0x0f,0x4c,
};
#if 0
u32 rtl8225_chan[] = {
	0,	
	0x085c, 
	0x08dc, 
	0x095c, 
	0x09dc, 
	0x0a5c, 
	0x0adc, 
	0x0b5c, 
	0x0bdc, 
	0x0c5c, 
	0x0cdc, 
	0x0d5c, 
	0x0ddc, 
	0x0e5c, 
	0x0f72, 
};
#endif

#if 0
u16 rtl8225z2_rxgain[]={	
	0x0400, 0x0401, 0x0402, 0x0403, 0x0404, 0x0405, 0x0408, 0x0409,
	0x040a, 0x040b, 0x0502, 0x0503, 0x0504, 0x0505, 0x0540, 0x0541,  
	0x0542, 0x0543, 0x0544, 0x0545, 0x0580, 0x0581, 0x0582, 0x0583,
	0x0584, 0x0585, 0x0588, 0x0589, 0x058a, 0x058b, 0x0643, 0x0644, 
	0x0645, 0x0680, 0x0681, 0x0682, 0x0683, 0x0684, 0x0685, 0x0688,
	0x0689, 0x068a, 0x068b, 0x068c, 0x0742, 0x0743, 0x0744, 0x0745,
	0x0780, 0x0781, 0x0782, 0x0783, 0x0784, 0x0785, 0x0788, 0x0789,
	0x078a, 0x078b, 0x078c, 0x078d, 0x0790, 0x0791, 0x0792, 0x0793,
	0x0794, 0x0795, 0x0798, 0x0799, 0x079a, 0x079b, 0x079c, 0x079d,  
	0x07a0, 0x07a1, 0x07a2, 0x07a3, 0x07a4, 0x07a5, 0x07a8, 0x07a9,  
	0x03aa, 0x03ab, 0x03ac, 0x03ad, 0x03b0, 0x03b1, 0x03b2, 0x03b3,  
	0x03b4, 0x03b5, 0x03b8, 0x03b9, 0x03ba, 0x03bb, 0x03bb
};
#endif

u16 rtl8225z2_rxgain[]={	
   0x0000,   0x0001,   0x0002,   0x0003,   0x0004,   0x0005,   0x0008,   0x0009,
   0x000a,   0x000b,   0x0102,   0x0103,   0x0104,   0x0105,   0x0140,   0x0141,   
   0x0142,   0x0143,   0x0144,   0x0145,   0x0180,   0x0181,   0x0182,   0x0183,
   0x0184,   0x0185,   0x0188,   0x0189,   0x018a,   0x018b,   0x0243,   0x0244,  
   0x0245,   0x0280,   0x0281,   0x0282,   0x0283,   0x0284,   0x0285,   0x0288,
   0x0289,   0x028a,   0x028b,   0x028c,   0x0342,   0x0343,   0x0344,   0x0345,
   0x0380,   0x0381,   0x0382,   0x0383,   0x0384,   0x0385,   0x0388,   0x0389,
   0x038a,   0x038b,   0x038c,   0x038d,   0x0390,   0x0391,   0x0392,   0x0393,
   0x0394,   0x0395,   0x0398,   0x0399,   0x039a,   0x039b,   0x039c,   0x039d,   
   0x03a0,   0x03a1,   0x03a2,   0x03a3,   0x03a4,   0x03a5,   0x03a8,   0x03a9,   
   0x03aa,   0x03ab,   0x03ac,	 0x03ad,   0x03b0,   0x03b1,   0x03b2,	 0x03b3,	  
   0x03b4,   0x03b5,   0x03b8,	 0x03b9,   0x03ba,   0x03bb,   0x03bb,
};



u8 ZEBRA2_CCK_OFDM_GAIN_SETTING[]={
        0x00,0x01,0x02,0x03,0x04,0x05,
        0x06,0x07,0x08,0x09,0x0a,0x0b,
        0x0c,0x0d,0x0e,0x0f,0x10,0x11,
        0x12,0x13,0x14,0x15,0x16,0x17,
        0x18,0x19,0x1a,0x1b,0x1c,0x1d,
        0x1e,0x1f,0x20,0x21,0x22,0x23,
};

#if 0
u8 rtl8225_agc[]={
	0x9e,0x9e,0x9e,0x9e,0x9e,0x9e,0x9e,0x9e,0x9d,0x9c,0x9b,0x9a,0x99,0x98,0x97,0x96,
	0x95,0x94,0x93,0x92,0x91,0x90,0x8f,0x8e,0x8d,0x8c,0x8b,0x8a,0x89,0x88,0x87,0x86,
	0x85,0x84,0x83,0x82,0x81,0x80,0x3f,0x3e,0x3d,0x3c,0x3b,0x3a,0x39,0x38,0x37,0x36,
	0x35,0x34,0x33,0x32,0x31,0x30,0x2f,0x2e,0x2d,0x2c,0x2b,0x2a,0x29,0x28,0x27,0x26,
	0x25,0x24,0x23,0x22,0x21,0x20,0x1f,0x1e,0x1d,0x1c,0x1b,0x1a,0x19,0x18,0x17,0x16,
	0x15,0x14,0x13,0x12,0x11,0x10,0x0f,0x0e,0x0d,0x0c,0x0b,0x0a,0x09,0x08,0x07,0x06,
	0x05,0x04,0x03,0x02,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
	0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
};
#endif

u8 rtl8225z2_tx_power_ofdm[]={
	0x42,0x00,0x40,0x00,0x40
};


u8 rtl8225z2_tx_power_cck_ch14[]={
	0x36,0x35,0x2e,0x1b,0x00,0x00,0x00,0x00
};

u8 rtl8225z2_tx_power_cck_B[] = {
	0x30,0x2f,0x29,0x21,0x19,0x10,0x08,0x04,
};

u8 rtl8225z2_tx_power_cck_A[] = {
	0x33,0x32,0x2b,0x23,0x1a,0x11,0x08,0x04,
};

u8 rtl8225z2_tx_power_cck[]={
	0x36,0x35,0x2e,0x25,0x1c,0x12,0x09,0x04
};


void rtl8225z2_set_gain(struct ieee80211_hw *dev, short gain)
{
	u8* rtl8225_gain;
	// struct rtl8186_wlan_priv *priv = dev->priv;
	
	u8 mode = rtl8186_wlan_get_mode(dev);
	if(mode == IEEE_B || mode == IEEE_G)
		rtl8225_gain = rtl8225z2_gain_bg;
	else
		rtl8225_gain = rtl8225z2_gain_a;
		
        write_phy_ofdm(dev, 0x0b, rtl8225_gain[gain * 3]);
        write_phy_ofdm(dev, 0x1b, rtl8225_gain[gain * 3 + 1]);
        write_phy_ofdm(dev, 0x1d, rtl8225_gain[gain * 3 + 2]);
	write_phy_ofdm(dev, 0x21, 0x37);

}


void rtl8225z2_rf_close(struct ieee80211_hw *dev)
{

	write_rtl8225(dev, 0x4, 0x1f);   
	
	force_pci_posting(dev);
	mdelay(1);
	
	rtl8180_set_anaparam(dev, RTL8225Z2_ANAPARAM_OFF);
	rtl8185_set_anaparam2(dev, RTL8225Z2_ANAPARAM2_OFF);
}

void rtl8225z2_SetTXPowerLevel(struct ieee80211_hw *dev, short ch)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	
	int i;
	u8 power;
	u8 *cck_power_table;
	u8 max_cck_power_level;
	u8 max_ofdm_power_level;
	u8 min_ofdm_power_level;	
	u8 cck_power_level = 0xff & correct_tx_power_cck[ch];
	u8 ofdm_power_level = 0xff & correct_tx_power_ofdm[ch];

	// return; // MYHACKS
		
	max_cck_power_level = 15;
	max_ofdm_power_level = 25; 
	min_ofdm_power_level = 10;
	
	if(ch == 14) {
		cck_power_table = rtl8225z2_tx_power_cck_ch14;
	} else {
		if(cck_power_level == 12) {
			cck_power_table = rtl8225z2_tx_power_cck_B;
		} else if(cck_power_level == 13) {
			cck_power_table = rtl8225z2_tx_power_cck_A;
		} else {
			cck_power_table = rtl8225z2_tx_power_cck;
		}
	}
	
	for(i=0;i<8;i++){
	
		power = cck_power_table[i];
		write_phy_cck(dev, 0x44 + i, power);
	}
	
	if(cck_power_level > 35) {
		cck_power_level = 35; 
	} else if((cck_power_level == 12)||(cck_power_level == 13)||(cck_power_level == 14)){
		cck_power_level = 12;
	} 

	if(cck_power_level >= 15) {
		cck_power_level = cck_power_level-2;			
	}

	// MYHACKS
	// cck_power_level = 10;
	// ofdm_power_level = 10;
	
	write_nic_byte(priv, CCK_TXAGC, ZEBRA2_CCK_OFDM_GAIN_SETTING[cck_power_level]);
	
	force_pci_posting(dev);
	mdelay(1);
	
	if(ofdm_power_level > 35)
		ofdm_power_level = 35;

        write_nic_byte(priv, OFDM_TXAGC, ZEBRA2_CCK_OFDM_GAIN_SETTING[ofdm_power_level]);
	{
		write_phy_ofdm(dev,2,0x62);
		write_phy_ofdm(dev,5,0);
		write_phy_ofdm(dev,6,0x40);
		write_phy_ofdm(dev,7,0);
		write_phy_ofdm(dev,8,0x40);	
	} 	

	printk(KERN_ERR "setting power level: ofdm: %d, cck:%d\n", ofdm_power_level, cck_power_level);

	force_pci_posting(dev);
	mdelay(1);
}


void rtl8225z2_rf_set_chan_internal(struct ieee80211_hw *dev, int ch)
{
	rtl8225z2_SetTXPowerLevel(dev, ch);

	write_rtl8225(dev, 0x7, rtl8225_chan_v2[ch]);

	force_pci_posting(dev);
	mdelay(10);
}


void rtl8225z2_rf_set_chan(struct ieee80211_hw *dev, struct ieee80211_conf *conf) {
	
	int ch = rtl8186_get_channel(conf);	
	rtl8225z2_rf_set_chan_internal(dev, ch);
}


void rtl8225z2_rf_init(struct ieee80211_hw *dev) 
{
	int i;
	short channel = 1;
	u32	data,addr;
	u16	brsr;
	struct rtl8186_wlan_priv *priv = dev->priv;
	
	// priv->chan = channel;
	
	rtl8180_set_anaparam(dev, RTL8225_ANAPARAM_ON);

	rtl8225_host_pci_init(dev);

	write_nic_dword(priv, RF_TIMING, 0x00088008);
	
	brsr = read_nic_word(priv, BRSR);
	
	write_nic_word(priv, BRSR, 0xffff); 


	write_nic_dword(priv, RF_PARA, 0x100044);
	
	#if 1  
	rtl8180_set_mode(dev, EPROM_CMD_CONFIG);
	write_nic_byte(priv, CONFIG3, 0x44);
	rtl8180_set_mode(dev, EPROM_CMD_NORMAL);
	#endif
	
	
	rtl8185_rf_pins_enable(dev);

	write_rtl8225(dev, 0x0, 0x0b7); mdelay(1);
	write_rtl8225(dev, 0x1, 0xee0); mdelay(1);
	write_rtl8225(dev, 0x2, 0x44d); mdelay(1);
	write_rtl8225(dev, 0x3, 0x441); mdelay(1);
	write_rtl8225(dev, 0x4, 0x8c3);mdelay(1);
	
	write_rtl8225(dev, 0x5, 0xc72);mdelay(1);
	write_rtl8225(dev, 0x6, 0xe6);  mdelay(1);
	write_rtl8225(dev, 0x7, 0x82a);  mdelay(1);
	write_rtl8225(dev, 0x8, 0x3f);  mdelay(1);
	write_rtl8225(dev, 0x9, 0x335);  mdelay(1);
	write_rtl8225(dev, 0xa, 0x9d4);  mdelay(1);

	write_rtl8225(dev, 0xb, 0x7bb);  mdelay(1);
	write_rtl8225(dev, 0xc, 0x850);  mdelay(1);
	write_rtl8225(dev, 0xd, 0xcdf);   mdelay(1);
	write_rtl8225(dev, 0xe, 0x2b);  mdelay(1);

	write_rtl8225(dev, 0xf, 0x114); 
	
	mdelay(100);
	
        data = read_rtl8225(dev, 6);
        if (!(data&0x00000080))
        {
                write_rtl8225(dev, 0x02, 0x0c4d);
                force_pci_posting(priv); mdelay(200);
                write_rtl8225(dev, 0x02, 0x044d);
                force_pci_posting(priv); mdelay(100);
                data = read_rtl8225(dev, 6);
                if (!(data&0x00000080))
                        {
                        }
        }
	
	
	write_rtl8225(dev, 0x0, 0x1b7);
	write_rtl8225(dev, 0x3, 0x002);	 
	write_rtl8225(dev, 0x5, 0x004);
	
	for(i=0;i<95;i++){
		write_rtl8225(dev, 0x1, (u8)(i+1));
		
		#if 0
		if(priv->phy_ver == 1) 
			write_rtl8225(dev, 0x2, rtl8225a_rxgain[i]);
		else
		#endif
		
		write_rtl8225(dev, 0x2, rtl8225z2_rxgain[i]);
	}
	write_rtl8225(dev, 0x0, 0xb7);
	mdelay(3000);

	write_rtl8225(dev, 0x2, 0xc4d);
	
	mdelay(200);
	
	write_rtl8225(dev, 0x2, 0x44d);
	
	mdelay(100);

	write_rtl8225(dev, 0x00, 0x02bf);
	write_rtl8225(dev, 0xff, 0xffff);

	
	
	rtl8185_rf_pins_enable(dev);
	for(i=0;i<128;i++){
			data = rtl8225_agc_v2[i];
			write_phy_ofdm(dev, 0xb, data);
			mdelay(1);
	
			addr = i + 0x80; 
			write_phy_ofdm(dev, 0xa, addr);
			mdelay(1);
	}
#if 0	
	for(i=0;i<128;i++){
		write_phy_ofdm(dev, 0xb, rtl8225_agc_v2[i]);
		
		mdelay(1); 
		write_phy_ofdm(dev, 0xa, (u8)i+ 0x80);
	
		mdelay(1); 
	}
#endif
		
	force_pci_posting(priv);
	mdelay(1);
	
	write_phy_ofdm(dev, 0x0, 0x1); mdelay(1);
	write_phy_ofdm(dev, 0x1, 0x2); mdelay(1);
	write_phy_ofdm(dev, 0x2, 0x62); mdelay(1);
	write_phy_ofdm(dev, 0x3, 0x0); mdelay(1);
	write_phy_ofdm(dev, 0x4, 0x0); mdelay(1);
	write_phy_ofdm(dev, 0x5, 0x0); mdelay(1);
	write_phy_ofdm(dev, 0x6, 0x40); mdelay(1);
	write_phy_ofdm(dev, 0x7, 0x0); mdelay(1);
	write_phy_ofdm(dev, 0x8, 0x40); mdelay(1);
	write_phy_ofdm(dev, 0x9, 0xfe); mdelay(1);

	write_phy_ofdm(dev, 0xa, 0x9); mdelay(1);
	write_phy_ofdm(dev, 0x18, 0xef); mdelay(1);
	write_phy_ofdm(dev, 0xb, 0x80); mdelay(1);
	write_phy_ofdm(dev, 0xc, 0x1);mdelay(1);

	write_phy_ofdm(dev, 0xd, 0x43); 
		
	write_phy_ofdm(dev, 0xe, 0xd3);mdelay(1);

	
	#if 0
	if(priv->card_8185 == 1){
		if(priv->card_8185_Bversion)
			write_phy_ofdm(dev, 0xf, 0x20);
		else
			write_phy_ofdm(dev, 0xf, 0x28);
	}else{
	#endif
	write_phy_ofdm(dev, 0xf, 0x38);mdelay(1);
	
	write_phy_ofdm(dev, 0x10, 0x84);mdelay(1);
	
	write_phy_ofdm(dev, 0x11, 0x06);mdelay(1);

	
	write_phy_ofdm(dev, 0x12, 0x20);mdelay(1);

	write_phy_ofdm(dev, 0x13, 0x20);mdelay(1);

#if 0
	}else{
		write_phy_ofdm(dev, 0x12, 0x0);
		write_phy_ofdm(dev, 0x13, 0x0);
	}
#endif
	write_phy_ofdm(dev, 0x14, 0x0); mdelay(1);
	write_phy_ofdm(dev, 0x15, 0x40); mdelay(1);
	write_phy_ofdm(dev, 0x16, 0x0); mdelay(1);
	write_phy_ofdm(dev, 0x17, 0x40); mdelay(1);
	
	
	write_phy_ofdm(dev, 0x18, 0xef);mdelay(1);
	write_phy_ofdm(dev, 0x19, 0x19); mdelay(1);
	write_phy_ofdm(dev, 0x1a, 0x20); mdelay(1);
	write_phy_ofdm(dev, 0x1b, 0x11);mdelay(1);
	
	write_phy_ofdm(dev, 0x1c, 0x4);mdelay(1);

	write_phy_ofdm(dev, 0x1d, 0xc5);mdelay(1); 
	
	write_phy_ofdm(dev, 0x1e, 0xb3);mdelay(1);
	write_phy_ofdm(dev, 0x1f, 0x75);mdelay(1);

	
	write_phy_ofdm(dev, 0x20, 0x1f);mdelay(1);
	write_phy_ofdm(dev, 0x21, 0x27);mdelay(1);
	write_phy_ofdm(dev, 0x22, 0x16);mdelay(1);

	write_phy_ofdm(dev, 0x23, 0x80);mdelay(1); 
	
	write_phy_ofdm(dev, 0x24, 0x46); mdelay(1);
	write_phy_ofdm(dev, 0x25, 0x20); mdelay(1);
	write_phy_ofdm(dev, 0x26, 0x90); mdelay(1);
	write_phy_ofdm(dev, 0x27, 0x88); mdelay(1);

	write_phy_cck(dev, 0x0, 0x98); mdelay(1);
	write_phy_cck(dev, 0x3, 0x20); mdelay(1);
	write_phy_cck(dev, 0x4, 0x7e); mdelay(1);
	write_phy_cck(dev, 0x5, 0x12); mdelay(1);
	write_phy_cck(dev, 0x6, 0xfc); mdelay(1);
	write_phy_cck(dev, 0x7, 0x78);mdelay(1);

	write_phy_cck(dev, 0x8, 0x2e);mdelay(1);

	write_phy_cck(dev, 0x10, 0x93); mdelay(1);
	write_phy_cck(dev, 0x11, 0x88); mdelay(1);
	write_phy_cck(dev, 0x12, 0x47); mdelay(1);
#if 0
	if(priv->card_8185 == 1 && priv->card_8185_Bversion)
		write_phy_cck(dev, 0x13, 0x98); 
	else
#endif
	write_phy_cck(dev, 0x13, 0xd0); 
		
	write_phy_cck(dev, 0x19, 0x0);
	write_phy_cck(dev, 0x1a, 0xa0);
	write_phy_cck(dev, 0x1b, 0x8);
	write_phy_cck(dev, 0x40, 0x86); 
	
	write_phy_cck(dev, 0x41, 0x8a);mdelay(1);

	
	write_phy_cck(dev, 0x42, 0x15); mdelay(1);
	write_phy_cck(dev, 0x43, 0x18); mdelay(1);
	
	
	write_phy_cck(dev, 0x44, 0x36); mdelay(1);
	write_phy_cck(dev, 0x45, 0x35); mdelay(1);
	write_phy_cck(dev, 0x46, 0x2e); mdelay(1);
	write_phy_cck(dev, 0x47, 0x25); mdelay(1);
	write_phy_cck(dev, 0x48, 0x1c); mdelay(1);
	write_phy_cck(dev, 0x49, 0x12); mdelay(1);
	write_phy_cck(dev, 0x4a, 0x9); mdelay(1);
	write_phy_cck(dev, 0x4b, 0x4); mdelay(1);
	write_phy_cck(dev, 0x4c, 0x5);mdelay(1);


	write_nic_byte(priv, 0x5b, 0x0d); mdelay(1);

	// rtl8225z2_rf_set_mode(dev); // TODO MY HACK
	
	rtl8225z2_SetTXPowerLevel(dev, channel);
	
	write_phy_cck(dev, 0x10, 0x9b); mdelay(1); 
	write_phy_ofdm(dev, 0x26, 0x90); mdelay(1); 
	
	rtl8185_tx_antenna(dev, 0);// MYHACKS 0x3); 
	
	write_nic_dword(priv, 0x94, 0x15c00002);
	rtl8185_rf_pins_enable(dev);
	
	// rtl8225_rf_set_chan(dev, channel);
	rtl8225z2_rf_set_chan_internal(dev, channel);
}

void rtl8225z2_rf_set_mode(struct ieee80211_hw *dev) 
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	
	if(rtl8186_wlan_get_mode(dev) == IEEE_A)
	{
		write_rtl8225(dev, 0x5, 0x1865);
		write_nic_dword(priv, RF_PARA, 0x10084);
		write_nic_dword(priv, RF_TIMING, 0xa8008);
		write_phy_ofdm(dev, 0x0, 0x0);
		write_phy_ofdm(dev, 0xa, 0x6);
		write_phy_ofdm(dev, 0xb, 0x99);
		write_phy_ofdm(dev, 0xf, 0x20);
		write_phy_ofdm(dev, 0x11, 0x7);
		
		rtl8225z2_set_gain(dev,4);
		
		write_phy_ofdm(dev,0x15, 0x40);
		write_phy_ofdm(dev,0x17, 0x40);
	
		write_nic_dword(priv, 0x94,0x10000000);
	}else{
	
		write_rtl8225(dev, 0x5, 0x1864);
		write_nic_dword(priv, RF_PARA, 0x10044);
		write_nic_dword(priv, RF_TIMING, 0xa8008);
		write_phy_ofdm(dev, 0x0, 0x1);
		write_phy_ofdm(dev, 0xa, 0x6);
		write_phy_ofdm(dev, 0xb, 0x99);
		write_phy_ofdm(dev, 0xf, 0x20);
		write_phy_ofdm(dev, 0x11, 0x7);
		
		rtl8225z2_set_gain(dev,4);
		
		write_phy_ofdm(dev,0x15, 0x40);
		write_phy_ofdm(dev,0x17, 0x40);
	
		write_nic_dword(priv, 0x94,0x04000002);
	}
}
