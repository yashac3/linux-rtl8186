#include <linux/delay.h>
#include <net/mac80211.h>

#include "rtl8186-wlan.h"
#include "rtl8225.h"


static const u16 rtl8225bcd_rxgain[] = {
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
	0x07aa, 0x07ab, 0x07ac, 0x07ad, 0x07b0, 0x07b1, 0x07b2, 0x07b3,
	0x07b4, 0x07b5, 0x07b8, 0x07b9, 0x07ba, 0x07bb, 0x07bb
};

static const u8 rtl8225_gain[] = {
	0x23, 0x88, 0x7c, 0xa5, /* -82dbm */
	0x23, 0x88, 0x7c, 0xb5, /* -82dbm */
	0x23, 0x88, 0x7c, 0xc5, /* -82dbm */
	0x33, 0x80, 0x79, 0xc5, /* -78dbm */
	0x43, 0x78, 0x76, 0xc5, /* -74dbm */
	0x53, 0x60, 0x73, 0xc5, /* -70dbm */
	0x63, 0x58, 0x70, 0xc5, /* -66dbm */
};

static const u8 rtl8225_threshold[] = {
	0x8d, 0x8d, 0x8d, 0x8d, 0x9d, 0xad, 0xbd
};

static const u8 rtl8225_tx_gain_cck_ofdm[] = {
	0x02, 0x06, 0x0e, 0x1e, 0x3e, 0x7e
};

static const u8 rtl8225_tx_power_cck[] = {
	0x18, 0x17, 0x15, 0x11, 0x0c, 0x08, 0x04, 0x02,
	0x1b, 0x1a, 0x17, 0x13, 0x0e, 0x09, 0x04, 0x02,
	0x1f, 0x1e, 0x1a, 0x15, 0x10, 0x0a, 0x05, 0x02,
	0x22, 0x21, 0x1d, 0x18, 0x11, 0x0b, 0x06, 0x02,
	0x26, 0x25, 0x21, 0x1b, 0x14, 0x0d, 0x06, 0x03,
	0x2b, 0x2a, 0x25, 0x1e, 0x16, 0x0e, 0x07, 0x03
};

static const u8 rtl8225_tx_power_cck_ch14[] = {
	0x18, 0x17, 0x15, 0x0c, 0x00, 0x00, 0x00, 0x00,
	0x1b, 0x1a, 0x17, 0x0e, 0x00, 0x00, 0x00, 0x00,
	0x1f, 0x1e, 0x1a, 0x0f, 0x00, 0x00, 0x00, 0x00,
	0x22, 0x21, 0x1d, 0x11, 0x00, 0x00, 0x00, 0x00,
	0x26, 0x25, 0x21, 0x13, 0x00, 0x00, 0x00, 0x00,
	0x2b, 0x2a, 0x25, 0x15, 0x00, 0x00, 0x00, 0x00
};

static const u8 rtl8225_tx_power_ofdm[] = {
	0x80, 0x90, 0xa2, 0xb5, 0xcb, 0xe4
};


void rtl8225_rf_set_tx_power(struct ieee80211_hw *dev, int channel)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	u8 cck_power, ofdm_power;
	const u8 *tmp;
	u32 reg;
	int i;

	cck_power = priv->channels[channel - 1].hw_value & 0xFF;
	ofdm_power = priv->channels[channel - 1].hw_value >> 8;

	cck_power = min(cck_power, (u8)35);
	ofdm_power = min(ofdm_power, (u8)35);

	rtl818x_iowrite8(priv, &priv->map->TX_GAIN_CCK,
			 rtl8225_tx_gain_cck_ofdm[cck_power / 6] >> 1);

	if (channel == 14)
		tmp = &rtl8225_tx_power_cck_ch14[(cck_power % 6) * 8];
	else
		tmp = &rtl8225_tx_power_cck[(cck_power % 6) * 8];

	for (i = 0; i < 8; i++)
		rtl8225_write_phy_cck(dev, 0x44 + i, *tmp++);

	msleep(1); /* FIXME: optional? */

	/* TODO: use set_anaparam2 dev.c_func*/
	/* anaparam2 on */
	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
	reg = rtl818x_ioread8(priv, &priv->map->CONFIG3);
	rtl818x_iowrite8(priv, &priv->map->CONFIG3, reg | RTL818X_CONFIG3_ANAPARAM_WRITE);
	rtl818x_iowrite32(priv, &priv->map->ANAPARAM2, RTL8225_ANAPARAM2_ON);
	rtl818x_iowrite8(priv, &priv->map->CONFIG3, reg & ~RTL818X_CONFIG3_ANAPARAM_WRITE);
	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_NORMAL);

	rtl818x_iowrite8(priv, &priv->map->TX_GAIN_OFDM,
			 rtl8225_tx_gain_cck_ofdm[ofdm_power/6] >> 1);

	tmp = &rtl8225_tx_power_ofdm[ofdm_power % 6];

	rtl8225_write_phy_ofdm(dev, 5, *tmp);
	rtl8225_write_phy_ofdm(dev, 7, *tmp);

	msleep(1);
}

void rtl8225_rf_init(struct ieee80211_hw *dev)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	int i;

	rtl8186_wlan_set_anaparam(priv, RTL8225_ANAPARAM_ON);

	/* host_pci_init */
	rtl818x_iowrite16(priv, &priv->map->RFPinsOutput, 0x0480);
	rtl818x_iowrite16(priv, &priv->map->RFPinsEnable, 0x1FFF);
	rtl818x_iowrite16(priv, &priv->map->RFPinsSelect, 0x0488);
	rtl818x_iowrite8(priv, &priv->map->GP_ENABLE, 0);
	rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
	msleep(200);	/* FIXME: ehh?? */
	rtl818x_iowrite8(priv, &priv->map->GP_ENABLE, 0xFF & ~(1 << 6));

	rtl818x_iowrite32(priv, &priv->map->RF_TIMING, 0x000a8008);

	/* TODO: check if we need really to change BRSR to do RF config */
	rtl818x_ioread16(priv, &priv->map->BRSR);
	rtl818x_iowrite16(priv, &priv->map->BRSR, 0xFFFF);
	rtl818x_iowrite32(priv, &priv->map->RF_PARA, 0x00100044);
	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
	rtl818x_iowrite8(priv, &priv->map->CONFIG3, 0x44);
	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_NORMAL);

	rtl8225_write(dev, 0x0, 0x067);
	rtl8225_write(dev, 0x1, 0xFE0);
	rtl8225_write(dev, 0x2, 0x44D);
	rtl8225_write(dev, 0x3, 0x441);
	rtl8225_write(dev, 0x4, 0x8BE);
	rtl8225_write(dev, 0x5, 0xBF0);		/* TODO: minipci */
	rtl8225_write(dev, 0x6, 0xAE6);
	rtl8225_write(dev, 0x7, rtl8225_chan[0]);
	rtl8225_write(dev, 0x8, 0x01F);
	rtl8225_write(dev, 0x9, 0x334);
	rtl8225_write(dev, 0xA, 0xFD4);
	rtl8225_write(dev, 0xB, 0x391);
	rtl8225_write(dev, 0xC, 0x050);
	rtl8225_write(dev, 0xD, 0x6DB);
	rtl8225_write(dev, 0xE, 0x029);
	rtl8225_write(dev, 0xF, 0x914); msleep(1);

	rtl8225_write(dev, 0x2, 0xC4D); msleep(100);

	rtl8225_write(dev, 0x0, 0x127);

	for (i = 0; i < ARRAY_SIZE(rtl8225bcd_rxgain); i++) {
		rtl8225_write(dev, 0x1, i + 1);
		rtl8225_write(dev, 0x2, rtl8225bcd_rxgain[i]);
	}

	rtl8225_write(dev, 0x0, 0x027);
	rtl8225_write(dev, 0x0, 0x22F);
	rtl818x_iowrite16(priv, &priv->map->RFPinsEnable, 0x1FFF);

	for (i = 0; i < ARRAY_SIZE(rtl8225_agc); i++) {
		rtl8225_write_phy_ofdm(dev, 0xB, rtl8225_agc[i]);
		msleep(1);
		rtl8225_write_phy_ofdm(dev, 0xA, 0x80 + i);
		msleep(1);
	}

	msleep(1);

	rtl8225_write_phy_ofdm(dev, 0x00, 0x01); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x01, 0x02); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x02, 0x62); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x03, 0x00); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x04, 0x00); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x05, 0x00); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x06, 0x00); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x07, 0x00); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x08, 0x00); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x09, 0xfe); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x0a, 0x09); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x0b, 0x80); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x0c, 0x01); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x0e, 0xd3); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x0f, 0x38); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x10, 0x84); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x11, 0x03); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x12, 0x20); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x13, 0x20); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x14, 0x00); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x15, 0x40); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x16, 0x00); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x17, 0x40); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x18, 0xef); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x19, 0x19); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x1a, 0x20); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x1b, 0x76); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x1c, 0x04); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x1e, 0x95); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x1f, 0x75); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x20, 0x1f); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x21, 0x27); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x22, 0x16); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x24, 0x46); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x25, 0x20); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x26, 0x90); msleep(1);
	rtl8225_write_phy_ofdm(dev, 0x27, 0x88); msleep(1);

	rtl8225_write_phy_cck(dev, 0x00, 0x98); msleep(1);
	rtl8225_write_phy_cck(dev, 0x03, 0x20); msleep(1);
	rtl8225_write_phy_cck(dev, 0x04, 0x7e); msleep(1);
	rtl8225_write_phy_cck(dev, 0x05, 0x12); msleep(1);
	rtl8225_write_phy_cck(dev, 0x06, 0xfc); msleep(1);
	rtl8225_write_phy_cck(dev, 0x07, 0x78); msleep(1);
	rtl8225_write_phy_cck(dev, 0x08, 0x2e); msleep(1);
	rtl8225_write_phy_cck(dev, 0x10, 0x93); msleep(1);
	rtl8225_write_phy_cck(dev, 0x11, 0x88); msleep(1);
	rtl8225_write_phy_cck(dev, 0x12, 0x47); msleep(1);
	rtl8225_write_phy_cck(dev, 0x13, 0xd0);
	rtl8225_write_phy_cck(dev, 0x19, 0x00);
	rtl8225_write_phy_cck(dev, 0x1a, 0xa0);
	rtl8225_write_phy_cck(dev, 0x1b, 0x08);
	rtl8225_write_phy_cck(dev, 0x40, 0x86);
	rtl8225_write_phy_cck(dev, 0x41, 0x8d); msleep(1);
	rtl8225_write_phy_cck(dev, 0x42, 0x15); msleep(1);
	rtl8225_write_phy_cck(dev, 0x43, 0x18); msleep(1);
	rtl8225_write_phy_cck(dev, 0x44, 0x1f); msleep(1);
	rtl8225_write_phy_cck(dev, 0x45, 0x1e); msleep(1);
	rtl8225_write_phy_cck(dev, 0x46, 0x1a); msleep(1);
	rtl8225_write_phy_cck(dev, 0x47, 0x15); msleep(1);
	rtl8225_write_phy_cck(dev, 0x48, 0x10); msleep(1);
	rtl8225_write_phy_cck(dev, 0x49, 0x0a); msleep(1);
	rtl8225_write_phy_cck(dev, 0x4a, 0x05); msleep(1);
	rtl8225_write_phy_cck(dev, 0x4b, 0x02); msleep(1);
	rtl8225_write_phy_cck(dev, 0x4c, 0x05); msleep(1);

	rtl818x_iowrite8(priv, &priv->map->TESTR, 0x0D); msleep(1);

	rtl8225_rf_set_tx_power(dev, 1);

	/* RX antenna default to A */
	rtl8225_write_phy_cck(dev, 0x10, 0x9b); msleep(1);	/* B: 0xDB */
	rtl8225_write_phy_ofdm(dev, 0x26, 0x90); msleep(1);	/* B: 0x10 */

	rtl818x_iowrite8(priv, &priv->map->TX_ANTENNA, 0x03);	/* B: 0x00 */
	msleep(1);
	rtl818x_iowrite32(priv, (__be32 __iomem *)((void __iomem *)priv->map + 0x94), 0x15c00002);
	rtl818x_iowrite16(priv, &priv->map->RFPinsEnable, 0x1FFF);

	rtl8225_write(dev, 0x0c, 0x50);
	/* set OFDM initial gain */
	rtl8225_write_phy_ofdm(dev, 0x0d, rtl8225_gain[4 * 4]);
	rtl8225_write_phy_ofdm(dev, 0x23, rtl8225_gain[4 * 4 + 1]);
	rtl8225_write_phy_ofdm(dev, 0x1b, rtl8225_gain[4 * 4 + 2]);
	rtl8225_write_phy_ofdm(dev, 0x1d, rtl8225_gain[4 * 4 + 3]);
	/* set CCK threshold */
	rtl8225_write_phy_cck(dev, 0x41, rtl8225_threshold[0]);
}


// void rtl8225_set_initial_gain(struct ieee80211_hw *dev, int initial_gain) {
//     struct r8180_priv *priv = ieee80211_priv(dev);

// 	switch(initial_gain)
// 	{
// 		case 1: /* -82dBm */
// 			rtl8225_write_phy_ofdm(dev, 0x97, 0x26);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0xa4, 0x86);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0x85, 0xfa);    mdelay(1);
// 			break;

// 		case 2: /* -82dBm */
// 			rtl8225_write_phy_ofdm(dev, 0x97, 0x36);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0xa4, 0x86);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0x85, 0xfa);    mdelay(1);
// 			break;

// 		case 3: /* -82dBm */
// 			rtl8225_write_phy_ofdm(dev, 0x97, 0x36);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0xa4, 0x86);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0x85, 0xfb);    mdelay(1);
// 			break;

// 		case 4: /* -78dBm */
// 			rtl8225_write_phy_ofdm(dev, 0x97, 0x46);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0xa4, 0x86);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0x85, 0xfb);    mdelay(1);
// 			break;

// 		case 5: /* -74dBm */
// 			rtl8225_write_phy_ofdm(dev, 0x97, 0x46);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0xa4, 0x96);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0x85, 0xfb);    mdelay(1);
// 			break;

// 		case 6:
// 			rtl8225_write_phy_ofdm(dev, 0x97, 0x56);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0xa4, 0x96);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0x85, 0xfc);    mdelay(1);
// 			break;

// 		case 7:
// 			rtl8225_write_phy_ofdm(dev, 0x97, 0x56);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0xa4, 0xa6);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0x85, 0xfc);    mdelay(1);
// 			break;

// 		case 8:
// 			rtl8225_write_phy_ofdm(dev, 0x97, 0x66);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0xa4, 0xb6);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0x85, 0xfc);    mdelay(1);
// 			break;

// 		default:
// 			rtl8225_write_phy_ofdm(dev, 0x97, 0x26);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0xa4, 0x86);    mdelay(1);
// 			rtl8225_write_phy_ofdm(dev, 0x85, 0xfa);    mdelay(1);
// 			break;
// 	}
// }
