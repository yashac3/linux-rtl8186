#include <linux/delay.h>
#include <net/mac80211.h>

#include "rtl8186-wlan.h"
#include "rtl8225.h"

void rtl8225_write(struct ieee80211_hw *dev, u8 addr, u16 data)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	u16 reg80, reg84, reg82;
	u32 bangdata;
	int i;

	bangdata = (data << 4) | (addr & 0xf);

	reg80 = rtl818x_ioread16(priv, &priv->map->RFPinsOutput) & 0xfff3;
	reg82 = rtl818x_ioread16(priv, &priv->map->RFPinsEnable);

	rtl818x_iowrite16(priv, &priv->map->RFPinsEnable, reg82 | 0x7);

	reg84 = rtl818x_ioread16(priv, &priv->map->RFPinsSelect);
	rtl818x_iowrite16(priv, &priv->map->RFPinsSelect, reg84 | 0x7 | 0x400);
	rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
	udelay(10);

	rtl818x_iowrite16(priv, &priv->map->RFPinsOutput, reg80 | (1 << 2));
	rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
	udelay(2);
	rtl818x_iowrite16(priv, &priv->map->RFPinsOutput, reg80);
	rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
	udelay(10);

	for (i = 15; i >= 0; i--) {
		u16 reg = reg80;

		if (bangdata & (1 << i))
			reg |= 1;

		if (i & 1)
			rtl818x_iowrite16(priv, &priv->map->RFPinsOutput, reg);

		rtl818x_iowrite16(priv, &priv->map->RFPinsOutput, reg | (1 << 1));
		rtl818x_iowrite16(priv, &priv->map->RFPinsOutput, reg | (1 << 1));

		if (!(i & 1))
			rtl818x_iowrite16(priv, &priv->map->RFPinsOutput, reg);
	}

	rtl818x_iowrite16(priv, &priv->map->RFPinsOutput, reg80 | (1 << 2));
	rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
	udelay(10);

	rtl818x_iowrite16(priv, &priv->map->RFPinsOutput, reg80 | (1 << 2));
	rtl818x_iowrite16(priv, &priv->map->RFPinsSelect, reg84 | 0x400);
	rtl818x_iowrite16(priv, &priv->map->RFPinsEnable, 0x1FFF);
}

u16 rtl8225_read(struct ieee80211_hw *dev, u8 addr)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	u16 reg80, reg82, reg84, out;
	int i;

	reg80 = rtl818x_ioread16(priv, &priv->map->RFPinsOutput);
	reg82 = rtl818x_ioread16(priv, &priv->map->RFPinsEnable);
	reg84 = rtl818x_ioread16(priv, &priv->map->RFPinsSelect) | 0x400;

	reg80 &= ~0xF;

	rtl818x_iowrite16(priv, &priv->map->RFPinsEnable, reg82 | 0x000F);
	rtl818x_iowrite16(priv, &priv->map->RFPinsSelect, reg84 | 0x000F);

	rtl818x_iowrite16(priv, &priv->map->RFPinsOutput, reg80 | (1 << 2));
	rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
	udelay(4);
	rtl818x_iowrite16(priv, &priv->map->RFPinsOutput, reg80);
	rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
	udelay(5);

	for (i = 4; i >= 0; i--) {
		u16 reg = reg80 | ((addr >> i) & 1);

		if (!(i & 1)) {
			rtl818x_iowrite16(priv, &priv->map->RFPinsOutput, reg);
			rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
			udelay(1);
		}

		rtl818x_iowrite16(priv, &priv->map->RFPinsOutput,
				  reg | (1 << 1));
		rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
		udelay(2);
		rtl818x_iowrite16(priv, &priv->map->RFPinsOutput,
				  reg | (1 << 1));
		rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
		udelay(2);

		if (i & 1) {
			rtl818x_iowrite16(priv, &priv->map->RFPinsOutput, reg);
			rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
			udelay(1);
		}
	}

	rtl818x_iowrite16(priv, &priv->map->RFPinsEnable, 0x000E);
	rtl818x_iowrite16(priv, &priv->map->RFPinsSelect, 0x040E);
	rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
	rtl818x_iowrite16(priv, &priv->map->RFPinsOutput,
			  reg80 | (1 << 3) | (1 << 1));
	rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
	udelay(2);
	rtl818x_iowrite16(priv, &priv->map->RFPinsOutput,
			  reg80 | (1 << 3));
	rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
	udelay(2);
	rtl818x_iowrite16(priv, &priv->map->RFPinsOutput,
			  reg80 | (1 << 3));
	rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
	udelay(2);

	out = 0;
	for (i = 11; i >= 0; i--) {
		rtl818x_iowrite16(priv, &priv->map->RFPinsOutput,
				  reg80 | (1 << 3));
		rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
		udelay(1);
		rtl818x_iowrite16(priv, &priv->map->RFPinsOutput,
				  reg80 | (1 << 3) | (1 << 1));
		rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
		udelay(2);
		rtl818x_iowrite16(priv, &priv->map->RFPinsOutput,
				  reg80 | (1 << 3) | (1 << 1));
		rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
		udelay(2);
		rtl818x_iowrite16(priv, &priv->map->RFPinsOutput,
				  reg80 | (1 << 3) | (1 << 1));
		rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
		udelay(2);

		if (rtl818x_ioread16(priv, &priv->map->RFPinsInput) & (1 << 1))
			out |= 1 << i;

		rtl818x_iowrite16(priv, &priv->map->RFPinsOutput,
				  reg80 | (1 << 3));
		rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
		udelay(2);
	}

	rtl818x_iowrite16(priv, &priv->map->RFPinsOutput,
			  reg80 | (1 << 3) | (1 << 2));
	rtl818x_ioread8(priv, &priv->map->EEPROM_CMD);
	udelay(2);

	rtl818x_iowrite16(priv, &priv->map->RFPinsEnable, reg82);
	rtl818x_iowrite16(priv, &priv->map->RFPinsSelect, reg84);
	rtl818x_iowrite16(priv, &priv->map->RFPinsOutput, 0x03A0);

	return out;
}


const u8 rtl8225_agc[] = {
	0x9e, 0x9e, 0x9e, 0x9e, 0x9e, 0x9e, 0x9e, 0x9e,
	0x9d, 0x9c, 0x9b, 0x9a, 0x99, 0x98, 0x97, 0x96,
	0x95, 0x94, 0x93, 0x92, 0x91, 0x90, 0x8f, 0x8e,
	0x8d, 0x8c, 0x8b, 0x8a, 0x89, 0x88, 0x87, 0x86,
	0x85, 0x84, 0x83, 0x82, 0x81, 0x80, 0x3f, 0x3e,
	0x3d, 0x3c, 0x3b, 0x3a, 0x39, 0x38, 0x37, 0x36,
	0x35, 0x34, 0x33, 0x32, 0x31, 0x30, 0x2f, 0x2e,
	0x2d, 0x2c, 0x2b, 0x2a, 0x29, 0x28, 0x27, 0x26,
	0x25, 0x24, 0x23, 0x22, 0x21, 0x20, 0x1f, 0x1e,
	0x1d, 0x1c, 0x1b, 0x1a, 0x19, 0x18, 0x17, 0x16,
	0x15, 0x14, 0x13, 0x12, 0x11, 0x10, 0x0f, 0x0e,
	0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x07, 0x06,
	0x05, 0x04, 0x03, 0x02, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01
};


const u32 rtl8225_chan[] = {
	0x085c, 0x08dc, 0x095c, 0x09dc, 0x0a5c, 0x0adc, 0x0b5c,
	0x0bdc, 0x0c5c, 0x0cdc, 0x0d5c, 0x0ddc, 0x0e5c, 0x0f72
};


static void rtl8225_rf_stop(struct ieee80211_hw *dev)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	u8 reg;

	printk(KERN_ERR "rtl8225_rf_stop()\n");

	rtl8225_write(dev, 0x4, 0x1f); msleep(1);

	if (priv->is_mmio) // TODO what's this?
		return;

	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
	reg = rtl818x_ioread8(priv, &priv->map->CONFIG3);
	rtl818x_iowrite8(priv, &priv->map->CONFIG3, reg | RTL818X_CONFIG3_ANAPARAM_WRITE);
	rtl818x_iowrite32(priv, &priv->map->ANAPARAM2, RTL8225_ANAPARAM2_OFF);
	rtl818x_iowrite32(priv, &priv->map->ANAPARAM, RTL8225_ANAPARAM_OFF);
	rtl818x_iowrite8(priv, &priv->map->CONFIG3, reg & ~RTL818X_CONFIG3_ANAPARAM_WRITE);
	rtl818x_iowrite8(priv, &priv->map->EEPROM_CMD, RTL818X_EEPROM_CMD_NORMAL);
}

void rtl8225_rf_set_channel(struct ieee80211_hw *dev,
				   struct ieee80211_conf *conf)
{
	// struct rtl8186_wlan_priv *priv = dev->priv;
	int chan =
		ieee80211_frequency_to_channel(conf->chandef.chan->center_freq);

	printk(KERN_ERR "rtl8225_rf_set_channel()\n");

	// if (priv->rf->init == rtl8225_rf_init)
	rtl8225_rf_set_tx_power(dev, chan);
	// else
		// rtl8225z2_rf_set_tx_power(dev, chan);

	rtl8225_write(dev, 0x7, rtl8225_chan[chan - 1]);
	msleep(10);
}

static const struct rtl818x_rf_ops rtl8225_ops = {
	.name		= "rtl8225",
	.init		= rtl8225_rf_init,
	.stop		= rtl8225_rf_stop,
	.set_chan	= rtl8225_rf_set_channel,
};

static const struct rtl818x_rf_ops rtl8225z2_ops = {
	.name		= "rtl8225z2",
	.init		= rtl8225z2_rf_init,
	.stop		= rtl8225z2_rf_close,
	.set_chan	= rtl8225z2_rf_set_chan,
};

const struct rtl818x_rf_ops * rtl8186_wlan_detect_rf(struct ieee80211_hw *dev)
{
	struct rtl8186_wlan_priv *priv = dev->priv;
	u16 reg8, reg9;

	rtl818x_iowrite16(priv, &priv->map->RFPinsOutput, 0x0480);
	rtl818x_iowrite16(priv, &priv->map->RFPinsSelect, 0x0488);
	rtl818x_iowrite16(priv, &priv->map->RFPinsEnable, 0x1FFF);
	msleep(100);

	rtl8225_write(dev, 0, 0x1B7);

	reg8 = rtl8225_read(dev, 8);
	reg9 = rtl8225_read(dev, 9);

	rtl8225_write(dev, 0, 0x0B7);

	if (reg8 == 0x588 && reg9 == 0x700) {
		return &rtl8225z2_ops;
	} else {
		return &rtl8225_ops;
	}
}
