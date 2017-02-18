/**
 * Micrel KSZ8863 I2C driver
 *
 * Copyright (c) 2015-2016 Microchip Technology Inc.
 * Copyright (c) 2010-2015 Micrel, Inc.
 *
 * Copyright 2009 Simtec Electronics
 *	http://www.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#if 0
#define DEBUG
#define DBG
#endif

#ifndef CONFIG_MICREL_SWITCH_EMBEDDED
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/cache.h>
#include <linux/crc32.h>
#include <linux/if_vlan.h>
#include <linux/ip.h>
#include <net/ip.h>
#endif

/* -------------------------------------------------------------------------- */

#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include "ksz_cfg_8863.h"
#ifndef PHY_RESET_NOT
#define PHY_RESET_NOT			PHY_RESET
#endif

/* -------------------------------------------------------------------------- */

#define HW_R(ks, reg)		i2c_rdreg8(ks, reg)
#define HW_W(ks, reg, val)	i2c_wrreg8(ks, reg, val)
#define HW_R8(ks, reg)		i2c_rdreg8(ks, reg)
#define HW_W8(ks, reg, val)	i2c_wrreg8(ks, reg, val)
#define HW_R16(ks, reg)		i2c_rdreg16(ks, reg)
#define HW_W16(ks, reg, val)	i2c_wrreg16(ks, reg, val)
#define HW_R32(ks, reg)		i2c_rdreg32(ks, reg)
#define HW_W32(ks, reg, val)	i2c_wrreg32(ks, reg, val)

#include "ksz_sw_phy.h"

#include "ksz_spi_net.h"

/* -------------------------------------------------------------------------- */

/*
 * I2C register read/write calls.
 *
 * All these calls issue I2C transactions to access the chip's registers. They
 * all require that the necessary lock is held to prevent accesses when the
 * chip is busy transfering packet data (RX/TX FIFO accesses).
 */

/**
 * i2c_wrreg - issue write register command
 * @ks:		The switch device structure.
 * @reg:	The register address.
 * @buf:	The data buffer to write.
 * @txl:	The length of data.
 *
 * This is the low level write call that issues the necessary i2c message(s)
 * to write data to the register specified in @reg.
 */
static void i2c_wrreg(struct sw_priv *ks, u8 reg, void *buf, unsigned txl)
{
	struct i2c_hw_priv *hw_priv = ks->hw_dev;
	struct i2c_msg msg;
	u8 rxd[10];
	struct i2c_client *i2c = hw_priv->i2cdev;
	struct i2c_adapter *adapter = i2c->adapter;

	if (!mutex_is_locked(&ks->lock))
		pr_alert("W not locked\n");

	rxd[0] = reg;
	memcpy(&rxd[1], buf, txl);

	msg.addr = i2c->addr;
	msg.flags = 0;
	msg.len = SW_SIZE + txl;
	msg.buf = rxd;

	if (i2c_transfer(adapter, &msg, 1) != 1)
		pr_alert("i2c_transfer() failed\n");
}

/**
 * i2c_wrreg32 - write 32bit register value to chip
 * @ks:		The switch device structure.
 * @reg:	The register address.
 * @val:	The value to write.
 *
 * Issue a write to put the value @val into the register specified in @reg.
 */
static void i2c_wrreg32(struct sw_priv *ks, u8 reg, unsigned val)
{
	struct i2c_hw_priv *hw_priv = ks->hw_dev;
	int cnt = 4;
	int i = 0;
	u8 *txb = (u8 *) hw_priv->txd;

	while (cnt) {
		txb[i++] = (u8)(val >> (8 * (cnt - 1)));
		cnt--;
	}
	i2c_wrreg(ks, reg, txb, 4);
}

/**
 * i2c_wrreg16 - write 16bit register value to chip
 * @ks:		The switch device structure.
 * @reg:	The register address.
 * @val:	The value to write.
 *
 * Issue a write to put the value @val into the register specified in @reg.
 */
static void i2c_wrreg16(struct sw_priv *ks, u8 reg, unsigned val)
{
	struct i2c_hw_priv *hw_priv = ks->hw_dev;
	int cnt = 2;
	int i = 0;
	u8 *txb = (u8 *) hw_priv->txd;

	while (cnt) {
		txb[i++] = (u8)(val >> (8 * (cnt - 1)));
		cnt--;
	}
	i2c_wrreg(ks, reg, txb, 2);
}

/**
 * i2c_wrreg8 - write 8bit register value to chip
 * @ks:		The switch device structure.
 * @reg:	The register address.
 * @val:	The value to write.
 *
 * Issue a write to put the value @val into the register specified in @reg.
 */
static void i2c_wrreg8(struct sw_priv *ks, u8 reg, unsigned val)
{
	struct i2c_hw_priv *hw_priv = ks->hw_dev;
	u8 *txb = (u8 *) hw_priv->txd;

	*txb = (u8) val;
	i2c_wrreg(ks, reg, txb, 1);
}

/**
 * i2c_rdreg - issue read register command and return the data
 * @ks:		The switch device structure.
 * @reg:	The register address.
 * @rxb:	The RX buffer to return the result into.
 * @rxl:	The length of data expected.
 *
 * This is the low level read call that issues the necessary i2c message(s)
 * to read data from the register specified in @reg.
 */
static void i2c_rdreg(struct sw_priv *ks, u8 reg, void *rxb, unsigned rxl)
{
	struct i2c_hw_priv *hw_priv = ks->hw_dev;
	struct i2c_msg msg[2];
	struct i2c_client *i2c = hw_priv->i2cdev;
	struct i2c_adapter *adapter = i2c->adapter;

	if (!mutex_is_locked(&ks->lock))
		pr_alert("R not locked\n");

	msg[0].addr = i2c->addr;
	msg[0].flags = 0;
	msg[0].len = SW_SIZE;
	msg[0].buf = &reg;

	msg[1].addr = i2c->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = rxl;
	msg[1].buf = rxb;

	if (i2c_transfer(adapter, msg, 2) != 2)
		pr_alert("i2c_transfer() failed\n");
}

/**
 * i2c_rdreg8 - read 8 bit register from device
 * @ks:		The switch device structure.
 * @reg:	The register address.
 *
 * Read a 8bit register from the chip, returning the result.
 */
static u8 i2c_rdreg8(struct sw_priv *ks, u8 reg)
{
	u8 rxb[1];

	i2c_rdreg(ks, reg, rxb, 1);
	return rxb[0];
}

/**
 * i2c_rdreg16 - read 16 bit register from device
 * @ks:		The switch device structure.
 * @reg:	The register address.
 *
 * Read a 16bit register from the chip, returning the result.
 */
static u16 i2c_rdreg16(struct sw_priv *ks, u8 reg)
{
	__le16 rx = 0;

	i2c_rdreg(ks, reg, &rx, 2);
	return be16_to_cpu(rx);
}

/**
 * i2c_rdreg32 - read 32 bit register from device
 * @ks:		The switch device structure.
 * @reg:	The register address.
 *
 * Read a 32bit register from the chip.
 *
 * Note, this read requires the address be aligned to 4 bytes.
 */
static u32 i2c_rdreg32(struct sw_priv *ks, u8 reg)
{
	__le32 rx = 0;

	i2c_rdreg(ks, reg, &rx, 4);
	return be32_to_cpu(rx);
}

/* -------------------------------------------------------------------------- */

#define KSZSW_REGS_SIZE			0x100

static struct sw_regs {
	int start;
	int end;
} sw_regs_range[] = {
	{ 0x000, 0x100 },
	{ 0, 0 }
};

static int check_sw_reg_range(unsigned addr)
{
	struct sw_regs *range = sw_regs_range;

	while (range->end > range->start) {
		if (range->start <= addr && addr < range->end)
			return true;
		range++;
	}
	return false;
}

static struct ksz_sw *get_sw_data(struct device *d)
{
	struct sw_priv *hw_priv = dev_get_drvdata(d);

	return &hw_priv->sw;
}

/* -------------------------------------------------------------------------- */

static u8 sw_r8(struct ksz_sw *sw, unsigned reg)
{
	return HW_R8(sw->dev, reg);
}

static u16 sw_r16(struct ksz_sw *sw, unsigned reg)
{
	return HW_R16(sw->dev, reg);
}

static u32 sw_r32(struct ksz_sw *sw, unsigned reg)
{
	return HW_R32(sw->dev, reg);
}

static void sw_w8(struct ksz_sw *sw, unsigned reg, unsigned val)
{
	HW_W8(sw->dev, reg, val);
}

static void sw_w16(struct ksz_sw *sw, unsigned reg, unsigned val)
{
	HW_W16(sw->dev, reg, val);
}

static void sw_w32(struct ksz_sw *sw, unsigned reg, unsigned val)
{
	HW_W32(sw->dev, reg, val);
}

#ifdef CONFIG_KSZ_STP
static u8 get_port_state(struct net_device *dev, struct net_device **br_dev)
{
	struct net_bridge_port *p;
	u8 state;

	/* This state is not defined in kernel. */
	state = STP_STATE_SIMPLE;
	if (br_port_exists(dev)) {
		p = br_port_get_rcu(dev);
		state = p->state;

		/* Port is under bridge. */
		*br_dev = p->br->dev;
	}
	return state;
}  /* get_port_state */
#endif
/**
 * sw_r_phy - read data from PHY register
 * @sw:		The switch instance.
 * @phy:	PHY address to read.
 * @reg:	PHY register to read.
 * @val:	Buffer to store the read data.
 *
 * This routine reads data from the PHY register.
 */
static void sw_r_phy(struct ksz_sw *sw, u16 phy, u16 reg, u16 *val)
{
	u8 ctrl;
	u8 restart;
	u8 link;
	u8 speed;
	u8 p = phy - 1;
	u16 data = 0;

	switch (reg) {
	case PHY_REG_CTRL:
		port_r(sw, p, P_PHY_CTRL, &ctrl);
		port_r(sw, p, P_NEG_RESTART_CTRL, &restart);
		port_r(sw, p, P_SPEED_STATUS, &speed);
		if (restart & PORT_LOOPBACK)
			data |= PHY_LOOPBACK;
		if (ctrl & PORT_FORCE_100_MBIT)
			data |= PHY_SPEED_100MBIT;
		if (ctrl & PORT_AUTO_NEG_ENABLE)
			data |= PHY_AUTO_NEG_ENABLE;
		if (restart & PORT_POWER_DOWN)
			data |= PHY_POWER_DOWN;
		if (restart & PORT_AUTO_NEG_RESTART)
			data |= PHY_AUTO_NEG_RESTART;
		if (ctrl & PORT_FORCE_FULL_DUPLEX)
			data |= PHY_FULL_DUPLEX;
		if (speed & PORT_HP_MDIX)
			data |= PHY_HP_MDIX;
		if (restart & PORT_FORCE_MDIX)
			data |= PHY_FORCE_MDIX;
		if (restart & PORT_AUTO_MDIX_DISABLE)
			data |= PHY_AUTO_MDIX_DISABLE;
		if (restart & PORT_REMOTE_FAULT_DISABLE)
			data |= PHY_REMOTE_FAULT_DISABLE;
		if (restart & PORT_TX_DISABLE)
			data |= PHY_TRANSMIT_DISABLE;
		if (restart & PORT_LED_OFF)
			data |= PHY_LED_DISABLE;
		break;
	case PHY_REG_STATUS:
		port_r(sw, p, P_LINK_STATUS, &link);
		port_r(sw, p, P_SPEED_STATUS, &speed);
		data = PHY_100BTX_FD_CAPABLE |
			PHY_100BTX_CAPABLE |
			PHY_10BT_FD_CAPABLE |
			PHY_10BT_CAPABLE |
			PHY_AUTO_NEG_CAPABLE;
		if (link & PORT_AUTO_NEG_COMPLETE)
			data |= PHY_AUTO_NEG_ACKNOWLEDGE;
		if (link & PORT_STATUS_LINK_GOOD)
			data |= PHY_LINK_STATUS;
		if (speed & PORT_REMOTE_FAULT)
			data |= PHY_REMOTE_FAULT;
		break;
	case PHY_REG_ID_1:
		data = 0x0022;
		break;
	case PHY_REG_ID_2:
		/* Use unique switch id to differentiate from regular PHY. */
		data = KSZ8863_SW_ID;
		break;
	case PHY_REG_AUTO_NEGOTIATION:
		port_r(sw, p, P_PHY_CTRL, &ctrl);
		data = PHY_AUTO_NEG_802_3;
		if (ctrl & PORT_AUTO_NEG_SYM_PAUSE)
			data |= PHY_AUTO_NEG_SYM_PAUSE;
		if (ctrl & PORT_AUTO_NEG_100BTX_FD)
			data |= PHY_AUTO_NEG_100BTX_FD;
		if (ctrl & PORT_AUTO_NEG_100BTX)
			data |= PHY_AUTO_NEG_100BTX;
		if (ctrl & PORT_AUTO_NEG_10BT_FD)
			data |= PHY_AUTO_NEG_10BT_FD;
		if (ctrl & PORT_AUTO_NEG_10BT)
			data |= PHY_AUTO_NEG_10BT;
		break;
	case PHY_REG_REMOTE_CAPABILITY:
		port_r(sw, p, P_LINK_STATUS, &link);
		data = PHY_AUTO_NEG_802_3;
		if (link & PORT_REMOTE_SYM_PAUSE)
			data |= PHY_AUTO_NEG_SYM_PAUSE;
		if (link & PORT_REMOTE_100BTX_FD)
			data |= PHY_AUTO_NEG_100BTX_FD;
		if (link & PORT_REMOTE_100BTX)
			data |= PHY_AUTO_NEG_100BTX;
		if (link & PORT_REMOTE_10BT_FD)
			data |= PHY_AUTO_NEG_10BT_FD;
		if (link & PORT_REMOTE_10BT)
			data |= PHY_AUTO_NEG_10BT;
		break;
	default:
		break;
	}
	*val = data;
}  /* sw_r_phy */

/**
 * sw_w_phy - write data to PHY register
 * @hw:		The switch instance.
 * @phy:	PHY address to write.
 * @reg:	PHY register to write.
 * @val:	Word data to write.
 *
 * This routine writes data to the PHY register.
 */
static void sw_w_phy(struct ksz_sw *sw, u16 phy, u16 reg, u16 val)
{
	u8 ctrl;
	u8 restart;
	u8 speed;
	u8 data;
	u8 p = phy - 1;

	switch (reg) {
	case PHY_REG_CTRL:
		port_r(sw, p, P_SPEED_STATUS, &speed);
		data = speed;
		if (val & PHY_HP_MDIX)
			data |= PORT_HP_MDIX;
		else
			data &= ~PORT_HP_MDIX;
		if (data != speed)
			port_w(sw, p, P_SPEED_STATUS, data);
		port_r(sw, p, P_PHY_CTRL, &ctrl);
		data = ctrl;
		if (val & PHY_AUTO_NEG_ENABLE)
			data |= PORT_AUTO_NEG_ENABLE;
		else
			data &= ~PORT_AUTO_NEG_ENABLE;
		if (val & PHY_SPEED_100MBIT)
			data |= PORT_FORCE_100_MBIT;
		else
			data &= ~PORT_FORCE_100_MBIT;
		if (val & PHY_FULL_DUPLEX)
			data |= PORT_FORCE_FULL_DUPLEX;
		else
			data &= ~PORT_FORCE_FULL_DUPLEX;
		if (data != ctrl)
			port_w(sw, p, P_PHY_CTRL, data);
		port_r(sw, p, P_NEG_RESTART_CTRL, &restart);
		data = restart;
		if (val & PHY_LED_DISABLE)
			data |= PORT_LED_OFF;
		else
			data &= ~PORT_LED_OFF;
		if (val & PHY_TRANSMIT_DISABLE)
			data |= PORT_TX_DISABLE;
		else
			data &= ~PORT_TX_DISABLE;
		if (val & PHY_AUTO_NEG_RESTART)
			data |= PORT_AUTO_NEG_RESTART;
		else
			data &= ~(PORT_AUTO_NEG_RESTART);
		if (val & PHY_REMOTE_FAULT_DISABLE)
			data |= PORT_REMOTE_FAULT_DISABLE;
		else
			data &= ~PORT_REMOTE_FAULT_DISABLE;
		if (val & PHY_POWER_DOWN)
			data |= PORT_POWER_DOWN;
		else
			data &= ~PORT_POWER_DOWN;
		if (val & PHY_AUTO_MDIX_DISABLE)
			data |= PORT_AUTO_MDIX_DISABLE;
		else
			data &= ~PORT_AUTO_MDIX_DISABLE;
		if (val & PHY_FORCE_MDIX)
			data |= PORT_FORCE_MDIX;
		else
			data &= ~PORT_FORCE_MDIX;
		if (val & PHY_LOOPBACK)
			data |= PORT_LOOPBACK;
		else
			data &= ~PORT_LOOPBACK;
		if (data != restart)
			port_w(sw, p, P_NEG_RESTART_CTRL, data);
		break;
	case PHY_REG_AUTO_NEGOTIATION:
		port_r(sw, p, P_PHY_CTRL, &ctrl);
		data = ctrl;
		data &= ~(PORT_AUTO_NEG_SYM_PAUSE |
			PORT_AUTO_NEG_100BTX_FD |
			PORT_AUTO_NEG_100BTX |
			PORT_AUTO_NEG_10BT_FD |
			PORT_AUTO_NEG_10BT);
		if (val & PHY_AUTO_NEG_SYM_PAUSE)
			data |= PORT_AUTO_NEG_SYM_PAUSE;
		if (val & PHY_AUTO_NEG_100BTX_FD)
			data |= PORT_AUTO_NEG_100BTX_FD;
		if (val & PHY_AUTO_NEG_100BTX)
			data |= PORT_AUTO_NEG_100BTX;
		if (val & PHY_AUTO_NEG_10BT_FD)
			data |= PORT_AUTO_NEG_10BT_FD;
		if (val & PHY_AUTO_NEG_10BT)
			data |= PORT_AUTO_NEG_10BT;
		if (data != ctrl)
			port_w(sw, p, P_PHY_CTRL, data);
		break;
	default:
		break;
	}
}  /* sw_w_phy */
static int ksz8863_probe(struct i2c_client *i2c,
	const struct i2c_device_id *i2c_id)
{
	struct i2c_hw_priv *hw_priv;
	struct sw_priv *ks;
	struct ksz_sw *sw;
	struct ksz_port *port;
	struct phy_device *phydev;
	struct phy_priv *priv;
	u16 id;
	int cnt;
	int i;
	int mib_port_count;
	int pi;
	int port_count;
	int ret;

	ks = kzalloc(sizeof(struct sw_priv), GFP_KERNEL);
	if (!ks)
		return -ENOMEM;

	ks->hw_dev = kzalloc(sizeof(struct i2c_hw_priv), GFP_KERNEL);
	if (!ks->hw_dev) {
		kfree(ks);
		return -ENOMEM;
	}
	hw_priv = ks->hw_dev;

	hw_priv->i2cdev = i2c;

	ks->intr_mode = intr_mode ? IRQF_TRIGGER_FALLING :
		IRQF_TRIGGER_LOW;
	ks->irq = i2c->irq;
	ks->dev = &i2c->dev;

	dev_set_drvdata(ks->dev, ks);

	mutex_init(&ks->hwlock);
	mutex_init(&ks->lock);

	/* simple check for a valid chip being connected to the bus */
	mutex_lock(&ks->lock);
	id = HW_R16(ks, REG_CHIP_ID0);
	mutex_unlock(&ks->lock);
	if ((id & SWITCH_CHIP_ID_MASK) != CHIP_ID_63) {
		dev_err(ks->dev, "failed to read device ID(0x%x)\n", id);
		ret = -ENODEV;
		goto err_sw;
	}
	dev_info(ks->dev, "chip id 0x%x\n", id);

	sw = &ks->sw;
	mutex_init(&sw->lock);
	sw->hwlock = &ks->hwlock;
	sw->reglock = &ks->lock;

	sw->dev_count = 1;

	port_count = SWITCH_PORT_NUM;
	mib_port_count = SWITCH_PORT_NUM;

	sw->mib_cnt = TOTAL_SWITCH_COUNTER_NUM;
	sw->mib_port_cnt = TOTAL_PORT_NUM;
	sw->port_cnt = TOTAL_PORT_NUM;
	sw->PORT_MASK = (1 << sw->mib_port_cnt) - 1;
	sw->HOST_PORT = SWITCH_PORT_NUM;
	sw->HOST_MASK = (1 << sw->HOST_PORT);

	sw->dev = ks;
	sw->id = sw_device_present;

	sw->info = kzalloc(sizeof(struct ksz_sw_info), GFP_KERNEL);
	if (!sw->info) {
		ret = -ENOMEM;
		goto err_sw;
	}

	sw->reg = &sw_reg_ops;
	sw->net_ops = &sw_net_ops;
	sw->ops = &sw_ops;

	INIT_DELAYED_WORK(&ks->link_read, link_read_work);
	INIT_DELAYED_WORK(&ks->stp_monitor, stp_work);

	ret = ksz_mii_init(ks);
	if (ret)
		goto err_mii;

	sw->multi_dev |= multi_dev;
	sw->stp |= stp;
	sw->fast_aging |= fast_aging;

	sw->phydev = ks->phydev;
	sw->counter = ks->counter;
	sw->monitor_timer_info = &ks->monitor_timer_info;
	sw->link_read = &ks->link_read;
	sw->stp_monitor = &ks->stp_monitor;

	sw_init_mib(sw);

	for (i = 0; i < TOTAL_PORT_NUM; i++)
		init_waitqueue_head(&ks->counter[i].counter);

	create_debugfs(ks);

	sw->ops->acquire(sw);
	id = HW_R16(ks, REG_MODE_INDICATOR);
	if (MODE_FLL == (id & MODE_RLL))
		sw->port_info[0].fiber = true;
	sw_init(sw);
	sw_setup(sw);
	sw->ops->release(sw);

#ifndef CONFIG_MICREL_SWITCH_EMBEDDED
	init_sw_sysfs(sw, &ks->sysfs, ks->dev);
#endif
	ret = sysfs_create_bin_file(&ks->dev->kobj,
		&kszsw_registers_attr);
	sema_init(&ks->proc_sem, 1);

	for (cnt = 0, pi = 0; cnt < port_count; cnt++, pi++) {
		/*
		 * Initialize to invalid value so that link detection
		 * is done.
		 */
		sw->port_info[pi].partner = 0xFF;
		sw->port_info[pi].state = media_disconnected;
	}
	sw->interface = PHY_INTERFACE_MODE_MII;
	for (i = 0; i <= SWITCH_PORT_NUM; i++) {
		sw->phy[i] = ks->bus->phy_map[i];
		phydev = sw->phy[i];
		if (!phydev)
			continue;
		priv = phydev->priv;
		port = &priv->port;
		port->port_cnt = port_count;
		port->mib_port_cnt = mib_port_count;
		port->first_port = 0;
		port->flow_ctrl = PHY_FLOW_CTRL;

		port->linked = &sw->port_info[port->first_port];
	}

	INIT_WORK(&ks->mib_read, ksz8863_mib_read_work);

	/* 500 ms timeout */
	ksz_init_timer(&ks->mib_timer_info, 500 * HZ / 1000,
		ksz8863_mib_monitor, ks);
	ksz_init_timer(&ks->monitor_timer_info, 100 * HZ / 1000,
		ksz8863_dev_monitor, ks);

	ksz_start_timer(&ks->mib_timer_info, ks->mib_timer_info.period);
	if (!sw->multi_dev && !sw->stp)
		ksz_start_timer(&ks->monitor_timer_info,
			ks->monitor_timer_info.period);

	sw_device_present++;

#ifdef CONFIG_NET_DSA_TAG_TAIL
	ksz_dsa_init();
#endif

	if (ks->irq <= 0)
		return 0;
	ks->intr_mask = INT_PORT_1_LINK_CHANGE | INT_PORT_2_LINK_CHANGE |
		INT_PORT_3_LINK_CHANGE | INT_PORT_1_2_LINK_CHANGE;
	mutex_lock(&ks->lock);
	HW_W(ks, REG_INT_ENABLE, 0);
	HW_W(ks, REG_INT_STATUS, ks->intr_mask);
	mutex_unlock(&ks->lock);
	ret = sw_start_interrupt(ks, dev_name(ks->dev));
	if (ret < 0)
		printk(KERN_WARNING "No switch interrupt\n");
	else {
		mutex_lock(&ks->lock);
		HW_W(ks, REG_INT_ENABLE, ks->intr_mask);
		mutex_unlock(&ks->lock);
	}

	return 0;

err_mii:
	kfree(sw->info);

err_sw:
	kfree(ks->hw_dev);
	kfree(ks);

	return ret;
}

static int ksz8863_remove(struct i2c_client *i2c)
{
	struct sw_priv *ks = dev_get_drvdata(&i2c->dev);
	struct ksz_sw *sw = &ks->sw;

#ifdef CONFIG_NET_DSA_TAG_TAIL
	ksz_dsa_cleanup();
#endif
	ksz_stop_timer(&ks->monitor_timer_info);
	ksz_stop_timer(&ks->mib_timer_info);
	flush_work(&ks->mib_read);

	sysfs_remove_bin_file(&ks->dev->kobj, &kszsw_registers_attr);
#ifndef CONFIG_MICREL_SWITCH_EMBEDDED
	exit_sw_sysfs(sw, &ks->sysfs, ks->dev);
#endif
	cancel_delayed_work_sync(&ks->link_read);
	cancel_delayed_work_sync(&ks->stp_monitor);
	delete_debugfs(ks);
	kfree(sw->info);
	ksz_mii_exit(ks);
	kfree(ks->hw_dev);
	kfree(ks);

	return 0;
}

#define I2C_SWITCH_NAME			"i2c-ksz8863"
#define I2C_SWITCH_ADDR			0x5F
#define I2C_SWITCH_INTR			-1

static int ksz8863_detect(struct i2c_client *i2c, struct i2c_board_info *info)
{
	strncpy(info->type, I2C_SWITCH_NAME, I2C_NAME_SIZE);
	info->irq = I2C_SWITCH_INTR;
	return 0;
}

static unsigned short i2c_address_list[] = {
	I2C_SWITCH_ADDR,

	I2C_CLIENT_END
};

static const struct i2c_device_id i2c_id[] = {
	{ I2C_SWITCH_NAME, 0 },
	{ },
};

static struct i2c_driver ksz8863_driver = {
	.driver.name	= I2C_SWITCH_NAME,
	.probe		= ksz8863_probe,
	.remove		= ksz8863_remove,
	.id_table	= i2c_id,

	/* Big enough to be accepted in all cases. */
	.class		= 0xffff,
	.detect		= ksz8863_detect,
	.address_list	= i2c_address_list,
};

static int __init ksz8863_init(void)
{
	int ret;

	ret = i2c_add_driver(&ksz8863_driver);
	if (ret)
		return ret;

	/* Probe not called. */
	if (!sw_device_present) {
		struct i2c_adapter *adap;

		/* Assume I2C bus starts at 0. */
		adap = i2c_get_adapter(0);

		/* I2C master may not be created yet. */
		if (!adap) {
#if !defined(CONFIG_I2C_KSZ8863_MODULE)
			struct i2c_board_info info = {
				.type	= I2C_SWITCH_NAME,
				.addr	= I2C_SWITCH_ADDR,
				.irq	= I2C_SWITCH_INTR,
			};

			ret = i2c_register_board_info(0, &info, 1);
#else
			return -ENODEV;
#endif
		} else
			i2c_put_adapter(adap);
	}
	return ret;
}

static void __exit ksz8863_exit(void)
{
	i2c_del_driver(&ksz8863_driver);
}

#ifndef CONFIG_MICREL_KSZ8863_EMBEDDED
module_init(ksz8863_init);
module_exit(ksz8863_exit);

module_param(fast_aging, int, 0);
module_param(multi_dev, int, 0);
module_param(stp, int, 0);
MODULE_PARM_DESC(fast_aging, "Fast aging");
MODULE_PARM_DESC(multi_dev, "Multiple device interfaces");
MODULE_PARM_DESC(stp, "STP support");
#endif

module_param(intr_mode, int, 0);
MODULE_PARM_DESC(intr_mode,
	"Configure which interrupt mode to use(0=level low, 1=falling)");

#ifndef CONFIG_MICREL_KSZ8863_EMBEDDED
MODULE_DESCRIPTION("KSZ8863 switch driver");
MODULE_AUTHOR("Tristram Ha <Tristram.Ha@microchip.com>");
MODULE_LICENSE("GPL");

MODULE_ALIAS("i2c:ksz8863");
#endif
