/*
 * Driver for Decawave DW1000 802.15.4 Wireless-PAN transceiver
 *
 * Copyright (c) 2016 Florian Tautz (dev@nyronet.de)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/netdevice.h>
#include <linux/byteorder/generic.h>
#include <linux/jiffies.h>
#include <linux/if_arp.h>
#include <linux/ip.h>


/* driver constants */
#define SPI_WRITE_BUFSIZE	1027
#define REG_MAX_ADDRLEN		3
#define TX_TIMEOUT		HZ / 10
#define DW1000_MTU		1000
#define DW1000_ADDR_LEN		4
#define DW1000_HDR_LEN		sizeof(struct dw1000_header)
#define CFG_RETRY_INTERVAL_MIN	1000
#define CFG_RETRY_INTERVAL_MAX	2000

/* IEEE 802.15.4 frame control flags */
#define FC_FRAMETYPE_DATA	0x01
#define FC_AR			BIT(5)
#define FC_PANID_COMPR		BIT(6)
#define FC_DADDR_SHORT		BIT(11)
#define FC_SADDR_SHORT		BIT(15)


/* spi transaction flags */
#define REG_FLAG_WRITE		0x80
#define REG_FLAG_SUBINDEX	0x40
#define REG_FLAG_EXTADDR	0x80

/* register addresses */
#define REG_DEV_ID	0x01000000
#define REG_EUI		0x01000001
#define REG_PANADR	0x01000003
#define REG_SYS_CFG	0x01000004
#define REG_SYS_TIME	0x01000006
#define REG_TX_FCTRL	0x01000008
#define REG_TX_BUFFER	0x01000009
#define REG_DX_TIME	0x0100000a
#define REG_RX_FWTO	0x0100000c
#define REG_SYS_CTRL	0x0100000d
#define REG_SYS_MASK	0x0100000e
#define REG_SYS_STATUS	0x0100000f
#define REG_RX_FINFO	0x01000010
#define REG_RX_BUFFER	0x01000011
#define REG_RX_FQUAL	0x01000012
#define REG_RX_TTCKI	0x01000013
#define REG_RX_TTCKO	0x01000014
#define REG_RX_TIME	0x01000015
#define REG_TX_TIME	0x01000017
#define REG_TX_ANTD	0x01000018
#define REG_SYS_STATE	0x01000019
#define REG_ACK_RESP_T	0x0100001a
#define REG_RX_SNIFF	0x0100001d
#define REG_TX_POWER	0x0100001e
#define REG_CHAN_CTRL	0x0100001f
#define REG_USR_SFD	0x01000021
#define REG_AGC_CTRL	0x01000023
#define REG_EXT_SYNC	0x01000024
#define REG_ACC_MEM	0x01000025
#define REG_GPIO_CTRL	0x01000026
#define REG_DRX_CONF	0x01000027
#define REG_RF_CONF	0x01000028
#define REG_TX_CAL	0x0100002a
#define REG_FS_CTRL	0x0100002b
#define REG_AON		0x0100002c
#define REG_OTP_IF	0x0100002d
#define REG_LDE_CTRL	0x0100002e
#define REG_DIG_DIAG	0x0100002f
#define REG_PMSC	0x01000036

/* subregister addresses */
#define SUBREG_AGC_CTRL1	0x02002302
#define SUBREG_AGC_TUNE1	0x02002304
#define SUBREG_AGC_TUNE2	0x0200230c
#define SUBREG_AGC_TUNE3	0x02002312
#define SUBREG_AGC_STAT1	0x0200231e
#define SUBREG_DRX_TUNE2	0x02002708
#define SUBREG_LDE_CFG1		0x032e0806
#define SUBREG_LDE_CFG2		0x032e1806
#define SUBREG_RF_TXCTRL	0x0200280c
#define SUBREG_AON_WCFG		0x02002c00
#define SUBREG_TC_PGDELAY	0x02002a0b
#define SUBREG_FS_PLLCFG	0x02002b07
#define SUBREG_FS_PLLTUNE	0x02002b0b
#define SUBREG_PMSC_CTRL0	0x02003600
#define SUBREG_OTP_CTRL		0x02002d06
#define SUBREG_LDOTUNE		0x02002830
#define SUBREG_OTP_ADDR		0x02002d04
#define SUBREG_OTP_CTRL		0x02002d06
#define SUBREG_OTP_RDAT		0x02002d0a
#define SUBREG_PAN_ID		0x02000302
#define SUBREG_DRX_TUNE0b	0x02002702
#define SUBREG_DRX_TUNE1a	0x02002704
#define SUBREG_DRX_TUNE1b	0x02002706
#define SUBREG_RF_RXCTRLH	0x0200280b

/* TX parameters */
#define TXBR_6M8	0x00004000 
#define PRF_16		0x00010000
#define PRF_64		0x00020000
#define TXPLEN_64	BIT(18)
#define TXPLEN_4096	0x00c00000

/* system configuration */
#define CFG_FFEN	BIT(0)
#define CFG_FFAD	BIT(3)
#define CFG_HIRQ_POL	BIT(9)
#define CFG_DIS_DRXB	BIT(12)
#define CFG_RXAUTR	BIT(29)
#define CFG_PHR_MODE_LONG BIT(16) | BIT(17)

/* system state control */
#define CTRL_TXSTRT	BIT(1)
#define CTRL_TRXOFF	BIT(6)
#define CTRL_RXENAB	BIT(8)

/* status flags */
#define STATUS_IRQS	BIT(0)
#define STATUS_TXFRB	BIT(4)
#define STATUS_TXPRS	BIT(5)
#define STATUS_TXPHS	BIT(6)
#define STATUS_TXFRS	BIT(7)
#define STATUS_RXPRD	BIT(8)
#define STATUS_RXSFDD	BIT(9)
#define STATUS_LDEDONE	BIT(10)
#define STATUS_RXPHD	BIT(11)
#define STATUS_RXDFR	BIT(13)
#define STATUS_RXFCG	BIT(14)
#define STATUS_AFFREJ	BIT(29)

/* device private data */
struct dw1000 {
	struct spi_device *spi;
	struct net_device *netdev;
	struct sk_buff *tx_buf;
	struct work_struct irq_work;
	struct work_struct tx_work;
	u8 *spi_write_buf;
	u8 rx_buf[1023];
	struct mutex device_lock; /* always acquire this before communicating with the device */
	u8 frseq;
	u8 channel;
	bool tx_pending;
	bool enable_dynamic_delay;
};

/* header format used by this driver.
 * source PAN ID and security field are omitted.
 */
struct dw1000_header {
	u16 frame_control;
	u8 seq;			/* 802.15.4 sequence number */
	u16 dest_panid;
	u16 dest_saddr;
	u16 src_saddr;
	__be16 protocol;	/* skb protocol type */
} __attribute__((packed));

/* hw address format used */
struct dw1000_addr {
	__be16 panid;
	__be16 saddr;
};

/* Load address into the spi_write_buf and return the address lengths.
 * Sets the required flags for subregister addresses but doesn't set r/w flags.
 */
static u8 dw1000_load_addr(struct dw1000 *dev, u32 addr)
{
	u8 addr_len = (u8)(addr >> 24);
	switch (addr_len) {
	case 1:
		dev->spi_write_buf[0] = (u8)addr;
		dev->spi_write_buf[0] &= ~REG_FLAG_SUBINDEX;
		break;
	case 2:
		dev->spi_write_buf[0] = (u8)(addr >> 8);
		dev->spi_write_buf[0] |= REG_FLAG_SUBINDEX;
		dev->spi_write_buf[1] = (u8)addr;
		dev->spi_write_buf[1] &= ~REG_FLAG_EXTADDR;
		break;
	case 3:
		dev->spi_write_buf[0] = (u8)(addr >> 16);
		dev->spi_write_buf[0] |= REG_FLAG_SUBINDEX;
		dev->spi_write_buf[1] = (u8)(addr >> 8);
		dev->spi_write_buf[1] |= REG_FLAG_EXTADDR;
		dev->spi_write_buf[2] = (u8)addr;
		break;
	default:
		dev_err(&dev->spi->dev, "invalid register address");
		break;
	}
	return addr_len;
}

/* read data from a DW1000 register */
static int reg_read(struct dw1000 *dev, u32 addr, void *buf, u16 len)
{
	u8 addr_len = dw1000_load_addr(dev, addr);
	dev->spi_write_buf[0] &= ~REG_FLAG_WRITE;
	return spi_write_then_read(dev->spi, dev->spi_write_buf, addr_len,
			buf, len);
}

//static int reg_read_u8(struct dw1000 *dev, u32 addr, u8 *value)
//{
//	return reg_read(dev, addr, value, sizeof(*value));
//}

static int reg_read_u16(struct dw1000 *dev, u32 addr, u16 *value)
{
	int ret = reg_read(dev, addr, value, sizeof(*value));
	if (ret < 0)
		return ret;
	le16_to_cpus(value);
	return 0;
}

static int reg_read_u32(struct dw1000 *dev, u32 addr, u32 *value)
{
	int ret = reg_read(dev, addr, value, sizeof(*value));
	if (ret < 0)
		return ret;
	le32_to_cpus(value);
	return 0;
}

/* write data to a DW1000 register */
static int reg_write(struct dw1000 *dev, u32 addr, void *buf, u16 len)
{
	u8 addr_len = dw1000_load_addr(dev, addr);
	dev->spi_write_buf[0] |= REG_FLAG_WRITE;
	if (len > (SPI_WRITE_BUFSIZE - 3)) {
		dev_err(&dev->netdev->dev, "write too long");
		return -1;
	}

	memcpy(&dev->spi_write_buf[addr_len], buf, len);
	return spi_write(dev->spi, dev->spi_write_buf, addr_len + len);
}

static int reg_write_u8(struct dw1000 *dev, u32 addr, u8 value)
{
	return reg_write(dev, addr, &value, sizeof(value));
}

static int reg_write_u16(struct dw1000 *dev, u32 addr, u16 value)
{
	cpu_to_le16s(&value);
	return reg_write(dev, addr, &value, sizeof(value));
}

static int reg_write_u32(struct dw1000 *dev, u32 addr, u32 value)
{
	cpu_to_le32s(&value);
	return reg_write(dev, addr, &value, sizeof(value));
}

/* read data from OTP memory. see manual section 6.3.3 */
static int otp_read(struct dw1000 *dev, u16 otp_addr, void *buf, u8 len)
{
	int ret;

	ret = reg_write_u16(dev, SUBREG_OTP_ADDR, otp_addr);
	if (ret < 0)
		return ret;
	ret = reg_write_u8(dev, SUBREG_OTP_CTRL, 0x03);
	if (ret < 0)
		return ret;
	ret = reg_write_u8(dev, SUBREG_OTP_CTRL, 0x01);
	if (ret < 0)
		return ret;
	ret = reg_read(dev, SUBREG_OTP_RDAT, buf, len);
	if (ret < 0)
		return ret;
	ret = reg_write_u8(dev, SUBREG_OTP_CTRL, 0x00);
	if (ret < 0)
		return ret;
	
	return 0;
}

/* read frame data from the transceiver and hand the received packet
 * over to the kernel.
 */
static int dw1000_receive_frame(struct dw1000 *dev)
{
	int ret;
	u16 frame_length;
	struct net_device *netdev = dev->netdev;
	struct sk_buff *skb;
	struct dw1000_header *hdr;

	ret = reg_read_u16(dev, REG_RX_FINFO, &frame_length);
	if (ret) {
		dev_err(&netdev->dev, "failed to read frame length");
		goto out_err;
	}
	frame_length &= 0x3ff; /* only keep length field */
	frame_length -= 2; /* strip FCS */

	ret = reg_read(dev, REG_RX_BUFFER, dev->rx_buf, frame_length);
	if (ret) {
		dev_err(&netdev->dev, "failed to read frame data");
		goto out_err;
	}
	
	skb = netdev_alloc_skb(netdev, frame_length);
	if (!skb) {
		dev_err(&netdev->dev, "failed to allocate skb");
		goto out_err;
	}

	memcpy(skb_put(skb, frame_length), dev->rx_buf, frame_length);

	skb->dev = netdev;
	skb_reset_mac_header(skb);
	hdr = (struct dw1000_header *)skb->data;
	skb_pull_inline(skb, DW1000_HDR_LEN);
	skb_reset_network_header(skb);

	skb->protocol = hdr->protocol;
	skb->pkt_type = PACKET_HOST;

	netdev->stats.rx_packets++;
	netdev->stats.rx_bytes += frame_length;

	ret = netif_rx_ni(skb);

	return 0;

out_err:
	return -1;
}

/* schedule bottom half for interrupt processing */
static irqreturn_t dw1000_handle_interrupt(int irq, void *dev_ptr)
{
	struct dw1000 *dev = dev_ptr;

	/* disable interrupts because the IRQ line will remain active
	 * until status flag is cleared */
	disable_irq_nosync(irq);

	schedule_work(&dev->irq_work);

	return IRQ_HANDLED;
}

static const u32 dynamic_delay_map[][2] = {
	{100, 1000},
	{200, 1500},
	{350, 2000},
	{500, 2500},
	{650, 3000},
	{800, 3500},
	{0, 3500},
};

/* If enabled, do inter frame delay according to empirical tests. */
static void dynamic_tx_delay(struct dw1000 *dev)
{
	u8 i;

	if (!dev->enable_dynamic_delay) {
		return;
	}

	for (i = 0; dynamic_delay_map[i][0] != 0 &&
			dev->tx_buf->len > dynamic_delay_map[i][0];
			i++) ;

	usleep_range(dynamic_delay_map[i][1], dynamic_delay_map[i][1] + 500);
}

/* Read system status flags and act accordingly.
 * Caller should hold device_lock
 */
static int dw1000_process_status(struct dw1000 *dev)
{
	int ret;
	struct net_device *netdev = dev->netdev;

	u32 status;
	u32 status_processed = STATUS_IRQS;

	ret = reg_read_u32(dev, REG_SYS_STATUS, &status);
	if (ret) {
		dev_err(&netdev->dev, "failed to read SYS_STATUS");
		goto out;
	}

	/* clear all status flags on the transceiver */
	ret = reg_write_u32(dev, REG_SYS_STATUS, status);
	if (ret < 0)
		goto out_err;


	/* transmission */
	if (status & STATUS_TXFRS) {
		netdev->stats.tx_packets++;
		netdev->stats.tx_bytes += dev->tx_buf->len;

		dev_kfree_skb(dev->tx_buf);

		ret = reg_write_u32(dev, REG_SYS_CTRL, CTRL_RXENAB);
		if (ret) {
			goto out_err;
		}

		dynamic_tx_delay(dev);

		dev->tx_pending = false;
		netif_start_queue(netdev);

		status_processed |= STATUS_TXFRS;
		status_processed |= STATUS_TXFRB | STATUS_TXPRS | STATUS_TXPHS;
		goto end_processing;
	}

	/* reception: frame ok */
	if (status & STATUS_RXDFR) {
		if (status & STATUS_RXFCG) { /* CRC good */
			ret = dw1000_receive_frame(dev);
			if (ret) {
				goto out_err;
			}

			ret = reg_write_u32(dev, REG_SYS_CTRL, CTRL_RXENAB);
			if (ret) {
				goto out_err;
			}

			status_processed |= STATUS_RXFCG;
			status_processed |= STATUS_RXSFDD | STATUS_RXPHD |
					STATUS_RXPRD | STATUS_LDEDONE;
		} else {
			dev_notice(&dev->spi->dev, "rx error, status %x",
					status);

			ret = reg_write_u32(dev, REG_SYS_CTRL, CTRL_RXENAB);
			if (ret) {
				goto out_err;
			}
		}

		status_processed |= STATUS_RXDFR;
		goto end_processing;
	}

	/* reception: frame rejected */
	//if (status & STATUS_AFFREJ) {
	//	dev_info(&netdev->dev, "frame rejected by transceiver");
	//	ret = reg_write_u32(dev, REG_SYS_CTRL, CTRL_RXENAB);
	//	if (ret) {
	//		goto out_err;
	//	}
	//	status_processed |= STATUS_AFFREJ;
	//	goto end_processing;
	//}

end_processing:
//	if (status & ~status_processed) {
//		dev_warn(&dev->spi->dev, "unprocessed status flags %x in %x",
//				status & ~status_processed, status);
//	}

	ret = 0;
	goto out;

out_err:
	dev_err(&netdev->dev, "error while processing sys_status: %d", ret);
out:
	return ret;
}

/* worker function for interrupt handling. process the status flags
 * and then re-enable interrupts.
 */
static void dw1000_irq_worker(struct work_struct *work)
{
	int ret;
	struct dw1000 *dev = container_of(work, struct dw1000, irq_work);

	mutex_lock(&dev->device_lock);
	ret = dw1000_process_status(dev);
	if (ret) {
		goto out_err;
	}

	enable_irq(dev->spi->irq);
	goto out;

out_err:
	dev_err(&dev->spi->dev, "irq_worker: unhandled error");
out:
	mutex_unlock(&dev->device_lock);
}

/* worker function for transmitting frames. write data to the transceiver
 * and initiate transmission.
 */
static void dw1000_xmit_worker(struct work_struct *work)
{
	int ret;
	struct dw1000 *dev = container_of(work, struct dw1000, tx_work);

	mutex_lock(&dev->device_lock);

	ret = reg_write_u32(dev, REG_SYS_CTRL, CTRL_TRXOFF);
	if (ret) {
		goto out_err;
	}

	/* handle status flags if there are any. processes frames that were
	 * received before disabling the transceiver but have not been
	 * read yet.
	 */
	ret = dw1000_process_status(dev);
	if (ret) {
		goto out_err;
	}

	/* write data to transceiver */
	ret = reg_write(dev, REG_TX_BUFFER, dev->tx_buf->data,
			dev->tx_buf->len);
	if (ret) {
		goto out_err;
	}

	/* extend length to make room for FCS */
	ret = reg_write_u32(dev, REG_TX_FCTRL, (dev->tx_buf->len + 2) |
			TXBR_6M8 | PRF_16 | TXPLEN_64);
	if (ret) {
		goto out_err;
	}

	/* start sending */
	ret = reg_write_u32(dev, REG_SYS_CTRL, CTRL_TXSTRT);
	if (ret) {
		goto out_err;
	}

	dev->netdev->trans_start = jiffies;

	goto out;

out_err:
	dev_err(&dev->netdev->dev, "error in xmit_worker");

out:
	mutex_unlock(&dev->device_lock);
}

/* Set channel configuration.
 * Caller has to handle stopping the transceiver etc.
 */
static int dw1000_set_channel(struct dw1000 *dev, u8 channel)
{
	int ret;
	u32 chan_ctrl = 0;
	u8 tc_pgdelay = 0;
	u32 fs_pllcfg = 0;
	u8 fs_plltune = 0;
	u32 rf_txctrl = 0;
	u8 rf_rxctrlh = 0;

	/* See manual 7.2.32 */
	chan_ctrl = channel | channel << 4;
	chan_ctrl |= (1 << 18); /* 16 MHz PRF */

	/* TODO reference manual */
	switch (channel) {
	case 1:
		chan_ctrl |= (1 << 22) | (1 << 27);
		tc_pgdelay = 0xc9;
		fs_pllcfg = 0x09000407;
		fs_plltune = 0x1e;
		rf_txctrl = 0x00005c40;
		rf_rxctrlh = 0xd8;
		break;
	case 2:
		chan_ctrl |= (4 << 22) | (4 << 27);
		tc_pgdelay = 0xc2;
		fs_pllcfg = 0x08400508;
		fs_plltune = 0x26;
		rf_txctrl = 0x00045ca0;
		rf_rxctrlh = 0xd8;
		break;
	case 3:
		chan_ctrl |= (5 << 22) | (5 << 27);
		tc_pgdelay = 0xc5;
		fs_pllcfg = 0x08401009;
		fs_plltune = 0x5e;
		rf_txctrl = 0x00086cc0;
		rf_rxctrlh = 0xd8;
		break;
	case 4:
		chan_ctrl |= (7 << 22) | (7 << 27);
		tc_pgdelay = 0x95;
		fs_pllcfg = 0x08400508;
		fs_plltune = 0x26;
		rf_txctrl = 0x00045c80;
		rf_rxctrlh = 0xbc;
		break;
	case 5:
		chan_ctrl |= (3 << 22) | (3 << 27);
		tc_pgdelay = 0xc0;
		fs_pllcfg = 0x0800041d;
		fs_plltune = 0xbe;
		rf_txctrl = 0x001e3fe0;
		rf_rxctrlh = 0xd8;
		break;
	case 7:
		chan_ctrl |= (8 << 22) | (8 << 27);
		tc_pgdelay = 0x93;
		fs_pllcfg = 0x0800041d;
		fs_plltune = 0xbe;
		rf_txctrl = 0x001e7de0;
		rf_rxctrlh = 0xbc;
		break;
	default:
		return -EINVAL;
	}
	
	ret = reg_write_u32(dev, REG_CHAN_CTRL, chan_ctrl);
	if (ret) {
		return ret;
	}

	ret = reg_write_u8(dev, SUBREG_TC_PGDELAY, tc_pgdelay);
	if (ret) {
		return ret;
	}

	ret = reg_write_u32(dev, SUBREG_FS_PLLCFG, fs_pllcfg);
	if (ret) {
		return ret;
	}

	ret = reg_write_u8(dev, SUBREG_FS_PLLTUNE, fs_plltune);
	if (ret) {
		return ret;
	}

	ret = reg_write_u32(dev, SUBREG_RF_TXCTRL, rf_txctrl);
	if (ret) {
		return ret;
	}

	ret = reg_write_u8(dev, SUBREG_RF_RXCTRLH, rf_rxctrlh);
	if (ret) {
		return ret;
	}

	dev->channel = channel;
	return 0;
}

/* Perform initial configuration of the transceiver when starting the device */
int dw1000_open(struct net_device *netdev)
{
	int ret;
	struct dw1000 *dev = netdev_priv(netdev);
	u8 ldotune[5];

	/* general system configuration */
	ret = reg_write_u32(dev, REG_SYS_CFG, CFG_HIRQ_POL | CFG_DIS_DRXB |
			CFG_RXAUTR | CFG_PHR_MODE_LONG | CFG_FFEN | CFG_FFAD);
	if (ret < 0)
		return ret;

	/* Apply default configuration according to 2.5.5 */

	ret = reg_write_u16(dev, SUBREG_AGC_TUNE1, 0x8870);
	if (ret < 0)
		return ret;
	
	/* See manual section 7.2.36.5 */
	ret = reg_write_u32(dev, SUBREG_AGC_TUNE2, 0x2502a907);
	if (ret < 0)
		return ret;
	
	/* PAC 8, 16MHz PRF, see manual table 31 */
	ret = reg_write_u32(dev, SUBREG_DRX_TUNE2, 0x311a002d);
	if (ret < 0)
		return ret;
	
	ret = reg_write_u8(dev, SUBREG_LDE_CFG1, 0x6d);
	if (ret < 0)
		return ret;

	ret = reg_write_u16(dev, SUBREG_LDE_CFG2, 0x1607);
	if (ret < 0)
		return ret;
	
	ret = reg_write_u32(dev, REG_TX_POWER, 0x0e082848);
	if (ret < 0)
		return ret;
	

	/* Receiver config, see manual section 7.2.40 */
	ret = reg_write_u16(dev, SUBREG_DRX_TUNE0b, 0x0001);
	if (ret < 0)
		return ret;
	
	ret = reg_write_u16(dev, SUBREG_DRX_TUNE1a, 0x0087);
	if (ret < 0)
		return ret;
	
	ret = reg_write_u16(dev, SUBREG_DRX_TUNE1b, 0x0010);
	if (ret < 0)
		return ret;

	ret = dw1000_set_channel(dev, 5);
	if (ret)
		return ret;
	

	/* LDELOAD, see manual section 2.5.5.10 */
	ret = reg_write_u16(dev, SUBREG_PMSC_CTRL0, 0x0301);
	if (ret < 0)
		return ret;
	ret = reg_write_u16(dev, SUBREG_OTP_CTRL, 0x8000);
	if (ret < 0)
		return ret;
	usleep_range(150, 500);
	ret = reg_write_u16(dev, SUBREG_PMSC_CTRL0, 0x0200);
	if (ret < 0)
		return ret;

	/* Load LDOTUNE from OTP memory */
	ret = otp_read(dev, 0x004, &ldotune[0], 4);
	if (ret < 0)
		return ret;
	if (ldotune[0] == 0) {
		dev_err(&dev->netdev->dev, "failed to load LDOTUNE");
		return -1;
	}
	ret = otp_read(dev, 0x005, &ldotune[4], 1);
	if (ret < 0)
		return ret;

	/* set ldotune */
	ret = reg_write(dev, SUBREG_LDOTUNE,
			&ldotune[0], sizeof(ldotune));
	if (ret < 0)
		return ret;

	/* enable interrupts */
	/* debugging config:
	ret = reg_write_u32(dev, REG_SYS_MASK, STATUS_TXFRS | BIT(12) | BIT(13) |
			BIT(15) | BIT(16) | BIT(17) | BIT(18) | BIT(20) |
			BIT(21) | BIT(26) | BIT(28) |
			STATUS_AFFREJ);
	*/
	ret = reg_write_u32(dev, REG_SYS_MASK,
			STATUS_TXFRS | STATUS_RXDFR); //| STATUS_AFFREJ);
	if (ret < 0)
		return ret;

	/* enable receiver */
	ret = reg_write_u32(dev, REG_SYS_CTRL, CTRL_RXENAB);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

int dw1000_stop(struct net_device *netdev)
{
	int ret;
	struct dw1000 *dev = netdev_priv(netdev);

	ret = reg_write_u32(dev, REG_SYS_CTRL, CTRL_TRXOFF);
	if (ret < 0) {
		dev_err(&netdev->dev, "failed to turn off transceiver");
	}

	// TODO: handle unfinished transmissions?

	return 0;
}

netdev_tx_t dw1000_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct dw1000 *dev = netdev_priv(netdev);

	netif_stop_queue(netdev);
	netdev->trans_start = jiffies;

	dev->tx_buf = skb;
	dev->tx_pending = true;
	schedule_work(&dev->tx_work);

	return NETDEV_TX_OK;
}

int dw1000_set_mac_address(struct net_device *netdev, void *addr_raw)
{
	int ret;
	struct dw1000 *dev = netdev_priv(netdev);
	struct sockaddr *addr = addr_raw;
	struct dw1000_addr *addr_struct = (struct dw1000_addr *)addr->sa_data;

	u32 panadr = be16_to_cpu(addr_struct->panid) << 16 |
			be16_to_cpu(addr_struct->saddr);

	ret = reg_write_u32(dev, REG_PANADR, panadr);
	if (ret) {
		dev_err(&netdev->dev, "failed to set PANADR");
		goto out;
	}

	memcpy(netdev->dev_addr, addr->sa_data, DW1000_ADDR_LEN);

	ret = 0;
out:
	return ret;
}

/* The kernel detected a timeout, reattempt transmission. */
void dw1000_tx_timeout(struct net_device *netdev)
{
	struct dw1000 *dev = netdev_priv(netdev);

	mutex_lock(&dev->device_lock);

	if (dev->tx_pending) {
		schedule_work(&dev->tx_work);
		dev_info(&netdev->dev, "tx_timeout");
	}

	mutex_unlock(&dev->device_lock);
}

/* net_device operations supported by this driver */
static struct net_device_ops dw1000_netdev_ops = {
	.ndo_open = dw1000_open,
	.ndo_stop = dw1000_stop,
	.ndo_start_xmit = dw1000_start_xmit,
	.ndo_set_mac_address = dw1000_set_mac_address,
	.ndo_tx_timeout = dw1000_tx_timeout,
};

/* Build header: 802.15.4 header followed by 2 octet protocol type from skb. */
int dw1000_header_create(struct sk_buff *skb, struct net_device *netdev,
		unsigned short type, const void *daddr,
		const void *saddr, unsigned int len)
{
	struct dw1000 *dev = netdev_priv(netdev);
	struct dw1000_header *hdr = (struct dw1000_header *)
			skb_push(skb, DW1000_HDR_LEN);
	skb_reset_mac_header(skb);
	skb->mac_len = DW1000_HDR_LEN;

	hdr->frame_control = FC_FRAMETYPE_DATA |
			FC_DADDR_SHORT |
			FC_SADDR_SHORT |
			FC_PANID_COMPR;
	hdr->seq = dev->frseq++;
	hdr->protocol = skb->protocol;

	/* ip packet -> derive from IP address */
	if (be16_to_cpu(skb->protocol) == ETH_P_IP) {
		struct iphdr *ip_header = (struct iphdr *)skb_network_header(skb);
		hdr->dest_panid = ip_header->daddr;
		be16_to_cpus(&hdr->dest_panid);
		cpu_to_le16s(&hdr->dest_panid);
		hdr->dest_saddr = (ip_header->daddr & 0xffff0000) >> 16;
		be16_to_cpus(&hdr->dest_saddr);
		cpu_to_le16s(&hdr->dest_saddr);
		hdr->src_saddr = (ip_header->saddr & 0xffff0000) >> 16;
		be16_to_cpus(&hdr->src_saddr);
		cpu_to_le16s(&hdr->src_saddr);
		return DW1000_HDR_LEN;
	} else {
		dev_info(&netdev->dev, "packet of type %d (0x%x)",
				skb->protocol, skb->protocol);
	}

	/* try to use destination address we were given */
	if (daddr) {
		const struct dw1000_addr *dest_addr = daddr;

		hdr->dest_panid = dest_addr->panid;
		be16_to_cpus(&hdr->dest_panid);
		cpu_to_le16s(&hdr->dest_panid);

		hdr->dest_saddr = dest_addr->saddr;
		be16_to_cpus(&hdr->dest_saddr);
		cpu_to_le16s(&hdr->dest_saddr);
		dev_info(&netdev->dev, "hdr->saddr = %04x", hdr->dest_saddr);

		return DW1000_HDR_LEN;
	}

	// TODO: broadcast address as last resort

	dev_warn(&netdev->dev, "header_create failed");
	return -DW1000_HDR_LEN;
}

/* Extract destination address from 802.15.4 header */
int dw1000_header_parse(const struct sk_buff *skb, unsigned char *haddr)
{
	struct dw1000_header *hdr = (struct dw1000_header *)skb_mac_header(skb);
	struct dw1000_addr *addr = (struct dw1000_addr *)haddr;
	addr->panid = hdr->dest_panid;
	addr->saddr = hdr->dest_saddr;
	return DW1000_ADDR_LEN;
}

/* hardware header operations */
static struct header_ops dw1000_header_ops = {
	.create = dw1000_header_create,
	.parse = dw1000_header_parse,
};

static int dw1000_hw_init(struct dw1000 *dev)
{
	int ret;
	u32 devid;

	dev_info(&dev->spi->dev, "hw_init");

	ret = reg_read_u32(dev, REG_DEV_ID, &devid);
	if (ret) {
		dev_err(&dev->spi->dev, "failed to read device id");
		goto out;
	}
	if (devid != 0xdeca0130) {
		dev_err(&dev->spi->dev, "invalid device id: %x", devid);
		ret = -EBADE;
		goto out;
	}

	// TODO: default panid, addr?

	ret = 0;

out:
	return ret;
}

/* initialize the net_device structure for this driver */
static void dw1000_netdev_setup(struct net_device *netdev)
{
	netdev->netdev_ops = &dw1000_netdev_ops;
	netdev->header_ops = &dw1000_header_ops;
	netdev->flags = IFF_NOARP; /* from linux/if.h */
	netdev->mtu = DW1000_MTU;
	netdev->type = ARPHRD_IEEE802154; /* from uapi/linux/if_arp.h */
	netdev->hard_header_len = sizeof(struct dw1000_header);
	netdev->addr_len = DW1000_ADDR_LEN;
	memset(netdev->broadcast, 0xff, DW1000_ADDR_LEN);
	netdev->tx_queue_len = 1;
	netdev->watchdog_timeo = TX_TIMEOUT;
	//netdev->destructor = ...; // TODO?
}

/* sysfs channel configuration: READ */
static ssize_t channel_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct net_device *netdev = to_net_dev(dev);
	struct dw1000 *dw_dev = netdev_priv(netdev);
	return scnprintf(buf, PAGE_SIZE, "%d\n", dw_dev->channel);
}

/* sysfs channel configuration: WRITE */
static ssize_t channel_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret;
	struct net_device *netdev = to_net_dev(dev);
	struct dw1000 *dw_dev = netdev_priv(netdev);
	u8 channel;

	if (sscanf(buf, "%hhu", &channel) != 1) {
		return -EINVAL;
	}

	mutex_lock(&dw_dev->device_lock);

	/* if a transmission is in progress, keep sleeping and retrying until
	 * it's done.
	 */
	while (dw_dev->tx_pending) {
		mutex_unlock(&dw_dev->device_lock);
		usleep_range(CFG_RETRY_INTERVAL_MIN, CFG_RETRY_INTERVAL_MAX);
		mutex_lock(&dw_dev->device_lock);
	}

	ret = reg_write_u32(dw_dev, REG_SYS_CTRL, CTRL_TRXOFF);
	if (ret) {
		goto out_err;
	}

	ret = dw1000_process_status(dw_dev);
	if (ret) {
		goto out_err;
	}

	ret = dw1000_set_channel(dw_dev, channel);
	if (ret) {
		goto out_err;
	}

	ret = reg_write_u32(dw_dev, REG_SYS_CTRL, CTRL_RXENAB);
	if (ret) {
		goto out_err;
	}

	mutex_unlock(&dw_dev->device_lock);
	return count;

out_err:
	dev_err(dev, "failed to set channel");
	/* attempt re-enabling receiver a last time to keep working */
	reg_write_u32(dw_dev, REG_SYS_CTRL, CTRL_RXENAB);
	mutex_unlock(&dw_dev->device_lock);
	return -1;
}

static DEVICE_ATTR_RW(channel);

/* sysfs IFS configuration: READ */
static ssize_t ifs_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct net_device *netdev = to_net_dev(dev);
	struct dw1000 *dw_dev = netdev_priv(netdev);
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			dw_dev->enable_dynamic_delay ? 1 : 0);
}

/* sysfs IFS configuration: WRITE */
static ssize_t ifs_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct net_device *netdev = to_net_dev(dev);
	struct dw1000 *dw_dev = netdev_priv(netdev);
	u8 ifs;

	if (sscanf(buf, "%hhu", &ifs) != 1) {
		return -EINVAL;
	}

	dw_dev->enable_dynamic_delay = (ifs != 0);

	return count;
}

static DEVICE_ATTR_RW(ifs);

/* Initialization function called when the module is loaded and the kernel
 * knows about a matching SPI device.
 * confirm that a DW1000 is connected and then perform initial setup of IRQ
 * and register net_device
 */
static int dw1000_probe(struct spi_device *spi)
{
	int ret = -ENOMEM;
	struct dw1000 *dev;
	struct net_device *netdev;

	dev_info(&spi->dev, "probe() IRQ %d", spi->irq);

	netdev = alloc_netdev(sizeof(*dev), "dw%d", NET_NAME_ENUM,
			dw1000_netdev_setup);
	if (netdev == NULL) {
		dev_err(&spi->dev, "alloc_netdev failed");
		ret = -ENOMEM;
		goto out;
	}

	dev = netdev_priv(netdev);
	dev->spi = spi;
	dev->netdev = netdev;
	dev->frseq = 0;
	dev->channel = 5;
	dev->tx_pending = false;
	dev->enable_dynamic_delay = true;

	dev->spi_write_buf = kmalloc(SPI_WRITE_BUFSIZE, GFP_KERNEL);
	if (dev->spi_write_buf == NULL) {
		dev_err(&spi->dev, "failed to allocate SPI buffer");
		ret = -ENOMEM;
		goto out_hw;
	}

	mutex_init(&dev->device_lock);
	INIT_WORK(&dev->irq_work, dw1000_irq_worker);
	INIT_WORK(&dev->tx_work, dw1000_xmit_worker);

	spi_set_drvdata(spi, dev);

	/* set up interrupt handler */
	ret = request_irq(spi->irq, dw1000_handle_interrupt, 0, "DW1000", dev);
	if (ret) {
		dev_err(&spi->dev, "interrupt setup failed");
		goto out_dev;
	}

	/* perform hardware initialization */
	ret = dw1000_hw_init(dev);
	if (ret) {
		dev_err(&spi->dev, "hw_init failed\n");
		goto out_dev;
	}

	ret = register_netdev(netdev);
	if (ret) {
		dev_err(&spi->dev, "register_netdev failed");
		goto out_irq;
	}

	ret = device_create_file(&netdev->dev, &dev_attr_channel);
	if (ret) {
		dev_err(&netdev->dev,
				"failed to register sysfs file 'channel'");
		goto out_netdev;
	}

	ret = device_create_file(&netdev->dev, &dev_attr_ifs);
	if (ret) {
		dev_err(&netdev->dev, "failed to register sysfs file 'ifs'");
		goto out_netdev;
	}

	return 0;

out_netdev:
	unregister_netdev(netdev);
out_irq:
	free_irq(spi->irq, dev);
out_dev:
	kfree(dev->spi_write_buf);
out_hw:
	free_netdev(netdev);
out:
	return ret;
}

/* driver unloading */
static int dw1000_remove(struct spi_device *spi)
{
	struct dw1000 *dev = spi_get_drvdata(spi);
	struct net_device *netdev = dev->netdev;

	dev_info(&netdev->dev, "remove()");

	unregister_netdev(netdev);
	free_irq(spi->irq, dev);
	kfree(dev->spi_write_buf);
	free_netdev(netdev);

	return 0;
}


/* device tree matching */
enum dw1000_modules { DW1000 };

static const struct of_device_id dw1000_of_match[] = {
	{ .compatible = "decawave,dw1000", .data = (void *)DW1000 },
	{ },
};
MODULE_DEVICE_TABLE(of, dw1000_of_match);


/* SPI driver registration */
static const struct spi_device_id dw1000_ids[] = {
	{ "dw1000", DW1000 },
	{ },
};
MODULE_DEVICE_TABLE(spi, dw1000_ids);

static struct spi_driver dw1000_driver = {
	.driver = {
		.of_match_table = of_match_ptr(dw1000_of_match),
		.name = "dw1000",
		.owner = THIS_MODULE,
	},
	.id_table = dw1000_ids,
	.probe = dw1000_probe,
	.remove = dw1000_remove,
};
module_spi_driver(dw1000_driver);

/* module meta information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Florian Tautz");
MODULE_DESCRIPTION("DW1000 SPI 802.15.4 Transceiver Driver");
