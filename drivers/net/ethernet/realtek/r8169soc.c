// SPDX-License-Identifier: GPL-2.0 OR MIT
/* r8169soc.c: RealTek 8169soc ethernet driver.
 *
 * Copyright (c) 2002 ShuChen <shuchen@realtek.com.tw>
 * Copyright (c) 2003 - 2007 Francois Romieu <romieu@fr.zoreil.com>
 * Copyright (c) 2014 YuKuen Wu <yukuen@realtek.com>
 * Copyright (c) 2015 Eric Wang <ericwang@realtek.com>
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 * Copyright (c) a lot of people too. Please respect their work.
 *
 * See MAINTAINERS file for support contact information.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/proc_fs.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/if_vlan.h>
#include <linux/crc32.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/version.h>
#include <linux/nvmem-consumer.h>
#include <linux/kthread.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/reset.h>

#include <soc/realtek/rtk_chip.h>

#include <linux/atomic.h>

#if defined(CONFIG_ARCH_RTD139x)
#include <linux/pinctrl/consumer.h>
#define R_K_DEFAULT		0x8
#define IDAC_FINE_DEFAULT	0x33
#elif defined(CONFIG_ARCH_RTD16xx)
#define RC_K_DEFAULT		0x8888
#define R_K_DEFAULT		0x8888
#define AMP_K_DEFAULT		0x7777
#define ADC_BIAS_K_DEFAULT	0x8888
#elif defined(CONFIG_ARCH_RTD13xx)
#define R_K_DEFAULT		0x8
#define IDAC_FINE_DEFAULT	0x77
#endif /* CONFIG_ARCH_RTD139x | CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD13xx */

#define RTL8169_VERSION "2.7.2"
#define MODULENAME "r8169"
#define PFX MODULENAME ": "

#if defined(CONFIG_ARCH_RTD139x)
#define RTL_HANDLE_RDU0
#elif defined(CONFIG_ARCH_RTD16xx) || defined(CONFIG_ARCH_RTD13xx)
#define RTL_TX_NO_CLOSE
#define RTL_ADJUST_FIFO_THRESHOLD
#define RTL_HANDLE_RDU0
#elif defined(CONFIG_ARCH_RTD16XXB)
#define RTL_OCP_MDIO
#define RTL_TX_NO_CLOSE
#define RTL_ADJUST_FIFO_THRESHOLD
#define RTL_HANDLE_RDU0
#endif /* CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD13xx | CONFIG_ARCH_RTD16XXB */

#define CURRENT_MDIO_PAGE 0xFFFFFFFF

#ifdef RTL8169_DEBUG
#define assert(expr) \
do {									\
	if (!(expr)) {							\
		pr_debug("Assertion failed! %s,%s,%s,line=%d\n",	\
		#expr, __FILE__, __func__, __LINE__);			\
	}								\
} while (0)
#define dprintk(fmt, args...) \
	pr_debug("\033[0;36m" PFX "\033[m" fmt, ## args)
#else
#define assert(expr) do {} while (0)
#define dprintk(fmt, args...)	do {} while (0)
#endif /* RTL8169_DEBUG */

#define R8169_MSG_DEFAULT \
	(NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_IFUP | NETIF_MSG_IFDOWN)

#define TX_SLOTS_AVAIL(tp) \
	((tp)->dirty_tx + NUM_TX_DESC - (tp)->cur_tx)

/* A skbuff with nr_frags needs nr_frags+1 entries in the tx queue */
#define TX_FRAGS_READY_FOR(tp, nr_frags) \
	(TX_SLOTS_AVAIL(tp) >= ((nr_frags) + 1))

/* Maximum number of multicast addresses to filter (vs. Rx-all-multicast).
 * The RTL chips use a 64 element hash table based on the Ethernet CRC.
 */
static const int multicast_filter_limit = 32;

#define MAX_READ_REQUEST_SHIFT	12
#define TX_DMA_BURST	4	/* Maximum PCI burst, '7' is unlimited */
#define InterFrameGap	0x03	/* 3 means InterFrameGap = the shortest one */

#define R8169_REGS_SIZE		256
#define R8169_NAPI_WEIGHT	64
#define NUM_TX_DESC	1024	/* Number of Tx descriptor registers */
#if defined(CONFIG_RTL_RX_NO_COPY)
#define NUM_RX_DESC	4096	/* Number of Rx descriptor registers */
#else /* CONFIG_RTL_RX_NO_COPY */
#define NUM_RX_DESC	1024	/* Number of Rx descriptor registers */
#endif /* CONFIG_RTL_RX_NO_COPY */
#define R8169_TX_RING_BYTES	(NUM_TX_DESC * sizeof(struct TxDesc))
#define R8169_RX_RING_BYTES	(NUM_RX_DESC * sizeof(struct RxDesc))

#define RTL8169_TX_TIMEOUT	(6 * HZ)

#if defined(CONFIG_RTL_RX_NO_COPY)
#define RX_BUF_SIZE	0x05F3	/* 0x05F3 = 1522bye + 1 */
#define RTK_RX_ALIGN	8
#endif /* CONFIG_RTL_RX_NO_COPY */

#define RTL_PROC 1

/* write/read MMIO register */
#define RTL_W8(reg, val8)	(writeb((val8), ioaddr + (reg)))
#define RTL_W16(reg, val16)	(writew((val16), ioaddr + (reg)))
#define RTL_W32(reg, val32)	(writel((val32), ioaddr + (reg)))
#define RTL_R8(reg)		(readb(ioaddr + (reg)))
#define RTL_R16(reg)		(readw(ioaddr + (reg)))
#define RTL_R32(reg)		(readl(ioaddr + (reg)))

enum mac_version {
	RTL_GIGA_MAC_VER_42 = 41,
	RTL_GIGA_MAC_NONE   = 0xff,
};

enum rtl_tx_desc_version {
	RTL_TD_0	= 0,
	RTL_TD_1	= 1,
};

#define JUMBO_1K	ETH_DATA_LEN
#define JUMBO_4K	(4 * 1024 - ETH_HLEN - 2)
#define JUMBO_6K	(6 * 1024 - ETH_HLEN - 2)
#define JUMBO_7K	(7 * 1024 - ETH_HLEN - 2)
#define JUMBO_9K	(9 * 1024 - ETH_HLEN - 2)

static const struct {
	const char *name;
	enum rtl_tx_desc_version txd_version;
	const char *fw_name;
	u16 jumbo_max;
	bool jumbo_tx_csum;
} rtl_chip_infos = {
	.name = "RTL8169SOC",
	.txd_version = RTL_TD_1,
	.fw_name = NULL,
	.jumbo_max = JUMBO_9K,
	.jumbo_tx_csum = false,
};

static const struct of_device_id r8169soc_dt_ids[] = {
	{ .compatible = "Realtek,r8168", },
	{},
};

MODULE_DEVICE_TABLE(of, r8169soc_dt_ids);

#if defined(CONFIG_RTL_RX_NO_COPY)
static int rx_buf_sz = 1523;	/* 0x05F3 = 1522bye + 1 */
static int rx_buf_sz_new = 1523;
#else
static int rx_buf_sz = 16383;
#endif /* CONFIG_RTL_RX_NO_COPY */

static struct {
	u32 msg_enable;
} debug = {-1};

enum rtl_registers {
	MAC0			= 0,	/* Ethernet hardware address. */
	MAC4			= 4,
	MAR0			= 8,	/* Multicast filter. */
	CounterAddrLow		= 0x10,
	CounterAddrHigh		= 0x14,
	LEDSEL			= 0x18,
	TxDescStartAddrLow	= 0x20,
	TxDescStartAddrHigh	= 0x24,
	TxHDescStartAddrLow	= 0x28,
	TxHDescStartAddrHigh	= 0x2c,
	FLASH			= 0x30,
	ERSR			= 0x36,
	ChipCmd			= 0x37,
	TxPoll			= 0x38,
	IntrMask		= 0x3c,
	IntrStatus		= 0x3e,

	TxConfig		= 0x40,
#define	TXCFG_AUTO_FIFO			BIT(7)	/* 8111e-vl */
#define	TXCFG_EMPTY			BIT(11)	/* 8111e-vl */

	RxConfig		= 0x44,
#define	RX128_INT_EN			BIT(15)	/* 8111c and later */
#define	RX_MULTI_EN			BIT(14)	/* 8111c only */
#define	RXCFG_FIFO_SHIFT		13
					/* No threshold before first PCI xfer */
#define	RX_FIFO_THRESH			(7 << RXCFG_FIFO_SHIFT)
#define	RX_EARLY_OFF			BIT(11)
#define	RXCFG_DMA_SHIFT			8
					/* Unlimited maximum PCI burst. */
#define	RX_DMA_BURST			(3 << RXCFG_DMA_SHIFT)	/* 128 bytes */

	RxMissed		= 0x4c,
	Cfg9346			= 0x50,
	Config0			= 0x51,
	Config1			= 0x52,
	Config2			= 0x53,
#define PME_SIGNAL			BIT(5)	/* 8168c and later */

	Config3			= 0x54,
	Config4			= 0x55,
	Config5			= 0x56,
	MultiIntr		= 0x5c,
	PHYAR			= 0x60,
	PHYstatus		= 0x6c,
	RxMaxSize		= 0xda,
	CPlusCmd		= 0xe0,
	IntrMitigate		= 0xe2,
	RxDescAddrLow		= 0xe4,
	RxDescAddrHigh		= 0xe8,
	EarlyTxThres		= 0xec,	/* 8169. Unit of 32 bytes. */

#define NoEarlyTx	0x3f	/* Max value : no early transmit. */

	MaxTxPacketSize		= 0xec,	/* 8101/8168. Unit of 128 bytes. */

#define TxPacketMax			(8064 >> 7)
#define EarlySize			0x27

	FuncEvent		= 0xf0,
	FuncEventMask		= 0xf4,
	FuncPresetState		= 0xf8,
	FuncForceEvent		= 0xfc,
};

enum rtl8110_registers {
	TBICSR			= 0x64,
	TBI_ANAR		= 0x68,
	TBI_LPAR		= 0x6a,
};

enum rtl8168_8101_registers {
	CSIDR			= 0x64,
	CSIAR			= 0x68,
#define	CSIAR_FLAG			0x80000000
#define	CSIAR_WRITE_CMD			0x80000000
#define	CSIAR_BYTE_ENABLE		0x0f
#define	CSIAR_BYTE_ENABLE_SHIFT		12
#define	CSIAR_ADDR_MASK			0x0fff
#define CSIAR_FUNC_CARD			0x00000000
#define CSIAR_FUNC_SDIO			0x00010000
#define CSIAR_FUNC_NIC			0x00020000
	PMCH			= 0x6f,
	EPHYAR			= 0x80,
#define	EPHYAR_FLAG			0x80000000
#define	EPHYAR_WRITE_CMD		0x80000000
#define	EPHYAR_REG_MASK			0x1f
#define	EPHYAR_REG_SHIFT		16
#define	EPHYAR_DATA_MASK		0xffff
	DLLPR			= 0xd0,
#define	PFM_EN				BIT(6)
	DBG_REG			= 0xd1,
#define	FIX_NAK_1			BIT(4)
#define	FIX_NAK_2			BIT(3)
	TWSI			= 0xd2,
	MCU			= 0xd3,
#define	NOW_IS_OOB			BIT(7)
#define	TX_EMPTY			BIT(5)
#define	RX_EMPTY			BIT(4)
#define	RXTX_EMPTY			(TX_EMPTY | RX_EMPTY)
#define	EN_NDP				BIT(3)
#define	EN_OOB_RESET			BIT(2)
#define	LINK_LIST_RDY			BIT(1)
#define	DIS_MCU_CLROOB			BIT(0)
	EFUSEAR			= 0xdc,
#define	EFUSEAR_FLAG			0x80000000
#define	EFUSEAR_WRITE_CMD		0x80000000
#define	EFUSEAR_READ_CMD		0x00000000
#define	EFUSEAR_REG_MASK		0x03ff
#define	EFUSEAR_REG_SHIFT		8
#define	EFUSEAR_DATA_MASK		0xff
};

enum rtl8168_registers {
	LED_FREQ		= 0x1a,
	EEE_LED			= 0x1b,
	ERIDR			= 0x70,
	ERIAR			= 0x74,
#define ERIAR_FLAG			0x80000000
#define ERIAR_WRITE_CMD			0x80000000
#define ERIAR_READ_CMD			0x00000000
#define ERIAR_ADDR_BYTE_ALIGN		4
#define ERIAR_TYPE_SHIFT		16
#define ERIAR_EXGMAC			(0x00 << ERIAR_TYPE_SHIFT)
#define ERIAR_MSIX			(0x01 << ERIAR_TYPE_SHIFT)
#define ERIAR_ASF			(0x02 << ERIAR_TYPE_SHIFT)
#define ERIAR_MASK_SHIFT		12
#define ERIAR_MASK_0001			(0x1 << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_0010			(0x2 << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_0011			(0x3 << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_0100			(0x4 << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_0101			(0x5 << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_1000			(0x8 << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_1100			(0xc << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_1111			(0xf << ERIAR_MASK_SHIFT)
	EPHY_RXER_NUM		= 0x7c,
	OCPDR			= 0xb0,	/* OCP GPHY access */
#define OCPDR_WRITE_CMD			0x80000000
#define OCPDR_READ_CMD			0x00000000
#define OCPDR_REG_MASK			0x7fff
#define OCPDR_REG_SHIFT			16
#define OCPDR_DATA_MASK			0xffff
	RDSAR1			= 0xd0,	/* 8168c only. Undocumented on 8168dp */
	MISC			= 0xf0,	/* 8168e only. */
#define TXPLA_RST			BIT(29)
#define DISABLE_LAN_EN			BIT(23) /* Enable GPIO pin */
#define PWM_EN				BIT(22)
#define RXDV_GATED_EN			BIT(19)
#define EARLY_TALLY_EN			BIT(16)
};

#if defined(RTL_TX_NO_CLOSE)
enum rtl8169soc_registers {
	TX_DESC_TAIL_IDX	= 0x20,	/* the last descriptor index */
	TX_DESC_CLOSE_IDX	= 0x22,	/* the closed descriptor index */
#define TX_DESC_CNT_MASK		0x3FFF
#define TX_DESC_CNT_SIZE		0x4000
};
#endif /* RTL_TX_NO_CLOSE */

enum rtl_register_content {
	/* InterruptStatusBits */
	SYSErr		= 0x8000,
	PCSTimeout	= 0x4000,
	SWInt		= 0x0100,
	TxDescUnavail	= 0x0080,
	RxFIFOOver	= 0x0040,
	LinkChg		= 0x0020,
	RxOverflow	= 0x0010,
	TxErr		= 0x0008,
	TxOK		= 0x0004,
	RxErr		= 0x0002,
	RxOK		= 0x0001,

	/* RxStatusDesc */
	RxBOVF		= BIT(24),
	RxFOVF		= BIT(23),
	RxRWT		= BIT(22),
	RxRES		= BIT(21),
	RxRUNT		= BIT(20),
	RxCRC		= BIT(19),

	/* ChipCmdBits */
	StopReq		= 0x80,
	CmdReset	= 0x10,
	CmdRxEnb	= 0x08,
	CmdTxEnb	= 0x04,
	RxBufEmpty	= 0x01,

	/* TXPoll register p.5 */
	HPQ		= 0x80,		/* Poll cmd on the high prio queue */
	NPQ		= 0x40,		/* Poll cmd on the low prio queue */
	FSWInt		= 0x01,		/* Forced software interrupt */

	/* Cfg9346Bits */
	Cfg9346_Lock	= 0x00,
	Cfg9346_Unlock	= 0xc0,

	/* rx_mode_bits */
	AcceptErr	= 0x20,
	AcceptRunt	= 0x10,
	AcceptBroadcast	= 0x08,
	AcceptMulticast	= 0x04,
	AcceptMyPhys	= 0x02,
	AcceptAllPhys	= 0x01,
#define RX_CONFIG_ACCEPT_MASK		0x3f

	/* TxConfigBits */
	TxInterFrameGapShift = 24,
	TxDMAShift = 8,	/* DMA burst value (0-7) is shift this many bits */

	/* Config1 register p.24 */
	LEDS1		= BIT(7),
	LEDS0		= BIT(6),
	Speed_down	= BIT(4),
	MEMMAP		= BIT(3),
	IOMAP		= BIT(2),
	VPD		= BIT(1),
	PMEnable	= BIT(0),	/* Power Management Enable */

	/* Config2 register p. 25 */
	ClkReqEn	= BIT(7),	/* Clock Request Enable */
	MSIEnable	= BIT(5),	/* 8169 only. Reserved in the 8168. */
	PCI_Clock_66MHz = 0x01,
	PCI_Clock_33MHz = 0x00,

	/* Config3 register p.25 */
	MagicPacket	= BIT(5),	/* Wake up when receives a Magic Packet */
	LinkUp		= BIT(4),	/* Wake up when the cable connection is re-established */
	Jumbo_En0	= BIT(2),	/* 8168 only. Reserved in the 8168b */
	Beacon_en	= BIT(0),	/* 8168 only. Reserved in the 8168b */

	/* Config4 register */
	Jumbo_En1	= BIT(1),	/* 8168 only. Reserved in the 8168b */

	/* Config5 register p.27 */
	BWF		= BIT(6),	/* Accept Broadcast wakeup frame */
	MWF		= BIT(5),	/* Accept Multicast wakeup frame */
	UWF		= BIT(4),	/* Accept Unicast wakeup frame */
	Spi_en		= BIT(3),
	LanWake		= BIT(1),	/* LanWake enable/disable */
	PMEStatus	= BIT(0),	/* PME status can be reset by PCI RST# */
	ASPM_en		= BIT(0),	/* ASPM enable */

	/* TBICSR p.28 */
	TBIReset	= 0x80000000,
	TBILoopback	= 0x40000000,
	TBINwEnable	= 0x20000000,
	TBINwRestart	= 0x10000000,
	TBILinkOk	= 0x02000000,
	TBINwComplete	= 0x01000000,

	/* CPlusCmd p.31 */
	EnableBist	= BIT(15),	/* 8168 8101 */
	Mac_dbgo_oe	= BIT(14),	/* 8168 8101 */
	Normal_mode	= BIT(13),	/* unused */
	Force_half_dup	= BIT(12),	/* 8168 8101 */
	Force_rxflow_en	= BIT(11),	/* 8168 8101 */
	Force_txflow_en	= BIT(10),	/* 8168 8101 */
	Cxpl_dbg_sel	= BIT(9),	/* 8168 8101 */
	ASF		= BIT(8),	/* 8168 8101 */
	PktCntrDisable	= BIT(7),	/* 8168 8101 */
	Mac_dbgo_sel	= 0x001c,	/* 8168 */
	RxVlan		= BIT(6),
	RxChkSum	= BIT(5),
	PCIDAC		= BIT(4),
	PCIMulRW	= BIT(3),
	INTT_0		= 0x0000,	/* 8168 */
	INTT_1		= 0x0001,	/* 8168 */
	INTT_2		= 0x0002,	/* 8168 */
	INTT_3		= 0x0003,	/* 8168 */

	/* rtl8169_PHYstatus */
	TBI_Enable	= 0x80,
	TxFlowCtrl	= 0x40,
	RxFlowCtrl	= 0x20,
	_1000bpsF	= 0x10,
	_100bps		= 0x08,
	_10bps		= 0x04,
	LinkStatus	= 0x02,
	FullDup		= 0x01,

	/* _TBICSRBit */
	TBILinkOK	= 0x02000000,

	/* DumpCounterCommand */
	CounterDump	= 0x8,
};

enum rtl_desc_bit {
	/* First doubleword. */
	DescOwn		= BIT(31), /* Descriptor is owned by NIC */
	RingEnd		= BIT(30), /* End of descriptor ring */
	FirstFrag	= BIT(29), /* First segment of a packet */
	LastFrag	= BIT(28), /* Final segment of a packet */
};

/* Generic case. */
enum rtl_tx_desc_bit {
	/* First doubleword. */
	TD_LSO		= BIT(27),		/* Large Send Offload */
#define TD_MSS_MAX			0x07ffu	/* MSS value */

	/* Second doubleword. */
	TxVlanTag	= BIT(17),		/* Add VLAN tag */
};

/* 8169, 8168b and 810x except 8102e. */
enum rtl_tx_desc_bit_0 {
	/* First doubleword. */
#define TD0_MSS_SHIFT			16	/* MSS position (11 bits) */
	TD0_TCP_CS	= BIT(16),		/* Calculate TCP/IP checksum */
	TD0_UDP_CS	= BIT(17),		/* Calculate UDP/IP checksum */
	TD0_IP_CS	= BIT(18),		/* Calculate IP checksum */
};

/* 8102e, 8168c and beyond. */
enum rtl_tx_desc_bit_1 {
	/* Second doubleword. */
#define TD1_MSS_SHIFT			18	/* MSS position (11 bits) */
	TD1_IP_CS	= BIT(29),		/* Calculate IP checksum */
	TD1_TCP_CS	= BIT(30),		/* Calculate TCP/IP checksum */
	TD1_UDP_CS	= BIT(31),		/* Calculate UDP/IP checksum */
};

static const struct rtl_tx_desc_info {
	struct {
		u32 udp;
		u32 tcp;
	} checksum;
	u16 mss_shift;
	u16 opts_offset;
} tx_desc_info = {
	.checksum = {
		.udp	= TD1_IP_CS | TD1_UDP_CS,
		.tcp	= TD1_IP_CS | TD1_TCP_CS
	},
	.mss_shift	= TD1_MSS_SHIFT,
	.opts_offset	= 1
};

enum rtl_rx_desc_bit {
	/* Rx private */
	PID1		= BIT(18), /* Protocol ID bit 1/2 */
	PID0		= BIT(17), /* Protocol ID bit 2/2 */

#define RxProtoUDP	(PID1)
#define RxProtoTCP	(PID0)
#define RxProtoIP	(PID1 | PID0)
#define RxProtoMask	RxProtoIP

	IPFail		= BIT(16), /* IP checksum failed */
	UDPFail		= BIT(15), /* UDP/IP checksum failed */
	TCPFail		= BIT(14), /* TCP/IP checksum failed */
	RxVlanTag	= BIT(16), /* VLAN tag available */
};

#define RsvdMask	0x3fffc000

struct TxDesc {
	__le32 opts1;
	__le32 opts2;
	__le64 addr;
};

struct RxDesc {
	__le32 opts1;
	__le32 opts2;
	__le64 addr;
};

struct ring_info {
	struct sk_buff	*skb;
	u32		len;
	u8		__pad[sizeof(void *) - sizeof(u32)];
};

enum features {
	RTL_FEATURE_WOL		= BIT(0),
	RTL_FEATURE_MSI		= BIT(1),
	RTL_FEATURE_GMII	= BIT(2),
};

struct rtl8169_counters {
	__le64	tx_packets;
	__le64	rx_packets;
	__le64	tx_errors;
	__le32	rx_errors;
	__le16	rx_missed;
	__le16	align_errors;
	__le32	tx_one_collision;
	__le32	tx_multi_collision;
	__le64	rx_unicast;
	__le64	rx_broadcast;
	__le32	rx_multicast;
	__le16	tx_aborted;
	__le16	tx_underrun;
};

enum rtl_flag {
	RTL_FLAG_TASK_ENABLED,
	RTL_FLAG_TASK_SLOW_PENDING,
	RTL_FLAG_TASK_RESET_PENDING,
	RTL_FLAG_TASK_PHY_PENDING,
	RTL_FLAG_MAX
};

struct rtl8169_stats {
	u64			packets;
	u64			bytes;
	struct u64_stats_sync	syncp;
};

#define RGMII0_PAD_CTRL_ADDR	0x9801a960
#define ISO_RGMII_MDIO_TO_GMAC	0x98007064

enum rtl_output_mode {
	OUTPUT_EMBEDDED_PHY,
	#if defined(CONFIG_ARCH_RTD129x)
	OUTPUT_RGMII_TO_MAC,
	OUTPUT_RGMII_TO_PHY,
	#elif defined(CONFIG_ARCH_RTD139x) || defined(CONFIG_ARCH_RTD16xx)
	OUTPUT_SGMII_TO_MAC,
	OUTPUT_SGMII_TO_PHY,
	#elif defined(CONFIG_ARCH_RTD13xx)
	OUTPUT_RGMII_TO_MAC,
	OUTPUT_RGMII_TO_PHY,
	OUTPUT_RMII,
	#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x |
		* CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD13xx
		*/
	OUTPUT_MAX
};

/* ISO base addr 0x98007000 */
enum iso_registers {
	ISO_UMSK_ISR		= 0x0004,
	ISO_PWRCUT_ETN		= 0x005c,
	ISO_ETN_TESTIO		= 0x0060,
	ISO_SOFT_RESET		= 0x0088,
	ISO_CLOCK_ENABLE	= 0x008c,
	ISO_POR_CTRL		= 0x0210,
	ISO_POR_VTH		= 0x0214,
	ISO_POR_DATAI		= 0x0218,
	#if defined(CONFIG_ARCH_RTD13xx)
	ISO_ETN_DBUS_CTRL	= 0x0fc0,
	#endif /* CONFIG_ARCH_RTD13xx */
};

#define POR_XV_MASK	0x00000111
#define POR_NUM		3

enum phy_addr_e {
	INT_PHY_ADDR = 1,	/* embedded PHY PHY ID */
	SERDES_DPHY_0 = 0,	/* embedded SerDes DPHY PHY ID 0 */
	SERDES_DPHY_1 = 1,	/* embedded SerDes DPHY PHY ID 1 */
	EXT_PHY_ADDR = 3,	/* external RTL8211FS SGMII PHY ID */
};

#if defined(CONFIG_ARCH_RTD129x)
enum rtl_rgmii_voltage {
	VOLTAGE_1_DOT_8V = 1,
	VOLTAGE_2_DOT_5V,
	VOLTAGE_3_DOT_3V,
	VOLTAGE_MAX
};

enum rtl_rgmii_delay {
	RGMII_DELAY_0NS,
	RGMII_DELAY_2NS,
	RGMII_DELAY_MAX
};
#elif defined(CONFIG_ARCH_RTD139x) || defined(CONFIG_ARCH_RTD16xx) || \
	defined(CONFIG_ARCH_RTD13xx)
/* SDS base addr 0x981c8000 */
#if defined(CONFIG_ARCH_RTD139x)
enum sds_registers {
	SDS_REG02	= 0x0008,
	SDS_REG28	= 0x0070,
	SDS_REG29	= 0x0074,
	SDS_MISC	= 0x1804,
	SDS_LINK	= 0x1810,
};
#elif defined(CONFIG_ARCH_RTD16xx)
enum sds_registers {
	SDS_REG02	= 0x0008,
	SDS_MISC	= 0x1804,
	SDS_LINK	= 0x180c,
	SDS_DEBUG	= 0x1810,
};
#endif

/* ISO testmux base addr 0x9804e000 */
enum testmux_registers {
	ISO_TESTMUX_MUXPAD0	= 0x0000,
	ISO_TESTMUX_MUXPAD1	= 0x0004,
	ISO_TESTMUX_MUXPAD2	= 0x0008,
	#if defined(CONFIG_ARCH_RTD13xx)
	ISO_TESTMUX_MUXPAD5	= 0x0014,
	ISO_TESTMUX_MUXPAD6	= 0x0018,
	ISO_TESTMUX_PFUNC9	= 0x0040, /* MDC/MDIO current */
	ISO_TESTMUX_PFUNC20	= 0x006c, /* RGMII current */
	ISO_TESTMUX_PFUNC21	= 0x0070, /* RGMII current */
	ISO_TESTMUX_PFUNC25	= 0x0090, /* RGMII BIAS */
	#endif
};

#if defined(CONFIG_ARCH_RTD16xx)
enum sgmii_swing_e {
	TX_Swing_550mV,
	TX_Swing_380mV,
	TX_Swing_250mV,
	TX_Swing_190mV
};
#else
#define SGMII_SWING		(0X3 << 8)
#define TX_SWING_1040MV		(0X0 << 8)	/* DEFAULT */
#define TX_SWING_693MV		(0X1 << 8)
#define TX_SWING_474MV		(0X2 << 8)
#define TX_SWING_352MV		(0X3 << 8)
#define TX_SWING_312MV		(0X4 << 8)
#endif /* CONFIG_ARCH_RTD16xx */

/* SBX and SC_WRAP base addr 0x9801c000 */
enum sbx_sc_wrap_registers {
	SBX_SB3_CHANNEL_REQ_MASK	= 0x020c,
	SBX_SB3_CHANNEL_REQ_BUSY	= 0x0210,
	SBX_ACP_CHANNEL_REQ_MASK	= 0x080c,
	SBX_ACP_CHANNEL_REQ_BUSY	= 0x0810,
	SBX_ACP_MISC_CTRL		= 0x0814,
	SC_WRAP_ACP_CRT_CTRL		= 0x1030,
	SC_WRAP_CRT_CTRL		= 0x1100,
	SC_WRAP_INTERFACE_EN		= 0x1124,
	SC_WRAP_ACP_CTRL		= 0x1800,
};
#elif defined(CONFIG_ARCH_RTD16XXB)
/* ISO testmux base addr 0x9804e000 */
enum testmux_registers {
	ISO_TESTMUX_MUXPAD2	= 0x0008,
	ISO_TESTMUX_PFUNC12	= 0x0050, /* MDC/MDIO current */
};
#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x |
	* CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD13xx |
	* CONFIG_ARCH_RTD16XXB
	*/

#define RTL_WPD_SIZE		16
#define RTL_WPD_MASK_SIZE	16

#ifdef RTL_PROC
static struct proc_dir_entry *rtw_proc;
#endif
/* wol_enable
 * BIT 0: WoL enable
 * BIT 1: CRC match
 * BIT 2: WPD
 */
enum wol_flags {
	WOL_MAGIC			= 0x1,
	WOL_CRC_MATCH			= 0x2,
	WOL_WPD				= 0x4,
};

#define WOL_BUF_LEN		128

struct rtl8169_private {
	void __iomem *mmio_addr;	/* memory map physical address */
	void __iomem *mmio_clkaddr;
	struct platform_device *pdev;
	struct net_device *dev;
	struct napi_struct napi;
	u32 msg_enable;
	u16 txd_version;
	u16 mac_version;
	u32 cur_rx; /* Index into the Rx descriptor buffer of next Rx pkt. */
	u32 cur_tx; /* Index into the Tx descriptor buffer of next Rx pkt. */
	u32 dirty_tx;

	#if defined(CONFIG_RTL_RX_NO_COPY)
	u32 dirty_rx;
	u32 last_dirty_tx;
	u32 tx_reset_count;
	u32 last_cur_rx;
	u32 rx_reset_count;
	u8 checkRDU;
	#endif /* CONFIG_RTL_RX_NO_COPY */

	struct rtl8169_stats rx_stats;
	struct rtl8169_stats tx_stats;
	struct TxDesc *TxDescArray;	/* 256-aligned Tx descriptor ring */
	struct RxDesc *RxDescArray;	/* 256-aligned Rx descriptor ring */
	dma_addr_t TxPhyAddr;
	dma_addr_t RxPhyAddr;

	#if defined(CONFIG_RTL_RX_NO_COPY)
	struct sk_buff *Rx_databuff[NUM_RX_DESC]; /* Rx data buffers */
	#else
	void *Rx_databuff[NUM_RX_DESC];	/* Rx data buffers */
	#endif /* CONFIG_RTL_RX_NO_COPY */

	struct ring_info tx_skb[NUM_TX_DESC];	/* Tx data buffers */
	u16 cp_cmd;

	u16 event_slow;

#ifdef RTL_PROC
	struct proc_dir_entry *dir_dev;
#endif

	struct mdio_ops {
		void (*write)(struct rtl8169_private *tp, int page, int reg, int value);
		int (*read)(struct rtl8169_private *tp, int page, int reg);
	} mdio_ops;

	struct mmd_ops {
		void (*write)(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr, u32 value);
		u32 (*read)(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr);
	} mmd_ops;

	struct pll_power_ops {
		void (*down)(struct rtl8169_private *tp);
		void (*up)(struct rtl8169_private *tp);
	} pll_power_ops;

	struct jumbo_ops {
		void (*enable)(struct rtl8169_private *tp);
		void (*disable)(struct rtl8169_private *tp);
	} jumbo_ops;

	struct csi_ops {
		void (*write)(struct rtl8169_private *tp, int addr, int value);
		u32 (*read)(struct rtl8169_private *tp, int addr);
	} csi_ops;

	int (*set_speed)(struct net_device *dev, u8 aneg, u16 sp, u8 dpx, u32 adv);
	int (*get_settings)(struct net_device *dev, struct ethtool_cmd *cmd);
	void (*phy_reset_enable)(struct rtl8169_private *tp);
	void (*hw_start)(struct net_device *dev);
	unsigned int (*phy_reset_pending)(struct rtl8169_private *tp);
	unsigned int (*link_ok)(void __iomem *ioaddr);
	int (*do_ioctl)(struct rtl8169_private *tp, struct mii_ioctl_data *data, int cmd);

	struct {
		DECLARE_BITMAP(flags, RTL_FLAG_MAX);
		struct mutex mutex; /* mutex for work */
		struct work_struct work;
	} wk;

	unsigned int features;

	struct mii_if_info mii;
	struct rtl8169_counters counters;
	u32 saved_wolopts;
	u32 opts1_mask;

	u8 wol_enable;
	u8 wol_crc_cnt;
	u8 wol_mask[RTL_WPD_SIZE][RTL_WPD_MASK_SIZE];
	u8 wol_mask_size[RTL_WPD_SIZE];
	u32 wol_crc[RTL_WPD_SIZE];

	u32 ocp_base;
	u32 led_cfg;

	/* For link status change. */
	struct task_struct *kthr;
	wait_queue_head_t thr_wait;
	int link_chg;
	u32 phy_irq[POR_NUM];
	u8 phy_irq_map[POR_NUM];
	u8 phy_irq_num;
	u32 phy_por_xv_mask;
	atomic_t phy_reinit_flag;

	#if defined(CONFIG_ARCH_RTD129x)
	/* RGMII */
	u8 output_mode; /* 0:embedded PHY, 1:RGMII to MAC, 2:RGMII to PHY */
	u8 rgmii_voltage; /* 1:1.8V, 2: 2.5V, 3:3.3V */
	u8 rgmii_tx_delay; /* 0: 0ns, 1: 2ns */
	u8 rgmii_rx_delay; /* 0: 0ns, 1: 2ns */
	#elif defined(CONFIG_ARCH_RTD139x) || defined(CONFIG_ARCH_RTD16xx)
	void __iomem *mmio_sbxaddr;
	void __iomem *mmio_otpaddr;
	void __iomem *mmio_sdsaddr;
	void __iomem *mmio_pinmuxaddr;

	/* SGMII */
	u8 output_mode; /* 0:embedded PHY, 1:SGMII to MAC, 2:SGMII to PHY */
	#if defined(CONFIG_ARCH_RTD16xx)
	u8 sgmii_swing; /* 0:640mV, 1:380mV, 2:250mV, 3:190mV */
	#endif /* CONFIG_ARCH_RTD16xx */

	bool bypass_enable; /* 0: disable, 1: enable */
	#elif defined(CONFIG_ARCH_RTD13xx)
	/* 0:embedded PHY, 1:RGMII to MAC, 2:RGMII to PHY, 3:RMII */
	u8 output_mode;
	u8 voltage; /* 1:1.8V, 2: 2.5V, 3:3.3V */
	u8 tx_delay; /* 0: 0ns, 1: 2ns */
	u8 rx_delay; /* 0 ~ 7 x 0.5ns */

	void __iomem *mmio_sbxaddr;
	void __iomem *mmio_otpaddr;
	void __iomem *mmio_pinmuxaddr;

	bool bypass_enable; /* 0: disable, 1: enable */
	#elif defined(CONFIG_ARCH_RTD16XXB)
	u8 output_mode; /* 0:embedded PHY */

	void __iomem *mmio_pinmuxaddr;

	bool bypass_enable; /* 0: disable, 1: enable */
	#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x |
		* CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD13xx |
		* CONFIG_ARCH_RTD16XXB
		*/
	u8 ext_phy_id; /* 0 ~ 31 */
	bool eee_enable; /* 0: disable, 1: enable */
	bool acp_enable; /* 0: disable, 1: enable */
	bool ext_phy;
};

MODULE_AUTHOR("Realtek and the Linux r8169soc crew <netdev@vger.kernel.org>");
MODULE_DESCRIPTION("RealTek RTL-8169soc Gigabit Ethernet driver");
module_param_named(debug, debug.msg_enable, int, 0644);
MODULE_PARM_DESC(debug, "Debug verbosity level (0=none, ..., 16=all)");
MODULE_LICENSE("GPL");
MODULE_VERSION(RTL8169_VERSION);

static void rtl_lock_work(struct rtl8169_private *tp)
{
	mutex_lock(&tp->wk.mutex);
}

static void rtl_unlock_work(struct rtl8169_private *tp)
{
	mutex_unlock(&tp->wk.mutex);
}

struct rtl_cond {
	bool (*check)(struct rtl8169_private *tp);
	const char *msg;
};

static void rtl_udelay(unsigned int d)
{
	udelay(d);
}

static bool rtl_loop_wait(struct rtl8169_private *tp, const struct rtl_cond *c,
			  void (*delay)(unsigned int),
			  unsigned int d, int n, bool high)
{
	int i;

	for (i = 0; i < n; i++) {
		delay(d);
		if (c->check(tp) == high)
			return true;
	}
	netif_err(tp, drv, tp->dev, "%s == %d (loop: %d, delay: %d).\n",
		  c->msg, !high, n, d);
	return false;
}

static bool rtl_udelay_loop_wait_high(struct rtl8169_private *tp,
				      const struct rtl_cond *c,
				      unsigned int d, int n)
{
	return rtl_loop_wait(tp, c, rtl_udelay, d, n, true);
}

static bool rtl_udelay_loop_wait_low(struct rtl8169_private *tp,
				     const struct rtl_cond *c,
				     unsigned int d, int n)
{
	return rtl_loop_wait(tp, c, rtl_udelay, d, n, false);
}

static __maybe_unused bool
rtl_msleep_loop_wait_high(struct rtl8169_private *tp,
			  const struct rtl_cond *c, unsigned int d, int n)
{
	return rtl_loop_wait(tp, c, msleep, d, n, true);
}

static bool rtl_msleep_loop_wait_low(struct rtl8169_private *tp,
				     const struct rtl_cond *c,
				     unsigned int d, int n)
{
	return rtl_loop_wait(tp, c, msleep, d, n, false);
}

#define DECLARE_RTL_COND(name)				\
static bool name ## _check(struct rtl8169_private *);	\
							\
static const struct rtl_cond name = {			\
	.check	= name ## _check,			\
	.msg	= #name					\
};							\
							\
static bool name ## _check(struct rtl8169_private *tp)

DECLARE_RTL_COND(rtl_eriar_cond)
{
	void __iomem *ioaddr = tp->mmio_addr;

	return RTL_R32(ERIAR) & ERIAR_FLAG;
}

static bool rtl_ocp_reg_failure(struct rtl8169_private *tp, u32 reg)
{
	if (reg & 0xffff0001) {
		netif_err(tp, drv, tp->dev, "Invalid ocp reg %x!\n", reg);
		return true;
	}
	return false;
}

static void rtl_ocp_write(struct rtl8169_private *tp, u32 reg, u32 value)
{
	void __iomem *ioaddr = tp->mmio_addr;
	unsigned int wdata;

	if (rtl_ocp_reg_failure(tp, reg))
		return;

	wdata = OCPDR_WRITE_CMD |
		(((reg >> 1) & OCPDR_REG_MASK) << OCPDR_REG_SHIFT) |
		(value & OCPDR_DATA_MASK);
	RTL_W32(OCPDR, wdata);
}

static u32 rtl_ocp_read(struct rtl8169_private *tp, u32 reg)
{
	void __iomem *ioaddr = tp->mmio_addr;
	unsigned int wdata;
	unsigned int rdata;

	if (rtl_ocp_reg_failure(tp, reg))
		return 0;

	wdata = OCPDR_READ_CMD |
		(((reg >> 1) & OCPDR_REG_MASK) << OCPDR_REG_SHIFT);
	RTL_W32(OCPDR, wdata);
	rdata = RTL_R32(OCPDR);
	return (rdata & OCPDR_DATA_MASK);
}

#define OCP_STD_PHY_BASE	0xa400

static __maybe_unused void mac_mcu_write(struct rtl8169_private *tp, int reg,
					 int value)
{
	if (reg == 0x1f) {
		tp->ocp_base = value << 4;
		return;
	}

	rtl_ocp_write(tp, tp->ocp_base + reg, value);
}

static __maybe_unused int mac_mcu_read(struct rtl8169_private *tp, int reg)
{
	return rtl_ocp_read(tp, tp->ocp_base + reg);
}

#if defined(CONFIG_ARCH_RTD129x)
#define MDIO_LOCK							\
do {									\
	/* disable interrupt from PHY to MCU */				\
	rtl_ocp_write(tp, 0xfc1e,					\
		rtl_ocp_read(tp, 0xfc1e) &				\
		~(BIT(1) | BIT(11) | BIT(12)));				\
} while (0)
#define MDIO_UNLOCK							\
do {									\
	u32 tmp;							\
	/* enable interrupt from PHY to MCU */				\
	tmp = rtl_ocp_read(tp, 0xfc1e);					\
	if (tp->output_mode == OUTPUT_RGMII_TO_MAC)			\
		tmp |= (BIT(11) | BIT(12)); /* ignore BIT(1):mac_intr*/	\
	else								\
		tmp |= (BIT(1) | BIT(11) | BIT(12));			\
	rtl_ocp_write(tp, 0xfc1e, tmp);					\
} while (0)
#elif defined(CONFIG_ARCH_RTD139x) || defined(CONFIG_ARCH_RTD16xx) || \
	defined(CONFIG_ARCH_RTD13xx) || defined(CONFIG_ARCH_RTD16XXB)
#define MDIO_WAIT_TIMEOUT	100
#define MDIO_LOCK							\
do {									\
	u32 wait_cnt = 0;						\
	u32 log_de4e = 0;						\
									\
	/* disable EEE IMR */						\
	rtl_ocp_write(tp, 0xE044,					\
		(rtl_ocp_read(tp, 0xE044) &				\
		~(BIT(3) | BIT(2) | BIT(1) | BIT(0))));			\
	/* disable timer 2 */						\
	rtl_ocp_write(tp, 0xE404,					\
		(rtl_ocp_read(tp, 0xE404) | BIT(9)));			\
	/* wait MDIO channel is free */					\
	log_de4e = BIT(0) & rtl_ocp_read(tp, 0xDE4E);			\
	log_de4e = (log_de4e << 1) |					\
		(BIT(0) & rtl_ocp_read(tp, 0xDE4E));			\
	/* check if 0 for continuous 2 times */				\
	while (0 != (((0x1 << 2) - 1) & log_de4e)) {			\
		wait_cnt++;						\
		udelay(1);						\
		log_de4e = (log_de4e << 1) | (BIT(0) &			\
			rtl_ocp_read(tp, 0xDE4E));			\
		if (wait_cnt > MDIO_WAIT_TIMEOUT)			\
			break;						\
	}								\
	/* enter driver mode */						\
	rtl_ocp_write(tp, 0xDE42,					\
		rtl_ocp_read(tp, 0xDE42) | BIT(0));			\
	if (wait_cnt > MDIO_WAIT_TIMEOUT)				\
		pr_err("%s:%d: MDIO lock failed\n", __func__, __LINE__);\
} while (0)

#define MDIO_UNLOCK							\
do {									\
	/* exit driver mode */						\
	rtl_ocp_write(tp, 0xDE42,					\
			rtl_ocp_read(tp, 0xDE42) & ~BIT(0));		\
	/* enable timer 2 */						\
	rtl_ocp_write(tp, 0xE404,					\
		(rtl_ocp_read(tp, 0xE404) & ~BIT(9)));			\
	/* enable EEE IMR */						\
	rtl_ocp_write(tp, 0xE044,					\
		(rtl_ocp_read(tp, 0xE044) |				\
		BIT(3) | BIT(2) | BIT(1) | BIT(0)));			\
} while (0)
#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x |
	* CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD16XXB
	*/

DECLARE_RTL_COND(rtl_int_phyar_cond)
{
	return rtl_ocp_read(tp, 0xDE0A) & BIT(14);
}

DECLARE_RTL_COND(rtl_phyar_cond)
{
	void __iomem *ioaddr = tp->mmio_addr;

	return RTL_R32(PHYAR) & 0x80000000;
}

static void __int_set_phy_addr(struct rtl8169_private *tp, int phy_addr)
{
	u32 tmp;

	tmp = readl(tp->mmio_clkaddr + ISO_PWRCUT_ETN);
	tmp &= ~(BIT(20) | BIT(19) | BIT(18) | BIT(17) | BIT(16));
	tmp |= (phy_addr << 16);
	writel(tmp, tp->mmio_clkaddr + ISO_PWRCUT_ETN);
}

#if !defined(RTL_OCP_MDIO)
static inline int __int_mdio_read(struct rtl8169_private *tp, int reg)
{
	int value;
	void __iomem *ioaddr = tp->mmio_addr;

	/* read reg */
	RTL_W32(PHYAR, 0x0 | (reg & 0x1f) << 16);

	value = rtl_udelay_loop_wait_high(tp, &rtl_phyar_cond, 25, 20) ?
		RTL_R32(PHYAR) & 0xffff : ~0;

	/* According to hardware specs a 20us delay is required after read
	 * complete indication, but before sending next command.
	 */
	usleep_range(20, 21);

	return value;
}

static inline void __int_mdio_write(struct rtl8169_private *tp, int reg, int value)
{
	void __iomem *ioaddr = tp->mmio_addr;

	/* write reg */
	RTL_W32(PHYAR, 0x80000000 | (reg & 0x1f) << 16 | (value & 0xffff));

	rtl_udelay_loop_wait_low(tp, &rtl_phyar_cond, 25, 20);
	/* According to hardware specs a 20us delay is required after write
	 * complete indication, but before sending next command.
	 */
	usleep_range(20, 21);
}
#endif /* !RTL_OCP_MDIO */

static int int_mdio_read(struct rtl8169_private *tp, int page, int reg)
{
	int value;
#if defined(RTL_OCP_MDIO)

	rtl_ocp_write(tp, 0xDE0C, page);
	rtl_ocp_write(tp, 0xDE0A, reg & 0x1f);

	value = rtl_udelay_loop_wait_low(tp, &rtl_int_phyar_cond, 25, 20) ?
		rtl_ocp_read(tp, 0xDE08) & 0xffff : ~0;

#else /* RTL_OCP_MDIO */
	MDIO_LOCK;

	/* write page */
	if (page != CURRENT_MDIO_PAGE)
		__int_mdio_write(tp, 0x1f, page);

	/* read reg */
	value = __int_mdio_read(tp, reg);

	MDIO_UNLOCK;
#endif /* RTL_OCP_MDIO */
	return value;
}

static void int_mdio_write(struct rtl8169_private *tp, int page, int reg, int value)
{
#if defined(RTL_OCP_MDIO)
	rtl_ocp_write(tp, 0xDE0C, page);
	rtl_ocp_write(tp, 0xDE08, value);
	rtl_ocp_write(tp, 0xDE0A, BIT(15) | (reg & 0x1f));

	rtl_udelay_loop_wait_low(tp, &rtl_int_phyar_cond, 25, 20);
#else /* RTL_OCP_MDIO */
	MDIO_LOCK;

	/* write page */
	if (page != CURRENT_MDIO_PAGE)
		__int_mdio_write(tp, 0x1f, page);

	/* write reg */
	__int_mdio_write(tp, reg, value);

	MDIO_UNLOCK;
#endif /* RTL_OCP_MDIO */
}

DECLARE_RTL_COND(rtl_ext_phyar_cond)
{
	return rtl_ocp_read(tp, 0xDE2A) & BIT(14);
}

DECLARE_RTL_COND(rtl_ephyar_cond)
{
	void __iomem *ioaddr = tp->mmio_addr;

	return RTL_R32(EPHYAR) & EPHYAR_FLAG;
}

static void __ext_set_phy_addr(struct rtl8169_private *tp, int phy_addr)
{
	u32 tmp;

	tmp = readl(tp->mmio_clkaddr + ISO_PWRCUT_ETN);
	tmp &= ~(BIT(25) | BIT(24) | BIT(23) | BIT(22) | BIT(21));
	tmp |= (phy_addr << 21);
	writel(tmp, tp->mmio_clkaddr + ISO_PWRCUT_ETN);
}

#if !defined(RTL_OCP_MDIO)
static inline int __ext_mdio_read(struct rtl8169_private *tp, int reg)
{
	int value;
	void __iomem *ioaddr = tp->mmio_addr;

	/* read reg */
	RTL_W32(EPHYAR, (reg & EPHYAR_REG_MASK) << EPHYAR_REG_SHIFT);

	value = rtl_udelay_loop_wait_high(tp, &rtl_ephyar_cond, 10, 100) ?
		RTL_R32(EPHYAR) & EPHYAR_DATA_MASK : ~0;

	return value;
}

static inline void __ext_mdio_write(struct rtl8169_private *tp, int reg, int value)
{
	void __iomem *ioaddr = tp->mmio_addr;

	/* write reg */
	RTL_W32(EPHYAR, EPHYAR_WRITE_CMD | (value & EPHYAR_DATA_MASK) |
		(reg & EPHYAR_REG_MASK) << EPHYAR_REG_SHIFT);

	rtl_udelay_loop_wait_low(tp, &rtl_ephyar_cond, 10, 100);

	usleep_range(10, 11);
}
#endif /* !RTL_OCP_MDIO */

static int ext_mdio_read(struct rtl8169_private *tp, int page, int reg)
{
	int value;
#if defined(RTL_OCP_MDIO)

	rtl_ocp_write(tp, 0xDE2C, page);
	rtl_ocp_write(tp, 0xDE2A, reg & 0x1f);

	value = rtl_udelay_loop_wait_low(tp, &rtl_ext_phyar_cond, 25, 20) ?
		rtl_ocp_read(tp, 0xDE28) & 0xffff : ~0;
#else /* RTL_OCP_MDIO */
	MDIO_LOCK;

	/* write page */
	if (page != CURRENT_MDIO_PAGE)
		__ext_mdio_write(tp, 0x1f, page);

	/* read reg */
	value = __ext_mdio_read(tp, reg);

	MDIO_UNLOCK;
#endif /* RTL_OCP_MDIO */
	return value;
}

static void ext_mdio_write(struct rtl8169_private *tp, int page, int reg, int value)
{
#if defined(RTL_OCP_MDIO)
	rtl_ocp_write(tp, 0xDE2C, page);
	rtl_ocp_write(tp, 0xDE28, value);
	rtl_ocp_write(tp, 0xDE2A, BIT(15) | (reg & 0x1f));

	rtl_udelay_loop_wait_low(tp, &rtl_ext_phyar_cond, 25, 20);
#else /* RTL_OCP_MDIO */
	MDIO_LOCK;

	/* write page */
	if (page != CURRENT_MDIO_PAGE)
		__ext_mdio_write(tp, 0x1f, page);

	/* write reg */
	__ext_mdio_write(tp, reg, value);

	MDIO_UNLOCK;
#endif /* RTL_OCP_MDIO */
}

static void rtl_init_mdio_ops(struct rtl8169_private *tp)
{
	struct mdio_ops *ops = &tp->mdio_ops;

	if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
		/* set int PHY addr */
		__int_set_phy_addr(tp, INT_PHY_ADDR);

		ops->write	= int_mdio_write;
		ops->read	= int_mdio_read;
	} else {
		/* set ext PHY addr */
		__ext_set_phy_addr(tp, EXT_PHY_ADDR);

		ops->write	= ext_mdio_write;
		ops->read	= ext_mdio_read;
	}
}

static inline void rtl_phy_write(struct rtl8169_private *tp, int page, int reg, u32 val)
{
	tp->mdio_ops.write(tp, page, reg, val);
}

static inline int rtl_phy_read(struct rtl8169_private *tp, int page, int reg)
{
	return tp->mdio_ops.read(tp, page, reg);
}

static void rtl_mdio_write(struct net_device *dev, int phy_id, int location, int val)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	rtl_phy_write(tp, 0, location, val);
}

static int rtl_mdio_read(struct net_device *dev, int phy_id, int location)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	return rtl_phy_read(tp, 0, location);
}

static inline void rtl_patchphy(struct rtl8169_private *tp, int page,
				int reg_addr, int value)
{
	rtl_phy_write(tp, page, reg_addr,
		      rtl_phy_read(tp, page, reg_addr) | value);
}

static inline void rtl_w1w0_phy(struct rtl8169_private *tp, int page,
				int reg_addr, int p, int m)
{
	rtl_phy_write(tp, page, reg_addr,
		      (rtl_phy_read(tp, page, reg_addr) | p) & ~m);
}

void int_mmd_write(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr, u32 value)
{
#if defined(RTL_OCP_MDIO)
	/* address mode */
	int_mdio_write(tp, 0, 13, (0x1f & dev_addr));
	/* write the desired address */
	int_mdio_write(tp, 0, 14, reg_addr);
	/* data mode, no post increment */
	int_mdio_write(tp, 0, 13, ((0x1f & dev_addr) | (0x1 << 14)));
	/* write the content of the selected register */
	int_mdio_write(tp, 0, 14, value);
#else /* RTL_OCP_MDIO */
	MDIO_LOCK;

	/* set page 0 */
	__int_mdio_write(tp, 0x1f, 0);
	/* address mode */
	__int_mdio_write(tp, 13, (0x1f & dev_addr));
	/* write the desired address */
	__int_mdio_write(tp, 14, reg_addr);
	/* data mode, no post increment */
	__int_mdio_write(tp, 13, ((0x1f & dev_addr) | (0x1 << 14)));
	/* write the content of the selected register */
	__int_mdio_write(tp, 14, value);

	MDIO_UNLOCK;
#endif /* RTL_OCP_MDIO */
}

u32 int_mmd_read(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr)
{
	u32 value;

#if defined(RTL_OCP_MDIO)
	/* address mode */
	int_mdio_write(tp, 0, 13, (0x1f & dev_addr));
	/* write the desired address */
	int_mdio_write(tp, 0, 14, reg_addr);
	/* data mode, no post increment */
	int_mdio_write(tp, 0, 13, ((0x1f & dev_addr) | (0x1 << 14)));
	/* read the content of the selected register */
	value = int_mdio_read(tp, 0, 14);
#else /* RTL_OCP_MDIO */
	MDIO_LOCK;

	/* set page 0 */
	__int_mdio_write(tp, 0x1f, 0);
	/* address mode */
	__int_mdio_write(tp, 13, (0x1f & dev_addr));
	/* write the desired address */
	__int_mdio_write(tp, 14, reg_addr);
	/* data mode, no post increment */
	__int_mdio_write(tp, 13, ((0x1f & dev_addr) | (0x1 << 14)));
	/* read the content of the selected register */
	value = __int_mdio_read(tp, 14);

	MDIO_UNLOCK;
#endif /* RTL_OCP_MDIO */

	return value;
}

void ext_mmd_write(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr, u32 value)
{
#if defined(RTL_OCP_MDIO)
	/* address mode */
	ext_mdio_write(tp, 0, 13, (0x1f & dev_addr));
	/* write the desired address */
	ext_mdio_write(tp, 0, 14, reg_addr);
	/* data mode, no post increment */
	ext_mdio_write(tp, 0, 13, ((0x1f & dev_addr) | (0x1 << 14)));
	/* write the content of the selected register */
	ext_mdio_write(tp, 0, 14, value);
#else /* RTL_OCP_MDIO */
	MDIO_LOCK;

	/* set page 0 */
	__ext_mdio_write(tp, 0x1f, 0);
	/* address mode */
	__ext_mdio_write(tp, 13, (0x1f & dev_addr));
	/* write the desired address */
	__ext_mdio_write(tp, 14, reg_addr);
	/* data mode, no post increment */
	__ext_mdio_write(tp, 13, ((0x1f & dev_addr) | (0x1 << 14)));
	/* write the content of the selected register */
	__ext_mdio_write(tp, 14, value);

	MDIO_UNLOCK;
#endif /* RTL_OCP_MDIO */
}

u32 ext_mmd_read(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr)
{
	u32 value;

#if defined(RTL_OCP_MDIO)
	/* address mode */
	ext_mdio_write(tp, 0, 13, (0x1f & dev_addr));
	/* write the desired address */
	ext_mdio_write(tp, 0, 14, reg_addr);
	/* data mode, no post increment */
	ext_mdio_write(tp, 0, 13, ((0x1f & dev_addr) | (0x1 << 14)));
	/* read the content of the selected register */
	value = ext_mdio_read(tp, 0, 14);
#else /* RTL_OCP_MDIO */
	MDIO_LOCK;

	/* set page 0 */
	__ext_mdio_write(tp, 0x1f, 0);
	/* address mode */
	__ext_mdio_write(tp, 13, (0x1f & dev_addr));
	/* write the desired address */
	__ext_mdio_write(tp, 14, reg_addr);
	/* data mode, no post increment */
	__ext_mdio_write(tp, 13, ((0x1f & dev_addr) | (0x1 << 14)));
	/* read the content of the selected register */
	value = __ext_mdio_read(tp, 14);

	MDIO_UNLOCK;
#endif /* RTL_OCP_MDIO */
	return value;
}

static void rtl_init_mmd_ops(struct rtl8169_private *tp)
{
	struct mmd_ops *ops = &tp->mmd_ops;

	if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
		ops->write	= int_mmd_write;
		ops->read	= int_mmd_read;
	} else {
		ops->write	= ext_mmd_write;
		ops->read	= ext_mmd_read;
	}
}

void rtl_mmd_write(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr, u32 value)
{
	tp->mmd_ops.write(tp, dev_addr, reg_addr, value);
}

u32 rtl_mmd_read(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr)
{
	return tp->mmd_ops.read(tp, dev_addr, reg_addr);
}

static void rtl_eri_write(struct rtl8169_private *tp, int addr, u32 mask,
			  u32 val, int type)
{
	void __iomem *ioaddr = tp->mmio_addr;

	WARN_ON((addr & 3) || (mask == 0));
	RTL_W32(ERIDR, val);
	RTL_W32(ERIAR, ERIAR_WRITE_CMD | type | mask | addr);

	rtl_udelay_loop_wait_low(tp, &rtl_eriar_cond, 100, 100);
}

static u32 rtl_eri_read(struct rtl8169_private *tp, int addr, int type)
{
	void __iomem *ioaddr = tp->mmio_addr;

	RTL_W32(ERIAR, ERIAR_READ_CMD | type | ERIAR_MASK_1111 | addr);

	return rtl_udelay_loop_wait_high(tp, &rtl_eriar_cond, 100, 100) ?
		RTL_R32(ERIDR) : ~0;
}

static void rtl_w1w0_eri(struct rtl8169_private *tp, int addr, u32 mask, u32 p,
			 u32 m, int type)
{
	u32 val;

	val = rtl_eri_read(tp, addr, type);
	rtl_eri_write(tp, addr, mask, (val & ~m) | p, type);
}

struct exgmac_reg {
	u16 addr;
	u16 mask;
	u32 val;
};

static void rtl_write_exgmac_batch(struct rtl8169_private *tp,
				   const struct exgmac_reg *r, int len)
{
	while (len-- > 0) {
		rtl_eri_write(tp, r->addr, r->mask, r->val, ERIAR_EXGMAC);
		r++;
	}
}

static u16 rtl_get_events(struct rtl8169_private *tp)
{
	void __iomem *ioaddr = tp->mmio_addr;

	return RTL_R16(IntrStatus);
}

static void rtl_ack_events(struct rtl8169_private *tp, u16 bits)
{
	void __iomem *ioaddr = tp->mmio_addr;

	RTL_W16(IntrStatus, bits);
	mmiowb();
}

static void rtl_irq_disable(struct rtl8169_private *tp)
{
	void __iomem *ioaddr = tp->mmio_addr;

	RTL_W16(IntrMask, 0);
	mmiowb();
}

static void rtl_irq_enable(struct rtl8169_private *tp, u16 bits)
{
	void __iomem *ioaddr = tp->mmio_addr;

	RTL_W16(IntrMask, bits);
}

#if defined(RTL_HANDLE_RDU0)
#define RTL_EVENT_NAPI_RX	(RxOK | RxErr | RxOverflow)
#else
#define RTL_EVENT_NAPI_RX	(RxOK | RxErr)
#endif /* RTL_HANDLE_RDU0 */
#define RTL_EVENT_NAPI_TX	(TxOK | TxErr)
#define RTL_EVENT_NAPI		(RTL_EVENT_NAPI_RX | RTL_EVENT_NAPI_TX)

static void rtl_irq_enable_all(struct rtl8169_private *tp)
{
	rtl_irq_enable(tp, RTL_EVENT_NAPI | tp->event_slow);
}

static void rtl8169_irq_mask_and_ack(struct rtl8169_private *tp)
{
	void __iomem *ioaddr = tp->mmio_addr;

	rtl_irq_disable(tp);
	rtl_ack_events(tp, RTL_EVENT_NAPI | tp->event_slow);
	RTL_R8(ChipCmd);
}

static unsigned int rtl8169_xmii_reset_pending(struct rtl8169_private *tp)
{
	int ret;

	ret = rtl_phy_read(tp, 0, MII_BMCR);
	return ret & BMCR_RESET;
}

static unsigned int rtl8169_xmii_link_ok(void __iomem *ioaddr)
{
	return RTL_R8(PHYstatus) & LinkStatus;
}

#if defined(CONFIG_ARCH_RTD129x) || defined(CONFIG_ARCH_RTD139x)
static unsigned int rtl8169_xmii_always_link_ok(void __iomem *ioaddr)
{
	return true;
}
#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x */

static void rtl8169_xmii_reset_enable(struct rtl8169_private *tp)
{
	unsigned int val;

	val = rtl_phy_read(tp, 0, MII_BMCR) | BMCR_RESET;
	rtl_phy_write(tp, 0, MII_BMCR, val & 0xffff);
}

void r8169_display_eee_info(struct net_device *dev, struct seq_file *m,
			    struct rtl8169_private *tp)
{
	/* display DUT and link partner EEE capability */
	unsigned int temp, tmp1, tmp2;
	int speed = 0;
	bool eee1000, eee100, eee10;
	int duplex;

	temp = rtl_phy_read(tp, 0x0a43, 26);
	if (0 == ((0x1 << 2) & temp)) {
		seq_printf(m, "%s: link is down\n", dev->name);
		return;
	}

	if ((0x1 << 3) & temp)
		duplex = 1;
	else
		duplex = 0;

	if ((0x0 << 4) == ((0x3 << 4) & temp)) {
		speed = 10;

		tmp1 = rtl_phy_read(tp, 0x0bcd, 19);

		tmp2 = rtl_phy_read(tp, 0xa60, 16);

		if (0 == ((0x1 << 4) & tmp1))
			eee10 = false;
		else
			eee10 = true;

		seq_printf(m, "%s: link speed = %dM %s, EEE = %s, PCS_Status = 0x%02x\n",
			   dev->name, speed,
			   duplex ? "full" : "half",
			   eee10 ? "Y" : "N",
			   tmp2 & 0xff);
		return;
	}
	if ((0x1 << 4) == ((0x3 << 4) & temp))
		speed = 100;
	if ((0x2 << 4) == ((0x3 << 4) & temp))
		speed = 1000;
	if ((0x1 << 8) == ((0x1 << 8) & temp)) {
		seq_printf(m, "%s: link speed = %dM %s, EEE = Y\n", dev->name,
			   speed, duplex ? "full" : "half");

		tmp1 = rtl_phy_read(tp, 0xa60, 16);

		tmp2 = rtl_phy_read(tp, 0xa5c, 19);
		seq_printf(m, "PCS_Status = 0x%02x, EEE_wake_error = 0x%04x\n",
			   tmp1 & 0xff, tmp2);
	} else {
		seq_printf(m, "%s: link speed = %dM %s, EEE = N\n", dev->name,
			   speed, duplex ? "full" : "half");

		temp = rtl_mmd_read(tp, 0x7, 0x3d);
		if (0 == (temp & (0x1 << 2)))
			eee1000 = false;
		else
			eee1000 = true;

		if (0 == (temp & (0x1 << 1)))
			eee100 = false;
		else
			eee100 = true;

		seq_printf(m, "%s: Link Partner EEE1000=%s, EEE100=%s\n",
			   dev->name, eee1000 ? "Y" : "N", eee100 ? "Y" : "N");
	}
}

static void __rtl8169_check_link_status(struct net_device *dev,
					struct rtl8169_private *tp,
					void __iomem *ioaddr, bool pm)
{
	if (tp->link_ok(ioaddr)) {
		/* This is to cancel a scheduled suspend if there's one. */
		if (pm)
			pm_request_resume(&tp->pdev->dev);
		netif_carrier_on(dev);
		if (net_ratelimit())
			netif_info(tp, ifup, dev, "link up\n");
	} else {
		netif_carrier_off(dev);
		netif_info(tp, ifdown, dev, "link down\n");
		if (pm)
			pm_schedule_suspend(&tp->pdev->dev, 5000);
	}
}

static void rtl8169_check_link_status(struct net_device *dev,
				      struct rtl8169_private *tp,
				      void __iomem *ioaddr)
{
	__rtl8169_check_link_status(dev, tp, ioaddr, false);
}

#define WAKE_ANY (WAKE_PHY | WAKE_MAGIC | WAKE_UCAST | WAKE_BCAST | WAKE_MCAST)

static u32 __rtl8169_get_wol(struct rtl8169_private *tp)
{
	u32 wolopts = 0;

	if (tp->wol_enable & WOL_MAGIC)
		wolopts |= WAKE_MAGIC;

	return wolopts;
}

static void rtl8169_get_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	rtl_lock_work(tp);

	wol->supported = WAKE_ANY;
	wol->wolopts = __rtl8169_get_wol(tp);

	rtl_unlock_work(tp);
}

static void __rtl8169_set_wol(struct rtl8169_private *tp, u32 wolopts)
{
	if (wolopts & WAKE_MAGIC)
		tp->wol_enable |= WOL_MAGIC;
	else
		tp->wol_enable &= ~WOL_MAGIC;
}

static int rtl8169_set_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	rtl_lock_work(tp);

	if (wol->wolopts)
		tp->features |= RTL_FEATURE_WOL;
	else
		tp->features &= ~RTL_FEATURE_WOL;
	__rtl8169_set_wol(tp, wol->wolopts);

	rtl_unlock_work(tp);

	device_set_wakeup_enable(&tp->pdev->dev, wol->wolopts);

	return 0;
}

static void rtl8169_get_drvinfo(struct net_device *dev,
				struct ethtool_drvinfo *info)
{
	strscpy(info->driver, MODULENAME, sizeof(info->driver));
	strscpy(info->version, RTL8169_VERSION, sizeof(info->version));
	strscpy(info->bus_info, "RTK-ETN", sizeof(info->bus_info));
}

static int rtl8169_get_regs_len(struct net_device *dev)
{
	return R8169_REGS_SIZE;
}

static int rtl8169_set_speed_xmii(struct net_device *dev,
				  u8 autoneg, u16 speed, u8 duplex, u32 adv)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	int giga_ctrl, bmcr;
	int rc = -EINVAL;

	if (autoneg == AUTONEG_ENABLE) {
		int auto_nego;

		auto_nego = rtl_phy_read(tp, 0, MII_ADVERTISE);
		auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL |
			ADVERTISE_100HALF | ADVERTISE_100FULL);

		if (adv & ADVERTISED_10baseT_Half)
			auto_nego |= ADVERTISE_10HALF;
		if (adv & ADVERTISED_10baseT_Full)
			auto_nego |= ADVERTISE_10FULL;
		if (adv & ADVERTISED_100baseT_Half)
			auto_nego |= ADVERTISE_100HALF;
		if (adv & ADVERTISED_100baseT_Full)
			auto_nego |= ADVERTISE_100FULL;

		auto_nego |= ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM;

		giga_ctrl = rtl_phy_read(tp, 0, MII_CTRL1000);
		giga_ctrl &= ~(ADVERTISE_1000FULL | ADVERTISE_1000HALF);

		/* The 8100e/8101e/8102e do Fast Ethernet only. */
		if (tp->mii.supports_gmii) {
			if (adv & ADVERTISED_1000baseT_Half)
				giga_ctrl |= ADVERTISE_1000HALF;
			if (adv & ADVERTISED_1000baseT_Full)
				giga_ctrl |= ADVERTISE_1000FULL;
		} else if (adv & (ADVERTISED_1000baseT_Half |
				ADVERTISED_1000baseT_Full)) {
			netif_info(tp, link, dev,
				   "PHY does not support 1000Mbps\n");
			goto out;
		}

		bmcr = BMCR_ANENABLE | BMCR_ANRESTART;

		rtl_phy_write(tp, 0, MII_ADVERTISE, auto_nego);
		rtl_phy_write(tp, 0, MII_CTRL1000, giga_ctrl);
	} else {
		giga_ctrl = 0;

		if (speed == SPEED_10)
			bmcr = 0;
		else if (speed == SPEED_100)
			bmcr = BMCR_SPEED100;
		else
			goto out;

		if (duplex == DUPLEX_FULL)
			bmcr |= BMCR_FULLDPLX;
	}

	rtl_phy_write(tp, 0, MII_BMCR, bmcr);

	rc = 0;
out:
	return rc;
}

static int rtl8169_set_speed(struct net_device *dev,
			     u8 autoneg, u16 speed, u8 duplex, u32 advertising)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	int ret;

	ret = tp->set_speed(dev, autoneg, speed, duplex, advertising);
	return ret;
}

static int rtl8169_set_settings(struct net_device *dev,
				struct ethtool_cmd *cmd)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	int ret;

	rtl_lock_work(tp);
	ret = rtl8169_set_speed(dev, cmd->autoneg, ethtool_cmd_speed(cmd),
				cmd->duplex, cmd->advertising);
	rtl_unlock_work(tp);

	return ret;
}

static netdev_features_t rtl8169_fix_features(struct net_device *dev,
					      netdev_features_t features)
{
	if (dev->mtu > TD_MSS_MAX)
		features &= ~NETIF_F_ALL_TSO;

	if (dev->mtu > JUMBO_1K && !rtl_chip_infos.jumbo_tx_csum)
		features &= ~NETIF_F_IP_CSUM;

	return features;
}

static void __rtl8169_set_features(struct net_device *dev,
				   netdev_features_t features)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	netdev_features_t changed = features ^ dev->features;
	void __iomem *ioaddr = tp->mmio_addr;

	if (!(changed & (NETIF_F_RXALL | NETIF_F_RXCSUM |
			 NETIF_F_HW_VLAN_CTAG_RX)))
		return;

	if (changed & (NETIF_F_RXCSUM | NETIF_F_HW_VLAN_CTAG_RX)) {
		if (features & NETIF_F_RXCSUM)
			tp->cp_cmd |= RxChkSum;
		else
			tp->cp_cmd &= ~RxChkSum;

		if (dev->features & NETIF_F_HW_VLAN_CTAG_RX)
			tp->cp_cmd |= RxVlan;
		else
			tp->cp_cmd &= ~RxVlan;

		RTL_W16(CPlusCmd, tp->cp_cmd);
		RTL_R16(CPlusCmd);
	}
	if (changed & NETIF_F_RXALL) {
		int tmp = (RTL_R32(RxConfig) & ~(AcceptErr | AcceptRunt));

		if (features & NETIF_F_RXALL)
			tmp |= (AcceptErr | AcceptRunt);
		RTL_W32(RxConfig, tmp);
	}
}

static int rtl8169_set_features(struct net_device *dev,
				netdev_features_t features)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	rtl_lock_work(tp);
	__rtl8169_set_features(dev, features);
	rtl_unlock_work(tp);

	return 0;
}

static inline u32 rtl8169_tx_vlan_tag(struct sk_buff *skb)
{
	return (skb_vlan_tag_present(skb)) ?
		TxVlanTag | swab16(skb_vlan_tag_get(skb)) : 0x00;
}

static void rtl8169_rx_vlan_tag(struct RxDesc *desc, struct sk_buff *skb)
{
	u32 opts2 = le32_to_cpu(desc->opts2);

	if (opts2 & RxVlanTag)
		__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q),
				       swab16(opts2 & 0xffff));
}

static int rtl8169_gset_xmii(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	return mii_ethtool_gset(&tp->mii, cmd);
}

static int rtl8169_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	int rc;

	rtl_lock_work(tp);
	rc = tp->get_settings(dev, cmd);
	rtl_unlock_work(tp);

	return rc;
}

static void rtl8169_get_regs(struct net_device *dev, struct ethtool_regs *regs,
			     void *p)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	u32 __iomem *data = tp->mmio_addr;
	u32 *dw = p;
	int i;

	rtl_lock_work(tp);
	for (i = 0; i < R8169_REGS_SIZE; i += 4)
		memcpy_fromio(dw++, data++, 4);
	rtl_unlock_work(tp);
}

static u32 rtl8169_get_msglevel(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	return tp->msg_enable;
}

static void rtl8169_set_msglevel(struct net_device *dev, u32 value)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	tp->msg_enable = value;
}

static const char rtl8169_gstrings[][ETH_GSTRING_LEN] = {
	"tx_packets",
	"rx_packets",
	"tx_errors",
	"rx_errors",
	"rx_missed",
	"align_errors",
	"tx_single_collisions",
	"tx_multi_collisions",
	"unicast",
	"broadcast",
	"multicast",
	"tx_aborted",
	"tx_underrun",
};

static int rtl8169_get_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(rtl8169_gstrings);
	default:
		return -EOPNOTSUPP;
	}
}

DECLARE_RTL_COND(rtl_counters_cond)
{
	void __iomem *ioaddr = tp->mmio_addr;

	return RTL_R32(CounterAddrLow) & CounterDump;
}

static void rtl8169_update_counters(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	void __iomem *ioaddr = tp->mmio_addr;
	struct device *d = &tp->pdev->dev;
	struct rtl8169_counters *counters;
	dma_addr_t paddr;
	u32 cmd;
	int node = dev->dev.parent ? dev_to_node(dev->dev.parent) : -1;

	/* Some chips are unable to dump tally counters when the receiver
	 * is disabled.
	 */
	if ((RTL_R8(ChipCmd) & CmdRxEnb) == 0)
		return;

	if (tp->acp_enable) {
		counters = kzalloc_node(sizeof(*counters), GFP_KERNEL, node);
		paddr = virt_to_phys(counters);
	} else {
		counters = dma_alloc_coherent(d, sizeof(*counters), &paddr,
					      GFP_KERNEL);
	}
	if (!counters)
		return;

	RTL_W32(CounterAddrHigh, (u64)paddr >> 32);
	cmd = (u64)paddr & DMA_BIT_MASK(32);
	RTL_W32(CounterAddrLow, cmd);
	RTL_W32(CounterAddrLow, cmd | CounterDump);

	if (rtl_udelay_loop_wait_low(tp, &rtl_counters_cond, 10, 1000))
		memcpy(&tp->counters, counters, sizeof(*counters));

	RTL_W32(CounterAddrLow, 0);
	RTL_W32(CounterAddrHigh, 0);

	if (tp->acp_enable)
		kfree(counters);
	else
		dma_free_coherent(d, sizeof(*counters), counters, paddr);
}

static void rtl8169_get_ethtool_stats(struct net_device *dev,
				      struct ethtool_stats *stats, u64 *data)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	ASSERT_RTNL();

	rtl8169_update_counters(dev);

	data[0] = le64_to_cpu(tp->counters.tx_packets);
	data[1] = le64_to_cpu(tp->counters.rx_packets);
	data[2] = le64_to_cpu(tp->counters.tx_errors);
	data[3] = le32_to_cpu(tp->counters.rx_errors);
	data[4] = le16_to_cpu(tp->counters.rx_missed);
	data[5] = le16_to_cpu(tp->counters.align_errors);
	data[6] = le32_to_cpu(tp->counters.tx_one_collision);
	data[7] = le32_to_cpu(tp->counters.tx_multi_collision);
	data[8] = le64_to_cpu(tp->counters.rx_unicast);
	data[9] = le64_to_cpu(tp->counters.rx_broadcast);
	data[10] = le32_to_cpu(tp->counters.rx_multicast);
	data[11] = le16_to_cpu(tp->counters.tx_aborted);
	data[12] = le16_to_cpu(tp->counters.tx_underrun);
}

static void rtl8169_get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
	switch (stringset) {
	case ETH_SS_STATS:
		memcpy(data, rtl8169_gstrings, sizeof(rtl8169_gstrings));
		break;
	}
}

static const struct ethtool_ops rtl8169_ethtool_ops = {
	.get_drvinfo		= rtl8169_get_drvinfo,
	.get_regs_len		= rtl8169_get_regs_len,
	.get_link		= ethtool_op_get_link,
	.get_settings		= rtl8169_get_settings,
	.set_settings		= rtl8169_set_settings,
	.get_msglevel		= rtl8169_get_msglevel,
	.set_msglevel		= rtl8169_set_msglevel,
	.get_regs		= rtl8169_get_regs,
	.get_wol		= rtl8169_get_wol,
	.set_wol		= rtl8169_set_wol,
	.get_strings		= rtl8169_get_strings,
	.get_sset_count		= rtl8169_get_sset_count,
	.get_ethtool_stats	= rtl8169_get_ethtool_stats,
	.get_ts_info		= ethtool_op_get_ts_info,
};

static void rtl8169_print_mac_version(struct rtl8169_private *tp)
{
	dprintk("mac_version = 0x%02x\n", tp->mac_version);
}

struct phy_reg {
	u16 reg;
	u16 val;
};

static __maybe_unused void rtl_rar_exgmac_set(struct rtl8169_private *tp,
					      u8 *addr)
{
	const u16 w[] = {
		addr[0] | (addr[1] << 8),
		addr[2] | (addr[3] << 8),
		addr[4] | (addr[5] << 8)
	};
	const struct exgmac_reg e[] = {
		{.addr = 0xe0, ERIAR_MASK_1111, .val = w[0] | (w[1] << 16)},
		{.addr = 0xe4, ERIAR_MASK_1111, .val = w[2]},
		{.addr = 0xf0, ERIAR_MASK_1111, .val = w[0] << 16},
		{.addr = 0xf4, ERIAR_MASK_1111, .val = w[1] | (w[2] << 16)}
	};

	rtl_write_exgmac_batch(tp, e, ARRAY_SIZE(e));
}

static void rtl8168g_2_hw_mac_mcu_patch(struct rtl8169_private *tp)
{
	#if defined(CONFIG_ARCH_RTD129x)
	unsigned int tmp;
	unsigned int cpu_revision;
	void __iomem *rgmii0_addr, *tmp_addr;
	#endif /* CONFIG_ARCH_RTD129x */

	/* power down PHY */
	rtl_phy_write(tp, 0, MII_BMCR,
		      rtl_phy_read(tp, 0, MII_BMCR) | BMCR_PDOWN);

	#if defined(CONFIG_ARCH_RTD129x)
	/* mac mcu patch from Tenno to support EEE/EEE+ */
	cpu_revision = get_rtd_chip_revision();
	if (cpu_revision == RTD_CHIP_A00 ||
	    cpu_revision == RTD_CHIP_A01 ||
	    cpu_revision >= RTD_CHIP_B00) {
		/* disable break point */
		rtl_ocp_write(tp, 0xfc28, 0);
		rtl_ocp_write(tp, 0xfc2a, 0);
		rtl_ocp_write(tp, 0xfc2c, 0);
		rtl_ocp_write(tp, 0xfc2e, 0);
		rtl_ocp_write(tp, 0xfc30, 0);
		rtl_ocp_write(tp, 0xfc32, 0);
		rtl_ocp_write(tp, 0xfc34, 0);
		rtl_ocp_write(tp, 0xfc36, 0);
		mdelay(3);

		/* disable base address */
		rtl_ocp_write(tp, 0xfc26, 0);

		/* patch code */
		rtl_ocp_write(tp, 0xf800, 0xE008);
		rtl_ocp_write(tp, 0xf802, 0xE012);
		rtl_ocp_write(tp, 0xf804, 0xE044);
		rtl_ocp_write(tp, 0xf806, 0xE046);
		rtl_ocp_write(tp, 0xf808, 0xE048);
		rtl_ocp_write(tp, 0xf80a, 0xE04A);
		rtl_ocp_write(tp, 0xf80c, 0xE04C);
		rtl_ocp_write(tp, 0xf80e, 0xE04E);
		rtl_ocp_write(tp, 0xf810, 0x44E3);
		rtl_ocp_write(tp, 0xf812, 0xC708);
		rtl_ocp_write(tp, 0xf814, 0x75E0);
		rtl_ocp_write(tp, 0xf816, 0x485D);
		rtl_ocp_write(tp, 0xf818, 0x9DE0);
		rtl_ocp_write(tp, 0xf81a, 0xC705);
		rtl_ocp_write(tp, 0xf81c, 0xC502);
		rtl_ocp_write(tp, 0xf81e, 0xBD00);
		rtl_ocp_write(tp, 0xf820, 0x01EE);
		rtl_ocp_write(tp, 0xf822, 0xE85A);
		rtl_ocp_write(tp, 0xf824, 0xE000);
		rtl_ocp_write(tp, 0xf826, 0xC72D);
		rtl_ocp_write(tp, 0xf828, 0x76E0);
		rtl_ocp_write(tp, 0xf82a, 0x49ED);
		rtl_ocp_write(tp, 0xf82c, 0xF026);
		rtl_ocp_write(tp, 0xf82e, 0xC02A);
		rtl_ocp_write(tp, 0xf830, 0x7400);
		rtl_ocp_write(tp, 0xf832, 0xC526);
		rtl_ocp_write(tp, 0xf834, 0xC228);
		rtl_ocp_write(tp, 0xf836, 0x9AA0);
		rtl_ocp_write(tp, 0xf838, 0x73A2);
		rtl_ocp_write(tp, 0xf83a, 0x49BE);
		rtl_ocp_write(tp, 0xf83c, 0xF11E);
		rtl_ocp_write(tp, 0xf83e, 0xC324);
		rtl_ocp_write(tp, 0xf840, 0x9BA2);
		rtl_ocp_write(tp, 0xf842, 0x73A2);
		rtl_ocp_write(tp, 0xf844, 0x49BE);
		rtl_ocp_write(tp, 0xf846, 0xF0FE);
		rtl_ocp_write(tp, 0xf848, 0x73A2);
		rtl_ocp_write(tp, 0xf84a, 0x49BE);
		rtl_ocp_write(tp, 0xf84c, 0xF1FE);
		rtl_ocp_write(tp, 0xf84e, 0x1A02);
		rtl_ocp_write(tp, 0xf850, 0x49C9);
		rtl_ocp_write(tp, 0xf852, 0xF003);
		rtl_ocp_write(tp, 0xf854, 0x4821);
		rtl_ocp_write(tp, 0xf856, 0xE002);
		rtl_ocp_write(tp, 0xf858, 0x48A1);
		rtl_ocp_write(tp, 0xf85a, 0x73A2);
		rtl_ocp_write(tp, 0xf85c, 0x49BE);
		rtl_ocp_write(tp, 0xf85e, 0xF10D);
		rtl_ocp_write(tp, 0xf860, 0xC313);
		rtl_ocp_write(tp, 0xf862, 0x9AA0);
		rtl_ocp_write(tp, 0xf864, 0xC312);
		rtl_ocp_write(tp, 0xf866, 0x9BA2);
		rtl_ocp_write(tp, 0xf868, 0x73A2);
		rtl_ocp_write(tp, 0xf86a, 0x49BE);
		rtl_ocp_write(tp, 0xf86c, 0xF0FE);
		rtl_ocp_write(tp, 0xf86e, 0x73A2);
		rtl_ocp_write(tp, 0xf870, 0x49BE);
		rtl_ocp_write(tp, 0xf872, 0xF1FE);
		rtl_ocp_write(tp, 0xf874, 0x48ED);
		rtl_ocp_write(tp, 0xf876, 0x9EE0);
		rtl_ocp_write(tp, 0xf878, 0xC602);
		rtl_ocp_write(tp, 0xf87a, 0xBE00);
		rtl_ocp_write(tp, 0xf87c, 0x0532);
		rtl_ocp_write(tp, 0xf87e, 0xDE00);
		rtl_ocp_write(tp, 0xf880, 0xE85A);
		rtl_ocp_write(tp, 0xf882, 0xE086);
		rtl_ocp_write(tp, 0xf884, 0x0A44);
		rtl_ocp_write(tp, 0xf886, 0x801F);
		rtl_ocp_write(tp, 0xf888, 0x8015);
		rtl_ocp_write(tp, 0xf88a, 0x0015);
		rtl_ocp_write(tp, 0xf88c, 0xC602);
		rtl_ocp_write(tp, 0xf88e, 0xBE00);
		rtl_ocp_write(tp, 0xf890, 0x0000);
		rtl_ocp_write(tp, 0xf892, 0xC602);
		rtl_ocp_write(tp, 0xf894, 0xBE00);
		rtl_ocp_write(tp, 0xf896, 0x0000);
		rtl_ocp_write(tp, 0xf898, 0xC602);
		rtl_ocp_write(tp, 0xf89a, 0xBE00);
		rtl_ocp_write(tp, 0xf89c, 0x0000);
		rtl_ocp_write(tp, 0xf89e, 0xC602);
		rtl_ocp_write(tp, 0xf8a0, 0xBE00);
		rtl_ocp_write(tp, 0xf8a2, 0x0000);
		rtl_ocp_write(tp, 0xf8a4, 0xC602);
		rtl_ocp_write(tp, 0xf8a6, 0xBE00);
		rtl_ocp_write(tp, 0xf8a8, 0x0000);
		rtl_ocp_write(tp, 0xf8aa, 0xC602);
		rtl_ocp_write(tp, 0xf8ac, 0xBE00);
		rtl_ocp_write(tp, 0xf8ae, 0x0000);

		/* enable base address */
		rtl_ocp_write(tp, 0xfc26, 0x8000);

		/* enable breakpoint */
		rtl_ocp_write(tp, 0xfc28, 0x01ED);
		rtl_ocp_write(tp, 0xfc2a, 0x0531);

		if (tp->eee_enable) {
			/* EEE MAC mode */
			tmp = rtl_ocp_read(tp, 0xe040) | (BIT(1) | BIT(0));
			rtl_ocp_write(tp, 0xe040, tmp);
			/* EEE+ MAC mode */
			tmp = rtl_ocp_read(tp, 0xe080) | BIT(1);
			rtl_ocp_write(tp, 0xe080, tmp);
		}
	}

	if (cpu_revision >= RTD_CHIP_B00) {
		if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
			tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(0) | BIT(1));
			tmp |= BIT(1); /* MII */
			rtl_ocp_write(tp, 0xea34, tmp);
		} else { /* RGMII */
			if (tp->output_mode == OUTPUT_RGMII_TO_PHY) {
				/* # ETN spec, MDC freq=2.5MHz */
				tmp = rtl_ocp_read(tp, 0xde30) &
					~(BIT(6) | BIT(7));
				rtl_ocp_write(tp, 0xde30, tmp);
				/* # ETN spec, set external PHY addr */
				tmp = rtl_ocp_read(tp, 0xde24) & ~(0x1F);
				tmp |= tp->ext_phy_id & 0x1F;
				rtl_ocp_write(tp, 0xde24, tmp);
			}

			tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(0) | BIT(1));
			tmp |= BIT(0); /* RGMII */

			if (tp->output_mode == OUTPUT_RGMII_TO_MAC) {
				tmp &= ~(BIT(3) | BIT(4));
				tmp |= BIT(4); /* speed: 1G */
				tmp |= BIT(2); /* full duplex */
			}
			if (tp->rgmii_rx_delay == RGMII_DELAY_0NS)
				tmp &= ~BIT(6);
			else
				tmp |= BIT(6);

			if (tp->rgmii_tx_delay == RGMII_DELAY_0NS)
				tmp &= ~BIT(7);
			else
				tmp |= BIT(7);

			rtl_ocp_write(tp, 0xea34, tmp);

			/* adjust RGMII voltage */
			rgmii0_addr = ioremap(RGMII0_PAD_CTRL_ADDR, 12);
			switch (tp->rgmii_voltage) {
			case VOLTAGE_1_DOT_8V:
				writel(0, rgmii0_addr);
				writel(0x44444444, rgmii0_addr + 4);
				writel(0x24444444, rgmii0_addr + 8);
				break;
			case VOLTAGE_2_DOT_5V:
				writel(0, rgmii0_addr);
				writel(0x44444444, rgmii0_addr + 4);
				writel(0x64444444, rgmii0_addr + 8);
				break;
			case VOLTAGE_3_DOT_3V:
				writel(0x3F, rgmii0_addr);
				writel(0, rgmii0_addr + 4);
				writel(0xA4000000, rgmii0_addr + 8);
			}
			__iounmap(rgmii0_addr);

			/* switch RGMII/MDIO to GMAC */
			tmp_addr = ioremap(ISO_RGMII_MDIO_TO_GMAC, 4);
			writel((readl(tmp_addr) | BIT(1)), tmp_addr);
			__iounmap(tmp_addr);

			if (tp->output_mode == OUTPUT_RGMII_TO_MAC) {
				/* force GMAC link up */
				rtl_ocp_write(tp, 0xde40, 0x30EC);
				/* ignore mac_intr from PHY */
				tmp = rtl_ocp_read(tp, 0xfc1e) & ~BIT(1);
				rtl_ocp_write(tp, 0xfc1e, tmp);
			}
		}
	}
	#endif /* CONFIG_ARCH_RTD129x */
}

static void rtl8168g_2_hw_phy_config(struct rtl8169_private *tp)
{
	void __iomem *ioaddr = tp->mmio_addr;

	/* set dis_mcu_clroob to avoid WOL fail when ALDPS mode is enabled */
	RTL_W8(MCU, RTL_R8(MCU) | DIS_MCU_CLROOB);
	#if defined(CONFIG_ARCH_RTD129x)
	/* enable ALDPS mode */
	rtl_w1w0_phy(tp, 0x0a43, 0x18, BIT(2), BIT(12) | BIT(1) | BIT(0));

	/* enable EEE of 10Mbps */
	rtl_patchphy(tp, 0x0a43, 0x19, BIT(4));
	#endif /* CONFIG_ARCH_RTD129x */
}

static void rtl_hw_phy_config(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	rtl8169_print_mac_version(tp);

	rtl8168g_2_hw_mac_mcu_patch(tp);
	rtl8168g_2_hw_phy_config(tp);
}

static void rtl_schedule_task(struct rtl8169_private *tp, enum rtl_flag flag)
{
	if (!test_and_set_bit(flag, tp->wk.flags))
		schedule_work(&tp->wk.work);
}

static void rtl_rar_set(struct rtl8169_private *tp, u8 *addr)
{
	void __iomem *ioaddr = tp->mmio_addr;

	rtl_lock_work(tp);

	RTL_W8(Cfg9346, Cfg9346_Unlock);

	RTL_W32(MAC4, addr[4] | addr[5] << 8);
	RTL_R32(MAC4);

	RTL_W32(MAC0, addr[0] | addr[1] << 8 | addr[2] << 16 | addr[3] << 24);
	RTL_R32(MAC0);

	RTL_W8(Cfg9346, Cfg9346_Lock);

	rtl_unlock_work(tp);
}

static void rtl_phy_reinit(struct rtl8169_private *tp)
{
	u32 tmp;

	/* fill fuse_rdy & rg_ext_ini_done */
	rtl_phy_write(tp, 0x0a46, 20,
		      rtl_phy_read(tp, 0x0a46, 20) | (BIT(1) | BIT(0)));

	/* init_autoload_done = 1 */
	tmp = rtl_ocp_read(tp, 0xe004);
	tmp |= BIT(7);
	rtl_ocp_write(tp, 0xe004, tmp);

	/* wait LAN-ON */
	tmp = 0;
	do {
		tmp += 1;
		mdelay(1);
		if (tmp >= 2000) {
			pr_err("PHY status is not 0x3, current = 0x%02x\n",
			       (rtl_phy_read(tp, 0x0a42, 16) & 0x07));
			break;
		}
	} while (0x3 != (rtl_phy_read(tp, 0x0a42, 16) & 0x07));
	pr_info("wait %d ms for PHY ready, current = 0x%x\n",
		tmp, rtl_phy_read(tp, 0x0a42, 16));

	tp->phy_reset_enable(tp);
}

static void rtl_phy_work(struct rtl8169_private *tp)
{
	rtl_phy_reinit(tp);
	atomic_dec(&tp->phy_reinit_flag);
}

static void rtl8169_release_board(struct platform_device *pdev,
				  struct net_device *dev, void __iomem *ioaddr)
{
	free_netdev(dev);
}

DECLARE_RTL_COND(rtl_phy_reset_cond)
{
	return tp->phy_reset_pending(tp);
}

static void rtl8169_phy_reset(struct net_device *dev,
			      struct rtl8169_private *tp)
{
	tp->phy_reset_enable(tp);
	rtl_msleep_loop_wait_low(tp, &rtl_phy_reset_cond, 1, 100);
}

static void rtl8169_init_phy(struct net_device *dev, struct rtl8169_private *tp)
{
	void __iomem *ioaddr = tp->mmio_addr;

	rtl_hw_phy_config(dev);

	/* disable now_is_oob */
	RTL_W8(MCU, RTL_R8(MCU) & ~NOW_IS_OOB);

	rtl8169_phy_reset(dev, tp);

	rtl8169_set_speed(dev, AUTONEG_ENABLE, SPEED_1000, DUPLEX_FULL,
			  ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full |
			  ADVERTISED_100baseT_Half | ADVERTISED_100baseT_Full |
			  (tp->mii.supports_gmii ?
			   ADVERTISED_1000baseT_Half |
			   ADVERTISED_1000baseT_Full : 0));
}

static int rtl_set_mac_address(struct net_device *dev, void *p)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

	rtl_rar_set(tp, dev->dev_addr);

	return 0;
}

static int rtl8169_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	struct mii_ioctl_data *data = if_mii(ifr);

	return netif_running(dev) ? tp->do_ioctl(tp, data, cmd) : -ENODEV;
}

static int rtl_xmii_ioctl(struct rtl8169_private *tp,
			  struct mii_ioctl_data *data, int cmd)
{
	switch (cmd) {
	case SIOCGMIIPHY:
		data->phy_id = 32; /* Internal PHY */
		return 0;

	case SIOCGMIIREG:
		data->val_out = rtl_phy_read(tp, 0, data->reg_num & 0x1f);
		return 0;

	case SIOCSMIIREG:
		rtl_phy_write(tp, 0, data->reg_num & 0x1f, data->val_in);
		return 0;
	}
	return -EOPNOTSUPP;
}

static void rtl_write_wakeup_pattern(struct rtl8169_private *tp, u32 idx)
{
	u8 i;
	u8 j;
	u32 reg_mask;
	u32 reg_shift;
	u32 reg_offset;

	reg_offset = idx & ~(BIT(0) | BIT(1));
	reg_shift = (idx % 4) * 8;
	switch (reg_shift) {
	case 0:
		reg_mask = ERIAR_MASK_0001;
		break;
	case 8:
		reg_mask = ERIAR_MASK_0010;
		break;
	case 16:
		reg_mask = ERIAR_MASK_0100;
		break;
	case 24:
		reg_mask = ERIAR_MASK_1000;
		break;
	default:
		pr_err("Invalid shift bit 0x%x, idx = %d\n", reg_shift, idx);
		return;
	}

	for (i = 0, j = 0; i < 0x80; i += 8, j++) {
		rtl_eri_write(tp, i + reg_offset, reg_mask,
			      tp->wol_mask[idx][j] << reg_shift, ERIAR_EXGMAC);
	}

	reg_offset = idx * 2;
	if (idx % 2) {
		reg_mask = ERIAR_MASK_1100;
		reg_offset -= 2;
		reg_shift = 16;
	} else {
		reg_mask = ERIAR_MASK_0011;
		reg_shift = 0;
	}
	rtl_eri_write(tp, (int)(0x80 + reg_offset), reg_mask,
		      tp->wol_crc[idx] << reg_shift, ERIAR_EXGMAC);
}

static void rtl_mdns_crc_wakeup(struct rtl8169_private *tp)
{
	int i;

	for (i = 0; i < tp->wol_crc_cnt; i++)
		rtl_write_wakeup_pattern(tp, i);
}

static void rtl_clear_wakeup_pattern(struct rtl8169_private *tp)
{
	u8 i;

	for (i = 0; i < 0x80; i += 4)
		rtl_eri_write(tp, i, ERIAR_MASK_1111, 0x0, ERIAR_EXGMAC);

	for (i = 0x80; i < 0x90; i += 4)
		rtl_eri_write(tp, i, ERIAR_MASK_1111, 0x0, ERIAR_EXGMAC);
}

static void rtl_wpd_set(struct rtl8169_private *tp)
{
	u32 tmp;
	u32 i;

	/* clear WPD registers */
	rtl_ocp_write(tp, 0xD23A, 0);
	rtl_ocp_write(tp, 0xD23C, 0);
	rtl_ocp_write(tp, 0xD23E, 0);
	for (i = 0; i < 128; i += 2)
		rtl_ocp_write(tp, 0xD240 + i, 0);

	tmp = rtl_ocp_read(tp, 0xC0C2);
	tmp &= ~BIT(4);
	rtl_ocp_write(tp, 0xC0C2, tmp);

	/* enable WPD */
	tmp = rtl_ocp_read(tp, 0xC0C2);
	tmp |= BIT(0);
	rtl_ocp_write(tp, 0xC0C2, tmp);
}

static void rtl_speed_down(struct rtl8169_private *tp)
{
	u32 adv;
	int lpa;

	lpa = rtl_phy_read(tp, 0, MII_LPA);

	if (lpa & (LPA_10HALF | LPA_10FULL))
		adv = ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full;
	else if (lpa & (LPA_100HALF | LPA_100FULL))
		adv = ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full |
			  ADVERTISED_100baseT_Half | ADVERTISED_100baseT_Full;
	else
		adv = ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full |
			  ADVERTISED_100baseT_Half | ADVERTISED_100baseT_Full |
			  (tp->mii.supports_gmii ?
			   ADVERTISED_1000baseT_Half |
			   ADVERTISED_1000baseT_Full : 0);

	rtl8169_set_speed(tp->dev, AUTONEG_ENABLE, SPEED_1000, DUPLEX_FULL,
			  adv);
}

static void rtl_wol_suspend_quirk(struct rtl8169_private *tp)
{
	void __iomem *ioaddr = tp->mmio_addr;

	RTL_W32(RxConfig, RTL_R32(RxConfig) |
		AcceptBroadcast | AcceptMulticast | AcceptMyPhys);
}

static bool rtl_wol_pll_power_down(struct rtl8169_private *tp)
{
	if (!(__rtl8169_get_wol(tp) & WAKE_ANY))
		return false;

	rtl_speed_down(tp);
	rtl_wol_suspend_quirk(tp);

	return true;
}

static void r8168_phy_power_up(struct rtl8169_private *tp)
{
	rtl_phy_write(tp, 0, MII_BMCR, BMCR_ANENABLE);
}

static void r8168_phy_power_down(struct rtl8169_private *tp)
{
	rtl_phy_write(tp, 0, MII_BMCR, BMCR_PDOWN);
}

static void r8168_pll_power_down(struct rtl8169_private *tp)
{
	void __iomem *ioaddr = tp->mmio_addr;

	RTL_W8(Cfg9346, Cfg9346_Unlock);
	if (tp->wol_enable & WOL_MAGIC) {
		/* enable now_is_oob */
		RTL_W8(MCU, RTL_R8(MCU) | NOW_IS_OOB);
		RTL_W8(Config5, RTL_R8(Config5) | LanWake);
		RTL_W8(Config3, RTL_R8(Config3) | MagicPacket);

		rtl_clear_wakeup_pattern(tp);
		if (tp->wol_crc_cnt > 0 && (tp->wol_enable & WOL_CRC_MATCH))
			rtl_mdns_crc_wakeup(tp);
		if (tp->wol_enable & WOL_WPD)
			rtl_wpd_set(tp);
	} else {
		RTL_W8(Config5, RTL_R8(Config5) & ~LanWake);
		RTL_W8(Config3, RTL_R8(Config3) & ~MagicPacket);
	}
	RTL_W8(Cfg9346, Cfg9346_Lock);

	if (rtl_wol_pll_power_down(tp))
		return;

	r8168_phy_power_down(tp);
}

static void r8168_pll_power_up(struct rtl8169_private *tp)
{
	r8168_phy_power_up(tp);
}

static inline void rtl_generic_op(struct rtl8169_private *tp,
				  void (*op)(struct rtl8169_private *))
{
	if (op)
		op(tp);
}

static inline void rtl_pll_power_down(struct rtl8169_private *tp)
{
	rtl_generic_op(tp, tp->pll_power_ops.down);
}

static inline void rtl_pll_power_up(struct rtl8169_private *tp)
{
	rtl_generic_op(tp, tp->pll_power_ops.up);
}

static void rtl_init_pll_power_ops(struct rtl8169_private *tp)
{
	struct pll_power_ops *ops = &tp->pll_power_ops;

	ops->down	= r8168_pll_power_down;
	ops->up		= r8168_pll_power_up;
}

static void rtl_init_rxcfg(struct rtl8169_private *tp)
{
	void __iomem *ioaddr = tp->mmio_addr;

	/* disable RX128_INT_EN to reduce CPU loading */
	RTL_W32(RxConfig, RX_DMA_BURST | RX_EARLY_OFF);
}

static void rtl8169_init_ring_indexes(struct rtl8169_private *tp)
{
	tp->dirty_tx = 0;
	tp->cur_tx = 0;
	tp->cur_rx = 0;

	#if defined(CONFIG_RTL_RX_NO_COPY)
	tp->dirty_rx = 0;
	#endif /* CONFIG_RTL_RX_NO_COPY */
}

static void rtl_init_jumbo_ops(struct rtl8169_private *tp)
{
	struct jumbo_ops *ops = &tp->jumbo_ops;

	/* No action needed for jumbo frames with r8169soc. */
	ops->disable	= NULL;
	ops->enable	= NULL;
}

DECLARE_RTL_COND(rtl_chipcmd_cond)
{
	void __iomem *ioaddr = tp->mmio_addr;

	return RTL_R8(ChipCmd) & CmdReset;
}

static void rtl_hw_reset(struct rtl8169_private *tp)
{
	void __iomem *ioaddr = tp->mmio_addr;

	RTL_W8(ChipCmd, CmdReset);

	rtl_udelay_loop_wait_low(tp, &rtl_chipcmd_cond, 100, 100);
}

static void rtl_rx_close(struct rtl8169_private *tp)
{
	void __iomem *ioaddr = tp->mmio_addr;

	RTL_W32(RxConfig, RTL_R32(RxConfig) & ~RX_CONFIG_ACCEPT_MASK);
}

DECLARE_RTL_COND(rtl_txcfg_empty_cond)
{
	void __iomem *ioaddr = tp->mmio_addr;

	return RTL_R32(TxConfig) & TXCFG_EMPTY;
}

static void rtl8169_hw_reset(struct rtl8169_private *tp)
{
	void __iomem *ioaddr = tp->mmio_addr;

	/* Disable interrupts */
	rtl8169_irq_mask_and_ack(tp);

	rtl_rx_close(tp);

	RTL_W8(ChipCmd, RTL_R8(ChipCmd) | StopReq);
	rtl_udelay_loop_wait_high(tp, &rtl_txcfg_empty_cond, 100, 666);

	rtl_hw_reset(tp);

	#if defined(RTL_TX_NO_CLOSE)
	/* disable tx no close mode */
	rtl_ocp_write(tp, 0xE610,
		      rtl_ocp_read(tp, 0xE610) & ~(BIT(4) | BIT(6)));
	#endif /* RTL_TX_NO_CLOSE */
}

static void rtl_set_rx_tx_config_registers(struct rtl8169_private *tp)
{
	void __iomem *ioaddr = tp->mmio_addr;

	/* Set DMA burst size and Interframe Gap Time */
	RTL_W32(TxConfig, (TX_DMA_BURST << TxDMAShift) |
		(InterFrameGap << TxInterFrameGapShift));
}

static void rtl_hw_start(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	tp->hw_start(dev);

	rtl_irq_enable_all(tp);
}

static void rtl_set_rx_tx_desc_registers(struct rtl8169_private *tp,
					 void __iomem *ioaddr)
{
	/* Magic spell: some iop3xx ARM board needs the TxDescAddrHigh
	 * register to be written before TxDescAddrLow to work.
	 * Switching from MMIO to I/O access fixes the issue as well.
	 */
	RTL_W32(TxDescStartAddrHigh, ((u64)tp->TxPhyAddr) >> 32);
	RTL_W32(TxDescStartAddrLow, ((u64)tp->TxPhyAddr) & DMA_BIT_MASK(32));
	RTL_W32(RxDescAddrHigh, ((u64)tp->RxPhyAddr) >> 32);
	RTL_W32(RxDescAddrLow, ((u64)tp->RxPhyAddr) & DMA_BIT_MASK(32));
}

static void rtl_set_rx_max_size(void __iomem *ioaddr, unsigned int rx_buf_sz)
{
	/* Low hurts. Let's disable the filtering. */
	RTL_W16(RxMaxSize, rx_buf_sz);
}

static void rtl_set_rx_mode(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	void __iomem *ioaddr = tp->mmio_addr;
	u32 mc_filter[2];	/* Multicast hash filter */
	int rx_mode;
	u32 tmp = 0;
	u32 data;

	if (dev->flags & IFF_PROMISC) {
		/* Unconditionally log net taps. */
		netif_notice(tp, link, dev, "Promiscuous mode enabled\n");
		rx_mode =
			AcceptBroadcast | AcceptMulticast | AcceptMyPhys |
			AcceptAllPhys;
		mc_filter[1] = 0xffffffff;
		mc_filter[0] = 0xffffffff;
	} else if ((netdev_mc_count(dev) > multicast_filter_limit) ||
		   (dev->flags & IFF_ALLMULTI)) {
		/* Too many to filter perfectly -- accept all multicasts. */
		rx_mode = AcceptBroadcast | AcceptMulticast | AcceptMyPhys;
		mc_filter[1] = 0xffffffff;
		mc_filter[0] = 0xffffffff;
	} else {
		struct netdev_hw_addr *ha;

		rx_mode = AcceptBroadcast | AcceptMyPhys;
		mc_filter[1] = 0;
		mc_filter[0] = 0;
		netdev_for_each_mc_addr(ha, dev) {
			int bit_nr = ether_crc(ETH_ALEN, ha->addr) >> 26;

			mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
			rx_mode |= AcceptMulticast;
		}
	}

	if (dev->features & NETIF_F_RXALL)
		rx_mode |= (AcceptErr | AcceptRunt);

	tmp = (RTL_R32(RxConfig) & ~RX_CONFIG_ACCEPT_MASK) | rx_mode;

	data = mc_filter[0];

	mc_filter[0] = swab32(mc_filter[1]);
	mc_filter[1] = swab32(data);

	RTL_W32(MAR0 + 4, mc_filter[1]);
	RTL_W32(MAR0 + 0, mc_filter[0]);

	RTL_W32(RxConfig, tmp);
}

static inline void rtl_csi_write(struct rtl8169_private *tp, int addr,
				 int value)
{
	if (tp->csi_ops.write)
		tp->csi_ops.write(tp, addr, value);
}

static u32 rtl_csi_read(struct rtl8169_private *tp, int addr)
{
	return tp->csi_ops.read ? tp->csi_ops.read(tp, addr) : ~0;
}

static void rtl_csi_access_enable(struct rtl8169_private *tp, u32 bits)
{
	u32 csi;

	csi = rtl_csi_read(tp, 0x070c) & 0x00ffffff;
	rtl_csi_write(tp, 0x070c, csi | bits);
}

static void rtl_csi_access_enable_1(struct rtl8169_private *tp)
{
	rtl_csi_access_enable(tp, 0x17000000);
}

DECLARE_RTL_COND(rtl_csiar_cond)
{
	void __iomem *ioaddr = tp->mmio_addr;

	return RTL_R32(CSIAR) & CSIAR_FLAG;
}

static void r8169_csi_write(struct rtl8169_private *tp, int addr, int value)
{
	void __iomem *ioaddr = tp->mmio_addr;

	RTL_W32(CSIDR, value);
	RTL_W32(CSIAR, CSIAR_WRITE_CMD | (addr & CSIAR_ADDR_MASK) |
		CSIAR_BYTE_ENABLE << CSIAR_BYTE_ENABLE_SHIFT);

	rtl_udelay_loop_wait_low(tp, &rtl_csiar_cond, 10, 100);
}

static u32 r8169_csi_read(struct rtl8169_private *tp, int addr)
{
	void __iomem *ioaddr = tp->mmio_addr;

	RTL_W32(CSIAR, (addr & CSIAR_ADDR_MASK) |
		CSIAR_BYTE_ENABLE << CSIAR_BYTE_ENABLE_SHIFT);

	return rtl_udelay_loop_wait_high(tp, &rtl_csiar_cond, 10, 100) ?
		RTL_R32(CSIDR) : ~0;
}

static void rtl_init_csi_ops(struct rtl8169_private *tp)
{
	struct csi_ops *ops = &tp->csi_ops;

	ops->write	= r8169_csi_write;
	ops->read	= r8169_csi_read;
}

static void rtl_led_set(struct rtl8169_private *tp)
{
	void __iomem *ioaddr = tp->mmio_addr;

	if (tp->led_cfg) {
		RTL_W32(LEDSEL, 0x00060000 | tp->led_cfg);
	} else {
		#if defined(CONFIG_ARCH_RTD129x)
		RTL_W32(LEDSEL, 0x0006804F);
		#elif defined(CONFIG_ARCH_RTD139x)
		RTL_W32(LEDSEL, 0x000679a9);
		#elif defined(CONFIG_ARCH_RTD16xx) || defined(CONFIG_ARCH_RTD13xx)
		RTL_W32(LEDSEL, 0x00067ca9);
		#elif defined(CONFIG_ARCH_RTD16XXB)
		RTL_W32(LEDSEL, 0x17000CA9);
		#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x |
			* CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD13xx |
			* CONFIG_ARCH_RTD16XXB
			*/
	}
}

static void rtl_hw_start_8168g_1(struct rtl8169_private *tp)
{
	void __iomem *ioaddr = tp->mmio_addr;

	RTL_W32(TxConfig, RTL_R32(TxConfig) | TXCFG_AUTO_FIFO);

	#if !defined(RTL_ADJUST_FIFO_THRESHOLD)
	rtl_eri_write(tp, 0xc8, ERIAR_MASK_0101, 0x080002, ERIAR_EXGMAC);
	rtl_eri_write(tp, 0xcc, ERIAR_MASK_0001, 0x38, ERIAR_EXGMAC);
	rtl_eri_write(tp, 0xd0, ERIAR_MASK_0001, 0x48, ERIAR_EXGMAC);
	rtl_eri_write(tp, 0xe8, ERIAR_MASK_1111, 0x00100006, ERIAR_EXGMAC);
	#endif /* !RTL_ADJUST_FIFO_THRESHOLD */

	rtl_csi_access_enable_1(tp);

	rtl_w1w0_eri(tp, 0xdc, ERIAR_MASK_0001, 0x00, 0x01, ERIAR_EXGMAC);
	rtl_w1w0_eri(tp, 0xdc, ERIAR_MASK_0001, 0x01, 0x00, ERIAR_EXGMAC);
	rtl_eri_write(tp, 0x2f8, ERIAR_MASK_0011, 0x1d8f, ERIAR_EXGMAC);

	RTL_W8(ChipCmd, CmdTxEnb | CmdRxEnb);
	RTL_W32(MISC, RTL_R32(MISC) & ~RXDV_GATED_EN);
	RTL_W8(MaxTxPacketSize, EarlySize);

	rtl_eri_write(tp, 0xc0, ERIAR_MASK_0011, 0x0000, ERIAR_EXGMAC);
	rtl_eri_write(tp, 0xb8, ERIAR_MASK_0011, 0x0000, ERIAR_EXGMAC);

	/* Adjust EEE LED frequency */
	RTL_W8(EEE_LED, RTL_R8(EEE_LED) & ~0x07);

	rtl_w1w0_eri(tp, 0x2fc, ERIAR_MASK_0001, 0x01, 0x06, ERIAR_EXGMAC);
	rtl_w1w0_eri(tp, 0x1b0, ERIAR_MASK_0011, 0x0000, 0x1000, ERIAR_EXGMAC);
}

static void rtl_hw_start_8168g_2(struct rtl8169_private *tp)
{
	void __iomem *ioaddr = tp->mmio_addr;

	rtl_hw_start_8168g_1(tp);

	rtl_led_set(tp);

	/* disable aspm and clock request before access ephy */
	RTL_W8(Config2, RTL_R8(Config2) & ~ClkReqEn);
	RTL_W8(Config5, RTL_R8(Config5) & ~ASPM_en);
}

static void rtl_hw_start_8168(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	void __iomem *ioaddr = tp->mmio_addr;
	#if defined(RTL_ADJUST_FIFO_THRESHOLD) || defined(CONFIG_ARCH_RTD13xx)
	u32 phy_status;
	#endif
	#if defined(CONFIG_ARCH_RTD13xx)
	u32 tmp;
	#endif

	RTL_W8(Cfg9346, Cfg9346_Unlock);

	RTL_W8(MaxTxPacketSize, TxPacketMax);

	rtl_set_rx_max_size(ioaddr, rx_buf_sz);

	#if defined(CONFIG_RTL_RX_NO_COPY)
	tp->cp_cmd |= RTL_R16(CPlusCmd) | PktCntrDisable | INTT_3;
	#else
	tp->cp_cmd |= RTL_R16(CPlusCmd) | PktCntrDisable | INTT_1;
	#endif /* CONFIG_RTL_RX_NO_COPY */

	RTL_W16(CPlusCmd, tp->cp_cmd);

	RTL_W16(IntrMitigate, 0x5151);

	rtl_set_rx_tx_desc_registers(tp, ioaddr);

	rtl_set_rx_tx_config_registers(tp);

	#if defined(RTL_TX_NO_CLOSE)
	/* enable tx no close mode */
	rtl_ocp_write(tp, 0xE610,
		      rtl_ocp_read(tp, 0xE610) | (BIT(4) | BIT(6)));
	#endif /* RTL_TX_NO_CLOSE */

	#if defined(RTL_HANDLE_RDU0)
	tp->event_slow &= ~RxOverflow;
	#endif /* RTL_HANDLE_RDU0 */

	#if defined(RTL_ADJUST_FIFO_THRESHOLD)
	/* TX FIFO threshold */
	rtl_ocp_write(tp, 0xE618, 0x0006);
	rtl_ocp_write(tp, 0xE61A, 0x0010);

	/* RX FIFO threshold */
	rtl_ocp_write(tp, 0xC0A0, 0x0002);
	rtl_ocp_write(tp, 0xC0A2, 0x0008);

	phy_status = rtl_ocp_read(tp, 0xde40);
	if ((phy_status & 0x0030) == 0x0020) {
		rtl_ocp_write(tp, 0xC0A4, 0x0088);
		rtl_ocp_write(tp, 0xC0A8, 0x00A8);
	} else {
		rtl_ocp_write(tp, 0xC0A4, 0x0038);
		rtl_ocp_write(tp, 0xC0A8, 0x0048);
	}
	#endif /* RTL_ADJUST_FIFO_THRESHOLD */

	#if defined(CONFIG_ARCH_RTD16xx)
	/* disable pause frame resend caused by nearfull for revision A01 */
	if (get_rtd_chip_revision() == RTD_CHIP_A01)
		rtl_ocp_write(tp, 0xE862, rtl_ocp_read(tp, 0xE862) | BIT(0));
	#endif /* CONFIG_ARCH_RTD16xx */

	RTL_R8(IntrMask);

	rtl_hw_start_8168g_2(tp);

	RTL_W8(Cfg9346, Cfg9346_Lock);

	RTL_W8(ChipCmd, CmdTxEnb | CmdRxEnb);

	#if defined(CONFIG_ARCH_RTD13xx)
	phy_status = rtl_ocp_read(tp, 0xde40);
	switch (tp->output_mode) {
	case OUTPUT_EMBEDDED_PHY:
		/* do nothing */
		break;
	case OUTPUT_RMII:
		/* adjust RMII interface setting, speed */
		tmp = rtl_ocp_read(tp, 0xea30) & ~(BIT(6) | BIT(5));
		switch (phy_status & 0x0030) { /* link speed */
		case 0x0000:
			/* 10M, RGMII clock speed = 2.5MHz */
			break;
		case 0x0010:
			/* 100M, RGMII clock speed = 25MHz */
			tmp |= BIT(5);
		}
		/* adjust RGMII interface setting, duplex */
		if ((phy_status & BIT(3)) == 0)
			/* ETN spec, half duplex */
			tmp &= ~BIT(4);
		else	/* ETN spec, full duplex */
			tmp |= BIT(4);
		rtl_ocp_write(tp, 0xea30, tmp);
		break;
	case OUTPUT_RGMII_TO_MAC:
	case OUTPUT_RGMII_TO_PHY:
		/* adjust RGMII interface setting, duplex */
		tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(4) | BIT(3));
		switch (phy_status & 0x0030) { /* link speed */
		case 0x0000:
			/* 10M, RGMII clock speed = 2.5MHz */
			break;
		case 0x0010:
			/* 100M, RGMII clock speed = 25MHz */
			tmp |= BIT(3);
			break;
		case 0x0020:
			/* 1000M, RGMII clock speed = 125MHz */
			tmp |= BIT(4);
			break;
		}
		/* adjust RGMII interface setting, duplex */
		if ((phy_status & BIT(3)) == 0)
			/* ETN spec, half duplex */
			tmp &= ~BIT(2);
		else	/* ETN spec, full duplex */
			tmp |= BIT(2);
		rtl_ocp_write(tp, 0xea34, tmp);
	}
	#endif /* CONFIG_ARCH_RTD13xx */

	rtl_set_rx_mode(dev);

	RTL_W16(MultiIntr, RTL_R16(MultiIntr) & 0xF000);
}

static int link_status(void *data)
{
	struct net_device *dev = data;
	struct rtl8169_private *tp = netdev_priv(dev);
	void __iomem *ioaddr = tp->mmio_addr;

	set_user_nice(current, 5);
	pr_info("[Ethernet] Watch link status change.\n");

	while (!kthread_should_stop()) {
		wait_event_interruptible(tp->thr_wait, kthread_should_stop() ||
					 tp->link_chg);
		if (kthread_should_stop())
			break;
		else if (!tp->link_chg)
			continue;
		else
			tp->link_chg = 0;

		/* Send uevent about link status */
		if (tp->link_ok(ioaddr))
			kobject_uevent(&tp->dev->dev.kobj, KOBJ_LINKUP);
		else
			kobject_uevent(&tp->dev->dev.kobj, KOBJ_LINKDOWN);
	}

	do_exit(0);
}

static int rtl8169_change_mtu(struct net_device *dev, int new_mtu)
{
	#if defined(CONFIG_RTL_RX_NO_COPY)
	struct rtl8169_private *tp = netdev_priv(dev);
	#endif /* CONFIG_RTL_RX_NO_COPY */

	if (new_mtu < ETH_ZLEN || new_mtu > rtl_chip_infos.jumbo_max)
		return -EINVAL;

	#if defined(CONFIG_RTL_RX_NO_COPY)
	if (!netif_running(dev)) {
		rx_buf_sz_new = (new_mtu > ETH_DATA_LEN) ?
			new_mtu + ETH_HLEN + 8 + 1 : RX_BUF_SIZE;
		rx_buf_sz = rx_buf_sz_new;
		goto out;
	}

	rx_buf_sz_new = (new_mtu > ETH_DATA_LEN) ?
		new_mtu + ETH_HLEN + 8 + 1 : RX_BUF_SIZE;

	rtl_schedule_task(tp, RTL_FLAG_TASK_RESET_PENDING);

out:
	#endif /* CONFIG_RTL_RX_NO_COPY */

	dev->mtu = new_mtu;
	netdev_update_features(dev);

	return 0;
}

static inline void rtl8169_make_unusable_by_asic(struct RxDesc *desc)
{
	desc->addr = cpu_to_le64(0x0badbadbadbadbadull);
	desc->opts1 &= ~cpu_to_le32(DescOwn | RsvdMask);
}

#if defined(CONFIG_RTL_RX_NO_COPY)
static void rtl8169_free_rx_databuff(struct rtl8169_private *tp,
				     struct sk_buff **data_buff,
				     struct RxDesc *desc)
{
	if (!tp->acp_enable)
		dma_unmap_single(&tp->pdev->dev, le64_to_cpu(desc->addr),
				 rx_buf_sz, DMA_FROM_DEVICE);

	dev_kfree_skb(*data_buff);
	*data_buff = NULL;
	rtl8169_make_unusable_by_asic(desc);
}
#else
static void rtl8169_free_rx_databuff(struct rtl8169_private *tp,
				     void **data_buff, struct RxDesc *desc)
{
	if (!tp->acp_enable)
		dma_unmap_single(&tp->pdev->dev, le64_to_cpu(desc->addr),
				 rx_buf_sz, DMA_FROM_DEVICE);

	kfree(*data_buff);
	*data_buff = NULL;
	rtl8169_make_unusable_by_asic(desc);
}
#endif /* CONFIG_RTL_RX_NO_COPY */

static inline void rtl8169_mark_to_asic(struct RxDesc *desc, u32 rx_buf_sz)
{
	u32 eor = le32_to_cpu(desc->opts1) & RingEnd;

	desc->opts1 = cpu_to_le32(DescOwn | eor | rx_buf_sz);
}

static inline void rtl8169_map_to_asic(struct RxDesc *desc, dma_addr_t mapping,
				       u32 rx_buf_sz)
{
	desc->addr = cpu_to_le64(mapping);
	wmb(); /* make sure this RX descriptor is ready */
	rtl8169_mark_to_asic(desc, rx_buf_sz);
}

static inline void *rtl8169_align(void *data)
{
	return (void *)ALIGN((long)data, 16);
}

#if defined(CONFIG_RTL_RX_NO_COPY)
static int
rtl8168_alloc_rx_skb(struct rtl8169_private *tp,
		     struct sk_buff **sk_buff,
		     struct RxDesc *desc,
		     int rx_buf_sz)
{
	struct device *d = &tp->pdev->dev;
	struct sk_buff *skb;
	dma_addr_t mapping;
	int ret = 0;

	skb = dev_alloc_skb(rx_buf_sz + RTK_RX_ALIGN);
	if (!skb)
		goto err_out;

	skb_reserve(skb, RTK_RX_ALIGN);

	if (tp->acp_enable) {
		mapping = virt_to_phys(skb->data);
	} else {
		mapping = dma_map_single(d, skb->data, rx_buf_sz,
					 DMA_FROM_DEVICE);

		if (unlikely(dma_mapping_error(d, mapping))) {
			if (unlikely(net_ratelimit()))
				netif_err(tp, drv, tp->dev,
					  "Failed to map RX DMA!\n");
			goto err_out;
		}
	}
	*sk_buff = skb;
	rtl8169_map_to_asic(desc, mapping, rx_buf_sz);

out:
	return ret;

err_out:
	if (skb)
		dev_kfree_skb(skb);
	ret = -ENOMEM;
	rtl8169_make_unusable_by_asic(desc);
	goto out;
}
#else
static struct sk_buff *rtl8169_alloc_rx_data(struct rtl8169_private *tp,
					     struct RxDesc *desc)
{
	void *data;
	dma_addr_t mapping;
	struct device *d = &tp->pdev->dev;
	struct net_device *dev = tp->dev;
	int node = dev->dev.parent ? dev_to_node(dev->dev.parent) : -1;

	data = kmalloc_node(rx_buf_sz, GFP_KERNEL, node);
	if (!data)
		return NULL;

	if (rtl8169_align(data) != data) {
		kfree(data);
		data = kmalloc_node(rx_buf_sz + 15, GFP_KERNEL, node);
		if (!data)
			return NULL;
	}

	if (tp->acp_enable) {
		mapping = virt_to_phys(rtl8169_align(data));
	} else {
		mapping = dma_map_single(d, rtl8169_align(data), rx_buf_sz,
					 DMA_FROM_DEVICE);
		if (unlikely(dma_mapping_error(d, mapping))) {
			if (net_ratelimit())
				netif_err(tp, drv, tp->dev,
					  "Failed to map RX DMA!\n");
			goto err_out;
		}
	}

	rtl8169_map_to_asic(desc, mapping, rx_buf_sz);
	return data;

err_out:
	kfree(data);
	return NULL;
}
#endif /* CONFIG_RTL_RX_NO_COPY */

static void rtl8169_rx_clear(struct rtl8169_private *tp)
{
	unsigned int i;

	for (i = 0; i < NUM_RX_DESC; i++) {
		if (tp->Rx_databuff[i]) {
			rtl8169_free_rx_databuff(tp, tp->Rx_databuff + i,
						 tp->RxDescArray + i);
		}
	}
}

static inline void rtl8169_mark_as_last_descriptor(struct RxDesc *desc)
{
	desc->opts1 |= cpu_to_le32(RingEnd);
}

#if defined(CONFIG_RTL_RX_NO_COPY)
static u32
rtl8168_rx_fill(struct rtl8169_private *tp,
		struct net_device *dev,
		u32 start,
		u32 end)
{
	u32 cur;

	for (cur = start; end - cur > 0; cur++) {
		int ret, i = cur % NUM_RX_DESC;

		if (tp->Rx_databuff[i])
			continue;
		ret = rtl8168_alloc_rx_skb(tp, tp->Rx_databuff + i,
					   tp->RxDescArray + i, rx_buf_sz);
		if (ret < 0)
			break;
		if (i == (NUM_RX_DESC - 1))
			rtl8169_mark_as_last_descriptor(tp->RxDescArray +
							NUM_RX_DESC - 1);
	}
	return cur - start;
}
#else
static int rtl8169_rx_fill(struct rtl8169_private *tp)
{
	unsigned int i;

	for (i = 0; i < NUM_RX_DESC; i++) {
		void *data;

		if (tp->Rx_databuff[i])
			continue;

		data = rtl8169_alloc_rx_data(tp, tp->RxDescArray + i);
		if (!data) {
			rtl8169_make_unusable_by_asic(tp->RxDescArray + i);
			goto err_out;
		}
		tp->Rx_databuff[i] = data;
	}

	rtl8169_mark_as_last_descriptor(tp->RxDescArray + NUM_RX_DESC - 1);
	return 0;

err_out:
	rtl8169_rx_clear(tp);
	return -ENOMEM;
}
#endif /* CONFIG_RTL_RX_NO_COPY */

static int rtl8169_init_ring(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	int ret;

	rtl8169_init_ring_indexes(tp);

	memset(tp->tx_skb, 0x0, NUM_TX_DESC * sizeof(struct ring_info));
	memset(tp->Rx_databuff, 0x0, NUM_RX_DESC * sizeof(void *));

	#if defined(CONFIG_RTL_RX_NO_COPY)
	ret = rtl8168_rx_fill(tp, dev, 0, NUM_RX_DESC);
	if (ret < NUM_RX_DESC)
		ret = -ENOMEM;
	else
		ret = 0;
	#else
	ret = rtl8169_rx_fill(tp);
	#endif /* CONFIG_RTL_RX_NO_COPY */

	return ret;
}

static void rtl8169_unmap_tx_skb(struct rtl8169_private *tp, struct device *d,
				 struct ring_info *tx_skb, struct TxDesc *desc)
{
	unsigned int len = tx_skb->len;

	if (!tp->acp_enable)
		dma_unmap_single(d, le64_to_cpu(desc->addr), len,
				 DMA_TO_DEVICE);

	desc->opts1 = 0x00;
	desc->opts2 = 0x00;
	desc->addr = 0x00;
	tx_skb->len = 0;
}

static void rtl8169_tx_clear_range(struct rtl8169_private *tp, u32 start,
				   unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		unsigned int entry = (start + i) % NUM_TX_DESC;
		struct ring_info *tx_skb = tp->tx_skb + entry;
		unsigned int len = tx_skb->len;

		if (len) {
			struct sk_buff *skb = tx_skb->skb;

			rtl8169_unmap_tx_skb(tp, &tp->pdev->dev, tx_skb,
					     tp->TxDescArray + entry);
			if (skb) {
				tp->dev->stats.tx_dropped++;
				dev_kfree_skb(skb);
				tx_skb->skb = NULL;
			}
		}
	}
}

static void rtl8169_tx_clear(struct rtl8169_private *tp)
{
	rtl8169_tx_clear_range(tp, tp->dirty_tx, NUM_TX_DESC);
	tp->cur_tx = 0;
	tp->dirty_tx = 0;
}

static void rtl_reset_work(struct rtl8169_private *tp)
{
	struct net_device *dev = tp->dev;
	int i;

	napi_disable(&tp->napi);
	netif_stop_queue(dev);
	synchronize_rcu();

	rtl8169_hw_reset(tp);

	#if defined(CONFIG_RTL_RX_NO_COPY)
	rtl8169_rx_clear(tp);
	rtl8169_tx_clear(tp);

	if (rx_buf_sz_new != rx_buf_sz)
		rx_buf_sz = rx_buf_sz_new;

	memset(tp->TxDescArray, 0x0, NUM_TX_DESC * sizeof(struct TxDesc));
	for (i = 0; i < NUM_TX_DESC; i++) {
		if (i == (NUM_TX_DESC - 1))
			tp->TxDescArray[i].opts1 = cpu_to_le32(RingEnd);
	}
	memset(tp->RxDescArray, 0x0, NUM_RX_DESC * sizeof(struct RxDesc));

	if (rtl8169_init_ring(dev) < 0) {
		napi_enable(&tp->napi);
		netif_wake_queue(dev);
		netif_warn(tp, drv, dev, "No memory. Try to restart......\n");
		msleep(1000);
		rtl_schedule_task(tp, RTL_FLAG_TASK_RESET_PENDING);
		return;
	}
	#else
	for (i = 0; i < NUM_RX_DESC; i++)
		rtl8169_mark_to_asic(tp->RxDescArray + i, rx_buf_sz);

	rtl8169_tx_clear(tp);
	rtl8169_init_ring_indexes(tp);
	#endif /* CONFIG_RTL_RX_NO_COPY */

	napi_enable(&tp->napi);
	rtl_hw_start(dev);
	netif_wake_queue(dev);
	rtl8169_check_link_status(dev, tp, tp->mmio_addr);
}

static void rtl8169_tx_timeout(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	rtl_schedule_task(tp, RTL_FLAG_TASK_RESET_PENDING);
}

static int rtl8169_xmit_frags(struct rtl8169_private *tp, struct sk_buff *skb,
			      u32 *opts)
{
	struct skb_shared_info *info = skb_shinfo(skb);
	unsigned int cur_frag, entry;
	struct TxDesc *txd = NULL;
	struct device *d = &tp->pdev->dev;

	entry = tp->cur_tx;
	for (cur_frag = 0; cur_frag < info->nr_frags; cur_frag++) {
		const skb_frag_t *frag = info->frags + cur_frag;
		dma_addr_t mapping;
		u32 status, len;
		void *addr;

		entry = (entry + 1) % NUM_TX_DESC;

		txd = tp->TxDescArray + entry;
		len = skb_frag_size(frag);
		addr = skb_frag_address(frag);
		if (tp->acp_enable) {
			mapping = virt_to_phys(addr);
		} else {
			mapping = dma_map_single(d, addr, len, DMA_TO_DEVICE);
			if (unlikely(dma_mapping_error(d, mapping))) {
				if (net_ratelimit())
					netif_err(tp, drv, tp->dev,
						  "Failed to map TX fragments DMA!\n");
				goto err_out;
			}
		}

		/* Anti gcc 2.95.3 bugware (sic) */
		status = opts[0] | len |
			(RingEnd * !((entry + 1) % NUM_TX_DESC));

		txd->opts1 = cpu_to_le32(status);
		txd->opts2 = cpu_to_le32(opts[1]);
		txd->addr = cpu_to_le64(mapping);

		tp->tx_skb[entry].len = len;
	}

	if (cur_frag) {
		tp->tx_skb[entry].skb = skb;
		txd->opts1 |= cpu_to_le32(LastFrag);
	}

	return cur_frag;

err_out:
	rtl8169_tx_clear_range(tp, tp->cur_tx + 1, cur_frag);
	return -EIO;
}

static inline bool rtl8169_tso_csum(struct rtl8169_private *tp,
				    struct sk_buff *skb, u32 *opts)
{
	const struct rtl_tx_desc_info *info = &tx_desc_info;
	u32 mss = skb_shinfo(skb)->gso_size;
	int offset = info->opts_offset;

	if (mss) {
		opts[0] |= TD_LSO;
		opts[offset] |= min(mss, TD_MSS_MAX) << info->mss_shift;
	} else if (skb->ip_summed == CHECKSUM_PARTIAL) {
		const struct iphdr *ip = ip_hdr(skb);

		if (ip->protocol == IPPROTO_TCP)
			opts[offset] |= info->checksum.tcp;
		else if (ip->protocol == IPPROTO_UDP)
			opts[offset] |= info->checksum.udp;
		else
			WARN_ON_ONCE(1);
	}
	return true;
}

static netdev_tx_t rtl8169_start_xmit(struct sk_buff *skb,
				      struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	unsigned int entry = tp->cur_tx % NUM_TX_DESC;
	struct TxDesc *txd = tp->TxDescArray + entry;
	void __iomem *ioaddr = tp->mmio_addr;
	struct device *d = &tp->pdev->dev;
	dma_addr_t mapping;
	u32 status, len;
	u32 opts[2];
	int frags;
	#if defined(RTL_TX_NO_CLOSE)
	u16 close_idx;
	u16 tail_idx;
	#endif /* RTL_TX_NO_CLOSE */

	if (unlikely(!TX_FRAGS_READY_FOR(tp, skb_shinfo(skb)->nr_frags))) {
		netif_err(tp, drv, dev, "BUG! Tx Ring full when queue awake!\n");
		goto err_stop_0;
	}

	#if defined(RTL_TX_NO_CLOSE)
	close_idx = RTL_R16(TX_DESC_CLOSE_IDX) & TX_DESC_CNT_MASK;
	tail_idx = tp->cur_tx & TX_DESC_CNT_MASK;

	if ((tail_idx > close_idx && (tail_idx - close_idx == NUM_TX_DESC)) ||
	    (tail_idx < close_idx &&
	     (TX_DESC_CNT_SIZE - close_idx + tail_idx == NUM_TX_DESC)))
		goto err_stop_0;
	#else
	if (unlikely(le32_to_cpu(txd->opts1) & DescOwn))
		goto err_stop_0;
	#endif /* RTL_TX_NO_CLOSE */

	opts[1] = cpu_to_le32(rtl8169_tx_vlan_tag(skb));
	opts[0] = DescOwn;

	if (!rtl8169_tso_csum(tp, skb, opts))
		goto err_update_stats;

	len = skb_headlen(skb);
	if (tp->acp_enable) {
		mapping = virt_to_phys(skb->data);
	} else {
		mapping = dma_map_single(d, skb->data, len, DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(d, mapping))) {
			if (net_ratelimit())
				netif_err(tp, drv, dev,
					  "Failed to map TX DMA!\n");
			goto err_dma_0;
		}
	}

	tp->tx_skb[entry].len = len;
	txd->addr = cpu_to_le64(mapping);

	frags = rtl8169_xmit_frags(tp, skb, opts);
	if (frags < 0) {
		goto err_dma_1;
	} else if (frags) {
		opts[0] |= FirstFrag;
	} else {
		opts[0] |= FirstFrag | LastFrag;
		tp->tx_skb[entry].skb = skb;
	}

	txd->opts2 = cpu_to_le32(opts[1]);

	skb_tx_timestamp(skb);

	wmb(); /* make sure txd->addr and txd->opts2 is ready */

	/* Anti gcc 2.95.3 bugware (sic) */
	status = opts[0] | len | (RingEnd * !((entry + 1) % NUM_TX_DESC));
	txd->opts1 = cpu_to_le32(status);

	tp->cur_tx += frags + 1;

	#if defined(RTL_TX_NO_CLOSE)
	RTL_W16(TX_DESC_TAIL_IDX, tp->cur_tx & TX_DESC_CNT_MASK);
	#endif /* RTL_TX_NO_CLOSE */

	wmb(); /* make sure this TX descriptor is ready */

	RTL_W8(TxPoll, NPQ);

	mmiowb();

	if (!TX_FRAGS_READY_FOR(tp, MAX_SKB_FRAGS)) {
		/* Avoid wrongly optimistic queue wake-up: rtl_tx thread must
		 * not miss a ring update when it notices a stopped queue.
		 */
		smp_wmb();
		netif_stop_queue(dev);
		/* Sync with rtl_tx:
		 * - publish queue status and cur_tx ring index (write barrier)
		 * - refresh dirty_tx ring index (read barrier).
		 * May the current thread have a pessimistic view of the ring
		 * status and forget to wake up queue, a racing rtl_tx thread
		 * can't.
		 */
		smp_mb();
		if (TX_FRAGS_READY_FOR(tp, MAX_SKB_FRAGS))
			netif_wake_queue(dev);
	}

	return NETDEV_TX_OK;

err_dma_1:
	rtl8169_unmap_tx_skb(tp, d, tp->tx_skb + entry, txd);
err_dma_0:
	dev_kfree_skb(skb);
err_update_stats:
	dev->stats.tx_dropped++;
	return NETDEV_TX_OK;

err_stop_0:
	netif_stop_queue(dev);
	dev->stats.tx_dropped++;
	return NETDEV_TX_BUSY;
}

static void rtl_tx(struct net_device *dev, struct rtl8169_private *tp)
{
	unsigned int dirty_tx, tx_left;
	#if defined(RTL_TX_NO_CLOSE)
	u16 close_idx;
	u16 dirty_tx_idx;
	void __iomem *ioaddr = tp->mmio_addr;
	#endif /* RTL_TX_NO_CLOSE */

	dirty_tx = tp->dirty_tx;
	#if defined(RTL_TX_NO_CLOSE)
	close_idx = RTL_R16(TX_DESC_CLOSE_IDX) & TX_DESC_CNT_MASK;
	dirty_tx_idx = dirty_tx & TX_DESC_CNT_MASK;
	smp_rmb(); /* make sure dirty_tx is updated */
	if (dirty_tx_idx <= close_idx)
		tx_left = close_idx - dirty_tx_idx;
	else
		tx_left = close_idx + TX_DESC_CNT_SIZE - dirty_tx_idx;
	#else
	smp_rmb(); /* make sure dirty_tx is updated */
	tx_left = tp->cur_tx - dirty_tx;
	#endif /* RTL_TX_NO_CLOSE */

	while (tx_left > 0) {
		unsigned int entry = dirty_tx % NUM_TX_DESC;
		struct ring_info *tx_skb = tp->tx_skb + entry;
		u32 status;

		rmb(); /* make sure this TX descriptor is ready */
		status = le32_to_cpu(tp->TxDescArray[entry].opts1);
		#if !defined(RTL_TX_NO_CLOSE)
		if (status & DescOwn)
			break;
		#endif /* !RTL_TX_NO_CLOSE */

		rtl8169_unmap_tx_skb(tp, &tp->pdev->dev, tx_skb,
				     tp->TxDescArray + entry);
		if (status & LastFrag) {
			u64_stats_update_begin(&tp->tx_stats.syncp);
			tp->tx_stats.packets++;
			tp->tx_stats.bytes += tx_skb->skb->len;
			u64_stats_update_end(&tp->tx_stats.syncp);
			dev_kfree_skb(tx_skb->skb);
			tx_skb->skb = NULL;
		}
		dirty_tx++;
		tx_left--;
	}

	if (tp->dirty_tx != dirty_tx) {
		tp->dirty_tx = dirty_tx;
		/* Sync with rtl8169_start_xmit:
		 * - publish dirty_tx ring index (write barrier)
		 * - refresh cur_tx ring index and queue status (read barrier)
		 * May the current thread miss the stopped queue condition,
		 * a racing xmit thread can only have a right view of the
		 * ring status.
		 */
		smp_mb();
		if (netif_queue_stopped(dev) &&
		    TX_FRAGS_READY_FOR(tp, MAX_SKB_FRAGS)) {
			netif_wake_queue(dev);
		}
		/* 8168 hack: TxPoll requests are lost when the Tx packets are
		 * too close. Let's kick an extra TxPoll request when a burst
		 * of start_xmit activity is detected (if it is not detected,
		 * it is slow enough). -- FR
		 */
		if (tp->cur_tx != dirty_tx) {
			void __iomem *ioaddr = tp->mmio_addr;

			RTL_W8(TxPoll, NPQ);
		}
	}
}

static inline int rtl8169_fragmented_frame(u32 status)
{
	return (status & (FirstFrag | LastFrag)) != (FirstFrag | LastFrag);
}

static inline void rtl8169_rx_csum(struct sk_buff *skb, u32 opts1)
{
	u32 status = opts1 & RxProtoMask;

	if ((status == RxProtoTCP && !(opts1 & TCPFail)) ||
	    (status == RxProtoUDP && !(opts1 & UDPFail)))
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	else
		skb_checksum_none_assert(skb);
}

#if defined(CONFIG_RTL_RX_NO_COPY)
static int rtl_rx(struct net_device *dev, struct rtl8169_private *tp,
		  u32 budget)
{
	unsigned int cur_rx, rx_left;
	unsigned int count, delta;
	struct device *d = &tp->pdev->dev;

	cur_rx = tp->cur_rx;

	rx_left = NUM_RX_DESC + tp->dirty_rx - cur_rx;
	rx_left = min(budget, rx_left);

	for (; rx_left > 0; rx_left--, cur_rx++) {
		unsigned int entry = cur_rx % NUM_RX_DESC;
		struct RxDesc *desc = tp->RxDescArray + entry;
		u32 status;

		rmb(); /* make sure this RX descriptor is ready */
		status = le32_to_cpu(desc->opts1) & tp->opts1_mask;

		if (status & DescOwn)
			break;
		if (unlikely(status & RxRES)) {
			netif_info(tp, rx_err, dev, "Rx ERROR. status = %08x\n",
				   status);
			dev->stats.rx_errors++;
			if (status & (RxRWT | RxRUNT))
				dev->stats.rx_length_errors++;
			if (status & RxCRC)
				dev->stats.rx_crc_errors++;
			if (status & RxFOVF) {
				rtl_schedule_task(tp, RTL_FLAG_TASK_RESET_PENDING);
				dev->stats.rx_fifo_errors++;
			}
			if ((status & (RxRUNT | RxCRC)) &&
			    !(status & (RxRWT | RxFOVF)) &&
			    (dev->features & NETIF_F_RXALL))
				goto process_pkt;
		} else {
			struct sk_buff *skb;
			dma_addr_t addr;
			int pkt_size;

process_pkt:

			skb = tp->Rx_databuff[entry];
			addr = le64_to_cpu(desc->addr);
			if (likely(!(dev->features & NETIF_F_RXFCS)))
				pkt_size = (status & 0x00003fff) - 4;
			else
				pkt_size = status & 0x00003fff;

			/* The driver does not support incoming fragmented
			 * frames. They are seen as a symptom of over-mtu
			 * sized frames.
			 */
			if (unlikely(rtl8169_fragmented_frame(status))) {
				dev->stats.rx_dropped++;
				dev->stats.rx_length_errors++;
				continue;
			}

			if (!tp->acp_enable)
				dma_sync_single_for_cpu(d, addr, pkt_size,
							DMA_FROM_DEVICE);
			tp->Rx_databuff[entry] = NULL;
			if (!tp->acp_enable)
				dma_unmap_single(d, addr, pkt_size,
						 DMA_FROM_DEVICE);

			rtl8169_rx_csum(skb, status);
			skb_put(skb, pkt_size);
			skb->protocol = eth_type_trans(skb, dev);

			rtl8169_rx_vlan_tag(desc, skb);

			napi_gro_receive(&tp->napi, skb);

			u64_stats_update_begin(&tp->rx_stats.syncp);
			tp->rx_stats.packets++;
			tp->rx_stats.bytes += pkt_size;
			u64_stats_update_end(&tp->rx_stats.syncp);
		}
	}

	count = cur_rx - tp->cur_rx;
	tp->cur_rx = cur_rx;

	delta = rtl8168_rx_fill(tp, dev, tp->dirty_rx, tp->cur_rx);
	/* netif_err(tp, drv, tp->dev, "delta =%x\n",delta); */
	tp->dirty_rx += delta;

	if (tp->dirty_rx + NUM_RX_DESC == tp->cur_rx) {
		rtl_schedule_task(tp, RTL_FLAG_TASK_RESET_PENDING);
		netif_err(tp, drv, tp->dev, "%s: Rx buffers exhausted\n",
			  dev->name);
	}

	return count;
}
#else
static struct sk_buff *rtl8169_try_rx_copy(void *data,
					   struct rtl8169_private *tp,
					   int pkt_size, dma_addr_t addr)
{
	struct sk_buff *skb;
	struct device *d = &tp->pdev->dev;

	data = rtl8169_align(data);
	if (!tp->acp_enable)
		dma_sync_single_for_cpu(d, addr, pkt_size, DMA_FROM_DEVICE);
	prefetch(data);
	skb = netdev_alloc_skb_ip_align(tp->dev, pkt_size);
	if (skb)
		memcpy(skb->data, data, pkt_size);
	if (!tp->acp_enable)
		dma_sync_single_for_device(d, addr, pkt_size, DMA_FROM_DEVICE);

	return skb;
}

static int rtl_rx(struct net_device *dev, struct rtl8169_private *tp,
		  u32 budget)
{
	unsigned int cur_rx, rx_left;
	unsigned int count;
	const unsigned int num_rx_desc = NUM_RX_DESC;

	cur_rx = tp->cur_rx;

	for (rx_left = min(budget, num_rx_desc); rx_left > 0;
		rx_left--, cur_rx++) {
		unsigned int entry = cur_rx % NUM_RX_DESC;
		struct RxDesc *desc = tp->RxDescArray + entry;
		u32 status;

		rmb(); /* make sure this RX descriptor is ready */
		status = le32_to_cpu(desc->opts1) & tp->opts1_mask;

		if (status & DescOwn)
			break;
		if (unlikely(status & RxRES)) {
			netif_info(tp, rx_err, dev, "Rx ERROR. status = %08x\n",
				   status);
			dev->stats.rx_errors++;
			if (status & (RxRWT | RxRUNT))
				dev->stats.rx_length_errors++;
			if (status & RxCRC)
				dev->stats.rx_crc_errors++;
			if (status & RxFOVF) {
				rtl_schedule_task(tp, RTL_FLAG_TASK_RESET_PENDING);
				dev->stats.rx_fifo_errors++;
			}
			if ((status & (RxRUNT | RxCRC)) &&
			    !(status & (RxRWT | RxFOVF)) &&
			    (dev->features & NETIF_F_RXALL))
				goto process_pkt;
		} else {
			struct sk_buff *skb;
			dma_addr_t addr;
			int pkt_size;

process_pkt:
			addr = le64_to_cpu(desc->addr);
			if (likely(!(dev->features & NETIF_F_RXFCS)))
				pkt_size = (status & 0x00003fff) - 4;
			else
				pkt_size = status & 0x00003fff;

			/* The driver does not support incoming fragmented
			 * frames. They are seen as a symptom of over-mtu
			 * sized frames.
			 */
			if (unlikely(rtl8169_fragmented_frame(status))) {
				dev->stats.rx_dropped++;
				dev->stats.rx_length_errors++;
				goto release_descriptor;
			}

			skb = rtl8169_try_rx_copy(tp->Rx_databuff[entry],
						  tp, pkt_size, addr);
			if (!skb) {
				dev->stats.rx_dropped++;
				goto release_descriptor;
			}

			rtl8169_rx_csum(skb, status);
			skb_put(skb, pkt_size);
			skb->protocol = eth_type_trans(skb, dev);

			rtl8169_rx_vlan_tag(desc, skb);

			napi_gro_receive(&tp->napi, skb);

			u64_stats_update_begin(&tp->rx_stats.syncp);
			tp->rx_stats.packets++;
			tp->rx_stats.bytes += pkt_size;
			u64_stats_update_end(&tp->rx_stats.syncp);
		}
release_descriptor:
		desc->opts2 = 0;
		wmb(); /* make sure this RX descriptor is useless */
		rtl8169_mark_to_asic(desc, rx_buf_sz);
	}

	count = cur_rx - tp->cur_rx;
	tp->cur_rx = cur_rx;

	return count;
}
#endif /* CONFIG_RTL_RX_NO_COPY */

static irqreturn_t phy_irq_handler(int irq, void *dev_instance)
{
	struct rtl8169_private *tp = dev_instance;
	u32 isr;
	u32 por;
	int i;
	bool hit = false;

	pr_info("%s:%d: irq %d, tp 0x%p\n", __func__, __LINE__, irq, tp);
	isr = readl(tp->mmio_clkaddr + ISO_UMSK_ISR);
	pr_info("ISO_UMSK_ISR = 0x%08x\n", isr);
	por = readl(tp->mmio_clkaddr + ISO_POR_DATAI);
	pr_info("ISO_POR_DATAI = 0x%08x\n", por);

	for (i = 0; i < tp->phy_irq_num; i++) {
		if (irq == tp->phy_irq[i] &&
		    (isr & (1 << tp->phy_irq_map[i]))) {
			pr_info("%s: phy_0: get IRQ %d for GPHY POR interrupt (%d)\n",
				tp->dev->name, irq, tp->phy_irq_map[i]);
			writel(1 << tp->phy_irq_map[i],
			       tp->mmio_clkaddr + ISO_UMSK_ISR);
			hit = true;
			break;
		}
	}

	if (hit) {
		if ((por & tp->phy_por_xv_mask) != tp->phy_por_xv_mask) {
			pr_err("%s: phy_0: GPHY has power issue (0x%x)\n",
			       tp->dev->name, por);
			goto out;
		}

		if (!test_bit(RTL_FLAG_TASK_ENABLED, tp->wk.flags))
			rtl_phy_reinit(tp);
		else if (atomic_inc_return(&tp->phy_reinit_flag) == 1)
			rtl_schedule_task(tp, RTL_FLAG_TASK_PHY_PENDING);
		else
			atomic_dec(&tp->phy_reinit_flag);
	}

out:
	return IRQ_RETVAL(IRQ_HANDLED);
}

static irqreturn_t rtl8169_interrupt(int irq, void *dev_instance)
{
	struct net_device *dev = dev_instance;
	struct rtl8169_private *tp = netdev_priv(dev);
	int handled = 0;
	u16 status;

	status = rtl_get_events(tp);
	if (status && status != 0xffff) {
		status &= RTL_EVENT_NAPI | tp->event_slow;
		if (status) {
			handled = 1;

			rtl_irq_disable(tp);
			napi_schedule(&tp->napi);
		}
	}
	return IRQ_RETVAL(handled);
}

/* Workqueue context.
 */
static void rtl_slow_event_work(struct rtl8169_private *tp)
{
	struct net_device *dev = tp->dev;
	void __iomem *ioaddr = tp->mmio_addr;
	u16 status;

	status = rtl_get_events(tp) & tp->event_slow;
	rtl_ack_events(tp, status);

	if (status & LinkChg) {
		__rtl8169_check_link_status(dev, tp, tp->mmio_addr, true);

		/* Add for link status change */
		tp->link_chg = 1;
		wake_up(&tp->thr_wait);

		/* prevet ALDPS enter MAC Powercut Tx/Rx disable */
		/* use MAC reset to set counter to offset 0 */
		if (tp->link_ok(ioaddr)) {
			if (RTL_R8(PHYstatus) & _100bps)
			{
				//#define CHIP_Bonding       0x98007028  // bit 11 : 0 => RTD1295 & RTD1294), 1 => (RTD1296)
				if (readl(tp->mmio_clkaddr + 0x28) &BIT(11))
				{
					// IGPIO32 pin mux : 0x98007314 [23:22] =00
					writel(readl(tp->mmio_clkaddr + 0x314) & ~0x00C00000, (tp->mmio_clkaddr + 0x314));
					// IGPIO32 : out direction => 0x98007118[0]=1, out data => 0x9800711c[0]
					writel(readl(tp->mmio_clkaddr + 0x118) | 0x00000001, (tp->mmio_clkaddr + 0x118));
					writel(readl(tp->mmio_clkaddr + 0x11c) & ~0x00000001, (tp->mmio_clkaddr + 0x11c));
				}
				else
				{
					// IGPIO30 pin mux : 0x98007314 [19:18] =00
					writel(readl(tp->mmio_clkaddr + 0x314) & ~0x000C0000, (tp->mmio_clkaddr + 0x314));
					// IGPIO30 : out direction => 0x98007100[30]=1, out data => 0x98007104[30]
					writel(readl(tp->mmio_clkaddr + 0x100) | 0x40000000, (tp->mmio_clkaddr + 0x100));
					writel(readl(tp->mmio_clkaddr + 0x104) & ~0x40000000, (tp->mmio_clkaddr + 0x104));
				}
			}	
			else
			{
				if (readl(tp->mmio_clkaddr + 0x28) &BIT(11))
				{
					writel(readl(tp->mmio_clkaddr + 0x314) & ~0x00C00000, (tp->mmio_clkaddr + 0x314));
					writel(readl(tp->mmio_clkaddr + 0x118) | 0x00000001, (tp->mmio_clkaddr + 0x118));
					writel(readl(tp->mmio_clkaddr + 0x11c) | 0x00000001, (tp->mmio_clkaddr + 0x11c));
				}
				else
				{
					writel(readl(tp->mmio_clkaddr + 0x314) & ~0x000C0000, (tp->mmio_clkaddr + 0x314));
					writel(readl(tp->mmio_clkaddr + 0x100) | 0x40000000, (tp->mmio_clkaddr + 0x100));
					writel(readl(tp->mmio_clkaddr + 0x104) | 0x40000000, (tp->mmio_clkaddr + 0x104));
				}
			}
			rtl_schedule_task(tp, RTL_FLAG_TASK_RESET_PENDING);
		} else {
			if (readl(tp->mmio_clkaddr + 0x28) &BIT(11))
			{
				writel(readl(tp->mmio_clkaddr + 0x314) & ~0x00C00000, (tp->mmio_clkaddr + 0x314));
				writel(readl(tp->mmio_clkaddr + 0x118) | 0x00000001, (tp->mmio_clkaddr + 0x118));
				writel(readl(tp->mmio_clkaddr + 0x11c) | 0x00000001, (tp->mmio_clkaddr + 0x11c));
			}
			else
			{
				writel(readl(tp->mmio_clkaddr + 0x314) & ~0x000C0000, (tp->mmio_clkaddr + 0x314));
				writel(readl(tp->mmio_clkaddr + 0x100) | 0x40000000, (tp->mmio_clkaddr + 0x100));
				writel(readl(tp->mmio_clkaddr + 0x104) | 0x40000000, (tp->mmio_clkaddr + 0x104));
			}
		}
	}

	rtl_irq_enable_all(tp);
}

static void rtl_task(struct work_struct *work)
{
	static const struct {
		int bitnr;
		void (*action)(struct rtl8169_private *tp);
	} rtl_work[] = {
		/* XXX - keep rtl_slow_event_work() as first element. */
		{ RTL_FLAG_TASK_SLOW_PENDING,	rtl_slow_event_work },
		{ RTL_FLAG_TASK_RESET_PENDING,	rtl_reset_work },
		{ RTL_FLAG_TASK_PHY_PENDING,	rtl_phy_work }
	};
	struct rtl8169_private *tp =
		container_of(work, struct rtl8169_private, wk.work);
	struct net_device *dev = tp->dev;
	int i;

	rtl_lock_work(tp);

	if (!netif_running(dev) ||
	    !test_bit(RTL_FLAG_TASK_ENABLED, tp->wk.flags))
		goto out_unlock;

	for (i = 0; i < ARRAY_SIZE(rtl_work); i++) {
		bool pending;

		pending = test_and_clear_bit(rtl_work[i].bitnr, tp->wk.flags);
		if (pending)
			rtl_work[i].action(tp);
	}

out_unlock:
	rtl_unlock_work(tp);
}

static int rtl8169_poll(struct napi_struct *napi, int budget)
{
	struct rtl8169_private *tp =
		container_of(napi, struct rtl8169_private, napi);
	struct net_device *dev = tp->dev;
	u16 enable_mask = RTL_EVENT_NAPI | tp->event_slow;
	int work_done = 0;
	u16 status;
	#if defined(RTL_HANDLE_RDU0) && defined(CONFIG_RTL_RX_NO_COPY)
	u32 old_dirty_rx;
	#endif /* RTL_HANDLE_RDU0 & CONFIG_RTL_RX_NO_COPY */

	status = rtl_get_events(tp);
	rtl_ack_events(tp, status & ~(tp->event_slow | RxOverflow));

	#if defined(RTL_HANDLE_RDU0) && defined(CONFIG_RTL_RX_NO_COPY)
	old_dirty_rx = tp->dirty_rx;
	#endif /* RTL_HANDLE_RDU0 & CONFIG_RTL_RX_NO_COPY */
	if (status & RTL_EVENT_NAPI_RX)
		work_done = rtl_rx(dev, tp, (u32)budget);

	#if defined(RTL_HANDLE_RDU0)
	#if defined(CONFIG_RTL_RX_NO_COPY)
	if ((status & RxOverflow) && tp->dirty_rx != old_dirty_rx)
		rtl_ack_events(tp, RxOverflow);
	#else
	if ((status & RxOverflow) && work_done > 0)
		rtl_ack_events(tp, RxOverflow);
	#endif /* CONFIG_RTL_RX_NO_COPY */
	#endif /* RTL_HANDLE_RDU0 */

	if (status & RTL_EVENT_NAPI_TX)
		rtl_tx(dev, tp);

	if (status & tp->event_slow) {
		enable_mask &= ~tp->event_slow;

		rtl_schedule_task(tp, RTL_FLAG_TASK_SLOW_PENDING);
	}

	if (work_done < budget) {
		napi_complete(napi);

		rtl_irq_enable(tp, enable_mask);
		mmiowb();
	}

	return work_done;
}

static void rtl8169_down(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	napi_disable(&tp->napi);
	netif_stop_queue(dev);

	rtl8169_hw_reset(tp);
	/* At this point device interrupts can not be enabled in any function,
	 * as netif_running is not true (rtl8169_interrupt, rtl8169_reset_task)
	 * and napi is disabled (rtl8169_poll).
	 */

	/* Give a racing hard_start_xmit a few cycles to complete. */
	synchronize_rcu();

	rtl8169_tx_clear(tp);

	rtl8169_rx_clear(tp);

	rtl_pll_power_down(tp);
}

static int rtl8169_close(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	struct platform_device *pdev = tp->pdev;

	/* Update counters before going down */
	rtl8169_update_counters(dev);

	rtl_lock_work(tp);
	clear_bit(RTL_FLAG_TASK_ENABLED, tp->wk.flags);

	rtl8169_down(dev);

	rtl_unlock_work(tp);

	free_irq(dev->irq, dev);

	if (tp->acp_enable) {
		kfree(tp->RxDescArray);
		kfree(tp->TxDescArray);
	} else {
		dma_free_coherent(&pdev->dev, R8169_RX_RING_BYTES,
				  tp->RxDescArray, tp->RxPhyAddr);
		dma_free_coherent(&pdev->dev, R8169_TX_RING_BYTES,
				  tp->TxDescArray, tp->TxPhyAddr);
	}
	tp->TxDescArray = NULL;
	tp->RxDescArray = NULL;

	return 0;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void rtl8169_netpoll(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	rtl8169_interrupt(tp->pdev->irq, dev);
}
#endif

static int rtl_open(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	void __iomem *ioaddr = tp->mmio_addr;
	struct platform_device *pdev = tp->pdev;
	int retval = -ENOMEM;
	int node = dev->dev.parent ? dev_to_node(dev->dev.parent) : -1;

	/* Rx and Tx descriptors needs 256 bytes alignment.
	 * dma_alloc_coherent provides more.
	 */
	if (tp->acp_enable) {
		tp->TxDescArray = kzalloc_node(R8169_TX_RING_BYTES,
					       GFP_KERNEL, node);
		tp->TxPhyAddr = virt_to_phys(tp->TxDescArray);
	} else {
		tp->TxDescArray = dma_alloc_coherent(&pdev->dev,
						     R8169_TX_RING_BYTES,
						     &tp->TxPhyAddr,
						     GFP_KERNEL);
	}
	if (!tp->TxDescArray)
		goto err_pm_runtime_put;

	if (tp->acp_enable) {
		tp->RxDescArray = kzalloc_node(R8169_RX_RING_BYTES,
					       GFP_KERNEL, node);
		tp->RxPhyAddr = virt_to_phys(tp->RxDescArray);
	} else {
		tp->RxDescArray = dma_alloc_coherent(&pdev->dev,
						     R8169_RX_RING_BYTES,
						     &tp->RxPhyAddr,
						     GFP_KERNEL);
	}
	if (!tp->RxDescArray)
		goto err_free_tx_0;

	retval = rtl8169_init_ring(dev);
	if (retval < 0)
		goto err_free_rx_1;

	INIT_WORK(&tp->wk.work, rtl_task);

	smp_mb(); /* make sure TX/RX rings are ready */

	retval = request_irq(dev->irq, rtl8169_interrupt,
			     (tp->features & RTL_FEATURE_MSI) ? 0 : IRQF_SHARED,
			     dev->name, dev);
	if (retval < 0)
		goto err_free_rx_2;

	rtl_lock_work(tp);

	set_bit(RTL_FLAG_TASK_ENABLED, tp->wk.flags);

	napi_enable(&tp->napi);

	rtl8169_init_phy(dev, tp);

	__rtl8169_set_features(dev, dev->features);

	rtl_pll_power_up(tp);

	rtl_hw_start(dev);

	netif_start_queue(dev);

	rtl_unlock_work(tp);

	tp->saved_wolopts = 0;

	rtl8169_check_link_status(dev, tp, ioaddr);
out:
	return retval;

err_free_rx_2:
	rtl8169_rx_clear(tp);
err_free_rx_1:
	if (tp->acp_enable)
		kfree(tp->RxDescArray);
	else
		dma_free_coherent(&pdev->dev, R8169_RX_RING_BYTES,
				  tp->RxDescArray, tp->RxPhyAddr);
	tp->RxDescArray = NULL;
err_free_tx_0:
	if (tp->acp_enable)
		kfree(tp->TxDescArray);
	else
		dma_free_coherent(&pdev->dev, R8169_TX_RING_BYTES,
				  tp->TxDescArray, tp->TxPhyAddr);
	tp->TxDescArray = NULL;
err_pm_runtime_put:
	goto out;
}

static struct rtnl_link_stats64 *
rtl8169_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *stats)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	unsigned int start;

	do {
		start = u64_stats_fetch_begin_irq(&tp->rx_stats.syncp);
		stats->rx_packets = tp->rx_stats.packets;
		stats->rx_bytes	= tp->rx_stats.bytes;
	} while (u64_stats_fetch_retry_irq(&tp->rx_stats.syncp, start));

	do {
		start = u64_stats_fetch_begin_irq(&tp->tx_stats.syncp);
		stats->tx_packets = tp->tx_stats.packets;
		stats->tx_bytes	= tp->tx_stats.bytes;
	} while (u64_stats_fetch_retry_irq(&tp->tx_stats.syncp, start));

	stats->rx_dropped	= dev->stats.rx_dropped;
	stats->tx_dropped	= dev->stats.tx_dropped;
	stats->rx_length_errors = dev->stats.rx_length_errors;
	stats->rx_errors	= dev->stats.rx_errors;
	stats->rx_crc_errors	= dev->stats.rx_crc_errors;
	stats->rx_fifo_errors	= dev->stats.rx_fifo_errors;
	stats->rx_missed_errors = dev->stats.rx_missed_errors;

	return stats;
}

#if defined(CONFIG_ARCH_RTD139x) || defined(CONFIG_ARCH_RTD16xx) || \
	defined(CONFIG_ARCH_RTD13xx) || defined(CONFIG_ARCH_RTD16XXB)
static void r8169soc_eee_init(struct rtl8169_private *tp, bool enable);
#endif /* CONFIG_ARCH_RTD139x | CONFIG_ARCH_RTD16xx |
	* CONFIG_ARCH_RTD13xx | CONFIG_ARCH_RTD16XXB
	*/

static void rtl8169_net_suspend(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	if (!netif_running(dev))
		return;

	netif_device_detach(dev);
	netif_stop_queue(dev);

	rtl_lock_work(tp);
	napi_disable(&tp->napi);
	clear_bit(RTL_FLAG_TASK_ENABLED, tp->wk.flags);
	rtl_unlock_work(tp);

	/* disable EEE if it is enabled */
	if (tp->eee_enable) {
		#if defined(CONFIG_ARCH_RTD129x)
		rtl_mmd_write(tp, 0x7, 0x3c, 0);

		/* turn off EEE */
		rtl_ocp_write(tp, 0xe040,
			      (rtl_ocp_read(tp, 0xe040) & ~0x3));
		/* turn off EEE+ */
		rtl_ocp_write(tp, 0xe080,
			      (rtl_ocp_read(tp, 0xe080) & ~0x2));
		#elif defined(CONFIG_ARCH_RTD139x) || \
			defined(CONFIG_ARCH_RTD16xx) || \
			defined(CONFIG_ARCH_RTD13xx) || \
			defined(CONFIG_ARCH_RTD16XXB)
		r8169soc_eee_init(tp, false);
		#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x |
			* CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD13xx |
			* CONFIG_ARCH_RTD16XXB
			*/
	}

	rtl8169_hw_reset(tp);

	rtl_pll_power_down(tp);
}

#ifdef CONFIG_PM

static int rtl8169_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl8169_private *tp = netdev_priv(ndev);
	void __iomem *ioaddr = tp->mmio_addr;
	#if defined(CONFIG_ARCH_RTD129x) || defined(CONFIG_ARCH_RTD139x) || \
		defined(CONFIG_ARCH_RTD16xx) || defined(CONFIG_ARCH_RTD13xx) || \
		defined(CONFIG_ARCH_RTD16XXB)
	u32 val;
	#endif

	pr_info("[RTK_ETN] Enter %s\n", __func__);

	rtl8169_net_suspend(ndev);

	/* turn off LED, and current solution is switch pad to GPIO input */
	if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
		#if defined(CONFIG_ARCH_RTD129x)
		val = readl(tp->mmio_clkaddr + 0x310) & ~0x3C000000;
		writel(val, (tp->mmio_clkaddr + 0x310));
		#elif defined(CONFIG_ARCH_RTD139x)
		val = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD1) & ~0x000000F0;
		writel(val, (tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD1));
		#elif defined(CONFIG_ARCH_RTD16xx)
		val = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD0) & ~0x3C000000;
		writel(val, (tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD0));
		val = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD2) & ~0x38000000;
		writel(val, (tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD2));
		#elif defined(CONFIG_ARCH_RTD13xx)
		val = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD2) & ~0x000003F0;
		writel(val, (tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD2));
		#elif defined(CONFIG_ARCH_RTD16XXB)
		val = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD2) & ~0x60003F00;
		writel(val, (tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD2));

		#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x |
			* CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD13xx |
			* CONFIG_ARCH_RTD16XXB
			*/
	}

	if (RTK_PM_STATE == PM_SUSPEND_STANDBY) {
		/* For idle mode */
		pr_info("[RTK_ETN] %s Idle mode\n", __func__);

		/* stop PLL for power saving */
		RTL_W16(CPlusCmd, RTL_R16(CPlusCmd) | BIT(2));
	} else {
		/* For suspend mode */
		pr_info("[RTK_ETN] %s Suspend mode\n", __func__);
	}

	pr_info("[RTK_ETN] Exit %s\n", __func__);

	return 0;
}

static void __rtl8169_resume(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	#if defined(CONFIG_ARCH_RTD129x) || defined(CONFIG_ARCH_RTD139x) || \
		defined(CONFIG_ARCH_RTD16xx) || defined(CONFIG_ARCH_RTD13xx) || \
		defined(CONFIG_ARCH_RTD16XXB)
	u32 val;
	#endif

	netif_device_attach(dev);

	/* turn on LED */
	if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
		#if defined(CONFIG_ARCH_RTD129x)
		val = readl(tp->mmio_clkaddr + 0x310) & ~0x3C000000;
		writel(val | 0x14000000, (tp->mmio_clkaddr + 0x310));
		#elif defined(CONFIG_ARCH_RTD139x)
		val = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD1) & ~0x000000F0;
		writel(val | 0x50, (tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD1));
		#elif defined(CONFIG_ARCH_RTD16xx)
		val = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD0) & ~0x3C000000;
		writel(val | 0x14000000, (tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD0));
		val = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD2) & ~0x38000000;
		writel(val | 0x18000000, (tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD2));
		#elif defined(CONFIG_ARCH_RTD13xx)
		val = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD2) & ~0x000003F0;
		writel(val | 0x00000090, (tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD2));
		#elif defined(CONFIG_ARCH_RTD16XXB)
		val = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD2) & ~0x60003F00;
		writel(val | 0x20000900, (tp->mmio_pinmuxaddr + ISO_TESTMUX_MUXPAD2));
		#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x |
			* CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD13xx |
			* CONFIG_ARCH_RTD16XXB
			*/
	}

	rtl_pll_power_up(tp);
	rtl_rar_set(tp, tp->dev->dev_addr);

	rtl_lock_work(tp);
	napi_enable(&tp->napi);
	set_bit(RTL_FLAG_TASK_ENABLED, tp->wk.flags);
	rtl_unlock_work(tp);

	rtl_schedule_task(tp, RTL_FLAG_TASK_RESET_PENDING);
}

static int rtl8169_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl8169_private *tp = netdev_priv(ndev);
	void __iomem *ioaddr = tp->mmio_addr;

	pr_info("[RTK_ETN] Enter %s\n", __func__);

	if (RTK_PM_STATE == PM_SUSPEND_STANDBY) {
		/* For idle mode */
		pr_info("[RTK_ETN] %s Idle mode\n", __func__);

		/* enable PLL from power saving */
		RTL_W16(CPlusCmd, RTL_R16(CPlusCmd) & ~BIT(2));
	} else {
		/* For suspend mode */
		pr_info("[RTK_ETN] %s Suspend mode\n", __func__);
	}

	if (netif_running(ndev))
		__rtl8169_resume(ndev);

	/* enable EEE according to tp->eee_enable */
	if (tp->eee_enable) {
		#if defined(CONFIG_ARCH_RTD129x)
		rtl_mmd_write(tp, 0x7, 0x3c, 0x6);

		/* turn on EEE */
		rtl_ocp_write(tp, 0xe040,
			      (rtl_ocp_read(tp, 0xe040) | 0x3));
		/* turn on EEE+ */
		rtl_ocp_write(tp, 0xe080,
			      (rtl_ocp_read(tp, 0xe080) | 0x2));
		#elif defined(CONFIG_ARCH_RTD139x) || \
			defined(CONFIG_ARCH_RTD16xx) || \
			defined(CONFIG_ARCH_RTD13xx) || \
			defined(CONFIG_ARCH_RTD16XXB)
		r8169soc_eee_init(tp, true);
		#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x |
			* CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD13xx |
			* CONFIG_ARCH_RTD16XXB
			*/
	}

	rtl8169_init_phy(ndev, tp);

	pr_info("[RTK_ETN] Exit %s\n", __func__);

	return 0;
}

static __maybe_unused int rtl8169_runtime_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl8169_private *tp = netdev_priv(ndev);

	if (!tp->TxDescArray)
		return 0;

	rtl_lock_work(tp);
	tp->saved_wolopts = __rtl8169_get_wol(tp);
	__rtl8169_set_wol(tp, WAKE_ANY);
	rtl_unlock_work(tp);

	rtl8169_net_suspend(ndev);

	return 0;
}

static __maybe_unused int rtl8169_runtime_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl8169_private *tp = netdev_priv(ndev);

	if (!tp->TxDescArray)
		return 0;

	rtl_lock_work(tp);
	__rtl8169_set_wol(tp, tp->saved_wolopts);
	tp->saved_wolopts = 0;
	rtl_unlock_work(tp);

	rtl8169_init_phy(ndev, tp);

	__rtl8169_resume(ndev);

	return 0;
}

static __maybe_unused int rtl8169_runtime_idle(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl8169_private *tp = netdev_priv(ndev);

	return tp->TxDescArray ? -EBUSY : 0;
}

static const struct dev_pm_ops rtl8169_pm_ops = {
	.suspend		= rtl8169_suspend,
	.resume			= rtl8169_resume,
	.freeze			= rtl8169_suspend,
	.thaw			= rtl8169_resume,
	.poweroff		= rtl8169_suspend,
	.restore		= rtl8169_resume,
	.runtime_suspend	= rtl8169_runtime_suspend,
	.runtime_resume		= rtl8169_runtime_resume,
	.runtime_idle		= rtl8169_runtime_idle,
};

#define RTL8169_PM_OPS	(&rtl8169_pm_ops)

#else /* !CONFIG_PM */

#define RTL8169_PM_OPS	NULL

#endif /* !CONFIG_PM */

static void rtl_shutdown(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct rtl8169_private *tp = netdev_priv(dev);

	rtl8169_net_suspend(dev);

	/* Restore original MAC address */
	rtl_rar_set(tp, dev->perm_addr);

	rtl8169_hw_reset(tp);

	if (system_state == SYSTEM_POWER_OFF) {
		if (__rtl8169_get_wol(tp) & WAKE_ANY)
			rtl_wol_suspend_quirk(tp);
	}
}

static int rtl_remove_one(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct rtl8169_private *tp = netdev_priv(dev);
	int i;

#ifdef RTL_PROC
	do {
		if (!tp->dir_dev)
			break;

		remove_proc_entry("wol_enable", tp->dir_dev);
		remove_proc_entry("phy_reinit", tp->dir_dev);
		remove_proc_entry("eee", tp->dir_dev);
		remove_proc_entry("test", tp->dir_dev);
		remove_proc_entry("driver_var", tp->dir_dev);
		remove_proc_entry("eth_phy", tp->dir_dev);
		remove_proc_entry("ext_regs", tp->dir_dev);
		remove_proc_entry("registers", tp->dir_dev);
		remove_proc_entry("tally", tp->dir_dev);
		remove_proc_entry("wpd_event", tp->dir_dev);
		remove_proc_entry("wol_packet", tp->dir_dev);

		if (!rtw_proc)
			break;

		remove_proc_entry(MODULENAME, rtw_proc);
		remove_proc_entry("eth0", init_net.proc_net);

		rtw_proc = NULL;

	} while (0);
#endif

	cancel_work_sync(&tp->wk.work);

	netif_napi_del(&tp->napi);

	unregister_netdev(dev);

	/* restore original MAC address */
	rtl_rar_set(tp, dev->perm_addr);

	rtl8169_release_board(pdev, dev, tp->mmio_addr);
	platform_set_drvdata(pdev, NULL);

	if (tp->kthr) {
		kthread_stop(tp->kthr);
		tp->kthr = NULL;
	}

	for (i = 0; i < tp->phy_irq_num; i++)
		free_irq(tp->phy_irq[i], tp);
	return 0;
}

static const struct net_device_ops rtl_netdev_ops = {
	.ndo_open		= rtl_open,
	.ndo_stop		= rtl8169_close,
	.ndo_get_stats64	= rtl8169_get_stats64,
	.ndo_start_xmit		= rtl8169_start_xmit,
	.ndo_tx_timeout		= rtl8169_tx_timeout,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= rtl8169_change_mtu,
	.ndo_fix_features	= rtl8169_fix_features,
	.ndo_set_features	= rtl8169_set_features,
	.ndo_set_mac_address	= rtl_set_mac_address,
	.ndo_do_ioctl		= rtl8169_ioctl,
	.ndo_set_rx_mode	= rtl_set_rx_mode,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= rtl8169_netpoll,
#endif

};

static struct rtl_cfg_info {
	void (*hw_start)(struct net_device *dev);
	unsigned int region;
	unsigned int align;
	u16 event_slow;
	unsigned int features;
	u8 default_ver;
} rtl_cfg_infos = {
	.hw_start	= rtl_hw_start_8168,
	.region		= 2,
	.align		= 8,
	.event_slow	= SYSErr | LinkChg | RxOverflow,
	.features	= RTL_FEATURE_GMII | RTL_FEATURE_MSI,
	.default_ver	= RTL_GIGA_MAC_VER_42,
};

DECLARE_RTL_COND(rtl_link_list_ready_cond)
{
	void __iomem *ioaddr = tp->mmio_addr;

	return RTL_R8(MCU) & LINK_LIST_RDY;
}

DECLARE_RTL_COND(rtl_rxtx_empty_cond)
{
	void __iomem *ioaddr = tp->mmio_addr;

	return (RTL_R8(MCU) & RXTX_EMPTY) == RXTX_EMPTY;
}

static void rtl_hw_init_8168g(struct rtl8169_private *tp)
{
	void __iomem *ioaddr = tp->mmio_addr;
	u32 data;

	tp->ocp_base = OCP_STD_PHY_BASE;

	RTL_W32(MISC, RTL_R32(MISC) | RXDV_GATED_EN);

	if (!rtl_udelay_loop_wait_high(tp, &rtl_txcfg_empty_cond, 100, 42))
		return;

	if (!rtl_udelay_loop_wait_high(tp, &rtl_rxtx_empty_cond, 100, 42))
		return;

	RTL_W8(ChipCmd, RTL_R8(ChipCmd) & ~(CmdTxEnb | CmdRxEnb));
	usleep_range(1000, 1100);
	RTL_W8(MCU, RTL_R8(MCU) & ~NOW_IS_OOB);

	data = rtl_ocp_read(tp, 0xe8de);
	data &= ~BIT(14);
	rtl_ocp_write(tp, 0xe8de, data);

	if (!rtl_udelay_loop_wait_high(tp, &rtl_link_list_ready_cond, 100, 42))
		return;

	data = rtl_ocp_read(tp, 0xe8de);
	data |= BIT(15);
	rtl_ocp_write(tp, 0xe8de, data);

	if (!rtl_udelay_loop_wait_high(tp, &rtl_link_list_ready_cond, 100, 42))
		return;
}

static void rtl_hw_initialize(struct rtl8169_private *tp)
{
	rtl_hw_init_8168g(tp);
}

#ifdef RTL_PROC
static int wol_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	pr_info("WoL setting:\n");
	pr_info("\tBIT 0:\t WoL enable\n");
	pr_info("\tBIT 1:\t CRC match\n");
	pr_info("\tBIT 2:\t WPD\n");
	pr_info("wol_enable = 0x%x\n", tp->wol_enable);
	seq_printf(m, "%d\n", tp->wol_enable);
	return 0;
}

static ssize_t wol_write_proc(struct file *file, const char __user *buffer,
			      size_t count, loff_t *pos)
{
	struct net_device *dev = (struct net_device *)
		((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	char tmp[32];
	u32 val = 0;
	u32 len = 0;
	int ret;

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		if (len)
			tmp[len - 1] = '\0';
		ret = kstrtou32(tmp, 0, &val);
		if (ret) {
			pr_err("invalid WoL setting [%s], ret = %d\n",
			       tmp, ret);
			return count;
		}
		tp->wol_enable = val;
		pr_info("set wol_enable = %x\n", tp->wol_enable);
	}
	return count;
}

static int wol_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, wol_read_proc, dev);
}

static const struct file_operations wol_proc_fops = {
	.open		= wol_proc_open,
	.read		= seq_read,
	.write		= wol_write_proc,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int phy_reinit_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	u32 isr;
	u32 por;

	isr = readl(tp->mmio_clkaddr + ISO_UMSK_ISR);
	seq_printf(m, "ISO_UMSK_ISR = 0x%08x\n", isr);
	por = readl(tp->mmio_clkaddr + ISO_POR_DATAI);
	seq_printf(m, "ISO_POR_DATAI = 0x%08x\n", por);

	seq_puts(m, "\n\nUsage: echo 1 > phy_reinit\n");

	return 0;
}

static ssize_t phy_reinit_write_proc(struct file *file,
				     const char __user *buffer,
				     size_t count, loff_t *pos)
{
	struct net_device *dev = (struct net_device *)
		((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	char tmp[32];
	u32 val = 0;
	u32 len = 0;
	int ret;

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		if (len)
			tmp[len - 1] = '\0';
		ret = kstrtou32(tmp, 0, &val);
		if (ret || val == 0) {
			pr_err("invalid phy_reinit [%s], ret = %d\n",
			       tmp, ret);
			return count;
		}
		pr_info("phy_reinit = %x\n", val);

		if (!test_bit(RTL_FLAG_TASK_ENABLED, tp->wk.flags))
			rtl_phy_reinit(tp);
		else if (atomic_inc_return(&tp->phy_reinit_flag) == 1)
			rtl_schedule_task(tp, RTL_FLAG_TASK_PHY_PENDING);
		else
			atomic_dec(&tp->phy_reinit_flag);
	}
	return count;
}

static int phy_reinit_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, phy_reinit_read_proc, dev);
}

static const struct file_operations phy_reinit_proc_fops = {
	.open		= phy_reinit_proc_open,
	.read		= seq_read,
	.write		= phy_reinit_write_proc,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int eee_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	unsigned short e040, e080;

	rtl_lock_work(tp);
	e040 = rtl_ocp_read(tp, 0xe040);	/* EEE */
	e080 = rtl_ocp_read(tp, 0xe080);	/* EEE+ */
	seq_printf(m, "%s: eee = %d, OCP 0xe040 = 0x%x, OCP 0xe080 = 0x%x\n",
		   dev->name, tp->eee_enable, e040, e080);
	r8169_display_eee_info(dev, m, tp);
	rtl_unlock_work(tp);

	return 0;
}

static ssize_t eee_write_proc(struct file *file, const char __user *buffer,
			      size_t count, loff_t *pos)
{
	char tmp[32];
	u32 val = 0;
	u32 len = 0;
	int ret;
	struct net_device *dev = (struct net_device *)
		((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		if (len)
			tmp[len - 1] = '\0';
		ret = kstrtou32(tmp, 0, &val);
		if (ret) {
			pr_err("invalid EEE setting [%s], ret = %d\n",
			       tmp, ret);
			return count;
		}
		pr_err("write %s eee = %x\n", dev->name, val);
	} else {
		return count;
	}

	rtl_lock_work(tp);
	if (val > 0 && !tp->eee_enable) {
		tp->eee_enable = true;

		/* power down PHY */
		rtl_phy_write(tp, 0, MII_BMCR,
			      rtl_phy_read(tp, 0, MII_BMCR) | BMCR_PDOWN);

		#if defined(CONFIG_ARCH_RTD129x)
		rtl_mmd_write(tp, 0x7, 0x3c, 0x6);

		/* turn on EEE */
		rtl_ocp_write(tp, 0xe040,
			      (rtl_ocp_read(tp, 0xe040) | 0x3));
		/* turn on EEE+ */
		rtl_ocp_write(tp, 0xe080,
			      (rtl_ocp_read(tp, 0xe080) | 0x2));
		#elif defined(CONFIG_ARCH_RTD139x) || \
			defined(CONFIG_ARCH_RTD16xx) || \
			defined(CONFIG_ARCH_RTD13xx) || \
			defined(CONFIG_ARCH_RTD16XXB)
		r8169soc_eee_init(tp, tp->eee_enable);
		#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x |
			* CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD13xx |
			* CONFIG_ARCH_RTD16XXB
			*/

		mdelay(100); /* wait PHY ready */
		rtl8169_phy_reset(dev, tp);
	} else if (val == 0 && tp->eee_enable) {
		tp->eee_enable = false;

		/* power down PHY */
		rtl_phy_write(tp, 0, MII_BMCR,
			      rtl_phy_read(tp, 0, MII_BMCR) | BMCR_PDOWN);

		#if defined(CONFIG_ARCH_RTD129x)
		rtl_mmd_write(tp, 0x7, 0x3c, 0);

		/* turn off EEE */
		rtl_ocp_write(tp, 0xe040,
			      (rtl_ocp_read(tp, 0xe040) & ~0x3));
		/* turn off EEE+ */
		rtl_ocp_write(tp, 0xe080,
			      (rtl_ocp_read(tp, 0xe080) & ~0x2));
		#elif defined(CONFIG_ARCH_RTD139x) || \
			defined(CONFIG_ARCH_RTD16xx) || \
			defined(CONFIG_ARCH_RTD13xx) || \
			defined(CONFIG_ARCH_RTD16XXB)
		r8169soc_eee_init(tp, tp->eee_enable);
		#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x |
			* CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD13xx |
			* CONFIG_ARCH_RTD16XXB
			*/

		mdelay(100); /* wait PHY ready */
		rtl8169_phy_reset(dev, tp);
	}
	rtl_unlock_work(tp);

	return count;
}

/* seq_file wrappers for procfile show routines. */
static int eee_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, eee_read_proc, dev);
}

static const struct file_operations eee_proc_fops = {
	.open		= eee_proc_open,
	.read		= seq_read,
	.write		= eee_write_proc,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void r8169soc_dump_regs(struct seq_file *m, struct rtl8169_private *tp)
{
	u32 addr;
	u32 val;
	u32 i;

	seq_printf(m, "ISO_UMSK_ISR\t[0x98007004] = %08x\n",
		   readl(tp->mmio_clkaddr + ISO_UMSK_ISR));
	seq_printf(m, "ISO_PWRCUT_ETN\t[0x9800705c] = %08x\n",
		   readl(tp->mmio_clkaddr + ISO_PWRCUT_ETN));
	seq_printf(m, "ISO_ETN_TESTIO\t[0x98007060] = %08x\n",
		   readl(tp->mmio_clkaddr + ISO_ETN_TESTIO));
	seq_printf(m, "ETN_RESET_CTRL\t[0x98007088] = %08x\n",
		   readl(tp->mmio_clkaddr + ISO_SOFT_RESET));
	seq_printf(m, "ETN_CLK_CTRL\t[0x9800708c] = %08x\n",
		   readl(tp->mmio_clkaddr + ISO_CLOCK_ENABLE));

	if (tp->output_mode != OUTPUT_EMBEDDED_PHY) {
		#if defined(CONFIG_ARCH_RTD139x)
		seq_printf(m, "SDS_REG02\t[0x981c8008] = %08x\n",
			   readl(tp->mmio_sdsaddr + SDS_REG02));
		seq_printf(m, "SDS_REG28\t[0x981c8070] = %08x\n",
			   readl(tp->mmio_sdsaddr + SDS_REG28));
		seq_printf(m, "SDS_REG29\t[0x981c8074] = %08x\n",
			   readl(tp->mmio_sdsaddr + SDS_REG29));
		seq_printf(m, "SDS_MISC\t\t[0x981c9804] = %08x\n",
			   readl(tp->mmio_sdsaddr + SDS_MISC));
		seq_printf(m, "SDS_LINK\t\t[0x981c9810] = %08x\n",
			   readl(tp->mmio_sdsaddr + SDS_LINK));
		#elif defined(CONFIG_ARCH_RTD16xx)
		seq_printf(m, "SDS_REG02\t[0x981c8008] = %08x\n",
			   readl(tp->mmio_sdsaddr + SDS_REG02));
		seq_printf(m, "SDS_MISC\t\t[0x981c9804] = %08x\n",
			   readl(tp->mmio_sdsaddr + SDS_MISC));
		seq_printf(m, "SDS_LINK\t\t[0x981c980c] = %08x\n",
			   readl(tp->mmio_sdsaddr + SDS_LINK));
		seq_printf(m, "SDS_DEBUG\t\t[0x981c9810] = %08x\n",
			   readl(tp->mmio_sdsaddr + SDS_DEBUG));
		#endif /* CONFIG_ARCH_RTD139x | CONFIG_ARCH_RTD16xx */
	}

	seq_puts(m, "ETN MAC regs:\n");
	for (i = 0; i < 256; i += 4) {
		addr = 0x98016000 + i;
		val = readl(tp->mmio_addr + i);
		seq_printf(m, "[%08x] = %08x\n", addr, val);
	}
}

static int test_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	rtl_lock_work(tp);
	r8169soc_dump_regs(m, tp);
	rtl_unlock_work(tp);

	return 0;
}

static ssize_t test_write_proc(struct file *file, const char __user *buffer,
			       size_t count, loff_t *pos)
{
	char tmp[32];
	u32 val = 0;
	u32 len = 0;
	int ret;
	struct net_device *dev = (struct net_device *)
			((struct seq_file *)file->private_data)->private;

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		if (len)
			tmp[len - 1] = '\0';
		ret = kstrtou32(tmp, 0, &val);
		if (ret) {
			pr_err("invalid test case [%s], ret = %d\n",
			       tmp, ret);
			return count;
		}
		pr_err("%s test case 0x%x\n", dev->name, val);
	}

	return count;
}

/* seq_file wrappers for procfile show routines. */
static int test_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, test_read_proc, dev);
}

static const struct file_operations test_proc_fops = {
	.open		= test_proc_open,
	.read		= seq_read,
	.write		= test_write_proc,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int driver_var_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	int i;

	seq_puts(m, "\nDump Driver Variable\n");

	rtl_lock_work(tp);
	seq_puts(m, "Variable\tValue\n----------\t-----\n");
	seq_printf(m, "MODULENAME\t%s\n", MODULENAME);
	seq_printf(m, "mac version\t%d\n", tp->mac_version);
	seq_printf(m, "chipset_name\t%s\n", rtl_chip_infos.name);
	seq_printf(m, "driver version\t%s\n", RTL8169_VERSION);
	seq_printf(m, "txd version\t%d\n", tp->txd_version);
	seq_printf(m, "msg_enable\t0x%x\n", tp->msg_enable);
	seq_printf(m, "mtu\t\t%d\n", dev->mtu);
	seq_printf(m, "NUM_RX_DESC\t0x%x\n", NUM_RX_DESC);
	seq_printf(m, "cur_rx\t\t0x%x\n", tp->cur_rx);
	#if defined(CONFIG_RTL_RX_NO_COPY)
	seq_printf(m, "dirty_rx\t0x%x\n", tp->dirty_rx);
	#endif /* CONFIG_RTL_RX_NO_COPY */
	seq_printf(m, "NUM_TX_DESC\t0x%x\n", NUM_TX_DESC);
	seq_printf(m, "cur_tx\t\t0x%x\n", tp->cur_tx);
	seq_printf(m, "dirty_tx\t0x%x\n", tp->dirty_tx);
	seq_printf(m, "rx_buf_sz\t%d\n", rx_buf_sz);
	seq_printf(m, "cp_cmd\t\t0x%x\n", tp->cp_cmd);
	seq_printf(m, "event_slow\t0x%x\n", tp->event_slow);
	seq_printf(m, "wol_enable\t0x%x\n", tp->wol_enable);
	seq_printf(m, "saved_wolopts\t0x%x\n", tp->saved_wolopts);
	seq_printf(m, "opts1_mask\t0x%x\n", tp->opts1_mask);
	seq_printf(m, "wol_crc_cnt\t%d\n", tp->wol_crc_cnt);
	seq_printf(m, "led_cfg\t\t0x%x\n", tp->led_cfg);
	seq_printf(m, "features\t0x%x\n", tp->features);
	seq_printf(m, "cfg features\t0x%x\n", rtl_cfg_infos.features);
	seq_printf(m, "eee_enable\t%d\n", tp->eee_enable);
	seq_printf(m, "acp_enable\t%d\n", tp->acp_enable);
	seq_printf(m, "ext_phy\t\t%d\n", tp->ext_phy);
	seq_printf(m, "output_mode\t%d\n", tp->output_mode);
	#if defined(CONFIG_ARCH_RTD129x)
	seq_printf(m, "rgmii_voltage\t%d\n", tp->rgmii_voltage);
	seq_printf(m, "rgmii_tx_delay\t%d\n", tp->rgmii_tx_delay);
	seq_printf(m, "rgmii_rx_delay\t%d\n", tp->rgmii_rx_delay);
	#elif defined(CONFIG_ARCH_RTD139x)
	seq_printf(m, "bypass_enable\t%d\n", tp->bypass_enable);
	#elif defined(CONFIG_ARCH_RTD16xx)
	seq_printf(m, "sgmii_swing\t%d\n", tp->sgmii_swing);
	seq_printf(m, "bypass_enable\t%d\n", tp->bypass_enable);
	#elif defined(CONFIG_ARCH_RTD13xx)
	seq_printf(m, "voltage \t%d\n", tp->voltage);
	seq_printf(m, "tx_delay\t%d\n", tp->tx_delay);
	seq_printf(m, "rx_delay\t%d\n", tp->rx_delay);
	seq_printf(m, "bypass_enable\t%d\n", tp->bypass_enable);
	#elif defined(CONFIG_ARCH_RTD16XXB)
	seq_printf(m, "bypass_enable\t%d\n", tp->bypass_enable);
	#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x |
		* CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD13xx |
		* CONFIG_ARCH_RTD16XXB
		*/
	seq_printf(m, "ETN IRQ\t\t%d\n", dev->irq);
	seq_printf(m, "phy_irq_num\t%d\n", tp->phy_irq_num);
	for (i = 0; i < tp->phy_irq_num; i++)
		seq_printf(m, "GPHY IRQ\t%d maps to ISR bit %d\n",
			   tp->phy_irq[i], tp->phy_irq_map[i]);
	seq_printf(m, "perm_addr\t%pM\n", dev->perm_addr);
	seq_printf(m, "dev_addr\t%pM\n", dev->dev_addr);
	rtl_unlock_work(tp);

	seq_putc(m, '\n');
	return 0;
}

static int driver_var_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, driver_var_read_proc, dev);
}

static const struct file_operations driver_var_proc_fops = {
	.open		= driver_var_proc_open,
	.read		= seq_read,
	.write		= NULL,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int tally_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	struct rtl8169_counters *counters = &tp->counters;

	rtl8169_update_counters(dev);

	seq_puts(m, "\nDump Tally Counter\n");
	seq_puts(m, "Statistics\tValue\n----------\t-----\n");
	seq_printf(m, "tx_packets\t%lld\n", le64_to_cpu(counters->tx_packets));
	seq_printf(m, "rx_packets\t%lld\n", le64_to_cpu(counters->rx_packets));
	seq_printf(m, "tx_errors\t%lld\n", le64_to_cpu(counters->tx_errors));
	seq_printf(m, "rx_errors\t%d\n", le32_to_cpu(counters->rx_errors));
	seq_printf(m, "rx_missed\t%d\n", le16_to_cpu(counters->rx_missed));
	seq_printf(m, "align_errors\t%d\n", le16_to_cpu(counters->align_errors));
	seq_printf(m, "tx_one_collision\t%d\n", le32_to_cpu(counters->tx_one_collision));
	seq_printf(m, "tx_multi_collision\t%d\n", le32_to_cpu(counters->tx_multi_collision));
	seq_printf(m, "rx_unicast\t%lld\n", le64_to_cpu(counters->rx_unicast));
	seq_printf(m, "rx_broadcast\t%lld\n", le64_to_cpu(counters->rx_broadcast));
	seq_printf(m, "rx_multicast\t%d\n", le32_to_cpu(counters->rx_multicast));
	seq_printf(m, "tx_aborted\t%d\n", le16_to_cpu(counters->tx_aborted));
	seq_printf(m, "tx_underrun\t%d\n", le16_to_cpu(counters->tx_underrun));

	seq_putc(m, '\n');
	return 0;
}

static int tally_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, tally_read_proc, dev);
}

static const struct file_operations tally_proc_fops = {
	.open		= tally_proc_open,
	.read		= seq_read,
	.write		= NULL,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int registers_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	int i, n, max = 256;
	u8 byte_rd;
	struct rtl8169_private *tp = netdev_priv(dev);
	void __iomem *ioaddr = tp->mmio_addr;

	seq_puts(m, "\nDump MAC Registers\n");
	seq_puts(m, "Offset\tValue\n------\t-----\n");

	rtl_lock_work(tp);
	for (n = 0; n < max;) {
		seq_printf(m, "\n0x%02x:\t", n);

		for (i = 0; i < 16 && n < max; i++, n++) {
			byte_rd = readb(ioaddr + n);
			seq_printf(m, "%02x ", byte_rd);
		}
	}
	rtl_unlock_work(tp);

	seq_putc(m, '\n');
	return 0;
}

static int registers_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, registers_read_proc, dev);
}

static const struct file_operations registers_proc_fops = {
	.open		= registers_proc_open,
	.read		= seq_read,
	.write		= NULL,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int eth_phy_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	int i, n, max = 16;
	u16 word_rd;
	struct rtl8169_private *tp = netdev_priv(dev);

	seq_puts(m, "\nDump Ethernet PHY\n");
	seq_puts(m, "\nOffset\tValue\n------\t-----\n ");

	rtl_lock_work(tp);
	seq_puts(m, "\n####################page 0##################\n ");
	for (n = 0; n < max;) {
		seq_printf(m, "\n0x%02x:\t", n);

		for (i = 0; i < 8 && n < max; i++, n++) {
			word_rd = rtl_phy_read(tp, 0, n);
			seq_printf(m, "%04x ", word_rd);
		}
	}
	rtl_unlock_work(tp);

	seq_putc(m, '\n');
	return 0;
}

static int eth_phy_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, eth_phy_read_proc, dev);
}

static const struct file_operations eth_phy_proc_fops = {
	.open		= eth_phy_proc_open,
	.read		= seq_read,
	.write		= NULL,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int ext_regs_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	int i, n, max = 256;
	u32 dword_rd;
	struct rtl8169_private *tp = netdev_priv(dev);

	seq_puts(m, "\nDump Extended Registers\n");
	seq_puts(m, "\nOffset\tValue\n------\t-----\n ");

	rtl_lock_work(tp);
	for (n = 0; n < max;) {
		seq_printf(m, "\n0x%02x:\t", n);

		for (i = 0; i < 4 && n < max; i++, n += 4) {
			dword_rd = rtl_eri_read(tp, n, ERIAR_EXGMAC);
			seq_printf(m, "%08x ", dword_rd);
		}
	}
	rtl_unlock_work(tp);

	seq_putc(m, '\n');
	return 0;
}

static int ext_regs_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, ext_regs_read_proc, dev);
}

static const struct file_operations ext_regs_proc_fops = {
	.open		= ext_regs_proc_open,
	.read		= seq_read,
	.write		= NULL,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int wpd_evt_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	u32 tmp;

	rtl_lock_work(tp);
	/* check if wol_own and wpd_en are both set */
	tmp = rtl_ocp_read(tp, 0xC0C2);
	if ((tmp & BIT(4)) == 0 || (tmp & BIT(0)) == 0) {
		seq_puts(m, "\nNo WPD event\n");
		rtl_unlock_work(tp);
		return 0;
	}

	seq_puts(m, "\nWPD event:\n");
	tmp = rtl_ocp_read(tp, 0xD23A) & 0x0F01;
	seq_printf(m, "Type (0: CRC match,  1: magic pkt) = %d\n",
		   tmp & 0x1);
	if ((tmp & 0x1) == 0)
		seq_printf(m, "CRC match ID = %d\n", tmp >> 8);
	seq_printf(m, "Original packet length = %d\n",
		   rtl_ocp_read(tp, 0xD23C));
	seq_printf(m, "Stored packet length = %d\n",
		   rtl_ocp_read(tp, 0xD23E));
	rtl_unlock_work(tp);

	seq_putc(m, '\n');
	return 0;
}

static int wpd_evt_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, wpd_evt_read_proc, dev);
}

static const struct file_operations wpd_evt_proc_fops = {
	.open		= wpd_evt_proc_open,
	.read		= seq_read,
	.write		= NULL,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int wol_pkt_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	int i;
	char wol_pkt[WOL_BUF_LEN];
	u32 len;
	u32 tmp;
	u16 *ptr;

	memset(wol_pkt, 0, WOL_BUF_LEN);

	rtl_lock_work(tp);
	/* check if wol_own and wpd_en are both set */
	tmp = rtl_ocp_read(tp, 0xC0C2);
	if ((tmp & BIT(4)) == 0 || (tmp & BIT(0)) == 0) {
		rtl_unlock_work(tp);
		return 0;
	}

	/* read 128-byte packet buffer */
	for (i = 0; i < 128; i += 2) {
		ptr = (u16 *)&wol_pkt[i];
		*ptr = rtl_ocp_read(tp, 0xD240 + i);
	}

	/* get stored packet length */
	len = rtl_ocp_read(tp, 0xD23E);
	for (i = 0; i < len; i++) {
		if ((i % 16) == 0)
			seq_puts(m, "\n");
		else if ((i % 8) == 0)
			seq_puts(m, "  ");
		seq_printf(m, "%02x ", wol_pkt[i]);
	}
	rtl_unlock_work(tp);

	seq_putc(m, '\n');
	return 0;
}

static int wol_pkt_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, wol_pkt_read_proc, dev);
}

static const struct file_operations wol_pkt_proc_fops = {
	.open		= wol_pkt_proc_open,
	.read		= seq_read,
	.write		= NULL,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#endif

static __maybe_unused
void *r8169soc_read_otp(struct rtl8169_private *tp, const char *name)
{
	struct device *dev = &tp->pdev->dev;
	struct device_node *np = dev->of_node;
	struct nvmem_cell *cell;
	unsigned char *buf;
	size_t buf_size;

	cell = of_nvmem_cell_get(np, name);
	if (IS_ERR(cell)) {
		dev_err(dev, "failed to get nvmem cell %s: %ld\n",
			name, PTR_ERR(cell));
		return ERR_CAST(cell);
	}

	buf = nvmem_cell_read(cell, &buf_size);
	if (IS_ERR(buf))
		dev_err(dev, "failed to read nvmem cell %s: %ld\n",
			name, PTR_ERR(buf));
	nvmem_cell_put(cell);
	return buf;
}

#if defined(CONFIG_ARCH_RTD139x)
static void r8169soc_reset_phy_gmac(struct rtl8169_private *tp)
{
	u32 tmp;
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");
	struct clk *clk_en_sds = NULL;
	struct reset_control *rstc_sds_reg = NULL;
	struct reset_control *rstc_sds = NULL;
	struct reset_control *rstc_pcie0_power = NULL;
	struct reset_control *rstc_pcie0_phy = NULL;
	struct reset_control *rstc_pcie0_sgmii_mdio = NULL;
	struct reset_control *rstc_pcie0_phy_mdio = NULL;

	clk_prepare_enable(clk_etn_sys);
	clk_prepare_enable(clk_etn_250m);

	/* reg_0x9800708c[12:11] = 00 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_disable_unprepare(clk_etn_sys);
	clk_disable_unprepare(clk_etn_250m);

	/* reg_0x98007088[10:9] = 00 */
	/* ISO spec, rstn_gphy & rstn_gmac */
	reset_control_assert(rstc_gphy);
	reset_control_assert(rstc_gmac);

	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	tmp = readl(tp->mmio_clkaddr + ISO_ETN_TESTIO);
	tmp |= BIT(1);
	writel(tmp, tp->mmio_clkaddr + ISO_ETN_TESTIO);

	/* reg_0x9800705c = 0x00616703 */
	/* ISO spec, default value */
	writel(0x00616703, tp->mmio_clkaddr + ISO_PWRCUT_ETN);

	/* RESET for SGMII if needed */
	if (tp->output_mode != OUTPUT_EMBEDDED_PHY) {
		clk_en_sds = clk_get(&tp->pdev->dev, "sds");
		rstc_sds_reg = reset_control_get_exclusive(&tp->pdev->dev, "sds_reg");
		rstc_sds = reset_control_get_exclusive(&tp->pdev->dev, "sds");
		rstc_pcie0_power =
			reset_control_get_exclusive(&tp->pdev->dev, "pcie0_power");
		rstc_pcie0_phy =
			reset_control_get_exclusive(&tp->pdev->dev, "pcie0_phy");
		rstc_pcie0_sgmii_mdio =
			reset_control_get_exclusive(&tp->pdev->dev, "pcie0_sgmii_mdio");
		rstc_pcie0_phy_mdio =
			reset_control_get_exclusive(&tp->pdev->dev, "pcie0_phy_mdio");

		clk_prepare_enable(clk_en_sds);

		/* reg_0x9800000c[7] = 0 */
		/* CRT spec, clk_en_sds */
		clk_disable_unprepare(clk_en_sds);

		/* reg_0x98000000[4:3] = 00 */
		/* CRT spec, rstn_sds_reg & rstn_sds */
		reset_control_assert(rstc_sds);
		reset_control_assert(rstc_sds_reg);

		/* reg_0x98000004[7] = 0 */
		/* reg_0x98000004[14] = 0 */
		/* CRT spec, rstn_pcie0_power & rstn_pcie0_phy */
		reset_control_assert(rstc_pcie0_power);
		reset_control_assert(rstc_pcie0_phy);

		/* reg_0x98000050[13] = 0 */
		/* reg_0x98000050[16] = 0 */
		/* CRT spec, rstn_pcie0_sgmii_mdio & rstn_pcie0_phy_mdio */
		reset_control_assert(rstc_pcie0_sgmii_mdio);
		reset_control_assert(rstc_pcie0_phy_mdio);

		reset_control_put(rstc_sds_reg);
		reset_control_put(rstc_sds);
		reset_control_put(rstc_pcie0_power);
		reset_control_put(rstc_pcie0_phy);
		reset_control_put(rstc_pcie0_sgmii_mdio);
		reset_control_put(rstc_pcie0_phy_mdio);
	}

	mdelay(1);

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
}

static void r8169soc_acp_init(struct rtl8169_private *tp)
{
	u32 tmp;

	/* SBX spec, Select ETN access DDR path. */
	if (tp->acp_enable) {
		/* reg_0x9801c20c[6] = 1 */
		/* SBX spec, Mask ETN_ALL to SB3 DBUS REQ */
		tmp = readl(tp->mmio_sbxaddr + SBX_SB3_CHANNEL_REQ_MASK);
		tmp |= BIT(6);
		writel(tmp, tp->mmio_sbxaddr + SBX_SB3_CHANNEL_REQ_MASK);

		pr_info("wait all SB3 access finished...");
		tmp = 0;
		while ((BIT(6) & readl(tp->mmio_sbxaddr + SBX_SB3_CHANNEL_REQ_BUSY)) != 0) {
			mdelay(1);
			tmp += 1;
			if (tmp >= 100) {
				pr_err("\n wait SB3 access failed (wait %d ms)\n", tmp);
				break;
			}
		}
		if (tmp < 100)
			pr_info("done.\n");

		/* reg_0x9801d100[29] = 0 */
		/* SCPU wrapper spec, CLKACP division, 0 = div 2, 1 = div 3 */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);
		tmp &= ~(BIT(29));
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);

		/* reg_0x9801d124[1:0] = 00 */
		/* SCPU wrapper spec, ACP master active, 0 = active */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_INTERFACE_EN);
		tmp &= ~(BIT(1) | BIT(0));
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_INTERFACE_EN);

		/* reg_0x9801d100[30] = 1 */
		/* SCPU wrapper spec, dy_icg_en_acp */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);
		tmp |= BIT(30);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);

		/* reg_0x9801d100[21] = 1 */
		/* SCPU wrapper spec, ACP CLK enable */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);
		tmp |= BIT(21);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);

		/* reg_0x9801d100[14] = 1 */
		/* Do not apply reset to ACP port axi3 master */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);
		tmp |= BIT(14);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);

		/* reg_0x9801d800[3:0] = 0111 */
		/* reg_0x9801d800[7:4] = 0111 */
		/* reg_0x9801d800[9:8] = 00 */
		/* reg_0x9801d800[20:16] = 01100 */
		/* reg_0x9801d800[28:24] = 01110 */
		/* Configure control of ACP port */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_ACP_CTRL);
		tmp &= ~((0x1f << 24) | (0x1f << 16) | (0x1 << 9) |
			(0xf << 4) | (0xf << 0));
		tmp |= (0x0e << 24) | (0x0c << 16) | (0x1 << 9) | (0x7 << 4) |
			(0x7 << 0);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_ACP_CTRL);

		/* reg_0x9801d030[28] = 1 */
		/* SCPU wrapper spec, dy_icg_en_acp */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);
		tmp |= BIT(28);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);

		/* reg_0x9801d030[16] = 1 */
		/* SCPU wrapper spec, ACP CLK Enable for acp of scpu_chip_top */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);
		tmp |= BIT(16);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);

		/* reg_0x9801d030[0] = 1 */
		/* Do not apply reset to ACP port axi3 master */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);
		tmp |= BIT(0);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);

		/* reg_0x9801c814[17] = 1 */
		/* through ACP to SCPU_ACP */
		tmp = readl(tp->mmio_sbxaddr + SBX_ACP_MISC_CTRL);
		tmp |= BIT(17);
		writel(tmp, tp->mmio_sbxaddr + SBX_ACP_MISC_CTRL);

		pr_info("ARM ACP on\n.");
	}
}

static void r8169soc_pll_clock_init(struct rtl8169_private *tp)
{
	u32 tmp;
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");

	/* reg_0x98007004[27] = 1 */
	/* ISO spec, ETN_PHY_INTR, clear ETN interrupt for ByPassMode */
	writel(BIT(27), tp->mmio_clkaddr + ISO_UMSK_ISR);

	/* reg_0x98007088[10] = 1 */
	/* ISO spec, reset bit of gphy */
	reset_control_deassert(rstc_gphy);

	mdelay(1);	/* wait 1ms for PHY PLL stable */

	/* In Hercules, EPHY need choose the bypass mode or Non-bypass mode */
	/* Bypass mode : ETN MAC bypass efuse update flow.
	 * SW need to take this sequence.
	 */
	/* Non-Bypass mode : ETN MAC set efuse update and efuse_rdy setting */
	/* Default : Bypass mode (0x9800_7060[1] = 1'b1) */
	if (!tp->bypass_enable) {
		/* reg_0x98007060[1] = 0 */
		/* ISO spec, bypass mode disable */
		tmp = readl(tp->mmio_clkaddr + ISO_ETN_TESTIO);
		tmp &= ~(BIT(1));
		writel(tmp, tp->mmio_clkaddr + ISO_ETN_TESTIO);
	} else {
		/* reg_0x98007060[1] = 1 */
		/* ISO spec, bypass mode enable */
		/* bypass mode, SW need to handle the EPHY Status check ,
		 * EFUSE data update and EPHY fuse_rdy setting.
		 */
		tmp = readl(tp->mmio_clkaddr + ISO_ETN_TESTIO);
		tmp |= BIT(1);
		writel(tmp, tp->mmio_clkaddr + ISO_ETN_TESTIO);
	}

	/* reg_0x98007088[9] = 1 */
	/* ISO spec, reset bit of gmac */
	reset_control_deassert(rstc_gmac);

	/* reg_0x9800708c[12:11] = 11 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_prepare_enable(clk_etn_sys);
	clk_prepare_enable(clk_etn_250m);

	mdelay(10);	/* wait 10ms for GMAC uC to be stable */

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
}

/* Hercules only uses 13 bits in OTP 0x9801_72C8[12:8] and 0x9801_72D8[7:0]
 * if 0x9801_72C8[12] is 1, then
 * 0x9801_72C8[11:8] (R-calibration) is used to set PHY
 * 0x9801_72D8[7:0] is used to set idac_fine for PHY
 */
static void r8169soc_load_otp_content(struct rtl8169_private *tp)
{
	u32 otp;
	u16 tmp;
	u8 *buf;

	buf = r8169soc_read_otp(tp, "para");
	if (IS_ERR(buf))
		goto set_idac;

	otp = *buf;
	/* OTP[4] = valid flag, OTP[3:0] = content */
	if (0 != ((0x1 << 4) & otp)) {
		tmp = otp ^ R_K_DEFAULT;	/* frc_r_value_default = 0x8 */
		rtl_phy_write(tp, 0x0bc0, 20,
			      tmp | (rtl_phy_read(tp, 0x0bc0, 20) & ~(0x1f << 0)));
	}

	kfree(buf);

set_idac:

	buf = r8169soc_read_otp(tp, "idac");
	if (IS_ERR(buf))
		return;

	otp = *buf;
	tmp = otp ^ IDAC_FINE_DEFAULT;      /* IDAC_FINE_DEFAULT = 0x33 */
	rtl_phy_write(tp, 0x0bc0, 23,
		      tmp | (rtl_phy_read(tp, 0x0bc0, 23) & ~(0xff << 0)));

	kfree(buf);
}

static u32 r8169soc_serdes_init(struct rtl8169_private *tp)
{
	u32 stable_ticks;
	u32 tmp;
	struct clk *clk_en_sds = clk_get(&tp->pdev->dev, "sds");
	struct reset_control *rstc_sds_reg =
		reset_control_get_exclusive(&tp->pdev->dev, "sds_reg");
	struct reset_control *rstc_sds =
		reset_control_get_exclusive(&tp->pdev->dev, "sds");
	struct reset_control *rstc_pcie0_power =
		reset_control_get_exclusive(&tp->pdev->dev, "pcie0_power");
	struct reset_control *rstc_pcie0_phy =
		reset_control_get_exclusive(&tp->pdev->dev, "pcie0_phy");
	struct reset_control *rstc_pcie0_sgmii_mdio =
		reset_control_get_exclusive(&tp->pdev->dev, "pcie0_sgmii_mdio");
	struct reset_control *rstc_pcie0_phy_mdio =
		reset_control_get_exclusive(&tp->pdev->dev, "pcie0_phy_mdio");

	/* reg_0x98000000[4:3] = 11 */
	/* CRT spec, rstn_sds_reg & rstn_sds */
	reset_control_deassert(rstc_sds);
	reset_control_deassert(rstc_sds_reg);

	/* reg_0x98000004[7] = 1 */
	/* reg_0x98000004[14] = 1 */
	/* CRT spec, rstn_pcie0_power & rstn_pcie0_phy */
	reset_control_deassert(rstc_pcie0_power);
	reset_control_deassert(rstc_pcie0_phy);

	/* reg_0x98000050[13] = 1 */
	/* reg_0x98000050[16] = 1 */
	/* CRT spec, rstn_pcie0_sgmii_mdio & rstn_pcie0_phy_mdio */
	reset_control_deassert(rstc_pcie0_sgmii_mdio);
	reset_control_deassert(rstc_pcie0_phy_mdio);

	/* reg_0x9800000c[7] = 1 */
	/* CRT spec, clk_en_sds */
	clk_prepare_enable(clk_en_sds);

	/* reg_0x9800705c[6] = 1 */
	/* ISO spec, set PCIE channel to SGMII */
	tmp = readl(tp->mmio_clkaddr + ISO_PWRCUT_ETN);
	tmp |= BIT(6);
	writel(tmp, tp->mmio_clkaddr + ISO_PWRCUT_ETN);

	/* reg_0x9800705c[7] = 1 */
	/* ISO spec, set internal MDIO to PCIE */
	tmp = readl(tp->mmio_clkaddr + ISO_PWRCUT_ETN);
	tmp |= BIT(7);
	writel(tmp, tp->mmio_clkaddr + ISO_PWRCUT_ETN);

	/* ### Beginning of SGMII DPHY register tuning ### */
	/* reg_0x9800705c[20:16] = 00000 */
	/* ISO spec, set internal PHY addr to SERDES_DPHY_0 */
	__int_set_phy_addr(tp, SERDES_DPHY_0);

	/* # DPHY spec, DPHY reg13[8:7]=00, choose 1.25GHz */
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x0d,
		       int_mdio_read(tp, CURRENT_MDIO_PAGE, 0x0d) & ~(BIT(8) | BIT(7)));

	/* # DPHY spec, 5GHz tuning */
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x4, 0x52f5);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x5, 0xead7);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x6, 0x0010);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0xa, 0xc653);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x1, 0xa830);

	/* reg_0x9800705c[20:16] = 00001 */
	/* ISO spec, set internal PHY addr to SERDES_DPHY_1 */
	__int_set_phy_addr(tp, SERDES_DPHY_1);

	/* TX_Swing_1040mV */
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x0, 0xd4aa);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x1, 0x88aa);

	/* reg_0x9800705c[20:16] = 00001 */
	/* ISO spec, set internal PHY addr to INT_PHY_ADDR */
	__int_set_phy_addr(tp, INT_PHY_ADDR);

	tp->ext_phy = true;
	mdelay(10);     /* wait for clock stable */
	/* ext_phy == true now */

	/* reg_0x981c8070[9:0] = 0000000110 */
	/* SDS spec, set SP_CFG_SDS_DBG_SEL to get PHY_Ready */
	tmp = readl(tp->mmio_sdsaddr + SDS_REG28);
	tmp &= ~(0x3ff << 0);
	tmp |= (0x006 << 0);
	writel(tmp, tp->mmio_sdsaddr + SDS_REG28);

	tmp = 0;
	stable_ticks = 0;
	while (stable_ticks < 10) {
		/* # SDS spec, wait for phy ready */
		while ((readl(tp->mmio_sdsaddr + SDS_REG29) & BIT(14))
		       == 0) {
			stable_ticks = 0;
			if (tmp >= 100) {
				stable_ticks = 10;
				pr_err("SGMII PHY not ready in 100ms\n");
				break;
			}
		}
		stable_ticks++;
		tmp++;
		mdelay(1);
	}
	pr_info("SGMII PHY is ready in %d ms", tmp);

	/* reg_0x9800705c[4] = 1 */
	/* ISO spec, set data path to SGMII */
	tmp = readl(tp->mmio_clkaddr + ISO_PWRCUT_ETN);
	tmp |= BIT(4);
	writel(tmp, tp->mmio_clkaddr + ISO_PWRCUT_ETN);

	/* reg_0x981c8008[9:8] = 00 */
	/* # SDS spec, SP_SDS_FRC_AN, SERDES auto mode */
	tmp = readl(tp->mmio_sdsaddr + SDS_REG02);
	tmp &= ~(BIT(9) | BIT(8));
	writel(tmp, tp->mmio_sdsaddr + SDS_REG02);

	/* # SDS spec, wait for SERDES link up */
	tmp = 0;
	stable_ticks = 0;
	while (stable_ticks < 10) {
		while ((BIT(13) | BIT(12)) !=
		       (readl(tp->mmio_sdsaddr + SDS_MISC) &
		       (BIT(13) | BIT(12)))) {
			stable_ticks = 0;
			if (tmp >= 100) {
				stable_ticks = 10;
				pr_err("SGMII link down in 100ms\n");
				break;
			}
		}
		stable_ticks++;
		tmp++;
		mdelay(1);
	}
	pr_info("SGMII link up in %d ms", tmp);

	reset_control_put(rstc_sds_reg);
	reset_control_put(rstc_sds);
	reset_control_put(rstc_pcie0_power);
	reset_control_put(rstc_pcie0_phy);
	reset_control_put(rstc_pcie0_sgmii_mdio);
	reset_control_put(rstc_pcie0_phy_mdio);
	return 0;
}

static void r8169soc_phy_iol_tuning(struct rtl8169_private *tp)
{
	switch (get_rtd_chip_revision()) {
	case RTD_CHIP_A00: /* TSMC, cut A */
	case RTD_CHIP_A01: /* TSMC, cut B */
		/* idacfine */
		int_mdio_write(tp, 0x0bc0, 23, 0x0088);

		/* abiq */
		int_mdio_write(tp, 0x0bc0, 21, 0x0004);

		/* ldvbias */
		int_mdio_write(tp, 0x0bc0, 22, 0x0777);

		/* iatt */
		int_mdio_write(tp, 0x0bd0, 16, 0x0300);

		/* vcm_ref, cf_l */
		int_mdio_write(tp, 0x0bd0, 17, 0xe8ca);
		break;
	case RTD_CHIP_A02: /* UMC, cut C */
		/* 100M Swing */
		/* idac_fine_mdix, idac_fine_mdi */
		int_mdio_write(tp, 0x0bc0, 23, 0x0044);

		/* 100M Tr/Tf */
		/* abiq_10m=0x0, abiq_100m_short=0x4, abiq_normal=0x6 */
		int_mdio_write(tp, 0x0bc0, 21, 0x0046);

		/* 10M */
		/* ldvbias_10m=0x7, ldvbias_10m_short=0x4, ldvbias_normal=0x4 */
		int_mdio_write(tp, 0x0bc0, 22, 0x0744);

		/* vcmref=0x0, cf_l=0x3 */
		int_mdio_write(tp, 0x0bd0, 17, 0x18ca);

		/* iatt=0x2 */
		int_mdio_write(tp, 0x0bd0, 16, 0x0200);
		break;

	/* default: */
	}
}

static void r8169soc_mdio_init(struct rtl8169_private *tp)
{
	u32 tmp;
	struct pinctrl *p_sgmii_mdio;
	struct pinctrl_state *ps_sgmii_mdio;
	void __iomem *ioaddr = tp->mmio_addr;

	/* ETN_PHY_INTR, wait interrupt from PHY and it means MDIO is ready */
	tmp = 0;
	while ((readl(tp->mmio_clkaddr + ISO_UMSK_ISR) & BIT(27))
	       == 0) {
		tmp += 1;
		mdelay(1);
		if (tmp >= 100) {
			pr_err("PHY PHY_Status timeout.\n");
			break;
		}
	}
	pr_info("wait %d ms for PHY interrupt. UMSK_ISR = 0x%x\n",
		tmp, readl(tp->mmio_clkaddr + ISO_UMSK_ISR));

	/* In Hercules ByPass mode,
	 * SW need to handle the EPHY Status check ,
	 * OTP data update and EPHY fuse_rdy setting.
	 */
	if (tp->bypass_enable) {
		/* PHY will stay in state 1 mode */
		tmp = 0;
		while (0x1 != (int_mdio_read(tp, 0x0a42, 16) & 0x07)) {
			tmp += 1;
			mdelay(1);
			if (tmp >= 2000) {
				pr_err("PHY status is not 0x1 in bypass mode, current = 0x%02x\n",
				       (int_mdio_read(tp, 0x0a42, 16) & 0x07));
				break;
			}
		}

		/* adjust FE PHY electrical characteristics */
		r8169soc_phy_iol_tuning(tp);

		/* 1. read OTP 0x9801_72C8[12:8]
		 * 2. xor 0x08
		 * 3. set value to PHY registers to correct R-calibration
		 * 4. read OTP 0x9801_72D8[7:0]
		 * 5. xor 0x33
		 * 6. set value to PHY registers to correct AMP
		 */
		r8169soc_load_otp_content(tp);

		/* fill fuse_rdy & rg_ext_ini_done */
		int_mdio_write(tp, 0x0a46, 20,
			       (int_mdio_read(tp, 0x0a46, 20) | (BIT(1) | BIT(0))));
	} else {
		/* adjust FE PHY electrical characteristics */
		r8169soc_phy_iol_tuning(tp);
	}

	/* init_autoload_done = 1 */
	tmp = rtl_ocp_read(tp, 0xe004);
	tmp |= BIT(7);
	rtl_ocp_write(tp, 0xe004, tmp);
	/* ee_mode = 3 */
	RTL_W8(Cfg9346, Cfg9346_Unlock);

	/* wait LAN-ON */
	tmp = 0;
	do {
		tmp += 1;
		mdelay(1);
		if (tmp >= 2000) {
			pr_err("PHY status is not 0x3, current = 0x%02x\n",
			       (int_mdio_read(tp, 0x0a42, 16) & 0x07));
			break;
		}
	} while (0x3 != (int_mdio_read(tp, 0x0a42, 16) & 0x07));
	pr_info("wait %d ms for PHY ready, current = 0x%x\n",
		tmp, int_mdio_read(tp, 0x0a42, 16));

	if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
		/* Init PHY path */
		/* reg_0x9800705c[5] = 0 */
		/* reg_0x9800705c[7] = 0 */
		/* ISO spec, set internal MDIO to access PHY */
		tmp = readl(tp->mmio_clkaddr + ISO_PWRCUT_ETN);
		tmp &= ~(BIT(7) | BIT(5));
		writel(tmp, tp->mmio_clkaddr + ISO_PWRCUT_ETN);

		/* reg_0x9800705c[4] = 0 */
		/* ISO spec, set data path to access PHY */
		tmp = readl(tp->mmio_clkaddr + ISO_PWRCUT_ETN);
		tmp &= ~(BIT(4));
		writel(tmp, tp->mmio_clkaddr + ISO_PWRCUT_ETN);

		/* # ETN spec, GMAC data path select MII-like(embedded GPHY),
		 * not SGMII(external PHY)
		 */
		tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(0) | BIT(1));
		tmp |= BIT(1); /* MII */
		rtl_ocp_write(tp, 0xea34, tmp);
	} else {
		/* SGMII */
		/* # ETN spec, adjust MDC freq=2.5MHz */
		rtl_ocp_write(tp, 0xDE30,
			      rtl_ocp_read(tp, 0xDE30) & ~(BIT(7) | BIT(6)));
		/* # ETN spec, set external PHY addr */
		rtl_ocp_write(tp, 0xDE24,
			      ((rtl_ocp_read(tp, 0xDE24) & ~(0x1f << 0)) |
					     (tp->ext_phy_id & 0x1f)));
		/* ISO mux spec, GPIO29 is set to MDC pin */
		/* ISO mux spec, GPIO46 is set to MDIO pin */
		p_sgmii_mdio = devm_pinctrl_get(&tp->pdev->dev);
		ps_sgmii_mdio = pinctrl_lookup_state(p_sgmii_mdio, "sgmii");
		pinctrl_select_state(p_sgmii_mdio, ps_sgmii_mdio);

		/* check if external PHY is available */
		pr_info("Searching external PHY...");
		tp->ext_phy = true;
		tmp = 0;
		while (ext_mdio_read(tp, 0x0a43, 31) != 0x0a43) {
			pr_info(".");
			tmp += 1;
			mdelay(1);
			if (tmp >= 2000) {
				pr_err("\n External SGMII PHY not found, current = 0x%02x\n",
				       ext_mdio_read(tp, 0x0a43, 31));
				break;
			}
		}
		if (tmp < 2000)
			pr_info("found.\n");

		/* lower SGMII TX swing of RTL8211FS to reduce EMI */
		/* TX swing = 470mV, default value */
		ext_mdio_write(tp, 0x0dcd, 16, 0x104e);

		tp->ext_phy = false;

		/* # ETN spec, GMAC data path select SGMII(external PHY),
		 * not MII-like(embedded GPHY)
		 */
		tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(0) | BIT(1));
		tmp |= BIT(1) | BIT(0); /* SGMII */
		rtl_ocp_write(tp, 0xea34, tmp);

		if (r8169soc_serdes_init(tp) != 0)
			pr_err("SERDES init failed\n");
		/* ext_phy == true now */

		/* SDS spec, auto update SGMII link capability */
		tmp = readl(tp->mmio_sdsaddr + SDS_LINK);
		tmp |= BIT(2);
		writel(tmp, tp->mmio_sdsaddr + SDS_LINK);
	}
}

static void r8169soc_eee_init(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
			/* 100M EEE capability */
			rtl_mmd_write(tp, 0x7, 60, BIT(1));
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) | (BIT(4) | BIT(2)));
			/* disable dynamic RX power in PHY */
			rtl_phy_write(tp, 0x0bd0, 21,
				      (rtl_phy_read(tp, 0x0bd0, 21) & ~BIT(8)) | BIT(9));
		} else { /* SGMII */
			/* 1000M & 100M EEE capability */
			rtl_mmd_write(tp, 0x7, 60, (BIT(2) | BIT(1)));
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) | (BIT(4) | BIT(2)));
		}
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040, (rtl_ocp_read(tp, 0xe040) | (BIT(1) | BIT(0))));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe08a, 0x00a7); /* timer to wait FEPHY ready */
		rtl_ocp_write(tp, 0xe080, (rtl_ocp_read(tp, 0xe080) | BIT(1)));
	} else {
		if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
			/* no EEE capability */
			rtl_mmd_write(tp, 0x7, 60, 0);
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) & ~(BIT(4) | BIT(2)));
		} else { /* SGMII */
			/* no EEE capability */
			rtl_mmd_write(tp, 0x7, 60, 0);
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) & ~(BIT(4) | BIT(2)));
		}
		/* reset to default value */
		rtl_ocp_write(tp, 0xe040, (rtl_ocp_read(tp, 0xe040) | BIT(13)));
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040, (rtl_ocp_read(tp, 0xe040) & ~(BIT(1) | BIT(0))));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080, (rtl_ocp_read(tp, 0xe080) & ~BIT(1)));
		rtl_ocp_write(tp, 0xe08a, 0x003f); /* default value */
	}
}
#endif /* CONFIG_ARCH_RTD139x */

#if defined(CONFIG_ARCH_RTD16xx)
static void r8169soc_reset_phy_gmac(struct rtl8169_private *tp)
{
	u32 tmp;
	struct clk *clk_etn_sys = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");
	struct clk *clk_en_sds = NULL;
	struct reset_control *rstc_sds_reg = NULL;
	struct reset_control *rstc_sds = NULL;
	struct reset_control *rstc_pcie0_power = NULL;
	struct reset_control *rstc_pcie0_phy = NULL;
	struct reset_control *rstc_pcie0_sgmii_mdio = NULL;
	struct reset_control *rstc_pcie0_phy_mdio = NULL;

	clk_prepare_enable(clk_etn_sys);
	clk_prepare_enable(clk_etn_250m);

	if (tp->output_mode != OUTPUT_EMBEDDED_PHY) {
		clk_en_sds = clk_get(&tp->pdev->dev, "sds");
		rstc_sds_reg =
			reset_control_get_exclusive(&tp->pdev->dev, "sds_reg");
		rstc_sds = reset_control_get_exclusive(&tp->pdev->dev, "sds");
		rstc_pcie0_power =
			reset_control_get_exclusive(&tp->pdev->dev, "pcie0_power");
		rstc_pcie0_phy =
			reset_control_get_exclusive(&tp->pdev->dev, "pcie0_phy");
		rstc_pcie0_sgmii_mdio =
			reset_control_get_exclusive(&tp->pdev->dev, "pcie0_sgmii_mdio");
		rstc_pcie0_phy_mdio =
			reset_control_get_exclusive(&tp->pdev->dev, "pcie0_phy_mdio");
	}

	/* reg_0x9800708c[12:11] = 00 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_disable_unprepare(clk_etn_sys);
	clk_disable_unprepare(clk_etn_250m);

	/* reg_0x98007088[10:9] = 00 */
	/* ISO spec, rstn_gphy & rstn_gmac */
	reset_control_assert(rstc_gphy);
	reset_control_assert(rstc_gmac);

	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	tmp = readl(tp->mmio_clkaddr + ISO_ETN_TESTIO);
	tmp |= BIT(1);
	writel(tmp, tp->mmio_clkaddr + ISO_ETN_TESTIO);

	/* reg_0x9800705c = 0x00616703 */
	/* ISO spec, default value */
	writel(0x00616703, tp->mmio_clkaddr + ISO_PWRCUT_ETN);

	if (tp->output_mode != OUTPUT_EMBEDDED_PHY) {
		/* RESET for SGMII if needed */
		clk_prepare_enable(clk_en_sds);

		/* reg_0x98000050[13:12] = 10 */
		/* CRT spec, clk_en_sds */
		clk_disable_unprepare(clk_en_sds);

		/* reg_0x98000000[7:6] = 10 */
		/* reg_0x98000000[9:8] = 10 */
		/* CRT spec, rstn_sds_reg & rstn_sds */
		reset_control_assert(rstc_sds);
		reset_control_assert(rstc_sds_reg);

		/* reg_0x98000004[25:24] = 10 CRT spec, rstn_pcie0_sgmii_mdio */
		/* reg_0x98000004[23:22] = 10 CRT spec, rstn_pcie0_phy_mdio */
		/* reg_0x98000004[19:18] = 10 CRT spec, rstn_pcie0_power */
		/* reg_0x98000004[13:12] = 10 CRT spec, rstn_pcie0_phy */
		reset_control_assert(rstc_pcie0_sgmii_mdio);
		reset_control_assert(rstc_pcie0_phy_mdio);
		reset_control_assert(rstc_pcie0_power);
		reset_control_assert(rstc_pcie0_phy);
	}

	mdelay(1);

	/* release resource */
	if (tp->output_mode != OUTPUT_EMBEDDED_PHY) {
		reset_control_put(rstc_sds_reg);
		reset_control_put(rstc_sds);
		reset_control_put(rstc_pcie0_power);
		reset_control_put(rstc_pcie0_phy);
		reset_control_put(rstc_pcie0_sgmii_mdio);
		reset_control_put(rstc_pcie0_phy_mdio);
	}
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
}

static void r8169soc_acp_init(struct rtl8169_private *tp)
{
	u32 tmp;

	/* SBX spec, Select ETN access DDR path. */
	if (tp->acp_enable) {
		/* reg_0x9801c20c[6] = 1 */
		/* SBX spec, Mask ETN_ALL to SB3 DBUS REQ */
		tmp = readl(tp->mmio_sbxaddr + SBX_SB3_CHANNEL_REQ_MASK);
		tmp |= BIT(6);
		writel(tmp, tp->mmio_sbxaddr + SBX_SB3_CHANNEL_REQ_MASK);

		pr_info("wait all SB3 access finished...");
		tmp = 0;
		while (0 != (BIT(6) & readl(tp->mmio_sbxaddr + SBX_SB3_CHANNEL_REQ_BUSY))) {
			mdelay(1);
			tmp += 1;
			if (tmp >= 100) {
				pr_err("\n wait SB3 access failed (wait %d ms)\n", tmp);
				break;
			}
		}
		if (tmp < 100)
			pr_info("done.\n");

		/* reg_0x9801d100[29] = 0 */
		/* SCPU wrapper spec, CLKACP division, 0 = div 2, 1 = div 3 */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);
		tmp &= ~(BIT(29));
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);

		/* reg_0x9801d124[1:0] = 00 */
		/* SCPU wrapper spec, ACP master active, 0 = active */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_INTERFACE_EN);
		tmp &= ~(BIT(1) | BIT(0));
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_INTERFACE_EN);

		/* reg_0x9801d100[30] = 1 */
		/* SCPU wrapper spec, dy_icg_en_acp */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);
		tmp |= BIT(30);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);

		/* reg_0x9801d100[21] = 1 */
		/* SCPU wrapper spec, ACP CLK enable */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);
		tmp |= BIT(21);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);

		/* reg_0x9801d100[14] = 1 */
		/* SCPU wrapper spec,
		 * Do not apply reset to ACP port axi3 master
		 */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);
		tmp |= BIT(14);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);

		/* reg_0x9801d800[3:0] = 0111 */
		/* reg_0x9801d800[7:4] = 0111 */
		/* reg_0x9801d800[9:8] = 00 */
		/* reg_0x9801d800[20:16] = 01100 */
		/* reg_0x9801d800[28:24] = 01110 */
		/* Configure control of ACP port */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_ACP_CTRL);
		tmp &= ~((0x1f << 24) | (0x1f << 16) | (0x1 << 9) |
			(0xf << 4) | (0xf << 0));
		tmp |= (0x0e << 24) | (0x0c << 16) | (0x1 << 9) | (0x7 << 4) |
			(0x7 << 0);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_ACP_CTRL);

		/* reg_0x9801d030[28] = 1 */
		/* SCPU wrapper spec, dy_icg_en_acp */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);
		tmp |= BIT(28);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);

		/* reg_0x9801d030[16] = 1 */
		/* SCPU wrapper spec, ACP CLK Enable for acp of scpu_chip_top */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);
		tmp |= BIT(16);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);

		/* reg_0x9801d030[0] = 1 */
		/* SCPU wrapper spec, Do not apply reset to ACP port axi3 master */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);
		tmp |= BIT(0);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);

		/* reg_0x9801c814[17] = 1 */
		/* through ACP to SCPU_ACP */
		tmp = readl(tp->mmio_sbxaddr + SBX_ACP_MISC_CTRL);
		tmp |= BIT(17);
		writel(tmp, tp->mmio_sbxaddr + SBX_ACP_MISC_CTRL);

		/* SBX spec, Remove mask ETN_ALL to ACP DBUS REQ */
		tmp = readl(tp->mmio_sbxaddr + SBX_ACP_CHANNEL_REQ_MASK);
		tmp &= ~BIT(1);
		writel(tmp, tp->mmio_sbxaddr + SBX_ACP_CHANNEL_REQ_MASK);

		pr_info("ARM ACP on\n.");
	} else {
		/* SBX spec, Mask ETN_ALL to ACP DBUS REQ */
		tmp = readl(tp->mmio_sbxaddr + SBX_ACP_CHANNEL_REQ_MASK);
		tmp |= BIT(1);
		writel(tmp, tp->mmio_sbxaddr + SBX_ACP_CHANNEL_REQ_MASK);

		pr_info("wait all ACP access finished...");
		tmp = 0;
		while (0 != (BIT(1) & readl(tp->mmio_sbxaddr + SBX_ACP_CHANNEL_REQ_BUSY))) {
			mdelay(1);
			tmp += 1;
			if (tmp >= 100) {
				pr_err("\n ACP channel is still busy (wait %d ms)\n", tmp);
				break;
			}
		}
		if (tmp < 100)
			pr_info("done.\n");

		/* SCPU wrapper spec, Inactive MP4 AINACTS signal */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_INTERFACE_EN);
		tmp |= (BIT(1) | BIT(0));
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_INTERFACE_EN);

		/* SCPU wrapper spec, nACPRESET_DVFS & CLKENACP_DVFS */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);
		tmp &= ~(BIT(21) | BIT(14));
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);

		/* SCPU wrapper spec, nACPRESET & CLKENACP */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);
		tmp &= ~(BIT(16) | BIT(0));
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);

		/* reg_0x9801c814[17] = 0 */
		/* SBX spec, Switch ETN_ALL to DC_SYS path */
		tmp = readl(tp->mmio_sbxaddr + SBX_ACP_MISC_CTRL);
		tmp &= ~BIT(17);
		writel(tmp, tp->mmio_sbxaddr + SBX_ACP_MISC_CTRL);

		/* SBX spec, Remove mask ETN_ALL to SB3 DBUS REQ */
		tmp = readl(tp->mmio_sbxaddr + SBX_SB3_CHANNEL_REQ_MASK);
		tmp &= ~BIT(6);
		writel(tmp, tp->mmio_sbxaddr + SBX_SB3_CHANNEL_REQ_MASK);

		pr_info("ARM ACP off\n.");
	}
}

static void r8169soc_pll_clock_init(struct rtl8169_private *tp)
{
	u32 tmp;
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");

	/* reg_0x98007004[27] = 1 */
	/* ISO spec, ETN_PHY_INTR, clear ETN interrupt for ByPassMode */
	writel(BIT(27), tp->mmio_clkaddr + ISO_UMSK_ISR);

	/* reg_0x98007088[10] = 1 */
	/* ISO spec, reset bit of gphy */
	reset_control_deassert(rstc_gphy);

	mdelay(1);	/* wait 1ms for PHY PLL stable */

	/* Thor only supports the bypass mode */
	/* Bypass mode : ETN MAC bypass efuse update flow.
	 * SW need to take this sequence.
	 */
	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	/* bypass mode, SW need to handle the EPHY Status check ,
	 * EFUSE data update and EPHY fuse_rdy setting.
	 */
	tmp = readl(tp->mmio_clkaddr + ISO_ETN_TESTIO);
	tmp |= BIT(1);
	writel(tmp, tp->mmio_clkaddr + ISO_ETN_TESTIO);

	/* reg_0x98007088[9] = 1 */
	/* ISO spec, reset bit of gmac */
	reset_control_deassert(rstc_gmac);

	/* reg_0x9800708c[12:11] = 11 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_prepare_enable(clk_etn_sys);
	clk_prepare_enable(clk_etn_250m);

	mdelay(10);		/* wait 10ms for GMAC uC to be stable */

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
}

static void r8169soc_load_otp_content(struct rtl8169_private *tp)
{
	u32 otp;
	u16 tmp;
	u8 *buf;

	/* RC-K 0x980174F8[27:24] */
	buf = r8169soc_read_otp(tp, "rc_k");
	if (IS_ERR(buf))
		goto set_r_amp_cal;

	otp = *buf;
	tmp = (otp << 12) | (otp << 8) | (otp << 4) | otp;
	tmp ^= RC_K_DEFAULT;
	int_mdio_write(tp, 0x0bcd, 22, tmp);
	int_mdio_write(tp, 0x0bcd, 23, tmp);

	kfree(buf);

set_r_amp_cal:

	buf = r8169soc_read_otp(tp, "r_amp_k");
	if (IS_ERR(buf))
		return;

	/* R-K 0x98017500[18:15] */
	otp = ((buf[5] & 0x80) >> 7) | ((buf[6] & 0x07) << 1);
	tmp = (otp << 12) | (otp << 8) | (otp << 4) | otp;
	tmp ^= R_K_DEFAULT;
	int_mdio_write(tp, 0x0bce, 16, tmp);
	int_mdio_write(tp, 0x0bce, 17, tmp);

	/* Amp-K 0x980174FC[15:0] */
	otp = buf[0] | (buf[1] << 8);
	tmp = otp ^ AMP_K_DEFAULT;
	int_mdio_write(tp, 0x0bca, 22, tmp);

	/* Bias-K 0x980174FC[31:16] */
	otp = buf[2] | (buf[3] << 8);
	tmp = otp ^ ADC_BIAS_K_DEFAULT;
	int_mdio_write(tp, 0x0bcf, 22, tmp);

	kfree(buf);
}

static u32 r8169soc_serdes_init(struct rtl8169_private *tp)
{
	u32 stable_ticks;
	u32 tmp;
	struct clk *clk_en_sds = clk_get(&tp->pdev->dev, "sds");
	struct reset_control *rstc_sds_reg =
		reset_control_get_exclusive(&tp->pdev->dev, "sds_reg");
	struct reset_control *rstc_sds =
		reset_control_get_exclusive(&tp->pdev->dev, "sds");
	struct reset_control *rstc_pcie0_power =
		reset_control_get_exclusive(&tp->pdev->dev, "pcie0_power");
	struct reset_control *rstc_pcie0_phy =
		reset_control_get_exclusive(&tp->pdev->dev, "pcie0_phy");
	struct reset_control *rstc_pcie0_sgmii_mdio =
		reset_control_get_exclusive(&tp->pdev->dev, "pcie0_sgmii_mdio");
	struct reset_control *rstc_pcie0_phy_mdio =
		reset_control_get_exclusive(&tp->pdev->dev, "pcie0_phy_mdio");

	/* reg_0x98000050[13:12] = 11 */
	/* CRT spec, clk_en_sds */
	clk_prepare_enable(clk_en_sds);

	/* reg_0x98000000[9:8] = 11 */
	/* reg_0x98000000[7:6] = 11 */
	/* CRT spec, rstn_sds_reg & rstn_sds */
	reset_control_deassert(rstc_sds);
	reset_control_deassert(rstc_sds_reg);

	/* reg_0x98000004[25:24] = 11   CRT spec, rstn_pcie0_sgmii_mdio */
	/* reg_0x98000004[23:22] = 11   CRT spec, rstn_pcie0_phy_mdio */
	/* reg_0x98000004[19:18] = 11   CRT spec, rstn_pcie0_power */
	/* reg_0x98000004[13:12] = 11   CRT spec, rstn_pcie0_phy */
	reset_control_deassert(rstc_pcie0_power);
	reset_control_deassert(rstc_pcie0_phy);
	reset_control_deassert(rstc_pcie0_sgmii_mdio);
	reset_control_deassert(rstc_pcie0_phy_mdio);

	/* reg_0x9800705c[6] = 1 */
	/* ISO spec, set PCIe channel to SGMII */
	tmp = readl(tp->mmio_clkaddr + ISO_PWRCUT_ETN);
	tmp |= BIT(6);
	writel(tmp, tp->mmio_clkaddr + ISO_PWRCUT_ETN);

	/* ### Beginning of SGMII DPHY register tuning ### */
	/* reg_0x9800705c[7] = 1 */
	/* ISO spec, set internal MDIO to PCIe */
	tmp = readl(tp->mmio_clkaddr + ISO_PWRCUT_ETN);
	tmp |= BIT(7);
	writel(tmp, tp->mmio_clkaddr + ISO_PWRCUT_ETN);

	/* reg_0x9800705c[20:16] = 00000 */
	/* ISO spec, set internal PHY addr to SERDES_DPHY_0 */
	__int_set_phy_addr(tp, SERDES_DPHY_0);

	/* # DPHY spec, 5GHz tuning */
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x4, 0x52f5);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x5, 0xead7);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x6, 0x0010);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0xa, 0xc653);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x1, 0xe030);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0xd, 0xee1c);

	/* reg_0x9800705c[20:16] = 00001 */
	/* ISO spec, set internal PHY addr to SERDES_DPHY_1 */
	__int_set_phy_addr(tp, SERDES_DPHY_1);

	/* TX_Swing_550mV by default */
	switch (tp->sgmii_swing) {
	case TX_Swing_190mV:
		int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x0, 0xd411);
		int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x1, 0x2277);
		break;
	case TX_Swing_250mV:
		int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x0, 0xd433);
		int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x1, 0x2244);
		break;
	case TX_Swing_380mV:
		int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x0, 0xd433);
		int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x1, 0x22aa);
		break;
	case TX_Swing_550mV: /* recommended by RDC */
	default:
		int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x0, 0xd455);
		int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x1, 0x2828);
	}

	/* reg_0x9800705c[20:16] = 00001 */
	/* ISO spec, set internal PHY addr to INT_PHY_ADDR */
	__int_set_phy_addr(tp, INT_PHY_ADDR);

	mdelay(10);		/* wait for clock stable */

	/* reg_0x9800705c[5] = 0 */
	/* reg_0x9800705c[7] = 0 */
	/* ISO spec, set internal MDIO to GPHY */
	tmp = readl(tp->mmio_clkaddr + ISO_PWRCUT_ETN);
	tmp &= ~(BIT(7) | BIT(5));
	writel(tmp, tp->mmio_clkaddr + ISO_PWRCUT_ETN);

	tp->ext_phy = true;
	/* ext_phy == true now */

	tmp = 0;
	stable_ticks = 0;
	while (stable_ticks < 10) {
		/* # SDS spec, wait for phy ready */
		while ((readl(tp->mmio_sdsaddr + SDS_LINK) & BIT(24)) == 0) {
			stable_ticks = 0;
			if (tmp >= 100) {
				stable_ticks = 10;
				pr_err("SGMII PHY not ready in 100ms\n");
				break;
			}
		}
		stable_ticks++;
		tmp++;
		mdelay(1);
	}
	pr_info("SGMII PHY is ready in %d ms", tmp);

	/* reg_0x9800705c[4] = 1 */
	/* ISO spec, set data path to SGMII */
	tmp = readl(tp->mmio_clkaddr + ISO_PWRCUT_ETN);
	tmp |= BIT(4);
	writel(tmp, tp->mmio_clkaddr + ISO_PWRCUT_ETN);

	/* reg_0x981c8008[9:8] = 00 */
	/* # SDS spec, SP_SDS_FRC_AN, SERDES auto mode */
	tmp = readl(tp->mmio_sdsaddr + SDS_REG02);
	tmp &= ~(BIT(9) | BIT(8));
	writel(tmp, tp->mmio_sdsaddr + SDS_REG02);

	/* # SDS spec, wait for SERDES link up */
	tmp = 0;
	stable_ticks = 0;
	while (stable_ticks < 10) {
		while ((readl(tp->mmio_sdsaddr + SDS_MISC) & BIT(12)) == 0) {
			stable_ticks = 0;
			if (tmp >= 100) {
				stable_ticks = 10;
				pr_err("SGMII link down in 100ms\n");
				break;
			}
		}
		stable_ticks++;
		tmp++;
		mdelay(1);
	}
	pr_info("SGMII link up in %d ms", tmp);

	reset_control_put(rstc_sds_reg);
	reset_control_put(rstc_sds);
	reset_control_put(rstc_pcie0_power);
	reset_control_put(rstc_pcie0_phy);
	reset_control_put(rstc_pcie0_sgmii_mdio);
	reset_control_put(rstc_pcie0_phy_mdio);
	return 0;
}

static void r8169soc_phy_iol_tuning(struct rtl8169_private *tp)
{
	/* for common mode voltage */
	int_mdio_write(tp, 0x0bc0, 17,
		       (int_mdio_read(tp, 0x0bc0, 17) & ~(0xff << 4)) | (0xb4 << 4));

	/* for 1000 Base-T, Transmitter Distortion */
	int_mdio_write(tp, 0x0a43, 27, 0x8082);
	int_mdio_write(tp, 0x0a43, 28,
		       (int_mdio_read(tp, 0x0a43, 28) & ~(0xff << 8)) | (0xae << 8));

	/* for 1000 Base-T, Master Filtered Jitter */
	int_mdio_write(tp, 0x0a43, 27, 0x807c);
	/* adjust ldvbias_busy, abiq_busy, gdac_ib_up_tm
	 * to accelerate slew rate
	 */
	int_mdio_write(tp, 0x0a43, 28, 0xf001);
	/* set CP_27to25=7 and REF_27to25_L=4 to decrease jitter */
	int_mdio_write(tp, 0x0bc5, 16, 0xc67f);
}

static void r8169soc_phy_sram_table(struct rtl8169_private *tp)
{
	/* enable echo power*2 */
	int_mdio_write(tp, 0x0a42, 22, 0x0f10);

	/* Channel estimation, 100Mbps adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x8087);	/* gain_i slope */
	int_mdio_write(tp, 0x0a43, 28, 0x42f0);	/* 0x43 => 0x42 */
	int_mdio_write(tp, 0x0a43, 27, 0x808e);	/* clen_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0x14a4);	/* 0x13 => 0x14 */
	/* adc peak adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x8088);	/* aagc_lvl_c initial value 0x3f0 => ok */
	int_mdio_write(tp, 0x0a43, 28, 0xf0eb);	/* delta_a slope 0x1e => 0x1d */
	/* cb0 adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x808c);	/* cb0_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0xef09);	/* 0x9ef => ok */
	int_mdio_write(tp, 0x0a43, 27, 0x808f);	/* delta_b slope */
	int_mdio_write(tp, 0x0a43, 28, 0xa4c6);	/* 0xa4 => ok */
	/* DAGC adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x808a);	/* cg_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0x400a);	/* 0xb40 => 0xa40 */
	int_mdio_write(tp, 0x0a43, 27, 0x8092);	/* delta_g slope */
	int_mdio_write(tp, 0x0a43, 28, 0xc21e);	/* 0x0c0 => 0x0c2 */

	/* 1000Mbps master adjustment */
	/* line adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x8099);	/* gain_i slope */
	int_mdio_write(tp, 0x0a43, 28, 0x2ae0);	/* 0x2e => 0x2a */
	int_mdio_write(tp, 0x0a43, 27, 0x80a0);	/* clen_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0xf28f);	/* 0xfe => 0xf2 */
	/* adc peak adjustment */
	/* aagc_lvl_c initial value 0x3e0 => 0x470 */
	int_mdio_write(tp, 0x0a43, 27, 0x809a);
	int_mdio_write(tp, 0x0a43, 28, 0x7084);	/* delta_a slope 0x0d => 0x10 */
	/* cb0 adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x809e);	/* cb0_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0xa008);	/* 0x7a1 => 0x8a0 */
	int_mdio_write(tp, 0x0a43, 27, 0x80a1);	/* delta_b slope */
	int_mdio_write(tp, 0x0a43, 28, 0x783d);	/* 0x8f => 0x78 */
	/* DAGC adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x809c);	/* cg_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0x8008);	/* 0xb00 => 0x880 */
	int_mdio_write(tp, 0x0a43, 27, 0x80a4);	/* delta_g slope */
	int_mdio_write(tp, 0x0a43, 28, 0x580c);	/* 0x063 => 0x058 */

	/* 1000Mbps slave adjustment */
	/* line adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x80ab);	/* gain_i slope */
	int_mdio_write(tp, 0x0a43, 28, 0x2b4a);	/* 0x2e => 0x2b */
	int_mdio_write(tp, 0x0a43, 27, 0x80b2);	/* clen_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0xf47f);	/* 0xfa => 0xf4 */
	/* adc peak adjustment */
	/* aagc_lvl_c initial value 0x44a => 0x488 */
	int_mdio_write(tp, 0x0a43, 27, 0x80ac);
	int_mdio_write(tp, 0x0a43, 28, 0x8884);	/* delta_a slope 0x0e => 0x10 */
	/* cb0 adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x80b0);	/* cb0_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0xa107);	/* 0x6a1 => 0x7a1 */
	int_mdio_write(tp, 0x0a43, 27, 0x80b3);	/* delta_b slope */
	int_mdio_write(tp, 0x0a43, 28, 0x683d);	/* 0x7f => 0x68 */
	/* DAGC adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x80ae);	/* cg_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0x2006);	/* 0x760 => 0x620 */
	int_mdio_write(tp, 0x0a43, 27, 0x80b6);	/* delta_g slope */
	int_mdio_write(tp, 0x0a43, 28, 0x850c);	/* 0x090 => 0x085 */
}

static void r8169soc_patch_gphy_uc_code(struct rtl8169_private *tp)
{
	u32 tmp;

	/* patch for GPHY uC firmware, adjust 1000M EEE lpi_waketx_timer = 1.3uS */
	#define PATCH_KEY_ADDR  0x8028   /* for RL6525 */
	#define PATCH_KEY       0x5600   /* for RL6525 */

	/* Patch request & wait for the asserting of patch_rdy */
	int_mdio_write(tp, 0x0b82, 16,
		       int_mdio_read(tp, 0x0b82, 16) | BIT(4));

	tmp = 0;
	while ((int_mdio_read(tp, 0x0b80, 16) & BIT(6)) == 0) {
		tmp += 1;
		mdelay(1);
		if (tmp >= 100) {
			pr_err("GPHY patch_rdy timeout.\n");
			break;
		}
	}
	pr_err("wait %d ms for GPHY patch_rdy. reg = 0x%x\n",
	       tmp, int_mdio_read(tp, 0x0b80, 16));
	pr_err("patch_rdy is asserted!!\n");

	/* Set patch_key & patch_lock */
	int_mdio_write(tp, 0, 27, PATCH_KEY_ADDR);
	int_mdio_write(tp, 0, 28, PATCH_KEY);
	int_mdio_write(tp, 0, 27, PATCH_KEY_ADDR);
	pr_info("check patch key = %04x\n", int_mdio_read(tp, 0, 28));
	int_mdio_write(tp, 0, 27, 0xb82e);
	int_mdio_write(tp, 0, 28, 0x0001);

	/* uC patch code, released by Digital Designer */
	int_mdio_write(tp, 0, 27, 0xb820);
	int_mdio_write(tp, 0, 28, 0x0290);

	int_mdio_write(tp, 0, 27, 0xa012);
	int_mdio_write(tp, 0, 28, 0x0000);

	int_mdio_write(tp, 0, 27, 0xa014);
	int_mdio_write(tp, 0, 28, 0x2c04);
	int_mdio_write(tp, 0, 28, 0x2c06);
	int_mdio_write(tp, 0, 28, 0x2c09);
	int_mdio_write(tp, 0, 28, 0x2c0c);
	int_mdio_write(tp, 0, 28, 0xd093);
	int_mdio_write(tp, 0, 28, 0x2265);
	int_mdio_write(tp, 0, 28, 0x9e20);
	int_mdio_write(tp, 0, 28, 0xd703);
	int_mdio_write(tp, 0, 28, 0x2502);
	int_mdio_write(tp, 0, 28, 0x9e40);
	int_mdio_write(tp, 0, 28, 0xd700);
	int_mdio_write(tp, 0, 28, 0x0800);
	int_mdio_write(tp, 0, 28, 0x9e80);
	int_mdio_write(tp, 0, 28, 0xd70d);
	int_mdio_write(tp, 0, 28, 0x202e);

	int_mdio_write(tp, 0, 27, 0xa01a);
	int_mdio_write(tp, 0, 28, 0x0000);

	int_mdio_write(tp, 0, 27, 0xa006);
	int_mdio_write(tp, 0, 28, 0x002d);

	int_mdio_write(tp, 0, 27, 0xa004);
	int_mdio_write(tp, 0, 28, 0x0507);

	int_mdio_write(tp, 0, 27, 0xa002);
	int_mdio_write(tp, 0, 28, 0x0501);

	int_mdio_write(tp, 0, 27, 0xa000);
	int_mdio_write(tp, 0, 28, 0x1264);

	int_mdio_write(tp, 0, 27, 0xb820);
	int_mdio_write(tp, 0, 28, 0x0210);

	mdelay(10);

	/* Clear patch_key & patch_lock */
	int_mdio_write(tp, 0, 27, PATCH_KEY_ADDR);
	int_mdio_write(tp, 0, 28, 0);
	int_mdio_write(tp, 0x0b82, 23, 0);

	/* Release patch request & wait for the deasserting of patch_rdy. */
	int_mdio_write(tp, 0x0b82, 16,
		       int_mdio_read(tp, 0x0b82, 16) & ~BIT(4));

	tmp = 0;
	while ((int_mdio_read(tp, 0x0b80, 16) & BIT(6)) != 0) {
		tmp += 1;
		mdelay(1);
		if (tmp >= 100) {
			pr_err("GPHY patch_rdy timeout.\n");
			break;
		}
	}
	pr_err("wait %d ms for GPHY patch_rdy. reg = 0x%x\n",
	       tmp, int_mdio_read(tp, 0x0b80, 16));

	pr_err("\npatch_rdy is de-asserted!!\n");
	pr_err("GPHY uC code patched.\n");
}

static void r8169soc_mdio_init(struct rtl8169_private *tp)
{
	u32 tmp;
	struct pinctrl *p_sgmii_mdio;
	struct pinctrl_state *ps_sgmii_mdio;
	void __iomem *ioaddr = tp->mmio_addr;

	/* ISO spec, ETN_PHY_INTR,
	 * wait interrupt from PHY and it means MDIO is ready
	 */
	tmp = 0;
	while ((readl(tp->mmio_clkaddr + ISO_UMSK_ISR) & BIT(27)) == 0) {
		tmp += 1;
		mdelay(1);
		if (tmp >= 100) {
			pr_err("PHY PHY_Status timeout.\n");
			break;
		}
	}
	pr_info("wait %d ms for PHY interrupt. UMSK_ISR = 0x%x\n",
		tmp, readl(tp->mmio_clkaddr + ISO_UMSK_ISR));

	/* In ByPass mode,
	 * SW need to handle the EPHY Status check ,
	 * OTP data update and EPHY fuse_rdy setting.
	 */
	/* PHY will stay in state 1 mode */
	tmp = 0;
	while ((int_mdio_read(tp, 0x0a42, 16) & 0x07) != 0x1) {
		tmp += 1;
		mdelay(1);
		if (tmp >= 2000) {
			pr_err("PHY status is not 0x1 in bypass mode, current = 0x%02x\n",
			       int_mdio_read(tp, 0x0a42, 16) & 0x07);
			break;
		}
	}

	/* fill fuse_rdy & rg_ext_ini_done */
	int_mdio_write(tp, 0x0a46, 20,
		       int_mdio_read(tp, 0x0a46, 20) | (BIT(1) | BIT(0)));

	/* init_autoload_done = 1 */
	tmp = rtl_ocp_read(tp, 0xe004);
	tmp |= BIT(7);
	rtl_ocp_write(tp, 0xe004, tmp);

	/* ee_mode = 3 */
	RTL_W8(Cfg9346, Cfg9346_Unlock);

	/* wait LAN-ON */
	tmp = 0;
	do {
		tmp += 1;
		mdelay(1);
		if (tmp >= 2000) {
			pr_err("PHY status is not 0x3, current = 0x%02x\n",
			       int_mdio_read(tp, 0x0a42, 16) & 0x07);
			break;
		}
	} while ((int_mdio_read(tp, 0x0a42, 16) & 0x07) != 0x3);
	pr_info("wait %d ms for PHY ready, current = 0x%x\n",
		tmp, int_mdio_read(tp, 0x0a42, 16));

	/* adjust PHY SRAM table */
	r8169soc_phy_sram_table(tp);

	/* GPHY patch code */
	r8169soc_patch_gphy_uc_code(tp);

	/* adjust PHY electrical characteristics */
	r8169soc_phy_iol_tuning(tp);

	/* load OTP contents (RC-K, R-K, Amp-K, and Bias-K)
	 * RC-K:        0x980174F8[27:24]
	 * R-K:         0x98017500[18:15]
	 * Amp-K:       0x980174FC[15:0]
	 * Bias-K:      0x980174FC[31:16]
	 */
	r8169soc_load_otp_content(tp);

	if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
		/* Init PHY path */
		/* reg_0x9800705c[5] = 0 */
		/* reg_0x9800705c[7] = 0 */
		/* ISO spec, set internal MDIO to access PHY */
		tmp = readl(tp->mmio_clkaddr + ISO_PWRCUT_ETN);
		tmp &= ~(BIT(7) | BIT(5));
		writel(tmp, tp->mmio_clkaddr + ISO_PWRCUT_ETN);

		/* reg_0x9800705c[4] = 0 */
		/* ISO spec, set data path to access PHY */
		tmp = readl(tp->mmio_clkaddr + ISO_PWRCUT_ETN);
		tmp &= ~(BIT(4));
		writel(tmp, tp->mmio_clkaddr + ISO_PWRCUT_ETN);

		/* ETN spec, GMAC data path select MII-like(embedded GPHY),
		 * not SGMII(external PHY)
		 */
		tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(0) | BIT(1));
		tmp |= BIT(1);	/* MII */
		rtl_ocp_write(tp, 0xea34, tmp);
	} else {
		/* SGMII */
		/* ETN spec, adjust MDC freq=2.5MHz */
		tmp = rtl_ocp_read(tp, 0xDE30) & ~(BIT(7) | BIT(6));
		rtl_ocp_write(tp, 0xDE30, tmp);
		/* ETN spec, set external PHY addr */
		tmp = rtl_ocp_read(tp, 0xDE24) & ~(0x1f << 0);
		rtl_ocp_write(tp, 0xDE24, tmp | (tp->ext_phy_id & 0x1f));
		/* ISO mux spec, GPIO29 is set to MDC pin */
		/* ISO mux spec, GPIO46 is set to MDIO pin */
		p_sgmii_mdio = devm_pinctrl_get(&tp->pdev->dev);
		ps_sgmii_mdio = pinctrl_lookup_state(p_sgmii_mdio, "sgmii");
		pinctrl_select_state(p_sgmii_mdio, ps_sgmii_mdio);

		/* check if external PHY is available */
		pr_info("Searching external PHY...");
		tp->ext_phy = true;
		tmp = 0;
		while (ext_mdio_read(tp, 0x0a43, 31) != 0x0a43) {
			pr_info(".");
			tmp += 1;
			mdelay(1);
			if (tmp >= 2000) {
				pr_err("\n External SGMII PHY not found, current = 0x%02x\n",
				       ext_mdio_read(tp, 0x0a43, 31));
				return;
			}
		}
		if (tmp < 2000)
			pr_info("found.\n");

		/* lower SGMII TX swing of RTL8211FS to reduce EMI */
		/* TX swing = 470mV, default value */
		ext_mdio_write(tp, 0x0dcd, 16, 0x104e);

		tp->ext_phy = false;

		/* ETN spec, GMAC data path select SGMII(external PHY),
		 * not MII-like(embedded GPHY)
		 */
		tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(0) | BIT(1));
		tmp |= BIT(1) | BIT(0); /* SGMII */
		rtl_ocp_write(tp, 0xea34, tmp);

		if (r8169soc_serdes_init(tp) != 0)
			pr_err("SERDES init failed\n");
		/* ext_phy == true now */

		/* SDS spec, auto update SGMII link capability */
		tmp = readl(tp->mmio_sdsaddr + SDS_DEBUG);
		tmp |= BIT(2);
		writel(tmp, tp->mmio_sdsaddr + SDS_DEBUG);

		/* enable 8b/10b symbol error
		 * even it is only valid in 1000Mbps
		 */
		ext_mdio_write(tp, 0x0dcf, 16,
			       (ext_mdio_read(tp, 0x0dcf, 16) &
			       ~(BIT(2) | BIT(1) | BIT(0))));
		ext_mdio_read(tp, 0x0dcf, 17);	/* dummy read */
	}
}

static void r8169soc_eee_init(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
			/* 1000M/100M EEE capability */
			rtl_mmd_write(tp, 0x7, 60, (BIT(2) | BIT(1)));
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) | (BIT(4) | BIT(2)));
			/* disable dynamic RX power in PHY */
			rtl_phy_write(tp, 0x0bd0, 21,
				      (rtl_phy_read(tp, 0x0bd0, 21) & ~BIT(8)) | BIT(9));
		} else {	/* SGMII */
			/* 1000M & 100M EEE capability */
			rtl_mmd_write(tp, 0x7, 60, (BIT(2) | BIT(1)));
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) | (BIT(4) | BIT(2)));
		}
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) | (BIT(1) | BIT(0)));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) | BIT(1));
	} else {
		if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
			/* no EEE capability */
			rtl_mmd_write(tp, 0x7, 60, 0);
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) & ~(BIT(4) | BIT(2)));
		} else {	/* SGMII */
			/* no EEE capability */
			rtl_mmd_write(tp, 0x7, 60, 0);
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) & ~(BIT(4) | BIT(2)));
		}
		/* reset to default value */
		rtl_ocp_write(tp, 0xe040, rtl_ocp_read(tp, 0xe040) | BIT(13));
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) & ~(BIT(1) | BIT(0)));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080, (rtl_ocp_read(tp, 0xe080) & ~BIT(1)));
	}
}
#endif /* CONFIG_ARCH_RTD16xx */

#if defined(CONFIG_ARCH_RTD13xx)
static void r8169soc_reset_phy_gmac(struct rtl8169_private *tp)
{
	u32 tmp;
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");

	clk_prepare_enable(clk_etn_sys);
	clk_prepare_enable(clk_etn_250m);

	/* reg_0x9800708c[12:11] = 00 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_disable_unprepare(clk_etn_sys);
	clk_disable_unprepare(clk_etn_250m);

	/* reg_0x98007088[10:9] = 00 */
	/* ISO spec, rstn_gphy & rstn_gmac */
	reset_control_assert(rstc_gphy);
	reset_control_assert(rstc_gmac);

	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	tmp = readl(tp->mmio_clkaddr + ISO_ETN_TESTIO);
	tmp |= BIT(1);
	writel(tmp, tp->mmio_clkaddr + ISO_ETN_TESTIO);

	/* reg_0x9800705c[5] = 0 */
	/* ISO spec, default value and specify internal/external PHY ID as 1 */
	tmp = readl(tp->mmio_clkaddr + ISO_PWRCUT_ETN);
	tmp &= ~BIT(5);
	writel(tmp, tp->mmio_clkaddr + ISO_PWRCUT_ETN);
	/* set int PHY addr */
	__int_set_phy_addr(tp, INT_PHY_ADDR);
	/* set ext PHY addr */
	__ext_set_phy_addr(tp, INT_PHY_ADDR);

	mdelay(1);

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
}

static void r8169soc_acp_init(struct rtl8169_private *tp)
{
	u32 tmp;

	/* SBX spec, Select ETN access DDR path. */
	if (tp->acp_enable) {
		/* reg_0x9801c20c[6] = 1 */
		/* SBX spec, Mask ETN_ALL to SB3 DBUS REQ */
		tmp = readl(tp->mmio_sbxaddr + SBX_SB3_CHANNEL_REQ_MASK);
		tmp |= BIT(6);
		writel(tmp, tp->mmio_sbxaddr + SBX_SB3_CHANNEL_REQ_MASK);

		pr_info("wait all SB3 access finished...");
		tmp = 0;
		while (0 != (BIT(6) & readl(tp->mmio_sbxaddr + SBX_SB3_CHANNEL_REQ_BUSY))) {
			mdelay(10);
			tmp += 10;
			if (tmp >= 100) {
				pr_err("\n wait SB3 access failed (wait %d ms)\n", tmp);
				break;
			}
		}
		if (tmp < 100)
			pr_info("done.\n");

		/* reg_0x9801d100[29] = 0 */
		/* SCPU wrapper spec, CLKACP division, 0 = div 2, 1 = div 3 */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);
		tmp &= ~(BIT(29));
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);

		/* reg_0x9801d124[1:0] = 00 */
		/* SCPU wrapper spec, ACP master active, 0 = active */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_INTERFACE_EN);
		tmp &= ~(BIT(1) | BIT(0));
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_INTERFACE_EN);

		/* reg_0x9801d100[30] = 1 */
		/* SCPU wrapper spec, dy_icg_en_acp */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);
		tmp |= BIT(30);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);

		/* reg_0x9801d100[21] = 1 */
		/* SCPU wrapper spec, ACP CLK enable */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);
		tmp |= BIT(21);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);

		/* reg_0x9801d100[14] = 1 */
		/* Do not apply reset to ACP port axi3 master */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);
		tmp |= BIT(14);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);

		/* reg_0x9801d800[3:0] = 0111 */
		/* reg_0x9801d800[7:4] = 0111 */
		/* reg_0x9801d800[9:8] = 00 */
		/* reg_0x9801d800[20:16] = 01100 */
		/* reg_0x9801d800[28:24] = 01110 */
		/* Configure control of ACP port */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_ACP_CTRL);
		tmp &= ~((0x1f << 24) | (0x1f << 16) | (0x1 << 9) |
			(0xf << 4) | (0xf << 0));
		tmp |= (0x0e << 24) | (0x0c << 16) | (0x1 << 9) | (0x7 << 4) |
			(0x7 << 0);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_ACP_CTRL);

		/* reg_0x9801d030[28] = 1 */
		/* SCPU wrapper spec, dy_icg_en_acp */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);
		tmp |= BIT(28);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);

		/* reg_0x9801d030[16] = 1 */
		/* ACP CLK Enable for acp of scpu_chip_top */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);
		tmp |= BIT(16);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);

		/* reg_0x9801d030[0] = 1 */
		/* Do not apply reset to ACP port axi3 master */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);
		tmp |= BIT(0);
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);

		/* reg_0x9801c814[17] = 1 */
		/* through ACP to SCPU_ACP */
		tmp = readl(tp->mmio_sbxaddr + SBX_ACP_MISC_CTRL);
		tmp |= BIT(17);
		writel(tmp, tp->mmio_sbxaddr + SBX_ACP_MISC_CTRL);

		/* SBX spec, Remove mask ETN_ALL to ACP DBUS REQ */
		tmp = readl(tp->mmio_sbxaddr + SBX_ACP_CHANNEL_REQ_MASK);
		tmp &= ~BIT(1);
		writel(tmp, tp->mmio_sbxaddr + SBX_ACP_CHANNEL_REQ_MASK);

		pr_info("ARM ACP on\n.");
	} else {
		/* SBX spec, Mask ETN_ALL to ACP DBUS REQ */
		tmp = readl(tp->mmio_sbxaddr + SBX_ACP_CHANNEL_REQ_MASK);
		tmp |= BIT(1);
		writel(tmp, tp->mmio_sbxaddr + SBX_ACP_CHANNEL_REQ_MASK);

		pr_info("wait all ACP access finished...");
		tmp = 0;
		while (0 != (BIT(1) & readl(tp->mmio_sbxaddr + SBX_ACP_CHANNEL_REQ_BUSY))) {
			mdelay(1);
			tmp += 1;
			if (tmp >= 100) {
				pr_err("\n ACP channel is still busy (wait %d ms)\n", tmp);
				break;
			}
		}
		if (tmp < 100)
			pr_info("done.\n");

		/* SCPU wrapper spec, Inactive MP4 AINACTS signal */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_INTERFACE_EN);
		tmp |= (BIT(1) | BIT(0));
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_INTERFACE_EN);

		/* SCPU wrapper spec, nACPRESET_DVFS & CLKENACP_DVFS */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);
		tmp &= ~(BIT(21) | BIT(14));
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_CRT_CTRL);

		/* SCPU wrapper spec, nACPRESET & CLKENACP */
		tmp = readl(tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);
		tmp &= ~(BIT(16) | BIT(0));
		writel(tmp, tp->mmio_sbxaddr + SC_WRAP_ACP_CRT_CTRL);

		/* reg_0x9801c814[17] = 0 */
		/* SBX spec, Switch ETN_ALL to DC_SYS path */
		tmp = readl(tp->mmio_sbxaddr + SBX_ACP_MISC_CTRL);
		tmp &= ~BIT(17);
		writel(tmp, tp->mmio_sbxaddr + SBX_ACP_MISC_CTRL);

		/* SBX spec, Remove mask ETN_ALL to SB3 DBUS REQ */
		tmp = readl(tp->mmio_sbxaddr + SBX_SB3_CHANNEL_REQ_MASK);
		tmp &= ~BIT(6);
		writel(tmp, tp->mmio_sbxaddr + SBX_SB3_CHANNEL_REQ_MASK);

		pr_info("ARM ACP off\n.");
	}
}

static void r8169soc_pll_clock_init(struct rtl8169_private *tp)
{
	u32 tmp;
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");

	/* reg_0x98007004[27] = 1 */
	/* ISO spec, ETN_PHY_INTR, clear ETN interrupt for ByPassMode */
	writel(BIT(27), tp->mmio_clkaddr + ISO_UMSK_ISR);

	/* reg_0x98007088[10] = 1 */
	/* ISO spec, reset bit of fephy */
	reset_control_deassert(rstc_gphy);

	mdelay(1);	/* wait 1ms for PHY PLL stable */

	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	tmp = readl(tp->mmio_clkaddr + ISO_ETN_TESTIO);
	tmp |= BIT(1);
	writel(tmp, tp->mmio_clkaddr + ISO_ETN_TESTIO);

	/* reg_0x98007088[9] = 1 */
	/* ISO spec, reset bit of gmac */
	reset_control_deassert(rstc_gmac);

	/* reg_0x9800708c[12:11] = 11 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_prepare_enable(clk_etn_sys);
	clk_prepare_enable(clk_etn_250m);

	mdelay(10);	/* wait 10ms for GMAC uC to be stable */

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
}

static void r8169soc_load_otp_content(struct rtl8169_private *tp)
{
	u32 otp;
	u16 tmp;
	u8 *buf;

	buf = r8169soc_read_otp(tp, "para");
	if (IS_ERR(buf))
		goto set_idac;

	/* enable 0x980174FC[4] */
	/* R-K 0x980174FC[3:0] */
	otp = *buf;
	if (otp & BIT(4)) {
		tmp = otp ^ R_K_DEFAULT;
		int_mdio_write(tp, 0x0bc0, 20,
			       (int_mdio_read(tp, 0x0bc0, 20) & ~(0x1f << 0)) | tmp);
	}

	kfree(buf);

set_idac:

	buf = r8169soc_read_otp(tp, "idac");
	if (IS_ERR(buf))
		return;

	/* Amp-K 0x98017500[7:0] */
	otp = *buf;
	tmp = otp ^ IDAC_FINE_DEFAULT;
	int_mdio_write(tp, 0x0bc0, 23,
		       (int_mdio_read(tp, 0x0bc0, 23) & ~(0xff << 0)) | tmp);

	kfree(buf);
}

static void r8169soc_phy_iol_tuning(struct rtl8169_private *tp)
{
	u16 tmp;

	/* rs_offset=rsw_offset=0xc */
	tmp = 0xcc << 8;
	int_mdio_write(tp, 0x0bc0, 20,
		       (int_mdio_read(tp, 0x0bc0, 20) & ~(0xff << 8)) | tmp);

	/* 100M Tr/Tf */
	/* reg_cf_l=0x2 */
	tmp = 0x2 << 11;
	int_mdio_write(tp, 0x0bd0, 17,
		       (int_mdio_read(tp, 0x0bd0, 17) & ~(0x7 << 11)) | tmp);

	/* 100M Swing */
	/* idac_fine_mdix, idac_fine_mdi */
	tmp = 0x55 << 0;
	int_mdio_write(tp, 0x0bc0, 23,
		       (int_mdio_read(tp, 0x0bc0, 23) & ~(0xff << 0)) | tmp);
}

static void r8169soc_mdio_init(struct rtl8169_private *tp)
{
	u32 tmp;
	struct pinctrl *p_rgmii_mdio;
	struct pinctrl_state *ps_rgmii_mdio;
	void __iomem *ioaddr = tp->mmio_addr;

	/* ISO spec, ETN_PHY_INTR, wait interrupt from PHY and it means MDIO is ready */
	tmp = 0;
	while ((readl(tp->mmio_clkaddr + ISO_UMSK_ISR) & BIT(27)) == 0) {
		tmp += 1;
		mdelay(1);
		if (tmp >= 100) {
			pr_err("PHY PHY_Status timeout.\n");
			break;
		}
	}
	pr_info("wait %d ms for PHY interrupt. UMSK_ISR = 0x%x\n",
		tmp, readl(tp->mmio_clkaddr + ISO_UMSK_ISR));

	/* In ByPass mode,
	 * SW need to handle the EPHY Status check ,
	 * OTP data update and EPHY fuse_rdy setting.
	 */
	/* PHY will stay in state 1 mode */
	tmp = 0;
	while (0x1 != (int_mdio_read(tp, 0x0a42, 16) & 0x07)) {
		tmp += 1;
		mdelay(1);
		if (tmp >= 2000) {
			pr_err("PHY status is not 0x1 in bypass mode, current = 0x%02x\n",
			       (int_mdio_read(tp, 0x0a42, 16) & 0x07));
			break;
		}
	}

	/* fix A00 known issue */
	if (get_rtd_chip_revision() == RTD_CHIP_A00) {
		/* fix AFE RX power as 1 */
		int_mdio_write(tp, 0x0bd0, 21, 0x7201);
	}

	/* ETN spec. set MDC freq = 8.9MHZ */
	tmp = rtl_ocp_read(tp, 0xde10) & ~(BIT(7) | BIT(6));
	tmp |= BIT(6); /* MDC freq = 8.9MHz */
	rtl_ocp_write(tp, 0xde10, tmp);

	/* fill fuse_rdy & rg_ext_ini_done */
	int_mdio_write(tp, 0x0a46, 20,
		       int_mdio_read(tp, 0x0a46, 20) | (BIT(1) | BIT(0)));

	/* init_autoload_done = 1 */
	tmp = rtl_ocp_read(tp, 0xe004);
	tmp |= BIT(7);
	rtl_ocp_write(tp, 0xe004, tmp);

	/* ee_mode = 3 */
	RTL_W8(Cfg9346, Cfg9346_Unlock);

	/* wait LAN-ON */
	tmp = 0;
	do {
		tmp += 1;
		mdelay(1);
		if (tmp >= 2000) {
			pr_err("PHY status is not 0x3, current = 0x%02x\n",
			       (int_mdio_read(tp, 0x0a42, 16) & 0x07));
			break;
		}
	} while (0x3 != (int_mdio_read(tp, 0x0a42, 16) & 0x07));
	pr_info("wait %d ms for PHY ready, current = 0x%x\n",
		tmp, int_mdio_read(tp, 0x0a42, 16));

	/* adjust PHY electrical characteristics */
	r8169soc_phy_iol_tuning(tp);

	/* load OTP contents (R-K, Amp)
	 * R-K:		0x980174FC[4:0]
	 * Amp:		0x98017500[7:0]
	 */
	r8169soc_load_otp_content(tp);

	switch (tp->output_mode) {
	case OUTPUT_EMBEDDED_PHY:
		/* ISO spec, set data path to FEPHY */
		tmp = rtl_ocp_read(tp, 0xea30);
		tmp &= ~(BIT(0));
		rtl_ocp_write(tp, 0xea30, tmp);

		tmp = rtl_ocp_read(tp, 0xea34);
		tmp &= ~(BIT(1) | BIT(0));
		tmp |= BIT(1);  /* FEPHY */
		rtl_ocp_write(tp, 0xea34, tmp);

		/* LED high active circuit */
		/* output current: 4mA */
		tmp = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC9);
		tmp &= ~(0xff << 16);
		writel(tmp, tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC9);
		break;
	case OUTPUT_RGMII_TO_MAC:
		pr_err("%s:%d: FIXME! PHY_TYPE_RGMII_MAC\n", __func__, __LINE__);
		break;
	case OUTPUT_RMII:
		pr_err("%s:%d: FIXME! PHY_TYPE_RMII\n", __func__, __LINE__);
		break;
	case OUTPUT_RGMII_TO_PHY:
		/* ISO mux spec, GPIO15 is set to MDC pin */
		/* ISO mux spec, GPIO14 is set to MDIO pin */
		/* ISO mux spec, GPIO65~76 is set to TX/RX pin */
		p_rgmii_mdio = devm_pinctrl_get(&tp->pdev->dev);
		ps_rgmii_mdio = pinctrl_lookup_state(p_rgmii_mdio, "rgmii");
		pinctrl_select_state(p_rgmii_mdio, ps_rgmii_mdio);

		/* no Schmitt_Trigger */
		switch (tp->voltage) {
		case 3: /* 3.3v */
			/* MODE2=1, MODE=0, SR=1, smt=0, pudsel=0, puden=0, E2=0 */
			/* GPIO[70:65] */
			tmp = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC20);
			tmp &= ~((0xf << 24) | (0xf << 20) | (0x1f << 16) |
				(0x1f << 12) | (0x1f << 8) | (0x1f << 4));
			tmp |= (0x0 << 24) | (0x0 << 20) | (0x0 << 16) |
				(0x0 << 12) | (0x0 << 8) | (0x0 << 4);
			writel(tmp, tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC20);

			/* GPIO[76:71] */
			tmp = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC21);
			tmp &= ~((0x3 << 30) | (0x1f << 25) | (0x1f << 20) |
				(0x1f << 15) | (0x1f << 10) | (0x1f << 5) |
				(0x1f << 0));
			tmp |= (0x2 << 30) | (0x10 << 25) | (0x10 << 20) |
				(0x10 << 15) | (0x10 << 10) | (0x10 << 5) |
				(0x10 << 0);
			writel(tmp, tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC21);

			/* DP=000, DN=111 */
			tmp = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC25);
			tmp &= ~((0x7 << 3) | (0x7 << 0));
			tmp |= (0x0 << 3) | (0x7 << 0);
			writel(tmp, tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC25);
			break;
		case 2: /* 2.5v */
			/* MODE2=0, MODE=1, SR=0, smt=0, pudsel=0, puden=0,
			 * E2=1
			 */
			/* GPIO[70:65] */
			tmp = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC20);
			tmp &= ~((0xf << 24) | (0xf << 20) | (0x1f << 16) |
				(0x1f << 12) | (0x1f << 8) | (0x1f << 4));
			tmp |= (0x1 << 24) | (0x1 << 20) | (0x1 << 16) |
				(0x1 << 12) | (0x1 << 8) | (0x1 << 4);
			writel(tmp, tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC20);

			/* GPIO[76:71] */
			tmp = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC21);
			tmp &= ~((0x3 << 30) | (0x1f << 25) | (0x1f << 20) |
				(0x1f << 15) | (0x1f << 10) | (0x1f << 5) |
				(0x1f << 0));
			tmp |= (0x1 << 30) | (0x1 << 25) | (0x1 << 20) |
				(0x1 << 15) | (0x1 << 10) | (0x1 << 5) |
				(0x1 << 0);
			writel(tmp, tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC21);

			/* DP=000, DN=111 */
			tmp = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC25);
			tmp &= ~((0x7 << 3) | (0x7 << 0));
			tmp |= (0x0 << 3) | (0x7 << 0);
			writel(tmp, tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC25);
			break;
		case 1: /* 1.8v */
		default:
			/* MODE2=0, MODE=0, SR=0, smt=0, pudsel=0, puden=0,
			 * E2=1
			 */
			/* GPIO[70:65] */
			tmp = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC20);
			tmp &= ~((0xf << 24) | (0xf << 20) | (0x1f << 16) |
				(0x1f << 12) | (0x1f << 8) | (0x1f << 4));
			tmp |= (0x1 << 24) | (0x1 << 20) | (0x1 << 16) |
				(0x1 << 12) | (0x1 << 8) | (0x1 << 4);
			writel(tmp, tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC20);

			/* GPIO[76:71] */
			tmp = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC21);
			tmp &= ~((0x3 << 30) | (0x1f << 25) | (0x1f << 20) |
				(0x1f << 15) | (0x1f << 10) | (0x1f << 5) |
				(0x1f << 0));
			tmp |= (0x0 << 30) | (0x1 << 25) | (0x1 << 20) |
				(0x1 << 15) | (0x1 << 10) | (0x1 << 5) |
				(0x1 << 0);
			writel(tmp, tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC21);

			/* DP=001, DN=110 */
			tmp = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC25);
			tmp &= ~((0x7 << 3) | (0x7 << 0));
			tmp |= (0x1 << 3) | (0x6 << 0);
			writel(tmp, tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC25);
		}

		/* check if external PHY is available */
		pr_info("Searching external PHY...");
		tp->ext_phy = true;
		tmp = 0;
		while (ext_mdio_read(tp, 0x0a43, 31) != 0x0a43) {
			pr_info(".");
			tmp += 1;
			mdelay(1);
			if (tmp >= 2000) {
				pr_err("\n External RGMII PHY not found, current = 0x%02x\n",
				       ext_mdio_read(tp, 0x0a43, 31));
				return;
			}
		}
		if (tmp < 2000)
			pr_info("found.\n");

		/* set RGMII RX delay */
		tmp = rtl_ocp_read(tp, 0xea34) &
			~(BIT(14) | BIT(9) | BIT(8) | BIT(6));
		switch (tp->rx_delay) {
		case 1:
			tmp |= (0x0 << 6) | (0x1 << 8);
			break;
		case 2:
			tmp |= (0x0 << 6) | (0x2 << 8);
			break;
		case 3:
			tmp |= (0x0 << 6) | (0x3 << 8);
			break;
		case 4:
			tmp |= (0x1 << 6) | (0x0 << 8);
			break;
		case 5:
			tmp |= (0x1 << 6) | (0x1 << 8);
			break;
		case 6:
			tmp |= (0x1 << 6) | (0x2 << 8);
			break;
		case 7:
			tmp |= (0x1 << 6) | (0x3 << 8);
		}
		rtl_ocp_write(tp, 0xea34, tmp);

		tmp = rtl_ocp_read(tp, 0xea36) & ~(BIT(0)); /* rg_clk_en */
		rtl_ocp_write(tp, 0xea36, tmp);

		/* external PHY RGMII timing tuning, rg_rgmii_id_mode = 1 (default) */
		tmp = ext_mdio_read(tp, 0x0d08, 21);
		switch (tp->rx_delay) {
		case 0:
		case 1:
		case 2:
		case 3:
			tmp |= BIT(3);
			break;
		case 4:
		case 5:
		case 6:
		case 7:
			tmp &= ~BIT(3);
		}
		ext_mdio_write(tp, 0x0d08, 21, tmp);

		/* set RGMII TX delay */
		if (tp->tx_delay == 0) {
			tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(7));
			rtl_ocp_write(tp, 0xea34, tmp);

			/* external PHY RGMII timing tuning, tx_dly_mode = 1 */
			ext_mdio_write(tp, 0x0d08, 17,
				       ext_mdio_read(tp, 0x0d08, 17) | BIT(8));
		} else {	/* tp->tx_delay == 1 (2ns) */
			tmp = rtl_ocp_read(tp, 0xea34) | (BIT(7));
			rtl_ocp_write(tp, 0xea34, tmp);

			/* external PHY RGMII timing tuning, tx_dly_mode = 0 */
			ext_mdio_write(tp, 0x0d08, 17,
				       ext_mdio_read(tp, 0x0d08, 17) & ~BIT(8));
		}

		/* ISO spec, data path is set to RGMII */
		tmp = rtl_ocp_read(tp, 0xea30) & ~(BIT(0));
		rtl_ocp_write(tp, 0xea30, tmp);

		tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(0) | BIT(1));
		tmp |= BIT(0); /* RGMII */
		rtl_ocp_write(tp, 0xea34, tmp);

		/* ext_phy == true now */

		break;
	default:
		pr_err("%s:%d: unsupport output mode (%d) in FPGA\n",
		       __func__, __LINE__, tp->output_mode);
	}
}

static void r8169soc_eee_init(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
			/* enable eee auto fallback function */
			rtl_phy_write(tp, 0x0a4b, 17,
				      rtl_phy_read(tp, 0x0a4b, 17) | BIT(2));
			/* 1000M/100M EEE capability */
			rtl_mmd_write(tp, 0x7, 60, (BIT(2) | BIT(1)));
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) | BIT(4) | BIT(2));
			/* keep RXC in LPI mode */
			rtl_mmd_write(tp, 0x3, 0,
				      rtl_mmd_read(tp, 0x3, 0) & ~BIT(10));
		} else { /* RGMII */
			/* enable eee auto fallback function */
			rtl_phy_write(tp, 0x0a4b, 17,
				      rtl_phy_read(tp, 0x0a4b, 17) | BIT(2));
			/* 1000M & 100M EEE capability */
			rtl_mmd_write(tp, 0x7, 60, (BIT(2) | BIT(1)));
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) | BIT(4) | BIT(2));
			/* stop RXC in LPI mode */
			rtl_mmd_write(tp, 0x3, 0,
				      rtl_mmd_read(tp, 0x3, 0) | BIT(10));
		}
		/* EEE Tw_sys_tx timing adjustment,
		 * make sure MAC would send data after FEPHY wake up
		 */
		/* default 0x001F, 100M EEE */
		rtl_ocp_write(tp, 0xe04a, 0x002f);
		/* default 0x0011, 1000M EEE */
		rtl_ocp_write(tp, 0xe04c, 0x001a);
		/* EEEP Tw_sys_tx timing adjustment,
		 * make sure MAC would send data after FEPHY wake up
		 */
		 /* default 0x3f, 10M EEEP */
		if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
			/* FEPHY needs 160us */
			rtl_ocp_write(tp, 0xe08a, 0x00a8);
		} else {	/* RGMII, GPHY needs 25us */
			rtl_ocp_write(tp, 0xe08a, 0x0020);
		}
		/* default 0x0005, 100M/1000M EEEP */
		rtl_ocp_write(tp, 0xe08c, 0x0008);
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) | BIT(1) | BIT(0));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) | BIT(1));
	} else {
		if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
			/* disable eee auto fallback function */
			rtl_phy_write(tp, 0x0a4b, 17,
				      rtl_phy_read(tp, 0x0a4b, 17) & ~BIT(2));
			/* 100M & 1000M EEE capability */
			rtl_mmd_write(tp, 0x7, 60, 0);
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) & ~(BIT(4) | BIT(2)));
			/* keep RXC in LPI mode */
			rtl_mmd_write(tp, 0x3, 0,
				      rtl_mmd_read(tp, 0x3, 0) & ~BIT(10));
		} else { /* RGMII */
			/* disable eee auto fallback function */
			rtl_phy_write(tp, 0x0a4b, 17,
				      rtl_phy_read(tp, 0x0a4b, 17) & ~BIT(2));
			/* 100M & 1000M EEE capability */
			rtl_mmd_write(tp, 0x7, 60, 0);
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) & ~(BIT(4) | BIT(2)));
			/* keep RXC in LPI mode */
			rtl_mmd_write(tp, 0x3, 0,
				      rtl_mmd_read(tp, 0x3, 0) & ~BIT(10));
		}
		/* reset to default value */
		rtl_ocp_write(tp, 0xe040, rtl_ocp_read(tp, 0xe040) | BIT(13));
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) & ~(BIT(1) | BIT(0)));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) & ~BIT(1));
		/* EEE Tw_sys_tx timing restore */
		/* default 0x001F, 100M EEE */
		rtl_ocp_write(tp, 0xe04a, 0x001f);
		/* default 0x0011, 1000M EEE */
		rtl_ocp_write(tp, 0xe04c, 0x0011);
		/* EEEP Tw_sys_tx timing restore */
		/* default 0x3f, 10M EEEP */
		rtl_ocp_write(tp, 0xe08a, 0x003f);
		/* default 0x0005, 100M/1000M EEEP */
		rtl_ocp_write(tp, 0xe08c, 0x0005);
	}
}
#endif /* CONFIG_ARCH_RTD13xx */

#if defined(CONFIG_ARCH_RTD16XXB)
static void r8169soc_reset_phy_gmac(struct rtl8169_private *tp)
{
	u32 tmp;
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");

	clk_prepare_enable(clk_etn_sys);
	clk_prepare_enable(clk_etn_250m);

	/* reg_0x9800708c[12:11] = 00 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_disable_unprepare(clk_etn_sys);
	clk_disable_unprepare(clk_etn_250m);

	/* reg_0x98007088[10:9] = 00 */
	/* ISO spec, rstn_gphy & rstn_gmac */
	reset_control_assert(rstc_gphy);
	reset_control_assert(rstc_gmac);

	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	tmp = readl(tp->mmio_clkaddr + ISO_ETN_TESTIO);
	tmp |= BIT(1);
	writel(tmp, tp->mmio_clkaddr + ISO_ETN_TESTIO);

	/* reg_0x9800705c[5] = 0 */
	/* ISO spec, default value and specify internal/external PHY ID as 1 */
	tmp = readl(tp->mmio_clkaddr + ISO_PWRCUT_ETN);
	tmp &= ~BIT(5);
	writel(tmp, tp->mmio_clkaddr + ISO_PWRCUT_ETN);
	/* set int PHY addr */
	__int_set_phy_addr(tp, INT_PHY_ADDR);
	/* set ext PHY addr */
	__ext_set_phy_addr(tp, INT_PHY_ADDR);

	mdelay(1);

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
}

static void r8169soc_acp_init(struct rtl8169_private *tp)
{
}

static void r8169soc_pll_clock_init(struct rtl8169_private *tp)
{
	u32 tmp;
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");

	/* reg_0x98007004[27] = 1 */
	/* ISO spec, ETN_PHY_INTR, clear ETN interrupt for ByPassMode */
	writel(BIT(27), tp->mmio_clkaddr + ISO_UMSK_ISR);

	/* reg_0x98007088[10] = 1 */
	/* ISO spec, reset bit of fephy */
	reset_control_deassert(rstc_gphy);

	mdelay(1);	/* wait 1ms for PHY PLL stable */

	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	tmp = readl(tp->mmio_clkaddr + ISO_ETN_TESTIO);
	tmp |= BIT(1);
	writel(tmp, tp->mmio_clkaddr + ISO_ETN_TESTIO);

	/* reg_0x98007088[9] = 1 */
	/* ISO spec, reset bit of gmac */
	reset_control_deassert(rstc_gmac);

	/* reg_0x9800708c[12:11] = 11 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_prepare_enable(clk_etn_sys);
	clk_prepare_enable(clk_etn_250m);

	mdelay(10);	/* wait 10ms for GMAC uC to be stable */

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
}

static void r8169soc_load_otp_content(struct rtl8169_private *tp)
{
}

static void r8169soc_phy_iol_tuning(struct rtl8169_private *tp)
{
	u16 tmp;

	tmp = 0xb4 << 4;
	int_mdio_write(tp, 0x0bc0, 17,
		       (int_mdio_read(tp, 0x0bc0, 17) & ~(0xff << 4)) | tmp);

	/* for 10M EEE differential voltage */
	/* Set LDVDC_L to 0x4 */
	tmp = 0x4 << 13;
	int_mdio_write(tp, 0x0bc0, 19,
		       (int_mdio_read(tp, 0x0bc0, 19) & ~(0x7 << 13)) | tmp);

	/* Change green table 10M LDVBIAS to 0x44 */
	tmp = 0x44 << 8;
	int_mdio_write(tp, 0x0a43, 27, 0x8050);
	int_mdio_write(tp, 0x0a43, 28,
		       (int_mdio_read(tp, 0x0a43, 28) & ~(0xff << 8)) | tmp);

	/* Change green table 100M short LDVBIAS to 0x44 */
	int_mdio_write(tp, 0x0a43, 27, 0x8057);
	int_mdio_write(tp, 0x0a43, 28,
		       (int_mdio_read(tp, 0x0a43, 28) & ~(0xff << 8)) | tmp);

	/* Change green table GIGA short LDVBIAS to 0x44 */
	int_mdio_write(tp, 0x0a43, 27, 0x8065);
	int_mdio_write(tp, 0x0a43, 28,
		       (int_mdio_read(tp, 0x0a43, 28) & ~(0xff << 8)) | tmp);

	/* Enable GDAC_IB_UP_TM to gain 6% amp for 10/100M */
	tmp = 0x1 << 7;
	int_mdio_write(tp, 0x0bcc, 21,
		       int_mdio_read(tp, 0x0bcc, 21) | tmp);
}

static void r8169soc_phy_sram_table(struct rtl8169_private *tp)
{
}

static void r8169soc_patch_phy_uc_code(struct rtl8169_private *tp)
{
}

static void r8169soc_mdio_init(struct rtl8169_private *tp)
{
	u32 tmp;
	void __iomem *ioaddr = tp->mmio_addr;

	mdelay(10);     /* wait for MDIO ready */

	/* PHY will stay in state 1 mode */
	tmp = 0;
	while (0x1 != (int_mdio_read(tp, 0x0a42, 16) & 0x07)) {
		tmp += 1;
		mdelay(1);
		if (tmp >= 2000) {
			pr_err("PHY status is not 0x1 in bypass mode, current = 0x%02x\n",
			       (int_mdio_read(tp, 0x0a42, 16) & 0x07));
			break;
		}
	}

	/* ETN spec. set MDC freq = 8.9MHZ */
	tmp = rtl_ocp_read(tp, 0xde10) & ~(BIT(7) | BIT(6));
	tmp |= BIT(6); /* MDC freq = 8.9MHz */
	rtl_ocp_write(tp, 0xde10, tmp);

	/* fill fuse_rdy & rg_ext_ini_done */
	int_mdio_write(tp, 0x0a46, 20,
		       int_mdio_read(tp, 0x0a46, 20) | (BIT(1) | BIT(0)));

	/* init_autoload_done = 1 */
	tmp = rtl_ocp_read(tp, 0xe004);
	tmp |= BIT(7);
	rtl_ocp_write(tp, 0xe004, tmp);

	/* ee_mode = 3 */
	RTL_W8(Cfg9346, Cfg9346_Unlock);

	/* wait LAN-ON */
	tmp = 0;
	do {
		tmp += 1;
		mdelay(1);
		if (tmp >= 2000) {
			pr_err("PHY status is not 0x3, current = 0x%02x\n",
			       (int_mdio_read(tp, 0x0a42, 16) & 0x07));
			break;
		}
	} while (0x3 != (int_mdio_read(tp, 0x0a42, 16) & 0x07));
	pr_info("wait %d ms for PHY ready, current = 0x%x\n",
		tmp, int_mdio_read(tp, 0x0a42, 16));

	/* adjust PHY SRAM table */
	r8169soc_phy_sram_table(tp);

	/* PHY patch code */
	r8169soc_patch_phy_uc_code(tp);

	/* adjust PHY electrical characteristics */
	r8169soc_phy_iol_tuning(tp);

	/* load OTP contents (R-K, Amp)
	 */
	r8169soc_load_otp_content(tp);

	switch (tp->output_mode) {
	case OUTPUT_EMBEDDED_PHY:
		/* ISO spec, set data path to FEPHY */
		tmp = rtl_ocp_read(tp, 0xea30);
		tmp &= ~(BIT(0));
		rtl_ocp_write(tp, 0xea30, tmp);

		tmp = rtl_ocp_read(tp, 0xea34);
		tmp &= ~(BIT(1) | BIT(0));
		tmp |= BIT(1);  /* FEPHY */
		rtl_ocp_write(tp, 0xea34, tmp);

		/* LED low active circuit */
		/* output current: 4mA */
		tmp = readl(tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC12);
		tmp &= ~GENMASK(14, 0);
		tmp |= (0x16 << 10) | (0x16 << 5) | (0x16 << 0);
		writel(tmp, tp->mmio_pinmuxaddr + ISO_TESTMUX_PFUNC12);
		break;
	default:
		pr_err("%s:%d: unsupport output mode (%d)\n",
		       __func__, __LINE__, tp->output_mode);
	}
}

static void r8169soc_eee_init(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		/* enable eee auto fallback function */
		rtl_phy_write(tp, 0x0a4b, 17,
			      rtl_phy_read(tp, 0x0a4b, 17) | BIT(2));
		/* 1000M/100M EEE capability */
		rtl_mmd_write(tp, 0x7, 60, (BIT(2) | BIT(1)));
		/* 10M EEE */
		rtl_phy_write(tp, 0x0a43, 25,
			      rtl_phy_read(tp, 0x0a43, 25) | (BIT(4) | BIT(2)));
		/* keep RXC in LPI mode */
		rtl_mmd_write(tp, 0x3, 0,
			      rtl_mmd_read(tp, 0x3, 0) & ~BIT(10));
		/* EEE Tw_sys_tx timing adjustment,
		 * make sure MAC would send data after FEPHY wake up
		 */
		/* default 0x001F, 100M EEE */
		rtl_ocp_write(tp, 0xe04a, 0x002f);
		/* default 0x0011, 1000M EEE */
		rtl_ocp_write(tp, 0xe04c, 0x001a);
		/* EEEP Tw_sys_tx timing adjustment,
		 * make sure MAC would send data after FEPHY wake up
		 */
		 /* default 0x3f, 10M EEEP */
		/* FEPHY needs 160us */
		rtl_ocp_write(tp, 0xe08a, 0x00a8);
		/* default 0x0005, 100M/1000M EEEP */
		rtl_ocp_write(tp, 0xe08c, 0x0008);
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      (rtl_ocp_read(tp, 0xe040) | BIT(1) | BIT(0)));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080,
			      (rtl_ocp_read(tp, 0xe080) | BIT(1)));
	} else {
		/* disable eee auto fallback function */
		rtl_phy_write(tp, 0x0a4b, 17,
			      rtl_phy_read(tp, 0x0a4b, 17) & ~BIT(2));
		/* 100M & 1000M EEE capability */
		rtl_mmd_write(tp, 0x7, 60, 0);
		/* 10M EEE */
		rtl_phy_write(tp, 0x0a43, 25,
			      rtl_phy_read(tp, 0x0a43, 25) & ~(BIT(4) | BIT(2)));
		/* keep RXC in LPI mode */
		rtl_mmd_write(tp, 0x3, 0,
			      rtl_mmd_read(tp, 0x3, 0) & ~BIT(10));
		/* reset to default value */
		rtl_ocp_write(tp, 0xe040,
			      (rtl_ocp_read(tp, 0xe040) | BIT(13)));
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      (rtl_ocp_read(tp, 0xe040) & ~(BIT(1) | BIT(0))));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080,
			      (rtl_ocp_read(tp, 0xe080) & ~BIT(1)));
		/* EEE Tw_sys_tx timing restore */
		/* default 0x001F, 100M EEE */
		rtl_ocp_write(tp, 0xe04a, 0x001f);
		/* default 0x0011, 1000M EEE */
		rtl_ocp_write(tp, 0xe04c, 0x0011);
		/* EEEP Tw_sys_tx timing restore */
		/* default 0x3f, 10M EEEP */
		rtl_ocp_write(tp, 0xe08a, 0x003f);
		/* default 0x0005, 100M/1000M EEEP */
		rtl_ocp_write(tp, 0xe08c, 0x0005);
	}
}
#endif /* CONFIG_ARCH_RTD16XXB */

static int
rtl_init_one(struct platform_device *pdev)
{
	struct rtl_cfg_info *cfg;
	struct rtl8169_private *tp;
	struct mii_if_info *mii;
	struct net_device *ndev;
	void __iomem *ioaddr;
	void __iomem *clkaddr;
	int chipset, i;
	int rc;
	int rtl_config;
	int mac_version;
	int led_config;
	u32 tmp;
	int irq;
	int retry;
	const char *mac_addr;
	struct property *wake_mask;
	char tmp_str[80];
	struct device_node *phy_0;
	u32 phy_irq_num = 0;
	u32 phy_irq_mask = 0;
	u32 phy_irq[POR_NUM] = {0, 0, 0};
	u8 phy_irq_map[POR_NUM + 1] = {0, 0, 0, 0};
	u32 phy_por_xv_mask;
	static char phy_irq_name[POR_NUM][IFNAMSIZ];
#ifdef RTL_PROC
	struct proc_dir_entry *dir_dev = NULL;
	struct proc_dir_entry *entry = NULL;
#endif
	#if defined(CONFIG_ARCH_RTD129x)
	struct clk *clk_etn_sys  = clk_get(&pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy = reset_control_get_exclusive(&pdev->dev, "gphy");
	struct reset_control *rstc_gmac = reset_control_get_exclusive(&pdev->dev, "gmac");
	u32 rgmii_voltage = 1;
	u32 rgmii_tx_delay = 0;
	u32 rgmii_rx_delay = 0;
	#elif defined(CONFIG_ARCH_RTD139x) || defined(CONFIG_ARCH_RTD16xx)
	void __iomem *sbxaddr;
	void __iomem *otpaddr;
	void __iomem *sdsaddr;
	void __iomem *pinmuxaddr;
	u32 bypass_enable = 0;
	u32 acp_enable = 0;
	#if defined(CONFIG_ARCH_RTD16xx)
	u32 sgmii_swing = 0;
	#endif /* CONFIG_ARCH_RTD16xx */
	#elif defined(CONFIG_ARCH_RTD13xx)
	void __iomem *sbxaddr;
	void __iomem *otpaddr;
	void __iomem *pinmuxaddr;
	u32 bypass_enable = 0;
	u32 acp_enable = 0;
	u32 voltage = 1;
	u32 tx_delay = 0;
	u32 rx_delay = 0;
	#elif defined(CONFIG_ARCH_RTD16XXB)
	void __iomem *pinmuxaddr;
	#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x |
		* CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD13xx |
		* CONFIG_ARCH_RTD16XXB
		*/
	u32 output_mode = 0;
	u32 ext_phy_id = 1;
	u32 eee_enable = 0;
	u32 wol_enable = 0;

	if (netif_msg_drv(&debug)) {
		pr_info("%s Gigabit Ethernet driver %s loaded\n",
			MODULENAME, RTL8169_VERSION);
	}

	if (of_property_read_u32(pdev->dev.of_node, "rtl-config", &rtl_config))
		dprintk("%s canot specified config", __func__);
	cfg = &rtl_cfg_infos;
	if (of_property_read_u32(pdev->dev.of_node, "mac-version", &mac_version))
		dprintk("%s canot specified config", __func__);
	if (of_property_read_u32(pdev->dev.of_node, "led-cfg", &led_config))
		led_config = 0;
	if (of_property_read_u32(pdev->dev.of_node, "output-mode", &output_mode))
		dprintk("%s can't get output mode", __func__);
	if (of_property_read_u32(pdev->dev.of_node, "ext-phy-id", &ext_phy_id))
		dprintk("%s can't get RGMII external PHY ID", __func__);
	if (of_property_read_u32(pdev->dev.of_node, "eee", &eee_enable))
		dprintk("%s can't get eee_enable", __func__);
	if (of_property_read_u32(pdev->dev.of_node, "wol-enable", &wol_enable))
		dprintk("%s can't get wol_enable", __func__);

	#if defined(CONFIG_ARCH_RTD129x)
	if (of_property_read_u32(pdev->dev.of_node, "rgmii-voltage", &rgmii_voltage))
		dprintk("%s can't get RGMII voltage", __func__);
	if (of_property_read_u32(pdev->dev.of_node, "rgmii-tx-delay", &rgmii_tx_delay))
		dprintk("%s can't get RGMII TX delay", __func__);
	if (of_property_read_u32(pdev->dev.of_node, "rgmii-rx-delay", &rgmii_rx_delay))
		dprintk("%s can't get RGMII RX delay", __func__);
	#elif defined(CONFIG_ARCH_RTD139x) || defined(CONFIG_ARCH_RTD16xx)
	if (of_property_read_u32(pdev->dev.of_node, "bypass", &bypass_enable))
		dprintk("%s can't get bypass", __func__);
	if (of_property_read_u32(pdev->dev.of_node, "acp", &acp_enable))
		dprintk("%s can't get acp", __func__);
	#if defined(CONFIG_ARCH_RTD16xx)
	if (of_property_read_u32(pdev->dev.of_node, "sgmii-swing", &sgmii_swing))
		dprintk("%s can't get sgmii-swing", __func__);
	#endif /* CONFIG_ARCH_RTD16xx */

	sbxaddr = of_iomap(pdev->dev.of_node, 2);
	otpaddr = of_iomap(pdev->dev.of_node, 3);
	sdsaddr = of_iomap(pdev->dev.of_node, 4);
	pinmuxaddr = of_iomap(pdev->dev.of_node, 5);

	#elif defined(CONFIG_ARCH_RTD13xx)
	if (of_property_read_u32(pdev->dev.of_node, "voltage", &voltage))
		dprintk("%s can't get voltage", __func__);
	if (of_property_read_u32(pdev->dev.of_node, "tx-delay", &tx_delay))
		dprintk("%s can't get TX delay", __func__);
	if (of_property_read_u32(pdev->dev.of_node, "rx-delay", &rx_delay))
		dprintk("%s can't get RX delay", __func__);
	if (of_property_read_u32(pdev->dev.of_node, "bypass", &bypass_enable))
		dprintk("%s can't get bypass", __func__);
	if (of_property_read_u32(pdev->dev.of_node, "acp", &acp_enable))
		dprintk("%s can't get acp", __func__);

	sbxaddr = of_iomap(pdev->dev.of_node, 2);
	otpaddr = of_iomap(pdev->dev.of_node, 3);
	pinmuxaddr = of_iomap(pdev->dev.of_node, 4);
	#elif defined(CONFIG_ARCH_RTD16XXB)
	pinmuxaddr = of_iomap(pdev->dev.of_node, 2);
	#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x |
		* CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD13xx |
		* CONFIG_ARCH_RTD16XXB
		*/

	if (of_get_property(pdev->dev.of_node, "force-Gb-off", NULL)) {
		pr_info("~~~ disable Gb features~~~\n");
		cfg->features &= ~RTL_FEATURE_GMII;
	}

	phy_0 = of_get_child_by_name(pdev->dev.of_node, "phy_0");
	if (phy_0) {
		phy_irq_num = of_irq_count(phy_0);
		pr_err("phy_0: get %d phy_irq\n", phy_irq_num);
		if (phy_irq_num > POR_NUM) {
			pr_err("phy_0: %d exceed max. GPHY IRQ number\n",
			       phy_irq_num);
			phy_irq_num = POR_NUM;
		}

		if (of_property_read_u32(phy_0, "irq-mask", &phy_irq_mask)) {
			pr_err("phy_0: no irq-mask is defined\n");
			phy_irq_num = 0;
		} else {
			tmp = 0;
			for (i = 0; i < 32; i++) {
				if (phy_irq_mask & (1 << i))
					phy_irq_map[tmp++] = i;

				if (tmp > phy_irq_num)
					break;
			}
			if (tmp != phy_irq_num) {
				pr_err("phy_0: IRQ number %d and IRQ mask 0x%08x don't match\n",
				       phy_irq_num, phy_irq_mask);
				phy_irq_num = min(tmp, phy_irq_num);
			}
		}
		for (i = 0; i < phy_irq_num; i++) {
			phy_irq[i] = irq_of_parse_and_map(phy_0, i);
			pr_err("phy_0: get phy_irq %d for bit %d\n",
			       phy_irq[i], phy_irq_map[i]);
		}

		if (of_property_read_u32(phy_0, "por-xv-mask",
					 &phy_por_xv_mask)) {
			pr_info("phy_0: no por-xv-mask is defined\n");
			phy_por_xv_mask = POR_XV_MASK;
		}
	}

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	if (!pdev->dev.coherent_dma_mask)
		pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	ioaddr = of_iomap(pdev->dev.of_node, 0);
	clkaddr = of_iomap(pdev->dev.of_node, 1);

	ndev = alloc_etherdev(sizeof(*tp));
	if (!ndev) {
		rc = -ENOMEM;
		goto out;
	}

	SET_NETDEV_DEV(ndev, &pdev->dev);
	ndev->netdev_ops = &rtl_netdev_ops;
	ndev->irq = irq;
	tp = netdev_priv(ndev);
	tp->dev = ndev;
	tp->pdev = pdev;
	tp->msg_enable = netif_msg_init(debug.msg_enable, R8169_MSG_DEFAULT);
	tp->mac_version = mac_version - 1;
	tp->mmio_addr = ioaddr;
	tp->mmio_clkaddr = clkaddr;
	tp->led_cfg = led_config;
	tp->output_mode = output_mode;
	tp->ext_phy_id = ext_phy_id;
	tp->eee_enable = !!(eee_enable > 0);
	tp->phy_irq_num = phy_irq_num;
	tp->phy_por_xv_mask = phy_por_xv_mask;
	for (i = 0; i < tp->phy_irq_num; i++) {
		tp->phy_irq[i] = phy_irq[i];
		tp->phy_irq_map[i] = phy_irq_map[i];
	}
	atomic_set(&tp->phy_reinit_flag, 0);

	#if defined(CONFIG_ARCH_RTD129x)
	tp->rgmii_voltage = rgmii_voltage;
	tp->rgmii_tx_delay = rgmii_tx_delay;
	tp->rgmii_rx_delay = rgmii_rx_delay;
	tp->acp_enable = false;
	#elif defined(CONFIG_ARCH_RTD139x) || defined(CONFIG_ARCH_RTD16xx)
	tp->mmio_sbxaddr = sbxaddr;
	tp->mmio_otpaddr = otpaddr;
	tp->mmio_sdsaddr = sdsaddr;
	tp->mmio_pinmuxaddr = pinmuxaddr;
	tp->bypass_enable = !!(bypass_enable > 0);
	tp->acp_enable = !!(acp_enable > 0);
	tp->ext_phy = false;
	#if defined(CONFIG_ARCH_RTD16xx)
	tp->sgmii_swing = sgmii_swing;
	#endif /* CONFIG_ARCH_RTD16xx */
	#elif defined(CONFIG_ARCH_RTD13xx)
	tp->mmio_sbxaddr = sbxaddr;
	tp->mmio_otpaddr = otpaddr;
	tp->mmio_pinmuxaddr = pinmuxaddr;
	tp->bypass_enable = !!(bypass_enable > 0);
	tp->acp_enable = !!(acp_enable > 0);
	tp->ext_phy = false;
	tp->voltage = voltage;
	tp->tx_delay = tx_delay;
	tp->rx_delay = rx_delay;
	#elif defined(CONFIG_ARCH_RTD16XXB)
	tp->mmio_pinmuxaddr = pinmuxaddr;
	tp->output_mode = 0;
	tp->bypass_enable = 1;
	tp->acp_enable = 0;
	tp->ext_phy = false;
	#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x |
		* CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD13xx |
		* CONFIG_ARCH_RTD16XXB
		*/

	mii = &tp->mii;
	mii->dev = ndev;
	mii->mdio_read = rtl_mdio_read;
	mii->mdio_write = rtl_mdio_write;
	mii->phy_id_mask = 0x1f;
	mii->reg_num_mask = 0x1f;
	mii->supports_gmii = !!(cfg->features & RTL_FEATURE_GMII);

	rtl_init_mdio_ops(tp);
	rtl_init_mmd_ops(tp);
	rtl_init_pll_power_ops(tp);
	rtl_init_jumbo_ops(tp);
	rtl_init_csi_ops(tp);

	#if defined(CONFIG_ARCH_RTD129x)
	if (likely(__clk_is_enabled(clk_etn_sys) && __clk_is_enabled(clk_etn_250m))) {
		/* enable again to prevent clk framework from disabling them */
		clk_prepare_enable(clk_etn_sys);
		clk_prepare_enable(clk_etn_250m);
	} else {
		/* 1. reg_0x98007088[10] = 1 */
		/* ISO spec, reset bit of gphy */
		reset_control_deassert(rstc_gphy);

		/* 2. CPU software waiting 200uS */
		usleep_range(200, 210);

		/* 3. reg_0x98007060[1] = 0 */
		/* ISO spec, Ethernet Boot up bypass gphy ready mode */
		tmp = readl(clkaddr + 0x60);
		tmp &= ~BIT(1);
		writel(tmp, clkaddr + 0x60);

		/* 4. reg_0x98007fc0[0] = 0 */
		/* ISO spec, Ethernet Boot up disable dbus clock gating */
		tmp = readl(clkaddr + 0xfc0);
		tmp &= ~BIT(0);
		writel(tmp, clkaddr + 0xfc0);

		/* 5. CPU software waiting 200uS */
		usleep_range(200, 210);

		/* 6. reg_0x9800708c[12:11] = 11 */
		/* ISO spec, clock enable bit for etn clock & etn 250MHz */
		clk_prepare_enable(clk_etn_sys);
		clk_prepare_enable(clk_etn_250m);

		/* 7. reg_0x9800708c[12:11] = 00 */
		/* ISO spec, clock enable bit for etn clock & etn 250MHz */
		clk_disable_unprepare(clk_etn_sys);
		clk_disable_unprepare(clk_etn_250m);

		/* 8. reg_0x98007088[9] = 1 */
		/* ISO spec, reset bit of gmac */
		reset_control_deassert(rstc_gmac);

		/* 9. reg_0x9800708c[12:11] = 11 */
		/* ISO spec, clock enable bit for etn clock & etn 250MHz */
		clk_prepare_enable(clk_etn_sys);
		clk_prepare_enable(clk_etn_250m);

		msleep(100);
	}
	#elif defined(CONFIG_ARCH_RTD139x) || defined(CONFIG_ARCH_RTD16xx)
	r8169soc_reset_phy_gmac(tp);
	r8169soc_acp_init(tp);
	r8169soc_pll_clock_init(tp);
	r8169soc_mdio_init(tp);
	/* after r8169soc_mdio_init(),
	 * SGMII : tp->ext_phy == true  ==> external MDIO,
	 * FE PHY: tp->ext_phy == false ==> internal MDIO
	 */

	r8169soc_eee_init(tp, tp->eee_enable);

	/* Enable ALDPS */
	rtl_phy_write(tp, 0x0a43, 24,
		      rtl_phy_read(tp, 0x0a43, 24) | BIT(2));
	#elif defined(CONFIG_ARCH_RTD13xx) || defined(CONFIG_ARCH_RTD16XXB)
	r8169soc_reset_phy_gmac(tp);
	r8169soc_acp_init(tp);
	r8169soc_pll_clock_init(tp);
	r8169soc_mdio_init(tp);
	/* after r8169soc_mdio_init(),
	 * RGMII : tp->ext_phy == true  ==> external MDIO,
	 * RMII  : tp->ext_phy == false ==> no MDIO,
	 * FE PHY: tp->ext_phy == false ==> internal MDIO
	 */

	r8169soc_eee_init(tp, tp->eee_enable);

	/* Enable ALDPS */
	rtl_phy_write(tp, 0x0a43, 24,
		      rtl_phy_read(tp, 0x0a43, 24) | BIT(2));
	#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x |
		* CONFIG_ARCH_RTD16xx | CONFIG_ARCH_RTD13xx |
		* CONFIG_ARCH_RTD16XXB
		*/

	tp->wol_enable = wol_enable;
	tp->wol_crc_cnt = 0;
	for (i = 0; i < RTL_WPD_SIZE; i++) {
		memset(tmp_str, 0, 80);
		sprintf(tmp_str, "wake-mask%d", i);
		wake_mask = of_find_property(pdev->dev.of_node, tmp_str, NULL);
		if (!wake_mask)
			break;
		tp->wol_mask_size[i] = wake_mask->length;
		memcpy(&tp->wol_mask[i][0], wake_mask->value, wake_mask->length);

		sprintf(tmp_str, "wake-crc%d", i);
		if (of_property_read_u32(pdev->dev.of_node, tmp_str, &tp->wol_crc[i]))
			break;
		tp->wol_crc_cnt += 1;
	}

#ifdef RTL_PROC
	do {
		/* create /proc/net/$rtw_proc_name */
		rtw_proc = proc_mkdir_data("eth0", 0555,
					   init_net.proc_net, NULL);

		if (!rtw_proc) {
			pr_info("procfs:create /proc/net/eth0 failed\n");
			break;
		}

		/* create /proc/net/$rtw_proc_name/$dev->name */
		if (!tp->dir_dev) {
			tp->dir_dev = proc_mkdir_data(MODULENAME,
						      S_IFDIR | 0555,
						      rtw_proc, ndev);
			dir_dev = tp->dir_dev;

			if (!dir_dev) {
				pr_info("procfs:create /proc/net/eth0/r8169 failed\n");

				if (rtw_proc) {
					remove_proc_entry("eth0",
							  init_net.proc_net);
					rtw_proc = NULL;
				}
				break;
			}
		} else {
			break;
		}

		entry = proc_create_data("wol_enable", S_IFREG | 0644,
					 dir_dev, &wol_proc_fops, NULL);
		if (!entry) {
			pr_info("procfs:create /proc/net/eth0/r8169/wol_enable failed\n");
			break;
		}

		entry = proc_create_data("phy_reinit", S_IFREG | 0644,
					 dir_dev, &phy_reinit_proc_fops, NULL);
		if (!entry) {
			pr_info("procfs:create /proc/net/eth0/r8169/phy_reinit failed\n");
			break;
		}

		entry = proc_create_data("eee", S_IFREG | 0644,
					 dir_dev, &eee_proc_fops, NULL);
		if (!entry) {
			pr_info("procfs:create /proc/net/eth0/r8169/eee failed\n");
			break;
		}

		entry = proc_create_data("test", S_IFREG | 0644,
					 dir_dev, &test_proc_fops, NULL);
		if (!entry) {
			pr_info("procfs:create /proc/net/eth0/r8169/test failed\n");
			break;
		}

		entry = proc_create_data("driver_var", S_IFREG | 0444,
					 dir_dev, &driver_var_proc_fops, NULL);
		if (!entry) {
			pr_info("procfs:create /proc/net/eth0/r8169/driver_var failed\n");
			break;
		}

		entry = proc_create_data("eth_phy", S_IFREG | 0444,
					 dir_dev, &eth_phy_proc_fops, NULL);
		if (!entry) {
			pr_info("procfs:create /proc/net/eth0/r8169/eth_phy failed\n");
			break;
		}

		entry = proc_create_data("ext_regs", S_IFREG | 0444,
					 dir_dev, &ext_regs_proc_fops, NULL);
		if (!entry) {
			pr_info("procfs:create /proc/net/eth0/r8169/ext_regs failed\n");
			break;
		}

		entry = proc_create_data("registers", S_IFREG | 0444,
					 dir_dev, &registers_proc_fops, NULL);
		if (!entry) {
			pr_info("procfs:create /proc/net/eth0/r8169/registers failed\n");
			break;
		}

		entry = proc_create_data("tally", S_IFREG | 0444,
					 dir_dev, &tally_proc_fops, NULL);
		if (!entry) {
			pr_info("procfs:create /proc/net/eth0/r8169/tally failed\n");
			break;
		}

		entry = proc_create_data("wpd_event", S_IFREG | 0444,
					 dir_dev, &wpd_evt_proc_fops, NULL);
		if (!entry) {
			pr_info("procfs:create /proc/net/eth0/r8169/wpd_event failed\n");
			break;
		}

		entry = proc_create_data("wol_packet", S_IFREG | 0444,
					 dir_dev, &wol_pkt_proc_fops, NULL);
		if (!entry) {
			pr_info("procfs:create /proc/net/eth0/r8169/wol_packet failed\n");
			break;
		}
	} while (0);
#endif

	rtl_init_rxcfg(tp);

	rtl_irq_disable(tp);

	rtl_hw_initialize(tp);

	rtl_hw_reset(tp);

	rtl_ack_events(tp, 0xffff);

	rtl8169_print_mac_version(tp);

	chipset = tp->mac_version;
	tp->txd_version = rtl_chip_infos.txd_version;

	RTL_W8(Cfg9346, Cfg9346_Unlock);
	RTL_W8(Config1, RTL_R8(Config1) | PMEnable);
	RTL_W8(Config5, RTL_R8(Config5) & PMEStatus);

	/* disable magic packet WOL */
	RTL_W8(Config3, RTL_R8(Config3) & ~MagicPacket);

	if ((RTL_R8(Config3) & (LinkUp | MagicPacket)) != 0)
		tp->features |= RTL_FEATURE_WOL;
	if ((RTL_R8(Config5) & (UWF | BWF | MWF)) != 0)
		tp->features |= RTL_FEATURE_WOL;
	RTL_W8(Cfg9346, Cfg9346_Lock);

	tp->set_speed = rtl8169_set_speed_xmii;
	tp->get_settings = rtl8169_gset_xmii;
	tp->phy_reset_enable = rtl8169_xmii_reset_enable;
	tp->phy_reset_pending = rtl8169_xmii_reset_pending;
	tp->link_ok = rtl8169_xmii_link_ok;
	tp->do_ioctl = rtl_xmii_ioctl;

	#if defined(CONFIG_ARCH_RTD129x)
	if ((get_rtd_chip_revision() >= RTD_CHIP_B00) &&
	    tp->output_mode == OUTPUT_RGMII_TO_MAC)
		tp->link_ok = rtl8169_xmii_always_link_ok;
	#elif defined(CONFIG_ARCH_RTD139x)
	if (tp->output_mode == OUTPUT_SGMII_TO_MAC)
		tp->link_ok = rtl8169_xmii_always_link_ok;
	#endif /* CONFIG_ARCH_RTD129x | CONFIG_ARCH_RTD139x */

	mutex_init(&tp->wk.mutex);

	/* Get MAC address */
	mac_addr = of_get_mac_address(pdev->dev.of_node);
	if (mac_addr)
		rtl_rar_set(tp, (u8 *)mac_addr);

	/* workaround: avoid getting deadbeef */
	#define RETRY_MAX	10
	for (retry = 0; retry < RETRY_MAX; retry++) {
		for (i = 0; i < ETH_ALEN; i++)
			ndev->dev_addr[i] = RTL_R8(MAC0 + i);

		if (*(u32 *)ndev->dev_addr == 0xdeadbeef) {
			pr_err("%s get invalid MAC address %pM, retry %d\n",
			       rtl_chip_infos.name, ndev->dev_addr, retry);
			tmp = RTL_R32(PHYAR);	/* read something else */
			usleep_range(10000, 11000);
		} else {
			break;
		}
	}
	if (retry == RETRY_MAX)
		pr_err("%s get invalid MAC address %pM, give up!\n",
		       rtl_chip_infos.name, ndev->dev_addr);

	rtl_led_set(tp);

	ndev->ethtool_ops = &rtl8169_ethtool_ops;
	ndev->watchdog_timeo = RTL8169_TX_TIMEOUT;

	netif_napi_add(ndev, &tp->napi, rtl8169_poll, R8169_NAPI_WEIGHT);

	/* don't enable SG, IP_CSUM and TSO by default - it might not work
	 * properly for all devices
	 */
	ndev->features |=
		NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO | NETIF_F_RXCSUM |
		NETIF_F_HW_VLAN_CTAG_TX | NETIF_F_HW_VLAN_CTAG_RX;

	ndev->hw_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
		NETIF_F_RXCSUM | NETIF_F_HW_VLAN_CTAG_TX |
		NETIF_F_HW_VLAN_CTAG_RX;
	ndev->vlan_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
		NETIF_F_HIGHDMA;

	ndev->hw_features |= NETIF_F_RXALL;
	ndev->hw_features |= NETIF_F_RXFCS;

	tp->hw_start = cfg->hw_start;
	tp->event_slow = cfg->event_slow;

	tp->opts1_mask = ~(RxBOVF | RxFOVF);

	rc = register_netdev(ndev);
	if (rc < 0)
		goto err_out_msi_4;

	platform_set_drvdata(pdev, ndev);

	netif_info(tp, probe, ndev, "%s, XID %08x IRQ %d\n",
		   rtl_chip_infos.name,
		   (u32)(RTL_R32(TxConfig) & 0x9cf0f8ff), ndev->irq);
	if (rtl_chip_infos.jumbo_max != JUMBO_1K) {
		netif_info(tp, probe, ndev, "jumbo features [frames: %d bytes, tx checksumming: %s]\n",
			   rtl_chip_infos.jumbo_max,
			   rtl_chip_infos.jumbo_tx_csum ? "ok" : "ko");
	}

	device_set_wakeup_enable(&pdev->dev, tp->features & RTL_FEATURE_WOL);

	netif_carrier_off(ndev);

	init_waitqueue_head(&tp->thr_wait);
	tp->link_chg = 0;
	tp->kthr = NULL;
	tp->kthr = kthread_create(link_status, (void *)ndev, "%s-linkstatus", ndev->name);
	if (IS_ERR(tp->kthr)) {
		tp->kthr = NULL;
		pr_warn("%s: unable to start kthread for link status.\n",
			ndev->name);
	} else {
		wake_up_process(tp->kthr);  /* run thread function */
	}

	if (tp->phy_irq_num) {
		/* monitor rising edge of POR interrupts */
		writel(0x00000333, tp->mmio_clkaddr + ISO_POR_CTRL);
		/* set voltage threshold of POR interrupts */
		writel(readl(tp->mmio_clkaddr + ISO_POR_VTH) | 0x00000330,
		       tp->mmio_clkaddr + ISO_POR_VTH);
		/* clear GPHY HV/DV/AV POR interrupts */
		writel(0x70000000, tp->mmio_clkaddr + ISO_UMSK_ISR);
	}

	for (i = 0; i < tp->phy_irq_num; i++) {
		memset(phy_irq_name[i], 0, IFNAMSIZ);
		sprintf(phy_irq_name[i], "eth_phy%d", i);
		rc = request_irq(tp->phy_irq[i], phy_irq_handler, IRQF_SHARED,
				 phy_irq_name[i], tp);
		if (rc < 0) {
			pr_err("%s: unable to request %s IRQ %d, ret = 0x%x\n",
			       ndev->name, phy_irq_name[i], tp->phy_irq[i], rc);
		} else {
			pr_info("%s: request %s IRQ %d successfully\n",
				ndev->name, phy_irq_name[i], tp->phy_irq[i]);
		}
	}
out:
	return rc;

err_out_msi_4:
	netif_napi_del(&tp->napi);
	free_netdev(ndev);
	goto out;
}

static struct platform_driver rtl8169_soc_driver = {
	.probe		= rtl_init_one,
	.remove		= rtl_remove_one,
	.shutdown	= rtl_shutdown,
	.driver = {
		.name		= MODULENAME,
		.owner		= THIS_MODULE,
		.pm		= RTL8169_PM_OPS,
		.of_match_table = of_match_ptr(r8169soc_dt_ids),
	},
};

module_platform_driver(rtl8169_soc_driver);
