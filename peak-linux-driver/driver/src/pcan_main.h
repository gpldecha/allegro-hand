/****************************************************************************
 * Copyright (C) 2001-2010  PEAK System-Technik GmbH
 *
 * linux@peak-system.com
 * www.peak-system.com
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Maintainer(s): Klaus Hitschler (klaus.hitschler@gmx.de)
 *
 * Major contributions by:
 *                Edouard Tisserant (edouard.tisserant@lolitech.fr) XENOMAI
 *                Laurent Bessard   (laurent.bessard@lolitech.fr)   XENOMAI
 *                Oliver Hartkopp   (oliver.hartkopp@volkswagen.de) socketCAN
 *                Stephane Grosjean (s.grosjean@peak-system.com)    USB-PRO
 *
 * Contributions: Marcel Offermans (marcel.offermans@luminis.nl)
 *                Philipp Baer (philipp.baer@informatik.uni-ulm.de)
 *                Marc Sowen (Marc.Sowen@ibeo-as.com)
 *****************************************************************************/

/****************************************************************************
 *
 * pcan_main.h - global defines to include in all files this module is made of
 *
 * $Id$
 *
 *****************************************************************************/
#ifndef __PCAN_MAIN_H__
#define __PCAN_MAIN_H__

/* INCLUDES */
#include "src/pcan_common.h"

#include <linux/types.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/time.h>

#ifdef LINUX_26
#include <linux/device.h>
#endif

#ifdef DEBUG
#define DEBUG_TX_ENG
#endif

//#define PCAN_USES_OLD_TX_ENGINE_STATE

#ifdef PCI_SUPPORT
#include <linux/pci.h>

#define PCAN_PCI_MINOR_BASE	0	/* the base of all pci device minors */
#endif

#ifdef ISA_SUPPORT
#define ISA_MINOR_BASE		8
#endif

#ifdef DONGLE_SUPPORT
#define PCAN_DNG_SP_MINOR_BASE	16	/* SP devs minors starting point */
#define PCAN_DNG_EPP_MINOR_BASE	24	/* EPP devs minors starting point */
#endif

#include <asm/atomic.h>

#ifdef PARPORT_SUBSYSTEM
#include <linux/parport.h>
#endif

#ifdef PCIEC_SUPPORT
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#endif

#ifdef USB_SUPPORT
#include <linux/usb.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,4,19)
typedef struct urb urb_t, *purb_t;
#endif

#define PCAN_USB_MINOR_BASE	32	/* USB dev minors starting point */

#define PCAN_USB_CMD_PER_DEV
#define PCAN_USB_PCAN_SYSFS
#define PCAN_USB_DONT_REGISTER_DEV	/* no usbmisc device created */
#endif

#ifdef PCCARD_SUPPORT
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
#include <pcmcia/cs_types.h>
#endif
#include <pcmcia/cs.h>
#endif
#include <pcmcia/cistpl.h>
#include <pcmcia/ds.h>

#define PCCARD_MINOR_BASE	40
#define PCAN_USB_MINOR_END	(PCCARD_MINOR_BASE-1)
#else
#define PCAN_USB_MINOR_END	-1
#endif

#ifdef NETDEV_SUPPORT
#include <linux/netdevice.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
#include <linux/can/dev.h>
#endif
#endif

struct pcan_udata;

/* PF_CAN is part of the Linux Mainline Kernel since v2.6.25
 * For older Kernels the PCAN driver includes the needed
 * defines from private files src/can.h and src/error.h */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25)
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#else /* before 2.6.25 pcan netdev contains private includes */
#include <src/can.h>
#include <src/error.h>

#define ARPHRD_CAN	280	/* to be moved to include/linux/if_arp.h */
#define ETH_P_CAN	0x000C	/* to be moved to include/linux/if_ether.h */
#endif

/* fix overlap in namespace between socketcan can/error.h and pcan.h */
#define CAN_ERR_BUSOFF_NETDEV	CAN_ERR_BUSOFF
#undef CAN_ERR_BUSOFF

#include <pcan.h>
#include <pcanfd.h>

#include "src/pcan_fifo.h"
#include "src/pcan_timing.h"

/* DEFINES */
#define CHANNEL_SINGLE	0	/* this is a single channel device */
#define CHANNEL_MASTER	1	/* multi channel device, master device */
#define CHANNEL_SLAVE	2	/* multi channel device, this is slave */

#define READBUFFER_SIZE		80	/* buffers used in readr/write call */
#define WRITEBUFFER_SIZE	512

#define PCAN_MAJOR	0	/* use dynamic major alloc, 91 otherwise */

#define READ_MESSAGE_COUNT	500	/* max read message count */
#define WRITE_MESSAGE_COUNT	500	/* max write message count */

/* parameter wBTR0BTR1 */
/* bitrate codes of BTR0/BTR1 registers */
#define CAN_BAUD_1M	0x0014	/*   1 MBit/s */
#define CAN_BAUD_500K	0x001C	/* 500 kBit/s */
#define CAN_BAUD_250K	0x011C	/* 250 kBit/s */
#define CAN_BAUD_125K	0x031C	/* 125 kBit/s */
#define CAN_BAUD_100K	0x432F	/* 100 kBit/s */
#define CAN_BAUD_50K	0x472F	/*  50 kBit/s */
#define CAN_BAUD_20K	0x532F	/*  20 kBit/s */
#define CAN_BAUD_10K	0x672F	/*  10 kBit/s */
#define CAN_BAUD_5K	0x7F7F	/*   5 kBit/s */

/* Activity states */
#define ACTIVITY_NONE		0	/* LED off           - set when the channel is created or deleted */
#define ACTIVITY_INITIALIZED	1	/* LED on            - set when the channel is initialized */
#define ACTIVITY_IDLE		2	/* LED slow blinking - set when the channel is ready to receive or transmit */
#define ACTIVITY_XMIT		3	/* LED fast blinking - set when the channel has received or transmitted */

/* this structure holds various channel properties */
typedef struct chn_props {
	u8 ucExternalClock : 1;	/* supplied with a external clock */
	u8 ucMasterDevice  : 2; /* clock master, slave, single */
} CHN_PROPS;

/* helper for fast conversion between SJA1000 and host data ordering */
typedef union {
	u8  uc[4];
	u32 ul;
} ULCONV;

typedef union {
	u8  uc[2];
	u16 uw;
} UWCONV;

/* uCAN device programming interface */
struct ucan_engine;
struct ucan_msg;
struct pcandev;
struct ucan_ops {

	int (*set_clk_domain)(struct pcandev *, struct pcanfd_init *);

	/* Tx path: commands and msgs sending interface */
	int (*send_cmd)(struct pcandev *);
	int (*send_msg)(struct pcandev *);

	/* Rx path: all rx messages handlers are stored into a table */
	int (**handle_msg_table)(struct ucan_engine *,
				 struct ucan_msg *, void *);
	int handle_msg_size;

	/* handler of msgs which ID is outside handle_msg_table[] */
	int (*handle_private_msg)(struct ucan_engine *,
				  struct ucan_msg *, void *);
};

struct ucan_engine {
	struct ucan_ops *	ops;		/* uCAN cmd/msg ops */
	struct pcandev *	devs;		/* uCAN channels */
	int			devs_count;	/* count of uCAN channels */

	void *	cmd_head;	/* buffer used to save uCAN cmds */
	int	cmd_size;	/* size of this buffer in bytes */
	int	cmd_len;	/* length of the cmd (in bytes) */
};

#ifdef PCIEC_SUPPORT
#define PCIEC_CHANNELS	2	/* maximum PCAN-PCIExpressCard channel number */

typedef struct {
	void __iomem *gpoutenable;	/* vaddr for bit-banging interface */
	void __iomem *gpin;
	void __iomem *gpout;
	struct	pcandev *dev[PCIEC_CHANNELS];	/* associated channels */

	struct i2c_adapter		adapter;	/* i2c adapter */
	struct i2c_algo_bit_data	algo_data;	/* bit banging if */

	u8	VCCenable;		/* reflection of VCCEN */
	u8	PCA9553_LS0Shadow;      /* Shadow reg holding LEDs state */
	int	run_activity_timer_cyclic;	/* sync flag stop conditions */

	struct delayed_work	activity_timer;	/* scan for activity timer */
} PCAN_PCIEC_CARD;
#endif

struct pcan_msi {
	int	msi_requested;
	int	msi_assigned;
};

struct ucan_pci_page {
	void *		vbase;
	dma_addr_t	lbase;
	u32		offset;
	u32		size;
};

/* timestamp sync in µs */
struct pcan_time_sync {
	u64		tv_ns;
	unsigned long	sync_count;
	unsigned long	sync_us_cnt;

	struct timeval	tv;		/* host_time */
	struct timeval	tv_tts;		/* count of device_us */
	struct timeval	tv_ts;		/* device_time = host_time0 + tv_tts */

	u64		ttv_us;		/* count of host_us */
	u64		tts_us;		/* count of device_us */

	u64		ts_us;		/* last sync'ed device timestamp */

	long		clock_drift;
};

/* 32-bits area describing the content of the beginning of Rx DMA area of the
 * PCI boards running uCAN */
struct ucan_pci_irq_status {
#ifdef __LITTLE_ENDIAN
	uint	irq_tag:4;
	uint	rx_cnt:7;
	uint	:5;
	uint	lnk:1;
	uint	:15;
#else
	uint	:15;
	uint	lnk:1;
	uint	:5;
	uint	rx_cnt:7;
	uint	irq_tag:4;
#endif
};

typedef struct {
	u32	dwConfigPort;		/* the configuration port, PCI only */
	void __iomem *can_port_addr;    /* virtual address of port */
	void __iomem *bar0_cfg_addr;	/* vaddr of the config port */
	struct pci_dev *pciDev;		/* remember the hosting PCI card */
#ifdef PCIEC_SUPPORT
	PCAN_PCIEC_CARD *card;		/* point to a card structure */
#endif
	/* PCIe uCAN specific */
	u64	ucan_cmd;

	u32	irq_tag;
	u32	irq_not_for_me;
	struct ucan_pci_irq_status irq_status;

	dma_addr_t rx_dma_laddr;
	void *	rx_dma_vaddr;

	dma_addr_t tx_dma_laddr;
	void *	tx_dma_vaddr;

	u16	tx_pages_free;
	u16	tx_page_index;

	struct ucan_pci_page *tx_pages;


} PCI_PORT;

typedef struct {
#ifdef PARPORT_SUBSYSTEM
	struct pardevice *pardev;	/* associated parallel port */
#endif
	u16	wEcr;			/* ECR register in case of EPP */
	u8	ucOldDataContent;	/* overwritten contents of port regs */
	u8	ucOldControlContent;
	u8	ucOldECRContent;

	pcan_lock_t	lock;	/* shared access to chip registers */
} DONGLE_PORT;

#ifdef PCAN_HANDLE_IRQ_SHARING
typedef struct {
	struct list_head	item;	/* link for items with same irq level */
	struct pcandev *	dev;	/* device with the same irq level */
} SAME_IRQ_ITEM;

typedef struct {
	struct list_head	same_irq_items;	/* list of SAME_IRQ_ITEM's */

	u16		same_irq_count;		/* count of devices */
	u16		same_irq_active;	/* count of active irqs */
} SAME_IRQ_LIST;
#endif

typedef struct {
#ifdef PCAN_HANDLE_IRQ_SHARING
	SAME_IRQ_ITEM	same;	/* each ISA_PORT belongs to one SAME_IRQ_LIST */
	SAME_IRQ_LIST	anchor;	/* the anchor for one irq level */

	SAME_IRQ_LIST *	my_anchor;	/* list of items for the same irq */
#endif
} ISA_PORT;

#ifdef PCCARD_SUPPORT
struct pcan_pccard;
typedef struct {
	struct pcan_pccard *card;	/* points to the associated pccard */
} PCCARD_PORT;
#endif

#ifdef USB_SUPPORT
typedef struct {
	u64	ullCumulatedTicks;	/* sum of all ticks */
	u64	ullOldCumulatedTicks;	/* old ... */
	u16	wStartTicks;		/* ticks at first init */
	u16	wLastTickValue;		/* Last aquired tick count */
	u16	wOldLastTickValue;	/* old ... */
	u8	ucLastTickValue;	/* the same for byte tick counts */

	struct timeval	StartTime;	/* time of first receive */
} PCAN_USB_TIME;

typedef struct {
	u8	ucNumber;		/* number (or address) of endpoint */
	u16	wDataSz;		/* supported max data transfer length */
} PCAN_ENDPOINT;

struct pcan_usb_interface;
typedef struct pcan_usb_port {
	struct pcan_usb_interface *usb_if;

	u8	ucHardcodedDevNr;

	u32	dwTelegramCount;	/* counter for telegrams */

	PCAN_USB_TIME usb_time;		/* PCAN_USB_TIME */

	struct urb	write_data;	/* pointer to write data urb */
	int	write_packet_size;	/* packet write buffer size */
	int	write_buffer_size;
	u8 *	write_buffer_addr;	/* buffer for to write data */

	PCAN_ENDPOINT pipe_write;

	u32	state;

#ifdef PCAN_USB_CMD_PER_DEV
	struct urb	urb_cmd_async;		/* async. cmd URB */
	atomic_t	cmd_async_complete;	/* flag set when async cmd  */
	struct urb	urb_cmd_sync;		/* sync cmd URB */
	atomic_t	cmd_sync_complete;	/* flag set when sync cmd  */
						/* finished */
#endif
	u8 *		cout_baddr;		/* command buffer address */
	int		cout_bsize;		/* command buffer size */

} USB_PORT;
#endif /* USB_SUPPORT */

struct pcan_adapter {
	const char *	name;
	int		index;
	int		can_count;
	int		opened_count;
	int		hw_ver_major;
	int		hw_ver_minor;
	int		hw_ver_subminor;
};

typedef struct __array_of_struct(pcanfd_available_clock, 1)
	pcanfd_mono_clock_device;

struct pcanfd_options {
	int req_size;
	int (*get)(struct pcandev *dev, struct pcanfd_option *, void *arg);
	int (*set)(struct pcandev *dev, struct pcanfd_option *, void *arg);
};

#define PCAN_DEV_LISTEN_ONLY	0x00000001
#define PCAN_DEV_USES_ALT_NUM	0x00000002
#define PCAN_DEV_IGNORE_RX	0x00000004
#define PCAN_DEV_LINKED		0x00000008
#define PCAN_DEV_CLEANED	0x00000010
#define PCAN_DEV_STATIC		0x00000020
#define PCAN_DEV_BUSLOAD_RDY	0x00000100
#define PCAN_DEV_ERRCNT_RDY	0x00000200
#define PCAN_DEV_OPENED		0x00001000
#define PCAN_DEV_MSI_SHARED	0x00002000
#define PCAN_DEV_BUS_ON		0x00004000
#define PCAN_DEV_TXPAUSE_RDY	0x00008000
#define PCAN_DEV_HWTS_RDY	0x00010000
#define PCAN_DEV_HWTSC_RDY	0x00020000

#define TX_ENGINE_CLOSED	0
#define TX_ENGINE_IDLE		1
#define TX_ENGINE_STARTED	2
#define TX_ENGINE_STOPPED	3
#define TX_ENGINE_BUSY		4

typedef struct pcandev {
	struct list_head	list;	/* link anchor for list of devices */

	int	nOpenPaths;	/* number of open paths linked to the device */
	int	nMajor;		/* device major (USB devices have their own) */
	int	nMinor;		/* the associated minor */
	char *	type;		/* the literal type of the device, info only */

	u16	wType;		/* (number type) to distinguish sp and epp */
	u16	wInitStep;	/* device specific init state */

	int	nChannel;	/* in case of multi-CAN board/adapter */
	int	opened_index;	/* open sequence order */

	struct pcan_adapter *adapter;	/* link to the real device */

	u32	dwPort;		/* the port of the transport layer */

	u32	ts_mode;

	u32	flags;
	u32	tx_iframe_delay_us;

	u32	allowed_msgs;
	u32	sysclock_Hz;
	const struct pcanfd_available_clocks *clocks_list;
	const struct pcanfd_bittiming_range *bittiming_caps;
	const struct pcanfd_bittiming_range *dbittiming_caps;

	struct pcanfd_options *	option;
	struct pcan_time_sync	time_sync;	/* used to sync clocks */

	struct device *		sysfs_dev;
	struct attribute **	sysfs_attrs;

	u32	device_alt_num;
	union {
		struct {
#ifdef __LITTLE_ENDIAN
			u32	mask;
			u32	code;
#else
			u32	code;
			u32	mask;
#endif
		};
		u64	value64;
	} acc_11b, acc_29b;

#ifdef NETDEV_SUPPORT
	struct net_device *netdev;	/* reference to net device for AF_CAN */
	struct delayed_work restart_work;
#endif

	struct ucan_engine ucan;	/* ref to the uCAN engine */

	union {
		DONGLE_PORT	dng;	/* private data of the various ports */
		ISA_PORT	isa;
		PCI_PORT	pci;
#ifdef PCCARD_SUPPORT
		PCCARD_PORT	pccard;
#endif
#ifdef USB_SUPPORT
		USB_PORT	usb;
#endif
	} port;

	struct chn_props	props;	/* various channel properties */

	/* read a register */
	u8   (*readreg)(struct pcandev *dev, u8 port);

	/* write a register */
	void (*writereg)(struct pcandev *dev, u8 port, u8 data);

	/* cleanup the interface */
	int  (*cleanup)(struct pcandev *dev);

	/* called at open of a path (open()) */
	int  (*open)(struct pcandev *dev);

	/* called at release of a path (close()) */
	int  (*release)(struct pcandev *dev);

	/* install the interrupt handler */
	int  (*req_irq)(struct pcandev *dev, struct pcan_udata *ctx);

	/* release the interrupt */
	void (*free_irq)(struct pcandev *dev, struct pcan_udata *ctx);

	/* open the device itself */
	int  (*device_open)(struct pcandev *dev, u16 btr0btr1,
						u8 bExtended, u8 bListenOnly);
	int  (*device_open_fd)(struct pcandev *dev, struct pcanfd_init *pfdi);

	/* release the device itself */
	void (*device_release)(struct pcandev *dev);

	/* write the device */
	int  (*device_write)(struct pcandev *dev, struct pcan_udata *ctx);

	/* interface to set or get special parameters from the device */
	int  (*device_params)(struct pcandev *dev, TPEXTRAPARAMS *params);

	pcan_event_t	in_event;
	pcan_event_t	out_event;

#ifdef PCAN_USES_OLD_TX_ENGINE_STATE
	atomic_t	tx_engine_state;
#else
	unsigned int	locked_tx_engine_state;
#endif
	pcan_lock_t	wlock;	/* mutual exclusion lock for write invocation */
	pcan_lock_t	isr_lock;	/* in isr */

	pcan_mutex_t	mutex;

#ifndef NO_RT
	rtdm_irq_t	irq_handle;	/* mandatory parameter in for Xenomai */
#endif

	int	bus_load;
	int	prev_bus_load;

	int	nLastError;	/* last error written */
	enum pcanfd_status	bus_state;
	u32	dwErrorCounter;		/* counts all fatal errors */
	u32	dwInterruptCounter;	/* counts all interrupts */
	u32	rx_frames_counter;	/* Rx frames read on device */
	u32	tx_frames_counter;	/* Tx frames written on device */
	u32	rx_bytes_counter;
	u32	tx_bytes_counter;

	struct pcanfd_init	def_init_settings;
	struct timeval		init_timestamp;
	struct pcanfd_init	init_settings;

	FIFO_MANAGER	readFifo;	/* manages the read fifo */
	FIFO_MANAGER	writeFifo;	/* manages the write fifo */

	struct pcanfd_msg *rMsg;
	struct pcanfd_msg *wMsg;

	void *		filter;	/* ID filter - currently associated to device */

	u16	wCANStatus;	/* status of CAN chip */
	u8	ucPhysicallyInstalled;	/* the device is PhysicallyInstalled */
	u8	ucActivityState;	/* state of a channel activity */

	u8	rx_error_counter;	/* Rx errors counter */
	u8	tx_error_counter;	/* Tx errors counter */
	u8	prev_rx_error_counter;	/* Rx errors counter previous value */
	u8	prev_tx_error_counter;	/* Tx errors counter previous value */

	u16	wIrq;		/* the associated irq */

} PCANDEV;

#ifdef USB_SUPPORT
struct pcan_usb_interface {
	struct pcan_adapter *adapter;
	struct usb_device *usb_dev;	/* Kernel USB device */
	struct usb_interface *usb_intf;

	int	can_count;
	int	index;
	int	opened_count;

	u32	state;
	int	cm_ignore_count;	/* nb of CM to ignore before handling */

	u8	ucHardcodedDevNr;
	u32	dwSerialNumber;		/* Serial number of device */
	u8	ucRevision;		/* the revision number of  */

#if 0
	wait_queue_head_t	usb_wait_queue;	/* wait queue for usb */
#endif
	atomic_t	active_urbs;	/* note all active urbs */

#ifndef PCAN_USB_CMD_PER_DEV
	struct urb	urb_cmd_async;		/* async. cmd URB */
	atomic_t	cmd_async_complete;	/* flag set when async cmd  */
#endif
	struct urb	urb_cmd_sync;		/* sync cmd URB */
	atomic_t	cmd_sync_complete;	/* flag set when sync cmd  */
	                                                 /* finished */

	struct urb	read_data;		/* pointer to read data urb */
	int		read_packet_size;	/* packet read buffer size */
	int		read_buffer_size;
	u8 *		read_buffer_addr[2];	/* read data transfer buffers */

	/* USB pipes to/from CAN controller(s) */
	PCAN_ENDPOINT pipe_cmd_in;
	PCAN_ENDPOINT pipe_cmd_out;
	PCAN_ENDPOINT pipe_read;

	int  (*device_init)(struct pcan_usb_interface *);

	int  (*device_get_snr)(struct pcan_usb_interface *, u32 *);
	int  (*device_msg_decode)(struct pcan_usb_interface *, u8 *, int );
	void (*device_free)(struct pcan_usb_interface *);

	int  (*device_ctrl_init)(struct pcandev *dev);
	void (*device_ctrl_cleanup)(struct pcandev *dev);
	int  (*device_ctrl_open)(struct pcandev *dev, u16, u8, u8 );
	int  (*device_ctrl_open_fd)(struct pcandev *dev,
					struct pcanfd_init *pfdi);
	int  (*device_ctrl_close)(struct pcandev *dev);
	int  (*device_ctrl_set_bus_on)(struct pcandev *dev);
	int  (*device_ctrl_set_bus_off)(struct pcandev *dev);
	int  (*device_ctrl_set_dnr)(struct pcandev *dev, u32);
	int  (*device_ctrl_get_dnr)(struct pcandev *dev, u32 *);
	int  (*device_ctrl_msg_encode)(struct pcandev *dev, u8 *, int *);

	u32		rx0_sync;	/* 1: should sync tv vs. ts */
	struct timeval	rx0_tv;		/* time of the 1st rx frame */
	u32		rx0_ts;		/* timestamp of 1st rx frame */

	/* fields to compute time from usb adapter to driver:
	 * time of usb transmission = (time of the reception of a response
	 *                           - time of the sending of a request) / 2 */
	struct timeval	tv_request;
	struct timeval	tv_response;
	u32		rtt;

	struct timer_list	calibration_timer;

	int 	frame_index;

	u32	ts_high;
	u32	ts_low;

	int	frag_rec_offset;

	struct pcandev dev[0];		/* a device for each CAN controller */
};
#endif

struct pcan_udata {

#ifdef NETDEV_SUPPORT
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
	struct net_device_stats		stats;	/* standard netdev statistics */

#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 31)
	struct can_priv			can;	/* must be the 1st one */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
	/* consider playing with bitrates only for 3.6+ and CAN-FD */
	struct can_bittiming_const	bt_const;
	struct can_bittiming_const	dbt_const;
#endif
#endif

#endif /* NETDEV_SUPPORT */

	int	open_flags;

	u8	pcReadBuffer[READBUFFER_SIZE];	/* used in read() call */
	u8 *	pcReadPointer;	/* points into current read data rest */
	int	nReadRest;	/* rest of data left to read */

	int	nTotalReadCount;        /* for test only */

	u8	pcWriteBuffer[WRITEBUFFER_SIZE];/* used in write() call */
	u8 *	pcWritePointer;	/* work pointer into buffer */
	int	nWriteCount;	/* count of written data bytes */

#ifdef NO_RT
	struct file *			filep;		/* back linkage */
#elif !defined(XENOMAI3)
	struct rtdm_dev_context *	context;	/* back linkage */
#endif

	struct pcandev *dev;	/* pointer to related device */
};

struct pcan_driver {
	int	nMajor;		/* the major number of Pcan interfaces */
	u16	wDeviceCount;	/* count of found devices */
	u16	wInitStep;	/* driver specific init state */

	struct timeval		sInitTime;	/* time when init was called */
	struct list_head	devices;	/* list of devices */
#ifdef HANDLE_HOTPLUG
	pcan_mutex_t		devices_lock;	/* devices list mutex */
#endif
	u8 *	szVersionString;	/* driver version string */

#ifdef PCCARD_SUPPORT
#ifndef LINUX_24
	struct pcmcia_driver	pccarddrv;	/* pccard driver structure */
#endif
#endif

#ifdef USB_SUPPORT
	struct usb_driver	usbdrv;		/* usb driver structure */
#endif

#if 1//def UDEV_SUPPORT

#ifdef PCI_SUPPORT
	struct pci_driver	pci_drv;	/* pci driver structure */
#endif

#ifdef ISA_SUPPORT
	/* legacy platform driver */
	struct device_driver	legacy_driver_isa;
#endif

#ifdef DONGLE_SUPPORT
	/* legacy platform driver */
	struct device_driver	legacy_driver_dongle;
#endif

	struct class *	class;	/* the associated class of pcan devices */
#endif /* UDEV_SUPPORT */
};

#ifndef NO_RT
struct rt_device {
	struct list_head	list;
	struct rtdm_device *	device;
};
#endif

/* the global driver object */
extern struct pcan_driver pcan_drv;
extern const struct pcanfd_bittiming_range sja1000_capabilities;

extern const pcanfd_mono_clock_device sja1000_clocks;

/* Global functions */
#ifdef USB_SUPPORT
struct pcan_usb_interface *pcan_usb_get_if(struct pcandev *pdev);
#endif

/* request time in msec, fast */
u32 get_mtime(void);

void pcan_add_device_in_list_ex(struct pcandev *dev, u32 flags);
static inline void pcan_add_device_in_list(struct pcandev *dev)
{
	pcan_add_device_in_list_ex(dev, 0);
}

void pcan_dev_remove_from_list(struct pcandev *dev);
int pcan_is_device_in_list(struct pcandev *dev);

#ifdef PCAN_USB_DONT_REGISTER_DEV
int pcan_find_free_minor(struct pcandev *pdev, int from, int until);
#endif

#define PCAN_DEVICE_ATTR(_v, _name, _show) \
	struct device_attribute pcan_dev_attr_##_v = \
				__ATTR(_name, S_IRUGO, _show, NULL)

#define PCAN_DEVICE_ATTR_RW(_v, _name, _show, _store) \
	struct device_attribute pcan_dev_attr_##_v = \
				__ATTR(_name, S_IRUGO|S_IWUSR, _show, _store)

static inline struct pcandev *to_pcandev(struct device *dev)
{
	return (struct pcandev *)dev_get_drvdata(dev);
}

void pcan_soft_init_ex(struct pcandev *dev, char *szType, u16 wType,
			const struct pcanfd_available_clocks *clocks,
			const struct pcanfd_bittiming_range *pc,
			u32 flags);
static inline void pcan_soft_init(struct pcandev *dev, char *szType, u16 wType)
{
	pcan_soft_init_ex(dev, szType, wType,
			(struct pcanfd_available_clocks *)&sja1000_clocks,
			&sja1000_capabilities, 0);
}

void dump_mem(char *prompt, void *p, int l);

#ifdef DEBUG_TX_ENG
static inline void pcan_set_tx_engine_dbg(struct pcandev *dev, int tx_eng,
						const char *f, int l)
{
#ifdef PCAN_USES_OLD_TX_ENGINE_STATE
	if (atomic_read(&dev->tx_engine_state) != tx_eng)
#else
	if (dev->locked_tx_engine_state != tx_eng)
#endif
		pr_info(DEVICE_NAME
			": %s(l=%u): CAN%u TX engine goes to %u\n",
			f, l, dev->nChannel+1, tx_eng);

#ifdef PCAN_USES_OLD_TX_ENGINE_STATE
	atomic_set(&dev->tx_engine_state, tx_eng);
#else
	dev->locked_tx_engine_state = tx_eng;
#endif
}

#define pcan_set_tx_engine(d, s)	pcan_set_tx_engine_dbg(d, s, \
							__func__, __LINE__)

#else
static inline void pcan_set_tx_engine(struct pcandev *dev, int tx_eng)
{
#ifdef PCAN_USES_OLD_TX_ENGINE_STATE
	atomic_set(&dev->tx_engine_state, tx_eng);
#else
	dev->locked_tx_engine_state = tx_eng;
#endif
}
#endif

int pcan_sync_decode(struct pcandev *dev, u32 ts_low, u32 ts_high,
					struct timeval *tv);
int pcan_sync_times(struct pcandev *dev, u32 ts_low, u32 ts_high, u32 dtv_us);

void pcan_set_bus_state(struct pcandev *dev, enum pcanfd_status bus_state);
void pcan_copy_err_counters(struct pcandev *dev, struct pcanfd_msg *pf);

int pcan_handle_busoff(struct pcandev *dev, struct pcanfd_msg *pf);
void pcan_handle_error_active(struct pcandev *dev, struct pcanfd_msg *pf);
int pcan_handle_error_status(struct pcandev *dev, struct pcanfd_msg *pf,
				int err_warning, int err_passive);
void pcan_handle_error_ctrl(struct pcandev *dev, struct pcanfd_msg *pf,
				int err_ctrl);
void pcan_handle_error_msg(struct pcandev *dev, struct pcanfd_msg *pf,
			int err_type, u8 err_code, int err_rx, int err_gen);
void pcan_handle_error_internal(struct pcandev *dev, struct pcanfd_msg *pf,
				int err_internal);
void pcan_handle_error_protocol(struct pcandev *dev, struct pcanfd_msg *pf,
				int err_protocol);

void pcan_soft_error_active(struct pcandev *dev);
void pcan_clear_status_bit(struct pcandev *dev, u16 bits);

u16 sja1000_bitrate(u32 dwBitRate, u32 sample_pt);

/* get bitrate in bps */
static inline u32 pcan_get_bps(u32 clk_Hz, struct pcan_bittiming *pbt)
{
	return clk_Hz / (pbt->brp * (1 + pbt->tseg1 + pbt->tseg2));
}

/* Indicate that a CAN frame is not a CAN 2.0 frame */
static inline int pcan_is_fd(struct pcanfd_msg *pf)
{
        return (pf->type == PCANFD_TYPE_CANFD_MSG);
}

static inline ssize_t show_u32(char *buf, u32 v)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", v);
}

static inline ssize_t show_int(char *buf, int v)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", v);
}

static inline ssize_t show_str(char *buf, char *str)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", str);
}

int pcan_chardev_rx(struct pcandev *dev, struct pcanfd_msg *cf);
int pcan_chardev_msg_rx(struct pcandev *dev, TPCANRdMsg *rdm);

void dev_unregister(void);
#ifdef NO_RT
void pcan_sysfs_dev_node_create(struct pcandev *dev);
void pcan_sysfs_dev_node_destroy(struct pcandev *dev);
#endif

void remove_dev_list(void);

void pcan_init_adapter(struct pcan_adapter *pa, const char *name, int index,
			int can_count);
struct pcan_adapter *pcan_alloc_adapter(const char *name, int index,
			int can_count);

void pcan_inherit_options(struct pcanfd_options *child_opts);

int pcan_sysfs_add_attr(struct device *dev, struct attribute *attrs);
int pcan_sysfs_add_attrs(struct device *dev, struct attribute **attrs);
void pcan_sysfs_del_attr(struct device *dev, struct attribute *attrs);
void pcan_sysfs_del_attrs(struct device *dev, struct attribute **attrs);

#endif /* __PCAN_MAIN_H__ */
