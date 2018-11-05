/*****************************************************************************
 * Copyright (C) 2001-2009  PEAK System-Technik GmbH
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
 *
 * Contributions: Marcel Offermans (marcel.offermans@luminis.nl)
 *                Philipp Baer     (philipp.baer@informatik.uni-ulm.de)
 *                Garth Zeglin     (garthz@ri.cmu.edu)
 *                Harald Koenig    (H.Koenig@science-computing.de)
 *****************************************************************************/
/*****************************************************************************
 *
 * pcan_main.c - the starting point of the driver,
 *               init and cleanup and proc interface
 *
 * $Id$
 *
 *****************************************************************************/
/* #define DEBUG */
/* #undef DEBUG */

#include "src/pcan_common.h"	/* must always be the 1st include */

/* #define KBUILD_MODNAME pcan */

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/capability.h>
#include <linux/param.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#include <asm/system.h>
#endif
#include <asm/uaccess.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(3,9,0)
/* if defined, create_proc_entry() is not used to create /proc/pcan */
#define CREATE_PROC_ENTRY_DEPRECATED
#endif

#ifdef DEBUG
#define DEBUG_PATCH
#define DEBUG_BUS_STATE
#define DEBUG_INVALID_BUS_STATE
#define DEBUG_RX_QUEUE
#define DEBUG_TIMESTAMP_SYNC
#define DEBUG_TIMESTAMP_DECODE
#define DEBUG_TIMESTAMP_IN_THE_FUTURE
#else
//#define DEBUG_PATCH
//#define DEBUG_RX_QUEUE
//#define DEBUG_TIMESTAMP_SYNC
//#define DEBUG_TIMESTAMP_DECODE
//#define DEBUG_TIMESTAMP_HWTYPE	HW_USB_PRO_FD
//#define DEBUG_TIMESTAMP_HWTYPE	HW_PCI_FD
//#define DEBUG_TIMESTAMP_IN_THE_FUTURE
//#define DEBUG_BUS_STATE
#endif

/* if defined, content of STATUS message is checked before being posted, to
 * prevent from flooding RX fifo with identical status msgs
 * This MUST be defined, especially when BUS_LOAD notifications are posted */
#define PCAN_LIMIT_STATUS_FLOODING

/* if defined, pcan fills Tx fifo with data[0]x CAN frames each time it receives
 * a frame with CAN-ID = PCAN_HANDLE_SYNC_FRAME.
 * This MUST NOT being defined except for test version only */
//#define PCAN_HANDLE_SYNC_FRAME	0x2008001

/* if defined, timestamp in Rx event ISNOT hardware based 
 * This SHOULD NOT be defined */
//#define PCAN_DONT_USE_HWTS

#define PCAN_HANDLE_CLOCK_DRIFT		2
/* if defined, clock drift between CPU clock and CAN clock is handled
 * if not defined, HWTIMESTAMP is made of a time base and an offset made of
 * the device clock ticks. These timestamps may give time in the future.
 *
 * Assuming that t(h) = t(d) * S(h) / S(d)
 *
 * with t(h) = Host time in µs 
 *       t(d) = Device time in µs
 *	 S(h) = Host time at sync in µs
 *	 S(d) = Device time at sync in µs
 *
 * Because floating artithmetic is not possible in the Kernel, to save time and
 * compute only at sync time the ratio S(h)/S(d), consider
 * S(h)/S(d) = 1 + k.10-6
 *
 * => t(h) = t(d) * (1 + k.10-6) = t(d) + t(d) * k.10-6
 * 
 * Knowing that t(d) = s(d).10+6 + u(d)
 *
 * with s(d) = count of second in device time and
 *      u(d) count of µs in device time
 *	 
 * => t(h) = t(d) + [s(d).10+6 + u(d)] * k.10-6
 *         = t(d) + [s(d) * k] + [u(d) * k.10-6]
 */

/* if defined, clock drift value is relative to the count of sync done, in order
 * to not handle first silly values...
 * Example: PCAN-USB-PRO Fd: ~200 syncs => clock_drift is stable ~72 µs/s */
#ifdef PCAN_HANDLE_CLOCK_DRIFT
#define PCAN_SYNC_STABLE_DRIFT_LEVEL	200

/* if defined, hw timestamps given to application are checked against future.
 * if any timestamp is greater than now, then it is set to now.
 * If not defined, application may receive timestamps from the future...
 */
#define PCAN_FIX_FUTURE_TS
#endif

#ifdef PCI_SUPPORT
#include "src/pcan_pci.h"	/* get support for PCAN-PCI */
#endif
#ifdef ISA_SUPPORT
#include "src/pcan_isa.h"	/* get support for PCAN-ISA and PCAN-104 */
#endif
#ifdef DONGLE_SUPPORT
#include "src/pcan_dongle.h"	/* get support for PCAN-Dongle */
#endif
#ifdef USB_SUPPORT
#include "src/pcan_usb_core.h"	/* get support for PCAN-USB */
#endif
#ifdef PCCARD_SUPPORT
#include "src/pcan_pccard.h"
#endif
#ifdef NETDEV_SUPPORT
#include "src/pcan_netdev.h"
#endif

#include "src/pcanfd_core.h"
#include "src/pcan_fifo.h"
#include "src/pcan_filter.h"
#include "src/pcan_sja1000.h"

#define DEFAULT_BTR0BTR1	CAN_BAUD_500K	/* defaults to 500 kbit/sec */
#define DEFAULT_DBITRATE	2000000		/* default data bitrate = 2M */

/* filled by module initialisation */
char *type[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
u16  io[8]    = {0, 0, 0, 0, 0, 0, 0, 0};
u8   irq[8]   = {0, 0, 0, 0, 0, 0, 0, 0};
u16  btr0btr1  = DEFAULT_BTR0BTR1;
char *assign  = NULL;
char *bitrate = NULL;
char *dbitrate = NULL;

/* the global driver object, create it */
struct pcan_driver pcan_drv = {};

static u32 pcan_def_bitrate = 0;
static u32 pcan_def_dbitrate = DEFAULT_DBITRATE;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
/* some stuff to support SysFS coming with kernel 2.6 */
#include <linux/device.h>
#endif

/* build current driver config string for output in kernel log and procfs */
const char current_config[] = " "
#ifdef DEBUG
"[dbg] "
#endif
#ifdef MODVERSIONS
"[mod] "
#endif
#ifdef ISA_SUPPORT
"[isa] "
#endif
#ifdef PCI_SUPPORT
"[pci] "
#endif
#ifdef PCIEC_SUPPORT
"[pec] "
#endif
#ifdef DONGLE_SUPPORT
"[dng] "
#endif
#ifdef PARPORT_SUBSYSTEM
"[par] "
#endif
#ifdef USB_SUPPORT
"[usb] "
#endif
#ifdef PCCARD_SUPPORT
"[pcc] "
#endif
#ifdef NETDEV_SUPPORT
"[net] "
#endif
#ifndef NO_RT
"[rt] "
#endif
;

#define PCAN_DEV_RXQSIZE_MIN	50
#define PCAN_DEV_RXQSIZE_MAX	999

#define PCAN_DEV_TXQSIZE_MIN	50
#define PCAN_DEV_TXQSIZE_MAX	999

extern ushort rxqsize;
extern ushort txqsize;

#define PCAN_DEV_DMA_MASK_DEF	64
#define PCAN_DEV_DMA_MASK_LOW	24
#define PCAN_DEV_DMA_MASK_HIGH	64

ushort dmamask = PCAN_DEV_DMA_MASK_DEF;

module_param(dmamask, ushort, 0644);
MODULE_PARM_DESC(dmamask, " ["
			__stringify(PCAN_DEV_DMA_MASK_LOW) ".."
			__stringify(PCAN_DEV_DMA_MASK_HIGH) "] (def="
			__stringify(PCAN_DEV_DMA_MASK_DEF) ")");

#define PCANFD_OPT_HWTIMESTAMP_DEF	PCANFD_OPT_HWTIMESTAMP_MAX
#define PCANFD_OPT_HWTIMESTAMP_LOW	PCANFD_OPT_HWTIMESTAMP_OFF
#define PCANFD_OPT_HWTIMESTAMP_HIGH	PCANFD_OPT_HWTIMESTAMP_MAX-1

static ushort deftsmode = PCANFD_OPT_HWTIMESTAMP_DEF;	/* use default ts_mode */
module_param(deftsmode, ushort, 0644);
MODULE_PARM_DESC(deftsmode, " default ts mode");

#ifdef UDEV_SUPPORT
#define SYSFS_SUPPORT
#endif

/* for procfs output the current_config is copied into this centered string */
char config[] = "*----------------------------------------------------------------------------";

#ifdef SYSFS_SUPPORT

/* linux < 2.6.27 use device_create_drvdata() */
#ifndef device_create_drvdata
#define	device_create_drvdata	device_create
#endif

static ssize_t show_pcan_devid(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev)->device_alt_num);
}

static ssize_t store_pcan_devid(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct pcandev *pdev = to_pcandev(dev);

	if (pdev->device_params) {
		char *endptr;
		int err;
		TPEXTRAPARAMS exp = {
			.nSubFunction = SF_SET_HCDEVICENO,
		};

		exp.func.dwSerialNumber = simple_strtoul(buf, &endptr, 0);
		if (*endptr != '\n')
			return -EINVAL;

		err = pdev->device_params(pdev, &exp);
		if (err)
			return err;

		pdev->device_alt_num = exp.func.dwSerialNumber;
	}

	return count;
}

static ssize_t show_pcan_hwtype(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev)->wType);
}

static ssize_t show_pcan_minor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_int(buf, to_pcandev(dev)->nMinor);
}

static ssize_t show_pcan_ctrlr_number(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pcandev *pdev = to_pcandev(dev);
	int c = pdev->nChannel;

#ifdef USB_SUPPORT
	if (pdev->wType == HW_USB_X6) {
		struct pcan_usb_interface *usb_if;

		usb_if = pcan_usb_get_if(pdev);

		c += usb_if->index * usb_if->can_count;
	}
#endif
	return show_int(buf, c);
}

static ssize_t show_pcan_bitrate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef NETDEV_SUPPORT
	struct can_bittiming *pbt = pcan_netdev_get_bittiming(to_pcandev(dev));
	return show_u32(buf, pbt->bitrate);
#else
	return show_u32(buf, to_pcandev(dev)->init_settings.nominal.bitrate);
#endif
}

static ssize_t show_pcan_nsp(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef NETDEV_SUPPORT
	struct can_bittiming *pbt = pcan_netdev_get_dbittiming(to_pcandev(dev));
	if (pbt)
		return show_u32(buf, pbt->sample_point);
#endif
	return show_u32(buf,
			to_pcandev(dev)->init_settings.nominal.sample_point);
}

static ssize_t show_pcan_nom_tq(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef NETDEV_SUPPORT
	struct can_bittiming *pbt = pcan_netdev_get_dbittiming(to_pcandev(dev));
	if (pbt)
		return show_u32(buf, pbt->tq);
#endif
	return show_u32(buf, to_pcandev(dev)->init_settings.nominal.tq);
}

static ssize_t show_pcan_nom_brp(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef NETDEV_SUPPORT
	struct can_bittiming *pbt = pcan_netdev_get_bittiming(to_pcandev(dev));
	return show_u32(buf, pbt->brp);
#else
	return show_u32(buf, to_pcandev(dev)->init_settings.nominal.brp);
#endif
}

static ssize_t show_pcan_nom_tseg1(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef NETDEV_SUPPORT
	struct can_bittiming *pbt = pcan_netdev_get_bittiming(to_pcandev(dev));
	u32 tseg1 = pbt->prop_seg + pbt->phase_seg1;
	return show_u32(buf, tseg1);
#else
	return show_u32(buf, to_pcandev(dev)->init_settings.nominal.tseg1);
#endif
}

static ssize_t show_pcan_nom_tseg2(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef NETDEV_SUPPORT
	struct can_bittiming *pbt = pcan_netdev_get_bittiming(to_pcandev(dev));
	return show_u32(buf, pbt->phase_seg2);
#else
	return show_u32(buf, to_pcandev(dev)->init_settings.nominal.tseg2);
#endif
}

static ssize_t show_pcan_nom_sjw(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef NETDEV_SUPPORT
	struct can_bittiming *pbt = pcan_netdev_get_bittiming(to_pcandev(dev));
	return show_u32(buf, pbt->sjw);
#else
	return show_u32(buf, to_pcandev(dev)->init_settings.nominal.sjw);
#endif
}

static ssize_t show_pcan_init_flags(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%08x\n",
					to_pcandev(dev)->init_settings.flags);
}

static ssize_t show_pcan_clock(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev)->init_settings.clock_Hz);
}

static ssize_t show_pcan_bus_state(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev)->bus_state);
}

static ssize_t show_pcan_rx_err_cnt(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev)->rx_error_counter);
}

static ssize_t show_pcan_tx_err_cnt(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev)->tx_error_counter);
}

static ssize_t show_pcan_bus_load(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pcandev *pdev = to_pcandev(dev);
	return snprintf(buf, PAGE_SIZE, "%u.%02u\n",
			pdev->bus_load / 100, pdev->bus_load % 100);
}

/* only when dev->adapter is not NULL! */
static ssize_t show_pcan_adapter_number(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_int(buf, to_pcandev(dev)->adapter->index);
}

static ssize_t show_pcan_adapter_name(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pcandev *pdev = to_pcandev(dev);
	int l = 0;

	if (pdev->adapter->name)
		l += snprintf(buf+l, PAGE_SIZE, "%s", pdev->adapter->name);

	buf[l++] = '\n';
	buf[l++] = '\0';

	return l;
}

static ssize_t show_pcan_adapter_version(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pcandev *pdev = to_pcandev(dev);
	int l = 0;

	if (pdev->adapter->hw_ver_major >= 0) {
		l += snprintf(buf+l, PAGE_SIZE, "%u",
					pdev->adapter->hw_ver_major);
		if (pdev->adapter->hw_ver_minor >= 0) {
			l += snprintf(buf+l, PAGE_SIZE, ".%u",
					pdev->adapter->hw_ver_minor);

			if (pdev->adapter->hw_ver_subminor >= 0)
				l += snprintf(buf+l, PAGE_SIZE, ".%u",
						pdev->adapter->hw_ver_subminor);
		}

		if (l >= PAGE_SIZE)
			l = PAGE_SIZE - 2;
	}

	buf[l++] = '\n';
	buf[l++] = '\0';

	return l;
}

/* /proc/pcan redundant */
static ssize_t show_pcan_type(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_str(buf, to_pcandev(dev)->type);
}

#ifdef NETDEV_SUPPORT
static ssize_t show_pcan_ndev(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pcandev *pdev = to_pcandev(dev);
	return show_str(buf, pdev->netdev ? pdev->netdev->name : "can?");
}
#endif

static ssize_t show_pcan_base(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", to_pcandev(dev)->dwPort);
}

static ssize_t show_rx_fifo_ratio(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pcandev *pdev = to_pcandev(dev);
	u32 fifo_ratio = pcan_fifo_ratio(&pdev->readFifo);
	return snprintf(buf, PAGE_SIZE, "%u.%02u\n",
			fifo_ratio / 100, fifo_ratio % 100);
}

static ssize_t show_tx_fifo_ratio(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pcandev *pdev = to_pcandev(dev);
	u32 fifo_ratio = pcan_fifo_ratio(&pdev->writeFifo);
	return snprintf(buf, PAGE_SIZE, "%u.%02u\n",
			fifo_ratio / 100, fifo_ratio % 100);
}

static ssize_t show_pcan_irq(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev)->wIrq);
}

static ssize_t show_pcan_btr0btr1(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pcandev *pdev = to_pcandev(dev);
	u32 dev_btr0btr1 = sja1000_bitrate(pdev->init_settings.nominal.bitrate,
				pdev->init_settings.nominal.sample_point);
	return snprintf(buf, PAGE_SIZE, "0x%04x\n", dev_btr0btr1);
}

static ssize_t show_pcan_read(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pcandev *pdev = to_pcandev(dev);
#ifdef NETDEV_SUPPORT
	struct net_device_stats *stats = (pdev->netdev) ?
				pcan_netdev_get_stats(pdev->netdev) : NULL;
	u32 dev_read = (stats) ? stats->rx_packets : 0;

#else
	u32 dev_read = pdev->readFifo.dwTotal;
#endif
	return show_u32(buf, dev_read);
}

static ssize_t show_pcan_write(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pcandev *pdev = to_pcandev(dev);
#ifdef NETDEV_SUPPORT
	struct net_device_stats *stats = (pdev->netdev) ?
				pcan_netdev_get_stats(pdev->netdev) : NULL;
	u32 dev_write = (stats) ? stats->tx_packets : 0;

#else
	u32 dev_write = pdev->writeFifo.dwTotal;
#endif
	return show_u32(buf, dev_write);
}

static ssize_t show_pcan_irqs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev)->dwInterruptCounter);
}

static ssize_t show_pcan_errors(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev)->dwErrorCounter);
}

static ssize_t show_pcan_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04x\n",
						to_pcandev(dev)->wCANStatus);
}

//static PCAN_DEVICE_ATTR(devid, devid, show_pcan_devid);
static PCAN_DEVICE_ATTR_RW(devid, devid, show_pcan_devid, store_pcan_devid);
static PCAN_DEVICE_ATTR(hwtype, hwtype, show_pcan_hwtype);
static PCAN_DEVICE_ATTR(minor, minor, show_pcan_minor);
static PCAN_DEVICE_ATTR(ctrlr_number, ctrlr_number, show_pcan_ctrlr_number);
static PCAN_DEVICE_ATTR(bitrate, nom_bitrate, show_pcan_bitrate);
static PCAN_DEVICE_ATTR(sample_point, nom_sample_point, show_pcan_nsp);
static PCAN_DEVICE_ATTR(nom_tq, nom_tq, show_pcan_nom_tq);
static PCAN_DEVICE_ATTR(nom_brp, nom_brp, show_pcan_nom_brp);
static PCAN_DEVICE_ATTR(nom_tseg1, nom_tseg1, show_pcan_nom_tseg1);
static PCAN_DEVICE_ATTR(nom_tseg2, nom_tseg2, show_pcan_nom_tseg2);
static PCAN_DEVICE_ATTR(nom_sjw, nom_sjw, show_pcan_nom_sjw);
static PCAN_DEVICE_ATTR(init_flags, init_flags, show_pcan_init_flags);
static PCAN_DEVICE_ATTR(clock, clock, show_pcan_clock);
static PCAN_DEVICE_ATTR(bus_state, bus_state, show_pcan_bus_state);
/* /proc/pcan redundant */
static PCAN_DEVICE_ATTR(type, type, show_pcan_type);
#ifdef NETDEV_SUPPORT
static PCAN_DEVICE_ATTR(ndev, ndev, show_pcan_ndev);
#endif
static PCAN_DEVICE_ATTR(base, base, show_pcan_base);
static PCAN_DEVICE_ATTR(irq, irq, show_pcan_irq);
static PCAN_DEVICE_ATTR(btr0btr1, btr0btr1, show_pcan_btr0btr1);
static PCAN_DEVICE_ATTR(read, read, show_pcan_read);
static PCAN_DEVICE_ATTR(write, write, show_pcan_write);
static PCAN_DEVICE_ATTR(errors, errors, show_pcan_errors);
static PCAN_DEVICE_ATTR(irqs, irqs, show_pcan_irqs);
static PCAN_DEVICE_ATTR(status, status, show_pcan_status);
static PCAN_DEVICE_ATTR(rx_fifo_ratio, rx_fifo_ratio, show_rx_fifo_ratio);
static PCAN_DEVICE_ATTR(tx_fifo_ratio, tx_fifo_ratio, show_tx_fifo_ratio);

static struct attribute *pcan_dev_sysfs_attrs[] = {
	&pcan_dev_attr_devid.attr,
	&pcan_dev_attr_hwtype.attr,
	&pcan_dev_attr_minor.attr,
	&pcan_dev_attr_ctrlr_number.attr,
	&pcan_dev_attr_bitrate.attr,
	&pcan_dev_attr_sample_point.attr,
	&pcan_dev_attr_nom_tq.attr,
	&pcan_dev_attr_nom_brp.attr,
	&pcan_dev_attr_nom_tseg1.attr,
	&pcan_dev_attr_nom_tseg2.attr,
	&pcan_dev_attr_nom_sjw.attr,
	&pcan_dev_attr_init_flags.attr,
	&pcan_dev_attr_clock.attr,
	&pcan_dev_attr_bus_state.attr,
	/* /proc/pcan redundant */
	&pcan_dev_attr_type.attr,
#ifdef NETDEV_SUPPORT
	&pcan_dev_attr_ndev.attr,
#endif
	&pcan_dev_attr_base.attr,
	&pcan_dev_attr_irq.attr,
	&pcan_dev_attr_btr0btr1.attr,
	&pcan_dev_attr_read.attr,
	&pcan_dev_attr_write.attr,
	&pcan_dev_attr_errors.attr,
	&pcan_dev_attr_irqs.attr,
	&pcan_dev_attr_status.attr,
	&pcan_dev_attr_rx_fifo_ratio.attr,
	&pcan_dev_attr_tx_fifo_ratio.attr,
	NULL
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
static struct attribute_group pcan_dev_attrs_group = {
	/* NULL ".name" => attrs will be created under pcanxxx node 
	 * .name = "pcan-dev", 
	 */
	.attrs = pcan_dev_sysfs_attrs,
};
static const struct attribute_group *pcan_dev_attrs_groups[] = {
	&pcan_dev_attrs_group,
	NULL,
};
#endif

static ssize_t show_pcan_dbitrate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef NETDEV_SUPPORT
	struct can_bittiming *pbt = pcan_netdev_get_dbittiming(to_pcandev(dev));
	if (pbt)
		return show_u32(buf, pbt->bitrate);
#endif
	return show_u32(buf, to_pcandev(dev)->init_settings.data.bitrate);
}

static ssize_t show_pcan_dsp(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef NETDEV_SUPPORT
	struct can_bittiming *pbt = pcan_netdev_get_dbittiming(to_pcandev(dev));
	if (pbt)
		return show_u32(buf, pbt->sample_point);
#endif
	return show_u32(buf, to_pcandev(dev)->init_settings.data.sample_point);
}

static ssize_t show_pcan_data_tq(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef NETDEV_SUPPORT
	struct can_bittiming *pbt = pcan_netdev_get_dbittiming(to_pcandev(dev));
	if (pbt)
		return show_u32(buf, pbt->tq);
#endif
	return show_u32(buf, to_pcandev(dev)->init_settings.data.tq);
}

static ssize_t show_pcan_data_brp(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef NETDEV_SUPPORT
	struct can_bittiming *pbt = pcan_netdev_get_dbittiming(to_pcandev(dev));
	if (pbt)
		return show_u32(buf, pbt->brp);
#endif
	return show_u32(buf, to_pcandev(dev)->init_settings.data.brp);
}

static ssize_t show_pcan_data_tseg1(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef NETDEV_SUPPORT
	struct can_bittiming *pbt = pcan_netdev_get_dbittiming(to_pcandev(dev));
	if (pbt) {
		u32 tseg1 = pbt->prop_seg + pbt->phase_seg1;
		return show_u32(buf, tseg1);
	}
#endif
	return show_u32(buf, to_pcandev(dev)->init_settings.data.tseg1);
}

static ssize_t show_pcan_data_tseg2(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef NETDEV_SUPPORT
	struct can_bittiming *pbt = pcan_netdev_get_dbittiming(to_pcandev(dev));
	if (pbt)
		return show_u32(buf, pbt->phase_seg2);
#endif
	return show_u32(buf, to_pcandev(dev)->init_settings.data.tseg2);
}

static ssize_t show_pcan_data_sjw(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef NETDEV_SUPPORT
	struct can_bittiming *pbt = pcan_netdev_get_dbittiming(to_pcandev(dev));
	if (pbt)
		return show_u32(buf, pbt->sjw);
#endif
	return show_u32(buf, to_pcandev(dev)->init_settings.data.sjw);
}

static PCAN_DEVICE_ATTR(dbitrate, data_bitrate, show_pcan_dbitrate);
static PCAN_DEVICE_ATTR(dsample_point, data_sample_point, show_pcan_dsp);
static PCAN_DEVICE_ATTR(data_tq, data_tq, show_pcan_data_tq);
static PCAN_DEVICE_ATTR(data_brp, data_brp, show_pcan_data_brp);
static PCAN_DEVICE_ATTR(data_tseg1, data_tseg1, show_pcan_data_tseg1);
static PCAN_DEVICE_ATTR(data_tseg2, data_tseg2, show_pcan_data_tseg2);
static PCAN_DEVICE_ATTR(data_sjw, data_sjw, show_pcan_data_sjw);

static struct attribute *pcan_dev_sysfs_fd_attrs[] = {
	&pcan_dev_attr_dbitrate.attr,
	&pcan_dev_attr_dsample_point.attr,
	&pcan_dev_attr_data_tq.attr,
	&pcan_dev_attr_data_brp.attr,
	&pcan_dev_attr_data_tseg1.attr,
	&pcan_dev_attr_data_tseg2.attr,
	&pcan_dev_attr_data_sjw.attr,

	NULL
};

static PCAN_DEVICE_ATTR(bus_load, bus_load, show_pcan_bus_load);
static PCAN_DEVICE_ATTR(rx_err_cnt, rx_error_counter, show_pcan_rx_err_cnt);
static PCAN_DEVICE_ATTR(tx_err_cnt, tx_error_counter, show_pcan_tx_err_cnt);

static struct attribute *pcan_dev_sysfs_err_cnt_attrs[] = {
	&pcan_dev_attr_rx_err_cnt.attr,
	&pcan_dev_attr_tx_err_cnt.attr,

	NULL
};

static PCAN_DEVICE_ATTR(adapter_number, adapter_number,\
			show_pcan_adapter_number);
static PCAN_DEVICE_ATTR(adapter_name, adapter_name, show_pcan_adapter_name);
static PCAN_DEVICE_ATTR(adapter_version, adapter_version,\
			show_pcan_adapter_version);

static struct attribute *pcan_dev_sysfs_adapter_attrs[] = {
	&pcan_dev_attr_adapter_number.attr,
	&pcan_dev_attr_adapter_name.attr,
	&pcan_dev_attr_adapter_version.attr,

	NULL,
};
#endif /* SYSFS_SUPPORT */

/* create a UDEV allocated device node */
void pcan_sysfs_dev_node_create(struct pcandev *dev)
{
#ifdef SYSFS_SUPPORT
	char tmp[32];

#ifdef PCAN_USB_PCAN_SYSFS
#ifndef PCAN_USB_DONT_REGISTER_DEV
	int minor = dev->nMinor;

	if (dev->nMajor == USB_MAJOR)
		minor += PCAN_USB_MINOR_BASE;
#endif
#endif

	/* tinker my device node name, eg. "pcanpci%d" */
	snprintf(tmp, sizeof(tmp), DEVICE_NAME "%s%s", dev->type, "%u");

	DPRINTK(KERN_DEBUG "%s: %s(%s, %d, %d)\n",
			DEVICE_NAME, __func__, tmp,
			pcan_drv.nMajor, dev->nMinor);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 11, 0)
	/* Should use WAIT_FOR key in Udev rules... */
	dev->sysfs_dev = device_create_drvdata(pcan_drv.class, NULL,
#if defined(PCAN_USB_PCAN_SYSFS) && !defined(PCAN_USB_DONT_REGISTER_DEV)
				MKDEV(pcan_drv.nMajor, minor),
				dev, tmp, minor);
#else
				MKDEV(dev->nMajor, dev->nMinor),
				dev, tmp, dev->nMinor);
#endif

	if (!IS_ERR(dev->sysfs_dev)) {
		pcan_sysfs_add_attrs(dev->sysfs_dev, pcan_dev_sysfs_attrs);
#else
	/* since 3.11, it is possible to add attrs when creating the device
	 * node, that is *BEFORE* the UEVENT is beeing sent to userspace!
	 * Doing this, Udev rules does not need of WAIT_FOR key anymore! */
	dev->sysfs_dev = device_create_with_groups(pcan_drv.class, NULL,
#if defined(PCAN_USB_PCAN_SYSFS) && !defined(PCAN_USB_DONT_REGISTER_DEV)
				MKDEV(pcan_drv.nMajor, minor),
				dev, pcan_dev_attrs_groups, tmp, minor);
#else
				MKDEV(dev->nMajor, dev->nMinor),
				dev, pcan_dev_attrs_groups, tmp, dev->nMinor);
#endif

	if (!IS_ERR(dev->sysfs_dev)) {
#endif /* KERNEL_VERSION(3, 11, 0) */

		/* these attrs are are not used with Udev rules... */
		if (dev->adapter)
			pcan_sysfs_add_attrs(dev->sysfs_dev,
						pcan_dev_sysfs_adapter_attrs);

		if (dev->device_open_fd)
			pcan_sysfs_add_attrs(dev->sysfs_dev,
						pcan_dev_sysfs_fd_attrs);

		if (dev->flags & PCAN_DEV_BUSLOAD_RDY)
			pcan_sysfs_add_attr(dev->sysfs_dev,
						&pcan_dev_attr_bus_load.attr);

		if (dev->flags & PCAN_DEV_ERRCNT_RDY)
			pcan_sysfs_add_attrs(dev->sysfs_dev,
						pcan_dev_sysfs_err_cnt_attrs);

		if (dev->sysfs_attrs)
			pcan_sysfs_add_attrs(dev->sysfs_dev, dev->sysfs_attrs);

	} else {
		 dev->sysfs_dev = NULL;
	}
#else
	dev->sysfs_dev = NULL;
#endif
}

/* destroy a UDEV allocated device node */
void pcan_sysfs_dev_node_destroy(struct pcandev *dev)
{
#ifdef SYSFS_SUPPORT
	DPRINTK(KERN_DEBUG "%s: %s(%p=\"%s\")\n",
			DEVICE_NAME, __func__, dev,
			(dev->sysfs_dev) ? dev->sysfs_dev->kobj.name : "none");

	if (dev->sysfs_dev) {

		if (dev->sysfs_attrs)
			pcan_sysfs_del_attrs(dev->sysfs_dev, dev->sysfs_attrs);

		if (dev->flags & PCAN_DEV_ERRCNT_RDY)
			pcan_sysfs_del_attrs(dev->sysfs_dev,
						pcan_dev_sysfs_err_cnt_attrs);

		if (dev->flags & PCAN_DEV_BUSLOAD_RDY)
			pcan_sysfs_del_attr(dev->sysfs_dev,
						&pcan_dev_attr_bus_load.attr);

		if (dev->device_open_fd)
			pcan_sysfs_del_attrs(dev->sysfs_dev,
						pcan_dev_sysfs_fd_attrs);

		if (dev->adapter)
			pcan_sysfs_del_attrs(dev->sysfs_dev,
						pcan_dev_sysfs_adapter_attrs);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 11, 0)
		pcan_sysfs_del_attrs(dev->sysfs_dev, pcan_dev_sysfs_attrs);
#endif
		device_del(dev->sysfs_dev);

		dev->sysfs_dev = NULL;
	}
#endif
}

#ifdef NO_RT
#include "pcan_main_linux.c"
#else
#include "pcan_main_rt.c"
#endif

#define DUMP_MAX	64

/*
 * void dump_mem(char *prompt, void *p, int l)
 */
void dump_mem(char *prompt, void *p, int l)
{
#ifdef DEBUG
	char *kern = KERN_DEBUG;
#else
	char *kern = KERN_INFO;
#endif
	uint8_t *pc = (uint8_t *)p;
	int i, ld;

	if (l > DUMP_MAX) {
		ld = DUMP_MAX/2;
		printk("%s%s: Dumping %s (%d/%d bytes):\n",
			kern, DEVICE_NAME, prompt?prompt:"memory", DUMP_MAX, l);
	} else {
		ld = l;
		printk("%s%s: Dumping %s (%d bytes):\n",
			kern, DEVICE_NAME, prompt?prompt:"memory", l);
	}
	for (i = 0; i < ld; ) {
		if (!(i % 16))
			printk("%s%s: ", kern, DEVICE_NAME);
		printk("%02X ", *pc++);
		if (!(++i % 16))
			printk("\n");
	}
	if (i % 16)
		printk("\n");

	if (ld < l) {
		printk("%s%s: ...\n", kern, DEVICE_NAME);
		pc = pc + l - ld;
		for (i = 0; i < ld; ) {
			if (!(i % 16))
				printk("%s%s: ", kern, DEVICE_NAME);
			printk("%02X ", *pc++);
			if (!(++i % 16))
				printk("\n");
		}
		if (i % 16)
			printk("\n");
	}
}

#if 1
/* keep this for historical reasons, in particular, to check how errors were
 * given to user application in the old API... */
#else
/* convert struct can_frame to struct TPCANMsg
 * To reduce the complexity (and CPU usage) there are no checks (e.g. for dlc)
 * here as it is assumed that the creator of the source struct has done this
 * work */
void frame2msg(struct can_frame *cf, TPCANMsg *msg)
{
	if (cf->can_id & CAN_ERR_FLAG) {
		memset(msg, 0, sizeof(*msg));
		msg->MSGTYPE = MSGTYPE_STATUS;
		msg->LEN     = 4;

		if (cf->can_id & CAN_ERR_CRTL) {
			// handle data overrun
			if (cf->data[1] & CAN_ERR_CRTL_RX_OVERFLOW)
				msg->DATA[3] |= CAN_ERR_OVERRUN;

			// handle CAN_ERR_BUSHEAVY
			if (cf->data[1] & CAN_ERR_CRTL_RX_WARNING)
				msg->DATA[3] |= CAN_ERR_BUSHEAVY;
		}

		if (cf->can_id & CAN_ERR_BUSOFF_NETDEV)
			msg->DATA[3] |= CAN_ERR_BUSOFF;

		return;
	}

	if (cf->can_id & CAN_RTR_FLAG)
		msg->MSGTYPE = MSGTYPE_RTR;
	else
		msg->MSGTYPE = MSGTYPE_STANDARD;

	if (cf->can_id & CAN_EFF_FLAG)
		msg->MSGTYPE |= MSGTYPE_EXTENDED;

	msg->ID  = cf->can_id & CAN_EFF_MASK; /* remove EFF/RTR/ERR flags */
	msg->LEN = cf->can_dlc; /* no need to check value range here */

	memcpy(&msg->DATA[0], &cf->data[0], 8); /* also copy trailing zeros */
}

/* convert struct TPCANMsg to struct can_frame
 * To reduce the complexity (and CPU usage) there are no checks (e.g. for dlc)
 * here as it is assumed that the creator of the source struct has done this
 * work */
void msg2frame(struct can_frame *cf, TPCANMsg *msg)
{
	cf->can_id = msg->ID;

	if (msg->MSGTYPE & MSGTYPE_RTR)
		cf->can_id |= CAN_RTR_FLAG;

	if (msg->MSGTYPE & MSGTYPE_EXTENDED)
		cf->can_id |= CAN_EFF_FLAG;

	// if (msg->MSGTYPE & MSGTYPE_??????)
	//   cf->can_id |= CAN_ERR_FLAG;

	cf->can_dlc = msg->LEN; /* no need to check value range here */

	memcpy(&cf->data[0], &msg->DATA[0], 8); /* also copy trailing zeros */
}
#endif

/* x = (x >= y) ? x - y : 0; */
static int subtract_timeval(struct timeval *x, struct timeval *y)
{
	if (x->tv_usec >= y->tv_usec)
		x->tv_usec -= y->tv_usec;
	else {
		if (x->tv_sec) {
			x->tv_sec--;
			x->tv_usec += (USEC_PER_SEC - y->tv_usec);
		} else
			goto fail;
	}

	if (x->tv_sec >= y->tv_sec) {
		x->tv_sec -= y->tv_sec;
		return 1;
	}

fail:
	return 0;
}

/* get relative time to start of driver */
static void to_drv_rel_time(struct timeval *tv)
{
	if (!subtract_timeval(tv, &pcan_drv.sInitTime)) {

#ifdef DEBUG
		pr_info(DEVICE_NAME
			": WARNING: \"%u.%06u s.\" < drv \"%u.%06u s.\"\n",
			(u32 )tv->tv_sec, (u32 )tv->tv_usec,
			(u32 )pcan_drv.sInitTime.tv_sec,
			(u32 )pcan_drv.sInitTime.tv_usec);
#endif
		tv->tv_sec = tv->tv_usec = 0;
	}
}

/* get relative time to start of device */
static void to_dev_rel_time(struct pcandev *dev, struct timeval *tv)
{
	if (!subtract_timeval(tv, &dev->init_timestamp)) {

#ifdef DEBUG
		pr_info(DEVICE_NAME
			": WARNING: \"%u.%06u s.\" < dev \"%u.%06u s.\"\n",
			(u32 )tv->tv_sec, (u32 )tv->tv_usec,
			(u32 )dev->init_timestamp.tv_sec,
			(u32 )dev->init_timestamp.tv_usec);
#endif
		tv->tv_sec = tv->tv_usec = 0;
	}
}

/* time is relative */
static void pcan_handle_timestamp(struct pcandev *dev, struct pcanfd_msg *pf)
{
	/* raw timestamsp cannot be changed according to any time base */
	if (dev->ts_mode == PCANFD_OPT_HWTIMESTAMP_RAW)
		return;

	switch (dev->init_settings.flags & PCANFD_INIT_TS_FMT_MASK) {

	case PCANFD_INIT_TS_DRV_REL:
		to_drv_rel_time(&pf->timestamp);
		break;

	case PCANFD_INIT_TS_DEV_REL: 
		to_dev_rel_time(dev, &pf->timestamp);
		break;

	case PCANFD_INIT_TS_HOST_REL:
		break;
	}
}

static int pcan_status_bus_load_rx(struct pcandev *dev, struct pcanfd_msg *pf)
{
	u8 *pb = pf->data;

	/* if user has not asked to be informed with bus_load notifications,
	 * or if bus load value has not changed, do nothing */
	if (!(dev->init_settings.flags & PCANFD_INIT_BUS_LOAD_INFO) ||
	     (dev->bus_load == dev->prev_bus_load))
		return -EEXIST;

	memset(pb, '\0', sizeof(pf->data));

#if 1
	/* now, bus_load is saved into all msgs, in ctrlr_data[] field */
#else
	pf->data_len = 2;
	memcpy(pb, &dev->bus_load, 2);
#endif
	dev->prev_bus_load = dev->bus_load;

	return 0;
}

#ifdef PCAN_LIMIT_STATUS_FLOODING
/* this function is used to prevent from flooding rx fifo with STATUS msgs 
 * given by the hardware, especially when this hw is able to give rx and tx
 * error counters: when bus goes into warning or passive states, there can be 
 * a lot a STATUS msgs pushed in the rx fifo. If no task is reading this fifo
 * fast enough, it can be quickly full, preventing from CAN frames to be pushed.
 * */
static int pcan_do_patch_status(void *item, void *arg)
{
	struct pcanfd_msg *fifo_msg = (struct pcanfd_msg *)item;
	struct pcanfd_msg *status_msg = (struct pcanfd_msg *)arg;

	/* if last msg is also the same status msg, excepting counters value,
	 * it can be overwritten */
	if ((fifo_msg->type == PCANFD_TYPE_STATUS) && /* same STATUS msg */
	    (fifo_msg->id == status_msg->id) &&		 /* same id. */
	    (fifo_msg->flags == status_msg->flags) && /* same flags */
	    (fifo_msg->data_len == status_msg->data_len) /* same content len */
	   ) {
		/* patch fifo item with the new message, that is:
		 * - new timestamp
		 * - new data bytes */
		*fifo_msg = *status_msg;

#ifdef DEBUG_PATCH
		pr_info(DEVICE_NAME ": %s(): STATUS[%u] patched\n",
				__func__, fifo_msg->id);
#endif
		return -EEXIST;
	}

	/* MUST return != 0 to only process last item */
	return -ENOENT;
}

/* this function patches the last msg pushed into the fifo with *arg */
static int pcan_do_patch_last(void *item, void *arg)
{
	struct pcanfd_msg *fifo_msg = (struct pcanfd_msg *)item;
	struct pcanfd_msg *new_msg = (struct pcanfd_msg *)arg;

	*fifo_msg = *new_msg;

#if 0//def DEBUG_PATCH
	pr_info(DEVICE_NAME ": %s(): event[%d] changed into event[%d]\n",
			__func__, fifo_msg->type, new_msg->type);
#endif

	/* MUST return != 0 to only process last item */
	return -EEXIST;
}

static int pcan_status_error_rx(struct pcandev *dev, struct pcanfd_msg *pf)
{
	u8 *pb = pf->data;

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(%s CAN%u, pf[id=%u]) "
			"bus=%u rx=%u/%u tx=%u/%u\n",
			__func__, dev->adapter->name, dev->nChannel+1,
			pf->id, dev->bus_state,
			dev->rx_error_counter, dev->prev_rx_error_counter,
			dev->tx_error_counter, dev->prev_tx_error_counter);
#endif
	/* Note: this works ONLY if bus_state ISNOT set BEFORE posting msg
	 * *pf... which is not guaranteed! Fixed by setting a value to prev_
	 * error counters different from error counters (see
	 * pcan_set_bus_state() */

	/* if always in the same state, do post only if counters have changed */
	if ((dev->bus_state == pf->id) &&
	    (dev->rx_error_counter == dev->prev_rx_error_counter) &&
	    (dev->tx_error_counter == dev->prev_tx_error_counter))
		return -EEXIST;

	memset(pb, '\0', sizeof(pf->data));

	/* default is: no counters given to userspace */
	pf->data_len = 0;

	if (dev->flags & PCAN_DEV_ERRCNT_RDY) {

		/* don't post if rx/tx counters don't match current bus state.
		 * The uCAN STATUS event should arrive next... */
		switch (dev->bus_state) {
		case PCANFD_ERROR_ACTIVE:
			if ((dev->rx_error_counter >= 96) ||
						(dev->tx_error_counter >= 96))
				return -EINVAL;

			break;
		case PCANFD_ERROR_WARNING:
			if ((dev->rx_error_counter >= 128) ||
						(dev->tx_error_counter >= 128))
				return -EINVAL;

			if ((dev->rx_error_counter < 96) &&
						(dev->tx_error_counter < 96))
				return -EINVAL;

			break;
		case PCANFD_ERROR_PASSIVE:
			if ((dev->rx_error_counter < 128) &&
						(dev->tx_error_counter < 128))
				return -EINVAL;
		default:
			break;
		}

#if 1
		/* now, rx/tx errror counters are saved into all msgs, in
		 * ctrlr_data[] field */
#else
		/* copy rx/tx error counters into pf->data[] */
		memcpy(pb, &dev->rx_error_counter,
					sizeof(dev->rx_error_counter));
		pb += sizeof(dev->rx_error_counter);

		memcpy(pb, &dev->tx_error_counter,
					sizeof(dev->tx_error_counter));
		pb += sizeof(dev->tx_error_counter);
		pf->data_len = pb - pf->data;
#endif
	}

	/* posting the same STATUS msg in the future will be discarded,
	 * except if error counters have changed (even with CAN controllers
	 * that don't give error counters...) */
	dev->prev_rx_error_counter = dev->rx_error_counter;
	dev->prev_tx_error_counter = dev->tx_error_counter;

	return 0;
}
#endif

#ifdef DEBUG_RX_QUEUE
static int pcan_do_count_msgs_type(void *item, void *arg)
{
	struct pcanfd_msg *fifo_msg = (struct pcanfd_msg *)item;
	u32 *pfs = (u32 *)arg;

	pfs[fifo_msg->type]++;

	return 0;
}
#endif

/* convert hw timestamp to local time with handling clock drift
 *
 * Return:
 *
 * 0 if timestamp can't be converted from hw timestamp (thus, pcan_main will
 *   set "host" timestamp when it will post the msg in Rx queue, and HWTIMESTAMP
 *   flag won'tbe set
 * 1 if timestamp can be computed from hw values.
 */
int pcan_sync_decode(struct pcandev *dev, u32 ts_low, u32 ts_high,
							struct timeval *tv)
{
#ifdef PCAN_DONT_USE_HWTS
	return 0;
#else
#ifdef PCAN_HANDLE_CLOCK_DRIFT
	struct timeval now;
#endif
	u64 ts_us = ((u64 )ts_high << 32) + ts_low;
	long dus;

	/* with this mode, raw 32-bits values are copied from device ts */
	if (dev->ts_mode == PCANFD_OPT_HWTIMESTAMP_RAW) {
		tv->tv_usec = do_div(ts_us, USEC_PER_SEC);
		tv->tv_sec = (__kernel_time_t )ts_us;

		return 1;
	}

	if (!dev->time_sync.ts_us ||
		(dev->ts_mode == PCANFD_OPT_HWTIMESTAMP_OFF)) {

		/* syncing has not yet started: give host time "only" */
		return 0;
	}


	/* dus = count of hw µs since last sync
	 *
	 * note: PCIe card sync ts is always >= event ts while
	 * USB CAN sync ts is always <= event ts */
	dus = ts_us - dev->time_sync.ts_us;

	/* use device_time (time seen by the device) at last sync,
	 * as (default) time base */
	*tv = dev->time_sync.tv_ts;

#if defined(PCAN_HANDLE_CLOCK_DRIFT) && PCAN_HANDLE_CLOCK_DRIFT == 1
	if (dev->time_sync.sync_count >= PCAN_SYNC_STABLE_DRIFT_LEVEL) {
		/* should add/sub a number of µs corresponding to clock drift
		 * applied to each sec. of device time passed */
		dus += dev->time_sync.clock_drift *
				dev->time_sync.tv_tts.tv_sec;

		/* as well as on the second part */
		dus += (dev->time_sync.clock_drift *
				dev->time_sync.tv_tts.tv_usec) / USEC_PER_SEC;
	}
#endif

	/* if sync time is greater than event time, sub the count of
	 * µs from this sync time to get the event time */
	timeval_add_us(tv, dus);

	if (dev->ts_mode != PCANFD_OPT_HWTIMESTAMP_COOKED)
		return 1;

#ifndef PCAN_HANDLE_CLOCK_DRIFT
	/* if clock drift ISNOT defind, use raw device timestamp, no matter it
	 * might given a time in the future... */
#else
	pcan_gettimeofday(&now);

#ifdef DEBUG_TIMESTAMP_DECODE
#ifdef DEBUG_TIMESTAMP_HWTYPE
	if (dev->wType == DEBUG_TIMESTAMP_HWTYPE)
#endif
		pr_info(DEVICE_NAME
			": %s(ts=%llu) sync=%llu => diff=%ld "
			"%ld.%06ld + %ld = %ld.%06ld (now=%ld.%06ld)\n",
			__func__,
			(unsigned long long)ts_us,
			(unsigned long long)dev->time_sync.ts_us,
			(long )(ts_us - dev->time_sync.ts_us),
			dev->time_sync.tv_ts.tv_sec,
			dev->time_sync.tv_ts.tv_usec,
			dus,
			tv->tv_sec, tv->tv_usec,
			now.tv_sec, now.tv_usec);
#endif

#if 1
	/* check whether this time is not from the future. */
	dus = timeval_diff(&now, tv);
	if (dus < 0) {
#if 0//def DEBUG_TIMESTAMP_IN_THE_FUTURE
#ifdef DEBUG_TIMESTAMP_HWTYPE
		if (dev->wType == DEBUG_TIMESTAMP_HWTYPE)
#endif

			pr_info(DEVICE_NAME " %ld.%06ld - %ld.%06ld = %ld us\n",
				now.tv_sec, now.tv_usec,
				tv->tv_sec, tv->tv_usec, dus);
#endif
		*tv = now;
	}
#endif
#endif /* PCAN_HANDLE_CLOCK_DRIFT */

	return 1;

#endif /* PCAN_DONT_USE_HWTS */
}

/* called to synchronize host time and hardware time
 * Note:
 * - USB-FD devices sync evry 1s.
 * - PCIe-FD devices sync every 1,2 ms */
int pcan_sync_times(struct pcandev *dev, u32 ts_low, u32 ts_high, u32 dts2_us)
{
#ifndef PCAN_DONT_USE_HWTS
	if (dev->time_sync.ts_us) {
		struct pcan_time_sync now;
		unsigned long dts_us, dtv_us;

		/* get host time between each sync */
		pcan_gettimeofday_ex(&now.tv, &now.tv_ns);
		dtv_us = timeval_to_us(&now.tv) -
					timeval_to_us(&dev->time_sync.tv);

		now.ts_us = ((u64 )ts_high << 32) + ts_low;

#ifdef DEBUG_TIMESTAMP_IN_THE_FUTURE
		/* be sure of this */
		if (now.ts_us < dev->time_sync.ts_us)
#ifdef DEBUG_TIMESTAMP_HWTYPE
			if (dev->wType == DEBUG_TIMESTAMP_HWTYPE)
#endif
				pr_warn(DEVICE_NAME
					": WTF hw now ts=%llu < prev ts=%llu\n",
					now.ts_us, dev->time_sync.ts_us);
#endif
		/* dts_us = count of µs in hardware time between 2x calls */
		dts_us = now.ts_us - dev->time_sync.ts_us;

		/* tv_ts = timeval for hw.
		 * This will be the time base of msg timestamps. */
		now.tv_ts = dev->time_sync.tv_ts;
		timeval_add_us(&now.tv_ts, dts_us);

#ifdef PCAN_HANDLE_CLOCK_DRIFT
#if PCAN_HANDLE_CLOCK_DRIFT == 1
		/* tv_tts = timeval for hw s. and  µs. accumulator.
		 * This will be used for applying clock drift factor, added to 
		 * above time base. */
		now.tv_tts = dev->time_sync.tv_tts;
		timeval_add_us(&now.tv_tts, dts_us);
#endif
		/* Now, calculate clock drift by comparing count of µs in both
		 * times: */
		/* ttv_us = count of host µs since start of sync */
		now.ttv_us = dev->time_sync.ttv_us + dtv_us;

		/* tts_us = count of hw µs since start of sync */
		now.tts_us = dev->time_sync.tts_us + dts_us;
		now.clock_drift = dev->time_sync.clock_drift;

		now.sync_count = dev->time_sync.sync_count;
		now.sync_us_cnt = dev->time_sync.sync_us_cnt;

		if (now.sync_count < PCAN_SYNC_STABLE_DRIFT_LEVEL)
			now.sync_count++;

#if PCAN_HANDLE_CLOCK_DRIFT == 2
		else {

		/* clock_dirft:
		 *
		 *	now.clock_drift =
		 *		((now.ttv_us - now.tts_us) * USEC_PER_SEC) /
		 *				now.tts_us;
		 *
		 * = count of µs to add/sub to each second of hw time to
		 *   limit clock drift between both host and hw clocks.
		 *
		 * => each s. hw time must be added/substracted by clock_drift
		 *    value:
		 *
		 *	timeval_add_us(&now.tv_ts,
		 *			(now.sync_us_cnt / USEC_PER_SEC) *
		 *			now.clock_drift);
		 *
		 * Unfortunately, divding a 64b int is not possible under 32b
		 * archs, so we must do with do_div() AND 1/x operation,
		 * something like:
		 *
		 * clock_drift = do_div(now.tts_us, now.ttv_us - now.tts_us);
		 *
		 * and:
		 *
		 * timeval_add_us(&now.tv_ts,
		 *			now.sync_us_cnt / now.clock_drift);
		 */
			//now.sync_us_cnt += dtv_us;
			now.sync_us_cnt += dts_us;

			/* since clock drift is a counter of µs to add each
			 * second, then wait for one s. to fix it */
			if (now.sync_us_cnt >= USEC_PER_SEC) {

				if (now.tts_us != now.ttv_us) {
					u64 tmp64 = now.tts_us;
					int k;

					/* clock_drift_32 = 10^6 / clock_drift_64 */
					u32 tmp32;

					if (now.ttv_us > now.tts_us) {
						tmp32 = (now.ttv_us - now.tts_us);
						k = 1;
					} else {
						tmp32 = (now.tts_us - now.ttv_us);
						k = -1;
					}

					do_div(tmp64, tmp32);

					now.clock_drift = k * (long )tmp64;
				} else {
					now.clock_drift = 0;
				}

				if (now.clock_drift) {
					timeval_add_us(&now.tv_ts,
						now.sync_us_cnt /
							now.clock_drift);
				}

				now.sync_us_cnt %= USEC_PER_SEC;
			}
		}
#endif
#endif

#ifdef DEBUG_TIMESTAMP_SYNC
#ifdef DEBUG_TIMESTAMP_HWTYPE
		if (dev->wType == DEBUG_TIMESTAMP_HWTYPE)
#endif
			pr_info(DEVICE_NAME
				": now=%ld.%06ld sync #%lu =%llu (%ld.%06ld) "
				"dtv=%lu dts=%lu "
#ifdef PCAN_HANDLE_CLOCK_DRIFT
				"ttv_us=%llu tts_us=%llu "
				"=> clk_drift=%ld"
#endif
				"\n",
				now.tv.tv_sec, now.tv.tv_usec,
				now.sync_count,
				(unsigned long long)now.ts_us,
				now.tv_ts.tv_sec, now.tv_ts.tv_usec,
				(unsigned long)dtv_us, dts_us
#ifdef PCAN_HANDLE_CLOCK_DRIFT
				, now.ttv_us, now.tts_us,
				now.clock_drift
#endif
				);
#endif

		dev->time_sync = now;

		return 1;
	}
#endif /* PCAN_DONT_USE_HWTS */

	memset(&dev->time_sync, '\0', sizeof(dev->time_sync));

	pcan_gettimeofday_ex(&dev->time_sync.tv, &dev->time_sync.tv_ns);
	dev->time_sync.ts_us = ((u64 )ts_high << 32) + ts_low;
	dev->time_sync.tv_ts = dev->time_sync.tv;

#if defined(DEBUG_TIMESTAMP_DECODE) || defined(DEBUG_TIMESTAMP_SYNC)
#ifdef DEBUG_TIMESTAMP_HWTYPE
	if (dev->wType == DEBUG_TIMESTAMP_HWTYPE)
#endif
		pr_info(DEVICE_NAME
			": %s() sync=%llu (%lu.%lu) tv_ts=%ld.%06ld\n",
			__func__,
			(unsigned long long)dev->time_sync.ts_us,
			(unsigned long)ts_high, (unsigned long)ts_low,
			dev->time_sync.tv_ts.tv_sec,
			dev->time_sync.tv_ts.tv_usec);
#endif
	return 0;
}

static void pcan_status_normalize_rx(struct pcandev *dev, struct pcanfd_msg *pf)
{
	u8 rxcnt, txcnt;

	if (!(pf->flags & PCANFD_ERRCNT))
		return;

	rxcnt = pf->ctrlr_data[PCANFD_RXERRCNT];
	txcnt = pf->ctrlr_data[PCANFD_TXERRCNT];

	switch (pf->id) {

	case PCANFD_ERROR_ACTIVE:
		if ((rxcnt < 96) && (txcnt < 96))
			break;

		pf->id = PCANFD_ERROR_WARNING;

	case PCANFD_ERROR_WARNING:
		if ((rxcnt < 128) && (txcnt < 128))
			break;

		pf->id = PCANFD_ERROR_PASSIVE;

	case PCANFD_ERROR_PASSIVE:
	default:
		break;
	}
}

/*
 * put received CAN frame into chardev receive FIFO
 * maybe this goes to a new file pcan_chardev.c some day.
 *
 * WARNING: this function returns >0 when frame HAS been enqueued,
 *                                0  when frame HAS NOT been enqueued,
 *                                <0 in case of error
 */
int pcan_chardev_rx(struct pcandev *dev, struct pcanfd_msg *pf)
{
	int err;

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(CAN%u): nOpenPaths=%d bExtended=%d "
		"pf[type=%u id=%u flags=%xh]\n",
		__func__, dev->nChannel+1,
		dev->nOpenPaths, !!(dev->allowed_msgs & PCANFD_ALLOWED_MSG_EXT),
		pf->type, pf->id, pf->flags);
#endif

	switch (pf->type) {
	case PCANFD_TYPE_NOP:
		return 0;

	case PCANFD_TYPE_STATUS:
		if (!(dev->allowed_msgs & PCANFD_ALLOWED_MSG_STATUS))
			return 0;
		break;

	case PCANFD_TYPE_ERROR_MSG:
		if (!(dev->allowed_msgs & PCANFD_ALLOWED_MSG_ERROR))
			return 0;
		break;

	case PCANFD_TYPE_CAN20_MSG:
	case PCANFD_TYPE_CANFD_MSG:

		/* inc rx frame counter, even if it is not posted! */
		dev->rx_frames_counter++;

#ifdef PCAN_HANDLE_SYNC_FRAME
#warning This version is for test ONLY !!!

		if ((pf->id == PCAN_HANDLE_SYNC_FRAME) &&
			(pf->data_len == 2) && (pf->data[0] > 0) &&
							(pf->data[0] <= 20)) {
extern int pcanfd_ioctl_send_msgs_nolock(struct pcandev *dev,
					 struct pcanfd_msgs *pl,
					 struct pcan_udata *ctx);
			u32 i;
			struct pcanfd_msg *pm;
			struct timeval tv_now;
			static struct __array_of_struct(pcanfd_msg, 20) ml;

			pcan_gettimeofday(&tv_now);
			ml.count = pf->data[0];

			pm = ml.list;
			for (i = 0; i < ml.count; i++, pm++) {
				pm->type = PCANFD_TYPE_CAN20_MSG;
				pm->data_len = 8;
				pm->id = 0x6321301;
				pm->flags = PCANFD_MSG_EXT|PCANFD_MSG_SNG;
				memcpy(pm->data, &tv_now.tv_sec, 4);
				memcpy(pm->data+4, &tv_now.tv_usec, 4);
			}

			/* do push n msgs into dev tx fifo */
			err = pcanfd_ioctl_send_msgs_nolock(dev,
					(struct pcanfd_msgs *)&ml, NULL);
			if (err) {
				pr_info(DEVICE_NAME ": unable to put %u msgs "
					"into device Tx fifo: err %d\n",
					pf->data[0], err);
			}
		}
#endif
		if (!(dev->allowed_msgs & PCANFD_ALLOWED_MSG_CAN))
			return 0;

		if (pf->flags & MSGTYPE_EXTENDED) {
			if (!(dev->allowed_msgs & PCANFD_ALLOWED_MSG_EXT)) {
#ifdef DEBUG_RX_QUEUE
				printk(KERN_DEBUG
					"%s: %s(): rx [type=%d id=%d] "
					"discarded (EXT fmt)\n",
					DEVICE_NAME, __func__,
					pf->type, pf->id);
#endif
				return 0;
			}

			/* then, check acceptance code/mask:
			 * mask:
			 * bit=1 = don't care;
			 * bit=0 = cmp this bit with coresponding bit in code */
			if ((pf->id & ~dev->acc_29b.mask) !=
							dev->acc_29b.code) {
#ifdef DEBUG_RX_QUEUE
				printk(KERN_DEBUG
					"%s: %s(): rx [type=%d id=%0xh] "
					"discarded "
					"(acc_29b[code=%xh mask=%xh])\n",
					DEVICE_NAME, __func__, pf->type, pf->id,
					dev->acc_29b.code, dev->acc_29b.mask);
#endif
				return 0;
			}

		} else {

			/* first, check acceptance code/mask */
			if ((pf->id & ~dev->acc_11b.mask) !=
							dev->acc_11b.code) {
#ifdef DEBUG_RX_QUEUE
				printk(KERN_DEBUG
					"%s: %s(): rx [type=%d id=%0xh] "
					"discarded "
					"(acc_11b[code=%xh mask=%xh])\n",
					DEVICE_NAME, __func__, pf->type, pf->id,
					dev->acc_11b.code, dev->acc_11b.mask);
#endif
				return 0;
			}
		}

		if (pf->flags & PCANFD_MSG_RTR)
			if (!(dev->allowed_msgs & PCANFD_ALLOWED_MSG_RTR)) {
#ifdef DEBUG_RX_QUEUE
			printk(KERN_DEBUG "%s: %s(): rx [type=%d id=%d] "
				"discarded (RTR frame)\n",
				DEVICE_NAME, __func__, pf->type, pf->id);
#endif
				return 0;
			}
		break;
	}

	/* MUST check here if any path has been opened before posting any
	 * messages */
	if (dev->nOpenPaths <= 0) {
#if 0//def DEBUG_RX_QUEUE
		printk(KERN_DEBUG
			"%s: %s(): [type=%d id=%d len=%d flags=%08xh "
			"ctrl_data=%02x %02x %02x %02x] "
			"discarded (nOpenPaths=%d)\n",
			DEVICE_NAME, __func__, pf->type, pf->id, pf->data_len,
			pf->flags,
			pf->ctrlr_data[0], pf->ctrlr_data[1],
			pf->ctrlr_data[2], pf->ctrlr_data[3],
			dev->nOpenPaths);
#endif
		return 0;
	}

	/* if no timestamp in this message, put current time */
	if (!(pf->flags & PCANFD_TIMESTAMP) ||
		(dev->ts_mode == PCANFD_OPT_HWTIMESTAMP_OFF)) {
		pcan_gettimeofday(&pf->timestamp);
		pf->flags |= PCANFD_TIMESTAMP;
		pf->flags &= ~PCANFD_HWTIMESTAMP;
#if defined(PCAN_FIX_FUTURE_TS) || defined(DEBUG_TIMESTAMP_IN_THE_FUTURE)
	} else if (dev->ts_mode != PCANFD_OPT_HWTIMESTAMP_RAW) {
		struct timeval now;

		pcan_gettimeofday(&now);

		if (timeval_is_older(&now, &pf->timestamp)) {
#ifdef DEBUG_TIMESTAMP_IN_THE_FUTURE
#ifdef DEBUG_TIMESTAMP_HWTYPE
			if (dev->wType == DEBUG_TIMESTAMP_HWTYPE)
#endif
				pr_err(DEVICE_NAME
					": WARNING: timestamp in the future: "
					"now=%ld.%06ld < tv=%ld.%06ld "
					"\n",
					now.tv_sec, now.tv_usec,
					pf->timestamp.tv_sec,
					pf->timestamp.tv_usec);
#endif
#ifdef PCAN_FIX_FUTURE_TS
			pf->timestamp = now;
			pf->flags &= ~PCANFD_HWTIMESTAMP;
#endif
		}
#endif
	}

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(): tv=%u.%06u s\n",
			__func__,
			(u32 )pf->timestamp.tv_sec,
			(u32 )pf->timestamp.tv_usec);
#endif
	/* default pcan gives timestamp relative to
	 * the time the driver has been loaded. */
	pcan_handle_timestamp(dev, pf);

#ifdef PCAN_LIMIT_STATUS_FLOODING
	if (dev->wCANStatus & CAN_ERR_OVERRUN) {

		/* rx fifo full: msg *pf will be lost.
		 * change last msg into a STATUS[PCANFD_RX_OVERFLOW] to
		 * inform user of the situation */
		struct pcanfd_msg full_msg;

		pcan_handle_error_internal(dev, &full_msg, PCANFD_RX_OVERFLOW);

		if (pf->flags & PCANFD_TIMESTAMP) {
			full_msg.flags |= PCANFD_TIMESTAMP;
			full_msg.timestamp = pf->timestamp;
		}

		pcan_fifo_foreach_back(&dev->readFifo, pcan_do_patch_last,
				       &full_msg);
		return 0;
	}
#endif

	if (dev->flags & PCAN_DEV_BUSLOAD_RDY) {
		pf->ctrlr_data[PCANFD_BUSLOAD_UNIT] = dev->bus_load / 100;
		pf->ctrlr_data[PCANFD_BUSLOAD_DEC] = dev->bus_load % 100;
		pf->flags |= PCANFD_BUSLOAD;
	} else {
		pf->flags &= ~PCANFD_BUSLOAD;
	}

	if (dev->flags & PCAN_DEV_ERRCNT_RDY) {
		pf->ctrlr_data[PCANFD_RXERRCNT] = dev->rx_error_counter;
		pf->ctrlr_data[PCANFD_TXERRCNT] = dev->tx_error_counter;
		pf->flags |= PCANFD_ERRCNT;
	} else {
		pf->flags &= ~PCANFD_ERRCNT;
	}

	switch (pf->type) {

	case PCANFD_TYPE_CAN20_MSG:
	case PCANFD_TYPE_CANFD_MSG:

#if 1
		/* better to let hardware specifc driver to simulate the 
		 * ERROR_ACTIVE state if the hw device is not able to
		 * notify it, by using pcan_soft_error_active() instead */
#else
		/* check whether the bus_state is known. 
		 * set it to ERROR_ACTIVE if not (with pushing a STATUS event
		 * made with frame timestamp. */
		if (dev->bus_state == PCANFD_UNKNOWN) {
			struct pcanfd_msg e;

			pcan_handle_error_active(dev, &e);
			if (pf->flags & PCANFD_TIMESTAMP) {
				e.flags |= PCANFD_TIMESTAMP;
				e.timestamp = pf->timestamp;
			}

			pcan_chardev_rx(dev, &e);
		}
#endif
		/* then check acceptance ranges */
		if (pcan_do_filter(dev->filter, pf)) {
#ifdef DEBUG_RX_QUEUE
			printk(KERN_DEBUG "%s: %s(): rx [type=%d id=%d] "
				"discarded (filtered)\n",
				DEVICE_NAME, __func__, pf->type, pf->id);
#endif
			return 0;
		}

		break;

	case PCANFD_TYPE_STATUS:
		/* other cases: message must be pushed into fifo: */

		/* pre-process these msgs before posting them to user, to
		 * prevent to flood its rx queue with lost of same msgs */
		switch (pf->id) {

		case PCANFD_ERROR_ACTIVE:
		case PCANFD_ERROR_WARNING:
		case PCANFD_ERROR_PASSIVE:
			pcan_status_normalize_rx(dev, pf);

		case PCANFD_ERROR_BUSOFF:
#ifdef PCAN_LIMIT_STATUS_FLOODING
			err = pcan_status_error_rx(dev, pf);
			if (err == -EEXIST)
				/* no need to push this STATUS because the same
				 * status has already been posted before.
				 * Return 0 to inform that nothing has been
				 * posted. */
				return 0;
#ifdef DEBUG_INVALID_BUS_STATE
			if (err == -EINVAL)
				pr_info(DEVICE_NAME ": %s CAN%u: "
					"bus_state=%d incompatibe with "
					"err counters rx=%u tx=%u\n",
					dev->adapter->name,
					dev->nChannel+1,
					dev->bus_state,
					dev->rx_error_counter,
					dev->tx_error_counter);
#endif
#endif
			break;

		case PCANFD_RX_EMPTY:
		case PCANFD_RX_OVERFLOW:
		case PCANFD_TX_EMPTY:
		case PCANFD_TX_OVERFLOW:
			break;

		case PCANFD_BUS_LOAD:
			err = pcan_status_bus_load_rx(dev, pf);
			if (err == -EEXIST)
				/* no need to push this STATUS because the same
				 * status has already been posted before.
				 * Return 0 to inform that nothing has been
				 * posted. */
				return 0;

			break;

		default:
			break;
		}

#ifdef PCAN_LIMIT_STATUS_FLOODING
		/* try to patch last posted msg if it was the same STATUS */
		err = pcan_fifo_foreach_back(&dev->readFifo,
						pcan_do_patch_status, pf);
		if (err == -EEXIST)
			/* STATUS patched. don't post the msg */
			return 0;
#endif
		break;
	}

	/* step forward in fifo */
	err = pcan_fifo_put(&dev->readFifo, pf);
	if (err >= 0)
		return 1;

	pcan_handle_error_internal(dev, NULL, PCANFD_RX_OVERFLOW);

#ifdef DEBUG_RX_QUEUE
	printk(KERN_DEBUG "%s: %s(): rx [type=%d id=%d] "
		"discarded (%u items rx queue full)\n",
		DEVICE_NAME, __func__, pf->type, pf->id, rxqsize);
	{
		u32 fs[4] = { 0, };
		pcan_fifo_foreach_back(&dev->readFifo,
						pcan_do_count_msgs_type, &fs);
		printk(KERN_DEBUG
			"%s: rx queue stats: CAN/FD msgs=%u/%u STATUS=%u\n",
			DEVICE_NAME, fs[PCANFD_TYPE_CAN20_MSG],
			fs[PCANFD_TYPE_CANFD_MSG],
			fs[PCANFD_TYPE_STATUS]);
	}
#endif

	return err;
}

static void pcan_init_session_counters(struct pcandev *dev)
{
	dev->rx_error_counter = 0;
	dev->tx_error_counter = 0;

	/* set a different value to prev_ counters so that 1st
	 * STATUS[ERROR_ACTIVE] will be posted ! */
	dev->prev_rx_error_counter = dev->rx_error_counter + 1;
	dev->prev_tx_error_counter = dev->tx_error_counter + 1;

	dev->bus_load = 0;
	dev->prev_bus_load = dev->bus_load + 1;

	dev->wCANStatus &= ~(CAN_ERR_BUSOFF|CAN_ERR_BUSHEAVY|\
							CAN_ERR_BUSLIGHT);
}

void pcan_set_bus_state(struct pcandev *dev, enum pcanfd_status bus_state)
{
	if (bus_state == dev->bus_state)
		return;

	switch (bus_state) {
	case PCANFD_ERROR_ACTIVE:
#ifdef DEBUG_BUS_STATE
		pr_info(DEVICE_NAME ": pcan%u enters ERROR_ACTIVE\n",
			dev->nMinor);
#endif
		pcan_init_session_counters(dev);
		break;
	case PCANFD_ERROR_WARNING:
#ifdef DEBUG_BUS_STATE
		pr_info(DEVICE_NAME ": pcan%u enters ERROR_WARNING\n",
			dev->nMinor);
#endif
		dev->wCANStatus |= CAN_ERR_BUSLIGHT;
		break;
	case PCANFD_ERROR_PASSIVE:
#ifdef DEBUG_BUS_STATE
		pr_info(DEVICE_NAME ": pcan%u enters ERROR_PASSIVE\n",
			dev->nMinor);
#endif
		dev->wCANStatus |= CAN_ERR_BUSHEAVY;
		break;
	case PCANFD_ERROR_BUSOFF:
#ifdef DEBUG_BUS_STATE
		pr_info(DEVICE_NAME ": pcan%u enters BUSOFF\n",
			dev->nMinor);
#endif
		dev->wCANStatus |= CAN_ERR_BUSOFF;
		dev->dwErrorCounter++;
		break;
	case PCANFD_UNKNOWN:
#ifdef DEBUG_BUS_STATE
		pr_info(DEVICE_NAME ": pcan%u back to UNKNWON\n",
			dev->nMinor);
#endif
		pcan_init_session_counters(dev);
		break;
	default:
		pr_err(DEVICE_NAME
			": trying to set unknown bus state %d to pcan%u\n",
			bus_state, dev->nMinor);
		return;
	}

	dev->bus_state = bus_state;

	/* this is done to pass first test of 1st call to pcan_status_error_rx()
	 * which is used to limit filling Rx queue with lots of STATUS msg:
	 * by setting a different value to prev_xx_error_counters, then we're
	 * sure that the 1st msg that notify from changing of bus state won't
	 * never be filtered. */
	dev->prev_rx_error_counter = dev->rx_error_counter + 1;
	dev->prev_tx_error_counter = dev->tx_error_counter + 1;
}

int pcan_handle_busoff(struct pcandev *dev, struct pcanfd_msg *pf)
{
	if (dev->bus_state == PCANFD_ERROR_BUSOFF) {
		pf->type = PCANFD_TYPE_NOP;
		return 0;
	}

	pr_info(DEVICE_NAME ": %s CAN%u: BUS OFF\n",
			dev->adapter->name, dev->nChannel+1);

	pcan_set_bus_state(dev, PCANFD_ERROR_BUSOFF);

	pf->type = PCANFD_TYPE_STATUS;
	pf->flags = PCANFD_ERROR_BUS;
	pf->id = PCANFD_ERROR_BUSOFF;

	/* In BUSOFF state only, wake up any task waiting for room in the
	 * device tx queue, because there is no chance for the BUS to go back to
	 * any active state without re-initializing it.
	 * Note that this "pf" message is to be put into the device rx queue,
	 * thus any task waiting for incoming messages will also be
	 * woken up. */
	pcan_event_signal(&dev->out_event);

	return 1;
}

void pcan_handle_error_active(struct pcandev *dev, struct pcanfd_msg *pf)
{
#ifdef DEBUG_BUS_STATE
	pr_info(DEVICE_NAME ": %s(%s CAN%u): bus=%u pf=%p\n",
		__func__, dev->adapter->name, dev->nChannel+1,
		dev->bus_state, pf);
#endif
	if (pf) {
		if (dev->bus_state != PCANFD_ERROR_ACTIVE) {

			/* inform only if controller was is in bad state */
			if (dev->bus_state == PCANFD_ERROR_BUSOFF)
				pr_info(DEVICE_NAME ": %s CAN%u: ACTIVE\n",
					dev->adapter->name, dev->nChannel+1);

			pf->type = PCANFD_TYPE_STATUS;
			pf->flags = PCANFD_ERROR_BUS;
			pf->id = PCANFD_ERROR_ACTIVE;

			/* be sure that copies of err counters are 0 */
			memset(pf->data, '\0', sizeof(pf->data));

		/* be sure to post nothing if already in ERROR_ACTIVE */
		} else {
			pf->type = PCANFD_TYPE_NOP;
		}
	}

	pcan_set_bus_state(dev, PCANFD_ERROR_ACTIVE);
}

#if 0
/* 
 * check and fix the bus state to ERROR_ACTIVE if it is UNKNOWN.
 * chardev mode		this fix only occurs when at least one path is opened.
 *			in this case, a STATUS event is posted.
 * netdev mode		the bus state is set to ERROR_ACTIVE once for all.
 */
void pcan_soft_error_active(struct pcandev *dev)
{
#ifdef DEBUG_BUS_STATE
	pr_info(DEVICE_NAME ": %s(%s CAN%u)\n", __func__,
			dev->adapter->name, dev->nChannel+1);
#endif
	if (dev->bus_state == PCANFD_UNKNOWN) {
#ifdef NETDEV_SUPPORT
		pcan_set_bus_state(dev, PCANFD_ERROR_ACTIVE);
#else
		struct pcanfd_msg e;

		pcan_handle_error_active(dev, &e);

		e.flags &= ~PCANFD_TIMESTAMP;
		if (pcan_chardev_rx(dev, &e) > 0)
			pcan_event_signal(&dev->in_event);
#endif
	}
}
#else
void pcan_soft_error_active(struct pcandev *dev)
{
	struct pcanfd_msg e;

#ifdef DEBUG_BUS_STATE
	pr_info(DEVICE_NAME ": %s(%s CAN%u)\n", __func__,
			dev->adapter->name, dev->nChannel+1);
#endif

	pcan_handle_error_active(dev, &e);

#ifdef NETDEV_SUPPORT
	pcan_netdev_rx(dev, &e);
#else
	if (pcan_chardev_rx(dev, &e) > 0)
		pcan_event_signal(&dev->in_event);
#endif
}
#endif

#if 1
/* rx/tx errros counters are copeid by pcan_chardev_rx() just before being
 * pushed into the rx fifo, when msg=[PCANFD_TYPE_STATUS+PCANFD_ERROR_BUS] */
#else
void pcan_copy_err_counters(struct pcandev *dev, struct pcanfd_msg *pf)
{
	u8 *pb = pf->data;

	/* copy rx and tx error counters in data field */
	memcpy(pb, &dev->rx_error_counter, sizeof(dev->rx_error_counter));
	pb += sizeof(dev->rx_error_counter);

	memcpy(pb, &dev->tx_error_counter, sizeof(dev->tx_error_counter));
	pb += sizeof(dev->tx_error_counter);
	pf->data_len = pb - pf->data;

	dev->prev_rx_error_counter = dev->rx_error_counter;
	dev->prev_tx_error_counter = dev->tx_error_counter;
}
#endif

int pcan_handle_error_status(struct pcandev *dev, struct pcanfd_msg *pf,
				int err_warning, int err_passive)
{
#ifdef DEBUG_BUS_STATE
	pr_info(DEVICE_NAME ": %s(%s CAN%u, EW=%u, EP=%u): bus=%u\n",
		__func__, dev->adapter->name, dev->nChannel+1,
		err_warning, err_passive, dev->bus_state);
#endif

	pf->type = PCANFD_TYPE_STATUS;
	pf->flags = PCANFD_ERROR_BUS;

	if (err_passive) {
		pf->id = PCANFD_ERROR_PASSIVE;
	} else if (err_warning) {
		pf->id = PCANFD_ERROR_WARNING;
	} else {

		/* in fact, no error bit set: back to ERROR_ACTIVE */
		if (dev->bus_state == PCANFD_ERROR_ACTIVE) {
			pf->type = PCANFD_TYPE_NOP;
			return 1;
		}

		return 0;
	}

#if 1
	/* automatically done by pcan_chardev_rx() now */
#else
	/* if always in the same state, do post only if counters have changed */
	if ((dev->bus_state == pf->id) &&
	    (dev->rx_error_counter == dev->prev_rx_error_counter) &&
	    (dev->tx_error_counter == dev->prev_tx_error_counter)) {
		pf->type = PCANFD_TYPE_NOP;
		return 1;
	}

	pcan_copy_err_counters(dev, pf);
#endif

	dev->dwErrorCounter++;
	pcan_set_bus_state(dev, pf->id);

	/* say that error state has been handled. */
	return 1;
}

void pcan_handle_error_ctrl(struct pcandev *dev, struct pcanfd_msg *pf,
				int err_ctrlr)
{
	if (pf) {
		pf->type = PCANFD_TYPE_STATUS;
		pf->flags = PCANFD_ERROR_CTRLR;
		pf->id = err_ctrlr;
		pf->data_len = 0;
	}

	switch (err_ctrlr) {
	case PCANFD_RX_OVERFLOW:
		dev->wCANStatus |= CAN_ERR_QOVERRUN;
		break;
	case PCANFD_RX_EMPTY:
		dev->wCANStatus |= CAN_ERR_QRCVEMPTY;
		break;
	case PCANFD_TX_OVERFLOW:
		dev->wCANStatus |= CAN_ERR_QXMTFULL;
		break;
	default:
		pr_err(DEVICE_NAME ": pcan%u: ctrlr error %d\n",
				dev->nMinor, err_ctrlr);
		dev->wCANStatus |= CAN_ERR_RESOURCE;
		break;
	}

	dev->dwErrorCounter++;
}

void pcan_handle_error_msg(struct pcandev *dev, struct pcanfd_msg *pf,
				int err_type, u8 err_code,
				int err_rx, int err_gen)
{
	if (pf) {
		pf->type = PCANFD_TYPE_ERROR_MSG;
		pf->id = err_type;
		if (err_rx)
			pf->flags |= PCANFD_ERRMSG_RX;
		if (err_gen)
			pf->flags |= PCANFD_ERRMSG_GEN;

		pf->data_len = 1;
		pf->data[0] = err_code;
	}

	dev->dwErrorCounter++;
}

void pcan_handle_error_internal(struct pcandev *dev, struct pcanfd_msg *pf,
				int err_internal)
{
	if (pf) {
		pf->type = PCANFD_TYPE_STATUS;
		pf->flags = PCANFD_ERROR_INTERNAL;
		pf->id = err_internal;
		pf->data_len = 0;
	}

	switch (err_internal) {
	case PCANFD_RX_OVERFLOW:
		dev->wCANStatus |= CAN_ERR_OVERRUN;
		break;
	case PCANFD_TX_OVERFLOW:
		dev->wCANStatus |= CAN_ERR_XMTFULL;
		break;
	default:
		pr_err(DEVICE_NAME ": pcan%u: internal error %d\n",
				dev->nMinor, err_internal);
		dev->wCANStatus |= CAN_ERR_RESOURCE;
		break;
	}

	dev->dwErrorCounter++;
}

void pcan_handle_error_protocol(struct pcandev *dev, struct pcanfd_msg *pf,
				int err_protocol)
{
	if (pf) {
		pf->type = PCANFD_TYPE_STATUS;
		pf->flags = PCANFD_ERROR_PROTOCOL;
		pf->id = err_protocol;
		pf->data_len = 0;
	}

	switch (err_protocol) {
	case PCANFD_RX_OVERFLOW:
		dev->wCANStatus |= CAN_ERR_QOVERRUN;	/* as old driver did */
		break;
	case PCANFD_TX_OVERFLOW:
		dev->wCANStatus |= CAN_ERR_QXMTFULL;
		break;
	default:
		pr_err(DEVICE_NAME ": pcan%u: protocol error %d\n",
				dev->nMinor, err_protocol);
		dev->wCANStatus |= CAN_ERR_RESOURCE;
		break;
	}

	dev->dwErrorCounter++;
}

/*
 * Clear any wCANStatus bit and post a STATUS message to the chardev
 * application with current bus status to notify user that the bit is cleared
 */
void pcan_clear_status_bit(struct pcandev *dev, u16 bits)
{
	if (!(dev->wCANStatus & bits))
		return;

	/* clear corresponding bits and generate a STATUS message to update
	 * the state in the application context */
	dev->wCANStatus &= ~bits;

#ifndef NETDEV_SUPPORT
	{
		struct pcanfd_msg s = {
			.type = PCANFD_TYPE_STATUS,
			.id = dev->bus_state,
			.flags = PCANFD_ERROR_BUS,
		};

		/* this is done to pass first test of pcan_status_error_rx()
		 * which is used to limit filling Rx queue with lots of STATUS
		 * msg: by setting a different value to prev_xx_error_counters,
		 * then we're sure that the 1st msg that notify from changing
		 * of bus state won't never be filtered. */
		dev->prev_rx_error_counter = dev->rx_error_counter + 1;
		dev->prev_tx_error_counter = dev->tx_error_counter + 1;

		if (pcan_chardev_rx(dev, &s) > 0)
			pcan_event_signal(&dev->in_event);
	}
#endif
}

/* request time in msec, fast */
u32 get_mtime(void)
{
	/* return (jiffies / HZ) * 1000; */
	return jiffies_to_msecs(jiffies);
}

/*
 * Safe add a new device to the driver registered devices list
 */
void pcan_add_device_in_list_ex(struct pcandev *dev, u32 flags)
{
#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(%p)\n", __func__, dev);
#endif
#ifdef HANDLE_HOTPLUG
	pcan_mutex_lock(&pcan_drv.devices_lock);
#endif

	list_add_tail(&dev->list, &pcan_drv.devices);
	dev->flags |= PCAN_DEV_LINKED | flags;
	pcan_drv.wDeviceCount++;

#ifdef HANDLE_HOTPLUG
	pcan_mutex_unlock(&pcan_drv.devices_lock);
#endif
}

/*
 * Safe remove a device from the driver registered devices list
 */
void pcan_dev_remove_from_list(struct pcandev *dev)
{
#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(%p)\n", __func__, dev);
#endif

	if (dev->flags & PCAN_DEV_LINKED) {

#ifdef HANDLE_HOTPLUG
		/* pcandev is being destroyed by remove_dev_list(), so let
		 * remove_dev_list() unlink the pcandev... */
		if (!pcan_mutex_trylock(&pcan_drv.devices_lock))
			return;
#endif
		list_del(&dev->list);
		dev->flags &= ~PCAN_DEV_LINKED;

		if (pcan_drv.wDeviceCount)
			pcan_drv.wDeviceCount--;

#ifdef HANDLE_HOTPLUG
		pcan_mutex_unlock(&pcan_drv.devices_lock);
#endif
	}
}

/*
 * Safe check whether a device is linked in the pcan driver devices list.
 */
int pcan_is_device_in_list(struct pcandev *dev)
{
	struct list_head *ptr;
	int found = 0;

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(%p)\n", __func__, dev);
#endif
#ifdef HANDLE_HOTPLUG
	pcan_mutex_lock(&pcan_drv.devices_lock);
#endif

	list_for_each(ptr, &pcan_drv.devices)
		if (dev == (struct pcandev *)ptr) {
			found = 1;
			break;
		}

#ifdef HANDLE_HOTPLUG
	pcan_mutex_unlock(&pcan_drv.devices_lock);
#endif
	return found;
}

#ifdef PCAN_USB_DONT_REGISTER_DEV
/*
 * Search for a free (unsed) minor in the specifed range [from..until]
 * 'pdev' can be NULL; use it if a registered device MUST be excluded from the
 * existing minors test.
 */
int pcan_find_free_minor(struct pcandev *pdev, int from, int until)
{
	int minor = (from >= 0) ? from : 0;

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(from=%d, until=%d)\n", __func__, from, until);
#endif

#ifdef HANDLE_HOTPLUG
	pcan_mutex_lock(&pcan_drv.devices_lock);
#endif
	while (1) {
		struct list_head *ptr;
		int found = 0;

		list_for_each(ptr, &pcan_drv.devices) {
			struct pcandev *dev = (struct pcandev *)ptr;

			/* be sure to exclude this from the test! */
			if (dev != pdev)
				if (dev->nMinor == minor) {
					found = 1;
					break;
				}
		}

		if (!found)
			break;

		++minor;
		if (until > from)
			if (minor > until) {
				minor = -ENODEV;
				break;
			}
	}

#ifdef HANDLE_HOTPLUG
	pcan_mutex_unlock(&pcan_drv.devices_lock);
#endif
	return minor;
}
#endif

/* called when 'cat /proc/pcan' invoked */
#ifdef CREATE_PROC_ENTRY_DEPRECATED
static int pcan_read_procmem(struct seq_file *m, void *v)
{
#else
static int pcan_read_procmem(char *page, char **start, off_t offset, int count,
							int *eof, void *data)
{
	int len = 0;
#endif
	struct pcandev *dev;
	struct list_head *ptr;

#ifdef CREATE_PROC_ENTRY_DEPRECATED
	seq_printf(m, "\n");
	seq_printf(m,
		"*------------- PEAK-System CAN interfaces (www.peak-system.com) -------------\n");
	seq_printf(m,
		"*------------- %s (%s) %s %s --------------\n",
		pcan_drv.szVersionString, CURRENT_VERSIONSTRING,
		__DATE__, __TIME__);
	seq_printf(m, "%s\n", config);
	seq_printf(m, "*--------------------- %d interfaces @ major %03d found -----------------------\n",
		pcan_drv.wDeviceCount, pcan_drv.nMajor);
	seq_printf(m,
		"*n -type- -ndev- --base-- irq --btr- --read-- --write- --irqs-- -errors- status\n");
#else
	len += sprintf(page + len, "\n");
	len += sprintf(page + len,
		"*------------- PEAK-System CAN interfaces (www.peak-system.com) -------------\n");
	len += sprintf(page + len,
		"*------------- %s (%s) %s %s --------------\n",
		pcan_drv.szVersionString, CURRENT_VERSIONSTRING,
		__DATE__, __TIME__);
	len += sprintf(page + len, "%s\n", config);
	len += sprintf(page + len,
		"*--------------------- %d interfaces @ major %03d found -----------------------\n",
		pcan_drv.wDeviceCount, pcan_drv.nMajor);
	len += sprintf(page + len,
		"*n -type- -ndev- --base-- irq --btr- --read-- --write- --irqs-- -errors- status\n");
#endif
#ifdef HANDLE_HOTPLUG
	/* enter critical section (get mutex) */
	pcan_mutex_lock(&pcan_drv.devices_lock);
#endif
	/* loop trough my devices */
	for (ptr = pcan_drv.devices.next; ptr != &pcan_drv.devices;
							ptr = ptr->next) {
		u32 dwPort = 0;
		u16 wIrq = 0, dev_btr0btr1;
		int minor;
#ifdef NETDEV_SUPPORT
		struct net_device_stats *stats; /* rx/tx stats */
#endif

		dev = (struct pcandev *)ptr;
		minor = dev->nMinor;
		dev_btr0btr1 =
			sja1000_bitrate(dev->init_settings.nominal.bitrate,
				dev->init_settings.nominal.sample_point);

		switch (dev->wType) {
		case HW_USB:
		case HW_USB_FD:
		case HW_USB_PRO:
		case HW_USB_PRO_FD:
		case HW_USB_X6:
#ifdef USB_SUPPORT
			/* get serial number of device */
			if (dev->ucPhysicallyInstalled) {
				dwPort = pcan_usb_get_if(dev)->dwSerialNumber;
				wIrq = dev->port.usb.ucHardcodedDevNr;
			} else {
				dwPort = 0x00dead00;  /* it is dead */
				wIrq = 0;
			}
#ifdef CONFIG_USB_DYNAMIC_MINORS
#ifndef PCAN_USB_DONT_REGISTER_DEV
			minor += PCAN_USB_MINOR_BASE;
#endif
#endif
#endif
			break;
		default:
			dwPort = dev->dwPort;
			wIrq = dev->wIrq;
			break;
		}

#ifdef NETDEV_SUPPORT
		stats = (dev->netdev) ?
				pcan_netdev_get_stats(dev->netdev) : NULL;
#endif
#ifdef CREATE_PROC_ENTRY_DEPRECATED
		seq_printf(m,
#else
		len += sprintf(page + len,
#endif
		"%2d %6s %6s %8x %03d 0x%04x %08lx %08lx %08x %08x 0x%04x\n",
			minor,
			dev->type,
#ifdef NETDEV_SUPPORT
			(dev->netdev) ? (dev->netdev->name) : "can?",
#else
			"-NA-",
#endif
			dwPort,
			wIrq,
			dev_btr0btr1,
#ifdef NETDEV_SUPPORT
			(stats) ? stats->rx_packets : 0,
			dev->writeFifo.dwTotal +
					((stats) ? stats->tx_packets : 0),
#else
			(unsigned long)dev->readFifo.dwTotal,
			(unsigned long)dev->writeFifo.dwTotal,
#endif
			dev->dwInterruptCounter,
			dev->dwErrorCounter,
			dev->wCANStatus);
	}

#ifdef HANDLE_HOTPLUG
	/* release mutex */
	pcan_mutex_unlock(&pcan_drv.devices_lock);
#endif
#ifdef CREATE_PROC_ENTRY_DEPRECATED
	return 0;
#else
	len += sprintf(page + len, "\n");

	*eof = 1;
	return len;
#endif
}

#ifdef CONFIG_SYSFS
int pcan_sysfs_add_attr(struct device *dev, struct attribute *attr)
{
	return sysfs_add_file_to_group(&dev->kobj, attr, NULL);
}

int pcan_sysfs_add_attrs(struct device *dev, struct attribute **attrs)
{
	int err = 0;
	struct attribute **ppa;

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(%p=\"%s\")\n", __func__, dev, dev->kobj.name);
#endif
	for (ppa = attrs; *ppa; ppa++) {
		err = pcan_sysfs_add_attr(dev, *ppa);
		if (err) {
			pr_err(DEVICE_NAME
				": failed to add \"%s\" to \"%s\" (err %d)\n",
				(*ppa)->name, dev->kobj.name, err);
			break;
		}
	}

	return err;
}

void pcan_sysfs_del_attr(struct device *dev, struct attribute *attr)
{
	sysfs_remove_file_from_group(&dev->kobj, attr, NULL);
}

void pcan_sysfs_del_attrs(struct device *dev, struct attribute **attrs)
{
	struct attribute **ppa;

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(%p=\"%s\")\n", __func__, dev, dev->kobj.name);
#endif
	for (ppa = attrs; *ppa; ppa++)
		pcan_sysfs_del_attr(dev, *ppa);
}
#else
int pcan_sysfs_add_attr(struct device *dev, struct attribute *attr)
{
	return 0;
}

int pcan_sysfs_add_attrs(struct device *dev, struct attribute **attrs)
{
	return 0;
}

void pcan_sysfs_del_attr(struct device *dev, struct attribute *attr) {}
void pcan_sysfs_del_attrs(struct device *dev, struct attribute **attrs) {}
#endif

#ifdef CREATE_PROC_ENTRY_DEPRECATED
static int open_callback(struct inode *inode, struct file *file)
{
	return single_open(file, pcan_read_procmem, NULL);
}

static struct proc_dir_entry *proc_file_entry;

static const struct file_operations proc_file_fops = {
	.owner = THIS_MODULE,
	.open  = open_callback,
	.read  = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#else
#define proc_file_entry		NULL
#endif

void remove_dev_list(void)
{
	struct pcandev *dev;
	struct list_head *pos, *n;

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif
#ifdef HANDLE_HOTPLUG
	pcan_mutex_lock(&pcan_drv.devices_lock);
#endif

	list_for_each_prev_safe(pos, n, &pcan_drv.devices) {
		dev = list_entry(pos, struct pcandev, list);

		/* prevent from calling more than once the cleanup proc */
		if (dev->cleanup && !(dev->flags & PCAN_DEV_CLEANED)) {
			dev->flags |= PCAN_DEV_CLEANED;
			dev->cleanup(dev);
		}

		dev->wInitStep = 0xff;

		if (dev->flags & PCAN_DEV_LINKED) {
			dev->flags &= ~PCAN_DEV_LINKED;
			list_del(pos);
		}

		pcan_mutex_destroy(&dev->mutex);

		if (!(dev->flags & PCAN_DEV_STATIC)) {
			/* free device object allocated memory */
			pcan_free(dev);
		}

		if (pcan_drv.wDeviceCount)
			pcan_drv.wDeviceCount--;
	}

#ifdef HANDLE_HOTPLUG
	pcan_mutex_unlock(&pcan_drv.devices_lock);
#endif
}

#ifdef UDEV_SUPPORT
static ssize_t pcan_show_version(struct class *cls,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)
				 struct class_attribute *attr,
#endif
				 char *buf)
{
	return sprintf(buf, "%s\n", CURRENT_VERSIONSTRING);
}

static struct class_attribute pcan_attr = {
	.attr = {
		.name = "version",
		.mode = S_IRUGO,
	},
	.show = pcan_show_version,
};
#endif

/* called when the device is removed 'rmmod pcan' */
void cleanup_module(void)
{
#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(init_step=%u)\n",
					__func__, pcan_drv.wInitStep);
#endif
	switch (pcan_drv.wInitStep) {

	case 4:
		remove_proc_entry(DEVICE_NAME, NULL);
	case 3:
		DEV_UNREGISTER();
	case 2:
#ifdef USB_SUPPORT
		pcan_usb_deinit();
#endif

#ifdef PCCARD_SUPPORT
		pcan_pccard_deinit();
#endif

#ifdef PCAN_PCI_EVENT_DRIVEN
		pcan_pci_deinit();
#endif
	case 1:
#ifdef UDEV_SUPPORT
		class_remove_file(pcan_drv.class, &pcan_attr);
		class_destroy(pcan_drv.class);
#endif

#ifdef NO_RT
		unregister_chrdev(pcan_drv.nMajor, DEVICE_NAME);
#endif

		REMOVE_DEV_LIST();

#ifdef HANDLE_HOTPLUG
		/* destroy mutex used to access pcan devices list */
		pcan_mutex_destroy(&pcan_drv.devices_lock);
#endif

	case 0:
		pcan_drv.wInitStep = 0;
	}

	pr_info(DEVICE_NAME ": removed.\n");
}

void pcan_init_adapter(struct pcan_adapter *pa, const char *name, int index,
			int can_count)
{
	if (!pa)
		return;

	pa->name = name;
	pa->index = index;
	pa->can_count = can_count;
	pa->opened_count = 0;
	pa->hw_ver_major = -1;
	pa->hw_ver_minor = -1;
	pa->hw_ver_subminor = -1;
}

struct pcan_adapter *pcan_alloc_adapter(const char *name, int index,
					int can_count)
{
	struct pcan_adapter *pa;

	pa = pcan_malloc(sizeof(struct pcan_adapter), GFP_KERNEL);
	pcan_init_adapter(pa, name, index, can_count);

	return pa;
}

static int pcan_get_channel_features(struct pcandev *dev,
					struct pcanfd_option *opt, void *c)
{
	u32 tmp32 = 0;

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%ld)\n",
		__func__, opt->size, sizeof(tmp32));
#endif

	opt->size = sizeof(tmp32);

	if (dev->device_open_fd)
		tmp32 |= PCANFD_FEATURE_FD;
	if (dev->flags & PCAN_DEV_TXPAUSE_RDY)
		tmp32 |= PCANFD_FEATURE_IFRAME_DELAYUS;
	if (dev->flags & PCAN_DEV_BUSLOAD_RDY)
		tmp32 |= PCANFD_FEATURE_BUSLOAD;
	if (dev->flags & PCAN_DEV_HWTS_RDY)
		tmp32 |= PCANFD_FEATURE_HWTIMESTAMP;

	if (dev->option[PCANFD_OPT_DEVICE_ID].get)
		tmp32 |= PCANFD_FEATURE_DEVICEID;

	if (pcan_copy_to_user(opt->value, &tmp32, opt->size, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int pcan_get_avclocks(struct pcandev *dev,
					struct pcanfd_option *opt, void *c)
{
	int lk;
	u32 ck;

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%ld)\n",
		__func__, opt->size, sizeof(struct pcanfd_available_clocks));
#endif

	/* Copy only the nb of clock corresponding to the user buffer */
	ck = dev->clocks_list->count;
	lk = sizeof(struct pcanfd_available_clocks_0) +
			ck * sizeof(struct pcanfd_available_clock);;

	if (opt->size > lk)
		opt->size = lk;
	else
		ck -= (lk - opt->size) / sizeof(struct pcanfd_available_clock);

	if (pcan_copy_to_user(opt->value, dev->clocks_list, opt->size, c)) {
		pr_err(DEVICE_NAME ": %s(1): copy_to_user() failure\n",
			__func__);
		return -EFAULT;
	}

	/* update count of clocks given to user (if needed) */
	if (ck < dev->clocks_list->count)
		if (pcan_copy_to_user(opt->value, &ck, sizeof(ck), c)) {
			pr_err(DEVICE_NAME ": %s(2): copy_to_user() failure\n",
				__func__);
			return -EFAULT;
		}

	return 0;
}

static int pcan_get_bittiming_range(struct pcandev *dev,
					struct pcanfd_option *opt, void *c)
{
#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%ld)\n",
		__func__, opt->size, sizeof(*dev->bittiming_caps));
#endif

	opt->size = sizeof(*dev->bittiming_caps);
	if (pcan_copy_to_user(opt->value, dev->bittiming_caps, opt->size, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int pcan_get_dbittiming_range(struct pcandev *dev,
					struct pcanfd_option *opt, void *c)
{
#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%ld)\n",
		__func__, opt->size, sizeof(*dev->dbittiming_caps));
#endif

	if (!dev->dbittiming_caps)
		return -EOPNOTSUPP;

	opt->size = sizeof(*dev->dbittiming_caps);
	if (pcan_copy_to_user(opt->value, dev->dbittiming_caps, opt->size, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int pcan_get_allowed_msgs(struct pcandev *dev,
					struct pcanfd_option *opt, void *c)
{
#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%ld)\n",
		__func__, opt->size, sizeof(dev->allowed_msgs));
#endif

	opt->size = sizeof(dev->allowed_msgs);
	if (pcan_copy_to_user(opt->value, &dev->allowed_msgs, opt->size, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int pcan_set_allowed_msgs(struct pcandev *dev,
					struct pcanfd_option *opt, void *c)
{
#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%ld)\n",
		__func__, opt->size, sizeof(dev->allowed_msgs));
#endif

	if (pcan_copy_from_user(&dev->allowed_msgs, opt->value,
					sizeof(dev->allowed_msgs), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int pcan_get_acc_filter_29b(struct pcandev *dev,
					struct pcanfd_option *opt, void *c)
{
#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%ld)\n",
		__func__, opt->size, sizeof(dev->acc_29b.value64));
#endif

	opt->size = sizeof(dev->acc_29b.value64);
	if (pcan_copy_to_user(opt->value, &dev->acc_29b.value64,
				opt->size, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int pcan_set_acc_filter_29b(struct pcandev *dev,
					struct pcanfd_option *opt, void *c)
{
#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%ld)\n",
		__func__, opt->size, sizeof(dev->acc_29b.value64));
#endif

	if (pcan_copy_from_user(&dev->acc_29b.value64, opt->value,
					sizeof(dev->acc_29b.value64), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	dev->acc_29b.mask &= CAN_MAX_EXTENDED_ID;
	dev->acc_29b.code &= ~dev->acc_29b.mask;

	return 0;
}

static int pcan_get_acc_filter_11b(struct pcandev *dev,
					struct pcanfd_option *opt, void *c)
{
#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%ld)\n",
		__func__, opt->size, sizeof(dev->acc_11b.value64));
#endif

	opt->size = sizeof(dev->acc_11b.value64);
	if (pcan_copy_to_user(opt->value, &dev->acc_11b.value64,
					opt->size, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int pcan_set_acc_filter_11b(struct pcandev *dev,
					struct pcanfd_option *opt, void *c)
{
#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%ld)\n",
		__func__, opt->size, sizeof(dev->acc_11b.value64));
#endif

	if (pcan_copy_from_user(&dev->acc_11b.value64, opt->value,
					sizeof(dev->acc_11b.value64), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	dev->acc_11b.mask &= CAN_MAX_STANDARD_ID;
	dev->acc_11b.code &= ~dev->acc_11b.mask;

	return 0;
}

static int pcan_get_ifrm_delay_us(struct pcandev *dev,
					struct pcanfd_option *opt, void *c)
{
#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%ld)\n",
		__func__, opt->size, sizeof(dev->tx_iframe_delay_us));
#endif

	if (!(dev->flags & PCAN_DEV_TXPAUSE_RDY))
		return -EOPNOTSUPP;

	opt->size = sizeof(dev->tx_iframe_delay_us);
	if (pcan_copy_to_user(opt->value, &dev->tx_iframe_delay_us,
					opt->size, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int pcan_set_ifrm_delay_us(struct pcandev *dev,
					struct pcanfd_option *opt, void *c)
{
#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%ld)\n",
		__func__, opt->size, sizeof(dev->acc.value64));
#endif

	if (!(dev->flags & PCAN_DEV_TXPAUSE_RDY))
		return -EOPNOTSUPP;

	if (pcan_copy_from_user(&dev->tx_iframe_delay_us, opt->value,
					sizeof(dev->tx_iframe_delay_us), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int pcan_get_ts_mode(struct pcandev *dev,
					struct pcanfd_option *opt, void *c)
{
#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%ld)\n",
		__func__, opt->size, sizeof(dev->ts_mode));
#endif

	opt->size = sizeof(dev->ts_mode);
	if (pcan_copy_to_user(opt->value, &dev->ts_mode, opt->size, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int pcan_set_ts_mode(struct pcandev *dev,
					struct pcanfd_option *opt, void *c)
{
	u32 tmp;

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%ld)\n",
		__func__, opt->size, sizeof(dev->ts_mode));
#endif

	if (pcan_copy_from_user(&tmp, opt->value, sizeof(tmp), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	switch (tmp) {
	case PCANFD_OPT_HWTIMESTAMP_COOKED:
		if (!(dev->flags & PCAN_DEV_HWTSC_RDY)) {
			// return -EOPNOTSUPP;
			tmp = PCANFD_OPT_HWTIMESTAMP_ON;
		}

	case PCANFD_OPT_HWTIMESTAMP_ON:
	case PCANFD_OPT_HWTIMESTAMP_RAW:
		if (!(dev->flags & PCAN_DEV_HWTS_RDY))
			return -EOPNOTSUPP;

	case PCANFD_OPT_HWTIMESTAMP_OFF:
		break;

	default:
		return -EINVAL;
	}
	
	dev->ts_mode = tmp;
	return 0;
}

static struct pcanfd_options pcan_def_opts[PCANFD_OPT_MAX] = 
{
	[PCANFD_OPT_CHANNEL_FEATURES] = {
		.req_size = sizeof(u32),
		.get = pcan_get_channel_features,
	},
	[PCANFD_OPT_AVAILABLE_CLOCKS] = {
		.req_size = sizeof(struct pcanfd_available_clocks_1),
		.get = pcan_get_avclocks,
	},
	[PCANFD_OPT_BITTIMING_RANGES] = {
		.req_size = sizeof(struct pcanfd_bittiming_range),
		.get = pcan_get_bittiming_range,
	},
	[PCANFD_OPT_DBITTIMING_RANGES] = {
		.req_size = sizeof(struct pcanfd_bittiming_range),
		.get = pcan_get_dbittiming_range,
	},
	[PCANFD_OPT_ALLOWED_MSGS] = {
		.req_size = sizeof(u32),
		.get = pcan_get_allowed_msgs,
		.set = pcan_set_allowed_msgs,
	},
	[PCANFD_OPT_ACC_FILTER_11B] = {
		.req_size = sizeof(u64),
		.get = pcan_get_acc_filter_11b,
		.set = pcan_set_acc_filter_11b,
	},
	[PCANFD_OPT_ACC_FILTER_29B] = {
		.req_size = sizeof(u64),
		.get = pcan_get_acc_filter_29b,
		.set = pcan_set_acc_filter_29b,
	},
	[PCANFD_OPT_IFRAME_DELAYUS] = {
		.req_size = sizeof(u32),
		.get = pcan_get_ifrm_delay_us,
		.set = pcan_set_ifrm_delay_us,
	},
	[PCANFD_OPT_HWTIMESTAMP_MODE] = {
		.req_size = sizeof(u32),
		.get = pcan_get_ts_mode,
		.set = pcan_set_ts_mode,
	},
};

void pcan_inherit_options(struct pcanfd_options *child_opts)
{
	int i;

	/* copy parent option only if child's isn't NULL */
	for (i = 0; i < PCANFD_OPT_MAX; i++) {
		if (!child_opts[i].req_size)
			child_opts[i].req_size = pcan_def_opts[i].req_size;
		if (!child_opts[i].get)
			child_opts[i].get = pcan_def_opts[i].get;
		if (!child_opts[i].set)
			child_opts[i].set = pcan_def_opts[i].set;
	}
};

/* init some equal parts of dev */
void pcan_soft_init_ex(struct pcandev *dev, char *szType, u16 wType,
			const struct pcanfd_available_clocks *clocks,
			const struct pcanfd_bittiming_range *pc,
			u32 flags)
{
	const u32 sysclock_Hz = clocks->list[0].clock_Hz;

	DPRINTK(KERN_DEBUG "%s: %s(\"%s\", wType=%04x, clk=%u)\n",
			DEVICE_NAME, __func__, szType, wType, sysclock_Hz);

	dev->wType = wType;
	dev->type = szType;

	switch (dev->wType) {
	case HW_ISA:
	case HW_DONGLE_SJA:
	case HW_DONGLE_SJA_EPP:
	case HW_DONGLE_PRO:
	case HW_DONGLE_PRO_EPP:
	case HW_ISA_SJA:
	case HW_PCI:
	case HW_PCCARD:
		/* all of these old devices are SJA1000 based devices */
		if (deftsmode == PCANFD_OPT_HWTIMESTAMP_DEF)
			dev->ts_mode = PCANFD_OPT_HWTIMESTAMP_OFF;
		else
			dev->ts_mode = deftsmode;
		flags |= PCAN_DEV_ERRCNT_RDY;
		break;
	case HW_USB:
	case HW_USB_PRO:
		/* these devices have hw timestamps but are not cooked ATM */
		flags |= PCAN_DEV_HWTS_RDY;
		if (deftsmode == PCANFD_OPT_HWTIMESTAMP_DEF)
			dev->ts_mode = PCANFD_OPT_HWTIMESTAMP_ON;
		else
			dev->ts_mode = deftsmode;
		break;

	default:
		/* these devices have hw timestamps can be cooked */
		flags |= PCAN_DEV_HWTS_RDY|PCAN_DEV_HWTSC_RDY;
		if (deftsmode == PCANFD_OPT_HWTIMESTAMP_DEF)
			dev->ts_mode = PCANFD_OPT_HWTIMESTAMP_COOKED;
		else
			dev->ts_mode = deftsmode;
		break;
	}
	dev->flags = flags;

	pcanfd_dev_open_init(dev);

	dev->option = pcan_def_opts;

	dev->nOpenPaths = 0;
	dev->nLastError = 0;
	dev->bus_state = PCANFD_UNKNOWN;
	dev->adapter = NULL;
	dev->wCANStatus = 0;
	dev->filter = NULL;
	dev->sysfs_attrs = NULL;

	dev->rMsg = NULL;
	dev->wMsg = NULL;

	dev->device_alt_num = 0xffffffff;

	dev->bittiming_caps = pc;
	dev->clocks_list = clocks;

	dev->sysclock_Hz = sysclock_Hz;

	dev->def_init_settings.flags = 0;
	dev->def_init_settings.clock_Hz = sysclock_Hz;

	if (pcan_def_bitrate) {
		dev->def_init_settings.nominal.bitrate = pcan_def_bitrate;
		dev->def_init_settings.nominal.brp = 0;

		/* normalize default bit-timings specs */
		pcan_bittiming_normalize(&dev->def_init_settings.nominal,
					dev->sysclock_Hz,
					dev->bittiming_caps);
	} else {

		/* first, compute nominal bitrate from BTR0BTR1 */
		pcan_btr0btr1_to_bittiming(&dev->def_init_settings.nominal,
					btr0btr1);

		/* if default clock is not 8*MHz, rebuild BTR0BTR1
		 * accordingly... */
		if (dev->sysclock_Hz != 8*MHz) {

			/* compute real bittimings with real clock value */
			dev->def_init_settings.nominal.brp = 0;

			pcan_bittiming_normalize(
					&dev->def_init_settings.nominal,
					dev->sysclock_Hz,
					dev->bittiming_caps);
		}
	}

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s() btr0btr1=%04xh => nominal bitrate=%u bps\n",
		__func__, btr0btr1, dev->def_init_settings.nominal.bitrate);
#endif

	/* do the same for dbitrate */
	dev->def_init_settings.data.bitrate = pcan_def_dbitrate;
	dev->dbittiming_caps = NULL;

	dev->init_settings = dev->def_init_settings;

	pcan_init_session_counters(dev);

	memset(&dev->props, 0, sizeof(dev->props));

	/* set default access functions */
	dev->device_open = NULL;
	dev->device_open_fd = NULL;
	dev->device_release = NULL;
	dev->device_write  = NULL;
	dev->cleanup = NULL;

	dev->device_params = NULL;    /* the default */

	dev->ucPhysicallyInstalled = 0;  /* assume the device's not installed */
	dev->ucActivityState = ACTIVITY_NONE;

	/* suppose the device ready to write frames */
	pcan_lock_init(&dev->isr_lock);
	pcan_set_tx_engine(dev, TX_ENGINE_CLOSED);

#if 0
	/* init fifos */
	pcan_fifo_init(&dev->readFifo, &dev->rMsg[0],
			&dev->rMsg[READ_MESSAGE_COUNT - 1],
			READ_MESSAGE_COUNT, sizeof(struct pcanfd_msg));

	pcan_fifo_init(&dev->writeFifo, &dev->wMsg[0],
			&dev->wMsg[WRITE_MESSAGE_COUNT - 1],
			WRITE_MESSAGE_COUNT, sizeof(struct pcanfd_msg));
#else
	memset(&dev->readFifo, '\0', sizeof(dev->readFifo));
	memset(&dev->writeFifo, '\0', sizeof(dev->readFifo));
#endif
	pcan_lock_init(&dev->wlock);
	pcan_mutex_init(&dev->mutex);
}

/* create all declared Peak legacy devices */
static int make_legacy_devices(void)
{
	int result = 0;
	int i;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	for (i = 0; ((i < 8) && (type[i] != NULL)); i++) {
		DPRINTK(KERN_DEBUG "%s: %s(): create devices for type=\"%s\"\n",
				DEVICE_NAME, __func__, type[i]);
#ifdef ISA_SUPPORT
		if (!strncmp(type[i], "isa", 4))
			result = pcan_create_isa_devices(type[i],
							io[i], irq[i]);
#endif

#ifdef DONGLE_SUPPORT
		if (!strncmp(type[i], "sp", 4) ||
					!strncmp(type[i], "epp", 4))
			result = pcan_create_dongle_devices(type[i],
							io[i], irq[i]);
#endif

		if (result)
			break;
	}

#if defined(ISA_SUPPORT) && defined(PCAN_HANDLE_IRQ_SHARING)
	/* create lists of devices with the same irqs */
	pcan_create_isa_shared_irq_lists();
#endif

	return result;
}

extern int strtounit(char *str, u32 *pv, char *units);

static int parmtoul(char *parm, u32 *pv)
{
	if (parm[0] == '0' && (parm[1] == 'x' || parm[1] == 'X')) {
		char *endptr = parm;
		u32 v = simple_strtoul(parm, &endptr, 16);
		if (*endptr)
			return -EINVAL;

		if (pv)
			*pv = v;
		return 'x';
	}

	return (!strtounit(parm, pv, "kM"))? -EINVAL : 'd';
}

/* called when the device is installed 'insmod pcan.o' or 'insmod pcan.ko' */
int init_module(void)
{
	int result = 0;

	memset(&pcan_drv, 0, sizeof(pcan_drv));
	pcan_drv.wInitStep = 0;

	/* in this version, "bitrate" parameter (and new "dbitrate" parameter)
	 * is a string parameter.
	 * Rule (for compatibilit purpose):
	 * - if the string starts with "0x" and if the value is < 0xffff,
	 *   then it is considered as a BTR0BTR1 value
	 * - otherwise, the parameter should be a numeric value, optionaly
	 *   followed by "M" (mega) or "k" (kilo), to specify a new default
	 *   value for the nominal bitrate of the CAN channels. */
	if (bitrate &&
	    (parmtoul(bitrate, &pcan_def_bitrate) == 'x') &&
	    (pcan_def_bitrate <= 0xffff)) {
		btr0btr1 = (u16 )pcan_def_bitrate;
		pcan_def_bitrate = 0;
	}

	if (dbitrate)
		strtounit(dbitrate, &pcan_def_dbitrate, "kM");

	/* check whether Rx/Tx queue default sizes are ok */
	if (rxqsize < PCAN_DEV_RXQSIZE_MIN)
		rxqsize = PCAN_DEV_RXQSIZE_MIN;

	if (rxqsize > PCAN_DEV_RXQSIZE_MAX)
		rxqsize = PCAN_DEV_RXQSIZE_MAX;

	if (txqsize < PCAN_DEV_TXQSIZE_MIN)
		txqsize = PCAN_DEV_TXQSIZE_MIN;

	if (txqsize > PCAN_DEV_TXQSIZE_MAX)
		txqsize = PCAN_DEV_TXQSIZE_MAX;

	//pr_info(DEVICE_NAME ": rxqsize=%u txqsize=%u\n", rxqsize, txqsize);

	if ((dmamask < PCAN_DEV_DMA_MASK_LOW) ||
			(dmamask > PCAN_DEV_DMA_MASK_HIGH))
		dmamask = PCAN_DEV_DMA_MASK_DEF;

	if (deftsmode > PCANFD_OPT_HWTIMESTAMP_MAX)
		deftsmode = PCANFD_OPT_HWTIMESTAMP_MAX;

#ifdef RTAI
	/* this should be done at least once */
	start_rt_timer(0);
#endif
	/* store time for timestamp relation, increments in usec */
	pcan_gettimeofday(&pcan_drv.sInitTime);

	/* get the release name global */
	pcan_drv.szVersionString = CURRENT_RELEASE;
	pcan_drv.nMajor = PCAN_MAJOR;

	pr_info(DEVICE_NAME ": %s (%s)\n", pcan_drv.szVersionString,
#if defined(__BIG_ENDIAN)
		"be"
#else
		"le"
#endif
		);

	pr_info(DEVICE_NAME ": driver config%s\n", current_config);
	if (!pcan_drv.sInitTime.tv_sec && !pcan_drv.sInitTime.tv_usec)
		pr_warn(DEVICE_NAME ": WARNING: got abnormal NULL time\n");

#ifdef DEBUG
	pr_info(DEVICE_NAME ": driver start time=%u.%06u s.\n",
		(u32 )pcan_drv.sInitTime.tv_sec,
		(u32 )pcan_drv.sInitTime.tv_usec);
	pr_info(DEVICE_NAME ": DEBUG is switched on\n");
#endif

	/* Copy the centered string only once and use sizeof() for
	 * compiletime value calculation and optimisation. Also ensure
	 * to have a valid current_config and that it fits into config[] */
	if ((sizeof(current_config) > 3) &&
				(sizeof(config) > sizeof(current_config)))
		strncpy(config + (sizeof(config)-sizeof(current_config))/2,
				current_config, sizeof(current_config)-1);

	INIT_LIST_HEAD(&pcan_drv.devices);
	pcan_drv.wDeviceCount = 0;

#ifdef HANDLE_HOTPLUG
	/* initialize mutex used to access pcan devices list */
	pcan_mutex_init(&pcan_drv.devices_lock);
#endif
#ifndef NO_RT
	INIT_LIST_HEAD(&rt_device_list);
#endif

	/* register the driver by the OS */
#ifdef NO_RT
	result = register_chrdev(pcan_drv.nMajor, DEVICE_NAME, &pcan_fops);
	if (result < 0) {
#ifdef HANDLE_HOTPLUG
		pcan_mutex_destroy(&pcan_drv.devices_lock);
#endif
		goto fail;
	}
	if (!pcan_drv.nMajor)
		pcan_drv.nMajor = result;
#endif

#ifdef UDEV_SUPPORT
	pcan_drv.class = class_create(THIS_MODULE, DEVICE_NAME);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)
	sysfs_attr_init(&pcan_attr.attr);
#endif
	result = class_create_file(pcan_drv.class, &pcan_attr);
	if (result) {
#ifdef HANDLE_HOTPLUG
		pcan_mutex_destroy(&pcan_drv.devices_lock);
#endif
		goto fail;
	}
#endif

	pcan_drv.wInitStep = 1;

#ifdef PCI_SUPPORT
#ifdef PCAN_PCI_EVENT_DRIVEN
	pcan_pci_init();
#else
	pcan_search_and_create_pci_devices();
#endif
	/* search pci devices */
#endif

	/* create isa and dongle devices */
	make_legacy_devices();

#ifdef USB_SUPPORT
	/* register usb devices only */
	pcan_usb_register_devices();
#endif

#ifdef PCCARD_SUPPORT
	pcan_pccard_register_devices();
#endif

#if !defined USB_SUPPORT && !defined PCCARD_SUPPORT
	/* no device found, stop all */
	if (!pcan_drv.wDeviceCount)
		goto fail;
#endif

	pcan_drv.wInitStep = 2;

	result = DEV_REGISTER();
	if (result < 0)
		goto fail;

	if (!pcan_drv.nMajor)
		pcan_drv.nMajor = result;

	pcan_drv.wInitStep = 3;

#ifdef CREATE_PROC_ENTRY_DEPRECATED
	proc_file_entry = proc_create(DEVICE_NAME, 0, NULL, &proc_file_fops);
	if (!proc_file_entry) {
		result = -ENOMEM;
		goto fail;
	}
#else
	/* create the proc entry */
	if (create_proc_read_entry(DEVICE_NAME, 0, NULL,
					pcan_read_procmem, NULL) == NULL) {
		/* maybe wrong if there is no proc filesystem configured */
		result = -ENODEV;
		goto fail;
	}
#endif
	pcan_drv.wInitStep = 4;

	pr_info(DEVICE_NAME ": major %d.\n", pcan_drv.nMajor);

	/* succeed */
	return 0;

fail:
	cleanup_module();
	return result;
}
