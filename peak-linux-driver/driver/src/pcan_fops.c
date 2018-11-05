/*****************************************************************************
 * Copyright (C) 2001-2007  PEAK System-Technik GmbH
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
 *                Arno (a.vdlaan@hccnet.nl)
 *                John Privitera (JohnPrivitera@dciautomation.com)
 *****************************************************************************/

/*****************************************************************************
 *
 * pcan_fops.c - all file operation functions, exports only struct fops
 *
 * $Id$
 *
 *****************************************************************************/
/* #define DEBUG */
/* #undef DEBUG */

#include "src/pcan_common.h"	/* must always be the 1st include */

#include <linux/kernel.h>   // DPRINTK()
#include <linux/slab.h>     // pcan_malloc()
#include <linux/fs.h>       // everything...
#include <linux/errno.h>    // error codes
#include <linux/types.h>    // size_t
#include <linux/proc_fs.h>  // proc
#include <linux/fcntl.h>    // O_ACCMODE
#include <linux/pci.h>      // all about pci
#include <linux/capability.h> // all about restrictions
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
#include <asm/system.h>     // cli(), *_flags
#endif
#include <asm/uaccess.h>    // copy_...
#include <linux/delay.h>    // mdelay()
#include <linux/poll.h>     // poll() and select()

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
#include <linux/moduleparam.h>
#endif

#include "src/pcan_main.h"
#include "src/pcan_pci.h"
#include "src/pcan_isa.h"
#include "src/pcan_dongle.h"
#include "src/pcan_sja1000.h"
#include "src/pcan_fifo.h"
#include "src/pcan_fops.h"
#include "src/pcan_parse.h"
#include "src/pcan_usb.h"
#include "src/pcan_filter.h"

#include "src/pcanfd_core.h"

/* if defined, fix very first open() then read() without any init() */
#define FIX_1ST_READ_WITHOUT_INIT

#ifndef MODULE_LICENSE
#define MODULE_LICENSE(x)
#endif
#ifndef MODULE_VERSION
#define MODULE_VERSION(x)
#endif

#ifdef DEBUG
#define DEBUG_ALLOC_FIFOS
#else
//#define DEBUG_ALLOC_FIFOS
#endif

MODULE_AUTHOR("s.grosjean@peak-system.com");
MODULE_AUTHOR("klaus.hitschler@gmx.de");
MODULE_DESCRIPTION("Driver for PEAK-System CAN interfaces");
MODULE_VERSION(CURRENT_RELEASE);
MODULE_SUPPORTED_DEVICE("PCAN-ISA, PCAN-PC/104, PCAN-Dongle, PCAN-PCI(e), PCAN-ExpressCard, PCAN-PCCard, PCAN-USB (compilation dependent)");
MODULE_LICENSE("GPL");

#if defined(module_param_array) && LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
extern char *type[8];
extern ushort io[8];
extern char irq[8];
extern ushort btr0btr1;
extern char *bitrate;
extern char *dbitrate;
extern char *assign;

ushort rxqsize = READ_MESSAGE_COUNT;
ushort txqsize = WRITE_MESSAGE_COUNT;

module_param_array(type, charp, NULL, 0444);
module_param_array(io, ushort, NULL, 0444);
module_param_array(irq, byte,  NULL, 0444);
module_param(btr0btr1, ushort, 0444);
module_param(bitrate, charp, 0444);
module_param(dbitrate, charp, 0444);
module_param(assign, charp, 0444);
module_param(rxqsize, ushort, 0444);
module_param(txqsize, ushort, 0444);
#else
MODULE_PARM(type, "0-8s");
MODULE_PARM(io, "0-8h");
MODULE_PARM(irq, "0-8b");
MODULE_PARM(btr0btr1, "h");
MODULE_PARM(bitrate, "s");
MODULE_PARM(dbitrate, "s");
MODULE_PARM(assign, "s");
MODULE_PARM(rxqsize, "h");
MODULE_PARM(txqsize, "h");
#endif

MODULE_PARM_DESC(type, "type of PCAN interface (isa, sp, epp)");
MODULE_PARM_DESC(io, "io-port address for either PCAN-ISA, PC/104 or Dongle");
MODULE_PARM_DESC(irq, "interrupt number for either PCAN-ISA, PC/104 or Dongle");
MODULE_PARM_DESC(btr0btr1, "initial bitrate (BTR0BTR1 format) for all channels");
MODULE_PARM_DESC(bitrate, "initial nominal bitrate for all channels");
MODULE_PARM_DESC(dbitrate, "initial data bitrate for all CAN-FD channels");
MODULE_PARM_DESC(assign, "assignment for netdevice names to CAN devices");

MODULE_PARM_DESC(rxqsize, " size of the Rx FIFO of a channel (def="
				__stringify(READ_MESSAGE_COUNT) ")");
MODULE_PARM_DESC(txqsize, " size of the Tx FIFO of a channel (def="
				__stringify(WRITE_MESSAGE_COUNT) ")");

#if defined(LINUX_24)
EXPORT_NO_SYMBOLS;
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,18) || LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#define minor(x)	MINOR(x)
#endif

/* opens a data path with a pcan device.
 * This function is called by:
 * - pcan_open()
 * - pcan_open_rt()
 * - pcan_netdev_open()
 */
int pcan_open_path(struct pcandev *dev, struct pcan_udata *irq_arg)
{
	int err = 0;

	pcan_mutex_lock(&dev->mutex);

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(): minor=%d, opened path=%d\n",
		__func__, dev->nMinor, dev->nOpenPaths);
#endif
	/* only the 1st open to this device makes a default init  */
	if (dev->nOpenPaths) {
		dev->nOpenPaths++;
		goto lbl_unlock_exit;
	}

	dev->rMsg = pcan_malloc(sizeof(dev->rMsg[0]) * rxqsize, GFP_KERNEL);
	if (!dev->rMsg) {
		err = -ENOMEM;
		goto lbl_unlock_exit;
	}

	dev->wMsg = pcan_malloc(sizeof(dev->wMsg[0]) * txqsize, GFP_KERNEL);
	if (!dev->wMsg) {
		err = -ENOMEM;
		goto lbl_unlock_free_rx;
	}

#ifdef NETDEV_SUPPORT
	/* in NETDEV, Rx FIFO is useless, since events are routed towards the
	 * socket buffer */
#else
	/* init Rx fifos */
	pcan_fifo_init(&dev->readFifo, dev->rMsg, dev->rMsg + rxqsize - 1,
			rxqsize, sizeof(struct pcanfd_msg));
#ifdef DEBUG_ALLOC_FIFOS
	pr_info(DEVICE_NAME "%s CAN%u: %u items Rx FIFO allocated\n",
		dev->adapter->name, dev->nChannel+1, rxqsize);
#endif
#endif

	/* init Tx fifo even in NETDEV mode (writing is always possible) */
	pcan_fifo_init(&dev->writeFifo, dev->wMsg, dev->wMsg + txqsize - 1,
			txqsize, sizeof(struct pcanfd_msg));
#ifdef DEBUG_ALLOC_FIFOS
	pr_info(DEVICE_NAME "%s CAN%u: %u items Tx FIFO allocated\n",
		dev->adapter->name, dev->nChannel+1, txqsize);
#endif

#ifndef FIX_1ST_READ_WITHOUT_INIT
	err = pcanfd_dev_reset(dev);
	if (err)
		goto lbl_unlock_free_all;
#endif
	/* open the interface special parts */
	if (dev->open) {
		err = dev->open(dev);
		if (err) {
			pr_err("%s: can't open interface specific!\n",
				DEVICE_NAME);
			goto lbl_unlock_free_all;
		}
	}

	/* special handling: probe here only for dongle devices,
	 * because connect after init is possible */
	switch (dev->wType) {

	case HW_DONGLE_SJA:
	case HW_DONGLE_SJA_EPP:

		/* no usb here, generic sja1000 call for dongle */
		err = sja1000_probe(dev);
		if (err) {
			pr_err("%s: %s-dongle minor %d (io=0x%04x, irq=%d) "
				"not found (err %d)\n",
				DEVICE_NAME, dev->type, dev->nMinor,
				dev->dwPort, dev->wIrq, err);
			dev->release(dev);
			goto lbl_unlock_free_all;
		}
		break;
	}

	/* initialize here the sync mechanism between ISR and fifo */
	pcan_event_init(&dev->in_event, 0);
	pcan_event_init(&dev->out_event, 0);

	/* install irq */
	if (dev->req_irq) {
		err = dev->req_irq(dev, irq_arg);
		if (err) {
			pr_err("%s: can't request irq from device (err %d)\n",
				DEVICE_NAME, err);
			goto lbl_unlock_free_all;
		}
	}

	dev->opened_index = dev->adapter->opened_count++;

	/* inc nOpenPath BEFORE calling _open() because some devices (USB for
	 * ex) may start sending notifications (interrupt based) before
	 * returning from the function. */
	dev->nOpenPaths++;

#ifdef FIX_1ST_READ_WITHOUT_INIT
	pcanfd_dev_reset(dev);
#endif
	pcan_mutex_unlock(&dev->mutex);

	/* open the device itself */
	err = pcanfd_dev_open(dev, &dev->init_settings);
	if (!err)
		return 0;

	pcan_mutex_lock(&dev->mutex);

	dev->nOpenPaths--;
	dev->adapter->opened_count--;

	if (dev->free_irq)
		dev->free_irq(dev, irq_arg);

	pr_err(DEVICE_NAME ": can't open device hardware itself (err %d)!\n",
		err);

lbl_unlock_free_all:
	pcan_free(dev->wMsg);
lbl_unlock_free_rx:
	pcan_free(dev->rMsg);

lbl_unlock_exit:
	pcan_mutex_unlock(&dev->mutex);

	return err;
}

/* find the pcandev according to given major,minor numbers
 * returns NULL pointer in the case of no success
 */
struct pcandev* pcan_search_dev(int major, int minor)
{
	struct pcandev *dev = (struct pcandev *)NULL;
	struct list_head *ptr;

#ifdef DEBUG
	pr_info("%s: %s(): major,minor=%d,%d\n",
		DEVICE_NAME, __func__, major, minor);
#endif

#ifdef PCAN_USB_PCAN_SYSFS
#ifndef PCAN_USB_DONT_REGISTER_DEV
	/* translate USB devices into pcan devices */
	if ((minor >= PCAN_USB_MINOR_BASE) && (minor < PCCARD_MINOR_BASE)) {
		major = USB_MAJOR;
		minor -= PCAN_USB_MINOR_BASE;
	}
#endif
#endif
#ifdef HANDLE_HOTPLUG
	pcan_mutex_lock(&pcan_drv.devices_lock);
#endif
	if (list_empty(&pcan_drv.devices)) {
#ifdef HANDLE_HOTPLUG
		pcan_mutex_unlock(&pcan_drv.devices_lock);
#endif
		DPRINTK(KERN_DEBUG "%s: no devices to select from!\n",
			DEVICE_NAME);
		return NULL;
	}

	/* loop through my devices */
	for (ptr = pcan_drv.devices.next; ptr != &pcan_drv.devices;
							ptr = ptr->next) {
		dev = (struct pcandev *)ptr;

#ifndef XENOMAI3
		if (dev->nMajor == major)
#endif
			if (dev->nMinor == minor)
				break;
	}

#ifdef HANDLE_HOTPLUG
	pcan_mutex_unlock(&pcan_drv.devices_lock);
#endif
	if (ptr == &pcan_drv.devices) {
		printk(KERN_DEBUG "%s: didn't find any pcan devices (%d,%d)\n",
			DEVICE_NAME, major, minor);
		return NULL;
	}

	return dev;
}

/* is called by pcan_release() and pcan_netdev_close() */
void pcan_release_path(struct pcandev *dev, struct pcan_udata *irq_arg)
{
	int err;

	pcan_mutex_lock(&dev->mutex);

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(): minor=%d, path=%d\n",
		__func__, dev->nMinor, dev->nOpenPaths);
#endif
	/* if it's the last release: init the chip for non-intrusive operation*/
	if (dev->nOpenPaths > 1) {
		dev->nOpenPaths--;

		pcan_mutex_unlock(&dev->mutex);

	} else if (dev->nOpenPaths == 1)  {

#ifndef PCAN_USES_OLD_TX_ENGINE_STATE
		pcan_lock_irqsave_ctxt flags;
#endif
		/* tell the world that the device is being closed now */
		dev->nOpenPaths = 0;

		pcan_mutex_unlock(&dev->mutex);

		if (pcan_task_can_wait()) {
#ifdef DEBUG
			pr_info("%s: preparing to wait: "
				"ucPhysicallyInstalled=%u fifo_empty=%u "
				"tx_engine_state=%d to=%d\n",
				DEVICE_NAME, dev->ucPhysicallyInstalled,
				pcan_fifo_empty(&dev->writeFifo),
#ifdef PCAN_USES_OLD_TX_ENGINE_STATE
				atomic_read(&dev->tx_engine_state),
#else
				dev->locked_tx_engine_state,
#endif
				MAX_WAIT_UNTIL_CLOSE);
#endif

			/* Now, wait for the tx fifo to be empty and for the 
			 * tx engine of the hardware to finish... */
			err = pcan_event_wait_timeout(dev->out_event,
					!dev->ucPhysicallyInstalled ||
					(pcan_fifo_empty(&dev->writeFifo) &&
#ifdef PCAN_USES_OLD_TX_ENGINE_STATE
					atomic_read(&dev->tx_engine_state) !=
#else
					dev->locked_tx_engine_state !=
#endif
							TX_ENGINE_STARTED),
					MAX_WAIT_UNTIL_CLOSE);

#ifdef DEBUG
			pr_info("%s: finished waiting: "
				"ucPhysicallyInstalled=%u fifo_empty=%u "
				"tx_engine_state=%d err=%d\n",
				DEVICE_NAME, dev->ucPhysicallyInstalled,
				pcan_fifo_empty(&dev->writeFifo),
#ifdef PCAN_USES_OLD_TX_ENGINE_STATE
				atomic_read(&dev->tx_engine_state),
#else
				dev->locked_tx_engine_state,
#endif
				err);
#endif
		}

#ifdef PCAN_USES_OLD_TX_ENGINE_STATE
		pcan_set_tx_engine(dev, TX_ENGINE_CLOSED);
#else
		pcan_lock_get_irqsave(&dev->isr_lock, flags);
		pcan_set_tx_engine(dev, TX_ENGINE_CLOSED);
		pcan_lock_put_irqrestore(&dev->isr_lock, flags);
#endif

		/* release the device itself */
		if (dev->device_release)
			dev->device_release(dev);

		dev->flags &= ~PCAN_DEV_OPENED;
		pcan_set_bus_state(dev, PCANFD_UNKNOWN);

		if (dev->release)
			dev->release(dev);

		/* release the device irq */
		if (dev->free_irq)
			dev->free_irq(dev, irq_arg);

		if (dev->adapter->opened_count > 0)
			dev->adapter->opened_count--;

		/* destroy useless syncs */
		pcan_event_free(&dev->in_event);
		pcan_event_free(&dev->out_event);

		/* destroy useless Rx/Tx fifos */
		pcan_free(dev->wMsg);
#ifdef DEBUG_ALLOC_FIFOS
		pr_info(DEVICE_NAME "%s CAN%u: Tx FIFO released\n",
			dev->adapter->name, dev->nChannel+1);
#endif
#ifdef NETDEV_SUPPORT
#else
		pcan_free(dev->rMsg);
#ifdef DEBUG_ALLOC_FIFOS
		pr_info(DEVICE_NAME "%s CAN%u: Rx FIFO released\n",
			dev->adapter->name, dev->nChannel+1);
#endif
#endif
	}
}

/* is called at user ioctl() with cmd = PCAN_GET_STATUS */
int pcan_ioctl_status_common(struct pcandev *dev, TPSTATUS *local)
{
	local->wErrorFlag = dev->wCANStatus;

	/* get infos for friends of polling operation */
	if (pcan_fifo_empty(&dev->readFifo))
		local->wErrorFlag |= CAN_ERR_QRCVEMPTY;

	if (pcan_fifo_full(&dev->writeFifo))
		local->wErrorFlag |= CAN_ERR_QXMTFULL;

	local->nLastError = dev->nLastError;

	return 0;
}

/* is called at user ioctl() with cmd = PCAN_GET_EXT_STATUS */
int pcan_ioctl_extended_status_common(struct pcandev *dev,
					TPEXTENDEDSTATUS *local)
{
	local->wErrorFlag = dev->wCANStatus;

	local->nPendingReads = dev->readFifo.nStored;

	/* get infos for friends of polling operation */
	if (pcan_fifo_empty(&dev->readFifo))
		local->wErrorFlag |= CAN_ERR_QRCVEMPTY;

	local->nPendingWrites = dev->writeFifo.nStored;

	if (pcan_fifo_full(&dev->writeFifo))
		local->wErrorFlag |= CAN_ERR_QXMTFULL;

	local->nLastError = dev->nLastError;

	return 0;
}

/* is called at user ioctl() with cmd = PCAN_DIAG */
int pcan_ioctl_diag_common(struct pcandev *dev, TPDIAG *local)
{
	local->wType = dev->wType;

	switch (dev->wType) {
	case HW_USB:
	case HW_USB_FD:
	case HW_USB_PRO:
	case HW_USB_PRO_FD:
	case HW_USB_X6:
#ifdef USB_SUPPORT 
		local->dwBase = pcan_usb_get_if(dev)->dwSerialNumber;
		local->wIrqLevel = dev->port.usb.ucHardcodedDevNr;
#endif
		break;
	default:
		local->dwBase = dev->dwPort;
		local->wIrqLevel = dev->wIrq;
		break;
	}

	local->dwReadCounter = dev->readFifo.dwTotal;
	local->dwWriteCounter = dev->writeFifo.dwTotal;
	local->dwIRQcounter = dev->dwInterruptCounter;
	local->dwErrorCounter = dev->dwErrorCounter;
	local->wErrorFlag = dev->wCANStatus;

	/* get infos for friends of polling operation */
	if (pcan_fifo_empty(&dev->readFifo))
		local->wErrorFlag |= CAN_ERR_QRCVEMPTY;

	if (pcan_fifo_full(&dev->writeFifo))
		local->wErrorFlag |= CAN_ERR_QXMTFULL;

	local->nLastError = dev->nLastError;
	local->nOpenPaths = dev->nOpenPaths;

	strncpy(local->szVersionString,
			pcan_drv.szVersionString, VERSIONSTRING_LEN);
	return 0;
}

#define PCANFD_MAX_MSGS	8

struct __array_of_struct(pcanfd_msg, PCANFD_MAX_MSGS);
#define pcanfd_max_msgs		pcanfd_msgs_PCANFD_MAX_MSGS

static int handle_pcanfd_send_msgs(struct pcandev *dev, void __user *up,
					struct pcan_udata *dev_priv, void *c)
{
	int l, err;

#ifdef pcanfd_max_msgs
	struct pcanfd_max_msgs msgfdl;

	l = sizeof(struct pcanfd_max_msgs);
	err = pcan_copy_from_user(&msgfdl, up, l, c);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	if (msgfdl.count > PCANFD_MAX_MSGS)
		msgfdl.count = PCANFD_MAX_MSGS;

	err = pcanfd_ioctl_send_msgs(dev,
				(struct pcanfd_msgs *)&msgfdl, dev_priv);

	/* copy only the count of msgs really sent (= pl->count) */
	if (pcan_copy_to_user(up, &msgfdl, sizeof(struct pcanfd_msgs_0), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
	}
#else
	struct pcanfd_msgs msgfdl, *pl;

	l = sizeof(struct pcanfd_msgs_0);
	err = pcan_copy_from_user(&msgfdl, up, l, c);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	/* ok. Nothing to send. So nothing done. Perfect. */
	if (!msgfdl.count)
		return 0;

	l += msgfdl.count * sizeof(msgfd);
	pl = pcan_malloc(l, GFP_KERNEL);
	if (!pl) {
		pr_err(DEVICE_NAME ": %s(): failed to alloc msgs list\n",
			__func__);
		return -ENOMEM;
	}

	if (pcan_copy_from_user(pl, up, l, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		pcan_free(pl);
		return -EFAULT;
	}
#if 0
	{
		int i;

		for (i = 0; i < pl->count; i++) {
			pr_info(DEVICE_NAME ": id=%x len=%u\n",
				pl->list[i].id,
				pl->list[i].data_len);
			
			dump_mem("data", pl->list[i].data,
						pl->list[i].data_len);
		}
	}
#endif
	err = pcanfd_ioctl_send_msgs(dev, pl, dev_priv);

	/* copy the count of msgs really sent (= pl->count) */
	if (pcan_copy_to_user(up, pl, sizeof(struct pcanfd_msgs_0), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
	}

	pcan_free(pl);
#endif

	return err;
}

static int handle_pcanfd_recv_msgs(struct pcandev *dev, void __user *up,
					struct pcan_udata *dev_priv, void *c)
{
	int l, err;

#ifdef pcanfd_max_msgs
	struct pcanfd_max_msgs msgfdl;

	l = sizeof(struct pcanfd_msgs_0);
	err = pcan_copy_from_user(&msgfdl, up, l, c);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	if (msgfdl.count > PCANFD_MAX_MSGS)
		msgfdl.count = PCANFD_MAX_MSGS;

	err = pcanfd_ioctl_recv_msgs(dev,
				(struct pcanfd_msgs *)&msgfdl, dev_priv);

	/* copy the count and the msgs received */
	l += msgfdl.count * sizeof(struct pcanfd_msg);
	if (pcan_copy_to_user(up, &msgfdl, l, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
	}
#else
	struct pcanfd_msgs msgfdl, *pl;

	l = sizeof(struct pcanfd_msgs_0);
	err = pcan_copy_from_user(&msgfdl, up, l, c);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	/* ok! no room for saving rcvd msgs!? Thus, nothing returned */
	if (!msgfdl.count)
		return 0;

	l += msgfdl.count * sizeof(msgfd);
	pl = pcan_malloc(l, GFP_KERNEL);
	if (!pl) {
		pr_err(DEVICE_NAME ": failed to alloc msgs list\n");
		return -ENOMEM;
	}

	pl->count = msgfdl.count;
	err = pcanfd_ioctl_recv_msgs(dev, pl, dev_priv);

	/* copy the count and the msgs received */
	l = sizeof(struct pcanfd_msgs_0) + pl->count * sizeof(msgfd);
	if (pcan_copy_to_user(up, pl, l, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
	}

	pcan_free(pl);
#endif

	return err;
}

static int handle_pcanfd_get_av_clocks(struct pcandev *dev, void __user *up,
					struct pcan_udata *dev_priv,
					void *c)
{
	struct pcanfd_available_clocks avclks;
	int l = sizeof(struct pcanfd_available_clocks_0);
	const void *kp;
	int err;

	err = pcan_copy_from_user(&avclks, up, l, c);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	/* copy only the count of clocks of the device */
	if (avclks.count >= dev->clocks_list->count) {
		kp = dev->clocks_list;
		l += dev->clocks_list->count *
				sizeof(struct pcanfd_available_clock);

	/* copy only the count of clocks requested by user */
	} else {
		up += l;
		kp = &dev->clocks_list->list;
		l += avclks.count *
				sizeof(struct pcanfd_available_clock);
	}

	if (pcan_copy_to_user(up, kp, l, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
	}

	return err;
}

static int handle_pcanfd_get_bittiming_ranges(struct pcandev *dev,
						void __user *up,
						struct pcan_udata *dev_priv,
						void *c)
{
	struct __array_of_struct(pcanfd_bittiming_range, 2) fdbtr;
	int l = sizeof(struct pcanfd_bittiming_ranges_0);
	//int l = sizeof(fdbtr.count);
	int err = pcan_copy_from_user(&fdbtr, up, l, c);
	u32 user_count;

	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	/* keep in memory the max given by user */
	user_count = fdbtr.count;

	/* CAN-FD: max of 2 bittiming ranges */
	memset(&fdbtr, '\0', sizeof(fdbtr));

	if (fdbtr.count < user_count) {
		fdbtr.list[fdbtr.count++] = *dev->bittiming_caps;

		if (dev->dbittiming_caps)
			if (fdbtr.count < user_count)
				fdbtr.list[fdbtr.count++] =
						*dev->dbittiming_caps;
	}

	/* copy the count of bittiming ranges read from the device */
	l += fdbtr.count * sizeof(struct pcanfd_bittiming_range);
	if (pcan_copy_to_user(up, &fdbtr, l, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
	}

	return err;
}

static int handle_pcanfd_get_option(struct pcandev *dev, void __user *up,
					struct pcan_udata *dev_priv, void *c)
{
	struct pcanfd_option opt;
	const int l = sizeof(opt);

	int err = pcan_copy_from_user(&opt, up, l, c);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	if (opt.name >= PCANFD_OPT_MAX) {
		pr_err(DEVICE_NAME ": invalid option name %d to get\n",
			opt.name);
		return -EINVAL;
	}

	if (!dev->option[opt.name].get) {
		return -EOPNOTSUPP;
	}

	if (dev->option[opt.name].req_size > 0)

		/* if user option buffer size is too small, return the 
		 * requested size with -ENOSPC */
		if (opt.size < dev->option[opt.name].req_size) {
			pr_warn(DEVICE_NAME
				": invalid option size %d < %d for option %d\n",
				opt.size, dev->option[opt.name].req_size,
				opt.name);
			opt.size = dev->option[opt.name].req_size;
			err = -ENOSPC;
			goto lbl_cpy_size;
		}

	err = dev->option[opt.name].get(dev, &opt, c);
	if (err)
		return err;

lbl_cpy_size:
	/* update 'size' field */
	if (pcan_copy_to_user(up+offsetof(struct pcanfd_option, size),
			 &opt.size, sizeof(opt.size), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
	}

	return err;
}

static int handle_pcanfd_set_option(struct pcandev *dev, void __user *up,
					struct pcan_udata *dev_priv, void *c)
{
	struct pcanfd_option opt;
	int l = sizeof(opt);

	int err = pcan_copy_from_user(&opt, up, l, c);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	if (opt.name >= PCANFD_OPT_MAX) {
		pr_err(DEVICE_NAME ": invalid option name %d to get\n",
			opt.name);
		return -EINVAL;
	}

	if (!dev->option[opt.name].set) {
		return -EOPNOTSUPP;
	}

	return dev->option[opt.name].set(dev, &opt, c);
}

/*
 * Inculde system specific entry points:
 */
#ifdef NO_RT
#include "pcan_fops_linux.c"
#else
#include "pcan_fops_rt.c"
#endif
