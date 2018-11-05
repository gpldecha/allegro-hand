/*****************************************************************************
 * Copyright (C) 2003-2010  PEAK System-Technik GmbH
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
 * Maintainer: Stephane Grosjean (s.grosjean@peak-system.com)
 *
 * Major contributions by:
 *                Oliver Hartkopp (oliver.hartkopp@volkswagen.de) socketCAN
 *                Klaus Hitschler (klaus.hitschler@gmx.de)
 *
 * Contributions: Philipp Baer (philipp.baer@informatik.uni-ulm.de)
 *                Tom Heinrich
 *                John Privitera (JohnPrivitera@dciautomation.com)
 *****************************************************************************/

/*****************************************************************************
 *
 * pcan_usb_core.c - the outer usb parts for pcan-usb and pcan-usb-pro support
 *
 * $Id: pcan_usb_core.c 626 2010-06-16 21:37:49Z khitschler $
 *
 *****************************************************************************/
/* #define DEBUG */
/* #undef DEBUG */

#include "src/pcan_common.h"     /* must always be the 1st include */

#ifdef USB_SUPPORT

#include <linux/stddef.h>        /* NULL */
#include <linux/errno.h>
#include <linux/slab.h>          /* pcan_malloc() */

#include <linux/usb.h>
#include <linux/net.h>

#include "src/pcan_main.h"
#include "src/pcan_fops.h"
#include "src/pcan_usb_core.h"
#include "src/pcan_usb.h"
#include "src/pcan_usbpro.h"
#include "src/pcanfd_usb.h"
#include "src/pcanfd_core.h"
#include "src/pcan_filter.h"

#ifdef NETDEV_SUPPORT
#include "src/pcan_netdev.h"     // for hotplug pcan_netdev_(un)register()
#endif

#ifdef DEBUG
#define PCAN_USB_DEBUG_WRITE
#define PCAN_USB_DEBUG_DECODE
#else
//#define PCAN_USB_DEBUG_WRITE
//#define PCAN_USB_DEBUG_DECODE
#endif

#define PCAN_USB_VENDOR_ID		0x0c72
#define PCAN_USB_PRODUCT_ID		0x000c
#define PCAN_USBPRO_PRODUCT_ID		0x000d

#define PCAN_USB_READ_BUFFER_SIZE_OLD	64   /* used len for PCAN-USB rev < 6*/
#define PCAN_USB_READ_BUFFER_SIZE	1024 /* buffer for read URB data (IN) */
#define PCAN_USB_READ_PACKET_SIZE	64   /* always 64 (USB1 device) */

#define PCAN_USB_WRITE_BUFFER_SIZE_OLD	64    // length for PCAN-USB rev < 6
//#define PCAN_USB_WRITE_BUFFER_SIZE	128   // buffer for write URB (OUT)
#define PCAN_USB_WRITE_BUFFER_SIZE	256   // ...says Win driver
#define PCAN_USB_WRITE_PACKET_SIZE	64    // always 64 (USB1 device)

/* Defines the size of one USB message that can be received from the device
 * Driver allocates one buffer of n x PCAN_USBPRO_READ_BUFFER_SIZE to handle
 * consecutive reads */
//#define PCAN_USBPRO_READ_BUFFER_SIZE   1024
#define PCAN_USBPRO_READ_BUFFER_SIZE	2048
//#define PCAN_USBPRO_READ_BUFFER_SIZE	4096

#define MAX_CYCLES_TO_WAIT_FOR_RELEASE	100   /* max schedules before release */

/* wait this time in seconds at startup to get first messages */
#define STARTUP_WAIT_TIME		0.01

static struct usb_device_id pcan_usb_ids[] = {
	{ USB_DEVICE(PCAN_USB_VENDOR_ID, PCAN_USB_PRODUCT_ID) },
	{ USB_DEVICE(PCAN_USB_VENDOR_ID, PCAN_USBPRO_PRODUCT_ID) },
	{ USB_DEVICE(PCAN_USB_VENDOR_ID, PCAN_USBFD_PRODUCT_ID) },
	{ USB_DEVICE(PCAN_USB_VENDOR_ID, PCAN_USBPROFD_PRODUCT_ID) },
	{ USB_DEVICE(PCAN_USB_VENDOR_ID, PCAN_USBCHIP_PRODUCT_ID) },
#ifdef PCAN_USBX6_PRODUCT_ID
	{ USB_DEVICE(PCAN_USB_VENDOR_ID, PCAN_USBX6_PRODUCT_ID) },
#endif
	{ }	/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, pcan_usb_ids);

#ifndef PCAN_USB_DONT_REGISTER_DEV
static struct usb_class_driver pcan_class = {
	.name = "pcanusb%d",
	.fops = &pcan_fops,

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
	.mode = S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
#endif

	.minor_base = PCAN_USB_MINOR_BASE,
};
#endif

static int usb_devices = 0;		/* the number of accepted usb_devices */

/* this function is global for USB adapters */
struct pcan_usb_interface *pcan_usb_get_if(struct pcandev *pdev)
{
#ifdef PCAN_USBX6_PRODUCT_ID
	return pdev->port.usb.usb_if;
#else
	return (struct pcan_usb_interface *)pdev->adapter;
#endif
}

/* forward declaration for chardev pcan_usb_write_notitfy() */
static int pcan_usb_write(struct pcandev *dev, struct pcan_udata *ctx);

static void pcan_usb_write_notify(struct urb *purb, struct pt_regs *pregs)
{
	struct pcandev *dev = purb->context;
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	pcan_lock_irqsave_ctxt lck_ctx;
	int err = purb->status;

#ifdef PCAN_USB_DEBUG_WRITE
	printk(KERN_INFO "%s(): status=%d actual_length=%d\n",
			__func__, purb->status, purb->actual_length);
#endif

#if 1
	/* SGr: useful? */
#else
	if (!usb_if) {
		pr_info(DEVICE_NAME "%s(%u): usb_if=NULL\n",
				__func__, __LINE__);
		return;
	}
#endif
	/* un-register outstanding urb */
	atomic_dec(&usb_if->active_urbs);

	/* don't count interrupts - count packets */
	dev->dwInterruptCounter++;

	pcan_lock_get_irqsave(&dev->isr_lock, lck_ctx);

	switch (err) {

	case 0:
		dev->tx_frames_counter++;
		err = pcan_usb_write(dev, NULL);
		if (!err)
			break;

#ifdef PCAN_USES_OLD_TX_ENGINE_STATE
		/* engine stopped */
		atomic_set(&dev->tx_engine_state, TX_ENGINE_STOPPED);
#else
		/* done by pcan_usb_write() */
		//dev->locked_tx_engine_state = TX_ENGINE_STOPPED;
#endif
		if (err == -ENODATA) {

			/* signal I'm ready to write again */
			pcan_event_signal(&dev->out_event);

#ifdef NETDEV_SUPPORT
			netif_wake_queue(dev->netdev);
#endif
			break;
		}

		dev->nLastError = err;

		/* build the error frame and put it into Rx FIFO */
		if (!(dev->wCANStatus & CAN_ERR_QXMTFULL)) {
			struct pcanfd_msg ef;

			pcan_handle_error_ctrl(dev, &ef, PCANFD_TX_OVERFLOW);
			if (pcan_xxxdev_rx(dev, &ef) > 0)
				pcan_event_signal(&dev->in_event);
		}
		break;

	default:

/*
	case -ECONNRESET:
	case -ESHUTDOWN:
 */
		pr_err(DEVICE_NAME ": %s(%u): USB abnormal err %d\n",
				__func__, __LINE__, err);

	case -ENOENT:	/* urb killed */

		/* engine stopped */
		pcan_set_tx_engine(dev, TX_ENGINE_STOPPED);
		break;
	}

	pcan_lock_put_irqrestore(&dev->isr_lock, lck_ctx);
}

static void pcan_usb_read_notify(struct urb *purb, struct pt_regs *pregs)
{
	struct pcan_usb_interface *usb_if = purb->context;
	struct pcandev *dev;
	u8 *read_buffer_addr = purb->transfer_buffer;
	const int read_buffer_len = purb->actual_length;
	int read_buffer_size;
	int err, d;

#if 0
	DPRINTK(KERN_DEBUG "%s: %s() status=%d\n",
	        DEVICE_NAME, __func__, purb->status);
#endif

	/* un-register outstanding urb */
	atomic_dec(&usb_if->active_urbs);

	/* do interleaving read, stop with first error */
	switch (purb->status) {

	case 0:
		break;

	case -ECONNRESET:	/* usb_unlink_urb() called */
	case -ENOENT:		/* urb killed */
	case -EPIPE:
		DPRINTK(KERN_DEBUG "%s: read data stream turned off (err %d)\n",
				DEVICE_NAME, purb->status);
		return;

#if 0
	/* here also are cases that have been seen to occur */
	case -EOVERFLOW:
#endif
	/* error codes when USB device is hot unplugged */
	case -ESHUTDOWN:	/* the ep is being disabled */
	case -EPROTO:
	case -EILSEQ:

	default:
		pr_err("%s: unhandled read data stream turned off (err %d)\n",
				DEVICE_NAME, purb->status);

		/* "unplug" all devices of the same USB adapter */
		dev = usb_if->dev;
		for (d = 0; d < usb_if->can_count; dev++, d++) {

			/* seems that this is the most reasonable thing to do
			 * most of the times...  */
			dev->ucPhysicallyInstalled = 0;

			/* unlock any waiting tasks */
			if (dev->nOpenPaths > 0) {
				pcan_event_signal(&dev->out_event);
				pcan_event_signal(&dev->in_event);
			}
		}
		return;
	}

	/* buffer interleave to increase speed */
	if (read_buffer_addr == usb_if->read_buffer_addr[0]) {
		FILL_BULK_URB(purb, usb_if->usb_dev,
				usb_rcvbulkpipe(usb_if->usb_dev,
						usb_if->pipe_read.ucNumber),
				usb_if->read_buffer_addr[1],
				usb_if->read_buffer_size,
				pcan_usb_read_notify, usb_if);
	} else {
		FILL_BULK_URB(purb, usb_if->usb_dev,
				usb_rcvbulkpipe(usb_if->usb_dev,
						usb_if->pipe_read.ucNumber),
				usb_if->read_buffer_addr[0],
				usb_if->read_buffer_size,
				pcan_usb_read_notify, usb_if);
	}

	/* start next urb */
	err = __usb_submit_urb(purb);
	if (err) {
		pr_err("%s: %s() URB submit failure %d\n",
		       DEVICE_NAME, __func__, err);
	} else {
		atomic_inc(&usb_if->active_urbs);
	}

	/* decoding the received one */
#ifdef PCAN_USB_DEBUG_DECODE
	pr_info("%s: got %u bytes URB, decoding it by packets of %u bytes:\n",
		DEVICE_NAME, read_buffer_len, usb_if->read_packet_size);
#endif

	for (read_buffer_size = 0; read_buffer_size < read_buffer_len; ) {
#ifdef PCAN_USB_DEBUG_DECODE
		pr_info("%s: decoding @offset %u:\n",
			DEVICE_NAME, read_buffer_size);
#endif
		err = usb_if->device_msg_decode(usb_if,
						read_buffer_addr,
						usb_if->read_packet_size);
		if (err < 0) {
#ifdef PCAN_USB_DEBUG_DECODE
			if (net_ratelimit())
				pr_err("%s: offset %d: msg decoding error %d\n",
				       DEVICE_NAME, read_buffer_size, err);
#endif
			/* no need to continue because error can be:
			 * - not enough space in rx fifo
			 * - decoding is out of sync.
			 */
			break;
		}

		read_buffer_addr += usb_if->read_packet_size;
		read_buffer_size += usb_if->read_packet_size;
	}
}

/* USB write functions */
static int pcan_usb_write(struct pcandev *dev, struct pcan_udata *ctx)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	USB_PORT *u = &dev->port.usb;
	int err = 0;

	u8 *write_buffer_addr = u->write_buffer_addr;
	int write_packet_size;
	int write_buffer_size;
#ifdef PCAN_USES_OLD_TX_ENGINE_STATE
	pcan_lock_irqsave_ctxt lck_ctx;
#endif

#if 0
	DPRINTK(KERN_DEBUG "%s: %s(CAN%u)\n",
		DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	/* don't do anything with non-existent hardware */
	if (!dev->ucPhysicallyInstalled)
		return -ENODEV;

#ifdef PCAN_USES_OLD_TX_ENGINE_STATE
	pcan_lock_get_irqsave(&dev->wlock, lck_ctx);
#endif

	for (write_buffer_size=0; write_buffer_size < u->write_buffer_size; ) {
		write_packet_size = u->write_packet_size;
		err = usb_if->device_ctrl_msg_encode(dev,
		                                     write_buffer_addr,
		                                     &write_packet_size);

		if (err >= 0) {
			write_buffer_size += u->write_packet_size;
			write_buffer_addr += u->write_packet_size;
#ifdef PCAN_USB_DEBUG_WRITE
			printk(KERN_INFO
				"%s: encoded %u bytes in %u bytes packet\n",
				DEVICE_NAME, write_packet_size,
				u->write_packet_size);
#endif
			continue;
		}

#ifdef PCAN_USB_DEBUG_WRITE
		printk(KERN_INFO "%s: err=%d: total=%u/%u\n", DEVICE_NAME,
				err, write_buffer_size, u->write_buffer_size);
#endif
		switch (err) {
		case -ENODATA:
			write_buffer_size += write_packet_size;
			break;

		case -ENOSPC:
			write_buffer_size += write_packet_size;
			err = 0;
			break;

		default:
			break;
		}

		break;
	}

	if (write_buffer_size > 0) {
#ifdef PCAN_USB_DEBUG_WRITE
		dump_mem("message sent to device",
			u->write_buffer_addr, write_buffer_size);

		printk(KERN_INFO
			"%s: submitting %u bytes buffer to usb EP#%d\n",
			DEVICE_NAME, write_buffer_size, u->pipe_write.ucNumber);
#endif

		FILL_BULK_URB(&u->write_data, usb_if->usb_dev,
		              usb_sndbulkpipe(usb_if->usb_dev,
				              u->pipe_write.ucNumber),
		              u->write_buffer_addr, write_buffer_size,
		              pcan_usb_write_notify, dev);

		/* remember the USB device is BUSY */
		pcan_set_tx_engine(dev, TX_ENGINE_STARTED);

		/* start next urb */
		err = __usb_submit_urb(&u->write_data);
		if (err) {
			dev->nLastError = err;
			dev->dwErrorCounter++;

			printk(KERN_ERR "%s: %s() URB submit failure %d\n",
			        DEVICE_NAME, __func__, err);
		} else {
			//dev->wCANStatus &= ~CAN_ERR_QXMTFULL;
			pcan_clear_status_bit(dev, CAN_ERR_QXMTFULL);
			atomic_inc(&usb_if->active_urbs);
		}
	}

#ifdef PCAN_USES_OLD_TX_ENGINE_STATE
	pcan_lock_put_irqrestore(&dev->wlock, lck_ctx);
#else
	if (err && (err != -EBUSY))
		pcan_set_tx_engine(dev, TX_ENGINE_STOPPED);
#endif

	return err;
}

static void pcan_usb_free_resources(struct pcan_usb_interface *usb_if)
{
	struct pcandev *dev = &usb_if->dev[0];
	USB_PORT *u;
	int c;

	for (c = 0; c < usb_if->can_count; c++, dev++) {
		u = &dev->port.usb;

		pcan_free(u->write_buffer_addr);
		pcan_free(u->cout_baddr);
	}

	pcan_free(usb_if->read_buffer_addr[0]);
}

/* usb resource allocation */
static int pcan_usb_alloc_resources(struct pcan_usb_interface *usb_if)
{
	struct pcandev *dev;
	USB_PORT *u;
	int err = 0;
	int c;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	/* make param URB */
#ifndef PCAN_USB_CMD_PER_DEV
	usb_init_urb(&usb_if->urb_cmd_async);
#endif
	usb_init_urb(&usb_if->urb_cmd_sync);

	/* allocate write buffer
	 * Check revision according to device id. */
	switch (le16_to_cpu(usb_if->usb_dev->descriptor.idProduct)) {

	case PCAN_USBFD_PRODUCT_ID:
	case PCAN_USBCHIP_PRODUCT_ID:
		usb_if->read_packet_size = 4096;
		usb_if->dev[0].port.usb.write_packet_size = 512;

		usb_if->read_buffer_size = usb_if->read_packet_size;
		usb_if->dev[0].port.usb.write_buffer_size =
				usb_if->dev[0].port.usb.write_packet_size;
		usb_if->dev[0].port.usb.cout_bsize = 512;
		break;
	case PCAN_USBPROFD_PRODUCT_ID:
		usb_if->read_packet_size = 4096;

		usb_if->dev[0].port.usb.write_packet_size =
				usb_if->dev[1].port.usb.write_packet_size = 512;

		usb_if->read_buffer_size = usb_if->read_packet_size;
		usb_if->dev[0].port.usb.write_buffer_size =
				usb_if->dev[0].port.usb.write_packet_size;
		usb_if->dev[1].port.usb.write_buffer_size =
				usb_if->dev[1].port.usb.write_packet_size;
		usb_if->dev[0].port.usb.cout_bsize =
				usb_if->dev[1].port.usb.cout_bsize = 512;
		break;
#ifdef PCAN_USBX6_PRODUCT_ID
	case PCAN_USBX6_PRODUCT_ID:
		usb_if->read_packet_size = 4096;

		usb_if->dev[0].port.usb.write_packet_size =
				usb_if->dev[1].port.usb.write_packet_size = 512;

		usb_if->read_buffer_size = usb_if->read_packet_size;
		usb_if->dev[0].port.usb.write_buffer_size =
				usb_if->dev[0].port.usb.write_packet_size;
		usb_if->dev[1].port.usb.write_buffer_size =
				usb_if->dev[1].port.usb.write_packet_size;
		usb_if->dev[0].port.usb.cout_bsize =
				usb_if->dev[1].port.usb.cout_bsize = 512;
		break;
#endif
	case PCAN_USBPRO_PRODUCT_ID:
		/* Rev 0x00 */

		/* Copied from Win32 Driver:
		 * DeviceContext->IsDeviceHighSpeed ? 512 : 64
		 * 512 bytes packet size leads to fragmentation issue while
		 * messages are 1024 bytes large */
		if (usb_if->usb_dev->speed == USB_SPEED_HIGH) {
			usb_if->read_packet_size = 1024;
			usb_if->dev[0].port.usb.write_packet_size =
			   usb_if->dev[1].port.usb.write_packet_size = 512;
		} else {
			usb_if->read_packet_size = 64;
			usb_if->dev[0].port.usb.write_packet_size =
			   usb_if->dev[1].port.usb.write_packet_size = 64;
		}

		usb_if->dev[0].port.usb.cout_bsize = PCAN_USB_WRITE_PACKET_SIZE;
		usb_if->dev[1].port.usb.cout_bsize = PCAN_USB_WRITE_PACKET_SIZE;

#ifdef PCAN_USBPRO_READ_BUFFER_SIZE
		usb_if->read_buffer_size = PCAN_USBPRO_READ_BUFFER_SIZE;
#else
		usb_if->read_buffer_size = usb_if->read_packet_size;
#endif

#ifdef PCAN_USBPRO_WRITE_BUFFER_SIZE
		usb_if->dev[0].port.usb.write_buffer_size =
				PCAN_USBPRO_WRITE_BUFFER_SIZE;
		usb_if->dev[1].port.usb.write_buffer_size =
				PCAN_USBPRO_WRITE_BUFFER_SIZE;
#else
		usb_if->dev[0].port.usb.write_buffer_size =
                               usb_if->dev[0].port.usb.write_packet_size;
		usb_if->dev[1].port.usb.write_buffer_size =
                               usb_if->dev[1].port.usb.write_packet_size;
#endif

		break;

	case PCAN_USB_PRODUCT_ID:
		usb_if->dev[0].port.usb.cout_bsize = PCAN_USB_WRITE_PACKET_SIZE;
		if (usb_if->ucRevision >= 7) {
			usb_if->read_buffer_size = PCAN_USB_READ_BUFFER_SIZE;
			usb_if->dev[0].port.usb.write_buffer_size =
						PCAN_USB_WRITE_BUFFER_SIZE;
			usb_if->read_packet_size = PCAN_USB_READ_PACKET_SIZE;
			usb_if->dev[0].port.usb.write_packet_size =
						PCAN_USB_WRITE_PACKET_SIZE;
			break;
		}
	default:
		usb_if->read_buffer_size = PCAN_USB_READ_BUFFER_SIZE_OLD;
		usb_if->dev[0].port.usb.write_buffer_size =
						PCAN_USB_WRITE_BUFFER_SIZE_OLD;
		usb_if->read_packet_size = PCAN_USB_READ_PACKET_SIZE;
		usb_if->dev[0].port.usb.write_packet_size =
						PCAN_USB_WRITE_PACKET_SIZE;
		break;
	}

	dev = &usb_if->dev[0];
	for (c = 0; c < usb_if->can_count; c++, dev++) {
		u = &dev->port.usb;

#ifdef PCAN_USB_CMD_PER_DEV
		/* make param URB */
		usb_init_urb(&u->urb_cmd_sync);
		usb_init_urb(&u->urb_cmd_async);
#endif
		u->write_buffer_addr =
				pcan_malloc(u->write_buffer_size, GFP_KERNEL);
		if (!u->write_buffer_addr) {
			err = -ENOMEM;
			goto fail;
		}

		DPRINTK(KERN_DEBUG
			"%s: %s() allocate %d bytes buffer for writing\n",
		        DEVICE_NAME, __func__, u->write_buffer_size);

		/* make write urb */
		usb_init_urb(&u->write_data);

		if (u->cout_bsize) {
			u->cout_baddr = pcan_malloc(u->cout_bsize, GFP_KERNEL);
			if (!u->cout_baddr) {
				err = -ENOMEM;
				goto fail;
			}
		} else {
			u->cout_baddr = NULL;
		}
	}

	/* allocate two read buffers for URB */
	usb_if->read_buffer_addr[0] =
			pcan_malloc(usb_if->read_buffer_size * 2, GFP_KERNEL);
	if (!usb_if->read_buffer_addr[0]) {
		err = -ENOMEM;
		goto fail;
	}

	DPRINTK(KERN_DEBUG
		"%s: %s() allocate %d buffers of %d bytes for reading\n",
	        DEVICE_NAME, __func__, 2, usb_if->read_buffer_size);

	usb_if->read_buffer_addr[1] = usb_if->read_buffer_addr[0]
	                            + usb_if->read_buffer_size;

	/* make read urb */
	usb_init_urb(&usb_if->read_data);

fail:
	return err;
}

static int pcan_kill_sync_urb(struct urb *urb)
{
	int err = 0;

	if (urb->status == -EINPROGRESS) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
		usb_kill_urb(urb);
#else
		err = usb_unlink_urb(urb);
#endif
		DPRINTK(KERN_DEBUG "%s: %s() done...\n", DEVICE_NAME, __func__);
	}

	return err;
}

static int pcan_usb_stop(struct pcandev *dev)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	USB_PORT *u = &dev->port.usb;
	int err = 0;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u), minor=%d\n",
	        DEVICE_NAME, __func__, dev->nChannel+1, dev->nMinor);

#if 0
	if (!usb_if) {
		pr_info(DEVICE_NAME "%s(%u): usb_if=NULL\n",
				__func__, __LINE__);
		return -ENODEV;
	}
#endif
	if (!(dev->flags & PCAN_DEV_OPENED))
		return 0;

	if (usb_if->device_ctrl_close)
		err = usb_if->device_ctrl_close(dev);

	if (usb_if->opened_count > 0)
		usb_if->opened_count--;

	/* unlink URBs for device/controller */
	pcan_kill_sync_urb(&u->write_data);

	DPRINTK(KERN_DEBUG "%s: have still %d active URBs on interface\n",
	        DEVICE_NAME, atomic_read(&usb_if->active_urbs));

	return usb_if->device_ctrl_set_bus_off(dev);
}

/* remove device resources */
static int pcan_usb_cleanup(struct pcandev *dev)
{
	if (dev) {
		USB_PORT *u = &dev->port.usb;

		DPRINTK(KERN_DEBUG "%s: %s(CAN%u): wInitStep=%d\n",
			DEVICE_NAME, __func__, dev->nChannel+1, dev->wInitStep);

		pcan_free(u->write_buffer_addr);
		u->write_buffer_addr = NULL;

		switch(dev->wInitStep) {
		case 4:
			dev->ucPhysicallyInstalled = 0;
#if 1
			/* Hem... These events are normally "destroyed" when
			 * the device was closed... ("normally" because the 
			 * 'pcan_event' is in fact not destroyed nor deleted 
			 * when running in non-RT... */
#else
			/* New: unlock any waiting task */
			pcan_event_signal(&dev->out_event);
			pcan_event_signal(&dev->in_event);
#endif

#ifdef NETDEV_SUPPORT
			pcan_netdev_unregister(dev);
#endif
		case 3:
			usb_devices--;
		case 2:
			pcan_dev_remove_from_list(dev);
		case 1:
		case 0:
			dev->filter = pcan_delete_filter_chain(dev->filter);
		}

	} else {
		DPRINTK(KERN_DEBUG "%s: %s(NULL dev)\n", DEVICE_NAME, __func__);
	}

	return 0;
}

#if 0
/* dummy entries for request and free irq */
static int pcan_usb_req_irq(struct pcandev *dev, struct pcan_udata *dev_priv)
{
	return 0;
}
#endif

static void pcan_usb_free_irq(struct pcandev *dev, struct pcan_udata *dev_priv)
{
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	/* mis-used here for another purpose
	 * pcan_usb_free_irq() calls when the last path to device just closing
	 * and the device itself is already plugged out */
	if ((dev) && (!dev->ucPhysicallyInstalled))
		pcan_usb_cleanup(dev);
}

/* interface depended open and close */
static int pcan_usb_open(struct pcandev *dev)
{
	DPRINTK(KERN_DEBUG "%s: %s(), minor = %d.\n",
	        DEVICE_NAME, __func__, dev->nMinor);

	return 0;
}

static int pcan_usb_release(struct pcandev *dev)
{
	DPRINTK(KERN_DEBUG "%s: %s(), minor = %d.\n",
	        DEVICE_NAME, __func__, dev->nMinor);

	return 0;
}

static int pcan_usb_device_open_fd(struct pcandev *dev,
					struct pcanfd_init *pfdi)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	USB_PORT *u = &dev->port.usb;
	int err = 0;

	DPRINTK(KERN_DEBUG "%s: %s(), minor = %d. (nOpenPaths=%d)\n",
	        DEVICE_NAME, __func__, dev->nMinor, dev->nOpenPaths);

#if 1
	/* TODO SGr 20160324: I think the below test is useless, since the 
	 * "device_open()" callabck is called *ONLY* once, when 
	 * "dev->nOpenPaths" is 0... */
#else
	/* in general, when second open() occurs
	 * remove and unlink urbs, when interface is already running */
	if ((dev->nOpenPaths) && (dev->device_release))
		dev->device_release(dev);
	else
#endif
	/* otherwise, first action: turn CAN off */
	if ((err = usb_if->device_ctrl_set_bus_off(dev)))
		goto fail;

	memset(&u->usb_time, '\0', sizeof(PCAN_USB_TIME));

	/* init hardware specific parts */
	if (usb_if->device_ctrl_open_fd) {
		err = usb_if->device_ctrl_open_fd(dev, pfdi);

	} else {
		TPCANInit init;

		pcan_fd_to_init(&init, pfdi);

		err = usb_if->device_ctrl_open(dev,
					init.wBTR0BTR1,
					init.ucCANMsgType & MSGTYPE_EXTENDED,
					init.ucListenOnly);
	}

	if (err)
		goto fail;

	usb_if->opened_count++;

	/* last action: turn CAN on */
	if ((err = usb_if->device_ctrl_set_bus_on(dev)))
		goto fail;

	/* delay to get first messages read */
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout((int)(STARTUP_WAIT_TIME * HZ + 0.9));

fail:
	return err;
}

/* emulated device access functions
 * call is only possible if device exists */
static int pcan_usb_device_open(struct pcandev *dev, uint16_t btr0btr1,
                                u8 bExtended, u8 bListenOnly)
{
	struct pcanfd_init fd_init;
	TPCANInit init = {
		.wBTR0BTR1 = btr0btr1,
		.ucCANMsgType = bExtended ? MSGTYPE_EXTENDED : MSGTYPE_STANDARD,
		.ucListenOnly = bListenOnly
	};

	return pcan_usb_device_open_fd(dev, pcan_init_to_fd(&fd_init, &init));
}

static void pcan_usb_device_release(struct pcandev *dev)
{
	DPRINTK(KERN_DEBUG "%s: %s(), minor=%d (nOpenPaths=%d).\n",
	        DEVICE_NAME, __func__, dev->nMinor, dev->nOpenPaths);

	/* test only mdelay(100); */

	pcan_usb_stop(dev);
}

/* get or set special device related parameters */
static int  pcan_usb_device_params(struct pcandev *dev, TPEXTRAPARAMS *params)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	USB_PORT *u = &dev->port.usb;
	int err;

	DPRINTK(KERN_DEBUG "%s: %s(%d)\n", DEVICE_NAME, __func__,
	        params->nSubFunction);

	switch (params->nSubFunction) {
	case SF_GET_SERIALNUMBER:
		err = usb_if->device_get_snr(usb_if,
						&params->func.dwSerialNumber);
		break;
	case SF_GET_HCDEVICENO:
		/* can cast to u32 * since "func" is an union with
		 * dwSerialNumber */
		err = usb_if->device_ctrl_get_dnr(dev,
				//(u32 *)&params->func.ucHCDeviceNo);
				&params->func.dwSerialNumber);
		break;
	case SF_SET_HCDEVICENO:
		/*
		 * err = usb_if->device_ctrl_set_dnr(dev,
		 *			params->func.ucHCDeviceNo);
		 */
		err = usb_if->device_ctrl_set_dnr(dev,
						params->func.dwSerialNumber);
		/* Should update dev object cache with new value
		 * (see /dev/pcan display)*/
		if (!err) {
			u->ucHardcodedDevNr = params->func.ucHCDeviceNo;
#if 1
			/* why not caching full 32b value in device_alt_num? */
			dev->device_alt_num = params->func.dwSerialNumber;
#else
			dev->device_alt_num = u->ucHardcodedDevNr;
#endif
		}
		break;

	default:
		DPRINTK(KERN_DEBUG "%s: Unknown sub-function %d!\n",
		        DEVICE_NAME, params->nSubFunction);

		return -EINVAL;
	}

	return err;
}

/* things to do after plugin or plugout of device (and power on too) */
#ifndef PCAN_USB_PCAN_SYSFS

#define PCAN_DEVICE_ATTR(_v, _name, _show) \
	struct device_attribute pcan_dev_attr_##_v = \
					__ATTR(_name, S_IRUGO, _show, NULL)

static struct pcandev *to_pcandev(struct device *dev, int c)
{
	struct usb_interface *interface = to_usb_interface(dev->parent);
	struct pcan_usb_interface *usb_if = usb_get_intfdata(interface);
	return (struct pcandev *)&usb_if->dev[c];
}

static ssize_t show_pcan_x_hwtype(int can_idx, struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev, can_idx)->wType);
}

static ssize_t show_pcan_x_devid(int can_idx, struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev, can_idx)->device_alt_num);
}
static ssize_t show_pcan_x_minor(int can_idx, struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	return show_int(buf, to_pcandev(dev, can_idx)->nMinor);
}

static ssize_t show_pcan_x_ctrl_number(int can_idx, struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	return show_int(buf, to_pcandev(dev, can_idx)->nChannel);
}

static ssize_t show_pcan_x_bitrate(int can_idx, struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf,
		to_pcandev(dev, can_idx)->init_settings.nominal.bitrate);
}

static ssize_t show_pcan_x_clock(int can_idx, struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev, can_idx)->init_settings.clock_Hz);
}

static ssize_t show_pcan_x_bus_state(int can_idx, struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev, can_idx)->bus_state);
}

static ssize_t show_pcan_x_rx_err_cnt(int can_idx, struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev, can_idx)->rx_error_counter);
}

static ssize_t show_pcan_x_tx_err_cnt(int can_idx, struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev, can_idx)->tx_error_counter);
}

static ssize_t show_pcan_x_bus_load(int can_idx, struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev, can_idx)->bus_load);
}

static ssize_t show_pcan_x_dbitrate(int can_idx, struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf,
			to_pcandev(dev, can_idx)->init_settings.data.bitrate);
}

/* /proc/pcan redundant */
static ssize_t show_pcan_x_type(int can_idx, struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_str(buf, to_pcandev(dev, can_idx)->type);
}

#ifdef NETDEV_SUPPORT
static ssize_t show_pcan_x_ndev(int can_idx, struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pcandev *pdev = to_pcandev(dev, can_idx);
	return show_str(buf, pdev->netdev ? pdev->netdev->name : "can?");
}
#endif

/* like with /proc/pcan, display Serial/Number instead */
static ssize_t show_pcan_x_serial_number(int can_idx, struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n",
		to_pcandev(dev, can_idx)->port.usb.usb_if->dwSerialNumber);
}

#if 1
/* don't display /proc/pcan useless irq column, since devid already exists */
#else
static ssize_t show_pcan_x_irq(int can_idx, struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf,
		to_pcandev(dev, can_idx)->port.usb.ucHardcodedDevNr);
}
#endif

static ssize_t show_pcan_x_btr0btr1(int can_idx, struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pcandev *pdev = to_pcandev(dev, can_idx);
	u32 dev_btr0btr1 = sja1000_bitrate(pdev->init_settings.nominal.bitrate,
				pdev->init_settings.nominal.sample_point);
	return snprintf(buf, PAGE_SIZE, "0x%04x\n", dev_btr0btr1);
}

static ssize_t show_pcan_x_read(int can_idx, struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pcandev *pdev = to_pcandev(dev, can_idx);
#ifdef NETDEV_SUPPORT
	struct net_device_stats *stats = (pdev->netdev) ?
				pcan_netdev_get_stats(dev->netdev) : NULL;
	u32 dev_read = (stats) ? stats->rx_packets : 0;

#else
	u32 dev_read = pdev->readFifo.dwTotal;
#endif
	return show_u32(buf, dev_read);
}

static ssize_t show_pcan_x_write(int can_idx, struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pcandev *pdev = to_pcandev(dev, can_idx);
#ifdef NETDEV_SUPPORT
	struct net_device_stats *stats = (pdev->netdev) ?
				pcan_netdev_get_stats(dev->netdev) : NULL;
	u32 dev_write = (stats) ? stats->tx_packets : 0;

#else
	u32 dev_write = pdev->writeFifo.dwTotal;
#endif
	return show_u32(buf, dev_write);
}

static ssize_t show_pcan_x_irqs(int can_idx, struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev, can_idx)->dwInterruptCounter);
}

static ssize_t show_pcan_x_errors(int can_idx, struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_pcandev(dev, can_idx)->dwErrorCounter);
}

static ssize_t show_pcan_x_status(int can_idx, struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04x\n",
					to_pcandev(dev, can_idx)->wCANStatus);
}

/*
 * per-channel 'show' callbacks
 */
#define PCAN_DEFINE_ATTR_SHOW(_name, _c)				\
	static ssize_t show_pcan_##_name##_c(struct device *dev,	\
			   struct device_attribute *attr, char *buf)	\
{									\
	return show_pcan_x_##_name(_c, dev, attr, buf);			\
}

/* Channel #0 show callbacks definitions */
PCAN_DEFINE_ATTR_SHOW(hwtype, 0)
PCAN_DEFINE_ATTR_SHOW(devid, 0)
PCAN_DEFINE_ATTR_SHOW(minor, 0)
PCAN_DEFINE_ATTR_SHOW(ctrl_number, 0)
PCAN_DEFINE_ATTR_SHOW(bitrate, 0)
PCAN_DEFINE_ATTR_SHOW(clock, 0)
PCAN_DEFINE_ATTR_SHOW(bus_state, 0)
PCAN_DEFINE_ATTR_SHOW(rx_err_cnt, 0)
PCAN_DEFINE_ATTR_SHOW(tx_err_cnt, 0)
PCAN_DEFINE_ATTR_SHOW(bus_load, 0)
PCAN_DEFINE_ATTR_SHOW(dbitrate, 0)

/* /proc/pcan redundant */
PCAN_DEFINE_ATTR_SHOW(type, 0)
#ifdef NETDEV_SUPPORT
PCAN_DEFINE_ATTR_SHOW(ndev, 0)
#endif
PCAN_DEFINE_ATTR_SHOW(serial_number, 0)
PCAN_DEFINE_ATTR_SHOW(btr0btr1, 0)
PCAN_DEFINE_ATTR_SHOW(read, 0)
PCAN_DEFINE_ATTR_SHOW(write, 0)
PCAN_DEFINE_ATTR_SHOW(irqs, 0)
PCAN_DEFINE_ATTR_SHOW(errors, 0)
PCAN_DEFINE_ATTR_SHOW(status, 0)

/* Channel #1 show callbacks definitions */
PCAN_DEFINE_ATTR_SHOW(hwtype, 1)
PCAN_DEFINE_ATTR_SHOW(devid, 1)
PCAN_DEFINE_ATTR_SHOW(minor, 1)
PCAN_DEFINE_ATTR_SHOW(ctrl_number, 1)
PCAN_DEFINE_ATTR_SHOW(bitrate, 1)
PCAN_DEFINE_ATTR_SHOW(clock, 1)
PCAN_DEFINE_ATTR_SHOW(bus_state, 1)
PCAN_DEFINE_ATTR_SHOW(rx_err_cnt, 1)
PCAN_DEFINE_ATTR_SHOW(tx_err_cnt, 1)
PCAN_DEFINE_ATTR_SHOW(bus_load, 1)
PCAN_DEFINE_ATTR_SHOW(dbitrate, 1)

/* /proc/pcan redundant */
PCAN_DEFINE_ATTR_SHOW(type, 1)
#ifdef NETDEV_SUPPORT
PCAN_DEFINE_ATTR_SHOW(ndev, 1)
#endif
PCAN_DEFINE_ATTR_SHOW(serial_number, 1)
PCAN_DEFINE_ATTR_SHOW(btr0btr1, 1)
PCAN_DEFINE_ATTR_SHOW(read, 1)
PCAN_DEFINE_ATTR_SHOW(write, 1)
PCAN_DEFINE_ATTR_SHOW(irqs, 1)
PCAN_DEFINE_ATTR_SHOW(errors, 1)
PCAN_DEFINE_ATTR_SHOW(status, 1)

/* Channel #0 attributes declaration */
static PCAN_DEVICE_ATTR(devid0, devid, show_pcan_devid0);
static PCAN_DEVICE_ATTR(hwtype0, hwtype, show_pcan_hwtype0);
static PCAN_DEVICE_ATTR(minor0, minor, show_pcan_minor0);
static PCAN_DEVICE_ATTR(ctrl_number0, ctrl_number, show_pcan_ctrl_number0);
static PCAN_DEVICE_ATTR(bitrate0, bitrate, show_pcan_bitrate0);
static PCAN_DEVICE_ATTR(clock0, clock, show_pcan_clock0);
static PCAN_DEVICE_ATTR(bus_state0, bus_state, show_pcan_bus_state0);

/* /proc/pcan redundant */
static PCAN_DEVICE_ATTR(type0, type, show_pcan_type0);
#ifdef NETDEV_SUPPORT
static PCAN_DEVICE_ATTR(ndev0, ndev, show_pcan_ndev0);
#endif
static PCAN_DEVICE_ATTR(serial_number0, serial_number, show_pcan_serial_number0);
static PCAN_DEVICE_ATTR(btr0btr10, btr0btr1, show_pcan_btr0btr10);
static PCAN_DEVICE_ATTR(read0, read, show_pcan_read0);
static PCAN_DEVICE_ATTR(write0, write, show_pcan_write0);
static PCAN_DEVICE_ATTR(irqs0, irqs, show_pcan_irqs0);
static PCAN_DEVICE_ATTR(errors0, errors, show_pcan_errors0);
static PCAN_DEVICE_ATTR(status0, status, show_pcan_status0);

/* Channel #1 attributes declaration */
static PCAN_DEVICE_ATTR(devid1, devid, show_pcan_devid1);
static PCAN_DEVICE_ATTR(hwtype1, hwtype, show_pcan_hwtype1);
static PCAN_DEVICE_ATTR(minor1, minor, show_pcan_minor1);
static PCAN_DEVICE_ATTR(ctrl_number1, ctrl_number, show_pcan_ctrl_number1);
static PCAN_DEVICE_ATTR(bitrate1, bitrate, show_pcan_bitrate1);
static PCAN_DEVICE_ATTR(clock1, clock, show_pcan_clock1);
static PCAN_DEVICE_ATTR(bus_state1, bus_state, show_pcan_bus_state1);

/* /proc/pcan redundant */
static PCAN_DEVICE_ATTR(type1, type, show_pcan_type1);
#ifdef NETDEV_SUPPORT
static PCAN_DEVICE_ATTR(ndev1, ndev, show_pcan_ndev1);
#endif
static PCAN_DEVICE_ATTR(serial_number1, serial_number, show_pcan_serial_number1);
static PCAN_DEVICE_ATTR(btr0btr11, btr0btr1, show_pcan_btr0btr11);
static PCAN_DEVICE_ATTR(read1, read, show_pcan_read1);
static PCAN_DEVICE_ATTR(write1, write, show_pcan_write1);
static PCAN_DEVICE_ATTR(irqs1, irqs, show_pcan_irqs1);
static PCAN_DEVICE_ATTR(errors1, errors, show_pcan_errors1);
static PCAN_DEVICE_ATTR(status1, status, show_pcan_status1);

#ifdef NETDEV_SUPPORT
#define PCAN_DEV_ATTRS_COUNT		20
#else
#define PCAN_DEV_ATTRS_COUNT		19
#endif

static struct attribute *pcan_usb_sysfs_attrs[][PCAN_DEV_ATTRS_COUNT] = {
	{
		&pcan_dev_attr_devid0.attr,
		&pcan_dev_attr_hwtype0.attr,
		&pcan_dev_attr_minor0.attr,
		&pcan_dev_attr_ctrl_number0.attr,
		&pcan_dev_attr_bitrate0.attr,
		&pcan_dev_attr_clock0.attr,
		&pcan_dev_attr_bus_state0.attr,

		/* /proc/pcan redundant */
		&pcan_dev_attr_type0.attr,
#ifdef NETDEV_SUPPORT
		&pcan_dev_attr_ndev0.attr,
#endif
		&pcan_dev_attr_serial_number0.attr,
		&pcan_dev_attr_btr0btr10.attr,
		&pcan_dev_attr_read0.attr,
		&pcan_dev_attr_write0.attr,
		&pcan_dev_attr_irqs0.attr,
		&pcan_dev_attr_errors0.attr,
		&pcan_dev_attr_status0.attr,

		NULL
	},
	{
		&pcan_dev_attr_devid1.attr,
		&pcan_dev_attr_hwtype1.attr,
		&pcan_dev_attr_minor1.attr,
		&pcan_dev_attr_ctrl_number1.attr,
		&pcan_dev_attr_bitrate1.attr,
		&pcan_dev_attr_clock1.attr,
		&pcan_dev_attr_bus_state1.attr,

		/* /proc/pcan redundant */
		&pcan_dev_attr_type1.attr,
#ifdef NETDEV_SUPPORT
		&pcan_dev_attr_ndev1.attr,
#endif
		&pcan_dev_attr_serial_number1.attr,
		&pcan_dev_attr_btr0btr11.attr,
		&pcan_dev_attr_read1.attr,
		&pcan_dev_attr_write1.attr,
		&pcan_dev_attr_irqs1.attr,
		&pcan_dev_attr_errors1.attr,
		&pcan_dev_attr_status1.attr,

		NULL
	},
};

static PCAN_DEVICE_ATTR(dbitrate0, dbitrate, show_pcan_dbitrate0);
static PCAN_DEVICE_ATTR(dbitrate1, dbitrate, show_pcan_dbitrate1);

static struct attribute *pcanfd_usb_sysfs_attrs[][2] = {
	{
		&pcan_dev_attr_dbitrate0.attr,
		NULL
	},
	{
		&pcan_dev_attr_dbitrate1.attr,
		NULL
	},
};

static PCAN_DEVICE_ATTR(bus_load0, bus_load, show_pcan_bus_load0);
static PCAN_DEVICE_ATTR(bus_load1, bus_load, show_pcan_bus_load1);

static struct attribute *pcan_usb_sysfs_bus_load_attrs[][2] = {
	{
		&pcan_dev_attr_bus_load0.attr,
		NULL
	},
	{
		&pcan_dev_attr_bus_load1.attr,
		NULL
	},
};

static PCAN_DEVICE_ATTR(rx_err_cnt0, rx_error_counter, show_pcan_rx_err_cnt0);
static PCAN_DEVICE_ATTR(rx_err_cnt1, rx_error_counter, show_pcan_rx_err_cnt1);

static PCAN_DEVICE_ATTR(tx_err_cnt0, tx_error_counter, show_pcan_tx_err_cnt0);
static PCAN_DEVICE_ATTR(tx_err_cnt1, tx_error_counter, show_pcan_tx_err_cnt1);

static struct attribute *pcan_usb_sysfs_err_cnt_attrs[][3] = {
	{
		&pcan_dev_attr_rx_err_cnt0.attr,
		&pcan_dev_attr_tx_err_cnt0.attr,
		NULL
	},
	{
		&pcan_dev_attr_rx_err_cnt1.attr,
		&pcan_dev_attr_tx_err_cnt1.attr,
		NULL
	},
};
#endif /* PCAN_USB_PCAN_SYSFS */

static int pcan_usb_get_devid(struct pcandev *dev,
				struct pcanfd_option *opt, void *c)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	u32 dev_id;

	int err = usb_if->device_ctrl_get_dnr(dev, &dev_id);
	if (err) {
		pr_err(DEVICE_NAME
			": %s() err %d getting dev number from %s CAN%d\n",
			__func__, err, dev->adapter->name, dev->nChannel+1);
		return err;
	}

	opt->size = sizeof(dev_id);
	if (pcan_copy_to_user(opt->value, &dev_id, opt->size, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int pcan_usb_set_devid(struct pcandev *dev,
				struct pcanfd_option *opt, void *c)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	USB_PORT *u = &dev->port.usb;
	u32 dev_id;

	int err = pcan_copy_from_user(&dev_id, opt->value, sizeof(u32), c);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	err = usb_if->device_ctrl_set_dnr(dev, dev_id);
	if (err) {
		pr_err(DEVICE_NAME
			": %s() err %d setting dev number to %s CAN%d\n",
			__func__, err, dev->adapter->name, dev->nChannel+1);
		return err;
	}

	/* Should update dev object cache with new value
	 * (see /dev/pcan display) */
	u->ucHardcodedDevNr = (u8 )dev_id;
	dev->device_alt_num = dev_id; //u->ucHardcodedDevNr;

	return 0;
}

/* USB device specific options */
static struct pcanfd_options pcan_usb_opts[PCANFD_OPT_MAX] =
{
	[PCANFD_OPT_DEVICE_ID] = {
		.req_size = sizeof(u32),
		.get = pcan_usb_get_devid,
		.set = pcan_usb_set_devid,
	},
};

static int pcan_usb_create_dev(struct pcan_usb_interface *usb_if,
                               int ctrl_index)
{
	struct pcandev *dev = (struct pcandev *)&usb_if->dev[ctrl_index];
	struct usb_device *usb_dev = usb_if->usb_dev;
	USB_PORT *u = &dev->port.usb;
	int err, retry;
#ifndef PCAN_USB_DONT_REGISTER_DEV
	void *h;
#endif

	switch (le16_to_cpu(usb_dev->descriptor.idProduct)) {
	case PCAN_USBFD_PRODUCT_ID:
	case PCAN_USBCHIP_PRODUCT_ID:

		/* init structure elements to defaults */
		ucan_soft_init(dev, "usbfd", HW_USB_FD, usb_if->adapter);
		break;
	case PCAN_USBPROFD_PRODUCT_ID:

		/* init structure elements to defaults */
		ucan_soft_init(dev, "usbfd", HW_USB_PRO_FD, usb_if->adapter);
		break;
#ifdef PCAN_USBX6_PRODUCT_ID
	case PCAN_USBX6_PRODUCT_ID:

		/* init structure elements to defaults */
		ucan_soft_init(dev, "usbfd", HW_USB_X6, usb_if->adapter);
		break;
#endif
	case PCAN_USBPRO_PRODUCT_ID:

		/* init structure elements to defaults */
		pcan_soft_init_ex(dev, "usb", HW_USB_PRO,
			(const struct pcanfd_available_clocks *)&sja2010_clocks,
			&sja2010_caps,
			PCAN_DEV_BUSLOAD_RDY);
		break;
	case PCAN_USB_PRODUCT_ID:
	default:
		/* init structure elements to defaults */
		pcan_soft_init(dev, "usb", HW_USB);

		/* Device Id. is a single-octet value in these old adapters,
		 * thus, the 'default' value is 0xff (instead of 0xffffffff) */
		dev->device_alt_num = 0xff;
		break;
	}

	dev->nChannel = ctrl_index;
	dev->adapter = usb_if->adapter;

	/* overrride with USB devices specific options callbacks */
	dev->option = pcan_usb_opts;

	dev->device_open = pcan_usb_device_open;

	/* override standard device access functions:
	 * if device is CANFD capable, set the CANFD open function. Otehrwise,
	 * set the deafult CAN 2.0 open function */
	if (usb_if->device_ctrl_open_fd)
		dev->device_open_fd = pcan_usb_device_open_fd;

	dev->device_write = pcan_usb_write;
	dev->device_release = pcan_usb_device_release;

	/* set this before any instructions, fill struct pcandev, part 1 */
	dev->readreg = NULL;
	dev->writereg = NULL;
	dev->cleanup = pcan_usb_cleanup;
	dev->free_irq = pcan_usb_free_irq;
	dev->open = pcan_usb_open;
	dev->release = pcan_usb_release;
	dev->filter = pcan_create_filter_chain();
	dev->device_params = pcan_usb_device_params;

#if 0//def DEBUG
	printk(KERN_DEBUG "%s: usb hardware revision = %d\n", DEVICE_NAME,
	       usb_if->ucRevision);
#endif
	dev->wInitStep = 1;

	/* assign the device as plugged in */
	dev->ucPhysicallyInstalled = 1;

	/* add this device to the list */
	pcan_add_device_in_list_ex(dev, PCAN_DEV_STATIC);
	dev->wInitStep = 2;

	usb_devices++;
	dev->wInitStep = 3;

	/* MUST do that before any attempt to write something... */
	dev->port.usb.usb_if = usb_if;

	/* get serial number as soon as possible */
	usb_if->device_get_snr(usb_if, &usb_if->dwSerialNumber);

	/* Get device number early too (sometimes, need to retry...) */
	for (retry = 3; retry; retry--) {
		u32 device_nr32;
		err = usb_if->device_ctrl_get_dnr(dev, &device_nr32);
		if (!err) {

			u->ucHardcodedDevNr = (u8 )device_nr32;

#ifdef DEBUG
			pr_info(DEVICE_NAME "%s(): CAN%u devid=%xh (%u)\n",
				__func__, ctrl_index, device_nr32, device_nr32);
#endif
			break;
		}
	}

#ifndef PCAN_USB_DONT_REGISTER_DEV
	/* Handle controller list per interface */
	h = usb_get_intfdata(usb_if->usb_intf);

	dev->nMajor = USB_MAJOR;
	dev->nMinor = -1;

	/* must tell that this interface is not in use for all controllers,
	 * especially for controllers > 0 (kernel>2.6.26) */
	usb_if->usb_intf->minor = -1;
	err = usb_register_dev(usb_if->usb_intf, &pcan_class);
	if (err < 0) {
		pr_err(DEVICE_NAME ": unable to register usb device\n");
		usb_set_intfdata(usb_if->usb_intf, h);
		goto reject;
	}

	dev->nMinor = usb_if->usb_intf->minor;
#else
	dev->nMajor = pcan_drv.nMajor;
	dev->nMinor = pcan_find_free_minor(dev, PCAN_USB_MINOR_BASE,
							PCAN_USB_MINOR_END);
	if (dev->nMinor < 0) {
		err = dev->nMinor;
		pr_err(DEVICE_NAME ": not enough minors\n");
		goto reject;
	}
#endif

#ifdef PCAN_USB_PCAN_SYSFS
	/* do register pcan dev under sysfs */
	pcan_sysfs_dev_node_create(dev);
#else
	dev->sysfs_dev = usb_if->usb_intf->usb_dev;

	pcan_sysfs_add_attrs(dev->sysfs_dev, pcan_usb_sysfs_attrs[ctrl_index]);

	if (usb_if->device_ctrl_open_fd)
		pcan_sysfs_add_attrs(dev->sysfs_dev,
				pcanfd_usb_sysfs_attrs[ctrl_index]);

	if (dev->flags & PCAN_DEV_BUSLOAD_RDY)
		pcan_sysfs_add_attrs(dev->sysfs_dev,
				pcan_usb_sysfs_bus_load_attrs[ctrl_index]);

	if (dev->flags & PCAN_DEV_ERRCNT_RDY)
		pcan_sysfs_add_attrs(dev->sysfs_dev,
				pcan_usb_sysfs_err_cnt_attrs[ctrl_index]);
#endif /* PCAN_USB_PCAN_SYSFS */

	/* set device in inactive state to prevent violating the bus */
	usb_if->device_ctrl_set_bus_off(dev);

	/* Call hardware supplied own callback to do some private init */
	if (usb_if->device_ctrl_init) {
		err = usb_if->device_ctrl_init(dev);
		if (err) {
			pr_err(DEVICE_NAME
				": CAN%u initialization not complete\n",
				ctrl_index+1);
			goto reject;
		}
	}

#ifdef NETDEV_SUPPORT
	pcan_netdev_register(dev);
#endif

	dev->wInitStep = 4;

	printk(KERN_INFO "%s: usb device minor %d found\n",
	       DEVICE_NAME, dev->nMinor);

	return 0;

reject:
	pcan_usb_cleanup(dev);

	pr_err(DEVICE_NAME
		": failed to register %s CAN%u as a new USB CAN channel "
		"err %d\n",
		dev->adapter->name, ctrl_index+1, err);

	return err;
}

#ifdef NETDEV_SUPPORT
static void pcan_usb_plugout_netdev(struct pcandev *dev)
{
	struct net_device *ndev = dev->netdev;

	if (ndev) {
		netif_stop_queue(ndev);
		pcan_netdev_unregister(dev);
	}
}
#endif

static int pcan_usb_plugin(struct usb_interface *interface,
                           const struct usb_device_id *id)
{
	struct usb_device *usb_dev = interface_to_usbdev(interface);
	struct usb_endpoint_descriptor *endpoint;
	struct usb_host_interface *iface_desc;
	struct pcan_usb_interface *usb_if;
	int (*device_init)(struct pcan_usb_interface *);
	int err, i, dev_ctrl_count, sizeof_if;

	DPRINTK(KERN_DEBUG "%s: %s(0x%04x, 0x%04x, 0x%04x)\n",
	        DEVICE_NAME, __func__,
	        usb_dev->descriptor.idVendor, usb_dev->descriptor.idProduct,
	        usb_dev->descriptor.bcdDevice);

	/* check endpoint addresses (numbers) and associated max data length 
	 * (only from setting 0)
	 * Since USB-PRO defines also a LIN interface, should reject it when
	 * adapter plugged: make use of endpoint addresses (no other way...) */
	iface_desc = &interface->altsetting[0];

	DPRINTK(KERN_DEBUG "%s: %s(): bNumEndpoints=%d\n",
	        DEVICE_NAME, __func__, iface_desc->desc.bNumEndpoints);

	for (i=0; i < iface_desc->desc.bNumEndpoints; i++) {
		struct usb_endpoint_descriptor *endpoint =
					&iface_desc->endpoint[i].desc;

		/* Below is the list of valid ep addreses. Any other ep address
		 * is considered as not-CAN interface address => no dev created
		 */
		switch (endpoint->bEndpointAddress) {
		case 0x01:
		case 0x81:
		case 0x02:
		case 0x82:
		case 0x03:
		case 0x83:
			break;
		default:
#ifdef DEBUG
			printk(KERN_INFO
			       "%s: %s(): EP address %02x not in CAN range.\n",
			       DEVICE_NAME, __func__,
			       endpoint->bEndpointAddress);
			printk(KERN_INFO
			       "%s: %s(): ignoring the whole USB interface\n",
			       DEVICE_NAME, __func__);
#endif
			return -ENODEV;
		}
	}

#if 0
	/* Does not work with PCAN-USB FD (and 3.14-rc2 ...)
	 *
	 * TODO: check if we could remove this call, because according to
	 * drivers/usb/core/message.c:
	 *
	 * "Instead, the driver [..] may use usb_set_interface() on the
	 * interface it claims" */
	if (le16_to_cpu(usb_dev->descriptor.idProduct) !=
						PCAN_USBFD_PRODUCT_ID) {

		/* take the 1st configuration (it's default) */
		err = usb_reset_configuration(usb_dev);
		if (err < 0) {
			pr_err(DEVICE_NAME
				": usb_reset_configuration() failed err %d\n",
				err);
			return err;
		}
	}
#endif
	/* only 1 interface is supported
	 * Note: HW_USB_PRO: interface#0 for CAN, #1 for LIN */
	err = usb_set_interface(usb_dev, 0, 0);
	if (err < 0) {
		printk(KERN_ERR "%s: usb_set_interface() failed! (err %d)\n",
		       DEVICE_NAME, err);
		return err;
	}

	/* Now, according to device id, create as many device as CAN
	 * controllers */
	switch (le16_to_cpu(usb_dev->descriptor.idProduct)) {
	case PCAN_USBFD_PRODUCT_ID:
	case PCAN_USBCHIP_PRODUCT_ID:
		dev_ctrl_count = 1;
		device_init = pcan_usbfd_init;
		break;
	case PCAN_USBPROFD_PRODUCT_ID:
		dev_ctrl_count = 2;
		device_init = pcan_usbfd_init;
		break;
#ifdef PCAN_USBX6_PRODUCT_ID
	case PCAN_USBX6_PRODUCT_ID:
		dev_ctrl_count = 2;
		device_init = pcan_usbfd_init;
		break;
#endif
	case PCAN_USBPRO_PRODUCT_ID:
		dev_ctrl_count = 2;
		device_init = pcan_usbpro_init;
		break;
	case PCAN_USB_PRODUCT_ID:
	default:
		dev_ctrl_count = 1;
		device_init = pcan_usb_init;
		break;
	}

	/* create our interface object for the USB device */
	sizeof_if = sizeof(struct pcan_usb_interface) +
					sizeof(struct pcandev) * dev_ctrl_count;

#ifdef DEBUG
	printk(KERN_INFO "%s: new ", DEVICE_NAME);
	if (usb_dev->speed == USB_SPEED_HIGH)
		printk("high speed ");
	printk("usb adapter with %u CAN controller(s) detected\n",
		dev_ctrl_count);
#endif

	usb_if = pcan_malloc(sizeof_if, GFP_KERNEL);
	if (!usb_if) {
		pr_err(DEVICE_NAME
		       ": pcan_malloc(%d) failed!\n", sizeof_if);
		return err;
	}

	memset(usb_if, '\0', sizeof_if);

	/* store pointer to kernel supplied usb_dev */
	usb_if->usb_dev = usb_dev;
	usb_if->usb_intf = interface;

	/* if usb interface is not a root port, then setup a zero based index
	 * so that the exported sysfs channel number will take it into account
	 */
	if (usb_dev->route) {

		/* note: usb port are 1-based numbers
		 * (see drivers/usb/core/usb.c#L472)
		 */
		usb_if->index = usb_dev->portnum - 1;
	}

#ifndef PCAN_USB_CMD_PER_DEV
	/* preset finish flags */
	atomic_set(&usb_if->cmd_sync_complete, 0);
	atomic_set(&usb_if->cmd_async_complete, 1);
#endif
	/* preset active URB counter */
	atomic_set(&usb_if->active_urbs, 0);

	/* get endpoint addresses (numbers) and associated max data length
	 * (only from setting 0) */

/*
 * USB-Pro
 *      Function   Interface   Endpoints            DeviceId
 *      ---------  ---------   -----------------------------------------
 *      Control                0
 *      CAN        0                                "CAN-Device",
 *                                                  USB\VID_0c72&PID_000d&MI_00
 *                             1=Command,           bidi for both CAN_Ctrller
 *                             2=CAN-Controller 0,  rcv (IN) both CAN-Ctrller,
 *                                                  transmit (OUT) CAN-Ctrl#0,
 *                             3=CAN-Controller 1   transmit (OUT) CAN-Ctrl#1
 *      LIN        1                                "LIN-Device",
 *                                                  USB\VID_0c72&PID_000d&MI_01
 *                             4=Command,
 *                             5=Controller 0,
 *                             6=Controller 1
 */
	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		PCAN_ENDPOINT *pipe_addr = NULL;

		endpoint = &iface_desc->endpoint[i].desc;

		DPRINTK(KERN_DEBUG "%s: %s(): EP[%d]={addr=%d max=%d}\n",
		        DEVICE_NAME, __func__, i, endpoint->bEndpointAddress,
		        endpoint->wMaxPacketSize);

		switch (endpoint->bEndpointAddress) {
		case 0x01:
			pipe_addr = &usb_if->pipe_cmd_out;
			break;

		case 0x81:
			pipe_addr = &usb_if->pipe_cmd_in;
			break;

		case 0x02:
			switch (le16_to_cpu(usb_dev->descriptor.idProduct)) {
			case PCAN_USBFD_PRODUCT_ID:
			case PCAN_USBCHIP_PRODUCT_ID:
			case PCAN_USBPROFD_PRODUCT_ID:
#ifdef PCAN_USBX6_PRODUCT_ID
			case PCAN_USBX6_PRODUCT_ID:
#endif
			case PCAN_USBPRO_PRODUCT_ID:
			case PCAN_USB_PRODUCT_ID:
			default:
				pipe_addr = &usb_if->dev[0].port.usb.pipe_write;
				break;
			}
			break;

		case 0x82:
			pipe_addr = &usb_if->pipe_read;
			break;

		case 0x03:
			switch (le16_to_cpu(usb_dev->descriptor.idProduct)) {
#if 0//def HW_USB_FD
			case PCAN_USBFD_PRODUCT_ID:
#warning TODO: check if 0x03 can be enum with PCANFD (dev[1] below is estonishing...)
#endif
			case PCAN_USBPROFD_PRODUCT_ID:
#ifdef PCAN_USBX6_PRODUCT_ID
			case PCAN_USBX6_PRODUCT_ID:
#endif
			case PCAN_USBPRO_PRODUCT_ID:
				pipe_addr = &usb_if->dev[1].port.usb.pipe_write;
				break;
			}

		case 0x83:
			/* Unused pipe for PCAN-USB-PRO
			 * But seems that need to be reset too... */
			/* TBD */
			break;

		default:
			continue;
		}

		if (pipe_addr) {
			pipe_addr->ucNumber = endpoint->bEndpointAddress;
			pipe_addr->wDataSz = le16_to_cpu(endpoint->wMaxPacketSize);
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30)
		usb_reset_endpoint(usb_dev, endpoint->bEndpointAddress);
#endif
	}

	/* ucRevision needs to be defined before allocating resources
	 * (PCAN-USB) */
#if defined(__LITTLE_ENDIAN)
	usb_if->ucHardcodedDevNr =
		(u8)(usb_if->usb_dev->descriptor.bcdDevice & 0xff);
	usb_if->ucRevision = (u8)(usb_if->usb_dev->descriptor.bcdDevice >> 8);
#elif defined(__BIG_ENDIAN)
	usb_if->ucHardcodedDevNr =
		(u8)(usb_if->usb_dev->descriptor.bcdDevice >> 8);
	usb_if->ucRevision = (u8)(usb_if->usb_dev->descriptor.bcdDevice & 0xff);
#else
#error "Please fix the endianness defines in <asm/byteorder.h>"
#endif

	DPRINTK(KERN_DEBUG "%s(): ucHardcodedDevNr=0x%02x ucRevision=0x%02X\n",
		__func__, usb_if->ucHardcodedDevNr, usb_if->ucRevision);

	/* MUST do this BEFORE calling pcan_usb_alloc_resources() */
	usb_if->can_count = dev_ctrl_count;

	/* resources MUST be allocated before calling device_init() */
	err = pcan_usb_alloc_resources(usb_if);
	if (err)
		goto reject;

	/* call initialisation callback for entire device */
	err = device_init(usb_if);
	if (err) {
		pr_err(DEVICE_NAME ": device_init() failure err %d\n", err);
		goto reject_free;
	}

#if 1
	/* install the reception part for the interface */
	if (!atomic_read(&usb_if->read_data.use_count)) {
		FILL_BULK_URB(&usb_if->read_data, usb_if->usb_dev,
		              usb_rcvbulkpipe(usb_if->usb_dev,
		                              usb_if->pipe_read.ucNumber),
		              usb_if->read_buffer_addr[0],
			      usb_if->read_buffer_size,
		              pcan_usb_read_notify, usb_if);

		/* submit urb */
		err = __usb_submit_urb(&usb_if->read_data);
		if (err) {
			pr_err(DEVICE_NAME ": %s() can't submit! (%d)\n",
				__func__, err);
			goto reject_free;
		}

		atomic_inc(&usb_if->active_urbs);
	}
#endif
	/* should be set BEFORE pcan_usb_create_dev() */
	usb_set_intfdata(interface, usb_if);

	/* next, initialize each controller */
	for (i = 0; i < dev_ctrl_count; i++) {

#ifdef PCAN_USB_CMD_PER_DEV
		/* preset finish flags */
		atomic_set(&usb_if->dev[i].port.usb.cmd_sync_complete, 0);
		atomic_set(&usb_if->dev[i].port.usb.cmd_async_complete, 1);
#endif
		err = pcan_usb_create_dev(usb_if, i);
		if (err)
			goto reject_free_all_dev;
	}

#if 0
	/* install the reception part for the interface */
	if (!atomic_read(&usb_if->read_data.use_count)) {
		FILL_BULK_URB(&usb_if->read_data, usb_if->usb_dev,
		              usb_rcvbulkpipe(usb_if->usb_dev,
		                              usb_if->pipe_read.ucNumber),
		              usb_if->read_buffer_addr[0],
			      usb_if->read_buffer_size,
		              pcan_usb_read_notify, usb_if);

		/* submit urb */
		if ((err = __usb_submit_urb(&usb_if->read_data))) {
			printk(KERN_ERR "%s: %s() can't submit! (%d)\n",
		          DEVICE_NAME, __func__, err);
			goto reject;
		}

		atomic_inc(&usb_if->active_urbs);
	}
#endif

	return 0;

reject_free_all_dev:

	/* remove ALL previously created devs for the same USB interface */
 	while (i--) {
		struct pcandev *dev = usb_if->dev + i;
		const int m = dev->nMinor;

#ifdef NETDEV_SUPPORT
		pcan_usb_plugout_netdev(dev);
#endif
#ifdef PCAN_USB_PCAN_SYSFS
		pcan_sysfs_dev_node_destroy(dev);
#endif
		pcan_usb_cleanup(dev);

		pr_info(DEVICE_NAME ": usb device minor %d removed\n", m);
	}

	pcan_kill_sync_urb(&usb_if->read_data);

reject_free:
	pcan_usb_free_resources(usb_if);
reject:
	pcan_free(usb_if);

	return err;
}

/* is called at plug out of device */
static void pcan_usb_plugout(struct usb_interface *interface)
{
	struct pcan_usb_interface *usb_if = usb_get_intfdata(interface);
	struct pcandev *dev;
	int c;

	DPRINTK(KERN_DEBUG "%s: %s(): usb_if=%p\n",
					DEVICE_NAME, __func__, usb_if);
	if (!usb_if) {
		pr_info(DEVICE_NAME "%s(%u): usb_if=NULL\n",
				__func__, __LINE__);
		return;
	}

	/* do it now in case of reenrance somewhere... */
	usb_set_intfdata(interface, NULL);

	dev = usb_if->dev;
	for (c = 0; c < usb_if->can_count; c++, dev++) {
		DPRINTK(KERN_DEBUG "%s: %s(%d)\n",
			        DEVICE_NAME, __func__, dev->nMinor);

#ifdef PCAN_USB_PCAN_SYSFS
		pcan_sysfs_dev_node_destroy(dev);
#else
		if (dev->flags & PCAN_DEV_ERRCNT_RDY)
			pcan_sysfs_del_attrs(dev->sysfs_dev,
					pcan_usb_sysfs_err_cnt_attrs[c]);

		if (dev->flags & PCAN_DEV_BUSLOAD_RDY)
			pcan_sysfs_del_attrs(dev->sysfs_dev,
					pcan_usb_sysfs_bus_load_attrs[c]);

		if (usb_if->device_ctrl_open_fd)
			pcan_sysfs_del_attrs(dev->sysfs_dev,
					pcanfd_usb_sysfs_attrs[c]);

		pcan_sysfs_del_attrs(dev->sysfs_dev,
					pcan_usb_sysfs_attrs[c]);
#endif

#ifdef NETDEV_SUPPORT
		pcan_usb_plugout_netdev(dev);
#endif

		/* mark this device as plugged out */
		dev->ucPhysicallyInstalled = 0;

#if 0
		/* do not remove resources if the device is still in use */
		if (!dev->nOpenPaths)
			pcan_usb_cleanup(dev);
#else
		/* Should close all dev resources EVEN if the device is in use,
		 * otherwise application may not be noticed that the device was
		 * removed: CAN_Open(); while (1) CAN_Read(h); */
		pcan_usb_cleanup(dev);
#endif

#ifdef PCAN_USB_CMD_PER_DEV
		pcan_kill_sync_urb(&dev->port.usb.urb_cmd_sync);
		pcan_kill_sync_urb(&dev->port.usb.urb_cmd_async);
#endif
		pcan_kill_sync_urb(&dev->port.usb.write_data);

		pcan_free(dev->port.usb.cout_baddr);

#ifndef PCAN_USB_DONT_REGISTER_DEV
		interface->minor = dev->nMinor;
		usb_deregister_dev(interface, &pcan_class);
#endif
	}

#ifndef PCAN_USB_CMD_PER_DEV
	pcan_kill_sync_urb(&usb_if->urb_cmd_async);
#endif
	pcan_kill_sync_urb(&usb_if->urb_cmd_sync);
	pcan_kill_sync_urb(&usb_if->read_data);

	pcan_free(usb_if->read_buffer_addr[0]);

	if (usb_if->device_free)
		usb_if->device_free(usb_if);

	usb_reset_device(usb_if->usb_dev);

	pcan_free(usb_if);
}

/* small interface to rest of driver, only init and deinit */
static int pcan_usb_core_init(void)
{
	DPRINTK(KERN_DEBUG
		"%s: %s() -------------------------------------------\n",
	        DEVICE_NAME, __func__);

	memset (&pcan_drv.usbdrv, 0, sizeof(pcan_drv.usbdrv));

	/* do inherit default options */
	pcan_inherit_options(pcan_usb_opts);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,24) && LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
	pcan_drv.usbdrv.owner = THIS_MODULE;
#endif

	pcan_drv.usbdrv.probe = pcan_usb_plugin;
	pcan_drv.usbdrv.disconnect = pcan_usb_plugout;
	pcan_drv.usbdrv.name = DEVICE_NAME;
	pcan_drv.usbdrv.id_table = pcan_usb_ids;

	return usb_register(&pcan_drv.usbdrv);
}

static int pcan_usb_do_cleanup(struct device *dev, void *arg)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct pcan_usb_interface *usb_if = \
			(struct pcan_usb_interface *)usb_get_intfdata(intf);
	struct pcandev *pdev;
	int c;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	if (!usb_if) {
		DPRINTK(KERN_DEBUG "%s: %s(): NULL usb_if\n",
						DEVICE_NAME, __func__);
		return 0;
	}

	/* Browse controllers list */
	pdev = usb_if->dev;
	for (c = 0; c < usb_if->can_count; c++, pdev++)
		if (pdev->ucPhysicallyInstalled)

			/* Last chance for URB submitting */
			if (usb_if->device_ctrl_cleanup)
				usb_if->device_ctrl_cleanup(pdev);

	return 0;
}

void pcan_usb_deinit(void)
{
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	if (pcan_drv.usbdrv.probe == pcan_usb_plugin) {

		/* Added this since it is the last chance for URB submitting */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
		int err = driver_for_each_device(
					&pcan_drv.usbdrv.drvwrap.driver,
					NULL, NULL, pcan_usb_do_cleanup);
#else
		int err = driver_for_each_device(&pcan_drv.usbdrv.driver,
					NULL, NULL, pcan_usb_do_cleanup);
#endif

		/* driver_for_each_device() is declared with "must_check"
		 * attribute so check err here, knowing that drv is not NULL
		 * (1st arg) and that pcan_usb_do_cleanup() always return 0 */
		if (err)
			err = 0;

		/* then it was registered
		 * unregister usb parts, makes a plugout of registered devices
		 */
		usb_deregister(&pcan_drv.usbdrv);
	}
}

/* init for usb based devices from peak */
int pcan_usb_register_devices(void)
{
	int err;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	if (!(err = pcan_usb_core_init())) {
		DPRINTK(KERN_DEBUG "%s: %s() is OK\n", DEVICE_NAME, __func__);
	}

	return err;
}
#endif /* USB_SUPPORT */
