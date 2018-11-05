/*****************************************************************************
 * Copyright (C) 2014 PEAK System-Technik GmbH
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Maintainer(s): Stephane Grosjean (s.grosjean@peak-system.com)
 *
 *****************************************************************************/

/*****************************************************************************
 *
 * pcan_usbfd.c - the inner parts for PCAN-USB (Pro) FD support
 *
 * $Id: pcan_usbfd_fd.c 615 2011-02-10 22:38:55Z stephane $
 *
 *****************************************************************************/
/* #define DEBUG */
/* #undef DEBUG */

#include "src/pcan_common.h"

#ifdef USB_SUPPORT

#include "src/pcan_fifo.h"
#include "src/pcanfd_usb.h"

#ifdef NETDEV_SUPPORT
#include "src/pcan_netdev.h"		/* for hotplug pcan_netdev_register() */
#else
#include <linux/can/dev.h>
#endif

#include "src/pcan_timing.h"		/* timing conversion */
#include "src/pcanfd_usb_fw.h"		/* PCAN-USB (Pro) FD fw structures */
#include "src/pcan_usbpro_fw.h"

/* if defined, tell how timestamp are handled */
//#define DEBUG_TIMESTAMP

/* if defined, tell how incoming messages and records are handled */
/*#define DEBUG_DECODE */

#define UCAN_USB_PRECISION_MASK_BITS	11
/* 0x7ff */
#define UCAN_USB_PRECISION_MASK		((1 << UCAN_USB_PRECISION_MASK_BITS)-1)

#define UCAN_USB_CMD_TIMEOUT		1000	/* ms to for USB requests */
#define TICKS(msec)			((msec * HZ) / 1000)

/* device state flags */
#define UCAN_USB_SHOULD_WAKEUP		0x00000001UL

/*
 * Private Data Structures
 */
struct pcan_usbfd_fw_info {
	u16	size_of;	/* sizeof this */
	u16	type;		/* type of this structure */
	u8	hw_type;	/* Type of hardware (HW_TYPE_xxx) */
	u8	bl_version[3];	/* Bootloader version */
	u8	hw_version;	/* Hardware version (PCB) */
	u8	fw_version[3];	/* Firmware version */
	__le32	dev_id[2];	/* "device id" per CAN */
	__le32	ser_no;		/* S/N */
	u32	flags;		/* special functions */
} __attribute__ ((packed));

#ifdef UCAN_USB_OPTION_FAST_FWD
/* enable to globally set the fast-forward option for PCAN-USB FD adapters */
static ushort fast_fwd = 0;
module_param(fast_fwd, ushort, 0444);
#endif

static int pcan_usbfd_devices = 0;
static int pcan_usbprofd_devices = 0;
static int pcan_usbx6_devices = 0;

/* Use several things from PCAN-USB Pro */
extern void pcan_usbpro_driver_loaded(struct pcan_usb_interface *usb_if,
					int can_lin, int loaded);
extern int pcan_usbpro_request(struct pcan_usb_interface *usb_if,
				int req_id, int req_value,
				void *req_addr, int req_size);

extern u32 pcan_usbpro_handle_response_rtt(struct pcan_usb_interface *usb_if);

#if 0
extern void pcan_usbpro_timestamp_decode(struct pcan_usb_interface *usb_if,
					u32 ts_us, struct timeval *tv);

extern void pcan_usbpro_time_sync(struct pcan_usb_interface *usb_if,
			struct timeval *tv, u32 ts_us, u32 dev_frame_index);
#endif

/*
 * URB status (LDD3 p339):
 * 0
 * -ENOENT(2)			The URB was stopped by call to usb_kill_urb()
 * -EOVERFLOW(75)		Too large packet
 * -EINPROGRESS(115)		The URB is always being processed by device
 */

/*
 * static void pcan_usbfd_submit_cmd_end(struct urb *purb,
 *							struct pt_regs *regs)
 *
 * Called when URB has been submitted to hardware
 */
static void pcan_usbfd_submit_cmd_end(struct urb *purb, struct pt_regs *regs)
{
	struct pcandev *dev = purb->context;
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	struct pcan_usb_port *u = &dev->port.usb;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u) = %d\n",
		 DEVICE_NAME, __func__, dev->nChannel+1, purb->status);

	/* un-register outstanding urb */
	atomic_dec(&usb_if->active_urbs);

	atomic_set(&u->cmd_sync_complete, 1);
}

/* static int pcan_usbfd_send_ucan_cmd(struct pcandev *dev)
 */
static int pcan_usbfd_send_ucan_cmd(struct pcandev *dev)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	struct pcan_usb_port *up = &dev->port.usb;
	struct urb *urb;
	u32 ms_timeout;
	int err = 0;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u): ->EP#%02X\n",
		DEVICE_NAME, __func__, dev->nChannel+1,
		 usb_if->pipe_cmd_out.ucNumber);

	/* don't do anything with non-existent hardware */
	if (!dev->ucPhysicallyInstalled)
		return -ENODEV;

	urb = &up->urb_cmd_sync;

	/* if a packet is not filled completely by commands, the command list
	 * is terminated with an "end of collection" record. */
	if (dev->ucan.cmd_len < dev->ucan.cmd_size) {
#if 1
		ucan_add_cmd(dev, UCAN_CMD_END_OF_COLLECTION);
#else
		void *cmd = ucan_add_cmd(dev, UCAN_CMD_END_OF_COLLECTION);
		memset(cmd, 0xff, sizeof(u64));
#endif
	}

#ifdef DEBUG
	//dump_mem("sent cmd", dev->ucan.cmd_head, 32); //dev->ucan.cmd_len);
	dump_mem("sent cmd", dev->ucan.cmd_head, dev->ucan.cmd_len);
#endif
	/* firmware is not able to re-assemble 512 bytes buffer in full-speed */
	if ((usb_if->usb_dev->speed != USB_SPEED_HIGH) &&
						(dev->ucan.cmd_len > 64)) {
		printk(KERN_ERR "%s: Warning: too large cmd (%dB) to be sent "
			"(cmd discarded!)\n",
			DEVICE_NAME, dev->ucan.cmd_len);
		return -ENOSPC;
	}

	FILL_BULK_URB(urb, usb_if->usb_dev,
			usb_sndbulkpipe(usb_if->usb_dev,
					usb_if->pipe_cmd_out.ucNumber),
			dev->ucan.cmd_head, dev->ucan.cmd_len,
			pcan_usbfd_submit_cmd_end, dev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
	urb->timeout = TICKS(UCAN_USB_CMD_TIMEOUT);
#endif

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err) {
		printk(KERN_ERR "%s: %s(): usb_submit_urb() failure (err %d)\n",
			DEVICE_NAME, __func__, err);
		goto fail;
	}

	atomic_inc(&usb_if->active_urbs);

	/* wait until submit is finished, either normal or thru timeout */
	ms_timeout = get_mtime() + UCAN_USB_CMD_TIMEOUT;
	while (!atomic_read(&up->cmd_sync_complete)) {
		schedule();

		if (get_mtime() >= ms_timeout) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
			usb_kill_urb(urb);
#else
			usb_unlink_urb(urb);
#endif
			break;
		}
	}

	atomic_set(&up->cmd_sync_complete, 0);

	err = urb->status;
fail:
	return err;
}

/*
 * Hardware Callbacks
 */

/* int ucan_usb_set_can_led(struct pcandev *dev, u8 mode )
 */
static int ucan_usb_set_can_led(struct pcandev *dev, u8 mode)
{
	struct ucan_usb_led *cmd;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u)\n", DEVICE_NAME, __func__,
		dev->nChannel+1);

	cmd = ucan_add_cmd(ucan_init_cmd(dev), UCAN_USB_CMD_LED_SET);
	if (cmd)
		cmd->mode = mode;

	/* send the command */
	return pcan_usbfd_send_ucan_cmd(dev);
}

/* int ucan_usb_set_en_option(struct pcandev *dev, u16 mask, u16 usb_mask)
 */
static int ucan_usb_set_en_option(struct pcandev *dev, u16 mask, u16 usb_mask)
{
	struct ucan_usb_option *cmd;

	cmd = ucan_add_cmd_set_en_option(ucan_init_cmd(dev), mask);
	if (cmd)
		cmd->usb_mask = cpu_to_le16(usb_mask);

	/* send the command */
	return pcan_usbfd_send_ucan_cmd(dev);
}

/* int ucan_usb_clr_dis_option(struct pcandev *dev, u16 mask, u16 usb_mask)
 */
static int ucan_usb_clr_dis_option(struct pcandev *dev, u16 mask, u16 usb_mask)
{
	struct ucan_usb_option *cmd;

	cmd = ucan_add_cmd_clr_dis_option(ucan_init_cmd(dev), mask);
	if (cmd)
		cmd->usb_mask = cpu_to_le16(usb_mask);

	/* send the command */
	return pcan_usbfd_send_ucan_cmd(dev);
}

/* int pcan_usbfd_get_fw_info(struct pcandev *dev, int should_print)
 */
static int pcan_usbfd_get_fw_info(struct pcan_usb_interface *usb_if,
				  int should_print)
{
	struct pcan_usbfd_fw_info * const pfi = \
			(struct pcan_usbfd_fw_info *)usb_if->dev->ucan.cmd_head;

	int err = pcan_usbpro_request(usb_if,
				USB_VENDOR_REQUEST_INFO,
				USB_VENDOR_REQUEST_wVALUE_INFO_FIRMWARE,
				pfi, sizeof(*pfi));
	if (err >= 0) {

		int i;

		if (should_print)
			pr_info(DEVICE_NAME
				": %s (%02xh PCB%02Xh) "
				"fw v%d.%d.%d "
				"bl v%d.%d.%d\n",
				usb_if->adapter->name,
				pfi->hw_type, pfi->hw_version,
				pfi->fw_version[0], pfi->fw_version[1],
				pfi->fw_version[2],
				pfi->bl_version[0], pfi->bl_version[1],
				pfi->bl_version[2]);

		/* save device id and serial num info for further read */
		usb_if->dwSerialNumber = le32_to_cpu(pfi->ser_no);
		usb_if->ucRevision = pfi->hw_version;

		usb_if->adapter->hw_ver_major = pfi->fw_version[0];
		usb_if->adapter->hw_ver_minor = pfi->fw_version[1];
		usb_if->adapter->hw_ver_subminor = pfi->fw_version[2];

		for (i = 0; i < usb_if->can_count; i++)
			usb_if->dev[i].port.usb.ucHardcodedDevNr =
				le32_to_cpu(pfi->dev_id[i]);

		return 0;
	}

	return  err;
}

/*
 * int pcan_usbfd_get_device_nr(struct pcandev *dev, u32 *p_device_nr)
 */
static int pcan_usbfd_get_device_nr(struct pcandev *dev, u32 *p_device_nr)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	struct pcan_usbfd_fw_info * const pfi = \
				(struct pcan_usbfd_fw_info *)dev->ucan.cmd_head;
	int err = pcan_usbpro_request(usb_if,
				USB_VENDOR_REQUEST_INFO,
				USB_VENDOR_REQUEST_wVALUE_INFO_FIRMWARE,
				pfi, sizeof(*pfi));
	if (!err) {

		u32 tmp32 = le32_to_cpu(pfi->dev_id[dev->nChannel]);

		if (tmp32 != 0xffffffff) {
			dev->device_alt_num = tmp32;
			dev->flags |= PCAN_DEV_USES_ALT_NUM;
		}
		if (p_device_nr)
			*p_device_nr = tmp32;
	}

	return err;
}

/*
 * int pcan_usbfd_set_device_nr(struct pcandev *dev, u32 device_nr)
 */
static int pcan_usbfd_set_device_nr(struct pcandev *dev, u32 device_nr)
{
	struct ucan_usb_device_id *cmd;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u, 0x%x)\n",
		 DEVICE_NAME, __func__, dev->nChannel+1, device_nr);

	cmd = ucan_add_cmd(ucan_init_cmd(dev), UCAN_USB_CMD_DEVID_SET);
	if (cmd)
		cmd->device_id = cpu_to_le32(device_nr);

	/* send the command */
	return pcan_usbfd_send_ucan_cmd(dev);
}

/*
 * int pcan_usbfd_get_serial_nr(struct pcan_usb_interface *usb_if,
 *					u32 *p_serial_num)
 *
 * Retrieve serial number from bootloader info
 */
static int pcan_usbfd_get_serial_nr(struct pcan_usb_interface *usb_if,
					u32 *p_serial_num)
{
#if 0
	int err;

	err = pcan_usbfd_get_fw_info(usb_if, 0);
	if (!err) {
		if (p_serial_num)
			*p_serial_num = usb_if->dwSerialNumber;
	}

	return err;
#else
	/* used cached value read from pcan_usbfd_init() */
	if (p_serial_num)
		*p_serial_num = usb_if->dwSerialNumber;

	return 0;
#endif
}

/* int ucan_usb_set_clck_domain(struct pcandev *dev, u32 clk_mode)
 */
static int ucan_usb_set_clck_domain(struct pcandev *dev, u32 clk_mode)
{
	struct ucan_usb_clock *cmd;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u, 0x%x)\n",
		 DEVICE_NAME, __func__, dev->nChannel+1, clk_mode);

	cmd = ucan_add_cmd(ucan_init_cmd(dev), UCAN_USB_CMD_CLK_SET);
	if (cmd)
		cmd->mode = (u8 )clk_mode;

	/* send the command */
	return pcan_usbfd_send_ucan_cmd(dev);
}

/*
 * static int pcan_usbfd_set_clk_domain(struct pcandev *dev,
 *						struct pcanfd_init *pfdi)
 */
static int pcan_usbfd_set_clk_domain(struct pcandev *dev,
						struct pcanfd_init *pfdi)
{
	/* select the clock for the CAN */
	switch (pfdi->clock_Hz) {
	case 20000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_20MHZ);
		break;
	case 24000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_24MHZ);
		break;
	case 30000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_30MHZ);
		break;
	case 40000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_40MHZ);
		break;
	case 60000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_60MHZ);
		break;
	case 80000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_80MHZ);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int pcan_usbfd_open_complete(struct pcan_usb_interface *usb_if,
				    struct pcandev *dev,
				    struct pcanfd_init *pfdi,
				    int err)
{
	u16 opt_mask = UCAN_OPTION_ERROR, usb_mask = 0;

	if (err) {
		pr_err("%s: failed to open %s CAN%u: err %d\n",
				DEVICE_NAME, dev->adapter->name,
				dev->nChannel+1, err);
		goto fail;
	}

#if 0
pr_info(DEVICE_NAME ": %s(): opened_count=%d\n", __func__, usb_if->opened_count);

	/* Note: if 'opened_count == 0' then this is the first device
	 * opened, ask for being notified of calibration msgs */
	if (usb_if->opened_count < 1)
#endif
		usb_mask |= UCAN_USB_OPTION_CALIBRATION;

#ifdef UCAN_USB_OPTION_FAST_FWD
	/* setup fast-forward option */
	if (fast_fwd) {
		pr_info("%s: fast-forward option set for %s CAN%u channel\n",
				DEVICE_NAME, dev->adapter->name,
				dev->nChannel+1);
		usb_mask |= UCAN_USB_OPTION_FAST_FWD;
	} else {
		/* be sure to clear any fast-forward option */
		ucan_usb_clr_dis_option(dev, 0, UCAN_USB_OPTION_FAST_FWD);
	}
#endif

	if (pfdi) {
		if (pfdi->flags & PCANFD_INIT_BUS_LOAD_INFO)
			opt_mask |= UCAN_OPTION_BUSLOAD;
	}

	if (!(opt_mask & UCAN_OPTION_BUSLOAD)) {
		ucan_usb_clr_dis_option(dev, UCAN_OPTION_BUSLOAD, 0);
	}

	/* ask for being notified of error messages */
	err = ucan_usb_set_en_option(dev, opt_mask, usb_mask);

fail:
	return err;
}

/* int pcan_usbfd_open_fd(struct pcandev *dev, struct pcanfd_init *pfdi)
 */
static int pcan_usbfd_open_fd(struct pcandev *dev, struct pcanfd_init *pfdi)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	int err;

#if defined(DEBUG) || defined(DEBUG_IRQ)
	pr_info("%s: %s(CAN%u, clk=%u Hz)\n",
		DEVICE_NAME, __func__, dev->nChannel+1, pfdi->clock_Hz);
#endif

	err = ucan_device_open_fd(dev, pfdi);

	return pcan_usbfd_open_complete(usb_if, dev, pfdi, err);
}

/* int pcan_usbfd_open(struct pcandev *dev, u16 btr0btr1, u8 listen_only)
 */
static int pcan_usbfd_open(struct pcandev *dev, u16 btr0btr1,
				u8 ext, u8 listen_only)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	int err;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u, btr0btr1=0x%04x, listen_only=%d)\n",
		 DEVICE_NAME, __func__, dev->nChannel+1, btr0btr1,
		listen_only);

	/* set (default) clock domain */
	ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_80MHZ);

	err = ucan_device_open(dev, btr0btr1, ext, listen_only);

	return pcan_usbfd_open_complete(usb_if, dev, NULL, err);
}

/* int pcan_usbfd_close(struct pcandev *dev)
 */
static int pcan_usbfd_close(struct pcandev *dev)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	u16 usb_mask = 0;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u): opened_count=%d\n",
		DEVICE_NAME, __func__,
		dev->nChannel+1, usb_if->opened_count);

	/* Note: when 'opened_count == 1' then this is the last device
	 * opened. We can clear anything regarding the whole interface */
	if (usb_if->opened_count <= 1) {

		/* now we can reset sync for the next time for all devices */
		usb_if->dev[0].time_sync.ts_us = 0;
		usb_mask |= UCAN_USB_OPTION_CALIBRATION;
	}

#ifdef UCAN_USB_OPTION_FAST_FWD
	/* clear fast-forward option */
	if (fast_fwd)
		usb_mask |= UCAN_USB_OPTION_FAST_FWD;
#endif

	/* turn off notifications */
	ucan_usb_clr_dis_option(dev,
			UCAN_OPTION_ERROR|UCAN_OPTION_BUSLOAD,
			usb_mask);

	return 0;
}

/* int pcan_usbfd_timestamp_decode(struct pcandev *dev, u32 ts_low,
 *						u32 ts_high, struct timeval *tv)
 */
static struct timeval *pcan_usbfd_timestamp_decode(struct pcandev *dev,
						u32 ts_low, u32 ts_high,
						struct timeval *tv)
{
#if 0
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);

	pcan_usbpro_timestamp_decode(usb_if, ts_low, tv);
#else
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);

	/* do nothing (that is, set host time rather than hw timestamps in msgs
	 * while we don't have receive the 1st calibration msg */
	if (usb_if->dev[0].time_sync.ts_us) {

		/* since calibration msgs donot use ts_high (always 0), then
		 * we must ignore ts_high read from the record too, and
		 * simulate our own ts_high by handling ts_low wrapping.
		 * Note that calibration msgs ts_low is always <= ts_low read
		 * from the record. */
		ts_high = usb_if->ts_high;
		if (ts_low < usb_if->ts_low) {

			/* if ts_low is lower than sync ts_low, it's maybe
			 * because:
			 * - ts counter has wrapped or
			 * - event has been delayed because of sync
			 * if ts counter has wrapped, then this ts_low 
			 * shouldn't be greater than the sync period */
			if (ts_low < USEC_PER_SEC) {
#if 1
				/* because the driver might receive event(s)
				 * AFTER having received the calibration msg
				 * *BUT* with a timestamp LOWER than the
				 * ts of the calibration msg, we must
				 * consider wrapping only if event ts_low is
				 * REALLY lower than calibration ts_low:
				 * the counter has wrapped if NEXT calibration
				 * ts will wrapp too */
				u32 next_ts_low = usb_if->ts_low + USEC_PER_SEC;
				if (next_ts_low < usb_if->ts_low) {
#endif

					ts_high++;

#ifdef DEBUG
					pr_info(DEVICE_NAME
						": device ts counter wrapped: "
						" %u:%u -> %u.%u\n",
						usb_if->ts_high, usb_if->ts_low,
						ts_high, ts_low);
#endif
				}
			}
		}	
	}	

#if 0
	pr_info(DEVICE_NAME
		": %s(ts_low=%d): usb_if[ts_high=%d ts_low=%d] => ts_high=%d\n",
		__func__, ts_low, usb_if->ts_high, usb_if->ts_low, ts_high);
#endif
	/* since time_sync.ts_us is not 0, then pcan_sync_decode() succeeds */
	return pcan_sync_decode(dev, ts_low, ts_high, tv) ? tv : NULL;
#endif
}

/* handle uCAN Rx CAN message */
static int pcan_usbfd_decode_canrx(struct ucan_engine *ucan,
					struct ucan_msg *rx_msg, void *arg)
{
	struct ucan_rx_msg *rm = (struct ucan_rx_msg *)rx_msg;
	struct pcandev *dev = (struct pcandev *)arg;
	struct timeval tv;
	int err;

	err = ucan_post_canrx_msg(dev, rm, 
			pcan_usbfd_timestamp_decode(dev,
						    le32_to_cpu(rm->ts_low),
						    le32_to_cpu(rm->ts_high),
						    &tv));
	if (err > 0)
		dev->port.usb.state |= UCAN_USB_SHOULD_WAKEUP;
	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	return err;
}

/* handle uCAN error message */
static int pcan_usbfd_decode_error(struct ucan_engine *ucan,
					 struct ucan_msg *rx_msg, void *arg)
{
	struct ucan_error_msg *er = (struct ucan_error_msg *)rx_msg;
	struct pcandev *dev = (struct pcandev *)arg;
	struct timeval tv;
	int err;

	err = ucan_post_error_msg(dev, er,
			pcan_usbfd_timestamp_decode(dev,
						    le32_to_cpu(er->ts_low),
						    le32_to_cpu(er->ts_high),
						    &tv));
	if (err > 0)
		dev->port.usb.state |= UCAN_USB_SHOULD_WAKEUP;
	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	return err;
}

/* handle uCAN bus_load message */
static int pcan_usbfd_decode_bus_load(struct ucan_engine *ucan,
					 struct ucan_msg *rx_msg, void *arg)
{
	struct ucan_bus_load_msg *bl = (struct ucan_bus_load_msg *)rx_msg;
	struct pcandev *dev = (struct pcandev *)arg;
	struct timeval tv;
	int err;

#if 0
	DPRINTK(KERN_DEBUG
		"%s: got bus_load msg: ts=0x%08x-%08x\n",
		DEVICE_NAME,
		le32_to_cpu(bl->ts_low), le32_to_cpu(bl->ts_high));
#endif
	err = ucan_post_bus_load_msg(dev, bl,
			pcan_usbfd_timestamp_decode(dev,
						    le32_to_cpu(bl->ts_low),
						    le32_to_cpu(bl->ts_high),
						    &tv));
	if (err > 0)
		dev->port.usb.state |= UCAN_USB_SHOULD_WAKEUP;
	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	return err;
}

/* handle uCAN status message */
static int pcan_usbfd_decode_status(struct ucan_engine *ucan,
					 struct ucan_msg *rx_msg, void *arg)
{
	struct ucan_status_msg *st = (struct ucan_status_msg *)rx_msg;
	struct pcandev *dev = (struct pcandev *)arg;
	struct timeval tv;
	int err;

	DPRINTK(KERN_DEBUG
		"%s: got status msg: ts=0x%08x-%08x EP=%u EW=%u BO=%u\n",
		DEVICE_NAME,
		le32_to_cpu(st->ts_low), le32_to_cpu(st->ts_high),
		!!UCAN_STMSG_PASSIVE(st), !!UCAN_STMSG_WARNING(st), 
		!!UCAN_STMSG_BUSOFF(st));

	err = ucan_post_status_msg(dev, st,
			pcan_usbfd_timestamp_decode(dev,
						    le32_to_cpu(st->ts_low),
						    le32_to_cpu(st->ts_high),
						    &tv));
	if (err > 0)
		dev->port.usb.state |= UCAN_USB_SHOULD_WAKEUP;
	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	/* bus is ok: if tx_engine idle, set it to STOPPED so that user will 
	 * initiate writing on it */
	if ((dev->bus_state != PCANFD_ERROR_BUSOFF) &&
#ifdef PCAN_USES_OLD_TX_ENGINE_STATE
			(atomic_read(&dev->tx_engine_state) == TX_ENGINE_IDLE))
#else
			(dev->locked_tx_engine_state == TX_ENGINE_IDLE))
#endif
		pcan_set_tx_engine(dev, TX_ENGINE_STOPPED);

	return err;
}

/* handle uCAN USB overrun message */
static int pcan_usbfd_decode_overrun(struct ucan_engine *ucan,
					struct ucan_msg *rx_msg, void *arg)
{
	struct ucan_usb_ovr_msg *ov = (struct ucan_usb_ovr_msg *)rx_msg;
	struct pcandev *dev = (struct pcandev *)arg;
	struct timeval tv;
	int err;

	DPRINTK(KERN_DEBUG
		"%s: got overrun msg: ts=0x%08x-%08x\n",
		DEVICE_NAME,
		le32_to_cpu(ov->ts_low), le32_to_cpu(ov->ts_high));

	err = ucan_post_overflow_msg(dev,
			pcan_usbfd_timestamp_decode(dev,
						    le32_to_cpu(ov->ts_low),
						    le32_to_cpu(ov->ts_high),
						    &tv));
	if (err > 0)
		dev->port.usb.state |= UCAN_USB_SHOULD_WAKEUP;
	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	return err;
}

/* handle USB calibration message */
static int pcan_usbfd_decode_ts(struct ucan_engine *ucan,
					struct ucan_msg *rx_msg, void *arg)
{
	struct ucan_usb_ts_msg *ts = (struct ucan_usb_ts_msg *)rx_msg;
	struct pcan_usb_interface *usb_if = (void *)arg;
	const u16 raw_frame_index = le16_to_cpu(ts->usb_frame_index);
	int dev_frame_index = raw_frame_index & UCAN_USB_PRECISION_MASK;
	u32 ts_low = le32_to_cpu(ts->ts_low);
	int d;

#ifdef DEBUG_TIMESTAMP
	printk(KERN_DEBUG
		"%s: calibration: ts=%-10u fr11=%-4u fr=%-5u fi=%d\n",
		DEVICE_NAME,
		//le32_to_cpu(ts->ts_high),	/* always 0 */
		ts_low,
		dev_frame_index,
		raw_frame_index,
		usb_if->frame_index);
#endif
	/* should wait until clock is stabilized */
	if (usb_if->cm_ignore_count > 0) {
		usb_if->cm_ignore_count--;
		return 0;
	}

#if 0
	/* use this message to compute RTT (here time diff between tv_request &
	 * tv_response) before doing synchronization
	 */
	pcan_usbpro_handle_response_rtt(usb_if);

	dev_frame_index = raw_frame_index & UCAN_USB_PRECISION_MASK;

	pcan_usbpro_time_sync(usb_if, &usb_if->tv_response,
				 le32_to_cpu(ts->ts_low), dev_frame_index);
#else

	/* get usb frame index as clock ref */
	if (usb_if->frame_index < 0) {
		usb_if->frame_index = dev_frame_index;
		return 0;
	}

	d = dev_frame_index - usb_if->frame_index;
	if (d < 0)
		d += (1 << 11); //UCAN_USB_PRECISION_MASK);

	/* knowing that d should be ~1000 */

	usb_if->frame_index = dev_frame_index;

	/* frame_index is a ms. clock: it contains a ms. counter which value
	 * is supposed to be fix (ex: 1000).
	 * Thus, this value is used for clocking host time */

	/* ts_high always 0, so we have to handle it by ourselves: */
	if (ts_low < usb_if->ts_low)
		usb_if->ts_high++;

	usb_if->ts_low = ts_low;

#if 0
	pr_info(DEVICE_NAME ": %s(ts_low=%d): usb_if[ts_high=%d ts_low=%d]\n",
		__func__, ts_low, usb_if->ts_high, usb_if->ts_low);
#endif
	/* handle time sync in dev[0] even if it is not opened */
	pcan_sync_times(&usb_if->dev[0], ts_low, usb_if->ts_high,
			d * USEC_PER_MSEC);

	/* do a copy of time_sync object for ALL channels devices */
	for (d = 1; d < usb_if->can_count; d++)
		memcpy(&usb_if->dev[d].time_sync, &usb_if->dev[0].time_sync,
			sizeof(usb_if->dev[0].time_sync));
#endif
	return 0;
}

/* int pcan_usbfd_msg_decode(struct pcan_usb_interface *usb_if,
 *				u8 *msg_addr, int msg_len)
 *
 * Decode a message received from PCAN-USB (Pro) FD
 */
static int pcan_usbfd_msg_decode(struct pcan_usb_interface *usb_if,
					u8 *msg_addr, int msg_len)
{
	int err = 0, d;

#if 0
	DPRINTK(KERN_DEBUG "%s: %s(%d)\n", DEVICE_NAME, __func__, msg_len);
	dump_mem("received msg", msg_addr, msg_len);
#endif
	/* do some init for each controller */
	for (d = 0; d < usb_if->can_count; d++)
		usb_if->dev[d].port.usb.state &= ~UCAN_USB_SHOULD_WAKEUP;

	/* call default uCAN rx CAN messages handler */
	err = ucan_handle_msgs_buffer(&usb_if->dev[0].ucan, msg_addr, msg_len);
	if (err < 0)
		if (err != -ENOSPC)
			dump_mem("received msg", msg_addr, msg_len);

	/* check if something is to be woken up */
	for (d = 0; d < usb_if->can_count; d++)
		if (usb_if->dev[d].port.usb.state & UCAN_USB_SHOULD_WAKEUP) {

			usb_if->dev[d].dwInterruptCounter++;
#ifdef DEBUG_DECODE
			printk(KERN_INFO "wakeup task reading CAN%u\n", d+1);
#endif
			pcan_event_signal(&usb_if->dev[d].in_event);
		}

	return err;
}

/*
 * void pcan_usbfd_cleanup(struct pcandev *dev)
 *
 * Last chance to submit URB before driver removal.
 */
static void pcan_usbfd_cleanup(struct pcandev *dev)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u)\n",
		 DEVICE_NAME, __func__, dev->nChannel+1);

	/* Sometimes, bus off request can't be submit when driver is removed
	 * so, when the device was not properly closed. So, move the bus off
	 * request here to be sure it is sent.  */
	ucan_set_bus_off(dev);

#ifdef USB_VENDOR_REQUEST_wVALUE_SETFKT_INTERFACE_DRIVER_LOADED
	/* No more need to switch off the LEDs by ourselves!
	 * Fw does it when we notify it from driver unload!  */
#else
	/* Switch LED off */
	ucan_usb_set_can_led(dev, UCAN_USB_LED_OFF);
#endif

	/* If last device, tell module that driver is unloaded */
	if (dev->nChannel == (usb_if->can_count-1)) {

		/* Tell module the CAN driver is unloaded */
		pcan_usbpro_driver_loaded(usb_if, 0, 0);
	}
}

/*
 * int pcan_usbfd_ctrl_init(struct pcandev *dev)
 *
 * Do CAN controller specific initialization.
 */
static int pcan_usbfd_ctrl_init(struct pcandev *dev)
{
	const struct pcan_usb_interface *usb_if = dev->port.usb.usb_if;
	const int c = dev->nChannel + (usb_if->index * usb_if->can_count) + 1;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u)\n", DEVICE_NAME, __func__, c);

	/* setup things at probe time */
	//ucan_device_probe(dev);

	pr_info("%s: %s channel %d device number=%u\n",
		DEVICE_NAME, dev->adapter->name, c, dev->device_alt_num);

#if 1
	/* this has been moved at open time */
#else
	/* set 80MHz clock domain */
	ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_80MHZ);
#endif
	/* set LED in default state (end of init phase) */
	ucan_usb_set_can_led(dev, UCAN_USB_LED_DEF);

	/* sending data is not allowed for the moment */

	/* can do this here because no isr is runing at the moment, and only
	 * one process can do this (actually, the 1st one) */
	pcan_set_tx_engine(dev, TX_ENGINE_IDLE);

	return 0;
}

/*
 * void pcan_usbfd_free(struct pcan_usb_interface *usb_if)
 */
static void pcan_usbfd_free(struct pcan_usb_interface *usb_if)
{
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	/* release dynamic memory ony once */
	switch (le16_to_cpu(usb_if->usb_dev->descriptor.idProduct)) {
	case PCAN_USBX6_PRODUCT_ID:
		if (usb_if->usb_dev->portnum < 3)
			break;
	default:
		pcan_free(usb_if->adapter);
		break;
	}
}

/* interface functions used to send commands / handle msgs to USB/uCAN */
static int (*pcan_usbfd_ucan_handlers[])(struct ucan_engine *,
					 struct ucan_msg *,
					 void *) = {
	[UCAN_MSG_CAN_RX] = pcan_usbfd_decode_canrx,
#if 0
	/* this handler "only" keeps a copy of rx/tx errors counters BUT don't
	 * push any msgs into the rx queue */
	[UCAN_MSG_ERROR] = ucan_handle_error,
	[UCAN_MSG_BUSLOAD] = ucan_handle_bus_load,
#else
	/* use this one if rx/tx errors counters are pushed into rx queue
	 * as STATUS messages */
	[UCAN_MSG_ERROR] = pcan_usbfd_decode_error,
	[UCAN_MSG_BUSLOAD] = pcan_usbfd_decode_bus_load,
#endif
	[UCAN_MSG_STATUS] = pcan_usbfd_decode_status,

	[UCAN_USB_MSG_CALIBRATION] = pcan_usbfd_decode_ts,
	[UCAN_USB_MSG_OVERRUN] = pcan_usbfd_decode_overrun,
};

static struct ucan_ops pcan_usbfd_ucan_ops = {
	.set_clk_domain = pcan_usbfd_set_clk_domain,
	.send_cmd = pcan_usbfd_send_ucan_cmd,
	.handle_msg_table = pcan_usbfd_ucan_handlers,
	.handle_msg_size = ARRAY_SIZE(pcan_usbfd_ucan_handlers),
};

static struct pcan_usb_interface *
		pcan_usbfd_get_same_if(struct pcan_usb_interface *usb_if)
{
	struct pcan_usb_interface *usb_dev_if = NULL;
	struct pcandev *dev;
	struct list_head *ptr;

#ifdef HANDLE_HOTPLUG
	pcan_mutex_lock(&pcan_drv.devices_lock);
#endif

	list_for_each(ptr, &pcan_drv.devices) {

		dev = (struct pcandev *)ptr;
		if (dev->wType != HW_USB_X6)
			goto lbl_continue;

		usb_dev_if = pcan_usb_get_if(dev);
		if (usb_dev_if->usb_dev->bus->busnum !=
					usb_if->usb_dev->bus->busnum)
			goto lbl_continue;

		/* see drivers/usb/core/usb.c usb_alloc_dev():
		 *
		 * if devices share the same bus, and the same parent, so they
		 * share the same interface.
		 *
		 * if devices share the same bus and the same port, so they
		 * share the same interface.
		 */
		if (usb_dev_if->usb_dev->route) {
			if (usb_dev_if->usb_dev->parent->portnum ==
					usb_if->usb_dev->parent->portnum)
				break;
		} else {
			if (usb_dev_if->usb_dev->portnum ==
					usb_if->usb_dev->portnum)
				break;
		}
lbl_continue:
		usb_dev_if = NULL;
	}

#ifdef HANDLE_HOTPLUG
	pcan_mutex_unlock(&pcan_drv.devices_lock);
#endif
	return usb_dev_if;
}

/*
 * int pcan_usbfd_init(struct pcan_usb_interface *usb_if)
 *
 * Do device specific initialization.
 */
int pcan_usbfd_init(struct pcan_usb_interface *usb_if)
{
	struct pcandev *dev;
	int c, should_print = 1;

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(bus=%d port=%d parent_port=%d)\n",
			__func__,
			usb_if->usb_dev->bus->busnum,
			usb_if->usb_dev->portnum,
			usb_if->usb_dev->parent->portnum);
#endif

	switch (le16_to_cpu(usb_if->usb_dev->descriptor.idProduct)) {
	case PCAN_USBPROFD_PRODUCT_ID:
		usb_if->adapter = pcan_alloc_adapter("PCAN-USB Pro FD",
						pcan_usbprofd_devices++,
						usb_if->can_count);
		break;

	case PCAN_USBX6_PRODUCT_ID:
		if (usb_if->usb_dev->portnum == 1) {
			usb_if->adapter = pcan_alloc_adapter("PCAN-USB X6",
						pcan_usbx6_devices++,
						6);
		} else {
			struct pcan_usb_interface *same_if =
					pcan_usbfd_get_same_if(usb_if);

			if (!same_if) 
				pr_err(DEVICE_NAME
					": unable to find same usb if\n");
			else {
				usb_if->adapter = same_if->adapter;
				should_print = 0;
			}
		}

		break;

	case PCAN_USBFD_PRODUCT_ID:
		usb_if->adapter = pcan_alloc_adapter("PCAN-USB FD",
						pcan_usbfd_devices++,
						usb_if->can_count);
		break;

	case PCAN_USBCHIP_PRODUCT_ID:
		usb_if->adapter = pcan_alloc_adapter("PCAN-USB Chip",
						pcan_usbfd_devices++,
						usb_if->can_count);
		break;
	}

	if (!usb_if->adapter)
		return -ENOMEM;

	/* Set PCAN-USB (Pro) FD hardware specific callbacks */
	usb_if->device_ctrl_init = pcan_usbfd_ctrl_init;
	usb_if->device_get_snr = pcan_usbfd_get_serial_nr;
	usb_if->device_msg_decode = pcan_usbfd_msg_decode;
	usb_if->device_free = pcan_usbfd_free;

	usb_if->device_ctrl_cleanup = pcan_usbfd_cleanup;
	usb_if->device_ctrl_open = pcan_usbfd_open;
	usb_if->device_ctrl_open_fd = pcan_usbfd_open_fd;
	usb_if->device_ctrl_close = pcan_usbfd_close;
	usb_if->device_ctrl_set_bus_on = ucan_set_bus_on;
	usb_if->device_ctrl_set_bus_off = ucan_set_bus_off;
	usb_if->device_ctrl_set_dnr = pcan_usbfd_set_device_nr;
	usb_if->device_ctrl_get_dnr = pcan_usbfd_get_device_nr;
	usb_if->device_ctrl_msg_encode = ucan_encode_msgs_buffer;

	for (dev = &usb_if->dev[c=0]; c < usb_if->can_count; c++, dev++) {
#if 1
		/* SGr: device_alt_num is now initialized to 0xffffffff, in
		 * pcan_soft_init(). *BUT* pcan_soft_init() is called a bit
		 * later, and after this function. This should not be a problem.
		 */
#else
		/* MUST initialize alt_num here (before creating devices),
		 * for Udev rules */
		dev->device_alt_num = 0xffffffff;
#endif
		/* set uCAN interface function to send cmds and handle msgs */
		dev->ucan.ops = &pcan_usbfd_ucan_ops;

		/* remember the list of channels in each channel */
		dev->ucan.devs = usb_if->dev;
		dev->ucan.devs_count = usb_if->can_count;

		/* use the allocated commands buffer for building uCAN cmds */
		dev->ucan.cmd_head = dev->port.usb.cout_baddr;
		dev->ucan.cmd_size = dev->port.usb.cout_bsize;
	}

	usb_if->cm_ignore_count = 0;
	usb_if->frame_index = -1;

	usb_if->ts_high = 0;
	usb_if->ts_low = 0;

	/* Tell module the CAN driver is loaded */
	pcan_usbpro_driver_loaded(usb_if, 0, 1);

	/* read fw info */
	pcan_usbfd_get_fw_info(usb_if, should_print);

	return 0;
}
#endif /* USB_SUPPORT */
