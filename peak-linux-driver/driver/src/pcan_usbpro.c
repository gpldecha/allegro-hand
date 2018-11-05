/*****************************************************************************
 * Copyright (C) 2003-2011  PEAK System-Technik GmbH
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
 * Maintainer(s): Stephane Grosjean (s.grosjean@peak-system.com)
 *
 * Major contributions by:
 *                Klaus Hitschler (klaus.hitschler@gmx.de)
 *                Oliver Hartkopp (oliver.hartkopp@volkswagen.de) socketCAN
 *
 *****************************************************************************/

/*****************************************************************************
 *
 * pcan_usbpro.c - the inner parts for pcan-usb-pro support
 *
 * $Id: pcan_usbpro.c 615 2011-02-10 22:38:55Z stephane $
 *
 * 2011-08-22 SGr
 * - pcan_usbpro_cleanup(): add the sending of the bus off request in cleanup
 *   callback since this is the last place we may submit urb to the device
 *   before driver removal. Also remove led off request and let fw handle them.
 * - also move calibration messages management to controller init/cleanup
 *   callbacks to be sure they are sent.
 * - tell usb-pro that the driver is loaded/unloaded in order to let the fw
 *   handle the leds by itself
 *
 * 2011-10-10 SGr
 * - error messages better support
 * - handle big/little endian support
 * - handle eventual record fragmentation (doesnot occur w/ 1024 packets)
 *
 *****************************************************************************/
/* #define DEBUG */
/* #undef DEBUG */

#include "src/pcan_common.h"

#ifdef USB_SUPPORT

#include <linux/sched.h>
#include "src/pcan_main.h"
#include "src/pcan_fifo.h"
#include "src/pcan_usbpro.h"

#ifdef NETDEV_SUPPORT
#include "src/pcan_netdev.h"     /* for hotplug pcan_netdev_register() */
#endif

#include "src/pcan_usbpro_fw.h"

#ifdef DEBUG
/* if defined, tell how timestamp are handled */
#define PCAN_DEBUG_TIMESTAMP
/* if defined, tell how incoming messages and records are handled */
#define PCAN_DEBUG_DECODE
#define PCAN_DEBUG_BITTIMING
#endif

#define PCAN_USBPRO_GET_BUSLOAD
//#define PCAN_USBPRO_USE_UFRAMES

#define PCAN_USBPRO_SYSCLK_HZ		(56 * MHz)

/* period in s. when calibtaion message are requested from USB-PRO
 * if not defined, only one calibration message is requested but none 
 * periodically */
//#define PCAN_USBPRO_DRIVER_CALIBRATION_PERIOD 30

#ifndef PCAN_USBPRO_USE_UFRAMES
#define PCAN_USBPRO_PRECISION_MASK	0x7ff
#else
#define PCAN_USBPRO_PRECISION_MASK	0x3fff
#endif

#define PCAN_USBPRO_CMD_TIMEOUT		1000  // ms timeout for USB requests
#define TICKS(msec)			((msec * HZ) / 1000)

#define PCAN_USBPRO_RTR			0x01
#define PCAN_USBPRO_EXT			0x02

#define PCAN_USBPRO_SR			0x80

/* PCAN-USB-PRO status flags */
#define PCAN_USBPRO_BUS_HEAVY		0x01
//#define PCAN_USBPRO_BUS_OFF		0x02
#define PCAN_USBPRO_BUS_OVERRUN		0x0c

/* interface private flags */
#define PCAN_USBPRO_WAIT_FOR_CALIBRATION_MSG	0x00000001UL

/* device state flags */
#define PCAN_USBPRO_SHOULD_WAKEUP	0x00000001UL
#define PCAN_USBPRO_BUS_OFF		0x00000002UL

/* pcan_usbpro_cmd_send_ex() flags */
#define PCAN_USBPRO_SEND_SYNC_MODE	0x00000001
#define PCAN_USBPRO_SEND_GET_TOD	0x00000002

/*
 * Private Data Structures
 */

/* Internal structure used to handle messages sent to bulk urb */

#define PCAN_USB_PRO_CMD_BUFFER_SIZE	512

struct pcan_usbpro_msg {
	u8 *	rec_ptr;
	int	rec_buffer_size;
	int	rec_buffer_len;
	union {
		u16 * rec_counter_read;
		u32 * rec_counter;
		u8 *  rec_buffer;
	} u;
};

const struct pcanfd_bittiming_range sja2010_caps = {

	.brp_min = 1,
	.brp_max = 1024,
	.brp_inc = 1,

	.tseg1_min = 1,		/* pcan_timing.c constant for v <= 7.13 */
	.tseg1_max = 16,
	.tseg2_min = 1,		/* pcan_timing.c constant for v <= 7.13 */
	.tseg2_max = 8,

	.sjw_min = 1,
	.sjw_max = 4,		/* Windows driver says 5 here */
};

const pcanfd_mono_clock_device sja2010_clocks = {
	.count = 1,
	.list = {
		[0] = { .clock_Hz = 56*MHz, .clock_src = 56*MHz, },
	}
};

static int pcan_usbpro_devices = 0;

/*
 * Private Functions
 */
#define pcan_usbpro_cmd_send(a,b)\
	        pcan_usbpro_cmd_send_ex(a,b,PCAN_USBPRO_SEND_SYNC_MODE)
#define pcan_usbpro_cmd_send_async(a,b)\
	        pcan_usbpro_cmd_send_ex(a,b,0)

#define pcan_usbpro_calibration_request(a,b)\
	        pcan_usbpro_calibration_request_ex(a,b,1)
#define pcan_usbpro_calibration_request_async(a,b)\
	        pcan_usbpro_calibration_request_ex(a,b,0)

/*
 * u32 pcan_timeval_delta_us(struct timeval *tv0, struct timeval *tv1)
 *
 * do tv1 - tv0, return number of usec
 * Note: tv1 is supposed to be more recent than tv0.
 *       if numerical value tv1 < tv0, supposing 2^32 wrapping
 */
u32 pcan_timeval_delta_us(struct timeval *tv0, struct timeval *tv1)
{
	u32 s, u;

	s = (tv0->tv_sec <= tv1->tv_sec) ? tv1->tv_sec - tv0->tv_sec :
	                               0xffffffff - tv0->tv_sec + tv1->tv_sec;

	if (tv0->tv_usec <= tv1->tv_usec)
		u = tv1->tv_usec - tv0->tv_usec;
	else {
		u = USEC_PER_SEC - tv0->tv_usec + tv1->tv_usec;
		s--;
	}

	return USEC_PER_SEC*s + u;
}

#ifdef PCAN_USBPRO_DRIVER_CALIBRATION_PERIOD

static int pcan_usbpro_calibration_request_ex(struct pcan_usb_interface *,
	                                           u16 onoff, int sync_mode);
/*
 * static void pcan_usbpro_calibration_timeout(unsigned long arg)
 */
static void pcan_usbpro_calibration_timeout(unsigned long arg)
{
	struct pcan_usb_interface *usb_if = (struct pcan_usb_interface *)arg;
	int err;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	/* activate calibration messaging, asynchronous mode only */
	/* (schedule() forbidden in timer routine) */
	if (pcan_usbpro_calibration_request_async(usb_if, 1) == -EWOULDBLOCK) {
		DPRINTK(KERN_DEBUG
			"%s:  can't request calibration for the moment\n",
		        DEVICE_NAME);

		/* restart timer to try later */
		mod_timer(&usb_if->calibration_timer,
		          jiffies + PCAN_USBPRO_DRIVER_CALIBRATION_PERIOD * HZ);
	}
}
#endif

/*
 * static int pcan_usbpro_sizeof_rec(u8 data_type)
 */
static int pcan_usbpro_sizeof_rec(u8 data_type)
{
	switch (data_type)
	{
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_8:
		return sizeof(struct pcan_usbpro_canmsg_rx);
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_4:
		return sizeof(struct pcan_usbpro_canmsg_rx) - 4;
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_0:
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RTR_RX:
		return sizeof(struct pcan_usbpro_canmsg_rx) - 8;

	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_STATUS_ERROR_RX:
		return sizeof(struct pcan_usbpro_canmsg_status_error_rx);

	case DATA_TYPE_USB2CAN_STRUCT_CALIBRATION_TIMESTAMP_RX:
		return sizeof(struct pcan_usbpro_calibration_ts_rx);

	case DATA_TYPE_USB2CAN_STRUCT_BUSLAST_RX:
		return sizeof(struct pcan_usbpro_buslast_rx);

	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_8:
		return sizeof(struct pcan_usbpro_canmsg_tx);
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_4:
		return sizeof(struct pcan_usbpro_canmsg_tx) - 4;
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_0:
		return sizeof(struct pcan_usbpro_canmsg_tx) - 8;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETBAUDRATE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETBAUDRATE:
		return sizeof(struct pcan_usbpro_baudrate);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUSACTIVATE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETCANBUSACTIVATE:
		return sizeof(struct pcan_usbpro_bus_activity);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETSILENTMODE:
		return sizeof(struct pcan_usbpro_silent_mode);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETDEVICENR:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR:
		return sizeof(struct pcan_usbpro_dev_nr);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETWARNINGLIMIT:
		return sizeof(struct pcan_usbpro_warning_limit);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETLOOKUP_EXPLICIT:
		return sizeof(struct pcan_usbpro_lookup_explicit);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETLOOKUP_GROUP:
		return sizeof(struct pcan_usbpro_lookup_group);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETFILTERMODE:
		return sizeof(struct pcan_usbpro_filter_mode);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETRESET_MODE:
		return sizeof(struct pcan_usbpro_reset_mode);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETERRORFRAME:
		return sizeof(struct pcan_usbpro_error_frame);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUS_ERROR_STATUS:
		return sizeof(struct pcan_usbpro_error_status);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETREGISTER:
		return sizeof(struct pcan_usbpro_set_register);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETREGISTER:
		return sizeof(struct pcan_usbpro_get_register);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_CALIBRATION_MSG:
		return sizeof(struct pcan_usbpro_calibration);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_BUSLAST_MSG:
		return sizeof(struct pcan_usbpro_buslast);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETSTRING:
		return sizeof(struct pcan_usbpro_set_string);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETSTRING:
		return sizeof(struct pcan_usbpro_get_string);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_STRING:
		return sizeof(struct pcan_usbpro_string);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SAVEEEPROM:
		return sizeof(struct pcan_usbpro_save_eeprom);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_USB_IN_PACKET_DELAY:
		return sizeof(struct pcan_usbpro_packet_delay);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_TIMESTAMP_PARAM:
		return sizeof(struct pcan_usbpro_timestamp_param);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_ERROR_GEN_ID:
		return sizeof(struct pcan_usbpro_error_gen_id);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_ERROR_GEN_NOW:
		return sizeof(struct pcan_usbpro_error_gen_now);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_SOFTFILER:
		return sizeof(struct pcan_usbpro_softfiler);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_CANLED:
		return sizeof(struct pcan_usbpro_set_can_led);

	default:
		printk(KERN_INFO "%s: %s(%d): unsupported data type\n",
		       DEVICE_NAME, __func__, data_type);
	}

	return -1;
}

/*
 * static u8 * pcan_usbpro_msg_init(struct pcan_usbpro_msg *pm,
 *                                       void *buffer_addr, int buffer_size)
 *
 * Initialize PCAN USB-PRO message data structure
 */
static u8 * pcan_usbpro_msg_init(struct pcan_usbpro_msg *pm,
				 void *buffer_addr, int buffer_size)
{
	if (buffer_size < 4)
		return NULL;

	pm->u.rec_buffer = (u8 *)buffer_addr;
	pm->rec_buffer_size = pm->rec_buffer_len = buffer_size;
	pm->rec_ptr = pm->u.rec_buffer + 4;

	return pm->rec_ptr;
}

static u8 * pcan_usbpro_msg_init_empty(struct pcan_usbpro_msg *pm,
				       void *buffer_addr, int buffer_size)
{
	u8 *pr = pcan_usbpro_msg_init(pm, buffer_addr, buffer_size);
	if (pr) {
		pm->rec_buffer_len = 4;
		*pm->u.rec_counter = 0;
	}
	return pr;
}

/*
 * static int pcan_usbpro_msg_add_rec(struct pcan_usbpro_msg *pm,
 *                                    int id, ...)
 *
 * Add one record to a message being built.
 */
static int pcan_usbpro_msg_add_rec(struct pcan_usbpro_msg *pm,
	                                int id, ...)
{
	int l, i;
	u8 *pc;
	va_list ap;

	va_start(ap, id);

	pc = pm->rec_ptr + 1;

	i = 0;
	switch (id) {
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_8:
		i += 4;
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_4:
		i += 4;
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_0:
		*pc++ = (u8 )va_arg(ap, int);	// client
		*pc++ = (u8 )va_arg(ap, int);	// flags
		*pc++ = (u8 )va_arg(ap, int);	// len
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32));  // id
		pc += 4;
		memcpy(pc, va_arg(ap, int *), i);
		pc += i;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETBAUDRATE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUSACTIVATE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUS_ERROR_STATUS:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETBAUDRATE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETDEVICENR:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		pc += 2; // dummy
		/* CCBT, devicenr */
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32));
		pc += 4;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETCANBUSACTIVATE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETSILENTMODE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETWARNINGLIMIT:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETFILTERMODE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETRESET_MODE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETERRORFRAME:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_TIMESTAMP_PARAM:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_ERROR_GEN_NOW:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		/* onoff, silentmode, warninglimit, filter_mode, reset, mode, */
		/* start_or_end, bit_pos */
		*(u16 *)pc = cpu_to_le16((u16 )va_arg(ap, int));
		pc += 2;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETLOOKUP_EXPLICIT:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_CANLED:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		*(u16 *)pc = cpu_to_le16((u16 )va_arg(ap, int)); // id_type,mode
		pc += 2;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // id, timeout
		pc += 4;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETLOOKUP_GROUP:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		*(u16 *)pc = cpu_to_le16((u16 )va_arg(ap, int)); // id_type
		pc += 2;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // id_start
		pc += 4;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // id_end
		pc += 4;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETREGISTER:
		*pc++ = (u8 )va_arg(ap, int);	// irq_off
		pc += 2; // dummy
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // address
		pc += 4;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // value
		pc += 4;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // mask
		pc += 4;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETREGISTER:
		*pc++ = (u8 )va_arg(ap, int);	// irq_off
		pc += 2; // dummy
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // address
		pc += 4;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // value
		pc += 4;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_CALIBRATION_MSG:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_USB_IN_PACKET_DELAY:
		pc++; // dummy
		/* mode, delay */
		*(u16 *)pc = cpu_to_le16((u16 )va_arg(ap, int)); // mode
		pc += 2;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_BUSLAST_MSG:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		pc++; // dummy
		*pc++ = (u8 )va_arg(ap, int);	// mode
		//*(u16 *)pc = cpu_to_le16((u16 )va_arg(ap, int)); // prescaler
		pc += 2; // prescale (readonly)
		//*(u16 *)pc = cpu_to_le32((u16 )va_arg(ap, int)); // sampletimequanta
		*(u16 *)pc = cpu_to_le16(4096); // sampletimequanta
		pc += 2;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETSTRING:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		*pc++ = (u8 )va_arg(ap, int);	// offset
		*pc++ = (u8 )va_arg(ap, int);	// len
		memcpy(pc, va_arg(ap, u8 *), 60);
		pc += 60;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETSTRING:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SAVEEEPROM:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		pc += 2; // dummy
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_STRING:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		pc += 2; // dummy
		memcpy(pc, va_arg(ap, u8 *), 250);
		pc += 250;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_ERROR_GEN_ID:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		*(u16 *)pc = cpu_to_le16((u16 )va_arg(ap, int)); // bit_pos
		pc += 2;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // id
		pc += 4;
		*(u16 *)pc = cpu_to_le16((u16 )va_arg(ap, int)); // ok_counter
		pc += 2;
		*(u16 *)pc = cpu_to_le16((u16 )va_arg(ap, int)); //error_counter
		pc += 2;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_SOFTFILER:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		pc += 2;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // accmask
		pc += 4;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // acccode
		pc += 4;
		break;

#if 0
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_CANLED:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		*(u16 *)pc =  (u16 )va_arg(ap, int); // mode
		pc += 2;
		*(u32 *)pc =  va_arg(ap, u32); // timeout
		pc += 4;
		break;
#endif

	default:
		printk(KERN_INFO "%s: %s(): unknown data type %02Xh (%d)\n",
		       DEVICE_NAME, __func__, id, id);
		pc--;
		break;
	}

	l = pc - pm->rec_ptr;
	if (l > 0) {
		*pm->u.rec_counter = cpu_to_le32(*pm->u.rec_counter+1);
		*(pm->rec_ptr) = (u8 )id;

		pm->rec_ptr = pc;
		pm->rec_buffer_len += l;
	}

	va_end(ap);

	return l;
}

/* URB status (LDD3 p339):
 * 0
 * -ENOENT(2)                The URB was stopped by call to usb_kill_urb()
 * -EOVERFLOW(75)            Too large packet
 * -EINPROGRESS(115)         The URB is always being processed by device
 */
/*
 * static void pcan_usbpro_submit_cmd_complete(struct urb *purb,
 *                                             struct pt_regs *regs)
 *
 * Called when URB has been submitted to hardware
 */
#ifdef LINUX_26
static void pcan_usbpro_submit_cmd_complete(struct urb *purb,
	                                         struct pt_regs *regs)
#else
static void pcan_usbpro_submit_cmd_complete(purb_t purb)
#endif
{
#ifdef PCAN_USB_CMD_PER_DEV
	struct pcandev *dev = purb->context;
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	struct pcan_usb_port *u = &dev->port.usb;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u) = %d\n",
	        DEVICE_NAME, __func__, dev->nChannel+1, purb->status);
#else
	struct pcan_usb_interface *usb_if = purb->context;

	DPRINTK(KERN_DEBUG "%s: %s() = %d\n",
	        DEVICE_NAME, __func__, purb->status);
#endif

	/* un-register outstanding urb */
	atomic_dec(&usb_if->active_urbs);

#ifdef PCAN_USB_CMD_PER_DEV
	if (purb == &u->urb_cmd_async)
		atomic_set(&u->cmd_async_complete, 1);
	else
		atomic_set(&u->cmd_sync_complete, 1);
#else
	if (purb == &usb_if->urb_cmd_async)
		atomic_set(&usb_if->cmd_async_complete, 1);
	else
		atomic_set(&usb_if->cmd_sync_complete, 1);
#endif
}

#ifdef PCAN_USB_CMD_PER_DEV
/*
 * static int pcan_usbpro_cmd_send(struct pcandev *dev,
 *                                 struct pcan_usbpro_msg *pum)
 */
static int pcan_usbpro_cmd_send_ex(struct pcandev *dev,
	                                struct pcan_usbpro_msg *pum,
	                                u32 flags)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	struct pcan_usb_port *up = &dev->port.usb;
	struct urb *urb;
	u32 ms_timeout;
	int err = 0;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u, flags=%u): ->EP#%02X\n",
		DEVICE_NAME, __func__, dev->nChannel+1, flags,
		usb_if->pipe_cmd_out.ucNumber);

	/* don't do anything with non-existent hardware */
	if (!dev->ucPhysicallyInstalled)
		return -ENODEV;

	if (!(flags & PCAN_USBPRO_SEND_SYNC_MODE)) {

		if (!atomic_read(&up->cmd_async_complete)) {
			err = -EWOULDBLOCK;
			goto fail;
		}

		urb = &up->urb_cmd_async;
	} else {

		urb = &up->urb_cmd_sync;
	}

#ifdef DEBUG
	dump_mem("sent message", &pum->u.rec_buffer[0], pum->rec_buffer_len);
#endif

	FILL_BULK_URB(urb, usb_if->usb_dev,
	              usb_sndbulkpipe(usb_if->usb_dev,
	                              usb_if->pipe_cmd_out.ucNumber),
	              &pum->u.rec_buffer[0], pum->rec_buffer_len,
	              pcan_usbpro_submit_cmd_complete, dev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
	urb->timeout = TICKS(PCAN_USBPRO_CMD_TIMEOUT);
#endif

	if (flags & PCAN_USBPRO_SEND_GET_TOD)
		pcan_gettimeofday(&usb_if->tv_request);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err) {
		printk(KERN_ERR "%s: %s(): usb_submit_urb() failure (err %d)\n",
		       DEVICE_NAME, __func__, err);
		goto fail;
	}

	atomic_inc(&usb_if->active_urbs);

	/* check if we need to wait for completion */
	if (!(flags & PCAN_USBPRO_SEND_SYNC_MODE))
		return 0;

	/* wait until submit is finished, either normal or thru timeout */
	ms_timeout = get_mtime() + PCAN_USBPRO_CMD_TIMEOUT;
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

	/* reset to 0 for next command */
	atomic_set(&up->cmd_sync_complete, 0);

	err = urb->status;

fail:

#ifdef DEBUG
	if (err)
		pr_debug("%s: %s(CAN%u): failed to send sync cmd (err %d)\n",
			DEVICE_NAME, __func__, dev->nChannel+1, err);
#endif
	return err;
}

#else
/*
 * static int pcan_usbpro_cmd_send(struct pcan_usb_interface *usb_if,
 *                                 struct pcan_usbpro_msg *pum)
 */
static int pcan_usbpro_cmd_send_ex(struct pcan_usb_interface *usb_if,
	                                struct pcan_usbpro_msg *pum,
	                                u32 flags)
{
	int err = 0;
	struct urb *urb;
	u32 ms_timeout;

	DPRINTK(KERN_DEBUG "%s: %s(): ->EP#%02X\n", DEVICE_NAME, __func__,
	        usb_if->pipe_cmd_out.ucNumber);

	// don't do anything with non-existent hardware
	if (!usb_if->dev[0].ucPhysicallyInstalled)
		return -ENODEV;

	if (!(flags & PCAN_USBPRO_SEND_SYNC_MODE)) {

		if (!atomic_read(&usb_if->cmd_async_complete))
			return -EWOULDBLOCK;

		urb = &usb_if->urb_cmd_async;
	} else {

		if (!atomic_read(&usb_if->cmd_sync_complete))
			return -EWOULDBLOCK;

		urb = &usb_if->urb_cmd_sync;
	}

#ifdef DEBUG
	dump_mem("sent message", &pum->u.rec_buffer[0], pum->rec_buffer_len);
#endif

	FILL_BULK_URB(urb, usb_if->usb_dev,
	              usb_sndbulkpipe(usb_if->usb_dev,
	                              usb_if->pipe_cmd_out.ucNumber),
	              &pum->u.rec_buffer[0], pum->rec_buffer_len,
	              pcan_usbpro_submit_cmd_complete, usb_if);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
	urb->timeout = TICKS(PCAN_USBPRO_CMD_TIMEOUT);
#endif

	if (flags & PCAN_USBPRO_SEND_GET_TOD)
		pcan_gettimeofday(&usb_if->tv_request);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err) {
		printk(KERN_ERR "%s: %s(): usb_submit_urb() failure error %d\n",
		       DEVICE_NAME, __func__, err);
		goto fail;
	}

	atomic_inc(&usb_if->active_urbs);

	/* check if we need to wait for completion */
	if (!(flags & PCAN_USBPRO_SEND_SYNC_MODE))
		return 0;

	// wait until submit is finished, either normal or thru timeout
	ms_timeout = get_mtime() + PCAN_USBPRO_CMD_TIMEOUT;
	while (!atomic_read(&usb_if->cmd_sync_complete)) {
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

fail:
	err = urb->status;

	atomic_set(&usb_if->cmd_sync_complete, 0);

	return err;
}
#endif

#ifdef PCAN_USB_CMD_PER_DEV
#ifdef LINUX_26
static void pcan_usbpro_read_complete(struct urb *purb, struct pt_regs *regs)
#else
static void pcan_usbpro_read_complete(purb_t purb)
#endif
{
	struct pcan_usb_interface *usb_if = purb->context;

	DPRINTK(KERN_DEBUG "%s: %s() = %d\n",
	        DEVICE_NAME, __func__, purb->status);

	/* un-register outstanding urb */
	atomic_set(&usb_if->cmd_sync_complete, 1);
}
#endif

/*
 * static int pcan_usbpro_read_rec(struct pcan_usb_interface *usb_if,
 *                                 void *pv, int maxlen)
 *
 * Wait for a record given in 'pv'.
 * Returns first one read if pv->data_type = 0;
 */
static int pcan_usbpro_read_rec(struct pcan_usb_interface *usb_if,
	                             void *pv, int maxlen)
{
	int err = 0, resubmit = 1;
	u32 ms_timeout;
	register purb_t urb = &usb_if->urb_cmd_sync;
	u8 tmp[PCAN_USB_PRO_CMD_BUFFER_SIZE];

	DPRINTK(KERN_DEBUG "%s: %s(): <-EP#%02X\n", DEVICE_NAME, __func__,
	        usb_if->pipe_cmd_in.ucNumber);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
	urb->timeout = TICKS(PCAN_USBPRO_CMD_TIMEOUT);
#endif

	ms_timeout = get_mtime() + PCAN_USBPRO_CMD_TIMEOUT;
	for (;;) {
		if (resubmit) {
			/* if not using actual length, should use
			*(u32 *)&tmp[0] = 0;
			  instead
			 */

			FILL_BULK_URB(urb, usb_if->usb_dev,
		                 usb_rcvbulkpipe(usb_if->usb_dev,
			                          usb_if->pipe_cmd_in.ucNumber),
		                 &tmp[0], sizeof(tmp),
#ifdef PCAN_USB_CMD_PER_DEV
		                 pcan_usbpro_read_complete,
#else
		                 pcan_usbpro_submit_cmd_complete,
#endif
				 usb_if);

			if (__usb_submit_urb(urb)) {
				printk(KERN_ERR
					"%s: %s() __usb_submit_urb() failure\n",
				      DEVICE_NAME, __func__);
				break;
			}

			atomic_inc(&usb_if->active_urbs);

			resubmit = 0;
		}

		/* Wait for callback to tell us that is finished */
		if (atomic_read(&usb_if->cmd_sync_complete)) {
			int r, rec_counter;
			u8 *pc = &tmp[0];

#ifdef DEBUG
			dump_mem("received message",
					&tmp[0], urb->actual_length);
#endif

			/* Sometimes, device wakes us up with 0 length message... */
			if (urb->actual_length > 0) {
				/* Should read into tmp if record is present.
				 * If not, loop again... */
				rec_counter = le32_to_cpu(*(u32 *)pc);

				/* This rec_counter sometimes contains other
				 * info in HIWORD */
				rec_counter &= 0xffff;

				pc += 4;

				DPRINTK(KERN_DEBUG
					"%s: %s(): message contains %d record(s)\n",
					DEVICE_NAME, __func__, rec_counter);

				for (r=0; r < rec_counter; r++) {
					int found_rec = 0;
					union pcan_usbpro_rec *pr = (union pcan_usbpro_rec *)pc;
					union pcan_usbpro_rec *ps;

					int rec_size = pcan_usbpro_sizeof_rec(pr->data_type);
					if (rec_size <= 0) {
						printk(KERN_INFO "%s: %s(): unable to read record in "
						       "response\n", DEVICE_NAME, __func__);

						dump_mem("unknown record",
							pc, urb->actual_length);
						err = -EBADMSG;
						goto fail;
					}

					ps = (union pcan_usbpro_rec *)pv;
					if (ps->data_type != 0) {
						if (pr->data_type == ps->data_type) {
							switch (pr->data_type) {
							case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUSACTIVATE:
							case DATA_TYPE_USB2CAN_STRUCT_FKT_GETBAUDRATE:
							case DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR:
								found_rec = (pr->bus_activity.channel == \
								                              ps->bus_activity.channel);
								break;
							default:
								DPRINTK(KERN_DEBUG "%s: %s(): no other criteria defined"
								        " for data type %02Xh: considering response"
								        " received\n",
								        DEVICE_NAME, __func__, pr->data_type);
								found_rec = 1;
								break;
							}
						}
					}
					else found_rec = 1;

					if (found_rec) {
						if (maxlen != rec_size)
							printk(KERN_INFO
							       "%s: %s(): maxlen=%d while rec_size=%d\n",
							       DEVICE_NAME, __func__, maxlen, rec_size);

						memcpy(pv, pr, maxlen);

						break;
					}

					pc += rec_size;
				}

				/* Record found */
				if (r < rec_counter)
					break;
			}

			/* Did not receive waited record. Resubmit and loop
			 * again... */
			resubmit = 1;
			atomic_set(&usb_if->cmd_sync_complete, 0);

			DPRINTK(KERN_DEBUG "%s: %s(): resubmitting URB...\n",
				DEVICE_NAME, __func__);
		}

		schedule();

		if (get_mtime() > ms_timeout) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
			usb_kill_urb(urb);
#else
			usb_unlink_urb(urb); /* any better solution here for Kernel 2.4 ? */
#endif
			break;
		}
	}

	err = urb->status;

fail:
	atomic_set(&usb_if->cmd_sync_complete, 0);

	return err;
}

/*
 * Hardware Callbacks
 */
static int pcan_usbpro_set_can_on(struct pcandev *dev)
{
	struct pcan_usbpro_msg ub;
	int err;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u)\n",
		DEVICE_NAME, __func__, dev->nChannel+1);

	pcan_usbpro_msg_init_empty(&ub, dev->port.usb.cout_baddr,
					dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SETCANBUSACTIVATE,
	                        dev->nChannel,
	                        1);

#ifdef PCAN_USB_CMD_PER_DEV
	err = pcan_usbpro_cmd_send(dev, &ub);
#else
	err = pcan_usbpro_cmd_send(dev->port.usb.usb_if, &ub);
#endif
	return err;
}

static int pcan_usbpro_set_can_off(struct pcandev *dev)
{
	USB_PORT *u = &dev->port.usb;
	struct pcan_usbpro_msg ub;
	int err;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u)\n",
		DEVICE_NAME, __func__, dev->nChannel+1);

#ifdef PCAN_USBPRO_BUS_OFF
	if (u->state & PCAN_USBPRO_BUS_OFF)
		return 0;
#endif

	pcan_usbpro_msg_init_empty(&ub, dev->port.usb.cout_baddr,
					dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SETCANBUSACTIVATE,
	                        dev->nChannel,
	                        0);
#ifdef PCAN_USB_CMD_PER_DEV
	err = pcan_usbpro_cmd_send(dev, &ub);
#else
	err = pcan_usbpro_cmd_send(u->usb_if, &ub);
#endif
#ifdef PCAN_USBPRO_BUS_OFF
	if (!err)
		u->state |= PCAN_USBPRO_BUS_OFF;
#endif

	return err;
}

/*
 * int pcan_usbpro_set_silent_mode(struct pcandev *dev, int silent_on)
 */
static int pcan_usbpro_set_silent_mode(struct pcandev *dev, int silent_on)
{
	struct pcan_usbpro_msg ub;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u, silent_on=%d)\n",
	        DEVICE_NAME, __func__, dev->nChannel+1, silent_on);

	pcan_usbpro_msg_init_empty(&ub, dev->port.usb.cout_baddr,
					dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SETSILENTMODE,
	                        dev->nChannel,
	                        silent_on);

#ifdef PCAN_USB_CMD_PER_DEV
	return pcan_usbpro_cmd_send(dev, &ub);
#else
	return pcan_usbpro_cmd_send(dev->port.usb.usb_if, &ub);
#endif
}

/*
 * int pcan_usbpro_set_filter_mode(struct pcandev *dev, u16 filter_mode)
 */
static int pcan_usbpro_set_filter_mode(struct pcandev *dev, u16 filter_mode)
{
	struct pcan_usbpro_msg ub;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u, mode=%d)\n", DEVICE_NAME, __func__,
	        dev->nChannel+1, filter_mode);

	pcan_usbpro_msg_init_empty(&ub, dev->port.usb.cout_baddr,
					dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SETFILTERMODE,
	                        dev->nChannel,
	                        filter_mode);

#ifdef PCAN_USB_CMD_PER_DEV
	return pcan_usbpro_cmd_send(dev, &ub);
#else
	return pcan_usbpro_cmd_send(dev->port.usb.usb_if, &ub);
#endif
}

/*
 * int pcan_usbpro_calibration_request_ex(struct pcan_usb_interface *usb_if,
 *                                        u16 onoff, int sync_mode)
 */
static int pcan_usbpro_calibration_request_ex(struct pcan_usb_interface *usb_if,
	                                    u16 onoff, int sync_mode)
{
	struct pcan_usbpro_msg ub;
	u32 send_flags = 0;
	int err;

	DPRINTK(KERN_DEBUG "%s: %s(mode=%d, sync=%d)\n", DEVICE_NAME, __func__,
	        onoff, sync_mode);

	pcan_usbpro_msg_init_empty(&ub, usb_if->dev->port.usb.cout_baddr,
					usb_if->dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                    DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_CALIBRATION_MSG,
	                    onoff);
	if (onoff) {
		send_flags |= PCAN_USBPRO_SEND_GET_TOD;

		/* Remember we ask for receiving calibration messages */
		usb_if->state |= PCAN_USBPRO_WAIT_FOR_CALIBRATION_MSG;

#ifdef PCAN_DEBUG_TIMESTAMP
		printk(KERN_INFO "%s: requesting for calibration messages...\n",
		       DEVICE_NAME);
#endif
	} else {
		/* Remember we ask for no more receiving any calibration msg */
		usb_if->state &= ~PCAN_USBPRO_WAIT_FOR_CALIBRATION_MSG;

#ifdef PCAN_DEBUG_TIMESTAMP
		printk(KERN_INFO "%s: stopping calibration messages...\n",
		       DEVICE_NAME);
#endif
	}

	if (sync_mode)
		send_flags |= PCAN_USBPRO_SEND_SYNC_MODE;

#ifdef PCAN_USB_CMD_PER_DEV
	err = pcan_usbpro_cmd_send_ex(usb_if->dev, &ub, send_flags);
#else
	err = pcan_usbpro_cmd_send_ex(usb_if, &ub, send_flags);
#endif
	if (err) {
		if (onoff)
			usb_if->state &= ~PCAN_USBPRO_WAIT_FOR_CALIBRATION_MSG;
		else
			usb_if->state |= PCAN_USBPRO_WAIT_FOR_CALIBRATION_MSG;
	}

	return err;
}

/*
 * int pcan_usbpro_setget_busload(struct pcandev *dev, u8 mode)
 */
static int pcan_usbpro_setget_busload(struct pcandev *dev, u8 mode)
{
	struct pcan_usbpro_msg ub;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u, mode=%d)\n",
	        DEVICE_NAME, __func__, dev->nChannel+1, mode);

	pcan_usbpro_msg_init_empty(&ub, dev->port.usb.cout_baddr,
					dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_BUSLAST_MSG,
	                        dev->nChannel,
	                        mode);
#ifdef PCAN_USB_CMD_PER_DEV
	return pcan_usbpro_cmd_send(dev, &ub);
#else
	return pcan_usbpro_cmd_send((dev->port.usb.usb_if, &ub);
#endif
}

/*
 * int pcan_usbpro_set_can_led(struct pcandev *dev, u8 mode, u32 timeout)
 */
static int pcan_usbpro_set_can_led(struct pcandev *dev, u8 mode, u32 timeout)
{
	struct pcan_usbpro_msg ub;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u, mode=%d timeout=0x%x)\n",
	        DEVICE_NAME, __func__, dev->nChannel+1, mode, timeout);

	pcan_usbpro_msg_init_empty(&ub, dev->port.usb.cout_baddr,
					dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SET_CANLED,
	                        dev->nChannel,
	                        mode,
	                        timeout);

#ifdef PCAN_USB_CMD_PER_DEV
	return pcan_usbpro_cmd_send(dev, &ub);
#else
	return pcan_usbpro_cmd_send(dev->port.usb.usb_if, &ub);
#endif
}

/*
 * int pcan_usbpro_get_device_nr(struct pcandev *dev, u32 *p_device_nr)
 */
static int pcan_usbpro_get_device_nr(struct pcandev *dev, u32 *p_device_nr)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	struct pcan_usbpro_dev_nr *pdn;
	struct pcan_usbpro_msg ub;
	int err;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u)\n", DEVICE_NAME, __func__,
	        dev->nChannel+1);

	/* Keep record addr in message, to get the response */
	pdn = (struct pcan_usbpro_dev_nr *)\
			pcan_usbpro_msg_init_empty(&ub,
						dev->port.usb.cout_baddr,
						dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR,
	                        dev->nChannel);

#ifdef PCAN_USB_CMD_PER_DEV
	err = pcan_usbpro_cmd_send(dev, &ub);
#else
	err =  pcan_usbpro_cmd_send(usb_if, &ub);
#endif
	if (!err) {
		u32 tmp32;

#ifndef PCAN_USB_CMD_PER_DEV
		// heuristic result - wait a little bit
		mdelay(5);
#endif
		err = pcan_usbpro_read_rec(usb_if, pdn, sizeof(*pdn));
		tmp32 = le32_to_cpu(pdn->serial_num);

		if (tmp32 != 0xffffffff) {
			dev->device_alt_num = tmp32;
			dev->flags |= PCAN_DEV_USES_ALT_NUM;
		}
		if (p_device_nr != NULL)
			*p_device_nr = tmp32;
	}

	return err;
}

/*
 * int pcan_usbpro_set_device_nr(struct pcandev *dev, u32 device_nr)
 */
static int pcan_usbpro_set_device_nr(struct pcandev *dev, u32 device_nr)
{
	struct pcan_usbpro_msg ub;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u)\n", DEVICE_NAME, __func__,
	        dev->nChannel+1);

	pcan_usbpro_msg_init_empty(&ub, dev->port.usb.cout_baddr,
					dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SETDEVICENR,
	                        dev->nChannel,
	                        device_nr);

#ifdef PCAN_USB_CMD_PER_DEV
	return pcan_usbpro_cmd_send(dev, &ub);
#else
	return pcan_usbpro_cmd_send(dev->port.usb.usb_if, &ub);
#endif
}

/*
 * int pcan_usbpro_set_baudrate(struct pcandev *dev, u32 baudrate)
 */
static int pcan_usbpro_set_baudrate(struct pcandev *dev, u32 baudrate)
{
	struct pcan_usbpro_msg ub;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u, 0x%08x)\n",
	        DEVICE_NAME, __func__, dev->nChannel+1, baudrate);

	pcan_usbpro_msg_init_empty(&ub, dev->port.usb.cout_baddr,
					dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SETBAUDRATE,
	                        dev->nChannel,
	                        baudrate);

#ifdef PCAN_USB_CMD_PER_DEV
	return pcan_usbpro_cmd_send(dev, &ub);
#else
	return pcan_usbpro_cmd_send(dev->port.usb.usb_if, &ub);
#endif
}

/*
 * u32 pcan_usbpro_convert_16MHzBTR0BTR1(u16 btr0btr1)
 */
static u32 pcan_usbpro_convert_16MHzBTR0BTR1(u16 btr0btr1)
{
	struct pcan_bittiming_abs at_original, at_convert;
	struct pcan_bittiming sja2010bt;
	const u32 sysclock_Hz = PCAN_USBPRO_SYSCLK_HZ;
	u32 ccbt;

	/* 1) Convert BTR0BTR1 16MHz SJA1000 bittiming specs into an abtsract
	 *    value */
	pcan_btr0btr1_to_abstract(&at_original, btr0btr1);

	/* 2) Convert abstract times for PCAN-USB-PRO CAN ctrl = SJA2010 */
	if (!pcan_convert_abstract(&at_convert, &at_original,
					&sja2010_caps, sysclock_Hz))
		return 0;

	/* 3) Get the bittiming values for PCAN-USB-PRO from the abstract
	 *    time */
	pcan_abstract_to_bittiming(&sja2010bt, &at_convert,
					&sja2010_caps, sysclock_Hz);

#ifdef PCAN_DEBUG_BITTIMING
	pr_info("%s: btr=%u brp=%u tseg1=%u tseg2=%u tq=%u sjw=%u "
		"sample_point=%u\n",
		DEVICE_NAME, sja2010bt.bitrate, sja2010bt.brp,
		sja2010bt.tseg1, sja2010bt.tseg2,
		sja2010bt.tq, sja2010bt.sjw, sja2010bt.sample_point);
#endif
	ccbt = (sja2010bt.tsam) << 23;
	ccbt |= ((sja2010bt.tseg2 - 1) << 20);
	ccbt |= ((sja2010bt.tseg1 - 1) << 16);
	ccbt |= ((sja2010bt.sjw - 1) << 14);
	ccbt |= (sja2010bt.brp - 1);

	return ccbt;
}

#if 0
/* only to avoid computing BTR0BTR1 conversion if defined */
#define PCAN_USBPRO_CCBT_33_3KBPS      0x1400D1
#define PCAN_USBPRO_CCBT_500KBPS       0x1C0006
#define PCAN_USBPRO_CCBT_1MBPS         0x140006
#endif

/*
 * int pcan_usbpro_set_BTR0BTR1(struct pcandev *dev, u16 btr0btr1)
 */
static int pcan_usbpro_set_BTR0BTR1(struct pcandev *dev, u16 btr0btr1)
{
	u32 CCBT; /* CAN Controller Bus Timing register */

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u, 0x%04x)\n",
	        DEVICE_NAME, __func__, dev->nChannel+1, btr0btr1);

	switch (btr0btr1) {
#ifdef PCAN_USBPRO_CCBT_33_3KBPS
	case 0x1D14: CCBT = PCAN_USBPRO_CCBT_33_3KBPS; break;
#endif
#ifdef PCAN_USBPRO_CCBT_500KBPS
	case 0x001C: CCBT = PCAN_USBPRO_CCBT_500KBPS; break;
#endif
#ifdef PCAN_USBPRO_CCBT_1MBPS
	case 0x0014: CCBT = PCAN_USBPRO_CCBT_1MBPS; break;
#endif

	default:
		/* BTR0BTR1 for a SJA1000@16MHz. */
		/* Need to convert this value for PCAN-USB-PRO w/ SJA2010 */
		CCBT = pcan_usbpro_convert_16MHzBTR0BTR1(btr0btr1);
		if (!CCBT) {
			printk(KERN_INFO
			       "%s: unable to convert BTR0BTR1 value 0x%04x\n",
			       DEVICE_NAME, btr0btr1);
			printk(KERN_INFO
			       "%s: won't be able to transfer on CAN%u\n",
			       DEVICE_NAME, dev->nChannel+1);

			return -EINVAL;
		}
	}

	return pcan_usbpro_set_baudrate(dev, CCBT);
}

/*
 * Since Kernel 4.13, transfer data must be dma capable. Stack might not be.
 */
static int pcan_usb_control_msg(struct usb_device *dev, unsigned int pipe,
		__u8 request, __u8 requesttype, __u16 value, __u16 index,
		void *data, __u16 size, int timeout)
{
	int err;
	void *b = pcan_malloc(size, GFP_KERNEL);
	if (!b)
		return -ENOMEM;

	memcpy(b, data, size);
	err = usb_control_msg(dev, pipe, request, requesttype, value, index,
			      b, size, timeout);

	memcpy(data, b, size);
	pcan_free(b);

	return err;
}

/*
 * int pcan_usbpro_get_serial_nr(struct pcan_usb_interface *usb_if, u32 *pdwSNR)
 *
 * Retrieve serial number from bootloader info
 */
static int pcan_usbpro_get_serial_nr(struct pcan_usb_interface *usb_if,
					u32 *pdwSNR)
{
	struct pcan_usbpro_bootloader_info bi;
	int err;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	err = pcan_usb_control_msg(usb_if->usb_dev,
		                   usb_rcvctrlpipe(usb_if->usb_dev, 0),
		                   USB_VENDOR_REQUEST_INFO,
		                   USB_DIR_IN | USB_TYPE_VENDOR|USB_RECIP_OTHER,
		                   USB_VENDOR_REQUEST_wVALUE_INFO_BOOTLOADER,
		                   0, &bi, sizeof(bi),
		                   USB_CTRL_GET_TIMEOUT);
	if (err >= 0) {
		if (pdwSNR) {
			*pdwSNR = le32_to_cpu(bi.serial_num_high);

			DPRINTK(KERN_DEBUG "%s: %s(): SNR=0x%08x\n",
			        DEVICE_NAME, __func__, *pdwSNR);
		}

		err = 0;
	} else {
		pr_info("%s: %s(): unable to retrieve S/N (err=%d)\n",
			DEVICE_NAME,__func__, err);
	}

	return err;
}

/*
 * int pcan_usbpro_open(struct pcandev *dev, u16 btr0btr1, u8 listen_only)
 */
static int pcan_usbpro_open(struct pcandev *dev, u16 btr0btr1,
			    u8 use_ext, u8 listen_only)
{
	int err = 0;
#ifdef DEBUG
	DPRINTK(KERN_DEBUG "%s: %s(CAN%u, btr0btr1=0x%04x, listen_only=%d)\n",
	        DEVICE_NAME, __func__, dev->nChannel+1, btr0btr1, listen_only);
#endif

	err = pcan_usbpro_set_BTR0BTR1(dev, btr0btr1);
	if (err)
		goto fail;

	err = pcan_usbpro_set_silent_mode(dev, listen_only);
	if (err)
		goto fail;

	/* Set filter mode: 0-> All OFF; 1->bypass */
	err = pcan_usbpro_set_filter_mode(dev, 1);
	if (err)
		goto fail;

fail:
	return err;
}

/*
 * int pcan_usbpro_close(struct pcandev *dev)
 */
static int pcan_usbpro_close(struct pcandev *dev)
{
#ifdef DEBUG
	DPRINTK(KERN_DEBUG "%s: %s(CAN%u)\n",
	        DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	return 0;
}

/*
 * Data Messages Handlers
 */
#ifdef PCAN_DEBUG_TIMESTAMP
/*
 * u32 pcan_usbpro_get_frame_number(struct pcan_usb_interface *usb_if)
 */
static u32 pcan_usbpro_get_frame_number(struct pcan_usb_interface *usb_if)
{
	return usb_get_current_frame_number(usb_if->usb_dev)
	     & PCAN_USBPRO_PRECISION_MASK;
}
#endif

/*
 * u32 pcan_usbpro_handle_response_rtt(struct pcan_usb_interface *usb_if)
 */
u32 pcan_usbpro_handle_response_rtt(struct pcan_usb_interface *usb_if)
{
	u32 delta_us;

	pcan_gettimeofday(&usb_if->tv_response);

#ifndef PCAN_USBPRO_DRIVER_CALIBRATION_PERIOD
	if (usb_if->rtt)
		return usb_if->rtt;
#endif

	delta_us = pcan_timeval_delta_us(&usb_if->tv_request,
						&usb_if->tv_response);

	if (!usb_if->rtt)
		usb_if->rtt = delta_us;

	usb_if->rtt = (usb_if->rtt + delta_us) >> 1;

#ifdef PCAN_DEBUG_TIMESTAMP
	printk(KERN_INFO "%s: got response in %uus (rtt=%u)\n",
	       DEVICE_NAME, delta_us, usb_if->rtt);
#endif

	return delta_us;
}

/*
 * void pcan_usbpro_time_sync(struct pcan_usb_interface *usb_if,
 *                            struct timeval *tv,
 *                            u32 ts_us,
 *                            u32 dev_frame_nb)
 *
 * Do synchronize host local time tv with adapter timestamp.
 * Any further timestamp will be an offset to that tv, taking RTT/2 into
 * account.
 */
void pcan_usbpro_time_sync(struct pcan_usb_interface *usb_if,
	                   struct timeval *tv,
	                   u32 ts_us,
	                   u32 dev_frame_nb)
{
#ifdef PCAN_DEBUG_TIMESTAMP
	const u32 host_frame_nb = pcan_usbpro_get_frame_number(usb_if);
#endif

	/* synchronize local time and adapter time, taking into account RTT/2 */
	/* USB transfer time + low level and adapter processing time */
	if (!tv)
		pcan_gettimeofday(&usb_if->rx0_tv);
	else
		usb_if->rx0_tv = *tv;

	usb_if->rx0_ts = ts_us;

#ifdef PCAN_USBPRO_DRIVER_CALIBRATION_PERIOD
	usb_if->rx0_ts += (usb_if->rtt >> 1);
#endif

	usb_if->rx0_sync = 0;

#ifdef PCAN_DEBUG_TIMESTAMP
	printk(KERN_INFO "%s: hw=%uus #0x%08x (host=%us.%uus #0x%08x)\n",
	       DEVICE_NAME, ts_us, dev_frame_nb,
	       (u32 )usb_if->rx0_tv.tv_sec,
	       (u32 )usb_if->rx0_tv.tv_usec, host_frame_nb);
#endif
}

/*
 * void pcan_usbpro_timestamp_decode(struct pcan_usb_interface *usb_if,
 *                                   u32 ts_us,
 *                                   struct timeval *tv)
 */
void pcan_usbpro_timestamp_decode(struct pcan_usb_interface *usb_if,
                                  u32 ts_us,
	                          struct timeval *tv)
{
	u32 delta_us;
	u32 delta_s;

	/* if should sync with system time, save and return it */
	if (usb_if->rx0_sync) {
		pcan_usbpro_time_sync(usb_if, NULL, ts_us, 0);
		*tv = usb_if->rx0_tv;
		return;
	}

	/* compute delta in ts, taking 32-bits wrapping into account */
	/* note: this works only if a ~2.000 s. timer updates rx0_tv AND */
	/* compute an estimated value of rx0_ts */
	if (ts_us >= usb_if->rx0_ts) {
		delta_us = ts_us - usb_if->rx0_ts;
	} else {
		delta_us = 0xffffffff - usb_if->rx0_ts;
		delta_us += ts_us;
	}

	/* number of s. to add to final time */
	delta_s = delta_us / USEC_PER_SEC;
	delta_us -= (delta_s * USEC_PER_SEC);

	tv->tv_usec = usb_if->rx0_tv.tv_usec + delta_us;
	if (tv->tv_usec >= USEC_PER_SEC) {
		tv->tv_usec -= USEC_PER_SEC;
		delta_s++;
	}
	tv->tv_sec = usb_if->rx0_tv.tv_sec + delta_s;

#ifdef PCAN_DEBUG_TIMESTAMP
	{
		struct timeval now;

		pcan_gettimeofday(&now);

		printk(KERN_INFO "%s: hw=%uus -> host=%us.%uus (now=%us.%uus, d=%uus.)\n",
		       DEVICE_NAME, ts_us, (u32 )tv->tv_sec, (u32 )tv->tv_usec,
		       (u32 )now.tv_sec, (u32 )now.tv_usec,
		       pcan_timeval_delta_us(tv, &now));
	}
#endif
}

/*
 * int pcan_usbpro_handle_canmsg_rx(struct pcan_usb_interface *usb_if,
 *                                  struct pcan_usbpro_canmsg_rx *rx)
 *
 * Handler of record types:
 * - DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_x
 * - DATA_TYPE_USB2CAN_STRUCT_CANMSG_RTR_RX
 */
static int pcan_usbpro_handle_canmsg_rx(struct pcan_usb_interface *usb_if,
	                                struct pcan_usbpro_canmsg_rx *rx)
{
	const int ctrl_index = (rx->len >> 4) & 0x0f;
	struct pcandev *dev = &usb_if->dev[ctrl_index];
	struct pcanfd_msg f;
	int err;

#if 1
	DPRINTK(KERN_DEBUG "%s: %s(CAN%u): "
	        "rx=[client=0x%02x flags=0x%02x len=0x%02x "
	        "timestamp32=0x%08x id=0x%08x]\n",
	        DEVICE_NAME, __func__, ctrl_index+1,
	        rx->client, rx->flags, rx->len, rx->timestamp32,
		le32_to_cpu(rx->id));
#endif

	dev->dwInterruptCounter++;

#if 1
	/* such test is now made by pcan_chardev_rx() func */
#else
	/* Don't send any message when device not opened */
	if (dev->nOpenPaths <= 0) {
		pr_info("%s: incoming message 0x%x (flags=%x) discarded: "
		        "CAN%u not yet opened\n",
		        DEVICE_NAME, rx->id, rx->flags, ctrl_index+1);
		return 0;
	}
#endif
	memset(&f, '\0', sizeof(f));

	f.type = PCANFD_TYPE_CAN20_MSG;
	f.flags = PCANFD_MSG_STD|PCANFD_HWTIMESTAMP|PCANFD_TIMESTAMP;
	f.id = le32_to_cpu(rx->id);
	f.data_len = rx->len & 0x0f;

	if (rx->flags & PCAN_USBPRO_RTR)
		f.flags |= PCANFD_MSG_RTR;
	if (rx->flags & PCAN_USBPRO_EXT)
		f.flags |= PCANFD_MSG_EXT;
	if (rx->client & PCAN_USBPRO_SR)
		f.flags |= PCANFD_MSG_SLF;

	memcpy(f.data, rx->data, f.data_len);

	pcan_usbpro_timestamp_decode(usb_if,
				le32_to_cpu(rx->timestamp32), &f.timestamp);

#ifdef PCAN_DEBUG_TIMESTAMP
	pr_info("%s: ts=%08u = %u.%06u s\n",
			DEVICE_NAME, le32_to_cpu(rx->timestamp32),
			(uint )f.timestamp.tv_sec, (uint)f.timestamp.tv_usec);
#endif
	err = pcan_xxxdev_rx(dev, &f);
	if (err > 0)
		dev->port.usb.state |= PCAN_USBPRO_SHOULD_WAKEUP;

	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	return err;
}

/*
 * int pcan_usbpro_handle_status_err_rx(struct pcan_usb_interface *usb_if,
 *                              struct pcan_usbpro_canmsg_status_error_rx *er)
 */
static int pcan_usbpro_handle_status_err_rx(struct pcan_usb_interface *usb_if,
	                    struct pcan_usbpro_canmsg_status_error_rx *er)
{
	const u32 raw_status = le32_to_cpu(er->status);
	const int ctrl_index = (er->channel >> 4) & 0x0f;
	struct pcandev *dev = &usb_if->dev[ctrl_index];
	struct pcanfd_msg f;
	int err;

#if 0
	DPRINTK(KERN_DEBUG "%s: %s(CAN%u): "
	        "rx=[status=0x%04x timestamp32=0x%08x error_frame=0x%08x]\n",
	        DEVICE_NAME, __func__, ctrl_index+1,
	        raw_status, le32_to_cpu(er->timestamp32),
	        le32_to_cpu(er->error_frame));
#endif

	dev->dwInterruptCounter++;

	memset(&f, '\0', sizeof(f));

	if (raw_status & FW_USBPRO_STATUS_MASK_BUS_S) {
		/* Bus Off */
		pcan_handle_busoff(dev, &f);
	} else {
		dev->rx_error_counter =
				(le32_to_cpu(er->error_frame) >> 16) & 0xff;

		dev->tx_error_counter =
				(le32_to_cpu(er->error_frame) >> 24) & 0xff;

		if (!pcan_handle_error_status(dev, &f,
			(raw_status & FW_USBPRO_STATUS_MASK_ERROR_S), /* EW */
			(dev->rx_error_counter > 127 || /* EP */
					dev->tx_error_counter > 127))) {

			/* no error bit (so, no error, back to active state) */
			pcan_handle_error_active(dev, &f);
		}
	}

	if (raw_status & FW_USBPRO_STATUS_MASK_OVERRUN_S)
		pcan_handle_error_protocol(dev, &f, PCANFD_RX_OVERFLOW);

	if (raw_status & FW_USBPRO_STATUS_MASK_QOVERRUN_S)
		pcan_handle_error_ctrl(dev, &f, PCANFD_RX_OVERFLOW);

	/* Don't send any null message */
	if (f.type == PCANFD_TYPE_NOP)
		return 0;
#if 1
	/* such test is now made by pcan_chardev_rx() func */
#else
	/* Don't send any null message or when device not opened */
	if (dev->OpenPaths <= 0) {
		pr_info("%s: error msg status=0x%x error=0x%x discarded: "
			"CAN%u open count=%u tx_err=%u rx_err=%u\n",
		        DEVICE_NAME, raw_status, le32_to_cpu(er->error_frame),
		        ctrl_index, dev->nOpenPaths,
			dev->tx_error_counter, dev->rx_error_counter);
		return 0;
	}
#endif
	f.flags |= PCANFD_HWTIMESTAMP|PCANFD_TIMESTAMP;
	pcan_usbpro_timestamp_decode(usb_if, le32_to_cpu(er->timestamp32),
								&f.timestamp);
	err = pcan_xxxdev_rx(dev, &f);
	if (err > 0)
		dev->port.usb.state |= PCAN_USBPRO_SHOULD_WAKEUP;

	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	return err;
}

/*
 * int pcan_usbpro_handle_calibration_msg(struct pcan_usb_interface *usb_if,
 *                                   struct pcan_usbpro_calibration_ts_rx *ts)
 * Handler of record types:
 * - DATA_TYPE_USB2CAN_STRUCT_CALIBRATION_TIMESTAMP_RX
 */
static int pcan_usbpro_handle_calibration_msg(struct pcan_usb_interface *usb_if,
				struct pcan_usbpro_calibration_ts_rx *ts)
{
	const u32 raw_frame_index = le32_to_cpu(ts->timestamp64[0]);
	u32 dev_frame_nb;
	int i;

#ifdef PCAN_DEBUG_TIMESTAMP
	printk(KERN_INFO "got calibration message: ts=0x%08x fr=0x%08x\n",
	       ts->timestamp64[1], ts->timestamp64[0]);
#endif

#if 0
	DPRINTK(KERN_DEBUG "%s: %s(): "
	        "ts.timestamp={0x%08x, 0x%08x}\n",
	        DEVICE_NAME, __func__, ts->timestamp64[0], ts->timestamp64[1]);
#endif

	if (!(usb_if->state & PCAN_USBPRO_WAIT_FOR_CALIBRATION_MSG))
		return 0;

	/* use this message to compute RTT (here time diff between tv_request &
	 * tv_response) before doing synchronization */
	pcan_usbpro_handle_response_rtt(usb_if);

#ifndef PCAN_USBPRO_USE_UFRAMES
	dev_frame_nb = raw_frame_index & PCAN_USBPRO_PRECISION_MASK;
#else
	dev_frame_nb = (raw_frame_index & PCAN_USBPRO_PRECISION_MASK) << 3
	            | ((raw_frame_index >> 11) & 0x7);
#endif

	pcan_usbpro_time_sync(usb_if, &usb_if->tv_response,
	                      le32_to_cpu(ts->timestamp64[1]), dev_frame_nb);

#ifdef PCAN_USBPRO_DRIVER_CALIBRATION_PERIOD
	/* turn off calibration message function.
	 * WARNING: must be done asynchronously (no schedule()!) since we
	 * already come from a completion handler */
	pcan_usbpro_calibration_request_async(usb_if, 0);

	/* set a timer to send next request in the future */
	mod_timer(&usb_if->calibration_timer,
	          jiffies + PCAN_USBPRO_DRIVER_CALIBRATION_PERIOD * HZ);
#endif

	/* force state to ERROR_ACTIVE */
	for (i = 0; i < usb_if->can_count; i++)
		pcan_soft_error_active(usb_if->dev + i);

	return 0;
}

/*
 * int pcan_usbpro_handle_buslast_rx(struct pcan_usb_interface *usb_if,
 *                                   struct pcan_usbpro_buslast_rx *bl)
 */
static int pcan_usbpro_handle_buslast_rx(struct pcan_usb_interface *usb_if,
	                                 struct pcan_usbpro_buslast_rx *bl)
{
	const int ctrl_index = (bl->channel >> 4) & 0x0f;
	struct pcandev *dev = &usb_if->dev[ctrl_index];
	u32 buslast_val = le16_to_cpu(bl->buslast_val);

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u): "
	        "bl=[buslast_val=0x%04x timestamp=0x%08x]\n",
	        DEVICE_NAME, __func__, ctrl_index+1,
	        bl->buslast_val, le32_to_cpu(bl->timestamp32));

	dev->bus_load = (u16 )((100 * buslast_val) / 4096);

	return 0;
}

/*
 * int pcan_usbpro_msg_decode(struct pcan_usb_interface *usb_if,
 *                            u8 *msg_addr, int msg_len)
 *
 * Decode a message from PCAN-USB-PRO.
 */
static int pcan_usbpro_msg_decode(struct pcan_usb_interface *usb_if,
	                          u8 *msg_addr, int msg_len)
{
	int err = 0;
	struct pcan_usbpro_msg usb_msg;
	u8 *rec_ptr, *msg_end;
	u16 rec_count;
	int d;

	//DPRINTK(KERN_DEBUG "%s: %s(%d)\n", DEVICE_NAME, __func__, msg_len);
	//dump_mem("received msg", msg_addr, msg_len);

	/* Apply any fragmentation offset to the packet address to seek on */
	/* the (simulated) record count */
	msg_addr -= usb_if->frag_rec_offset;
	msg_len += usb_if->frag_rec_offset;

	/* For USB-PRO, messages MUST be at least 4-bytes length */
	rec_ptr = pcan_usbpro_msg_init(&usb_msg, msg_addr, msg_len);
	if (rec_ptr == NULL) {
		DPRINTK(KERN_DEBUG "%s: %s() invalid msg len %d (ignored)\n",
		        DEVICE_NAME, __func__, msg_len);
		return 0;
	}

#ifdef PCAN_DEBUG_DECODE
	printk(KERN_INFO "decoding incoming message: "
	                 "rec_counter=%d (0x%08x) len=%d:\n",
	                 *usb_msg.u.rec_counter_read, *usb_msg.u.rec_counter,
	                 msg_len);
#endif

	/* do some init for each controller */
	for (d=0; d < usb_if->can_count; d++)
		usb_if->dev[d].port.usb.state &= ~PCAN_USBPRO_SHOULD_WAKEUP;

	/* Loop reading all the records from the incoming message */
	msg_end = msg_addr + msg_len;
	for (rec_count=le16_to_cpu(*usb_msg.u.rec_counter_read); rec_count > 0;
	                                                          rec_count--) {
		union pcan_usbpro_rec *pr = (union pcan_usbpro_rec *)rec_ptr;
		int sizeof_rec = pcan_usbpro_sizeof_rec(pr->data_type);

#ifdef PCAN_DEBUG_DECODE
		printk(KERN_INFO "%02u: record=0x%02x (%d) sizeof=%d\n",
		       rec_count, pr->data_type, pr->data_type, sizeof_rec);
#endif
		if (sizeof_rec <= 0) {
			pr_info("%s: got unsupported record in rx msg:\n",
					DEVICE_NAME);
			dump_mem("message content", msg_addr, msg_len);
			err = -ENOTSUPP;
			break;
		}

		/* Check if the record goes out of current packet */
		if (rec_ptr + sizeof_rec > msg_end) {
			/* Yes, it does: donot handle it here but wait for next
			 * loop decoding on the next packet.
			 * Uses current packet to simulate message header with
			 * the correct record counter */
			rec_ptr -= 4;
			*(u32 *)rec_ptr = cpu_to_le32(rec_count);

			err = rec_count;
			break;
		}

		switch (pr->data_type) {
		case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_8:
		case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_4:
		case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_0:
		case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RTR_RX:
			err = pcan_usbpro_handle_canmsg_rx(usb_if,
							&pr->canmsg_rx);
			if (err < 0)
				goto fail;
			break;

		case DATA_TYPE_USB2CAN_STRUCT_CANMSG_STATUS_ERROR_RX:
			err = pcan_usbpro_handle_status_err_rx(usb_if,
			                           &pr->canmsg_status_error_rx);
			if (err < 0)
				goto fail;
			break;

		case DATA_TYPE_USB2CAN_STRUCT_CALIBRATION_TIMESTAMP_RX:
			err = pcan_usbpro_handle_calibration_msg(usb_if,
			                                &pr->calibration_ts_rx);
			break;

		case DATA_TYPE_USB2CAN_STRUCT_BUSLAST_RX:
			err = pcan_usbpro_handle_buslast_rx(usb_if,
							&pr->buslast_rx);
			break;

		default:
			printk(KERN_INFO "%s: %s(): unhandled rec 0x%02x(%d)\n",
			       DEVICE_NAME, __func__, pr->data_type,
			       pr->data_type);
			dump_mem("message content", msg_addr, msg_len);
			break;
		}

		rec_ptr += sizeof_rec;
	}

	/* Always compute next packet offset to seek on (simulated) record
	 * counter. If all records have been processed here, next packet will
	 * start with a correct message header */
	usb_if->frag_rec_offset = (rec_count > 0) ? (msg_end - rec_ptr) : 0;

fail:

	/* check if something is to be waken up */
	for (d=0; d < usb_if->can_count; d++) {
		if (usb_if->dev[d].port.usb.state & PCAN_USBPRO_SHOULD_WAKEUP) {

			usb_if->dev[d].dwInterruptCounter++;
#ifdef PCAN_DEBUG_DECODE
			printk(KERN_INFO "wakeup task reading CAN%u\n", d+1);
#endif
			pcan_event_signal(&usb_if->dev[d].in_event);
		}
	}

	return err;
}

/*
 * int pcan_usbpro_msg_encode(struct pcandev *dev,
 *                            u8 *buffer_addr, int *buffer_size)
 *
 * Reads into CAN fifo and encode data in USB messages.
 *
 * @return -ENODATA  if no more data in CAN fifo,
 *         -ENOSPC if *buffer_size is not large enough to store a TX_x
 *                   record
 *         0         if output buffer is full of TX_x records,
 *         any other -ERR.
 */
static int pcan_usbpro_msg_encode(struct pcandev *dev,
	                          u8 *buffer_addr, int *buffer_size)
{
	const int rec_max_len = \
	   pcan_usbpro_sizeof_rec(DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_8);

	int err = 0;
	int buffer_high_water;
	struct pcan_usbpro_msg usb_msg;

#if 0
	DPRINTK(KERN_DEBUG "%s: %s(buffer_size=%d) rec_max_size=%d "
	        "msg_in_fifo=%d fifo_empty=%d\n", DEVICE_NAME, __func__,
	        *buffer_size, rec_max_len,
	        dev->writeFifo.nStored, pcan_fifo_empty(&dev->writeFifo));
#endif

	/* In order to accelerate things... */
	if (pcan_fifo_empty(&dev->writeFifo)) {
		*buffer_size = 0;
		return -ENODATA;
	}

	if (*buffer_size < rec_max_len) {
		DPRINTK(KERN_DEBUG "%s: %s(): not enough room to store %d "
			"bytes record into %d bytes buffer\n",
		        DEVICE_NAME, __func__, rec_max_len, *buffer_size);

		*buffer_size = 0;
		return -ENOSPC;
	}

	if (!pcan_usbpro_msg_init_empty(&usb_msg, buffer_addr, *buffer_size)) {
		*buffer_size = 0;
		return -ENOSPC;
	}

	/* Loop while sure to have enough room to store any TX_x record */
	buffer_high_water = *buffer_size - rec_max_len;

	while (usb_msg.rec_buffer_len < buffer_high_water) {
		struct pcanfd_msg f;
		u8 data_type, client, len, flags;

		// release fifo buffer and step forward in fifo
		if ((err = pcan_fifo_get(&dev->writeFifo, &f))) {
			if (err != -ENODATA) {
				printk(KERN_ERR
					"%s: %s(): can't get data out of "
					"writeFifo, available data=%d err=%d\n",
				       DEVICE_NAME, __func__,
				       dev->writeFifo.nStored, err);
			}

			break;
		}

#ifdef MSGTYPE_PARAMETER
		if (f.flags & (MSGTYPE_STATUS | MSGTYPE_PARAMETER)) {
#else
		if (f.flags & MSGTYPE_STATUS) {
#endif
			printk(KERN_INFO
				"%s: %s() CAN msg type %02Xh ignored\n",
				DEVICE_NAME, __func__, f.flags);
			continue;
		}

		if ((f.flags & MSGTYPE_RTR) || (f.data_len == 0)) {
			data_type = DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_0;
		} else if (f.data_len <= 4) {
			data_type = DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_4;
		} else {
			data_type = DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_8;
		}

#if 0
		/* Check if enough room to add one more TX_X */
		if (usb.msg.rec_buffer_len + pcan_usbpro_sizeof_rec(data_type) > \
		     usb_msg.rec_buffer_size) {
			err = -ENOSPC;
			break;
		}
#endif

		len = (dev->nChannel << 4) | (f.data_len & 0x0f);

		flags = 0;
#ifdef MSGTYPE_SINGLESHOT
		if (f.flags & MSGTYPE_SINGLESHOT)
			flags |= 0x08;
#endif
		if (f.flags & MSGTYPE_EXTENDED)
			flags |= 0x02;
		if (f.flags & MSGTYPE_RTR)
			flags |= 0x01;

		client = 0;
#ifdef MSGTYPE_SELFRECEIVE
		if (f.flags & MSGTYPE_SELFRECEIVE) {
			flags |= 0x04;
			client |= PCAN_USBPRO_SR;
		}
#endif
		pcan_usbpro_msg_add_rec(&usb_msg,
		                        data_type,
		                        client,
		                        flags,
		                        len,
		                        f.id,
		                        f.data);
	}

	/* Don't return rec_buffer_len if no record at all has been copied,
	 * since it is initialized to 4... */
	*buffer_size = (*usb_msg.u.rec_counter == 0) ? 0
						: usb_msg.rec_buffer_len;

	return err;
}

/*
 * int pcan_usbpro_request(struct pcan_usb_interface *usb_if,
 *                         int req_id, int req_value,
 *                         void *req_addr, int req_size)
 * Send USB Vendor type request
 */
int pcan_usbpro_request(struct pcan_usb_interface *usb_if,
			int req_id, int req_value,
			void *req_addr, int req_size)
{
	int err;
	u8 req_type;
	unsigned int p;

	req_type = USB_TYPE_VENDOR | USB_RECIP_OTHER;
	switch (req_id) {
	case USB_VENDOR_REQUEST_FKT:
		/* Host to device */
		p = usb_sndctrlpipe(usb_if->usb_dev, 0);
		break;

	case USB_VENDOR_REQUEST_INFO:
	default:
		/* Device to host */
		p = usb_rcvctrlpipe(usb_if->usb_dev, 0);
		req_type |= USB_DIR_IN;
		memset(req_addr, '\0', req_size);
	}

	err = pcan_usb_control_msg(usb_if->usb_dev,
	                      p,
	                      req_id,
	                      req_type,
	                      req_value,
	                      0, req_addr, req_size,
	                      2*USB_CTRL_GET_TIMEOUT);
	if (err < 0)
		DPRINTK(KERN_INFO "%s: unable to request usb"
			"[type=%d value=%d] err=%d\n",
		        DEVICE_NAME, req_id, req_value, err);

	else {
		//dump_mem("request content", req_addr, req_size);
		err = 0;
	}

	return err;
}

void pcan_usbpro_driver_loaded(struct pcan_usb_interface *usb_if,
	                            int can_lin, int loaded)
{
#ifdef USB_VENDOR_REQUEST_wVALUE_SETFKT_INTERFACE_DRIVER_LOADED
	/* Tell the USB-PRO that the Driver is loaded */
	u8 buffer[16];

	buffer[0] = (u8 )can_lin;	/* Interface CAN=0 LIN=1 */
	buffer[1] = !!loaded;		/* Driver loaded 0/1 */

	pcan_usbpro_request(usb_if,
	                    USB_VENDOR_REQUEST_FKT,
	                    USB_VENDOR_REQUEST_wVALUE_SETFKT_INTERFACE_DRIVER_LOADED,
	                    &buffer[0], sizeof(buffer));
#endif
}

/*
 * void pcan_usbpro_cleanup(struct pcandev *dev)
 *
 * Last chance to submit URB before driver removal.
 */
static void pcan_usbpro_cleanup(struct pcandev *dev)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);

#ifdef DEBUG

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u)\n",
	        DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	/* Sometimes, bus off request can't be submit when driver is removed so,
	 * when the device was not properly closed. So, move the bus off request
	 * here to be sure it is sent. */
	pcan_usbpro_set_can_off(dev);

#ifdef USB_VENDOR_REQUEST_wVALUE_SETFKT_INTERFACE_DRIVER_LOADED
	/* No more need to switch off the LEDs by ourselves!
	 * Fw does it when we notify it from driver unload! */
#else
	/* Switch LED off */
	pcan_usbpro_set_can_led(dev, 4, 0xffffffff); /* orange off */
#endif

#ifdef PCAN_USBPRO_GET_BUSLOAD
	pcan_usbpro_setget_busload(dev, 0);
#endif

	/* If last device, tell module that driver is unloaded */
	if (dev->nChannel == (usb_if->can_count-1)) {

		/* turn off calibration message */
		pcan_usbpro_calibration_request(usb_if, 0);

		/* Tell module the CAN driver is unloaded */
		pcan_usbpro_driver_loaded(usb_if, 0, 0);

		/* release adapter object */
		pcan_free(usb_if->adapter);
	}
}

/*
 * int pcan_usbpro_ctrl_init(struct pcandev *dev)
 *
 * Do CAN controller specific initialization.
 */
static int pcan_usbpro_ctrl_init(struct pcandev *dev)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	int err;

	DPRINTK(KERN_DEBUG "%s: %s(CAN%u)\n",
	        DEVICE_NAME, __func__, dev->nChannel+1);

	/* set bittiming capabilities */
	dev->bittiming_caps = &sja2010_caps;

#ifdef PCAN_USBPRO_GET_BUSLOAD
	err = pcan_usbpro_setget_busload(dev, 1);
#endif

	/* now, seems that may avoid to retry... 
	 * request now for 1st calibration message (on last ctrlr only) */
	if (dev->nChannel == (usb_if->can_count-1)) {
		/* Tell device the CAN driver is loaded */
		//pcan_usbpro_driver_loaded(usb_if, 0, 1);

		err = pcan_usbpro_calibration_request(usb_if, 1);
		if (err)
			return err;
	}

	pr_info("%s: %s channel %d device number=%u\n",
	        DEVICE_NAME, usb_if->adapter->name,
		dev->nChannel+1, dev->device_alt_num);

	/* set LED in default state (end of init phase) */
	pcan_usbpro_set_can_led(dev, 0, 1);

	return 0;
}

/*
 * void pcan_usbpro_free(struct pcan_usb_interface *usb_if)
 */
static void pcan_usbpro_free(struct pcan_usb_interface *usb_if)
{
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

#ifdef PCAN_USBPRO_DRIVER_CALIBRATION_PERIOD
	del_timer_sync(&usb_if->calibration_timer);
#endif
}

/*
 * int pcan_usbpro_init(struct pcan_usb_interface *usb_if)
 *
 * Do device specifc initialization.
 */
int pcan_usbpro_init(struct pcan_usb_interface *usb_if)
{
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	usb_if->adapter = pcan_alloc_adapter("PCAN-USB Pro",
						pcan_usbpro_devices++, 2);
	if (!usb_if->adapter)
		return -ENOMEM;

	/* Set PCAN-USB-PRO hardware specific callbacks */
	usb_if->device_ctrl_init = pcan_usbpro_ctrl_init;
	usb_if->device_get_snr = pcan_usbpro_get_serial_nr;
	usb_if->device_msg_decode = pcan_usbpro_msg_decode;
	usb_if->device_free = pcan_usbpro_free;

	usb_if->device_ctrl_cleanup = pcan_usbpro_cleanup;
	usb_if->device_ctrl_open = pcan_usbpro_open;
	usb_if->device_ctrl_close = pcan_usbpro_close;
	usb_if->device_ctrl_set_bus_on = pcan_usbpro_set_can_on;
	usb_if->device_ctrl_set_bus_off = pcan_usbpro_set_can_off;
	usb_if->device_ctrl_set_dnr = pcan_usbpro_set_device_nr;
	usb_if->device_ctrl_get_dnr = pcan_usbpro_get_device_nr;
	usb_if->device_ctrl_msg_encode = pcan_usbpro_msg_encode;

#if 1
	/* SGr: device_alt_num is now initialized to 0xffffffff, in
	 * pcan_soft_init(). *BUT* pcan_soft_init() is called a bit
	 * later, and after this function. This should not be a problem.  */
#else
	/*
	 * MUST initialize alt_num here (before creating devices) for Udev rules
	 */
	usb_if->dev[0].device_alt_num = 0xffffffff;
	usb_if->dev[1].device_alt_num = 0xffffffff;
#endif

#if 0
	{
		extern u16 sja1000_bitrate(u32 dwBitRate);

		struct pcan_bittiming bt;
		const u32 sysclock_Hz = PCAN_USBPRO_SYSCLK_HZ;
		u16 btr0btr1;

		memset(&bt, '\0', sizeof(bt));
		bt.bitrate = 1000000;
		btr0btr1 = sja1000_bitrate(bt.bitrate);
		pcan_bitrate_to_bittiming(&bt, &sja2010_caps, sysclock_Hz);
		pr_info("%s: btr=%u brp=%u tseg1=%u tseg2=%u tq=%u sjw=%u "
			"sample_point=%u btr0btr1=%04xh\n",
			DEVICE_NAME, bt.bitrate, bt.brp, bt.tseg1, bt.tseg2,
			bt.tq, bt.sjw, bt.sample_point, btr0btr1);

		memset(&bt, '\0', sizeof(bt));
		bt.bitrate = 500000;
		btr0btr1 = sja1000_bitrate(bt.bitrate);
		pcan_bitrate_to_bittiming(&bt, &sja2010_caps, sysclock_Hz);
		pr_info("%s: btr=%u brp=%u tseg1=%u tseg2=%u tq=%u sjw=%u "
			"sample_point=%u btr0btr1=%04xh\n",
			DEVICE_NAME, bt.bitrate, bt.brp, bt.tseg1, bt.tseg2,
			bt.tq, bt.sjw, bt.sample_point, btr0btr1);

		memset(&bt, '\0', sizeof(bt));
		bt.bitrate = 250000;
		btr0btr1 = sja1000_bitrate(bt.bitrate);
		pcan_bitrate_to_bittiming(&bt, &sja2010_caps, sysclock_Hz);
		pr_info("%s: btr=%u brp=%u tseg1=%u tseg2=%u tq=%u sjw=%u "
			"sample_point=%u btr0btr1=%04xh\n",
			DEVICE_NAME, bt.bitrate, bt.brp, bt.tseg1, bt.tseg2,
			bt.tq, bt.sjw, bt.sample_point, btr0btr1);
	}
#endif

#ifdef USB_VENDOR_REQUEST_wVALUE_INFO_FIRMWARE
	/* Firmware info */
	{
		struct pcan_usbpro_ext_firmware_info fi;

		if (pcan_usbpro_request(usb_if,
		                        USB_VENDOR_REQUEST_INFO,
		                        USB_VENDOR_REQUEST_wVALUE_INFO_FIRMWARE,
		                        &fi, sizeof(fi)) >= 0) {
			printk(KERN_INFO
			       "%s: %s fw v%d.%d.%d (%02d/%02d/%02d)\n",
			       DEVICE_NAME, usb_if->adapter->name,
			       fi.version[0], fi.version[1], fi.version[2],
			       fi.day, fi.month, fi.year);

#ifdef DEBUG
			printk(KERN_INFO "%s: type=0x%08x\n",
			       DEVICE_NAME, fi.fw_type);
#endif
			usb_if->adapter->hw_ver_major = fi.version[0];
			usb_if->adapter->hw_ver_minor = fi.version[1];
			usb_if->adapter->hw_ver_subminor = fi.version[2];
		}
	}
#endif

#ifdef USB_VENDOR_REQUEST_wVALUE_INFO_BOOTLOADER
	/* Bootloader info */
	{
		struct pcan_usbpro_bootloader_info bi;

		if (pcan_usbpro_request(usb_if,
		                        USB_VENDOR_REQUEST_INFO,
		                        USB_VENDOR_REQUEST_wVALUE_INFO_BOOTLOADER,
		                        &bi, sizeof(bi)) >= 0) {
#ifdef DEBUG
			printk(KERN_INFO
			       "%s: bootloader v%d.%d.%d (%02d/%02d/%02d)\n",
			       DEVICE_NAME,
			       bi.version[0], bi.version[1], bi.version[2],
			       bi.day, bi.month, bi.year);

			printk(KERN_INFO
			       "%s: S/N=%08X.%08X hw=0x%08x.0x%08x\n",
			       DEVICE_NAME,
			       bi.serial_num_high, bi.serial_num_low,
			       bi. hw_type, bi.hw_rev);
#endif
			usb_if->ucRevision = (u8 )bi.hw_rev;
		}
	}
#endif

#if 0//def USB_VENDOR_REQUEST_wVALUE_INFO_DEVICENR
	/* Device Number */
	{
		struct pcan_usbpro_device_nr dnr;

		if (pcan_usbpro_request(usb_if,
		                        USB_VENDOR_REQUEST_INFO,
		                        USB_VENDOR_REQUEST_wVALUE_INFO_DEVICENR,
		                        &dnr, sizeof(dnr)) >= 0) {
			printk(KERN_INFO
			       "%s: Device #0x%08x\n",
			       DEVICE_NAME, dnr.device_nr);

			usb_if->ucHardcodedDevNr = (u8 )dnr.device_nr;
		}
	}
#endif

#ifdef USB_VENDOR_REQUEST_wVALUE_INFO_CPLD
	/* CPLD */
	{
		struct pcan_usbpro_cpld_info ci;

		if (pcan_usbpro_request(usb_if,
		                        USB_VENDOR_REQUEST_INFO,
		                        USB_VENDOR_REQUEST_wVALUE_INFO_CPLD,
		                        &ci, sizeof(ci)) >= 0) {
#ifdef DEBUG
			printk(KERN_INFO "%s: CPLD=0x%08x\n",
			       DEVICE_NAME, ci.cpld_nr);
#endif
		}
	}
#endif

#if 0//def USB_VENDOR_REQUEST_wVALUE_INFO_MODE
	/* Mode */
	{
		struct pcan_usbpro_info_mode mi;

		if (pcan_usbpro_request(usb_if,
		                        USB_VENDOR_REQUEST_INFO,
		                        USB_VENDOR_REQUEST_wVALUE_INFO_MODE,
		                        &mi, sizeof(mi)) >= 0) {
			printk(KERN_INFO "%s: mode[%d] = 0x%02x\n",
			       DEVICE_NAME, dev->nChannel, mi.mode[dev->nChannel]);
		}
	}
#endif

#if 0//def USB_VENDOR_REQUEST_wVALUE_INFO_TIMEMODE
	/* Time Mode */
	{
		struct pcan_usbpro_time_mode tm;

		if (pcan_usbpro_request(usb_if,
		                        USB_VENDOR_REQUEST_INFO,
		                        USB_VENDOR_REQUEST_wVALUE_INFO_TIMEMODE,
		                        &tm, sizeof(tm)) >= 0) {
			printk(KERN_INFO "%s: Time mode %d flags=0x%04x\n",
			       DEVICE_NAME, tm.time_mode, tm.flags);
		}
	}
#endif

#if 0
	/* check for well-know bitrate values */
	{
		struct {
			u32 bitrate;
			u16 btr0btr1;
			u32 ccbt;
		} timing[] = {
			{ 500000,  0x001c, 0x1c0006 },
			{ 1000000, 0x0014, 0x140006 },
			{ 33300,   0x1d14, 0x1400d1 },
			{ 0, }
		};
		int i;
		for (i=0; timing[i].bitrate != 0; i++) {
			u32 tmp = pcan_usbpro_convert_16MHzBTR0BTR1(timing[i].btr0btr1);
			if (tmp != timing[i].ccbt) {
				printk(KERN_ERR
				       "%s: BTR0BTR1=%04X CCBT=%08X!=%08X\n",
				       DEVICE_NAME, timing[i].btr0btr1, tmp,
				       timing[i].ccbt);
				printk(KERN_ERR
				       "%s: transfer at %u bps won't work! "
				       "(MHz=%u)\n",
				       DEVICE_NAME, timing[i].bitrate, MHz);
			}
		}
	}

#endif

#ifdef PCAN_USBPRO_DRIVER_CALIBRATION_PERIOD
	setup_timer(&usb_if->calibration_timer,
	            pcan_usbpro_calibration_timeout, (unsigned long)usb_if);
#else
	/* request for hardware periodic calibration messages is done now when
	 * last controller is initialized */
	//pcan_usbpro_calibration_request_async(usb_if, 1);
#endif

	/* Tell module the CAN driver is loaded */
	pcan_usbpro_driver_loaded(usb_if, 0, 1);

	return 0;
}
#endif /* USB_SUPPORT */
