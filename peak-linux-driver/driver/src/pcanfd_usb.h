/*****************************************************************************
 * Copyright (C) 2001-2007	PEAK System-Technik GmbH
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
 * pcan_usbpro_fd.h - the inner usb parts header for PCAN-USB Pro FD support 
 * 
 * $Id: pcan_usbpro_fd.h 615 2010-02-14 22:38:55Z khitschler $ 
 * 
 *****************************************************************************/

#ifndef __PCAN_USBFD_H__
#define __PCAN_USBFD_H__

/*
 * INCLUDES
 */
#include <linux/types.h>
#include <linux/usb.h>

#include "src/pcan_main.h"
#include "src/pcanfd_ucan.h"

/*
 * DEFINES
 */
#define PCAN_USBPROFD_PRODUCT_ID	0x0011
#define PCAN_USBFD_PRODUCT_ID		0x0012
#define PCAN_USBCHIP_PRODUCT_ID		0x0013
#define PCAN_USBX6_PRODUCT_ID		0x0014

#ifdef LINUX_26
#define __usb_submit_urb(x)	usb_submit_urb(x, GFP_ATOMIC)
#define __usb_alloc_urb(x)	usb_alloc_urb(x, GFP_ATOMIC)
#define FILL_BULK_URB(a, b, c, d, e, f, g)	\
		usb_fill_bulk_urb(a, b, c, d, e, (usb_complete_t)f, (void *)g)
#else
#define __usb_submit_urb(x)	usb_submit_urb(x)
#define __usb_alloc_urb(x)	usb_alloc_urb(x)
#endif

/*
 * External API
 */
#ifdef __cplusplus__
extern "C" {
#endif

int pcan_usbfd_init(struct pcan_usb_interface *);

#ifdef __cplusplus__
};
#endif

#endif
