/*****************************************************************************
 * Copyright (C) 2001-2008 PEAK System-Technik GmbH
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
 *****************************************************************************/

/****************************************************************************
 *
 * all parts to handle the interface specific parts of pcan-dongle
 *
 * $Id$
 *
 *****************************************************************************/
//#define DEBUG

#include "src/pcan_common.h"	/* must always be the 1st include */

#include <linux/errno.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <linux/sched.h>
#include <linux/slab.h>
#ifdef PARPORT_SUBSYSTEM
#include <linux/parport.h>
#endif
#include <linux/delay.h>

#include "src/pcan_dongle.h"
#include "src/pcan_sja1000.h"
#include "src/pcan_filter.h"

#define DNG_PORT_SIZE		4	/* address range of the dongle-port */
#define ECR_PORT_SIZE		1	/* size of the associated ECR reg */
#define DNG_DEFAULT_COUNT	4	/* count of defaults for init */

/*
 * When isnmod pcan.ko fails because of:
 *
 * [ 3835.317426] pcan: failed to claim port 0x0378 (err -16)
 * 
 * Then, look at dmesg "parport" info msgs:
 *
 * [    9.471830] systemd-udevd[304]: starting version 204
 * [   10.686614] lp: driver loaded but no devices found
 * [   10.778588] ppdev: user-space parallel port driver
 * [   10.817385] parport_pc 00:05: reported by Plug and Play ACPI
 * [   10.817489] parport0: PC-style at 0x378 (0x778), irq 7, dma 3 [PCSPP,TRISTATE,COMPAT,EPP,ECP,DMA]
 * [   10.912218] lp0: using parport0 (interrupt-driven).
 *
#ifndef PARPORT_SUBSYSTEM
 * $ sudo rmmod parport
 * rmmod: ERROR: Module parport is in use by: lp ppdev parport_pc
 * $ sudo rmmod lp ppdev parport_pc
 * $ sudo rmmod parport
#endif
 */
static u16 dng_ports[] = {0x378, 0x278, 0x3bc, 0x2bc};
static u8  dng_irqs[]  = {7, 5, 7, 5};

static struct pcan_adapter dng_adapter = {
	.name = "PCAN-IDongle",
	.hw_ver_major = -1,
};

static u16 epp_devices = 0;		/* ... epp_devices */
static u16 sp_devices  = 0;		/* ... sp_devices */

static unsigned char nibble_decode[32] = {
	0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf,
	0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf,
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7,
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7
};

/* enable and disable irqs */
static void pcan_dongle_irq_disable(struct pcandev *dev)
{
	u16 pc = (u16)dev->dwPort + 2;
	outb(inb(pc) & ~0x10, pc);
}

static void pcan_dongle_irq_enable(struct pcandev *dev)
{
	u16 pc = (u16)dev->dwPort + 2;
	outb(inb(pc) | 0x10, pc);
}

/* functions for SP port */

/* read a register */
static u8 pcan_dongle_sp_readreg(struct pcandev *dev, u8 port)
{
	u16 pa = (u16)dev->dwPort;
	u16 pb = pa + 1;
	u16 pc = pb + 1;
	u8  b0, b1;
	u8  irqEnable = inb(pc) & 0x10; /* don't influence irqEnable */
	pcan_lock_irqsave_ctxt lck_ctx;

	pcan_lock_get_irqsave(&dev->port.dng.lock, lck_ctx);

	outb((0x0B ^ 0x0D) | irqEnable, pc);
	outb((port & 0x1F) | 0x80,      pa);
	outb((0x0B ^ 0x0C) | irqEnable, pc);
	b1 = nibble_decode[inb(pb) >> 3];
	outb(0x40, pa);
	b0 = nibble_decode[inb(pb) >> 3];
	outb((0x0B ^ 0x0D) | irqEnable, pc);

	pcan_lock_put_irqrestore(&dev->port.dng.lock, lck_ctx);

	return (b1 << 4) | b0;
}

/* write a register */
static void pcan_dongle_writereg(struct pcandev *dev, u8 port, u8 data)
{
	u16 pa = (u16)dev->dwPort;
	u16 pc = pa + 2;
	u8  irqEnable = inb(pc) & 0x10; /* don't influence irqEnable */
	pcan_lock_irqsave_ctxt lck_ctx;

	pcan_lock_get_irqsave(&dev->port.dng.lock, lck_ctx);

	outb((0x0B ^ 0x0D) | irqEnable, pc);
	outb(port & 0x1F,               pa);
	outb((0x0B ^ 0x0C) | irqEnable, pc);
	outb(data,                      pa);
	outb((0x0B ^ 0x0D) | irqEnable, pc);

	pcan_lock_put_irqrestore(&dev->port.dng.lock, lck_ctx);
}

/* functions for EPP port */

/* read a register */
static u8 pcan_dongle_epp_readreg(struct pcandev *dev, u8 port)
{
	u16 pa = (u16)dev->dwPort;
	u16 pc = pa + 2;
	u8 wert;
	u8 irqEnable = inb(pc) & 0x10; /* don't influence irqEnable */
	pcan_lock_irqsave_ctxt lck_ctx;

	pcan_lock_get_irqsave(&dev->port.dng.lock, lck_ctx);

	outb((0x0B ^ 0x0F) | irqEnable, pc);
	outb((port & 0x1F) | 0x80,      pa);
	outb((0x0B ^ 0x2E) | irqEnable, pc);
	wert = inb(pa);
	outb((0x0B ^ 0x0F) | irqEnable, pc);

	pcan_lock_put_irqrestore(&dev->port.dng.lock, lck_ctx);

	return wert;
}

static void pcan_dongle_free_irq(struct pcandev *dev,
						struct pcan_udata *dev_priv)
{
#ifndef NO_RT
	rtdm_irq_free(&dev->irq_handle);
#elif !defined(PARPORT_SUBSYSTEM)
	free_irq(dev->wIrq, dev);
#endif
	dev->wInitStep--;
}

/* release and probe functions */
static int pcan_dongle_cleanup(struct pcandev *dev)
{
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	switch (dev->wInitStep) {
	case 4:
		pcan_dongle_free_irq(dev, NULL);
	case 3:
		if (dev->wType == HW_DONGLE_SJA)
			sp_devices--;
		else
			epp_devices--;

		dng_adapter.can_count = sp_devices + epp_devices;
	case 2:
#ifdef PARPORT_SUBSYSTEM
	case 1:
#ifdef NO_RT
		parport_unregister_device(dev->port.dng.pardev);
#endif

#else
		if (dev->wType == HW_DONGLE_SJA_EPP)
			release_region(dev->port.dng.wEcr, ECR_PORT_SIZE);
	case 1:
		release_region(dev->dwPort, DNG_PORT_SIZE);
#endif
	case 0:
		dev->filter = pcan_delete_filter_chain(dev->filter);
	}

	return 0;
}

/* to switch epp on or restore register */
static void pcan_dongle_set_ecr(struct pcandev *dev)
{
	u16 wEcr = dev->port.dng.wEcr;

	dev->port.dng.ucOldECRContent = inb(wEcr);
	outb((dev->port.dng.ucOldECRContent & 0x1F) | 0x20, wEcr);

	if (dev->port.dng.ucOldECRContent == 0xff)
		printk(KERN_DEBUG "%s: really ECP mode configured?\n",
			DEVICE_NAME);
}

static void pcan_dongle_restore_ecr(struct pcandev *dev)
{
	u16 wEcr = dev->port.dng.wEcr;

	outb(dev->port.dng.ucOldECRContent, wEcr);

	DPRINTK(KERN_DEBUG "%s: restore ECR\n", DEVICE_NAME);
}

#if defined(NO_RT) && defined(PARPORT_SUBSYSTEM)
static void pcan_parport_irq_handler(void *arg)
{
	pcan_sja1000_irqhandler((struct pcandev *)arg);
}
#endif

static int pcan_dongle_probe(struct pcandev *dev) /* probe for type */
{
#ifdef PARPORT_SUBSYSTEM
	struct parport *p;

	DPRINTK(KERN_DEBUG "%s: %s() - PARPORT_SUBSYSTEM\n",
		DEVICE_NAME, __func__);

	/* probe does not probe for the sja1000 device here - this is done at
	 * sja1000_open() */
	p = parport_find_base(dev->dwPort);
	if (!p) {
		pr_err("%s: found no parport\n", DEVICE_NAME);
		return -ENXIO;
	}

#ifdef NO_RT
	/* register my device at the parport in no realtime */
	dev->port.dng.pardev = parport_register_device(p, DEVICE_NAME,
					NULL, NULL,
					pcan_parport_irq_handler, 0, dev);
	if (!dev->port.dng.pardev) {
		pr_err("%s: found no parport device\n", DEVICE_NAME);
		return -ENODEV;
	}
#endif

	return 0;

#else /* PARPORT_SUBSYSTEM */

	int err;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	err = ___request_region(dev->dwPort, DNG_PORT_SIZE,
							DEVICE_NAME);
	if (!err) {
		dev->wInitStep = 1;

		if (dev->wType == HW_DONGLE_SJA_EPP) {
			err = ___request_region(dev->port.dng.wEcr,
						ECR_PORT_SIZE, DEVICE_NAME);
			if (!err)
				dev->wInitStep = 2;
		}
	}

	return err;
#endif /* PARPORT_SUBSYSTEM */
}

/* interface depended open and close */
static int pcan_dongle_open(struct pcandev *dev)
{
	int err = 0;
	u16 wPort;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

#if defined(NO_RT) && defined(PARPORT_SUBSYSTEM)
	err = parport_claim(dev->port.dng.pardev);
	if (err) {
		pr_err("%s: can't claim parport (err %d)\n", DEVICE_NAME, err);
		return err;
	}

	if (dev->port.dng.pardev->port->irq == PARPORT_IRQ_NONE) {
		pr_err("%s: no irq associated to parport.\n", DEVICE_NAME);
		return -ENXIO;
	}
#endif

	/* save port state */
	wPort = (u16)dev->dwPort;

	/* save old port contents */
	dev->port.dng.ucOldDataContent = inb(wPort);
	dev->port.dng.ucOldControlContent = inb(wPort + 2);

	/* switch to epp mode if possible */
	if (dev->wType == HW_DONGLE_SJA_EPP)
		pcan_dongle_set_ecr(dev);

	/* enable irqs */
#ifdef PARPORT_SUBSYSTEM
	/* parport_enable_irq(dev->port.dng.pardev->port); not working
	 * since 2.4.18 */
#endif
	pcan_dongle_irq_enable(dev);

	return err;
}

static int pcan_dongle_release(struct pcandev *dev)
{
	u16 wPort = (u16)dev->dwPort;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	/* disable irqs */
#ifdef PARPORT_SUBSYSTEM
	/* parport_disable_irq(dev->port.dng.pardev->port); not working since
	 * 2.4.18 */
#endif
	pcan_dongle_irq_disable(dev);

	if (dev->wType == HW_DONGLE_SJA_EPP)
		pcan_dongle_restore_ecr(dev);

	/* restore port state */
	outb(dev->port.dng.ucOldDataContent, wPort);
	outb(dev->port.dng.ucOldControlContent, wPort + 2);

#if defined(NO_RT) && defined(PARPORT_SUBSYSTEM)
	parport_release(dev->port.dng.pardev);
#endif

	return 0;
}

static int pcan_dongle_req_irq(struct pcandev *dev, struct pcan_udata *ctx)
{
	int err = 0;

#ifndef NO_RT
	err = rtdm_irq_request(&dev->irq_handle,
				dev->wIrq,
				sja1000_irqhandler,
				RTDM_IRQTYPE_SHARED | RTDM_IRQTYPE_EDGE,
				DEVICE_NAME, dev);

#elif !defined(PARPORT_SUBSYSTEM)
	err = request_irq(dev->wIrq, sja1000_irqhandler,
				IRQF_SHARED, DEVICE_NAME, dev);
#endif

	if (err) {
		pr_err("%s: failed requesting IRQ %u (err %d)\n",
			DEVICE_NAME, dev->wIrq, err);
		return err;
	}

	dev->wInitStep++;
	return 0;
}

static int pcan_dongle_init(struct pcandev *dev, u32 dwPort, u16 wIrq,
								char *type)
{
	int err;

	DPRINTK(KERN_DEBUG "%s: %s(), dng_devices = %d\n",
		DEVICE_NAME, __func__, dng_adapter.can_count);

	/* set this before any instructions, fill struct pcandev, part 1 */
	dev->wInitStep   = 0;
	dev->cleanup     = pcan_dongle_cleanup;
	dev->req_irq     = pcan_dongle_req_irq;
	dev->free_irq    = pcan_dongle_free_irq;
	dev->open        = pcan_dongle_open;
	dev->release     = pcan_dongle_release;
	dev->filter      = pcan_create_filter_chain();

	pcan_lock_init(&dev->port.dng.lock);

	/* fill struct pcandev, 1st check if a default is set */
	if (!dwPort) {
		/* there's no default available */
		if (dng_adapter.can_count >= DNG_DEFAULT_COUNT)
			return -ENODEV;

		dev->dwPort = dng_ports[dng_adapter.can_count];
	} else {
		dev->dwPort = dwPort;
	}

	if (!wIrq) {
		if (dng_adapter.can_count >= DNG_DEFAULT_COUNT)
			return -ENODEV;

		dev->wIrq = dng_irqs[dng_adapter.can_count];
	} else {
		dev->wIrq = wIrq;
	}

	dev->adapter = &dng_adapter;
	dev->nMajor = pcan_drv.nMajor;
	if (dev->wType == HW_DONGLE_SJA) {
		dev->nMinor        = PCAN_DNG_SP_MINOR_BASE + sp_devices;
		dev->readreg       = pcan_dongle_sp_readreg;
		dev->writereg      = pcan_dongle_writereg;
		dev->port.dng.wEcr = 0; /* set to anything */
	} else {
		dev->nMinor        = PCAN_DNG_EPP_MINOR_BASE + epp_devices;
		dev->readreg       = pcan_dongle_epp_readreg;
		dev->writereg      = pcan_dongle_writereg;
		dev->port.dng.wEcr = (u16)dev->dwPort + 0x402;
	}

	/* is the device really available? */
	err = pcan_dongle_probe(dev);
	if (err < 0) {
		printk(KERN_ERR "%s: failed to claim port 0x%04x (err %d)\n",
			DEVICE_NAME, dev->dwPort, err);
		return err;
	}

	dev->ucPhysicallyInstalled = 1;

	if (dev->wType == HW_DONGLE_SJA)
		sp_devices++;
	else
		epp_devices++;

	dng_adapter.can_count = sp_devices + epp_devices;

	dev->wInitStep = 3;

	printk(KERN_INFO
		"%s: %s-dongle device minor %d prepared (io=0x%04x irq=%d)\n",
		DEVICE_NAME, dev->type, dev->nMinor, dev->dwPort,
		dev->wIrq);

	return 0;
}

int pcan_create_dongle_devices(char *type, u32 io, u16 irq)
{
	struct pcandev *dev;
	int err;

	DPRINTK(KERN_DEBUG "%s: %s(%s, 0x%x, %d)\n",
		DEVICE_NAME, __func__, type, io, irq);

	dev = (struct pcandev *)pcan_malloc(sizeof(struct pcandev), GFP_KERNEL);
	if (!dev) {
		err = -ENOMEM;
		goto fail;
	}

	pcan_soft_init(dev, type,
		(!strncmp(type, "sp", 4)) ? HW_DONGLE_SJA : HW_DONGLE_SJA_EPP);

	dev->device_open = sja1000_open;
	dev->device_write = sja1000_write;
	dev->device_release = sja1000_release;

	err = pcan_dongle_init(dev, io, irq, type);
	if (err) {
		dev->cleanup(dev);
		goto fail_free;
	}

	/* add this device to the list */
	pcan_add_device_in_list(dev);

	return 0;

fail_free:
	pcan_free(dev);
fail:
	printk(KERN_ERR "%s: %s() failed! (err %d)\n",
		DEVICE_NAME, __func__, err);

	return err;
}
