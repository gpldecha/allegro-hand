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
 * Contributions: Philipp Baer (philipp.baer@informatik.uni-ulm.de)
 *****************************************************************************/

/*****************************************************************************
 *
 * all parts of the isa hardware for pcan-isa devices
 *
 * $Id$
 *
 *****************************************************************************/
/* #define DEBUG */

#include "src/pcan_common.h"	/* must always be the 1st include */

#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <asm/io.h>

#include "src/pcan_isa.h"
#include "src/pcan_sja1000.h"
#include "src/pcan_filter.h"

#define ISA_ISR_LOOP_MAX	100

#define ISA_PORT_SIZE		0x20	/* address range of the isa-port, */
					/* enough for PeliCAN mode */
#define ISA_DEFAULT_COUNT	2	/* count of defaults for init */

static u16 isa_ports[] = {0x300, 0x320};	/* def values for pcan-isa */
static u8 isa_irqs[] = {10, 5};

static struct pcan_adapter isa_adapter = {
	.name = "PCAN-ISA",
	.hw_ver_major = -1,
};

#ifdef PCAN_HANDLE_IRQ_SHARING
static inline void init_same_irq_list(struct pcandev *dev)
{
	DPRINTK(KERN_DEBUG "%s: %s(%p\n", DEVICE_NAME, __func__, dev);

	INIT_LIST_HEAD(&dev->port.isa.anchor.same_irq_items);
	dev->port.isa.anchor.same_irq_count = 0;
	dev->port.isa.anchor.same_irq_active = 0;
	dev->port.isa.same.dev = dev;
	dev->port.isa.my_anchor = NULL;
}

/* create lists of devices with the same irq - base for shared irq handling */
void pcan_create_isa_shared_irq_lists(void)
{
	struct pcandev *od, *id;
	struct list_head *op, *ip;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	/* loop over all devices for a ISA port with same irq level and not
	 * myself */
	for (op = pcan_drv.devices.next; op != &pcan_drv.devices;
							op = op->next) {
		od = (struct pcandev *)op;

		/* if it is a ISA device and still not associated */
		if ((od->wType == HW_ISA_SJA) && (!od->port.isa.my_anchor)) {

			od->port.isa.my_anchor = &od->port.isa.anchor;

			/* I'm the first and - maybe - the only one */
			od->port.isa.my_anchor->same_irq_count++;
			list_add_tail(&od->port.isa.same.item,
				&od->port.isa.my_anchor->same_irq_items);

			DPRINTK(KERN_DEBUG "%s: main Irq=%d dev=%p count=%d\n",
				DEVICE_NAME, od->wIrq, od,
				od->port.isa.my_anchor->same_irq_count);

			/* now search for other devices with the same irq */
			for (ip = op->next; ip != &pcan_drv.devices;
								ip = ip->next) {
				id = (struct pcandev *)ip;

				/* if it is a ISA device and the irq level is
				 * the same and it is still not associated */
				if ((id->wType == HW_ISA_SJA) &&
					(!id->port.isa.my_anchor) &&
					(id->wIrq == od->wIrq)) {
					/* point and associate to the first
					 * with the same irq level */
					id->port.isa.my_anchor =
						od->port.isa.my_anchor;
					id->port.isa.my_anchor->same_irq_count++;
					list_add_tail(&id->port.isa.same.item,
						&id->port.isa.my_anchor->same_irq_items);

					DPRINTK(KERN_DEBUG "%s: sub Irq=%d "
						"dev=%p, count=%d\n",
						DEVICE_NAME,
						id->wIrq, id,
						id->port.isa.my_anchor->same_irq_count);
				}
			}
		}
	}
}

#ifndef NO_RT
static int pcan_isa_irqhandler(rtdm_irq_t *irq_context)
{
	SAME_IRQ_LIST *my_anchor = rtdm_irq_get_arg(irq_context, SAME_IRQ_LIST);

#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
/* only one irq-handler per irq level for ISA shared interrupts */
static irqreturn_t pcan_isa_irqhandler(int irq, void *dev_id,
							struct pt_regs *regs)
{
	SAME_IRQ_LIST *my_anchor = (SAME_IRQ_LIST *)dev_id;
#else
static irqreturn_t pcan_isa_irqhandler(int irq, void *dev_id)
{
	/* loop the list of irq-handlers for all devices with the same
	 * irq-level until at least all devices are one time idle. */
	SAME_IRQ_LIST *my_anchor = (SAME_IRQ_LIST *)dev_id;
#endif
	struct list_head *ptr;
	struct pcandev *dev;
	int ret = 0;
	u16 loop_count = 0;
	u16 stop_count = ISA_ISR_LOOP_MAX;

#if 0
	DPRINTK(KERN_DEBUG "%s: %s(%p)\n", DEVICE_NAME, __func__, my_anchor);
#endif

	/* loop over all ISA devices with same irq level */
	for (ptr = my_anchor->same_irq_items.next;
			loop_count < my_anchor->same_irq_count;
							ptr = ptr->next) {
		if (ptr != &my_anchor->same_irq_items) {
			dev = ((SAME_IRQ_ITEM *)ptr)->dev;

			/* DPRINTK(KERN_DEBUG "%s: dev=%p\n", DEVICE_NAME, dev);
			 */
			if (!pcan_sja1000_irqhandler(dev))
				loop_count++;
			else {
				ret = 1;
				/* reset, I need at least
				 * my_anchor->same_irq_count loops without a
				 * pending request */
				loop_count = 0;
			}

			if (!stop_count--) {
#ifdef DEBUG
				printk(KERN_INFO
					"%s: ISA ISR stopped after %u loops\n",
					DEVICE_NAME, ISA_ISR_LOOP_MAX);
#endif
				break;
			}
		}
	}

	return PCAN_IRQ_RETVAL(ret);
}

static int pcan_isa_req_irq(struct pcandev *dev, struct pcan_udata *dev_priv)
{
	int err;

	DPRINTK(KERN_DEBUG "%s: %s(%p)\n", DEVICE_NAME, __func__, dev);

	/* the first device */
	if (!dev->port.isa.my_anchor->same_irq_active) {
#ifndef NO_RT
		err = rtdm_irq_request(&dev->irq_handle,
				dev->wIrq,
				pcan_isa_irqhandler,
				RTDM_IRQTYPE_SHARED | RTDM_IRQTYPE_EDGE,
#else
		err = request_irq(dev->wIrq,
				pcan_isa_irqhandler,
				0,
#endif
				DEVICE_NAME,
				dev->port.isa.my_anchor);
		if (err)
			return err;

		/* count all ISA devices with same irq */
		dev->port.isa.my_anchor->same_irq_active++;
	}

	dev->wInitStep++;

	return 0;
}

static void pcan_isa_free_irq(struct pcandev *dev, struct pcan_udata *dev_priv)
{
	DPRINTK(KERN_DEBUG "%s: %s(%p) L=%u\n",
			DEVICE_NAME, __func__, dev, __LINE__);

	dev->port.isa.my_anchor->same_irq_active--;

	/* the last device */
	if (!dev->port.isa.my_anchor->same_irq_active)
#ifdef NO_RT
		free_irq(dev->wIrq, dev->port.isa.my_anchor);
#else
		rtdm_irq_free(&dev->irq_handle);
#endif
	dev->wInitStep = 3;
}

#else	/* PCAN_HANDLE_IRQ_SHARING */

#ifndef NO_RT
static int pcan_isa_irqhandler(rtdm_irq_t *irq_context)
{
	struct pcandev *dev = rtdm_irq_get_arg(irq_context, struct pcandev);

#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
static irqreturn_t pcan_isa_irqhandler(int irq, void *arg, struct pt_regs *regs)
{
	struct pcandev *dev = (struct pcandev *)arg;
#else
static irqreturn_t pcan_isa_irqhandler(int irq, void *arg)
{
	struct pcandev *dev = (struct pcandev *)arg;
#endif
	int err;

	err = pcan_sja1000_irqhandler(dev);

	return PCAN_IRQ_RETVAL(err);
}

static int pcan_isa_req_irq(struct pcandev *dev, struct pcan_udata *dev_priv)
{
	int err;

	DPRINTK(KERN_DEBUG "%s: %s(%p)\n", DEVICE_NAME, __func__, dev);

#ifdef NO_RT
	err = request_irq(dev->wIrq,
				pcan_isa_irqhandler,
				IRQF_SHARED,
#else
	err = rtdm_irq_request(&dev->irq_handle,
				dev->wIrq,
				pcan_isa_irqhandler,
				RTDM_IRQTYPE_SHARED | RTDM_IRQTYPE_EDGE,
#endif
				DEVICE_NAME,
				dev);
	if (err) {
		pr_err("%s: failed requesting isa IRQ %u (err %d)\n",
			DEVICE_NAME, dev->wIrq, err);
		return err;
	}

	dev->wInitStep++;

	return 0;
}

static void pcan_isa_free_irq(struct pcandev *dev, struct pcan_udata *dev_priv)
{
	DPRINTK(KERN_DEBUG "%s: %s(%p) L=%u\n",
			DEVICE_NAME, __func__, dev, __LINE__);
#ifdef NO_RT
	free_irq(dev->wIrq, dev);
#else
	rtdm_irq_free(&dev->irq_handle);
#endif
	dev->wInitStep = 3;
}

#endif	/* PCAN_HANDLE_IRQ_SHARING */

/* read a register */
static u8 pcan_isa_readreg(struct pcandev *dev, u8 port)
{
	return inb(dev->dwPort + port);
}

/* write a register */
static void pcan_isa_writereg(struct pcandev *dev, u8 port, u8 data)
{
	outb(data, dev->dwPort + port);
}

/* release and probe function */
static int pcan_isa_cleanup(struct pcandev *dev)
{
	DPRINTK(KERN_DEBUG "%s: %s(): wInitStep=%u\n",
		DEVICE_NAME, __func__, dev->wInitStep);

	switch(dev->wInitStep) {
	case 4:
		pcan_isa_free_irq(dev, NULL);
	case 3:
		//isa_devices--;
		isa_adapter.can_count--;
	case 2:
	case 1:
		release_region(dev->dwPort, ISA_PORT_SIZE);
	case 0:
		dev->filter = pcan_delete_filter_chain(dev->filter);
	}

	return 0;
}

/* interface depended open and close */
static int pcan_isa_open(struct pcandev *dev)
{
	return 0;
}

static int pcan_isa_release(struct pcandev *dev)
{
	return 0;
}

static int pcan_isa_init(struct pcandev *dev, u32 dwPort, u16 wIrq)
{
	int err = 0;

	DPRINTK(KERN_DEBUG "%s: %s(), isa_devices = %d\n",
		DEVICE_NAME, __func__, isa_adapter.can_count);

	/* set this before any instructions, fill struct pcandev, part 1 */
	dev->wInitStep   = 0;
	dev->readreg     = pcan_isa_readreg;
	dev->writereg    = pcan_isa_writereg;
	dev->cleanup     = pcan_isa_cleanup;
	dev->req_irq     = pcan_isa_req_irq;
	dev->free_irq    = pcan_isa_free_irq;
	dev->open        = pcan_isa_open;
	dev->release     = pcan_isa_release;
	dev->filter      = pcan_create_filter_chain();

	/* reject illegal combination */
	if ((!dwPort && wIrq) || (dwPort && !wIrq))
		return -EINVAL;

	/* a default is requested */
	if (!dwPort) {
		/* there's no default available */
		if (isa_adapter.can_count >= ISA_DEFAULT_COUNT)
			return -ENODEV;

		dev->dwPort = isa_ports[isa_adapter.can_count];
		dev->wIrq = isa_irqs[isa_adapter.can_count];
	} else {
		dev->dwPort = dwPort;
		dev->wIrq = wIrq;
	}

	dev->adapter = &isa_adapter;
	dev->nMajor = pcan_drv.nMajor;
	dev->nMinor = ISA_MINOR_BASE + isa_adapter.can_count;

	/* request address range reservation */
	err = ___request_region(dev->dwPort, ISA_PORT_SIZE, DEVICE_NAME);
	if (err)
		return err;

	dev->wInitStep = 2;

#ifdef PCAN_HANDLE_IRQ_SHARING
	init_same_irq_list(dev);
#endif
	isa_adapter.can_count++;

	dev->wInitStep = 3;

	printk(KERN_INFO
		"%s: isa SJA1000 device minor %d expected (io=0x%04x,irq=%d)\n",
		DEVICE_NAME, dev->nMinor, dev->dwPort,
		dev->wIrq);

	return 0;
}

/* create all declared isa devices */
int pcan_create_isa_devices(char* type, u32 io, u16 irq)
{
	struct pcandev *dev;
	int err;

	/* create isa devices */
	DPRINTK(KERN_DEBUG "%s: %s(0x%x, %d)\n",
		DEVICE_NAME, __func__, io, irq);

	dev = (struct pcandev *)pcan_malloc(sizeof(struct pcandev), GFP_KERNEL);
	if (!dev) {
		err = -ENOMEM;
		goto fail;
	}

	dev->nChannel = isa_adapter.can_count;

	pcan_soft_init(dev, type, HW_ISA_SJA);

	dev->device_open = sja1000_open;
	dev->device_write = sja1000_write;
	dev->device_release = sja1000_release;

	err = pcan_isa_init(dev, io, irq);
	if (!err)
		err = sja1000_probe(dev);

	if (err) {
		dev->cleanup(dev);
		pcan_free(dev);
		goto fail;
	}

	dev->ucPhysicallyInstalled = 1;

	/* add this device to the list */
	pcan_add_device_in_list(dev);

	return 0;

fail:
	printk(KERN_ERR "%s: isa CAN device creation failed (err %d)\n",
			DEVICE_NAME, err);

	return err;
}
