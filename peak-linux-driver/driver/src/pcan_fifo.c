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
 * Contributions: John Privitera  (JohnPrivitera@dciautomation.com)
 *****************************************************************************/
/* #define DEBUG */

/*
 * pcan_fifo.c - manages the ring buffers for read and write data
 *
 * $Id$
 */
#include "src/pcan_common.h"

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/sched.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
#include <asm/system.h>     /* cli(), save_flags(), restore_flags() */
#endif

#include "src/pcan_fifo.h"

int pcan_fifo_reset(FIFO_MANAGER *anchor)
{
	pcan_lock_irqsave_ctxt lck_ctx;

	pcan_lock_get_irqsave(&anchor->lock, lck_ctx);

	anchor->dwTotal = 0;
	anchor->nStored = 0;
	anchor->r = anchor->w = anchor->bufferBegin;

	pcan_lock_put_irqrestore(&anchor->lock, lck_ctx);

	DPRINTK(KERN_DEBUG "%s: %s() %d %p %p\n",
		DEVICE_NAME, __func__, anchor->nStored, anchor->r, anchor->w);

	return 0;
}

int pcan_fifo_init(FIFO_MANAGER *anchor, void *bufferBegin,
		   void *bufferEnd, int nCount, u16 wCopySize)
{
	anchor->wStepSize = (bufferBegin == bufferEnd) ? 0 : \
			    ((bufferEnd - bufferBegin) / (nCount - 1));

	/* check for fatal program errors */
	if ((anchor->wStepSize < wCopySize) ||
				(bufferBegin > bufferEnd) || (nCount <= 1))
		return -EINVAL;

	anchor->wCopySize = wCopySize;
	anchor->nCount = nCount;

	anchor->bufferBegin = bufferBegin;
	anchor->bufferEnd = bufferEnd;

	pcan_lock_init(&anchor->lock);

	return pcan_fifo_reset(anchor);
}

int pcan_fifo_put(FIFO_MANAGER *anchor, void *pvPutData)
{
	pcan_lock_irqsave_ctxt lck_ctx;
	int err;

#ifdef DEBUG
	printk(KERN_DEBUG "%s: %s() %d %p %p\n",
		DEVICE_NAME, __func__, anchor->nStored, anchor->r, anchor->w);
#endif

	pcan_lock_get_irqsave(&anchor->lock, lck_ctx);

	if (anchor->nStored < anchor->nCount) {
		memcpy(anchor->w, pvPutData, anchor->wCopySize);

		err = anchor->nStored++;
		anchor->dwTotal++;

		if (anchor->w < anchor->bufferEnd)
			anchor->w += anchor->wStepSize;
		else
			anchor->w = anchor->bufferBegin;
	} else
		err = -ENOSPC;

	pcan_lock_put_irqrestore(&anchor->lock, lck_ctx);

	return err;
}

int pcan_fifo_get(FIFO_MANAGER *anchor, void *pvGetData)
{
	int err = 0;
	pcan_lock_irqsave_ctxt lck_ctx;

#ifdef DEBUG
	printk(KERN_DEBUG "%s: %s() %d %p %p\n",
		DEVICE_NAME, __func__, anchor->nStored, anchor->r, anchor->w);
#endif

	pcan_lock_get_irqsave(&anchor->lock, lck_ctx);

	if (anchor->nStored > 0) {
		if (pvGetData)
			memcpy(pvGetData, anchor->r, anchor->wCopySize);

		anchor->nStored--;
		if (anchor->r < anchor->bufferEnd)
			anchor->r += anchor->wStepSize;
		else
			anchor->r = anchor->bufferBegin;
	} else
		err = -ENODATA;

	pcan_lock_put_irqrestore(&anchor->lock, lck_ctx);

	return err;
}

int pcan_fifo_foreach_back(FIFO_MANAGER *anchor,
			int (*pf)(void *item, void *arg), void *arg)
{
	u32 i;
	void *p;
	int err = 0;
	pcan_lock_irqsave_ctxt lck_ctx;

#ifdef DEBUG
	printk(KERN_DEBUG "%s: %s() %d %p %p\n",
		DEVICE_NAME, __func__, anchor->nStored, anchor->r, anchor->w);
#endif

	pcan_lock_get_irqsave(&anchor->lock, lck_ctx);

	p = anchor->w;
	for (i = 0; i < anchor->nStored; i++) {

		if (p == anchor->bufferBegin)
			p = anchor->bufferEnd;
		else
			p -= anchor->wStepSize;

		err = pf(p, arg);
		if (err)
			break;
	}

	pcan_lock_put_irqrestore(&anchor->lock, lck_ctx);

	return err;
}

int pcan_fifo_peek(FIFO_MANAGER *anchor, void *pvGetData)
{
	int err = 0;
	pcan_lock_irqsave_ctxt lck_ctx;

#ifdef DEBUG
	printk(KERN_DEBUG "%s: %s()) %d %p %p\n",
		DEVICE_NAME, __func__, anchor->nStored, anchor->r, anchor->w);
#endif

	pcan_lock_get_irqsave(&anchor->lock, lck_ctx);

	if (anchor->nStored > 0)
		memcpy(pvGetData, anchor->r, anchor->wCopySize);
	else
		err = -ENODATA;

	pcan_lock_put_irqrestore(&anchor->lock, lck_ctx);

	return err;
}

u32 pcan_fifo_ratio(FIFO_MANAGER *anchor)
{
	return anchor->nCount ? (anchor->nStored * 10000) / anchor->nCount : 0;
}

#if 0
int pcan_fifo_not_full(FIFO_MANAGER *anchor)
{
#ifdef PCAN_FIX_NOT_FULL_TEST_LOCK_USAGE
	int r;
	pcan_lock_irqsave_ctxt lck_ctx;

	pcan_lock_get_irqsave(&anchor->lock, lck_ctx);

	r = (anchor->nStored < anchor->nCount);

	pcan_lock_put_irqrestore(&anchor->lock, lck_ctx);

	return r;
#else
	return (anchor->nStored < (anchor->nCount - 1));
#endif
}

/* returns true if fifo is empty */
int pcan_fifo_empty(FIFO_MANAGER *anchor)
{
	return !anchor->nStored;
}
#endif
