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
 *****************************************************************************/

/*
 * pcan_fifo.h - all about fifo buffer management
 *
 * $Id$
 */
#ifndef __PCAN_FIFO_H__
#define __PCAN_FIFO_H__

#include "src/pcan_common.h"

typedef struct {
	u16	wStepSize;	/* size of bytes to step to next entry */
	u16	wCopySize;	/* size of bytes to copy */
	void *	bufferBegin;	/* points to first element */
	void *	bufferEnd;	/* points to the last element */
	u32	nCount;		/* max count of elements in fifo */
	u32	nStored;	/* count of currently rcvd stored msgs */
	u32	dwTotal;	/* received messages */
	void *	r;		/* next Msg to read from the read buffer */
	void *	w;		/* next Msg to write into read buffer */
	pcan_lock_t lock;	/* mutual exclusion lock (if not NULL) */
} FIFO_MANAGER;

static inline int pcan_fifo_empty(FIFO_MANAGER *anchor)
{
	return !anchor->nStored;
}

static inline int pcan_fifo_full(FIFO_MANAGER *anchor)
{
	return anchor->nStored >= anchor->nCount;
}

static inline int pcan_fifo_status(FIFO_MANAGER *anchor)
{
	return anchor->nStored;
}

int pcan_fifo_reset(FIFO_MANAGER *anchor);
int pcan_fifo_init(FIFO_MANAGER *anchor, void *bufferBegin,
		void *bufferEnd, int nCount, u16 wCopySize);
int pcan_fifo_put(FIFO_MANAGER *anchor, void *pvPutData);
int pcan_fifo_get(FIFO_MANAGER *anchor, void *pvPutData);
int pcan_fifo_peek(FIFO_MANAGER *anchor, void *pvGetData);

int pcan_fifo_foreach_back(FIFO_MANAGER *anchor,
		int (*pf)(void *item, void *arg), void *arg);
u32 pcan_fifo_ratio(FIFO_MANAGER *anchor);
#endif
