/*****************************************************************************
 * Copyright (C) 2001-2010  PEAK System-Technik GmbH
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

/*****************************************************************************
 *
 * global defines to include in all files this module is made of
 * ! this must always the 1st include in all files !
 *
 * $Id$
 *
 *****************************************************************************/

#ifndef __PCAN_COMMON_H__
#define __PCAN_COMMON_H__

//#define DEBUG_MEM

#ifndef __KERNEL__
#define __KERNEL__
#endif
#ifndef MODULE
#define MODULE
#endif

#include <linux/kernel.h>

/* if this file is not found: please look @ /boot/vmlinuz.version.h and make a
 * symlink */
#include <linux/version.h>

#include <linux/module.h>
#include <linux/stringify.h>

/* support for MODVERSIONS */
#ifndef AUTOCONF_INCLUDED
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,38)
#include <generated/autoconf.h>
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16)
#include <linux/autoconf.h>
#else
#include <linux/config.h>
#endif
#define AUTOCONF_INCLUDED
#endif

#if defined(CONFIG_MODVERSIONS) && !defined(MODVERSIONS)
#define MODVERSIONS
#endif

#ifdef MODVERSIONS
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
#include <config/modversions.h>
#endif
#else
#include <linux/modversions.h>
#endif
#endif

/* to get smoothly into udev */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,17)
#if 1//ndef XENOMAI3
#define UDEV_SUPPORT
#endif
#endif

/* #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35) */
#ifndef pr_warn
#define pr_warn(fmt, ...)	printk(KERN_WARNING pr_fmt(fmt), ##__VA_ARGS__)
#endif

#ifndef NO_COMPAT
#ifdef CONFIG_COMPAT
#define PCAN_CONFIG_COMPAT
#endif
#endif

static inline u64 timeval_to_us(struct timeval *tv)
{
	return ((u64 )tv->tv_sec * USEC_PER_SEC) + tv->tv_usec;
}

static inline void timeval_add_us(struct timeval *tv, signed long us)
{
	tv->tv_usec += us;
#if 0
	tv->tv_sec += tv->tv_usec / USEC_PER_SEC;
	tv->tv_usec %= USEC_PER_SEC;
#else
	while (tv->tv_usec >= USEC_PER_SEC) {
		tv->tv_sec++;
		tv->tv_usec -= USEC_PER_SEC;
	}
	while (tv->tv_usec < 0) {
		tv->tv_sec--;
		tv->tv_usec += USEC_PER_SEC;
	}
#endif
}

static inline signed long timeval_diff(struct timeval *tv0, struct timeval *tv1)
{
	return (long )(timeval_to_us(tv0) - timeval_to_us(tv1));
}

static inline int timeval_cmp(struct timeval *tv0, struct timeval *tv1)
{
	return (tv0->tv_sec != tv1->tv_sec) ? tv0->tv_sec - tv1->tv_sec
						: tv0->tv_usec - tv1->tv_usec;
}

#define timeval_is_older(t1, t2)	(timeval_cmp(t1, t2) < 0)

#ifdef NO_RT

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,17,0)
#include <linux/hrtimer.h>
#endif

#define PCAN_IRQF_SHARED		IRQF_SHARED

static inline int pcan_task_can_wait(void)
{
	return 1;
}

static inline u64 pcan_getnow_ns(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,17,0)
	return ktime_to_ns(ktime_get());
#else
	return ktime_get_ns();
#endif
}

static inline void pcan_getnow_ts(struct timespec *ts, u64 *ptv_ns)
{
	getrawmonotonic(ts);
	if (ptv_ns)
		*ptv_ns = pcan_getnow_ns();
}

static inline void pcan_gettimeofday_ex(struct timeval *tv, u64 *ptv_ns)
{
#ifdef USES_MONOTONIC_CLOCK
	struct timespec ts;

	pcan_getnow_ts(&ts, ptv_ns);

	//do_div(ts64.tv_nsec, NSEC_PER_USEC);
	tv->tv_usec = ts.tv_nsec / NSEC_PER_USEC;
	tv->tv_sec = ts.tv_sec;
#else
	do_gettimeofday(tv);
	if (ptv_ns)
		*ptv_ns = pcan_getnow_ns();
#endif
}

#include <linux/uaccess.h>

static inline int pcan_copy_from_user(void *to, const void __user *from,
					int size, void *c)
{
	return copy_from_user(to, from, size);
}

static inline int pcan_copy_to_user(void __user *to, const void *from,
					size_t size, void *c)
{
	return copy_to_user(to, from, size);
}

#else
#ifdef XENOMAI3
#include <rtdm/driver.h>

typedef struct rtdm_fd rtdm_user_info_t;
#else
#include <rtdm/rtdm_driver.h>
#endif
#include <asm/div64.h>

#define PCAN_IRQF_SHARED		RTDM_IRQTYPE_SHARED

static inline int pcan_task_can_wait(void)
{
	return rtdm_in_rt_context();
}

#if RTDM_API_VER < 5
typedef nanosecs_abs_t uint64_t;
#endif

static inline u64 pcan_getnow_ns(void)
{
	return rtdm_clock_read();
}

static inline void pcan_gettimeofday_ex(struct timeval *tv, u64 *ptv_ns)
{
	nanosecs_abs_t current_time = rtdm_clock_read();

	if (ptv_ns)
		*ptv_ns = current_time;

	/* a = do_div(b, c); <=> { a = b % c ; b = b / c; } */
	tv->tv_usec = (do_div(current_time, NSEC_PER_SEC) / NSEC_PER_USEC);
	tv->tv_sec = current_time;
}

static inline int pcan_copy_from_user(void *to, const void __user *from,
					int size, void *c)
{
	if (c) {
		if (!rtdm_read_user_ok((rtdm_user_info_t *)c, from, size) ||
		    rtdm_copy_from_user((rtdm_user_info_t *)c, to, from, size))
			return -EFAULT;
	} else {
		memcpy(to, from, size);
	}

	return 0;
}

static inline int pcan_copy_to_user(void __user *to, const void *from,
					size_t size, void *c)
{
	if (c) {
		if (!rtdm_rw_user_ok((rtdm_user_info_t *)c, to, size) ||
		    rtdm_copy_to_user((rtdm_user_info_t *)c, to, from, size))
			return -EFAULT;
	} else {
		memcpy(to, from, size);
	}

	return 0;
}

#endif

static inline void pcan_gettimeofday(struct timeval *tv)
{
	pcan_gettimeofday_ex(tv, NULL);
}

/* support for PARPORT_SUBSYSTEM */
#if !defined(CONFIG_PARPORT_MODULE) && !defined(CONFIG_PARPORT) && defined(PARPORT_SUBSYSTEM)
#undef PARPORT_SUBSYSTEM
#endif

/* support for USB */
#if !defined(CONFIG_USB_MODULE) && !defined(CONFIG_USB) && defined(USB_SUPPORT)
#undef USB_SUPPORT
#endif

/* support for PCCARD */
#if !defined(CONFIG_PCMCIA_MODULE) && !defined(CONFIG_PCMCIA) && !defined(CONFIG_PCCARD) && defined(PCCARD_SUPPORT)
#undef PCCARD_SUPPORT
#endif

/* support for PCIe (need I2C algo) */
#if !defined(CONFIG_I2C_ALGOBIT) && !defined(CONFIG_I2C_ALGOBIT_MODULE)
#undef PCIEC_SUPPORT
#endif

/* hotplug may need mutex locking around the global devices list.
 * On the other hand, don't add useless mechanism in the other case. */
#if defined(USB_SUPPORT) || defined(PCCARD_SUPPORT) || defined(NETDEV_SUPPORT) || defined(UDEV_SUPPORT)
#define HANDLE_HOTPLUG
#endif

/* support only versions 2.4.x and 2.6.x */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,0)
#error "This kernel is too old and not supported"
#else
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
#define LINUX_24 /* >= LINUX 2.4.x && < LINUX 2.6 */
#else
#define LINUX_26 /* >= LINUX 2.6 */
#endif
#endif

#define kHz				1000
#define MHz				(1000*kHz)
#define GHz				(1000*MHz)

#ifdef DEBUG_MEM
#define pcan_malloc(a, b)						\
({									\
	void *p;							\
	pr_info(DEVICE_NAME ": %s(%u): alloc(%u): ",			\
		__FILE__, __LINE__, a);					\
	p = kmalloc(a, b);						\
	printk("%p\n", p);						\
	p;								\
})
#define pcan_free(p)							\
({									\
	pr_info(DEVICE_NAME ": %s(%u): free(%p)\n",			\
		__FILE__, __LINE__, p);					\
	kfree(p);							\
})
#else
#define pcan_malloc(a, b)		kmalloc(a, b)
#define pcan_free(p)			kfree(p);
#endif

/* some preparative definitions for kernel 2.6.x */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,23)
typedef void irqreturn_t;
#define IRQ_NONE
#define IRQ_HANDLED
#define IRQ_RETVAL(x)
#endif

/* IRQ's return val for Linux and RealTime */
#ifdef NO_RT
#define PCAN_IRQ_NONE			IRQ_NONE
#define PCAN_IRQ_HANDLED		IRQ_HANDLED
#define PCAN_IRQ_RETVAL(x)		IRQ_RETVAL(x)
#else
#define PCAN_IRQ_NONE			RTDM_IRQ_NONE
#define PCAN_IRQ_HANDLED		RTDM_IRQ_HANDLED
#define PCAN_IRQ_RETVAL(x)		x
#endif

/* spinlock and mutex common interface */
#ifdef NO_RT
typedef unsigned long	pcan_lock_irqsave_ctxt;
typedef spinlock_t	pcan_lock_t;

#define pcan_lock_init(l)		spin_lock_init(l)
#define pcan_lock_get_irqsave(l, f)	spin_lock_irqsave(l, f)
#define pcan_lock_put_irqrestore(l, f)	spin_unlock_irqrestore(l, f)

typedef struct mutex	pcan_mutex_t;

#define pcan_mutex_init(m)		mutex_init(m)
#define pcan_mutex_lock(m)		mutex_lock(m)
#define pcan_mutex_unlock(m)		mutex_unlock(m)
#define pcan_mutex_trylock(m)		mutex_trylock(m)
#define pcan_mutex_destroy(m)		mutex_destroy(m)

#else
typedef rtdm_lockctx_t	pcan_lock_irqsave_ctxt;
typedef rtdm_lock_t	pcan_lock_t;

#define pcan_lock_init(l)		rtdm_lock_init(l)
#define pcan_lock_get_irqsave(l, f)	rtdm_lock_get_irqsave(l, f)
#define pcan_lock_put_irqrestore(l, f)	rtdm_lock_put_irqrestore(l, f)

typedef rtdm_mutex_t	pcan_mutex_t;

#define pcan_mutex_init(m)		rtdm_mutex_init(m)
#define pcan_mutex_lock(m)		rtdm_mutex_lock(m)
#define pcan_mutex_unlock(m)		rtdm_mutex_unlock(m)
#define pcan_mutex_trylock(m)		(!rtdm_mutex_timedlock(m, -1, NULL))
#define pcan_mutex_destroy(m)		rtdm_mutex_destroy(m)

#endif

/* event/queue common interface */
#ifdef NO_RT
typedef wait_queue_head_t	pcan_event_t;

#define pcan_event_init(e, v)	init_waitqueue_head(e)
#define pcan_event_free(e)
#define pcan_event_signal(e)	wake_up_interruptible(e)
#define pcan_event_wait_timeout(e, c, t)				\
({									\
	int err;							\
									\
	if (t) {							\
		err = wait_event_interruptible_timeout(e, c,		\
						msecs_to_jiffies(t));	\
		if (!err)						\
			err = -ETIMEDOUT;				\
	} else {							\
		err = wait_event_interruptible(e, c);			\
	}								\
	err;								\
})

#define pcan_event_wait(e, c)	wait_event_interruptible(e, c)
#else
typedef rtdm_event_t		pcan_event_t;

#define pcan_event_init(e, v)	rtdm_event_init(e, v)
#define pcan_event_free(e)	rtdm_event_destroy(e)
#define pcan_event_signal(e)	rtdm_event_signal(e)
#define pcan_event_wait_timeout(e, c, t)				\
({									\
	nanosecs_rel_t ns = (nanosecs_rel_t )t * 1000000LL;		\
	rtdm_toseq_t ts, *pts = &ts;					\
	int err = 0;							\
									\
	if (t)								\
		rtdm_toseq_init(&ts, ns);				\
	else								\
		pts = NULL;						\
	while (!(c)) {							\
		err = rtdm_event_timedwait(&e, ns, pts);		\
		if (err < 0)						\
			break;						\
	}								\
	err;								\
})
#define pcan_event_wait(e, c)	pcan_event_wait_timeout(e, c, 0)
#endif

/* count of function variables changed from 2.6.19 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#define IRQHANDLER(name, p1, p2, p3)	name(p1, p2, p3)
#else
#define IRQHANDLER(name, p1, p2, p3)	name(p1, p2)
#endif

/* switch to disable all printks for not debugging */
#ifdef DEBUG
#define DPRINTK				printk
#else
#define DPRINTK(stuff...)
#endif

/* to manage differences between kernel versions */
int ___request_region(unsigned long from, unsigned long length,
		const char *name);

/* follow current interrupt definition changes */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
#define IRQF_DISABLED			SA_INTERRUPT
#define IRQF_SHARED			SA_SHIRQ
#endif

/* different data sink alternatives */
#ifdef NETDEV_SUPPORT
#define pcan_xxxdev_rx(d, f)		pcan_netdev_rx(d, f)
#else
#define pcan_xxxdev_rx(d, f)		pcan_chardev_rx(d, f)
#endif

/* moved from pcan_main.h */
#define DEVICE_NAME	"pcan"	/* the name of the module and the proc entry */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
/* This has been added in 2.6.24 */
#define list_for_each_prev_safe(pos, n, head) \
			for (pos = (head)->prev, n = pos->prev; \
				prefetch(pos->prev), pos != (head); \
						pos = n, n = pos->prev)
#endif

/* set here the current release of the driver 'Release_date_nr' synchronous
 * with SVN
 */
#define CURRENT_RELEASE                 "Release_20171027_n"
#define PCAN_VERSION_MAJOR              8
#define PCAN_VERSION_MINOR              5
#define PCAN_VERSION_SUBMINOR           1
#define CURRENT_VERSIONSTRING		__stringify(PCAN_VERSION_MAJOR)"."\
					__stringify(PCAN_VERSION_MINOR)"."\
					__stringify(PCAN_VERSION_SUBMINOR)

#endif
