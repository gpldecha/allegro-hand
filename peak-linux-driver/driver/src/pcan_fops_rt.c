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
 *                Stephane Grosjean (s.grosjean@peak-system.com)    USB-PRO
 *
 * Contributions: Marcel Offermans (marcel.offermans@luminis.nl)
 *                Arno (a.vdlaan@hccnet.nl)
 *                John Privitera (JohnPrivitera@dciautomation.com)
 *****************************************************************************/

/*****************************************************************************
 *
 * pcan_fops_rt.c - all file operation functions, exports only struct fops
 *
 * $Id: pcan_fops_rt.c $
 *
 *****************************************************************************/

#if RTDM_API_VER < 6
#define IOCTL_REQUEST_TYPE int
#else
#define IOCTL_REQUEST_TYPE unsigned int
#endif

#ifdef XENOMAI3
#define RTDM_SUBCLASS_PCAN	0
#endif

/* CAN-FD new API */
extern int pcanfd_dev_reset(struct pcandev *dev);
extern int pcan_dev_start_writing(struct pcandev *dev, struct pcan_udata *ctx);
extern int __pcan_dev_start_writing(struct pcandev *dev, struct pcan_udata *ctx);

static int copy_from_user_rt(rtdm_user_info_t *user_info,
				void *to, const void __user *from, size_t size)
{
	if (user_info) {
		if (!rtdm_read_user_ok(user_info, from, size) ||
			rtdm_copy_from_user(user_info, to, from, size))
			return -EFAULT;
	} else {
		memcpy(to, from, size);
	}

	return 0;
}

static int copy_to_user_rt(rtdm_user_info_t *user_info,
				void __user *to, const void *from, size_t size)
{
	if (user_info) {
		if (!rtdm_rw_user_ok(user_info, to, size) ||
			rtdm_copy_to_user(user_info, to, from, size))
			return -EFAULT;
	} else {
		memcpy(to, from, size);
	}

	return 0;
}

/*
 * called when the path is opened
 */
#ifdef XENOMAI3
static int pcan_open_nrt(struct rtdm_fd *fd, int oflags)
{
	int _major = 0;
	int _minor = rtdm_fd_minor(fd);
	struct pcan_udata *ctx = (struct pcan_udata *)rtdm_fd_to_private(fd);
#else
static int pcan_open_nrt(struct rtdm_dev_context *context,
			 rtdm_user_info_t *user_info, int oflags)
{
	int _major = MAJOR(context->device->device_id);
	int _minor = MINOR(context->device->device_id);
	struct pcan_udata *ctx = (struct pcan_udata *)context->dev_private;
#endif
	struct pcandev *dev;

#ifdef DEBUG
	pr_info("%s: %s(major=%d minor=%d, oflags=%08xh)\n",
		DEVICE_NAME, __func__, _major, _minor, oflags);
#endif

	/* TODO: get the device major number from xenomai structure... */
	dev = pcan_search_dev(_major, _minor);
	if (!dev)
		return -ENODEV;

	ctx->dev = dev;
	ctx->open_flags = oflags;
#ifndef XENOMAI3
	ctx->context = context;
#endif
	ctx->nReadRest = 0;
	ctx->nTotalReadCount = 0;
	ctx->pcReadPointer = ctx->pcReadBuffer;
	ctx->nWriteCount = 0;
	ctx->pcWritePointer = ctx->pcWriteBuffer;

	return pcan_open_path(dev, ctx);
}

/*
 * called when the path is closed.
 *
 * Note: (RTDM-and-Application.pdf 2.2 p3)
 *
 * "Closing a device instance is sensitive to the correct
 *  context. If the instance has been created in nonreal-time
 *  context, it cannot be closed within a realtime
 *  task"
 */
#ifdef XENOMAI3
static void pcan_close_nrt(struct rtdm_fd *fd)
{
	struct pcan_udata *ctx = (struct pcan_udata *)rtdm_fd_to_private(fd);

#else
static int pcan_close_nrt(struct rtdm_dev_context *context,
				rtdm_user_info_t *user_info)
{
	struct pcan_udata *ctx = (struct pcan_udata *)context->dev_private;
#endif
	struct pcandev *dev = ctx->dev;

#ifdef DEBUG
	pr_info("%s: %s(): RT task=%d\n", DEVICE_NAME, __func__,
			rtdm_in_rt_context());
#endif

	if (dev) {
		pcan_release_path(dev, ctx);
		ctx->dev = NULL;
#ifndef XENOMAI3
		ctx->context = NULL;
#endif
	}

#ifndef XENOMAI3
	return 0;
#endif
}

/* is called at user ioctl() with cmd = PCAN_INIT */
static int pcan_ioctl_init_rt(rtdm_user_info_t *user_info,
			struct pcan_udata *ctx, TPCANInit __user *pi)
{
	struct pcandev *dev = ctx->dev;
	struct pcanfd_init init_fd;
	TPCANInit init;
	int err;

#ifdef DEBUG
	pr_info("%s: %s()\n", DEVICE_NAME, __func__);
#endif

	err = copy_from_user_rt(user_info, &init, pi, sizeof(init));
	if (err)
		return  -EFAULT;

	return pcanfd_ioctl_set_init(dev, pcan_init_to_fd(&init_fd, &init));
}

/* is called at user ioctl() with cmd = PCAN_WRITE_MSG */
static int pcan_ioctl_write_rt(rtdm_user_info_t *user_info,
			struct pcan_udata *ctx, TPCANMsg __user *usr)
{
	struct pcandev *dev = ctx->dev;
	struct pcanfd_msg cf;
	TPCANMsg msg;
	int err;

#if 0
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);
#endif
	/* get from user space */
	if (copy_from_user_rt(user_info, &msg, usr, sizeof(msg))) {
		err = -EFAULT;
		goto fail;
	}

	/* do some minimal (but mandatory!) check */
	if (msg.LEN > 8) {
		err = -EINVAL;
		goto fail;
	}

	/* convert old-style TPCANMsg into new-style struct pcanfd_msg */
	err = pcanfd_ioctl_send_msg(dev, pcan_msg_to_fd(&cf, &msg), ctx);
	if (err)
		goto fail;

	return 0;

fail:
#ifdef DEBUG
        pr_err("%s: failed to write CAN frame (err %d)\n", DEVICE_NAME, err);
#endif
	return err;
}

/* is called at user ioctl() with cmd = PCAN_READ_MSG */
static int pcan_ioctl_read_rt(rtdm_user_info_t *user_info,
			struct pcan_udata *ctx, TPCANRdMsg __user *usr)
{
	struct pcandev *dev = ctx->dev;
	struct pcanfd_msg msgfd;
	TPCANRdMsg msg;
	int err;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	do {
		err = pcanfd_ioctl_recv_msg(dev, &msgfd, ctx);
		if (err)
			return err;

		if (pcan_is_fd(&msgfd)) {
#ifdef DEBUG
			pr_info("%s: CAN-FD frame discarded "
				"(CAN 2.0 application)\n", DEVICE_NAME);
#endif
			err = -EINVAL;
		}
	} while (err);

	if (copy_to_user_rt(user_info, usr,
				pcan_fd_to_msg(&msg, &msgfd), sizeof(*usr)))
		err = -EFAULT;

	return err;
}

/* is called at user ioctl() with cmd = PCAN_GET_STATUS */
static int pcan_ioctl_status_rt(rtdm_user_info_t *user_info,
			struct pcan_udata *ctx, TPSTATUS __user *status)
{
	struct pcandev *dev = ctx->dev;
	TPSTATUS local;
	int err = 0;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	pcan_ioctl_status_common(dev, &local);

	if (copy_to_user_rt(user_info, status, &local, sizeof(local))) {
		err = -EFAULT;
		goto fail;
	}

	dev->wCANStatus = 0;
	dev->nLastError = 0;

fail:
	return err;
}

/* is called at user ioctl() with cmd = PCAN_GET_EXT_STATUS */
static int pcan_ioctl_extended_status_rt(rtdm_user_info_t *user_info,
			struct pcan_udata *ctx, TPEXTENDEDSTATUS __user *status)
{
	struct pcandev *dev = ctx->dev;
	TPEXTENDEDSTATUS local;
	int err = 0;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	pcan_ioctl_extended_status_common(dev, &local);

	if (copy_to_user_rt(user_info, status, &local, sizeof(local))) {
		err = -EFAULT;
		goto fail;
	}

	dev->wCANStatus = 0;
	dev->nLastError = 0;

fail:
	return err;
}

/* is called at user ioctl() with cmd = PCAN_DIAG */
static int pcan_ioctl_diag_rt(rtdm_user_info_t *user_info,
			struct pcan_udata *ctx, TPDIAG __user *diag)
{
	struct pcandev *dev = ctx->dev;
	TPDIAG local;
	int err = 0;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	pcan_ioctl_diag_common(dev, &local);

	if (copy_to_user_rt(user_info, diag, &local, sizeof(local)))
		err = -EFAULT;

	return err;
}

/* get BTR0BTR1 init values */
static int pcan_ioctl_BTR0BTR1_rt(rtdm_user_info_t *user_info,
			struct pcan_udata *ctx, TPBTR0BTR1 __user *BTR0BTR1)
{
	TPBTR0BTR1 local;
	int err = 0;

#ifdef DEBUG
	pr_info("%s: %s()\n", DEVICE_NAME, __func__);
#endif

	if (copy_from_user_rt(user_info, &local, BTR0BTR1, sizeof(local))) {
		err = -EFAULT;
		goto fail;
	}

	/* this does not influence hardware settings, only BTR0BTR1 values are
	 * calculated */
	local.wBTR0BTR1 = sja1000_bitrate(local.dwBitRate, 0);
	if (!local.wBTR0BTR1) {
		err = -EFAULT;
		goto fail;
	}

	if (copy_to_user_rt(user_info, BTR0BTR1, &local, sizeof(*BTR0BTR1)))
		err = -EFAULT;

fail:
	return err;
}

/* add a message filter_element into the filter chain or delete all
 * filter_elements */
static int pcan_ioctl_msg_filter_rt(rtdm_user_info_t *user_info,
			struct pcan_udata *ctx, TPMSGFILTER __user *filter)
{
	struct pcandev *dev = ctx->dev;
	TPMSGFILTER local_filter;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	/* filter == NULL -> delete the filter_elements in the chain */
	if (!filter) {
		pcan_delete_filter_all(dev->filter);
		return 0;
	}

	if (copy_from_user_rt(user_info, &local_filter,
					filter, sizeof(local_filter)))
		return -EFAULT;

	return pcan_add_filter(dev->filter, local_filter.FromID,
				local_filter.ToID, local_filter.MSGTYPE);
}

#if 0

#define PCANFD_MAX_MSGS	8

struct __array_of_struct(pcanfd_msg, PCANFD_MAX_MSGS);
#define pcanfd_max_msgs		pcanfd_msgs_PCANFD_MAX_MSGS

static int handle_pcanfd_send_msgs(struct pcandev *dev, void __user *up,
					struct pcan_udata *dev_priv,
					rtdm_user_info_t *user_info)
{
	int l, err;

#ifdef pcanfd_max_msgs
	struct pcanfd_max_msgs msgfdl;

	l = sizeof(struct pcanfd_max_msgs);
	err = copy_from_user_rt(user_info, &msgfdl, up, l);
	if (err) {
		pr_err("%s: %s(): copy_from_user_rt() failure\n",
			DEVICE_NAME, __func__);
		return -EFAULT;
	}

	if (msgfdl.count > PCANFD_MAX_MSGS)
		msgfdl.count = PCANFD_MAX_MSGS;

	err = pcanfd_ioctl_send_msgs(dev,
				(struct pcanfd_msgs *)&msgfdl, dev_priv);

	/* copy only the count of msgs really sent (= pl->count) */
	if (copy_to_user_rt(user_info, up,
				&msgfdl, sizeof(struct pcanfd_msgs_0))) {
		pr_err("%s: %s(): copy_to_user_rt() failure\n",
			DEVICE_NAME, __func__);
		err = -EFAULT;
	}
#else
	struct pcanfd_msgs msgfdl, *pl;

	l = sizeof(struct pcanfd_msgs_0);
	err = copy_from_user_rt(user_info, &msgfdl, up, l);
	if (err) {
		pr_err("%s: %s(): copy_from_user_rt() failure\n",
			DEVICE_NAME, __func__);
		return -EFAULT;
	}

	/* ok. Nothing to send. So nothing done. Perfect. */
	if (!msgfdl.count)
		return 0;

	l += msgfdl.count * sizeof(msgfd);
	pl = pcan_malloc(l, GFP_KERNEL);
	if (!pl) {
		pr_err("%s: %s(): failed to alloc msgs list\n",
			DEVICE_NAME, __func__);
		return -ENOMEM;
	}

	if (copy_from_user_rt(user_info, pl, up, l)) {
		pr_err("%s: %s(): copy_from_user_rt() failure\n",
			DEVICE_NAME, __func__);
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
	if (copy_to_user_rt(user_info, up, pl, sizeof(struct pcanfd_msgs_0))) {
		pr_err("%s: %s(): copy_to_user_rt() failure\n",
			DEVICE_NAME, __func__);
		err = -EFAULT;
	}

	pcan_free(pl);
#endif

	return err;
}

static int handle_pcanfd_recv_msgs(struct pcandev *dev, void __user *up,
					struct pcan_udata *dev_priv,
					rtdm_user_info_t *user_info)
{
	int l, err;

#ifdef pcanfd_max_msgs
	struct pcanfd_max_msgs msgfdl;

	l = sizeof(struct pcanfd_msgs_0);
	err = copy_from_user_rt(user_info, &msgfdl, up, l);
	if (err) {
		pr_err("%s: %s(): copy_from_user_rt() failure\n",
			DEVICE_NAME, __func__);
		return -EFAULT;
	}

	if (msgfdl.count > PCANFD_MAX_MSGS)
		msgfdl.count = PCANFD_MAX_MSGS;

	err = pcanfd_ioctl_recv_msgs(dev,
				(struct pcanfd_msgs *)&msgfdl, dev_priv);

	/* copy the count and the msgs received */
	l += msgfdl.count * sizeof(struct pcanfd_msg);
	if (copy_to_user_rt(user_info, up, &msgfdl, l)) {
		pr_err("%s: %s(): copy_to_user_rt() failure\n",
			DEVICE_NAME, __func__);
		err = -EFAULT;
	}
#else
	struct pcanfd_msgs msgfdl, *pl;

	l = sizeof(struct pcanfd_msgs_0);
	err = copy_from_user_rt(user_info, &msgfdl, up, l);
	if (err) {
		pr_err("%s: %s(): copy_from_user_rt() failure\n",
			DEVICE_NAME, __func__);
		return -EFAULT;
	}

	/* ok! no room for saving rcvd msgs!? Thus, nothing returned */
	if (!msgfdl.count)
		return 0;

	l += msgfdl.count * sizeof(msgfd);
	pl = pcan_malloc(l, GFP_KERNEL);
	if (!pl) {
		pr_err("%s: failed to alloc msgs list\n", DEVICE_NAME);
		return -ENOMEM;
	}

	pl->count = msgfdl.count;
	err = pcanfd_ioctl_recv_msgs(dev, pl, dev_priv);

	/* copy the count and the msgs received */
	l = sizeof(struct pcanfd_msgs_0) + pl->count * sizeof(msgfd);
	if (copy_to_user_rt(user_info, up, pl, l)) {
		pr_err("%s: %s(): copy_to_user_rt() failure\n",
			DEVICE_NAME, __func__);
		err = -EFAULT;
	}

	pcan_free(pl);
#endif

	return err;
}

static int handle_pcanfd_get_av_clocks(struct pcandev *dev, void __user *up,
					struct pcan_udata *dev_priv,
					rtdm_user_info_t *user_info)
{
	struct pcanfd_available_clocks avclks;
	int l = sizeof(struct pcanfd_available_clocks_0);
	const void *kp;
	int err;

	err = copy_from_user_rt(user_info, &avclks, up, l);
	if (err) {
		pr_err("%s: %s(): copy_from_user_rt(user_info, ) failure\n",
			DEVICE_NAME, __func__);
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

	if (copy_to_user_rt(user_info, up, kp, l)) {
		pr_err("%s: %s(): copy_to_user_rt() failure\n",
			DEVICE_NAME, __func__);
		err = -EFAULT;
	}

	return err;
}

static int handle_pcanfd_get_bittiming_ranges(struct pcandev *dev,
						void __user *up,
						struct pcan_udata *dev_priv,
						rtdm_user_info_t *user_info)
{
	struct __array_of_struct(pcanfd_bittiming_range, 2) fdbtr;
	int l = sizeof(struct pcanfd_bittiming_ranges_0);
	//int l = sizeof(fdbtr.count);
	int err = copy_from_user_rt(user_info, &fdbtr, up, l);
	u32 user_count;

	if (err) {
		pr_err("%s: %s(): copy_from_user_rt() failure\n",
			DEVICE_NAME, __func__);
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
	if (copy_to_user_rt(user_info, up, &fdbtr, l)) {
		pr_err("%s: %s(): copy_to_user_rt() failure\n",
			DEVICE_NAME, __func__);
		err = -EFAULT;
	}

	return err;
}

static int handle_pcanfd_get_option(struct pcandev *dev, void __user *up,
					struct pcan_udata *dev_priv,
					rtdm_user_info_t *user_info)
{
	struct pcanfd_option opt;
	const int l = sizeof(opt);

	int err = copy_from_user_rt(user_info, &opt, up, l);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user_rt() failure\n",
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

	err = dev->option[opt.name].get(dev, &opt);
	if (err)
		return err;

lbl_cpy_size:
	/* update 'size' field */
	if (copy_to_user_rt(user_info, up+offsetof(struct pcanfd_option, size),
			 &opt.size, sizeof(opt.size))) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user_rt() failure\n",
			__func__);
		err = -EFAULT;
	}

	return err;
}

static int handle_pcanfd_set_option(struct pcandev *dev, void __user *up,
					struct pcan_udata *dev_priv,
					rtdm_user_info_t *user_info)
{
	struct pcanfd_option opt;
	int l = sizeof(opt);

	int err = copy_from_user_rt(user_info, &opt, up, l);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user_rt() failure\n",
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

	return dev->option[opt.name].set(dev, &opt);
}
#endif

/* is called at user ioctl() call */
#ifdef XENOMAI3
static int pcan_ioctl_rt(struct rtdm_fd *user_info,
				IOCTL_REQUEST_TYPE cmd, void *arg)
{
	struct pcan_udata *ctx = (struct pcan_udata *)rtdm_fd_to_private(user_info);

#else
static int pcan_ioctl_rt(struct rtdm_dev_context *context,
				rtdm_user_info_t *user_info,
				IOCTL_REQUEST_TYPE cmd, void *arg)
{
	struct pcan_udata *ctx = (struct pcan_udata *)context->dev_private;
#endif
	struct pcandev *dev = ctx->dev;
	struct pcanfd_init fdi;
	struct pcanfd_state fds;
	struct pcanfd_msg msgfd;
	void __user *up = (void __user *)arg;
	int l, err;

	/* if the device is plugged out */
	if (!dev->ucPhysicallyInstalled)
		return -ENODEV;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif
	switch (cmd) {
	case PCAN_INIT:
		err = pcan_ioctl_init_rt(user_info, ctx,
						(TPCANInit __user *)arg);
		break;
	case PCAN_READ_MSG:
		/* support blocking and nonblocking IO */
		err = pcan_ioctl_read_rt(user_info, ctx,
						(TPCANRdMsg __user *)arg);
		break;
	case PCAN_WRITE_MSG:
		/* support blocking and nonblocking IO */
		err = pcan_ioctl_write_rt(user_info, ctx,
						(TPCANMsg __user *)arg);
		break;
	case PCAN_GET_STATUS:
		err = pcan_ioctl_status_rt(user_info, ctx,
						(TPSTATUS __user *)arg);
		break;
	case PCAN_GET_EXT_STATUS:
		err = pcan_ioctl_extended_status_rt(user_info, ctx,
						(TPEXTENDEDSTATUS __user *)arg);
		break;
	case PCAN_DIAG:
		err = pcan_ioctl_diag_rt(user_info, ctx,
						(TPDIAG __user *)arg);
		break;
	case PCAN_BTR0BTR1:
		err = pcan_ioctl_BTR0BTR1_rt(user_info, ctx,
						(TPBTR0BTR1 __user *)arg);
		break;
	case PCAN_MSG_FILTER:
		err = pcan_ioctl_msg_filter_rt(user_info, ctx,
						(TPMSGFILTER *)arg);
		break;

	/* CAN-FD new API */
	case PCANFD_SET_INIT:
		err = copy_from_user_rt(user_info, &fdi, up, sizeof(fdi));
		if (err)
			return -EFAULT;

		err = pcanfd_ioctl_set_init(dev, &fdi);
		break;

	case PCANFD_GET_INIT:
		err = pcanfd_ioctl_get_init(dev, &fdi);
		if (err)
			break;

		err = copy_to_user_rt(user_info, up, &fdi, sizeof(fdi));
		if (err)
			return -EFAULT;
		break;

	case PCANFD_GET_STATE:
		err = pcanfd_ioctl_get_state(dev, &fds);
		if (err)
			break;

		err = copy_to_user_rt(user_info, up, &fds, sizeof(fds));
		if (err)
			return -EFAULT;

		break;

#ifdef PCANFD_ADD_FILTER
	case PCANFD_ADD_FILTER:
		if (arg) {
			struct pcanfd_msg_filter mf;

			err = copy_from_user_rt(user_info, &mf, up, sizeof(mf));
			if (err)
				return -EFAULT;

			err = pcanfd_ioctl_add_filter(dev, &mf);
		} else {
			err = pcanfd_ioctl_add_filter(dev, NULL);
		}
		break;
#endif
	case PCANFD_ADD_FILTERS:
		if (arg) {
			struct pcanfd_msg_filters mfl, *pfl;

			l = sizeof(struct pcanfd_msg_filters_0);
			err = copy_from_user_rt(user_info, &mfl, up, l);
			if (err)
				return -EFAULT;

			if (!mfl.count)
				return 0;

			l += mfl.count * sizeof(struct pcanfd_msg_filter);
			pfl = pcan_malloc(l, GFP_KERNEL);
			if (!pfl) {
				pr_err("%s: failed to alloc filter list\n",
						DEVICE_NAME);
				return -ENOMEM;
			}

			if (copy_from_user_rt(user_info, pfl, up, l)) {
				pcan_free(pfl);
				return -EFAULT;
			}

			err = pcanfd_ioctl_add_filters(dev, pfl);

			pcan_free(pfl);
		} else {
			err = pcanfd_ioctl_add_filters(dev, NULL);
		}
		break;

	case PCANFD_GET_FILTERS:
		if (arg) {
			struct pcanfd_msg_filters mfl, *pfl;

			l = sizeof(struct pcanfd_msg_filters_0);
			//l = sizeof(mfl.count);
			err = copy_from_user_rt(user_info, &mfl, up, l);
			if (err)
				return -EFAULT;

			if (!mfl.count)
				return 0;

			l += mfl.count * sizeof(struct pcanfd_msg_filter);
			pfl = pcan_malloc(l, GFP_KERNEL);
			if (!pfl) {
				pr_err("%s: failed to alloc filter list\n",
						DEVICE_NAME);
				return -ENOMEM;
			}

			pfl->count = mfl.count;
			err = pcanfd_ioctl_get_filters(dev, pfl);

			/* copy the count and the filter received */
			l = sizeof(struct pcanfd_msg_filters_0) +
				pfl->count * sizeof(struct pcanfd_msg_filter);

			if (copy_to_user_rt(user_info, up, pfl, l)) {
				pr_err("%s: %s(): copy_to_user_rt() failure\n",
						DEVICE_NAME, __func__);
				err = -EFAULT;
			}

			pcan_free(pfl);
		} else {
			err = pcanfd_ioctl_get_filters(dev, NULL);
		}
		break;

	case PCANFD_SEND_MSG:
		err = copy_from_user_rt(user_info, &msgfd, up, sizeof(msgfd));
		if (err)
			return -EFAULT;

		err = pcanfd_ioctl_send_msg(dev, &msgfd, ctx);
		break;

	case PCANFD_RECV_MSG:
		err = pcanfd_ioctl_recv_msg(dev, &msgfd, ctx);
		if (err)
			break;

		err = copy_to_user_rt(user_info, up, &msgfd, sizeof(msgfd));
		if (err)
			return -EFAULT;
		break;

	case PCANFD_SEND_MSGS:
		err = handle_pcanfd_send_msgs(dev, up, ctx, user_info);
		break;

	case PCANFD_RECV_MSGS:
		err = handle_pcanfd_recv_msgs(dev, up, ctx, user_info);
		break;

	case PCANFD_GET_AVAILABLE_CLOCKS:
		err = handle_pcanfd_get_av_clocks(dev, up, ctx, user_info);
		break;

	case PCANFD_GET_BITTIMING_RANGES:
		err = handle_pcanfd_get_bittiming_ranges(dev, up, ctx,
								user_info);
		break;

	case PCANFD_GET_OPTION:
		err = handle_pcanfd_get_option(dev, up, ctx, user_info);
		break;

	case PCANFD_SET_OPTION:
		err = handle_pcanfd_set_option(dev, up, ctx, user_info);
		break;

	default:
		pr_err("%s: %s(cmd=%u): unsupported cmd\n",
			DEVICE_NAME, __func__, cmd);
		err = -ENOTTY;
		break;
	}

	return err;
}

static int pcan_ioctl_nrt(struct rtdm_dev_context *context,
				rtdm_user_info_t *user_info,
				IOCTL_REQUEST_TYPE cmd, void *arg)
{
	switch (cmd) {
	case PCAN_WRITE_MSG:
	case PCANFD_SEND_MSG:
	case PCANFD_SEND_MSGS:
	case PCAN_READ_MSG:
	case PCANFD_RECV_MSG:
	case PCANFD_RECV_MSGS:
		pr_warn(DEVICE_NAME
			": WARNING[%p] ioctl(%x) called from non RT context!\n",
			rtdm_task_current(), _IOC_NR(cmd));
	default:
		break;
	}

	return pcan_ioctl_rt(context, user_info, cmd, arg);
}

/* this structure is used in init_module(void) */
#ifdef XENOMAI3
struct rtdm_driver pcandrv_rt = {
	.profile_info = RTDM_PROFILE_INFO(pcan,
					RTDM_CLASS_MISC,
					RTDM_SUBCLASS_PCAN,
					0),

	.context_size = sizeof(struct pcan_udata),
	.device_flags = RTDM_NAMED_DEVICE,
	.device_count = 32,
	.ops = {
		.open = pcan_open_nrt,
		.ioctl_rt = pcan_ioctl_rt,
		.ioctl_nrt = pcan_ioctl_rt,
		.close = pcan_close_nrt,
	},
};
#else
struct rtdm_device pcandev_rt = {
	.device_flags = RTDM_NAMED_DEVICE,
	.context_size = sizeof(struct pcan_udata),
	.struct_version =  RTDM_DEVICE_STRUCT_VER,

	.device_name = "",

	/* Named device instance creation for real-time contexts.
	 * usage is deprecated and should be NULL */
	.open_rt = NULL,

	/* Named device instance creation for non-real-time contexts */
	.open_nrt = pcan_open_nrt,

	.ops = {
		/* usage is deprecated and should be NULL */
		.close_rt = NULL,
		.close_nrt = pcan_close_nrt,

		.ioctl_rt = pcan_ioctl_rt,
		.ioctl_nrt = pcan_ioctl_nrt,
	},

	.device_class = RTDM_CLASS_CAN,
	.driver_version = RTDM_DRIVER_VER(PCAN_VERSION_MAJOR,
					  PCAN_VERSION_MINOR,
					  PCAN_VERSION_SUBMINOR),
	.driver_name = "pcan_driver",
	.provider_name = "PEAK-System Technik GmbH",
	.proc_name = pcandev_rt.device_name,
};
#endif
