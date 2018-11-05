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
 * Maintainer(s): Stephane Grosjean (s.grosjean@peak-system.com)
 *
 * Major contributions by:
 *                Klaus Hitschler (klaus.hitschler@gmx.de)
 *                Edouard Tisserant (edouard.tisserant@lolitech.fr) XENOMAI
 *                Laurent Bessard   (laurent.bessard@lolitech.fr)   XENOMAI
 *                Oliver Hartkopp   (oliver.hartkopp@volkswagen.de) socketCAN
 *
 * Contributions: Marcel Offermans (marcel.offermans@luminis.nl)
 *                Arno (a.vdlaan@hccnet.nl)
 *                John Privitera (JohnPrivitera@dciautomation.com)
 *****************************************************************************/
/* #define DEBUG */
/* #undef DEBUG */

/*****************************************************************************
 *
 * pcan_fops_linux.c - all file operation functions, exports only struct fops
 *
 * $Id: pcan_fops_linux.c $
 *
 *****************************************************************************/

#if 1
/* 2015-06-16 (SGr note):
 * seems that this function is useless...
 * Keep it a while, just for historical reason...
 */
#else
/*
 * wait until write fifo is empty, max time in msec
 */
void wait_for_empty_fifo(struct pcandev *dev, u32 mTime)
{
	u32 dwStart = get_mtime();

	/* not need to wait for anything if device not plugged! */
	if (!dev->ucPhysicallyInstalled)
		return;

	while (!atomic_read(&dev->hw_is_ready_to_send) &&
				((get_mtime() - dwStart) < mTime))
		schedule();

	/* force it */
	atomic_set(&dev->hw_is_ready_to_send, 1);
}
#endif

/* is called when the path is opened */
static int pcan_open(struct inode *inode, struct file *filep)
{
	struct pcandev *dev;
	struct pcan_udata *dev_priv;
	int _major = MAJOR(inode->i_rdev);
	int _minor = minor(inode->i_rdev);
	int err;

	DPRINTK(KERN_DEBUG "%s: %s(), major/minor = %d/%d\n",
			               DEVICE_NAME, __func__, _major, _minor);

	dev = pcan_search_dev(_major, _minor);
	if (!dev)
		return -ENODEV;

	/* create file object */
	dev_priv = pcan_malloc(sizeof(struct pcan_udata), GFP_KERNEL);
	if (!dev_priv) {
		pr_err("%s: %s(): memory allocation failed!\n",
				DEVICE_NAME, __func__);
		return -ENOMEM;
	}

	/* fill file object and init read and write method buffers */
	dev_priv->dev = dev;
	dev_priv->open_flags = filep->f_flags;
	dev_priv->filep = filep;

	if (filep->f_mode & FMODE_READ) {
		dev_priv->nReadRest = 0;
		dev_priv->nTotalReadCount = 0;
		dev_priv->pcReadPointer = dev_priv->pcReadBuffer;
	}

	if (filep->f_mode & FMODE_WRITE) {
		dev_priv->nWriteCount = 0;
		dev_priv->pcWritePointer = dev_priv->pcWriteBuffer;
	}

	filep->private_data = (void *)dev_priv;

	err = pcan_open_path(dev, dev_priv);
	if (err)
		pcan_free(dev_priv);

	return err;
}

static int pcan_release(struct inode *inode, struct file *filep)
{
	struct pcan_udata *dev_priv = (struct pcan_udata *)filep->private_data;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	/* free the associated irq and allocated memory */
	if (dev_priv) {
		if (dev_priv->dev)
			pcan_release_path(dev_priv->dev, dev_priv);

		pcan_free(dev_priv);
	}
	return 0;
}

/*
 * is called at user ioctl() with cmd = PCAN_INIT
 */
static int pcan_ioctl_init(struct pcandev *dev, TPCANInit __user *pi)
{
	TPCANInit init;
	struct pcanfd_init init_fd;
	int err;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	err = copy_from_user(&init, pi, sizeof(init));
	if (err) {
		pr_err("%s: %s(): copy_from_user() failure\n",
				DEVICE_NAME, __func__);
		return -EFAULT;
	}

	return pcanfd_ioctl_set_init(dev, pcan_init_to_fd(&init_fd, &init));
}

/*
 * is called at user ioctl() with cmd = PCAN_WRITE_MSG
 */
static int pcan_ioctl_write(struct pcandev *dev, TPCANMsg __user *usr,
						struct pcan_udata *dev_priv)
{
	struct pcanfd_msg cf;
	TPCANMsg msg;
	int err;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	/* get from user space */
	if (copy_from_user(&msg, usr, sizeof(msg))) {
		pr_err("%s: %s(): copy_from_user() failure\n",
				DEVICE_NAME, __func__);
		err = -EFAULT;
		goto fail;
	}

	/* do some minimal (but mandatory!) check */
	if (msg.LEN > 8) {
		pr_err("%s: trying to send msg %xh  with invalid data len %d\n",
				DEVICE_NAME, msg.ID, msg.LEN);
		err = -EINVAL;
		goto fail;
	}

	/* convert old-style TPCANMsg into new-style struct pcanfd_msg */
	err = pcanfd_ioctl_send_msg(dev, pcan_msg_to_fd(&cf, &msg), dev_priv);
	if (err)
		goto fail;

	return 0;

fail:
#ifdef DEBUG
        pr_err("%s: failed to write CAN frame (err %d)\n", DEVICE_NAME, err);
#endif
	return err;
}

/*
 * is called at user ioctl() with cmd = PCAN_READ_MSG
 */
static int pcan_ioctl_read(struct pcandev *dev, TPCANRdMsg __user *usr,
						struct pcan_udata *dev_priv)
{
	struct pcanfd_msg msgfd;
	TPCANRdMsg msg;
	int err;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	do {
		err = pcanfd_ioctl_recv_msg(dev, &msgfd, dev_priv);
		if (err)
			return err;

		if (pcan_is_fd(&msgfd)) {
			pr_err("%s: CAN-FD frame discarded "
				"(CAN 2.0 application)\n", DEVICE_NAME);
			err = -EINVAL;
		}
	} while (err);

	if (copy_to_user(usr, pcan_fd_to_msg(&msg, &msgfd), sizeof(*usr)))
		err = -EFAULT;

	return err;
}

/*
 * is called at user ioctl() with cmd = PCAN_GET_STATUS
 */
static int pcan_ioctl_status(struct pcandev *dev, TPSTATUS __user *status)
{
	TPSTATUS local;
	int err = 0;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	pcan_ioctl_status_common(dev, &local);

	if (copy_to_user(status, &local, sizeof(local))) {
		err = -EFAULT;
		goto fail;
	}

	dev->wCANStatus = 0;
	dev->nLastError = 0;

fail:
	return err;
}

/*
 * is called at user ioctl() with cmd = PCAN_GET_EXT_STATUS
 */
int pcan_ioctl_extended_status(struct pcandev *dev,
						TPEXTENDEDSTATUS __user *status)
{
	TPEXTENDEDSTATUS local;
	int err = 0;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	pcan_ioctl_extended_status_common(dev, &local);

	if (copy_to_user(status, &local, sizeof(local))) {
		err = -EFAULT;
		goto fail;
	}

	dev->wCANStatus = 0;
	dev->nLastError = 0;

fail:
	return err;
}

/*
 * is called at user ioctl() with cmd = PCAN_DIAG
 */
static int pcan_ioctl_diag(struct pcandev *dev, TPDIAG __user *diag)
{
	TPDIAG local;
	int err = 0;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	pcan_ioctl_diag_common(dev, &local);

	if (copy_to_user(diag, &local, sizeof(local)))
		err = -EFAULT;

	return err;
}

/*
 * get BTR0BTR1 init values
 */
static int pcan_ioctl_BTR0BTR1(struct pcandev *dev, TPBTR0BTR1 __user *BTR0BTR1)
{
	TPBTR0BTR1 local;
	int err = 0;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	if (copy_from_user(&local, BTR0BTR1, sizeof(local))) {
		err = -EFAULT;
		goto fail;
	}

	/* this does not influence hardware settings, only BTR0BTR1 values
	 * are calculated */
	local.wBTR0BTR1 = sja1000_bitrate(local.dwBitRate, 0 /* TODO */);
	if (!local.wBTR0BTR1) {
		err = -EFAULT;
		goto fail;
	}

	if (copy_to_user(BTR0BTR1, &local, sizeof(*BTR0BTR1)))
		err = -EFAULT;

fail:
	return err;
}

/*
 * add a message filter_element into the filter chain or delete all
 * filter_elements
 */
static int pcan_ioctl_msg_filter(struct pcandev *dev,
						TPMSGFILTER __user *filter)
{
	TPMSGFILTER local_filter;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	/* filter == NULL -> delete the filter_elements in the chain */
	if (!filter) {
		pcan_delete_filter_all(dev->filter);
		return 0;
	}

	if (copy_from_user(&local_filter, filter, sizeof(local_filter)))
		return -EFAULT;

	return pcan_add_filter(dev->filter, local_filter.FromID,
				local_filter.ToID, local_filter.MSGTYPE);
}

/*
 * set or get extra parameters from the devices
 */
static int pcan_ioctl_extra_parameters(struct pcandev *dev,
						TPEXTRAPARAMS __user *params)
{
	TPEXTRAPARAMS local;
	int err = 0;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	if (copy_from_user(&local, params, sizeof(local))) {
		pr_err("%s: %s(): copy_from_user() failure\n",
				DEVICE_NAME, __func__);
		err = -EFAULT;
		goto fail;
	}

	if (!dev->device_params) {
		pr_err("%s: %s(): NULL device_params address\n",
				DEVICE_NAME, __func__);
		err = -EINVAL;
		goto fail;
	}

	err = dev->device_params(dev, &local);
	if (err)
		goto fail;

	if (copy_to_user(params, &local, sizeof(*params)))
		err = -EFAULT;

fail:
	return err;
}

#if 0
#define PCANFD_MAX_MSGS	8

struct __array_of_struct(pcanfd_msg, PCANFD_MAX_MSGS);
#define pcanfd_max_msgs		pcanfd_msgs_PCANFD_MAX_MSGS

static int handle_pcanfd_send_msgs(struct pcandev *dev, void __user *up,
						struct pcan_udata *dev_priv)
{
	int l, err;

#ifdef pcanfd_max_msgs
	struct pcanfd_max_msgs msgfdl;

	l = sizeof(struct pcanfd_max_msgs);
	err = copy_from_user(&msgfdl, up, l);
	if (err) {
		pr_err("%s: %s(): copy_from_user() failure\n",
			DEVICE_NAME, __func__);
		return -EFAULT;
	}

	if (msgfdl.count > PCANFD_MAX_MSGS)
		msgfdl.count = PCANFD_MAX_MSGS;

	err = pcanfd_ioctl_send_msgs(dev,
				(struct pcanfd_msgs *)&msgfdl, dev_priv);

	/* copy only the count of msgs really sent (= pl->count) */
	if (copy_to_user(up, &msgfdl, sizeof(struct pcanfd_msgs_0))) {
		pr_err("%s: %s(): copy_to_user() failure\n",
			DEVICE_NAME, __func__);
		err = -EFAULT;
	}
#else
	struct pcanfd_msgs msgfdl, *pl;

	l = sizeof(struct pcanfd_msgs_0);
	err = copy_from_user(&msgfdl, up, l);
	if (err) {
		pr_err("%s: %s(): copy_from_user() failure\n",
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

	if (copy_from_user(pl, up, l)) {
		pr_err("%s: %s(): copy_from_user() failure\n",
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
	if (copy_to_user(up, pl, sizeof(struct pcanfd_msgs_0))) {
		pr_err("%s: %s(): copy_to_user() failure\n",
			DEVICE_NAME, __func__);
		err = -EFAULT;
	}

	pcan_free(pl);
#endif

	return err;
}

static int handle_pcanfd_recv_msgs(struct pcandev *dev, void __user *up,
						struct pcan_udata *dev_priv)
{
	int l, err;

#ifdef pcanfd_max_msgs
	struct pcanfd_max_msgs msgfdl;

	l = sizeof(struct pcanfd_msgs_0);
	err = copy_from_user(&msgfdl, up, l);
	if (err) {
		pr_err("%s: %s(): copy_from_user() failure\n",
			DEVICE_NAME, __func__);
		return -EFAULT;
	}

	if (msgfdl.count > PCANFD_MAX_MSGS)
		msgfdl.count = PCANFD_MAX_MSGS;

	err = pcanfd_ioctl_recv_msgs(dev,
				(struct pcanfd_msgs *)&msgfdl, dev_priv);

	/* copy the count and the msgs received */
	l += msgfdl.count * sizeof(struct pcanfd_msg);
	if (copy_to_user(up, &msgfdl, l)) {
		pr_err("%s: %s(): copy_to_user() failure\n",
			DEVICE_NAME, __func__);
		err = -EFAULT;
	}
#else
	struct pcanfd_msgs msgfdl, *pl;

	l = sizeof(struct pcanfd_msgs_0);
	err = copy_from_user(&msgfdl, up, l);
	if (err) {
		pr_err("%s: %s(): copy_from_user() failure\n",
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
	if (copy_to_user(up, pl, l)) {
		pr_err("%s: %s(): copy_to_user() failure\n",
			DEVICE_NAME, __func__);
		err = -EFAULT;
	}

	pcan_free(pl);
#endif

	return err;
}

static int handle_pcanfd_get_av_clocks(struct pcandev *dev, void __user *up,
						struct pcan_udata *dev_priv)
{
	struct pcanfd_available_clocks avclks;
	int l = sizeof(struct pcanfd_available_clocks_0);
	const void *kp;
	int err;

	err = copy_from_user(&avclks, up, l);
	if (err) {
		pr_err("%s: %s(): copy_from_user() failure\n",
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

	if (copy_to_user(up, kp, l)) {
		pr_err("%s: %s(): copy_to_user() failure\n",
			DEVICE_NAME, __func__);
		err = -EFAULT;
	}

	return err;
}

static int handle_pcanfd_get_bittiming_ranges(struct pcandev *dev,
						void __user *up,
						struct pcan_udata *dev_priv)
{
	struct __array_of_struct(pcanfd_bittiming_range, 2) fdbtr;
	int l = sizeof(struct pcanfd_bittiming_ranges_0);
	//int l = sizeof(fdbtr.count);
	int err = copy_from_user(&fdbtr, up, l);
	u32 user_count;

	if (err) {
		pr_err("%s: %s(): copy_from_user() failure\n",
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
	if (copy_to_user(up, &fdbtr, l)) {
		pr_err("%s: %s(): copy_to_user() failure\n",
			DEVICE_NAME, __func__);
		err = -EFAULT;
	}

	return err;
}

static int handle_pcanfd_get_option(struct pcandev *dev, void __user *up,
						struct pcan_udata *dev_priv)
{
	struct pcanfd_option opt;
	const int l = sizeof(opt);

	int err = copy_from_user(&opt, up, l);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
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
	if (copy_to_user(up+offsetof(struct pcanfd_option, size),
			 &opt.size, sizeof(opt.size))) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		err = -EFAULT;
	}

	return err;
}

static int handle_pcanfd_set_option(struct pcandev *dev, void __user *up,
						struct pcan_udata *dev_priv)
{
	struct pcanfd_option opt;
	int l = sizeof(opt);

	int err = copy_from_user(&opt, up, l);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
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

static long __pcan_ioctl(struct file *filep, unsigned int cmd, void __user *up)
{
	struct pcan_udata *dev_priv = (struct pcan_udata *)filep->private_data;
	struct pcandev *dev = dev_priv->dev;
	struct pcanfd_init fdi;
	struct pcanfd_state fds;
	struct pcanfd_msg msgfd;
	int err, l;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u, cmd=%u)\n",
		DEVICE_NAME, __func__, dev->nChannel+1, cmd);
#endif

	/* check whether this device is always linked. */
	if (!pcan_is_device_in_list(dev))
		return -ENODEV;

	/* if the device is plugged out */
	if (!dev->ucPhysicallyInstalled)
		return -ENODEV;

	switch (cmd) {
	case PCAN_INIT:
		err = pcan_ioctl_init(dev, (TPCANInit __user *)up);
		break;
	case PCAN_READ_MSG:
		/* support blocking and nonblocking IO */
		err = pcan_ioctl_read(dev, (TPCANRdMsg __user *)up, dev_priv);
		break;
	case PCAN_WRITE_MSG:
		/* support blocking and nonblocking IO */
		err = pcan_ioctl_write(dev, (TPCANMsg __user *)up, dev_priv);
		break;
	case PCAN_GET_STATUS:
		err = pcan_ioctl_status(dev, (TPSTATUS __user *)up);
		break;
	case PCAN_GET_EXT_STATUS:
		err = pcan_ioctl_extended_status(dev,
						(TPEXTENDEDSTATUS __user *)up);
		break;
	case PCAN_DIAG:
		err = pcan_ioctl_diag(dev, (TPDIAG __user *)up);
		break;
	case PCAN_BTR0BTR1:
		err = pcan_ioctl_BTR0BTR1(dev, (TPBTR0BTR1 __user *)up);
		break;
	case PCAN_MSG_FILTER:
		err = pcan_ioctl_msg_filter(dev, (TPMSGFILTER __user *)up);
		break;
	case PCAN_EXTRA_PARAMS:
		err = pcan_ioctl_extra_parameters(dev,
						(TPEXTRAPARAMS __user *)up);
		break;

	/* CAN-FD new API */
	case PCANFD_SET_INIT:
		err = copy_from_user(&fdi, up, sizeof(fdi));
		if (err)
			return -EFAULT;

		err = pcanfd_ioctl_set_init(dev, &fdi);
		break;

	case PCANFD_GET_INIT:
		err = pcanfd_ioctl_get_init(dev, &fdi);
		if (err)
			break;

		err = copy_to_user(up, &fdi, sizeof(fdi));
		if (err)
			return -EFAULT;
		break;

	case PCANFD_GET_STATE:
		err = pcanfd_ioctl_get_state(dev, &fds);
		if (err)
			break;

		err = copy_to_user(up, &fds, sizeof(fds));
		if (err)
			return -EFAULT;

		break;

#ifdef PCANFD_ADD_FILTER
	case PCANFD_ADD_FILTER:
		if (up) {
			struct pcanfd_msg_filter mf;

			err = copy_from_user(&mf, up, sizeof(mf));
			if (err)
				return -EFAULT;

			err = pcanfd_ioctl_add_filter(dev, &mf);
		} else {
			err = pcanfd_ioctl_add_filter(dev, NULL);
		}
		break;
#endif

	case PCANFD_ADD_FILTERS:
		if (up) {
			struct pcanfd_msg_filters mfl, *pfl;

			l = sizeof(struct pcanfd_msg_filters_0);
			//l = sizeof(mfl.count);
			err = copy_from_user(&mfl, up, l);
			if (err) {
				pr_err("%s: %s(): copy_from_user() failure\n",
					DEVICE_NAME, __func__);
				return -EFAULT;
			}

			if (!mfl.count)
				return 0;

			l += mfl.count * sizeof(struct pcanfd_msg_filter);
			pfl = pcan_malloc(l, GFP_KERNEL);
			if (!pfl) {
				pr_err("%s: failed to alloc filter list\n",
						DEVICE_NAME);
				return -ENOMEM;
			}

			if (copy_from_user(pfl, up, l)) {
				pcan_free(pfl);
				pr_err("%s: %s(): copy_from_user() failure\n",
					DEVICE_NAME, __func__);
				return -EFAULT;
			}

			err = pcanfd_ioctl_add_filters(dev, pfl);

			pcan_free(pfl);
		} else {
			err = pcanfd_ioctl_add_filters(dev, NULL);
		}
		break;

	case PCANFD_GET_FILTERS:
		if (up) {
			struct pcanfd_msg_filters mfl, *pfl;

			l = sizeof(struct pcanfd_msg_filters_0);
			//l = sizeof(mfl.count);
			err = copy_from_user(&mfl, up, l);
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

			if (copy_to_user(up, pfl, l)) {
				pr_err("%s: %s(): copy_to_user() failure\n",
						DEVICE_NAME, __func__);
				err = -EFAULT;
			}

			pcan_free(pfl);
		} else {
			err = pcanfd_ioctl_get_filters(dev, NULL);
		}
		break;

	case PCANFD_SEND_MSG:
		err = copy_from_user(&msgfd, up, sizeof(msgfd));
		if (err)
			return -EFAULT;

		err = pcanfd_ioctl_send_msg(dev, &msgfd, dev_priv);
		break;

	case PCANFD_RECV_MSG:
		err = pcanfd_ioctl_recv_msg(dev, &msgfd, dev_priv);
		if (err)
			break;

		err = copy_to_user(up, &msgfd, sizeof(msgfd));
		if (err)
			return -EFAULT;
		break;

	case PCANFD_SEND_MSGS:
		err = handle_pcanfd_send_msgs(dev, up, dev_priv, NULL);
		break;

	case PCANFD_RECV_MSGS:
		err = handle_pcanfd_recv_msgs(dev, up, dev_priv, NULL);
		break;

	case PCANFD_GET_AVAILABLE_CLOCKS:
		err = handle_pcanfd_get_av_clocks(dev, up, dev_priv, NULL);
		break;

	case PCANFD_GET_BITTIMING_RANGES:
		err = handle_pcanfd_get_bittiming_ranges(dev, up,
							 dev_priv, NULL);
		break;

	case PCANFD_GET_OPTION:
		err = handle_pcanfd_get_option(dev, up, dev_priv, NULL);
		break;

	case PCANFD_SET_OPTION:
		err = handle_pcanfd_set_option(dev, up, dev_priv, NULL);
		break;

	default:
		pr_err("%s: %s(cmd=%u): unsupported cmd "
			"(dir=%u type=%u nr=%u size=%u)\n",
			DEVICE_NAME, __func__, cmd,
			_IOC_DIR(cmd), _IOC_TYPE(cmd),
			_IOC_NR(cmd), _IOC_SIZE(cmd));
		err = -ENOTTY;
		break;
	}

#ifdef DEBUG
	pr_info("%s: %s(CAN%u, cmd=%u): returns %d\n",
		DEVICE_NAME, __func__, dev->nChannel+1, cmd, err);
#endif

	return err;

}

/*
 * is called at user ioctl() call
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
static int pcan_ioctl(struct inode *inode,
		struct file *filep, unsigned int cmd, unsigned long arg)
#else
static long pcan_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
#endif
{
	return __pcan_ioctl(filep, cmd, (void __user *)arg);
}

#ifdef PCAN_CONFIG_COMPAT
/*
 * 32-bit application using 64-bit driver
 */
#define __array_of_struct32(_n, _x)					\
	_n##s_##_x {							\
		__u32		count;					\
		struct _n	list[_x];				\
	} __aligned(4)

struct pcanfd_msg32 {
	__u16	type;
	__u16	data_len;
	__u32	id;
	__u32	flags;
	struct compat_timeval	timestamp;
	__u8	ctrlr_data[PCANFD_MAXCTRLRDATALEN];
	__u8	data[PCANFD_MAXDATALEN] __attribute__((aligned(8)));
} __aligned(4);

struct __array_of_struct32(pcanfd_msg32, 0);

#define pcanfd_msgs32		pcanfd_msg32s_0

struct pcanfd_state32 {
	__u16	ver_major, ver_minor, ver_subminor;

	struct compat_timeval	tv_init;

	compat_int_t	bus_state;

	__u32	device_id;

	__u32	open_counter;
	__u32	filters_counter;

	__u16	hw_type;
	__u16	channel_number;

	__u16	can_status;
	__u16	bus_load;

	__u32	tx_max_msgs;
	__u32	tx_pending_msgs;
	__u32	rx_max_msgs;
	__u32	rx_pending_msgs;
	__u32	tx_frames_counter;
	__u32	rx_frames_counter;
	__u32	tx_error_counter;
	__u32	rx_error_counter;

	__u64	host_time_ns;
	__u64	hw_time_ns;
} __aligned(4);

#define PCANFD_GET_STATE32	_IOR(PCAN_MAGIC_NUMBER, PCANFD_SEQ_GET_STATE,\
					struct pcanfd_state32)

#define PCANFD_SEND_MSG32	_IOW(PCAN_MAGIC_NUMBER, PCANFD_SEQ_SEND_MSG,\
					struct pcanfd_msg32)

#define PCANFD_RECV_MSG32	_IOR(PCAN_MAGIC_NUMBER, PCANFD_SEQ_RECV_MSG,\
					struct pcanfd_msg32)

#define PCANFD_SEND_MSGS32	_IOWR(PCAN_MAGIC_NUMBER, PCANFD_SEQ_SEND_MSGS,\
					struct pcanfd_msgs32)

#define PCANFD_RECV_MSGS32	_IOWR(PCAN_MAGIC_NUMBER, PCANFD_SEQ_RECV_MSGS,\
					struct pcanfd_msgs32)

/* because of the struct timeval different size, we must
 * do some manual copy... */
static void copy_from_msg32(struct pcanfd_msg *msgfd,
					const struct pcanfd_msg32 *msgfd32)
{
	/* note: copying msgs from userspace means that this is a message
	 * to send: no need to copy every fields... */
	msgfd->type = msgfd32->type;
	msgfd->data_len = msgfd32->data_len;
	msgfd->id = msgfd32->id;
	msgfd->flags = msgfd32->flags;
	memcpy(msgfd->data, msgfd32->data, PCANFD_MAXDATALEN);
}

static void copy_to_msg32(struct pcanfd_msg32 *msgfd32,
					const struct pcanfd_msg *msgfd)
{
#if 0
	msgfd32.type = msgfd.type;
	msgfd32.data_len = msgfd.data_len;
	msgfd32.id = msgfd.id;
	msgfd32.flags = msgfd.flags;
	msgfd32.timestamp.tv_sec = msgfd.timestamp.tv_sec;
	msgfd32.timestamp.tv_usec = msgfd.timestamp.tv_usec;
	memcpy(msgfd32.ctrlr_data, msgfd.ctrlr_data, PCANFD_MAXCTRLRDATALEN);
	memcpy(msgfd32.data, msgfd.data, PCANFD_MAXDATALEN);
#else
	void *pd = msgfd32;
	const void *ps = msgfd;

	memcpy(pd, ps, offsetof(struct pcanfd_msg, timestamp));

	msgfd32->timestamp.tv_sec = msgfd->timestamp.tv_sec;
	msgfd32->timestamp.tv_usec = msgfd->timestamp.tv_usec;

	memcpy(pd+offsetof(struct pcanfd_msg32, ctrlr_data),
		ps+offsetof(struct pcanfd_msg, ctrlr_data),
		sizeof(*msgfd32) - offsetof(struct pcanfd_msg32, ctrlr_data));
#endif
}

static int handle_pcanfd_send_msgs32(struct pcandev *dev, void __user *up,
						struct pcan_udata *dev_priv)
{
	struct pcanfd_msg32s_0 __user *pl32 = (struct pcanfd_msg32s_0 *)up;
	int i, l, err;

#ifdef pcanfd_max_msgs
	struct pcanfd_max_msgs msgfdl;

	l = sizeof(*pl32);
	err = copy_from_user(&msgfdl, up, l);
	if (err) {
		pr_err("%s: %s(): copy_from_user() failure\n",
			DEVICE_NAME, __func__);
		return -EFAULT;
	}

	if (msgfdl.count > PCANFD_MAX_MSGS)
		msgfdl.count = PCANFD_MAX_MSGS;

	for (i = 0; i < msgfdl.count; i++) {
		struct pcanfd_msg32 m32;

		err = copy_from_user(&m32, &pl32->list[i], sizeof(m32));
		if (err) {
			pr_err("%s: %s(): copy_from_user() failure\n",
				DEVICE_NAME, __func__);
			return -EFAULT;
		}

		copy_from_msg32(&msgfdl.list[i], &m32);
	}
	err = pcanfd_ioctl_send_msgs(dev,
				(struct pcanfd_msgs *)&msgfdl, dev_priv);

	/* copy only the count of msgs really sent (= pl->count) */
	if (copy_to_user(pl32, &msgfdl, sizeof(*pl32))) {
		pr_err("%s: %s(): copy_to_user() failure\n",
			DEVICE_NAME, __func__);
		err = -EFAULT;
	}
#else
	struct pcanfd_msgs msgfdl, *pl;

	l = sizeof(*pl32);
	err = copy_from_user(&msgfdl, up, l);
	if (err) {
		pr_err("%s: %s(): copy_from_user() failure\n",
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

	for (i = 0; i < msgfdl.count; i++) {
		struct pcanfd_msg32 m32;

		err = copy_from_user(&m32, &pl32->list[i], sizeof(m32));
		if (err) {
			pr_err("%s: %s(): copy_from_user() failure\n",
				DEVICE_NAME, __func__);
			return -EFAULT;
		}

		copy_from_msg32(&pl->list[i], &m32);
	}

	pl->count = i;
#if 0
	for (i = 0; i < pl->count; i++) {
		pr_info(DEVICE_NAME ": id=%x len=%u\n",
			pl->list[i].id,
			pl->list[i].data_len);
			
		dump_mem("data", pl->list[i].data, pl->list[i].data_len);
	}
#endif
	err = pcanfd_ioctl_send_msgs(dev, pl, dev_priv);

	/* copy the count of msgs really sent (= pl->count) */
	if (copy_to_user(pl32, pl, sizeof(*pl32))) {
		pr_err("%s: %s(): copy_to_user() failure\n",
			DEVICE_NAME, __func__);
		err = -EFAULT;
	}

	pcan_free(pl);
#endif

	return err;
}

static int handle_pcanfd_recv_msgs32(struct pcandev *dev, void __user *up,
						struct pcan_udata *dev_priv)
{
	struct pcanfd_msg32s_0 __user *pl32 = (struct pcanfd_msg32s_0 *)up;
	int i, l, err;

#ifdef pcanfd_max_msgs
	struct pcanfd_max_msgs msgfdl;

	l = sizeof(*pl32);
	err = copy_from_user(&msgfdl, up, l);
	if (err) {
		pr_err("%s: %s(): copy_from_user() failure\n",
			DEVICE_NAME, __func__);
		return -EFAULT;
	}

	if (msgfdl.count > PCANFD_MAX_MSGS)
		msgfdl.count = PCANFD_MAX_MSGS;

	err = pcanfd_ioctl_recv_msgs(dev,
				(struct pcanfd_msgs *)&msgfdl, dev_priv);

	/* copy the count of msgs received */
	if (copy_to_user(pl32, &msgfdl, sizeof(*pl32))) {
		pr_err("%s: %s(): copy_to_user() failure\n",
			DEVICE_NAME, __func__);
		err = -EFAULT;
	}

	/* copy the msgs received */
	for (i = 0; i < msgfdl.count; i++) {
		struct pcanfd_msg32 m32;

		copy_to_msg32(&m32, &msgfdl.list[i]);

		if (copy_to_user(&pl32->list[i], &m32, sizeof(m32))) {
			pr_err("%s: %s(): copy_to_user() failure\n",
				DEVICE_NAME, __func__);
			err = -EFAULT;
		}
	}
#else
	struct pcanfd_msgs msgfdl, *pl;

	l = sizeof(*pl32);
	err = copy_from_user(&msgfdl, up, l);
	if (err) {
		pr_err("%s: %s(): copy_from_user() failure\n",
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

	/* copy the count of msgs received */
	if (copy_to_user(pl32, pl, sizeof(*pl32))) {
		pr_err("%s: %s(): copy_to_user() failure\n",
			DEVICE_NAME, __func__);
		err = -EFAULT;
	}

	/* copy the msgs received */
	for (i = 0; i < msgfdl.count; i++) {
		struct pcanfd_msg32 m32;

		copy_to_msg32(&m32, &msgfdl.list[i]);

		if (copy_to_user(&pl32->list[i], &m32, sizeof(m32))) {
			pr_err("%s: %s(): copy_to_user() failure\n",
				DEVICE_NAME, __func__);
			err = -EFAULT;
		}
	}

	pcan_free(pl);
#endif

	return err;
}


static long pcan_compat_ioctl(struct file *filep, unsigned int cmd,
						unsigned long arg)
{
	struct pcan_udata *dev_priv = (struct pcan_udata *)filep->private_data;
	struct pcandev *dev = dev_priv->dev;
	void __user *argp = compat_ptr(arg);
	struct pcanfd_msg32 msgfd32;
	struct pcanfd_msg msgfd;
	struct pcanfd_state fds;
	struct pcanfd_state32 fds32;
	void *ps, *pd;
	long err;

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(cmd=%u) NR=%u SIZE=%u\n",
		__func__, cmd, _IOC_NR(cmd), _IOC_SIZE(cmd));
#endif
	switch (cmd) {

	case PCANFD_GET_STATE32:
		err = pcanfd_ioctl_get_state(dev, &fds);
		if (err)
			break;

		/* because of the struct timeval different size, we must
		 * do some manual copy... */
		ps = &fds;
		pd = &fds32;

		memcpy(pd, ps, offsetof(struct pcanfd_state, tv_init));

		fds32.tv_init.tv_sec = fds.tv_init.tv_sec;
		fds32.tv_init.tv_usec = fds.tv_init.tv_usec;

		memcpy(pd+offsetof(struct pcanfd_state32, bus_state),
			ps+offsetof(struct pcanfd_state, bus_state),
			sizeof(fds32) - 
				offsetof(struct pcanfd_state32, bus_state));

#if 0
		dump_mem("fds", &fds, sizeof(fds));
		dump_mem("fds32", &fds32, sizeof(fds32));
#endif
		err = copy_to_user(up, &fds32, sizeof(fds32));
		if (err)
			return -EFAULT;

		break;

	case PCANFD_SEND_MSG32:
		err = copy_from_user(&msgfd32, argp, sizeof(msgfd32));
		if (err)
			return -EFAULT;

		copy_from_msg32(&msgfd, &msgfd32);

		err = pcanfd_ioctl_send_msg(dev, &msgfd, dev_priv);
		break;

	case PCANFD_RECV_MSG32:
		err = pcanfd_ioctl_recv_msg(dev, &msgfd, dev_priv);
		if (err)
			break;

		copy_to_msg32(&msgfd32, &msgfd),

		err = copy_to_user(argp, &msgfd32, sizeof(msgfd32));
		if (err)
			return -EFAULT;
		break;

	case PCANFD_SEND_MSGS32:
		err = handle_pcanfd_send_msgs32(dev, argp, dev_priv);
		break;

	case PCANFD_RECV_MSGS32:
		err = handle_pcanfd_recv_msgs32(dev, argp, dev_priv);
		break;

	default:
		err = __pcan_ioctl(filep, cmd, argp);
	}

	return err;
}
#endif

/*
 * is called when read from the path
 */
static ssize_t pcan_read(struct file *filep, char *buf, size_t count,
								loff_t *f_pos)
{
	int err;
	int len = 0;
	struct pcanfd_msg f;
	struct pcan_udata *dev_priv = (struct pcan_udata *)filep->private_data;
	struct pcandev *dev = dev_priv->dev;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	/* check whether this device is always linked. */
	if (!pcan_is_device_in_list(dev))
		return -ENODEV;

	/* if the device is plugged out */
	if (!dev->ucPhysicallyInstalled)
		return -ENODEV;

	if (dev_priv->nReadRest <= 0) {

		err = pcanfd_ioctl_recv_msg(dev, &f, dev_priv);
		if (err)
			return err;

		dev_priv->nReadRest =
			pcan_make_output(dev_priv->pcReadBuffer, &f);
		dev_priv->pcReadPointer = dev_priv->pcReadBuffer;
	}

	/* give the data to the user */
	if (count > dev_priv->nReadRest) {
		/* put all data to user */
		len = dev_priv->nReadRest;
		dev_priv->nReadRest = 0;
		if (copy_to_user(buf, dev_priv->pcReadPointer, len)) {
			pr_err("%s: %s(): copy_to_user() failure\n",
				DEVICE_NAME, __func__);
			return -EFAULT;
		}
		dev_priv->pcReadPointer = dev_priv->pcReadBuffer;
	} else {
		/* put only partial data to user */
		len = count;
		dev_priv->nReadRest -= count;
		if (copy_to_user(buf, dev_priv->pcReadPointer, len)) {
			pr_err("%s: %s(): copy_to_user() failure\n",
				DEVICE_NAME, __func__);
			return -EFAULT;
		}
		dev_priv->pcReadPointer =
				(u8 *)((u8*)dev_priv->pcReadPointer + len);
	}

	*f_pos += len;
	dev_priv->nTotalReadCount += len;

	return len;
}

static int pcan_write_line(struct file *filep, u8 *ptr, size_t count)
{
	struct pcan_udata *dev_priv = (struct pcan_udata *)filep->private_data;
	struct pcandev *dev = dev_priv->dev;
	u32 amount = (u32)(dev_priv->pcWritePointer - ptr - 1);
	u32 offset = (u32)(ptr - dev_priv->pcWriteBuffer + 1);
	int err;

	if ((amount > WRITEBUFFER_SIZE) || (offset > WRITEBUFFER_SIZE)) {
		pr_err(DEVICE_NAME ": %s() fault: %zu %u, %u: \n",
			__func__, count, amount, offset);
		return -EFAULT;
	}

	if (pcan_parse_input_idle(dev_priv->pcWriteBuffer)) {
		struct pcanfd_msg msgfd;

		if (pcan_parse_input_message(dev_priv->pcWriteBuffer, &msgfd)) {
			struct pcanfd_init fdi;

			err = pcan_parse_input_init(dev_priv->pcWriteBuffer,
									&fdi);
			if (err)
				return err;
#if 0
			DPRINTK(KERN_DEBUG
				"%s: ***** Init 0x%04x 0x%02x 0x%02x\n",
				DEVICE_NAME, Init.wBTR0BTR1, Init.ucCANMsgType,
				Init.ucListenOnly);
#endif
			/* init the associated chip and the fifos again
			 * with new parameters
			 */
			err = pcanfd_ioctl_set_init(dev, &fdi);
			if (err)
				return err;
		} else {
#if 0 // ------- print out message, begin -----------
			int i = 0;

			DPRINTK(KERN_DEBUG "%s: *** 0x%08x 0x%02x %d . ",
				DEVICE_NAME, f.id, f.flags, f.data_len);

			while (i++ < f.data_len)
				DPRINTK(KERN_DEBUG "0x%02x ", f.data[i]);

			DPRINTK(KERN_DEBUG " ***\n");
#endif // ------- print out message, end ------------

			err = pcanfd_ioctl_send_msg(dev, &msgfd, dev_priv);
			if (err)
				if (err != -ENODATA)
					return err;
		}
	}

	/* move rest of amount data in buffer offset steps to left */
	memmove(dev_priv->pcWriteBuffer, ptr + 1, amount);
	dev_priv->pcWritePointer -= offset;

	return 0;
}

static ssize_t pcan_write(struct file *filep, const char *buf, size_t count,
								loff_t *f_pos)
{
	struct pcan_udata *dev_priv = (struct pcan_udata *)filep->private_data;
	struct pcandev *dev = dev_priv->dev;
	int err = 0;
	u32 dwRest;
	u8 *ptr;

#ifdef DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif

	/* check whether this device is always linked. */
	if (!pcan_is_device_in_list(dev))
		return -ENODEV;

	/* if the device is plugged out	*/
	if (!dev->ucPhysicallyInstalled)
		return -ENODEV;

	/* calculate remaining buffer space */
	dwRest = WRITEBUFFER_SIZE -
		(dev_priv->pcWritePointer - dev_priv->pcWriteBuffer); /* nRest > 0! */
	count  = (count > dwRest) ? dwRest : count;

	if (copy_from_user(dev_priv->pcWritePointer, buf, count)) {
		pr_err("%s: %s(): copy_from_user() failure\n",
				DEVICE_NAME, __func__);
		return -EFAULT;
	}

	/* adjust working pointer to end */
	dev_priv->pcWritePointer += count;

	/* iterate search blocks ending with '\n' */
	while (1) {

		/* search first '\n' from begin of buffer */
		ptr = dev_priv->pcWriteBuffer;
		while ((*ptr != '\n') && (ptr < dev_priv->pcWritePointer))
			ptr++;

		/* parse input when a CR was found */
		if ((*ptr == '\n') && (ptr < dev_priv->pcWritePointer)) {

			err = pcan_write_line(filep, ptr, count);
			if (err)
				return err;
		} else
			break; /* no CR found */
	}

	if (dev_priv->pcWritePointer >=
				(dev_priv->pcWriteBuffer + WRITEBUFFER_SIZE)) {
		/* reject all */
		dev_priv->pcWritePointer = dev_priv->pcWriteBuffer;
		return -EFAULT;
	}

	return count;
}

/*
 * is called at poll or select
 */
static unsigned int pcan_poll(struct file *filep, poll_table *wait)
{
	struct pcan_udata *dev_priv = (struct pcan_udata *)filep->private_data;
	struct pcandev *dev = dev_priv->dev;
	unsigned int mask = 0;

#if 0//def DEBUG
	pr_info("%s: %s(CAN%u)\n", DEVICE_NAME, __func__, dev->nChannel+1);
#endif
	pcan_mutex_lock(&dev->mutex);

	/* if the device is plugged out	*/
	if (dev->ucPhysicallyInstalled) {

		poll_wait(filep, &dev->in_event, wait);
		poll_wait(filep, &dev->out_event, wait);

		/* return on ops that could be performed without blocking */
		if (!pcan_fifo_empty(&dev->readFifo))
			mask |= POLLIN | POLLRDNORM;

		if (!pcan_fifo_full(&dev->writeFifo))
			mask |= POLLOUT | POLLWRNORM;
	}

	pcan_mutex_unlock(&dev->mutex);

	return mask;
}

/*
 * this structure is used in init_module(void)
 */
struct file_operations pcan_fops = {
	/*
	 * marrs:  added owner, which is used to implement a use count that
	 *         disallows rmmod calls when the driver is still in use (as
	 *         suggested by Duncan Sands on the linux-kernel mailinglist)
	 */
	owner:      THIS_MODULE,
	open:       pcan_open,
	release:    pcan_release,
	read:       pcan_read,
	write:      pcan_write,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
	ioctl:      pcan_ioctl,
#else
	unlocked_ioctl: pcan_ioctl,
#endif
#ifdef PCAN_CONFIG_COMPAT
	compat_ioctl: pcan_compat_ioctl,
#endif
	poll:       pcan_poll,
};
