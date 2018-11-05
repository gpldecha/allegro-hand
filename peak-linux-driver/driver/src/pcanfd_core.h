/*
 * CAN-FD extension to PEAK-System CAN products.
 *
 * Copyright (C) 2015 Stephane Grosjean <s.grosjean@peak-system.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#ifndef __pcanfd_core_h__
#define __pcanfd_core_h__

#include "src/pcan_common.h"
#include "src/pcan_main.h"
#include "src/pcan_fops.h"

int pcan_bittiming_normalize(struct pcan_bittiming *pbt,
			u32 clock_Hz, const struct pcanfd_bittiming_range *caps);
struct pcan_bittiming *pcan_btr0btr1_to_bittiming(struct pcan_bittiming *pbt,
						  u16 btr0btr1);
struct pcanfd_init *pcan_init_to_fd(struct pcanfd_init *pfdi,
				    const TPCANInit *pi);
TPCANInit *pcan_fd_to_init(TPCANInit *pi, struct pcanfd_init *pfdi);
struct pcanfd_msg *pcan_msg_to_fd(struct pcanfd_msg *pf, const TPCANMsg *msg);
TPCANRdMsg *pcan_fd_to_msg(TPCANRdMsg *msg, const struct pcanfd_msg *pf);

/* CAN-FD new API */
int pcanfd_dev_reset(struct pcandev *dev);
void pcanfd_dev_open_init(struct pcandev *dev);
int pcanfd_dev_open(struct pcandev *dev, struct pcanfd_init *pfdi);
int pcanfd_ioctl_set_init(struct pcandev *dev, struct pcanfd_init *pfdi);
int pcanfd_ioctl_get_init(struct pcandev *dev, struct pcanfd_init *pfdi);
int pcanfd_ioctl_get_state(struct pcandev *dev, struct pcanfd_state *pfds);
int pcanfd_ioctl_add_filter(struct pcandev *dev, struct pcanfd_msg_filter *pf);
int pcanfd_ioctl_add_filters(struct pcandev *dev,
						struct pcanfd_msg_filters *pfl);
int pcanfd_ioctl_get_filters(struct pcandev *dev,
						struct pcanfd_msg_filters *pfl);
int pcanfd_ioctl_send_msg(struct pcandev *dev, struct pcanfd_msg *pmsgfd,
						struct pcan_udata *dev_priv);
int pcanfd_ioctl_send_msgs(struct pcandev *dev, struct pcanfd_msgs *pl,
						struct pcan_udata *dev_priv);
int pcanfd_ioctl_recv_msg(struct pcandev *dev, struct pcanfd_msg *pmsgfd,
						struct pcan_udata *dev_priv);
int pcanfd_ioctl_recv_msgs(struct pcandev *dev, struct pcanfd_msgs *pl,
						struct pcan_udata *dev_priv);
#endif
