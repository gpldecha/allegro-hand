/*****************************************************************************
 * Copyright (C) 2001-2015  PEAK System-Technik GmbH
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
 *****************************************************************************/

/*****************************************************************************
 * pcanfdtst.c - a small program to test CAN[FD] ransfer to/from PCAN channels.
 *
 * $Id: pcnafdtst.c 864 2015-07-07 15:00:03Z stephane $
 *
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>

#ifndef _BSD_SOURCE
#define _BSD_SOURCE
#endif

#include <endian.h>

/* max number of bytes to display on one 80 columns line */
#define PUTMSG_DATABYTES_PER_LINE		10

#ifdef XENOMAI
#include <sys/mman.h>		/* mlockall() */
#include <native/timer.h>
#include <rtdk.h>		/* rt_fprintf() */

#ifdef __COBALT__
#include <alchemy/task.h>
#else
#include <native/task.h>
#endif

#define RT
#define ONE_TASK_PER_DEVICE

#define __printf		rt_printf
#define __fprintf		rt_fprintf
#define __vfprintf		rt_vfprintf
#define __usleep(u)		rt_task_sleep(1000*u)

static inline void rt_gettimeofday(struct timeval *tv)
{
	/* native skin timer function */
	RTIME now = rt_timer_read();
	tv->tv_sec = now / 1000000000ULL;
	tv->tv_usec = (now % 1000000000ULL) / 1000;
}

#define __gettimeofday(tv, x)	rt_gettimeofday(tv)

#elif defined(RTAI)
#include <rtai_lxrt.h>

#define RT
#define ONE_TASK_PER_DEVICE

#define __printf		print_to_screen
//#define __fprintf(...)
//#define __vfprintf(...)

static inline void rt_gettimeofday(struct timeval *tv)
{
	RTIME now = rt_get_time_ns();
	tv->tv_sec = now / 1000000000;
	tv->tv_usec = (now % 1000000000) / 1000;
}

#define __gettimeofday(tv, x)	rt_gettimeofday(tv)

#endif

/* if defined BEFORE including libpcanfd.h, tests can be made with using
 * old-CAN API. */
/* #define PCANFD_OLD_STYLE_API */

/* if defined, pcanfd_send_msgs_list() and pcanfd_recv_msgs_list() are used
 * instead of pcanfd_send_msgs() and pcanfd_recv_msgs() */
/* #define USES_MSGS_LIST */

/* this defines the maximum count of us. we admit with -T between host time and
 * received message time. If difference is greater, then rx test stops. */
#define PCANFD_TS_MAX_DELTA	1000000

#include <libpcanfd.h>

#ifndef __printf
#define __printf		printf
#endif
#ifndef __fprintf
#define __fprintf		fprintf
#endif
#ifndef __vfprintf
#define __vfprintf		vfprintf
#endif
#ifndef __gettimeofday
#define __gettimeofday(tv, x)	gettimeofday(tv, x)
#endif
#ifndef __usleep
#define __usleep(t)		usleep(t)
#endif

/* number max of /dev/pcan interfaces tested at the same time */
#define TST_DEV_PCAN_MAX	8

enum tst_status { NOK, OK };

static enum {
	TST_MODE_UNKNOWN,
	TST_MODE_TX,
	TST_MODE_RX,
	TST_MODE_NONE
} tst_mode = TST_MODE_UNKNOWN;

static enum log_level {
	QUIET,
	NORMAL,
	VERBOSE,
	DEBUG,
	ALWAYS
} tst_verbose = NORMAL;

static const char *txt_status[PCANFD_STATUS_COUNT] = {
	"UNKNOWN",
	"ACTIVE",
	"WARNING",
	"PASSIVE",
	"BUSOFF",
	"RX_EMPTY",
	"RX_OVRFL",
	"TX_EMPTY",
	"TX_OVRFL",
	"BUS_ERR",
	"BUS_LOAD",
};

static int tst_puts_timestamps = 0;
static int tst_check_timestamps = 0;

static FILE *tst_stdlog = NULL;
static char *tst_stdlog_filename = NULL;

static __u32 tst_flags = OFD_SAMPLEPT|PCANFD_INIT_TS_HOST_REL;
//static __u32 tst_flags = OFD_SAMPLEPT|PCANFD_INIT_TS_DRV_REL;
//static __u32 tst_flags = OFD_SAMPLEPT|PCANFD_INIT_TS_DEV_REL;
static __u32 tst_can_id = 0;
static __u32 tst_can_id_random = 1;
static __u32 tst_data_length = 0;
static __u32 tst_data_length_random = 1;
static __u32 tst_pause_us = 1000;
static __u32 tst_tx_pause_us = 0;
static __u32 tst_max_loop = 0;
static __u32 tst_bitrate, tst_dbitrate, tst_clock_Hz;
static __u32 tst_sample_pt = 0, tst_dsample_pt = 0;
static __u32 tst_incr_bytes = 0;
static __u32 tst_msgs_count = 1;
static int tst_fdmax = -1;
static int tst_sig_caught= 0;
static int pcan_device_count = 0;
static int pcan_device_opened = 0;
static __u32 tst_tx_count = 0;
static __u32 tst_rx_count = 0;
static __u32 tst_ts_mode = PCANFD_OPT_HWTIMESTAMP_MAX;
#ifndef ONE_TASK_PER_DEVICE
static struct timeval *tst_timeout_ptr = NULL;
#endif
static struct timeval tst_start;
static __u32 tst_timeout_ms = 1000;
static __u32 tst_msg_flags = PCANFD_MSG_STD;
static __u32 tst_ids[2];
static int tst_ids_set = 0;

static struct pcan_device {
	char *	name;
#ifdef PCANFD_OLD_STYLE_API
	HANDLE	handle;
#endif
#ifdef XENOMAI
	RT_TASK rt_task;
#elif defined(RTAI)
	RT_TASK *rt_task;
	pthread_t rt_thread;
#endif
	int	fd;
	__u32	flags;
	__u32	clock_Hz;
	__u32	bitrate;
	__u32	sample_pt;
	__u32	dbitrate;
	__u32	dsample_pt;
	__u32	can_id;
	__u32	can_id_random;
	__u32	data_length;
	__u32	data_length_random;
	__u32	pause_us;
	__u32	tx_pause_us;
	__u32	incr_bytes;
	__u32	msgs_count;
	__u32	msg_flags;
	__u32	ts_mode;
	__u32	features;

	struct timeval init_time;

	struct pcanfd_msgs *can_tx_msgs;
	struct pcanfd_msgs *can_rx_msgs;

	struct pcanfd_msg_filters *msg_filters;

	__u64	seq_counter;

	__u32	send_calls;
	__u32	recv_calls;
	__u32	should_resend;

	__u32	tx_packets;
	__u32	tx_bytes;
	__u32	tx_eagain;
	__u32	rx_packets;
	__u32	rx_bytes;
	__u32	rx_seq_chk_error;

	int	ids_count;
	__u32	ids[2];;

} pcan_device[TST_DEV_PCAN_MAX];

static void signal_handler(int s);
static void usage(char *errmsg);

/*
 * Writes verbose/debug/normal information to an ouput stream file
 * (default is stdout)
 */
static void lprintf(enum log_level lvl, char *fmt, ...)
{
	va_list ap;
	FILE *pfout = tst_stdlog ? tst_stdlog : stdout;

	va_start(ap, fmt);

	if (lvl != ALWAYS)
		switch (tst_verbose) {
		case NORMAL:
			if (lvl == VERBOSE)
				goto lbl_exit;
		case VERBOSE:
			if (lvl == DEBUG)
				goto lbl_exit;
		case DEBUG:
		default:
			break;
		case QUIET:
			goto lbl_exit;
		}

	if (tst_puts_timestamps) {
		struct timeval tv;
		__gettimeofday(&tv, NULL);
		__fprintf(pfout, "%010u.%06u: ",
			(uint )tv.tv_sec, (uint )tv.tv_usec);
	}

	__vfprintf(pfout, fmt, ap);

lbl_exit:
	va_end(ap);
}

static void init_logs(void)
{
	if (tst_stdlog_filename) {
		tst_stdlog = fopen(tst_stdlog_filename, "w");
		lprintf(VERBOSE, "--- start logging ---\n");
		if (tst_stdlog)
			fflush(tst_stdlog);
	}
}

static void exit_logs(void)
{
	lprintf(VERBOSE, "--- stop logging ---\n");

	if (tst_stdlog) {
		fflush(tst_stdlog);
		fclose(tst_stdlog);
		tst_stdlog = NULL;
	}
}

/*
 * Initialize all what it should be for the application.
 * This function should taken into account that it can be called several times.
 */
static void init_application(void)
{
	struct pcan_device *pdev;
	__u32 non_blocking_mode_flag = 0;
#ifdef PCANFD_OLD_STYLE_API
	DWORD err;
#else
	int err;
#endif
	int i;

	if (tst_mode == TST_MODE_UNKNOWN)
		usage("No test mode specified on command line");

	if (pcan_device_count <= 0)
		usage("No CAN interface specified on command line");

	__gettimeofday(&tst_start, NULL);
	lprintf(DEBUG, "Time base: %u.%06u s.\n",
			(__u32 )tst_start.tv_sec, (__u32 )tst_start.tv_usec);

	init_logs();

	lprintf(VERBOSE, "start opening %d devices:\n", pcan_device_count);

#ifdef RT
	lprintf(DEBUG, "(running RT version)\n");
#else

	/* if more than one pcan dev is being tested, open all devices in 
	 * non-blocking mode */
	//if (pcan_device_count > 1 || tst_mode == TST_MODE_TX) {
	if (pcan_device_count > 1) {
		non_blocking_mode_flag |= OFD_NONBLOCKING;
	}
#endif
	tst_fdmax = -1;
	pcan_device_opened = 0;
	for (pdev = &pcan_device[i = 0]; i < pcan_device_count; i++, pdev++) {
		struct pcanfd_msg *pcan_msg;

		/* init the struct msgs for the device */
		pdev->can_tx_msgs = malloc(sizeof(struct pcanfd_msgs) +
			pdev->msgs_count * sizeof(struct pcanfd_msg));

		if (!pdev->can_tx_msgs) {
			lprintf(ALWAYS, "memory allocation failure!");
			exit(1);
		}

		pdev->can_rx_msgs = malloc(sizeof(struct pcanfd_msgs) +
			pdev->msgs_count * sizeof(struct pcanfd_msg));

		if (!pdev->can_rx_msgs) {
			lprintf(ALWAYS, "memory allocation failure!");
			exit(1);
		}

		pdev->msg_filters = malloc(sizeof(struct pcanfd_msg_filters) +
			1 * sizeof(struct pcanfd_msg_filter));

		if (!pdev->msg_filters) {
			lprintf(ALWAYS, "memory allocation failure!");
			exit(1);
		}
		pdev->msg_filters->count = 1;

		pdev->flags |= non_blocking_mode_flag;

		lprintf(VERBOSE,
			"opening \"%s\" with flags=%08xh "
			"bitrate=%u bps sample_pt=%d "
			"dbitrate=%u bps dsample_pt=%d "
			"clock=%u Hz\n",
			pdev->name, pdev->flags,
			pdev->bitrate, pdev->sample_pt,
			pdev->dbitrate, pdev->dsample_pt,
			pdev->clock_Hz);

		/* check some args */
#ifdef PCANFD_OLD_STYLE_API
		pdev->handle = LINUX_CAN_Open(pdev->name, O_RDWR);
		if (!pdev->handle) {
			lprintf(ALWAYS,
				"failed to open \"%s\" (errno %d)\n",
				pdev->name, errno);
			continue;
		}

		err = CAN_Init(pdev->handle,
				pdev->bitrate,	/* BTR0BTR1 format only! */
				(pdev->flags & PCANFD_MSG_EXT) ?
					CAN_INIT_TYPE_EX : CAN_INIT_TYPE_ST);
		if (err) {
			lprintf(ALWAYS,
				"failed to init \"%s\" to BTR0BTR1=%04xh "
				"(err %d)\n",
				pdev->name, pdev->bitrate, err);
			continue;
		}

		pdev->fd = LINUX_CAN_FileHandle(pdev->handle);
#else
		pdev->fd = pcanfd_open(pdev->name, pdev->flags,
				pdev->bitrate, pdev->sample_pt,
				pdev->dbitrate, pdev->dsample_pt,
				pdev->clock_Hz);

		if (pdev->fd < 0) {
			lprintf(ALWAYS,
				"failed to open \"%s\" (err %d)\n",
				pdev->name, pdev->fd);
			continue;
		}
#endif
		__gettimeofday(&pdev->init_time, NULL);

		pcan_device_opened++;

		if (pdev->fd > tst_fdmax)
			tst_fdmax = pdev->fd;

		lprintf(VERBOSE, "\"%s\" opened (fd=%d)\n",
						pdev->name, pdev->fd);

		/* setup filter mask */
		if (pdev->ids_count > 0) {
			struct pcanfd_msg_filter fm = {
				.id_from = pdev->ids[0],
				.id_to = pdev->ids[1],
				.msg_flags = pdev->msg_flags,
			};

			if (pdev->ids_count == 1)
				fm.id_to = fm.id_from;

			lprintf(VERBOSE, "adding filter [0x%x..0x%x] flg=%xh "
					"to \"%s\"\n",
				fm.id_from, fm.id_to, fm.msg_flags, pdev->name);

			pdev->msg_filters->list[0] = fm;
			err = pcanfd_add_filters(pdev->fd, pdev->msg_filters);
			if (err)
				lprintf(ALWAYS,
					"error %d while adding filter "
					"[%xh..%xh]\n",
					err, fm.id_from, fm.id_to);
		} else {
			/* useles when opening, for tests only */
			err = pcanfd_del_filters(pdev->fd);
			if (err)
				lprintf(ALWAYS,
					"error %d while deleting filters\n",
					err);
		}

		/* init tx messages area (first one indeed) */
		pcan_msg = pdev->can_tx_msgs->list;
		if (!pdev->can_id_random)
			pcan_msg->id = pdev->can_id;

		pdev->can_tx_msgs->count = pdev->msgs_count;

		/* only for tx tests */
		pcan_msg->type = PCANFD_TYPE_CAN20_MSG;
		pcan_msg->flags = pdev->msg_flags;

		if (pdev->flags & PCANFD_INIT_FD) {
			pcan_msg->type = PCANFD_TYPE_CANFD_MSG;
#if 0
			if (pdev->flags & OFD_DBITRATE)
				pcan_msg->flags |= PCANFD_MSG_BRS;
#endif
		}

		/* get channel features */
		err = pcanfd_get_option(pdev->fd,
					PCANFD_OPT_CHANNEL_FEATURES,
					&pdev->features,
					sizeof(pdev->features));
		if (err < 0) {
			lprintf(ALWAYS,
			    	"error %d while reading channel features\n",
				err);

			pdev->features = 0;
		} else {
			lprintf(DEBUG, "channel features: %08xh\n",
				pdev->features);
		}

		/* if command line defines any tx pause value, set the 
		 * corresponding option (if supported) */
		if (pdev->tx_pause_us) {

			err = pcanfd_set_option(pdev->fd,
						PCANFD_OPT_IFRAME_DELAYUS,
						&pdev->tx_pause_us,
						sizeof(pdev->tx_pause_us));
			if (err)
				lprintf(ALWAYS,
				    "error %d while setting TX_PAUSE[%u µs]\n",
				    err, pdev->tx_pause_us);
		}

		if (pdev->ts_mode != PCANFD_OPT_HWTIMESTAMP_MAX) {

			/* (pdev->features & PCANFD_FEATURE_HWTIMESTAMP) */
			err = pcanfd_set_option(pdev->fd,
						PCANFD_OPT_HWTIMESTAMP_MODE,
						&pdev->ts_mode,
						sizeof(pdev->ts_mode));
			if (err)
				lprintf(ALWAYS,
				    "error %d while setting TS mode to %u\n",
				    err, pdev->ts_mode);
		}

		/* init rx msgs area */
		pdev->can_rx_msgs->count = pdev->msgs_count;
	}

	if (!pcan_device_opened)
		usage("No pcan device is opened. Exiting application");

	lprintf(DEBUG, "tst_fdmax=%d\n", tst_fdmax);

	tst_tx_count = 0;
	tst_rx_count = 0;

	tst_sig_caught = 0;
	signal(SIGUSR1, signal_handler);
	signal(SIGHUP, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGINT, signal_handler);

	srand(time(NULL));
}

/*
 * Counter-part of init_application(). 
 * This function has to close/free/release everything that has been
 * opened/allocated/taken during the application life cycle.
 * Moreover, it has to taken into account that it might be called several times.
 */
static void close_application(void)
{
	struct pcan_device *pdev;
	int i;

	for (pdev = &pcan_device[i = 0]; i < pcan_device_count; i++, pdev++) {
#ifdef PCANFD_OLD_STYLE_API
		if (pdev->handle) {
			lprintf(DEBUG, "\"%s\" closed (handle=%p)\n",
				pdev->name, pdev->handle);
			CAN_Close(pdev->handle);
			pdev->handle = NULL;
			pdev->fd = -1;
#else
		if (pdev->fd >= 0) {
			lprintf(DEBUG, "\"%s\" closed (fd=%d)\n",
				pdev->name, pdev->fd);
			pdev->fd = pcanfd_close(pdev->fd);
#endif
			switch (tst_mode) {

			case TST_MODE_TX:
				lprintf(ALWAYS,
					"%s < [packets=%u calls=%u bytes=%u "
					"eagain=%u]\n",
					pdev->name,
					pdev->tx_packets, pdev->send_calls,
					pdev->tx_bytes,
					pdev->tx_eagain);
				break;
			case TST_MODE_RX:
				lprintf(ALWAYS,
					"%s > [packets=%u calls=%u bytes=%u "
					"seq_err=%u]\n",
					pdev->name,
					pdev->rx_packets, pdev->recv_calls,
					pdev->rx_bytes,
					pdev->rx_seq_chk_error);
			default:
				break;
			}
		}
		if (pdev->can_tx_msgs) {
			free(pdev->can_tx_msgs);
			pdev->can_tx_msgs = NULL;
		}
		if (pdev->can_rx_msgs) {
			free(pdev->can_rx_msgs);
			pdev->can_rx_msgs = NULL;
		}
		if (pdev->msg_filters) {
			free(pdev->msg_filters);
			pdev->msg_filters = NULL;
		}
	}

	lprintf(VERBOSE, "all %d devices closed\n", pcan_device_opened);

	exit_logs();
}

/*
 * Do what must be done before exiting the application.
 */
static int exit_application(int err)
{
	close_application();

	if (pcan_device_opened > 0) {

		switch (tst_mode) {

		case TST_MODE_TX:
			lprintf(ALWAYS, "sent frames: %u\n", tst_tx_count);
			break;
		case TST_MODE_RX:
			lprintf(ALWAYS, "received frames: %u\n", tst_rx_count);
		default:
			break;
		}
	}

	exit(err);

	/* just to avoid warnings... */
	return err;
}

static void usage(char *errmsg)
{
	if (errmsg)
		fprintf(stderr, "\n%s\n\n", errmsg);

	fprintf(stderr, "Setup CAN[FD] tests between CAN channels over the pcan driver (>= v8.x)\n");

	fprintf(stderr, "\nWARNING\n");
	fprintf(stderr, "\tThis application comes with ABSOLUTELY NO WARRANTY. This is free\n\tsoftware and you are welcome to redistribute it under certain\n\tconditions. For details, see attached COPYING file.\n");
	fprintf(stderr, "\nUSAGE\n");
	fprintf(stderr, "\t$ pcanfdtst MODE [OPTIONS] CAN [CAN...]\n");
	fprintf(stderr, "\nMODE\n");
	fprintf(stderr, "\ttx    generate CAN traffic on the specified CAN interfaces\n");
	fprintf(stderr, "\trx    check CAN traffic received on the specified CAN interfaces\n");
	fprintf(stderr, "\nCAN\n");
#ifdef RT
	fprintf(stderr, "\tpcanx         indicate which RT CAN interface is used in the test.\n");
#else
	fprintf(stderr, "\t/dev/pcanx    indicate which CAN interface is used in the test.\n");
#endif
	fprintf(stderr, "\t              Several CAN interfaces can be specified. In that case,\n");
	fprintf(stderr, "\t              each one is opened in non-blocking mode.\n");
	fprintf(stderr, "\nOPTIONS\n");
	fprintf(stderr, "\t-a | --accept f-t    add message filter [f...t]\n");
#ifdef PCANFD_OLD_STYLE_API
	fprintf(stderr, "\t-b | --bitrate v     set BTR0BTR1\n");
#else
	fprintf(stderr, "\t-b | --bitrate v     set [nominal] bitrate to \"v\" bps\n");
	fprintf(stderr, "\t     --btr0btr1      bitrates with BTR0BTR1 format\n");
	fprintf(stderr, "\t-B | --brs           data bitrate used for sending CANFD msgs\n");
	fprintf(stderr, "\t-c | --clock v       select clock frequency \"v\" Hz\n");
#endif
	fprintf(stderr, "\t-D | --debug         (maybe too) lot of display\n");
#ifndef PCANFD_OLD_STYLE_API
	fprintf(stderr, "\t-d | --dbitrate v    set data bitrate to \"v\" bps\n");
	fprintf(stderr, "\t     --dsample-pt v  define the data bitrate sample point ratio x 10000\n");
	fprintf(stderr, "\t-f | --fd            select CAN-FD ISO mode\n");
	fprintf(stderr, "\t-F | --fd-non-iso    select CAN-FD non-ISO mode\n");
#endif
	fprintf(stderr, "\t-h | --help          display this help\n");
	fprintf(stderr, "\t-i | --id v|r        set fixed CAN Id. \"v\" or randomly\n");
	fprintf(stderr, "\t-is v|r              set fixed standard CAN Id. \"v\" or randomly\n");
	fprintf(stderr, "\t-ie v|r              set fixed extented CAN Id. \"v\" or randomly\n");
	fprintf(stderr, "\t-I | --incr v        \"v\"=nb of data bytes to use for increment counter\n");
	fprintf(stderr, "\t-l | --len v         set fixed CAN dlc \"v\" for tests\n");
#ifndef PCANFD_OLD_STYLE_API
	fprintf(stderr, "\t-m | --mul v         tx/rx \"v\" msgs at once\n");
#endif
	fprintf(stderr, "\t-n v                 do \"v\" test loops and stop\n");
	fprintf(stderr, "\t-o | --listen-only   set pcan device in listen-only mode\n");
	fprintf(stderr, "\t-p | --pause-us v    \"v\" us. pause between sys calls (rx/tx def=0/%u)\n", tst_pause_us);
	fprintf(stderr, "\t-P | --tx-pause-us v force a pause of \"v\" us. between each Tx frame\n");
	fprintf(stderr, "\t                     (if hw supports it)\n");
	fprintf(stderr, "\t-q | --quiet         nothing is displayed\n");
	fprintf(stderr, "\t-r | --rtr           set the RTR flag to msgs sent\n");
	fprintf(stderr, "\t     --no-rtr        clear the RTR flag from msgs sent\n");
	fprintf(stderr, "\t-s | --stdmsg-only   don't handle extended msgs\n");
	fprintf(stderr, "\t     --sample-pt v   define the bitrate sample point ratio x 10000\n");
	fprintf(stderr, "\t-t | --timeout-ms v  wait \"v\" ms. for events\n");
	fprintf(stderr, "\t-T | --check-ts      check host vs. driver timestatmps, stop if wrong\n");
	fprintf(stderr, "\t     --ts-mode v     set hw timestamp mode to v (hw dependant)\n");
	fprintf(stderr, "\t-u | --bus-load      get bus load notifications from the driver\n");
	fprintf(stderr, "\t-v | --verbose       things are (very much) explained\n");
	fprintf(stderr, "\t-w | --with-ts       logs are prefixed with time of day (s.us)\n");

	fprintf(stderr, "\n");
	exit_application(0);
}

/*
 * strtounit(argv, "kM");
 * strtounit(argv, "ms");
 */
static __u32 strtounit(const char *str, const char *units)
{
	char *endptr;
	__u32 m = 1;

	__u32 v = strtoul(str, &endptr, 0);
	if (*endptr) {
		if (units) {
			const char *pu;

			/* might not be invalid if found char is a unit */
			for (pu = units; *pu; pu++) {
				m *= 1000;
				if (*endptr == *pu)
					break;
			}
			if (!*pu)
				usage("Unknown unit character in numeric "
				      "value on command line");

		} else {
			char tmp[512];
			snprintf(tmp, sizeof(tmp),
				"Invalid character in numeric value \"%s\" on "
				"command line", str);
			usage(tmp);
		}
	}

	lprintf(DEBUG, "\"%s\" converted into %u\n", str, v * m);
	return v * m;
}

static int strtoulist(const char *str, int n, __u32 *pl)
{
	int i;

	if (!pl)
		return 0;

	for (i = 0; *str && (i < n); i++) {

		char *endptr;

		__u32 v	= strtoul(str, &endptr, 0);
		lprintf(DEBUG, "i=%d: \"%s\" converted into %u (end='%c')\n",
				i, str, v, *endptr);
		switch (*endptr) {
		case ',':
		case ':':
		case '-':
			endptr++;
		case '\0':
			str = endptr;
			break;
		default:
			return i;
		}

		*pl++ = v;
	}

	return i;
}

/*
 * Linux signal handler.
 *
 * Note that all children do inherit from their parent's signal handler
 */
static void signal_handler(int s)
{
	lprintf(VERBOSE, "%s(signal=%d)\n", __func__, s);

	switch (s) {

	case SIGHUP:
		close_application();
		init_application();
		tst_sig_caught = s;
		break;

	case SIGUSR1:
		exit_logs();
		init_logs();
		tst_sig_caught = s;
		break;
#ifdef RT
#ifdef XENOMAI
	case SIGXCPU:
		lprintf(VERBOSE,
			"RT task has switched into secondary mode...\n");
		tst_sig_caught = s;
		break;
#endif
	case SIGINT:

		lprintf(VERBOSE, "Stopping RT tasks...\n");

		/* main task has been INTR by user: lprintf RT tasks to stop:
		 * here, we "simply" set the number max of loops to 1 so that
		 * the INTeRrupted tasks will end their test loops and exit. */
		tst_max_loop = 1;
		tst_sig_caught = s;
		break;
#endif
	default:
		tst_sig_caught = 0;
		break;
	}
}

/*
 * Handle a system error in te context of a device.
 */
static enum tst_status handle_errno(int _errno, struct pcan_device *dev)
{
	switch (_errno) {

	case EINTR:
		if (tst_sig_caught) {
			tst_sig_caught = 0;
			return OK;
		}
		lprintf(VERBOSE, "Interrupted!\n");
		break;

	case ENETDOWN:
		lprintf(ALWAYS, "Tx waiting stopped because of BUS-OFF\n");
		break;

	case ETIMEDOUT:
		/* error code returned when Tx task tired to wait for room in
		 * the Tx FIFIO. This generally occurs because the CAN bus
		 * state has changed */
		lprintf(VERBOSE, "WARNING: Tx FIFO ran out of space\n");

		/* return OK to continue trying to write, hoping the bus
		 * state comes back to ACTIVE */
		return OK;

	case EAGAIN:
		if (dev) {
			if (!(dev->flags & OFD_NONBLOCKING))
				lprintf(ALWAYS,
					"ABNORMAL errno %d: system says "
					"task is NOT ABLE to wait!\n", _errno);
			break;
		}

	default:
		lprintf(ALWAYS, "system call failure (errno=%d)\n", _errno);
		break;
	}

	return NOK;
}

static int to_relative_time(struct timeval *ptv, struct timeval *ptb)
{
	if (ptv->tv_sec >= ptb->tv_sec) {
		ptv->tv_sec -= ptb->tv_sec;
		if (ptv->tv_usec >= ptb->tv_usec) {
			ptv->tv_usec -= tst_start.tv_usec;
		} else {
			ptv->tv_sec--;
			ptv->tv_usec += 1000000 - ptb->tv_usec;
		}

		return 1;
	}

	return 0;
}

static enum tst_status do_check_timestamps(struct pcanfd_msg *pcan_msg,
						struct timeval *now)
{
	struct timeval d;

	if (!(pcan_msg->flags & PCANFD_TIMESTAMP))
		return OK;

	/* check if message timestamps is "correct", that is:
	 * - not greater than now
	 * - not *very* different than now
	 */
	timersub(now, &pcan_msg->timestamp, &d);

	if ((d.tv_sec < 0) || (!d.tv_sec && d.tv_usec < 0)) {
		printf("WARNING: message timestamp from the future!\n");
		return NOK;
	}

	if ((d.tv_sec > 0) || (d.tv_usec > PCANFD_TS_MAX_DELTA)) {
		printf("WARNING: message timestamp too far from now "
			"(> 0.%u s.)\n", PCANFD_TS_MAX_DELTA);
		return NOK;
	}

	return OK;
}

/*
 * This function displays an event received from the device.
 * */
static enum tst_status putmsg(struct pcan_device *dev, char dir,
					struct pcanfd_msg *pcan_msg)
{
	char tmp[512];
	int i, l = 0, li;
	struct timeval now, *ptv;
	enum log_level ll = NORMAL;

	__gettimeofday(&now, NULL);

	if (pcan_msg->flags & PCANFD_TIMESTAMP)
		ptv = &pcan_msg->timestamp;
	else {
		ptv = &now;
	}

	/* change to relative time */
	switch (dev->flags & PCANFD_INIT_TS_FMT_MASK) {

	/* use this timestamp mode to get application relative times */
	case PCANFD_INIT_TS_HOST_REL:
		//to_relative_time(ptv, &tst_start);
		break;

	case PCANFD_INIT_TS_DEV_REL:

		/* if timestamp is simulated (Tx test case, for example),
		 * must translate it accoding to the chosen time base */
		if (ptv == &now) {
			to_relative_time(ptv, &dev->init_time);
			break;
		}

	case PCANFD_INIT_TS_DRV_REL:

	/* other cases: display timestamp "as is" */
	default:
		break;
	}

#if 0
	l += sprintf(tmp+l, "now=%llu (%u) %6u.%06u ",
			(unsigned long long )rt_timer_read(), sizeof(RTIME),
			(uint )now.tv_sec, (uint )now.tv_usec);
#endif
	l += sprintf(tmp+l, "%6u%c%06u ",
			(uint )ptv->tv_sec,
			(pcan_msg->flags & PCANFD_HWTIMESTAMP) ? '.' : '~',
			(uint )ptv->tv_usec);

	l += sprintf(tmp+l, "%-11s %c ", dev->name, dir);

	switch (pcan_msg->type) {

	case PCANFD_TYPE_STATUS:

		/* always display status msg, even in Quiet mode */
		ll = ALWAYS;

		switch (pcan_msg->id) {

		case PCANFD_ERROR_ACTIVE:
		case PCANFD_ERROR_WARNING:
		case PCANFD_ERROR_PASSIVE:
		case PCANFD_ERROR_BUSOFF:
			l += sprintf(tmp+l, "BUS STATE=%-8s",
					txt_status[pcan_msg->id]);

			if (pcan_msg->flags & PCANFD_ERRCNT) {
				l += sprintf(tmp+l, " [Rx:%u Tx:%u]",
					pcan_msg->ctrlr_data[PCANFD_RXERRCNT],
					pcan_msg->ctrlr_data[PCANFD_TXERRCNT]);
			}
			break;
		case PCANFD_RX_EMPTY:
		case PCANFD_RX_OVERFLOW:
		case PCANFD_TX_EMPTY:
		case PCANFD_TX_OVERFLOW:
		case PCANFD_BUS_ERROR:
			if (pcan_msg->flags & PCANFD_ERROR_CTRLR)
				l += sprintf(tmp+l, "CTR ERR=%-8s",
						txt_status[pcan_msg->id]);
			else if (pcan_msg->flags & PCANFD_ERROR_INTERNAL)
				l += sprintf(tmp+l, "DRV ERR=%-8s",
						txt_status[pcan_msg->id]);
			break;
		case PCANFD_BUS_LOAD:
			if (pcan_msg->flags & PCANFD_BUSLOAD) {
				l += sprintf(tmp+l, "%s=%u.%02u%%",
					txt_status[pcan_msg->id],
					pcan_msg->ctrlr_data[PCANFD_BUSLOAD_UNIT],
					pcan_msg->ctrlr_data[PCANFD_BUSLOAD_DEC]);
			}

			/* note that this kind os msg is not displayed in QUIET
			 * mode */
			ll = NORMAL;

			break;
		default:
			l += sprintf(tmp+l, "INVALID STATUS=%u", pcan_msg->id);
			break;
		}

		break;

	case PCANFD_TYPE_CAN20_MSG:
	case PCANFD_TYPE_CANFD_MSG:

		if (pcan_msg->flags & PCANFD_MSG_EXT)
			l += sprintf(tmp+l, "%08x ", pcan_msg->id);
		else
			l += sprintf(tmp+l, "     %03x ", pcan_msg->id);

		l += sprintf(tmp+l, "%c%c%c%c%c",
			(pcan_msg->flags & PCANFD_MSG_RTR) ? 'r' : '.',
			(pcan_msg->flags & PCANFD_MSG_EXT) ? 'e' : '.',
			(pcan_msg->flags & PCANFD_MSG_SLF) ? 's' : '.',
			(pcan_msg->flags & PCANFD_MSG_BRS) ? 'b' : '.',
			(pcan_msg->flags & PCANFD_MSG_ESI) ? 'i' : '.');
		li = l;
		for (i = 0; i < pcan_msg->data_len; ) {
			if (!(i % PUTMSG_DATABYTES_PER_LINE)) {
				while (l < li)
					tmp[l++] = ' ';
				tmp[l++] = ' ';
				tmp[l++] = '[';
			}
			l += sprintf(tmp+l, "%02x", pcan_msg->data[i]);
			if (++i >= pcan_msg->data_len) {
				tmp[l++] = ']';
				break;
			}
			if (!(i % PUTMSG_DATABYTES_PER_LINE)) {
				sprintf(tmp+l, "]\n");
				lprintf(NORMAL, tmp);
				l = 0;
				continue;
			}

			tmp[l++] = ' ';
		}
		break;

	default:
		l += sprintf(tmp+l, "%u (Unknown message discarded)",
				pcan_msg->type);
		break;
	}

	l += sprintf(tmp+l, "\n");
	lprintf(ll, tmp);

	if (tst_check_timestamps)
		return do_check_timestamps(pcan_msg, &now);

	return OK;
}

/*
 * This function handles TX test, according to arguments passed on command line
 */
static enum tst_status handle_tx_tst(struct pcan_device *dev)
{
	struct pcanfd_msg *pcan_msg = dev->can_tx_msgs->list;
	int i, err, m;

	if (!dev->should_resend) {

		memset(pcan_msg->data, '\0', sizeof(pcan_msg->data));

		if (dev->can_id_random) {
			int r = rand();
			if (dev->msg_flags & PCANFD_MSG_EXT)
				pcan_msg->id = r & CAN_MAX_EXTENDED_ID;
			else
				pcan_msg->id = r & CAN_MAX_STANDARD_ID;
		}

		if (dev->incr_bytes) {
			__u64 can_counter;

			can_counter = htole64(dev->seq_counter);
			memcpy(pcan_msg->data, &can_counter, dev->incr_bytes);

			dev->seq_counter++;
			pcan_msg->data_len = dev->incr_bytes;
		} else if (dev->data_length_random) {
			int r = rand();

			if (pcan_msg->type == PCANFD_TYPE_CANFD_MSG)
				pcan_msg->data_len = r % (PCANFD_MAXDATALEN+1);
			else
				pcan_msg->data_len = r % (PCAN_MAXDATALEN+1);
		} else {
			pcan_msg->data_len = dev->data_length;
		}

		dev->can_tx_msgs->count = dev->msgs_count;

		/* fill the entire list with a copy of the 1st msg */
		for (i = 1; i < dev->msgs_count; i++)
			memcpy(dev->can_tx_msgs->list + i,
				pcan_msg, sizeof(struct pcanfd_msg));
	}

	if (dev->msgs_count > 1) {
#ifndef USES_MSGS_LIST
		__u32 msgs_to_send = dev->can_tx_msgs->count;

		err = pcanfd_send_msgs(dev->fd, dev->can_tx_msgs);
		lprintf(DEBUG, "pcanfd_send_msgs(%d, %u) returns %d "
			"(msgs count=%d)\n",
			dev->fd, msgs_to_send, err,
			dev->can_tx_msgs->count);

		if (!err)
#else
		err = pcanfd_send_msgs_list(dev->fd,
					    dev->can_tx_msgs->count,
					    dev->can_tx_msgs->list);
		lprintf(DEBUG, "pcanfd_send_msgs_list(%d, %u) returns %d\n",
			dev->fd, dev->can_tx_msgs->count, err);

		if (err > 0)
#endif
			dev->should_resend = 0;

		else if (err == -EAGAIN) {
			/* the file descriptor has been opened in non-blocking
			 * mode, but there is not enough room to store all the
			 * 'dev->can_tx_msgs->count' msgs. 
			 * Have to wait next select(write) ok to resend it... */
			dev->should_resend = 1;
			dev->tx_eagain++;

			/* this is actually not an error */
			err = 0;
		}

	} else {
#ifdef PCANFD_OLD_STYLE_API
		TPCANRdMsg msgv1;

		if (!pcanfd_to_msg(&msgv1, pcan_msg)) {
			lprintf(ALWAYS, "CAN-FD messages won't be sent!\n");
			return NOK;
		}

		err = CAN_Write(dev->handle, &msgv1.Msg);
		lprintf(DEBUG, "CAN_Write(%p) returns %d\n", dev->handle, err);
#else
		err = pcanfd_send_msg(dev->fd, pcan_msg);
		lprintf(DEBUG, "pcanfd_send_msg(%d, "
				"msg id=%xh flags=%08xh len=%u) returns %d\n",
			dev->fd,
			pcan_msg->id, pcan_msg->flags, pcan_msg->data_len,
			err);
#endif
	}

	if (err)
		return handle_errno(-err, dev);

	/* keep the count of msgs really sent */
	m = dev->can_tx_msgs->count;

	tst_tx_count += m;

	dev->send_calls++;
	dev->tx_packets += m;
	dev->tx_bytes += m * pcan_msg->data_len;

	//if (tst_verbose >= NORMAL)
		for (i = 0; i < dev->msgs_count; i++)
			putmsg(dev, '<', pcan_msg);

	return OK;
}

static enum tst_status handle_rx_tst_status(struct pcan_device *dev,
						struct pcanfd_msg *can_msg)
{
	lprintf(DEBUG, "%s(%d)\n", __func__, can_msg->type);

	//if (tst_verbose >= NORMAL)
	return putmsg(dev, '>', can_msg);
}

/*
 * This function handles RX test, according to arguments passed on command line
 */
static enum tst_status handle_rx_tst(struct pcan_device *dev)
{
	struct pcanfd_msg *pcan_msg = dev->can_rx_msgs->list;
	__u64 seq_counter = 0;
	int i, m, err;

	/* be sure to multi read *ONLY* when in RX mode (in TX mode, a single
	 * read MUST be used because this function is called when at least ONE
	 * (and maybe only ONE) msgs is present in the RX queue). */
	if ((tst_mode == TST_MODE_RX) && (dev->msgs_count > 1)) {
#ifndef USES_MSGS_LIST
		dev->can_rx_msgs->count = dev->msgs_count;

		err = pcanfd_recv_msgs(dev->fd, dev->can_rx_msgs);

		lprintf(DEBUG, "pcanfd_recv_msgs(%d, %u) returns %d "
			"(msgs count=%u)\n",
			dev->fd, dev->msgs_count, err,
			dev->can_rx_msgs->count);
#else
		dev->can_rx_msgs->count = 0;

		err = pcanfd_recv_msgs_list(dev->fd,
					dev->msgs_count,
					dev->can_rx_msgs->list);

		lprintf(DEBUG, "pcanfd_recv_msgs_list(%d, %u) returns %d\n",
			dev->fd, dev->msgs_count, err);

		if (err > 0) {
			dev->can_rx_msgs->count = err;
			err = 0;
		}
#endif

	} else {
#ifdef PCANFD_OLD_STYLE_API
		TPCANRdMsg msgv1;

		err = LINUX_CAN_Read(dev->handle, &msgv1);
		lprintf(DEBUG, "LINUX_CAN_Read(%p) returns %d\n",
				dev->handle, err);
		pcanmsg_to_fd(pcan_msg, &msgv1);
#else
		err = pcanfd_recv_msg(dev->fd, pcan_msg);

		lprintf(DEBUG, "pcanfd_recv_msg(%d) returns %d\n",
				dev->fd, err);
#endif
		/* to simplify further processing... */
		dev->can_rx_msgs->count = 1;
	}

	if (err)
		return handle_errno(-err, dev);

	dev->recv_calls++;

	for (m = 0; m < dev->can_rx_msgs->count; m++, pcan_msg++) {

		lprintf(DEBUG, "Got msg type=%u id=%xh flags=%xh len=%u "
			    "ts=%u.%06u s.\n",
				pcan_msg->type, pcan_msg->id, pcan_msg->flags,
				pcan_msg->data_len,
				(__u32 )pcan_msg->timestamp.tv_sec,
				(__u32 )pcan_msg->timestamp.tv_usec);

		if (pcan_msg->type == PCANFD_TYPE_STATUS) {
			if (handle_rx_tst_status(dev, pcan_msg) != OK)
				return NOK;

			continue;
		}

		tst_rx_count++;

		dev->rx_packets++;
		dev->rx_bytes += pcan_msg->data_len;

		//if (tst_verbose >= NORMAL)
		if (putmsg(dev, '>', pcan_msg) != OK)
			return NOK;

		/* if an ID has been specified, check it with the one received.
		 * if not match, the received msg is silently discarded. */
		if (dev->can_id)
			if (pcan_msg->id != dev->can_id)
				continue;

		/* check sequence */
		if (dev->incr_bytes) {
			__u64 can_counter = 0;

			memcpy(&can_counter, pcan_msg->data, dev->incr_bytes);

			seq_counter = le64toh(can_counter);
			if (seq_counter != dev->seq_counter) {
				dev->rx_seq_chk_error++;
				lprintf(ALWAYS,
					"Seq Check Error: %s > %llu (%llxh) "
					"while waiting for %llu (%llxh)\n",
					dev->name, seq_counter, seq_counter,
					dev->seq_counter, dev->seq_counter);
			}

			/* update next sequence number to wait for, if the
			 * number of received packets is a multiple of 
			 * sgs_count */
			if ((dev->rx_packets > 0) &&
					!(dev->rx_packets % dev->msgs_count)) {
				__u64 mask64 = 0;
				for (i = 0; i < dev->incr_bytes; i++) {
					mask64 <<= 8;
					mask64 |= 0xff;
				}
				dev->seq_counter = (seq_counter + 1) & mask64;
			}
		}
	}

	return OK;
}

/*
 * This function handles read/write operations from one device.
 *
 * - If the device has been opened in non-blocking mode (pdev->flags &
 *   OFD_NONBLOCKING), reading/writing operations won't block (single-task AND
 *   multi-device mode)
 *
 * - If the device has been opened in blocking mode (single-task AND
 *   single-device mode OR multi-task mode), then read/write operation is able
 *   to block.
 */
static enum tst_status handle_single_device(struct pcan_device *pdev)
{
	struct pcanfd_state dev_state;
	enum tst_status tst = OK;
	int err;

	/* if file descriptor no more opened, do nothing... */
	if (pdev->fd < 0)
		return OK;

	switch (tst_mode) {

	case TST_MODE_TX:
	case TST_MODE_NONE:

		/* when writing, should also have a look to the read side,
		 * since bus errors are events posted in the device rx queue.
		 * When such an error occurs, the driver wakes up any task
		 * waiting for room in the Tx queue. That's the reason why
		 * we should also check the Rx queue next... */
		err = pcanfd_get_state(pdev->fd, &dev_state);
		if (err) {
			tst = handle_errno(-err, pdev);
			break;
		}

		lprintf(DEBUG, "%s state: hw=%02Xh bus=%u "
			"pending rx=%u/%u tx=%u/%u\n",
			pdev->name, dev_state.hw_type, dev_state.bus_state,
			dev_state.rx_pending_msgs, dev_state.rx_max_msgs,
			dev_state.tx_pending_msgs, dev_state.tx_max_msgs);

		/* if nothing to read, can break here */
		if (!dev_state.rx_pending_msgs)
			break;

		/* otherwise, read the rx queue to process the pending msg */
		if (tst_mode == TST_MODE_NONE)
			break;

	case TST_MODE_RX:
		tst = handle_rx_tst(pdev);
		break;

	default:
		return tst_mode;
	}

	/* handle TX test part here */
	if (tst_mode == TST_MODE_TX)
		tst = handle_tx_tst(pdev);

	if (pdev->pause_us)
		if (__usleep(pdev->pause_us))
			tst = handle_errno(errno, pdev);

	return tst;
}

#ifndef ONE_TASK_PER_DEVICE
/*
 * Handles all the events coming from the opened devices at once (using select()
 * system call).
 *
 * This function is obvioulsy to be used in mono-task environment.
 */
static enum tst_status handle_several_devices(void)
{
	int i, use_select = 0, fd_count;
	struct pcan_device *pdev;
	enum tst_status tst = OK;
	fd_set fds_read, fds_write;

	FD_ZERO(&fds_read);
	FD_ZERO(&fds_write);

	for (pdev = &pcan_device[i = 0]; i < pcan_device_count; i++, pdev++) {

		/* if file descriptor no more opened, do nothing... */
		if (pdev->fd < 0)
			continue;

		/* use select() system call if device has been opened in 
		 * non-blocking mode. Corresponding bit is always set,
		 * to be sure to read/write, even in blocking mode. */
		switch (tst_mode) {
		case TST_MODE_TX:
			FD_SET(pdev->fd, &fds_write);
			if (pdev->flags & OFD_NONBLOCKING)
				use_select++;

#if 1
			/* Note: when writing, should also
			 * have a look to the read side, since
			 * bus errors are events now... */
#else
			break;
#endif
		case TST_MODE_RX:
			FD_SET(pdev->fd, &fds_read);
			if (pdev->flags & OFD_NONBLOCKING)
				use_select++;
		default:
			break;
		}
	}

	/* if, at least, one device is ready to work, wait for any events
	 * coming from the driver. */
	if (use_select) {
		if (tst_timeout_ptr) {
			tst_timeout_ptr->tv_sec = tst_timeout_ms / 1000;
			tst_timeout_ptr->tv_usec =
						(tst_timeout_ms % 1000) * 1000;

			lprintf(DEBUG, "waiting for event on %u pcan "
					"devices during %u ms...\n",
					use_select, tst_timeout_ms);
		} else {
			lprintf(DEBUG, "waiting for event on %u pcan "
					"devices...\n", use_select);
		}

		fd_count = select(tst_fdmax+1, &fds_read, &fds_write,
						NULL, tst_timeout_ptr);
		if (!fd_count) {
			/* timeout */
			lprintf(DEBUG, "Timeout!\n");
			return tst;

		} else if (fd_count < 0) {
			tst = handle_errno(errno, NULL);
		} else {

			lprintf(DEBUG, "got %u event(s) from the pcan "
					"devices\n", fd_count);
		}
	}

	/* now, loop on devices to check which one has something to say... */
	for (pdev = &pcan_device[i = 0]; i < pcan_device_count; i++, pdev++) {

		/* if something happened on the device (or if the device was
		 * opened in blocking-mode, check it now */
		if (FD_ISSET(pdev->fd, &fds_read) ||
					(FD_ISSET(pdev->fd, &fds_write))) {

			tst = handle_single_device(pdev);
			if (tst != OK)
				break;
		}
	}

	return tst;
}
#endif

/*
 * Device task main loop: handle test for a given device
 */
static void dev_main_loop(void *arg)
{
	struct pcan_device *pdev = (struct pcan_device *)arg;
	enum tst_status tst = OK;
	int loop_count;

#ifdef RTAI
	int task_id = getpid() * TST_DEV_PCAN_MAX + (pdev - pcan_device);

	/* RTAI: setup this task for hard real time */
	pdev->rt_task = rt_task_init_schmod(task_id,
						1, 0, 0, SCHED_FIFO, 0xF);
	if (!pdev->rt_task) {
		lprintf(ALWAYS,
			"WARNING: %s(): rt_task_init_schmod(%u) failed\n",
				__func__, task_id);
		//return;
	}

	rt_make_hard_real_time();
#endif

	if (!tst_max_loop)
		lprintf(VERBOSE, "starts infinite loop\n");
	else
		lprintf(VERBOSE, "running %u loops\n", tst_max_loop);

	/* run test forever or until error or end of loop condition */
	for (loop_count = 1; tst == OK; loop_count++) {

		lprintf(DEBUG, "loop #%u (max=%u)\n", loop_count, tst_max_loop);

#ifdef ONE_TASK_PER_DEVICE
		tst = handle_single_device(pdev);
#else
		tst = handle_several_devices();
#endif

		if (tst_max_loop)
			if (loop_count >= tst_max_loop) {
				lprintf(VERBOSE, "stop test after %u loops\n",
						loop_count);
				break;
			}
	}

#ifdef RTAI
	rt_make_soft_real_time();
	if (pdev->rt_task)
		rt_task_delete(pdev->rt_task);
#endif

	lprintf(VERBOSE, "end of test loop (tst=%u).\n", tst);
}

/*
 * Application main process 
 */
static void run_application(void)
{
#ifdef XENOMAI
	struct pcan_device *pdev;
	int i, err;

	/* lock all of the calling process virtual address space into RAM */
	mlockall(MCL_CURRENT | MCL_FUTURE);

	/* be notified if task enters secondary mode */
	signal(SIGXCPU, signal_handler);

	rt_task_set_mode(0, T_WARNSW, NULL);

#ifdef __COBALT__
	/* stdio support is automatically enabled by libcobalt. */
#else
	/* Perform auto-init of rt_print buffers if the task doesn't do so */
	rt_print_auto_init(1);
#endif
	for (pdev = &pcan_device[i = 0]; i < pcan_device_count; i++, pdev++) {
		char tmp[32];

		/* create the RT task that will handle the test on this
		 * device */
		sprintf(tmp, "%s_%d_tst", pdev->name, tst_mode);
		err = rt_task_spawn(&pdev->rt_task,
				    tmp,
				    0,
				    50,
				    0 /*|T_JOINABLE */,
				    dev_main_loop,
				    pdev);
		if (err)
			lprintf(ALWAYS, "failed to spawn the RT task: err=%d\n",
					err);
	}

	/* main process just sleeps... */
	pause();

	for (pdev = &pcan_device[i = 0]; i < pcan_device_count; i++, pdev++) {
		/* destroy the tasks */
		rt_task_delete(&pdev->rt_task);
	}

#elif defined(RTAI)
	struct pcan_device *pdev;
	int i, err;

	rt_allow_nonroot_hrt();

	/* lock all of the calling process virtual address space into RAM */
	mlockall(MCL_CURRENT | MCL_FUTURE);

	for (pdev = &pcan_device[i = 0]; i < pcan_device_count; i++, pdev++) {

		err = pthread_create(&pdev->rt_thread,
				     NULL,
				     (void *(*)(void *))dev_main_loop,
				     pdev);
		if (err)
			lprintf(ALWAYS, "failed to spawn the RT task: err=%d\n",
					err);
	}

	/* main task just sleeps... */
	pause();

	/* on ^C, join all threads before suicide */
	for (pdev = &pcan_device[i = 0]; i < pcan_device_count; i++, pdev++) {
		/* forward ^C to tasks */
		pthread_kill(pdev->rt_thread, SIGINT);

		/* destroy the tasks */
		pthread_join(pdev->rt_thread, NULL);
	}

#else

	/* in non-RT context, simply run a single-task main loop... */
	dev_main_loop(NULL);
#endif
}

int main(int argc, char *argv[])
{
	struct pcan_device *pdev = pcan_device;
	enum {
		IN_BITRATE, IN_SAMPLE_PT,
		IN_DBITRATE, IN_DSAMPLE_PT, IN_TSMODE,
		IN_CLOCK, IN_MAXLOOP, IN_ID, IN_PAUSE, IN_TXPAUSE,
		IN_LENGTH, IN_INCR, IN_TIMEOUT, IN_MUL, IN_ACCEPT,
		IDLE
	} opt_state = IDLE;
	int i;

	for (i = 1; i < argc; i++) {

		lprintf(DEBUG, "state=%u: argv[%u]=\"%s\"\n",
				opt_state, i, argv[i]);

		if (opt_state != IDLE) {
			switch (opt_state) {
			case IN_BITRATE:
#ifdef PCANFD_OLD_STYLE_API
				tst_bitrate = strtounit(argv[i], NULL);
				if (tst_bitrate > 0xffff)
					usage("bitrate MUST match BTR0BTR1 "
						"format");
#else
				tst_bitrate = strtounit(argv[i], "kM");
				tst_flags |= OFD_BITRATE;
#endif
				if (tst_flags & OFD_BTR0BTR1)
					lprintf(DEBUG, "--btr0btr1 0x%04x\n",
							tst_bitrate);
				else
					lprintf(DEBUG, "--bitrate %u\n",
							tst_bitrate);
				break;

			case IN_SAMPLE_PT:
				tst_sample_pt = strtounit(argv[i], NULL);
				lprintf(DEBUG, "--sample_pt %u\n",
					tst_sample_pt);
				break;
#ifndef PCANFD_OLD_STYLE_API
			case IN_DBITRATE:
				tst_dbitrate = strtounit(argv[i], "kM");
				tst_flags |= OFD_DBITRATE;
				if (tst_flags & OFD_BTR0BTR1)
					lprintf(DEBUG, "--dbitrate 0x%04x\n",
							tst_dbitrate);
				else
					lprintf(DEBUG, "--dbitrate %u\n",
							tst_dbitrate);
				break;

			case IN_DSAMPLE_PT:
				tst_dsample_pt = strtounit(argv[i], NULL);
				lprintf(DEBUG, "--dsample_pt %u\n",
					tst_dsample_pt);
				break;

			case IN_CLOCK:
				tst_clock_Hz = strtounit(argv[i], "kM");
				tst_flags |= OFD_CLOCKHZ;
				lprintf(DEBUG, "--clock %u\n", tst_clock_Hz);
				break;
#endif
			case IN_TSMODE:
				tst_ts_mode = strtounit(argv[i], NULL);
				lprintf(DEBUG, "--ts-mode %u\n", tst_ts_mode);
				break;

			case IN_MAXLOOP:
				tst_max_loop = strtounit(argv[i], "kM");
				lprintf(DEBUG, "-n %u\n", tst_max_loop);
				break;

			case IN_ID:
				if (argv[i][0] == 'r' && !argv[i][1]) {
					tst_can_id_random = 1;
					lprintf(DEBUG, "-i%c r\n",
					(tst_msg_flags & PCANFD_MSG_EXT) ?
								'e' : 's');
				} else {
					tst_can_id_random = 0;
					tst_can_id = strtounit(argv[i], NULL);
					lprintf(DEBUG, "-i%c 0x%x\n",
					(tst_msg_flags & PCANFD_MSG_EXT) ?
							'e' : 's', tst_can_id);
				}
				break;

			case IN_ACCEPT:
				if (!strcmp(argv[i], "all"))
					tst_ids_set = 0;
				else {
					tst_ids_set =
						strtoulist(argv[i], 2, tst_ids);
				}
				switch (tst_ids_set) {
				case 2:
					lprintf(DEBUG, "--accept 0x%x-0x%x\n",
							tst_ids[0], tst_ids[1]);
					break;
				case 1:
					lprintf(DEBUG, "--accept %x\n",
							tst_ids[0]);
					break;
				case 0:
					lprintf(DEBUG, "--accept all\n");
				}
				break;

			case IN_LENGTH:
				if (argv[i][0] == 'r' && !argv[i][1]) {
					tst_data_length_random = 1;
					lprintf(DEBUG, "--len r\n");
				} else {
					tst_data_length_random = 0;
					tst_data_length = strtounit(argv[i],
									NULL);
					lprintf(DEBUG, "--len %u\n",
							tst_data_length);
				}
				break;

			case IN_INCR:
				tst_incr_bytes = strtounit(argv[i], NULL);
				lprintf(DEBUG, "--incr %u\n", tst_incr_bytes);
				break;

#ifndef PCANFD_OLD_STYLE_API
			case IN_MUL:
				tst_msgs_count = strtounit(argv[i], NULL);
				lprintf(DEBUG, "--mul %u\n", tst_msgs_count);
				break;
#endif
			case IN_PAUSE:
				tst_pause_us = strtounit(argv[i], "ms");
				lprintf(DEBUG, "--pause-us %u\n", tst_pause_us);
				break;

			case IN_TXPAUSE:
				tst_tx_pause_us = strtounit(argv[i], "ms");
				lprintf(DEBUG, "--tx-pause-us %u\n",
					tst_tx_pause_us);
				break;

			case IN_TIMEOUT:
				tst_timeout_ms = strtounit(argv[i], "s");
				lprintf(DEBUG, "--timeout-ms %u\n",
							tst_timeout_ms);
			default:
				break;
			}

			opt_state = IDLE;
			continue;
		}

		lprintf(DEBUG, "- check for '%c'\n", argv[i][0]);

		if (argv[i][0] == '-') {
			char opt = argv[i][1];

			if (opt == '-') {
				lprintf(DEBUG, "(long option detected)\n");
				if (
				   !strcmp(argv[i]+2, "accept")
				|| !strcmp(argv[i]+2, "bitrate")
#ifndef PCANFD_OLD_STYLE_API
				|| !strcmp(argv[i]+2, "clock")
				|| !strcmp(argv[i]+2, "dbitrate")
				|| !strcmp(argv[i]+2, "fd")
#endif
				|| !strcmp(argv[i]+2, "help")
				|| !strcmp(argv[i]+2, "id")
				|| !strcmp(argv[i]+2, "len")
#ifndef PCANFD_OLD_STYLE_API
				|| !strcmp(argv[i]+2, "mul")
#endif
				|| !strcmp(argv[i]+2, "pause-us")
				|| !strcmp(argv[i]+2, "quiet")
				|| !strcmp(argv[i]+2, "rtr")
				|| !strcmp(argv[i]+2, "stdmsg-only")
				|| !strcmp(argv[i]+2, "timeout-ms")
				|| !strcmp(argv[i]+2, "verbose")
				|| !strcmp(argv[i]+2, "with-ts")
				) {

					opt = argv[i][2];
				} else if (!strcmp(argv[i]+2, "brs")
					|| !strcmp(argv[i]+2, "debug")
#ifndef PCANFD_OLD_STYLE_API
					|| !strcmp(argv[i]+2, "fd-non-iso")
#endif
					|| !strcmp(argv[i]+2, "incr")
					) {
					opt = toupper(argv[i][2]);

				} else if (!strcmp(argv[i]+2, "listen-only")
					) {
					opt = argv[i][9];

				} else if (!strcmp(argv[i]+2, "bus-load")
					) {
					opt = argv[i][3];

				} else if (!strcmp(argv[i]+2, "check-ts")) {
					opt = 'T';

				} else if (!strcmp(argv[i]+2, "no-rtr")) {
					tst_msg_flags &= ~PCANFD_MSG_RTR;
					continue;
				} else if (!strcmp(argv[i]+2, "btr0btr1")) {
					tst_flags |= OFD_BTR0BTR1;
					opt_state = IN_BITRATE;
					continue;
				} else if (!strcmp(argv[i]+2, "sample-pt")) {
					opt_state = IN_SAMPLE_PT;
					continue;
				} else if (!strcmp(argv[i]+2, "dsample-pt")) {
					opt_state = IN_DSAMPLE_PT;
					continue;
				} else if (!strcmp(argv[i]+2, "tx-pause-us")) {
					opt_state = IN_TXPAUSE;
					continue;
				} else if (!strcmp(argv[i]+2, "ts-mode")) {
					opt_state = IN_TSMODE;
					continue;
				}
			}

			lprintf(DEBUG, "- check for option '%c'\n", opt);

			switch (opt) {
			case 'a':
				opt_state = IN_ACCEPT;
				break;
			case 'b':
				opt_state = IN_BITRATE;
				break;
			case 'B':
				tst_msg_flags |= PCANFD_MSG_BRS;
				break;
#ifndef PCANFD_OLD_STYLE_API
			case 'c':
				opt_state = IN_CLOCK;
				break;
			case 'd':
				opt_state = IN_DBITRATE;
				break;
#endif
			case 'D':
				tst_verbose = DEBUG;
				break;
#ifndef PCANFD_OLD_STYLE_API
			case 'f':
				tst_flags |= PCANFD_INIT_FD;
				break;
			case 'F':
				tst_flags |= PCANFD_INIT_FD|\
					     PCANFD_INIT_FD_NON_ISO;
				break;
#endif
			case 'h':
				usage(NULL);
				break;
			case 'i':
				opt_state = IN_ID;
				switch (argv[i][2]) {
				case 'e':
					tst_msg_flags |= PCANFD_MSG_EXT;
					break;
				case 's':
					tst_msg_flags &= ~PCANFD_MSG_EXT;
					break;
				}
				break;
			case 'I':
				opt_state = IN_INCR;
				break;
			case 'l':
				opt_state = IN_LENGTH;
				break;
#ifndef PCANFD_OLD_STYLE_API
			case 'm':
				opt_state = IN_MUL;
				break;
#endif
			case 'n':
				opt_state = IN_MAXLOOP;
				break;
			case 'o':
				tst_flags |= PCANFD_INIT_LISTEN_ONLY;
				break;
			case 'p':
				opt_state = IN_PAUSE;
				break;
			case 'P':
				opt_state = IN_TXPAUSE;
				break;
			case 'q':
				tst_verbose = QUIET;
				break;
			case 'r':
				tst_msg_flags |= PCANFD_MSG_RTR;
				break;

			case 's':
				tst_flags |= PCANFD_INIT_STD_MSG_ONLY;
				break;
			case 't':
				opt_state = IN_TIMEOUT;
				break;
			case 'T':
				tst_check_timestamps = 1;
				break;
			case 'u':
				tst_flags |= PCANFD_INIT_BUS_LOAD_INFO;
				break;
			case 'v':
				tst_verbose = VERBOSE;
				break;
			case 'w':
				tst_puts_timestamps = 1;
				break;
			default:
				usage("Unknow option on command line");
				break;
			}

			continue;
		}

		if (!strncmp(argv[i], "tx", 2)) {
			tst_mode = TST_MODE_TX;
		} else if (!strncmp(argv[i], "rx", 2)) {
			tst_mode = TST_MODE_RX;
			tst_pause_us = 0;
		} else if (!strncmp(argv[i], "none", 4)) {
			tst_mode = TST_MODE_NONE;
		} else if (pcan_device_count < TST_DEV_PCAN_MAX) {
			memset(pdev, '\0', sizeof(*pdev));

			pdev->name = argv[i];
			pdev->fd = -1;
			pdev->flags = tst_flags;
			pdev->bitrate = tst_bitrate;
			pdev->sample_pt = tst_sample_pt;
			pdev->dbitrate = tst_dbitrate;
			pdev->dsample_pt = tst_dsample_pt;
			pdev->clock_Hz = tst_clock_Hz;
			pdev->can_id = tst_can_id;
			pdev->can_id_random = tst_can_id_random;
			pdev->data_length = tst_data_length;
			pdev->data_length_random = tst_data_length_random;
			pdev->pause_us = tst_pause_us;
			pdev->tx_pause_us = tst_tx_pause_us;
			pdev->incr_bytes = tst_incr_bytes;
			pdev->msgs_count = tst_msgs_count;
			pdev->msg_flags = tst_msg_flags;
			pdev->seq_counter = 0;
			pdev->ids_count = tst_ids_set;
			pdev->ts_mode = tst_ts_mode;
			memcpy(pdev->ids, tst_ids,
					pdev->ids_count*sizeof(tst_ids[0]));

			pcan_device_count++;
			pdev++;
		}
	}

	init_application();

	run_application();

	return exit_application(0);
}
