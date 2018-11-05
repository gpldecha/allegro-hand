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
 * Maintainer(s): Stephane Grosjean (s.grosjean@peak-system.com)
 *
 * Major contributions by:
 *                Klaus Hitschler   (klaus.hitschler@gmx.de)
 *                Edouard Tisserant (edouard.tisserant@lolitech.fr) XENOMAI
 *                Laurent Bessard   (laurent.bessard@lolitech.fr)   XENOMAI
 *                Oliver Hartkopp   (oliver.hartkopp@volkswagen.de) socketCAN
 *
 * Contributions: Arnaud Westenberg (arnaud@wanadoo.nl)
 *                Matt Waters (Matt.Waters@dynetics.com)
 *                Benjamin Kolb (Benjamin.Kolb@bigfoot.de)
 *****************************************************************************/

/*****************************************************************************
 *
 * pcan_sja1000.c - all about sja1000 init and data handling
 *
 * $Id$
 *
 *****************************************************************************/
/* #define DEBUG */
/* #undef DEBUG */

//#define PCAN_SJA1000_STATS

/* if defined, bitrate to BTR0BTR1 conversion is speed-up for some common values
 * of bitrate */
//#define PCAN_SJA1000_USES_CONST_BTR0BTR1_IN_CONV

#include "src/pcan_common.h"

#include <linux/sched.h>
#include <asm/errno.h>
#include <asm/byteorder.h>
#include <linux/delay.h>

#include "src/pcan_main.h"
#include "src/pcan_fifo.h"
#include "src/pcan_sja1000.h"
#include "src/pcanfd_core.h"

#ifdef NETDEV_SUPPORT
#include "src/pcan_netdev.h"
#endif

/* sja1000 registers, only PELICAN mode - TUX like it */
#define SJA1000_MOD		0	/* mode register */
#define SJA1000_CMR		1
#define SJA1000_SR		2
#define SJA1000_IR		3
#define SJA1000_IER		4	/* acceptance code */
#define SJA1000_BTR0		6	/* bus timing 0 */
#define SJA1000_BTR1		7	/* bus timing 1 */
#define SJA1000_OCR		8	/* output control */
#define SJA1000_TR		9

#define ARBIT_LOST_CAPTURE    11      /* transmit buffer: Identifier */
#define ERROR_CODE_CAPTURE    12      /* RTR bit und data length code */
#define ERROR_WARNING_LIMIT   13      /* start byte of data field */
#define RX_ERROR_COUNTER      14
#define TX_ERROR_COUNTER      15

#define ACCEPTANCE_CODE_BASE  16
#define RECEIVE_FRAME_BASE    16
#define TRANSMIT_FRAME_BASE   16

#define ACCEPTANCE_MASK_BASE  20

#define RECEIVE_MSG_COUNTER   29
#define RECEIVE_START_ADDRESS 30

#define CLKDIVIDER            31      /* set bit rate and pelican mode */

/* important sja1000 register contents, SJA1000_MOD register */
#define SLEEP_MODE             0x10
#define ACCEPT_FILTER_MODE     0x08
#define SELF_TEST_MODE         0x04
#define LISTEN_ONLY_MODE       0x02
#define RESET_MODE             0x01
#define NORMAL_MODE            0x00

/* SJA1000_CMR register */
#define TRANSMISSION_REQUEST   0x01
#define ABORT_TRANSMISSION     0x02
#define SINGLE_SHOT_REQUEST    0x03
#define RELEASE_RECEIVE_BUFFER 0x04
#define CLEAR_DATA_OVERRUN     0x08
#define SELF_RX_REQUEST        0x10
#define SELF_RX_SS_REQUEST     0x12

/* SJA1000_SR register */
#define SJA1000_SR_BS		0x80	/* bus status bit */
#define SJA1000_SR_ES		0x40	/* error status bit */
#define SJA1000_SR_TS		0x20	/* transmitting status bit */
#define SJA1000_SR_RS		0x10	/* receiving status bit */
#define SJA1000_SR_TCS		0x08	/* transmission complete status bit */
#define SJA1000_SR_TBS		0x04	/* transmit buffer status bit */
#define SJA1000_SR_DOS		0x02	/* data overrun status bit */
#define SJA1000_SR_RBS		0x01	/* receive buffer status bit */

/* INTERRUPT STATUS register */
#define BUS_ERROR_INTERRUPT    0x80
#define ARBIT_LOST_INTERRUPT   0x40
#define ERROR_PASSIV_INTERRUPT 0x20
#define WAKE_UP_INTERRUPT      0x10
#define DATA_OVERRUN_INTERRUPT 0x08
#define ERROR_WARN_INTERRUPT   0x04
#define TRANSMIT_INTERRUPT     0x02
#define RECEIVE_INTERRUPT      0x01

/* INTERRUPT ENABLE register */
#define BUS_ERROR_SJA1000_IER    0x80
#define ARBIT_LOST_SJA1000_IER   0x40
#define ERROR_PASSIV_SJA1000_IER 0x20
#define WAKE_UP_SJA1000_IER      0x10
#define DATA_OVERRUN_SJA1000_IER 0x08
#define ERROR_WARN_SJA1000_IER   0x04
#define TRANSMIT_SJA1000_IER     0x02
#define RECEIVE_SJA1000_IER      0x01

/* OUTPUT CONTROL register */
#define SJA1000_OCR_TRANSISTOR_P1  0x80
#define SJA1000_OCR_TRANSISTOR_N1  0x40
#define SJA1000_OCR_POLARITY_1     0x20
#define SJA1000_OCR_TRANSISTOR_P0  0x10
#define SJA1000_OCR_TRANSISTOR_N0  0x08
#define SJA1000_OCR_POLARITY_0     0x04
#define SJA1000_OCR_MODE_1         0x02
#define SJA1000_OCR_MODE_0         0x01

/* TRANSMIT or RECEIVE BUFFER */
#define BUFFER_EFF                    0x80 /* set for 29 bit identifier */
#define BUFFER_RTR                    0x40 /* set for RTR request */
#define BUFFER_DLC_MASK               0x0f

/* CLKDIVIDER register */
#define CAN_MODE                      0x80
#define CAN_BYPASS                    0x40
#define RXINT_OUTPUT_ENABLE           0x20
#define CLOCK_OFF                     0x08
#define CLOCK_DIVIDER_MASK            0x07

/* additional informations */
#define CLOCK_HZ			(8*MHz)	/* sja100 frequency */

/* time for mode register to change mode */
#define MODE_REGISTER_SWITCH_TIME	100 /* msec */

/* some CLKDIVIDER register contents, hardware architecture dependend */
#define PELICAN_SINGLE	(CAN_MODE | CAN_BYPASS | 0x07 | CLOCK_OFF)
#define PELICAN_MASTER	(CAN_MODE | CAN_BYPASS | 0x07            )
#define PELICAN_DEFAULT	(CAN_MODE                                )
#define CHIP_RESET	PELICAN_SINGLE

/* hardware depended setup for SJA1000_OCR register */
#define SJA1000_OCR_SETUP	(SJA1000_OCR_TRANSISTOR_P0 | \
				 SJA1000_OCR_TRANSISTOR_N0 | \
				 SJA1000_OCR_MODE_1)

/* the interrupt enables */
#define SJA1000_IER_SETUP	(RECEIVE_SJA1000_IER | \
				 TRANSMIT_SJA1000_IER | \
				 DATA_OVERRUN_SJA1000_IER | \
				 BUS_ERROR_SJA1000_IER | \
				 ERROR_PASSIV_SJA1000_IER | \
				 ERROR_WARN_SJA1000_IER)

/* the maximum number of handled messages in one Rx interrupt */
#define MAX_MESSAGES_PER_INTERRUPT	8

/* the maximum number of handled sja1000 interrupts in 1 handler entry */
#define MAX_INTERRUPTS_PER_ENTRY	6	/* max seen is 3 */

/* constants from Arnaud Westenberg email:arnaud@wanadoo.nl */
#define MAX_TSEG1	15
#define MAX_TSEG2	7
#define BTR1_SAM	(1<<1)

static uint irqmaxloop = MAX_INTERRUPTS_PER_ENTRY;
module_param(irqmaxloop, uint, 0644);
MODULE_PARM_DESC(irqmaxloop, " max loops in ISR per CAN (0=default "
				__stringify(MAX_INTERRUPTS_PER_ENTRY)"=def)");

static uint irqmaxrmsg = MAX_MESSAGES_PER_INTERRUPT;
module_param(irqmaxrmsg, uint, 0644);
MODULE_PARM_DESC(irqmaxrmsg, " max msgs read per Rx IRQ (0=default "
				__stringify(MAX_MESSAGES_PER_INTERRUPT)"=def)");

/* Public timing capabilites
 * .sysclock_Hz = 16*MHz =>  Clock Rate = 8 MHz */
const struct pcanfd_bittiming_range sja1000_capabilities = {

	.brp_min = 1,
	.brp_max = 64,
	.brp_inc = 1,

	.tseg1_min = 1,		/* constant for v <= 7.13 */
	.tseg1_max = MAX_TSEG1+1,
	.tseg2_min = 1,		/* constant for v <= 7.13 */
	.tseg2_max = MAX_TSEG2+1,

	.sjw_min = 1,
	.sjw_max = 4,
};

//const struct __array_of_struct(pcanfd_available_clock, 1) sja1000_clocks = {
const pcanfd_mono_clock_device sja1000_clocks = {
	.count = 1,
	.list = {
		[0] = { .clock_Hz = 8*MHz, .clock_src = 16*MHz, },
	}
};

#ifdef PCAN_SJA1000_STATS
/* Well, this is only for debugging purpose since this is not attached to a
 * single device! */
struct pcandev_stats {
	unsigned long int_count;
	unsigned long int_no_count;
	unsigned long int_tx_count;
	unsigned long int_ovr_count;
	unsigned long int_rx_count;
	unsigned long int_err_count;
	unsigned long wakup_w_count;
	unsigned long wakup_r_count;
	unsigned long write_count;
	unsigned long _write_count;
	unsigned long write_frm_count;
	unsigned long max_int;
	unsigned long max_rx_msgs;
};

static struct pcandev_stats dev_stats[4];

static void sja1000_print_stats(const char *name, int c)
{
	struct pcandev_stats *s = dev_stats + c;

	if (c < 0 || c >= 4) {
		pr_err(DEVICE_NAME ": %s(c=%d) channel index not in [0...4[\n",
			__func__, c);
		return;
	}
	pr_info(DEVICE_NAME
		": %s CAN%u: "
		"INT=%lu (NONE=%lu TX=%lu RX=%lu OVR=%lu ERR=%lu MAX=%lu)\n",
		name, c,
		s->int_count, s->int_no_count, s->int_tx_count, s->int_rx_count,
		s->int_ovr_count, s->int_err_count, s->max_int);
	pr_info(DEVICE_NAME ":      WAKEUP_W=%lu WAKEUP_R=%lu MAX_R_MSGS=%lu\n",
		s->wakup_w_count, s->wakup_r_count, s->max_rx_msgs);
	pr_info(DEVICE_NAME ":      WRITE=%lu _WRITE=%lu WRITE_FRM=%lu\n",
		s->write_count, s->_write_count, s->write_frm_count);
}
#endif

static inline u8 __sja1000_write_cmd(struct pcandev *dev, u8 data)
{
	dev->writereg(dev, SJA1000_CMR, data);
	//wmb();

	/* draw a breath after writing the command register */
	return dev->readreg(dev, SJA1000_SR);
}

/* guards writing sja1000's command register in multicore environments */
static inline u8 sja1000_write_cmd(struct pcandev *dev, u8 data)
{
	u8 sr;
	pcan_lock_irqsave_ctxt lck_ctx;

	pcan_lock_get_irqsave(&dev->wlock, lck_ctx);

	sr = __sja1000_write_cmd(dev, data);

	pcan_lock_put_irqrestore(&dev->wlock, lck_ctx);
	//wmb();

	return sr;
}

/* define the maximum loops polling on a SR bit to go to 1.
 * when no udelay() between each readreg(), then this should be > 210.
 * when udelay(10) is put between each readreg(), should be > 21 */
#define SJA1000_SR_MAX_POLL	50

static inline int pcan_pcie_wait_for_status(struct pcandev *dev, u8 mask)
{
	int i;

	for (i = SJA1000_SR_MAX_POLL; i; i--) {
		if (dev->readreg(dev, SJA1000_SR) & mask)
			break;
		udelay(10);
	}

	return i;
}

/* switches the chip into reset mode */
static int set_reset_mode(struct pcandev *dev)
{
	u32 dwStart = get_mtime();
	u8 tmp;

	tmp = dev->readreg(dev, SJA1000_MOD);
	while (!(tmp & RESET_MODE) &&
			((get_mtime() - dwStart) < MODE_REGISTER_SWITCH_TIME)) {
		/* force into reset mode */
		dev->writereg(dev, SJA1000_MOD, RESET_MODE);
		//wmb();
		udelay(1);
		tmp = dev->readreg(dev, SJA1000_MOD);
	}

	if (!(tmp & RESET_MODE))
		return -EIO;

	return 0;
}

/* switches the chip back from reset mode */
static int set_normal_mode(struct pcandev *dev, u8 ucModifier)
{
	u32 dwStart = get_mtime();
	u8  tmp;

	tmp = dev->readreg(dev, SJA1000_MOD);
	while ((tmp != ucModifier) &&
			((get_mtime() - dwStart) < MODE_REGISTER_SWITCH_TIME)) {
		/* force into normal mode */
		dev->writereg(dev, SJA1000_MOD, ucModifier);
		//wmb();
		udelay(1);
		tmp = dev->readreg(dev, SJA1000_MOD);
	}

	if (tmp != ucModifier)
		return -EIO;

	return 0;
}

/* interrupt enable and disable */
static inline void sja1000_irq_enable_mask(struct pcandev *dev, u8 mask)
{
	DPRINTK(KERN_DEBUG "%s: %s(%u, mask=%02xh)\n",
		DEVICE_NAME, __func__, dev->nMinor, mask);

	dev->writereg(dev, SJA1000_IER, mask);
}

static inline void sja1000_irq_disable_mask(struct pcandev *dev, u8 mask)
{
	DPRINTK(KERN_DEBUG "%s: %s(%u, mask=%02xh)\n",
		DEVICE_NAME, __func__, dev->nMinor, mask);

	dev->writereg(dev, SJA1000_IER, ~mask);
}

static inline void sja1000_irq_enable(struct pcandev *dev)
{
	DPRINTK(KERN_DEBUG "%s: %s(%u)\n", DEVICE_NAME, __func__, dev->nMinor);

	//dev->writereg(dev, SJA1000_IER, SJA1000_IER_SETUP);
	sja1000_irq_enable_mask(dev, SJA1000_IER_SETUP);
}

static inline void sja1000_irq_disable(struct pcandev *dev)
{
	DPRINTK(KERN_DEBUG "%s: %s(%u)\n", DEVICE_NAME, __func__, dev->nMinor);

	//dev->writereg(dev, SJA1000_IER, 0);
	sja1000_irq_disable_mask(dev, 0xff);
}

/* find the proper clock divider */
static inline u8 clkdivider(struct pcandev *dev)
{
	/* crystal based */
	if (!dev->props.ucExternalClock)
		return PELICAN_DEFAULT;

	/* configure clock divider register, switch into pelican mode,
	 * depended of of type */
	switch (dev->props.ucMasterDevice) {
	case CHANNEL_SLAVE:
	case CHANNEL_SINGLE:
		/* neither a slave nor a single device distribute the clock */
		return PELICAN_SINGLE;

	default:
		/* ...but a master does */
		return PELICAN_MASTER;
	}
}

/* init CAN-chip */
int sja1000_open(struct pcandev *dev, u16 btr0btr1, u8 bExtended,
		 u8 bListenOnly)
{
	int err;
	u8 _clkdivider = clkdivider(dev);
	u8 ucModifier = (bListenOnly) ? LISTEN_ONLY_MODE : NORMAL_MODE;

	DPRINTK(KERN_DEBUG "%s: %s(%u, btr0btr1=%04xh, ext=%u, lonly=%u)\n",
			DEVICE_NAME, __func__, dev->nMinor, btr0btr1, bExtended,
			bListenOnly);

	sja1000_irq_disable(dev);

	sja1000_write_cmd(dev, ABORT_TRANSMISSION);

	/* switch to reset */
	err = set_reset_mode(dev);
	if (err) {
		pr_err("%s: set_reset_mode(CAN%u) failed (err %d)\n",
				DEVICE_NAME, dev->nChannel+1, err);
		goto fail;
	}

	/* configure clock divider register, switch into pelican mode,
	 * depended of of type */
	dev->writereg(dev, CLKDIVIDER, _clkdivider);

	/* configure acceptance code registers */
	dev->writereg(dev, ACCEPTANCE_CODE_BASE,   0xff);
	dev->writereg(dev, ACCEPTANCE_CODE_BASE+1, 0xff);
	dev->writereg(dev, ACCEPTANCE_CODE_BASE+2, 0xff);
	dev->writereg(dev, ACCEPTANCE_CODE_BASE+3, 0xff);

	/* configure all acceptance mask registers to don't care */
	dev->writereg(dev, ACCEPTANCE_MASK_BASE,   0xff);
	dev->writereg(dev, ACCEPTANCE_MASK_BASE+1, 0xff);
	dev->writereg(dev, ACCEPTANCE_MASK_BASE+2, 0xff);
	dev->writereg(dev, ACCEPTANCE_MASK_BASE+3, 0xff);

	/* configure bus timing registers */
	dev->writereg(dev, SJA1000_BTR0, (u8)((btr0btr1 >> 8) & 0xff));
	dev->writereg(dev, SJA1000_BTR1, (u8)((btr0btr1     ) & 0xff));

	/* configure output control registers */
	dev->writereg(dev, SJA1000_OCR, SJA1000_OCR_SETUP);

	/* clear error counters */
	dev->writereg(dev, RX_ERROR_COUNTER, 0x00);
	dev->writereg(dev, TX_ERROR_COUNTER, 0x00);

	/* clear any pending interrupt */
	dev->readreg(dev, SJA1000_IR);

	/* enter normal operating mode */
	err = set_normal_mode(dev, ucModifier);
	if (err) {
		pr_err("%s: set_normal_mode(CAN%u) failed (err %d)\n",
				DEVICE_NAME, dev->nChannel+1, err);
		goto fail;
	}

#ifdef PCAN_SJA1000_STATS
	memset(dev_stats + dev->nChannel, 0, sizeof(struct pcandev_stats));
#endif

	/* enable CAN interrupts */
	sja1000_irq_enable(dev);

	/* setup (and notify) the initial state to ERROR_ACTIVE */
	pcan_soft_error_active(dev);
fail:
	return err;
}

/* release CAN-chip.
 *
 * This callabck is called by:
 *
 * 1 - ioctl(SET_INIT) if the device was opened before.
 * 2 - close() as the 1st callback called.
 */
void sja1000_release(struct pcandev *dev)
{
	DPRINTK(KERN_DEBUG "%s: %s(%u)\n", DEVICE_NAME, __func__, dev->nMinor);

	/* disable CAN interrupts and set chip in reset mode
	 * Note: According to SJA100 DS, an ABORT cmd generates a TI, so
	 * disable SJA1000 INT before ABORTing*/
	sja1000_irq_disable(dev);

	/* abort pending transmissions */
	sja1000_write_cmd(dev, ABORT_TRANSMISSION);
	//__sja1000_write_cmd(dev, ABORT_TRANSMISSION);

	//sja1000_irq_disable(dev);
	set_reset_mode(dev);

#ifdef PCAN_SJA1000_STATS
	sja1000_print_stats(dev->adapter->name, dev->nChannel);
#endif
}

/* read CAN-data from chip, supposed a message is available */
static int __sja1000_read(struct pcandev *dev)
{
	int msgs = irqmaxrmsg;
	u8 fi, dlc;
	ULCONV localID;
	struct pcanfd_msg f;
	int result = 0;
	int i;

#if 0
	DPRINTK(KERN_DEBUG "%s: %s(%u)\n", DEVICE_NAME, __func__, dev->nMinor);
#endif
	for (msgs = 0; msgs < irqmaxrmsg; msgs++) {

		u8 dreg = dev->readreg(dev, SJA1000_SR);
		if (!(dreg & SJA1000_SR_RBS)) {
			break;
		}

		fi = dev->readreg(dev, RECEIVE_FRAME_BASE);

		dlc = fi & BUFFER_DLC_MASK;
		if (dlc > 8)
			dlc = 8;

		if (fi & BUFFER_EFF) {
			/* extended frame format (EFF) */
			f.flags = MSGTYPE_EXTENDED;
			dreg = RECEIVE_FRAME_BASE + 5;

#if defined(__LITTLE_ENDIAN)
			localID.uc[3] = dev->readreg(dev, RECEIVE_FRAME_BASE+1);
			localID.uc[2] = dev->readreg(dev, RECEIVE_FRAME_BASE+2);
			localID.uc[1] = dev->readreg(dev, RECEIVE_FRAME_BASE+3);
			localID.uc[0] = dev->readreg(dev, RECEIVE_FRAME_BASE+4);
#else
			localID.uc[0] = dev->readreg(dev, RECEIVE_FRAME_BASE+1);
			localID.uc[1] = dev->readreg(dev, RECEIVE_FRAME_BASE+2);
			localID.uc[2] = dev->readreg(dev, RECEIVE_FRAME_BASE+3);
			localID.uc[3] = dev->readreg(dev, RECEIVE_FRAME_BASE+4);
#endif
			f.id = localID.ul >> 3;

		} else {
			/* standard frame format (SFF) */
			f.flags = MSGTYPE_STANDARD;
			dreg = RECEIVE_FRAME_BASE + 3;

			localID.ul = 0;
#if defined(__LITTLE_ENDIAN)
			localID.uc[3] = dev->readreg(dev, RECEIVE_FRAME_BASE+1);
			localID.uc[2] = dev->readreg(dev, RECEIVE_FRAME_BASE+2);
#else
			localID.uc[0] = dev->readreg(dev, RECEIVE_FRAME_BASE+1);
			localID.uc[1] = dev->readreg(dev, RECEIVE_FRAME_BASE+2);
#endif
			f.id = localID.ul >> 21;
		}

		/* clear aligned data section */
		*(__u64 *)&f.data[0] = (__u64)0;

		for (i = 0; i < dlc; i++)
			f.data[i] = dev->readreg(dev, dreg++);

		/* release the receive buffer asap */

		/* Note: [AN97076 p 35]: "(in PeliCAN mode the Receive
		 * Interrupt (RI) is cleared first, when giving the Release
		 * Buffer command) */
#ifdef PCAN_SJA1000_LOCK_ENTIRE_ISR
		/* SGr note: no need any other mutex access */
		dreg = __sja1000_write_cmd(dev, RELEASE_RECEIVE_BUFFER);
#else
		dreg = sja1000_write_cmd(dev, RELEASE_RECEIVE_BUFFER);
#endif

		/* complete now frame to give to application: */
		f.type = PCANFD_TYPE_CAN20_MSG;
		if (fi & BUFFER_RTR)
			f.flags |= MSGTYPE_RTR;

		f.data_len = dlc;

		/* put into specific data sink and save the last result */
		if (pcan_xxxdev_rx(dev, &f) > 0)
			result++;
#if 1
		/* SGr note: why? */
#else
		/* give time to settle */
		udelay(1);
#endif
	}

#ifdef PCAN_SJA1000_STATS
	if (dev_stats[dev->nChannel].max_rx_msgs < msgs)
		dev_stats[dev->nChannel].max_rx_msgs = msgs;
#endif
	return result;
}

/* write CAN-data to chip */
static int sja1000_write_msg(struct pcandev *dev, struct pcanfd_msg *pf)
{
	ULCONV localID;
	u8 fi, dreg, cmd;
	int i;

#if 1
	if (!(dev->readreg(dev, SJA1000_SR) & SJA1000_SR_TBS))
		pr_warn(DEVICE_NAME ": CAN%u TBS==0!\n", dev->nChannel+1);
#endif

	fi = pf->data_len;
	localID.ul = pf->id;

#ifdef PCAN_SJA1000_STATS
	dev_stats[dev->nChannel].write_frm_count++;
#endif

	if (pf->flags & PCANFD_MSG_RTR)
		fi |= BUFFER_RTR;

	if (pf->flags & PCANFD_MSG_EXT) {
		dreg = TRANSMIT_FRAME_BASE + 5;
		fi |= BUFFER_EFF;

		localID.ul <<= 3;

		dev->writereg(dev, TRANSMIT_FRAME_BASE, fi);

#if defined(__LITTLE_ENDIAN)
		dev->writereg(dev, TRANSMIT_FRAME_BASE + 1, localID.uc[3]);
		dev->writereg(dev, TRANSMIT_FRAME_BASE + 2, localID.uc[2]);
		dev->writereg(dev, TRANSMIT_FRAME_BASE + 3, localID.uc[1]);
		dev->writereg(dev, TRANSMIT_FRAME_BASE + 4, localID.uc[0]);
#else
		dev->writereg(dev, TRANSMIT_FRAME_BASE + 1, localID.uc[0]);
		dev->writereg(dev, TRANSMIT_FRAME_BASE + 2, localID.uc[1]);
		dev->writereg(dev, TRANSMIT_FRAME_BASE + 3, localID.uc[2]);
		dev->writereg(dev, TRANSMIT_FRAME_BASE + 4, localID.uc[3]);
#endif
	} else {
		dreg = TRANSMIT_FRAME_BASE + 3;

		localID.ul <<= 21;

		dev->writereg(dev, TRANSMIT_FRAME_BASE, fi);

#if defined(__LITTLE_ENDIAN)
		dev->writereg(dev, TRANSMIT_FRAME_BASE + 1, localID.uc[3]);
		dev->writereg(dev, TRANSMIT_FRAME_BASE + 2, localID.uc[2]);
#else
		dev->writereg(dev, TRANSMIT_FRAME_BASE + 1, localID.uc[0]);
		dev->writereg(dev, TRANSMIT_FRAME_BASE + 2, localID.uc[1]);
#endif
	}

	for (i = 0; i < pf->data_len; i++)
		dev->writereg(dev, dreg++, pf->data[i]);

	/* request a transmission */

	if (pf->flags & PCANFD_MSG_SNG) {
		if (pf->flags & PCANFD_MSG_SLF)
			cmd = SELF_RX_SS_REQUEST;
		else
			cmd = SINGLE_SHOT_REQUEST;
	} else {
		if (pf->flags & PCANFD_MSG_SLF)
			cmd = SELF_RX_REQUEST;
		else
			cmd = TRANSMISSION_REQUEST;
	}

	/* SGr Note: why using a mutex for a single command while we're already
	 * in a critical section? */
	//__sja1000_write_cmd(dev, cmd);
	sja1000_write_cmd(dev, cmd);

#if 1
	/* Note: waiting for TBS is not a good idea here because this function
	 * is caleld by the ISR, and the ISR doesn't need (and shouldn't)
	 * actually wait! */
#else
	/* PCIe: be sure that the transmission has completed */
	if (!pcan_pcie_wait_for_status(dev, SJA1000_SR_TBS)) {
		pr_info(DEVICE_NAME
			": %s CAN%u: ABNORMAL state: TBS==0\n", 
			dev->adapter->name, dev->nChannel+1);
		return -EIO;
	}
#endif

	/* Tx engine is started: next write will be made by ISR */
	pcan_set_tx_engine(dev, TX_ENGINE_STARTED);

	return 0;
}

static int __sja1000_write(struct pcandev *dev, struct pcan_udata *ctx)
{
	struct pcanfd_msg pf;
	int err;

	/* get a fifo element and step forward */
	err = pcan_fifo_get(&dev->writeFifo, &pf);
	if (!err) {
		err = sja1000_write_msg(dev, &pf);

#ifdef PCAN_SJA1000_STATS
		dev_stats[dev->nChannel].write_count++;
#endif
	}

	return err;
}

/* write CAN-data from FIFO to chip
 *
 * This function is generally called from user-space task, when TX_ENGINE is
 * STOPPED. This call is protected by isr_lock lock.
 */
int sja1000_write(struct pcandev *dev, struct pcan_udata *ctx)
{
	int err = -EIO;

#if 0
	DPRINTK(KERN_DEBUG "%s: %s(%u) %d\n",
		DEVICE_NAME, __func__, dev->nMinor, dev->writeFifo.nStored);
#endif
	/* Check (and wait for) the SJA1000 Tx buffer to be empty before
	 * writing in */
	if (pcan_pcie_wait_for_status(dev, SJA1000_SR_TBS))
		err = __sja1000_write(dev, ctx);
	else
		pr_err(DEVICE_NAME
			": %s() failed: %s CAN%u Transmit Buffer not empty\n",
			__func__, dev->adapter->name, dev->nChannel+1);

	return err;
}

/* SJA1000 interrupt handler */
irqreturn_t __pcan_sja1000_irqhandler(struct pcandev *dev)
{
	irqreturn_t ret = PCAN_IRQ_NONE;
	int j, err;
	u32 rwakeup = 0;
	u32 wwakeup = 0;
	int tx_frames_count = 0;
	struct pcanfd_msg ef;

#if 0
	/* check if INT are enabled for this device */
	if (!dev->readreg(dev, SJA1000_IER))
		return ret;
#endif
	memset(&ef, 0, sizeof(ef));

	for (j = 0; j < irqmaxloop; j++) {

		/* Note: [AN97076 p 35]: "all interrupt flags are cleared" */
		u8 irqstatus = dev->readreg(dev, SJA1000_IR);
		u8 chipstatus;

#if 0
		chipstatus = dev->readreg(dev, SJA1000_SR);
		if (chipstatus & SJA1000_SR_RBS)
			irqstatus |= RECEIVE_INTERRUPT;
#endif
		if (!irqstatus)
			break;

		dev->dwInterruptCounter++;

		/* quick hack to badly workaround write stall
		 * if ((irqstatus & TRANSMIT_INTERRUPT) ||
		 *     (!atomic_read(&dev->hw_is_ready_to_send) &&
		 *      !pcan_fifo_empty(&dev->writeFifo) &&
		 *      (dev->readreg(dev, SJA1000_SR) & SJA1000_SR_TBS)))
		 */
		if (irqstatus & TRANSMIT_INTERRUPT) {
#ifndef PCAN_SJA1000_LOCK_ENTIRE_ISR
			pcan_lock_irqsave_ctxt flags;

			pcan_lock_get_irqsave(&dev->isr_lock, flags);
#endif
#ifdef PCAN_SJA1000_STATS
			dev_stats[dev->nChannel].int_tx_count++;
#endif
#if 0
			chipstatus = dev->readreg(dev, SJA1000_SR);
			if (!(chipstatus & SJA1000_SR_TCS)) {
				pr_info(DEVICE_NAME ": CAN%u: TCS=0\n",
					dev->nChannel+1);
			}
			if (chipstatus & SJA1000_SR_TS) {
				pr_info(DEVICE_NAME ": CAN%u: TS=1\n",
					dev->nChannel+1);
			}
#endif

			/* handle transmission */
			err = __sja1000_write(dev, NULL);
			switch (err) {
			case -ENODATA:
				pcan_set_tx_engine(dev, TX_ENGINE_STOPPED);
				wwakeup++;
				break;
			case 0:
				tx_frames_count++;
				dev->tx_frames_counter++;
				break;
			default:
				dev->nLastError = err;
				dev->dwErrorCounter++;
				dev->wCANStatus |= CAN_ERR_QXMTFULL;
			}

#ifndef PCAN_SJA1000_LOCK_ENTIRE_ISR
			pcan_lock_put_irqrestore(&dev->isr_lock, flags);
#endif

			/* reset to ACTIVITY_IDLE by cyclic timer */
			dev->ucActivityState = ACTIVITY_XMIT;
		}

		if (irqstatus & RECEIVE_INTERRUPT) {
#ifdef PCAN_SJA1000_STATS
			dev_stats[dev->nChannel].int_rx_count++;
#endif
#if 0
			chipstatus = dev->readreg(dev, SJA1000_SR);
			if (!(chipstatus & SJA1000_SR_RBS)) {
				pr_info(DEVICE_NAME ": CAN%u: RBS=0\n",
					dev->nChannel+1);
			}
			if (chipstatus & SJA1000_SR_RS) {
				pr_info(DEVICE_NAME ": CAN%u: RS=1\n",
					dev->nChannel+1);
			}
#endif
			/* handle reception: put to input queues */
			err = __sja1000_read(dev);

			/* successfully enqueued at least ONE msg into FIFO */
			if (err > 0)
				rwakeup++;

			/* reset to ACTIVITY_IDLE by cyclic timer */
			dev->ucActivityState = ACTIVITY_XMIT;
		}

		if (irqstatus & DATA_OVERRUN_INTERRUPT) {
#ifdef PCAN_SJA1000_STATS
			dev_stats[dev->nChannel].int_ovr_count++;
#endif
#if 0
			chipstatus = dev->readreg(dev, SJA1000_SR);
			if (!(chipstatus & SJA1000_SR_DOS)) {
				pr_info(DEVICE_NAME ": CAN%u: DOS=0\n",
					dev->nChannel+1);
			}
#endif
#ifdef PCAN_SJA1000_LOCK_ENTIRE_ISR
			/* SGr note: no need any other exclusive access */
			__sja1000_write_cmd(dev, CLEAR_DATA_OVERRUN);
#else
			sja1000_write_cmd(dev, CLEAR_DATA_OVERRUN);
#endif

#if 0
			DPRINTK(KERN_DEBUG "%s: %s(%d), DATA_OVR\n",
				DEVICE_NAME, __func__, dev->nMinor);
#endif
			/* handle data overrun */
			pcan_handle_error_ctrl(dev, &ef, PCANFD_RX_OVERFLOW);

			/* reset to ACTIVITY_IDLE by cyclic time */
			dev->ucActivityState = ACTIVITY_XMIT;
		}

		/* should have a look to RX_ERROR_COUNTER and TX_ERROR_COUNTER
		 * SJA1000 registers! Reading these value could be nice... */
		if (irqstatus & (ERROR_PASSIV_INTERRUPT|ERROR_WARN_INTERRUPT)) {

			chipstatus = dev->readreg(dev, SJA1000_SR);

			DPRINTK(KERN_DEBUG
				"%s: %s(%u), irqstatus=%xh chipstatus=0x%02x\n",
				DEVICE_NAME, __func__, dev->nMinor,
				irqstatus, chipstatus);

			switch (chipstatus & (SJA1000_SR_BS | SJA1000_SR_ES)) {
			case 0x00:
				/* error active, clear only local status */
				pcan_handle_error_active(dev, &ef);
				break;
			case SJA1000_SR_BS:
			case SJA1000_SR_BS | SJA1000_SR_ES:
				/* bus-off */
				pcan_handle_busoff(dev, &ef);
				break;
			case SJA1000_SR_ES:

				/* either enter or leave error passive status */
				if (irqstatus & ERROR_PASSIV_INTERRUPT) {
					/* enter error passive state */
					pcan_handle_error_status(dev, &ef,
									0, 1);
				} else {
					/* warning limit reached event */
					pcan_handle_error_status(dev, &ef,
									1, 0);
				}
				break;
			}

			/* (simply to enter into next condition) */
			irqstatus |= BUS_ERROR_INTERRUPT;

		} else if (irqstatus & BUS_ERROR_INTERRUPT) {

			u8 ecc = dev->readreg(dev, ERROR_CODE_CAPTURE);

			/* count each error signal even if it does not change
			 * any bus nor error state */
			dev->dwErrorCounter++;
#ifdef PCAN_SJA1000_STATS
			dev_stats[dev->nChannel].int_err_count++;
#endif
			pcan_handle_error_msg(dev, &ef,
					(ecc & 0xc0) >> 6, (ecc & 0x1f),
					(ecc & 0x20), 0);

			DPRINTK(KERN_DEBUG "%s: %s(%u) BUS_ERROR %02xh\n",
				DEVICE_NAME, __func__, dev->nMinor, ecc);
		}

		if (irqstatus & BUS_ERROR_INTERRUPT) {

			dev->rx_error_counter = 
				dev->readreg(dev, RX_ERROR_COUNTER);
			dev->tx_error_counter =
				dev->readreg(dev, TX_ERROR_COUNTER);

			/* reset to ACTIVITY_IDLE by cyclic timer */
			dev->ucActivityState = ACTIVITY_XMIT;
		}

		/* if any error condition occurred, send an error frame to
		 * userspace */
		if (ef.type) {

			/* put into specific data sink */
			if (pcan_xxxdev_rx(dev, &ef) > 0)
				rwakeup++;

			/* clear for next loop */
			memset(&ef, 0, sizeof(ef));
		}

		ret = PCAN_IRQ_HANDLED;
	}


#ifdef PCAN_SJA1000_STATS
	if (dev_stats[dev->nChannel].max_int < j)
		dev_stats[dev->nChannel].max_int = j;

	if (ret == PCAN_IRQ_HANDLED)
		dev_stats[dev->nChannel].int_count++;
#endif

	if (wwakeup) {
#ifdef PCAN_SJA1000_STATS
		dev_stats[dev->nChannel].wakup_w_count++;
#endif
		/* signal I'm ready to write */
		pcan_event_signal(&dev->out_event);

#ifdef NETDEV_SUPPORT
		if (dev->netdev)
			netif_wake_queue(dev->netdev);
#endif
	}

	if (rwakeup) {
#ifdef PCAN_SJA1000_STATS
		dev_stats[dev->nChannel].wakup_r_count++;
#endif
		pcan_event_signal(&dev->in_event);
	}

	return ret;
}

irqreturn_t pcan_sja1000_irqhandler(struct pcandev *dev)
{
	irqreturn_t err;
#ifdef PCAN_SJA1000_LOCK_ENTIRE_ISR
	pcan_lock_irqsave_ctxt lck_ctx;

	pcan_lock_get_irqsave(&dev->isr_lock, lck_ctx);
#endif

	err = __pcan_sja1000_irqhandler(dev);

#ifdef PCAN_SJA1000_LOCK_ENTIRE_ISR
	pcan_lock_put_irqrestore(&dev->isr_lock, lck_ctx);
#endif

	return err;
}

#ifndef NO_RT
int sja1000_irqhandler(rtdm_irq_t *irq_context)
{
	struct pcan_udata *ctx = rtdm_irq_get_arg(irq_context,
							struct pcan_udata);
	struct pcandev *dev = ctx->dev;

#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
irqreturn_t sja1000_irqhandler(int irq, void *arg, struct pt_regs *pt)
{
	struct pcandev *dev = (struct pcandev *)arg;
#else
irqreturn_t sja1000_irqhandler(int irq, void *arg)
{
	struct pcandev *dev = (struct pcandev *)arg;
#endif

	return pcan_sja1000_irqhandler(dev);
}

/* probe for a sja1000 - use it only in reset mode! */
int sja1000_probe(struct pcandev *dev)
{
	u8 tmp;
	u8 _clkdivider = clkdivider(dev);

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	/* do some check on module parameters */
	if (!irqmaxloop)
		irqmaxloop = MAX_INTERRUPTS_PER_ENTRY;

	if (!irqmaxrmsg)
		irqmaxrmsg = MAX_MESSAGES_PER_INTERRUPT;

	/* trace the clockdivider register to test for sja1000 / 82c200 */
	tmp = dev->readreg(dev, CLKDIVIDER);
	DPRINTK(KERN_DEBUG "%s: CLKDIVIDER traced (0x%02x)\n",
		DEVICE_NAME, tmp);

	if (tmp & 0x10)
		goto fail;

	/*  until here, it's either a 82c200 or a sja1000 */
	if (set_reset_mode(dev))
		goto fail;

	/* switch to PeliCAN mode */
	dev->writereg(dev, CLKDIVIDER, _clkdivider);

	/* precautionary disable interrupts */
	sja1000_irq_disable(dev);
	//wmb();
	DPRINTK(KERN_DEBUG "%s: Hopefully switched to PeliCAN mode\n",
		DEVICE_NAME);

	/* new 7.5: PELICAN mode takes sometimes longer: adding some delay
	 * solves the problem (many thanks to Hardi Stengelin)
	 */
	udelay(10);  /* Wait until the pelican mode is activ */

	tmp = dev->readreg(dev, SJA1000_SR);
	DPRINTK(KERN_DEBUG "%s: SJA1000_SR traced (0x%02x)\n",
		DEVICE_NAME, tmp);
	if ((tmp & 0x30) != 0x30)
		goto fail;

	if (tmp & SJA1000_SR_TBS)
		/* Writing is now ok */
		pcan_set_tx_engine(dev, TX_ENGINE_STOPPED);

	/* clear any pending INT */
	tmp = dev->readreg(dev, SJA1000_IR);
	DPRINTK(KERN_DEBUG "%s: SJA1000_IR traced (0x%02x)\n",
		DEVICE_NAME, tmp);
	if (tmp & 0xfb)
		goto fail;

	tmp = dev->readreg(dev, RECEIVE_MSG_COUNTER);
	DPRINTK(KERN_DEBUG "%s: RECEIVE_MSG_COUNTER traced (0x%02x)\n",
		DEVICE_NAME, tmp);
	if (tmp)
		goto fail;

	DPRINTK(KERN_DEBUG "%s: %s() is OK\n", DEVICE_NAME, __func__);
	return 0;

fail:
	DPRINTK(KERN_DEBUG "%s: %s() failed\n", DEVICE_NAME, __func__);

	/* no such device or address */
	return -ENXIO;
}

#if 1
/*
 * No more used since v8.x: pcan_timing.c now handles everything that concerns
 * bittiming for all kind of CAN hardware.
 */
#else
/*
 * calculate BTR0BTR1 for odd bitrates
 *
 * most parts of this code is from Arnaud Westenberg email:arnaud@wanadoo.nl
 * www.home.wanadoo.nl/arnaud
 *
 * Set communication parameters.
 * param rate baud rate in Hz
 * param clock frequency of sja1000 clock in Hz
 * param sjw synchronization jump width (0-3) prescaled clock cycles
 * param sampl_pt sample point in % (0-100) sets (TSEG1+2)/(TSEG1+TSEG2+3) ratio
 * param flags fields BTR1_SAM, OCMODE, OCPOL, OCTP, OCTN, CLK_OFF, CBP
 */
static int sja1000_baud_rate(int rate, int flags)
{
	int best_error = 1000000000;
	int error;
	int best_tseg = 0, best_brp = 0, best_rate = 0, brp = 0;
	int tseg = 0, tseg1 = 0, tseg2 = 0;
	int clock = CLOCK_HZ;
	u16 wBTR0BTR1;
	int sjw = 0;
	int sampl_pt = 90;

	/* some heuristic specials */
	if (rate > ((1000000 + 500000) / 2))
		sampl_pt = 75;

	if (rate < ((12500 + 10000) / 2))
		sampl_pt = 75;

	if (rate < ((100000 + 125000) / 2))
		sjw = 1;

	/* tseg even = round down, odd = round up */
	for (tseg = (0 + 0 + 2) * 2;
			tseg <= (MAX_TSEG2 + MAX_TSEG1 + 2) * 2 + 1; tseg++) {

		brp = clock / ((1 + tseg / 2) * rate) + tseg % 2;
		if ((brp == 0) || (brp > 64))
			continue;

		error = rate - clock / (brp * (1 + tseg / 2));
		if (error < 0)
			error = -error;

		if (error <= best_error) {
			best_error = error;
			best_tseg = tseg/2;
			best_brp = brp-1;
			best_rate = clock/(brp*(1+tseg/2));
		}
	}

	if (best_error && (rate / best_error < 10)) {
		DPRINTK(KERN_ERR
			"%s: bitrate %d is not possible with %d Hz clock\n",
			DEVICE_NAME, rate, 2 * clock);

		return 0;
	}

	tseg2 = best_tseg - (sampl_pt * (best_tseg + 1)) / 100;

	if (tseg2 < 0)
		tseg2 = 0;

	if (tseg2 > MAX_TSEG2)
		tseg2 = MAX_TSEG2;

	tseg1 = best_tseg - tseg2 - 2;

	if (tseg1 > MAX_TSEG1) {
		tseg1 = MAX_TSEG1;
		tseg2 = best_tseg-tseg1-2;
	}

	wBTR0BTR1 = ((sjw<<6 | best_brp) << 8) | \
			(((flags & BTR1_SAM) != 0)<<7 | tseg2<<4 | tseg1);

	return wBTR0BTR1;
}
#endif

/* get BTR0BTR1 init values */
u16 sja1000_bitrate(u32 dwBitRate, u32 sample_pt)
{
	struct pcan_bittiming bt;
	u16 wBTR0BTR1;

#ifdef PCAN_SJA1000_USES_CONST_BTR0BTR1_IN_CONV
	/* get default const values */
	switch (dwBitRate) {
	case 1000000:
		wBTR0BTR1 = CAN_BAUD_1M;
		break;
	case 500000:
		wBTR0BTR1 = CAN_BAUD_500K;
		break;
	case 250000:
		wBTR0BTR1 = CAN_BAUD_250K;
		break;
	case 125000:
		wBTR0BTR1 = CAN_BAUD_125K;
		break;
	case 100000:
		wBTR0BTR1 = CAN_BAUD_100K;
		break;
	case 50000:
		wBTR0BTR1 = CAN_BAUD_50K;
		break;
	case 20000:
		wBTR0BTR1 = CAN_BAUD_20K;
		break;
	case 10000:
		wBTR0BTR1 = CAN_BAUD_10K;
		break;
	case 5000:
		wBTR0BTR1 = CAN_BAUD_5K;
		break;
	case 0:
		wBTR0BTR1 = 0;
		break;

	default:
		/* calculate for exotic values */
#else
	{
#endif
		memset(&bt, '\0', sizeof(bt));
		bt.bitrate = dwBitRate;
		bt.sjw = sja1000_capabilities.sjw_min;
		bt.sample_point = sample_pt;

		pcan_bitrate_to_bittiming(&bt, &sja1000_capabilities, CLOCK_HZ);

		wBTR0BTR1  = ((bt.sjw - 1) & 0x3) << 14;
		wBTR0BTR1 |= ((bt.brp - 1) & 0x3f) << 8;
		wBTR0BTR1 |= (!!bt.tsam) << 7;
		wBTR0BTR1 |= ((bt.tseg2 - 1) & 0x7) << 4;
		wBTR0BTR1 |= ((bt.tseg1 - 1) & 0xf);
	}

	DPRINTK(KERN_DEBUG "%s: %s() %u bps = 0x%04x\n",
		DEVICE_NAME, __func__, dwBitRate, wBTR0BTR1);

	return wBTR0BTR1;
}
