/*****************************************************************************
 * Copyright (C) 2006-2010  PEAK System-Technik GmbH
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

/****************************************************************************
 *
 * system dependend parts to handle pcan-pccard
 * special code for kernels greater and equal than 2.6.17
 *
 * $Id$
 *
 *****************************************************************************/

/*
 * called at CARD_REMOVAL
 */
static void pccard_plugout(struct pcmcia_device *link)
{
	PCAN_PCCARD *card = link->priv;

	DPRINTK(KERN_DEBUG "%s: %s(0x%p)\n", DEVICE_NAME, __func__, link);

	pccard_release_all_devices(card);
	pcmcia_disable_device(link);
}

/* helper function to get retrieved configuraton */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)
static int pcan_pccard_conf_check(struct pcmcia_device *link,
				cistpl_cftable_entry_t *cfg,
				cistpl_cftable_entry_t *dflt,
				unsigned int vcc, void *private)
{
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	if ((cfg->io.nwin > 0) || (dflt->io.nwin > 0)) {
		cistpl_io_t *io  = (cfg->io.nwin > 0) ? &cfg->io  : &dflt->io;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
		link->io.BasePort1 = io->win[0].base;
		link->io.NumPorts1 = io->win[0].len;

		/* only this kind of access is yet supported */
		link->io.Attributes1 = IO_DATA_PATH_WIDTH_8;
		link->io.IOAddrLines = io->flags &  CISTPL_IO_LINES_MASK;

		if (!pcmcia_request_io(link, &link->io))
#else
		struct resource *pr = link->resource[0];

		pr->start = io->win[0].base;
		pr->end = io->win[0].len;
		pr->name = dev_name(&link->dev);

		/* only this kind of access is yet supported */
		pr->flags |= IO_DATA_PATH_WIDTH_8;
		link->io_lines = io->flags &  CISTPL_IO_LINES_MASK;

		if (!pcmcia_request_io(link))
#endif

			return 0;
	}

	return -ENODEV;
}
#else
static int pcan_pccard_conf_check(struct pcmcia_device *link, void *private)
{
	struct resource *pr = link->resource[0];

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	pr->name = dev_name(&link->dev);
	pr->flags &= ~IO_DATA_PATH_WIDTH;

	/* only this kind of access is yet supported */
	pr->flags |= IO_DATA_PATH_WIDTH_8;
	link->io_lines = 10;

	return pcmcia_request_io(link);
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28)
/* called at CARD_INSERTION or startup with already inserted card */
int pcmcia_loop_config(struct pcmcia_device *link,
			int  (*conf_check) (struct pcmcia_device *link,
			cistpl_cftable_entry_t *cf,
			cistpl_cftable_entry_t *dflt,
			unsigned int vcc, void *priv_data), void *priv_data)
{
	tuple_t         tuple;
	cisdata_t       buf[CISTPL_END + 1];
	cisparse_t      parse;
	config_info_t   conf;
	cistpl_vers_1_t *cv;
	int             err;
	int             last_fn;

	DPRINTK(KERN_DEBUG "%s: %s(0x%p)\n", DEVICE_NAME, __func__, link);

	/* show manufacturer string in log */
	CS_PREPARE(CISTPL_VERS_1);
	CS_CHECK(GetFirstTuple, pcmcia_get_first_tuple(link, &tuple));
	CS_CHECK(GetTupleData, pcmcia_get_tuple_data(link, &tuple));
	CS_CHECK(ParseTuple, pcmcia_parse_tuple(link, &tuple, &parse));
	cv = &parse.version_1;

	/* specify what to request */
	CS_PREPARE(CISTPL_CONFIG);
	CS_CHECK(GetFirstTuple, pcmcia_get_first_tuple(link, &tuple));
	CS_CHECK(GetTupleData, pcmcia_get_tuple_data(link, &tuple));
	CS_CHECK(ParseTuple, pcmcia_parse_tuple(link, &tuple, &parse));

	link->conf.ConfigBase = parse.config.base;
	link->conf.Present    = parse.config.rmask[0];

	CS_CHECK(GetConfigurationInfo,
			pcmcia_get_configuration_info(link, &conf));

	tuple.DesiredTuple = CISTPL_CFTABLE_ENTRY;
	tuple.Attributes   = 0;
	CS_CHECK(GetFirstTuple, pcmcia_get_first_tuple(link, &tuple));
	while (1) {
		cistpl_cftable_entry_t *cfg = &parse.cftable_entry;
		cistpl_cftable_entry_t dflt = { .index = 0 };

		if (!pcmcia_get_tuple_data(link, &tuple) ||
				!pcmcia_parse_tuple(link, &tuple, &parse))
			goto next_entry;

		if (cfg->flags & CISTPL_CFTABLE_DEFAULT)
			dflt = *cfg;
		if (cfg->index == 0)
			goto next_entry;

		link->conf.ConfigIndex = cfg->index;

		if (!conf_check(link, cfg, &dflt, link->socket->socket.Vcc,
					priv_data))
			break;

next_entry:
		CS_CHECK(GetNextTuple, pcmcia_get_next_tuple(link, &tuple));
	}

cs_failed:
	cs_error(link, last_fn, err);

	return err;
}
#endif

/* called at CARD_INSERTION or startup with already inserted card */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
static int pccard_plugin(struct pcmcia_device *link)
{
	PCAN_PCCARD     *card = link->priv;
	int             err;
	int             last_fn;

	DPRINTK(KERN_DEBUG "%s: %s(0x%p)\n", DEVICE_NAME, __func__, link);

	CS_CHECK(RequestIO,
		pcmcia_loop_config(link, pcan_pccard_conf_check, NULL));

	/* show manufacturer string in log */
	if ((!link->prod_id[0]) || (!link->prod_id[1]) ||
			(!link->prod_id[2]) || (!link->prod_id[3]))
		printk(KERN_ERR "%s: can't retrieve manufacturer string!\n",
			DEVICE_NAME);
	else
		printk(KERN_INFO "%s: %s %s %s %s\n", DEVICE_NAME,
			link->prod_id[0], link->prod_id[1],
			link->prod_id[2], link->prod_id[3]);

	/* exclusive irq lines are not supported up from 2.6.28 */
	link->irq.Attributes = IRQ_TYPE_DYNAMIC_SHARING;
	link->irq.IRQInfo1 = IRQ_LEVEL_ID;

	/* we use our own handler */
	link->irq.Handler = NULL;
	CS_CHECK(RequestIRQ, pcmcia_request_irq(link, &link->irq));

	link->conf.Attributes = CONF_ENABLE_IRQ;
	link->conf.IntType = INT_MEMORY_AND_IO;
	link->conf.Present = PRESENT_OPTION | PRESENT_STATUS;
	CS_CHECK(RequestConfiguration,
			pcmcia_request_configuration(link, &link->conf));

	DPRINTK(KERN_DEBUG "%s: pccard found: base1=0x%04x, size=%d, irq=%d\n",
			DEVICE_NAME, link->io.BasePort1, link->io.NumPorts1,
			link->irq.AssignedIRQ);

	/* init (cardmgr) devices associated with that card
	 * (is that necessary?) */
	card->node.major = pcan_drv.nMajor;
	card->node.minor = PCCARD_MINOR_BASE;
	strcpy(card->node.dev_name, DEVICE_NAME);
	link->dev_node = &card->node;

	/* create device descriptors associated with the card
	 * - get relevant parts to get independend from dev_link_t */
	card->basePort = link->io.BasePort1;
	card->commonIrq = link->irq.AssignedIRQ;
	err = pccard_create_all_devices(card);
	if (err)
		goto fail;

	return 0;

cs_failed:
	cs_error(link, last_fn, err);

fail:
	pccard_plugout(link);

	DPRINTK(KERN_DEBUG "%s: %s() failed!\n", DEVICE_NAME, __func__);
	return -ENODEV;
}
#else
static int pccard_plugin(struct pcmcia_device *link)
{
	PCAN_PCCARD *card = link->priv;
	int err;

	DPRINTK(KERN_DEBUG "%s: %s(0x%p)\n", DEVICE_NAME, __func__, link);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
	link->config_flags |= CONF_AUTO_SET_IO;
#endif

	err = pcmcia_loop_config(link, pcan_pccard_conf_check, NULL);
	if (err) {
		printk(KERN_ERR "%s: pcmcia_loop_config() = %d!\n",
			DEVICE_NAME, err);
		goto fail;
	}

	/* show manufacturer string in log */
	if ((!link->prod_id[0]) || (!link->prod_id[1]) ||
			(!link->prod_id[2]) || (!link->prod_id[3]))
		printk(KERN_ERR "%s: can't retrieve manufacturer string!\n",
			DEVICE_NAME);
	else
		printk(KERN_INFO "%s: %s %s %s %s\n", DEVICE_NAME,
			link->prod_id[0], link->prod_id[1],
			link->prod_id[2], link->prod_id[3]);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
	/* exclusive irq lines are not supported up from 2.6.28 */
	link->irq.Attributes = IRQ_TYPE_DYNAMIC_SHARING;
	link->irq.Handler = NULL;	/* we use our own handler */
	if ((err = pcmcia_request_irq(link, &link->irq))) {
		printk(KERN_ERR "%s: pcmcia_request_irq() = %d\n",
			DEVICE_NAME, err);
		goto fail;
	}
#else
	/* see Documentation/pcmcia/driver-changes.txt:
	 * Instead of the old pcmcia_request_irq() interface, drivers may now
	 * choose between:
	 * - calling request_irq/free_irq directly. Use the IRQ from
	 * *p_dev->irq.
	 * (this is the job of pccard_create_all_devices() call below) */
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)
	link->conf.Attributes = CONF_ENABLE_IRQ;
	link->conf.IntType = INT_MEMORY_AND_IO;
	link->conf.Present = PRESENT_OPTION | PRESENT_STATUS;
	if ((err = pcmcia_request_configuration(link, &link->conf))) {
		printk(KERN_WARNING "%s: pcmcia_request_configuration() = %d\n",
			DEVICE_NAME, err);
#else
	link->config_flags |= CONF_ENABLE_IRQ;
	link->config_regs = PRESENT_OPTION;
	if ((err = pcmcia_enable_device(link))) {
		printk(KERN_ERR "%s: pcmcia_enable_device() = %d!\n",
			DEVICE_NAME, err);
#endif
		goto fail;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
	/* init (cardmgr) devices associated with that card
	 * (is that necessary?) */
	card->node.major = pcan_drv.nMajor;
	card->node.minor = PCCARD_MINOR_BASE;
	strcpy(card->node.dev_name, DEVICE_NAME);
	link->dev_node = &card->node;
	card->commonIrq = link->irq.AssignedIRQ;
#else
	card->commonIrq = link->irq;
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
	card->basePort = link->io.BasePort1;
	card->numPort = link->io.NumPorts1;
#else
	card->basePort = link->resource[0]->start;
	card->numPort = link->resource[0]->end;
#endif

	DPRINTK(KERN_DEBUG "%s: pccard found: base1=0x%04x, size=%d, irq=%d\n",
		DEVICE_NAME, card->basePort, card->numPort, card->commonIrq);

	/* create device descriptors associated with the card -
	 * get relevant parts to get independent from dev_link_t */
	err = pccard_create_all_devices(card);
	if (err)
		goto fail;

	return 0;

fail:
	pccard_plugout(link);

	DPRINTK(KERN_DEBUG "%s: %s() failed!\n", DEVICE_NAME, __func__);
	return -ENODEV;
}
#endif

/* probe entry */
static int pccard_probe(struct pcmcia_device *link)
{
	PCAN_PCCARD *card;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	/* Create new PCAN_PCCARD device */
	card = pcan_malloc(sizeof(*card), GFP_KERNEL);
	DPRINTK(KERN_DEBUG "%s: pcan_malloc() = 0x%p\n", DEVICE_NAME, card);
	if (!card) {
		printk(KERN_ERR "%s: can't allocate card memory\n",
				DEVICE_NAME);
		return -ENOMEM;
	}

	memset(card, 0, sizeof(*card));
	link->priv = card;
	card->pcc_dev = link;

	pcan_init_adapter(&card->adapter, "PCAN-PC Card", pccard_adapters++,
							 PCCARD_CHANNELS);
	return pccard_plugin(link);
}

/* detach entry */
static void pccard_detach(struct pcmcia_device *link)
{
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	pccard_plugout(link);
	pccard_adapters--;

	DPRINTK(KERN_DEBUG "%s: pcan_free(PCAN_PCCARD=0x%p)\n",
				DEVICE_NAME, link->priv);
	pcan_free(link->priv);
	link->priv = NULL;
}

/* suspend entry */
static int pccard_suspend(struct pcmcia_device *link)
{
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	return 0;
}

/* resume entry */
static int pccard_resume(struct pcmcia_device *link)
{
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	return 0;
}
