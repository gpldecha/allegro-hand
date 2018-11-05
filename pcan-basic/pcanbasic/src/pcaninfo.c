/*****************************************************************************
 * Copyright (C) 2001-2016  PEAK System-Technik GmbH
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
 * Maintainer(s): Fabrice Vergnaud (f.vergnaud@peak-system.com)
 *
 *****************************************************************************/

/**
 * @file pcaninfo.c
 * @brief Tools to get information on PCAN devices
 *
 * $Id: pcaninfo.c 125 2018-08-03 14:06:52Z Fabrice $
 *
 */

#include "pcaninfo.h"

/*
 * INCLUDES
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <dirent.h>		/* scandir */
#include <math.h>		/* floor */
#include <pcan.h>		/* PCAN HW types */
#include <pcanfd.h>		/* PCAN_ERROR_BUS types */
#include <time.h>		/* time */

#include "pcanlog.h"

#ifndef HW_PCI_FD
#define HW_PCI_FD 19
#endif
/*
 * DEFINES
 */
/** Sysfs path to retrieve pcan mmodule information */
#define PCAN_MODULE_PATH	"/sys/module/pcan"
/** Sysfs path to retrieve pcan-pci/pcmcia information */
#define PCAN_CLASS_PATH		"/sys/class/pcan"
/** prefixed used by pcan usb devices */
#define PCAN_USBMISC_PREFIX	"pcan"
/** Sysfs path to retrieve version information */
#define PCAN_VERSION_PATH	PCAN_CLASS_PATH "/version"
/** Sysfs path to retrieve version information */
#define PCAN_PROC_PATH		"/proc/pcan"

/**
 * @defgroup PCAN_FILEINFO PCAN files in found Sysfs
 *
 * @{
 */
/** @cond Doxygen_Suppress */
#define PCAN_FILEINFO_ADAPTER_NAME		"adapter_name"
#define PCAN_FILEINFO_ADAPTER_NB		"adapter_number"
#define PCAN_FILEINFO_ADAPTER_VERSION	"adapter_version"
#define PCAN_FILEINFO_BASE		"base"
#define PCAN_FILEINFO_NOM_BITRATE 	"nom_bitrate"
#define PCAN_FILEINFO_NOM_BRP 	"nom_brp"
#define PCAN_FILEINFO_NOM_SJW 	"nom_sjw"
#define PCAN_FILEINFO_NOM_TSEG1	"nom_tseg1"
#define PCAN_FILEINFO_NOM_TSEG2	"nom_tseg2"
#define PCAN_FILEINFO_BTR0BTR1	"btr0btr1"
#define PCAN_FILEINFO_BUSLOAD	"bus_load"
#define PCAN_FILEINFO_BUSSTATE	"bus_state"
#define PCAN_FILEINFO_CLOCK 	"clock"
#define PCAN_FILEINFO_CTRLNB 	"ctrlr_number"
#define PCAN_FILEINFO_DATA_BITRATE 	"data_bitrate"
#define PCAN_FILEINFO_DATA_BRP 	"data_brp"
#define PCAN_FILEINFO_DATA_SJW 	"data_sjw"
#define PCAN_FILEINFO_DATA_TSEG1	"data_tseg1"
#define PCAN_FILEINFO_DATA_TSEG2	"data_tseg2"
#define PCAN_FILEINFO_DEV 		"dev"
#define PCAN_FILEINFO_DEVICE 	"device"
#define PCAN_FILEINFO_DEVID 	"devid"
#define PCAN_FILEINFO_ERRORS 	"errors"
#define PCAN_FILEINFO_HWTYPE 	"hwtype"
#define PCAN_FILEINFO_IRQ		"irq"
#define PCAN_FILEINFO_IRQS	 	"irqs"
#define PCAN_FILEINFO_MINOR 	"minor"
#define PCAN_FILEINFO_POWER 	"power"
#define PCAN_FILEINFO_READ 		"read"
#define PCAN_FILEINFO_RXERR		"rx_error_counter"
#define PCAN_FILEINFO_SN	 	"serial_number"
#define PCAN_FILEINFO_STATUS 	"status"
#define PCAN_FILEINFO_SUBSYSTEM	"subsystem"
#define PCAN_FILEINFO_TXERR		"tx_error_counter"
#define PCAN_FILEINFO_TYPE	 	"type"
#define PCAN_FILEINFO_UEVENT 	"uevent"
#define PCAN_FILEINFO_WRITE 	"write"
#define PCAN_FILEINFO_RX_FIFO_RATIO	"rx_fifo_ratio"
#define PCAN_FILEINFO_TX_FIFO_RATIO	"tx_fifo_ratio"
/** @endcond */
/** @} */

/** legacy: sysfs pcan file prefix, used with pcan driver prior to v8.0 */
#define PCAN_FILEINFO_PREFIX_LEGACY	"pcan_"
/**
 * @def LEGACY_GET_FILEINFO_NAME(file)
 * @brief A macro that returns the legacy name of pcan files in PCAN_CLASS_PATH
 * */
#define LEGACY_GET_FILEINFO_NAME(file) (PCAN_FILEINFO_PREFIX_LEGACY file)

/**
 * @def MAX(x,y)
 * @brief A macro that returns the maximum of @a x and @a y.
 */
#define MAX(x,y) ((x) > (y) ? (x) : (y))
/**
 * @def MIN(x,y)
 * @brief A macro that returns the minimum of @a x and @a y.
 */
#define MIN(x,y) ((x) < (y) ? (x) : (y))


/** "No error" status error code */
#define PCANINFO_ERR_OK 	0

/* PRIVATE FUNCTIONS DECLARATIONS */
/** function used by scandir to get all files except '.' and '..' */
static int classdir_selector(const struct dirent *ent);
 /**
  * @fn int load_devinfo(struct pcaninfo * pci)
  * @brief Retrieves information for a PCAN device.
  *
  * @param pci [in,out] {buffer to store PCAN data.
  * 	classpath and name must be set.}
  * @return a PCANINFO_ERR_xx status code
  */
static int load_devinfo(struct pcaninfo * pci);
/**
 * @fn int parse_file(struct pcaninfo *pci, char *path, char *filename)
 * @brief Parses a sysfs pcan file and populates the struct pcaninfo buffer
 *
 * @param[in, out] pci buffer to store data
 * @param[in] path Path of the file to read
 * @param[in] filename Name of the file to read
 * @return a PCANINFO_ERR_xx status code
 */
static int parse_file(struct pcaninfo *pci, char *path, char *filename);

/**
 * @fn char * pretty_unit(ulong val, char * buffer, size_t len)
 * @brief Formats a value with a valid SI unit.
 *
 * @param[in] val Value to format
 * @param[in, out] Buffer to store the formatted string
 * @param[in] Size of the buffer
 * @return The 'buffer' param
 */
static char * pretty_unit(ulong val, char * buffer, size_t len);
/**
 * @fn char * pretty_bus_state(uint state, char * buffer, size_t len)
 * @brief Formats a PCAN bus state.
 *
 * @param[in] val Value to format
 * @param[in, out] Buffer to store the formatted string
 * @param[in] Size of the buffer
 * @return The 'buffer' param
 */
static char * pretty_bus_state(uint state, char * buffer, size_t len);

/* PRIVATE FUNCTIONS */
int classdir_selector(const struct dirent *ent) {
	if (ent->d_name[0] == '.') {
		if (ent->d_name[1] == 0)
			return 0;
		if (ent->d_name[1] == '.' &&
			ent->d_name[2] == 0)
				return 0;
	}
	if (ent->d_type == DT_REG)
		return 0;
	return 1;
}
int classfile_selector(const struct dirent *ent) {
	if (ent->d_name[0] == '.') {
		if (ent->d_name[1] == 0)
			return 0;
		if (ent->d_name[1] == '.' &&
			ent->d_name[2] == 0)
				return 0;
	}
	if (ent->d_type == DT_DIR)
		return 0;
	return 1;
}

int load_devinfo(struct pcaninfo * pci) {
	struct dirent **ent;
	int i, n;
	char * path;

	/* initialization */
	n = strnlen(pci->classpath, PCANINFO_MAX_CHAR_SIZE) + strnlen(pci->name, PCANINFO_MAX_CHAR_SIZE) + 2;	/* 2 = '/' + '\0' */
	path = (char *) malloc(n);
	if (path == NULL) {
		pcanlog_log(LVL_NORMAL, "ERROR: failed to allocate memory to scan directory '%s/%s'.\n", pci->classpath, pci->name);
		return errno = ENOMEM;
	}
	snprintf(path, n, "%s/%s", pci->classpath, pci->name);
	snprintf(pci->path, sizeof(pci->path),"/dev/%s", pci->name);
	pcanlog_log(LVL_DEBUG, "Scanning directory '%s'...\n", path);
	/* scan sys dirs */
	ent = NULL;
	n = scandir(path, &ent, classfile_selector, alphasort);
	/* foreach file, load info */
	for (i = 0; i < n; i++) {
#ifdef _DIRENT_HAVE_D_TYPE
		if (ent[i]->d_type == DT_REG)	/* skip directories & links*/
#endif
		parse_file(pci, path, ent[i]->d_name);
		free(ent[i]);
	}
	switch (pci->hwtype) {
		case HW_ISA:
		case HW_ISA_SJA:
			pci->hwcategory = PCANINFO_HW_ISA;
			break;
		case HW_DONGLE_SJA:
		case HW_DONGLE_SJA_EPP:
		case HW_DONGLE_PRO:
		case HW_DONGLE_PRO_EPP:
			pci->hwcategory = PCANINFO_HW_DNG;
			break;
		case HW_PCI:
		case HW_PCI_FD:
			pci->hwcategory = PCANINFO_HW_PCI;
			break;
		case HW_USB:
		case HW_USB_PRO:
		case HW_USB_PRO_FD:
		case HW_USB_FD:
		case HW_USB_X6:
			pci->hwcategory = PCANINFO_HW_USB;
			break;
		case HW_PCCARD:
			pci->hwcategory = PCANINFO_HW_PCC;
			break;
		default:
			pci->hwcategory = PCANINFO_HW_NONE;
			break;
	}
	/* mark the structure as initialized */
	pci->availflag |= PCANINFO_FLAG_INITIALIZED;
	time(&pci->time_update);
	/* uninitialize */
	if (ent)
		free(ent);
	if (path)
		free(path);
	return PCANINFO_ERR_OK;
}

int parse_file(struct pcaninfo *pci, char *path, char *filename) {
	char * filepath;
	FILE * f;
	char * line;
	size_t len;
	ssize_t read;

	/* Invalid argument ? */
	if (pci == NULL || path == NULL || filename == NULL ||
			path[0] == 0 || filename[0] == 0)
		return errno = EINVAL;
	/* initialization */
	filepath = (char *) malloc(strlen(path) + strlen(filename) + 2);
	if (filepath == NULL) {
		pcanlog_log(LVL_NORMAL, "ERROR: failed to allocate memory to parse file '%s/%s'.\n", path, filename);
		return errno = ENOMEM;
	}
	sprintf(filepath, "%s/%s", path, filename);
	pcanlog_log(LVL_DEBUG, "Parsing file '%s'...\n", filepath);

	line = NULL;
	len = 0;
	/* open file */
	f = fopen(filepath, "r");
	if (f == NULL) {
		errno = ENOENT;
		pcanlog_log(LVL_NORMAL, "ERROR: failed to open file '%s'.\n", filepath);
	}
	else
	{
		/* read first line */
		read = getline(&line, &len, f);
		if (read != -1) {
			/* fill in the right PCANINFO parameter based on filename */
			if (strcmp(filename, PCAN_FILEINFO_ADAPTER_NAME) == 0) {
				len = MIN(PCANINFO_MAX_CHAR_SIZE, read);
				strncpy(pci->adapter_name, line, len);
				if (len >= 1 && pci->adapter_name[len - 1] == '\n')
					pci->adapter_name[len - 1] = 0;
				pci->adapter_name[PCANINFO_MAX_CHAR_SIZE - 1] = 0;
				pci->availflag |= PCANINFO_FLAG_ADAPTER_NAME;
			}
			else if (strcmp(filename, PCAN_FILEINFO_ADAPTER_NB) == 0) {
				pci->adapter_nb = (uint)strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_ADAPTER_NB;
			}
			else if (strcmp(filename, PCAN_FILEINFO_ADAPTER_VERSION) == 0) {
				len = MIN(PCANINFO_MAX_CHAR_SIZE, read);
				strncpy(pci->adapter_version, line, len);
				if (len >= 1 && pci->adapter_version[len - 1] == '\n')
					pci->adapter_version[len - 1] = 0;
				pci->adapter_version[PCANINFO_MAX_CHAR_SIZE - 1] = 0;
				pci->availflag |= PCANINFO_FLAG_ADAPTER_VERSION;
			}
			else if (strcmp(filename, PCAN_FILEINFO_NOM_BITRATE) == 0 ||
					strcmp(filename, LEGACY_GET_FILEINFO_NAME(PCAN_FILEINFO_NOM_BITRATE)) == 0) {
				pci->nom_bitrate = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_NOM_BITRATE;
			}
			else if (strcmp(filename, PCAN_FILEINFO_BTR0BTR1) == 0) {
				pci->btr0btr1 = (uint)strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_BTR0BTR1;
			}
			else if (strcmp(filename, PCAN_FILEINFO_CLOCK) == 0 ||
					strcmp(filename, LEGACY_GET_FILEINFO_NAME(PCAN_FILEINFO_CLOCK)) == 0) {
				pci->clock = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_CLOCK;
			}
			else if (strcmp(filename, PCAN_FILEINFO_CTRLNB) == 0) {
				pci->ctrlnb = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_CTRLNB;
			}
			else if (strcmp(filename, PCAN_FILEINFO_DATA_BITRATE) == 0 ||
					strcmp(filename, LEGACY_GET_FILEINFO_NAME(PCAN_FILEINFO_DATA_BITRATE)) == 0) {
				pci->data_bitrate = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_DATA_BITRATE;
			}
			else if (strcmp(filename, PCAN_FILEINFO_DEV) == 0 ||
					strcmp(filename, LEGACY_GET_FILEINFO_NAME(PCAN_FILEINFO_DEV)) == 0) {
				len = MIN(PCANINFO_MAX_CHAR_SIZE, read);
				strncpy(pci->dev, line, len);
				if (len >= 1 && pci->dev[len - 1] == '\n')
					pci->dev[len - 1] = 0;
				pci->dev[PCANINFO_MAX_CHAR_SIZE - 1] = 0;
				pci->availflag |= PCANINFO_FLAG_DEV;
			}
			else if (strcmp(filename, PCAN_FILEINFO_DEVID) == 0 ||
					strcmp(filename, LEGACY_GET_FILEINFO_NAME(PCAN_FILEINFO_DEVID)) == 0) {
				pci->devid = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_DEVID;
			}
			else if (strcmp(filename, PCAN_FILEINFO_ERRORS) == 0) {
				pci->errors = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_ERRORS;
			}
			else if (strcmp(filename, PCAN_FILEINFO_HWTYPE) == 0 ||
					strcmp(filename, LEGACY_GET_FILEINFO_NAME(PCAN_FILEINFO_HWTYPE)) == 0) {
				pci->hwtype = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_HWTYPE;
			}
			else if (strcmp(filename, PCAN_FILEINFO_IRQS) == 0) {
				pci->irqs = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_IRQS;
			}
			else if (strcmp(filename, PCAN_FILEINFO_MINOR) == 0 ||
					strcmp(filename, LEGACY_GET_FILEINFO_NAME(PCAN_FILEINFO_MINOR)) == 0) {
				pci->minor = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_MINOR;
			}
			else if (strcmp(filename, PCAN_FILEINFO_READ) == 0) {
				pci->read = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_READ;
			}
			else if (strcmp(filename, PCAN_FILEINFO_SN) == 0) {
				pci->sn = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_SN;
			}
			else if (strcmp(filename, PCAN_FILEINFO_STATUS) == 0) {
				pci->status = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_STATUS;
			}
			else if (strcmp(filename, PCAN_FILEINFO_TYPE) == 0) {
				len = MIN(PCANINFO_MAX_CHAR_SIZE, read);
				strncpy(pci->type, line, len);
				if (len >= 1 && pci->type[len - 1] == '\n')
					pci->type[len - 1] = 0;
				pci->type[PCANINFO_MAX_CHAR_SIZE - 1] = 0;
				pci->availflag |= PCANINFO_FLAG_TYPE;
			}
			else if (strcmp(filename, PCAN_FILEINFO_WRITE) == 0) {
				pci->write = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_WRITE;
			}
			else if (strcmp(filename, PCAN_FILEINFO_BASE) == 0) {
				pci->base = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_BASE;
			}
			else if (strcmp(filename, PCAN_FILEINFO_IRQ) == 0) {
				pci->irq = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_IRQ;
			}
			else if (strcmp(filename, PCAN_FILEINFO_BUSLOAD) == 0) {
				pci->bus_load = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_BUSLOAD;
			}
			else if (strcmp(filename, PCAN_FILEINFO_BUSSTATE) == 0) {
				pci->bus_state = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_BUSSTATE;
			}
			else if (strcmp(filename, PCAN_FILEINFO_RXERR) == 0) {
				pci->rxerr = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_RXERR;
			}
			else if (strcmp(filename, PCAN_FILEINFO_TXERR) == 0) {
				pci->txerr = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_TXERR;
			}
			else if (strcmp(filename, PCAN_FILEINFO_RX_FIFO_RATIO) == 0) {
				pci->rx_fifo_ratio = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_RX_FIFO_RATIO;
			}
			else if (strcmp(filename, PCAN_FILEINFO_TX_FIFO_RATIO) == 0) {
				pci->tx_fifo_ratio = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_TX_FIFO_RATIO;
			}
			/* handle extra flags with availflag_ex */
			else if (strcmp(filename, PCAN_FILEINFO_NOM_BRP) == 0) {
				pci->nom_brp = (uint)strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_NOM_BRP;
			}
			else if (strcmp(filename, PCAN_FILEINFO_NOM_SJW) == 0) {
				pci->nom_sjw = (uint)strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_NOM_SJW;
			}
			else if (strcmp(filename, PCAN_FILEINFO_NOM_TSEG1) == 0) {
				pci->nom_tseg1 = (uint)strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_NOM_TSEG1;
			}
			else if (strcmp(filename, PCAN_FILEINFO_NOM_TSEG2) == 0) {
				pci->nom_tseg2 = (uint)strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_NOM_TSEG2;
			}
			else if (strcmp(filename, PCAN_FILEINFO_DATA_BRP) == 0) {
				pci->data_brp = (uint)strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_DATA_BRP;
			}
			else if (strcmp(filename, PCAN_FILEINFO_DATA_SJW) == 0) {
				pci->data_sjw = (uint)strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_DATA_SJW;
			}
			else if (strcmp(filename, PCAN_FILEINFO_DATA_TSEG1) == 0) {
				pci->data_tseg1 = (uint)strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_DATA_TSEG1;
			}
			else if (strcmp(filename, PCAN_FILEINFO_DATA_TSEG2) == 0) {
				pci->data_tseg2 = (uint)strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_DATA_TSEG2;
			}
			else {
				/* unsupported files */
				if (strcmp(filename, PCAN_FILEINFO_UEVENT) != 0) {
					pcanlog_log(LVL_DEBUG, "WARNING: unsupported file '%s'.\n", filename);
				}
			}
		}
		else {
			pcanlog_log(LVL_NORMAL, "ERROR: failed to read line in file '%s'.\n", filename);
			errno = EIO;
		}
	}
	/* uninitialize */
	if (line)
		free(line);
	if (f)
		fclose(f);
	if (filepath)
		free(filepath);
	return errno;
}

char * pretty_unit(ulong val, char * buffer, size_t len) {
	float fval;
	ulong e;
	char unit;

	/* identify SI prefix */
	if (val >= 1000000) {
		fval = val / 1000000.0f;
		unit = 'M';
	}
	else if (val >= 1000) {
		fval = val / 1000.0f;
		unit = 'k';
	}
	else {
		fval = floor(val);
		unit = '\0';
	}
	/* format buffer avoiding useless decimals */
	e = floor(fval);
	if (fval - e > 0)
		snprintf(buffer, len, "%.03f %c", fval, unit);
	else
		snprintf(buffer, len, "%lu %c", e, unit);
	return buffer;
}

char * pretty_bus_state(uint state, char * buffer, size_t len) {

	switch (state) {
	case PCANFD_ERROR_PASSIVE:
		snprintf(buffer, len, "Passive");
		break;
	case PCANFD_ERROR_WARNING:
		snprintf(buffer, len, "Warning");
		break;
	case PCANFD_ERROR_BUSOFF:
		snprintf(buffer, len, "BUS OFF");
		break;
	case PCANFD_ERROR_ACTIVE:
		snprintf(buffer, len, "OK");
		break;
	default:
		snprintf(buffer, len, "Closed / Unknown");
		break;
	}
	return buffer;
}

/*
 * GLOBAL FUNCTIONS
 */

int pcaninfo_update(struct pcaninfo * pci) {
	if (pci == NULL || pci->classpath == NULL || pci->name[0] == 0)
		return errno = EINVAL;
	return load_devinfo(pci);
}

int pcaninfo_get(struct pcaninfo_list ** pcilist, int do_init) {
	struct pcaninfo_list *pcil;
	struct dirent **entpcan;
	int npcan, i, len;
	int ires;
	char * path;

	/* scan sys 'pcan' dir */
	ires = 0;
	entpcan = NULL;
	path = PCAN_CLASS_PATH;
	npcan = scandir(path, &entpcan, classdir_selector, alphasort);
	if (npcan < 0) {
		pcanlog_log(LVL_NORMAL, "ERROR: failed to scan directory (errno=%d) '%s'\n", errno, path);
		npcan = 0;
	}
	pcanlog_log(LVL_VERBOSE, "Found %d devices in '%s'\n", npcan, path);
	/* initialize each device information */
	len = MAX(0, npcan);
	i = sizeof(*pcil) + len * sizeof(pcil->infos[0]);
	pcil = (struct pcaninfo_list *) calloc(1, i);
	if (pcil == NULL) {
		ires = ENOMEM;
		goto pcaninfo_get_free;
	}
	pcil->length = len;
	if (npcan > 0) {
		for (i = 0; i < npcan; i++) {
			pcil->infos[i].classpath = PCAN_CLASS_PATH;
			strncpy(pcil->infos[i].name, entpcan[i]->d_name, entpcan[i]->d_reclen);
			if (do_init)
				load_devinfo(&pcil->infos[i]);
			free(entpcan[i]);
		}
	}
	/* read driver version */
	pcaninfo_driver_version(pcil->version, sizeof(pcil->version));

	*pcilist = pcil;
	/* uninitialize */
pcaninfo_get_free:
	if (entpcan)
		free(entpcan);
	return ires;
}

void pcaninfo_output(struct pcaninfo * pci) {
	char tmp[PCANINFO_MAX_CHAR_SIZE];
	int separator;
	/* Invalid argument ? */
	if (pci == NULL)
		return;

	fprintf(stdout, "  * %s: (%s/%s)\n", pci->name, pci->classpath, pci->name);
	fprintf(stdout, "  \t- file: %s\n", pci->path);

	/* print device info */
	separator = 0;
	if (pci->availflag & PCANINFO_FLAG_DEV) {
		fprintf(stdout, "  \t- %s: \"%s\"\n", PCAN_FILEINFO_DEV, pci->dev);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_MINOR) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_MINOR, pci->minor);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_BASE) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_BASE, pci->base);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_IRQ) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_IRQ, pci->irq);
		separator++;
	}
	if (separator)
		fprintf(stdout, "  \t-----------------\n");

	/* print hardware info */
	separator = 0;
	if (pci->availflag & PCANINFO_FLAG_ADAPTER_NAME) {
		fprintf(stdout, "  \t- %s: \"%s\"\n", PCAN_FILEINFO_ADAPTER_NAME, pci->adapter_name);
		separator++;
		}
	if (pci->availflag & PCANINFO_FLAG_ADAPTER_NB) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_ADAPTER_NB, pci->adapter_nb);
		separator++;
		}
	if (pci->availflag & PCANINFO_FLAG_ADAPTER_VERSION) {
		fprintf(stdout, "  \t- %s: \"%s\"\n", PCAN_FILEINFO_ADAPTER_VERSION, pci->adapter_version);
		separator++;
		}
	if (pci->availflag & PCANINFO_FLAG_TYPE) {
		fprintf(stdout, "  \t- %s: \"%s\"\n", PCAN_FILEINFO_TYPE, pci->type);
		separator++;
		}
	if (pci->availflag & PCANINFO_FLAG_HWTYPE) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_HWTYPE, pci->hwtype);
		separator++;
		}
	if (pci->availflag & PCANINFO_FLAG_DEVID) {
		fprintf(stdout, "  \t- %s: 0x%02x\n", PCAN_FILEINFO_DEVID, pci->devid);
		separator++;
		}
	if (pci->availflag & PCANINFO_FLAG_SN) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_SN, pci->sn);
		separator++;
		}
	if (pci->availflag & PCANINFO_FLAG_CTRLNB) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_CTRLNB, pci->ctrlnb);
		separator++;
		}
	if (separator)
		fprintf(stdout, "  \t-----------------\n");

	/* print Channel connection info */
	separator = 0;
	if (pci->availflag & PCANINFO_FLAG_CLOCK) {
		fprintf(stdout, "  \t- %s: %sHz\n", PCAN_FILEINFO_CLOCK,
				pretty_unit(pci->clock, tmp, PCANINFO_MAX_CHAR_SIZE));
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_NOM_BITRATE) {
		fprintf(stdout, "  \t- %s: %sBit/s\n", PCAN_FILEINFO_NOM_BITRATE,
				pretty_unit(pci->nom_bitrate, tmp, PCANINFO_MAX_CHAR_SIZE));
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_BTR0BTR1) {
		fprintf(stdout, "  \t- %s: 0x%x\n", PCAN_FILEINFO_BTR0BTR1,
				pci->btr0btr1);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_DATA_BITRATE) {
		fprintf(stdout, "  \t- %s: %sBit/s\n", PCAN_FILEINFO_DATA_BITRATE,
				pretty_unit(pci->data_bitrate, tmp, PCANINFO_MAX_CHAR_SIZE));
		separator++;
	}
	if (separator)
		fprintf(stdout, "  \t-----------------\n");

	/* print Channel More connection info */
	separator = 0;
	if (pci->availflag_ex & PCANINFO_FLAG_EX_NOM_BRP) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_NOM_BRP, pci->nom_brp);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_NOM_SJW) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_NOM_SJW, pci->nom_sjw);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_NOM_TSEG1) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_NOM_TSEG1, pci->nom_tseg1);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_NOM_TSEG2) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_NOM_TSEG2, pci->nom_tseg2);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_DATA_BRP) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_DATA_BRP, pci->data_brp);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_DATA_SJW) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_DATA_SJW, pci->data_sjw);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_DATA_TSEG1) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_DATA_TSEG1, pci->data_tseg1);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_DATA_TSEG2) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_DATA_TSEG2, pci->data_tseg2);
		separator++;
	}
	if (separator)
		fprintf(stdout, "  \t-----------------\n");

	/* print Bus stats */
	separator = 0;
	if (pci->availflag & PCANINFO_FLAG_BUSSTATE) {
		fprintf(stdout, "  \t- %s: %s (%d)\n", PCAN_FILEINFO_BUSSTATE,
				pretty_bus_state(pci->bus_state, tmp, PCANINFO_MAX_CHAR_SIZE),
				pci->bus_state);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_BUSLOAD) {
		fprintf(stdout, "  \t- %s: %d%%\n", PCAN_FILEINFO_BUSLOAD,
				pci->bus_load);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_RXERR) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_RXERR, pci->rxerr);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_TXERR) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_TXERR, pci->txerr);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_RX_FIFO_RATIO) {
		fprintf(stdout, "  \t- %s: %d%%\n", PCAN_FILEINFO_RX_FIFO_RATIO, pci->rx_fifo_ratio);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_TX_FIFO_RATIO) {
		fprintf(stdout, "  \t- %s: %d%%\n", PCAN_FILEINFO_TX_FIFO_RATIO, pci->tx_fifo_ratio);
		separator++;
	}
	if (separator)
		fprintf(stdout, "  \t-----------------\n");

	/* print IO stats */
	separator = 0;
	if (pci->availflag & PCANINFO_FLAG_IRQS) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_IRQS, pci->irqs);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_STATUS) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_STATUS, pci->status);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_ERRORS) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_ERRORS, pci->errors);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_READ) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_READ, pci->read);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_WRITE) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_WRITE, pci->write);
		separator++;
	}
	if (separator)
		fprintf(stdout, "  \t-----------------\n");
}

int pcaninfo_print(void) {
	struct pcaninfo_list *pcilist;
	int i, ires;

	pcilist = NULL;
	ires = pcaninfo_get(&pcilist, 1);
	if (ires != 0)
		return ires;

	if (pcilist->version[0] != 0)
		fprintf(stdout, "PCAN driver version: %s\n\n", pcilist->version);
	else
		fprintf(stdout, "PCAN driver not found\n\n");
	fprintf(stdout, "Found %d PCAN devices\n", pcilist->length);
	for (i = 0; i < pcilist->length; i++) {
		pcaninfo_output(&pcilist->infos[i]);
		fprintf(stdout, "\n");
	}
	free(pcilist);
	return ires;
}

int pcaninfo_driver_version(char *buffer, uint size) {
	FILE * f;
	char * line;
	size_t len, lenver;
	ssize_t read;

	/* Invalid argument ? */
	if (buffer == NULL || size == 0)
		return errno = EINVAL;
	line = NULL;
	len = 0;
	/* open file */
	f = fopen(PCAN_VERSION_PATH, "r");
	if (f == NULL) {
		pcanlog_log(LVL_NORMAL, "ERROR: failed to open file (errno=%d) '%s'.\n", errno, PCAN_VERSION_PATH);
		errno = ENOENT;
		/* try to detect old PCAN driver */
		f = fopen(PCAN_PROC_PATH, "r");
		if (f != NULL)
			snprintf(buffer, size, "prior to 8.0");

	}
	else
	{
		/* read first line */
		read = getline(&line, &len, f);
		if (read != -1) {
			lenver = MIN(size, read);
			strncpy(buffer, line, lenver);
			if (lenver > 1 && buffer[lenver - 1] == '\n')
				buffer[lenver - 1] = 0;
		}
	}
	/* uninitialize */
	if (line)
		free(line);
	if (f)
		fclose(f);
	return 0;
}
