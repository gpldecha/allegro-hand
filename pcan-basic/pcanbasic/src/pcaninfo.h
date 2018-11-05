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
 * @file pcaninfo.h
 * @brief Function prototypes to get information on PCAN devices
 * through the 'sysfs' file system.
 *
 * $Id: pcaninfo.h 102 2017-10-04 10:43:23Z Fabrice $
 *
 */

#ifndef __PCANINFO_H__
#define __PCANINFO_H__

/*
 * INCLUDES
 */
#include <sys/types.h>
#include <time.h>		/* time_t */

/*
 * DEFINES
 */
#define PCANINFO_MAX_CHAR_SIZE 256	/**< Max buffer size used in PCANINFO */

/**
 * @defgroup PCANINFO_FLAGS PCANINFO flag for structure parameter 'availflag'
 * Those flags states if the corresponding PCANINFO parameter is set
 *
 * @{
 */
#define PCANINFO_FLAG_INITIALIZED	(1<<0)	/**< structure is fully initialized */
#define PCANINFO_FLAG_NOM_BITRATE	(1<<1)	/**< 'nom_bitrate' parameter is defined */
#define PCANINFO_FLAG_BTR0BTR1		(1<<2)	/**< 'btr0btr1' parameter is defined */
#define PCANINFO_FLAG_CLOCK			(1<<3)	/**< 'clock' parameter is defined */
#define PCANINFO_FLAG_DATA_BITRATE	(1<<4)	/**< 'data_bitrate' parameter is defined */
#define PCANINFO_FLAG_DEV			(1<<5)	/**< 'dev' parameter is defined */
#define PCANINFO_FLAG_DEVID			(1<<6)	/**< 'devid' parameter is defined */
#define PCANINFO_FLAG_ERRORS		(1<<7)	/**< 'errors' parameter is defined */
#define PCANINFO_FLAG_HWTYPE		(1<<8)	/**< 'hwtype' parameter is defined */
#define PCANINFO_FLAG_IRQS			(1<<9)	/**< 'irqs' parameter is defined */
#define PCANINFO_FLAG_MINOR			(1<<10)	/**< 'minor' parameter is defined */
#define PCANINFO_FLAG_READ			(1<<11)	/**< 'read' parameter is defined */
#define PCANINFO_FLAG_SN			(1<<12)	/**< 'sn' parameter is defined */
#define PCANINFO_FLAG_STATUS		(1<<13)	/**< 'status' parameter is defined */
#define PCANINFO_FLAG_TYPE			(1<<14)	/**< 'type' parameter is defined */
#define PCANINFO_FLAG_WRITE			(1<<15)	/**< 'write' parameter is defined */
#define PCANINFO_FLAG_BASE			(1<<16)	/**< 'base' parameter is defined */
#define PCANINFO_FLAG_IRQ			(1<<17)	/**< 'irq' parameter is defined */
#define PCANINFO_FLAG_BUSLOAD		(1<<18)	/**< 'bus_load' parameter is defined */
#define PCANINFO_FLAG_BUSSTATE		(1<<19)	/**< 'bus_state' parameter is defined */
#define PCANINFO_FLAG_RXERR			(1<<20)	/**< 'rx_error_counter' parameter is defined */
#define PCANINFO_FLAG_TXERR			(1<<21)	/**< 'tx_error_counter' parameter is defined */
#define PCANINFO_FLAG_CTRLNB		(1<<22)	/**< 'ctrl_number' parameter is defined */
#define PCANINFO_FLAG_ADAPTER_NB	(1<<23)	/**< 'adapter_number' parameter is defined */
#define PCANINFO_FLAG_ADAPTER_NAME	(1<<24)	/**< 'adapter_name' parameter is defined */
#define PCANINFO_FLAG_ADAPTER_VERSION	(1<<25)	/**< 'adapter_version' parameter is defined */
#define PCANINFO_FLAG_RX_FIFO_RATIO	(1<<26)	/**< 'adapter_version' parameter is defined */
#define PCANINFO_FLAG_TX_FIFO_RATIO	(1<<27)	/**< 'adapter_version' parameter is defined */

#define PCANINFO_FLAG_EX_NOM_BRP	(1<<0)	/**< 'nom_brp' parameter is defined */
#define PCANINFO_FLAG_EX_NOM_SJW	(1<<1)	/**< 'nom_sjw' parameter is defined */
#define PCANINFO_FLAG_EX_NOM_TSEG1	(1<<2)	/**< 'nom_tseg1' parameter is defined */
#define PCANINFO_FLAG_EX_NOM_TSEG2	(1<<3)	/**< 'nom_tseg2' parameter is defined */
#define PCANINFO_FLAG_EX_DATA_BRP	(1<<4)	/**< 'data_brp' parameter is defined */
#define PCANINFO_FLAG_EX_DATA_SJW	(1<<5)	/**< 'data_sjw' parameter is defined */
#define PCANINFO_FLAG_EX_DATA_TSEG1	(1<<6)	/**< 'data_tseg1' parameter is defined */
#define PCANINFO_FLAG_EX_DATA_TSEG2	(1<<7)	/**< 'data_tseg2' parameter is defined */

/** @} */

/**
 * Defines the number of available categories in PCANINFO Hardware
 */
#define PCANINFO_HW_COUNT	9
/**
 * PCANINFO_HW PCANINFO Hardware category defines general
 * hardware categories for PCAN hardware in order to simplify mappings
 * with PCANBasic harware definitions.
 */
enum pcaninfo_hw {
	PCANINFO_HW_NONE 	= 0x00U,	/**< Undefined, unknown or not selected PCAN device value */
	PCANINFO_HW_PEAKCAN	= 0x01U,	/**< PCAN Non-Plug&Play devices */
	PCANINFO_HW_ISA		= 0x02U,	/**< PCAN-ISA, PCAN-PC/104, and PCAN-PC/104-Plus */
	PCANINFO_HW_DNG		= 0x03U,	/**< PCAN-Dongle */
	PCANINFO_HW_PCI		= 0x04U,	/**< PCAN-PCI, PCAN-cPCI, PCAN-miniPCI, and PCAN-PCI Express and similar FD products */
	PCANINFO_HW_USB		= 0x05U,	/**< PCAN-USB and PCAN-USB Pro and similar FD products */
	PCANINFO_HW_PCC		= 0x06U,	/**< PCAN-PC Card */
	PCANINFO_HW_VIRTUAL	= 0x07U,	/**< PCAN Virtual hardware */
	PCANINFO_HW_LAN		= 0x08U		/**< PCAN Gateway devices */
};

/**
 * Stores information on a PCAN Device
 */
struct pcaninfo {
	char * classpath;					/**< Device's class path (const char no need to be freed) */
	char name[PCANINFO_MAX_CHAR_SIZE];	/**< Device name */
	char path[PCANINFO_MAX_CHAR_SIZE];	/**< Device path */
	uint availflag; 					/**< Each bit defines if a parameter
										     is set (see PCANINFO_FLAG_xx) */
	uint availflag_ex; 					/**< Each bit defines if a parameter
										     is set (see PCANINFO_FLAG_EX_xx) */
	uint base;							/**< I/O port for non plug'n play harware */
	ulong nom_bitrate;					/**< Nominal bitrate */
	uint nom_brp;						/**< Nominal Bit rate point */
	uint nom_sjw;						/**< Nominal SyncWidthJump */
	uint nom_tseg1;						/**< Nominal Tseg1 */
	uint nom_tseg2;						/**< Nominal Tseg2 */
	uint btr0btr1;						/**< Nominal bitrate as BTR0BTR1 value*/
	ulong clock;						/**< Device's clock frequency */
	ulong data_bitrate;					/**< Data bitrate (FD only) */
	uint data_brp;						/**< Data Bit rate point (FD only) */
	uint data_sjw;						/**< Data SyncWidthJump (FD only) */
	uint data_tseg1;					/**< Data Tseg1 (FD only) */
	uint data_tseg2;					/**< Data Tseg2 (FD only) */
	char dev[PCANINFO_MAX_CHAR_SIZE];	/**< Unix device ID */
	uint devid;							/**< PCAN channel device ID */
	uint errors;						/**< number of CAN errors */
	uint hwtype;						/**< Hardware type code */
	uint irq;							/**< Interrupt for non plug'n play harware */
	uint irqs;							/**< number of interrupts */
	uint minor;							/**< Unix device minor */
	uint read;							/**< Number of CAN frames read */
	uint sn;							/**< Serial Number */
	uint status;						/**< CAN bus status */
	char type[PCANINFO_MAX_CHAR_SIZE];	/**< Hardware type as a string*/
	uint write;							/**< Number of CAN frames written */
	uint bus_load;						/**< Bus load */
	uint bus_state;						/**< Bus state */
	uint rxerr;							/**< Rx error counter */
	uint txerr;							/**< Tx error counter */
	uint ctrlnb;						/**< Controller number */
	uint adapter_nb;					/**< Adapter number */
	char adapter_name[PCANINFO_MAX_CHAR_SIZE];		/**< Adapter name */
	char adapter_version[PCANINFO_MAX_CHAR_SIZE];	/**< Adapter version */
	uint rx_fifo_ratio;					/**< Filling ratio of the rx queue */
	uint tx_fifo_ratio;					/**< Filling ratio of the tx queue */

	time_t time_update;
	enum pcaninfo_hw hwcategory;		/**< Hardware category code */
};

/**
 * A list of PCANINFO structures along with its actual size
 */
struct pcaninfo_list {
	int length;			/**< Length of the array infos */
	char version[24];	/**< String version of the PCAN driver */
	struct pcaninfo infos[0];	/**< Array of PCANINFO structure */
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @fn int pcaninfo_update(PCANINFO * pci)
 * @brief Updates data fields in a PCANINFO structure
 * (classpath and name must be initialized).
 *
 * @param[in, out] pci Pointer to the PCANINFO structure to update
 * @return Status error code (0 if no error)
 */
int pcaninfo_update(struct pcaninfo * pci);

/**
 * @fn int pcaninfo_get(struct pcaninfo_list ** pcilist)
 * @brief Retrieves available PCAN devices' information.
 *
 * @param[out] pcilist buffer to store PCAN devices' information
 * @param[in] do_init state if the PCANINFO should be fully initialized (value=1)
 * 		otherwise (value=0) only the following members are valid:
 * 		'classpath', 'name'.
 * @return Status error code (0 if no error)
 */
int pcaninfo_get(struct pcaninfo_list ** pcilist, int do_init);

/**
 * @fn void pcaninfo_output(PCANINFO * pci)
 * @brief Prints a PCANINFO structure to std output
 *
 * @param[in] pci Pointer to the PCANINFO structure to output
 */
void pcaninfo_output(struct pcaninfo * pci);

/**
 * @fn int pcaninfo_print(void)
 * @brief Discovers PCAN devices and prints to std output their information
 *
 * @return Status error code (0 if no error)
 */
int pcaninfo_print(void);

/**
 * @fn pcaninfo_driver_version(char *buffer, uint size)
 * @brief Gets the version of the PCAN driver
 *
 * @param[out] buffer a buffer to store the version as a string
 * @param[in] size size of the buffer (15*char is enough)
 * @return Status error code (0 if no error)
 */
int pcaninfo_driver_version(char *buffer, uint size);

#ifdef __cplusplus
};
#endif

#endif /* __PCANINFO_H__ */
