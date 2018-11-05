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
 * @file pcbtrace.h
 * @brief Function's prototypes for the tracer functions of Linux PCANBasic
 *
 * $Id: pcbtrace.h 102 2017-10-04 10:43:23Z Fabrice $
 *
 */

#ifndef __PCBTRACE_H__
#define __PCBTRACE_H__

/*
 * INCLUDES
 */
#include <sys/types.h>
#include <sys/time.h>
#include <stdio.h>
#include "../PCANBasic.h"
#include "pcaninfo.h"

/*
 * DEFINES
 */
#define PCBTRACE_MAX_CHAR_SIZE 256	/**< Max buffer size used in struct pcbtrace */

/**
 * A structure to hold the context information for a PCANBasic tracer
 */
struct pcbtrace_ctx {
	char directory[PCBTRACE_MAX_CHAR_SIZE];			/**< path to the trace directory */
	char chname[PCBTRACE_MAX_CHAR_SIZE];			/**< short name of the TPANHANDLE */
	char filename_chunk[PCBTRACE_MAX_CHAR_SIZE];	/**< base name of the segmented trace's file */
	char filename[PCBTRACE_MAX_CHAR_SIZE];			/**< current name of the trace's file */
	uint idx;								/**< index of the file (for segmented traces) */
	ushort status;		/**< status of the tracer */
	ushort maxsize;		/**< maximum size of the trace file in MB */
	uint flags;			/**< trace configuration (see TRACE_FILE_xxx in PCANBasic.h) */
	FILE *pfile;		/**< file descriptor */
	ulong msg_cnt;		/**< count the number of CAN messages traced */
	struct timeval time_start;
};

/**
 * @fn void pcbtrace_set_defaults(struct pcbtrace_ctx *ctx)
 * @brief Initializes a PCANBasic tracer context with default values.
 *
 * @param ctx pointer to a context information of the PCANBasic tracer
 */
void pcbtrace_set_defaults(struct pcbtrace_ctx *ctx);

/**
 * @fn int pcbtrace_open(struct pcbtrace_ctx *ctx, enum pcaninfo_hw hw, uint ch_idx)
 * @brief Opens a trace file based on the context information.
 *
 * @param ctx pointer to a context information of the PCANBasic tracer
 * @param hw type of the PCAN hardware
 * @param ch_idx channel index
 * @return 0 on success or an errno otherwise
 */
int pcbtrace_open(struct pcbtrace_ctx *ctx, enum pcaninfo_hw hw, uint ch_idx);

/**
 * @fn int pcbtrace_close(struct pcbtrace_ctx *ctx)
 * @brief Closes a PCANBasic tracer.
 *
 * @param ctx pointer to a context information of the PCANBasic tracer
 * @return 0 on success or an errno otherwise
 */
int pcbtrace_close(struct pcbtrace_ctx *ctx);

/**
 * @fn int pcbtrace_write_msg(struct pcbtrace_ctx *ctx, TPCANMsgFD *msg, int data_len, struct timeval *tv, int rx)
 * @brief Writes a CAN FD message to the trace file.
 *
 * @param ctx pointer to a context information of the PCANBasic tracer
 * @param msg pointer to the CAN FD message to output
 * @param data_len the real data length of the message
 * @param tv timestamp of the message
 * @param rx 1 if the message was received or 0 if it was transmitted
 * @return 0 on success or an errno otherwise
 */
int pcbtrace_write_msg(struct pcbtrace_ctx *ctx, TPCANMsgFD *msg, int data_len, struct timeval *tv, int rx);

/**
 * @fn int pcbtrace_write(struct pcbtrace_ctx *ctx, const char * buffer, uint size)
 * @brief Writes a string message to the trace file.
 *
 * @param ctx pointer to a context information of the PCANBasic tracer
 * @param buffer pointer to the string to output
 * @param size size of the buffer
 * @return 0 on success or an errno otherwise
 */
int pcbtrace_write(struct pcbtrace_ctx *ctx, const char * buffer, uint size);

#endif
