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
 * @file pcanlog.c
 * @brief Function to log stuff
 *
 * $Id: pcanlog.c 108 2017-10-06 15:11:18Z Fabrice $
 *
 */
#include "pcanlog.h"

/*
 * INCLUDES
 */
#include <stdio.h>
#include <sys/time.h>	/* gettimeofday() */
#include <stdarg.h>		/* va_start */
#include <string.h>		/* strnlen */
#include "pcblog.h"

/**
 * @def MIN(x,y)
 * @brief A macro that returns the minimum of @a x and @a y.
 */
#define MIN(x,y) ((x) < (y) ? (x) : (y))

/*
 * PRIVATE VARIABLES
 */

/* Stores the configuration of the logger */
static struct {
	 PCANLOG_LEVEL lvl;	/**< log level */
	 FILE * f;			/**< log file descriptor (NULL for stdout) */
	 int btimestamp;	/**< add a prefixed timestamp */
 } g_pcanlog = {LVL_NORMAL, NULL, 1};
 
 int pcanlog_should_write(const PCANLOG_LEVEL lvl) {
	 switch (g_pcanlog.lvl) {
		case LVL_NORMAL:
			if (lvl == LVL_VERBOSE || lvl == LVL_DEBUG)
				return 0;
			break;
		case LVL_VERBOSE:
			if (lvl == LVL_DEBUG)
				return 0;
			break;
		case LVL_DEBUG:
		case LVL_ALWAYS:
			break;
		case LVL_QUIET:
			return 0;
	 }
	 return 1;
 }
 /*
  * GLOBAL FUNCTIONS
  */
void pcanlog_set(const PCANLOG_LEVEL lvl, const char *filename, const int showtime) {
	g_pcanlog.lvl = lvl;
	if (g_pcanlog.f != NULL) {
		fflush(g_pcanlog.f);
		fclose(g_pcanlog.f);
		g_pcanlog.f = NULL;
	}
	if (filename != NULL) {
		g_pcanlog.f = fopen(filename, "w");
	}
	g_pcanlog.btimestamp = showtime;
}
 
void pcanlog_log(const PCANLOG_LEVEL lvl, const char *fmt, ...) {
	va_list ap;
	FILE *pfout;
	char buf[512];

	pfout = (g_pcanlog.f != NULL) ? g_pcanlog.f : stdout;

	va_start(ap, fmt);
	if (!pcanlog_should_write(lvl))
		goto lbl_exit;
	if (g_pcanlog.btimestamp) {
		struct timeval tv;
		gettimeofday(&tv, NULL);
		fprintf(pfout, "%010u.%06u: ", (unsigned int )tv.tv_sec, (unsigned int )tv.tv_usec);
	}
	vfprintf(pfout, fmt, ap);
	if (lvl == LVL_DEBUG)
		fflush(pfout);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	pcblog_write(buf, MIN(sizeof(buf), strnlen(buf, sizeof(buf))));

lbl_exit:
	va_end(ap);
}

void pcanlog_write(const PCANLOG_LEVEL lvl, const char *fmt, ...) {
	va_list ap;
	FILE *pfout;
	char buf[512];

	pfout = (g_pcanlog.f != NULL) ? g_pcanlog.f : stdout;

	va_start(ap, fmt);
	if (!pcanlog_should_write(lvl))
		goto lbl_exit;
	vfprintf(pfout, fmt, ap);
	if (lvl == LVL_DEBUG)
		fflush(pfout);
	va_end(ap);
	pcblog_write(buf, MIN(sizeof(buf), strnlen(buf, sizeof(buf))));

lbl_exit:
	va_end(ap);
}
