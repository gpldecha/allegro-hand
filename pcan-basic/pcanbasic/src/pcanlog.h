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
 * @file pcanlog.h
 * @brief Function prototypes to log stuff
 *
 * $Id: pcanlog.h 87 2016-09-08 09:02:03Z Fabrice $
 *
 */

#ifndef __PCANLOG_H__
#define __PCANLOG_H__

/**
 * Defines log verbosity
 */
typedef enum {
	LVL_QUIET,		/**< log seen when using silent mode */
	LVL_NORMAL,		/**< default log */
	LVL_VERBOSE,	/**< log seen when using verbose mode */
	LVL_DEBUG,		/**< log seen when using debug mode */
	LVL_ALWAYS		/**< log always displayed */
} PCANLOG_LEVEL;


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @fn void pcanlog_set(PCANLOG_LEVEL lvl, char *filename, int showtime)
 * @brief Configures the logging system.
 *
 * @param[in] lvl The maximum level to be displayed
 * @param[in] filename The filename to write the log
 * @param[in] showtime State to prefix the log with a timestamp
 */
void pcanlog_set(const PCANLOG_LEVEL lvl, const char *filename, const int showtime);

/**
 * @fn void pcanlog_log(PCANLOG_LEVEL lvl, char *fmt, ...)
 * @brief Logs an entry (with a timestamp if optien is set)
 *
 * @param[in] lvl level of the log
 * @param[in] fmt Formatted string
 */
void pcanlog_log(const PCANLOG_LEVEL lvl, const char *fmt, ...);

/**
 * @fn void pcanlog_log(PCANLOG_LEVEL lvl, char *fmt, ...)
 * @brief Writes a raw message in the log
 *
 * @param[in] lvl level of the log
 * @param[in] fmt Formatted string
 */
void pcanlog_write(const PCANLOG_LEVEL lvl, const char *fmt, ...);

#ifdef __cplusplus
};
#endif

#endif /* __PCANLOG_H__ */
