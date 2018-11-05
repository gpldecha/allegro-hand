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
 * @file pcblog.h
 * @brief Function prototypes to handle PCANBasic logging features.
 *
 * $Id: pcblog.h 87 2016-09-08 09:02:03Z Fabrice $
 *
 */
#include "../PCANBasic.h"

/** maximum size for the path of the log's directory */
#define PCAN_LOG_MAX_PATH	256

/**
 * @fn void pcblog_nprint(const char *s, unsigned long len)
 * @brief Logs a formated string
 *
 * @param[in] s buffer containing the message to log
 * @param[in] len size of the buffer
 */
void pcblog_write(const char *s, unsigned long len);

/**
 * @fn void pcblog_write_entry(const char *sfunc)
 * @brief Formats and logs the entry in a named function
 *
 * @param[in] sfunc name of the entered function
 */
void pcblog_write_entry(const char *sfunc);

/**
 * @fn void pcblog_write_param(const char *sfunc, const char *sparams)
 * @brief Formats and logs the entry in a named function
 *
 * @param[in] sfunc name of the entered function
 * @param[in] sparams formatted parameters of the function
 */
void pcblog_write_param(const char *sfunc, const char *sparams);

/**
 * @fn void pcblog_write_exit(const char *sfunc, TPCANStatus sts)
 * @brief Formats and logs the leaving of a function
 *
 * @param[in] sfunc name of the entered function
 * @param[in] sts return status of the PCANBasic function
 */
void pcblog_write_exit(const char *sfunc, TPCANStatus sts);

/**
 * @fn void pcblog_write_exception(const char *sfunc)
 * @brief Formats and logs an exception
 *
 * @param[in] sfunc name of the entered function
 */
void pcblog_write_exception(const char *sfunc);

/**
 * @fn void pcblog_write_entry(const char *sfunc)
 * @brief Formats and logs a CAN message from a PCANBasic channel.
 *
 * @param[in] channel a PCANBasic channel handle
 * @param[in] direction of the message: Rx (LOG_FUNCTION_READ) or Tx (LOG_FUNCTION_WRITE)
 * @param[in] pmsg buffer containing the CAN message
 */
void pcblog_write_can_msg(TPCANHandle channel, int direction, TPCANMsg* pmsg);


/**
 * @fn void pcblog_get_location(char *buffer, unsigned int size)
 * @brief Gets the directory of the logs
 *
 * @param[out] buffer to store the logging directory path
 * @param[in] size size of the buffer
 */
void pcblog_get_location(char *buffer, unsigned int size);
/**
 * @fn void pcblog_set_location(const char *buffer)
 * @brief Sets the logs' directory
 *
 * @param[in] path to the directory to store the logs
 * (must be NULL terminated and less than PCAN_LOG_MAX_PATH)
 */
void pcblog_set_location(const char *buffer);

/**
 * @fn int pcblog_get_status(void)
 * @brief Gets the logger's configuration
 *
 * @return the logger's status (PCAN_PARAMETER_ON) enabled or (PCAN_PARAMETER_OFF) disabled
 */
int pcblog_get_status(void);
/**
 * @fn void pcblog_set_status(int status)
 * @brief Sets the logger's state
 *
 * @param[in] status (PCAN_PARAMETER_ON) to enable or (PCAN_PARAMETER_OFF) to disable logging
 */
void pcblog_set_status(int status);

/**
 * @fn int pcblog_get_config(void)
 * @brief Gets the logger's configuration
 *
 * @return the logger's configuration to set (see LOG_FUNCTION_xxx)
 */
int pcblog_get_config(void);
/**
 * @fn void pcblog_set_config(int flags)
 * @brief Sets the logger's configuration
 *
 * @param[in] flags the configuration to set (see LOG_FUNCTION_xxx)
 */
void pcblog_set_config(int flags);
