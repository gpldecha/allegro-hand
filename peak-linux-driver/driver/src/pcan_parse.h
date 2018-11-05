/****************************************************************************
 * Copyright (C) 2001-2007  PEAK System-Technik GmbH
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
 ****************************************************************************/

/****************************************************************************
 *
 * pcan_parse.h - header for read input parser and write output formatter
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef __PCAN_PARSE_H__
#define __PCAN_PARSE_H__

#include <pcanfd.h>

int pcan_make_output(char *buffer, struct pcanfd_msg *pf);
int pcan_parse_input_idle(char *buffer);
int pcan_parse_input_message(char *buffer, struct pcanfd_msg *pf);
int pcan_parse_input_init(char *buffer, struct pcanfd_init *pi);

#endif
