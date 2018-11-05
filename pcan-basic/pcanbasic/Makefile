#  Makefile
#
#  ~~~~~~~~~~~~
#
#  PCANBasic
#
#  ~~~~~~~~~~~~
#
#  ------------------------------------------------------------------
#  Last change: $Date: 2016-01-20 16:16:25 +0100 (mer. 20 janv. 2016) $
#  Language: make
#  ------------------------------------------------------------------
#
#  Copyright (C) 1999-2016  PEAK-System Technik GmbH, Darmstadt
#  more Info at http://www.peak-system.com
#  ------------------------------------------------------------------
# linux@peak-system.com
# www.peak-system.com
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#
#****************************************************************************

PCANFD_HEADER = /usr/include/pcanfd.h

ifneq ($(wildcard $(PCANFD_HEADER)), )
$(info Making lib PCANBasic with FD support (PCAN driver >8.0))
include Makefile_latest.mk
else
# Making lib PCANBasic WITHOUT FD support (PCAN driver <8.0)
$(warning Making lib PCANBasic WITHOUT FD support (PCAN driver <8.0))
include Makefile_legacy.mk
endif
