#  Makefile
#
#  ~~~~~~~~~~~~
#
#  PCANBasic
#
#  ~~~~~~~~~~~~
#
#  ------------------------------------------------------------------
#  Author : Thomas Haber (thomas@toem.de)
#  Last change: $Date: 2017-10-06 17:11:18 +0200 (ven. 06 oct. 2017) $
#  Derived from: Klaus Hitschler (klaus.hitschler@gmx.de)
#  Language: make
#  ------------------------------------------------------------------
#
#  Copyright (C) 1999-2010  PEAK-System Technik GmbH, Darmstadt
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

CC	= gcc
LN	= ln -sf

SRC     = src
INC     = -I. -I./src/libpcanfd
FILES   = $(SRC)/libpcanbasic.c
FILES   += $(SRC)/pcaninfo.c
FILES   += $(SRC)/pcanlog.c
FILES   += $(SRC)/pcbcore.c
FILES   += $(SRC)/pcblog.c
FILES   += $(SRC)/pcbtrace.c
FILES   += $(SRC)/libpcanfd/src/libpcanfd.c
DBG     = -g
RT      = NO_RT

# get API version
SED_GET_VERSION = 's/^\#.*[\t\f ]+([0-9]+)[\t\f \r\n]*/\1/'
VERSION_FILE = 'src/version.h'
MAJOR = $(shell cat $(VERSION_FILE) | grep VERSION_MAJOR | sed -re $(SED_GET_VERSION))
MINOR = $(shell cat $(VERSION_FILE) | grep VERSION_MINOR | sed -re $(SED_GET_VERSION))
PATCH = $(shell cat $(VERSION_FILE) | grep VERSION_PATCH | sed -re $(SED_GET_VERSION))

ifeq ($(RT), XENOMAI)
#****************************************************************************
# Define flags for XENOMAI installation only
#
USB = NO_USB_SUPPORT
PCC = NO_PCCARD_SUPPORT

INC     = -I. -I../driver -I/usr/xenomai/include

SKIN = xeno
RT_DIR          ?= /usr/xenomai
RT_CONFIG       ?= $(RT_DIR)/bin/xeno-config
RT_LIB_DIR      ?= $(shell $(RT_CONFIG) --library-dir) -Wl,-rpath $(shell $(RT_CONFIG) --library-dir)
RT_CFLAGS       ?= $(shell $(RT_CONFIG) --$(SKIN)-cflags)
endif

ifeq ($(RT), RTAI)
#****************************************************************************
# Define flags for RTAI installation only
#
USB = NO_USB_SUPPORT
PCC = NO_PCCARD_SUPPORT

INC     = -I. -I../driver -I/usr/realtime/include

SKIN = lxrt
RT_DIR          ?= /usr/realtime
RT_CONFIG       ?= $(RT_DIR)/bin/rtai-config
RT_LIB_DIR      ?= $(shell $(RT_CONFIG) --library-dir) -Wl,-rpath $(shell $(RT_CONFIG) --library-dir)
RT_CFLAGS       ?= $(shell $(RT_CONFIG) --$(SKIN)-cflags)
endif

ifeq ($(HOSTTYPE),x86_64)
  LIBPATH = /usr/lib64
else
  LIBPATH = /usr/lib
endif

# Define targets
LDNAME  = libpcanbasic.so
SONAME  = $(LDNAME).$(MAJOR)
TARGET  = $(SONAME).$(MINOR).$(PATCH)
SONAME_OLD  = $(LDNAME).0

ifneq ($(RT), NO_RT)
  CFLAGS = -fPIC -shared -O2 -Wall -Wl,-soname,$(SONAME) -lc -lm $(INC) -D$(RT) $(RT_CFLAGS) -L$(RT_LIB_DIR) -lrtdm
else
  CFLAGS = -fPIC -shared -O2 -Wall -Wl,-soname,$(SONAME) -lc -lm $(INC) -D$(RT)
endif

#********** entries *********************

all: message $(TARGET)

$(TARGET) : $(FILES)
	$(CC) $(FILES) $(CFLAGS) -o $(TARGET) 
	$(LN) $(TARGET) $(SONAME)
	$(LN) $(SONAME) $(LDNAME)

clean:
	rm -f $(SRC)/*~ $(SRC)/*.o *~ *.so.* *.so 

.PHONY: message
message:
	@ echo "*** Making lib PCANBasic with FD support (PCAN driver >8.0)"
	@ echo "***"
	@ echo "***"
	@ echo "*** target=$(LDNAME)" 
	@ echo "*** version=$(MAJOR).$(MINOR).$(PATCH)"
	@ echo "***"
	@ echo "*** $(CC) version=$(shell $(CC) -dumpversion)"
	@ echo "***"

#********** this entry is reserved for root access only *********************
install:
	cp $(TARGET) $(LIBPATH)/$(TARGET)
	$(LN) $(LIBPATH)/$(TARGET) $(LIBPATH)/$(SONAME)
	$(LN) $(LIBPATH)/$(TARGET) $(LIBPATH)/$(SONAME_OLD)
	$(LN) $(LIBPATH)/$(SONAME) $(LIBPATH)/$(LDNAME)
	cp PCANBasic.h /usr/include/PCANBasic.h
	chmod 644 /usr/include/PCANBasic.h
	/sbin/ldconfig
  
uninstall:
	-rm $(LIBPATH)/$(TARGET)
	-rm $(LIBPATH)/$(SONAME)
	-rm $(LIBPATH)/$(SONAME_OLD)
	-rm $(LIBPATH)/$(LDNAME)
	-rm /usr/include/PCANBasic.h
	/sbin/ldconfig

uninstall-purge: uninstall
	-rm $(LIBPATH)/$(LDNAME).*
	/sbin/ldconfig