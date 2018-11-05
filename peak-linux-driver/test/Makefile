#****************************************************************************
# Copyright (C) 2001-2010  PEAK System-Technik GmbH
#
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
# Maintainer(s): Klaus Hitschler (klaus.hitschler@gmx.de)
#****************************************************************************

#****************************************************************************
#
# Makefile - Makefile for receivetest and transmittest programs
#
# $Id$
#
#****************************************************************************

PCANDRV_DIR := ../driver
PCANLIB_DIR := ../lib

SRC := src
INC := -I. -I$(PCANLIB_DIR) -I$(PCANDRV_DIR)
RT ?= NO_RT

BINDIR = $(DESTDIR)/usr/local/bin

ifeq ($(RT), XENOMAI)

# Xenomai directory, xeno-config and library directory
RT_DIR := /usr/xenomai
RT_CONFIG := $(RT_DIR)/bin/xeno-config

CC = $(shell $(RT_CONFIG) --cc)

# User space application compiler options
# Note: --xeno-cflags is deprecated
# USERAPP_CFLAGS ?= $(shell $(RT_CONFIG) --xeno-cflags)
# USERAPP_LDFLAGS ?= $(shell $(RT_CONFIG) --xeno-ldflags)

# "native" lib has been renamed into "alchemy" since Xenomai 3
XENO_VER := $(shell $(RT_CONFIG) --version)
XENO_VER_LIST := $(subst ., ,$(XENO_VER))
XENO_VER_MAJ := $(word 1,$(XENO_VER_LIST))
#XENO_VER_MIN := $(word 2,$(XENO_VER_LIST))

ifeq ($(XENO_VER_MAJ),3)
# Xenomai 3
XENO_NATIVE := alchemy
else
XENO_NATIVE := native
endif

# Since lpcanfd links to lrtdm, we just need the native skin
SKIN := $(XENO_NATIVE)

RT_LIB_DIR := $(shell $(RT_CONFIG) --library-dir)
USERAPP_CFLAGS := $(shell $(RT_CONFIG) --skin=$(SKIN) --cflags)
USERAPP_LDFLAGS := -Wl,-rpath $(RT_LIB_DIR) $(shell $(RT_CONFIG) --skin=$(SKIN) --ldflags)
endif

ifeq ($(RT), RTAI)

# Rtai directory, rtai-config and library directory
RT_DIR := /usr/realtime
RT_CONFIG := $(RT_DIR)/bin/rtai-config

# User space application compiler options
SKIN := lxrt
USERAPP_CFLAGS := $(shell $(RT_CONFIG) --$(SKIN)-cflags)
USERAPP_LDFLAGS := $(shell $(RT_CONFIG) --$(SKIN)-ldflags)
endif

LDLIBS := -L$(PCANLIB_DIR)/lib

ifneq ($(RT), NO_RT)
DBGFLAGS :=
else
DBGFLAGS := -g
endif

ifneq ($(DESTDIR),)
COMMON_FLAGS := --sysroot=$(DESTDIR)
endif

CFLAGS := -D$(RT) $(DBGFLAGS) $(INC) $(USERAPP_CFLAGS) $(OPTS_CFLAGS) $(COMMON_FLAGS)
LDFLAGS := $(USERAPP_LDFLAGS) $(LDLIBS) $(OPTS_LDFLAGS) $(COMMON_FLAGS)

# take different source files for realtime and 'normal' test programms
ifeq ($(RT), NO_RT)
TARGET1 := receivetest
FILES1 := $(SRC)/$(TARGET1).c $(SRC)/common.c

TARGET2 := transmitest
FILES2 := $(SRC)/$(TARGET2).cpp $(SRC)/common.c $(SRC)/parser.cpp
else
TARGET1 := receivetest
FILES1 := $(SRC)/$(TARGET1)_rt.c $(SRC)/common.c

TARGET2 := transmitest
FILES2 := $(SRC)/$(TARGET2)_rt.cpp $(SRC)/common.c $(SRC)/parser.cpp
endif

TARGET3 := bitratetest
FILES3 := $(SRC)/$(TARGET3).c $(SRC)/common.c

TARGET4 := filtertest
FILES4 := $(SRC)/$(TARGET4).cpp $(SRC)/common.c

TARGET5 := pcan-settings
FILES5 := $(SRC)/$(TARGET5).c

TARGET6 := pcanfdtst
FILES6 := $(SRC)/$(TARGET6).c

ALL = $(TARGET1) $(TARGET2) $(TARGET3) $(TARGET4) $(TARGET6) $(TARGET5)

all: $(ALL)

$(TARGET1): $(FILES1)
	$(CC) $(CFLAGS) $^ -lpcanfd $(LDFLAGS) -o $@

$(TARGET2): $(FILES2)
	$(CC) $(CFLAGS) $^ -lpcanfd -lstdc++ $(LDFLAGS) -o $@
	
$(TARGET3): $(FILES3)
	$(CC) $(CFLAGS) $^ -lpcanfd $(LDFLAGS) -o $@

$(TARGET4): $(FILES4)
	$(CC) $(CFLAGS) $^ -lpcanfd -lstdc++ $(LDFLAGS) -o $@

$(TARGET5): $(FILES5)
ifeq ($(RT), NO_RT)
	$(CC) $(CFLAGS) $^ -lpopt $(LDFLAGS) -g -o $@
else
# this application doesn't link to lpcanfd but use rtdm API
	$(CC) $(CFLAGS) $^ -lpopt -lrtdm $(LDFLAGS) -g -o $@
endif

$(TARGET6): $(FILES6)
	$(CC) $(CFLAGS) $^ -lpcanfd $(LDFLAGS) -o $@

clean:
	-rm -f $(SRC)/*~ $(SRC)/*.o *~ $(ALL)
	
install:
	cp $(ALL) $(BINDIR)

uninstall:
	-cd $(BINDIR); rm -f $(ALL)

xeno:
	$(MAKE) RT=XENOMAI

rtai:
	$(MAKE) RT=RTAI

# Need:
# $ sudo apt-get install gcc-multilib  g++-multilib
# $ sudo apt-get install libpopt-dev:i386
all32:
	$(MAKE) OPTS_CFLAGS=-m32 OPTS_LDFLAGS=-m32
