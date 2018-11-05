#****************************************************************************
# Copyright (C) 2001-2007  PEAK System-Technik GmbH
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
# Makefile - Makefile for the shared library libpcan.so.x.x
#
# $Id$
#
#****************************************************************************

PCANDRV_DIR := ../driver

INC := -I. -I$(PCANDRV_DIR)
FILES := src/libpcan.c
FILESFD := src/libpcanfd.c
DBG := -g
RT ?= NO_RT

ifeq ($(RT), XENOMAI)
RT_DIR := /usr/xenomai
RT_CONFIG := $(RT_DIR)/bin/xeno-config

SKIN := rtdm
RT_CFLAGS := $(shell $(RT_CONFIG) --skin $(SKIN) --cflags)
RT_LDFLAGS := -Wl,-rpath $(shell $(RT_CONFIG) --library-dir) $(shell $(RT_CONFIG) --skin $(SKIN) --ldflags)
endif

ifeq ($(RT), RTAI)
RT_DIR ?= /usr/realtime
RT_CONFIG ?= $(RT_DIR)/bin/rtai-config

SKIN := lxrt
RT_CFLAGS ?= $(shell $(RT_CONFIG) --$(SKIN)-cflags)
endif

LDNAME := libpcan.so
SONAME := $(LDNAME).0
TARGET := $(SONAME).6

LDNAMEFD := libpcanfd.so
SONAMEFD := $(LDNAMEFD).0
TARGETFD := $(SONAMEFD).1

LSNAMEFD := $(basename $(LDNAMEFD)).a

# Xenomai 3 does *NOT* like -O2 with _wrappers_ (and gcc 4.8 ?)
#CFLAGS += -D$(RT) $(INC) -fPIC -shared -O2 -Wall $(RT_CFLAGS)
CFLAGS += -D$(RT) $(INC) -fPIC -Wall -Wcast-align -Wredundant-decls $(RT_CFLAGS)

LDFLAGS += $(RT_LDFLAGS)

ALL := $(LDNAME) $(TARGET) $(LDNAMEFD) $(TARGETFD) $(LSNAMEFD)

# test if running compiler is able to build 32-bit applications.
# In order to know what kind of executable the linker builds:
# $ ld --print-output-format
GCC_32OPT := $(shell $(CC) -print-multi-lib | awk -F ";" '/^32/ { sub("@","-",$$2); print $$2 }')
ifneq ($(GCC_32OPT),)
# Running a 64-bit host needs to build the 32-bit libpcan too.
# Test if building 32-bit libpcan is possible:
define LIBC32_NOK
$(shell echo "
#include <sys/cdefs.h>\n
int main() { return 0; }" | $(CC) $(GCC_32OPT) -o /dev/null -x c - 2>&1)
endef
ifeq ($(LIBC32_NOK),)
ALL32 := $(foreach a,$(ALL),lib32/$(a))
CFLAGS32 = $(GCC_32OPT) # -m32
LDFLAGS32 = $(GCC_32OPT) # -m32
else
#$(error from:$(LIBC32_NOK):to)
ALL32 = no_i386_libc
# compiler is able to build 32-bit binaries but system lacks of any 32-bit
# libc. For example:
# $ apt-get install libc6-dev-i386
endif
else
# compiler ISNOT able to build any 32-bit binary.
ALL32 = no_m32_compiler
endif

all: $(foreach a,$(ALL),lib/$(a)) $(ALL32)

ALLOBJ := $(foreach f,$(FILES),obj/$(basename $(notdir $(f))).o)
lib/$(TARGET): $(ALLOBJ)
	@mkdir -p $(dir $@)
	$(CC) -shared -Wl,-soname,$(SONAME) -o $@ $^ $(LDFLAGS)

lib/$(LDNAME): lib/$(TARGET)
	cd $(dir $@); ln -sf $(TARGET) $(LDNAME)

# libpcanfd.c includes libpcan.c
ALLFDOBJ := $(foreach f,$(FILESFD),obj/$(basename $(notdir $(f))).o)
lib/$(TARGETFD): $(ALLFDOBJ)
	@mkdir -p $(dir $@)
	$(CC) -shared -Wl,-soname,$(SONAMEFD) -o $@ $^ $(LDFLAGS)

lib/$(LDNAMEFD): lib/$(TARGETFD)
	cd $(dir $@); ln -sf $(TARGETFD) $(LDNAMEFD)

lib/$(LSNAMEFD): $(ALLFDOBJ)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^

obj/%.o: src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -DPCANFD_OLD_STYLE_API -c $< -o $@

clean:
	rm -f src/*~
	rm -rf obj lib obj32 lib32

ifneq ($(CFLAGS32),)
ALLOBJ32 := $(foreach f,$(FILES),obj32/$(basename $(notdir $(f))).o)
lib32/$(TARGET) : $(ALLOBJ32)
	@mkdir -p $(dir $@)
	$(CC) -shared -Wl,-soname,$(SONAME) -o $@ $^ $(LDFLAGS) $(LDFLAGS32)

lib32/$(LDNAME): lib32/$(TARGET)
	cd $(dir $@); ln -sf $(TARGET) $(LDNAME)

ALLFDOBJ32 := $(foreach f,$(FILESFD),obj32/$(basename $(notdir $(f))).o)
lib32/$(TARGETFD) : $(ALLFDOBJ32)
	@mkdir -p $(dir $@)
	$(CC) -shared -Wl,-soname,$(SONAMEFD) -o $@ $^ $(LDFLAGS) $(LDFLAGS32)

lib32/$(LDNAMEFD): lib32/$(TARGETFD)
	cd $(dir $@); ln -sf $(TARGETFD) $(LDNAMEFD)

lib32/$(LSNAMEFD): $(ALLFDOBJ32)
	$(AR) rcs $@ $^

obj32/%.o: src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CFLAGS32) -DPCANFD_OLD_STYLE_API -c $< -o $@
else
BUILD_ARCH_IS_64=$(shell $(CC) -dumpmachine | grep 64)
.PHONY: no_i386_libc no_m32_compiler
no_i386_libc:
ifneq ($(BUILD_ARCH_IS_64),)
	@echo
	@echo "Info: only 64-bit version of libpcan can be built. 32-bit version of libpcan can't because the 32-bit version of libc seems not being installed..."
	@echo
endif

no_m32_compiler:
ifneq ($(BUILD_ARCH_IS_64),)
	@echo
	@echo "Info: only 64-bit version of libpcan can be built. 32-bit version of libpcan can't because 64-bit compiler is not able to build any 32-bit binary."
	@echo
endif
endif

# Default libraries installation directory:
LIBPATH := /usr/lib
LIB32PATH := $(LIBPATH)32
INCPATH := /usr/include

ifeq ($(DESTDIR_DEV),)
DESTDIR_DEV = $(DESTDIR)
endif

# root access only
install:
	mkdir -p $(DESTDIR)$(LIBPATH)
	cp -d lib/* $(DESTDIR)$(LIBPATH)
	if [ -d lib32 ]; then \
		mkdir -p $(DESTDIR)$(LIB32PATH); \
		cp -d lib32/* $(DESTDIR)$(LIB32PATH); \
	fi
	mkdir -p $(DESTDIR_DEV)$(INCPATH)
	chmod 644 libpcan.h libpcanfd.h
	cp libpcan.h libpcanfd.h $(DESTDIR_DEV)$(INCPATH)
ifeq ($(DESTDIR),)
	/sbin/ldconfig
endif

uninstall:
	-rm -f $(DESTDIR_DEV)$(INCPATH)/libpcan.h \
	       $(DESTDIR_DEV)$(INCPATH)/libpcanfd.h
	-for f in $(ALL); do \
		rm -f $(DESTDIR)$(LIBPATH)/$$f; \
		rm -f $(DESTDIR)$(LIB32PATH)/$$f; \
	 done
ifeq ($(DESTDIR),)
	/sbin/ldconfig
endif

xeno:
	$(MAKE) RT=XENOMAI

rtai:
	$(MAKE) RT=RTAI
