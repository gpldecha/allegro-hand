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
#****************************************************************************

# Define programs and commands.
AS      = $(CROSS_COMPILE)as
CC      = $(CROSS_COMPILE)gcc
CPP     = $(CC) -E
AR      = $(CROSS_COMPILE)ar
NM      = $(CROSS_COMPILE)nm
STRIP   = $(CROSS_COMPILE)strip
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
REMOVE 	= rm -f
COPY	= cp
LINK	= ln -sf

# Define source
SRC     = src
SRC_PCANBASIC = ../../pcanbasic/src
FILES   = $(SRC)/main.c
FILES   += $(SRC_PCANBASIC)/pcanlog.c
FILES   += $(SRC_PCANBASIC)/libpcanfd/src/libpcanfd.c
FILES   += $(SRC_PCANBASIC)/pcblog.c
FILES   += $(SRC_PCANBASIC)/pcbtrace.c
FILES   += $(SRC_PCANBASIC)/pcbcore.c
FILES   += $(SRC_PCANBASIC)/pcaninfo.c
RT	= NO_RT

ifeq ($(HOSTTYPE),x86_64)
  LIBPATH = /usr/lib64
else
  LIBPATH = /usr/lib
endif

# Target file name (without extension).
TARGET  = pcaninfo
CFLAG_DEBUG=yes

# Define all object files.
COBJ      = $(SRC:.c=.o) $(SRC_PCANBASIC:.c=.o)

# Place -D or -U options for C here
CDEFS =  

# Place -I options here
CINCS = -I. -I../../pcanbasic/src -I../../pcanbasic/src/libpcanfd/src -I../../pcanbasic/src/libpcanfd

# Place -D or -U options for ASM here
ADEFS =  -D$(RUN_MODE)

# Compiler flags.
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
#
# Flags for C and C++


ifeq ($(CFLAG_DEBUG),yes)
	CFLAGS = -ggdb
else
	CFLAGS = -O2
endif
CFLAGS += $(CDEFS) $(CINCS) -D$(RT)
CFLAGS += -Wall -Wcast-align -Wcast-qual -Wimplicit 
CFLAGS += -Wpointer-arith -Wswitch
CFLAGS += -Wredundant-decls -Wreturn-type -Wshadow -Wunused
CFLAGS += -lm


# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after:
MSG_FLASH = Creating load file for Flash:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling C:
MSG_COMPILINGCPP = Compiling C++:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:


all:  begin gccversion build finished end

build: $(TARGET)

$(TARGET) : $(FILES)
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) $(FILES) $(CFLAGS) -o $(TARGET)

clean:
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) $(SRC)/*~ $(SRC)/*.o $(SRC_PCANBASIC)/*~ $(SRC_PCANBASIC)/*.o *~ *.so.* *.so $(TARGET)
	
# Eye candy.
begin:
	@echo
	@echo $(MSG_BEGIN)

finished:
	@echo
	@echo $(MSG_ERRORS_NONE)

end:
	@echo $(MSG_END)
	@echo

# Display compiler version information.
gccversion : 
	@$(CC) --version
  
#********** this entry is reserved for root access only *********************
install:
	cp $(TARGET) /usr/local/bin/$(TARGET)
	chmod 755 /usr/local/bin/$(TARGET)
  
uninstall:
	rm /usr/local/bin/$(TARGET)
